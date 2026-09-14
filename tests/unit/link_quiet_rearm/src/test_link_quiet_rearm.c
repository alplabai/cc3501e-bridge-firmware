/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * Unit tests for src/link_quiet_rearm.h's link_quiet_rearm_tick() -- the
 * pure once-per-episode DECISION behind #142's quiet-armed detector.  See
 * that header's own top comment for the full root-cause writeup (including
 * BOTH rejected designs -- b3dc1e2's background task, and dfd5280's
 * quiet-ms-based latch clear, the exact bug test_reinit_alone_does_not_
 * clear_the_latch below pins) and the WHO/WHEN safety argument; this file
 * covers only the pure arithmetic, the same split as sock_prefetch_arm.h /
 * wifi_connect_fail_skip.h.
 *
 * NOT COVERED HERE: cc3501e_hw_link_heal() itself (hal/ti/cc3501e_hw_ti.c)
 * and its call sites in hal/ti/cc3501e_hw_ti_wifi.c -- the real callers,
 * TI-SDK-only, never linked on the host.  Reviewed but not bench-verified
 * as of this change.
 */

#include <stdint.h>
#include <zephyr/ztest.h>

#include "link_quiet_rearm.h"

ZTEST_SUITE(link_quiet_rearm, NULL, NULL, NULL, NULL, NULL);

#define REARM_MS 3000u

/* ---- the basic gate conditions: each must independently block a fire ---- */

ZTEST(link_quiet_rearm, test_not_idle_header_never_fires)
{
	link_quiet_rearm_state_t st = { 0 };

	zassert_false(link_quiet_rearm_tick(
	                  &st, false /* at_idle_header */, REARM_MS, REARM_MS, 0u /* xfer_count */),
	              "mid-frame (not PH_REQ_HEADER) must never fire the quiet-rearm heal");
	zassert_false(st.healed_pending, "a blocked fire must not arm the latch");
}

ZTEST(link_quiet_rearm, test_not_quiet_enough_never_fires)
{
	link_quiet_rearm_state_t st = { 0 };

	zassert_false(link_quiet_rearm_tick(&st, true, REARM_MS - 1u, REARM_MS, 0u),
	              "one ms short of the threshold must not fire");
}

/* ---- the fire + once-per-episode latch ---------------------------------- */

ZTEST(link_quiet_rearm, test_fires_exactly_at_threshold)
{
	link_quiet_rearm_state_t st = { 0 };

	const bool fired = link_quiet_rearm_tick(&st, true, REARM_MS, REARM_MS, 5u /* xfer_count */);

	zassert_true(fired, "quiet_ms == rearm_ms (>=) must fire");
	zassert_true(st.healed_pending, "a fire must arm the latch");
	zassert_equal(st.heal_fired_xfer_count, 5u, "the latch must record the xfer_count AT the fire");
}

ZTEST(link_quiet_rearm, test_does_not_refire_within_the_same_episode_no_xfer_change)
{
	link_quiet_rearm_state_t st = { 0 };

	/* Fire once; xfer_count is 5 at the fire and stays 5 (no host activity,
	 * and no re-arm either -- the plain "wedge persists" case). */
	zassert_true(link_quiet_rearm_tick(&st, true, 3000u, REARM_MS, 5u), "setup: first fire");

	zassert_false(link_quiet_rearm_tick(&st, true, 3010u, REARM_MS, 5u),
	              "xfer_count unchanged -- must not re-fire");
	zassert_true(st.healed_pending, "the latch must still be armed -- no host evidence");

	zassert_false(link_quiet_rearm_tick(&st, true, 60000u, REARM_MS, 5u),
	              "a permanently wedged link must fire ONCE, not every call, until a REAL "
	              "transfer lands (SPI_close without transferCancel risks a DMA leak per-reinit)");
}

/* THE REGRESSION (host review of dfd5280): the heal's OWN reinit re-arms the
 * header, which resets bridge_transport_spi_quiet_ms() (by design -- see
 * link_quiet_rearm.h's own top comment) but must NOT by itself look like
 * host evidence.  xfer_count only moves in on_transfer() -- a re-arm alone
 * must never advance it.  This test feeds exactly that shape: quiet_ms drops
 * to ~0 (as it would immediately after the heal's own spi_open_and_arm()
 * re-arm stamp), but xfer_count is UNCHANGED (no real transfer happened) --
 * the latch must stay armed and the detector must not re-fire even after a
 * full further rearm_ms of (apparent) quiet. */
ZTEST(link_quiet_rearm, test_reinit_alone_does_not_clear_the_latch)
{
	link_quiet_rearm_state_t st = { 0 };

	zassert_true(link_quiet_rearm_tick(&st, true, 3000u, REARM_MS, 7u), "setup: first fire");

	/* Immediately after the heal's own reinit: quiet_ms reads ~0 (the re-arm
	 * stamped it), xfer_count is STILL 7 -- no real transfer occurred. */
	zassert_false(link_quiet_rearm_tick(&st, true, 0u, REARM_MS, 7u),
	              "the heal's own re-arm must not satisfy quiet_ms >= rearm_ms yet anyway");
	zassert_true(st.healed_pending, "xfer_count unchanged -- the latch must NOT clear");

	/* A full further rearm_ms elapses with STILL no real transfer (xfer_count
	 * still 7) -- this is exactly the shape that used to fire again every
	 * rearm_ms against a genuinely deaf slave (9 fires in 30 s, measured). */
	zassert_false(link_quiet_rearm_tick(&st, true, REARM_MS, REARM_MS, 7u),
	              "quiet_ms alone reaching the threshold again must NOT re-fire while "
	              "xfer_count is still unchanged from the original fire");
	zassert_true(st.healed_pending, "the latch must remain armed with zero host evidence");
}

ZTEST(link_quiet_rearm, test_real_host_transfer_clears_latch_then_can_refire)
{
	link_quiet_rearm_state_t st = { 0 };

	zassert_true(link_quiet_rearm_tick(&st, true, 3000u, REARM_MS, 7u), "setup: first fire");

	/* A REAL host transfer lands (on_transfer() bumped g_xfer_count to 8):
	 * quiet_ms is small (a phase just completed) and xfer_count differs from
	 * the fire's snapshot -- the latch must clear.  Small quiet_ms also means
	 * this same call must not immediately re-fire (clearing the latch and
	 * firing are two different questions). */
	zassert_false(link_quiet_rearm_tick(&st, true, 5u, REARM_MS, 8u),
	              "quiet_ms just dropped to 5 ms -- nowhere near a fresh 3 s wedge yet");
	zassert_false(st.healed_pending, "a real transfer (xfer_count moved) must clear the latch");

	/* A SECOND, independent quiet episode after the latch cleared must fire
	 * again, latched against the NEW xfer_count. */
	zassert_true(link_quiet_rearm_tick(&st, true, REARM_MS, REARM_MS, 8u),
	             "a second, independent quiet episode after the latch cleared must fire again");
	zassert_equal(st.heal_fired_xfer_count,
	              8u,
	              "the second fire must snapshot the CURRENT xfer_count, not the first fire's");
}
