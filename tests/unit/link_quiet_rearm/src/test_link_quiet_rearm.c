/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * Unit tests for src/link_quiet_rearm.h's link_quiet_rearm_tick() -- the
 * pure once-per-episode DECISION behind #142's quiet-armed detector.  See
 * that header's own top comment for the full root-cause writeup (including
 * the REJECTED first design, host review of b3dc1e2) and the WHO/WHEN
 * safety argument; this file covers only the pure arithmetic, the same
 * split as sock_prefetch_arm.h / wifi_connect_fail_skip.h.
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

	zassert_false(
	    link_quiet_rearm_tick(&st, false /* at_idle_header */, REARM_MS, REARM_MS, REARM_MS),
	    "mid-frame (not PH_REQ_HEADER) must never fire the quiet-rearm heal");
	zassert_false(st.healed_pending, "a blocked fire must not arm the latch");
}

ZTEST(link_quiet_rearm, test_not_quiet_enough_never_fires)
{
	link_quiet_rearm_state_t st = { 0 };

	zassert_false(link_quiet_rearm_tick(&st, true, REARM_MS - 1u, REARM_MS - 1u, REARM_MS),
	              "one ms short of the threshold must not fire");
}

/* ---- the fire + once-per-episode latch ---------------------------------- */

ZTEST(link_quiet_rearm, test_fires_exactly_at_threshold)
{
	link_quiet_rearm_state_t st = { 0 };

	const bool fired = link_quiet_rearm_tick(&st, true, REARM_MS, REARM_MS, REARM_MS);

	zassert_true(fired, "quiet_ms == rearm_ms (>=) must fire");
	zassert_true(st.healed_pending, "a fire must arm the latch");
	zassert_equal(st.heal_fired_at_ms, REARM_MS, "the latch must record the firing timestamp");
}

ZTEST(link_quiet_rearm, test_does_not_refire_within_the_same_episode)
{
	link_quiet_rearm_state_t st = { 0 };

	/* Fire once at t=3000. */
	zassert_true(link_quiet_rearm_tick(&st, true, 3000u, 3000u, REARM_MS), "setup: first fire");

	/* 10 ms later, the host STILL has not clocked anything and nothing
	 * re-armed: quiet_ms grew by the same 10 ms as now_ms, so
	 * since_heal == quiet_ms's own growth -- must NOT re-fire. */
	zassert_false(link_quiet_rearm_tick(&st, true, 3010u, 3010u, REARM_MS),
	              "10 ms into the same wedge episode, with zero progress, must not re-fire");
	zassert_true(st.healed_pending, "the latch must still be armed -- no phase completed");

	/* Even much later in the same stuck episode. */
	zassert_false(link_quiet_rearm_tick(&st, true, 60000u, 60000u, REARM_MS),
	              "a permanently wedged link must fire ONCE, not every call, until a phase "
	              "completes (SPI_close without transferCancel risks a DMA leak per-reinit)");
}

ZTEST(link_quiet_rearm, test_latch_clears_once_a_phase_completes_then_can_refire)
{
	link_quiet_rearm_state_t st = { 0 };

	/* Fire once at t=3000. */
	zassert_true(link_quiet_rearm_tick(&st, true, 3000u, 3000u, REARM_MS), "setup: first fire");

	/* A phase completes at t=3200 (200 ms after the heal): g_last_xfer_ms
	 * jumps forward, so quiet_ms observed at t=3205 is small (5 ms) while
	 * since_heal is 205 ms -- quiet_ms < since_heal proves a NEW phase
	 * landed after the heal fired, so the latch must clear.  5 ms is also
	 * short of REARM_MS, so this same call must still report false (the
	 * latch clearing and a fresh fire are two different questions). */
	zassert_false(link_quiet_rearm_tick(&st, true, 5u, 3205u, REARM_MS),
	              "quiet_ms just dropped to 5 ms -- nowhere near a fresh 3 s wedge yet");
	zassert_false(st.healed_pending, "a genuinely completed phase must clear the latch");

	/* Time passes with no further traffic; once quiet_ms crosses REARM_MS
	 * again (a SECOND, independent wedge), the detector must fire again. */
	zassert_true(link_quiet_rearm_tick(&st, true, REARM_MS, 3205u + REARM_MS, REARM_MS),
	             "a second, independent quiet episode after the latch cleared must fire again");
}

ZTEST(link_quiet_rearm, test_latch_reset_boundary_is_strict_less_than)
{
	/* Pins the exact `quiet_ms < since_heal` boundary (not <=) -- a mutation
	 * that flips this to <= or > would only be caught by testing the EQUAL
	 * case directly. */
	link_quiet_rearm_state_t st = { 0 };

	zassert_true(link_quiet_rearm_tick(&st, true, 3000u, 3000u, REARM_MS), "setup: first fire");

	/* now_ms = 3300 (300 ms since the heal); quiet_ms reported as EXACTLY
	 * 300 as well -- since_heal == quiet_ms, the boundary case.  No NEW
	 * information: this quiet_ms is exactly consistent with "nothing has
	 * completed since the heal, quiet_ms grew in lockstep with wall time" --
	 * must NOT be read as a completed phase. */
	zassert_false(link_quiet_rearm_tick(&st, true, 300u, 3300u, REARM_MS),
	              "quiet_ms == since_heal is the no-new-evidence boundary -- must stay latched");
	zassert_true(st.healed_pending, "the boundary case must not clear the latch");
}

/* #142 (host review of b3dc1e2), item 8: the real fix stamps g_last_xfer_ms
 * not only in on_transfer() but also in spi_open_and_arm()/hw_release()/
 * hw_suspend() -- ANY re-arm or deliberate teardown resets the quiet clock,
 * so a reinit fired here never inherits a stale quiet_ms measured from
 * before that re-arm (the OTA-erase scenario the old design got wrong: a
 * reinit right after quiesce(false) used to see quiet_ms computed against a
 * last-xfer stamp that could be tens of seconds old, from BEFORE the flash
 * op).  This function only ever sees the RESULT of that stamping (a smaller
 * quiet_ms), so this test pins the same "quiet_ms dropped -> latch clears,
 * and the NEXT quiet period is measured fresh from here" contract as the
 * phase-completion test above, but framed explicitly around a re-arm rather
 * than a completed request/reply cycle -- the two are indistinguishable to
 * this pure function by design (it only ever sees bridge_transport_spi_
 * quiet_ms()'s output, never WHY it changed). */
ZTEST(link_quiet_rearm, test_rearm_refreshes_the_stamp_so_next_quiet_period_starts_fresh)
{
	link_quiet_rearm_state_t st = { 0 };

	/* Fire once at t=3000 (a genuine wedge). */
	zassert_true(link_quiet_rearm_tick(&st, true, 3000u, 3000u, REARM_MS), "setup: first fire");

	/* bridge_transport_spi_hw_reinit() -> spi_open_and_arm() runs shortly
	 * after (t=3010): it stamps g_last_xfer_ms itself, so the VERY NEXT
	 * observed quiet_ms is ~0, not still climbing from the t=3000 heal. */
	zassert_false(link_quiet_rearm_tick(&st, true, 0u, 3010u, REARM_MS),
	              "immediately after the reinit's own re-arm stamp, quiet_ms must read ~0");
	zassert_false(st.healed_pending,
	              "the re-arm's own stamp must clear the latch immediately, "
	              "not wait for a host-driven phase to do it");

	/* A full FRESH rearm_ms must elapse from THIS re-arm, not from the
	 * original t=3000 heal, before the detector may fire again. */
	zassert_false(link_quiet_rearm_tick(&st, true, REARM_MS - 1u, 3010u + REARM_MS - 1u, REARM_MS),
	              "1 ms short of a FULL fresh window measured from the re-arm must not fire");
	zassert_true(link_quiet_rearm_tick(&st, true, REARM_MS, 3010u + REARM_MS, REARM_MS),
	             "a full fresh rearm_ms measured from the re-arm must fire");
}
