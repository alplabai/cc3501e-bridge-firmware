/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * Unit tests for src/link_quiet_rearm.h's link_quiet_rearm_tick() -- the
 * pure once-per-episode DECISION behind #142's quiet-armed detector.  See
 * that header's own top comment for the full root-cause writeup; this file
 * covers only the pure arithmetic, the same split as sock_prefetch_arm.h /
 * wifi_connect_fail_skip.h.
 *
 * NOT COVERED HERE: cc3501e_hw_link_tick() itself (hal/ti/cc3501e_hw_ti.c)
 * -- the real caller, TI-SDK-only, never linked on the host -- and the
 * cc3501e_link_task wiring / g_reinit_mutex in src/main.c and
 * hal/ti/transport_hw_ti_spi.c, also TI-only.  Those are reviewed but not
 * bench-verified as of this change; see this fix's own commit message.
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

	zassert_false(link_quiet_rearm_tick(&st,
	                                    false /* at_idle_header */,
	                                    false /* reply_armed */,
	                                    REARM_MS,
	                                    REARM_MS,
	                                    REARM_MS,
	                                    false /* gated */),
	              "mid-frame (not PH_REQ_HEADER) must never fire the quiet-rearm heal");
	zassert_false(st.healed_pending, "a blocked fire must not arm the latch");
}

ZTEST(link_quiet_rearm, test_reply_armed_never_fires)
{
	link_quiet_rearm_state_t st = { 0 };

	zassert_false(link_quiet_rearm_tick(&st,
	                                    true /* at_idle_header */,
	                                    true /* reply_armed */,
	                                    REARM_MS,
	                                    REARM_MS,
	                                    REARM_MS,
	                                    false),
	              "an armed reply phase must never fire the quiet-rearm heal (#106)");
	zassert_false(st.healed_pending, "a blocked fire must not arm the latch");
}

ZTEST(link_quiet_rearm, test_not_quiet_enough_never_fires)
{
	link_quiet_rearm_state_t st = { 0 };

	zassert_false(
	    link_quiet_rearm_tick(&st, true, false, REARM_MS - 1u, REARM_MS - 1u, REARM_MS, false),
	    "one ms short of the threshold must not fire");
}

ZTEST(link_quiet_rearm, test_gated_never_fires_even_if_otherwise_due)
{
	link_quiet_rearm_state_t st = { 0 };

	zassert_false(link_quiet_rearm_tick(&st,
	                                    true /* at_idle_header */,
	                                    false /* reply_armed */,
	                                    REARM_MS + 1000u /* quiet_ms, well past */,
	                                    REARM_MS + 1000u,
	                                    REARM_MS,
	                                    true /* gated: polled or quiesced */),
	              "polled/quiesced (an OTA session owns the slave) must suppress the heal "
	              "even when every other condition is met");
	zassert_false(st.healed_pending, "a gated tick must not touch the latch either way");
}

/* ---- the fire + once-per-episode latch ---------------------------------- */

ZTEST(link_quiet_rearm, test_fires_exactly_at_threshold)
{
	link_quiet_rearm_state_t st = { 0 };

	const bool fired = link_quiet_rearm_tick(&st, true, false, REARM_MS, REARM_MS, REARM_MS, false);

	zassert_true(fired, "quiet_ms == rearm_ms (>=) must fire");
	zassert_true(st.healed_pending, "a fire must arm the latch");
	zassert_equal(st.heal_fired_at_ms, REARM_MS, "the latch must record the firing timestamp");
}

ZTEST(link_quiet_rearm, test_does_not_refire_within_the_same_episode)
{
	link_quiet_rearm_state_t st = { 0 };

	/* Fire once at t=3000. */
	zassert_true(link_quiet_rearm_tick(&st, true, false, 3000u, 3000u, REARM_MS, false),
	             "setup: first fire");

	/* 10 ms later, the host STILL has not clocked anything: quiet_ms grew by
	 * the same 10 ms as now_ms (nothing reset g_last_xfer_ms), so
	 * since_heal == quiet_ms's own growth -- must NOT re-fire. */
	zassert_false(link_quiet_rearm_tick(&st, true, false, 3010u, 3010u, REARM_MS, false),
	              "10 ms into the same wedge episode, with zero progress, must not re-fire");
	zassert_true(st.healed_pending, "the latch must still be armed -- no phase completed");

	/* Even much later in the same stuck episode. */
	zassert_false(link_quiet_rearm_tick(&st, true, false, 60000u, 60000u, REARM_MS, false),
	              "a permanently wedged link must fire ONCE, not every tick, until a phase "
	              "completes (SPI_close without transferCancel risks a DMA leak per-reinit)");
}

ZTEST(link_quiet_rearm, test_latch_clears_once_a_phase_completes_then_can_refire)
{
	link_quiet_rearm_state_t st = { 0 };

	/* Fire once at t=3000. */
	zassert_true(link_quiet_rearm_tick(&st, true, false, 3000u, 3000u, REARM_MS, false),
	             "setup: first fire");

	/* A phase completes at t=3200 (200 ms after the heal): g_last_xfer_ms
	 * jumps forward, so quiet_ms observed at t=3205 is small (5 ms) while
	 * since_heal is 205 ms -- quiet_ms < since_heal proves a NEW phase
	 * landed after the heal fired, so the latch must clear.  5 ms is also
	 * short of REARM_MS, so this same call must still report false (the
	 * latch clearing and a fresh fire are two different questions). */
	zassert_false(link_quiet_rearm_tick(&st, true, false, 5u, 3205u, REARM_MS, false),
	              "quiet_ms just dropped to 5 ms -- nowhere near a fresh 3 s wedge yet");
	zassert_false(st.healed_pending, "a genuinely completed phase must clear the latch");

	/* Time passes with no further traffic; once quiet_ms crosses REARM_MS
	 * again (a SECOND, independent wedge), the detector must fire again. */
	zassert_true(
	    link_quiet_rearm_tick(&st, true, false, REARM_MS, 3205u + REARM_MS, REARM_MS, false),
	    "a second, independent quiet episode after the latch cleared must fire again");
}

ZTEST(link_quiet_rearm, test_latch_reset_boundary_is_strict_less_than)
{
	/* Pins the exact `quiet_ms < since_heal` boundary (not <=) -- a mutation
	 * that flips this to <= or > would only be caught by testing the EQUAL
	 * case directly. */
	link_quiet_rearm_state_t st = { 0 };

	zassert_true(link_quiet_rearm_tick(&st, true, false, 3000u, 3000u, REARM_MS, false),
	             "setup: first fire");

	/* now_ms = 3300 (300 ms since the heal); quiet_ms reported as EXACTLY
	 * 300 as well -- since_heal == quiet_ms, the boundary case.  No NEW
	 * information: this quiet_ms is exactly consistent with "nothing has
	 * completed since the heal, quiet_ms grew in lockstep with wall time" --
	 * must NOT be read as a completed phase. */
	zassert_false(link_quiet_rearm_tick(&st, true, false, 300u, 3300u, REARM_MS, false),
	              "quiet_ms == since_heal is the no-new-evidence boundary -- must stay latched");
	zassert_true(st.healed_pending, "the boundary case must not clear the latch");
}

ZTEST(link_quiet_rearm, test_gated_mid_episode_does_not_clear_or_advance_latch)
{
	/* An OTA session starting mid-wedge-episode must not let the gate itself
	 * masquerade as "a phase completed" -- the function must simply skip the
	 * tick, latch untouched, so the SAME episode is still latched once the
	 * gate lifts. */
	link_quiet_rearm_state_t st = { 0 };

	zassert_true(link_quiet_rearm_tick(&st, true, false, 3000u, 3000u, REARM_MS, false),
	             "setup: first fire");

	zassert_false(link_quiet_rearm_tick(&st, true, false, 0u, 3000u, REARM_MS, true /* gated */),
	              "a gated tick must report false unconditionally");
	zassert_true(st.healed_pending, "a gated tick must leave the latch exactly as it was");

	/* Gate lifts; still the same stuck episode (no progress) -- must not
	 * re-fire immediately just because a gated tick happened in between. */
	zassert_false(link_quiet_rearm_tick(&st, true, false, 3010u, 3010u, REARM_MS, false),
	              "after the gate lifts, the same ungated episode must still be latched");
}
