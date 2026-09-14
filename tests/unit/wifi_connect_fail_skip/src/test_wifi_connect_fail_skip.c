/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * Unit tests for src/wifi_connect_fail_skip.c -- wifi_wait_host_frame(), the
 * pure poll/wait behind the WIFI_CONNECT_STA failure-exit drain-reinit skip.
 * The real Wlan_Connect body + g_host_txn_count plumbing (hal/ti/
 * cc3501e_hw_ti.c, hal/ti/cc3501e_hw_ti_wifi.c) needs the vendored TI
 * SimpleLink SDK and is built ONLY for CC3501E_HAL_BACKEND=ti, never linked
 * into a host test binary -- this is the host-testable half of that fix.
 *
 * count_fn / sleep_ms_fn are injected fakes (no vendored SDK, no real
 * clock): g_fake_values[] scripts what successive count_fn() calls return
 * (index 0 is the function's OWN baseline sample, per wifi_wait_host_frame's
 * contract -- it never takes an externally-supplied baseline); past the end
 * of the script the fake holds the last scripted value (no further change).
 * fake_sleep_ms_fn() does not actually sleep -- it only accumulates elapsed
 * time and a call count, so a test can assert the wait is BOUNDED without
 * a real clock.
 */

#include <zephyr/ztest.h>

#include "wifi_connect_fail_skip.h"

ZTEST_SUITE(cc3501e_wifi_connect_fail_skip, NULL, NULL, NULL, NULL, NULL);

static uint32_t g_fake_values[8];
static int      g_fake_num_values;
static int      g_fake_call_count;

static uint32_t fake_count_fn(void)
{
	const int idx = g_fake_call_count;

	g_fake_call_count++;
	if (idx < g_fake_num_values) {
		return g_fake_values[idx];
	}
	return g_fake_values[g_fake_num_values - 1]; /* past the script: hold steady. */
}

static uint32_t g_fake_total_slept_ms;
static uint32_t g_fake_sleep_call_count;

static void fake_sleep_ms_fn(uint32_t ms)
{
	g_fake_total_slept_ms += ms;
	g_fake_sleep_call_count++;
}

static void fake_reset(void)
{
	g_fake_call_count       = 0;
	g_fake_num_values       = 0;
	g_fake_total_slept_ms   = 0u;
	g_fake_sleep_call_count = 0u;
}

/* A frame lands on the SECOND polling check (elapsed 20 ms of a 150 ms
 * window): baseline (call 1) = 5, first check (call 2) = 5 (unchanged),
 * second check (call 3) = 6 (changed) -- must return true immediately,
 * without waiting out the rest of the 150 ms window. */
ZTEST(cc3501e_wifi_connect_fail_skip, test_frame_at_second_poll_returns_true_and_stops_early)
{
	fake_reset();
	g_fake_values[0]  = 5u;
	g_fake_values[1]  = 5u;
	g_fake_values[2]  = 6u;
	g_fake_num_values = 3;

	const bool served = wifi_wait_host_frame(fake_count_fn, fake_sleep_ms_fn, 150u, 10u);

	zassert_true(served, "a count change must report a served frame");
	zassert_equal(g_fake_sleep_call_count, 2u, "must stop polling the instant a frame lands");
	zassert_equal(g_fake_total_slept_ms, 20u, "must not sleep past the change");
}

/* A frame lands on the very FIRST polling check (elapsed 10 ms). */
ZTEST(cc3501e_wifi_connect_fail_skip, test_frame_at_first_poll_returns_true)
{
	fake_reset();
	g_fake_values[0]  = 100u;
	g_fake_values[1]  = 101u;
	g_fake_num_values = 2;

	const bool served = wifi_wait_host_frame(fake_count_fn, fake_sleep_ms_fn, 150u, 10u);

	zassert_true(served, "a change on the first check must still report served");
	zassert_equal(g_fake_sleep_call_count, 1u, "must stop after exactly one poll");
}

/* The built-in falsifier: the count NEVER changes -- no frame lands in the
 * whole 150 ms / 10 ms-step window.  Must return false AND the total sleep
 * must be bounded at exactly the window (15 steps of 10 ms), not run away. */
ZTEST(cc3501e_wifi_connect_fail_skip, test_no_frame_ever_returns_false_and_bounds_total_sleep)
{
	fake_reset();
	g_fake_values[0]  = 42u;
	g_fake_num_values = 1;

	const bool served = wifi_wait_host_frame(fake_count_fn, fake_sleep_ms_fn, 150u, 10u);

	zassert_false(served, "no change in the whole window must NOT report served");
	zassert_equal(g_fake_sleep_call_count, 15u, "must poll exactly window_ms/step_ms times");
	zassert_equal(g_fake_total_slept_ms, 150u, "total sleep must be bounded at window_ms");
}

/* Documents the shape of the ordering bug the blocker (host review of
 * 580f748) was about -- NOT a regression test for it: the actual bug lived
 * entirely in TI-only code (hal/ti/cc3501e_hw_ti_wifi.c's old
 * txn_count_after_reinit snapshot, taken at the ROLE-UP reinit, long before
 * any failure exit), which this host build never links or exercises, and
 * wifi_wait_host_frame() itself is a NEW function with no old
 * implementation to regress against -- there is no way for this test to
 * fail against "the old design".  What it DOES pin down is the CONTRACT
 * that makes the fix correct: the counter may have ALREADY advanced, by
 * unrelated host polling, before this wait is ever called -- simulated here
 * by a NON-ZERO count that is already "settled" the moment
 * wifi_wait_host_frame takes its own baseline sample.  A stale, externally-
 * supplied baseline would have reported this as "served" on the strength of
 * that EARLIER advance; this function takes its OWN fresh baseline as call
 * 1, so with nothing changing AFTER that sample it must report false,
 * exactly like the never-changes case above. */
ZTEST(cc3501e_wifi_connect_fail_skip, test_advance_before_this_calls_own_baseline_does_not_count)
{
	fake_reset();
	g_fake_values[0]  = 37u; /* already-advanced value, settled before this call started */
	g_fake_num_values = 1;

	const bool served = wifi_wait_host_frame(fake_count_fn, fake_sleep_ms_fn, 150u, 10u);

	zassert_false(served,
	              "an advance that happened before this call's own baseline sample "
	              "must not be reported as a served frame");
}

/* Saturation, not wrap: g_host_txn_count SATURATES at UINT32_MAX rather than
 * wrapping (cc3501e_hw_ti.c's cc3501e_hw_notify_reply_sent()), so the
 * ceiling itself is a value the real counter can actually reach and hold at
 * -- two samples at that ceiling must read as "unchanged", same as any other
 * steady value. */
ZTEST(cc3501e_wifi_connect_fail_skip, test_saturated_ceiling_steady_returns_false)
{
	fake_reset();
	g_fake_values[0]  = UINT32_MAX;
	g_fake_num_values = 1;

	const bool served = wifi_wait_host_frame(fake_count_fn, fake_sleep_ms_fn, 150u, 10u);

	zassert_false(served, "a steady value at the saturated ceiling must not report served");
}
