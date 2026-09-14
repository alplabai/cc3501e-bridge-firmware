/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * Unit tests for src/wifi_connect_fail_skip.c -- the pure "has the slave
 * answered a host frame since the connect body's own last reinit" decision
 * behind the WIFI_CONNECT_STA failure-exit drain-reinit skip.  The real
 * Wlan_Connect body + g_host_txn_count plumbing (hal/ti/cc3501e_hw_ti.c,
 * hal/ti/cc3501e_hw_ti_wifi.c) needs the vendored TI SimpleLink SDK and is
 * built ONLY for CC3501E_HAL_BACKEND=ti, never linked into a host test
 * binary -- this is the host-testable half of that fix.
 */

#include <zephyr/ztest.h>

#include "wifi_connect_fail_skip.h"

ZTEST_SUITE(cc3501e_wifi_connect_fail_skip, NULL, NULL, NULL, NULL, NULL);

/* The common case the fix exists for: at least one host frame completed
 * since the reinit -- the slave is demonstrably still armed, so the drain
 * may skip paying a second, destructive reinit. */
ZTEST(cc3501e_wifi_connect_fail_skip, test_count_advanced_by_one_skips)
{
	zassert_true(wifi_connect_fail_skip_reinit(10u, 11u),
	             "a single advanced frame must permit the skip");
}

/* Several frames served -- still just "advanced", still skips. */
ZTEST(cc3501e_wifi_connect_fail_skip, test_count_advanced_by_many_skips)
{
	zassert_true(wifi_connect_fail_skip_reinit(10u, 500u),
	             "many advanced frames must still permit the skip");
}

/* The built-in falsifier: NOTHING completed since the reinit -- the slave
 * may have gone dead partway through this attempt, so the caller must still
 * reinit as before this fix. */
ZTEST(cc3501e_wifi_connect_fail_skip, test_count_unchanged_does_not_skip)
{
	zassert_false(wifi_connect_fail_skip_reinit(42u, 42u),
	              "no frame served since the reinit must NOT permit the skip");
}

/* Both zero (a cold boot, or the accessor never having incremented) is the
 * same "unchanged" case, pinned separately since 0 is also often an
 * off-by-one edge for an unsigned comparison. */
ZTEST(cc3501e_wifi_connect_fail_skip, test_count_both_zero_does_not_skip)
{
	zassert_false(wifi_connect_fail_skip_reinit(0u, 0u),
	              "two zero samples must NOT permit the skip");
}

/* Wrap-around of the counter type: even though g_host_txn_count itself
 * SATURATES rather than wraps (see wifi_connect_fail_skip.h), the decision
 * is a plain inequality and must stay correct if the counter type ever did
 * wrap -- "now" landing numerically BELOW "at_reinit" after a wrap is still
 * "the two differ", i.e. still a served frame, and must still skip. */
ZTEST(cc3501e_wifi_connect_fail_skip, test_wrap_around_low_now_still_skips)
{
	zassert_true(wifi_connect_fail_skip_reinit(UINT32_MAX, 0u),
	             "a wrapped counter (now < at_reinit) must still count as advanced");
}

/* The UINT32_MAX/UINT32_MAX unchanged case at the saturated ceiling itself --
 * pins the boundary the real (saturating) counter can actually reach. */
ZTEST(cc3501e_wifi_connect_fail_skip, test_saturated_ceiling_unchanged_does_not_skip)
{
	zassert_false(wifi_connect_fail_skip_reinit(UINT32_MAX, UINT32_MAX),
	              "two identical saturated-ceiling samples must NOT permit the skip");
}
