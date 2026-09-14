/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * Unit tests for src/wifi_retry.c -- the pure retry-delay + reason-restore
 * decisions behind cc3501e_hw_wifi_connect_sta()'s bounded reason-30 retry.
 * This is the host-testable half of that fix: the real Wlan_Connect/event
 * plumbing (hal/ti/cc3501e_hw_ti_wifi.c) needs the vendored TI SimpleLink
 * SDK and is built ONLY for CC3501E_HAL_BACKEND=ti, never linked into a host
 * test binary.  wifi_retry.h's whole reason for existing is to pull the two
 * DECISIONS that fix actually turns on -- which delay, and whether to
 * restore -- out into something silicon-free that CAN be exercised here. */

#include <zephyr/ztest.h>

#include "wifi_retry.h"

ZTEST_SUITE(cc3501e_wifi_retry, NULL, NULL, NULL, NULL, NULL);

/* AUTHENTICATION_REJECTED(30) deny-lists the BSSID in OUR OWN driver for the
 * full ~10 s expiry (drv_ti_internal.h:370) -- the retry must wait the LONG
 * (denylist) delay, not the short comeback one. */
ZTEST(cc3501e_wifi_retry, test_delay_auth_event_uses_denylist_delay)
{
	const uint32_t delay =
	    wifi_retry_delay_ms(WIFI_RETRY_EVENT_AUTHENTICATION_REJECTED, 1000u, 11000u);
	zassert_equal(delay, 11000u, "AUTHENTICATION_REJECTED must wait the denylist delay");
}

/* ASSOCIATION_REJECTED(30) with a comeback-time IE deny-lists only for the
 * AP's own short comeback interval (drv_ti_mlme.c:1293-1327) -- the retry
 * only needs the SHORT (comeback) delay here. */
ZTEST(cc3501e_wifi_retry, test_delay_assoc_event_uses_comeback_delay)
{
	const uint32_t delay =
	    wifi_retry_delay_ms(WIFI_RETRY_EVENT_ASSOCIATION_REJECTED, 1000u, 11000u);
	zassert_equal(delay, 1000u, "ASSOCIATION_REJECTED must wait only the comeback delay");
}

/* A DISCONNECT's ReasonCode landing on 30 is an un-excluded, un-proven shape
 * (see wifi_retry.h) -- the conservative default (denylist) applies, same as
 * AUTH, so this can never under-wait. */
ZTEST(cc3501e_wifi_retry, test_delay_disconnect_event_uses_denylist_delay)
{
	const uint32_t delay = wifi_retry_delay_ms(WIFI_RETRY_EVENT_DISCONNECT, 1000u, 11000u);
	zassert_equal(delay, 11000u, "DISCONNECT must default to the conservative denylist delay");
}

/* NONE (nothing recorded -- should not happen at the one call site that reads
 * this, since retry eligibility already required a recorded reason of
 * exactly 30) also gets the conservative default. */
ZTEST(cc3501e_wifi_retry, test_delay_none_event_uses_denylist_delay)
{
	const uint32_t delay = wifi_retry_delay_ms(WIFI_RETRY_EVENT_NONE, 1000u, 11000u);
	zassert_equal(delay, 11000u, "NONE must default to the conservative denylist delay");
}

/* Reason 3 (WLAN_REASON_DEAUTH_LEAVING) on a retry pass is our own pre-retry
 * disconnect cleanup or hostap's generic SME give-up timer replaying itself --
 * never a fresh, informative AP verdict -- so it must be restored. */
ZTEST(cc3501e_wifi_retry, test_restore_true_for_reason_3)
{
	zassert_true(wifi_retry_should_restore_first_pass(3),
	             "reason 3 (WLAN_REASON_DEAUTH_LEAVING) must restore the first pass");
}

/* Reason 0 (nothing recorded on the retry pass at all) must also restore. */
ZTEST(cc3501e_wifi_retry, test_restore_true_for_reason_0)
{
	zassert_true(wifi_retry_should_restore_first_pass(0),
	             "reason 0 (nothing recorded) must restore the first pass");
}

/* A real AP-issued reject code on the retry pass (e.g. 17,
 * AP_UNABLE_TO_HANDLE_NEW_STA) is the retry's OWN genuine outcome and must be
 * kept, not overwritten by the first pass's. */
ZTEST(cc3501e_wifi_retry, test_restore_false_for_other_reason)
{
	zassert_false(wifi_retry_should_restore_first_pass(17),
	              "a real AP reject code on the retry pass must be published as-is");
}
