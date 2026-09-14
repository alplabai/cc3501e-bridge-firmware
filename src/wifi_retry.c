/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * See wifi_retry.h for the contract on both functions -- this file has no
 * dependency on the rest of the firmware (no protocol.c, no worker.c, no HAL,
 * no TI SDK) by design, so it is fully host-testable pulled out of
 * hal/ti/cc3501e_hw_ti_wifi.c.
 */

#include "wifi_retry.h"

uint32_t wifi_retry_delay_ms(wifi_retry_event_t event,
                             uint32_t           comeback_delay_ms,
                             uint32_t           denylist_delay_ms)
{
	if (event == WIFI_RETRY_EVENT_ASSOCIATION_REJECTED) {
		return comeback_delay_ms;
	}
	return denylist_delay_ms;
}

bool wifi_retry_should_restore_first_pass(int16_t retry_pass_reason)
{
	return retry_pass_reason == 3 /* WLAN_REASON_DEAUTH_LEAVING */ || retry_pass_reason == 0;
}

int16_t wifi_retry_sanitize_reason(int16_t reason, bool own_disconnect_issued)
{
	if (reason == 3 /* WLAN_REASON_DEAUTH_LEAVING */ && own_disconnect_issued) {
		return 0;
	}
	return reason;
}
