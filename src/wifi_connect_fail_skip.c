/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * See wifi_connect_fail_skip.h for the contract -- this file has no
 * dependency on the rest of the firmware (no protocol.c, no worker.c, no
 * HAL, no TI SDK) by design, so it is fully host-testable pulled out of
 * hal/ti/cc3501e_hw_ti_wifi.c.
 */

#include "wifi_connect_fail_skip.h"

bool wifi_connect_fail_skip_reinit(uint32_t txn_count_at_reinit, uint32_t txn_count_now)
{
	return txn_count_now != txn_count_at_reinit;
}
