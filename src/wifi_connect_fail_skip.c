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

bool wifi_wait_host_frame(uint32_t (*count_fn)(void),
                          void (*sleep_ms_fn)(uint32_t),
                          uint32_t window_ms,
                          uint32_t step_ms)
{
	const uint32_t baseline = count_fn(); /* THIS call's own sample -- see the header. */

	for (uint32_t elapsed_ms = 0u; elapsed_ms < window_ms; elapsed_ms += step_ms) {
		sleep_ms_fn(step_ms);
		if (count_fn() != baseline) {
			return true; /* a frame landed -- stop polling immediately. */
		}
	}
	return false; /* window exhausted with no change: no evidence of life. */
}
