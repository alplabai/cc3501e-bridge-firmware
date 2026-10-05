/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * cc3501e-bridge firmware: slice arithmetic for the non-blocking SOCK_CONNECT
 * wait in hal/ti/cc3501e_hw_ti_sock.c.
 *
 * A blocking lwip_connect() runs 12-21 s (SYN retransmits) inside the worker
 * body, and worker_run_pending() and cc3501e_hw_tick() -> cc3501e_hw_link_heal()
 * share ONE task loop (src/main.c), so every SPI self-heal is frozen for that
 * whole time.  The connect is therefore O_NONBLOCK plus a wait split into short
 * slices, with cc3501e_hw_link_heal(false) run between them.  This header owns
 * only the "how long may the next slice wait" decision so it is host-testable;
 * the lwIP calls themselves are TI-SDK-only and never link on the host.
 */
#ifndef CC3501E_SOCK_CONNECT_SLICE_H
#define CC3501E_SOCK_CONNECT_SLICE_H

#include <stdint.h>

/* Milliseconds the next wait slice may last: min(slice_ms, budget_ms - elapsed_ms),
 * or 0 when the budget is spent (caller gives up with a timeout).  elapsed_ms is
 * an unsigned difference of two uptime stamps, so it is wrap-safe by construction. */
static inline uint32_t sock_connect_slice_ms(uint32_t elapsed_ms,
                                             uint32_t budget_ms,
                                             uint32_t slice_ms)
{
	if (elapsed_ms >= budget_ms) {
		return 0u;
	}
	const uint32_t left = budget_ms - elapsed_ms;
	return left < slice_ms ? left : slice_ms;
}

#endif /* CC3501E_SOCK_CONNECT_SLICE_H */
