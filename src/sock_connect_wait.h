/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * cc3501e-bridge firmware: the non-blocking SOCK_CONNECT wait loop behind
 * cc3501e_hw_sock_connect() (hal/ti/cc3501e_hw_ti_sock.c), with every lwIP call
 * behind an ops table so the branches are host-testable.
 *
 * A blocking lwip_connect() runs 12-21 s inside the worker body, and
 * worker_run_pending() and cc3501e_hw_tick() -> cc3501e_hw_link_heal() share ONE
 * task loop (src/main.c), so every SPI self-heal was frozen that whole time
 * (finding 4, bench 2026-10-04 v0.9.0).  So: O_NONBLOCK connect, then wait in
 * short slices, running a heal between slices that timed out.
 *
 * The loop ends when select reports (lwIP's own SYN give-up, TCP_SYNMAXRTX,
 * always makes the socket writable/errored) -- NOT at a budget of our own: a cap
 * equal to lwIP's SYN limit could abandon a socket whose late SYN-ACK is still
 * in flight.  CC3501E_SOCK_CONNECT_CAP_MS is only a safety net above that.
 */
#ifndef CC3501E_SOCK_CONNECT_WAIT_H
#define CC3501E_SOCK_CONNECT_WAIT_H

#include <stdbool.h>
#include <stdint.h>

#define CC3501E_SOCK_CONNECT_SLICE_MS 100u
#define CC3501E_SOCK_CONNECT_CAP_MS   60000u

typedef struct {
	int (*get_flags)(void *ctx);                        /* <0 on failure */
	int (*set_flags)(void *ctx, int flags);             /* <0 on failure */
	int (*start_connect)(void *ctx);                    /* 0 / -1 */
	bool (*in_progress)(void *ctx);                     /* errno == EINPROGRESS after connect */
	int (*wait_writable)(void *ctx, uint32_t slice_ms); /* >0 ready, 0 timeout, <0 error */
	int (*so_error)(void *ctx, int *err_out);           /* 0 if the getsockopt itself worked */
	uint32_t (*now_ms)(void *ctx);
	void (*heal)(void *ctx); /* cc3501e_hw_link_heal(false) */
	int nonblock_flag;       /* O_NONBLOCK */
} sock_connect_ops_t;

/* Length of the next wait slice: min(slice, cap - elapsed), 0 once the cap is spent. */
static inline uint32_t sock_connect_slice_ms(uint32_t elapsed_ms)
{
	if (elapsed_ms >= CC3501E_SOCK_CONNECT_CAP_MS) {
		return 0u;
	}
	const uint32_t left = CC3501E_SOCK_CONNECT_CAP_MS - elapsed_ms;
	return left < CC3501E_SOCK_CONNECT_SLICE_MS ? left : CC3501E_SOCK_CONNECT_SLICE_MS;
}

/* Returns 0 connected, -1 failure.  The socket's ORIGINAL flags are restored on
 * EVERY exit once they were read; a failed restore is itself a failure, so the
 * caller never keeps a socket left non-blocking (the SOCK_RECV worker path
 * relies on SO_RCVTIMEO bounding a blocking recv). */
static inline int sock_connect_run(const sock_connect_ops_t *o, void *ctx)
{
	const int orig = o->get_flags(ctx);
	if (orig < 0) {
		return -1;
	}
	if (o->set_flags(ctx, orig | o->nonblock_flag) < 0) {
		return -1; /* never fall back to the unbounded blocking connect */
	}
	int rc = o->start_connect(ctx);
	if (rc != 0) {
		if (!o->in_progress(ctx)) {
			rc = -1;
		} else {
			const uint32_t t0 = o->now_ms(ctx);
			rc                = -1;
			for (;;) {
				const uint32_t slice = sock_connect_slice_ms(o->now_ms(ctx) - t0);
				if (slice == 0u) {
					break; /* safety cap */
				}
				const int n = o->wait_writable(ctx, slice);
				if (n > 0) {
					int err = 0;
					if (o->so_error(ctx, &err) == 0 && err == 0) {
						rc = 0;
					}
					break;
				}
				if (n < 0) {
					break;
				}
				o->heal(ctx);
			}
		}
	}
	if (o->set_flags(ctx, orig) < 0) {
		return -1;
	}
	return rc;
}

#endif /* CC3501E_SOCK_CONNECT_WAIT_H */
