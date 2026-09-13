/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * See sock_recv_commit.h for why this exists and exactly what it does and
 * does not own.
 */

#include "sock_recv_commit.h"

uint32_t
sock_recv_commit(uint32_t *tail, uint32_t *uncommitted, uint32_t head, bool replay, uint32_t cap)
{
	if (!replay) {
		/* The previous call's serve is now confirmed collected (a
		 * DIFFERENT request arrived, so poll_by_repeat() is not retrying
		 * the one that served *uncommitted bytes) -- fold it into the
		 * committed tail. */
		*tail += *uncommitted;
	}
	/* Either way, THIS call's own serve (computed below) is what is
	 * uncommitted from here on -- the previous count has either just been
	 * folded in above, or (on a replay) is being re-served identically and
	 * superseded by this call's own count, which is the same value again. */
	*uncommitted = 0u;

	/* Free-running; unsigned wrap is correct here, exactly like
	 * hal/ti/cc3501e_hw_ti_sock.c's own ring_used(). */
	const uint32_t used = head - *tail;
	const uint32_t n    = (used < cap) ? used : cap;

	*uncommitted = n;
	return n;
}
