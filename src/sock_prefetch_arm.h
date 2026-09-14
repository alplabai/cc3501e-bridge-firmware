/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * cc3501e-bridge firmware: SOCK_RECV prefetch ring's arm-decision arithmetic.
 *
 * ===================== WHY THIS EXISTS =====================
 * hal/ti/cc3501e_hw_ti_sock.c's cc3501e_hw_sock_prefetch(handle, true) is
 * called from cc3501e_hw_sock_connect(), gated on the connected socket's own
 * SO_TYPE being SOCK_STREAM (MINOR, host review of c354208 -- an earlier
 * version called it for ANY connected socket, and a connected UDP/SOCK_DGRAM
 * socket then held the ring under this file's own first-connect-wins rule,
 * locking out the STREAM socket this fast path actually exists for).  It
 * used to re-arm the SINGLE prefetch ring for the NEW handle
 * UNCONDITIONALLY, even when a DIFFERENT handle was already armed and
 * holding pumped-but-unserved bytes in it (MAJOR, host review, 1118c99):
 * re-arming resets rx_ring.head/tail/uncommitted to 0, silently discarding
 * whatever the FIRST handle's socket had already delivered into the ring.
 * The first handle then falls to the worker-routed path
 * (cc3501e_hw_sock_recv_ring() returns -1 for a handle that is not the
 * armed one) and reads PAST the hole with lwip_recvfrom(), reporting OK --
 * any application holding two concurrent TCP client sockets hit this the
 * moment the second one connected.
 *
 * FIX: do not re-arm while a DIFFERENT handle is already armed.  The FIRST
 * handle keeps the ring; every OTHER concurrently-open handle uses the
 * worker-routed path (protocol_sockets.c's own worker-fallback replay cache
 * covers it) for its whole lifetime, or until the armed handle closes and
 * this decision is asked again for a later connect.  Re-arming for the SAME
 * handle while it is still armed (sock_prefetch_should_arm() also returns
 * true for that) is a TRUE no-op at the call site -- it does NOT reset the
 * ring (MINOR, host review of c354208; an earlier version reset it even
 * here, dropping that same handle's own not-yet-served bytes).  A handle
 * NUMBER the host reuses after SOCK_CLOSE always arrives as armed == 0, not
 * armed == requested, since close disarms first -- so that reuse case is the
 * ordinary cold arm, not this same-handle path.
 *
 * PURE ARITHMETIC, SILICON-FREE.  This file owns ONLY the arm/no-arm
 * decision -- one comparison, no ring buffer, no lwIP -- so it links and
 * runs identically on the host, exactly like sock_recv_commit.h's own
 * tail/uncommitted arithmetic is split out for the identical reason:
 * hal/ti/cc3501e_hw_ti_sock.c's cc3501e_hw_sock_prefetch() is the sole
 * caller, and it is TI-SDK-only (needs lwIP/CC3501E_WIFI), never buildable
 * on the host.  Splitting the DECISION out this way makes it unit-testable
 * where the ring itself cannot be linked at all.
 * ============================================================
 */

#ifndef CC3501E_BRIDGE_SOCK_PREFETCH_ARM_H
#define CC3501E_BRIDGE_SOCK_PREFETCH_ARM_H

#include <stdint.h>

/*
 * sock_prefetch_should_arm -- may the ring be (re-)armed for @p requested?
 *
 *   @p armed      the ring's CURRENT fd_plus1 (0 = nothing armed).
 *   @p requested  the handle a STREAM socket just connected on and wants to
 *                 prefetch for.
 *
 * Returns true when the ring is free (@p armed == 0) or already armed for
 * THIS SAME handle -- the latter is a TRUE no-op at the call site (MINOR,
 * host review of c354208): it does NOT reset the ring, since the handle
 * asking is the one already holding it, and a reset here would drop that
 * SAME handle's own pumped-but-not-yet-served bytes.  NOT the same case as
 * a handle NUMBER the host reuses after SOCK_CLOSE -- close disarms first
 * (cc3501e_hw_sock_close()'s own unconditional disarm call), so a reused
 * number always arrives here as @p armed == 0, the ordinary cold-arm case
 * above, never @p armed == @p requested.  Returns false when a DIFFERENT
 * handle currently holds the ring, so the caller must leave it alone rather
 * than reset it out from under that handle's pumped-but-unserved bytes.
 *
 * TEST COVERAGE: tests/unit/sock_prefetch_arm/ covers this function's own
 * three branches directly.  The hal/ti/cc3501e_hw_ti_sock.c call site
 * itself -- the SO_TYPE gate at cc3501e_hw_sock_connect(), and the
 * same-handle-no-op skip inside cc3501e_hw_sock_prefetch() -- is TI-SDK-only
 * (needs lwIP/CC3501E_WIFI) and cannot link on the host, so it is covered
 * only by review and by the run12 two-concurrent-socket bench, not by any
 * host unit test. */
static inline int sock_prefetch_should_arm(uint16_t armed, uint16_t requested)
{
	return (armed == 0u) || (armed == requested);
}

#endif /* CC3501E_BRIDGE_SOCK_PREFETCH_ARM_H */
