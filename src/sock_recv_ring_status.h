/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * cc3501e-bridge firmware: SOCK_RECV prefetch ring's drained-status decision.
 *
 * ===================== WHY THIS EXISTS =====================
 * hal/ti/cc3501e_hw_ti_sock.c's cc3501e_hw_sock_pump() used to ignore a real
 * lwip_recv() failure (RST etc, n < 0 with errno neither EAGAIN nor
 * EWOULDBLOCK) entirely -- only n == 0 (orderly close) was ever recorded.
 * After an RST on the currently-armed handle, cc3501e_hw_sock_recv_ring()
 * therefore had nothing to report but "armed but empty" (-2, BUSY) forever:
 * the pump could never refill a dead socket, so every poll answered BUSY
 * until the host gave up at its own timeout_ms and reported ALP_ERR_TIMEOUT
 * -- not the actual failure.
 *
 * FIX: the pump now also latches a STICKY rx_ring.peer_error on a real
 * lwip_recv() failure (hal/ti/cc3501e_hw_ti_sock.c), mirroring the STICKY
 * rx_ring.peer_closed it already keeps for an orderly close.  Once
 * sock_recv_commit() reports nothing left to serve, this function is the
 * whole DECISION of what cc3501e_hw_sock_recv_ring() answers: EOF (0),
 * a terminal failure (-3), or "empty for now, more may arrive" (-2).
 *
 * peer_closed and peer_error can never BOTH be true for the same arm cycle:
 * cc3501e_hw_sock_pump()'s own top-of-function guard returns immediately
 * once EITHER is set, so whichever the pump records first is the last thing
 * it ever records until the next arm resets both -- there is no call
 * ordering where the pump could set the second one after the first.  This
 * function still checks peer_closed FIRST regardless, as defensive
 * precedence: an EOF ALWAYS answers as EOF, never as an error, even if that
 * invariant were ever violated by a future change to the pump.
 *
 * PURE ARITHMETIC, SILICON-FREE -- two booleans in, one of three sentinels
 * out, no ring buffer, no lwIP -- exactly the same split sock_prefetch_arm.h
 * and sock_recv_commit.h already use for the same reason:
 * hal/ti/cc3501e_hw_ti_sock.c is TI-SDK-only (needs lwIP/CC3501E_WIFI) and
 * never linked on the host, so splitting the DECISION out this way is what
 * makes it unit-testable where the ring itself cannot be.
 * ============================================================
 */

#ifndef CC3501E_BRIDGE_SOCK_RECV_RING_STATUS_H
#define CC3501E_BRIDGE_SOCK_RECV_RING_STATUS_H

#include <stdbool.h>

/*
 * sock_recv_ring_drained_status -- once sock_recv_commit() has reported
 * nothing left to serve for this call, decide what
 * cc3501e_hw_sock_recv_ring() should return.
 *
 *   @p peer_closed  rx_ring.peer_closed -- the pump saw an orderly close
 *                   (lwip_recv() n == 0) on the armed fd.
 *   @p peer_error   rx_ring.peer_error -- the pump saw a REAL lwip_recv()
 *                   failure (n < 0, errno neither EAGAIN nor EWOULDBLOCK) on
 *                   the armed fd.
 *
 * Returns:
 *   0   EOF.  The stream ended in an orderly way; the caller answers
 *       RESP_OK with 0 bytes, matching the worker path's own "peer closed"
 *       contract.
 *  -3   Terminal failure.  Nothing further will ever arrive on this fd (the
 *       pump's own guard stops calling lwip_recv() on it once this is set),
 *       so the caller must NOT answer BUSY (that would just spin the host to
 *       its poll_by_repeat timeout) -- it answers the same status a genuine
 *       worker-path socket failure gets.
 *  -2   Armed but momentarily empty, peer still connected.  The caller
 *       answers BUSY so the host re-polls; the pump may still refill the
 *       ring by the time it does. */
static inline int sock_recv_ring_drained_status(bool peer_closed, bool peer_error)
{
	if (peer_closed) {
		return 0;
	}
	if (peer_error) {
		return -3;
	}
	return -2;
}

#endif /* CC3501E_BRIDGE_SOCK_RECV_RING_STATUS_H */
