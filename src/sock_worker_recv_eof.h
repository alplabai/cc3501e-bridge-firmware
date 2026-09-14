/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * cc3501e-bridge firmware: worker-path sticky-EOF latch decision + the
 * bounded table operations it drives.
 *
 * ===================== WHY THIS EXISTS =====================
 * hal/ti/cc3501e_hw_ti_sock.c's cc3501e_hw_sock_recv() records an orderly
 * TCP close (n == 0 from lwip_recvfrom()) in a per-fd sock_eof[] table, so a
 * LATER recv on that fd answers OK/0 without calling lwIP again -- see that
 * file's own block comment above sock_eof for the ENOTCONN-after-FIN bug
 * this closes.
 *
 * BLOCKER (host review of bfb5f08): a recv with @p want == 0 (max_len or cap
 * clamped to 0 -- alp-sdk's own cc3501e_sock_recv() API explicitly allows
 * this: `if (buf == NULL && cap > 0u) return ALP_ERR_INVAL;` rejects a NULL
 * buffer only when cap is nonzero, so cap == 0 with any buffer, including
 * NULL, is a legal call.  Traced against alp-sdk's own
 * src/zephyr/console/alp_console_companion_sock.c's companion_serve_one(),
 * the one in-tree caller that computes its own cap from remaining buffer
 * space (`sizeof(req) - used - 1u`): that loop's own `used >= sizeof(req) -
 * 1u` break fires before cap would ever reach 0 in its CURRENT form, so
 * this is not a reachable-today caller, only a legal one -- firmware
 * correctness must not depend on what any particular caller happens to do)
 * ALSO returns n == 0 from lwip_recvfrom(), but for a completely different
 * reason: TI's
 * lwip_recv_tcp() given a requested length of 0 copies zero bytes and
 * returns 0 with errno 0 -- it never reaches the ERR_CLSD path at all, so
 * this says NOTHING about whether the peer actually closed.  The original
 * fix latched on n == 0 alone (gated only on SOCK_STREAM), so a single
 * want == 0 poll on a perfectly live socket permanently latched a FALSE
 * EOF, and the host's own 3-zero completion heuristic then reported a
 * truncated stream as finished -- silent data loss, worse than the bug
 * being fixed.  FIX: only latch when @p want > 0 too.
 *
 * PURE, SILICON-FREE, exactly the same split as sock_prefetch_arm.h /
 * sock_recv_commit.h / sock_recv_ring_status.h: the real call site
 * (hal/ti/cc3501e_hw_ti_sock.c) is TI-SDK-only (needs lwIP/CC3501E_WIFI)
 * and never linked on the host, so splitting the DECISION -- and the table
 * ops it drives -- out this way is what makes either unit-testable at all.
 * The table itself stays a CALLER-OWNED bool[] + length rather than a fixed
 * size baked in here: the real table's size is MEMP_NUM_NETCONN, a macro
 * only visible after <lwip/sockets.h>, which this header must not depend
 * on to stay linkable on the host.
 * ============================================================
 */

#ifndef CC3501E_BRIDGE_SOCK_WORKER_RECV_EOF_H
#define CC3501E_BRIDGE_SOCK_WORKER_RECV_EOF_H

#include <stdbool.h>

/*
 * sock_worker_recv_eof_should_latch -- given this call's own lwip_recvfrom()
 * result, should the fd's sticky EOF bit be set?
 *
 *   @p n         the byte count lwip_recvfrom() returned.  Callers only
 *                reach this for n >= 0 (a negative/error result is handled
 *                separately and never latches).
 *   @p want      the byte count actually asked for this call
 *                (min(max_len, cap)).
 *   @p is_stream true for a SOCK_STREAM (TCP) socket.
 *
 * True only for n == 0 && want > 0 && is_stream -- a genuine orderly TCP
 * close.  n == 0 with want == 0 asked lwIP for nothing and proves nothing
 * about the peer (the BLOCKER this header exists to close); n == 0 on a
 * DGRAM socket is a legitimate empty UDP datagram, not EOF. */
static inline bool sock_worker_recv_eof_should_latch(int n, unsigned want, bool is_stream)
{
	return n == 0 && want > 0u && is_stream;
}

/*
 * sock_worker_recv_eof_check / _set -- bounded operations over a
 * caller-owned bool[] table indexed by fd.  An out-of-range fd (< 0 or >=
 * @p table_len) is always a silent no-op -- never a fault, never a partial
 * write -- so the real call site's fixed-size sock_eof[MEMP_NUM_NETCONN]
 * never needs its own bounds check duplicated at every use. */
static inline bool sock_worker_recv_eof_check(const bool *table, int table_len, int fd)
{
	return fd >= 0 && fd < table_len && table[fd];
}

static inline void sock_worker_recv_eof_set(bool *table, int table_len, int fd, bool value)
{
	if (fd >= 0 && fd < table_len) {
		table[fd] = value;
	}
}

#endif /* CC3501E_BRIDGE_SOCK_WORKER_RECV_EOF_H */
