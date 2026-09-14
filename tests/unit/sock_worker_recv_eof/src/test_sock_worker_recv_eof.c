/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * Unit tests for sock_worker_recv_eof.h -- the worker-path sticky-EOF latch
 * DECISION (sock_worker_recv_eof_should_latch()) and the bounded table ops
 * (sock_worker_recv_eof_check() / _set()) behind
 * hal/ti/cc3501e_hw_ti_sock.c's cc3501e_hw_sock_recv() fix.  Split out of
 * that TI-SDK-only (lwIP, CC3501E_WIFI) file the same way
 * sock_prefetch_arm.h / sock_recv_commit.h / sock_recv_ring_status.h
 * already are, so it is host-testable where the real HAL cannot be linked
 * at all.
 *
 * Covers the host review of bfb5f08's own mutation probes directly:
 *   - M1 (drop the SOCK_STREAM gate): test_udp_n_zero_does_not_latch.
 *   - M2 (drop clear-on-close): test_clear_resets_the_bit.
 *   - the BLOCKER (drop the want > 0 gate): test_tcp_want_zero_does_not_latch.
 *
 * NOT COVERED HERE: the call site itself -- cc3501e_hw_sock_recv()'s own
 * want == 0 short-circuit (which must not call lwIP at all, not just not
 * latch) and sock_is_stream()'s SO_TYPE probe -- lives in
 * hal/ti/cc3501e_hw_ti_sock.c and cannot link on the host.  Covered only by
 * review; not yet bench-verified. */

#include <stdbool.h>
#include <zephyr/ztest.h>

#include "sock_worker_recv_eof.h"

ZTEST_SUITE(sock_worker_recv_eof, NULL, NULL, NULL, NULL, NULL);

/* ---- sock_worker_recv_eof_should_latch() -------------------------------- */

ZTEST(sock_worker_recv_eof, test_udp_n_zero_does_not_latch)
{
	/* M1: an earlier version of the call site dropped the is_stream gate
	 * entirely -- a 0-byte UDP datagram (n == 0, want > 0, DGRAM) is
	 * legitimate and must never latch EOF. */
	zassert_false(sock_worker_recv_eof_should_latch(0, 64u, false),
	              "a 0-byte UDP recv is a legitimate empty datagram, not EOF");
}

ZTEST(sock_worker_recv_eof, test_tcp_want_zero_does_not_latch)
{
	/* THE BLOCKER: want == 0 on a STREAM socket returns n == 0 from TI's
	 * lwip_recv_tcp() (a plain 0-byte copy, errno 0) whether or not the
	 * peer has closed -- must never latch. */
	zassert_false(sock_worker_recv_eof_should_latch(0, 0u, true),
	              "want == 0 proves nothing about the peer -- must not latch");
}

ZTEST(sock_worker_recv_eof, test_tcp_n_zero_want_positive_latches)
{
	/* The genuine case this table exists for: an orderly TCP close. */
	zassert_true(sock_worker_recv_eof_should_latch(0, 64u, true),
	             "n == 0, want > 0, STREAM -- an orderly close, must latch");
}

ZTEST(sock_worker_recv_eof, test_tcp_positive_n_does_not_latch)
{
	zassert_false(sock_worker_recv_eof_should_latch(12, 64u, true),
	              "data actually arrived -- not EOF");
}

/* ---- sock_worker_recv_eof_check() / _set() ------------------------------ */

ZTEST(sock_worker_recv_eof, test_set_then_check_latches_the_bit)
{
	bool table[4] = { false, false, false, false };

	sock_worker_recv_eof_set(table, 4, 2, true);
	zassert_true(sock_worker_recv_eof_check(table, 4, 2), "fd 2 now latched");
	zassert_false(sock_worker_recv_eof_check(table, 4, 1), "a DIFFERENT fd is untouched");
}

ZTEST(sock_worker_recv_eof, test_clear_resets_the_bit)
{
	/* M2: an earlier version of the call site no-op'd the clear-on-close. */
	bool table[4] = { false, false, false, false };

	sock_worker_recv_eof_set(table, 4, 2, true);
	zassert_true(sock_worker_recv_eof_check(table, 4, 2), "latched before close");

	sock_worker_recv_eof_set(table, 4, 2, false);
	zassert_false(sock_worker_recv_eof_check(table, 4, 2),
	              "cc3501e_hw_sock_close()'s clear must reset the bit, or a reused "
	              "fd number starts pre-latched");
}

ZTEST(sock_worker_recv_eof, test_out_of_range_fd_is_a_no_op)
{
	bool table[4] = { false, false, false, false };

	/* Above the table: check is false, set touches nothing (no overrun -- if
	 * it did, ASan/UBSan in this suite's build would already have caught it,
	 * but the point under test is the RETURN behaviour). */
	zassert_false(sock_worker_recv_eof_check(table, 4, 10),
	              "fd >= table_len -> false, not a fault");
	sock_worker_recv_eof_set(table, 4, 10, true);
	zassert_false(sock_worker_recv_eof_check(table, 4, 3), "the out-of-range set touched nothing");

	/* Negative fd (defensive -- production never calls with one, since
	 * fd = handle - 1 and handle == 0 is rejected before this table is ever
	 * touched, but the helper must not fault if it ever were). */
	zassert_false(sock_worker_recv_eof_check(table, 4, -1), "negative fd -> false, not a fault");
	sock_worker_recv_eof_set(table, 4, -1, true);
	zassert_false(sock_worker_recv_eof_check(table, 4, 0), "the negative-fd set touched nothing");
}
