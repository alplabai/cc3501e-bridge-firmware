/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * Unit tests for sock_recv_ring_status.h's sock_recv_ring_drained_status() --
 * the bug 2 fix (ring path never reports a reset): cc3501e_hw_sock_pump()
 * used to ignore a real lwip_recv() failure (RST etc) entirely, so an armed,
 * drained ring with a dead peer answered BUSY forever and the host spun to
 * its own timeout instead of seeing the failure.  This is the pure
 * DECISION split out of that TI-SDK-only (lwIP, CC3501E_WIFI) file, the same
 * way sock_prefetch_arm.h's and sock_recv_commit.h's own arithmetic is
 * split out of it, so it is host-testable where the ring itself cannot be
 * linked at all.
 *
 * NOT COVERED HERE: the call site itself -- cc3501e_hw_sock_pump()'s n < 0
 * errno branch that actually sets rx_ring.peer_error, and the
 * top-of-function guard that stops the pump from calling lwip_recv() again
 * once either sticky flag is set -- lives in hal/ti/cc3501e_hw_ti_sock.c and
 * cannot link on the host.  Covered only by review and by a bench RST
 * injection. */

#include <zephyr/ztest.h>

#include "sock_recv_ring_status.h"

ZTEST_SUITE(sock_recv_ring_status, NULL, NULL, NULL, NULL, NULL);

ZTEST(sock_recv_ring_status, test_neither_flag_is_busy)
{
	zassert_equal(sock_recv_ring_drained_status(false, false),
	              -2,
	              "drained, peer still connected -> BUSY, more may arrive");
}

ZTEST(sock_recv_ring_status, test_peer_closed_is_eof)
{
	zassert_equal(sock_recv_ring_drained_status(true, false),
	              0,
	              "drained + orderly close -> EOF, RESP_OK with 0 bytes");
}

ZTEST(sock_recv_ring_status, test_peer_error_is_terminal_failure)
{
	zassert_equal(sock_recv_ring_drained_status(false, true),
	              -3,
	              "drained + a real lwIP failure -> terminal error, never BUSY");
}

ZTEST(sock_recv_ring_status, test_peer_closed_wins_if_somehow_both_are_set)
{
	/* Cannot happen in production (cc3501e_hw_sock_pump()'s top-of-function
	 * guard stops it from ever setting both -- see this header's own
	 * comment), but EOF must still take precedence defensively: a stream
	 * that genuinely ended must never be reported as an error. */
	zassert_equal(sock_recv_ring_drained_status(true, true),
	              0,
	              "EOF takes precedence over an error flag, defensively");
}
