/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * Unit tests for src/sock_recv_commit.c -- the pure tail/uncommitted
 * arithmetic behind the SOCK_RECV lazy-commit fix (silent data loss on a
 * CRC-rejected reply, host review).
 *
 * The real ring (hal/ti/cc3501e_hw_ti_sock.c's rx_ring, 64 KB, TCM-placed)
 * needs the TI SimpleLink SDK and is built ONLY for CC3501E_HAL_BACKEND=ti
 * -- it is never linked into a host test binary (see that file's own top
 * comment).  sock_recv_commit.h's whole reason for existing is to pull the
 * DECISION a CRC-rejected-reply bug actually lives in -- commit or replay,
 * how many bytes -- out into something silicon-free that CAN be exercised
 * here, even though the byte-copying and lwIP plumbing around it cannot be.
 * (tests/unit/sock_recv_replay/ covers the OTHER host-testable half of this
 * fix -- protocol_sockets.c's `replay` DECISION itself -- over the wire, via
 * `--wrap=cc3501e_hw_sock_recv_ring`.) */

#include <zephyr/ztest.h>

#include "sock_recv_commit.h"

ZTEST_SUITE(cc3501e_sock_recv_commit, NULL, NULL, NULL, NULL, NULL);

/* A same-seq re-issue (replay=true) re-serves from the SAME tail the
 * original call did, and can return the SAME byte count if nothing new
 * arrived -- proving a CRC-rejected reply's retry gets back exactly what
 * the lost reply would have. */
ZTEST(cc3501e_sock_recv_commit, test_replay_returns_same_bytes_when_nothing_new_arrived)
{
	uint32_t tail        = 0u;
	uint32_t uncommitted = 0u;

	const uint32_t n1 =
	    sock_recv_commit(&tail, &uncommitted, /*head=*/100u, /*replay=*/false, /*cap=*/50u);
	zassert_equal(n1, 50u, "first serve: min(used=100, cap=50)");
	zassert_equal(
	    tail, 0u, "first serve does not advance tail (nothing was uncommitted before it)");
	zassert_equal(uncommitted, 50u, "first serve's own bytes are now the uncommitted count");

	/* Same seq, same handle, IDENTICAL head (no new pump data) -- exactly a
	 * CRC-rejected reply's poll_by_repeat() retry. */
	const uint32_t n2 =
	    sock_recv_commit(&tail, &uncommitted, /*head=*/100u, /*replay=*/true, /*cap=*/50u);
	zassert_equal(
	    n2, n1, "a replay with no new data returns the SAME byte count as the lost reply");
	zassert_equal(
	    tail, 0u, "a replay does not advance tail -- it re-serves from the same position");
}

/* A replay can return MORE bytes than the original call if the ring gained
 * data meanwhile (the producer's head keeps moving independently of the
 * held-back tail) -- correct stream-prefix semantics -- but never FEWER,
 * since head only grows and tail is unchanged from the call being
 * replayed. */
ZTEST(cc3501e_sock_recv_commit, test_replay_returns_at_least_as_many_bytes_never_fewer)
{
	uint32_t tail        = 0u;
	uint32_t uncommitted = 0u;

	const uint32_t n1 =
	    sock_recv_commit(&tail, &uncommitted, /*head=*/40u, /*replay=*/false, /*cap=*/200u);
	zassert_equal(n1, 40u, "first serve: min(used=40, cap=200)");

	/* More data arrived in the ring before the retry landed. */
	const uint32_t n2 =
	    sock_recv_commit(&tail, &uncommitted, /*head=*/90u, /*replay=*/true, /*cap=*/200u);
	zassert_true(n2 >= n1, "a replay never returns fewer bytes than the call it replays");
	zassert_equal(
	    n2, 90u, "a replay serves everything now available from the SAME tail, not just n1");
	zassert_equal(
	    tail, 0u, "still not advanced -- this poll is still a replay of the original call");
}

/* A NEW seq (replay=false) commits the previous call's serve -- tail
 * advances past it -- and then serves the NEXT unconsumed chunk, proving
 * the ordinary (non-retry) case still behaves like the pre-fix eager
 * commit, just one call later. */
ZTEST(cc3501e_sock_recv_commit, test_new_seq_commits_previous_serve_and_advances)
{
	uint32_t tail        = 0u;
	uint32_t uncommitted = 0u;

	const uint32_t n1 =
	    sock_recv_commit(&tail, &uncommitted, /*head=*/100u, /*replay=*/false, /*cap=*/50u);
	zassert_equal(n1, 50u, "first serve: min(used=100, cap=50)");
	zassert_equal(tail, 0u, "tail not yet advanced -- n1 is only uncommitted so far");

	/* A genuinely different logical recv (new seq -> replay=false): the
	 * PREVIOUS call's 50 bytes are now confirmed collected (this proves the
	 * host moved on), so they retire from the ring here. */
	const uint32_t n2 =
	    sock_recv_commit(&tail, &uncommitted, /*head=*/100u, /*replay=*/false, /*cap=*/50u);
	zassert_equal(tail, 50u, "committing the previous serve advances tail by its byte count");
	zassert_equal(
	    n2, 50u, "second serve: min(used=100-50=50, cap=50) -- the NEXT chunk, not a repeat");
}

/* Exercises the REAL reset path: cc3501e_hw_sock_prefetch()
 * (hal/ti/cc3501e_hw_ti_sock.c, TI-only, unreachable from this host suite --
 * see this file's top comment) calls this exact sock_recv_commit_reset(),
 * not a hand-rolled `uncommitted = 0u;` -- so calling it here proves the
 * SAME code production runs, not a stand-in for it.  rx_ring.head/tail
 * cannot follow this same seam (see sock_recv_commit.h's doc comment on
 * sock_recv_commit_reset() for why), so this test still assigns tail
 * directly for the "new handle" head/tail reset half of arming -- only the
 * uncommitted half is the real call. */
ZTEST(cc3501e_sock_recv_commit, test_prefetch_reset_clears_uncommitted)
{
	uint32_t tail        = 10u;
	uint32_t uncommitted = 5u; /* stale, from a previous handle's session */

	/* cc3501e_hw_sock_prefetch(new_handle, true) resets head = tail = 0
	 * directly, and uncommitted via sock_recv_commit_reset(). */
	tail = 0u;
	sock_recv_commit_reset(&uncommitted);

	/* The new handle's first-ever serve: if the stale uncommitted=5 had
	 * survived, a non-replay call would wrongly fold it into tail
	 * (tail += 5), skipping 5 bytes of the NEW handle's stream before ever
	 * serving a single byte of it. */
	const uint32_t n =
	    sock_recv_commit(&tail, &uncommitted, /*head=*/30u, /*replay=*/false, /*cap=*/50u);
	zassert_equal(
	    tail, 0u, "a properly reset uncommitted leaves tail at 0 for the new handle's first serve");
	zassert_equal(
	    n, 30u, "min(used=30, cap=50), not min(used=25, cap=50) -- no stale bytes skipped");
}
