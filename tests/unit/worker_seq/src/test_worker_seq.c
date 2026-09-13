/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * Unit tests for worker.c's worker_discard_stale_terminal() and
 * worker_reclaim_matching_terminal() (protocol_sockets.c's SOCK_SEND cache /
 * stale-send guard): the first must discard a same-opcode DONE/ERR job
 * tagged with a DIFFERENT request byte, the second must reclaim one tagged
 * with a MATCHING byte, and both must leave a QUEUED/RUNNING job COMPLETELY
 * ALONE either way -- there is no terminal result yet for anything to
 * misclaim or reclaim.
 *
 * tests/unit/transport_spi/src/test_transport_spi.c and
 * tests/unit/sock_send_done/src/test_sock_send_done.c both link the
 * SILICON-FREE stub build (no CC3501E_WIFI), where worker_submit_payload()
 * runs the job SYNCHRONOUSLY to a terminal state before the submit call even
 * returns (worker.c) -- a job is never externally observable as QUEUED/
 * RUNNING there, so neither suite can drive this case.  This target links
 * the SAME sources but WITH CC3501E_WIFI defined -- the real-firmware shape,
 * where submit only queues and a separate worker_run_pending() drain runs
 * the HAL body (never called here) -- so it can leave a job genuinely
 * QUEUED and call worker_discard_stale_terminal() against it directly,
 * without ever touching the wire/dispatch layer at all.
 *
 * This is exactly the case a reviewer mutation on worker_discard_stale_terminal()
 * found untested: changing its terminal check from `(state == DONE || state
 * == ERR)` to `state != IDLE` left every existing test green, because none
 * of them could ever present it with a QUEUED job.  On real TI firmware that
 * mutant would let a same-opcode request with a different seq evict a
 * RUNNING send WHILE the drain's lwip_send() is still reading job.req in
 * place -- the next submit then overwrites job.req out from under it,
 * corrupting the TCP bytes actually in flight. */

#include <stddef.h>
#include <zephyr/ztest.h>

#include "alp/protocol/cc3501e.h"
#include "protocol_internal.h" /* handle_sock_send() -- called directly, no wire framing needed */
#include "worker.h"

ZTEST(cc3501e_worker_seq, test_queued_sock_send_with_different_seq_stays_busy)
{
	worker_init();

	/* alp_cc3501e_sock_send_t = handle(2) flags(1) seq(1) data_len(2)
	 * reserved2(2) = 8 B, then 1 B of data -> 9 B total. */
	uint8_t req[9]                                   = { 0 };
	req[offsetof(alp_cc3501e_sock_send_t, seq)]      = 1u; /* seq A */
	req[offsetof(alp_cc3501e_sock_send_t, data_len)] = 1u;
	req[sizeof(alp_cc3501e_sock_send_t)]             = 0xAAu;

	/* With CC3501E_WIFI defined, worker_submit_payload() only transitions
	 * IDLE -> QUEUED here and returns -- it does NOT run the HAL body (that
	 * is worker_run_pending()'s job, never called in this test), so the job
	 * stays genuinely in flight for the check below. */
	zassert_equal(worker_submit_payload(ALP_CC3501E_CMD_SOCK_SEND, req, (uint16_t)sizeof req),
	              1,
	              "submit accepts IDLE -> QUEUED");

	/* A DIFFERENT seq (B) must NOT discard a QUEUED job: there is no
	 * terminal result yet for it to misclaim.  Returns 0 -- nothing
	 * discarded -- on the real (unmutated) implementation. */
	zassert_equal(worker_discard_stale_terminal(
	                  ALP_CC3501E_CMD_SOCK_SEND, offsetof(alp_cc3501e_sock_send_t, seq), 2u),
	              0,
	              "a different seq must not discard a QUEUED job");

	/* The job is still exactly where the submit left it: QUEUED, under seq
	 * A, untouched by the check above. */
	size_t            out_len = 0;
	int8_t            err     = 0;
	enum worker_state st      = worker_poll(ALP_CC3501E_CMD_SOCK_SEND, NULL, 0u, &out_len, &err);
	zassert_equal(st, WORKER_QUEUED, "the QUEUED job is exactly where the submit left it");
}

/* worker_reclaim_matching_terminal()'s own correctness (the mirror image of
 * worker_discard_stale_terminal(), added for protocol_sockets.c's cache-hit
 * reclaim): direct, worker.c-level coverage, because NO wire-observable test
 * can distinguish "reclaimed immediately at the cache hit" from "left
 * sitting until some OTHER mechanism (a mismatched-seq discard, or an
 * unrelated opcode's orphan-discard) cleans it up" -- both converge on the
 * SAME final wire behaviour in every scenario this suite's sibling targets
 * can construct, since every one of those other mechanisms already resets a
 * stale terminal job as a side effect.  This test is what actually proves
 * the reclaim helper's own match/mismatch/QUEUED logic, independent of
 * whether any caller's wire-level effect happens to be redundant with it.
 *
 * Deliberately GET_MAC, NOT SOCK_SEND: worker_reclaim_matching_terminal() is
 * opcode-generic (it takes @p cmd), so the choice of opcode here is
 * arbitrary -- but worker.c's worker_execute() calls
 * protocol_sock_send_on_worker_complete() UNCONDITIONALLY whenever cmd ==
 * SOCK_SEND, filling protocol_sockets.c's static #88 cache as a side effect
 * of running worker_run_pending() below.  That cache is never reset between
 * tests in this file (there is no reset hook for it, unlike worker_init()
 * for the job slot), so a SOCK_SEND completion here would leak into
 * test_lock_in_stale_completion_does_not_leak_into_new_send below,
 * whichever runs second.  GET_MAC sidesteps that entirely: this test is
 * about worker.c's primitive, not protocol_sockets.c's cache, and using an
 * opcode the cache never looks at keeps it that way. */
ZTEST(cc3501e_worker_seq, test_reclaim_matching_terminal)
{
	worker_init();

	/* Byte layout is arbitrary -- the primitive under test only compares one
	 * byte at a caller-chosen offset, it has no opinion on what opcode's
	 * request struct that offset belongs to. */
	uint8_t req[9] = { 0 };
	req[3]         = 5u; /* the "seq"-shaped byte this test tracks */
	req[8]         = 0x55u;

	zassert_equal(worker_submit_payload(ALP_CC3501E_CMD_GET_MAC, req, (uint16_t)sizeof req),
	              1,
	              "submit accepts IDLE -> QUEUED");

	/* Still QUEUED (CC3501E_WIFI: no auto-execute) -- reclaim must be a
	 * no-op even though the tracked byte WOULD match once terminal. */
	zassert_equal(worker_reclaim_matching_terminal(ALP_CC3501E_CMD_GET_MAC, 3u, 5u),
	              0,
	              "a QUEUED job is not reclaimed even on a matching byte");

	/* Run the drain for real: the stub HAL's cc3501e_hw_get_mac() is
	 * CC3501E_HW_ERR_NOTIMPL, so this takes the job to WORKER_ERR -- a
	 * genuine terminal state, not a wire-level fake. */
	worker_run_pending();

	/* A MISMATCHED byte must not reclaim a terminal job -- same rule
	 * worker_discard_stale_terminal() follows, just the opposite verdict. */
	zassert_equal(worker_reclaim_matching_terminal(ALP_CC3501E_CMD_GET_MAC, 3u, 6u),
	              0,
	              "a mismatched byte does not reclaim");
	size_t out_len = 0;
	int8_t err     = 0;
	zassert_equal(worker_poll(ALP_CC3501E_CMD_GET_MAC, NULL, 0u, &out_len, &err),
	              WORKER_ERR,
	              "the job is still there, terminal, after the mismatched attempt");

	/* The MATCHING byte reclaims it. */
	zassert_equal(worker_reclaim_matching_terminal(ALP_CC3501E_CMD_GET_MAC, 3u, 5u),
	              1,
	              "a matching byte reclaims the terminal job");
	zassert_equal(worker_poll(ALP_CC3501E_CMD_GET_MAC, NULL, 0u, &out_len, &err),
	              WORKER_IDLE,
	              "reclaimed -> IDLE");
}

/* The lock-in test: S1 QUEUED, a DIFFERENT seq S2 arrives and must stay
 * BUSY (S1 is not terminal -- nothing to discard, nothing cached yet), THEN
 * S1 completes and refills the cache under ITS OWN seq, THEN S2 is polled
 * AGAIN and must discard S1's now-terminal, differently-seq'd job and
 * submit itself fresh (not be answered from S1's cache, and not collect
 * S1's job by opcode-only match), and finally S2 completes and is answered
 * with ITS OWN outcome, never S1's.  Exercises handle_sock_send() directly
 * (this target links the full protocol stack, protocol_internal.h included
 * above) with worker_run_pending() stepping the drain manually between
 * polls -- the one thing test_transport_spi.c / sock_send_done.c cannot do,
 * since their stub-build submits always run to a terminal state
 * SYNCHRONOUSLY and never leave a poll-able QUEUED window. */
ZTEST(cc3501e_worker_seq, test_lock_in_stale_completion_does_not_leak_into_new_send)
{
	uint8_t reply[32];
	size_t  reply_len = 0u;

	/* alp_cc3501e_sock_send_t = handle(2) flags(1) seq(1) data_len(2)
	 * reserved2(2) = 8 B, then 1 B of data -> 9 B total. */
	uint8_t req_s1[9]                                   = { 0 };
	req_s1[offsetof(alp_cc3501e_sock_send_t, seq)]      = 1u; /* S1 */
	req_s1[offsetof(alp_cc3501e_sock_send_t, data_len)] = 1u;
	req_s1[sizeof(alp_cc3501e_sock_send_t)]             = 0xAAu;
	uint8_t req_s2[9]                                   = { 0 };
	req_s2[offsetof(alp_cc3501e_sock_send_t, seq)]      = 2u; /* S2 */
	req_s2[offsetof(alp_cc3501e_sock_send_t, data_len)] = 1u;
	req_s2[sizeof(alp_cc3501e_sock_send_t)]             = 0xBBu;

	/* S1 submits and stays QUEUED (CC3501E_WIFI: no auto-execute). */
	zassert_equal(handle_sock_send(req_s1, sizeof req_s1, reply, sizeof reply, &reply_len),
	              ALP_CC3501E_RESP_ERR_BUSY,
	              "S1 submits -> BUSY (QUEUED)");

	/* S2, a DIFFERENT seq, arrives while S1 is still QUEUED: nothing is
	 * cached yet (S1 has not completed) and S1 is not terminal, so the
	 * stale-job guard must leave it alone -- S2 reports BUSY (S1's job
	 * genuinely in flight), not a submit of its own. */
	zassert_equal(handle_sock_send(req_s2, sizeof req_s2, reply, sizeof reply, &reply_len),
	              ALP_CC3501E_RESP_ERR_BUSY,
	              "S2 while S1 is QUEUED -> BUSY; S1 is not discarded or collected");

	/* Run the drain for real: S1 completes (WORKER_ERR, stub HAL NOTIMPL)
	 * and protocol_sock_send_on_worker_complete() caches it under seq 1. */
	worker_run_pending();

	/* Poll S2 again: the cache holds seq 1, not seq 2, so it is invalidated;
	 * S1's now-terminal job (seq 1) is discarded as stale (different seq);
	 * S2 submits fresh -- BUSY, not S1's stale NOT_READY. */
	zassert_equal(handle_sock_send(req_s2, sizeof req_s2, reply, sizeof reply, &reply_len),
	              ALP_CC3501E_RESP_ERR_BUSY,
	              "S2 submits fresh after S1's stale completion is discarded, not collected");

	/* Run the drain again: S2 completes for real. */
	worker_run_pending();

	/* S2's own poll collects ITS OWN outcome -- never S1's. */
	zassert_equal(handle_sock_send(req_s2, sizeof req_s2, reply, sizeof reply, &reply_len),
	              ALP_CC3501E_RESP_ERR_NOT_READY,
	              "S2 gets its own outcome, never S1's");
}

static void reset_worker(void *fixture)
{
	(void)fixture;
	worker_init();
}

ZTEST_SUITE(cc3501e_worker_seq, NULL, NULL, reset_worker, NULL, NULL);
