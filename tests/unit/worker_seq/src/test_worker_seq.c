/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * Unit test for worker.c's worker_discard_stale_terminal() (protocol_sockets.c's
 * SOCK_SEND stale-send guard): it must discard a same-opcode DONE/ERR job
 * tagged with a different request byte, but leave a QUEUED/RUNNING job
 * COMPLETELY ALONE regardless of that byte -- there is no terminal result
 * yet for anything to misclaim.
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

static void reset_worker(void *fixture)
{
	(void)fixture;
	worker_init();
}

ZTEST_SUITE(cc3501e_worker_seq, NULL, NULL, reset_worker, NULL, NULL);
