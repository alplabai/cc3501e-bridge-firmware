/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * Unit tests for worker.c's job.seq guard (the request-identity fix): a
 * same-opcode DONE/ERR job tagged with a DIFFERENT seq than the polling
 * request is stale and uncollectable (see worker_poll's doc comment), but a
 * job that is still QUEUED/RUNNING is genuinely in flight regardless of
 * seq -- there is no terminal result yet for anyone to misclaim.
 *
 * test_transport_spi.c already covers the DONE and ERR variants of the
 * discard (test_stale_seq_done_result_is_discarded_not_collected /
 * test_stale_seq_err_result_is_discarded_not_collected) and the ordinary
 * same-seq collect (test_same_seq_collects) through the wire.  It cannot
 * cover the QUEUED/RUNNING-stays-busy case: that suite links the
 * SILICON-FREE build (no CC3501E_WIFI), where worker_submit_payload() runs
 * the job SYNCHRONOUSLY to a terminal state before the submit call even
 * returns (worker.c), so a job is never externally observable as
 * QUEUED/RUNNING there. This suite links the SAME sources but WITH
 * CC3501E_WIFI defined -- the real-firmware shape, where submit only
 * queues and a separate worker_run_pending() drain runs the HAL body -- so
 * it can leave a job genuinely QUEUED and poll it directly, without ever
 * touching the wire/dispatch layer at all. */

#include <zephyr/ztest.h>

#include "alp/protocol/cc3501e.h"
#include "worker.h"

ZTEST(cc3501e_worker_seq, test_queued_job_with_different_seq_stays_busy)
{
	worker_init();

	/* Submit under seq 1.  With CC3501E_WIFI defined, worker_submit() only
	 * transitions IDLE -> QUEUED here and returns -- it does NOT run the HAL
	 * body (that is worker_run_pending()'s job, never called in this test),
	 * so the job stays genuinely in flight for both polls below. */
	zassert_equal(worker_submit(ALP_CC3501E_CMD_GET_MAC, 1u), 1, "submit accepts IDLE -> QUEUED");

	/* A poll carrying the SAME seq sees the job it is actually waiting for:
	 * still busy, as expected. */
	zassert_equal(worker_poll(ALP_CC3501E_CMD_GET_MAC, 1u, NULL, 0u, NULL, NULL),
	              WORKER_QUEUED,
	              "same-seq poll of a QUEUED job -> still busy");

	/* A poll carrying a DIFFERENT seq is NOT a stale-result collision -- there
	 * is no terminal result yet for it to wrongly collect -- so it reports
	 * the SAME busy state instead of being discarded.  The job.seq guard
	 * (worker.c) is scoped to WORKER_DONE/WORKER_ERR only; this is the QUEUED
	 * case that guard must NOT touch. */
	zassert_equal(worker_poll(ALP_CC3501E_CMD_GET_MAC, 2u, NULL, 0u, NULL, NULL),
	              WORKER_QUEUED,
	              "different-seq poll of a QUEUED job -> still busy, not discarded");

	/* The original seq-1 caller can still poll and find its own job exactly
	 * where it left it -- the different-seq poll above must not have reset
	 * or reassigned it. */
	zassert_equal(worker_poll(ALP_CC3501E_CMD_GET_MAC, 1u, NULL, 0u, NULL, NULL),
	              WORKER_QUEUED,
	              "the original seq-1 poll still finds its own job QUEUED");
}

static void reset_worker(void *fixture)
{
	(void)fixture;
	worker_init();
}

ZTEST_SUITE(cc3501e_worker_seq, NULL, NULL, reset_worker, NULL, NULL);
