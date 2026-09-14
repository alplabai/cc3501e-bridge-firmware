/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * Unit test for worker_poll()'s WORKER_DONE truncation guard (worker.c,
 * `if (job.result_len > out_cap)`) -- MINOR 7 of the 1118c99 review (kills
 * mutation M6, "worker_poll `result_len > out_cap` guard neutralised").
 *
 * BLE_GATT_READ (protocol_ble.c's handle_ble_gatt_read() ->
 * handle_worker_routed_payload_reply(), the SAME generic helper SOCK_RECV
 * uses) is deliberately chosen over SOCK_RECV for this: SOCK_RECV has its
 * OWN opcode-specific cache (protocol_sockets.c) whose own oversize guard
 * would intercept an injected oversize length BEFORE worker_poll() is ever
 * reached, making it impossible to prove THIS guard specifically fired.
 * BLE_GATT_READ has no such cache, so an oversized `--wrap`ped
 * cc3501e_hw_ble_gatt_read() result reaches worker_poll()'s own guard
 * uncontaminated.
 *
 * `--wrap=cc3501e_hw_ble_gatt_read` reports an out_len of
 * CC3501E_REPLY_DATA_MAX + 3 -- ABOVE the wire's real ceiling but still
 * within worker.c's OWN defensive job.result[] clamp
 * (ALP_CC3501E_MAX_PAYLOAD), so it reaches worker_poll()'s out_cap check
 * exactly the way a misbehaving/future HAL body could.  With the guard
 * intact, the collect must answer ALP_CC3501E_RESP_ERR_NO_MEM, never a
 * truncated OK. */

#include <stddef.h>
#include <string.h>
#include <zephyr/ztest.h>

#include "alp/protocol/cc3501e.h"
#include "alp/protocol/crc16.h" /* alp_crc16_ccitt_false[_update] -- the canonical algorithm */
#include "cc3501e_hw.h"         /* CC3501E_HW_OK */
#include "protocol.h" /* CC3501E_REPLY_PAD / CC3501E_FRAME_MAX_BYTES / CC3501E_REPLY_DATA_MAX */
#include "transport.h"
#include "worker.h" /* worker_init -- the worker `job` is a static; reset it per test */

/* The oversized length this test injects -- 3 B past the wire's real
 * ceiling, but still within worker.c's own job.result[] clamp
 * (ALP_CC3501E_MAX_PAYLOAD), so it is worker_poll()'s guard specifically
 * that must catch it. */
#define OVERSIZE_LEN ((uint16_t)(CC3501E_REPLY_DATA_MAX + 3))

int __wrap_cc3501e_hw_ble_gatt_read(uint16_t handle, uint8_t *out, uint16_t cap, uint16_t *out_len)
{
	(void)handle;
	(void)cap; /* deliberately ignored -- simulates a HAL body that reports more than asked */
	for (uint16_t i = 0u; i < OVERSIZE_LEN; i++) {
		out[i] = (uint8_t)(i & 0xFFu);
	}
	if (out_len != NULL) *out_len = OVERSIZE_LEN;
	return CC3501E_HW_OK;
}

/* ---- Wire harness -- deliberately duplicated from test_transport_spi.c ----
 * rather than shared, matching this suite's own existing precedent. */

static inline size_t reply_padded_payload(size_t payload)
{
	const size_t with_crc = payload + (size_t)ALP_CC3501E_CRC_BYTES;

	return ((with_crc + CC3501E_REPLY_PAD - 1u) / CC3501E_REPLY_PAD) * CC3501E_REPLY_PAD;
}

static inline size_t reply_wire(size_t data_len)
{
	return (size_t)ALP_CC3501E_HEADER_BYTES + reply_padded_payload(1u + data_len);
}

static void transaction_raw(const uint8_t *bytes, size_t len)
{
	spi_slave_cs_low();
	for (size_t i = 0; i < len; i++) {
		spi_slave_rx_byte(bytes[i]);
	}
	spi_slave_cs_high();
}

static void transaction(const uint8_t *bytes, size_t len)
{
	uint8_t        framed[CC3501E_FRAME_MAX_BYTES];
	const size_t   payload_len = len - (size_t)ALP_CC3501E_HEADER_BYTES;
	const uint16_t wire_len    = (uint16_t)(payload_len + (size_t)ALP_CC3501E_CRC_BYTES);

	framed[0] = bytes[0];
	framed[1] = bytes[1];
	framed[2] = (uint8_t)(wire_len & 0xFFu);
	framed[3] = (uint8_t)((wire_len >> 8) & 0xFFu);
	if (payload_len > 0u) {
		memcpy(&framed[ALP_CC3501E_HEADER_BYTES], &bytes[ALP_CC3501E_HEADER_BYTES], payload_len);
	}

	uint16_t crc = alp_crc16_ccitt_false(framed, ALP_CC3501E_HEADER_BYTES);
	if (payload_len > 0u) {
		crc = alp_crc16_ccitt_false_update(crc, &framed[ALP_CC3501E_HEADER_BYTES], payload_len);
	}
	framed[ALP_CC3501E_HEADER_BYTES + payload_len]      = (uint8_t)(crc & 0xFFu);
	framed[ALP_CC3501E_HEADER_BYTES + payload_len + 1u] = (uint8_t)((crc >> 8) & 0xFFu);

	transaction_raw(framed, (size_t)ALP_CC3501E_HEADER_BYTES + wire_len);
}

static size_t drain(uint8_t *out, size_t cap)
{
	size_t n = 0;
	while (spi_slave_tx_pending() && n < cap) {
		out[n++] = spi_slave_tx_next_byte();
	}
	return n;
}

/* alp_cc3501e req = handle(LE16) = 2 B. */
static void build_gatt_read(uint8_t *out, uint16_t handle)
{
	out[0] = ALP_CC3501E_CMD_BLE_GATT_READ;
	out[1] = 0x00u;
	out[2] = 2u; /* payload_len (logical, pre-CRC-reframe) */
	out[3] = 0x00u;
	out[4] = (uint8_t)(handle & 0xFFu);
	out[5] = (uint8_t)((handle >> 8) & 0xFFu);
}

static void reset_worker(void *fixture)
{
	(void)fixture;
	worker_init();
}

ZTEST_SUITE(cc3501e_worker_poll_guard, NULL, NULL, reset_worker, NULL, NULL);

ZTEST(cc3501e_worker_poll_guard, test_oversize_result_is_no_mem_not_truncated_ok)
{
	static uint8_t reply[CC3501E_FRAME_MAX_BYTES];
	uint8_t        req[6];
	build_gatt_read(req, 1u);

	/* Submit: on the stub's synchronous path the (wrapped) HAL body already
	 * ran and worker_execute() already published the oversized result --
	 * this handler's own ack is still BUSY on the IDLE->QUEUED edge
	 * regardless. */
	transaction(req, sizeof req);
	(void)drain(reply, sizeof reply);
	zassert_equal(reply[4], ALP_CC3501E_RESP_ERR_BUSY, "submit -> BUSY");

	/* Collect: with the guard intact, this MUST be RESP_ERR_NO_MEM, never a
	 * truncated RESP_OK carrying fewer bytes than were actually read. */
	transaction(req, sizeof req);
	size_t n = drain(reply, sizeof reply);
	zassert_equal(reply[4], ALP_CC3501E_RESP_ERR_NO_MEM, "oversize result reported as NO_MEM");
	zassert_equal(n, reply_wire(0u), "a bare-status NO_MEM reply, no truncated data payload");
}

/* The wire-level test above cannot actually distinguish worker_poll()'s own
 * guard from protocol_build_reply()'s independent "Defence-in-depth" clamp
 * (protocol.c, ~971-978): handle_worker_routed_payload_reply() threads its
 * OWN reply_cap parameter straight from protocol_build_reply()'s own
 * reply_data_cap computation (protocol.c ~934-935 pass
 * `&reply_frame[CC3501E_REPLY_DATA_OFF]` / `reply_cap - OFF - CRC_BYTES`
 * verbatim into the handler chain) -- so worker_poll()'s `out_cap` and
 * protocol_build_reply()'s `reply_data_cap` are the SAME number by
 * construction, not merely coincidentally equal.  Neutering worker_poll()'s
 * guard alone therefore can NEVER change the final wire status a black-box
 * test observes: protocol_build_reply()'s own clamp still downgrades the
 * (now falsely WORKER_DONE-reported) oversize result to NO_MEM one layer up,
 * byte-for-byte the same as the test above already asserts.  That is by
 * design -- defence IN DEPTH -- but it also means mutating worker_poll()'s
 * guard alone is invisible to any test that only inspects the wire reply.
 *
 * What the mutation DOES change, invisibly to the wire test, is that
 * worker_poll() itself, with its own guard gone, still executes
 * `memcpy(out, job.result, n)` with n == job.result_len == OVERSIZE_LEN
 * against a caller buffer only out_cap bytes long -- an out-of-bounds write
 * PAST the caller's buffer that happens before protocol_build_reply() ever
 * gets a chance to clamp anything.  That overrun is real and exactly what
 * the guard exists to prevent; it is just papered over one layer up by the
 * time the reply hits the wire.  This test calls worker_poll() directly
 * (bypassing the wire and protocol_build_reply() entirely) with a
 * deliberately undersized out_cap and a canary byte placed immediately after
 * the caller's buffer.  With M6 applied (`if (job.result_len > out_cap)`
 * neutered) the mutant is actually caught by `st` -- the memcpy runs and
 * worker_poll() returns WORKER_DONE instead of WORKER_ERR, so the FIRST
 * assertion below already fails and the test never reaches the canary
 * check (NIT, host review of c354208 -- an earlier version of this comment
 * called this "caught in the act" by the canary, which overstated what
 * actually fires first).  The canary is kept anyway: it is the only thing
 * in this test that would ALSO catch a future mutant narrow enough to
 * leave `st`/`err`/`out_len` looking correct while still overrunning `out`
 * by a small, bounded amount. */
ZTEST(cc3501e_worker_poll_guard, test_guard_prevents_memcpy_overrun_past_out_cap)
{
	uint8_t req[2] = { 1u, 0u }; /* handle = 1, LE16 -- BLE_GATT_READ's whole payload */

	zassert_equal(worker_submit_payload(ALP_CC3501E_CMD_BLE_GATT_READ, req, (uint16_t)sizeof req),
	              1,
	              "submit accepts IDLE -> QUEUED (stub: runs synchronously to DONE)");

	/* out_cap is exactly one byte short of job.result_len (OVERSIZE_LEN):
	 * with the guard gone, the memcpy overruns `out` by precisely 1 byte,
	 * into canary[0] -- small and deterministic, never a wild stack smash. */
	struct {
		uint8_t out[OVERSIZE_LEN - 1u];
		uint8_t canary[4];
	} buf;
	memset(buf.canary, 0xCCu, sizeof buf.canary);

	size_t                  out_len = 0u;
	int8_t                  err     = 0;
	const enum worker_state st =
	    worker_poll(ALP_CC3501E_CMD_BLE_GATT_READ, buf.out, sizeof buf.out, &out_len, &err);

	zassert_equal(st, WORKER_ERR, "the guard must reject, not truncate-and-succeed");
	zassert_equal(err, CC3501E_HW_ERR_NO_MEM, "rejected as NO_MEM");
	zassert_equal(out_len, 0u, "no partial length reported on rejection");
	zassert_equal(buf.canary[0],
	              0xCCu,
	              "the guard must reject BEFORE any memcpy touches the buffer -- a "
	              "corrupted canary means the overrun already happened");
}
