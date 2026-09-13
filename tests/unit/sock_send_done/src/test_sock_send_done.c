/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * Unit tests for protocol_sockets.c's handle_sock_send() STALE-SEND GUARD
 * (worker_discard_stale_terminal(), worker.h) on the WORKER_DONE path --
 * i.e. a real SOCK_SEND success, not the NOTIMPL/WORKER_ERR the plain stub
 * HAL always produces.
 *
 * tests/unit/transport_spi/src/test_transport_spi.c's SOCK_SEND cases
 * (test_sock_send_stale_seq_is_discarded_and_new_send_submits,
 * test_sock_send_same_seq_collects) link hal/cc3501e_hw_stub.c unmodified,
 * where cc3501e_hw_sock_send() always returns CC3501E_HW_ERR_NOTIMPL -- so
 * WORKER_DONE, and therefore the #88 reply cache's store arm (`if (st ==
 * ALP_CC3501E_RESP_OK)` in handle_sock_send()), is structurally unreachable
 * there. This TU reaches it by linking the SAME FW_SOURCES + stub HAL but
 * with the linker's `--wrap=cc3501e_hw_sock_send`: every call the firmware
 * makes to cc3501e_hw_sock_send() is redirected to __wrap_cc3501e_hw_sock_send()
 * below, which reports success and echoes back the queued count -- turning
 * a real WORKER_DONE (and therefore a real, cacheable RESP_OK) into an
 * ordinary code path this suite can drive over the wire.
 *
 * Covers the three DONE-path cases the plain stub suite cannot:
 *   - a DONE job with a different seq is discarded and the new send submits;
 *   - the SAME seq after a collect is served from the #88 cache (proven via
 *     g_worker_execs -- DIAG_GET_STATS -- NOT incrementing on the cache hit);
 *   - a NEW seq after a collect submits fresh (the ordinary case: the prior
 *     collect already reset the slot, so this is a sanity/regression check,
 *     not a case the guard itself has to do anything for).
 */

#include <stddef.h>
#include <string.h>
#include <zephyr/ztest.h>

#include "alp/protocol/cc3501e.h"
#include "alp/protocol/crc16.h" /* alp_crc16_ccitt_false[_update] -- the canonical algorithm */
#include "cc3501e_hw.h"         /* CC3501E_HW_OK */
#include "protocol.h"           /* CC3501E_REPLY_PAD / CC3501E_FRAME_MAX_BYTES */
#include "transport.h"
#include "worker.h" /* worker_init -- the worker `job` is a static; reset it per test */

/* Redirects every cc3501e_hw_sock_send() call the firmware makes (worker.c's
 * ALP_CC3501E_CMD_SOCK_SEND case) to here (`-Wl,--wrap=cc3501e_hw_sock_send`,
 * tests/unit/CMakeLists.txt) -- a real socket stack's ordinary success:
 * every byte offered is reported queued.  The original stub definition
 * (hal/cc3501e_hw_stub.c) is still linked, renamed to
 * __real_cc3501e_hw_sock_send by the same wrap, but nothing here calls it --
 * this suite exists specifically to NOT get NOTIMPL. */
int __wrap_cc3501e_hw_sock_send(uint16_t       handle,
                                uint8_t        flags,
                                const uint8_t *data,
                                uint16_t       data_len,
                                uint16_t      *sent_out)
{
	(void)handle;
	(void)flags;
	(void)data;
	if (sent_out != NULL) *sent_out = data_len;
	return CC3501E_HW_OK;
}

/* ---- Wire harness -- deliberately duplicated from test_transport_spi.c ----
 * rather than shared, matching this suite's own existing precedent
 * (test_transport_spi_no_crc.c is likewise a near-duplicate, not a shared
 * header): each -DCC3501E_WIFI / --wrap test target here is a small,
 * independent, from-scratch executable rather than a refactor of the
 * production wire suite. */

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
	if (len == 0u) {
		transaction_raw(bytes, len);
		return;
	}

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

static uint32_t diag_stat_u32(const uint8_t *reply, size_t off)
{
	return (uint32_t)reply[off] | ((uint32_t)reply[off + 1u] << 8) |
	       ((uint32_t)reply[off + 2u] << 16) | ((uint32_t)reply[off + 3u] << 24);
}

/* g_worker_execs, via DIAG_GET_STATS (issue #102) -- see test_transport_spi.c's
 * diag_stat_u32 callers for the same pattern.  Offset 13 in the reply data:
 * status(1, not counted here) + frames_ok(4) + frames_err(4) = 9 bytes in,
 * then worker_execs(LE32) -- i.e. reply[5..8]=frames_ok, reply[9..12]=frames_err,
 * reply[13..16]=worker_execs. */
static uint32_t worker_execs(void)
{
	uint8_t       reply[32];
	const uint8_t s[] = { ALP_CC3501E_CMD_DIAG_GET_STATS, 0x00u, 0x00u, 0x00u };
	transaction(s, sizeof s);
	(void)drain(reply, sizeof reply);
	return diag_stat_u32(reply, 13u);
}

/* alp_cc3501e_sock_send_t = handle(2) flags(1) seq(1) data_len(2) reserved2(2)
 * = 8 B, then data inline -> payload_len 9 for a 1-byte send.  Builds a
 * SOCK_SEND request frame with the given seq and one data byte. */
static void build_send(uint8_t *out, uint8_t seq, uint8_t data_byte)
{
	out[0]  = ALP_CC3501E_CMD_SOCK_SEND;
	out[1]  = 0x00u; /* outer header flags -- SOCK_SEND is exempt from the generic latch */
	out[2]  = 9u;    /* payload_len (logical, pre-CRC-reframe) */
	out[3]  = 0x00u;
	out[4]  = 0u; /* handle LE16 */
	out[5]  = 0u;
	out[6]  = 0u; /* flags */
	out[7]  = seq;
	out[8]  = 1u; /* data_len LE16 = 1 */
	out[9]  = 0u;
	out[10] = 0u; /* reserved2 */
	out[11] = 0u;
	out[12] = data_byte;
}

ZTEST(cc3501e_sock_send_done, test_stale_seq_is_discarded_and_new_send_submits)
{
	uint8_t reply[32];
	uint8_t send_a[13];
	uint8_t send_b[13];
	build_send(send_a, 1u, 0xAAu);
	build_send(send_b, 2u, 0xBBu);

	/* Submit send_a and WALK AWAY -- no second transaction(send_a) to collect
	 * it.  __wrap_cc3501e_hw_sock_send() makes the stub's synchronous submit
	 * reach a REAL WORKER_DONE this time, but this handler's own ack is
	 * always BUSY on the IDLE->QUEUED edge regardless, so that terminal DONE
	 * is left sitting in the slot, never collected, never cached. */
	transaction(send_a, sizeof send_a);
	(void)drain(reply, sizeof reply);
	zassert_equal(reply[4], ALP_CC3501E_RESP_ERR_BUSY, "send_a submits -> BUSY");

	/* send_b: a genuinely different logical send.  Before this guard,
	 * worker_poll() would match the slot by OPCODE ALONE and collect send_a's
	 * abandoned DONE as send_b's own answer (RESP_OK + send_a's queued
	 * count), with send_b's payload never submitted.  With the guard, the
	 * seq mismatch (1 vs 2) discards send_a's DONE, so the fall-through
	 * meets WORKER_IDLE and submits send_b fresh -> BUSY, not OK. */
	transaction(send_b, sizeof send_b);
	(void)drain(reply, sizeof reply);
	zassert_equal(reply[4],
	              ALP_CC3501E_RESP_ERR_BUSY,
	              "send_a's stale DONE is discarded; send_b submits fresh instead of "
	              "being answered with send_a's stale count");

	/* send_b's OWN result -- 1 byte queued, its own data_len -- proves its
	 * payload genuinely reached the worker and was genuinely executed. */
	transaction(send_b, sizeof send_b);
	size_t n = drain(reply, sizeof reply);
	zassert_equal(n, reply_wire(2u), "collect reply = header + status + 2B queued-count");
	zassert_equal(reply[4], ALP_CC3501E_RESP_OK, "send_b collects its own DONE");
	zassert_equal((uint16_t)(reply[5] | ((uint16_t)reply[6] << 8)),
	              1u,
	              "send_b's own 1-byte payload was queued, not send_a's");
}

ZTEST(cc3501e_sock_send_done, test_same_seq_after_collect_served_from_cache)
{
	uint8_t reply[32];
	uint8_t send_c[13];
	build_send(send_c, 3u, 0xCCu);

	transaction(send_c, sizeof send_c);
	(void)drain(reply, sizeof reply);
	zassert_equal(reply[4], ALP_CC3501E_RESP_ERR_BUSY, "send_c submits -> BUSY");

	transaction(send_c, sizeof send_c); /* collect -- stores the #88 cache */
	(void)drain(reply, sizeof reply);
	zassert_equal(reply[4], ALP_CC3501E_RESP_OK, "send_c collects -> OK, and caches");

	const uint32_t execs_before = worker_execs();

	/* A THIRD, byte-identical frame is what poll_by_repeat() sends when it
	 * never saw the second transaction's reply -- an ordinary retry.  It
	 * must be served from the #88 cache: same answer, and -- proven via
	 * g_worker_execs -- the worker must NOT run again. */
	transaction(send_c, sizeof send_c);
	size_t n = drain(reply, sizeof reply);
	zassert_equal(n, reply_wire(2u), "cached reply = header + status + 2B queued-count");
	zassert_equal(reply[4], ALP_CC3501E_RESP_OK, "same-seq retry served from the #88 cache");

	const uint32_t execs_after = worker_execs();
	zassert_equal(
	    execs_after, execs_before, "the cache hit did not re-run the worker body (no re-transmit)");
}

ZTEST(cc3501e_sock_send_done, test_new_seq_after_collect_submits_fresh)
{
	uint8_t reply[32];
	uint8_t send_e[13];
	uint8_t send_f[13];
	build_send(send_e, 5u, 0xEEu);
	build_send(send_f, 6u, 0xFFu);

	transaction(send_e, sizeof send_e);
	(void)drain(reply, sizeof reply);
	transaction(send_e, sizeof send_e); /* collect send_e -- resets the slot to IDLE */
	(void)drain(reply, sizeof reply);
	zassert_equal(reply[4], ALP_CC3501E_RESP_OK, "send_e collects -> OK");

	/* send_f: a genuinely new send under a new seq, arriving AFTER a clean
	 * collect (not an abandoned one).  The slot is already IDLE -- the prior
	 * collect's own WORKER_DONE branch reset it -- so this is the ordinary
	 * submit path, not a case the guard has to intervene in.  Kept as an
	 * explicit regression check that the guard's presence does not perturb
	 * the everyday collect-then-new-send cycle. */
	transaction(send_f, sizeof send_f);
	(void)drain(reply, sizeof reply);
	zassert_equal(reply[4], ALP_CC3501E_RESP_ERR_BUSY, "send_f submits fresh -> BUSY");

	transaction(send_f, sizeof send_f);
	size_t n = drain(reply, sizeof reply);
	zassert_equal(n, reply_wire(2u), "collect reply = header + status + 2B queued-count");
	zassert_equal(reply[4], ALP_CC3501E_RESP_OK, "send_f collects its own DONE");
}

static void reset_worker(void *fixture)
{
	(void)fixture;
	worker_init();
}

ZTEST_SUITE(cc3501e_sock_send_done, NULL, NULL, reset_worker, NULL, NULL);
