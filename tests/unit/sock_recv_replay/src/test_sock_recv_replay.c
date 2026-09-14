/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * Unit tests for protocol_sockets.c's handle_sock_recv() REPLAY DECISION --
 * the `replay = (seq != 0 && seq == last_recv_seq && handle == last_recv_handle)`
 * computation feeding the SOCK_RECV lazy-commit fix (silent data loss on a
 * CRC-rejected reply, host review).
 *
 * tests/unit/transport_spi/src/test_transport_spi.c's SOCK_RECV cases link
 * hal/cc3501e_hw_stub.c unmodified, where cc3501e_hw_sock_recv_ring() always
 * returns -1 (no prefetch ring at all on the stub) -- so the fast path's
 * `replay` computation and its cc3501e_hw_sock_recv_ring() call are never
 * even reached there; every SOCK_RECV instead falls straight through to the
 * worker-routed path. This TU reaches the fast path by linking the SAME
 * FW_SOURCES + stub HAL but with the linker's `--wrap=cc3501e_hw_sock_recv_ring`
 * (same technique as sock_send_done's `--wrap=cc3501e_hw_sock_send`): every
 * call the firmware makes to cc3501e_hw_sock_recv_ring() is redirected to
 * __wrap_cc3501e_hw_sock_recv_ring() below, which records the `replay`
 * (and `handle`) argument it was called with and reports "0 bytes, ring
 * engaged" -- turning the otherwise-unreachable-on-host replay decision into
 * something this suite can drive, and inspect, over the wire.
 */

#include <stddef.h>
#include <string.h>
#include <zephyr/ztest.h>

#include "alp/protocol/cc3501e.h"
#include "alp/protocol/crc16.h" /* alp_crc16_ccitt_false[_update] -- the canonical algorithm */
#include "protocol.h"           /* CC3501E_REPLY_PAD / CC3501E_FRAME_MAX_BYTES */
#include "transport.h"

/* Redirects every cc3501e_hw_sock_recv_ring() call the firmware makes
 * (protocol_sockets.c's handle_sock_recv() fast path) to here
 * (`-Wl,--wrap=cc3501e_hw_sock_recv_ring`, tests/unit/CMakeLists.txt).
 * Always reports "this handle IS the prefetched one, 0 bytes available" --
 * rc = 0, *out_len = 0 -- which is enough to keep the fast path from
 * falling through to the worker-routed path (rc == -1 would) while keeping
 * the reply trivial to build (ALP_CC3501E_RESP_OK, empty payload).  The
 * actual byte-serving arithmetic is sock_recv_commit's own job and is
 * covered by tests/unit/sock_recv_commit/ instead; this suite exists
 * purely to observe what `replay` the fast path computed and passed in. */
static bool     g_last_replay;
static uint16_t g_last_handle;
static uint32_t g_wrap_calls;

int __wrap_cc3501e_hw_sock_recv_ring(uint16_t  handle,
                                     uint8_t  *buf,
                                     uint16_t  cap,
                                     bool      replay,
                                     uint16_t *out_len)
{
	(void)buf;
	(void)cap;
	g_last_replay = replay;
	g_last_handle = handle;
	g_wrap_calls++;
	if (out_len != NULL) *out_len = 0u;
	return 0;
}

/* ---- Wire harness -- deliberately duplicated from test_transport_spi.c ----
 * rather than shared, matching this suite's own existing precedent (see
 * test_sock_send_done.c's identical note). */

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

/* alp_cc3501e_sock_recv_t = handle(2) max_len(2) = 4 B.  Builds a SOCK_RECV
 * request frame with the given header seq (flags bits 3..7,
 * ALP_CC3501E_FLAG_REQ_SEQ_SHIFT) and handle. */
static void build_recv(uint8_t *out, uint8_t seq, uint16_t handle)
{
	out[0] = ALP_CC3501E_CMD_SOCK_RECV;
	out[1] = (uint8_t)(seq << ALP_CC3501E_FLAG_REQ_SEQ_SHIFT);
	out[2] = 4u; /* payload_len (logical, pre-CRC-reframe) */
	out[3] = 0x00u;
	out[4] = (uint8_t)(handle & 0xFFu);
	out[5] = (uint8_t)((handle >> 8) & 0xFFu);
	out[6] = 0u; /* max_len LE16 = 0 -- irrelevant, the wrap ignores cap */
	out[7] = 0u;
}

/* Issues one SOCK_RECV and returns the `replay` the fast path passed to
 * cc3501e_hw_sock_recv_ring() for it -- i.e. drives handle_sock_recv() over
 * the wire and reads back what __wrap_cc3501e_hw_sock_recv_ring() saw. */
static bool recv_replay(uint8_t seq, uint16_t handle)
{
	uint8_t req[8];
	uint8_t reply[64];

	build_recv(req, seq, handle);
	const uint32_t calls_before = g_wrap_calls;
	transaction(req, sizeof req);
	size_t n = drain(reply, sizeof reply);

	/* Reply DATA = alp_cc3501e_sock_recv_resp_t (24 B: from(20) + data_len(2)
	 * -- plus reserved padding, per the struct) + 0 received bytes, since
	 * the wrap always reports *out_len = 0. */
	zassert_equal(g_wrap_calls, calls_before + 1u, "the fast path engaged the wrapped ring");
	zassert_equal(n,
	              reply_wire(sizeof(alp_cc3501e_sock_recv_resp_t)),
	              "OK reply = header + status + the recv-resp header (0 bytes of data)");
	zassert_equal(reply[4], ALP_CC3501E_RESP_OK, "the wrap's rc=0 answers OK");

	return g_last_replay;
}

ZTEST_SUITE(cc3501e_sock_recv_replay, NULL, NULL, NULL, NULL, NULL);

/* A same-seq, same-handle re-issue is exactly poll_by_repeat()'s retry of a
 * CRC-rejected reply -- must read as a replay. */
ZTEST(cc3501e_sock_recv_replay, test_same_seq_same_handle_is_replay)
{
	zassert_false(recv_replay(1u, 100u), "priming call: no prior state to match yet");
	zassert_true(recv_replay(1u, 100u), "same seq (1) + same handle (100) -> replay");
}

/* A genuinely new logical recv -- next seq, same handle -- must NOT replay;
 * it is the ordinary "serve the next chunk" case. */
ZTEST(cc3501e_sock_recv_replay, test_different_seq_is_not_replay)
{
	zassert_false(recv_replay(2u, 200u), "priming call");
	zassert_false(recv_replay(3u, 200u), "different seq (3 vs priming's 2) -> not a replay");
}

/* seq 0 (ALP_CC3501E_REQ_SEQ_NONE) never claims a replay -- reserved, same
 * as the generic retry latch's own reservation (protocol.c) -- regardless
 * of what the handle's last recorded seq happens to be. */
ZTEST(cc3501e_sock_recv_replay, test_seq_zero_is_never_replay)
{
	zassert_false(recv_replay(4u, 300u), "priming call, seq 4");
	zassert_false(recv_replay(0u, 300u), "seq 0 on the SAME handle -> still not a replay");
}

/* Same seq, but a DIFFERENT handle -- a different socket's recv racing in
 * cannot be mistaken for the first handle's own retry. */
ZTEST(cc3501e_sock_recv_replay, test_different_handle_is_not_replay)
{
	zassert_false(recv_replay(5u, 400u), "priming call, handle 400");
	zassert_false(recv_replay(5u, 401u),
	              "same seq (5), different handle (401 vs 400) -> not a replay");
}
