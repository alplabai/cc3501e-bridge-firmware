/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * Unit tests for protocol_sockets.c's handle_sock_recv() WORKER-FALLBACK
 * REPLAY CACHE (g_sock_recv_wk_cached et al.) -- the fix for the KNOWN
 * FOLLOW-UP that block used to document: a handle NOT owned by the prefetch
 * ring (UDP, or a STREAM socket accepted but never armed) falls back to
 * handle_worker_routed_payload_reply(), a submit/collect pair with no
 * lazy-commit ring to hold bytes back in, so a CRC-rejected reply's
 * poll_by_repeat() retry used to consume the NEXT bytes off the socket
 * instead of re-serving the lost ones.
 *
 * tests/unit/transport_spi/src/test_transport_spi.c's SOCK_RECV cases link
 * hal/cc3501e_hw_stub.c unmodified, where cc3501e_hw_sock_recv_ring() always
 * returns -1 (no ring at all on the stub) -- so every SOCK_RECV there already
 * falls straight to the worker-routed path, but cc3501e_hw_sock_recv() also
 * always returns CC3501E_HW_ERR_NOTIMPL, so a genuine RESP_OK -- and
 * therefore this cache's store arm -- never fires there.  This TU reaches it
 * by linking the SAME FW_SOURCES + stub HAL but with the linker's
 * `--wrap=cc3501e_hw_sock_recv` (same technique as sock_send_done's
 * `--wrap=cc3501e_hw_sock_send`): every call the firmware makes to
 * cc3501e_hw_sock_recv() is redirected to __wrap_cc3501e_hw_sock_recv()
 * below, which reports success with a fixed, recognisable payload -- turning
 * a real WORKER_DONE (and therefore a real, cacheable RESP_OK) into a code
 * path this suite can drive and inspect over the wire.
 *
 * Covers:
 *   - a collected reply is retained, then the SAME seq+handle is re-served
 *     byte-identically WITHOUT a new HW recv (proven via a call counter the
 *     wrap itself keeps, mirroring sock_send_done's g_worker_execs check);
 *   - a DIFFERENT seq (same handle) submits a new recv instead of replaying;
 *   - a DIFFERENT handle (same seq) submits a new recv instead of replaying;
 *   - seq 0 never replays, even against its own immediately-preceding call;
 *   - SOCK_CLOSE on the cached handle invalidates the cache, so the next
 *     recv on that same handle number submits fresh;
 *   - a MAX-SIZE reply (the ACTUAL bug shape: an accepted TCP socket read
 *     reporting the full worker.c data_cap, ~4071 B -- the exact run10 loss
 *     size, well above a withdrawn 256 B cap that shipped in an earlier,
 *     incorrect version of this fix and left this exact case uncached) is
 *     retained and re-served byte-for-byte identical, proving the cache is
 *     sized for the largest reply this path can ever produce, not a
 *     "typical small" case.
 *
 * Every assertion below compares g_wrap_calls against a BASELINE captured at
 * the START of the test (not an absolute count) -- reset_worker() (the
 * ZTEST_SUITE "before" hook) only re-arms the worker job slot between tests,
 * matching sock_send_done.c's own precedent of leaving the completion-time
 * cache itself untouched between tests; each test also uses handle numbers
 * no other test in this file reuses, so a leftover cache entry from a
 * previous test can never coincidentally match. */

#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <zephyr/ztest.h>

#include "alp/protocol/cc3501e.h"
#include "alp/protocol/crc16.h" /* alp_crc16_ccitt_false[_update] -- the canonical algorithm */
#include "cc3501e_hw.h"         /* CC3501E_HW_OK */
#include "protocol.h"           /* CC3501E_REPLY_PAD / CC3501E_FRAME_MAX_BYTES */
#include "transport.h"
#include "worker.h" /* worker_init -- the worker `job` is a static; reset it per test */

/* Redirects every cc3501e_hw_sock_recv() call the firmware makes (worker.c's
 * ALP_CC3501E_CMD_SOCK_RECV case) to here (`-Wl,--wrap=cc3501e_hw_sock_recv`,
 * tests/unit/CMakeLists.txt) -- a real socket stack's ordinary success.  The
 * original stub definition (hal/cc3501e_hw_stub.c) is still linked, renamed
 * to __real_cc3501e_hw_sock_recv by the same wrap, but nothing here calls it
 * -- this suite exists specifically to NOT get NOTIMPL.
 *
 * Two modes, selected by g_wrap_large_mode:
 *   - default (false): one fixed, recognisable byte (0x42) -- what every
 *     small-reply case below needs.
 *   - true: fills up to BIG_RECV_DATA_LEN bytes (below) of an incrementing
 *     0..255 pattern and reports that many received.  BIG_RECV_DATA_LEN is
 *     the wire's own true ceiling (protocol.h's CC3501E_REPLY_DATA_MAX) minus
 *     the recv-resp header -- the actual largest reply
 *     protocol_build_reply()'s reply_cap can ever carry for THIS opcode, not
 *     worker.c's own internal data_cap constant (which -- see
 *     protocol_sockets.c's cache block comment -- is 2 B more generous than
 *     that ceiling under CC3501E_WIRE_CRC=ON, a separate, pre-existing gap
 *     this suite does not exercise).  This is still the actual run10 bug
 *     shape: an ACCEPTED TCP socket read reporting a reply at the wire's real
 *     maximum, far above the withdrawn 256 B cap. */
static uint32_t g_wrap_calls;
static bool     g_wrap_large_mode;
static uint16_t g_wrap_large_n; /* bytes reported by the last large-mode call */

/* protocol.h's CC3501E_REPLY_DATA_MAX is the wire's own documented ceiling on
 * a handler's total reply DATA (status byte's payload); the recv-resp header
 * always precedes the received bytes, so this is the most data a worker-
 * fallback SOCK_RECV reply can actually carry on the wire. */
#define BIG_RECV_DATA_LEN (CC3501E_REPLY_DATA_MAX - (uint16_t)sizeof(alp_cc3501e_sock_recv_resp_t))

int __wrap_cc3501e_hw_sock_recv(uint16_t  handle,
                                uint16_t  max_len,
                                uint8_t  *buf,
                                uint16_t  cap,
                                uint16_t *recv_len_out,
                                uint8_t   from_addr[4],
                                uint16_t *from_port_out)
{
	(void)handle;
	(void)max_len;
	g_wrap_calls++;
	if (g_wrap_large_mode) {
		const uint16_t n = (cap < (uint16_t)BIG_RECV_DATA_LEN) ? cap : (uint16_t)BIG_RECV_DATA_LEN;
		for (uint16_t i = 0u; i < n; i++) {
			buf[i] = (uint8_t)(i & 0xFFu);
		}
		if (recv_len_out != NULL) *recv_len_out = n;
		g_wrap_large_n = n;
	} else {
		if (cap > 0u && buf != NULL) buf[0] = 0x42u;
		if (recv_len_out != NULL) *recv_len_out = (cap > 0u) ? 1u : 0u;
	}
	if (from_addr != NULL) memset(from_addr, 0, 4u);
	if (from_port_out != NULL) *from_port_out = 0u;
	return CC3501E_HW_OK;
}

/* ---- Wire harness -- deliberately duplicated from test_transport_spi.c ----
 * rather than shared, matching this suite's own existing precedent
 * (test_sock_send_done.c / test_sock_recv_replay.c are likewise
 * near-duplicates, not a shared header). */

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
 * ALP_CC3501E_FLAG_REQ_SEQ_SHIFT) and handle; max_len 0 = "no cap beyond the
 * reply buffer", same as the ring fast-path suite's build_recv(). */
static void build_recv(uint8_t *out, uint8_t seq, uint16_t handle)
{
	out[0] = ALP_CC3501E_CMD_SOCK_RECV;
	out[1] = (uint8_t)(seq << ALP_CC3501E_FLAG_REQ_SEQ_SHIFT);
	out[2] = 4u; /* payload_len (logical, pre-CRC-reframe) */
	out[3] = 0x00u;
	out[4] = (uint8_t)(handle & 0xFFu);
	out[5] = (uint8_t)((handle >> 8) & 0xFFu);
	out[6] = 0u; /* max_len LE16 = 0 */
	out[7] = 0u;
}

/* alp_cc3501e_sock_close_t = handle(2) reserved(2) = 4 B. */
static void build_close(uint8_t *out, uint16_t handle)
{
	out[0] = ALP_CC3501E_CMD_SOCK_CLOSE;
	out[1] = 0x00u;
	out[2] = 4u;
	out[3] = 0x00u;
	out[4] = (uint8_t)(handle & 0xFFu);
	out[5] = (uint8_t)((handle >> 8) & 0xFFu);
	out[6] = 0u;
	out[7] = 0u;
}

/* Reply DATA = alp_cc3501e_sock_recv_resp_t (24 B) + 1 data byte (0x42, the
 * wrap's fixed payload). */
#define RECV_REPLY_LEN (sizeof(alp_cc3501e_sock_recv_resp_t) + 1u)

ZTEST(cc3501e_sock_recv_worker_cache, test_same_seq_same_handle_replayed_without_new_hw_recv)
{
	uint8_t        reply[64];
	uint8_t        req[8];
	const uint32_t calls_before = g_wrap_calls;
	build_recv(req, 1u, 100u);

	/* Submits and, on the stub's synchronous path, reaches a REAL
	 * WORKER_DONE inside this SAME transaction -- the cache is filled at
	 * completion, but this handler's own ack is always BUSY on the
	 * IDLE->QUEUED edge regardless (handle_worker_routed_payload_reply's
	 * contract), same shape as SOCK_SEND. */
	transaction(req, sizeof req);
	(void)drain(reply, sizeof reply);
	zassert_equal(reply[4], ALP_CC3501E_RESP_ERR_BUSY, "first poll submits -> BUSY");
	zassert_equal(g_wrap_calls, calls_before + 1u, "the submit ran the HW body once");

	/* Same seq+handle: a cache hit, served byte-identically, no new HW recv. */
	transaction(req, sizeof req);
	size_t n = drain(reply, sizeof reply);
	zassert_equal(
	    n, reply_wire(RECV_REPLY_LEN), "cached reply = header + status + resp header + 1 B");
	zassert_equal(reply[4], ALP_CC3501E_RESP_OK, "same-seq+handle retry served from the cache");
	zassert_equal(reply[5u + (uint32_t)sizeof(alp_cc3501e_sock_recv_resp_t)],
	              0x42u,
	              "cached byte is the original one");
	zassert_equal(
	    g_wrap_calls, calls_before + 1u, "the cache hit did not call cc3501e_hw_sock_recv() again");

	/* A THIRD, byte-identical frame -- exactly what poll_by_repeat() sends
	 * when it never saw the second transaction's reply -- must ALSO be
	 * served from the cache. */
	transaction(req, sizeof req);
	n = drain(reply, sizeof reply);
	zassert_equal(n, reply_wire(RECV_REPLY_LEN), "third poll still served from the cache");
	zassert_equal(reply[4], ALP_CC3501E_RESP_OK, "third poll: still OK");
	zassert_equal(
	    g_wrap_calls, calls_before + 1u, "still no new HW recv on the third, identical poll");
}

ZTEST(cc3501e_sock_recv_worker_cache, test_different_seq_submits_new)
{
	uint8_t        reply[64];
	uint8_t        req_a[8];
	uint8_t        req_b[8];
	const uint32_t calls_before = g_wrap_calls;
	build_recv(req_a, 2u, 200u);
	build_recv(req_b, 3u, 200u); /* same handle, different seq */

	transaction(req_a, sizeof req_a);
	(void)drain(reply, sizeof reply);
	zassert_equal(g_wrap_calls, calls_before + 1u, "req_a's submit ran the HW body once");

	/* req_b invalidates req_a's cache entry (different seq) and, since the
	 * stale-job guard evicts req_a's uncollected DONE (same opcode, no
	 * job.req field can tell the two apart on a shared handle -- see
	 * worker_discard_stale_recv()'s doc comment, worker.h), submits fresh:
	 * a SECOND real HW recv, not req_a's stale reply. */
	transaction(req_b, sizeof req_b);
	(void)drain(reply, sizeof reply);
	zassert_equal(g_wrap_calls, calls_before + 2u, "different seq (3 vs 2) submits a NEW hw recv");

	/* req_b's own same-seq poll: its own cache hit. */
	transaction(req_b, sizeof req_b);
	size_t n = drain(reply, sizeof reply);
	zassert_equal(n, reply_wire(RECV_REPLY_LEN), "req_b's own cache hit");
	zassert_equal(reply[4], ALP_CC3501E_RESP_OK, "req_b collects its own DONE");
	zassert_equal(g_wrap_calls, calls_before + 2u, "req_b's own cache hit ran no further hw recv");
}

ZTEST(cc3501e_sock_recv_worker_cache, test_different_handle_submits_new)
{
	uint8_t        reply[64];
	uint8_t        req_a[8];
	uint8_t        req_b[8];
	const uint32_t calls_before = g_wrap_calls;
	build_recv(req_a, 4u, 300u);
	build_recv(req_b, 4u, 301u); /* same seq, different handle */

	transaction(req_a, sizeof req_a);
	(void)drain(reply, sizeof reply);
	zassert_equal(g_wrap_calls, calls_before + 1u, "req_a's submit ran the HW body once");

	/* Same seq (4), but a DIFFERENT handle -- must NOT be served handle
	 * 300's cached reply; it is a genuinely different socket's recv. */
	transaction(req_b, sizeof req_b);
	(void)drain(reply, sizeof reply);
	zassert_equal(g_wrap_calls,
	              calls_before + 2u,
	              "same seq, different handle (301 vs 300) submits a NEW hw recv");

	transaction(req_b, sizeof req_b);
	size_t n = drain(reply, sizeof reply);
	zassert_equal(n, reply_wire(RECV_REPLY_LEN), "req_b's own cache hit");
	zassert_equal(reply[4], ALP_CC3501E_RESP_OK, "req_b collects its own DONE");
	zassert_equal(g_wrap_calls, calls_before + 2u, "req_b's own cache hit ran no further hw recv");
}

ZTEST(cc3501e_sock_recv_worker_cache, test_seq_zero_never_replays)
{
	uint8_t        reply[64];
	uint8_t        req_a[8];
	uint8_t        req_z[8];
	const uint32_t calls_before = g_wrap_calls;
	build_recv(req_a, 5u, 400u);
	build_recv(req_z, 0u, 400u); /* same handle, seq 0 -- ALP_CC3501E_REQ_SEQ_NONE */

	transaction(req_a, sizeof req_a);
	(void)drain(reply, sizeof reply);
	zassert_equal(g_wrap_calls, calls_before + 1u, "req_a's submit ran the HW body once");

	/* seq 0 on the SAME handle must still submit fresh -- seq 0 never
	 * claims a replay, same reservation the ring fast path and the generic
	 * retry latch both use. */
	transaction(req_z, sizeof req_z);
	(void)drain(reply, sizeof reply);
	zassert_equal(g_wrap_calls, calls_before + 2u, "seq 0 submits a NEW hw recv, never a replay");

	/* seq 0's own immediate re-issue must ALSO not replay -- seq 0 never
	 * matches itself either. */
	transaction(req_z, sizeof req_z);
	(void)drain(reply, sizeof reply);
	zassert_equal(
	    g_wrap_calls, calls_before + 3u, "seq 0 reissued against itself still submits fresh");
}

ZTEST(cc3501e_sock_recv_worker_cache, test_sock_close_invalidates_the_cache)
{
	uint8_t        reply[64];
	uint8_t        req[8];
	uint8_t        close_req[8];
	const uint32_t calls_before = g_wrap_calls;
	build_recv(req, 6u, 500u);
	build_close(close_req, 500u);

	transaction(req, sizeof req);
	(void)drain(reply, sizeof reply);
	zassert_equal(g_wrap_calls, calls_before + 1u, "the recv's submit ran the HW body once");

	/* Sanity: before the close, the SAME seq+handle is still a cache hit. */
	transaction(req, sizeof req);
	(void)drain(reply, sizeof reply);
	zassert_equal(g_wrap_calls, calls_before + 1u, "pre-close: still served from the cache");

	/* SOCK_CLOSE on handle 500 -- reaches WORKER_IDLE and submits (BUSY);
	 * the point under test is that it invalidates the cache regardless of
	 * cc3501e_hw_sock_close()'s own outcome (stub: NOTIMPL). */
	transaction(close_req, sizeof close_req);
	(void)drain(reply, sizeof reply);

	/* The SAME seq+handle, reissued after the close, must NOT be served the
	 * dead socket's stale cached reply -- a reopened handle 500 (or a stale
	 * host retry racing the close) must get a genuine recv. */
	transaction(req, sizeof req);
	(void)drain(reply, sizeof reply);
	zassert_equal(g_wrap_calls,
	              calls_before + 2u,
	              "post-close: the same seq+handle submits fresh, not cached");
}

/* THE ACTUAL BUG SHAPE (host review of a5881d1): an ACCEPTED TCP socket read
 * -- STREAM, accepted but not yet armed for prefetch, exactly the fallback
 * this cache exists for -- reports up to worker.c's own data_cap
 * (ALP_CC3501E_MAX_PAYLOAD - WK_SOCK_RECV_HDR - 1, ~4071 B), the exact run10
 * loss size.  A withdrawn version of this fix capped the cache at 256 B and
 * left exactly this case uncached -- it "fixed" only small UDP-sized
 * replies, missing the bug's own reproduction shape entirely.  This proves
 * the cache now covers the FULL range worker.c can ever report, not a
 * partial one. */
ZTEST(cc3501e_sock_recv_worker_cache, test_max_size_reply_replayed_byte_identical)
{
	static uint8_t reply[CC3501E_FRAME_MAX_BYTES];
	uint8_t        req[8];
	const uint32_t calls_before = g_wrap_calls;
	build_recv(req, 8u, 700u);

	g_wrap_large_mode = true;

	transaction(req, sizeof req);
	(void)drain(reply, sizeof reply);
	zassert_equal(g_wrap_calls, calls_before + 1u, "the submit ran the HW body once");
	const uint16_t n = g_wrap_large_n;
	/* Sanity: this run must actually exceed the withdrawn 256 B cap, or it
	 * would not have caught that regression. */
	zassert_true(n > 256u, "the reply must exceed the withdrawn 256 B cap to prove the fix");

	/* Same seq+handle: a cache hit, served byte-identically at FULL size, no
	 * new HW recv. */
	transaction(req, sizeof req);
	size_t got = drain(reply, sizeof reply);
	zassert_equal(got,
	              reply_wire(sizeof(alp_cc3501e_sock_recv_resp_t) + (size_t)n),
	              "cached reply = header + status + resp header + the FULL data_cap bytes");
	zassert_equal(reply[4], ALP_CC3501E_RESP_OK, "same-seq+handle retry served from the cache");

	bool bytes_match = true;
	for (uint16_t i = 0u; i < n; i++) {
		const uint8_t got_byte =
		    reply[5u + (uint32_t)sizeof(alp_cc3501e_sock_recv_resp_t) + (uint32_t)i];
		if (got_byte != (uint8_t)(i & 0xFFu)) {
			bytes_match = false;
			break;
		}
	}
	zassert_true(bytes_match, "every cached data byte matches the original 0..255 pattern exactly");
	zassert_equal(
	    g_wrap_calls, calls_before + 1u, "the cache hit did not call cc3501e_hw_sock_recv() again");

	g_wrap_large_mode = false; /* restore the default for every test after this one */
}

static void reset_worker(void *fixture)
{
	(void)fixture;
	g_wrap_large_mode = false;
	worker_init();
}

ZTEST_SUITE(cc3501e_sock_recv_worker_cache, NULL, NULL, reset_worker, NULL, NULL);
