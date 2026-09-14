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
 *   - seq 0 EVENTUALLY COLLECTS its own completion (plain submit/collect,
 *     same as every other worker-routed opcode) but never replays a
 *     DIFFERENT completion;
 *   - SOCK_CLOSE on the cached handle invalidates the cache, so the next
 *     recv on that same handle number submits fresh;
 *   - a ring-served recv on a DIFFERENT, armed handle does NOT let a stale
 *     worker-fallback entry age past a wrapped-seq collision (BLOCKER 1);
 *   - an UNRELATED SOCK_OPEN between a recv's submit and its retry does NOT
 *     drop the retry's cache hit (BLOCKER 2);
 *   - a cross-handle interleave BEFORE a recv's own retry is a documented,
 *     PINNED residual (three genuine reads, no misattribution) -- MAJOR 5,
 *     not a bug this suite expects fixed;
 *   - a MAX-SIZE reply (the ACTUAL bug shape: an accepted TCP socket read
 *     reporting the full worker.c data_cap, ~4071 B -- the exact run10 loss
 *     size, well above a withdrawn 256 B cap that shipped in an earlier,
 *     incorrect version of this fix and left this exact case uncached) is
 *     retained and re-served byte-for-byte identical, proving the cache is
 *     sized for the largest reply this path can ever produce, not a
 *     "typical small" case;
 *   - the host bench app's OWN recv granularity (max_len 4071, which exceeds
 *     the wire's real ceiling) bounds worker.c's own cap to the wire's real
 *     ceiling and the reply carries exactly what was consumed, not a
 *     silently truncated subset -- the worker.c data_cap fix this suite
 *     exists to prove, independent of the cache above.
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
 *   - true: fills up to @p cap bytes (capped again at BIG_RECV_DATA_LEN, the
 *     wire's own true ceiling -- protocol.h's CC3501E_REPLY_DATA_MAX minus
 *     the recv-resp header) of an incrementing 0..255 pattern and reports
 *     that many received -- simulating a peer with plenty of data queued, so
 *     the byte count actually reported is whatever worker.c's own (now
 *     wire-ceiling-bounded, see its SOCK_RECV case) @p cap allows.  This is
 *     the actual run10 bug shape: an ACCEPTED TCP socket read reporting a
 *     reply at the wire's real maximum, far above the withdrawn 256 B cap. */
static uint32_t g_wrap_calls;
static bool     g_wrap_large_mode;
static uint16_t g_wrap_large_n;      /* bytes reported by the last large-mode call */
static uint16_t g_wrap_last_cap;     /* @p cap worker.c passed on the last call */
static uint16_t g_wrap_last_max_len; /* @p max_len worker.c passed on the last call */
/* cc3501e-bridge-firmware bug 1 (worker-path recv after EOF answers
 * RESP_ERR_RADIO): models what the REAL TI HAL now does after the
 * hal/ti/cc3501e_hw_ti_sock.c sticky-EOF fix -- every recv on an fd that has
 * already seen EOF reports CC3501E_HW_OK with 0 bytes, not just the first.
 * The fix itself lives in that TI-SDK-only file and cannot link on the host
 * (see the block comment above cc3501e_hw_sock_recv()'s sock_eof table for
 * the full argument, and this suite's own top comment for why --wrap is
 * this file's way of reaching a real RESP_OK on the host at all) -- this
 * mode instead proves the PLUMBING this fix depends on: that
 * protocol_sockets.c threads a HAL-reported OK/0-bytes through to the wire
 * as RESP_OK on EVERY worker-routed poll, not just the first, so the fixed
 * HAL's repeated OK/0 answers actually reach the host as EOF rather than
 * falling into some other latent short-circuit. */
static bool g_wrap_eof_mode;

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
	g_wrap_calls++;
	g_wrap_last_cap     = cap;
	g_wrap_last_max_len = max_len;
	if (g_wrap_eof_mode) {
		if (recv_len_out != NULL) *recv_len_out = 0u;
		if (from_addr != NULL) memset(from_addr, 0, 4u);
		if (from_port_out != NULL) *from_port_out = 0u;
		return CC3501E_HW_OK;
	}
	if (g_wrap_large_mode) {
		/* Simulates a peer with AT LEAST this many bytes queued -- i.e. a
		 * full read that consumes exactly what worker.c's own data_cap (the
		 * @p cap this call received) allows, the same shape a real
		 * lwip_recvfrom() has when the socket has plenty of data queued. */
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

/* Captures the @p cap this suite's ring wrap was last called with -- the
 * ONLY way a host test can observe protocol_sockets.c's own room
 * computation (item 8, host review of bfb5f08): the wrap itself never
 * consulted @p cap before this, so nothing else in this file's behaviour
 * depends on it changing meaning here. */
static uint16_t g_wrap_ring_last_cap;

/* Redirects every cc3501e_hw_sock_recv_ring() call to here
 * (`-Wl,--wrap=cc3501e_hw_sock_recv_ring`, tests/unit/CMakeLists.txt).
 * Four fixed handles, so every OTHER test in this file (none of which uses
 * any of them) still sees the SAME rc == -1 the plain stub's own
 * cc3501e_hw_sock_recv_ring() always returns, unaffected:
 *
 *   777 -- "ring-owned, OK", 1 byte -- BLOCKER 1 test below
 *          (test_ring_recv_does_not_age_out_the_worker_fallback_cache).
 *   778 -- rc == -3, "ring-owned, drained and errored" (bug 2 fix, host
 *          review of bfb5f08's own MAJOR 3: protocol_sockets.c's
 *          handle_sock_recv() rc == -3 branch had no test) --
 *          test_ring_error_answers_radio_not_busy_or_worker below.
 *   779 -- rc == -2, "ring-owned, armed but momentarily empty, peer still
 *          connected" -- item 8's max_len == 0 room fix
 *          (test_ring_max_len_zero_* below): the ONLY way to reach
 *          handle_sock_recv()'s rc == -2 branch from this suite, since 777
 *          and 778 never return it.
 *   everything else -- "not mine, fall through" (-1). */
int __wrap_cc3501e_hw_sock_recv_ring(uint16_t  handle,
                                     uint8_t  *buf,
                                     uint16_t  cap,
                                     bool      replay,
                                     uint16_t *out_len)
{
	(void)replay;
	g_wrap_ring_last_cap = cap;
	if (handle == 777u) {
		buf[0] = 0x77u;
		if (out_len != NULL) *out_len = 1u;
		return 1;
	}
	if (handle == 778u) {
		if (out_len != NULL) *out_len = 0u;
		return -3;
	}
	if (handle == 779u) {
		if (out_len != NULL) *out_len = 0u;
		return -2;
	}
	if (out_len != NULL) *out_len = 0u;
	return -1;
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
 * ALP_CC3501E_FLAG_REQ_SEQ_SHIFT) and handle, max_len LE16 = 0.
 *
 * max_len == 0 no longer means "no cap" on the wire (item 8, host review of
 * bfb5f08 -- see protocol_sockets.c's handle_sock_recv() room computation);
 * it is harmless here regardless because every WORKER-FALLBACK test in this
 * file drives __wrap_cc3501e_hw_sock_recv(), which never reads @p max_len at
 * all (only worker.c's own @p cap, which max_len == 0 leaves at the wire
 * ceiling) -- so those tests are unaffected by this room fix.  Only the
 * RING-owned handles (777/778/779, __wrap_cc3501e_hw_sock_recv_ring above)
 * are, and the tests that need a NONZERO max_len to prove that use
 * build_recv_ml() below instead. */
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

/* Same as build_recv(), with an explicit, possibly-nonzero max_len -- for
 * the item 8 room-computation tests below, which need to drive BOTH the
 * max_len == 0 and the max_len != 0 arms of handle_sock_recv()'s room
 * clamp against the SAME ring-owned handle. */
static void build_recv_ml(uint8_t *out, uint8_t seq, uint16_t handle, uint16_t max_len)
{
	build_recv(out, seq, handle);
	out[6] = (uint8_t)(max_len & 0xFFu);
	out[7] = (uint8_t)((max_len >> 8) & 0xFFu);
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

/* alp_cc3501e_sock_open_t = family | type | protocol | reserved = 4 B. */
static void build_open(uint8_t *out, uint8_t seq)
{
	out[0] = ALP_CC3501E_CMD_SOCK_OPEN;
	out[1] = (uint8_t)(seq << ALP_CC3501E_FLAG_REQ_SEQ_SHIFT);
	out[2] = 4u;
	out[3] = 0u;
	out[4] = (uint8_t)ALP_CC3501E_SOCK_FAMILY_IPV4;
	out[5] = 1u; /* type: arbitrary, non-zero */
	out[6] = 0u;
	out[7] = 0u;
}

/* transaction() + drain() + return the status byte -- the shape every test
 * below that only cares about BUSY/OK/NO_MEM (not the reply's own byte
 * content) repeats. */
static int poll_status(const uint8_t *req, uint8_t *reply, size_t cap)
{
	transaction(req, 8u);
	(void)drain(reply, cap);
	return reply[4];
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

/* MINOR residual (host review of 9c989dc): same seq, same handle, but a
 * DIFFERENT max_len -- the invalidation check and the cache fill both used
 * to key on (seq, handle) alone, so this used to be served req_a's cached
 * reply (sized/consumed for req_a's OWN max_len) as if it were a
 * byte-identical retry of it.  The SDK host never actually varies max_len
 * on a retry (poll_by_repeat() always resends an identical frame), so this
 * is a defensive fix, not a reachable-today one -- but firmware correctness
 * must not depend on what a caller happens to do. */
ZTEST(cc3501e_sock_recv_worker_cache, test_same_seq_same_handle_different_max_len_submits_new)
{
	uint8_t        reply[64];
	uint8_t        req_a[8];
	uint8_t        req_b[8];
	const uint32_t calls_before = g_wrap_calls;
	build_recv_ml(req_a, 17u, 310u, 64u);
	build_recv_ml(req_b, 17u, 310u, 0u); /* same seq + handle, DIFFERENT max_len */

	transaction(req_a, sizeof req_a);
	(void)drain(reply, sizeof reply);
	zassert_equal(g_wrap_calls, calls_before + 1u, "req_a's submit ran the HW body once");

	/* Same seq (17), same handle (310), but max_len differs (0 vs 64) --
	 * must NOT be served req_a's cached reply. */
	transaction(req_b, sizeof req_b);
	(void)drain(reply, sizeof reply);
	zassert_equal(g_wrap_calls,
	              calls_before + 2u,
	              "same seq+handle, different max_len (0 vs 64) submits a NEW hw recv");

	transaction(req_b, sizeof req_b);
	size_t n = drain(reply, sizeof reply);
	zassert_equal(n, reply_wire(RECV_REPLY_LEN), "req_b's own cache hit");
	zassert_equal(reply[4], ALP_CC3501E_RESP_OK, "req_b collects its own DONE");
	zassert_equal(g_wrap_calls, calls_before + 2u, "req_b's own cache hit ran no further hw recv");
}

/* REWRITTEN TWICE.  First (MAJOR 3, host review of 1118c99): a seq-0 host
 * (one that never assigns a retry-protection identity -- every bare
 * cc3501e_request() call site, or a pre-v8 host) gets the PLAIN
 * submit/collect protocol every OTHER worker-routed opcode already gives
 * it, not a discard-and-reread on every poll.  The OLD version of this test
 * asserted the OPPOSITE ("seq 0 reissued against itself still submits
 * fresh") -- that assertion was itself pinning the bug:
 * worker_discard_stale_recv() firing unconditionally on every seq-0 poll
 * (because seq 0 never hit the cache at all, by the OLD gate) discarded the
 * just-finished job and re-read on EVERY poll, so a seq-0 host could never
 * collect its own recv, only ever consume fresh (different) socket bytes
 * believing it was still waiting for its FIRST answer.
 *
 * Second (BLOCKER, host review of c354208): that fix on its own let seq 0
 * REPLAY a completed recv.  Widening the serve gate to unconditional
 * `g_sock_recv_wk_cached` made every later seq-0 poll on the same handle a
 * cache hit forever, with nothing to ever tell "collect-again" apart from "a
 * genuinely new recv" -- seq 0 carries no identity, so they are
 * byte-identical requests.  Fixed with SERVE ONCE, THEN FORGET
 * (handle_sock_recv()'s own comment on the serve site): a seq-0 hit clears
 * the cache the instant it is served, so poll 2 collects (still the SAME
 * single hw read from poll 1), and poll 3 -- whether the host means it as
 * "collect again" or as a genuinely new recv, indistinguishable here -- MUST
 * fall through and submit fresh, a SECOND hw read. */
ZTEST(cc3501e_sock_recv_worker_cache, test_seq_zero_eventually_collects)
{
	uint8_t        reply[64];
	uint8_t        req[8];
	const uint32_t calls_before = g_wrap_calls;
	build_recv(req, 0u, 990u); /* seq 0 -- ALP_CC3501E_REQ_SEQ_NONE */

	transaction(req, sizeof req);
	(void)drain(reply, sizeof reply);
	zassert_equal(reply[4], ALP_CC3501E_RESP_ERR_BUSY, "seq0 first poll submits -> BUSY");
	zassert_equal(g_wrap_calls, calls_before + 1u, "seq0 submit ran the HW body once");

	transaction(req, sizeof req);
	int s1 = drain(reply, sizeof reply) > 0 ? reply[4] : -1;
	zassert_equal(s1, ALP_CC3501E_RESP_OK, "seq0 second poll collects");
	zassert_equal(g_wrap_calls, calls_before + 1u, "collecting did not itself read the socket");

	/* Poll 3: the entry was forgotten the instant poll 2 served it, so this
	 * MUST submit fresh -- a genuinely new recv must never be answered
	 * poll 1's stale bytes again. */
	transaction(req, sizeof req);
	int s2 = drain(reply, sizeof reply) > 0 ? reply[4] : -1;
	zassert_equal(s2, ALP_CC3501E_RESP_ERR_BUSY, "seq0 third poll submits fresh -> BUSY");
	zassert_equal(g_wrap_calls, calls_before + 2u, "seq0 third poll ran a NEW hw read");
}

/* seq 0 must still never replay a DIFFERENT completion -- only its OWN
 * immediately-preceding one (see test_seq_zero_eventually_collects' own
 * comment for why THAT case is now safely a cache hit).  A seq-0 recv on a
 * DIFFERENT handle is a mismatch by the invalidation check regardless of
 * either side being 0, so it must submit fresh. */
ZTEST(cc3501e_sock_recv_worker_cache, test_seq_zero_does_not_replay_a_different_handle)
{
	uint8_t        reply[64];
	uint8_t        req_a[8];
	uint8_t        req_z[8];
	const uint32_t calls_before = g_wrap_calls;
	build_recv(req_a, 5u, 401u);
	build_recv(req_z, 0u, 402u); /* DIFFERENT handle, seq 0 */

	transaction(req_a, sizeof req_a);
	(void)drain(reply, sizeof reply);
	zassert_equal(g_wrap_calls, calls_before + 1u, "req_a's submit ran the HW body once");

	transaction(req_z, sizeof req_z);
	(void)drain(reply, sizeof reply);
	zassert_equal(
	    g_wrap_calls, calls_before + 2u, "seq 0 on a DIFFERENT handle submits fresh, not a replay");
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
 * (CC3501E_REPLY_DATA_MAX - WK_SOCK_RECV_HDR, ~4069 B under
 * CC3501E_WIRE_CRC=ON), the exact run10 loss size.  A withdrawn version of
 * this fix capped the cache at 256 B and left exactly this case uncached --
 * it "fixed" only small UDP-sized replies, missing the bug's own
 * reproduction shape entirely.  This proves the cache now covers the FULL
 * range worker.c can ever report, not a partial one. */
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

/* worker.c's SOCK_RECV data_cap fix, exercised at the HOST BENCH APP'S OWN
 * recv granularity (host review): the bench issues max_len 4071 per call --
 * ABOVE CC3501E_REPLY_DATA_MAX - sizeof(alp_cc3501e_sock_recv_resp_t) (4069 B
 * under CC3501E_WIRE_CRC=ON) -- so a worker-fallback socket hit the
 * pre-fix truncation on EVERY full read, not a rare boundary.  Proves
 * worker.c never asks cc3501e_hw_sock_recv() for more than the wire can
 * carry, and that the reply it builds carries EXACTLY what was consumed, not
 * a silently truncated subset. */
ZTEST(cc3501e_sock_recv_worker_cache, test_max_len_4071_bounds_hw_cap_and_reply_matches_consumed)
{
	static uint8_t reply[CC3501E_FRAME_MAX_BYTES];
	uint8_t        req[8];
	const uint32_t calls_before = g_wrap_calls;
	build_recv(req, 9u, 800u);
	req[6] = (uint8_t)(4071u & 0xFFu); /* max_len LE16 = 4071 -- the bench's own recv() size */
	req[7] = (uint8_t)((4071u >> 8) & 0xFFu);

	g_wrap_large_mode = true;

	/* First transaction is the IDLE->QUEUED submit: on the stub's synchronous
	 * path the HW body already ran (and worker.c's cap arithmetic already
	 * ran with it), but this handler's own ack is always BUSY on that edge
	 * regardless -- same contract every other test in this suite relies on.
	 * The bounds this test cares about are already latched in
	 * g_wrap_last_cap/g_wrap_last_max_len at this point. */
	transaction(req, sizeof req);
	(void)drain(reply, sizeof reply);
	zassert_equal(g_wrap_calls, calls_before + 1u, "the submit ran the HW body once");
	zassert_equal(
	    g_wrap_last_max_len, 4071u, "the request's own max_len reached the HAL body unchanged");
	zassert_true(
	    g_wrap_last_cap <= (uint16_t)BIG_RECV_DATA_LEN,
	    "hw recv's cap must never exceed CC3501E_REPLY_DATA_MAX minus the recv-resp header");
	zassert_true(
	    g_wrap_last_cap < 4071u,
	    "4071 B genuinely exceeds the wire ceiling here, so the cap must be the SMALLER one");

	/* Second, same-seq+handle transaction: the cache-hit collect -- proves
	 * the reply actually delivered carries EXACTLY what was consumed. */
	transaction(req, sizeof req);
	size_t         got      = drain(reply, sizeof reply);
	const uint16_t consumed = g_wrap_large_n; /* what the wrap actually reported back */
	zassert_equal(got,
	              reply_wire(sizeof(alp_cc3501e_sock_recv_resp_t) + (size_t)consumed),
	              "the reply carries EXACTLY the consumed bytes -- no silent truncation");
	zassert_equal(reply[4], ALP_CC3501E_RESP_OK, "the capped read fits the reply -- no NO_MEM");
	zassert_equal(
	    g_wrap_calls, calls_before + 1u, "the cache hit did not call cc3501e_hw_sock_recv() again");

	g_wrap_large_mode = false;
}

/* BLOCKER 1 (host review, 1118c99): the worker-fallback cache's invalidate-
 * on-mismatch check used to run AFTER the ring fast path, which returns
 * EARLY -- so a recv the RING serves never reached it at all.  Scenario:
 * H1 (worker path) recvs and is COLLECTED (host advances its per-recv seq
 * past the value it used); many SUBSEQUENT recvs on an ARMED, DIFFERENT
 * handle H2 -- served entirely by the ring, never touching the code that
 * used to sit after it -- eventually wrap the SHARED 5-bit header seq back
 * to the value H1's OWN stale cache entry still held; H1's NEXT recv,
 * carrying that wrapped value, then matched the stale entry and was served
 * H1's OLD bytes as OK with NO hw read at all.  This test does not need 30
 * intervening calls to reproduce it: ONE ring-served H2 recv is enough to
 * either invalidate H1's entry (fixed) or leave it untouched (bug) -- the
 * wrapped-seq collision is then simulated directly by reissuing H1's
 * ORIGINAL wire bytes verbatim. */
ZTEST(cc3501e_sock_recv_worker_cache, test_ring_recv_does_not_age_out_the_worker_fallback_cache)
{
	uint8_t reply[64];
	uint8_t h1[8];
	uint8_t h2[8];
	build_recv(h1, 1u, 994u); /* worker-path handle */
	build_recv(h2, 2u, 777u); /* ring-owned handle, per __wrap_cc3501e_hw_sock_recv_ring above */

	int sub = poll_status(h1, reply, sizeof reply);
	zassert_equal(sub, ALP_CC3501E_RESP_ERR_BUSY, "H1 submit");
	int got = poll_status(h1, reply, sizeof reply);
	zassert_equal(got, ALP_CC3501E_RESP_OK, "H1 collected -- the host advances its seq from here");

	int ring = poll_status(h2, reply, sizeof reply);
	zassert_equal(ring, ALP_CC3501E_RESP_OK, "the armed H2 recv is served by the ring fast path");

	/* H1's bytes reissued VERBATIM -- simulating the shared 5-bit counter
	 * having wrapped back to seq 1 after enough intervening ring-served H2
	 * calls.  With the invalidation check running on H2's OWN dispatch too,
	 * H1's stale entry is already gone by the time this lands, so it MUST
	 * submit a fresh recv, never replay the old one. */
	const uint32_t calls_before = g_wrap_calls;
	int            again        = poll_status(h1, reply, sizeof reply);
	zassert_equal(g_wrap_calls,
	              calls_before + 1u,
	              "a new H1 recv at a wrapped-back seq must read the socket, not replay");
	zassert_equal(again, ALP_CC3501E_RESP_ERR_BUSY, "the new H1 recv submits fresh");
}

/* BLOCKER 2 (host review, 1118c99): handle_sock_open() used to invalidate
 * the worker-fallback cache UNCONDITIONALLY on every SOCK_OPEN, including
 * one for a completely UNRELATED new socket -- dropping a same-seq retry's
 * only chance to collect a worker-routed recv that had already completed
 * (and, on the wire, been consumed off the peer socket) but not yet been
 * collected by the host.  SOCK_OPEN must never touch this cache: only
 * SOCK_CLOSE frees a host-visible handle number (verified against every
 * hal/ti/cc3501e_hw_ti_sock.c lwip_close() site -- see handle_sock_open()'s
 * own comment). */
ZTEST(cc3501e_sock_recv_worker_cache, test_sock_open_between_retry_still_served_from_cache)
{
	uint8_t reply[64];
	uint8_t req[8];
	uint8_t open_req[8];
	build_recv(req, 12u, 991u);
	build_open(open_req, 7u);

	int            sr           = poll_status(req, reply, sizeof reply);
	int            so           = poll_status(open_req, reply, sizeof reply);
	const uint32_t calls_before = g_wrap_calls;
	int            sr2          = poll_status(req, reply, sizeof reply);

	zassert_equal(sr, ALP_CC3501E_RESP_ERR_BUSY, "recv submit");
	zassert_equal(so, ALP_CC3501E_RESP_ERR_BUSY, "an UNRELATED SOCK_OPEN submits too");
	zassert_equal(sr2, ALP_CC3501E_RESP_OK, "the recv retry after SOCK_OPEN is still a cache hit");
	zassert_equal(g_wrap_calls, calls_before, "the retry after SOCK_OPEN must not re-read");
}

/* MAJOR 5 (host review, 1118c99): the CROSS-handle residual, stated
 * precisely and DEMONSTRATED here rather than fixed -- see
 * protocol_sockets.c's own RESIDUAL comment (g_sock_recv_wk_cached) and
 * sock_recv_commit.h's RESIDUAL 1/2 for the full argument.  A single cache
 * entry cannot hold two DIFFERENT handles' completions at once: if a second
 * handle's recv is dispatched BEFORE the first handle's own retry arrives
 * (two sockets driven from two different host threads/ctxs are not
 * serialised against each other -- ctx->sock_busy's own doc comment says so
 * outright), the first handle's entry is correctly invalidated (never
 * served to the WRONG handle) but is then gone -- its own eventual retry
 * must submit fresh, consuming NEW socket bytes rather than recovering the
 * original ones.  This is the SAFE, documented tradeoff (no corruption, no
 * wedge -- worker_discard_stale_recv()'s unconditional-by-opcode reclaim
 * keeps the single job slot from ever deadlocking on an abandoned job)
 * versus the WORSE alternative of refusing the second handle until the
 * first retries, which a host that never retries would wedge forever.
 * Building per-handle state to close this is deliberately NOT done; this
 * test exists to PIN the current, accepted behaviour (three genuine hw
 * reads, one per logical recv, no misattribution) so a future change that
 * accidentally reintroduces stale-data misattribution here is caught. */
ZTEST(cc3501e_sock_recv_worker_cache, test_cross_handle_interleave_is_a_documented_residual)
{
	uint8_t        reply[64];
	uint8_t        r1[8];
	uint8_t        r2[8];
	const uint32_t calls_before = g_wrap_calls;
	build_recv(r1, 13u, 992u);
	build_recv(r2, 13u, 993u); /* same candidate seq, DIFFERENT handle */

	int a = poll_status(r1, reply, sizeof reply);
	zassert_equal(a, ALP_CC3501E_RESP_ERR_BUSY, "H1 submit");
	zassert_equal(g_wrap_calls, calls_before + 1u, "H1's own submit read the socket once");

	/* H2 lands BEFORE H1's own retry: invalidates H1's entry (different
	 * handle) and, since H1's job was never collected, discards it and
	 * submits its own fresh recv -- a SECOND real read, not H1's stale
	 * reply. */
	int b = poll_status(r2, reply, sizeof reply);
	zassert_equal(b, ALP_CC3501E_RESP_ERR_BUSY, "H2 submit");
	zassert_equal(g_wrap_calls, calls_before + 2u, "H2's own submit read the socket once");

	/* H1's retry: H2's entry is now cached, a mismatch for H1's own
	 * (seq, handle) -- invalidated, and H2's uncollected job discarded in
	 * turn.  H1 submits fresh: a THIRD real read.  This is the documented
	 * residual, not a bug -- H1's ORIGINAL bytes are gone, but this fresh
	 * read is genuinely H1's own, not H2's misattributed to H1. */
	int c = poll_status(r1, reply, sizeof reply);
	zassert_equal(
	    c, ALP_CC3501E_RESP_ERR_BUSY, "H1 retry submits fresh (its original bytes are lost)");
	zassert_equal(g_wrap_calls, calls_before + 3u, "H1's retry read the socket a THIRD time");
}

/* BLOCKER probe (host review of c354208): a genuinely NEW seq-0 recv on the
 * SAME handle a just-collected seq-0 recv used must never be answered the
 * OLD recv's bytes again -- seq 0 carries no identity, so nothing but the
 * cache's own "serve once, then forget" rule (handle_sock_recv()'s serve
 * site) can tell the two apart. */
ZTEST(cc3501e_sock_recv_worker_cache, test_probe_seq0_new_recv_after_collect_is_not_replayed)
{
	uint8_t        reply[64];
	uint8_t        req[8];
	const uint32_t before = g_wrap_calls;
	build_recv(req, 0u, 995u);

	int s1 = poll_status(req, reply, sizeof reply);
	zassert_equal(s1, ALP_CC3501E_RESP_ERR_BUSY, "seq0 recv #1 submit");
	int s2 = poll_status(req, reply, sizeof reply);
	zassert_equal(s2, ALP_CC3501E_RESP_OK, "seq0 recv #1 collected");
	zassert_equal(g_wrap_calls, before + 1u, "recv #1 = one socket read");

	/* Host now issues a genuinely NEW logical recv on the same handle; seq 0
	 * carries no identity, so the frame is byte-identical.  It must read the
	 * socket, never be answered recv #1's bytes again as OK. */
	int s3 = poll_status(req, reply, sizeof reply);
	zassert_equal(
	    g_wrap_calls, before + 2u, "seq0 recv #2 must read the socket, not replay recv #1");
	zassert_true(s3 != ALP_CC3501E_RESP_OK, "seq0 recv #2 must not be a duplicate OK");
}

/* DOCUMENTS A KNOWN, DELIBERATELY UNFIXED RESIDUAL (MAJOR, host review of
 * c354208; see the SECOND residual paragraph in the block comment above
 * g_sock_recv_wk_cached, and the matching note above the invalidation check
 * in handle_sock_recv()): a PRE-alp-sdk#2108 host, sharing one 5-bit header
 * seq across every opcode, can alias this cache through opcodes that never
 * touch it.  A completed, collected recv (seq 5, handle 996) stays cached
 * (seq != 0, so it is not the "serve once" case above); 30 SUBSEQUENT
 * SOCK_OPEN polls -- none of them SOCK_RECV, so none of them run this
 * file's invalidate-on-mismatch check -- advance the shared counter without
 * ever touching this entry; the host's NEXT logical recv on the SAME handle
 * H, now a genuinely different request, can be assigned seq 5 again by the
 * wrapped shared counter -- indistinguishable from the first at this
 * cache's (seq, handle) granularity -- and IS served the first recv's stale
 * bytes as OK, with NO socket read.  This is asserted here as the CURRENT,
 * ACTUAL behaviour, not the desired one: fixing it by invalidating this
 * cache on every other worker-routed opcode would reopen BLOCKER 2's class
 * of loss (a same-seq SOCK_RECV retry landing after an intervening,
 * unrelated opcode would then lose its own cached reply).  The real fix is
 * a dedicated per-opcode counter (alp-sdk#2108), already adopted by the
 * host this firmware ships against; this test exists so a future change
 * that happens to close this alias for pre-#2108 hosts too does not do so
 * by accident, unnoticed. */
ZTEST(cc3501e_sock_recv_worker_cache, test_probe_shared_counter_wrap_via_other_opcodes)
{
	uint8_t reply[64];
	uint8_t req[8];
	uint8_t op[8];
	build_recv(req, 5u, 996u);

	zassert_equal(poll_status(req, reply, sizeof reply), ALP_CC3501E_RESP_ERR_BUSY, "recv submit");
	zassert_equal(poll_status(req, reply, sizeof reply),
	              ALP_CC3501E_RESP_OK,
	              "recv collected; host advances");

	/* 30 non-SOCK_RECV poll_by_repeat calls (seq 6..31, 1..4) wrap the shared counter back to 5. */
	uint8_t seq = 5u;
	for (int i = 0; i < 30; i++) {
		seq = (seq >= 31u) ? 1u : (uint8_t)(seq + 1u);
		build_open(op, seq);
		(void)poll_status(op, reply, sizeof reply);
		(void)poll_status(op, reply, sizeof reply);
	}
	zassert_equal(seq, 4u, "counter sits one before the wrap");

	const uint32_t before = g_wrap_calls;
	int            st     = poll_status(req, reply, sizeof reply); /* NEW recv, wrapped seq 5 */
	zassert_equal(g_wrap_calls,
	              before,
	              "KNOWN RESIDUAL: aliased through non-recv opcodes, no socket read happens");
	zassert_equal(st, ALP_CC3501E_RESP_OK, "KNOWN RESIDUAL: served the stale recv as OK");
}

/* PLUMBING CHECK, NOT A TEST OF THE FIX (host review of bfb5f08, MINOR):
 * this passes against UNFIXED firmware too -- it does not exercise
 * hal/ti/cc3501e_hw_ti_sock.c's sticky-EOF table at all (that TI-SDK-only
 * file never links into a host test, see this suite's own top comment), it
 * only proves that protocol_sockets.c's plumbing already threads a
 * HAL-reported OK/0 through to the wire as RESP_OK on repeated polls, not
 * just the first -- a precondition the real fix depends on, not the fix
 * itself.  g_wrap_eof_mode here stands in for what the sticky-EOF table
 * makes the REAL HAL do on silicon; the table's own latch decision (n,
 * want, is_stream) and the want == 0 BLOCKER fix are covered on the host
 * instead by tests/unit/sock_worker_recv_eof/ (sock_worker_recv_eof.h),
 * which sock_worker_recv_eof_should_latch()'s own comment cross-references.
 * Uses two DIFFERENT seqs on the same handle (not a same-seq retry) so
 * each poll reaches the wrapped HAL body fresh, not a cache replay. */
ZTEST(cc3501e_sock_recv_worker_cache, test_worker_plumbing_repeats_hal_ok_zero_as_wire_ok_zero)
{
	uint8_t        reply[64];
	uint8_t        req_a[8];
	uint8_t        req_b[8];
	const uint32_t calls_before = g_wrap_calls;
	build_recv(req_a, 10u, 900u);
	build_recv(req_b, 11u, 900u); /* same handle, different seq -- a later, new recv */

	g_wrap_eof_mode = true;

	int sub_a = poll_status(req_a, reply, sizeof reply);
	zassert_equal(sub_a, ALP_CC3501E_RESP_ERR_BUSY, "first EOF recv submits");
	int got_a = poll_status(req_a, reply, sizeof reply);
	zassert_equal(got_a, ALP_CC3501E_RESP_OK, "first EOF recv answers OK, not RESP_ERR_RADIO");
	/* data_len (alp_cc3501e_sock_recv_resp_t, DATA-relative bytes 20..21,
	 * LE16) must actually be 0, not just the status byte -- a handler that
	 * answered OK with a stale/garbage byte count would pass the status
	 * check above and still be wrong. */
	zassert_equal(reply[5u + (uint32_t)offsetof(alp_cc3501e_sock_recv_resp_t, data_len)],
	              0u,
	              "data_len low byte is 0");
	zassert_equal(reply[5u + (uint32_t)offsetof(alp_cc3501e_sock_recv_resp_t, data_len) + 1u],
	              0u,
	              "data_len high byte is 0");

	int sub_b = poll_status(req_b, reply, sizeof reply);
	zassert_equal(sub_b, ALP_CC3501E_RESP_ERR_BUSY, "second, later EOF recv submits fresh");
	int got_b = poll_status(req_b, reply, sizeof reply);
	zassert_equal(
	    got_b, ALP_CC3501E_RESP_OK, "second EOF recv ALSO answers OK, not RESP_ERR_RADIO");
	zassert_equal(reply[5u + (uint32_t)offsetof(alp_cc3501e_sock_recv_resp_t, data_len)],
	              0u,
	              "second recv's data_len low byte is also 0");
	zassert_equal(reply[5u + (uint32_t)offsetof(alp_cc3501e_sock_recv_resp_t, data_len) + 1u],
	              0u,
	              "second recv's data_len high byte is also 0");

	zassert_equal(g_wrap_calls,
	              calls_before + 2u,
	              "both recvs actually reached the HAL body, not a cache replay");

	g_wrap_eof_mode = false;
}

/* MAJOR 3 (host review of bfb5f08): the ring path's rc == -3 branch
 * (protocol_sockets.c's handle_sock_recv(), src/sock_recv_ring_status.h's
 * "drained and errored" answer) had no test at all.  handle 778 is wrapped
 * to return -3 from cc3501e_hw_sock_recv_ring() (see the wrap's own doc
 * comment above) -- this must answer RESP_ERR_RADIO, the SAME status a
 * genuine worker-path socket failure gets, NEVER RESP_ERR_BUSY (that would
 * spin the host to its poll_by_repeat timeout waiting for bytes that are
 * never coming -- M3b) and NEVER fall through to the worker-routed path
 * (that would make cc3501e_hw_sock_recv() a SECOND reader of an
 * already-failed fd -- the exact #7 hazard -2/BUSY's own comment
 * describes, and the M3c mutation the host review named: -3 misread as
 * "not my handle" and forwarded to the worker). */
ZTEST(cc3501e_sock_recv_worker_cache, test_ring_error_answers_radio_not_busy_or_worker)
{
	uint8_t        reply[64];
	uint8_t        req[8];
	const uint32_t calls_before = g_wrap_calls;
	build_recv(req, 14u, 778u); /* rc == -3, per __wrap_cc3501e_hw_sock_recv_ring above */

	int st = poll_status(req, reply, sizeof reply);
	zassert_equal(st,
	              ALP_CC3501E_RESP_ERR_RADIO,
	              "a drained, errored ring answers RESP_ERR_RADIO -- never BUSY (M3b), "
	              "never falls through to the worker (M3c)");
	zassert_equal(g_wrap_calls,
	              calls_before,
	              "the worker-routed cc3501e_hw_sock_recv() was never called -- the ring "
	              "stays the sole reader of this fd (#7)");
}

/* ITEM 8 (host review of bfb5f08): the ring path's room computation used to
 * treat max_len == 0 as "no cap" -- see the block comment above this room
 * clamp in protocol_sockets.c's handle_sock_recv() for why that is wrong
 * (alp-sdk's cc3501e_sock_recv() never sends wire max_len 0 for a nonzero
 * cap) and the silent-data-loss shape it caused.  handle 779 is wrapped to
 * return rc == -2 ("armed but empty, peer still connected") REGARDLESS of
 * what @p cap it is called with (see the wrap's own doc comment) -- so the
 * ONLY thing distinguishing these two tests is what room
 * handle_sock_recv() itself computed and passed in, captured into
 * g_wrap_ring_last_cap. */
ZTEST(cc3501e_sock_recv_worker_cache, test_ring_max_len_zero_computes_zero_room_and_answers_ok)
{
	uint8_t reply[64];
	uint8_t req[8];
	build_recv(req, 15u, 779u); /* max_len 0 */

	int st = poll_status(req, reply, sizeof reply);
	zassert_equal(g_wrap_ring_last_cap, 0u, "max_len == 0 clamps room to 0, not the buffer size");
	zassert_equal(st,
	              ALP_CC3501E_RESP_OK,
	              "max_len == 0 answers OK immediately, never BUSY -- consistent with the "
	              "worker path's want == 0 short-circuit (sock_worker_recv_eof.h)");
	/* data_len (DATA-relative bytes 20..21) must be 0: this reply carries a
	 * well-formed, zeroed header, not a bare/short status. */
	zassert_equal(reply[5u + (uint32_t)offsetof(alp_cc3501e_sock_recv_resp_t, data_len)],
	              0u,
	              "data_len low byte is 0");
	zassert_equal(reply[5u + (uint32_t)offsetof(alp_cc3501e_sock_recv_resp_t, data_len) + 1u],
	              0u,
	              "data_len high byte is 0");
}

ZTEST(cc3501e_sock_recv_worker_cache, test_ring_nonzero_max_len_still_clamps_room_and_stays_busy)
{
	uint8_t reply[64];
	uint8_t req[8];
	build_recv_ml(req, 16u, 779u, 64u); /* SAME handle as above, max_len 64 this time */

	int st = poll_status(req, reply, sizeof reply);
	zassert_equal(g_wrap_ring_last_cap,
	              64u,
	              "a genuinely nonzero max_len is still clamped to itself, unaffected by the "
	              "max_len == 0 fix");
	zassert_equal(
	    st, ALP_CC3501E_RESP_ERR_BUSY, "max_len != 0 on an armed-but-empty ring is still BUSY");
}

static void reset_worker(void *fixture)
{
	(void)fixture;
	g_wrap_large_mode = false;
	g_wrap_eof_mode   = false;
	worker_init();
}

ZTEST_SUITE(cc3501e_sock_recv_worker_cache, NULL, NULL, reset_worker, NULL, NULL);
