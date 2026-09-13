/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * cc3501e-bridge firmware: TCP/UDP socket command-family handlers
 * (0x20..0x26).  Split out of protocol.c (issue #461); protocol_dispatch()
 * in protocol.c still owns the single command-family switch that routes
 * here.
 *
 * All five WORKER-ROUTE their (blocking) lwIP body off the SPI ISR:
 * every socket op is a tcpip_apimsg round-trip to the lwIP core thread
 * (connect/recv also wait on the network).  OPEN/SEND/RECV return reply
 * DATA (handle / byte-count / bytes) via the payload+reply seam;
 * CONNECT/CLOSE carry a payload only.  Socket payloads are validated
 * field-by-field here (the wire structs are naturally packed, but parse
 * defensively -- the request buffer alignment is transport-defined).
 * The raw req is forwarded to the worker, which re-parses the same
 * struct in the drain.
 */

#include <stdbool.h>
#include <stddef.h> /* offsetof -- SOCK_SEND's own seq field, not a magic wire offset */
#include <string.h>

#include "protocol_internal.h"
#include "../hal/cc3501e_hw.h"

/* SOCK_SEND retry-safe reply cache (issue #88 / alp-sdk#1746).
 *
 * The worker-routed socket opcodes had no request identity, so a host poll
 * that re-sent the identical frame -- which is exactly what poll_by_repeat()
 * does on BUSY/IO -- was indistinguishable, once the worker had already
 * finished the job and freed its slot, from a brand-new request:
 * handle_worker_routed_payload_reply()'s WORKER_IDLE edge submits whatever it
 * is handed.  For CMD_SOCK_SEND that meant a lost/misframed reply caused the
 * payload to be TRANSMITTED AGAIN.
 *
 * protocol_spi.c solves the analogous problem for SPI1_TRANSFER by serving a
 * matching-seq retry straight out of the WORKER JOB SLOT.  That shape does
 * not fit here: sockets share that single slot with every other worker-
 * routed op (Wi-Fi scan, BLE, ...), and worker_poll()'s orphan-discard arm
 * destroys a terminal result the instant any OTHER opcode polls before the
 * host collects it (see the comment there) -- an interleaved poll between a
 * send completing and its retry landing would silently defeat an in-slot
 * cache.  A dedicated static cache sidesteps that entirely, and it is cheap:
 * the reply is at most a 2-byte queued-count, so this is a handful of bytes
 * of static RAM, nothing like SPI1's 4 KB (see protocol_spi.c's own note on
 * that cost, offered as the upgrade path if a host ever needs more than the
 * single most-recent send cached).
 *
 * FILLED EXACTLY ONE WAY: protocol_sock_send_on_worker_complete() (further
 * down), called from worker.c's worker_execute() the INSTANT a SOCK_SEND job
 * reaches a terminal state -- DONE or ERR -- inside the SAME critical
 * section that publishes job.state, strictly BEFORE it flips.  That ordering
 * is what makes the cache authoritative before ANY poll, on any context, for
 * any opcode, can first observe the job as terminal: even worker_poll()'s
 * orphan-discard arm (a DIFFERENT opcode's poll throwing an uncollected
 * SOCK_SEND job away) can never run ahead of the cache being filled (host
 * review, alp-sdk#2035-era).  handle_sock_send() below never stores into
 * this cache itself any more -- a same-seq poll always finds the
 * completion-time entry FIRST, so a separate collect-time store would be
 * unreachable dead code (confirmed by a reviewer mutation: neutering an
 * earlier version of that store left every test green).
 *
 * BOTH OUTCOMES ARE CACHED, not just success: a same seq is BY DEFINITION a
 * retry of the identical logical send -- the host assigns ONE seq per FRAME
 * (one per iteration of cc3501e_sock_send()'s remainder-retry loop,
 * chips/cc3501e/cc3501e_sockets.c, not once per call to that function: each
 * iteration carries different remaining bytes, so it IS a new logical send
 * and gets a new seq; that function's own bounded post-timeout grace
 * re-poll re-sends the SAME frame buffer unmodified, so it reuses THAT
 * frame's seq, not a fresh one) -- so whatever the first execution produced
 * -- OK with a queued count, or a decoded ERR -- IS the correct answer for
 * every later poll of that same seq.  lwIP can fail AFTER queueing (the
 * SimpleLink SDK's api_msg.c has a real path where tcp_write() succeeds and
 * a LATER tcp_output() still returns ERR_RTE, which the SDK's sockets.c
 * surfaces as a plain failed send() despite bytes already having been
 * written, and hal/ti/cc3501e_hw_ti_sock.c maps that to
 * CC3501E_HW_ERR_IO) -- so treating ERR as "safe to just retry" would risk
 * the SAME duplicate-bytes class this cache exists to prevent.  A cached
 * ERR is therefore just as sticky as a cached OK.
 *
 * INVALIDATED ON A DIFFERENT SEQ (mirrors protocol.c's retry_latch_serve()'s
 * "different seq: drop it" rule, issue #102): a request whose seq does NOT
 * match what is cached proves the host has moved on to a new logical send,
 * so handle_sock_send() drops the stale entry before anything else --
 * unlike the generic latch (any worker-routed op's completion can refresh
 * it), this cache only changes on a SOCK_SEND completion, so an old entry
 * could otherwise survive indefinitely.  Safe under the host API's own
 * usage contract, not because anything here enforces it: cc3501e_sock_send()
 * is a single-caller-per-ctx operation -- it stages its whole remainder-
 * retry loop's payload in ctx->sock_buf, a per-ctx scratch buffer shared
 * across that entire call (alp-sdk's chips/cc3501e/core.h).  ctx->sock_busy
 * does NOT serialise concurrent callers: its own doc comment says it
 * "catches same-call-stack reentrancy, not two truly concurrent callers on
 * one ctx", and its check-then-set (`if (ctx->sock_busy) return ...;
 * ctx->sock_busy = true;`, cc3501e_sockets.c) is itself unlocked, a TOCTOU
 * race for genuinely concurrent callers.  Under the single-caller contract,
 * seq only ever moves forward (then wraps).  If that contract is violated
 * anyway (two threads sending on one ctx), an OLDER seq's frame can land on
 * the wire AFTER a newer one's, and this invalidation would then discard
 * and re-execute BOTH sends -- a consequence of the pre-existing misuse
 * (the two threads are already corrupting each other's writes to the
 * shared sock_buf before this firmware ever sees a frame), not a new hazard
 * this invalidation introduces.
 *
 * RESIDUAL: SEQ-WRAP ALIASING, NOT FULLY CLOSED.  The cache is keyed on an
 * 8-bit seq alone, and the invalidation above drops the entry on any
 * SOCK_SEND that reaches THIS DISPATCH with a different seq -- dispatched,
 * not merely completed -- so closing the alias now needs 255 consecutive
 * host seq increments whose FRAMES NEVER REACH THIS HANDLER at all: a
 * host-side failure before the frame is even sent (cc3501e_sock_send()
 * reporting NOT_READY, or its transport-lock acquire timing out --
 * cc3501e_lock_acquire() inside poll_by_repeat(), alp-sdk#2035), or a
 * firmware-side rejection that returns before the seq check runs (this
 * handler's own length check above, or a CRC failure at the framing layer
 * beneath protocol_dispatch(), which never reaches this TU at all).  If
 * that happens 255 times running, the host's per-ctx counter wraps back to
 * a value that is STILL cached, and a genuinely NEW send assigned that same
 * seq is indistinguishable from a retry of the old one -- it would be
 * answered the stale cached outcome WITHOUT EXECUTING.  Nothing narrower
 * than a wider identity (more than 8 bits) closes that; the different-seq
 * invalidation above at least bounds the staleness window to "no
 * DISPATCHED different seq since", not "forever". */
static volatile bool               g_sock_send_cached;
static volatile uint8_t            g_sock_send_seq;
static volatile alp_cc3501e_resp_t g_sock_send_status;
static volatile uint8_t            g_sock_send_reply[2]; /* valid iff g_sock_send_status == OK */

/* Mirrors handle_worker_routed_payload_reply()'s WORKER_ERR mapping
 * (protocol.c) byte-for-byte, so a cached ERR answers identically to what a
 * genuine collect through that path would have produced.  SOCK_SEND's own
 * HAL body (cc3501e_hw_sock_send) has not been observed to return
 * CC3501E_HW_ERR_STATE -- that code is BLE_GATT_REGISTER / sock_listen's --
 * but the mapping stays complete rather than assume it never will. */
static alp_cc3501e_resp_t sock_send_hw_err_to_resp(int hw_rv)
{
	if (hw_rv == CC3501E_HW_ERR_NOTIMPL) return ALP_CC3501E_RESP_ERR_NOT_READY;
	if (hw_rv == CC3501E_HW_ERR_INVAL) return ALP_CC3501E_RESP_ERR_INVALID;
	if (hw_rv == CC3501E_HW_ERR_STATE) return ALP_CC3501E_RESP_ERR_STATE;
	return ALP_CC3501E_RESP_ERR_RADIO;
}

/* See worker.h for the full contract.  Caches the FINAL outcome either way
 * -- RESP_OK with the queued-count reply, or the decoded RESP_ERR_* -- see
 * the cache block comment above for why a cached ERR is correct here, not
 * merely tolerated. */
void protocol_sock_send_on_worker_complete(uint8_t seq, int hw_rv, const uint8_t *data, size_t len)
{
	if (hw_rv == CC3501E_HW_OK) {
		if (len != sizeof(g_sock_send_reply)) return; /* defensive; worker.c always passes 2 */
		memcpy((void *)g_sock_send_reply, data, sizeof(g_sock_send_reply));
		g_sock_send_status = ALP_CC3501E_RESP_OK;
	} else {
		g_sock_send_status = sock_send_hw_err_to_resp(hw_rv);
	}
	g_sock_send_seq    = seq;
	g_sock_send_cached = true;
}

/* SOCK_OPEN (0x20): req = alp_cc3501e_sock_open_t { family | type | protocol |
 * reserved } = 4 B.  Reply DATA = alp_cc3501e_sock_handle_t (4 B). */
alp_cc3501e_resp_t handle_sock_open(const uint8_t *req,
                                    size_t         req_len,
                                    uint8_t       *reply_data,
                                    size_t         reply_cap,
                                    size_t        *reply_data_len)
{
	*reply_data_len = 0u;
	if (req_len != sizeof(alp_cc3501e_sock_open_t)) return ALP_CC3501E_RESP_ERR_INVALID;
	if (req[0] != (uint8_t)ALP_CC3501E_SOCK_FAMILY_IPV4) {
		return ALP_CC3501E_RESP_ERR_INVALID; /* v1 IP stack is IPv4-only */
	}
	return handle_worker_routed_payload_reply(ALP_CC3501E_CMD_SOCK_OPEN,
	                                          req,
	                                          req_len,
	                                          sizeof(alp_cc3501e_sock_handle_t),
	                                          reply_data,
	                                          reply_cap,
	                                          reply_data_len);
}

/* SOCK_CONNECT (0x21): req = alp_cc3501e_sock_connect_t = 24 B.  No reply data. */
alp_cc3501e_resp_t handle_sock_connect(const uint8_t *req,
                                       size_t         req_len,
                                       uint8_t       *reply_data,
                                       size_t         reply_cap,
                                       size_t        *reply_data_len)
{
	(void)reply_data;
	(void)reply_cap;
	if (req_len != sizeof(alp_cc3501e_sock_connect_t)) return ALP_CC3501E_RESP_ERR_INVALID;
	if (req[4] != (uint8_t)ALP_CC3501E_SOCK_FAMILY_IPV4) { /* peer.family */
		return ALP_CC3501E_RESP_ERR_INVALID;
	}
	return handle_worker_routed_payload(ALP_CC3501E_CMD_SOCK_CONNECT, req, req_len, reply_data_len);
}

/* SOCK_BIND (0x25): req = alp_cc3501e_sock_bind_t = 24 B.  No reply data.
 *
 * Byte-for-byte the SOCK_CONNECT layout (handle | reserved | sock_addr), so the
 * validation and the worker-side parse are the same shape; only the endpoint's
 * meaning differs.  An all-zero local.addr is INADDR_ANY, which is what a
 * server on the soft-AP binds -- the AP address does not exist until the role
 * is up -- so unlike CONNECT there is nothing to reject about a zero address. */
alp_cc3501e_resp_t handle_sock_bind(const uint8_t *req,
                                    size_t         req_len,
                                    uint8_t       *reply_data,
                                    size_t         reply_cap,
                                    size_t        *reply_data_len)
{
	(void)reply_data;
	(void)reply_cap;
	if (req_len != sizeof(alp_cc3501e_sock_bind_t)) return ALP_CC3501E_RESP_ERR_INVALID;
	if (req[4] != (uint8_t)ALP_CC3501E_SOCK_FAMILY_IPV4) { /* local.family */
		return ALP_CC3501E_RESP_ERR_INVALID;
	}
	return handle_worker_routed_payload(ALP_CC3501E_CMD_SOCK_BIND, req, req_len, reply_data_len);
}

/* SOCK_LISTEN (0x26): req = alp_cc3501e_sock_listen_t { handle | backlog |
 * reserved } = 4 B.  No reply data.
 *
 * Makes the socket passive; it does NOT wait for a connection.  Each inbound
 * connection is accepted on the housekeeping tick by
 * cc3501e_hw_sock_accept_pump() and delivered to the host as an
 * EVT_SOCK_ACCEPTED entry on the event ring -- see the wire-protocol v9 note in
 * <alp/protocol/cc3501e.h> for why there is no accept opcode. */
alp_cc3501e_resp_t handle_sock_listen(const uint8_t *req,
                                      size_t         req_len,
                                      uint8_t       *reply_data,
                                      size_t         reply_cap,
                                      size_t        *reply_data_len)
{
	(void)reply_data;
	(void)reply_cap;
	if (req_len != sizeof(alp_cc3501e_sock_listen_t)) return ALP_CC3501E_RESP_ERR_INVALID;
	if (req[0] == 0u && req[1] == 0u) return ALP_CC3501E_RESP_ERR_INVALID; /* handle 0 invalid */
	return handle_worker_routed_payload(ALP_CC3501E_CMD_SOCK_LISTEN, req, req_len, reply_data_len);
}

/* SOCK_SEND (0x22): req = alp_cc3501e_sock_send_t (8 B) + data_len inline bytes.
 * Reply DATA = uint16_t LE queued-byte count (RESP_OK only).
 *
 * req[3] is alp_cc3501e_sock_send_t.seq (v7; formerly `reserved`, always 0
 * through v6 -- see the wire-compat note on CC3501E_FW_IMPLEMENTS_PROTOCOL in
 * protocol_meta.c).  The host assigns it once per FRAME -- one per iteration
 * of cc3501e_sock_send()'s remainder-retry loop (chips/cc3501e/
 * cc3501e_sockets.c), and that function's own bounded post-timeout grace
 * re-poll reuses the SAME frame's seq rather than assigning a fresh one --
 * so a poll carrying that SAME seq is, by definition, the SAME logical
 * frame -- answered from the cache above WITHOUT touching the worker at
 * all, whatever that frame's outcome was (OK or a decoded ERR -- see the
 * cache block's comment).  A DIFFERENT seq invalidates whatever was cached
 * and falls through toward the worker-routed path. */
alp_cc3501e_resp_t handle_sock_send(const uint8_t *req,
                                    size_t         req_len,
                                    uint8_t       *reply_data,
                                    size_t         reply_cap,
                                    size_t        *reply_data_len)
{
	*reply_data_len = 0u;
	if (req_len < sizeof(alp_cc3501e_sock_send_t)) return ALP_CC3501E_RESP_ERR_INVALID;
	const uint16_t data_len = (uint16_t)req[4] | ((uint16_t)req[5] << 8);
	if (req_len != sizeof(alp_cc3501e_sock_send_t) + (size_t)data_len) {
		return ALP_CC3501E_RESP_ERR_INVALID; /* declared length must match the frame */
	}

	const uint8_t seq = req[3];

	/* A different seq than what is cached proves the host has moved on to a
	 * new logical send; drop the stale entry here rather than leave it to
	 * linger until some LATER send happens to overwrite it (see the cache
	 * block's INVALIDATED ON A DIFFERENT SEQ comment above). */
	if (g_sock_send_cached && seq != g_sock_send_seq) {
		g_sock_send_cached = false;
	}

	if (g_sock_send_cached) {
		/* This exact poll is what would once have reached
		 * handle_worker_routed_payload_reply()'s own WORKER_DONE/WORKER_ERR
		 * -> worker_reset() path -- now short-circuited by the cache hit
		 * above.  Reclaim the slot here instead of leaving that to chance:
		 * without it, a finished SOCK_SEND job would sit there, terminal,
		 * until some UNRELATED opcode's poll happened to orphan-discard it
		 * (worker_poll()'s own arm). */
		(void)worker_reclaim_matching_terminal(
		    ALP_CC3501E_CMD_SOCK_SEND, offsetof(alp_cc3501e_sock_send_t, seq), seq);
		if (g_sock_send_status == ALP_CC3501E_RESP_OK) {
			if (reply_cap < sizeof(g_sock_send_reply)) return ALP_CC3501E_RESP_ERR_NO_MEM;
			memcpy(reply_data, (const void *)g_sock_send_reply, sizeof(g_sock_send_reply));
			*reply_data_len = sizeof(g_sock_send_reply);
		}
		return g_sock_send_status;
	}

	/* STALE-JOB GUARD (mirrors protocol_spi.c's SPI1_TRANSFER seq check, one
	 * level up): reaching here means this request's seq is NOT cached -- it
	 * just invalidated whatever was cached above, or nothing ever was.  This
	 * is either the very first SOCK_SEND ever, or the host has moved on to a
	 * genuinely NEW send under a NEW seq while an OLDER send's job may still
	 * be sitting in the worker slot: submitted, finished, but this handler
	 * never got the chance to answer it before the host gave up on
	 * poll_by_repeat.  worker_poll() inside
	 * handle_worker_routed_payload_reply() below matches the job slot by
	 * OPCODE ALONE, so without this check that call would hand the OLD job's
	 * finished result back as if it were THIS request's answer -- RESP_OK
	 * (or a decoded ERR) plus the OLD send's own reply -- and THIS request's
	 * actual payload would never be submitted to the worker at all.
	 *
	 * worker_discard_stale_terminal() ATOMICALLY compares the sitting job's
	 * ORIGINALLY-submitted seq (worker.c's job.req at this same offset)
	 * against THIS request's seq and, on a mismatch, evicts it -- one
	 * critical section, so the compare-and-reset cannot race the drain/ISR
	 * even in principle (see worker.h).  QUEUED/RUNNING is left alone
	 * regardless of seq: there is no terminal result yet for it to
	 * misclaim, so the fall-through correctly reports BUSY.
	 *
	 * RESIDUAL, BOTH DIRECTIONS, STATED PLAINLY -- this narrows the failure
	 * cc3501e-bridge-firmware#107's abandoned-seq scenario names, it does
	 * not close it to zero, and it is a TRADE, not a strict improvement:
	 *
	 *   - Once an old send's bytes are genuinely QUEUED into lwIP (the
	 *     worker body ran, MSG_DONTWAIT accepted some or all of them), that
	 *     queuing already happened -- evicting the sitting job here only
	 *     stops this firmware from MISREPORTING that outcome as THIS
	 *     request's, it cannot un-send bytes already handed to the socket.
	 *     The host's own post-timeout collect grace (alp-sdk#2035's
	 *     cc3501e_sock_send(), chips/cc3501e/cc3501e_sockets.c) exists
	 *     precisely to collect the old send before giving up -- not to
	 *     cache it: the cache here is filled at COMPLETION
	 *     (protocol_sock_send_on_worker_complete(), worker.h), whether or
	 *     not this grace's own collect ever happens.  If the grace ALSO
	 *     fails and the host then reissues the SAME remaining bytes under a
	 *     NEW seq, those bytes queue TWICE.
	 *     BEFORE this guard existed, that specific case -- host gives up,
	 *     reissues under a new seq -- happened to come out RIGHT: the new
	 *     request would collect the old send's stale OK/count instead of
	 *     submitting, so the resend was silently swallowed rather than
	 *     duplicated.  This guard trades that (wrong for every OTHER reason:
	 *     the new request's own data is never sent, and a genuinely
	 *     different send gets someone else's byte count) for the
	 *     duplicate-bytes outcome above in this one narrow window.  No
	 *     counter tracks how often either side fires; this paragraph is the
	 *     only record of the trade.
	 *   - In the CC3501E_WIRE_CRC=OFF build the wire has no CRC trailer, so a
	 *     single bit flip landing in req[3] on an ordinary retry reads as a
	 *     DIFFERENT seq: this guard evicts the job's own still-good result
	 *     (and the cache-invalidation above drops its cached answer too) and
	 *     the fall-through submits again, queuing the same bytes twice.
	 *     SPI1_TRANSFER (protocol_spi.c) carries the identical exposure from
	 *     its own req[3]-seq check under the same build option -- this is
	 *     not a new class of risk, just the same one CRC-off already
	 *     accepts, now shared by a second opcode. */
	(void)worker_discard_stale_terminal(
	    ALP_CC3501E_CMD_SOCK_SEND, offsetof(alp_cc3501e_sock_send_t, seq), seq);

	return handle_worker_routed_payload_reply(
	    ALP_CC3501E_CMD_SOCK_SEND, req, req_len, 2u, reply_data, reply_cap, reply_data_len);
}

/* LAZY-COMMIT replay identity for the fast path below (silent SOCK_RECV
 * data loss on a CRC-rejected reply, host review): alp_cc3501e_sock_recv_t
 * carries no seq of its own, so a poll_by_repeat() retry of a CRC-rejected
 * reply is byte-identical to the request that produced it EXCEPT for the
 * generic 5-bit header seq (protocol.c's s_current_req_seq), which
 * poll_by_repeat() holds constant across every retry of one logical call.
 * DISPATCH CONTEXT ONLY, same as the ring itself -- see
 * hal/ti/cc3501e_hw_ti_sock.c's CONCURRENCY comment above rx_ring; the task-
 * side pump never reads or writes these, so no volatile and no cross-
 * context ordering concern. */
static uint8_t  last_recv_seq;
static uint16_t last_recv_handle;

/* SOCK_RECV (0x23): req = alp_cc3501e_sock_recv_t { handle | max_len } = 4 B.
 * Reply DATA = alp_cc3501e_sock_recv_resp_t (24 B) + received bytes inline. */
alp_cc3501e_resp_t handle_sock_recv(const uint8_t *req,
                                    size_t         req_len,
                                    uint8_t       *reply_data,
                                    size_t         reply_cap,
                                    size_t        *reply_data_len)
{
	*reply_data_len = 0u;
	if (req_len != sizeof(alp_cc3501e_sock_recv_t)) return ALP_CC3501E_RESP_ERR_INVALID;

	/* FAST PATH: serve from the prefetch ring, synchronously.
	 *
	 * This runs in the SPI callback and must not call lwIP -- but it does not
	 * need to: cc3501e_hw_sock_pump() has already done the lwIP read on the task
	 * and left the bytes in a ring, so all that happens here is a memcpy.  Same
	 * shape as cc3501e_hw_ota_write, which is synchronous for the same reason.
	 *
	 * That turns CMD_SOCK_RECV from a submit/collect PAIR -- worker routed, so
	 * the first request always answers BUSY and the host must come back -- into
	 * ONE bridge transaction with no worker round trip and no wait for the
	 * worker loop to come round.  cc3501e_hw_sock_recv_ring returns -1 when this
	 * handle is not the prefetched one, and then we fall through to the original
	 * worker path unchanged.
	 *
	 * KNOWN FOLLOW-UP, NOT FIXED HERE: the fallback below --
	 * handle_worker_routed_payload_reply() for a handle that is NOT the
	 * prefetched one (UDP sockets, and STREAM sockets accepted but never
	 * armed for prefetch) -- has the SAME CRC-rejected-reply data-loss hole
	 * this fast path just closed: cc3501e_hw_sock_recv()'s lwip_recvfrom()
	 * also has no way to re-deliver bytes a lost reply already consumed from
	 * the socket.  It is excluded from the generic retry latch for the same
	 * "stream-consuming, not idempotent" reason SOCK_RECV as a whole is (see
	 * protocol.c's retry_latch_applies()), and it is worker-routed rather
	 * than a single synchronous call, which does not fit this fast path's
	 * lazy-commit shape (there is no ring to hold bytes back in). */
	{
		const uint16_t handle  = (uint16_t)((uint16_t)req[0] | ((uint16_t)req[1] << 8));
		const uint16_t max_len = (uint16_t)((uint16_t)req[2] | ((uint16_t)req[3] << 8));
		const size_t   hdr     = sizeof(alp_cc3501e_sock_recv_resp_t);
		if (reply_cap > hdr) {
			size_t room = reply_cap - hdr;
			if (max_len != 0u && room > (size_t)max_len) room = (size_t)max_len;

			/* seq 0 (ALP_CC3501E_REQ_SEQ_NONE) never claims a replay, same
			 * reservation the generic retry latch uses -- a host that does not
			 * assign one (every bare cc3501e_request() call site, or a pre-v8
			 * host) would otherwise read every frame as "seq 0, same as last",
			 * i.e. always a replay of whatever was last served. */
			const uint8_t seq = protocol_current_req_seq();
			const bool replay = (seq != 0u && seq == last_recv_seq && handle == last_recv_handle);

			uint16_t  got = 0u;
			const int rc =
			    cc3501e_hw_sock_recv_ring(handle, &reply_data[hdr], (uint16_t)room, replay, &got);

			/* Record THIS call's identity for the NEXT one to compare against --
			 * but only when this handle actually engaged the ring (rc != -1).
			 * Recording it unconditionally would let an UNRELATED handle's
			 * SOCK_RECV (one this ring does not own, rc == -1, worker-routed
			 * below) overwrite last_recv_seq/last_recv_handle in between the
			 * prefetched handle's own original serve and its retry, making
			 * that retry's replay check compare against the wrong call and
			 * miss the very case this fix exists for. */
			if (rc != -1) {
				last_recv_seq    = seq;
				last_recv_handle = handle;
			}

			if (rc == -2) {
				/* Armed for this handle but momentarily empty.  The pump is the
				 * ONLY reader of this fd -- do NOT fall through and submit a
				 * worker job, or cc3501e_hw_sock_recv()'s lwip_recvfrom() becomes
				 * a second reader on the same socket and the stream loses a chunk
				 * (#7).  BUSY is what poll_by_repeat retries on, so the host comes
				 * back and the pump will have staged the bytes by then. */
				*reply_data_len = 0u;
				return ALP_CC3501E_RESP_ERR_BUSY;
			}
			if (rc >= 0) {
				/* from[] is zeroed for STREAM sockets; data_len then the bytes. */
				memset(reply_data, 0, hdr);
				reply_data[sizeof(alp_cc3501e_sock_addr_t)]      = (uint8_t)(got & 0xFFu);
				reply_data[sizeof(alp_cc3501e_sock_addr_t) + 1u] = (uint8_t)((got >> 8) & 0xFFu);
				*reply_data_len                                  = hdr + (size_t)got;
				return ALP_CC3501E_RESP_OK;
			}
		}
	}

	return handle_worker_routed_payload_reply(ALP_CC3501E_CMD_SOCK_RECV,
	                                          req,
	                                          req_len,
	                                          sizeof(alp_cc3501e_sock_recv_resp_t),
	                                          reply_data,
	                                          reply_cap,
	                                          reply_data_len);
}

/* SOCK_CLOSE (0x24): req = alp_cc3501e_sock_close_t { handle | reserved } = 4 B.
 * No reply data. */
alp_cc3501e_resp_t handle_sock_close(const uint8_t *req,
                                     size_t         req_len,
                                     uint8_t       *reply_data,
                                     size_t         reply_cap,
                                     size_t        *reply_data_len)
{
	(void)reply_data;
	(void)reply_cap;
	if (req_len != sizeof(alp_cc3501e_sock_close_t)) return ALP_CC3501E_RESP_ERR_INVALID;
	return handle_worker_routed_payload(ALP_CC3501E_CMD_SOCK_CLOSE, req, req_len, reply_data_len);
}
