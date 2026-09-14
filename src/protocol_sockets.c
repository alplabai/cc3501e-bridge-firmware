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
 * genuine collect through that path would have produced.  Shared by BOTH
 * completion-time caches in this file -- SOCK_SEND's above and SOCK_RECV's
 * worker-fallback cache below -- since the mapping is generic (HAL error
 * code -> wire response), not specific to either opcode's HAL body.  Neither
 * cc3501e_hw_sock_send() nor cc3501e_hw_sock_recv() has been observed to
 * return CC3501E_HW_ERR_STATE -- that code is BLE_GATT_REGISTER / sock_listen's
 * -- but the mapping stays complete rather than assume it never will. */
static alp_cc3501e_resp_t sock_worker_hw_err_to_resp(int hw_rv)
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
		g_sock_send_status = sock_worker_hw_err_to_resp(hw_rv);
	}
	g_sock_send_seq    = seq;
	g_sock_send_cached = true;
}

/* SOCK_RECV WORKER-FALLBACK retry-safe reply cache.
 *
 * Closes the KNOWN FOLLOW-UP handle_sock_recv()'s ring fast path used to
 * document below (host review): a handle NOT owned by the prefetch ring --
 * UDP, or a STREAM socket accepted but never armed for prefetch -- falls
 * back to handle_worker_routed_payload_reply(), a submit/collect PAIR with no
 * lazy-commit ring to hold bytes back in.  cc3501e_hw_sock_recv()'s
 * lwip_recvfrom() has no way to re-deliver bytes a lost reply already
 * consumed from the socket, so a CRC-rejected reply followed by
 * poll_by_repeat()'s identical-frame retry used to hand the retry the NEXT
 * bytes off the socket -- the lost reply's bytes were gone for good, and for
 * a STREAM socket that silently dropped data reported OK.
 *
 * SAME SHAPE as the #88/#107 SOCK_SEND cache immediately above, adapted for a
 * request that carries no seq of its own: alp_cc3501e_sock_recv_t is just
 * { handle | max_len } (v9 protocol, <alp/protocol/cc3501e.h>), so this cache
 * keys on the request's *generic* 5-bit header seq
 * (protocol_current_req_seq(), the SAME field the ring fast path's own
 * lazy-commit replay check above already uses) PLUS the handle, rather than a
 * per-opcode wire field.  Since that seq does not ride in job.req the way
 * SOCK_SEND's does, protocol_sock_recv_note_submit() (worker.h) carries it
 * across the submit -> completion gap: protocol.c's
 * handle_worker_routed_payload_reply() calls it on the WORKER_IDLE -> QUEUED
 * submit edge (the same seam WIFI_CONNECT_STA already special-cases there for
 * cc3501e_hw_wifi_mark_connecting()), and worker.c's worker_execute() reads
 * the handle straight out of job.req and pairs it with the noted seq to fill
 * this cache -- split across protocol_sock_recv_worker_invalidate() / _copy()
 * / _publish() (worker.h, MINOR 6 of the 1118c99 review) so the multi-KB
 * memcpy runs OUTSIDE any critical section; only the final key/status/len/
 * cached publish and job.state itself are set inside one, mirroring
 * protocol_sock_send_on_worker_complete()'s own ordering (see worker.h for
 * why that ordering is load-bearing).
 *
 * BOTH OUTCOMES ARE CACHED (OK with the received bytes, or a decoded ERR),
 * for the identical reason the SOCK_SEND cache does: a same seq+handle poll
 * is a retry of the SAME logical recv GIVEN a host with a dedicated SOCK_RECV
 * counter (alp-sdk#2108) -- see the RESIDUAL paragraph below for the
 * pre-#2108 case, where the header seq is a single counter shared by every
 * opcode and a same (seq, handle) pair is not always the same logical
 * request.
 *
 * SIZED TO THE WIRE'S OWN CEILING, NO EXEMPTION: the cap is
 * CC3501E_REPLY_DATA_MAX (protocol.h's "Maximum reply DATA bytes a handler
 * may emit" -- ALP_CC3501E_MAX_PAYLOAD - 1 - ALP_CC3501E_CRC_BYTES = 4093 B
 * under the default CC3501E_WIRE_CRC=ON, 4095 B under =OFF), not
 * ALP_CC3501E_MAX_PAYLOAD: that constant IS the actual bound
 * protocol_build_reply() enforces on every handler's reply_cap, so it is what
 * "the reply buffer size, whichever actually bounds it" resolves to here --
 * ALP_CC3501E_MAX_PAYLOAD alone overstates it by 1 + ALP_CC3501E_CRC_BYTES,
 * bytes no handler's DATA can ever actually occupy.  A first attempt capped
 * this at 256 B and MISSED THE ACTUAL BUG: the exact run10 loss shape is an
 * ACCEPTED TCP socket read at ~4071 B per cc3501e_sock_recv() call -- ABOVE
 * that smaller cap -- so the 256 B version left the bulk case, the one the
 * bug report is about, uncached and still losing a block on a CRC-rejected
 * reply.  There is no "typical UDP datagram" traffic pattern narrow enough to
 * justify a partial cap here; this path has to cover the same range of reply
 * sizes the ring fast path does.
 *
 * CLOSED, NOT JUST NARROWED: an earlier version of this comment described a
 * PRE-EXISTING gap in worker.c's own SOCK_RECV data_cap -- it predated the
 * wire MAJOR 4 CRC-trailer tax (#2035) and could compute a reply up to 2 B
 * LARGER than CC3501E_REPLY_DATA_MAX under CC3501E_WIRE_CRC=ON (24 + 4071 =
 * 4095 vs the 4093 B ceiling), which worker_poll()'s out_cap clamp then
 * silently truncated -- SILENT DATA LOSS, not merely a cache-defensive
 * corner case: the truncated bytes had already been consumed off the lwIP
 * socket by cc3501e_hw_sock_recv() before the truncation ever happened, and
 * the host bench app's own recv granularity (exactly 4071 B/call) meant a
 * worker-fallback socket hit this on EVERY full read, not a rare boundary.
 * worker.c's data_cap (worker_execute()'s SOCK_RECV case) is now itself
 * bounded by CC3501E_REPLY_DATA_MAX minus the recv-resp header, so a
 * completed job can never again report more bytes than this cache -- or the
 * wire -- can carry.  This cache's own defensive guard below
 * (`len > sizeof(g_sock_recv_wk_reply)`) is consequently UNREACHABLE for
 * SOCK_RECV in ordinary operation; kept as a LOUD backstop (caches a
 * deterministic RESP_ERR_NO_MEM rather than the earlier silent
 * decline-to-cache) for any future regression in that bound, not because the
 * gap it once covered still exists.  worker_poll() itself (worker.c) carries
 * the identical loud guard for every OTHER worker-routed payload-reply
 * opcode.
 *
 * RAM: a MAX_PAYLOAD-sized second buffer does not fit in DRAM_NON_SECURE
 * alongside the WiFi+BLE stacks -- measured: the real `--wifi --ble`
 * production image link-FAILS with it left in ordinary .bss ("program will
 * not fit into available memory" -- tiarmlnk, GROUP_4 needed 0x2487 B where
 * only a 0x16cb B hole remained).  hal/ti/cc3501e_hw_ti_sock.c's own 64 KB
 * rx_ring solves the identical problem the identical way: g_sock_recv_wk_reply
 * below carries the SAME `.bss.sock_ring` section attribute that ring uses,
 * which ti/build_ti.sh's (and build_ti.ps1's) linker-script patch places into
 * TCM_DRAM_NON_SECURE ahead of the generic `.bss` catch-all -- TCM is safe
 * here for the identical reason it is safe for the ring: this buffer is
 * CPU-only memory (the worker's memcpy in, this handler's memcpy out), no DMA
 * engine ever addresses it.  The GROUP rule matches by INPUT SECTION NAME, not
 * by source file, so this array's own `.bss.sock_ring` input section joins the
 * ring's in the same output section with no script change needed -- verified
 * in cc3501e-bridge.map (both symbols resolve at 0x200xxxxx, TCM_DRAM_NON_SECURE,
 * not the 0x28xxxxxx DRAM_NON_SECURE bank the earlier 256 B version used).
 * The three small scalars below (seq/handle/status/len) stay in ordinary
 * static storage -- a few bytes, no different from any other file-scope
 * static in this TU, nothing forces them into TCM too.
 *
 * RAM COST: 4093 B (g_sock_recv_wk_reply, in TCM; 4095 B under
 * CC3501E_WIRE_CRC=OFF) + 6 B (seq/handle/status/len, ordinary .bss) + 1 B
 * (g_sock_recv_job_seq, ordinary .bss) -- still far larger than SOCK_SEND's
 * 2-byte cache because a recv reply carries the received DATA, not a small
 * fixed count, but it lives where the ring already proved 15x that much fits.
 * Single most-recent entry only, the same tradeoff the SOCK_SEND cache
 * already makes (see its own block comment).
 *
 * INVALIDATED ON A DIFFERENT SEQ, A DIFFERENT HANDLE, OR SOCK_CLOSE OF THE
 * CACHED HANDLE: handle_sock_recv() below drops a mismatched entry -- run
 * FIRST, right after computing this dispatch's own seq, BEFORE the ring fast
 * path (see the hoisting comment there for why the ORDER is load-bearing) --
 * mirroring handle_sock_send()'s own different-seq rule; handle_sock_close()
 * drops it when the handle being closed matches (a closed handle's last recv
 * reply must never outlive the socket -- the ONLY path that frees a
 * host-visible handle number for reuse, confirmed against every
 * hal/ti/cc3501e_hw_ti_sock.c lwip_close() site: the sock_open failure path,
 * the accept-table overflow path, and the accept-event-ring-full path each
 * close an fd that was NEVER handed back to the host as a valid handle in
 * the first place).  SOCK_OPEN does NOT invalidate (see handle_sock_open()'s
 * own comment for why an earlier version doing so was itself a bug, not a
 * safety net).
 *
 * RESIDUAL -- STATED PRECISELY (host review, MAJOR 5 of the 1118c99 review;
 * do not read this as fully closed by alp-sdk#2108's dedicated SOCK_RECV seq
 * counter): recovery is guaranteed ONLY for an IMMEDIATE same-handle retry --
 * this dispatch's own seq+handle exactly matching what THIS SAME handle's
 * immediately-preceding worker-routed completion cached.  A CROSS-handle loss
 * path remains even with the dedicated counter: recv (A, H1) consumes bytes
 * off the socket but the host times out waiting for the reply (poll_by_repeat
 * gives up, never retries with the SAME candidate seq A); the very next recv
 * the host issues, on a DIFFERENT handle H2, is assigned that SAME candidate
 * seq A (the host's per-recv counter had not yet advanced past A, since it
 * only advances on an ALP_OK collect, which H1 never delivered) -- H2's
 * dispatch invalidates H1's (A, H1) entry (different handle) before H2 ever
 * touches the cache, and H1's OWN next recv is assigned A+1, so H1's lost
 * block is gone for good.  Building per-handle state to close this is
 * deliberately NOT done here; filed as a follow-up.
 *
 * A SECOND, SEPARATE residual (MAJOR, host review of c354208): a PRE-#2108
 * host -- one still using the single 5-bit header seq shared by every
 * opcode, not a dedicated SOCK_RECV counter -- can alias THIS cache through
 * opcodes that never touch it at all.  recv (5, H) completes and is
 * collected (host advances past 5); 30 SUBSEQUENT SOCK_OPEN/SOCK_SEND/etc.
 * polls (none of them SOCK_RECV, so none of them run this file's
 * invalidate-on-mismatch check) wrap the shared counter back to 5; the
 * host's NEXT recv on the SAME handle H, now logically a brand-new request,
 * is ALSO assigned seq 5 -- indistinguishable from the first at this cache's
 * (seq, handle) granularity -- and is served the FIRST recv's stale bytes as
 * OK, with no socket read.  Deliberately NOT fixed by invalidating this
 * cache on every OTHER worker-routed opcode: that closes this hole by
 * reopening BLOCKER 2's (a same-seq SOCK_RECV retry landing after an
 * intervening, unrelated opcode would then lose its own cached reply the
 * identical way SOCK_OPEN's old unconditional invalidation did).  A
 * dedicated per-opcode counter (alp-sdk#2108) is the actual fix, already
 * adopted by the host this firmware ships against; a pre-#2108 host remains
 * exposed to this alias.  See
 * test_probe_shared_counter_wrap_via_other_opcodes for a test that documents
 * -- not fixes -- this. */
#define CC3501E_SOCK_RECV_WK_CACHE_CAP CC3501E_REPLY_DATA_MAX

static volatile bool               g_sock_recv_wk_cached;
static volatile uint8_t            g_sock_recv_wk_seq;
static volatile uint16_t           g_sock_recv_wk_handle;
static volatile alp_cc3501e_resp_t g_sock_recv_wk_status;
static volatile uint16_t           g_sock_recv_wk_reply_len;
/* .bss.sock_ring: TCM placement (see the block comment above) -- ONLY the
 * byte buffer needs it, so only this one array carries the attribute; the
 * scalars above stay in ordinary static storage. */
static volatile uint8_t g_sock_recv_wk_reply[CC3501E_SOCK_RECV_WK_CACHE_CAP]
    __attribute__((section(".bss.sock_ring")));

/* WRITTEN ONLY from protocol.c's WORKER_IDLE submit edge (SPI-ISR/dispatch
 * context); READ from worker.c's worker_execute() at completion, which on
 * the real (CC3501E_WIFI) build runs on the DRAIN THREAD -- a genuinely
 * different context, unlike last_recv_seq/last_recv_handle further down
 * (dispatch-context-only on both ends).  volatile for that cross-context
 * hand-off, the same reasoning worker.c applies to job.req/job.state.  Cannot
 * be overwritten mid-flight by an unrelated SOCK_RECV: worker_poll() matches
 * an in-flight job by OPCODE ALONE, so a second SOCK_RECV dispatch landing
 * while this one is QUEUED/RUNNING only ever observes QUEUED/RUNNING and
 * answers BUSY without reaching the IDLE submit edge again. */
static volatile uint8_t g_sock_recv_job_seq;

void protocol_sock_recv_note_submit(uint8_t seq)
{
	g_sock_recv_job_seq = seq;
}

/* Step 1 (worker.h): clear the cache first, inside worker.c's OWN short
 * critical section, before either ~4 KB copy below -- see worker.h's block
 * comment on the 3-step split for the full ordering argument. */
void protocol_sock_recv_worker_invalidate(void)
{
	g_sock_recv_wk_cached = false;
}

/* Step 2 (worker.h): the actual byte copy, called OUTSIDE any critical
 * section.  Safe because step 1 already published g_sock_recv_wk_cached =
 * false -- nothing reads g_sock_recv_wk_reply while cached is false, so a
 * dispatch that preempts this copy cannot observe a half-written buffer.
 * Mirrors the defensive clamp step 3 repeats below: skip the memcpy (never
 * overrun the buffer) when the caller's own len exceeds it -- worker.c only
 * calls this when hw_rv == CC3501E_HW_OK, matching the original combined
 * function's own gate. */
void protocol_sock_recv_worker_copy(const uint8_t *data, size_t len)
{
	if (len > sizeof(g_sock_recv_wk_reply)) return;
	memcpy((void *)g_sock_recv_wk_reply, data, len);
}

/* Step 3 (worker.h): the small scalars only, inside worker.c's FINAL short
 * critical section (the same one that flips job.state to DONE/ERR) --
 * cached = true is written LAST, same release-ordering reason job.state is
 * written last in worker.c. */
void protocol_sock_recv_worker_publish(uint16_t handle, int hw_rv, size_t len)
{
	g_sock_recv_wk_seq    = g_sock_recv_job_seq;
	g_sock_recv_wk_handle = handle;
	if (hw_rv == CC3501E_HW_OK && len > sizeof(g_sock_recv_wk_reply)) {
		/* UNREACHABLE IN PRACTICE, stated precisely (host review, NIT 8 of
		 * the 1118c99 review): worker.c's SOCK_RECV data_cap bounds len to
		 * CC3501E_REPLY_DATA_MAX - sizeof(alp_cc3501e_sock_recv_resp_t), so
		 * the actual maximum len this function is ever called with is
		 * sizeof(alp_cc3501e_sock_recv_resp_t) + that same bound ==
		 * CC3501E_REPLY_DATA_MAX exactly (4093 B under CC3501E_WIRE_CRC=ON,
		 * 4095 B under =OFF) -- precisely sizeof(g_sock_recv_wk_reply), never
		 * larger.  Kept as a LOUD backstop (a deterministic error, not a
		 * silent decline-to-cache) for any FUTURE regression in that bound:
		 * if this branch ever DID fire, the handle it fires for is PINNED to
		 * RESP_ERR_NO_MEM on every subsequent same-key poll until the host
		 * issues SOCK_CLOSE on it (the only thing that invalidates this
		 * entry once cached, besides a genuinely different seq or handle) --
		 * not a silent hang, but a sticky error worth knowing about if it is
		 * ever observed on a real device. */
		g_sock_recv_wk_reply_len = 0u;
		g_sock_recv_wk_status    = ALP_CC3501E_RESP_ERR_NO_MEM;
		g_sock_recv_wk_cached    = true;
		return;
	}
	if (hw_rv == CC3501E_HW_OK) {
		g_sock_recv_wk_reply_len = (uint16_t)len;
		g_sock_recv_wk_status    = ALP_CC3501E_RESP_OK;
	} else {
		g_sock_recv_wk_reply_len = 0u;
		g_sock_recv_wk_status    = sock_worker_hw_err_to_resp(hw_rv);
	}
	g_sock_recv_wk_cached = true;
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
	/* DOES NOT invalidate the SOCK_RECV worker-fallback cache (an earlier
	 * version of this function did, unconditionally, on every OPEN) --
	 * BLOCKER, host review, 1118c99: a SOCK_OPEN for an UNRELATED new socket
	 * can land between a worker-routed recv (A, H) that CRC-failed on the
	 * wire and the host's own same-seq retry -- poll_by_repeat() does not
	 * serialise SOCK_RECV against every other opcode, only against itself --
	 * and clearing the cache here dropped (A, H)'s already-consumed reply
	 * out from under that retry, which then read FRESH bytes off the socket
	 * and lost the first block.  SOCK_OPEN never needs to invalidate this
	 * cache at all: it can only ever hand back a handle number that is
	 * currently free, and the ONLY thing that frees a host-visible handle
	 * number is handle_sock_close() below -- which already invalidates a
	 * matching cache entry itself, at the moment the number actually becomes
	 * reusable, confirmed against every hal/ti/cc3501e_hw_ti_sock.c
	 * lwip_close() site: the sock_open failure path, the accept-table
	 * overflow path, and the accept-event-ring-full path each close an fd
	 * that was NEVER handed back to the host as a valid handle. */
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
 * DISPATCH CONTEXT ONLY.  Unlike hal/ti/cc3501e_hw_ti_sock.c's own
 * `uncommitted` (which the worker task also writes, on arm/disarm -- see
 * that file's CONCURRENCY comment above rx_ring), these two are written
 * ONLY from this function, which only ever runs in the SPI dispatch
 * callback: no task-context writer exists, so no volatile and no cross-
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
	 * WORKER-FALLBACK REPLAY, FIXED BELOW: the fallback --
	 * handle_worker_routed_payload_reply() for a handle that is NOT the
	 * prefetched one (UDP sockets, and STREAM sockets accepted but never
	 * armed for prefetch) -- used to have the SAME CRC-rejected-reply
	 * data-loss hole this fast path closes: cc3501e_hw_sock_recv()'s
	 * lwip_recvfrom() has no way to re-deliver bytes a lost reply already
	 * consumed from the socket.  It is excluded from the generic retry latch
	 * for the same "stream-consuming, not idempotent" reason SOCK_RECV as a
	 * whole is (see protocol.c's retry_latch_applies()), and it is
	 * worker-routed rather than a single synchronous call, so it cannot reuse
	 * this fast path's lazy-commit shape (there is no ring to hold bytes back
	 * in) -- it gets its own completion-time reply cache instead, below. */
	const uint16_t handle = (uint16_t)((uint16_t)req[0] | ((uint16_t)req[1] << 8));

	/* seq 0 (ALP_CC3501E_REQ_SEQ_NONE) never claims a replay AGAINST A
	 * DIFFERENT completion, same reservation the generic retry latch uses --
	 * a host that does not assign one (every bare cc3501e_request() call
	 * site, or a pre-v8 host) would otherwise read every frame as "seq 0,
	 * same as last", i.e. always a replay of whatever was last served.
	 * Shared by the fast path below AND the worker-fallback cache further
	 * down -- both key off this same generic per-dispatch seq.  The RING
	 * fast path's own `replay` decision still excludes seq 0 explicitly
	 * (`seq != 0u && ...` below); the WORKER-FALLBACK cache further down
	 * cannot rely on its invalidate-on-mismatch check alone for the same
	 * safety -- a genuinely new seq-0 recv on the SAME handle is
	 * byte-identical to the one that just filled the cache, so that check
	 * has nothing to compare that differs (BLOCKER, host review of c354208).
	 * It instead serves a seq-0 hit ONCE and then forgets it, at the serve
	 * site further down -- see that comment for why. */
	const uint8_t seq = protocol_current_req_seq();

	/* WORKER-FALLBACK REPLAY CACHE invalidation (see the block comment above
	 * g_sock_recv_wk_cached) -- MUST run HERE, before the ring fast path
	 * below, not after it (BLOCKER, host review, 1118c99): the ring's own
	 * rc>=0 / rc==-2 arms RETURN EARLY, so a check placed after the ring
	 * block never runs at all for a request the ring itself serves.  With
	 * the header seq a SINGLE 5-bit counter shared by every opcode (pre-
	 * alp-sdk#2108, and still true for a host without that fix), that made
	 * this a cheaply reachable bug: a worker-fallback recv (A, H1) completes
	 * and the host collects it (OK, host advances past A); ~30 SUBSEQUENT
	 * recvs on an ARMED, different handle H2 -- served entirely by the ring,
	 * never reaching the code that used to sit below the ring block -- wrap
	 * the shared counter back to A; H1's NEXT recv, carrying that wrapped A,
	 * then matched g_sock_recv_wk_seq/g_sock_recv_wk_handle (still H1's OLD,
	 * uncleared entry) and was served H1's STALE bytes as OK with NO hw read
	 * at all -- roughly 1 in 31 H1 recvs in a mixed ring+worker-path
	 * download.  Running this check on EVERY SOCK_RECV dispatch, ring-served
	 * or not, closes that: H2's own dispatch now invalidates H1's entry the
	 * moment its (seq, handle) differs, regardless of which path serves H2's
	 * OWN reply.
	 *
	 * NOT CLOSED, and cannot be from here: this check only runs when a
	 * SOCK_RECV is dispatched.  A pre-#2108 host's shared counter can wrap
	 * back to a stale entry's seq via 30 intervening NON-recv opcodes
	 * instead (SOCK_OPEN, SOCK_SEND, ...) -- none of them reach this file's
	 * invalidation at all, so the entry survives untouched until a recv on
	 * the SAME handle happens to land on the same wrapped seq.  See the
	 * SECOND residual paragraph in the block comment above
	 * g_sock_recv_wk_cached; deliberately not fixed by invalidating on every
	 * other opcode (that trades this for BLOCKER 2's class of loss). */
	if (g_sock_recv_wk_cached && (seq != g_sock_recv_wk_seq || handle != g_sock_recv_wk_handle)) {
		g_sock_recv_wk_cached = false;
	}

	{
		const uint16_t max_len = (uint16_t)((uint16_t)req[2] | ((uint16_t)req[3] << 8));
		const size_t   hdr     = sizeof(alp_cc3501e_sock_recv_resp_t);
		if (reply_cap > hdr) {
			size_t room = reply_cap - hdr;
			if (max_len != 0u && room > (size_t)max_len) room = (size_t)max_len;

			const bool replay = (seq != 0u && seq == last_recv_seq && handle == last_recv_handle);

			uint16_t  got = 0u;
			const int rc =
			    cc3501e_hw_sock_recv_ring(handle, &reply_data[hdr], (uint16_t)room, replay, &got);

			/* Record THIS call's identity for the NEXT one to compare against
			 * -- UNCONDITIONALLY, on every SOCK_RECV dispatch, including
			 * rc == -1 (a handle this ring does not own: UDP, or a STREAM
			 * socket accepted but never armed for prefetch).
			 *
			 * An earlier version of this fix updated last_recv_seq/
			 * last_recv_handle only when rc != -1, reasoning that an
			 * unrelated handle's call must not clobber the prefetched
			 * handle's own retry window.  That reasoning does not hold
			 * against a SINGLE-CTX, SINGLE-CALLER host: cc3501e_core.c's
			 * sock_busy flag plus poll_by_repeat() issuing one call at a
			 * time mean a CRC-rejected reply's retry is always the very next
			 * SOCK_RECV, on the SAME handle, for THAT usage pattern.
			 * OVERCLAIMED here in an earlier version of this comment as "an
			 * h2 recv landing between h1's lost reply and h1's retry cannot
			 * happen" (host review, MAJOR 5 of the 1118c99 review) -- it can:
			 * sock_busy's own doc comment says outright it "catches
			 * same-call-stack reentrancy, not two truly concurrent callers",
			 * so two sockets driven from two DIFFERENT host threads/ctxs are
			 * not serialised against each other at all, and h2's recv can
			 * land in exactly that gap.  What the rc-gated version
			 * actually did was leave last_recv_seq/last_recv_handle FROZEN
			 * across every rc == -1 call, so up to 30 recvs on an unarmed
			 * handle could tick the host's shared per-dispatch seq counter
			 * through most of a cycle without ever updating last_*, and the
			 * 31st recv on the ARMED handle would then alias whatever seq
			 * that handle's own last serve happened to hold -- wrongly read
			 * as a replay, silently duplicating a block, still reported OK.
			 * Recording every call removes that: last_* now always reflects
			 * the most recent SOCK_RECV dispatch, armed or not, so an
			 * armed handle's seq can only collide with its OWN prior serve,
			 * not with however many unarmed calls ran in between.  See
			 * sock_recv_commit.h for the narrower residual recording every
			 * call still leaves (same-handle aliasing after exactly the
			 * right number of INTERVENING seq-allocating calls) and why
			 * only a dedicated host-side SOCK_RECV seq counter removes it
			 * entirely. */
			last_recv_seq    = seq;
			last_recv_handle = handle;

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
			if (rc == -3) {
				/* Ring drained AND the pump recorded a real lwIP failure on this
				 * fd (RST etc, see hal/cc3501e_hw.h's doc comment on this
				 * function and hal/ti/cc3501e_hw_ti_sock.c's peer_error field).
				 * Terminal, same reason -2 above must not fall through
				 * (#7) -- but unlike -2, nothing is ever coming, so answering
				 * BUSY would just spin the host to its poll_by_repeat timeout
				 * instead of reporting the failure.  Same status a genuine
				 * worker-path socket failure gets (sock_worker_hw_err_to_resp()
				 * above maps CC3501E_HW_ERR_IO to this too). */
				*reply_data_len = 0u;
				return ALP_CC3501E_RESP_ERR_RADIO;
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

	/* WORKER-FALLBACK REPLAY CACHE serve.  seq == 0 is INCLUDED here
	 * (MAJOR, host review, 1118c99): the invalidation check above already
	 * guarantees that a still-valid g_sock_recv_wk_cached means its
	 * (seq, handle) is EXACTLY this dispatch's own -- including when both
	 * are 0 -- since any mismatch, on ANY dispatch, would have already
	 * cleared it.  Serving it here is therefore exactly "collect the job
	 * THIS SAME handle's immediately-preceding dispatch just submitted", the
	 * identical plain submit/collect protocol every OTHER worker-routed
	 * opcode already gives a seq-0 host.  An earlier version of this gate
	 * additionally required `seq != 0u`, which sent EVERY seq-0 poll past
	 * this branch into the unconditional STALE-JOB GUARD below instead --
	 * discarding the just-finished job and resubmitting on EVERY poll, so a
	 * seq-0 host could never collect, only ever re-read.
	 *
	 * BLOCKER, host review of c354208: that fix alone let seq 0 replay a
	 * DIFFERENT completion.  seq 0 carries no identity, so a GENUINELY NEW
	 * seq-0 recv on the SAME handle is byte-identical to the one that just
	 * filled this cache -- the invalidation check above cannot tell them
	 * apart, since there is nothing for it to compare that differs.  Poll 1
	 * submits, poll 2 collects (correct), and a THIRD, logically new recv on
	 * the same handle then matched this same entry and was served recv 1's
	 * bytes again as OK, with no socket read -- the exact class of bug this
	 * cache exists to prevent, now on the read side.  Fixed by treating a
	 * seq-0 hit as SERVE ONCE, THEN FORGET: the cache is cleared the instant
	 * it is served for seq 0, below, so the NEXT seq-0 poll on this handle
	 * -- collect-again or a genuinely new recv, indistinguishable at this
	 * layer -- always falls through to the STALE-JOB GUARD and submits
	 * fresh.  A seq-carrying host (seq != 0) is unaffected: its NEXT
	 * same-key poll is a genuine retry of the SAME logical recv (the host
	 * only advances its own counter after collecting OK), so its entry may
	 * safely survive repeat serves the way SOCK_SEND's cache does. */
	if (g_sock_recv_wk_cached) {
		if (seq == 0u) {
			g_sock_recv_wk_cached = false; /* serve once, then forget -- see above */
		}
		/* This exact poll is what would once have reached
		 * handle_worker_routed_payload_reply()'s own WORKER_DONE/WORKER_ERR
		 * -> worker_reset() path -- now short-circuited by the cache hit
		 * above.  Reclaim the slot here instead of leaving that to chance,
		 * same reason handle_sock_send() does after its own cache hit.  The
		 * 1-byte key (handle's low byte) is the same approximation
		 * worker_discard_stale_terminal()/worker_reclaim_matching_terminal()
		 * already accept for SOCK_SEND's own seq -- sufficient to reclaim
		 * THIS job, since the full seq+handle match above already proved
		 * this reply is ours. */
		(void)worker_reclaim_matching_terminal(ALP_CC3501E_CMD_SOCK_RECV,
		                                       offsetof(alp_cc3501e_sock_recv_t, handle),
		                                       (uint8_t)(handle & 0xFFu));
		if (g_sock_recv_wk_status == ALP_CC3501E_RESP_OK) {
			if (reply_cap < (size_t)g_sock_recv_wk_reply_len) return ALP_CC3501E_RESP_ERR_NO_MEM;
			memcpy(reply_data, (const void *)g_sock_recv_wk_reply, g_sock_recv_wk_reply_len);
			*reply_data_len = g_sock_recv_wk_reply_len;
		}
		return g_sock_recv_wk_status;
	}

	/* STALE-JOB GUARD, same purpose as handle_sock_send()'s (protocol_sockets.c,
	 * same file) but a DIFFERENT mechanism: reaching here means this
	 * request's seq+handle is NOT cached -- either the very first
	 * worker-routed SOCK_RECV ever, or the host has moved on to a genuinely
	 * NEW recv while an OLDER recv's job may still be sitting in the worker
	 * slot, finished but never collected.  worker_poll() inside
	 * handle_worker_routed_payload_reply() below matches the job slot by
	 * OPCODE ALONE, so without this check that call would hand the OLD job's
	 * finished bytes back as if they were THIS request's answer -- and
	 * unlike SOCK_SEND, that OLD job can share THIS request's own handle (a
	 * job.req byte-compare cannot tell them apart; see
	 * worker_discard_stale_recv()'s doc comment, worker.h, for why), so this
	 * uses the unconditional-by-opcode variant instead: the cache-miss check
	 * just above it already proved, with FULL (seq, handle) precision, that
	 * whatever is sitting there is not this request's. */
	(void)worker_discard_stale_recv();

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
	{
		const uint16_t handle = (uint16_t)((uint16_t)req[0] | ((uint16_t)req[1] << 8));
		if (g_sock_recv_wk_cached && handle == g_sock_recv_wk_handle) {
			/* A closed handle's last recv reply must not outlive the socket:
			 * a LATER handle-number reuse must never be answered from it (see
			 * the cache's own block comment). */
			g_sock_recv_wk_cached = false;
		}
	}
	return handle_worker_routed_payload(ALP_CC3501E_CMD_SOCK_CLOSE, req, req_len, reply_data_len);
}
