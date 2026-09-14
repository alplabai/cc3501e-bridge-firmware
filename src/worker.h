/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * cc3501e-bridge firmware: async-job worker (the submit/poll seam).
 *
 * ===================== WHY THIS EXISTS (P0-4) =====================
 * The SPI-slave dispatch + reply build (protocol.c / transport_*.c) run
 * SYNCHRONOUS + FAST in the driver ISR/SwiP -- a blocking command's reply
 * must therefore ALWAYS be a fast read of in-memory state, never a direct
 * call into a blocking Wlan_* / psa_fwu_* body (those can take seconds).
 * This worker is that decoupling: the ISR SUBMITS a job and reads back its
 * cached result; the slow HAL body runs OUTSIDE the ISR, on the main
 * loop / bringup_task, via worker_run_pending() (the drain).  While the
 * drain blocks (Wi-Fi init can take seconds), the SPI ISR keeps answering
 * the host's poll re-issues from the shared state -- that is the point.
 *
 * Single in-flight job is sufficient for v0.2 (the META + Wi-Fi getters
 * the host polls one at a time).  The shared `state` lives in a `volatile`
 * struct so the ISR (poll) and the drain (run) observe each other's
 * writes; the few multi-field transitions are made under a short critical
 * section (see worker.c) so the ISR never reads a half-updated result.
 *
 * SILICON-FREE: this TU pulls in NO TI SDK.  The blocking bodies it calls
 * are the cc3501e_hw_* HAL shims (hal/cc3501e_hw.h); on the stub/native
 * backend those are NOTIMPL and the drain runs the job SYNCHRONOUSLY from
 * worker_submit() so host-side ztests stay deterministic.
 * =================================================================
 */

#ifndef CC3501E_BRIDGE_WORKER_H
#define CC3501E_BRIDGE_WORKER_H

#include <stddef.h>
#include <stdint.h>

#include "alp/protocol/cc3501e.h"

/* Lifecycle of the single in-flight job.  IDLE -> QUEUED (submit) ->
 * RUNNING -> DONE/ERR (drain), then back to IDLE once the host has polled
 * the result out (protocol.c resets it). */
enum worker_state {
	WORKER_IDLE = 0, /* no job; ready to accept a submit                 */
	WORKER_QUEUED,   /* submitted, awaiting the drain                    */
	WORKER_RUNNING,  /* drain is executing the blocking HAL body         */
	WORKER_DONE,     /* result ready (result[]/result_len valid)         */
	WORKER_ERR,      /* job failed; err carries the CC3501E_HW_* code    */
};

/* Initialise the worker to IDLE (called once from transport init / main). */
void worker_init(void);

/* Ground truth for "worker_execute() actually ran the HAL body" (issue
 * #102).  DEFINED and incremented in worker.c's worker_execute() -- the ONE
 * call site for every cc3501e_hw_* body, on either path (the ISR-
 * synchronous stub submit or the real drain thread) -- and read by
 * protocol_diag.c's DIAG_GET_STATS handler alongside protocol.c's
 * g_retry_latch_hits: a bench run that deliberately drops one reply and
 * lets the retry land tells "correctly de-duped" (this counter stays put,
 * the latch-hit counter increments) from "re-executed" (this counter
 * increments again) by comparing the two.  volatile: the real build's
 * increment (drain-thread context) and DIAG_GET_STATS's read (SPI-ISR
 * context) are different contexts, unlike the SPI-ISR-only counters in
 * protocol_internal.h. */
extern volatile uint32_t g_worker_execs;

/*
 * worker_submit -- queue a job for the drain.  @p cmd is the opcode the
 * job services (e.g. ALP_CC3501E_CMD_GET_MAC).  For argument-free getters
 * (GET_MAC / scan / ble); jobs that carry a request payload use
 * worker_submit_payload() instead.
 *
 * Returns 1 if the job was accepted (state IDLE -> QUEUED), 0 if the
 * worker was busy (a different/earlier job is still in flight).
 *
 * On the SILICON-FREE / stub backend (no CC3501E_WIFI) the job is run
 * SYNCHRONOUSLY here so the result is immediately available on the next
 * poll -- this keeps native_sim ztests deterministic with no main loop.
 */
int worker_submit(uint8_t cmd);

/*
 * worker_submit_payload -- like worker_submit, but for a job that carries a
 * request payload (WIFI_CONNECT_STA / WIFI_AP_START: the
 * alp_cc3501e_wifi_connect_t header + inline ssid + psk).  @p payload / @p len
 * are copied into a worker-owned buffer so the drain can run the blocking
 * association off the SPI ISR.  @p len must be <= ALP_CC3501E_MAX_PAYLOAD and
 * is expected to be validated by the caller.  Same IDLE->QUEUED accept / busy
 * semantics and synchronous-stub behaviour as worker_submit.
 */
int worker_submit_payload(uint8_t cmd, const uint8_t *payload, uint16_t len);

/*
 * worker_poll -- read back a completed job's result WITHOUT blocking.
 * Safe to call from the SPI ISR.  Only succeeds when a DONE/ERR job
 * matching @p cmd is present.
 *
 *   cmd      -- the opcode the caller expects the in-flight job to be.
 *   out      -- buffer for the result bytes (DONE only).
 *   out_cap  -- capacity of @p out.
 *   out_len  -- [out] bytes written to @p out.
 *   err      -- [out] the CC3501E_HW_* code (ERR only; 0 on DONE).
 *
 * Returns:
 *   WORKER_DONE  -- result copied into out/out_len (err = 0).  The caller
 *                   MUST then reset the worker (worker_reset) so the next
 *                   command can submit.
 *   WORKER_ERR   -- job failed; err set.  Caller resets too.
 *   WORKER_QUEUED/WORKER_RUNNING -- still in flight (caller replies BUSY).
 *   WORKER_IDLE  -- no job, or a job for a DIFFERENT cmd is in flight
 *                   (caller treats both as "submit/BUSY", see protocol.c).
 */
enum worker_state
worker_poll(uint8_t cmd, uint8_t *out, size_t out_cap, size_t *out_len, int8_t *err);

/* Return the worker to IDLE after the host has consumed a DONE/ERR result
 * (or to abandon a job).  Called from protocol.c once a GET_MAC poll has
 * copied the result into the reply. */
void worker_reset(void);

/*
 * worker_discard_stale_terminal -- narrow, SOCK_SEND-only escape hatch (see
 * protocol_sockets.c's handle_sock_send).  ATOMICALLY, in ONE critical
 * section: if a TERMINAL (DONE/ERR) job for @p cmd is sitting in the slot
 * AND the request byte it was originally submitted with at job.req[@p
 * req_off] differs from @p req_byte, resets the worker to IDLE (exactly
 * worker_reset()'s effect) and returns 1.  Otherwise touches nothing and
 * returns 0 -- including when the job is QUEUED/RUNNING, which is left
 * alone regardless of @p req_byte: there is no terminal result yet for it
 * to misclaim.
 *
 * The peek-then-reset shape this replaced ran as two separate critical
 * sections and was correct only because protocol_dispatch() runs the whole
 * peek+compare+reset+fall-through sequence inside one SPI callback, with
 * nothing else able to touch the job in between.  Folding it into one
 * critical section removes that fragile assumption: the compare and the
 * reset now happen atomically wrt the drain/ISR the same way every other
 * job-state transition in this file does, and a future caller (or a future
 * change to when this runs) cannot reopen the window by accident.
 *
 * This is deliberately NOT a general "job identity" mechanism: it does not
 * change worker_poll()'s opcode-only matching, add a seq field to the job,
 * or touch worker_submit/worker_submit_payload's signatures.  It exists
 * ONLY so a caller that already owns a stronger, opcode-specific identity
 * of its own (SOCK_SEND's per-send seq, offsetof(alp_cc3501e_sock_send_t,
 * seq)) can evict a stale terminal result it knows is not its own BEFORE
 * falling through to the generic worker-routed helper -- exactly the same
 * pattern protocol_spi.c's handle_spi1_transfer() already uses for
 * SPI1_TRANSFER, one level up (there the check runs against the job's OWN
 * already-collected reply data; here it runs against the job's stored
 * request, since a still-DONE-but-uncollected job has never had its reply
 * read out through this seam). */
int worker_discard_stale_terminal(uint8_t cmd, size_t req_off, uint8_t req_byte);

/*
 * worker_reclaim_matching_terminal -- the mirror image of
 * worker_discard_stale_terminal(): ATOMICALLY, in ONE critical section, if
 * a TERMINAL (DONE/ERR) job for @p cmd is sitting in the slot AND the
 * request byte it was originally submitted with at job.req[@p req_off]
 * MATCHES @p req_byte (rather than differs), resets the worker to IDLE and
 * returns 1.  Otherwise touches nothing and returns 0 -- including when
 * the job is QUEUED/RUNNING, left alone regardless of @p req_byte for the
 * same reason worker_discard_stale_terminal() leaves it alone: there is no
 * terminal result yet for anything to reclaim.
 *
 * Needed because protocol_sock_send_on_worker_complete() (below) fills
 * protocol_sockets.c's #88 cache BEFORE job.state flips (worker_execute()),
 * so a poll carrying the SAME seq as a job that has just gone terminal
 * finds the cache hit FIRST and never reaches
 * handle_worker_routed_payload_reply()'s own WORKER_DONE/WORKER_ERR ->
 * worker_reset() path any more -- that path used to be what freed the
 * slot.  Without this, a finished SOCK_SEND job would sit in the slot,
 * terminal, until some UNRELATED opcode's poll happened to orphan-discard
 * it.  handle_sock_send() calls this right after a cache hit to reclaim
 * the slot itself instead of leaving that to chance. */
int worker_reclaim_matching_terminal(uint8_t cmd, size_t req_off, uint8_t req_byte);

/*
 * worker_discard_stale_recv -- SOCK_RECV-only variant of the eviction half of
 * worker_discard_stale_terminal() above, WITHOUT a req-byte compare.
 * ATOMICALLY, in ONE critical section: if a TERMINAL (DONE/ERR) job for
 * ALP_CC3501E_CMD_SOCK_RECV is sitting in the slot, resets it to IDLE
 * (worker_reset()'s effect) and returns 1; otherwise (IDLE, QUEUED/RUNNING,
 * or a DIFFERENT opcode's job) touches nothing and returns 0.
 *
 * Why SOCK_RECV cannot reuse the byte-keyed helper above: that helper needs
 * a CALLER-OWNED identity byte inside job.req that reliably DIFFERS between
 * the stale job and the current request.  SOCK_SEND has one (its own
 * per-frame seq, alp_cc3501e_sock_send_t.seq).  alp_cc3501e_sock_recv_t is
 * just { handle | max_len } -- nothing in it changes between two DIFFERENT
 * logical recvs on the SAME handle, so a job.req[handle] byte-compare cannot
 * tell "a stale recv on this same handle" from "this handle's own retry":
 * both carry an identical handle byte.  The identity that DOES disambiguate
 * them is the request's generic header seq, which protocol_sockets.c's own
 * worker-fallback cache (g_sock_recv_wk_seq/g_sock_recv_wk_handle) tracks
 * OUTSIDE job.req -- and by the time protocol_sockets.c calls this function,
 * it has ALREADY compared this request's (seq, handle) against that cache
 * with FULL precision and found no match.  Any terminal SOCK_RECV job still
 * sitting in the slot at that point is, by construction, not this request's,
 * so no further per-byte check is needed before evicting it. */
int worker_discard_stale_recv(void);

/*
 * protocol_sock_send_on_worker_complete -- SOCK_SEND-ONLY completion hook.
 * DEFINED in protocol_sockets.c (owner of the #88 seq-keyed reply cache:
 * g_sock_send_cached / g_sock_send_seq / g_sock_send_status /
 * g_sock_send_reply), CALLED from HERE -- worker.c's worker_execute() --
 * the instant a SOCK_SEND job reaches a terminal state, inside the SAME
 * critical section that publishes job.state (see worker_execute()).  That
 * placement is load-bearing, not cosmetic: it guarantees the cache entry
 * for @p seq exists BEFORE any poll, on any context, for any opcode, can
 * first observe this job as terminal -- so even worker_poll()'s orphan-
 * discard arm (a DIFFERENT opcode's poll throwing this uncollected job
 * away) can never run ahead of the cache being filled.  Closes the
 * remaining duplicate-bytes gap worker_discard_stale_terminal() alone does
 * not: a SOCK_SEND result that a different opcode's poll discards before
 * the host's own same-seq re-issue arrives used to leave that re-issue with
 * nothing to collect AND nothing cached (the #88 cache was filled only on
 * collect), so it re-submitted and queued the same bytes twice.
 *
 * @p seq is the completed job's OWN request seq, read from job.req at the
 * wire offset alp_cc3501e_sock_send_t.seq occupies -- safe to read without
 * the critical section (job.req is stable from submit through this point;
 * nothing writes it again before the NEXT submit, which cannot happen
 * before this job is collected or discarded).
 *
 * @p hw_rv is the RAW cc3501e_hw_sock_send() return (CC3501E_HW_OK or a
 * CC3501E_HW_ERR_* code) -- worker.c stays wire-response-agnostic; mapping
 * a HW code to an ALP_CC3501E_RESP_* belongs to the protocol layer, same as
 * everywhere else worker.c's callers already do it.  @p data / @p len are
 * the 2-byte queued-count reply, valid on CC3501E_HW_OK only.
 *
 * BOTH OUTCOMES ARE CACHED -- a non-OK @p hw_rv is stored too, not skipped.
 * A same seq is BY DEFINITION the same logical send (the host assigns one
 * per send, cc3501e_sock_send()), so whatever the first execution produced
 * is the correct, final answer for every later poll of that seq, ERR
 * included: lwIP can fail AFTER queueing bytes (the SimpleLink SDK's
 * api_msg.c has a real path where tcp_write() succeeds and a LATER
 * tcp_output() still returns ERR_RTE, which the SDK's sockets.c surfaces as
 * a plain failed send() despite bytes already having been written, and
 * hal/ti/cc3501e_hw_ti_sock.c maps that to CC3501E_HW_ERR_IO) -- so treating
 * a non-OK result as "safe to just retry" would risk duplicating those
 * already-queued bytes, the exact class of bug this cache exists to
 * prevent.  See protocol_sockets.c's definition for the HW-code mapping and
 * the cache's own comment for the different-seq invalidation rule that
 * keeps a stale entry from outliving its seq. */
void protocol_sock_send_on_worker_complete(uint8_t seq, int hw_rv, const uint8_t *data, size_t len);

/*
 * protocol_sock_recv_note_submit -- SOCK_RECV-ONLY submit-edge hook.  DEFINED
 * in protocol_sockets.c, CALLED from protocol.c's handle_worker_routed_payload_reply()
 * on the WORKER_IDLE -> QUEUED submit edge, the SAME seam
 * handle_worker_routed_payload() already special-cases for WIFI_CONNECT_STA
 * (cc3501e_hw_wifi_mark_connecting()).
 *
 * Unlike SOCK_SEND, whose per-frame seq rides in the wire payload itself
 * (alp_cc3501e_sock_send_t.seq) and is therefore still sitting in job.req at
 * completion time, alp_cc3501e_sock_recv_t carries NO seq of its own (v9
 * protocol) -- the identity the worker-fallback replay cache needs is the
 * request's *generic* 5-bit header seq (protocol.c's s_current_req_seq /
 * protocol_current_req_seq(), the same field the ring fast path's own
 * lazy-commit replay check already uses).  That value is not part of job.req
 * and would not survive the submit -> completion gap on its own, so this
 * hook stashes it into protocol_sockets.c's own static right at submit time.
 *
 * @p seq is protocol.c's s_current_req_seq AT THE MOMENT OF SUBMIT.  Safe to
 * read without a critical section and safe against being overwritten before
 * this job resolves: worker_poll() matches an in-flight job by OPCODE ALONE,
 * so a second SOCK_RECV dispatch landing while this one is QUEUED/RUNNING
 * never reaches the IDLE submit edge again (it reads QUEUED/RUNNING and
 * answers BUSY) -- the single job slot guarantees at most one SOCK_RECV
 * submit is ever pending at a time. */
void protocol_sock_recv_note_submit(uint8_t seq);

/*
 * protocol_sock_recv_worker_invalidate / _copy / _publish -- SOCK_RECV-ONLY
 * completion hook, split into THREE steps (host review, MINOR 6 of the
 * 1118c99 review) instead of one atomic call like protocol_sock_send's
 * above.  SOCK_SEND's own cache copies at most 2 B, cheap enough to do
 * inside worker_execute()'s single publish critical section; SOCK_RECV's
 * copies up to CC3501E_REPLY_DATA_MAX (4093/4095 B) TWICE (once into
 * job.result, once into protocol_sockets.c's own cache) -- doing both
 * memcpys with interrupts masked (worker_critical_enter() is __disable_irq()
 * on real silicon) held the SPI-ISR-sensitive link's interrupts off for the
 * time of an ~8 KB copy, once per worker-routed recv.  worker.c's
 * worker_execute() now calls these three in order:
 *
 *   1. protocol_sock_recv_worker_invalidate() -- INSIDE a short critical
 *      section, BEFORE either copy: clears g_sock_recv_wk_cached so a
 *      dispatch landing in the gap below sees a clean cache MISS rather than
 *      a half-updated entry.
 *   2. protocol_sock_recv_worker_copy() -- OUTSIDE any critical section
 *      (interrupts stay enabled): the actual byte copy into the cache's own
 *      buffer.  Safe precisely because step 1 already published
 *      g_sock_recv_wk_cached = false: nothing reads g_sock_recv_wk_reply
 *      while cached is false, so a concurrent dispatch cannot observe a
 *      partially-written buffer.  (job.result's OWN copy is safe by the
 *      IDENTICAL argument using job.state instead: see worker.c.)
 *   3. protocol_sock_recv_worker_publish() -- INSIDE the SAME final critical
 *      section that flips job.state to DONE/ERR: the small scalar fields
 *      only (seq/handle/status/len, then cached = true last).
 *
 * A dispatch that lands between step 1 and step 3 sees g_sock_recv_wk_cached
 * false (cache miss) AND job.state still WORKER_RUNNING (not yet DONE/ERR):
 * it falls through to the generic worker-routed path, whose worker_poll()
 * reports the job QUEUED/RUNNING and answers BUSY -- never a torn read,
 * never data served from a half-copied buffer.
 *
 * TWO FACTS THIS SAFETY ARGUMENT RESTS ON (NIT, host review of c354208):
 *
 *   1. Dispatch (handle_sock_recv(), protocol_sockets.c) runs in the SPI
 *      transport's SWI/HWI context (transport_hw_ti_spi.c's ISR chain), and
 *      step 2's copy runs on the WORKER TASK -- different execution
 *      contexts on the SAME core, so they cannot literally run at the same
 *      instant; "lands between step 1 and step 3" above means a dispatch
 *      whose ISR preempts the task mid-copy (or is already pending when the
 *      task's critical section in step 1 or 3 exits), not a true SMP race.
 *   2. SOCK_CLOSE is itself worker-routed and answers BUSY while a SOCK_RECV
 *      job is QUEUED/RUNNING (worker_poll() matches the single job slot by
 *      opcode alone, so a close landing mid-recv reads the WRONG opcode
 *      in-flight and reports busy rather than running) -- so a close on the
 *      handle a RUNNING recv is filling this cache for cannot complete until
 *      a LATER dispatch, by which time the recv has already published and
 *      handle_sock_close() (protocol_sockets.c) re-invalidates the entry it
 *      just filled.  If SOCK_CLOSE ever became synchronous (answered from
 *      dispatch context without going through the worker), this ordering
 *      would break: a close could complete WHILE step 3 is still publishing,
 *      and this cache would resurrect a closed handle's entry the next time
 *      the closed handle NUMBER is reused.
 *
 * @p handle and @p max_len are both read out of job.req (SOCK_RECV's own
 * worker_execute() case already computes them at offsets 0 and 2) -- unlike
 * SOCK_SEND's seq, neither rides anywhere else, so neither needs a separate
 * submit-time capture.  @p hw_rv / @p data / @p len follow the same contract
 * as the SOCK_SEND hook: the raw HAL return code (mapped to an
 * ALP_CC3501E_RESP_* by protocol_sockets.c, not here) and the reply bytes,
 * valid on CC3501E_HW_OK only.  BOTH OUTCOMES ARE CACHED for the same
 * reason: a same seq+handle+max_len poll is a retry of the SAME logical
 * recv, GIVEN a host with a dedicated SOCK_RECV counter (alp-sdk#2108) -- a
 * pre-#2108 host sharing one counter across every opcode can alias two
 * DIFFERENT logical recvs onto the same (seq, handle) pair; see
 * protocol_sockets.c's own RESIDUAL comment above g_sock_recv_wk_cached.
 *
 * @p max_len JOINS THE KEY (host review of 9c989dc, MINOR): a same-seq,
 * same-handle retry whose max_len DIFFERS from the original is not the
 * same logical recv -- the SDK host always resends byte-identical
 * payloads, so this is a defensive, not a reachable-today, fix.  Without it
 * a same-seq+handle poll carrying a SMALLER max_len than the cached entry's
 * own (e.g. 0, where the original was nonzero) would still be served the
 * cached entry sized for the ORIGINAL, larger max_len -- firmware
 * correctness must not depend on what a caller happens to do. */
void protocol_sock_recv_worker_invalidate(void);
void protocol_sock_recv_worker_copy(const uint8_t *data, size_t len);
void protocol_sock_recv_worker_publish(uint16_t handle, uint16_t max_len, int hw_rv, size_t len);

/*
 * worker_run_pending -- THE DRAIN.  Runs OUTSIDE the ISR, from main()'s
 * loop / bringup_task.  If a job is QUEUED it transitions it to RUNNING,
 * calls the (possibly blocking) HAL body, stores the result, and sets
 * DONE/ERR.  No-op when nothing is queued.  On the stub backend the job
 * has already completed synchronously in worker_submit(), so this is a
 * cheap no-op there.
 */
void worker_run_pending(void);

#endif /* CC3501E_BRIDGE_WORKER_H */
