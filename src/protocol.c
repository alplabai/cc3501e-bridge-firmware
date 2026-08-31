/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * cc3501e-bridge firmware: top-level dispatch + transport-agnostic
 * framing.
 *
 * Both the SPI transport (default) and the SDIO transport (optional,
 * customer-selectable) feed validated request frames into
 * protocol_dispatch() here -- one command set, one set of reply codes,
 * transport-agnostic (the gd32-bridge SPI + I2C model).
 *
 * The per-command-family handlers (META, stream-write, GPIO proxy, SPI1
 * passthrough, camera enables, Wi-Fi, sockets, BLE, power policy,
 * diagnostics, OTA)
 * live in the sibling protocol_<family>.c TUs (split out of this file,
 * issue #461); this file keeps only:
 *
 *   - protocol_dispatch(): the single opcode -> handler switch (the
 *     command-family "table" the header/README describe).
 *   - protocol_build_reply(): the shared request/reply framing wrapper
 *     every transport calls.
 *   - the worker-routed state-machine helpers (handle_worker_routed and
 *     friends) that several families' handlers call to move a blocking
 *     HAL body off the SPI ISR -- declared in protocol_internal.h,
 *     defined here as the one shared TU.
 *   - the diagnostics frame counters (g_last_error / g_frames_ok /
 *     g_frames_err) protocol_build_reply() updates centrally; the
 *     GET_DIAG_INFO / DIAG_GET_STATS handlers in protocol_diag.c read
 *     them via protocol_internal.h's extern declarations.
 *
 * Scope: protocol_dispatch() below routes 52 opcodes -- META, Wi-Fi,
 * BLE, sockets, GPIO proxy, SPI1 host passthrough, camera enables, power
 * policy, diagnostics (including GET_PENDING_EVENTS) and OTA -- onto the
 * per-family handlers,
 * which reach TI's CC35xx Wi-Fi / NimBLE / lwIP / psa_fwu APIs through
 * the HAL backend.  Routed is not proven: BRINGUP_STATUS.md records what
 * is silicon-validated, and sockets do NOT connect (alp-sdk#1746).
 *
 * The rejection contract is unchanged and still load-bearing: an opcode
 * this firmware does not implement is answered with
 * ALP_CC3501E_RESP_ERR_INVALID, per the protocol header ("firmware
 * rejects opcodes it does not implement with
 * ALP_CC3501E_RESP_ERR_INVALID").
 */

#include <stdbool.h>
#include <string.h>
#include "protocol_internal.h"

/* Diagnostics state (firmware-side): last_error = the most recent non-OK
 * response emitted; the frame counters feed DIAG_GET_STATS.  All are
 * updated centrally here in protocol_build_reply(); protocol_diag.c only
 * reads them (see the extern declarations in protocol_internal.h). */
uint8_t  g_last_error = ALP_CC3501E_RESP_OK;
uint32_t g_frames_ok;
uint32_t g_frames_err;
uint32_t g_retry_latch_hits;

/* --------------------------------------------------------------- */
/* Generic worker-routed request-identity latch (issue #102)         */
/* --------------------------------------------------------------- */
/*
 * #89 gave CMD_SOCK_SEND a request seq (protocol_sockets.c's
 * g_sock_send_*) so a lost reply could not make poll_by_repeat()'s retry
 * look like a brand-new send.  That fix is scoped to one opcode; #102 is
 * every OTHER worker-routed opcode (25 of them) having no request identity
 * at all.  Rather than duplicate the cache per family, this lifts it into
 * the ONE seam every one of them already funnels through -- the three
 * helpers below -- so all 25 get it with zero per-opcode edits.  SOCK_SEND
 * keeps its own cache unchanged (folding it in is a follow-up, see the PR:
 * the bench that would validate the fold is down).  SPI1_TRANSFER also
 * keeps its own existing mechanism (protocol_spi.c) -- it hand-rolls its
 * own poll loop and never calls these helpers.  SOCK_RECV's ring fast path
 * (protocol_sockets.c) is untouched; only its worker-routed FALLBACK (rare,
 * ring miss) funnels through here, same as any other opcode.
 *
 * MECHANISM: bits 3..7 of the request header's flags byte (0x08..0x80) are
 * unused by every host up to protocol v7 -- protocol_dispatch() used to
 * discard the whole byte with `(void)flags`.  Repurposing them as a 5-bit
 * retry seq costs zero wire bytes, the same shape as v7's SOCK_SEND bump.
 * s_current_req_seq is extracted from flags ONCE per protocol_dispatch()
 * call (see there) and read by the three helpers below; a plain static is
 * safe because the transport is strict request/reply lockstep (one frame
 * decoded end-to-end per protocol_dispatch() call, see protocol.h's framing
 * doc) -- there is no re-entrant or cross-context caller to race with.
 */
/* The seq's shift/mask and the reserved value are WIRE facts, so they come from
 * <alp/protocol/cc3501e.h> -- the same header this firmware compiles against,
 * not a local copy of the same numbers.  A duplicate would be a second source of
 * truth for a field both sides have to agree on byte-for-byte, which is exactly
 * the drift the shared-header arrangement exists to prevent (see the repo README
 * on why the firmware #includes the header directly rather than mirroring it).
 *
 * Seq 0 is RESERVED to mean "this request carries no identity", so the usable
 * seq space is 1..ALP_CC3501E_REQ_SEQ_LAST.  A host that never assigns one
 * leaves the reserved flags bits zero, and without that reservation every such
 * frame would read as "seq 0, latch valid" -- i.e. a retry of the previous
 * same-opcode command, forever, with the different-seq invalidation unable to
 * fire. */
#define CC3501E_REQ_SEQ_SHIFT ALP_CC3501E_FLAG_REQ_SEQ_SHIFT
#define CC3501E_REQ_SEQ_MASK  ALP_CC3501E_REQ_SEQ_MASK
#define CC3501E_REQ_SEQ_NONE  ALP_CC3501E_REQ_SEQ_NONE

static uint8_t s_current_req_seq;

/*
 * The single most-recently-COLLECTED worker-routed outcome, cached so a
 * matching-(cmd,seq) retry -- a lost/misframed reply that made
 * poll_by_repeat() re-send the identical frame -- is served WITHOUT
 * resubmitting to the worker.  ~38 B of static RAM.
 *
 * FAILURE MODE, STATED PLAINLY -- this is a residual risk being accepted,
 * not a case the design eliminates.  seq wraps mod 32.  A genuinely NEW
 * command could be served a STALE cached reply if ALL of: same opcode,
 * same 5-bit seq as the cached entry, no intervening worker-routed
 * completion (ANY other worker-routed opcode's DONE/ERR overwrites this
 * one latch -- see retry_latch_store), AND no intervening same-cmd frame
 * carrying a DIFFERENT seq (retry_latch_serve's invalidation below closes
 * that specific gap).
 *
 * DO NOT read "~32 intervening logical commands, none worker-routed" as
 * rare.  The non-worker-routed set is the HIGH-FREQUENCY one:
 * GET_PENDING_EVENTS (the console workqueue polls it periodically),
 * WIFI_STATUS (polled throughout a connect), the SOCK_RECV ring fast path,
 * SPI1_TRANSFER, OTA_WRITE, PING, DIAG_*.  Thirty-two of those is SECONDS
 * OF ORDINARY IDLE, not a contrived sequence.
 *
 * The concrete case: `wifi ap-stop` latches at seq N -> ~32 event polls
 * wrap the 5-bit counter -> the user runs `ap-stop` again -> it is served
 * RESP_OK straight from this latch, and poll_by_repeat accepts it (AP_STOP
 * is not on the dead-phase-alias list) -> the AP never stopped and the host
 * believes it did.  On a part where ap-stop already wedges the bridge
 * (alp-sdk#1564), that is the bad direction.
 *
 * Reserving seq 0 (CC3501E_REQ_SEQ_NONE) removes the much larger
 * unset-seq class, but NOT this one.  Closing it properly needs a wider
 * seq field or a per-opcode counter; measuring the real hit rate is the
 * soak test named in the PR's bench procedure.
 */
static struct {
	bool    valid;
	uint8_t cmd;
	uint8_t seq;
	uint8_t status; /* alp_cc3501e_resp_t */
	uint8_t len;    /* <= sizeof(data); a bigger successful reply is never cached */
	uint8_t data[32];
} s_retry_latch;

void protocol_reset_retry_latch(void)
{
	s_retry_latch.valid = false;
}

/* Serve @p cmd's request from the latch when the request's seq (already
 * extracted into s_current_req_seq by protocol_dispatch) matches the cached
 * entry -- WITHOUT touching the worker.  @p reply_data / @p reply_cap may
 * be NULL / 0 for a helper that never produces reply data
 * (handle_worker_routed_payload): the cached len for THOSE opcodes is
 * always 0 (every store call site for that helper passes len 0), so
 * nothing is ever copied through a NULL pointer.
 *
 * Also implements the MANDATORY item-4 mitigation: a same-cmd request
 * carrying a DIFFERENT seq proves the host has moved on to a new logical
 * command, so the stale entry is dropped right here rather than waiting for
 * an unrelated opcode to eventually overwrite the single latch.  Mirrors
 * protocol_spi.c's SPI1_TRANSFER "different seq: drop it and start fresh"
 * rule. */

/* Opcodes the GENERIC latch must not touch, and why.
 *
 * CMD_SOCK_SEND carries its OWN identity: an 8-bit seq at req[3]
 * (alp_cc3501e_sock_send_t.seq, v7), served from g_sock_send_reply in
 * protocol_sockets.c.  That key is strictly STRONGER than this latch's 5-bit
 * header seq, and handle_sock_send falls through to
 * handle_worker_routed_payload_reply after correctly missing its own cache --
 * so a generic latch keyed only on (cmd, 5-bit seq) would OVERRIDE the
 * authoritative decision and serve the previous send's reply for a genuinely
 * different one.  On the wire that is RESP_OK plus the PREVIOUS send's byte
 * count, i.e. cc3501e_sock_send() reporting success for bytes never
 * transmitted -- issue #88 reintroduced on the data path, with a weaker key.
 * It needs no host bug: any send cadence whose intervening-command count is
 * congruent to 0 (mod 32) hits it, and a chunked transfer loop IS a fixed
 * cadence.
 *
 * CMD_SOCK_RECV is excluded for the same family of reason: its reply is
 * WK_SOCK_RECV_HDR (24) + recv_len, so a generic cap of 32 would latch it
 * exactly when recv_len <= 8 and drop it otherwise -- a size-dependent replay
 * policy on a STREAM-CONSUMING opcode, where replaying a reply and
 * re-consuming the ring are different things.  Leave recv to its own path.
 *
 * Both are exclusions from THIS mechanism only; neither loses the protection
 * it already has. */
static bool retry_latch_applies(alp_cc3501e_cmd_t cmd)
{
	switch (cmd) {
	case ALP_CC3501E_CMD_SOCK_SEND: /* stronger 8-bit identity of its own */
	case ALP_CC3501E_CMD_SOCK_RECV: /* stream-consuming; see above */
		return false;
	default:
		return true;
	}
}
static bool retry_latch_serve(alp_cc3501e_cmd_t   cmd,
                              uint8_t            *reply_data,
                              size_t              reply_cap,
                              size_t             *reply_data_len,
                              alp_cc3501e_resp_t *status)
{
	if (!retry_latch_applies(cmd)) {
		return false;
	}
	if (s_current_req_seq == CC3501E_REQ_SEQ_NONE) {
		return false; /* request carries no identity -- see retry_latch_store */
	}
	if (!s_retry_latch.valid || s_retry_latch.cmd != (uint8_t)cmd) {
		return false; /* nothing cached for this opcode */
	}
	if (s_retry_latch.seq != s_current_req_seq) {
		s_retry_latch.valid = false; /* different logical command -- drop it */
		return false;
	}
	if ((size_t)s_retry_latch.len > reply_cap) {
		/* Should not happen (the len<=32 cap on store, and each opcode always
		 * routes through the same one of the three helpers) -- fall through to
		 * a fresh submit rather than risk a buffer overrun. */
		return false;
	}
	if (s_retry_latch.len > 0u) {
		memcpy(reply_data, s_retry_latch.data, s_retry_latch.len);
	}
	*reply_data_len = s_retry_latch.len;
	*status         = (alp_cc3501e_resp_t)s_retry_latch.status;
	g_retry_latch_hits++;
	return true;
}

/* Cache a worker-routed collect-edge outcome (WORKER_DONE / WORKER_ERR in
 * the three helpers below).  The collecting frame IS a retry of the logical
 * command, so s_current_req_seq is already the right seq to store.
 *
 * Caps cached data at 32 bytes.  Recomputed maxima for every opcode this
 * latch actually covers: GET_MAC 6 B, WIFI_GET_RSSI 1 B, SOCK_OPEN 4 B,
 * SPI1_CONFIGURE 8 B (which DOES route here, via
 * handle_worker_routed_payload_reply in protocol_spi.c -- an earlier version
 * of this comment wrongly said it used its own family cache),
 * BLE_GATT_REGISTER <= 18 B (2 + 2*8, per ALP_CC3501E_BLE_GATT_MAX_CHARS 8),
 * every argless op 0 B.  The large-reply opcodes this excludes
 * (WIFI_SCAN_START / BLE_SCAN_START / BLE_GATT_READ) are idempotent reads
 * whose re-execution on a lost reply is slow, not harmful, so leaving them
 * uncached is the deliberate tradeoff.  SOCK_SEND and SOCK_RECV are excluded
 * outright -- see retry_latch_applies().
 *
 * A completion too big to cache still counts as "any completion overwrites
 * the single latch" -- the failure-mode analysis on s_retry_latch above
 * depends on that -- so it still drops whatever WAS cached rather than
 * leave a stale, unrelated entry behind. */
static void
retry_latch_store(alp_cc3501e_cmd_t cmd, alp_cc3501e_resp_t status, const uint8_t *data, size_t len)
{
	if (!retry_latch_applies(cmd)) {
		return; /* exempt opcode -- never store, so it can never be served */
	}
	if (s_current_req_seq == CC3501E_REQ_SEQ_NONE) {
		/* seq 0 means "this request carries no identity" -- never latch it.
		 * Without this, a host that does not assign a seq (every bare
		 * cc3501e_request() call site, and any pre-v8 host, whose reserved
		 * flags bits are zero) would send seq 0 on EVERY frame: each one
		 * would look like a retry of the last, and the different-seq
		 * invalidation could never fire because the seq never changes. */
		s_retry_latch.valid = false;
		return;
	}
	if (len > sizeof(s_retry_latch.data)) {
		s_retry_latch.valid = false;
		return;
	}
	s_retry_latch.cmd    = (uint8_t)cmd;
	s_retry_latch.seq    = s_current_req_seq;
	s_retry_latch.status = (uint8_t)status;
	s_retry_latch.len    = (uint8_t)len;
	if (len > 0u) {
		memcpy(s_retry_latch.data, data, len);
	}
	s_retry_latch.valid = true;
}

/* --------------------------------------------------------------- */
/* Worker-routed state-machine helpers                               */
/* --------------------------------------------------------------- */

/* Generic worker-routed handler for an ARGUMENT-FREE command whose HAL body
 * may BLOCK for seconds (a radio op) and therefore must NOT run in the SPI
 * ISR that dispatches the handler.  GET_MAC pioneered the seam (P0-4/P0-6);
 * WIFI_SCAN_START and WIFI_GET_RSSI share it -- the only per-command knobs
 * are the opcode and the reply-capacity floor.
 *
 * The command is POLL-BY-REPEAT: the host re-issues it (it maps
 * RESP_ERR_BUSY -> ALP_ERR_BUSY and retries) until the worker has the
 * result.  The state machine, identical to the original handle_get_mac:
 *
 *   - worker has DONE for @cmd -> copy the result bytes into the reply,
 *     reset the worker to IDLE, return RESP_OK.
 *   - worker has ERR for @cmd  -> reset, map the HAL code (NOTIMPL ->
 *     NOT_READY, INVAL -> INVALID, STATE -> the distinct ERR_STATE -- see
 *     <alp/protocol/cc3501e.h> -- else RADIO).  The stub backend lands
 *     here (NOT_READY).
 *   - worker IDLE              -> submit the job, return BUSY.
 *   - worker QUEUED/RUNNING, or busy with another cmd -> return BUSY.
 *
 * @min_cap is the reply-data floor (NO_MEM below it -- the worker's DONE
 * copy is capped at @reply_cap, so a too-small reply buffer must be caught
 * up front).  @req_len must be 0 (the routed ops carry no request payload).
 *
 * On the stub/native backend worker_submit() runs the job synchronously,
 * so the host needs exactly one extra poll (submit -> BUSY, re-issue ->
 * the cached NOT_READY/RESP_OK). */
alp_cc3501e_resp_t handle_worker_routed(alp_cc3501e_cmd_t cmd,
                                        size_t            min_cap,
                                        size_t            req_len,
                                        uint8_t          *reply_data,
                                        size_t            reply_cap,
                                        size_t           *reply_data_len)
{
	if (req_len != 0u) return ALP_CC3501E_RESP_ERR_INVALID;
	if (reply_cap < min_cap) return ALP_CC3501E_RESP_ERR_NO_MEM;

	alp_cc3501e_resp_t cached_status = ALP_CC3501E_RESP_OK;
	if (retry_latch_serve(cmd, reply_data, reply_cap, reply_data_len, &cached_status)) {
		return cached_status;
	}

	size_t                  n   = 0u;
	int8_t                  err = 0;
	const enum worker_state st  = worker_poll((uint8_t)cmd, reply_data, reply_cap, &n, &err);

	switch (st) {
	case WORKER_DONE:
		worker_reset();
		*reply_data_len = n; /* command-specific payload (6 for GET_MAC, etc.) */
		retry_latch_store(cmd, ALP_CC3501E_RESP_OK, reply_data, n);
		return ALP_CC3501E_RESP_OK;
	case WORKER_ERR:
		worker_reset();
		if (err == CC3501E_HW_ERR_NOTIMPL) {
			retry_latch_store(cmd, ALP_CC3501E_RESP_ERR_NOT_READY, NULL, 0u);
			return ALP_CC3501E_RESP_ERR_NOT_READY;
		}
		if (err == CC3501E_HW_ERR_INVAL) {
			retry_latch_store(cmd, ALP_CC3501E_RESP_ERR_INVALID, NULL, 0u);
			return ALP_CC3501E_RESP_ERR_INVALID;
		}
		/* CC3501E_HW_ERR_STATE: today only cc3501e_hw_ble_gatt_register's NimBLE
		 * ble_gatts_mutable() reject (BLE_HS_EBUSY) -- a deterministic, terminal
		 * refusal, not the generic radio/protocol RESP_ERR_RADIO bucket below. */
		if (err == CC3501E_HW_ERR_STATE) {
			retry_latch_store(cmd, ALP_CC3501E_RESP_ERR_STATE, NULL, 0u);
			return ALP_CC3501E_RESP_ERR_STATE;
		}
		retry_latch_store(cmd, ALP_CC3501E_RESP_ERR_RADIO, NULL, 0u);
		return ALP_CC3501E_RESP_ERR_RADIO;
	case WORKER_IDLE:
		/* No job in flight: queue one and ask the host to re-issue. */
		(void)worker_submit((uint8_t)cmd);
		return ALP_CC3501E_RESP_ERR_BUSY;
	default: /* WORKER_QUEUED / WORKER_RUNNING (incl. another cmd in flight) */
		return ALP_CC3501E_RESP_ERR_BUSY;
	}
}

/* As handle_worker_routed, but for a job that CARRIES a request payload
 * (WIFI_CONNECT_STA / WIFI_AP_START).  The caller must have validated @p req /
 * @p req_len already; on the IDLE edge the payload is stashed via
 * worker_submit_payload so the drain runs the blocking association off the SPI
 * ISR.  These ops carry no reply payload (min_cap 0). */
alp_cc3501e_resp_t handle_worker_routed_payload(alp_cc3501e_cmd_t cmd,
                                                const uint8_t    *req,
                                                size_t            req_len,
                                                size_t           *reply_data_len)
{
	*reply_data_len = 0u;

	alp_cc3501e_resp_t cached_status = ALP_CC3501E_RESP_OK;
	if (retry_latch_serve(cmd, NULL, 0u, reply_data_len, &cached_status)) {
		return cached_status;
	}

	size_t                  n   = 0u;
	int8_t                  err = 0;
	const enum worker_state st  = worker_poll((uint8_t)cmd, NULL, 0u, &n, &err);

	switch (st) {
	case WORKER_DONE:
		worker_reset();
		retry_latch_store(cmd, ALP_CC3501E_RESP_OK, NULL, 0u);
		return ALP_CC3501E_RESP_OK;
	case WORKER_ERR:
		worker_reset();
		if (err == CC3501E_HW_ERR_NOTIMPL) {
			retry_latch_store(cmd, ALP_CC3501E_RESP_ERR_NOT_READY, NULL, 0u);
			return ALP_CC3501E_RESP_ERR_NOT_READY;
		}
		if (err == CC3501E_HW_ERR_INVAL) {
			retry_latch_store(cmd, ALP_CC3501E_RESP_ERR_INVALID, NULL, 0u);
			return ALP_CC3501E_RESP_ERR_INVALID;
		}
		/* CC3501E_HW_ERR_STATE: today only cc3501e_hw_ble_gatt_register's NimBLE
		 * ble_gatts_mutable() reject (BLE_HS_EBUSY) -- a deterministic, terminal
		 * refusal, not the generic radio/protocol RESP_ERR_RADIO bucket below. */
		if (err == CC3501E_HW_ERR_STATE) {
			retry_latch_store(cmd, ALP_CC3501E_RESP_ERR_STATE, NULL, 0u);
			return ALP_CC3501E_RESP_ERR_STATE;
		}
		retry_latch_store(cmd, ALP_CC3501E_RESP_ERR_RADIO, NULL, 0u);
		return ALP_CC3501E_RESP_ERR_RADIO;
	case WORKER_IDLE:
		/* No job in flight: queue THIS one (with its payload) + return BUSY (the
		 * submit ack).  For a STA connect, latch the status to CONNECTING NOW --
		 * synchronously, before the drain runs the seconds-long body -- so a host
		 * CMD_WIFI_STATUS poll during the brief queued window never reads a stale
		 * CONNECTED/FAILED from a previous attempt.  The body then publishes the
		 * terminal CONNECTED/FAILED; the host collects it via CMD_WIFI_STATUS (no
		 * poll-by-repeat on the connect opcode). */
		if (cmd == ALP_CC3501E_CMD_WIFI_CONNECT_STA) {
			cc3501e_hw_wifi_mark_connecting();
		}
		(void)worker_submit_payload((uint8_t)cmd, req, (uint16_t)req_len);
		return ALP_CC3501E_RESP_ERR_BUSY;
	default: /* QUEUED / RUNNING (incl. another cmd in flight) */
		return ALP_CC3501E_RESP_ERR_BUSY;
	}
}

/* As handle_worker_routed_payload, but for a payload-carrying job that ALSO
 * returns reply DATA (BLE_GATT_READ: the attribute handle goes up in the
 * payload, the attribute value comes back).  The worker copies its DONE bytes
 * straight into @p reply_data -- the SAME reply path handle_worker_routed uses
 * for the record-returning scans -- so a single job round-trips a value off the
 * SPI ISR.  Poll-by-repeat, @min_cap is the reply-data floor.  The caller must
 * have validated @p req / @p req_len already. */
alp_cc3501e_resp_t handle_worker_routed_payload_reply(alp_cc3501e_cmd_t cmd,
                                                      const uint8_t    *req,
                                                      size_t            req_len,
                                                      size_t            min_cap,
                                                      uint8_t          *reply_data,
                                                      size_t            reply_cap,
                                                      size_t           *reply_data_len)
{
	*reply_data_len = 0u;
	if (reply_cap < min_cap) return ALP_CC3501E_RESP_ERR_NO_MEM;

	alp_cc3501e_resp_t cached_status = ALP_CC3501E_RESP_OK;
	if (retry_latch_serve(cmd, reply_data, reply_cap, reply_data_len, &cached_status)) {
		return cached_status;
	}

	size_t                  n   = 0u;
	int8_t                  err = 0;
	const enum worker_state st  = worker_poll((uint8_t)cmd, reply_data, reply_cap, &n, &err);

	switch (st) {
	case WORKER_DONE:
		worker_reset();
		*reply_data_len = n; /* the worker copied the attribute value into reply_data */
		retry_latch_store(cmd, ALP_CC3501E_RESP_OK, reply_data, n);
		return ALP_CC3501E_RESP_OK;
	case WORKER_ERR:
		worker_reset();
		if (err == CC3501E_HW_ERR_NOTIMPL) {
			retry_latch_store(cmd, ALP_CC3501E_RESP_ERR_NOT_READY, NULL, 0u);
			return ALP_CC3501E_RESP_ERR_NOT_READY;
		}
		if (err == CC3501E_HW_ERR_INVAL) {
			retry_latch_store(cmd, ALP_CC3501E_RESP_ERR_INVALID, NULL, 0u);
			return ALP_CC3501E_RESP_ERR_INVALID;
		}
		/* CC3501E_HW_ERR_STATE: today only cc3501e_hw_ble_gatt_register's NimBLE
		 * ble_gatts_mutable() reject (BLE_HS_EBUSY) -- a deterministic, terminal
		 * refusal, not the generic radio/protocol RESP_ERR_RADIO bucket below. */
		if (err == CC3501E_HW_ERR_STATE) {
			retry_latch_store(cmd, ALP_CC3501E_RESP_ERR_STATE, NULL, 0u);
			return ALP_CC3501E_RESP_ERR_STATE;
		}
		retry_latch_store(cmd, ALP_CC3501E_RESP_ERR_RADIO, NULL, 0u);
		return ALP_CC3501E_RESP_ERR_RADIO;
	case WORKER_IDLE:
		/* No job in flight: queue THIS one (with its payload) + return BUSY. */
		(void)worker_submit_payload((uint8_t)cmd, req, (uint16_t)req_len);
		return ALP_CC3501E_RESP_ERR_BUSY;
	default: /* QUEUED / RUNNING (incl. another cmd in flight) */
		return ALP_CC3501E_RESP_ERR_BUSY;
	}
}

/* --------------------------------------------------------------- */
/* Dispatch                                                          */
/* --------------------------------------------------------------- */

typedef alp_cc3501e_resp_t (*cmd_handler_t)(const uint8_t *, size_t, uint8_t *, size_t, size_t *);

/* Sparse switch on opcode -- keeps the table small without losing the
 * single-handler-table property.  A new feature group slots in here as
 * its HAL body lands.  Each case routes to the owning family's
 * protocol_<family>.c handler (declared in protocol_internal.h). */
alp_cc3501e_resp_t protocol_dispatch(uint8_t        cmd,
                                     uint8_t        flags,
                                     const uint8_t *req,
                                     size_t         req_len,
                                     uint8_t       *reply_data,
                                     size_t         reply_cap,
                                     size_t        *reply_data_len)
{
	/* Bits 3..7 of flags carry the v8 request-retry seq (issue #102); bits
	 * 0..2 (RESP_REQUIRED / ASYNC_EVENT / CONTINUATION) still don't alter
	 * dispatch.  Stashed here, once, for the three worker-routed helpers in
	 * this file to read -- see the "generic worker-routed request-identity
	 * latch" section above for why one seam suffices for every opcode. */
	s_current_req_seq = (uint8_t)((flags >> CC3501E_REQ_SEQ_SHIFT) & CC3501E_REQ_SEQ_MASK);
	*reply_data_len   = 0u;

	cmd_handler_t h = NULL;
	switch (cmd) {
	case ALP_CC3501E_CMD_PING:
		h = handle_ping;
		break;
	case ALP_CC3501E_CMD_GET_VERSION:
		h = handle_get_version;
		break;
	case ALP_CC3501E_CMD_GET_MAC:
		h = handle_get_mac;
		break;
	case ALP_CC3501E_CMD_RESET:
		h = handle_reset;
		break;
	case ALP_CC3501E_CMD_GET_DIAG_INFO:
		h = handle_get_diag_info;
		break;
	case ALP_CC3501E_CMD_GET_PENDING_EVENTS:
		h = handle_get_pending_events;
		break;
	/* OTA firmware update (v0.2). */
	case ALP_CC3501E_CMD_OTA_BEGIN:
		h = handle_ota_begin;
		break;
	case ALP_CC3501E_CMD_OTA_WRITE:
		h = handle_ota_write;
		break;
	case ALP_CC3501E_CMD_OTA_FINISH:
		h = handle_ota_finish;
		break;
	case ALP_CC3501E_CMD_OTA_ABORT:
		h = handle_ota_abort;
		break;
	case ALP_CC3501E_CMD_OTA_STATUS:
		h = handle_ota_status;
		break;
	case ALP_CC3501E_CMD_OTA_PROMOTE:
		h = handle_ota_promote;
		break;
	case ALP_CC3501E_CMD_OTA_UPDATE_MODE:
		h = handle_ota_update_mode;
		break;
	case ALP_CC3501E_CMD_STREAM_WRITE:
		h = handle_stream_write;
		break;
	/* GPIO proxy + camera enables (v0.4). */
	case ALP_CC3501E_CMD_GPIO_CONFIGURE:
		h = handle_gpio_configure;
		break;
	case ALP_CC3501E_CMD_GPIO_WRITE:
		h = handle_gpio_write;
		break;
	case ALP_CC3501E_CMD_GPIO_READ:
		h = handle_gpio_read;
		break;
	case ALP_CC3501E_CMD_GPIO_SET_INTERRUPT:
		h = handle_gpio_set_interrupt;
		break;
	/* SPI1 host passthrough (v0.6).  The E1M connector's SPI1 lands on the
	 * CC3501E, not the Alif, so these relay the host's master transfers.
	 * NOT the inter-chip bridge -- that is SPI0. */
	case ALP_CC3501E_CMD_SPI1_CONFIGURE:
		h = handle_spi1_configure;
		break;
	case ALP_CC3501E_CMD_SPI1_TRANSFER:
		h = handle_spi1_transfer;
		break;
	case ALP_CC3501E_CMD_SPI1_RELEASE:
		h = handle_spi1_release;
		break;
	case ALP_CC3501E_CMD_CAM_ENABLE:
		h = handle_cam_enable;
		break;
	case ALP_CC3501E_CMD_CAM_DISABLE:
		h = handle_cam_disable;
		break;
	/* Wi-Fi (v0.2). */
	case ALP_CC3501E_CMD_WIFI_SCAN_START:
		h = handle_wifi_scan_start;
		break;
	case ALP_CC3501E_CMD_WIFI_SCAN_STOP:
		h = handle_wifi_scan_stop;
		break;
	case ALP_CC3501E_CMD_WIFI_CONNECT_STA:
		h = handle_wifi_connect_sta;
		break;
	case ALP_CC3501E_CMD_WIFI_DISCONNECT:
		h = handle_wifi_disconnect;
		break;
	case ALP_CC3501E_CMD_WIFI_AP_START:
		h = handle_wifi_ap_start;
		break;
	case ALP_CC3501E_CMD_WIFI_AP_STOP:
		h = handle_wifi_ap_stop;
		break;
	case ALP_CC3501E_CMD_WIFI_GET_RSSI:
		h = handle_wifi_get_rssi;
		break;
	case ALP_CC3501E_CMD_WIFI_GET_IP:
		h = handle_wifi_get_ip;
		break;
	case ALP_CC3501E_CMD_WIFI_STATUS:
		h = handle_wifi_status;
		break;
	/* TCP/UDP sockets (v0.5). */
	case ALP_CC3501E_CMD_SOCK_OPEN:
		h = handle_sock_open;
		break;
	case ALP_CC3501E_CMD_SOCK_CONNECT:
		h = handle_sock_connect;
		break;
	case ALP_CC3501E_CMD_SOCK_SEND:
		h = handle_sock_send;
		break;
	case ALP_CC3501E_CMD_SOCK_RECV:
		h = handle_sock_recv;
		break;
	case ALP_CC3501E_CMD_SOCK_CLOSE:
		h = handle_sock_close;
		break;
	/* BLE 5.4 (v0.3). */
	case ALP_CC3501E_CMD_BLE_ENABLE:
		h = handle_ble_enable;
		break;
	case ALP_CC3501E_CMD_BLE_DISABLE:
		h = handle_ble_disable;
		break;
	case ALP_CC3501E_CMD_BLE_ADV_START:
		h = handle_ble_adv_start;
		break;
	case ALP_CC3501E_CMD_BLE_ADV_STOP:
		h = handle_ble_adv_stop;
		break;
	case ALP_CC3501E_CMD_BLE_SCAN_START:
		h = handle_ble_scan_start;
		break;
	case ALP_CC3501E_CMD_BLE_SCAN_STOP:
		h = handle_ble_scan_stop;
		break;
	case ALP_CC3501E_CMD_BLE_CONNECT:
		h = handle_ble_connect;
		break;
	case ALP_CC3501E_CMD_BLE_DISCONNECT:
		h = handle_ble_disconnect;
		break;
	case ALP_CC3501E_CMD_BLE_GATT_REGISTER:
		h = handle_ble_gatt_register;
		break;
	case ALP_CC3501E_CMD_BLE_GATT_NOTIFY:
		h = handle_ble_gatt_notify;
		break;
	case ALP_CC3501E_CMD_BLE_GATT_READ:
		h = handle_ble_gatt_read;
		break;
	case ALP_CC3501E_CMD_BLE_GATT_WRITE:
		h = handle_ble_gatt_write;
		break;
	/* Power policy + diagnostics (configurability). */
	case ALP_CC3501E_CMD_POWER_POLICY:
		h = handle_power_policy;
		break;
	case ALP_CC3501E_CMD_DIAG_GET_STATS:
		h = handle_diag_get_stats;
		break;
	case ALP_CC3501E_CMD_DIAG_LOG_LEVEL:
		h = handle_diag_log_level;
		break;
	default:
		/* Unknown, or a known opcode whose firmware body has not landed
		 * yet.  The header's contract is RESP_ERR_INVALID. */
		return ALP_CC3501E_RESP_ERR_INVALID;
	}
	return h(req, req_len, reply_data, reply_cap, reply_data_len);
}

/* --------------------------------------------------------------- */
/* Transport-agnostic framing                                        */
/* --------------------------------------------------------------- */

size_t protocol_build_reply(const uint8_t *req_frame,
                            size_t         req_len,
                            uint8_t       *reply_frame,
                            size_t         reply_cap)
{
	/* The caller guarantees a full-size reply buffer; guard anyway. */
	if (reply_cap < CC3501E_REPLY_DATA_OFF) {
		return 0u;
	}

	const uint8_t      cmd_echo = (req_len >= 1u) ? req_frame[0] : 0u;
	alp_cc3501e_resp_t status   = ALP_CC3501E_RESP_ERR_PROTOCOL;
	size_t             data_len = 0u;

	if (req_len >= (size_t)ALP_CC3501E_HEADER_BYTES) {
		const uint8_t  flags       = req_frame[1];
		const uint16_t payload_len = (uint16_t)req_frame[2] | ((uint16_t)req_frame[3] << 8);

		/* Captured byte count must match the declared payload exactly. */
		if ((size_t)ALP_CC3501E_HEADER_BYTES + (size_t)payload_len == req_len) {
			const uint8_t *req = (payload_len > 0u) ? &req_frame[ALP_CC3501E_HEADER_BYTES] : NULL;
			status             = protocol_dispatch(cmd_echo,
			                                       flags,
			                                       req,
			                                       payload_len,
			                                       &reply_frame[CC3501E_REPLY_DATA_OFF],
			                                       reply_cap - CC3501E_REPLY_DATA_OFF,
			                                       &data_len);
		}
	}

	/* Defence-in-depth: a handler must never report more data than the reply
	 * buffer holds, but if one did, (uint16_t)(1u + data_len) would TRUNCATE the
	 * length silently and frame a corrupt reply.  Clamp + fail closed instead. */
	const size_t reply_data_cap =
	    (reply_cap > CC3501E_REPLY_DATA_OFF) ? (reply_cap - CC3501E_REPLY_DATA_OFF) : 0u;
	if (data_len > reply_data_cap) {
		data_len = 0u;
		status   = ALP_CC3501E_RESP_ERR_NO_MEM;
	}

	/* Diagnostics bookkeeping: count OK vs error replies and latch the last
	 * non-OK status, for GET_DIAG_INFO / DIAG_GET_STATS.  Runs AFTER the clamp so
	 * a clamped NO_MEM is counted as the error it is. */
	if (status == ALP_CC3501E_RESP_OK) {
		g_frames_ok++;
	} else {
		g_frames_err++;
		g_last_error = (uint8_t)status;
	}

	/* Frame the reply: [cmd | flags=0 | payload_len(LE) | status | data | pad].
	 * flags = 0 -> solicited reply.  NOTHING in this firmware ever sets
	 * ALP_CC3501E_FLAG_ASYNC_EVENT: async events are never emitted as frames of
	 * their own, they are queued in the event ring and handed back inside an
	 * ordinary GET_PENDING_EVENTS reply.  payload = status(1) + data + pad. */
	/* Pad the reply payload up to a multiple of CC3501E_REPLY_PAD so the HOST's
	 * DMA can move it as ONE burst-aligned chunk.
	 *
	 * The host's DW SSI raises its DMA request on a FIFO watermark, so its driver
	 * has to shrink the burst until it divides the transfer exactly -- an ODD
	 * length collapses the burst to 1, i.e. one DMA transaction per byte.  Even
	 * with that handled by splitting off a tail chunk, an unaligned length still
	 * costs a SECOND PL330 setup + semaphore round trip per transfer, which is
	 * what kept DMA slower than PIO on this bridge.  Reply payload is
	 * 1 + data_len, so without padding roughly half of all frames are odd.
	 *
	 * The declared payload_len INCLUDES the pad, so payload_len is NOT
	 * status + data.  That is safe only for a reply payload that is
	 * SELF-DELIMITING.  SOCK_RECV is: data_len sits inside
	 * alp_cc3501e_sock_recv_resp_t.  GET_PENDING_EVENTS was NOT -- its reply data
	 * is a bare packed entry list -- so an empty ring's 7 zero pad bytes were
	 * walked as three "opcode 0x00, len 0" entries, ~5.8 phantom events per
	 * second (alp-sdk#1740; the host walk now stops at a zero opcode).  Any NEW
	 * variable-length reply payload must carry its own count or terminator.
	 * Costs at most CC3501E_REPLY_PAD-1 bytes of wire out of ~1.7 KB. */
	uint16_t reply_payload = (uint16_t)(1u + data_len);
	{
		const uint16_t rem = (uint16_t)(reply_payload % CC3501E_REPLY_PAD);

		if (rem != 0u) {
			const uint16_t pad = (uint16_t)(CC3501E_REPLY_PAD - rem);

			/* Only pad if it still fits the caller's frame buffer. */
			if ((size_t)ALP_CC3501E_HEADER_BYTES + reply_payload + pad <= reply_cap) {
				memset(&reply_frame[CC3501E_REPLY_STATUS_OFF + reply_payload], 0, pad);
				reply_payload = (uint16_t)(reply_payload + pad);
			}
		}
	}
	reply_frame[0]                        = cmd_echo;
	reply_frame[1]                        = 0u;
	reply_frame[2]                        = (uint8_t)(reply_payload & 0xFFu);
	reply_frame[3]                        = (uint8_t)((reply_payload >> 8) & 0xFFu);
	reply_frame[CC3501E_REPLY_STATUS_OFF] = (uint8_t)status;
	return (size_t)ALP_CC3501E_HEADER_BYTES + reply_payload;
}
