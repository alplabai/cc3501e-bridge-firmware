/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * cc3501e-bridge firmware: META command-family handlers (PING,
 * GET_VERSION, GET_MAC, RESET) plus the proto-v2 bulk-data STREAM_WRITE
 * sink.  Split out of protocol.c (issue #461); protocol_dispatch() in
 * protocol.c still owns the single command-family switch that routes
 * here.
 *
 * This TU owns the META group ONLY.  The Wi-Fi, BLE, socket, GPIO,
 * camera, power, diagnostics and OTA families live in the sibling
 * protocol_<family>.c TUs, and protocol_dispatch() routes every one of
 * them.  An opcode that NO family implements is answered with
 * ALP_CC3501E_RESP_ERR_INVALID, per the protocol header's contract
 * ("firmware rejects opcodes it does not implement with
 * ALP_CC3501E_RESP_ERR_INVALID").
 *
 * Handlers that touch hardware (MAC read, self-reset) call the
 * cc3501e_hw_* shims declared in ../hal/cc3501e_hw.h.  The stub backend
 * (hal/cc3501e_hw_stub.c) keeps the protocol path exercisable on the
 * host with no TI SDK; the real bodies live in hal/ti/ and link against
 * TI's SimpleLink CC33xx SDK on the bench build.
 */

#include "protocol_internal.h"

/* Running total of bytes sunk by CMD_STREAM_WRITE (proto v2 bulk stream). */
static uint32_t g_stream_bytes;

/* --------------------------------------------------------------- */
/* META handlers (opcodes 0x00..0x0F)                                */
/* --------------------------------------------------------------- */

/* PING (0x00): liveness probe.  Empty request, empty reply data --
 * the bare RESP_OK status the transport prepends is the "firmware is
 * alive" signal the host's first post-boot handshake waits for. */
alp_cc3501e_resp_t handle_ping(const uint8_t *req,
                               size_t         req_len,
                               uint8_t       *reply_data,
                               size_t         reply_cap,
                               size_t        *reply_data_len)
{
	(void)req;
	(void)reply_data;
	(void)reply_cap;
	if (req_len != 0u) return ALP_CC3501E_RESP_ERR_INVALID;
	*reply_data_len = 0u;
	return ALP_CC3501E_RESP_OK;
}

/* The wire protocol this firmware tree IMPLEMENTS.  It is deliberately a
 * literal, not a copy of the header's value: its whole job is to disagree with
 * <alp/protocol/cc3501e.h> when the two drift apart.
 *
 * This firmware compiles the CANONICAL header out of an alp-sdk checkout rather
 * than mirroring it, so the wire contract cannot silently fork -- but only as
 * long as the checkout it reaches is the RIGHT one.  Before this tree was
 * extracted from alp-sdk the build guessed that path two directories up and was
 * always right; afterwards the same guess lands on whichever sibling checkout
 * happens to be there, on whatever branch it happens to be on.  That is not
 * hypothetical: on 2026-08-28 a protocol-4 header reached this protocol-5 build
 * and it failed on the first missing opcode.
 *
 * A missing opcode is the LUCKY failure -- it is loud.  A header that differs
 * only in a VALUE would compile cleanly and produce an image that answers
 * GET_VERSION with one number while behaving like another, which the host would
 * then trust.  This assert makes that case just as loud, at compile time.
 *
 * Bumping the protocol means changing BOTH this literal and the header, in the
 * same change.
 *
 * v7 (alp-sdk#1746 / issue #88): CMD_SOCK_SEND's request byte @3 (formerly
 * always-zero `reserved`, now `seq` in <alp/protocol/cc3501e.h>) is a retry
 * identity.  Root cause: the worker-routed socket opcodes had none, so a host
 * poll that re-sent the identical frame after the worker had already
 * completed the job was indistinguishable from a new request -- see
 * handle_sock_send() in protocol_sockets.c for the fix and
 * cc3501e_sock_send() in alp-sdk for the host half.  The bump is semantic,
 * not structural (the byte was already on the wire): it exists so THIS
 * firmware never reads an OLD host's always-zero byte 3 as a real seq, which
 * would let it serve a stale cached reply for a genuinely new send.
 *
 * v8 (issue #102): the SAME defect, generalised.  #89/v7 covered SOCK_SEND
 * only; every other worker-routed opcode (25 of them -- see protocol.c's
 * "generic worker-routed request-identity latch" section) still had no
 * request identity at all.  v8 repurposes bits 3..7 of the HEADER flags
 * byte (0x08..0x80, unused by every host through v7 -- protocol.c's
 * protocol_dispatch() used to `(void)flags` the whole thing) as a 5-bit
 * retry seq the host assigns once per logical command and re-sends
 * unchanged on a poll_by_repeat() retry.  Zero wire bytes added; the bump is
 * semantic, exactly like v7's, and for the identical reason: an OLD (v7)
 * host always sends flags bits 3..7 == 0, and a v8 firmware reading that as
 * "seq 0" WITHOUT the version gate would see every one of that host's
 * same-opcode frames as a matching retry and could serve a stale cached
 * reply for a genuinely new command -- the same silent-drop shape the v7
 * paragraph above describes, just triggered by the header instead of one
 * struct's reserved byte.  The GET_VERSION gate is what stops it, as before.
 *
 * PAIRED ALP-SDK CHANGE NOT YET LANDED.  This firmware repo and alp-sdk are
 * separate repos + separate PRs (ADR 0031); the host-side half -- assigning
 * the seq once per logical command in poll_by_repeat() and re-sending it
 * unchanged on retries, plus bumping ALP_CC3501E_PROTOCOL_VERSION 7 -> 8 --
 * is deliberately NOT part of this change (see the PR body).  Until it
 * lands, this firmware's v8 disagrees with alp-sdk's default-branch v7 on
 * purpose: the assert below is EXPECTED to fail CI's "protocol version
 * parity" job and this repo's own stub-build / gen_protocol_vectors.py
 * --check gates in the meantime -- the atomicity cost the README already
 * documents for the repo split, not a regression to chase here. */
#define CC3501E_FW_IMPLEMENTS_PROTOCOL 8

_Static_assert(ALP_CC3501E_PROTOCOL_VERSION == CC3501E_FW_IMPLEMENTS_PROTOCOL,
               "<alp/protocol/cc3501e.h> is not the protocol version this firmware "
               "implements -- the build is pointed at the wrong (or a stale) alp-sdk "
               "checkout; pass -AlpSdkRoot / ALP_SDK_ROOT at the right one");

/* GET_VERSION (0x01): wire-protocol compatibility gate.  Returns the
 * 16-bit ALP_CC3501E_PROTOCOL_VERSION (LE); the host refuses to talk
 * to a firmware whose value != its compile-time ALP_CC3501E_PROTOCOL_VERSION.
 *
 * This returns the PROTOCOL version, not the firmware RELEASE version
 * (firmware-version.txt), matching the host driver's compatibility
 * check `version != ALP_CC3501E_PROTOCOL_VERSION` and the chip-header
 * doc.  (The diag-struct comment in <alp/protocol/cc3501e.h> that says
 * GET_VERSION returns the release version is a documentation
 * discrepancy -- tracked in DESIGN.md; the release version is reported
 * separately via GET_DIAG_INFO.fw_version in v2 firmware.) */
alp_cc3501e_resp_t handle_get_version(const uint8_t *req,
                                      size_t         req_len,
                                      uint8_t       *reply_data,
                                      size_t         reply_cap,
                                      size_t        *reply_data_len)
{
	(void)req;
	if (req_len != 0u) return ALP_CC3501E_RESP_ERR_INVALID;
	if (reply_cap < 2u) return ALP_CC3501E_RESP_ERR_NO_MEM;
	put_le16(reply_data, (uint16_t)ALP_CC3501E_PROTOCOL_VERSION);
	*reply_data_len = 2u;
	return ALP_CC3501E_RESP_OK;
}

/* STREAM_WRITE (0x45, proto v2): bulk-data stream sink.  Accepts an opaque
 * payload (up to ALP_CC3501E_MAX_PAYLOAD-header bytes), counts + discards it,
 * and acks with empty reply data.  A back-to-back sequence is a FRAMED bulk
 * stream whose per-frame payload phase rides the host DMA path; because every
 * frame is acked the link never desyncs (unlike raw throwaway clocking).  The
 * running total is reported via GET_DIAG_INFO for throughput accounting.
 * Synchronous (no worker): a memory sink can't block. */
alp_cc3501e_resp_t handle_stream_write(const uint8_t *req,
                                       size_t         req_len,
                                       uint8_t       *reply_data,
                                       size_t         reply_cap,
                                       size_t        *reply_data_len)
{
	(void)req;
	(void)reply_data;
	(void)reply_cap;
	g_stream_bytes += (uint32_t)req_len;
	*reply_data_len = 0u;
	return ALP_CC3501E_RESP_OK;
}

/* GET_MAC (0x03): the CC3501E's factory MAC (6 bytes, big-endian wire
 * order as TI stores it).  Read from the radio subsystem via the HAL.
 *
 * Routed through the async worker (P0-4/P0-6): the real CC3501E_WIFI body
 * (Wlan_Get, preceded by a one-time Wi-Fi init) blocks for SECONDS, which
 * MUST NOT happen in the SPI ISR that runs this handler.  See
 * handle_worker_routed for the poll-by-repeat state machine. */
alp_cc3501e_resp_t handle_get_mac(const uint8_t *req,
                                  size_t         req_len,
                                  uint8_t       *reply_data,
                                  size_t         reply_cap,
                                  size_t        *reply_data_len)
{
	(void)req;
	return handle_worker_routed(
	    ALP_CC3501E_CMD_GET_MAC, 6u, req_len, reply_data, reply_cap, reply_data_len);
}

/* RESET (0x02): host-requested soft reset.  Reply OK now; the HAL
 * defers the actual reboot until after the reply has been clocked back
 * to the host (so the host sees the ack), then resets the chip -- which
 * the host detects as the firmware going quiet then re-PINGing alive.
 * On the stub backend the deferred reset is a no-op. */
alp_cc3501e_resp_t handle_reset(const uint8_t *req,
                                size_t         req_len,
                                uint8_t       *reply_data,
                                size_t         reply_cap,
                                size_t        *reply_data_len)
{
	(void)req;
	(void)reply_data;
	(void)reply_cap;
	if (req_len != 0u) return ALP_CC3501E_RESP_ERR_INVALID;
	cc3501e_hw_request_reset();
	*reply_data_len = 0u;
	return ALP_CC3501E_RESP_OK;
}
