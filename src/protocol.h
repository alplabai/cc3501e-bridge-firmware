/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * cc3501e-bridge firmware: internal protocol header.
 *
 * Declares the dispatcher entry that every transport (SPI default,
 * SDIO optional) feeds into.  Per the project-memory rule "don't fork
 * the protocol -- one framing format, one command set, one set of
 * reply codes; only the transport layer differs" (the same model the
 * gd32-bridge uses for its SPI + I2C transports), both transports call
 * protocol_dispatch().
 *
 * The wire contract -- opcodes, flags, payload structs, response codes
 * -- is the CANONICAL host-side header include/alp/protocol/cc3501e.h,
 * included DIRECTLY here.  This firmware is its own repository now
 * (extracted from alp-sdk:firmware/cc3501e, alp-sdk#1370), so that header
 * is compiled out of an alp-sdk checkout the build is pointed at with
 * -DALP_SDK_ROOT: there is still no mirrored copy to drift out of sync,
 * but a protocol change is now two commits in two repos instead of one,
 * and CI pins protocol-version.txt against the header to catch the gap.
 */

#ifndef CC3501E_BRIDGE_PROTOCOL_H
#define CC3501E_BRIDGE_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

/* Canonical wire contract -- single source of truth, no mirror.
 * Resolved via the firmware CMake's include path onto
 * ${ALP_SDK_ROOT}/include (an alp-sdk checkout; the build FATAL_ERRORs
 * when the header is not there). */
#include "alp/protocol/cc3501e.h"

/* CC3501E_WIRE_CRC (customer-selectable, CMakeLists.txt) picks which wire
 * shape this image speaks: 1 (default) = wire MAJOR 4, the mandatory
 * CRC-16/CCITT-FALSE trailer on every frame; 0 = wire MAJOR 3
 * (@ref ALP_CC3501E_PROTOCOL_MAJOR_LEGACY), byte-identical to the 3.1 wire.
 * Every real build defines this via the CMake option or ti/build_ti.sh's
 * -DCC3501E_WIRE_CRC; this fallback exists so a build that has NOT picked
 * that up (a raw ad-hoc compile, an IDE that has not re-run CMake) fails
 * SAFE -- the strict, CRC-required wire -- rather than silently compiling
 * out the #1378 corruption guard. */
#ifndef CC3501E_WIRE_CRC
#define CC3501E_WIRE_CRC 1
#endif

/* --------------------------------------------------------------- */
/* Reply framing contract (firmware side)                            */
/* --------------------------------------------------------------- */
/*
 * On-wire reply frame mirrors the request shape (see the frame
 * diagram in <alp/protocol/cc3501e.h>):
 *
 *   +--------+--------+--------+--------+===================================+
 *   |  cmd   | flags  |  payload_len (LE)| payload (status+data+pad+crc)     |
 *   +--------+--------+--------+--------+===================================+
 *
 * Per the protocol header's stated convention -- "Response status
 * codes carried in the first byte of every response payload" -- the
 * reply payload is [status:u8][data...][zero pad][crc:u16 LE].
 * protocol_dispatch() writes only the DATA bytes (after the status) and
 * RETURNS the status; the transport prepends the status byte and builds
 * the 4-byte header.  payload_len COUNTS THE PAD AND THE CRC -- see
 * protocol_build_reply() below before adding any variable-length reply.
 *
 * WIRE MAJOR 4 (#2035): every request AND every reply now carries a
 * mandatory 2-byte CRC-16/CCITT-FALSE trailer (@ref ALP_CC3501E_CRC_BYTES,
 * <alp/protocol/cc3501e.h>), covering the frame's 4 header bytes plus every
 * payload byte except the trailing 2 CRC bytes themselves.  This firmware is
 * the STRICT side of the migration: it requires the CRC on every incoming
 * request unconditionally (a 3.1-shaped, CRC-less request is rejected with
 * ALP_CC3501E_RESP_ERR_PROTOCOL, never executed) and emits the new
 * CRC-bearing reply shape unconditionally -- there is no dual-mode firmware.
 *
 * ONE NORMATIVE EXCEPTION: CMD_GET_VERSION (0x01) is accepted WITH OR
 * WITHOUT the CRC trailer -- see protocol_build_reply()'s implementation
 * comment.  A host does not know a peer's wire major until GET_VERSION
 * answers it, and it appends a request CRC only once that major is already
 * negotiated, so the FIRST GET_VERSION any host sends a fresh peer is, by
 * construction, the zero-payload 3.1 shape.  Requiring a CRC on it
 * unconditionally would mean no host could ever discover a MAJOR-4 peer at
 * all, and OTA -- the only path from 3.1 to 4.0 firmware -- rides this same
 * gated request path, making that failure mode permanent.  Every other
 * opcode keeps the unconditional requirement.
 *
 * The HOST is the bilingual side of the migration (chips/cc3501e/cc3501e_core.c
 * in alp-sdk); see the MAJOR-4 paragraph above ALP_CC3501E_PROTOCOL_MAJOR in
 * <alp/protocol/cc3501e.h> for the full migration order.  NEVER ship a host
 * that refuses wire MAJOR 3 before every coprocessor in the fleet has been
 * OTA'd to 4.0 -- OTA is the only path from 3.1 to 4.0 firmware, and it needs
 * a host that can still talk to a 3.1 board.
 *
 * WIRE FRAMING (current E1M-AEN HW rev): the Alif dwc-ssi master drives
 * hardware SS0 around each protocol phase while the CC3501E SPI slave
 * advances on transfer-complete callbacks.  A request/reply is clocked as
 * four SS0-framed phases -- request header, request payload, reply header,
 * reply payload -- each side deriving the next length from an exchanged
 * header.  READY gates reply phases so the host does not clock a slave that
 * has not re-armed yet.  The Alif-side driver (alp-sdk chips/cc3501e/cc3501e_core.c)
 * implements the matching sequence + reads the status byte; the TI SPI-slave
 * backend (hal/ti/transport_hw_ti_spi.c) implements the slave side.  Both are
 * reconciled to this header (the spec) and bench-validated on AEN801.
 * Async events ride an attention edge on that same READY wire (#130,
 * alp-sdk#1721); see DESIGN.md for its two limits and for the dedicated
 * HOST_IRQ pad that is still a future board rev.
 */

/* Maximum reply DATA bytes a handler may emit (after the status byte).
 * The reply payload is status(1) + data + the mandatory 2-byte CRC trailer
 * (wire MAJOR 4); the whole payload stays within the protocol's
 * ALP_CC3501E_MAX_PAYLOAD ceiling, so the CRC costs 2 bytes of headroom a
 * handler could use for DATA before MAJOR 4. */
#if CC3501E_WIRE_CRC
#define CC3501E_REPLY_DATA_MAX (ALP_CC3501E_MAX_PAYLOAD - 1u - ALP_CC3501E_CRC_BYTES)
#else
#define CC3501E_REPLY_DATA_MAX (ALP_CC3501E_MAX_PAYLOAD - 1u)
#endif

/* Whole-frame sizes (header + max payload), shared by every transport. */
#define CC3501E_FRAME_MAX_BYTES (ALP_CC3501E_HEADER_BYTES + ALP_CC3501E_MAX_PAYLOAD)

/* Byte offsets within a reply frame: the status byte is the first
 * payload byte; reply DATA follows it. */
#define CC3501E_REPLY_STATUS_OFF (ALP_CC3501E_HEADER_BYTES) /* index 4 */

/* Reply payloads are padded up to a multiple of this so the HOST can DMA them as
 * one burst-aligned chunk, and the declared payload_len INCLUDES the pad AND
 * the wire-MAJOR-4 CRC trailer (see protocol_build_reply).  8 = the host's
 * default DW SSI burst, fifo_depth/2 with fifo_depth 16. */
#define CC3501E_REPLY_PAD      8u
#define CC3501E_REPLY_DATA_OFF (ALP_CC3501E_HEADER_BYTES + 1u) /* index 5 */

/* --------------------------------------------------------------- */
/* Wire MAJOR 4: the CRC trailer's 2-byte sizing tax (#2035)         */
/* --------------------------------------------------------------- */
/*
 * <alp/protocol/cc3501e.h>'s ALP_CC3501E_SPI1_MAX_XFER (4088) and
 * ALP_CC3501E_OTA_MAX_CHUNK (ALP_CC3501E_MAX_PAYLOAD - 4) are pinned by that
 * header's own _Static_asserts against the PRE-CRC framing, where a
 * request's declared wire payload_len WAS its logical (opcode-struct +
 * data) payload.  Wire MAJOR 4 appends a mandatory 2-byte CRC trailer
 * INSIDE that same payload_len (protocol_build_reply() strips it back off
 * before handing the logical payload to protocol_dispatch()), so it costs 2
 * bytes of the identical ALP_CC3501E_MAX_PAYLOAD ceiling every other
 * request payload already has to fit in.
 *
 * A maxed-out request built against the PRE-CRC constants would therefore
 * be 2 bytes too long to declare a legal wire payload_len at all: the
 * transport clamps a request header claiming more than
 * ALP_CC3501E_MAX_PAYLOAD (hal/ti/transport_hw_ti_spi.c's on_transfer()),
 * so the last 2 bytes of such a request -- which is exactly where the CRC
 * trailer lives -- would never be clocked, and the frame fails the CRC
 * check as silent, un-diagnosable corruption instead of a clean chunk-size
 * rejection.  This firmware reports AND enforces 2 bytes less than the
 * header's constants so a maxed-out chunk saturates the wire ceiling
 * exactly again, the same way ALP_CC3501E_SPI1_MAX_XFER and
 * ALP_CC3501E_OTA_MAX_CHUNK did before MAJOR 4.  The header itself is not
 * changed here (it is canonical, and this firmware does not edit it) --
 * only what THIS firmware reports (CMD_SPI1_CONFIGURE's max_xfer, worker.c)
 * and enforces (protocol_spi.c, protocol_ota.c) shrinks.
 *
 * CC3501E_WIRE_CRC=OFF (the customer-selectable no-CRC build, CMakeLists.txt)
 * pays none of this: a request payload_len on that wire is still exactly the
 * logical (opcode-struct + data) payload, no trailer riding inside it, so the
 * 2-byte reservation above is not just unneeded there but WRONG -- reporting
 * it from a no-CRC build silently wastes 2 bytes per transfer the host would
 * otherwise use, and the two _V4 names below resolve to the header's raw,
 * unreserved constants instead. */
#if CC3501E_WIRE_CRC
#define CC3501E_SPI1_MAX_XFER_V4 (ALP_CC3501E_SPI1_MAX_XFER - ALP_CC3501E_CRC_BYTES)
#define CC3501E_OTA_MAX_CHUNK_V4 (ALP_CC3501E_OTA_MAX_CHUNK - ALP_CC3501E_CRC_BYTES)
#else
#define CC3501E_SPI1_MAX_XFER_V4 ALP_CC3501E_SPI1_MAX_XFER
#define CC3501E_OTA_MAX_CHUNK_V4 ALP_CC3501E_OTA_MAX_CHUNK
#endif

/* --------------------------------------------------------------- */
/* Dispatcher                                                        */
/* --------------------------------------------------------------- */
/*
 * protocol_dispatch -- called by a transport once a complete request
 * frame (header + payload) has been validated (framing OK).
 *
 * Inputs:
 *   cmd          -- opcode (one of ALP_CC3501E_CMD_* / EVT_*).
 *   flags        -- request flags byte (ALP_CC3501E_FLAG_*).
 *   req          -- pointer to req_len request payload bytes (may be
 *                   NULL when req_len == 0).
 *   req_len      -- request payload length.
 *
 * Outputs (caller-supplied):
 *   reply_data       -- buffer for the reply DATA bytes (the bytes that
 *                       follow the status byte in the reply payload).
 *   reply_cap        -- capacity of reply_data.
 *   reply_data_len   -- [out] number of DATA bytes written.
 *
 * Return: the response status (ALP_CC3501E_RESP_*) the transport
 *         emits as the first reply-payload byte.  Unknown or
 *         not-yet-implemented opcodes return ALP_CC3501E_RESP_ERR_INVALID
 *         (per the header's contract: firmware rejects opcodes it does
 *         not implement with ALP_CC3501E_RESP_ERR_INVALID).
 */
alp_cc3501e_resp_t protocol_dispatch(uint8_t        cmd,
                                     uint8_t        flags,
                                     const uint8_t *req,
                                     size_t         req_len,
                                     uint8_t       *reply_data,
                                     size_t         reply_cap,
                                     size_t        *reply_data_len);

/*
 * protocol_build_reply -- the transport-agnostic framing wrapper.
 *
 * Parses a complete request FRAME (4-byte header + payload + the mandatory
 * wire-MAJOR-4 CRC trailer), verifies the CRC, validates the framing, runs
 * protocol_dispatch() with the CRC stripped back off, and writes a complete
 * reply FRAME (header + status + data + PAD + CRC) into @p reply_frame.
 * Every transport (SPI, SDIO) calls this so the on-wire framing is
 * byte-identical regardless of which link the customer selected.
 *
 *   req_frame / req_len  -- the received request frame.  A frame too short
 *                           to hold a header, whose declared payload_len
 *                           doesn't match req_len, too short to even hold
 *                           the mandatory CRC trailer (a 3.1-shaped,
 *                           CRC-less request), or whose CRC does not
 *                           verify, yields a RESP_ERR_PROTOCOL reply frame
 *                           -- never executed.  This firmware is the STRICT
 *                           side of the wire-MAJOR-4 migration: the CRC is
 *                           required unconditionally, with no dual-mode
 *                           fallback (see the MAJOR-4 paragraph in this
 *                           header's top comment).  ALL OF THIS IS THE
 *                           CC3501E_WIRE_CRC=ON (default) SHAPE.  With
 *                           CC3501E_WIRE_CRC=OFF this function instead
 *                           builds the byte-identical-to-3.1 shape: no
 *                           trailer required or verified on any request
 *                           (GET_VERSION alone still tolerates one present,
 *                           see protocol.c), no trailer appended to any
 *                           reply, and @ref ALP_CC3501E_RESP_OK_LEGACY (not
 *                           @ref ALP_CC3501E_RESP_OK) is the wire OK byte.
 *   reply_frame          -- output buffer; MUST be at least
 *                           CC3501E_FRAME_MAX_BYTES.
 *   reply_cap            -- capacity of reply_frame.
 *
 * PADDING -- read this before adding a variable-length reply.  The reply
 * payload (status + data + the 2-byte CRC trailer) is rounded UP to a
 * multiple of CC3501E_REPLY_PAD with ZERO bytes, and the declared
 * payload_len INCLUDES that pad AND the CRC, so payload_len is NOT
 * status + data.  The CRC itself always occupies the LAST 2 bytes of the
 * padded span (<alp/protocol/cc3501e.h>'s ALP_CC3501E_REPLY_PAD doc), so a
 * bare-status reply (1 byte) still pads to CC3501E_REPLY_PAD exactly as
 * before MAJOR 4 -- the CRC simply consumes 2 of what used to be 7 pad
 * bytes, costing zero extra wire bytes on the common case.  The host
 * clocks the padded length as one burst-aligned DMA chunk; that is the
 * whole point of it.  The pad is skipped only when it would not fit
 * @p reply_cap.
 *
 * The consequence is a rule on handlers: ANY NEW VARIABLE-LENGTH REPLY
 * PAYLOAD MUST BE SELF-DELIMITING -- it has to carry its own count or
 * its own terminator, because payload_len no longer delimits the data.
 * GET_PENDING_EVENTS shipped without that and the pad bytes were walked
 * as events: an empty ring returned 7 zero bytes, decoded as three
 * "opcode 0x00, len 0" entries, ~5.8 phantom events per second
 * (alp-sdk#1740; the host walk now stops at a zero opcode).  SOCK_RECV
 * is safe because alp_cc3501e_sock_recv_resp_t carries its own data_len.
 *
 * Returns the reply frame length in bytes (always >= header + 1).
 */
size_t protocol_build_reply(const uint8_t *req_frame,
                            size_t         req_len,
                            uint8_t       *reply_frame,
                            size_t         reply_cap);

/*
 * protocol_reset_retry_latch -- invalidate the generic worker-routed
 * request-identity latch (protocol.c's static s_retry_latch, issue #102).
 *
 * Production boot needs no call to this: a static struct's `valid` field is
 * zero-initialised, so the latch already starts empty.  It exists purely so
 * the host unit-test binary can clear the SAME file-static between ZTEST
 * cases -- it is a single process that never reboots between tests, so
 * without this a latch entry written by one test (e.g. a BLE_ENABLE
 * collected under seq 0) would be served back as a cache hit to a LATER,
 * unrelated test that happens to exercise the same opcode with the same
 * (default-zero) seq, turning an expected BUSY/submit into a served-from-
 * cache status and breaking that test.  Mirrors worker_init(), which exists
 * for the identical reason (see the worker.h doc comment); called from
 * tests/unit/transport_spi/src/test_transport_spi.c's per-test reset hook,
 * never from main().
 */
void protocol_reset_retry_latch(void);

/**
 * @brief Precompute the wire-CRC lookup table from the boot path.
 *
 * MUST be called before the SPI slave is armed for its first request frame.
 * The table used to be built lazily on first use, and first use is
 * protocol_build_reply() running in SPI interrupt context -- ~100 us at
 * 160 MHz, against a host reply gate that is a blind fixed 200 us already
 * mostly spent on normal dispatch.  Overshooting it once desyncs the link
 * permanently: the host clocks into an unarmed slave, the stray bytes sit in
 * the RX FIFO, and every later transfer returns the previous phase's TX
 * bytes.  Root-caused on silicon 2026-09-10.
 *
 * Idempotent, and safe to call on either CC3501E_WIRE_CRC arm -- the OFF
 * build still CRCs the payload_len==2 GET_VERSION shape.
 */
void protocol_crc16_table_init(void);

#endif /* CC3501E_BRIDGE_PROTOCOL_H */
