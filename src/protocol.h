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

/* --------------------------------------------------------------- */
/* Reply framing contract (firmware side)                            */
/* --------------------------------------------------------------- */
/*
 * On-wire reply frame mirrors the request shape (see the frame
 * diagram in <alp/protocol/cc3501e.h>):
 *
 *   +--------+--------+--------+--------+===========================+
 *   |  cmd   | flags  |  payload_len (LE)| payload (status+data+pad) |
 *   +--------+--------+--------+--------+===========================+
 *
 * Per the protocol header's stated convention -- "Response status
 * codes carried in the first byte of every response payload" -- the
 * reply payload is [status:u8][data...][zero pad].  protocol_dispatch()
 * writes only the DATA bytes (after the status) and RETURNS the status;
 * the transport prepends the status byte and builds the 4-byte header.
 * payload_len COUNTS THE PAD -- see protocol_build_reply() below before
 * adding any variable-length reply.
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
 * The reply payload is status(1) + data; the whole payload stays within
 * the protocol's ALP_CC3501E_MAX_PAYLOAD ceiling. */
#define CC3501E_REPLY_DATA_MAX (ALP_CC3501E_MAX_PAYLOAD - 1u)

/* Whole-frame sizes (header + max payload), shared by every transport. */
#define CC3501E_FRAME_MAX_BYTES (ALP_CC3501E_HEADER_BYTES + ALP_CC3501E_MAX_PAYLOAD)

/* Byte offsets within a reply frame: the status byte is the first
 * payload byte; reply DATA follows it. */
#define CC3501E_REPLY_STATUS_OFF (ALP_CC3501E_HEADER_BYTES) /* index 4 */

/* Reply payloads are padded up to a multiple of this so the HOST can DMA them as
 * one burst-aligned chunk, and the declared payload_len INCLUDES the pad (see
 * protocol_build_reply).  8 = the host's default DW SSI burst, fifo_depth/2 with
 * fifo_depth 16. */
#define CC3501E_REPLY_PAD      8u
#define CC3501E_REPLY_DATA_OFF (ALP_CC3501E_HEADER_BYTES + 1u) /* index 5 */

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
 * Parses a complete request FRAME (4-byte header + payload), validates
 * the framing, runs protocol_dispatch(), and writes a complete reply
 * FRAME (header + status + data + PAD) into @p reply_frame.  Every
 * transport (SPI, SDIO) calls this so the on-wire framing is
 * byte-identical regardless of which link the customer selected.
 *
 *   req_frame / req_len  -- the received request frame.  A frame too
 *                           short to hold a header, or whose declared
 *                           payload_len doesn't match req_len, yields a
 *                           RESP_ERR_PROTOCOL reply frame.
 *   reply_frame          -- output buffer; MUST be at least
 *                           CC3501E_FRAME_MAX_BYTES.
 *   reply_cap            -- capacity of reply_frame.
 *
 * PADDING -- read this before adding a variable-length reply.  The reply
 * payload (status + data) is rounded UP to a multiple of
 * CC3501E_REPLY_PAD with ZERO bytes, and the declared payload_len
 * INCLUDES that pad, so payload_len is NOT status + data.  The host
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

#endif /* CC3501E_BRIDGE_PROTOCOL_H */
