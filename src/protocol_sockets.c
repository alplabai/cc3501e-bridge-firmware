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
 * is handed.  For CMD_SOCK_SEND that meant a lost/misframed RESP_OK reply
 * caused the payload to be TRANSMITTED AGAIN.
 *
 * protocol_spi.c solves the analogous problem for SPI1_TRANSFER by serving a
 * matching-seq retry straight out of the WORKER JOB SLOT.  That shape does
 * not fit here: sockets share that single slot with every other worker-
 * routed op (Wi-Fi scan, BLE, ...), and worker_poll()'s orphan-discard arm
 * destroys a terminal result the instant any OTHER opcode polls before the
 * host collects it (see the comment there) -- an interleaved poll between a
 * send completing and its retry landing would silently defeat an in-slot
 * cache.  A dedicated static cache sidesteps that entirely, and it is cheap:
 * the send reply is a 2-byte queued-count, so this is 4 bytes of static RAM
 * (valid flag + seq + the 2 reply bytes), nothing like SPI1's 4 KB (see
 * protocol_spi.c's own note on that cost, offered as the upgrade path if a
 * host ever needs more than the single most-recent send cached). */
static bool    g_sock_send_cached;
static uint8_t g_sock_send_seq;
static uint8_t g_sock_send_reply[2];

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
 * Reply DATA = uint16_t LE queued-byte count.
 *
 * req[3] is alp_cc3501e_sock_send_t.seq (v7; formerly `reserved`, always 0
 * through v6 -- see the wire-compat note on CC3501E_FW_IMPLEMENTS_PROTOCOL in
 * protocol_meta.c).  A retry of an already-completed send carries the SAME
 * seq (the host assigns it once per logical send, see alp-sdk's
 * cc3501e_sock_send()), so a matching seq is served from g_sock_send_reply
 * WITHOUT touching the worker at all -- no submit, no re-transmit.  Anything
 * else (a genuinely new send, or the very first one) falls through to the
 * worker-routed path unchanged, exactly as before this fix. */
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
	if (g_sock_send_cached && seq == g_sock_send_seq) {
		if (reply_cap < sizeof(g_sock_send_reply)) return ALP_CC3501E_RESP_ERR_NO_MEM;
		memcpy(reply_data, g_sock_send_reply, sizeof(g_sock_send_reply));
		*reply_data_len = sizeof(g_sock_send_reply);
		return ALP_CC3501E_RESP_OK;
	}

	const alp_cc3501e_resp_t st = handle_worker_routed_payload_reply(
	    ALP_CC3501E_CMD_SOCK_SEND, req, req_len, 2u, reply_data, reply_cap, reply_data_len);
	if (st == ALP_CC3501E_RESP_OK) {
		/* The worker body (worker.c's ALP_CC3501E_CMD_SOCK_SEND case) always
		 * publishes exactly 2 result bytes on success, so *reply_data_len is 2
		 * here; cache defensively on the actual length anyway. */
		if (*reply_data_len == sizeof(g_sock_send_reply)) {
			memcpy(g_sock_send_reply, reply_data, sizeof(g_sock_send_reply));
			g_sock_send_seq    = seq;
			g_sock_send_cached = true;
		}
	}
	return st;
}

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
	 * worker path unchanged. */
	{
		const uint16_t handle  = (uint16_t)((uint16_t)req[0] | ((uint16_t)req[1] << 8));
		const uint16_t max_len = (uint16_t)((uint16_t)req[2] | ((uint16_t)req[3] << 8));
		const size_t   hdr     = sizeof(alp_cc3501e_sock_recv_resp_t);
		if (reply_cap > hdr) {
			size_t room = reply_cap - hdr;
			if (max_len != 0u && room > (size_t)max_len) room = (size_t)max_len;
			uint16_t  got = 0u;
			const int rc =
			    cc3501e_hw_sock_recv_ring(handle, &reply_data[hdr], (uint16_t)room, &got);
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
