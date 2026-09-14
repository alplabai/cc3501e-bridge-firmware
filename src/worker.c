/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * cc3501e-bridge firmware: async-job worker (submit/poll/drain).
 *
 * The de-risked seam that keeps the SPI ISR fast (see worker.h): the ISR
 * SUBMITS a job + POLLS its cached result; the slow, possibly-blocking HAL
 * body runs OUTSIDE the ISR in worker_run_pending() (the drain), called
 * from main()'s loop / bringup_task.  A single in-flight job suffices for
 * v0.2.
 *
 * This TU is SILICON-FREE -- it pulls in NO TI SDK; the blocking bodies
 * are the cc3501e_hw_* HAL shims.  Concurrency with the SPI ISR is handled
 * two ways:
 *   1. The shared `job` struct is `volatile`, and `state` is the SINGLE
 *      synchronisation variable.  The drain and the ISR are ordered around
 *      it: result[]/result_len/err are only WRITTEN while state is QUEUED/
 *      RUNNING (the ISR never reads them then -- it sees < DONE and replies
 *      BUSY), and only READ once state == DONE/ERR (the drain never writes
 *      them then).  `state` is the last field written when publishing a
 *      result and the first checked when polling -- a release/acquire-style
 *      ordering that the short critical section below makes atomic.
 *   2. The multi-field publish (write result THEN flip state to DONE) and
 *      the poll's read-then-reset are wrapped in worker_critical_enter/
 *      exit -- weak no-ops here (correct for the single-threaded native
 *      build) that the ti backend overrides with __disable_irq/restore so
 *      the SPI ISR cannot observe a half-updated job on real silicon.
 */

#include <stdbool.h>
#include <stddef.h> /* offsetof -- SOCK_SEND's own seq field, not a magic wire offset */
#include <string.h>

#include "worker.h"
#include "protocol.h"  /* CC3501E_SPI1_MAX_XFER_V4 -- the CONFIGURE reply's max_xfer */
#include "transport.h" /* bridge_transport_spi_hw_reinit -- post-radio-op SPI re-sync */
#include "../hal/cc3501e_hw.h"

/* ---- ISR-vs-drain mutual exclusion (weak; ti backend overrides) ---- *
 * Default no-ops are correct for the host/native build (single-threaded,
 * no ISR).  hal/ti/cc3501e_hw_ti.c provides strong versions that mask
 * interrupts (__disable_irq / restore PRIMASK) so the SPI ISR cannot read
 * a half-published result on silicon.  Kept here (not in transport.h) so
 * the worker owns its own synchronisation contract. */
__attribute__((weak)) unsigned long worker_critical_enter(void)
{
	return 0u;
}

__attribute__((weak)) void worker_critical_exit(unsigned long key)
{
	(void)key;
}

/* ---- Bridge READY/host-IRQ line (weak; ti backend drives a real GPIO) ---- *
 * The transport's flow-control to the host: a radio op kills the SPI-slave DMA
 * (the bridge cannot be serviced while it runs), so the worker drives the line
 * BUSY before every blocking radio op and READY again once the slave is
 * re-armed (after the post-op bridge_transport_spi_hw_reinit).  The host gates
 * its clocking on READY so it never drives a transaction into a dead slave.
 * Default no-ops (host/native build has no host-IRQ line); the ti backend
 * (hal/ti/cc3501e_hw_ti.c) overrides them to drive CC35 GPIO17 (E1M IO16). */
__attribute__((weak)) void cc3501e_bridge_busy(void)
{
}

__attribute__((weak)) void cc3501e_bridge_ready(void)
{
}

/* Weak no-op: only the TI backend drives the attention wire, and only when built
 * with CC3501E_ATTN_PULSE.  Every other backend (stub, native_sim) links this. */
__attribute__((weak)) void cc3501e_bridge_attn_pulse(void)
{
}

/* The single in-flight job + its cached result.  `volatile`: written by
 * the drain (worker_run_pending / synchronous worker_submit), read by the
 * SPI ISR (worker_poll). */
static struct {
	volatile enum worker_state state;
	volatile uint8_t           job_cmd;
	volatile uint8_t           result[ALP_CC3501E_MAX_PAYLOAD];
	volatile uint16_t          result_len;
	volatile int8_t            err;
	/* Request payload for jobs that carry one (WIFI_CONNECT_STA / WIFI_AP_START).
	 * Parameterless jobs (GET_MAC / scan / ble) leave req_len 0.  Written by the
	 * ISR in worker_submit_payload while state goes IDLE->QUEUED; read by the drain
	 * while RUNNING -- the same state-ordered hand-off as result[]. */
	volatile uint8_t  req[ALP_CC3501E_MAX_PAYLOAD];
	volatile uint16_t req_len;
} job;

/* Ground truth for "the HAL body actually ran" (issue #102's DIAG_GET_STATS
 * pairing with g_retry_latch_hits -- protocol.c / protocol_diag.c).
 * worker_execute() is the ONE place any cc3501e_hw_* body is called from, on
 * either path (the ISR-synchronous stub submit or the real drain thread), so
 * incrementing it here -- and nowhere else -- is what lets a bench run tell
 * "reply lost, correctly de-duped" (this stays put, the latch hit counts)
 * from "reply lost, re-executed" (this increments again).  volatile: the
 * real (CC3501E_WIFI) build increments it from worker_run_pending()'s drain
 * context, which protocol_diag.c's DIAG_GET_STATS handler (SPI-ISR context)
 * reads from a different context -- unlike g_frames_ok/g_frames_err/
 * g_retry_latch_hits, which are only ever touched from the SPI-ISR side. */
volatile uint32_t g_worker_execs;

/* Little-endian store helper for the socket reply wire (parallels protocol.c). */
static void wk_put_le16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)(v & 0xFFu);
	p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

/* Little-endian store helper for the SPI1 config reply (actual SCK in Hz). */
static void wk_put_le32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)(v & 0xFFu);
	p[1] = (uint8_t)((v >> 8) & 0xFFu);
	p[2] = (uint8_t)((v >> 16) & 0xFFu);
	p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

/* Read a 16-bit LE field out of the (volatile) request buffer byte-by-byte. */
static uint16_t wk_get_le16(const volatile uint8_t *p, size_t off)
{
	return (uint16_t)p[off] | ((uint16_t)p[off + 1u] << 8);
}

/* Same, 32-bit (SPI1 freq_hz). */
static uint32_t wk_get_le32(const volatile uint8_t *p, size_t off)
{
	return (uint32_t)p[off] | ((uint32_t)p[off + 1u] << 8) | ((uint32_t)p[off + 2u] << 16) |
	       ((uint32_t)p[off + 3u] << 24);
}

/* recv reply header = alp_cc3501e_sock_recv_resp_t: from(sock_addr 20) |
 * data_len(LE16) | reserved(LE16).  The received bytes follow inline. */
#define WK_SOCK_RECV_HDR 24u

/* Run the blocking HAL body for @p cmd and publish DONE/ERR.  Called from
 * the drain (RUNNING) and from the synchronous stub path in worker_submit.
 * NOT ISR context -- may block.  v0.2 services only GET_MAC; future
 * opcodes add cases here (each a fast-failing default keeps it honest). */
static void worker_execute(uint8_t cmd)
{
	uint8_t buf[ALP_CC3501E_MAX_PAYLOAD];
	size_t  len = 0u;
	int     rv  = CC3501E_HW_ERR_NOTIMPL;

	/* One increment per drain execution, unconditionally -- issue #102's
	 * ground truth that the HAL body ran, independent of what it returned
	 * (OK, an HW_ERR_*, or the default NOTIMPL below for an opcode nobody
	 * added a case for). */
	g_worker_execs++;

	switch (cmd) {
	case ALP_CC3501E_CMD_GET_MAC: {
		uint8_t mac[6];
		rv = cc3501e_hw_get_mac(mac); /* WiFi build: lazy-inits, then Wlan_Get (blocks) */
		if (rv == CC3501E_HW_OK) {
			memcpy(buf, mac, 6u);
			len = 6u;
		}
		break;
	}
	case ALP_CC3501E_CMD_WIFI_GET_RSSI: {
		int8_t r = 0;
		rv       = cc3501e_hw_wifi_get_rssi(
		    &r); /* WiFi build: lazy-starts radio, then Wlan_Get (blocks) */
		if (rv == CC3501E_HW_OK) {
			buf[0] = (uint8_t)r;
			len    = 1u;
		}
		break;
	}
	case ALP_CC3501E_CMD_WIFI_SCAN_START:
		/* Packs the AP-record list into buf in the host's wire format (see
		 * cc3501e_hw_wifi_scan); blocks on the scan + event rendezvous.
		 * Capped at CC3501E_REPLY_DATA_MAX (protocol.h), not
		 * ALP_CC3501E_MAX_PAYLOAD: that is the actual ceiling
		 * protocol_build_reply() enforces on any handler's reply DATA (the
		 * status byte and, under CC3501E_WIRE_CRC=ON, the CRC trailer both
		 * ride inside the same MAX_PAYLOAD budget) -- passing the full
		 * MAX_PAYLOAD here let the scan fill more records than a reply can
		 * ever carry, silently truncated by worker_poll()'s collect (host
		 * review, same class of bug as the SOCK_RECV data_cap fix below). */
		rv = cc3501e_hw_wifi_scan(buf, CC3501E_REPLY_DATA_MAX, &len);
		break;
	case ALP_CC3501E_CMD_BLE_ENABLE:
		/* Wi-Fi-first (shared HIF) then nimble_host_start -- blocks ~2s, so it
		 * is worker-routed off the SPI ISR exactly like GET_MAC.  Argless: an
		 * OK result carries no payload (len stays 0). */
		rv = cc3501e_hw_ble_enable();
		break;
	case ALP_CC3501E_CMD_BLE_SCAN_START:
		/* Packs the discovered-advertiser list into buf (see cc3501e_hw_ble_scan);
		 * runs a NimBLE GAP discovery that blocks for the scan window, so it is
		 * worker-routed off the SPI ISR exactly like WIFI_SCAN_START.  Capped
		 * at CC3501E_REPLY_DATA_MAX, not ALP_CC3501E_MAX_PAYLOAD -- see the
		 * WIFI_SCAN_START case above for why. */
		rv = cc3501e_hw_ble_scan(buf, CC3501E_REPLY_DATA_MAX, &len);
		break;
	case ALP_CC3501E_CMD_BLE_ADV_START: {
		/* Ext-adv config+start BLOCKS on the shared-HIF HCI ack (2 s), so -- like
		 * the connect ops -- it MUST run here in the drain, not the SPI ISR (that
		 * was the adv-wedge -4).  The 7-byte header (connectable | reserved |
		 * itvl_min LE16 | itvl_max LE16 | adv_data_len) + inline adv_data were
		 * stashed in job.req by worker_submit_payload; the protocol handler already
		 * length-validated them.  Argless reply (OK carries no payload). */
		const uint8_t  connectable  = (uint8_t)job.req[0];
		const uint16_t itvl_min     = (uint16_t)job.req[2] | ((uint16_t)job.req[3] << 8);
		const uint16_t itvl_max     = (uint16_t)job.req[4] | ((uint16_t)job.req[5] << 8);
		const uint8_t  adv_data_len = (uint8_t)job.req[6];
		rv                          = cc3501e_hw_ble_adv_start(
		    connectable, itvl_min, itvl_max, (const uint8_t *)job.req + 7, adv_data_len);
		break;
	}
	case ALP_CC3501E_CMD_BLE_ADV_STOP:
		/* Stops the adv set; issues HCI over the shared HIF + re-syncs the bridge
		 * SPI (blocks), so it is worker-routed off the SPI ISR.  Argless. */
		rv = cc3501e_hw_ble_adv_stop();
		break;
	case ALP_CC3501E_CMD_BLE_SCAN_STOP:
		/* Same shape as BLE_ADV_STOP: the body is
		 * busy(); nimble_scan_stop(); spi_hw_reinit(); ready() -- it tears the SPI
		 * slave down and re-opens it, so it must not run in the callback that is
		 * building this command's own reply.  Argless.  Issue #5. */
		rv = cc3501e_hw_ble_scan_stop();
		break;
	case ALP_CC3501E_CMD_BLE_DISCONNECT:
		/* Blocks on the disconnect HCI event over the shared HIF, then re-syncs
		 * the bridge.  Argless.  Issue #5. */
		rv = cc3501e_hw_ble_disconnect();
		break;
	case ALP_CC3501E_CMD_BLE_DISABLE:
		/* Tears down adv+scan via NimBLE; issues HCI over the shared HIF + re-syncs
		 * the bridge SPI (blocks), so it is worker-routed off the SPI ISR.  Argless. */
		rv = cc3501e_hw_ble_disable();
		break;
	case ALP_CC3501E_CMD_BLE_CONNECT:
		/* GAP connect blocks on the connection-complete HCI event over the shared
		 * HIF, so it is worker-routed off the SPI ISR.  Payload = addr_type(1) |
		 * addr[6], stashed in job.req by worker_submit_payload (validated by the
		 * protocol handler).  Argless reply. */
		rv = cc3501e_hw_ble_connect((uint8_t)job.req[0], (const uint8_t *)job.req + 1);
		break;
	case ALP_CC3501E_CMD_BLE_GATT_REGISTER: {
		/* Registers the attribute table (blocks in ble_gatts_start), so it is worker-
		 * routed off the SPI ISR.  Payload = the descriptor (job.req_len bytes, already
		 * validated by protocol_ble.c).  Reply = status(1) | num_handles(1) |
		 * attr_handle(LE16)*num_handles -- see the wire-format doc block in
		 * <alp/protocol/cc3501e.h> and handle_worker_routed_payload_reply. */
		uint16_t handles[ALP_CC3501E_BLE_GATT_MAX_CHARS];
		uint16_t num_handles = 0u;
		rv                   = cc3501e_hw_ble_gatt_register((const uint8_t *)job.req,
		                                                    job.req_len,
		                                                    handles,
		                                                    ALP_CC3501E_BLE_GATT_MAX_CHARS,
		                                                    &num_handles);
		if (rv == CC3501E_HW_OK) {
			buf[0] = 0u; /* status: OK (the frame-level resp is authoritative; this mirrors it) */
			buf[1] = (uint8_t)num_handles;
			for (uint16_t i = 0u; i < num_handles; i++) {
				wk_put_le16(&buf[2u + 2u * i], handles[i]);
			}
			len = 2u + 2u * (size_t)num_handles;
		}
		break;
	}
	case ALP_CC3501E_CMD_BLE_GATT_NOTIFY: {
		/* Pushes a notification (blocks on HCI over the shared HIF), so it is
		 * worker-routed off the SPI ISR.  Payload = handle(LE16) | data[job.req_len-2]. */
		const uint16_t handle = (uint16_t)job.req[0] | ((uint16_t)job.req[1] << 8);
		rv                    = cc3501e_hw_ble_gatt_notify(
		    handle, (const uint8_t *)job.req + 2, (uint16_t)(job.req_len - 2u));
		break;
	}
	case ALP_CC3501E_CMD_BLE_GATT_WRITE: {
		/* GATT write (blocks on HCI over the shared HIF), so it is worker-routed off
		 * the SPI ISR.  Payload = handle(LE16) | data[job.req_len-2]. */
		const uint16_t handle = (uint16_t)job.req[0] | ((uint16_t)job.req[1] << 8);
		rv                    = cc3501e_hw_ble_gatt_write(
		    handle, (const uint8_t *)job.req + 2, (uint16_t)(job.req_len - 2u));
		break;
	}
	case ALP_CC3501E_CMD_BLE_GATT_READ: {
		/* GATT read (blocks on the read-response HCI over the shared HIF), so it is
		 * worker-routed off the SPI ISR.  Payload = handle(LE16); the attribute value
		 * is packed into buf and published so the payload+reply worker path copies it
		 * back to the host (see handle_worker_routed_payload_reply).  Capped at
		 * CC3501E_REPLY_DATA_MAX, not ALP_CC3501E_MAX_PAYLOAD -- see the
		 * WIFI_SCAN_START case above for why. */
		const uint16_t handle  = (uint16_t)job.req[0] | ((uint16_t)job.req[1] << 8);
		uint16_t       out_len = 0u;
		rv = cc3501e_hw_ble_gatt_read(handle, buf, (uint16_t)CC3501E_REPLY_DATA_MAX, &out_len);
		if (rv == CC3501E_HW_OK) {
			len = out_len;
		}
		break;
	}
	case ALP_CC3501E_CMD_WIFI_CONNECT_STA:
	case ALP_CC3501E_CMD_WIFI_AP_START: {
		/* Association BLOCKS until the connect/IP event (seconds), so -- unlike the
		 * other Wlan_* ops which were already worker-routed -- this MUST run here in
		 * the drain, not in protocol_dispatch's SPI-ISR context (a blocking wait in
		 * the ISR either hung the bridge or could not pend at all -> the -4/-2
		 * connect failures).  The request payload (alp_cc3501e_wifi_connect_t header
		 * + inline ssid + psk) was stashed in job.req by worker_submit_payload; its
		 * length was already validated by the protocol handler before submit. */
		const alp_cc3501e_wifi_connect_t *c =
		    (const alp_cc3501e_wifi_connect_t *)(const void *)job.req;
		const uint8_t *ssid = (const uint8_t *)job.req + sizeof(*c);
		const uint8_t *psk  = ssid + c->ssid_len;
		rv = (cmd == ALP_CC3501E_CMD_WIFI_AP_START)
		         ? cc3501e_hw_wifi_ap_start(ssid, c->ssid_len, psk, c->psk_len, c->security)
		         : cc3501e_hw_wifi_connect_sta(ssid, c->ssid_len, psk, c->psk_len, c->security);
		break;
	}
	case ALP_CC3501E_CMD_WIFI_DISCONNECT:
		/* Wlan_Disconnect() is an NWP command over the shared HIF.  Inline it
		 * blocked the SPI callback framing this reply and left the slave
		 * unresynced, because the busy/ready bracket and the post-op
		 * bridge_transport_spi_hw_reinit() both live in this drain.  Argless.
		 * Issue #5. */
		rv = cc3501e_hw_wifi_disconnect();
		break;
	case ALP_CC3501E_CMD_WIFI_AP_STOP:
		/* Tears the soft-AP down; blocks while the radio goes down and the bridge
		 * SPI re-syncs, exactly like BLE_ADV_STOP / BLE_DISABLE above -- so it is
		 * worker-routed off the SPI ISR.  Argless.
		 *
		 * This case is the fix for alp-sdk#1563: handle_wifi_ap_stop() used to call
		 * cc3501e_hw_wifi_ap_stop() INLINE in protocol_dispatch, i.e. in the SPI-ISR
		 * context, which is the one thing the AP_START comment above says must not
		 * happen ("a blocking wait in the ISR either hung the bridge or could not
		 * pend at all").  Bench-measured on E1M-AEN801 (2026-08-18), that inline
		 * call reproduced the whole symptom set: an ap-stop with NO AP running
		 * returned OK (nothing to tear down, so it never blocked), an ap-stop with
		 * an AP ACTUALLY RUNNING returned -4 every time (the reply could not be
		 * framed while the ISR was blocked), and the transport was left wedged at
		 * -5 for every later request until the board was reset. */
		rv = cc3501e_hw_wifi_ap_stop();
		break;
	case ALP_CC3501E_CMD_SOCK_OPEN: {
		/* job.req = alp_cc3501e_sock_open_t: family(0) | type(1) | protocol(2) |
		 * reserved(3).  Reply DATA = alp_cc3501e_sock_handle_t: handle(LE16) |
		 * reserved(2). */
		uint16_t handle = 0u;
		rv              = cc3501e_hw_sock_open(job.req[0], job.req[1], job.req[2], &handle);
		if (rv == CC3501E_HW_OK) {
			wk_put_le16(buf, handle);
			buf[2] = 0u;
			buf[3] = 0u;
			len    = 4u;
		}
		break;
	}
	case ALP_CC3501E_CMD_SOCK_CONNECT: {
		/* job.req = alp_cc3501e_sock_connect_t: handle(LE16 @0) | reserved(@2) |
		 * peer sock_addr @4 { family(@4) | reserved(@5) | port(LE16 @6) |
		 * addr[16] @8 }.  v1 IPv4: only addr[8..11] are meaningful. */
		const uint16_t handle = wk_get_le16(job.req, 0u);
		const uint8_t  family = job.req[4];
		const uint16_t port   = wk_get_le16(job.req, 6u);
		uint8_t        addr[4];
		for (unsigned i = 0u; i < 4u; ++i)
			addr[i] = job.req[8u + i];
		rv = cc3501e_hw_sock_connect(handle, family, port, addr);
		break;
	}
	case ALP_CC3501E_CMD_SOCK_SEND: {
		/* job.req = alp_cc3501e_sock_send_t: handle(LE16 @0) | flags(@2) |
		 * seq(@3, v7 -- protocol_sockets.c's handle_sock_send() already served
		 * a matching retry off its own cache before this ever submits) |
		 * data_len(LE16 @4) | reserved2(@6) | data @8.  Reply DATA = uint16_t
		 * LE byte count actually queued. */
		const uint16_t handle   = wk_get_le16(job.req, 0u);
		const uint8_t  flags    = job.req[2];
		const uint16_t data_len = wk_get_le16(job.req, 4u);
		uint16_t       sent     = 0u;
		/* The data rides inline in job.req at offset 8 (the handler validated
		 * req_len == 8 + data_len <= MAX_PAYLOAD); pass it in place, mirroring how
		 * the WIFI_CONNECT case reads ssid/psk straight out of job.req. */
		rv = cc3501e_hw_sock_send(handle, flags, (const uint8_t *)&job.req[8], data_len, &sent);
		if (rv == CC3501E_HW_OK) {
			wk_put_le16(buf, sent);
			len = 2u;
		}
		break;
	}
	case ALP_CC3501E_CMD_SOCK_RECV: {
		/* job.req = alp_cc3501e_sock_recv_t: handle(LE16 @0) | max_len(LE16 @2).
		 * Reply = recv_resp header (WK_SOCK_RECV_HDR) + up to max_len bytes.
		 *
		 * data_cap is bounded by TWO things, both load-bearing:
		 *
		 *   1. CC3501E_REPLY_DATA_MAX (protocol.h) minus the recv-resp header --
		 *      the ACTUAL ceiling protocol_build_reply() enforces on this
		 *      handler's reply DATA, not a flat "MAX_PAYLOAD - 1" that predates
		 *      the wire MAJOR 4 CRC-trailer tax (#2035).  The old flat formula
		 *      could ask cc3501e_hw_sock_recv() for up to 4071 B (24 + 4071 =
		 *      4095), 2 B more than CC3501E_REPLY_DATA_MAX (4093 B) under the
		 *      default CC3501E_WIRE_CRC=ON actually allows -- lwip_recvfrom()
		 *      would CONSUME those bytes off the socket, then worker_poll()'s
		 *      collect silently truncated the reply by 2 B, permanently losing
		 *      already-read stream data with no error reported (host review:
		 *      the host bench app reads exactly 4071 B per call, so a
		 *      worker-fallback socket -- UDP, or STREAM accepted but not yet
		 *      armed for prefetch -- hit this on every full read).
		 *   2. The request's own max_len -- NOT "0 = no cap" (an earlier
		 *      version of this comment claimed that; wrong, NIT, host review
		 *      1118c99): the TI HAL's cc3501e_hw_sock_recv()
		 *      (hal/ti/cc3501e_hw_ti_sock.c) computes
		 *      `want = (max_len < cap) ? max_len : cap`, so max_len == 0
		 *      makes it read ZERO bytes there, not "whatever the wire
		 *      ceiling allows" -- a host is expected to always send its own
		 *      receive-buffer's real capacity (nonzero) per
		 *      alp_cc3501e_sock_recv_t's own "max_len: host receive-buffer
		 *      capacity for this request" field doc.  Bounding by it here
		 *      regardless keeps lwip_recvfrom() from EVER being asked for more
		 *      than the host itself requested.  NOT the same as the ring fast
		 *      path's room computation (protocol_sockets.c's handle_sock_recv(),
		 *      NIT, host review of c354208 -- an earlier version of this comment
		 *      claimed they mirrored each other; they do not): the ring treats
		 *      max_len == 0 as "no cap beyond the reply buffer" (its own
		 *      `if (max_len != 0u && room > max_len) room = max_len` leaves
		 *      `room` at the buffer's own size when max_len is 0), while this
		 *      path's cc3501e_hw_sock_recv() call passes max_len straight
		 *      through, and the TI HAL's own `want = (max_len < cap) ? max_len :
		 *      cap` then reads EXACTLY ZERO bytes for max_len == 0.  A
		 *      worker-path recv with max_len 0 therefore always reports 0 bytes,
		 *      where a ring-served one on the same request would return up to a
		 *      full buffer -- a real behavioural difference between the two
		 *      paths for that one input, not a bug this change introduces or
		 *      fixes.
		 *
		 * With both bounds in place, cc3501e_hw_sock_recv() can never report
		 * more than the reply -- and this cache's own
		 * g_sock_recv_wk_reply[CC3501E_REPLY_DATA_MAX] -- can hold; the
		 * defensive guard in protocol_sock_recv_worker_publish() and
		 * worker_poll()'s own truncation guard below are both then
		 * unreachable for this opcode, kept only as loud backstops. */
		const uint16_t handle   = wk_get_le16(job.req, 0u);
		const uint16_t max_len  = wk_get_le16(job.req, 2u);
		uint16_t       data_cap = (uint16_t)(CC3501E_REPLY_DATA_MAX - WK_SOCK_RECV_HDR);
		if (max_len != 0u && max_len < data_cap) {
			data_cap = max_len;
		}
		uint8_t  from_addr[4] = { 0 };
		uint16_t from_port    = 0u;
		uint16_t recv_len     = 0u;
		rv                    = cc3501e_hw_sock_recv(
		    handle, max_len, &buf[WK_SOCK_RECV_HDR], data_cap, &recv_len, from_addr, &from_port);
		if (rv == CC3501E_HW_OK) {
			/* Build the from sock_addr (family | reserved | port(LE16) | addr[16]);
			 * STREAM leaves it zeroed (recv fills only for DGRAM). */
			memset(buf, 0, WK_SOCK_RECV_HDR);
			buf[0] = (uint8_t)ALP_CC3501E_SOCK_FAMILY_IPV4;
			wk_put_le16(&buf[2], from_port);
			for (unsigned i = 0u; i < 4u; ++i)
				buf[4u + i] = from_addr[i];
			wk_put_le16(&buf[20], recv_len); /* data_len */
			len = (size_t)WK_SOCK_RECV_HDR + (size_t)recv_len;
		}
		break;
	}
	case ALP_CC3501E_CMD_SOCK_CLOSE: {
		/* job.req = alp_cc3501e_sock_close_t: handle(LE16 @0) | reserved(@2). */
		rv = cc3501e_hw_sock_close(wk_get_le16(job.req, 0u));
		break;
	}
	case ALP_CC3501E_CMD_SOCK_BIND: {
		/* job.req = alp_cc3501e_sock_bind_t -- the SOCK_CONNECT layout with the
		 * LOCAL endpoint: handle(LE16 @0) | reserved(@2) | local sock_addr @4
		 * { family(@4) | reserved(@5) | port(LE16 @6) | addr[16] @8 }.  An
		 * all-zero addr[0..3] is INADDR_ANY. */
		const uint16_t handle = wk_get_le16(job.req, 0u);
		const uint8_t  family = job.req[4];
		const uint16_t port   = wk_get_le16(job.req, 6u);
		uint8_t        addr[4];
		for (unsigned i = 0u; i < 4u; ++i)
			addr[i] = job.req[8u + i];
		rv = cc3501e_hw_sock_bind(handle, family, port, addr);
		break;
	}
	case ALP_CC3501E_CMD_SOCK_LISTEN: {
		/* job.req = alp_cc3501e_sock_listen_t: handle(LE16 @0) | backlog(@2) |
		 * reserved(@3).  Returns as soon as the socket is passive; inbound
		 * connections are accepted on the tick, not here -- a blocking accept in
		 * the drain would hold READY LOW until a client happened to connect. */
		rv = cc3501e_hw_sock_listen(wk_get_le16(job.req, 0u), job.req[2]);
		break;
	}
	case ALP_CC3501E_CMD_SPI1_CONFIGURE: {
		/* job.req = alp_cc3501e_spi1_configure_t: freq_hz(LE32 @0) | mode(@4) |
		 * bits_per_word(@5) | cs(@6) | reserved(@7); protocol_spi.c validated it.
		 * SPI_open() can block on a power domain, which is why this is here and
		 * not inline in the SPI0 slave callback.  Reply DATA =
		 * alp_cc3501e_spi1_config_resp_t: the ACTUAL divider output, this
		 * firmware's max chunk, and the accepted word size. */
		uint32_t actual_freq_hz = 0u;
		rv                      = cc3501e_hw_spi1_configure(
		    wk_get_le32(job.req, 0u), job.req[4], job.req[5], job.req[6], &actual_freq_hz);
		if (rv == CC3501E_HW_OK) {
			wk_put_le32(buf, actual_freq_hz);
			/* Report CC3501E_SPI1_MAX_XFER_V4 (protocol.h), not the header's
			 * ALP_CC3501E_SPI1_MAX_XFER -- wire MAJOR 4's mandatory request CRC
			 * trailer costs 2 bytes of the same payload_len ceiling a maxed chunk
			 * already saturates, so the host must chunk 2 bytes smaller or its
			 * last chunk truncates silently at exactly the largest transfer. */
			wk_put_le16(&buf[4], (uint16_t)CC3501E_SPI1_MAX_XFER_V4);
			buf[6] = job.req[5];
			buf[7] = 0u;
			len    = 8u;
		}
		break;
	}
	case ALP_CC3501E_CMD_SPI1_TRANSFER: {
		/* job.req = alp_cc3501e_spi1_transfer_t: len(LE16 @0) | flags(@2) |
		 * seq(@3) | tx_fill(@4) | reserved(@5..7), then the TX bytes inline at 8
		 * (absent when NO_TX).  protocol_spi.c already checked the payload length
		 * EXACTLY, so the bytes past the header are this request's, not a longer
		 * previous job's leftovers in job.req.
		 *
		 * NO_TX / NO_RX collapse into NULL pointers, the same convention TI's
		 * SPI_Transaction already uses -- no separate HAL flag argument.  The RX
		 * bytes land straight at buf[4], i.e. immediately after the reply header,
		 * so a 4 KB chunk is never copied twice. */
		const uint16_t xfer_len = wk_get_le16(job.req, 0u);
		const uint8_t  flags    = job.req[2];
		const bool     no_rx    = (flags & ALP_CC3501E_SPI1_XFER_NO_RX) != 0u;
		const bool     no_tx    = (flags & ALP_CC3501E_SPI1_XFER_NO_TX) != 0u;
		const bool     cs_hold  = (flags & ALP_CC3501E_SPI1_XFER_CS_HOLD) != 0u;

		rv = cc3501e_hw_spi1_transfer(no_tx ? NULL : (const uint8_t *)&job.req[8],
		                              no_rx ? NULL : &buf[4],
		                              xfer_len,
		                              job.req[4],
		                              cs_hold);
		if (rv == CC3501E_HW_OK) {
			/* Self-delimiting reply: the declared payload_len includes
			 * protocol_build_reply's zero pad, so the RX count has to ride in the
			 * data itself or the host walks pad bytes as data (alp-sdk#1740). */
			wk_put_le16(buf, no_rx ? 0u : xfer_len);
			buf[2] = (uint8_t)(cs_hold ? ALP_CC3501E_SPI1_XFER_CS_HOLD : 0u); /* echoes CS_HOLD */
			buf[3] = job.req[3];                                              /* echo seq    */
			len    = 4u + (no_rx ? 0u : (size_t)xfer_len);
		}
		break;
	}
	case ALP_CC3501E_CMD_SPI1_RELEASE:
		/* Deassert CS, SPI_close() the instance, free the bus.  Argless, and OK
		 * with nothing open -- it is the escape hatch out of a lost CS_HOLD
		 * chain, so it must never fail on state. */
		rv = cc3501e_hw_spi1_release();
		break;
	default:
		rv = CC3501E_HW_ERR_NOTIMPL;
		break;
	}

	/* Defensive clamp before publish: every HAL body above is CONTRACTED to
	 * report len <= ALP_CC3501E_MAX_PAYLOAD (it fills buf, which is exactly
	 * that size), but a misbehaving backend that reports a larger len must
	 * corrupt at most its own answer -- never overrun job.result[] and smash
	 * the worker state the SPI ISR reads.  Truncation is safe to publish:
	 * the poller copies min(out_cap, result_len) and the protocol layer
	 * length-checks every reply. */
	if (len > sizeof(job.result)) {
		len = sizeof(job.result);
	}

	/* SOCK_RECV-ONLY: invalidate protocol_sockets.c's worker-fallback cache
	 * FIRST, in its OWN short critical section, before either ~4 KB copy
	 * below (MINOR, host review, 1118c99).  The ORIGINAL shape did both the
	 * job.result copy AND the cache's own copy while holding ONE critical
	 * section across both -- on real silicon worker_critical_enter() is
	 * __disable_irq(), so that held the SPI-ISR-sensitive link's interrupts
	 * masked for the time of an ~8 KB memcpy, once per worker-routed recv.
	 * Clearing the cache here means a dispatch that lands in the gap between
	 * this critical section and the final one below sees a clean cache MISS
	 * (not a half-updated entry) -- see worker.h's block comment on
	 * protocol_sock_recv_worker_invalidate/_copy/_publish for the full
	 * 3-step argument. */
	if (cmd == ALP_CC3501E_CMD_SOCK_RECV) {
		const unsigned long inv_key = worker_critical_enter();
		protocol_sock_recv_worker_invalidate();
		worker_critical_exit(inv_key);
	}

	/* The actual byte copies -- OUTSIDE any critical section, interrupts
	 * enabled throughout.
	 *
	 * job.result's copy is safe here for the SAME reason this file's own top
	 * comment already states: "result[]/result_len/err are only WRITTEN
	 * while state is QUEUED/RUNNING (the ISR never reads them then -- it
	 * sees < DONE and replies BUSY), and only READ once state == DONE/ERR".
	 * job.state is still WORKER_RUNNING for the whole of this function (it
	 * was set QUEUED->RUNNING before worker_execute() was ever called, and
	 * does not flip to DONE/ERR until the critical section below), so
	 * nothing reads job.result until that flip publishes it -- writing it
	 * unprotected, before the flip, is exactly the release-pattern the
	 * ORIGINAL single critical section already relied on, just with the
	 * write moved earlier and outside the lock.
	 *
	 * g_sock_recv_wk_reply's copy (protocol_sock_recv_worker_copy()) is safe
	 * by the identical argument using g_sock_recv_wk_cached instead of
	 * job.state: the critical section just above already published
	 * g_sock_recv_wk_cached = false, so nothing reads that buffer until this
	 * function's OWN final critical section below publishes cached = true. */
	if (rv == CC3501E_HW_OK) {
		memcpy((void *)job.result, buf, len);
	}
	if (cmd == ALP_CC3501E_CMD_SOCK_RECV && rv == CC3501E_HW_OK) {
		protocol_sock_recv_worker_copy(buf, len);
	}

	/* Publish everything ELSE atomically wrt the SPI ISR, in ONE final short
	 * critical section: job.result was already written above, so this
	 * section is back down to small scalar stores (job.result_len/err/state,
	 * SOCK_SEND's own <=2 B cache copy, and SOCK_RECV's cache scalars) --
	 * state flips LAST so a poller never sees DONE with stale/partial
	 * bytes. */
	const unsigned long key = worker_critical_enter();
	/* SOCK_SEND-ONLY: publish into protocol_sockets.c's #88 seq-keyed reply
	 * cache in this SAME critical section, strictly BEFORE job.state flips
	 * below -- see protocol_sock_send_on_worker_complete()'s doc comment
	 * (worker.h) for why the ordering is load-bearing, not cosmetic: it is
	 * what stops worker_poll()'s orphan-discard arm (a DIFFERENT opcode's
	 * poll, possibly on a different host thread) from ever observing this
	 * job as terminal before the cache entry a same-seq re-issue would need
	 * already exists.  Its OWN copy is at most 2 B, cheap enough to stay
	 * inside this critical section unlike SOCK_RECV's -- see worker.h. */
	if (cmd == ALP_CC3501E_CMD_SOCK_SEND) {
		protocol_sock_send_on_worker_complete(
		    job.req[offsetof(alp_cc3501e_sock_send_t, seq)], rv, buf, len);
	}
	/* SOCK_RECV-ONLY: the FINAL step of the 3-step split above -- small
	 * scalars only (the byte copy already happened, unprotected, above).
	 * The handle rides in job.req (unlike SOCK_SEND's seq, no separate
	 * capture needed for it), computed identically to the SOCK_RECV case
	 * above (wk_get_le16(job.req, 0u)) -- recomputed here rather than
	 * threading it out of the switch, since this call must stay inside this
	 * one critical section regardless of which case ran. */
	if (cmd == ALP_CC3501E_CMD_SOCK_RECV) {
		protocol_sock_recv_worker_publish(wk_get_le16(job.req, 0u), rv, len);
	}
	if (rv == CC3501E_HW_OK) {
		job.result_len = (uint16_t)len;
		job.err        = 0;
		job.state      = WORKER_DONE;
	} else {
		job.result_len = 0u;
		job.err        = (int8_t)rv;
		job.state      = WORKER_ERR;
	}
	worker_critical_exit(key);
}

void worker_init(void)
{
	const unsigned long key = worker_critical_enter();
	job.state               = WORKER_IDLE;
	job.job_cmd             = 0u;
	job.result_len          = 0u;
	job.err                 = 0;
	worker_critical_exit(key);
}

int worker_submit(uint8_t cmd)
{
	const unsigned long key = worker_critical_enter();
	if (job.state != WORKER_IDLE) {
		worker_critical_exit(key);
		return 0; /* a job is already in flight (single in-flight, v0.2) */
	}
	job.job_cmd    = cmd;
	job.result_len = 0u;
	job.err        = 0;
	job.state      = WORKER_QUEUED;
	worker_critical_exit(key);

#ifndef CC3501E_WIFI
	/* SILICON-FREE / stub / native backend: no main-loop drain runs in the
	 * host ztests, and the HAL body is NOTIMPL (non-blocking).  Run the job
	 * SYNCHRONOUSLY here so the result is ready on the caller's next poll --
	 * the host re-issues GET_MAC once and gets the (NOT_READY) answer.  The
	 * REAL (CC3501E_WIFI) build leaves the job QUEUED for worker_run_pending,
	 * so the SPI ISR is never blocked by the seconds-long Wlan_* body. */
	job.state = WORKER_RUNNING;
	worker_execute(cmd);
#endif
	return 1;
}

int worker_submit_payload(uint8_t cmd, const uint8_t *payload, uint16_t len)
{
	if (len > ALP_CC3501E_MAX_PAYLOAD) {
		return 0; /* would overflow job.req -- caller validated, but stay defensive */
	}
	const unsigned long key = worker_critical_enter();
	if (job.state != WORKER_IDLE) {
		worker_critical_exit(key);
		return 0; /* a job is already in flight (single in-flight, v0.2) */
	}
	job.job_cmd    = cmd;
	job.result_len = 0u;
	job.err        = 0;
	if (len > 0u && payload != NULL) {
		memcpy((void *)job.req, payload, len);
	}
	job.req_len = len;
	job.state   = WORKER_QUEUED;
	worker_critical_exit(key);

#ifndef CC3501E_WIFI
	/* Same synchronous stub path as worker_submit (see there). */
	job.state = WORKER_RUNNING;
	worker_execute(cmd);
#endif
	return 1;
}

enum worker_state
worker_poll(uint8_t cmd, uint8_t *out, size_t out_cap, size_t *out_len, int8_t *err)
{
	if (out_len != NULL) *out_len = 0u;
	if (err != NULL) *err = 0;

	const unsigned long     key = worker_critical_enter();
	const enum worker_state st  = job.state;

	/* No job, or a job for a DIFFERENT cmd: report IDLE so the caller can
	 * submit (or, if a different job is mid-flight, treat it as busy).
	 *
	 * TERMINAL RESULT FOR ANOTHER OPCODE = UNCLAIMABLE -- DISCARD IT.  Only a
	 * poll carrying the SAME opcode collects a DONE/ERR and calls worker_reset()
	 * (protocol.c), so a result nobody comes back for pinned the single job slot
	 * FOREVER: from then on every other worker-routed opcode fell into the
	 * "other cmd busy" arm below and answered RESP_ERR_BUSY, with nothing in the
	 * system able to clear it short of a reset.  That is reachable in ordinary
	 * use -- a host that gives up on CMD_SOCK_RECV after its timeout and then
	 * issues any other worker-routed op strands the finished recv result and
	 * wedges the bridge for everything else.
	 *
	 * A terminal state means the worker is NOT running, so the slot is free to
	 * reuse: drop the orphaned result and report IDLE so the new opcode can
	 * submit.  Only QUEUED/RUNNING is genuinely busy. */
	if (st == WORKER_IDLE || job.job_cmd != cmd) {
		enum worker_state ret = WORKER_IDLE;
		if (job.job_cmd != cmd && st != WORKER_IDLE) {
			if (st == WORKER_DONE || st == WORKER_ERR) {
				job.state      = WORKER_IDLE;
				job.job_cmd    = 0u;
				job.result_len = 0u;
				job.err        = 0;
			} else {
				ret = WORKER_RUNNING; /* other cmd genuinely in flight */
			}
		}
		worker_critical_exit(key);
		return ret;
	}

	if (st == WORKER_DONE) {
		if (job.result_len > out_cap) {
			/* NEVER silently truncate a payload reply.  job.result_len larger
			 * than the CALLER's actual reply capacity means some opcode's own
			 * cap (worker_execute()'s switch) let a HAL body read or produce
			 * more than the reply frame can ever carry -- the general form of
			 * the SOCK_RECV data-loss class (host review): a truncating
			 * memcpy here used to silently drop the overrun and report OK
			 * with a short reply, and for a socket recv those bytes were
			 * already irrecoverably consumed from lwIP.  Report a real error
			 * instead.  Every worker-routed opcode's own cap is now bounded
			 * by CC3501E_REPLY_DATA_MAX at the source (worker_execute()'s
			 * SOCK_RECV / WIFI_SCAN_START / BLE_SCAN_START / BLE_GATT_READ
			 * cases), so this should never actually fire -- it is a LOUD
			 * backstop, not a silent one, for any future opcode that gets its
			 * own cap wrong. */
			if (out_len != NULL) *out_len = 0u;
			if (err != NULL) *err = CC3501E_HW_ERR_NO_MEM;
			worker_critical_exit(key);
			return WORKER_ERR;
		}
		const size_t n = job.result_len;
		if (out != NULL && n > 0u) memcpy(out, (const void *)job.result, n);
		if (out_len != NULL) *out_len = n;
		worker_critical_exit(key);
		return WORKER_DONE;
	}
	if (st == WORKER_ERR) {
		if (err != NULL) *err = job.err;
		worker_critical_exit(key);
		return WORKER_ERR;
	}

	/* QUEUED or RUNNING: still in flight. */
	worker_critical_exit(key);
	return st;
}

void worker_reset(void)
{
	const unsigned long key = worker_critical_enter();
	job.state               = WORKER_IDLE;
	job.job_cmd             = 0u;
	job.result_len          = 0u;
	job.err                 = 0;
	worker_critical_exit(key);
}

int worker_discard_stale_terminal(uint8_t cmd, size_t req_off, uint8_t req_byte)
{
	const unsigned long key       = worker_critical_enter();
	int                 discarded = 0;
	if (job.job_cmd == cmd && (job.state == WORKER_DONE || job.state == WORKER_ERR) &&
	    req_off < (size_t)job.req_len && job.req[req_off] != req_byte) {
		job.state      = WORKER_IDLE;
		job.job_cmd    = 0u;
		job.result_len = 0u;
		job.err        = 0;
		discarded      = 1;
	}
	worker_critical_exit(key);
	return discarded;
}

int worker_reclaim_matching_terminal(uint8_t cmd, size_t req_off, uint8_t req_byte)
{
	const unsigned long key       = worker_critical_enter();
	int                 reclaimed = 0;
	if (job.job_cmd == cmd && (job.state == WORKER_DONE || job.state == WORKER_ERR) &&
	    req_off < (size_t)job.req_len && job.req[req_off] == req_byte) {
		job.state      = WORKER_IDLE;
		job.job_cmd    = 0u;
		job.result_len = 0u;
		job.err        = 0;
		reclaimed      = 1;
	}
	worker_critical_exit(key);
	return reclaimed;
}

int worker_discard_stale_recv(void)
{
	const unsigned long key       = worker_critical_enter();
	int                 discarded = 0;
	if (job.job_cmd == ALP_CC3501E_CMD_SOCK_RECV &&
	    (job.state == WORKER_DONE || job.state == WORKER_ERR)) {
		job.state      = WORKER_IDLE;
		job.job_cmd    = 0u;
		job.result_len = 0u;
		job.err        = 0;
		discarded      = 1;
	}
	worker_critical_exit(key);
	return discarded;
}

void worker_run_pending(void)
{
	/* Promote QUEUED -> RUNNING atomically so the ISR can't double-submit
	 * while we execute; the long HAL body then runs OUTSIDE the lock. */
	const unsigned long key = worker_critical_enter();
	uint8_t             cmd = 0u;
	int                 go  = 0;
	if (job.state == WORKER_QUEUED) {
		job.state = WORKER_RUNNING;
		cmd       = job.job_cmd;
		go        = 1;
	}
	worker_critical_exit(key);

	if (go) {
		cc3501e_bridge_busy(); /* radio op about to kill the slave DMA -> hold host off */
		/* Whether the slave is armed for the host's next clock.  Starts true:
		 * the ops that SKIP the re-init below UNCONDITIONALLY (the socket data
		 * and control ops, the SPI1 passthrough ops, and the two BLE ops listed
		 * there) never tore the slave down here, so their READY raise stays
		 * unconditional as before.  Issue #5.
		 *
		 * WIFI_GET_RSSI and WIFI_CONNECT_STA are NOT in that unconditional
		 * group, for two different reasons -- neither is "never tore the slave
		 * down" the way sockets/SPI1/the two BLE ops are.  RSSI's HAL body DOES
		 * make one Wlan_Get() NWP call; hal/ti/transport_hw_ti_spi.c's file
		 * header used to claim (uncorrected, see the dated correction there)
		 * that a short Wlan_Get disrupts the bridge SPI's DMA the same as
		 * Wlan_Start.  RSSI's skip below is CONDITIONAL (only when Wi-Fi was
		 * already running when the run began) and UNMEASURED by bench, though a
		 * 2026-09-13 SDK source audit supports it -- see rssi_read below.
		 * CONNECT_STA's SUCCESS path re-arms the slave
		 * ITSELF and hands this drain the real outcome via
		 * cc3501e_hw_wifi_connect_sta_take_reinit() below, which OVERWRITES
		 * this starting `true` with that outcome -- so a body reinit that
		 * failed to arm still leaves `rearmed` false here.  Both are exempt (in
		 * their respective conditions) from paying a SECOND reinit, not from
		 * tracking the real state.
		 *
		 * Known ceiling of the unconditional-skip group: if an interrupt-side
		 * re-arm fails DURING a long skipped body (arm_transfer leaves READY
		 * low and bumps g_arm_fail_count), the unconditional raise below
		 * reports an armed slave that is not, until the tick's arm-fail
		 * self-heal re-inits it.  Harmless on a board whose READY is an
		 * unconnected net; the data-op and SPI1 skips have always carried the
		 * same ceiling. */
		bool rearmed = true;
		worker_execute(cmd); /* may block for seconds (Wlan_* init + get) */

		/* Radio<->SPI coexistence fix: the worker body just ran a radio HAL op
		 * (a Wlan_* call), during which the bridge SPI slave could not be
		 * serviced -- so the host may have read 0x00000000 / drifted byte
		 * alignment.  Request a clean slave re-sync now that the op is done and
		 * BEFORE the next host poll lands (worker_execute has already published
		 * DONE/ERR above, but the host re-issues GET_MAC to collect it).  This is
		 * the weak transport hook: a no-op on the stub/native build (no radio,
		 * no SPI slave), the real FIFO-flush + poll-loop re-arm on the ti
		 * backend (hal/ti/transport_hw_ti_spi.c). */
		/* SKIP for the socket DATA ops.  This re-init exists for RADIO ops: a
		 * Wlan_* call kills the slave's DMA, so the link must be re-established
		 * afterwards.  cc3501e_hw_sock_recv / _send are lwIP buffer operations
		 * (lwip_recvfrom / lwip_send) and make no such call.
		 *
		 * Paying it anyway is not merely wasted time, it is destructive: the
		 * cc3501e_bridge_busy() / cc3501e_bridge_ready() bracket around it is the
		 * only thing telling the host "do not clock now", and READY is an OPEN
		 * CONNECTION on this board (0 edges in 20000 samples), so the host ignores
		 * it and clocks straight into a slave that is being closed and re-opened.
		 *
		 * Silicon-measured 2026-08-24 at a 256 B reply cap, same host image, only
		 * this branch differing: WITH the skip a 262144 B stream runs at
		 * ~25.4 kB/s with zero misses; WITHOUT it the same stream collapses to
		 * 3-22 B/s.  (An earlier "this changes nothing" reading was wrong -- both
		 * sides of that comparison had the skip.)
		 *
		 * The socket CONTROL ops are now exempt too -- see socket_control below.
		 * This comment used to call keeping the re-init on OPEN / CONNECT / CLOSE
		 * "deliberately conservative, because connect can drive the stack hard
		 * enough to touch the HIF".  That was never measured, and it did not hold
		 * up: SEND drives far more traffic through the stack than any control op,
		 * and it runs with no re-init at all at the rate quoted above. */
		/* Re-assert BUSY immediately before the re-init.  The bracket taken above
		 * has almost certainly been released by now: every BLE HAL body ends with
		 * its own cc3501e_bridge_ready() (cc3501e_hw_ti_ble.c:116, :133, :166,
		 * :179, :206, :245, :263, :278, :326, :345, :362), and the hooks are raw
		 * level writes -- so that inner ready() cancels this drain's outer busy()
		 * and the SPI_close/SPI_open below would otherwise run with the line HIGH
		 * and the host free to clock.  That is the alp-sdk#1691 condition the
		 * comments say must never happen, and it made
		 * hal/ti/cc3501e_hw_ti_sock.c:57's claim that "worker_run_pending() holds
		 * READY LOW across the whole job" false for every BLE opcode.
		 *
		 * A depth-counted bracket is the tempting fix and is UNSOUND here: these
		 * same hooks are driven per-SPI-phase from the transfer-complete callback
		 * (transport_hw_ti_spi.c on_transfer -> busy(), arm_transfer -> ready()),
		 * and arm_transfer's g_arm_fail_count path returns WITHOUT its ready() --
		 * so every failed arm would leak one level of depth and the line would
		 * eventually never rise again.  Re-asserting here needs no shared counter
		 * and no cross-context atomicity.  Issue #5. */
		/* SKIP the drain's re-init only where a re-init here would be WRONG.  The
		 * list below is per-opcode and each entry has a DIFFERENT reason, so it must
		 * not be read as one rule:
		 *
		 *   BLE_DISCONNECT  -- its HAL body ends with bridge_transport_spi_hw_reinit(),
		 *      so paying the drain's too means TWO back-to-back SPI_close/SPI_open
		 *      cycles, each re-rolling the 12-attempt SPI_open this transport calls the
		 *      roll where "the first roll that loses killed the link permanently".
		 *      Silicon-measured on E1M-AEN801, same host image, only this branch
		 *      differing: with the double re-init `ble scan-stop` wedged every time
		 *      ("scan stopped" then get_version -5); with the skip it did not.
		 *
		 *      TREAT THAT MECHANISM AS UNPROVEN (#60).  The same control run had two
		 *      OTHER worker-routed BLE ops pay the identical double re-init and NOT
		 *      wedge, and the exemption was applied to 2 of the 11 opcodes with that
		 *      shape.  So "two back-to-back SPI_close/SPI_open wedges the link" does
		 *      not follow from the evidence offered for it -- the correlation with
		 *      scan-stop specifically is what was observed.  #48 later measured the
		 *      scan-stop re-init itself as the wedge and removed it, which fits the
		 *      observation better than the double-re-init story does.  The skip is
		 *      kept for BLE_DISCONNECT because paying a re-init twice is pointless
		 *      regardless, not because the wedge mechanism is established.
		 *
		 *   BLE_SCAN_STOP   -- skipped for the OPPOSITE reason.  #48 REMOVED the
		 *      re-init from cc3501e_hw_ble_scan_stop() entirely, having measured that
		 *      the re-init WAS the wedge (3/32 vs 1/104 booted trials).  So this path
		 *      must have no re-init from the body AND none from the drain.  The comment
		 *      here used to say "its HAL body ALREADY ends with a re-init", which #48
		 *      made false -- and a well-meaning cleanup of that stale line would have
		 *      restored the drain's re-init and silently reverted the fix.
		 *
		 * WIFI_DISCONNECT WAS in this list, was REMOVED for the reason below, and
		 * (#106 run9) is BACK ON IT below under wifi_disconnect -- for a THIRD,
		 * different reason than either the old entry or the removal.  History,
		 * kept rather than deleted:
		 *
		 * It was in this list once already, asserting (like BLE_SCAN_STOP /
		 * BLE_DISCONNECT) that its body ALREADY reinit.  It did not: its HAL
		 * body cc3501e_hw_wifi_disconnect() contains NO re-init at all -- it
		 * calls Wlan_Disconnect() and returns -- so skipping the drain's left a
		 * Wlan_* radio op with no SPI re-sync from either side, which is the gap
		 * the removal fixed.  It then took the drain's re-init unconditionally.
		 *
		 * CORRECTED (2026-09-14, #106 run9): that removal's premise -- that
		 * EVERY Wlan_* call needs a re-sync from somewhere, body or drain, or
		 * else there is "a gap" -- is the same "every Wlan_* kills the DMA"
		 * assumption the RSSI source audit already refuted for Wlan_Get.  Traced
		 * the same way for Wlan_Disconnect (see wifi_disconnect below): its
		 * synchronous execution posts an RTOS message and returns, touching no
		 * DMA/SPI/interrupt-mask at all, so there was never a "gap" to close on
		 * that call in the first place -- paying the drain's reinit after it is
		 * the SAME destructive no-op the socket/SPI1 groups warn about, not a
		 * fix for a real one.  The 2026-09-14 exemption below does not repeat
		 * the earlier bug (a static "body already reinit" claim that was false):
		 * it rests on "body makes no DMA-affecting call", sourced this time. */
		/* KEEPING THIS LIST IN SYNC IS MANUAL, AND IT HAS ALREADY DRIFTED ONCE (#61).
		 * The predicate below restates, here, a fact that actually lives in each HAL
		 * body -- whether that body calls bridge_transport_spi_hw_reinit().  Nothing
		 * enforces the correspondence: #48 removed the re-init from
		 * cc3501e_hw_ble_scan_stop() and this list kept asserting the body still had
		 * one, and WIFI_DISCONNECT sat here for its whole life with no re-init in its
		 * body at all.  If you add or remove a bridge_transport_spi_hw_reinit() in any
		 * hal/ti/cc3501e_hw_ti_*.c body, RE-CHECK THIS LIST in the same change.
		 *
		 * The socket and SPI1 exemptions rest on the OTHER fact: that the body makes
		 * no Wlan_* call.  Adding one to any cc3501e_hw_sock_* or cc3501e_hw_spi1_*
		 * body -- a Wlan_Get for RSSI in connect, say -- silently leaves that radio
		 * op with no re-sync.  Re-check this list for that too.
		 *
		 * WIFI_GET_RSSI (#106) is the counter-example to that OTHER fact: its body
		 * DOES call Wlan_Get(WLAN_GET_RSSI) and is still exempt below.  It rests on
		 * neither the "no radio op" fact nor a "body already reinit" fact -- it is
		 * exempt because paying the re-init AFTER it is what wedged associated
		 * boots, measured directly (see rssi_read below).  Do not fold it into the
		 * socket/SPI1 "no Wlan_* call" reasoning above, and do not assume a future
		 * cc3501e_hw_sock_* or cc3501e_hw_spi1_* body picking up a Wlan_Get gets
		 * the same pass for free -- that would need its own measurement, same as
		 * this one.
		 *
		 * WIFI_DISCONNECT (#106 run9) straddles BOTH facts rather than fitting
		 * either alone: cc3501e_hw_wifi_disconnect() makes NO Wlan_* call at all
		 * when Wi-Fi was never started (the socket/SPI1 fact, unconditionally
		 * true for that branch), and when it WAS started its one Wlan_Disconnect()
		 * call is sourced safe the same way WIFI_GET_RSSI's Wlan_Get is (see
		 * wifi_disconnect below) -- unlike RSSI, that source trace found NO DMA
		 * traffic on the call at all, not merely DMA traffic proven not to
		 * collide.  Re-check wifi_disconnect's own comment, not this paragraph,
		 * if Wlan_Disconnect's implementation ever changes.
		 *
		 * WIFI_CONNECT_STA has TWO body reinits, and only the SECOND one changed
		 * this list.  The FIRST (between the STA role-up and Wlan_Connect, gated on
		 * role_up_was_latched -- see cc3501e_hw_wifi_connect_sta) predates #106 and
		 * does not appear here: the whole association after it -- Wlan_Connect, the
		 * 30 s event wait, DHCP, and every FAILURE exit's Wlan_Disconnect cleanup --
		 * still runs afterward and still needs a reinit from somewhere, which stays
		 * this drain's job for every one of those exits.
		 *
		 * The SECOND (#106, right before the body's SUCCESS-path wifi_conn_set
		 * (CONNECTED)) is what wifi_connect_body_reinit below skips.  Unlike
		 * BLE_SCAN_STOP / BLE_DISCONNECT it is not a static per-opcode fact -- it is
		 * signalled PER RUN by cc3501e_hw_wifi_connect_sta_take_reinit(), because
		 * only the SUCCESS exit takes that reinit.  Do NOT fold WIFI_CONNECT_STA
		 * into a static `cmd ==` entry here: that would wrongly skip the drain's
		 * reinit on every FAILURE exit too, which still needs it exactly as before
		 * #106.
		 *
		 * Same static-exemption caution applies to WIFI_SCAN_START, which has had a
		 * body reinit (between its own role-up and Wlan_Scan) since long before this
		 * list and is NOT on it: its own post-body drain reinit is still required. */
		bool       armed_by_connect_body = false;
		const bool wifi_connect_body_reinit =
		    (cmd == ALP_CC3501E_CMD_WIFI_CONNECT_STA) &&
		    cc3501e_hw_wifi_connect_sta_take_reinit(&armed_by_connect_body);
		if (wifi_connect_body_reinit) {
			/* Trust the body's own arm outcome over the optimistic `true` this
			 * function started with -- see the comment on `rearmed`'s declaration. */
			rearmed = armed_by_connect_body;
		}
		const bool body_already_reinit = (cmd == ALP_CC3501E_CMD_BLE_SCAN_STOP) ||
		                                 (cmd == ALP_CC3501E_CMD_BLE_DISCONNECT) ||
		                                 wifi_connect_body_reinit;
		/* SPI1 host passthrough is exempt for the SAME reason as the two socket
		 * data ops above, and it is the cleanest case in the list: these opcodes
		 * drive a SEPARATE MASTER instance (GPIO_31/32/33/34 + GPIO_15) and make
		 * no Wlan_* call at all, so the SPI0 slave's DMA was never killed and
		 * there is nothing to re-establish.  Paying the re-init anyway would
		 * close and re-open a perfectly live slave behind a busy/ready bracket
		 * that only works when the READY pad's input-enable pinctrl group is
		 * populated (alp-sdk chips/cc3501e/cc3501e_sockets.c, silicon-measured
		 * 2026-08-24) -- on a board without it READY reads stuck low, i.e.
		 * exactly the destructive no-op measured on the socket path.  It would
		 * also re-roll the 12-attempt SPI_open on every 4 KB chunk of a flash
		 * write, which is the hot loop this family exists for. */
		const bool spi1_passthrough = (cmd == ALP_CC3501E_CMD_SPI1_CONFIGURE) ||
		                              (cmd == ALP_CC3501E_CMD_SPI1_TRANSFER) ||
		                              (cmd == ALP_CC3501E_CMD_SPI1_RELEASE);
		/* Socket CONTROL ops are exempt for the same reason as the data ops.  Their
		 * HAL bodies in hal/ti/cc3501e_hw_ti_sock.c are lwIP calls (lwip_socket,
		 * lwip_connect, lwip_close, lwip_bind, lwip_listen) and the worker makes
		 * no Wlan_* call for them.  CONNECT and CLOSE do put ARP / SYN / FIN / RST
		 * frames on the Wi-Fi transmit path through the netif output function --
		 * but SOCK_SEND drives far more traffic down that same path and has run
		 * without a re-init since 2026-08-24.  BIND and LISTEN are exempt on code
		 * reading alone: neither transmits anything.
		 *
		 * What the re-init cost them, measured on e1m-aen-evk-01 (#106), station
		 * console app, `sock tcp-get` against a LAN host with no listener (OPEN,
		 * CONNECT, CLOSE per call, three calls per boot): 6 of 7 boots wedged --
		 * an op timed out at the host's 15 s budget and the next get_version
		 * answered -5.  Of those 7, only 3 had associated; 2 of the 3 wedged, one
		 * of them (B4) on the FIRST SOCK_OPEN of the boot straight after a good
		 * get_version, before any connect had run.  So the trigger is neither the
		 * long connect block nor AP mode.  An OPEN body takes about a millisecond,
		 * so its re-init lands while the host is still polling at 1-2 ms.
		 *
		 * NOT established by that run: a separate style with ONE call per boot
		 * against an address nothing answers survived 5 of 5, but at the measured
		 * per-call wedge rate that is plausible by chance, so it does not show a
		 * slow-cadence re-init is safe.  Nor does anything yet show a 12-21 s
		 * lwip_connect is survivable WITHOUT the re-init that used to follow it.
		 * The before/after bench run on this change is what settles both. */
		const bool socket_control =
		    (cmd == ALP_CC3501E_CMD_SOCK_OPEN) || (cmd == ALP_CC3501E_CMD_SOCK_CONNECT) ||
		    (cmd == ALP_CC3501E_CMD_SOCK_CLOSE) || (cmd == ALP_CC3501E_CMD_SOCK_BIND) ||
		    (cmd == ALP_CC3501E_CMD_SOCK_LISTEN);
		/* WIFI_GET_RSSI is its OWN group, exempt for a DIFFERENT reason than every
		 * group above, and CONDITIONALLY: only when Wi-Fi was ALREADY started
		 * when this run's body began.  Its HAL body (cc3501e_hw_wifi_get_rssi,
		 * hal/ti/cc3501e_hw_ti_wifi.c) DOES make one Wlan_Get(WLAN_GET_RSSI) NWP
		 * call -- lazy_start (a no-op once Wi-Fi has been started AT ALL; it
		 * checks wifi_started, not the STA role) plus one synchronous interrogate
		 * round trip, milliseconds long -- so this is not a "body makes no Wlan_*
		 * call" case like sockets/SPI1 above.  cc3501e_hw_wifi_get_rssi_take_
		 * reinit_skip() reports the already-started fact for the run that just
		 * completed; if Wi-Fi was NOT yet started, lazy_start() ran Wlan_Start()
		 * and its OWN reinit and threw the result away, so this drain must NOT
		 * skip its own reinit then (see that function and cc3501e_hw.h).
		 *
		 * #106 run6: every associated-boot link wedge (6 of 6) began with a
		 * WIFI_GET_RSSI worker op failing; every RSSI read on a non-wedged boot
		 * succeeded (16 of 16).  run7 isolated the TRIGGER to the drain's re-init
		 * landing inside the host's dense poll window, not the radio call itself:
		 * two host images identical but for the poll_by_repeat backoff floor
		 * (CONFIG_ALP_SDK_CC3501E_POLL_GAP_MIN_MS), each associated boot reading
		 * `wifi status` (which performs an RSSI read) up to 30 times at 1 s spacing
		 * -- floor 1 ms (the default) wedged 7 of 7 associated boots, at reads as
		 * early as 0 and as late as 26 (67 RSSI ops total, all 7 boots); floor 50 ms
		 * wedged 0 of 5 (155 RSSI ops, 30/30 clean each boot).  Wedge signature:
		 * rssi -4 after the host's 10 s budget, then ip -5, then get_version -5 --
		 * the same transport-desync signature the socket_control measurement above
		 * shows for the same mechanism.  BOTH run7 images still re-inited after
		 * every RSSI read, so run7 alone shows the RE-INIT-UNDER-DENSE-POLLING
		 * mechanism, not that skipping the re-init is safe.
		 *
		 * The skip itself (as opposed to the trigger run7 found) is now supported
		 * by a 2026-09-13 reading of TI's SimpleLink Wi-Fi SDK 10.10.01.08 source
		 * -- see the dated correction in hal/ti/transport_hw_ti_spi.c's file
		 * header for the full citation trail.  In short: a Wlan_Get's DMA traffic
		 * is scoped to channel 11 (HOSTDMA_DRIVER_CH_HIF) only, never touches the
		 * bridge's channels 12/13, runs under a plain mutex (not an interrupt
		 * mask, so the bridge's own DMA-completion ISR keeps running), and the
		 * SDK's one GLOBAL DMA reset (DMAWFF3_initHw) is called ONLY by the
		 * bridge's own SPI driver, never by any Wi-Fi source.  This is still a
		 * SOURCE reading, not a bench result: it is UNMEASURED, not established.
		 * Wlan_Start's bench-observed kill (the claim this whole skip descends
		 * from) stands, but its mechanism is UNEXPLAINED by that same source
		 * audit -- so the audit narrows what needs a bench run without settling
		 * it. The pending bench run has two possible outcomes: (a) wedges
		 * disappear even at the 1 ms poll floor once this conditional skip ships,
		 * confirming the skip is safe; or (b) the link still goes dead after an
		 * RSSI read regardless of poll floor, which would mean a Wlan_Get DOES
		 * disturb the slave by some mechanism this source audit missed and the
		 * skip must be reverted. */
		bool       rssi_already_started = false;
		const bool rssi_reported_skip =
		    (cmd == ALP_CC3501E_CMD_WIFI_GET_RSSI) &&
		    cc3501e_hw_wifi_get_rssi_take_reinit_skip(&rssi_already_started);
		const bool rssi_read = rssi_reported_skip && rssi_already_started;
		/* WIFI_DISCONNECT (#106 run9) is its OWN group too, exempt for a
		 * DIFFERENT reason than RSSI even though both call one Wlan_* function.
		 * Its HAL body (cc3501e_hw_wifi_disconnect, hal/ti/cc3501e_hw_ti_wifi.c)
		 * calls Wlan_Disconnect(WLAN_ROLE_STA, NULL) directly -- no lazy_start(),
		 * no Wlan_Start(), no role-up, so there is no RSSI-style "first radio op
		 * of the boot already tried and threw away its own reinit result" hazard
		 * to gate against; if Wi-Fi was never started this body returns OK with
		 * NO Wlan_* call at all (checked: `if (!wifi_started) return
		 * CC3501E_HW_OK;`), which is exactly the "body makes no Wlan_* call"
		 * shape the socket/SPI1 groups already rest on.  So unlike RSSI this
		 * exemption is UNCONDITIONAL -- there is no runtime handoff, because
		 * there is no branch where skipping would be wrong.
		 *
		 * When Wi-Fi WAS started, Wlan_Disconnect() -> CME_WlanDisconnect() is
		 * traced against the SDK source the same way the RSSI audit traced
		 * Wlan_Get (TI SimpleLink Wi-Fi SDK 10.10.01.08,
		 * source/ti/net/wifi_stack/): app_entry/wlan_if.c's Wlan_Disconnect()
		 * (STA path) calls cme/cme.c's CME_WlanDisconnect(), which builds a
		 * cmeMsg_t and calls pushMsg2Queue() -> osi_MsgQWrite() ->
		 * MessageQueueP_post() -- an RTOS message-queue post, synchronously
		 * returning once queued.  The actual disconnect radio work runs LATER,
		 * asynchronously, on the CME task that drains that queue -- outside
		 * this function's (and this worker job's) synchronous window entirely.
		 * The only other calls on this path, set_cond_in_process_wlan_
		 * discconnect()/set_finish_wlan_disconnect() (wlan_if.c), take
		 * wlan_if_lock()/unlock(), which is osi_LockObjLock -- the same mutex
		 * primitive (SemaphoreP_pend in this build's linked adaptation layer,
		 * see hal/ti/transport_hw_ti_spi.c's dated correction) already audited
		 * for RSSI, not an interrupt mask.  So Wlan_Disconnect()'s SYNCHRONOUS
		 * execution touches no DMA, no SPI, and no interrupt masking at all --
		 * a stronger case than RSSI's, whose Wlan_Get is fully synchronous DMA
		 * traffic on channel 11 (merely proven not to collide with the bridge's
		 * channels 12/13).  Here there is no DMA traffic on this path to begin
		 * with.
		 *
		 * #106 run9 (GPE 0.254.9.0, e1m-aen-evk-01): WIFI_DISCONNECT hung ~10 s
		 * then the link returned -4/-5 on 3 of 11 calls -- C-01 (a NON-associated
		 * boot), C-03 (after a successful association), P2-03 (the socket-
		 * throughput app's final disconnect); the other 8 succeeded.  Consistent
		 * with the SAME mechanism as every other exemption in this file: the
		 * drain's SPI_close/SPI_open lands 1-15 ms after submit, inside the
		 * host's dense poll window, and desyncs the transport -- not a DMA
		 * collision from the disconnect call itself, which the trace above shows
		 * never reaches the DMA at all.
		 *
		 * FALSIFIER: if Wlan_Disconnect has some OTHER path to the slave's DMA
		 * this trace missed (e.g. via the async CME-task processing later, or a
		 * side effect this trace did not follow), wedges will move to the op
		 * AFTER a successful disconnect, or a disconnect itself will still wedge
		 * with this skip in place.  Not yet observed; open to a future bench
		 * run, same as the RSSI skip's own falsifier above.
		 *
		 * WIFI_AP_STOP (Wlan_RoleDown) and GET_MAC (lazy_start + a Wlan_Get, the
		 * same shape as RSSI) are UNMEASURED candidates for this same class of
		 * exemption and are deliberately left OFF this list and untouched here --
		 * neither has a source trace or a bench run behind it yet.
		 *
		 * See "WIFI_DISCONNECT WAS in this list and has been REMOVED" above for
		 * why it was taken OFF this list once already, and the 2026-09-14 note
		 * appended there on why that removal's premise does not hold either. */
		const bool wifi_disconnect = (cmd == ALP_CC3501E_CMD_WIFI_DISCONNECT);
		if (cmd != ALP_CC3501E_CMD_SOCK_RECV && cmd != ALP_CC3501E_CMD_SOCK_SEND &&
		    !socket_control && !spi1_passthrough && !body_already_reinit && !rssi_read &&
		    !wifi_disconnect) {
			cc3501e_bridge_busy();
			rearmed = bridge_transport_spi_hw_reinit();
		}

		/* CONNECT / AP_START are FIRE-AND-FORGET at the worker level: their outcome
		 * is mirrored into the HAL connection-status latch (read NON-blocking by
		 * CMD_WIFI_STATUS), so the host never collects their DONE/ERR through this
		 * single-job slot.  Free the slot to IDLE here so a SUBSEQUENT connect can
		 * submit -- otherwise the slot would stay DONE/ERR and the next CONNECT would
		 * re-collect the stale result instead of starting a fresh association.
		 *
		 * CORRECTED (2026-09-13): this used to say resetting here "MUST happen
		 * BEFORE cc3501e_bridge_ready() below", as if THAT ordering were what
		 * closes a stale-pickup race.  It is not, and by the time execution
		 * reaches this line READY has typically ALREADY been raised.  The reinit
		 * that set `rearmed` for this job -- either this cmd's own body
		 * (WIFI_CONNECT_STA's SUCCESS path, cc3501e_hw_wifi_connect_sta) or the
		 * drain's own reinit call just above -- goes through
		 * bridge_transport_spi_hw_reinit() -> spi_open_and_arm() ->
		 * arm_request_header() -> arm_transfer(), and arm_transfer() raises
		 * READY itself, as a side effect, on a successful arm
		 * (hal/ti/transport_hw_ti_spi.c ~572) -- independently of this
		 * function's own `if (rearmed) cc3501e_bridge_ready();` a few lines
		 * down.  On top of that, the SPI ISR's own per-transaction re-arm cycle
		 * (on_transfer's re-arm on every SERVICED request) has typically already
		 * raised READY more than once DURING the body, well before this point --
		 * see the matching correction on cc3501e_hw_wifi_connect_sta()'s own
		 * reinit comment for that case.  So a host CONNECT landing before this
		 * reset had that opportunity before #106 and still does; this reset is
		 * not what stands between it and a stale pickup.
		 *
		 * Resetting the slot HERE remains correct and worth keeping regardless
		 * of READY timing: without it the slot stays DONE/ERR forever (nothing
		 * ever polls CONNECT/AP_START to collect and clear it), which jams every
		 * SUBSEQUENT connect attempt behind a stale result on slot occupancy
		 * alone.  If the stale-CONNECT-pickup race ever needs closing for real,
		 * the fix belongs in worker_execute()'s own publish critical section
		 * (publish CONNECT/AP_START as IDLE there directly instead of DONE/ERR),
		 * not in ordering this reset against cc3501e_bridge_ready().  All the
		 * other worker-routed ops (GET_MAC / SCAN / RSSI / BLE) stay
		 * poll-by-repeat: the host collects their DONE/ERR, which resets the
		 * slot in protocol.c (handle_worker_routed). */
		if (cmd == ALP_CC3501E_CMD_WIFI_CONNECT_STA || cmd == ALP_CC3501E_CMD_WIFI_AP_START) {
			worker_reset();
		}
		/* Raise READY only if the slave is ACTUALLY armed.  arm_transfer()
		 * deliberately leaves the line LOW when SPI_transfer() rejects the
		 * re-arm, so the host's READY gate times out visibly instead of it
		 * clocking a frame into a slave that never latches.  Raising it here
		 * unconditionally threw that away and re-introduced the #1133 lie.
		 * On a failed arm the line stays LOW and cc3501e_hw_tick()'s
		 * g_arm_fail_count self-heal drives the recovery.  Issue #5. */
		if (rearmed) {
			cc3501e_bridge_ready(); /* slave re-armed -> host may clock again */
		}
	}
}
