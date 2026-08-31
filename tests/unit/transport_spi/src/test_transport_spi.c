/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * Unit tests for the cc3501e-bridge SPI-slave transport seams
 * (cc3501e-bridge-firmware:src/transport_spi.c) + the shared framing/dispatch
 * (cc3501e-bridge-firmware:src/protocol.c), exercised against the silicon-free
 * stub HAL backend.  These are the PRODUCTION code paths, not a mock --
 * the same protocol_build_reply() the SDIO transport uses.
 *
 * Wire framing (see <alp/protocol/cc3501e.h>): 4-byte LE header
 * [cmd | flags | payload_len(LE16)] + payload; the reply payload's
 * first byte is the response status (ALP_CC3501E_RESP_*).  No SOF, no
 * CRC -- the link is a short hardwired point-to-point bus.
 */

#include <string.h>
#include <zephyr/ztest.h>

#include "alp/protocol/cc3501e.h"
#include "cc3501e_hw.h" /* diag sources GET_DIAG_INFO must actually read (#1562) */
#include "protocol.h"   /* CC3501E_REPLY_PAD -- replies are burst-aligned */
#include "transport.h"
#include "worker.h" /* worker_init -- the worker `job` is a static; reset it per test */

/* Replays one request CS transaction through the seams, the way the TI
 * HAL backend's ISR path does: reset staging, feed the bytes, decode. */
/* Expected WIRE length of a reply carrying @p data_len bytes of data.
 *
 * protocol_build_reply() pads every reply payload up to a multiple of
 * CC3501E_REPLY_PAD so the host's DW SSI can move it as ONE burst-aligned DMA
 * chunk.  The pad bytes are never interpreted -- each reply carries its own
 * length -- but they DO change the byte count these tests observe, so the
 * expectations are expressed as "header + padded(status + data)" instead of a
 * hardcoded number that silently encodes the pre-padding layout.
 *
 * Deriving it from CC3501E_REPLY_PAD rather than restating the arithmetic keeps
 * the suite honest if the pad width is ever retuned: the padding landed without
 * this helper and left every status-only expectation asserting 5 against a real
 * 12, which reddened the whole suite. */
static inline size_t reply_padded_payload(size_t payload)
{
	return ((payload + CC3501E_REPLY_PAD - 1u) / CC3501E_REPLY_PAD) * CC3501E_REPLY_PAD;
}

static inline size_t reply_wire(size_t data_len)
{
	return (size_t)ALP_CC3501E_HEADER_BYTES + reply_padded_payload(1u + data_len);
}

static void transaction(const uint8_t *bytes, size_t len)
{
	spi_slave_cs_low();
	for (size_t i = 0; i < len; i++) {
		spi_slave_rx_byte(bytes[i]);
	}
	spi_slave_cs_high();
}

/* Drains the staged reply the way the HAL clocks the host's read FIFO. */
static size_t drain(uint8_t *out, size_t cap)
{
	size_t n = 0;
	while (spi_slave_tx_pending() && n < cap) {
		out[n++] = spi_slave_tx_next_byte();
	}
	return n;
}

/* Assert the 4-byte reply header echoes @cmd, is solicited (flags 0),
 * and declares @payload_len. */
static void assert_reply_header(const uint8_t *r, uint8_t cmd, uint16_t payload_len)
{
	zassert_equal(r[0], cmd, "reply echoes the request cmd");
	zassert_equal(r[1], 0x00u, "solicited reply: flags == 0");
	/* The header declares the PADDED payload -- callers pass the logical length
	 * (status + data) and the padding is accounted for here, so all 28 call
	 * sites keep stating intent rather than restating the pad arithmetic. */
	zassert_equal((uint16_t)(r[2] | ((uint16_t)r[3] << 8)),
	              (uint16_t)reply_padded_payload(payload_len),
	              "reply payload_len (padded to CC3501E_REPLY_PAD)");
}

ZTEST(cc3501e_bridge_transport, test_ping_ok)
{
	const uint8_t ping[] = { ALP_CC3501E_CMD_PING, 0x00u, 0x00u, 0x00u };
	uint8_t       reply[32];

	transport_spi_init();
	transaction(ping, sizeof ping);
	size_t n = drain(reply, sizeof reply);

	/* Reply = header(4) + status(1), no data. */
	zassert_equal(n, reply_wire(0u), "PING reply is header + status");
	assert_reply_header(reply, ALP_CC3501E_CMD_PING, 1u);
	zassert_equal(reply[4], ALP_CC3501E_RESP_OK, "PING -> RESP_OK");
}

ZTEST(cc3501e_bridge_transport, test_get_version_returns_protocol_version)
{
	const uint8_t gv[] = { ALP_CC3501E_CMD_GET_VERSION, 0x00u, 0x00u, 0x00u };
	uint8_t       reply[32];

	transport_spi_init();
	transaction(gv, sizeof gv);
	size_t n = drain(reply, sizeof reply);

	/* Reply = header(4) + status(1) + version(2, LE). */
	zassert_equal(n, reply_wire(2u), "GET_VERSION reply is header + status + u16");
	assert_reply_header(reply, ALP_CC3501E_CMD_GET_VERSION, 3u);
	zassert_equal(reply[4], ALP_CC3501E_RESP_OK, "GET_VERSION -> RESP_OK");
	const uint16_t version = (uint16_t)reply[5] | ((uint16_t)reply[6] << 8);
	zassert_equal(version,
	              (uint16_t)ALP_CC3501E_PROTOCOL_VERSION,
	              "GET_VERSION returns the wire-protocol version (the host's compat gate)");
}

ZTEST(cc3501e_bridge_transport, test_get_mac_not_ready_on_stub)
{
	const uint8_t gm[] = { ALP_CC3501E_CMD_GET_MAC, 0x00u, 0x00u, 0x00u };
	uint8_t       reply[32];

	transport_spi_init();

	/* GET_MAC is now POLL-BY-REPEAT through the async worker (P0-4/P0-6):
	 * the first request submits the job and replies BUSY (the host maps
	 * RESP_ERR_BUSY -> ALP_ERR_BUSY and re-issues).  On the silicon-free
	 * stub the worker runs the job SYNCHRONOUSLY at submit, so a single
	 * re-issue then returns the cached result.  The stub HAL has no radio
	 * -> the job's HAL body reports NOTIMPL, which the handler maps to
	 * NOT_READY rather than a fabricated MAC. */
	transaction(gm, sizeof gm);
	size_t n = drain(reply, sizeof reply);
	zassert_equal(n, reply_wire(0u), "first GET_MAC reply is header + status");
	assert_reply_header(reply, ALP_CC3501E_CMD_GET_MAC, 1u);
	zassert_equal(reply[4],
	              ALP_CC3501E_RESP_ERR_BUSY,
	              "first GET_MAC submits the job -> BUSY (host retries)");

	/* Host re-issues GET_MAC: the worker has the (synchronous-on-stub)
	 * result cached -> NOT_READY. */
	transaction(gm, sizeof gm);
	n = drain(reply, sizeof reply);
	zassert_equal(n, reply_wire(0u), "GET_MAC error reply is header + status");
	assert_reply_header(reply, ALP_CC3501E_CMD_GET_MAC, 1u);
	zassert_equal(
	    reply[4], ALP_CC3501E_RESP_ERR_NOT_READY, "re-issued GET_MAC on stub -> NOT_READY");
}

ZTEST(cc3501e_bridge_transport, test_ota_promote_not_ready_on_stub)
{
	/* OTA_PROMOTE requests the swap-reboot for an image already committed to
	 * STAGED (the unjam path when a bare reset left a slot pending).  It is a
	 * direct handler (not worker-routed), so one transaction resolves it: the
	 * silicon-free stub HAL has no PSA-FWU -> cc3501e_hw_ota_promote reports
	 * NOTIMPL, mapped to NOT_READY rather than a fabricated OK. */
	const uint8_t op[] = { ALP_CC3501E_CMD_OTA_PROMOTE, 0x00u, 0x00u, 0x00u };
	uint8_t       reply[32];

	transport_spi_init();
	transaction(op, sizeof op);
	size_t n = drain(reply, sizeof reply);
	zassert_equal(n, reply_wire(0u), "OTA_PROMOTE error reply is header + status");
	assert_reply_header(reply, ALP_CC3501E_CMD_OTA_PROMOTE, 1u);
	zassert_equal(reply[4],
	              ALP_CC3501E_RESP_ERR_NOT_READY,
	              "OTA_PROMOTE on the silicon-free stub -> NOT_READY");
}

ZTEST(cc3501e_bridge_transport, test_ota_promote_with_payload_is_invalid)
{
	/* OTA_PROMOTE takes no payload; a non-empty one is a caller bug -> INVALID. */
	const uint8_t op[] = { ALP_CC3501E_CMD_OTA_PROMOTE, 0x00u, 0x01u, 0x00u, 0xAAu };
	uint8_t       reply[32];

	transport_spi_init();
	transaction(op, sizeof op);
	size_t n = drain(reply, sizeof reply);
	zassert_equal(n, reply_wire(0u), "bad-len OTA_PROMOTE reply is header + status");
	assert_reply_header(reply, ALP_CC3501E_CMD_OTA_PROMOTE, 1u);
	zassert_equal(
	    reply[4], ALP_CC3501E_RESP_ERR_INVALID, "OTA_PROMOTE with payload -> RESP_ERR_INVALID");
}

ZTEST(cc3501e_bridge_transport, test_ota_update_mode_reports_current_mode)
{
	/* OTA_UPDATE_MODE (0x47): req = mode(1); reply = mode(1) | ota_state(1) |
	 * reserved(2).  Asking for the mode the device is ALREADY in is a no-op by
	 * contract -- it must answer OK and arm no reboot, because the host confirms
	 * entry by RE-ISSUING this opcode until the mode byte matches, and a
	 * non-idempotent handler would reboot the device in a loop.
	 *
	 * The stub build links transport_spi.c's weak bridge_transport_spi_polled(),
	 * which returns false, so mode 0 is the "already there" request here.
	 *
	 * The 4-byte reply is load-bearing, not decoration: a dead bus phase clocks
	 * back literal 0x00 for every byte and 0x00 is ALSO ALP_CC3501E_RESP_OK, so a
	 * bare-status reply to the one opcode whose job is to be the last frame before
	 * a blackout would be byte-identical to a link that just died. */
	const uint8_t op[] = { ALP_CC3501E_CMD_OTA_UPDATE_MODE, 0x00u, 0x01u, 0x00u, 0x00u };
	uint8_t       reply[32];

	transport_spi_init();
	transaction(op, sizeof op);
	size_t n = drain(reply, sizeof reply);
	zassert_equal(n, reply_wire(4u), "OTA_UPDATE_MODE reply is header + status + 4 data bytes");
	assert_reply_header(reply, ALP_CC3501E_CMD_OTA_UPDATE_MODE, 5u);
	zassert_equal(reply[4], ALP_CC3501E_RESP_OK, "no-op mode request -> OK");
	zassert_equal(reply[5], 0x00u, "mode byte reports the NORMAL bridge on the stub");
}

ZTEST(cc3501e_bridge_transport, test_ota_update_mode_bad_request_is_invalid)
{
	/* Exactly one payload byte, value 0 or 1.  Everything else is a caller bug:
	 * no payload, two payload bytes, and an out-of-range mode all -> INVALID. */
	const uint8_t no_payload[] = { ALP_CC3501E_CMD_OTA_UPDATE_MODE, 0x00u, 0x00u, 0x00u };
	const uint8_t two_bytes[]  = {
		ALP_CC3501E_CMD_OTA_UPDATE_MODE, 0x00u, 0x02u, 0x00u, 0x00u, 0x00u
	};
	const uint8_t bad_mode[]    = { ALP_CC3501E_CMD_OTA_UPDATE_MODE, 0x00u, 0x01u, 0x00u, 0x02u };
	const uint8_t *const reqs[] = { no_payload, two_bytes, bad_mode };
	const size_t         lens[] = { sizeof no_payload, sizeof two_bytes, sizeof bad_mode };
	uint8_t              reply[32];

	transport_spi_init();
	for (size_t i = 0; i < sizeof reqs / sizeof reqs[0]; i++) {
		transaction(reqs[i], lens[i]);
		size_t n = drain(reply, sizeof reply);
		zassert_equal(n, reply_wire(0u), "bad OTA_UPDATE_MODE reply is header + status");
		assert_reply_header(reply, ALP_CC3501E_CMD_OTA_UPDATE_MODE, 1u);
		zassert_equal(reply[4],
		              ALP_CC3501E_RESP_ERR_INVALID,
		              "malformed OTA_UPDATE_MODE -> RESP_ERR_INVALID");
	}
}

ZTEST(cc3501e_bridge_transport, test_unknown_opcode_rejected)
{
	/* An opcode in the reserved vendor-extension range (>= 0x80) is never a
	 * v1 command -> RESP_ERR_INVALID (the Wi-Fi/GPIO groups are now
	 * implemented, so a real opcode no longer serves as the "unknown" probe). */
	const uint8_t op[] = { ALP_CC3501E_CMD_RESERVED_VENDOR_BASE, 0x00u, 0x00u, 0x00u };
	uint8_t       reply[32];

	transport_spi_init();
	transaction(op, sizeof op);
	size_t n = drain(reply, sizeof reply);

	zassert_equal(n, reply_wire(0u), "unknown-opcode reply is header + status");
	assert_reply_header(reply, ALP_CC3501E_CMD_RESERVED_VENDOR_BASE, 1u);
	zassert_equal(reply[4],
	              ALP_CC3501E_RESP_ERR_INVALID,
	              "unimplemented opcode -> RESP_ERR_INVALID (header contract)");
}

ZTEST(cc3501e_bridge_transport, test_ping_with_payload_is_invalid)
{
	/* PING takes no payload; a non-empty one is a caller bug.  Also
     * exercises the payload-framing path (declared len == captured). */
	const uint8_t ping_pl[] = { ALP_CC3501E_CMD_PING, 0x00u, 0x01u, 0x00u, 0xABu };
	uint8_t       reply[32];

	transport_spi_init();
	transaction(ping_pl, sizeof ping_pl);
	size_t n = drain(reply, sizeof reply);

	zassert_equal(n, reply_wire(0u), "reply is header + status");
	zassert_equal(reply[4], ALP_CC3501E_RESP_ERR_INVALID, "PING with payload -> INVALID");
}

ZTEST(cc3501e_bridge_transport, test_bad_payload_len_is_protocol_error)
{
	/* Header declares a 5-byte payload but the transaction carried none
     * -> framing mismatch. */
	const uint8_t bad[] = { ALP_CC3501E_CMD_PING, 0x00u, 0x05u, 0x00u };
	uint8_t       reply[32];

	transport_spi_init();
	transaction(bad, sizeof bad);
	size_t n = drain(reply, sizeof reply);

	zassert_equal(n, reply_wire(0u), "reply is header + status");
	zassert_equal(reply[4], ALP_CC3501E_RESP_ERR_PROTOCOL, "length mismatch -> PROTOCOL error");
}

/* An empty transaction (CS toggled, no bytes) must rewind the drain
 * cursor so the host's reply-read re-serves the staged reply. */
ZTEST(cc3501e_bridge_transport, test_empty_transaction_rewinds_reply)
{
	const uint8_t ping[] = { ALP_CC3501E_CMD_PING, 0x00u, 0x00u, 0x00u };
	uint8_t       first[32], again[32];

	transport_spi_init();
	transaction(ping, sizeof ping);
	size_t n_first = drain(first, sizeof first);
	zassert_false(spi_slave_tx_pending(), "reply drained");

	transaction(NULL, 0u); /* CS pulse, nothing captured */

	zassert_true(spi_slave_tx_pending(), "empty transaction rewinds the staged reply");
	size_t n_again = drain(again, sizeof again);
	zassert_equal(n_again, n_first, "full reply re-armed");
	zassert_mem_equal(again, first, n_first, "identical bytes re-armed");
}

/* A fresh request replaces the staged reply; the rewind must never
 * resurrect a previous command's reply once a new one decodes. */
ZTEST(cc3501e_bridge_transport, test_new_request_replaces_staged_reply)
{
	const uint8_t ping[] = { ALP_CC3501E_CMD_PING, 0x00u, 0x00u, 0x00u };
	const uint8_t gv[]   = { ALP_CC3501E_CMD_GET_VERSION, 0x00u, 0x00u, 0x00u };
	uint8_t       buf[32];

	transport_spi_init();
	transaction(ping, sizeof ping);
	(void)drain(buf, sizeof buf);

	transaction(gv, sizeof gv);
	size_t n = drain(buf, sizeof buf);
	zassert_equal(n, reply_wire(2u), "GET_VERSION reply staged");
	zassert_equal(buf[0], ALP_CC3501E_CMD_GET_VERSION, "current reply, not the old PING");

	transaction(NULL, 0u);
	size_t n2 = drain(buf, sizeof buf);
	zassert_equal(n2, reply_wire(2u), "rewind re-arms the CURRENT reply");
	zassert_equal(buf[0], ALP_CC3501E_CMD_GET_VERSION, "still GET_VERSION after rewind");
}

/* GPIO proxy (v0.4): configure -> write -> read round-trip through the
 * production dispatch + the stub HAL's in-memory pin model. */
ZTEST(cc3501e_bridge_transport, test_gpio_write_then_read)
{
	uint8_t reply[32];
	transport_spi_init();

	const uint8_t cfg[] = {
		ALP_CC3501E_CMD_GPIO_CONFIGURE, 0x00u, 0x04u, 0x00u, 14u, ALP_CC3501E_GPIO_DIR_OUTPUT,
		ALP_CC3501E_GPIO_PULL_NONE,     0x00u
	};
	transaction(cfg, sizeof cfg);
	size_t n = drain(reply, sizeof reply);
	zassert_equal(n, reply_wire(0u), "configure reply = header + status");
	zassert_equal(reply[4], ALP_CC3501E_RESP_OK, "GPIO_CONFIGURE -> OK");

	const uint8_t wr[] = { ALP_CC3501E_CMD_GPIO_WRITE, 0x00u, 0x04u, 0x00u, 14u, 1u, 0x00u, 0x00u };
	transaction(wr, sizeof wr);
	(void)drain(reply, sizeof reply);
	zassert_equal(reply[4], ALP_CC3501E_RESP_OK, "GPIO_WRITE -> OK");

	const uint8_t rd[] = { ALP_CC3501E_CMD_GPIO_READ, 0x00u, 0x01u, 0x00u, 14u };
	transaction(rd, sizeof rd);
	n = drain(reply, sizeof reply);
	zassert_equal(n, reply_wire(1u), "read reply = header + status + level");
	assert_reply_header(reply, ALP_CC3501E_CMD_GPIO_READ, 2u);
	zassert_equal(reply[4], ALP_CC3501E_RESP_OK, "GPIO_READ -> OK");
	zassert_equal(reply[5], 1u, "GPIO_READ reflects the written level");
}

ZTEST(cc3501e_bridge_transport, test_cam_enable_ok)
{
	uint8_t reply[32];
	transport_spi_init();
	const uint8_t cam[] = { ALP_CC3501E_CMD_CAM_ENABLE, 0x00u, 0x01u, 0x00u, 0u };
	transaction(cam, sizeof cam);
	size_t n = drain(reply, sizeof reply);
	zassert_equal(n, reply_wire(0u), "cam reply = header + status");
	zassert_equal(reply[4], ALP_CC3501E_RESP_OK, "CAM_ENABLE -> OK on stub");
}

ZTEST(cc3501e_bridge_transport, test_gpio_write_bad_len_invalid)
{
	uint8_t reply[32];
	transport_spi_init();
	/* GPIO_WRITE declares a 1-byte payload but the struct is 4 -> INVALID. */
	const uint8_t wr[] = { ALP_CC3501E_CMD_GPIO_WRITE, 0x00u, 0x01u, 0x00u, 14u };
	transaction(wr, sizeof wr);
	(void)drain(reply, sizeof reply);
	zassert_equal(reply[4], ALP_CC3501E_RESP_ERR_INVALID, "wrong-length GPIO_WRITE -> INVALID");
}

/* Wi-Fi (v0.2): the stub HAL has no radio, so a well-formed request parses
 * cleanly and maps to NOT_READY, while a malformed one is rejected at the
 * protocol layer (INVALID) before reaching the HAL. */

/* WIFI_SCAN_START is now WORKER-ROUTED (mirrors GET_MAC): the seconds-long
 * Wlan_Scan MUST NOT run in the SPI ISR, so the handler is POLL-BY-REPEAT --
 * the first request submits the job and replies BUSY, the host re-issues and
 * collects the cached result.  On the silicon-free stub the worker runs the
 * job synchronously at submit, and the stub HAL's scan reports NOTIMPL ->
 * the re-issue maps to NOT_READY. */
ZTEST(cc3501e_bridge_transport, test_wifi_scan_start_not_ready)
{
	uint8_t reply[32];
	transport_spi_init();
	const uint8_t s[] = { ALP_CC3501E_CMD_WIFI_SCAN_START, 0x00u, 0x00u, 0x00u };

	transaction(s, sizeof s);
	size_t n = drain(reply, sizeof reply);
	zassert_equal(n, reply_wire(0u), "first scan reply = header + status");
	assert_reply_header(reply, ALP_CC3501E_CMD_WIFI_SCAN_START, 1u);
	zassert_equal(reply[4],
	              ALP_CC3501E_RESP_ERR_BUSY,
	              "first SCAN_START submits the job -> BUSY (host retries)");

	transaction(s, sizeof s);
	n = drain(reply, sizeof reply);
	zassert_equal(n, reply_wire(0u), "re-issued scan reply = header + status");
	assert_reply_header(reply, ALP_CC3501E_CMD_WIFI_SCAN_START, 1u);
	zassert_equal(
	    reply[4], ALP_CC3501E_RESP_ERR_NOT_READY, "re-issued SCAN_START on stub -> NOT_READY");
}

/* A finished job NOBODY comes back for must not wedge the single job slot.
 *
 * Only a poll carrying the SAME opcode collects a DONE/ERR and resets the slot
 * (protocol.c).  A host that abandons one worker-routed op -- e.g. gives up on
 * CMD_SOCK_RECV after its timeout -- and then issues ANY other worker-routed op
 * used to strand the finished result: worker_poll() saw job_cmd != cmd with a
 * terminal state, reported "other cmd busy", and nothing in the system could
 * ever clear it.  Every later worker-routed opcode answered RESP_ERR_BUSY
 * forever, which on silicon looks like the whole bridge wedging.
 *
 * Here SCAN_START is submitted and DELIBERATELY not collected (on the stub the
 * worker runs it synchronously at submit, so the slot is terminal immediately),
 * then GET_RSSI must still be able to run to completion. */
ZTEST(cc3501e_bridge_transport, test_abandoned_job_does_not_wedge_the_slot)
{
	uint8_t reply[32];
	transport_spi_init();
	const uint8_t scan[] = { ALP_CC3501E_CMD_WIFI_SCAN_START, 0x00u, 0x00u, 0x00u };
	const uint8_t rssi[] = { ALP_CC3501E_CMD_WIFI_GET_RSSI, 0x00u, 0x00u, 0x00u };

	/* Submit SCAN and walk away -- its result is now orphaned in the slot. */
	transaction(scan, sizeof scan);
	size_t n = drain(reply, sizeof reply);
	zassert_equal(n, reply_wire(0u), "SCAN submit reply = header + status");
	zassert_equal(reply[4], ALP_CC3501E_RESP_ERR_BUSY, "SCAN submits -> BUSY");

	/* A DIFFERENT opcode must get its own turn: first transaction discards the
	 * orphan and submits, second collects.  Before the fix both answered BUSY. */
	transaction(rssi, sizeof rssi);
	n = drain(reply, sizeof reply);
	zassert_equal(n, reply_wire(0u), "RSSI submit reply = header + status");
	assert_reply_header(reply, ALP_CC3501E_CMD_WIFI_GET_RSSI, 1u);
	zassert_equal(reply[4],
	              ALP_CC3501E_RESP_ERR_BUSY,
	              "the orphaned SCAN result is discarded and RSSI submits");

	transaction(rssi, sizeof rssi);
	n = drain(reply, sizeof reply);
	zassert_equal(n, reply_wire(0u), "RSSI collect reply = header + status");
	assert_reply_header(reply, ALP_CC3501E_CMD_WIFI_GET_RSSI, 1u);
	zassert_equal(reply[4],
	              ALP_CC3501E_RESP_ERR_NOT_READY,
	              "RSSI reaches its own result (NOT_READY on the radio-less stub) "
	              "instead of being wedged on BUSY forever");
}

/* WIFI_GET_RSSI is worker-routed the same way (poll-by-repeat): submit -> BUSY,
 * re-issue -> the cached NOT_READY on the radio-less stub. */
ZTEST(cc3501e_bridge_transport, test_wifi_get_rssi_not_ready)
{
	uint8_t reply[32];
	transport_spi_init();
	const uint8_t r[] = { ALP_CC3501E_CMD_WIFI_GET_RSSI, 0x00u, 0x00u, 0x00u };

	transaction(r, sizeof r);
	size_t n = drain(reply, sizeof reply);
	zassert_equal(n, reply_wire(0u), "first rssi reply = header + status");
	assert_reply_header(reply, ALP_CC3501E_CMD_WIFI_GET_RSSI, 1u);
	zassert_equal(reply[4],
	              ALP_CC3501E_RESP_ERR_BUSY,
	              "first GET_RSSI submits the job -> BUSY (host retries)");

	transaction(r, sizeof r);
	n = drain(reply, sizeof reply);
	zassert_equal(n, reply_wire(0u), "re-issued rssi reply = header + status");
	assert_reply_header(reply, ALP_CC3501E_CMD_WIFI_GET_RSSI, 1u);
	zassert_equal(
	    reply[4], ALP_CC3501E_RESP_ERR_NOT_READY, "re-issued GET_RSSI on stub -> NOT_READY");
}

ZTEST(cc3501e_bridge_transport, test_wifi_connect_sta_parses_then_not_ready)
{
	uint8_t reply[32];
	transport_spi_init();
	/* connect_t {ssid_len=4, psk_len=8, security=WPA2(1), rsvd} + "wifi" + "password";
	 * payload_len = 4 (header) + 4 (ssid) + 8 (psk) = 16. */
	const uint8_t req[] = { ALP_CC3501E_CMD_WIFI_CONNECT_STA,
		                    0x00u,
		                    16u,
		                    0x00u,
		                    4u,
		                    8u,
		                    1u,
		                    0u,
		                    'w',
		                    'i',
		                    'f',
		                    'i',
		                    'p',
		                    'a',
		                    's',
		                    's',
		                    'w',
		                    'o',
		                    'r',
		                    'd' };
	/* WIFI_CONNECT_STA is WORKER-ROUTED (Wlan_Connect blocks for seconds, so it must run
	 * off the SPI ISR): the first request submits the job and replies BUSY; the host
	 * re-issues and collects the cached result, which on the radio-less stub is NOT_READY. */
	transaction(req, sizeof req);
	size_t n = drain(reply, sizeof reply);
	zassert_equal(n, reply_wire(0u), "first connect reply = header + status");
	assert_reply_header(reply, ALP_CC3501E_CMD_WIFI_CONNECT_STA, 1u);
	zassert_equal(reply[4],
	              ALP_CC3501E_RESP_ERR_BUSY,
	              "first CONNECT submits the job -> BUSY (host retries)");

	transaction(req, sizeof req);
	n = drain(reply, sizeof reply);
	zassert_equal(n, reply_wire(0u), "re-issued connect reply = header + status");
	assert_reply_header(reply, ALP_CC3501E_CMD_WIFI_CONNECT_STA, 1u);
	zassert_equal(
	    reply[4], ALP_CC3501E_RESP_ERR_NOT_READY, "re-issued CONNECT on stub -> NOT_READY");
}

ZTEST(cc3501e_bridge_transport, test_wifi_connect_bad_len_invalid)
{
	uint8_t reply[32];
	transport_spi_init();
	/* connect_t says ssid_len=4 psk_len=8 (needs 16 payload) but only 8 sent. */
	const uint8_t req[] = {
		ALP_CC3501E_CMD_WIFI_CONNECT_STA, 0x00u, 8u, 0x00u, 4u, 8u, 1u, 0u, 'w', 'i', 'f', 'i'
	};
	transaction(req, sizeof req);
	(void)drain(reply, sizeof reply);
	zassert_equal(reply[4], ALP_CC3501E_RESP_ERR_INVALID, "connect length mismatch -> INVALID");
}

/* BLE (v0.3): no BLE host on the stub -> well-formed requests parse and map
 * to NOT_READY; malformed ones are rejected (INVALID) at the protocol layer. */

/* BLE_ENABLE is WORKER-ROUTED (the real body starts Wi-Fi + NimBLE, blocks for
 * seconds -- must run off the SPI ISR), so it is poll-by-repeat exactly like
 * GET_MAC: the first request submits the job and replies BUSY; the re-issue
 * collects the cached result, which on the radio-less stub is NOT_READY. */
ZTEST(cc3501e_bridge_transport, test_ble_enable_not_ready)
{
	uint8_t reply[32];
	transport_spi_init();
	const uint8_t e[] = { ALP_CC3501E_CMD_BLE_ENABLE, 0x00u, 0x00u, 0x00u };

	transaction(e, sizeof e);
	size_t n = drain(reply, sizeof reply);
	zassert_equal(n, reply_wire(0u), "first BLE_ENABLE reply = header + status");
	assert_reply_header(reply, ALP_CC3501E_CMD_BLE_ENABLE, 1u);
	zassert_equal(reply[4],
	              ALP_CC3501E_RESP_ERR_BUSY,
	              "first BLE_ENABLE submits the job -> BUSY (host retries)");

	transaction(e, sizeof e);
	n = drain(reply, sizeof reply);
	zassert_equal(n, reply_wire(0u), "re-issued BLE_ENABLE reply = header + status");
	assert_reply_header(reply, ALP_CC3501E_CMD_BLE_ENABLE, 1u);
	zassert_equal(reply[4], ALP_CC3501E_RESP_ERR_NOT_READY, "no BLE host on stub -> NOT_READY");
}

ZTEST(cc3501e_bridge_transport, test_ble_adv_start_parses_then_not_ready)
{
	uint8_t reply[32];
	transport_spi_init();
	/* connectable=1, rsvd=0, imin=100, imax=200, adv_data_len=3, adv={02 01 06};
	 * packed header is 7 bytes -> payload_len = 7 + 3 = 10. */
	const uint8_t req[] = { ALP_CC3501E_CMD_BLE_ADV_START,
		                    0x00u,
		                    10u,
		                    0x00u,
		                    1u,
		                    0u,
		                    100u,
		                    0u,
		                    200u,
		                    0u,
		                    3u,
		                    0x02u,
		                    0x01u,
		                    0x06u };
	/* BLE_ADV_START is now worker-routed (its NimBLE HCI commands block on the
	 * host task and must not run in the SPI ISR): the first request submits the
	 * job and replies BUSY; the host re-issues, and on the silicon-free stub the
	 * worker ran synchronously at submit -> NOT_READY (stub HAL has no radio). */
	transaction(req, sizeof req);
	size_t n = drain(reply, sizeof reply);
	zassert_equal(n, reply_wire(0u), "adv reply = header + status");
	assert_reply_header(reply, ALP_CC3501E_CMD_BLE_ADV_START, 1u);
	zassert_equal(reply[4], ALP_CC3501E_RESP_ERR_BUSY, "well-formed adv submits the job -> BUSY");

	transaction(req, sizeof req);
	n = drain(reply, sizeof reply);
	zassert_equal(n, reply_wire(0u), "re-issued adv reply = header + status");
	assert_reply_header(reply, ALP_CC3501E_CMD_BLE_ADV_START, 1u);
	zassert_equal(reply[4], ALP_CC3501E_RESP_ERR_NOT_READY, "re-issued adv on stub -> NOT_READY");
}

ZTEST(cc3501e_bridge_transport, test_ble_adv_start_bad_len_invalid)
{
	uint8_t reply[32];
	transport_spi_init();
	/* adv_data_len=3 needs payload 10, but only 7 sent. */
	const uint8_t req[] = {
		ALP_CC3501E_CMD_BLE_ADV_START, 0x00u, 7u, 0x00u, 1u, 0u, 100u, 0u, 200u, 0u, 3u
	};
	transaction(req, sizeof req);
	(void)drain(reply, sizeof reply);
	zassert_equal(reply[4], ALP_CC3501E_RESP_ERR_INVALID, "adv length mismatch -> INVALID");
}

ZTEST(cc3501e_bridge_transport, test_ble_connect_bad_len_invalid)
{
	uint8_t reply[32];
	transport_spi_init();
	/* BLE_CONNECT needs 7 bytes (addr_type + addr[6]); send 4. */
	const uint8_t req[] = { ALP_CC3501E_CMD_BLE_CONNECT, 0x00u, 4u, 0x00u, 0u, 1u, 2u, 3u };
	transaction(req, sizeof req);
	(void)drain(reply, sizeof reply);
	zassert_equal(reply[4], ALP_CC3501E_RESP_ERR_INVALID, "short BLE_CONNECT -> INVALID");
}

/* Configurability + diagnostics (firmware-side, no radio): these are fully
 * functional on the stub -- config is accepted (OK) and diag/stats return
 * real firmware-tracked data. */
ZTEST(cc3501e_bridge_transport, test_get_diag_info)
{
	uint8_t reply[40];
	transport_spi_init();
	const uint8_t d[] = { ALP_CC3501E_CMD_GET_DIAG_INFO, 0x00u, 0x00u, 0x00u };
	transaction(d, sizeof d);
	size_t n = drain(reply, sizeof reply);
	zassert_equal(n, reply_wire(16u), "diag reply = header + status + 16B struct");
	assert_reply_header(reply, ALP_CC3501E_CMD_GET_DIAG_INFO, 17u);
	zassert_equal(reply[4], ALP_CC3501E_RESP_OK, "GET_DIAG_INFO -> OK");
	const uint16_t fw = (uint16_t)reply[5] | ((uint16_t)reply[6] << 8);
	/* Marker is DERIVED from firmware-version.txt (packed (MINOR<<8)|PATCH) and
	 * fed in via -DCC3501E_BRIDGE_FW_VERSION_U16 by this test's CMakeLists, so
	 * this asserts against the single-sourced constant, never a drifting
	 * literal.  0.2.0 -> 0x0200. */
	zassert_equal(fw,
	              CC3501E_BRIDGE_FW_VERSION_U16,
	              "fw_version = derived marker (firmware-version.txt 0.2.0 -> 0x0200)");
	zassert_equal(reply[7], (uint8_t)ALP_CC3501E_RESET_POWER_ON, "stub reset cause = POWER_ON");
	/* Role now comes from cc3501e_hw_radio_role() instead of a hardcoded
	 * ROLE_OFF (#1562).  The stub brings no radio up, so OFF is still the right
	 * answer here -- but it is now the HAL's answer, not a literal.
	 *
	 * HONEST LIMIT: every stub diag getter returns 0 and ROLE_OFF is 0, so this
	 * assertion CANNOT distinguish "reads the HAL" from "re-hardcoded to 0".
	 * It pins the wire layout and the reply length; the discriminating check for
	 * the role actually tracking the AP is the bench run on real silicon, where
	 * an AP that is up must report ROLE_WIFI_AP (2). Do not read a green run
	 * here as proof the field is live. */
	zassert_equal(reply[8], (uint8_t)ALP_CC3501E_ROLE_OFF, "stub has no radio role up -> OFF");
	/* free_heap_bytes must be the heap again, not the Wi-Fi event ID that
	 * squatted here (#1562); the event ID moved to reserved[0] = reply[18]. */
	/* Offsets: reply = header(4) + status(1) + struct, so struct byte i is at
	 * reply[5+i].  free_heap_bytes is struct[8..11] -> reply[13..16]. */
	const uint32_t heap = (uint32_t)reply[13] | ((uint32_t)reply[14] << 8) |
	                      ((uint32_t)reply[15] << 16) | ((uint32_t)reply[16] << 24);
	zassert_equal(heap, cc3501e_hw_free_heap_bytes(), "free_heap_bytes = the HAL's heap source");
	zassert_equal(reply[18],
	              (uint8_t)(cc3501e_hw_wifi_last_event_id() & 0xFFu),
	              "reserved[0] = low byte of the last Wi-Fi event ID");
	zassert_equal(reply[19], 0u, "reserved[1] still zero");
	zassert_equal(reply[20], 0u, "reserved[2] still zero");
}

ZTEST(cc3501e_bridge_transport, test_power_policy_ok)
{
	uint8_t reply[16];
	transport_spi_init();
	/* policy=BALANCED(1) | wake=HOST_SPI(0x01) | rsvd(2) | idle_ms=1000 (LE32) */
	const uint8_t pp[] = {
		ALP_CC3501E_CMD_POWER_POLICY, 0x00u, 8u, 0x00u, 1u, 0x01u, 0u, 0u, 0xE8u, 0x03u, 0u, 0u
	};
	transaction(pp, sizeof pp);
	(void)drain(reply, sizeof reply);
	zassert_equal(reply[4], ALP_CC3501E_RESP_OK, "POWER_POLICY accepted -> OK");
}

ZTEST(cc3501e_bridge_transport, test_power_policy_bad_len_invalid)
{
	uint8_t reply[16];
	transport_spi_init();
	const uint8_t pp[] = { ALP_CC3501E_CMD_POWER_POLICY, 0x00u, 4u, 0x00u, 1u, 0u, 0u, 0u };
	transaction(pp, sizeof pp);
	(void)drain(reply, sizeof reply);
	zassert_equal(reply[4], ALP_CC3501E_RESP_ERR_INVALID, "short POWER_POLICY -> INVALID");
}

ZTEST(cc3501e_bridge_transport, test_diag_get_stats_counts_frames)
{
	uint8_t reply[32]; /* reply_wire(16u) = 28 B; 24 was sized for the old 8B payload */
	transport_spi_init();
	/* A PING first guarantees >= 1 OK frame before we read the stats. */
	const uint8_t ping[] = { ALP_CC3501E_CMD_PING, 0x00u, 0x00u, 0x00u };
	transaction(ping, sizeof ping);
	(void)drain(reply, sizeof reply);

	const uint8_t s[] = { ALP_CC3501E_CMD_DIAG_GET_STATS, 0x00u, 0x00u, 0x00u };
	transaction(s, sizeof s);
	/* 16B, not 8: issue #102 appended worker_execs(LE32) | retry_latch_hits(LE32)
	 * after the original frames_ok/frames_err pair -- see test_worker_routed_-
	 * retry_seq_served_from_latch_and_counted below for those two fields. */
	size_t n = drain(reply, sizeof reply);
	zassert_equal(n, reply_wire(16u), "stats reply = header + status + 16B");
	zassert_equal(reply[4], ALP_CC3501E_RESP_OK, "DIAG_GET_STATS -> OK");
	const uint32_t frames_ok = (uint32_t)reply[5] | ((uint32_t)reply[6] << 8) |
	                           ((uint32_t)reply[7] << 16) | ((uint32_t)reply[8] << 24);
	zassert_true(frames_ok >= 1u, "OK frames counted (>= the prior PING)");
}

/* issue #102: the generic worker-routed retry latch.  BLE_ENABLE stands in
 * for the class -- worker-routed + argless, exactly the shape
 * handle_worker_routed serves.  seq rides flags bits 3..7 (CC3501E_REQ_SEQ_-
 * SHIFT/MASK in protocol.c): seq 5 = 5<<3 = 0x28, seq 6 = 6<<3 = 0x30. */
static uint32_t diag_stat_u32(const uint8_t *reply, size_t off)
{
	return (uint32_t)reply[off] | ((uint32_t)reply[off + 1u] << 8) |
	       ((uint32_t)reply[off + 2u] << 16) | ((uint32_t)reply[off + 3u] << 24);
}

ZTEST(cc3501e_bridge_transport, test_worker_routed_retry_seq_served_from_latch_and_counted)
{
	uint8_t reply[32];
	transport_spi_init();

	const uint8_t stats[] = { ALP_CC3501E_CMD_DIAG_GET_STATS, 0x00u, 0x00u, 0x00u };
	transaction(stats, sizeof stats);
	(void)drain(reply, sizeof reply);
	const uint32_t execs_before = diag_stat_u32(reply, 13u);
	const uint32_t hits_before  = diag_stat_u32(reply, 17u);

	const uint8_t seq5[] = { ALP_CC3501E_CMD_BLE_ENABLE, 0x28u /* seq=5 */, 0x00u, 0x00u };

	/* Submit -- no job in flight yet, so this is a fresh IDLE->QUEUED edge
	 * regardless of the latch (which is empty for BLE_ENABLE at this seq). */
	transaction(seq5, sizeof seq5);
	(void)drain(reply, sizeof reply);
	zassert_equal(reply[4], ALP_CC3501E_RESP_ERR_BUSY, "submit -> BUSY");

	/* Collect -- the stub ran the body synchronously at submit, so this reaches
	 * WORKER_ERR (no BLE host on stub) and is the collect edge that latches the
	 * outcome under seq 5. */
	transaction(seq5, sizeof seq5);
	(void)drain(reply, sizeof reply);
	zassert_equal(reply[4], ALP_CC3501E_RESP_ERR_NOT_READY, "collected -> NOT_READY");

	/* A THIRD, byte-identical frame (same opcode, same seq) is what
	 * poll_by_repeat() sends when it never saw the second transaction's reply.
	 * It must be served from the latch: same answer, and -- proven below via
	 * g_worker_execs -- the worker must NOT run again. */
	transaction(seq5, sizeof seq5);
	(void)drain(reply, sizeof reply);
	zassert_equal(reply[4], ALP_CC3501E_RESP_ERR_NOT_READY, "retry served from the latch");

	transaction(stats, sizeof stats);
	(void)drain(reply, sizeof reply);
	const uint32_t execs_after = diag_stat_u32(reply, 13u);
	const uint32_t hits_after  = diag_stat_u32(reply, 17u);
	zassert_equal(execs_after,
	              execs_before + 1u,
	              "worker_execute() ran exactly once for the submit+collect, "
	              "not again for the latched retry");
	zassert_equal(hits_after, hits_before + 1u, "the retry was served from the latch exactly once");

	/* A DIFFERENT seq for the SAME opcode is a genuinely new logical command
	 * (item 4's mandatory mitigation): the stale latch entry must be dropped,
	 * not served, and the worker must resubmit. */
	const uint8_t seq6[] = { ALP_CC3501E_CMD_BLE_ENABLE, 0x30u /* seq=6 */, 0x00u, 0x00u };
	transaction(seq6, sizeof seq6);
	(void)drain(reply, sizeof reply);
	zassert_equal(reply[4],
	              ALP_CC3501E_RESP_ERR_BUSY,
	              "different seq, same opcode -> stale latch dropped, worker resubmits");
}

ZTEST(cc3501e_bridge_transport, test_diag_log_level_ok)
{
	uint8_t reply[16];
	transport_spi_init();
	const uint8_t l[] = { ALP_CC3501E_CMD_DIAG_LOG_LEVEL, 0x00u, 1u, 0x00u, 2u };
	transaction(l, sizeof l);
	(void)drain(reply, sizeof reply);
	zassert_equal(reply[4], ALP_CC3501E_RESP_OK, "DIAG_LOG_LEVEL accepted -> OK");
}

/* ---- GPIO proxy (0x50..0x53) + camera enables (0x60/0x61) ---- *
 * The stub HAL simulates the pads in RAM, so these exercise the full
 * wire contract: payload struct layout, dispatch, and the read-back. */

ZTEST(cc3501e_bridge_transport, test_gpio_configure_write_read_roundtrip)
{
	uint8_t reply[16];
	transport_spi_init();

	/* CONFIGURE pad 13 as OUTPUT, no pull (payload = gpio_configure_t, 4 B). */
	const uint8_t cfg[] = {
		ALP_CC3501E_CMD_GPIO_CONFIGURE, 0x00u, 0x04u, 0x00u, 13u, ALP_CC3501E_GPIO_DIR_OUTPUT,
		ALP_CC3501E_GPIO_PULL_NONE,     0u
	};
	transaction(cfg, sizeof cfg);
	size_t n = drain(reply, sizeof reply);
	zassert_equal(n, reply_wire(0u), "CONFIGURE reply = header + status");
	assert_reply_header(reply, ALP_CC3501E_CMD_GPIO_CONFIGURE, 1u);
	zassert_equal(reply[4], ALP_CC3501E_RESP_OK, "CONFIGURE -> OK");

	/* WRITE pad 13 high (payload = gpio_write_t, 4 B). */
	const uint8_t wr[] = { ALP_CC3501E_CMD_GPIO_WRITE, 0x00u, 0x04u, 0x00u, 13u, 1u, 0u, 0u };
	transaction(wr, sizeof wr);
	n = drain(reply, sizeof reply);
	zassert_equal(reply[4], ALP_CC3501E_RESP_OK, "WRITE -> OK");

	/* READ pad 13 (request payload = 1 B); reply = header + status + level. */
	const uint8_t rd[] = { ALP_CC3501E_CMD_GPIO_READ, 0x00u, 0x01u, 0x00u, 13u };
	transaction(rd, sizeof rd);
	n = drain(reply, sizeof reply);
	zassert_equal(n, reply_wire(1u), "READ reply = header + status + level");
	assert_reply_header(reply, ALP_CC3501E_CMD_GPIO_READ, 2u);
	zassert_equal(reply[4], ALP_CC3501E_RESP_OK, "READ -> OK");
	zassert_equal(reply[5], 1u, "READ reflects the WRITE-high");

	/* WRITE low, READ back 0. */
	const uint8_t wr0[] = { ALP_CC3501E_CMD_GPIO_WRITE, 0x00u, 0x04u, 0x00u, 13u, 0u, 0u, 0u };
	transaction(wr0, sizeof wr0);
	(void)drain(reply, sizeof reply);
	transaction(rd, sizeof rd);
	(void)drain(reply, sizeof reply);
	zassert_equal(reply[5], 0u, "READ reflects the WRITE-low");
}

ZTEST(cc3501e_bridge_transport, test_gpio_configure_bad_len_invalid)
{
	uint8_t reply[16];
	transport_spi_init();
	/* CONFIGURE with a 2-byte payload (gpio_configure_t is 4) -> INVALID. */
	const uint8_t bad[] = { ALP_CC3501E_CMD_GPIO_CONFIGURE, 0x00u, 0x02u, 0x00u, 13u, 1u };
	transaction(bad, sizeof bad);
	(void)drain(reply, sizeof reply);
	zassert_equal(reply[4], ALP_CC3501E_RESP_ERR_INVALID, "short CONFIGURE -> INVALID");
}

ZTEST(cc3501e_bridge_transport, test_cam_enable_disable_ok)
{
	uint8_t reply[16];
	transport_spi_init();
	const uint8_t en[] = { ALP_CC3501E_CMD_CAM_ENABLE, 0x00u, 0x01u, 0x00u, 0u };
	transaction(en, sizeof en);
	(void)drain(reply, sizeof reply);
	zassert_equal(reply[4], ALP_CC3501E_RESP_OK, "CAM_ENABLE(0) -> OK");

	const uint8_t dis[] = { ALP_CC3501E_CMD_CAM_DISABLE, 0x00u, 0x01u, 0x00u, 0u };
	transaction(dis, sizeof dis);
	(void)drain(reply, sizeof reply);
	zassert_equal(reply[4], ALP_CC3501E_RESP_OK, "CAM_DISABLE(0) -> OK");
}

/* OTA (v0.2): the OTA opcodes are SYNCHRONOUS handlers (no worker), so a
 * well-formed request reaches the HAL and the radio-less stub reports NOTIMPL
 * -> RESP_ERR_NOT_READY in a single transaction; malformed ones are rejected
 * (INVALID) at the protocol layer before the HAL. */
ZTEST(cc3501e_bridge_transport, test_ota_begin_not_ready)
{
	uint8_t reply[32];
	transport_spi_init();
	/* total_len = 0x00010000 (LE) -> payload {00 00 01 00}. */
	const uint8_t b[] = {
		ALP_CC3501E_CMD_OTA_BEGIN, 0x00u, 0x04u, 0x00u, 0x00u, 0x00u, 0x01u, 0x00u
	};
	transaction(b, sizeof b);
	size_t n = drain(reply, sizeof reply);
	zassert_equal(n, reply_wire(0u), "ota begin reply = header + status");
	assert_reply_header(reply, ALP_CC3501E_CMD_OTA_BEGIN, 1u);
	zassert_equal(reply[4], ALP_CC3501E_RESP_ERR_NOT_READY, "no PSA-FWU on stub -> NOT_READY");
}

ZTEST(cc3501e_bridge_transport, test_ota_begin_bad_len_invalid)
{
	uint8_t reply[32];
	transport_spi_init();
	/* declared 3-byte payload (total_len must be 4) -> INVALID. */
	const uint8_t b[] = { ALP_CC3501E_CMD_OTA_BEGIN, 0x00u, 0x03u, 0x00u, 0x00u, 0x00u, 0x01u };
	transaction(b, sizeof b);
	(void)drain(reply, sizeof reply);
	zassert_equal(reply[4], ALP_CC3501E_RESP_ERR_INVALID, "OTA_BEGIN wrong length -> INVALID");
}

ZTEST(cc3501e_bridge_transport, test_ota_write_not_ready)
{
	uint8_t reply[32];
	transport_spi_init();
	/* offset = 0 (LE32) + 4 image bytes -> payload_len 8. */
	const uint8_t w[] = { ALP_CC3501E_CMD_OTA_WRITE,
		                  0x00u,
		                  0x08u,
		                  0x00u,
		                  0x00u,
		                  0x00u,
		                  0x00u,
		                  0x00u,
		                  0xDEu,
		                  0xADu,
		                  0xBEu,
		                  0xEFu };
	transaction(w, sizeof w);
	size_t n = drain(reply, sizeof reply);
	zassert_equal(n, reply_wire(0u), "ota write reply = header + status");
	assert_reply_header(reply, ALP_CC3501E_CMD_OTA_WRITE, 1u);
	zassert_equal(reply[4], ALP_CC3501E_RESP_ERR_NOT_READY, "no PSA-FWU on stub -> NOT_READY");
}

ZTEST(cc3501e_bridge_transport, test_ota_write_no_data_invalid)
{
	uint8_t reply[32];
	transport_spi_init();
	/* offset only, no image bytes (req_len must be >= 5) -> INVALID. */
	const uint8_t w[] = {
		ALP_CC3501E_CMD_OTA_WRITE, 0x00u, 0x04u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u
	};
	transaction(w, sizeof w);
	(void)drain(reply, sizeof reply);
	zassert_equal(reply[4], ALP_CC3501E_RESP_ERR_INVALID, "OTA_WRITE without data -> INVALID");
}

ZTEST(cc3501e_bridge_transport, test_ota_finish_not_ready)
{
	uint8_t reply[32];
	transport_spi_init();
	const uint8_t f[] = { ALP_CC3501E_CMD_OTA_FINISH, 0x00u, 0x00u, 0x00u };
	transaction(f, sizeof f);
	size_t n = drain(reply, sizeof reply);
	zassert_equal(n, reply_wire(0u), "ota finish reply = header + status");
	zassert_equal(reply[4], ALP_CC3501E_RESP_ERR_NOT_READY, "no session on stub -> NOT_READY");
}

ZTEST(cc3501e_bridge_transport, test_ota_abort_not_ready)
{
	uint8_t reply[32];
	transport_spi_init();
	const uint8_t a[] = { ALP_CC3501E_CMD_OTA_ABORT, 0x00u, 0x00u, 0x00u };
	transaction(a, sizeof a);
	(void)drain(reply, sizeof reply);
	zassert_equal(reply[4], ALP_CC3501E_RESP_ERR_NOT_READY, "OTA_ABORT on stub -> NOT_READY");
}

ZTEST(cc3501e_bridge_transport, test_ota_status_not_ready)
{
	uint8_t reply[32];
	transport_spi_init();
	const uint8_t s[] = { ALP_CC3501E_CMD_OTA_STATUS, 0x00u, 0x00u, 0x00u };
	transaction(s, sizeof s);
	(void)drain(reply, sizeof reply);
	/* stub ota_status returns NOTIMPL before the 12-byte body is framed. */
	zassert_equal(reply[4], ALP_CC3501E_RESP_ERR_NOT_READY, "OTA_STATUS on stub -> NOT_READY");
}

/* ------------------------------------------------------------------ */
/* SPI1 host passthrough (0x55..0x57)                                   */
/*                                                                      */
/* CI links the stub HAL, whose SPI1 body is a WIRE LOOP (MOSI tied to  */
/* MISO), so these cover the halves CI can actually check: request      */
/* validation, the inline-TX / self-delimiting-RX framing, the seq      */
/* duplicate suppression and the CS_HOLD state gate.  Clock rate, CS    */
/* timing and bus errors are silicon-only -- nothing here is evidence   */
/* for those.                                                           */
/*                                                                      */
/* Every opcode in the family is worker-routed, and on the stub the job */
/* runs SYNCHRONOUSLY inside worker_submit(), so a host needs exactly   */
/* two transactions: submit -> BUSY, re-issue -> the result.            */
/* ------------------------------------------------------------------ */

/* CONFIGURE at 10 MHz (0x00989680), mode 0, 8 bits/word, CS0. */
static const uint8_t spi1_configure_req[] = {
	ALP_CC3501E_CMD_SPI1_CONFIGURE,
	0x00u,
	0x08u, /* payload_len 8           */
	0x00u,
	0x80u,
	0x96u,
	0x98u,
	0x00u, /* freq_hz LE32            */
	0x00u, /* mode 0 = CPOL 0, CPHA 0 */
	0x08u, /* bits_per_word           */
	(uint8_t)ALP_CC3501E_SPI1_CS0,
	0x00u, /* reserved                */
};

/* Send @p req twice: once to submit (asserting the BUSY ack) and once to
 * collect.  Returns the collected status; the reply bytes stay in @p reply. */
static uint8_t spi1_collect(const uint8_t *req, size_t req_len, uint8_t *reply, size_t cap)
{
	transaction(req, req_len);
	(void)drain(reply, cap);
	zassert_equal(reply[4], ALP_CC3501E_RESP_ERR_BUSY, "worker-routed submit acks BUSY");
	transaction(req, req_len);
	(void)drain(reply, cap);
	return reply[4];
}

/* Put the bus back in the closed state EVERY SPI1 test starts from -- call
 * this first, before anything else in the test body.  The handler's
 * g_configured/g_cs_held and the stub's open flag are file statics that
 * OUTLIVE the per-test worker_init(), so "nothing is open" has to be
 * asserted at the top of each test rather than inherited from whatever the
 * previous test (or ztest's execution order) happened to leave behind. */
static void spi1_release(uint8_t *reply, size_t cap)
{
	const uint8_t rel[] = { ALP_CC3501E_CMD_SPI1_RELEASE, 0x00u, 0x00u, 0x00u };
	zassert_equal(spi1_collect(rel, sizeof rel, reply, cap),
	              ALP_CC3501E_RESP_OK,
	              "RELEASE is the escape hatch: releasing nothing still succeeds");
}

ZTEST(cc3501e_bridge_transport, test_spi1_transfer_before_configure_is_not_ready)
{
	uint8_t reply[64];
	transport_spi_init();
	spi1_release(reply, sizeof reply); /* guarantee no instance is open */

	const uint8_t xfer[] = {
		ALP_CC3501E_CMD_SPI1_TRANSFER,
		0x00u,
		0x09u,
		0x00u,
		0x01u,
		0x00u,
		0x00u,
		0x01u,
		0x00u,
		0x00u,
		0x00u,
		0x00u,
		0xA5u,
	};
	transaction(xfer, sizeof xfer);
	(void)drain(reply, sizeof reply);
	/* Rejected in the handler, so it never reaches the worker: no BUSY first. */
	zassert_equal(reply[4],
	              ALP_CC3501E_RESP_ERR_NOT_READY,
	              "TRANSFER before a successful CONFIGURE -> NOT_READY");
}

ZTEST(cc3501e_bridge_transport, test_spi1_configure_then_transfer_loops_back)
{
	uint8_t reply[64];
	transport_spi_init();
	spi1_release(reply, sizeof reply); /* known state, independent of test order */

	zassert_equal(spi1_collect(spi1_configure_req, sizeof spi1_configure_req, reply, sizeof reply),
	              ALP_CC3501E_RESP_OK,
	              "CONFIGURE -> OK");
	zassert_equal((uint32_t)reply[5] | ((uint32_t)reply[6] << 8) | ((uint32_t)reply[7] << 16) |
	                  ((uint32_t)reply[8] << 24),
	              10000000u,
	              "the reply reports the ACTUAL rate (the stub has no divider, so it matches)");
	zassert_equal((uint16_t)reply[9] | ((uint16_t)reply[10] << 8),
	              (uint16_t)ALP_CC3501E_SPI1_MAX_XFER,
	              "CONFIGURE hands the host this firmware's chunk size");
	zassert_equal(reply[11], 0x08u, "the accepted bits_per_word echoes back");

	/* len 4, flags 0 (single-shot), seq 1, then the TX bytes inline. */
	const uint8_t xfer[] = {
		ALP_CC3501E_CMD_SPI1_TRANSFER,
		0x00u,
		0x0Cu,
		0x00u,
		0x04u,
		0x00u,
		0x00u,
		0x01u,
		0x00u,
		0x00u,
		0x00u,
		0x00u,
		0xDEu,
		0xADu,
		0xBEu,
		0xEFu,
	};
	zassert_equal(spi1_collect(xfer, sizeof xfer, reply, sizeof reply),
	              ALP_CC3501E_RESP_OK,
	              "TRANSFER -> OK");
	zassert_equal((uint16_t)reply[5] | ((uint16_t)reply[6] << 8),
	              4u,
	              "the reply carries its own RX count, so the host stops before the pad");
	zassert_equal(reply[7], 0x00u, "flags 0 -> the CS readback bit is clear");
	zassert_equal(reply[8], 0x01u, "the reply echoes the request seq");
	zassert_mem_equal(&reply[9], &xfer[12], 4u, "the stub loops MOSI straight back on MISO");
}

ZTEST(cc3501e_bridge_transport, test_spi1_repeated_seq_serves_the_cached_result)
{
	uint8_t reply[64];
	transport_spi_init();
	spi1_release(reply, sizeof reply); /* known state, independent of test order */
	(void)spi1_collect(spi1_configure_req, sizeof spi1_configure_req, reply, sizeof reply);

	const uint8_t xfer[] = {
		ALP_CC3501E_CMD_SPI1_TRANSFER,
		0x00u,
		0x0Cu,
		0x00u,
		0x04u,
		0x00u,
		0x00u,
		0x07u,
		0x00u,
		0x00u,
		0x00u,
		0x00u,
		0x11u,
		0x22u,
		0x33u,
		0x44u,
	};
	zassert_equal(spi1_collect(xfer, sizeof xfer, reply, sizeof reply),
	              ALP_CC3501E_RESP_OK,
	              "the first issue of seq 7 clocks the bus");

	/* Re-issue the IDENTICAL frame, which is what the host's ALP_ERR_IO retry
	 * does.  A fresh submit would answer BUSY first; answering OK on the very
	 * first transaction is the observable proof that the CACHED result came back
	 * and the device was not clocked a second time -- the difference between a
	 * repeated read and a double flash page program. */
	transaction(xfer, sizeof xfer);
	(void)drain(reply, sizeof reply);
	zassert_equal(reply[4], ALP_CC3501E_RESP_OK, "the same seq serves the cache, no re-clock");
	zassert_equal(reply[8], 0x07u, "the cached reply still echoes seq 7");
	zassert_mem_equal(&reply[9], &xfer[12], 4u, "the cached RX bytes come back intact");

	/* A DIFFERENT seq is a new logical transfer: the cache is dropped and the job
	 * resubmitted, so this one goes back to the BUSY/collect pair. */
	uint8_t next[sizeof xfer];
	memcpy(next, xfer, sizeof xfer);
	next[7] = 0x08u;
	zassert_equal(spi1_collect(next, sizeof next, reply, sizeof reply),
	              ALP_CC3501E_RESP_OK,
	              "a new seq starts a new transfer");
	zassert_equal(reply[8], 0x08u, "and its reply echoes the NEW seq");
}

ZTEST(cc3501e_bridge_transport, test_spi1_configure_refused_while_cs_is_held)
{
	uint8_t reply[64];
	transport_spi_init();
	spi1_release(reply, sizeof reply); /* known state, independent of test order */
	(void)spi1_collect(spi1_configure_req, sizeof spi1_configure_req, reply, sizeof reply);

	/* CS_HOLD leaves the chain open. */
	const uint8_t held[] = {
		ALP_CC3501E_CMD_SPI1_TRANSFER,
		0x00u,
		0x0Au,
		0x00u,
		0x02u,
		0x00u,
		ALP_CC3501E_SPI1_XFER_CS_HOLD,
		0x02u,
		0x00u,
		0x00u,
		0x00u,
		0x00u,
		0x5Au,
		0xA5u,
	};
	zassert_equal(spi1_collect(held, sizeof held, reply, sizeof reply),
	              ALP_CC3501E_RESP_OK,
	              "a CS_HOLD chunk -> OK");
	zassert_equal(reply[7],
	              (uint8_t)ALP_CC3501E_SPI1_XFER_CS_HOLD,
	              "the reply flags byte reads CS back as still asserted");

	/* Re-opening the instance underneath the chain would drop CS mid-transaction.
	 * ERR_STATE is the TERMINAL reject (0x09) the host does not re-poll. */
	transaction(spi1_configure_req, sizeof spi1_configure_req);
	(void)drain(reply, sizeof reply);
	zassert_equal(reply[4],
	              ALP_CC3501E_RESP_ERR_STATE,
	              "CONFIGURE during an unfinished CS_HOLD chain -> ERR_STATE");

	/* And the escape hatch still works, which is the whole point of RELEASE. */
	spi1_release(reply, sizeof reply);
}

ZTEST(cc3501e_bridge_transport, test_spi1_request_validation)
{
	uint8_t reply[64];
	transport_spi_init();
	spi1_release(reply, sizeof reply); /* known state, independent of test order */
	(void)spi1_collect(spi1_configure_req, sizeof spi1_configure_req, reply, sizeof reply);

	/* An undefined flag bit is REJECTED, not ignored: a host that sets a later
	 * firmware's flag must learn this peer cannot honour it. */
	const uint8_t bad_flag[] = {
		ALP_CC3501E_CMD_SPI1_TRANSFER,
		0x00u,
		0x09u,
		0x00u,
		0x01u,
		0x00u,
		0x08u,
		0x01u,
		0x00u,
		0x00u,
		0x00u,
		0x00u,
		0xFFu,
	};
	transaction(bad_flag, sizeof bad_flag);
	(void)drain(reply, sizeof reply);
	zassert_equal(reply[4], ALP_CC3501E_RESP_ERR_INVALID, "an undefined flag bit -> INVALID");

	/* The declared len must match the inline TX byte count EXACTLY -- a short
	 * frame would otherwise clock whatever sits past the payload. */
	const uint8_t short_tx[] = {
		ALP_CC3501E_CMD_SPI1_TRANSFER,
		0x00u,
		0x0Au,
		0x00u,
		0x04u,
		0x00u,
		0x00u,
		0x01u,
		0x00u,
		0x00u,
		0x00u,
		0x00u,
		0xAAu,
		0xBBu,
	};
	transaction(short_tx, sizeof short_tx);
	(void)drain(reply, sizeof reply);
	zassert_equal(reply[4], ALP_CC3501E_RESP_ERR_INVALID, "len 4 with 2 inline bytes -> INVALID");

	/* NO_TX carries NO inline bytes; sending some is the same disagreement. */
	const uint8_t no_tx_with_bytes[] = {
		ALP_CC3501E_CMD_SPI1_TRANSFER,
		0x00u,
		0x09u,
		0x00u,
		0x01u,
		0x00u,
		ALP_CC3501E_SPI1_XFER_NO_TX,
		0x01u,
		0x00u,
		0x00u,
		0x00u,
		0x00u,
		0xCCu,
	};
	transaction(no_tx_with_bytes, sizeof no_tx_with_bytes);
	(void)drain(reply, sizeof reply);
	zassert_equal(reply[4], ALP_CC3501E_RESP_ERR_INVALID, "NO_TX with inline TX bytes -> INVALID");

	/* CONFIGURE: v6 accepts 8 bits/word only, mode 0..3, CS0/CS1. */
	uint8_t cfg[sizeof spi1_configure_req];
	memcpy(cfg, spi1_configure_req, sizeof cfg);
	cfg[9] = 16u; /* bits_per_word */
	transaction(cfg, sizeof cfg);
	(void)drain(reply, sizeof reply);
	zassert_equal(reply[4], ALP_CC3501E_RESP_ERR_INVALID, "bits_per_word != 8 -> INVALID");

	memcpy(cfg, spi1_configure_req, sizeof cfg);
	cfg[8] = 4u; /* mode */
	transaction(cfg, sizeof cfg);
	(void)drain(reply, sizeof reply);
	zassert_equal(reply[4], ALP_CC3501E_RESP_ERR_INVALID, "mode > 3 -> INVALID");

	memcpy(cfg, spi1_configure_req, sizeof cfg);
	cfg[10] = 2u; /* cs */
	transaction(cfg, sizeof cfg);
	(void)drain(reply, sizeof reply);
	zassert_equal(reply[4], ALP_CC3501E_RESP_ERR_INVALID, "cs > 1 -> INVALID");

	/* RELEASE takes no payload, and a malformed one must NOT tear the bus down. */
	const uint8_t rel_with_payload[] = { ALP_CC3501E_CMD_SPI1_RELEASE, 0x00u, 0x01u, 0x00u, 0x00u };
	transaction(rel_with_payload, sizeof rel_with_payload);
	(void)drain(reply, sizeof reply);
	zassert_equal(reply[4], ALP_CC3501E_RESP_ERR_INVALID, "RELEASE with a payload -> INVALID");
}

/* The worker's `job` is a file-static singleton shared across the whole TU, so a
 * worker-routed test that submits but never collects its result (the body runs
 * synchronously on the stub and caches ERR) would leave the worker non-IDLE and
 * make EVERY later worker-routed poll report "other cmd busy".  Reset it before
 * each test so the cases are independent of order.
 *
 * protocol_reset_retry_latch() is the identical fix for the SAME class of bug
 * in protocol.c's issue-#102 retry latch: it too is a file-static that
 * outlives each ZTEST case, and without resetting it a latch entry one test
 * writes (e.g. BLE_ENABLE collected under seq 0, which every OLDER test uses
 * since none of them set flags) would be served back as a cache hit to a
 * LATER, unrelated test exercising the same opcode -- turning an expected
 * BUSY/submit into a served-from-cache status. */
static void reset_worker(void *fixture)
{
	(void)fixture;
	worker_init();
	protocol_reset_retry_latch();
}

ZTEST_SUITE(cc3501e_bridge_transport, NULL, NULL, reset_worker, NULL, NULL);
