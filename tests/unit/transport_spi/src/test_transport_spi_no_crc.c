/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * Unit tests for CC3501E_WIRE_CRC=0 (the customer-selectable no-CRC build,
 * CMakeLists.txt) -- built as a SEPARATE executable (see tests/unit/CMakeLists.txt)
 * against the SAME production sources as test_transport_spi.c, just compiled
 * with the opposite value of that switch.
 *
 * This file does NOT re-derive the ~50 cases test_transport_spi.c already
 * covers -- the opcode table, dispatch, and every handler's own payload
 * validation are identical code, untouched by CC3501E_WIRE_CRC.  It pins
 * only what CC3501E_WIRE_CRC=0 actually CHANGES in src/protocol.c's
 * protocol_build_reply() #else arm and src/protocol_meta.c's
 * CC3501E_FW_WIRE_VERSION / CC3501E_FW_IMPLEMENTS_PROTOCOL_MAJOR:
 *
 *   1. No CRC trailer is required on a request, or appended to a reply --
 *      the wire is byte-identical to the 3.1 shape (test_ping_ok_no_crc).
 *   2. GET_VERSION reports ALP_CC3501E_PROTOCOL_MAJOR_LEGACY (3), not
 *      ALP_CC3501E_PROTOCOL_MAJOR (4) (test_get_version_reports_legacy_major).
 *   3. The wire OK byte is ALP_CC3501E_RESP_OK_LEGACY (0x00), not
 *      ALP_CC3501E_RESP_OK (0x5A) (same test, and test_ping_ok_no_crc).
 *   4. GET_VERSION keeps its dual-shape carve-out even here: a request that
 *      DOES carry a valid CRC trailer is still accepted, not rejected
 *      (test_get_version_accepts_crc_trailer_too) -- the "in both builds"
 *      requirement from src/protocol.h's protocol_build_reply() doc.
 *   5. The CRC-reserved headroom macros (CC3501E_SPI1_MAX_XFER_V4,
 *      CC3501E_OTA_MAX_CHUNK_V4) resolve to the header's FULL, unreserved
 *      constants on this build, and CMD_SPI1_CONFIGURE's reply actually
 *      reports that value (test_spi1_configure_reports_unreserved_max_xfer)
 *      -- the "real trap" the task that added this option called out: a
 *      no-CRC build that still reported the CRC-reserved max would silently
 *      waste 2 bytes of every transfer the host clamps to.
 */

#include <string.h>
#include <zephyr/ztest.h>

#include "alp/protocol/cc3501e.h"
#include "alp/protocol/crc16.h" /* alp_crc16_ccitt_false[_update] -- for the one dual-shape vector */
#include "protocol.h"           /* CC3501E_REPLY_PAD, CC3501E_SPI1_MAX_XFER_V4 */
#include "transport.h"
#include "worker.h" /* worker_init -- the worker `job` is a static; reset it per test */

#if CC3501E_WIRE_CRC
#error "this TU exists to exercise CC3501E_WIRE_CRC=0; check tests/unit/CMakeLists.txt"
#endif

/* Expected WIRE length of a reply carrying @p data_len bytes of data on the
 * CC3501E_WIRE_CRC=0 shape: header + (status + data) padded to
 * CC3501E_REPLY_PAD, NO CRC trailer -- contrast test_transport_spi.c's
 * reply_wire(), which adds ALP_CC3501E_CRC_BYTES before padding. */
static inline size_t reply_wire_no_crc(size_t data_len)
{
	const size_t unpadded = 1u + data_len;
	const size_t padded =
	    ((unpadded + CC3501E_REPLY_PAD - 1u) / CC3501E_REPLY_PAD) * CC3501E_REPLY_PAD;

	return (size_t)ALP_CC3501E_HEADER_BYTES + padded;
}

/* Sends @p bytes exactly as given -- there is no CRC to append on this wire,
 * so every request is already in its final on-wire form (contrast
 * test_transport_spi.c's transaction(), which reframes a logical request
 * into the CRC-bearing wire shape). */
static void send(const uint8_t *bytes, size_t len)
{
	spi_slave_cs_low();
	for (size_t i = 0; i < len; i++) {
		spi_slave_rx_byte(bytes[i]);
	}
	spi_slave_cs_high();
}

static size_t drain(uint8_t *out, size_t cap)
{
	size_t n = 0;
	while (spi_slave_tx_pending() && n < cap) {
		out[n++] = spi_slave_tx_next_byte();
	}
	return n;
}

ZTEST(cc3501e_bridge_transport_no_crc, test_ping_ok_no_crc)
{
	/* header(4) + payload(0) -- no trailer -- byte-identical to what the
	 * 3.1 wire (pre-#2035) sent for the same request. */
	const uint8_t ping[] = { ALP_CC3501E_CMD_PING, 0x00u, 0x00u, 0x00u };
	uint8_t       reply[32];

	transport_spi_init();
	send(ping, sizeof ping);
	size_t n = drain(reply, sizeof reply);

	zassert_equal(n, reply_wire_no_crc(0u), "no-CRC PING reply has no trailer");
	zassert_equal(reply[0], ALP_CC3501E_CMD_PING, "reply echoes the request cmd");
	zassert_equal(reply[1], 0x00u, "solicited reply: flags == 0");
	zassert_equal((uint16_t)(reply[2] | ((uint16_t)reply[3] << 8)),
	              (uint16_t)CC3501E_REPLY_PAD,
	              "declared payload_len is JUST the padded status (no +CRC_BYTES)");
	zassert_equal(reply[4],
	              ALP_CC3501E_RESP_OK_LEGACY,
	              "the wire OK byte on this build is 0x00 (RESP_OK_LEGACY), not 0x5A");
	/* Byte-exact match against the whole 12-byte reply, the same assertion
	 * style as tests/protocol_vectors_no_crc.txt's ping_reply_ok vector. */
	const uint8_t expect[] = {
		0x00u, 0x00u, 0x08u, 0x00u, /* header: cmd=PING flags=0 len=8      */
		0x00u, 0x00u, 0x00u, 0x00u, /* status=OK_LEGACY + zero pad ...     */
		0x00u, 0x00u, 0x00u, 0x00u, /* ... (7 pad bytes total, no CRC)     */
	};
	zassert_mem_equal(reply, expect, sizeof expect, "byte-identical to the 3.1 wire");
}

ZTEST(cc3501e_bridge_transport_no_crc, test_get_version_reports_legacy_major)
{
	const uint8_t gv[] = { ALP_CC3501E_CMD_GET_VERSION, 0x00u, 0x00u, 0x00u };
	uint8_t       reply[32];

	transport_spi_init();
	send(gv, sizeof gv);
	size_t n = drain(reply, sizeof reply);

	zassert_equal(n, reply_wire_no_crc(2u), "GET_VERSION reply is header + status + u16, no CRC");
	zassert_equal(reply[4], ALP_CC3501E_RESP_OK_LEGACY, "GET_VERSION -> the legacy OK byte");
	const uint16_t version = (uint16_t)reply[5] | ((uint16_t)reply[6] << 8);
	const uint8_t  major   = (uint8_t)(version >> 8);
	zassert_equal(major,
	              (uint8_t)ALP_CC3501E_PROTOCOL_MAJOR_LEGACY,
	              "a CC3501E_WIRE_CRC=0 build MUST report MAJOR 3 (LEGACY), never 4 -- "
	              "reporting 4 here would tell a MAJOR-4 host this peer speaks a wire it "
	              "does not (see src/protocol_meta.c's CC3501E_FW_IMPLEMENTS_PROTOCOL_MAJOR)");
	zassert_true(major != (uint8_t)ALP_CC3501E_PROTOCOL_MAJOR, "must not equal MAJOR 4 either");
}

ZTEST(cc3501e_bridge_transport_no_crc, test_get_version_accepts_crc_trailer_too)
{
	/* "GET_VERSION is accepted with or without a CRC trailer, in both
	 * builds" -- a host with a stale cached major (e.g. this board was
	 * OTA'd DOWN from a MAJOR-4 image) may send the CRC'd shape even to a
	 * legacy peer; this build must still answer it, with its ordinary
	 * legacy-shaped (no-CRC) reply -- not reject it. */
	uint8_t        framed[8];
	const uint16_t wire_len = (uint16_t)ALP_CC3501E_CRC_BYTES;

	framed[0]          = ALP_CC3501E_CMD_GET_VERSION;
	framed[1]          = 0x00u;
	framed[2]          = (uint8_t)(wire_len & 0xFFu);
	framed[3]          = (uint8_t)((wire_len >> 8) & 0xFFu);
	const uint16_t crc = alp_crc16_ccitt_false(framed, ALP_CC3501E_HEADER_BYTES);
	framed[4]          = (uint8_t)(crc & 0xFFu);
	framed[5]          = (uint8_t)((crc >> 8) & 0xFFu);

	uint8_t reply[32];

	transport_spi_init();
	send(framed, (size_t)ALP_CC3501E_HEADER_BYTES + wire_len);
	size_t n = drain(reply, sizeof reply);

	zassert_equal(
	    n, reply_wire_no_crc(2u), "still gets the full legacy-shaped reply, not PROTOCOL");
	zassert_equal(
	    reply[4], ALP_CC3501E_RESP_OK_LEGACY, "accepted -> the legacy OK byte, not rejected");
	const uint16_t version = (uint16_t)reply[5] | ((uint16_t)reply[6] << 8);
	zassert_equal((uint8_t)(version >> 8),
	              (uint8_t)ALP_CC3501E_PROTOCOL_MAJOR_LEGACY,
	              "the reply itself is still the plain legacy shape, un-influenced by the "
	              "request having carried a CRC");
}

ZTEST(cc3501e_bridge_transport_no_crc, test_ping_with_trailing_bytes_is_invalid)
{
	/* Unlike GET_VERSION, PING has no dual-shape carve-out: two trailing
	 * bytes are just an unexpected non-empty payload to handle_ping(),
	 * rejected on the handler's own terms -- exactly like a real 3.1
	 * firmware, and proof this build does not silently widen CRC
	 * tolerance to opcodes other than GET_VERSION. */
	const uint8_t ping_pl[] = { ALP_CC3501E_CMD_PING, 0x00u, 0x02u, 0x00u, 0xDEu, 0xADu };
	uint8_t       reply[32];

	transport_spi_init();
	send(ping_pl, sizeof ping_pl);
	size_t n = drain(reply, sizeof reply);

	zassert_equal(n, reply_wire_no_crc(0u), "reply is header + status, no CRC");
	zassert_equal(reply[4], ALP_CC3501E_RESP_ERR_INVALID, "PING with a payload -> INVALID");
}

/* CONFIGURE at 10 MHz (0x00989680), mode 0, 8 bits/word, CS0 -- same request
 * test_transport_spi.c's test_spi1_configure_then_transfer_loops_back uses. */
static const uint8_t spi1_configure_req[] = {
	ALP_CC3501E_CMD_SPI1_CONFIGURE,
	0x00u,
	0x08u, /* payload_len 8 -- no CRC tax on this wire */
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

ZTEST(cc3501e_bridge_transport_no_crc, test_spi1_configure_reports_unreserved_max_xfer)
{
	/* Worker-routed: submit (BUSY) then collect, same as every worker-routed
	 * opcode on the silicon-free stub (the job runs synchronously). */
	uint8_t reply[32];

	transport_spi_init();
	send(spi1_configure_req, sizeof spi1_configure_req);
	(void)drain(reply, sizeof reply);
	zassert_equal(reply[4], ALP_CC3501E_RESP_ERR_BUSY, "worker-routed submit acks BUSY");

	send(spi1_configure_req, sizeof spi1_configure_req);
	size_t n = drain(reply, sizeof reply);
	(void)n;
	zassert_equal(reply[4], ALP_CC3501E_RESP_OK_LEGACY, "CONFIGURE -> OK (the legacy OK byte)");

	const uint16_t max_xfer = (uint16_t)reply[9] | ((uint16_t)reply[10] << 8);
	zassert_equal(
	    max_xfer, (uint16_t)CC3501E_SPI1_MAX_XFER_V4, "CONFIGURE reports CC3501E_SPI1_MAX_XFER_V4");
	zassert_equal(max_xfer,
	              (uint16_t)ALP_CC3501E_SPI1_MAX_XFER,
	              "on CC3501E_WIRE_CRC=0 that resolves to the header's FULL, unreserved "
	              "ALP_CC3501E_SPI1_MAX_XFER -- there is no CRC trailer eating 2 bytes of "
	              "payload_len on this wire, so reserving them would silently waste 2 bytes "
	              "of every transfer the host clamps to (the trap this option's task named)");
}

static void reset_worker(void *fixture)
{
	(void)fixture;
	worker_init();
	protocol_reset_retry_latch();
}

ZTEST_SUITE(cc3501e_bridge_transport_no_crc, NULL, NULL, reset_worker, NULL, NULL);
