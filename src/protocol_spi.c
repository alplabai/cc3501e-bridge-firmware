/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * cc3501e-bridge firmware: SPI1 host-passthrough command-family handlers
 * (0x55..0x57).  protocol_dispatch() in protocol.c owns the single
 * command-family switch that routes here.
 *
 * WHAT THIS BUS IS.  The E1M connector's SPI1 lands on the CC3501E, not on
 * the Alif (E1M-AEN-2626-R2 netlist: AG10 SCK -> CC35 GPIO_32, AG9 MOSI ->
 * GPIO_33, AG8 MISO -> GPIO_34, AH9 CS0 -> GPIO_31, AH8 CS1 -> GPIO_15).
 * The host therefore cannot drive it directly; the CC3501E is the SPI
 * CONTROLLER and relays the host's bytes.  These three opcodes are that
 * relay: CONFIGURE acquires the controller, TRANSFER clocks one chunk,
 * RELEASE frees it.
 *
 * NOT THE BRIDGE.  The inter-chip link this firmware answers on is CC35
 * SPI0 (GPIO_27/28/29 + GPIO16), a SLAVE instance.  Nothing in this family
 * may reconfigure those pads.
 *
 * WHY ALL THREE ARE WORKER-ROUTED.  The GPIO proxy runs inline in the SPI0
 * slave ISR because its bodies are register pokes.  A polled 4088-byte
 * master transfer is not: at 10 MHz it is ~800 us inside the ISR, which
 * stalls the slave's re-arm -- the wedge signature this firmware has spent
 * months chasing (see the worker-routing notes in protocol.c / worker.h).
 * SPI_open() in CONFIGURE can block on a power domain for the same reason.
 * So every opcode here submits to the single worker job and the host
 * poll-by-repeats, exactly like the Wi-Fi/BLE/socket families.  Side effect
 * worth stating: a Wi-Fi scan and a SPI1 transfer share that one slot and
 * serialise; the loser gets RESP_ERR_BUSY, which the host retries.
 *
 * ONLY TRANSFER HAND-ROLLS THE POLL LOOP, and the reason is DATA LOSS.
 * poll_by_repeat retries ALP_ERR_IO, and re-issuing a TRANSFER that already
 * ran would re-clock the far-end device -- a double flash page program, not
 * a repeated read.  The wire carries a seq byte for exactly that, so the
 * DONE edge here serves the CACHED result when the seq matches instead of
 * resetting the worker and re-clocking.  handle_worker_routed_payload_reply
 * resets unconditionally on DONE and so cannot express that; CONFIGURE and
 * RELEASE, which carry no such hazard, use the shared helpers unchanged.
 */

#include <stdbool.h>

#include "protocol_internal.h"

/* Every flag bit this firmware defines.  Anything outside it is rejected
 * rather than ignored, so a host that sets a LATER firmware's flag learns
 * this peer cannot honour it instead of having the flag silently dropped. */
#define SPI1_XFER_FLAGS_ALL \
	(ALP_CC3501E_SPI1_XFER_CS_HOLD | ALP_CC3501E_SPI1_XFER_NO_RX | ALP_CC3501E_SPI1_XFER_NO_TX)

/* Bus state, owned here rather than in the HAL because the HAL's transfer
 * entry point takes cs_hold as an argument and exposes no way to ask for it
 * back.  Both are written only from the handlers below, which run in the
 * SPI0 slave callback -- one context, so no volatile and no critical
 * section.
 *
 * g_cs_held mirrors the CS_HOLD echo bit of the last COLLECTED transfer's
 * reply (what the worker actually left on the wire), not what a not-yet-
 * collected request asked for, so there is one source of truth for it. */
static bool g_configured;
static bool g_cs_held;

/* SPI1_CONFIGURE (0x55): req = alp_cc3501e_spi1_configure_t (8 B); reply DATA =
 * alp_cc3501e_spi1_config_resp_t (8 B).  Idempotent -- re-issuing re-opens the
 * instance with the new parameters. */
alp_cc3501e_resp_t handle_spi1_configure(const uint8_t *req,
                                         size_t         req_len,
                                         uint8_t       *reply_data,
                                         size_t         reply_cap,
                                         size_t        *reply_data_len)
{
	*reply_data_len = 0u;
	if (req_len != sizeof(alp_cc3501e_spi1_configure_t)) return ALP_CC3501E_RESP_ERR_INVALID;

	const uint8_t mode          = req[4];
	const uint8_t bits_per_word = req[5];
	const uint8_t cs            = req[6];

	if (mode > 3u) return ALP_CC3501E_RESP_ERR_INVALID;           /* (CPOL << 1) | CPHA */
	if (bits_per_word != 8u) return ALP_CC3501E_RESP_ERR_INVALID; /* v6 accepts 8 only  */
	if (cs > (uint8_t)ALP_CC3501E_SPI1_CS1) return ALP_CC3501E_RESP_ERR_INVALID;

	/* Re-opening the instance underneath a half-finished CS_HOLD chain would
	 * drop CS mid-transaction and leave the far-end device parsing garbage.
	 * RESP_ERR_STATE is the TERMINAL reject the host does not retry (it peeks
	 * the 0x09 sentinel), which is right here: re-polling would never clear
	 * this by itself, the host has to finish the chain or RELEASE. */
	if (g_cs_held) return ALP_CC3501E_RESP_ERR_STATE;

	/* freq_hz is deliberately not range-checked: the divider rounds whatever it
	 * is handed, and the reply reports the rate the hardware actually produced
	 * so the host reads back rather than assumes. */
	const alp_cc3501e_resp_t st =
	    handle_worker_routed_payload_reply(ALP_CC3501E_CMD_SPI1_CONFIGURE,
	                                       req,
	                                       req_len,
	                                       sizeof(alp_cc3501e_spi1_config_resp_t),
	                                       reply_data,
	                                       reply_cap,
	                                       reply_data_len);
	if (st == ALP_CC3501E_RESP_OK) {
		g_configured = true;
		g_cs_held    = false; /* a fresh SPI_open leaves CS deasserted */
	}
	return st;
}

/* SPI1_TRANSFER (0x56): req = alp_cc3501e_spi1_transfer_t (8 B) + the TX bytes
 * inline (absent when NO_TX); reply DATA = alp_cc3501e_spi1_transfer_resp_t
 * (4 B) + the RX bytes inline (none when NO_RX).  flags == 0 is the cheap
 * single-shot (assert, clock, deassert); CS_HOLD continues the same device
 * transaction into the next chunk.  len == 0 with flags == 0 is a pure CS
 * deassert, which is why this family needs no CS opcode. */
alp_cc3501e_resp_t handle_spi1_transfer(const uint8_t *req,
                                        size_t         req_len,
                                        uint8_t       *reply_data,
                                        size_t         reply_cap,
                                        size_t        *reply_data_len)
{
	*reply_data_len = 0u;
	if (req_len < sizeof(alp_cc3501e_spi1_transfer_t)) return ALP_CC3501E_RESP_ERR_INVALID;

	const uint16_t len   = (uint16_t)req[0] | ((uint16_t)req[1] << 8);
	const uint8_t  flags = req[2];
	const uint8_t  seq   = req[3];
	const bool     no_rx = (flags & ALP_CC3501E_SPI1_XFER_NO_RX) != 0u;
	const bool     no_tx = (flags & ALP_CC3501E_SPI1_XFER_NO_TX) != 0u;

	if ((flags & (uint8_t)~SPI1_XFER_FLAGS_ALL) != 0u) return ALP_CC3501E_RESP_ERR_INVALID;
	if (len > ALP_CC3501E_SPI1_MAX_XFER) return ALP_CC3501E_RESP_ERR_INVALID;

	/* EXACT length check, not >=.  A short frame would make the worker clock
	 * whatever stale bytes sit past the payload in job.req; a long one means
	 * host and firmware disagree about the encoding, and guessing which is
	 * right is how a page program lands at the wrong offset. */
	const size_t want = no_tx ? sizeof(alp_cc3501e_spi1_transfer_t)
	                          : sizeof(alp_cc3501e_spi1_transfer_t) + (size_t)len;
	if (req_len != want) return ALP_CC3501E_RESP_ERR_INVALID;

	/* The reserved bytes (off 5..7) are deliberately NOT checked.  Rejecting an
	 * undefined FLAG bit stops a later firmware's flag from being silently
	 * ignored by this one; a reserved byte carries no such meaning, so
	 * rejecting it would only break hosts that fail to zero their padding. */

	if (!g_configured) return ALP_CC3501E_RESP_ERR_NOT_READY;

	const size_t min_cap = sizeof(alp_cc3501e_spi1_transfer_resp_t) + (no_rx ? 0u : (size_t)len);
	if (reply_cap < min_cap) return ALP_CC3501E_RESP_ERR_NO_MEM;

	size_t            n   = 0u;
	int8_t            err = 0;
	enum worker_state st =
	    worker_poll(ALP_CC3501E_CMD_SPI1_TRANSFER, reply_data, reply_cap, &n, &err);

	if (st == WORKER_DONE) {
		/* DUPLICATE SUPPRESSION.  The worker slot IS the cache: worker_poll does
		 * not reset, so a completed transfer's reply header + RX bytes are still
		 * sitting in it.  Serving them for a matching seq is what makes the
		 * host's ordinary ALP_ERR_IO re-issue safe on a bus that drives flash --
		 * a re-clock there is a SECOND page program, not a repeated read. */
		if (n >= sizeof(alp_cc3501e_spi1_transfer_resp_t) && reply_data[3] == seq) {
			g_cs_held       = (reply_data[2] & ALP_CC3501E_SPI1_XFER_CS_HOLD) != 0u;
			*reply_data_len = n;
			return ALP_CC3501E_RESP_OK;
		}
		/* Different seq: the host has moved on to the next logical transfer and
		 * will never come back for this result.  Drop it and start fresh.
		 *
		 * ponytail: the cache lives exactly as long as the worker slot does, and
		 * worker_poll() discards a terminal result the moment ANY other
		 * worker-routed opcode polls (its orphan-discard arm).  So a retry that
		 * arrives after an interleaved Wi-Fi/BLE/socket op does re-clock the bus.
		 * A dedicated (seq, reply) buffer closes that for 4 KB of static RAM; add
		 * it if a host ever interleaves inside a retry. */
		worker_reset();
		st = WORKER_IDLE;
	}

	switch (st) {
	case WORKER_ERR:
		worker_reset();
		/* The chain is broken and CS may be anywhere.  Clear the latch so the
		 * host's way back -- CONFIGURE, which re-opens, or RELEASE, which
		 * deasserts unconditionally -- is not itself refused with ERR_STATE. */
		g_cs_held = false;
		if (err == CC3501E_HW_ERR_NOTIMPL) return ALP_CC3501E_RESP_ERR_NOT_READY;
		if (err == CC3501E_HW_ERR_INVAL) return ALP_CC3501E_RESP_ERR_INVALID;
		/* EVERY other failure is RESP_ERR_STATE, and this is the one place the
		 * family deliberately diverges from hw_to_resp().  RESP_ERR_RADIO would
		 * map host-side to ALP_ERR_IO, which poll_by_repeat RETRIES -- and a
		 * retried transfer re-clocks the device.  A local controller refusing a
		 * transfer is deterministic anyway, so retrying only burns the poll
		 * budget to reach the same answer and then reports it as a timeout.
		 * RESP_ERR_STATE is already the terminal-reject code the host peeks for. */
		return ALP_CC3501E_RESP_ERR_STATE;
	case WORKER_IDLE:
		/* The whole frame (header + inline TX) goes to the worker, which
		 * re-parses it in the drain -- the same hand-off the socket family uses. */
		(void)worker_submit_payload(ALP_CC3501E_CMD_SPI1_TRANSFER, req, (uint16_t)req_len);
		return ALP_CC3501E_RESP_ERR_BUSY;
	default: /* QUEUED / RUNNING, or another opcode holding the slot */
		return ALP_CC3501E_RESP_ERR_BUSY;
	}
}

/* SPI1_RELEASE (0x57): no request payload, no reply data.  The escape hatch --
 * it must never fail on state, so releasing with nothing open is RESP_OK. */
alp_cc3501e_resp_t handle_spi1_release(const uint8_t *req,
                                       size_t         req_len,
                                       uint8_t       *reply_data,
                                       size_t         reply_cap,
                                       size_t        *reply_data_len)
{
	(void)req; /* RELEASE carries no request payload */

	/* Checked HERE rather than left to handle_worker_routed's identical check:
	 * a malformed RELEASE must be a pure parse rejection and must NOT fall
	 * through to the teardown below and drop a live CS_HOLD chain. */
	if (req_len != 0u) {
		*reply_data_len = 0u;
		return ALP_CC3501E_RESP_ERR_INVALID;
	}

	const alp_cc3501e_resp_t st = handle_worker_routed(
	    ALP_CC3501E_CMD_SPI1_RELEASE, 0u, req_len, reply_data, reply_cap, reply_data_len);
	/* Anything but BUSY is terminal for the job, and teardown must leave nothing
	 * latched even when the HAL reported a failure -- otherwise a botched
	 * release strands g_cs_held and locks CONFIGURE out forever, which is the
	 * exact wedge this opcode exists to break. */
	if (st != ALP_CC3501E_RESP_ERR_BUSY) {
		g_configured = false;
		g_cs_held    = false;
	}
	return st;
}
