/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * cc3501e-bridge HAL: TI backend -- host SPI1 passthrough (proto v6).
 *
 * NAMING, because this tree already uses "SPI1" for something else: everywhere
 * in the bridge transport, "SPI1" means the ALIF's SPI1 controller driving the
 * inter-chip link into the CC3501E's SPI0 slave.  Here "SPI1" means the
 * CC3501E's OWN SPI1 controller -- a SECOND, unrelated bus.  The E1M connector's
 * SPI1 pins do not land on the Alif at all; the E1M-AEN-2626-R2 netlist wires
 * them to this chip:
 *
 *   connector E1   net              CC3501E pad   role here
 *   AG10 SPI1_SCLK WIFI_SPI1.SCK    GPIO_32       SCK   (controller drives)
 *   AG9  SPI1_MOSI WIFI_SPI1.MOSI   GPIO_33       data
 *   AG8  SPI1_MISO WIFI_SPI1.MISO   GPIO_34       data
 *   AH9  SPI1_CS0  WIFI_SPI1.CS0    GPIO_31       CS0 (software, active low)
 *   AH8  SPI1_CS1  WIFI_SPI1.CS1    GPIO_15       CS1 (software, active low)
 *
 * So a host that wants to talk to a peripheral on the connector's SPI1 cannot
 * clock it itself: the CC3501E is the only controller on that bus, and these
 * three entry points relay the host's transfers onto it.  Which CC35 pad is
 * PICO and which is POCI is a SysConfig question, not a question for this file
 * -- see the board file for the MOSI/MISO assignment note.
 *
 * UNTOUCHABLE, and nothing in this file goes near it: the inter-chip bridge is
 * CONFIG_SPI_0 on GPIO_16/27/28/29, opened as a PERIPHERAL by
 * transport_hw_ti_spi.c.  Separate SysConfig instance, separate pads, separate
 * DMA channels.  Breaking that link bricks the board's only control path.
 *
 * THREADING: every body here MUST run on the worker/bring-up task, never in the
 * SPI0 dispatch ISR.  SPI_open() can block on a power domain and SPI_transfer()
 * in SPI_MODE_BLOCKING pends on a semaphore (or, above the driver's
 * minDmaTransferSize, on a DMA completion) -- either one inside the slave's ISR
 * stalls its re-arm, which is exactly the wedge signature this firmware spent
 * months chasing.  The protocol layer routes CMD_SPI1_* through the worker for
 * that reason; do not add an inline dispatch path.
 *
 * Built ONLY for CC3501E_HAL_BACKEND=ti (the bench build), against TI's
 * SimpleLink CC35xx SDK.  CI builds the stub backend instead, so this file is
 * never on the SDK-free path.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "ti_drivers_config.h" /* CONFIG_SPI_1 (SysConfig board file) */

#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/dpl/ClockP.h> /* tick period -> a FINITE transfer timeout */

#include "alp/protocol/cc3501e.h"

#include "../cc3501e_hw.h"

#ifndef CONFIG_SPI_1
/* Fail with the actual cause instead of an undeclared-identifier cascade: the
 * SysConfig board file has no second SPI instance yet.  It must be declared
 * AFTER spi0 in ti/cc3501e_aen_wifi.syscfg -- instance ORDER is what keeps
 * CONFIG_SPI_0 == 0, and re-ordering silently moves the whole inter-chip
 * transport to a different index. */
#error \
    "CONFIG_SPI_1 missing: add the SPI1 controller instance (after spi0) to the SysConfig board file"
#endif

#if !defined(CC3501E_WIFI)
/* Every body in this file MUST run on the worker/bring-up task (see the file
 * header) -- but worker_submit_payload() only defers to that task when
 * CC3501E_WIFI is defined (src/worker.c); without it, it calls
 * worker_execute() SYNCHRONOUSLY from wherever it was called, which for
 * CMD_SPI1_TRANSFER is the SPI0 slave dispatch callback.  A polled 4088-byte
 * controller transfer at 10 MHz is ~800 us -- exactly the ISR stall this
 * file's THREADING note exists to forbid.  Fail the build instead of linking
 * a silent ISR-stall trap. */
#error \
    "cc3501e_hw_ti_spi_master.c requires CC3501E_WIFI (the deferred worker): the synchronous stub path would run a 4088-byte transfer in the SPI0 ISR"
#endif
#endif

/* Software chip-selects.  BOTH are GPIOs driven by this file, not by the SPI
 * IP: SPIWFF3DMA carries exactly ONE hardware csnSel per SPI_Config entry, so a
 * single instance cannot hardware-frame two selects, and
 * SPIWFF3DMA_CMD_SET_CSN_PIN re-muxes a new pad with the OLD csnPinMux (GPIO31
 * wants mux 4, GPIO15 wants mux 16 -- it would apply the wrong function).  The
 * instance therefore runs "Three Pin" and the CS edges below are scheduler-timed,
 * not clock-edge-exact.  A peripheral demanding sub-microsecond CS-to-first-clock
 * setup will not work on this bus, and no protocol change fixes that. */
#define SPI1_CS0_PAD 31u /* connector E1 AH9, net WIFI_SPI1.CS0 */
#define SPI1_CS1_PAD 15u /* connector E1 AH8, net WIFI_SPI1.CS1 */

/* Open instance, or NULL when the bus is released.  Doubles as the "was
 * CONFIGURE ever accepted" flag -- a TRANSFER with no handle is the NOT_READY
 * case, not a failure worth retrying. */
static SPI_Handle spi1;

/* Which pad the last CONFIGURE selected, and whether it is currently driven LOW
 * (asserted).  cs_held is the open-CS_HOLD-chain state the CONFIGURE guard
 * rejects on: re-opening the instance mid-chain would drop CS on the floor and
 * leave the far-end device half-way through a command. */
static uint8_t cs_pad = SPI1_CS0_PAD;
static bool    cs_held;

/* Fill window for a NO_TX transfer (tx == NULL -> clock `len` copies of
 * tx_fill).  A literal NULL txBuf is NOT usable here: SPIWFF3DMA's arm path
 * treats it as "no TX descriptor to program" and can refuse the transfer
 * outright -- the same trap transport_hw_ti_spi.c's dummy_tx_zero documents.
 *
 * ponytail: 64-byte window walked in a loop instead of a MAX_XFER-sized static
 * buffer -- 4 KB of permanent BSS to send one repeated byte is not a trade this
 * part's RAM budget wants.  CS stays asserted across the window boundaries, so
 * the wire sees one transaction with small gaps in SCK (legal on a synchronous
 * bus).  If a bench measurement ever shows the per-window overhead dominating a
 * read-only flash drain, widen this buffer -- nothing else changes. */
static uint8_t fill_chunk[64];

/* Clamp the blocking transfer's wait so a stolen DMA channel cannot park the
 * worker task forever.  The radio has been observed re-muxing DMA channels out
 * from under the bridge SPI (which is why CONFIG_SPI_0 pins ch12/13); if that
 * ever happens to this instance the transfer never completes, and
 * SPI_WAIT_FOREVER would turn it into a permanently dead worker -- i.e. a dead
 * bridge -- rather than one failed transfer.  Budget = a full MAX_XFER chunk at
 * the configured clock plus 100 ms of slack, so a slow bus is never cut short. */
static uint32_t xfer_timeout_ticks(uint32_t freq_hz)
{
	const uint64_t us = (((uint64_t)ALP_CC3501E_SPI1_MAX_XFER * 8u * 1000000u) / freq_hz) + 100000u;
	uint32_t       period_us = (uint32_t)ClockP_getSystemTickPeriod();
	if (period_us == 0u) {
		period_us = 1000u; /* never divide by a bogus DPL answer */
	}
	uint64_t ticks = (us + period_us - 1u) / period_us;
	if (ticks == 0u) {
		ticks = 1u;
	}
	if (ticks > 0xFFFFFFFEu) {
		ticks = 0xFFFFFFFEu; /* stay clear of SPI_WAIT_FOREVER (~0) */
	}
	return (uint32_t)ticks;
}

static void cs_drive(bool assert_low)
{
	GPIO_write(cs_pad, assert_low ? 0u : 1u);
	cs_held = assert_low;
}

/* One vendor transfer.  Returns false for BOTH "the driver refused the arm" and
 * "it completed with anything other than COMPLETED" -- the caller maps that to
 * CC3501E_HW_ERR_STATE, deliberately NOT ERR_IO: a local controller refusing to
 * clock is deterministic, and ERR_IO reaches the host as ALP_ERR_IO, which
 * poll_by_repeat RETRIES -- burning the poll budget to reach the same answer and
 * ending in a misleading timeout. */
static bool spi1_xfer(const void *tx, void *rx, size_t count)
{
	/* A LOCAL transaction is safe here only because the mode is BLOCKING: the
	 * driver is done with the descriptor by the time SPI_transfer() returns.
	 * (transport_hw_ti_spi.c must keep its transaction static -- callback mode,
	 * where the driver still owns it after the call.)  Zeroed whole so any SDK
	 * field this file does not know about starts defined. */
	SPI_Transaction t;
	memset(&t, 0, sizeof(t));
	t.count = count; /* FRAMES; dataSize is 8, so frames == bytes */
	t.txBuf = (void *)tx;
	t.rxBuf = rx; /* NULL is fine on the RX side -- the reply phases already rely on it */
	t.arg   = NULL;
	if (!SPI_transfer(spi1, &t)) {
		return false;
	}
	return t.status == SPI_TRANSFER_COMPLETED;
}

int cc3501e_hw_spi1_configure(uint32_t  freq_hz,
                              uint8_t   mode,
                              uint8_t   bits_per_word,
                              uint8_t   cs,
                              uint32_t *actual_freq_hz_out)
{
	static const SPI_FrameFormat fmt[4] = {
		SPI_POL0_PHA0, SPI_POL0_PHA1, SPI_POL1_PHA0, SPI_POL1_PHA1
	};

	/* freq_hz == 0 is rejected alongside the wire-validated fields: it is a
	 * nonsense clock AND it would divide by zero in the timeout budget above. */
	if ((freq_hz == 0u) || (mode > 3u) || (bits_per_word != 8u) ||
	    (cs > (uint8_t)ALP_CC3501E_SPI1_CS1)) {
		return CC3501E_HW_ERR_INVAL;
	}
	if (cs_held) {
		/* An unfinished CS_HOLD chain owns the bus.  Terminal reject (-> RESP_ERR_STATE,
		 * which the host does not retry): re-opening the instance underneath a
		 * half-issued device command would strand that device mid-transaction.  The
		 * host finishes its chain, or calls RELEASE. */
		return CC3501E_HW_ERR_STATE;
	}

	/* Idempotent by contract: a second CONFIGURE re-opens with new parameters. */
	if (spi1 != NULL) {
		SPI_close(spi1);
		spi1 = NULL;
	}

	/* Drive the select DEASSERTED before the instance exists, so opening the bus
	 * cannot glitch a device into listening.  A previously selected pad is left as
	 * an output driven HIGH -- also deasserted, and re-muxing it back to an input
	 * would let its net float on a board whose pull is external. */
	cs_pad = (cs == (uint8_t)ALP_CC3501E_SPI1_CS1) ? SPI1_CS1_PAD : SPI1_CS0_PAD;
	if (GPIO_setConfig(cs_pad,
	                   GPIO_CFG_OUTPUT_INTERNAL | GPIO_CFG_PULL_NONE_INTERNAL |
	                       GPIO_CFG_OUT_HIGH) != 0) {
		/* The pad is not GPIO-configurable -- on this part that means the SysConfig
		 * table emitted GPIOWFF3_DO_NOT_CONFIG for it, in which case GPIO_write()
		 * is silently DROPPED and CS never moves.  That is the GPIO17 READY-line
		 * failure repeating; report it instead of clocking a bus whose select is
		 * stuck high. */
		return CC3501E_HW_ERR_IO;
	}
	cs_drive(false);

	SPI_Params params;
	SPI_Params_init(&params);
	params.mode            = SPI_CONTROLLER; /* the CC35 clocks this bus; see the file header */
	params.frameFormat     = fmt[mode];      /* mode = (CPOL << 1) | CPHA */
	params.dataSize        = bits_per_word;
	params.bitRate         = freq_hz;
	params.transferMode    = SPI_MODE_BLOCKING;
	params.transferTimeout = xfer_timeout_ticks(freq_hz);

	spi1 = SPI_open(CONFIG_SPI_1, &params);
	if (spi1 == NULL) {
		/* ERR_IO -> RESP_ERR_RADIO -> ALP_ERR_IO, which the host DOES retry, and that
		 * is the right call here (unlike a refused transfer): an open loses to a
		 * handle that has not finished closing or to a momentarily busy DMA, both of
		 * which clear on their own.  RESP_ERR_RADIO on a SPI1 op means bus-level open
		 * failure, NOT anything RF. */
		return CC3501E_HW_ERR_IO;
	}

	if (actual_freq_hz_out != NULL) {
		/* KNOWN GAP, stated rather than papered over: TI Drivers exposes no read-back
		 * of the rate the divider actually produced, and the CC35xx SPI divider
		 * formula is not published in SWRU626.  Echoing the REQUEST here would be a
		 * fabricated hardware fact -- the host's own doc promises "the SCK the
		 * divider ACTUALLY produced", and a real divider rounds, so a value that is
		 * always bit-identical to the request is not that.  0 is the honest answer
		 * until a real read-back or a computed quantisation exists; the console
		 * prints it as "unknown" rather than a bare 0 Hz.
		 * ponytail: report unknown now, derive when the source clock is confirmed
		 * on silicon. */
		*actual_freq_hz_out = 0u;
	}
	return CC3501E_HW_OK;
}

int cc3501e_hw_spi1_transfer(const uint8_t *tx,
                             uint8_t       *rx,
                             uint16_t       len,
                             uint8_t        tx_fill,
                             bool           cs_hold)
{
	if (spi1 == NULL) {
		/* No CONFIGURE has been accepted.  NOTIMPL is the HAL code that maps to
		 * RESP_ERR_NOT_READY (hw_to_resp), which is the contract's answer for a
		 * TRANSFER on an unopened bus. */
		return CC3501E_HW_ERR_NOTIMPL;
	}
	if (len > ALP_CC3501E_SPI1_MAX_XFER) {
		return CC3501E_HW_ERR_INVAL;
	}

	/* Assert unconditionally: the pad is already LOW when this chunk continues a
	 * CS_HOLD chain, and re-driving the same level is a no-op on the wire. */
	cs_drive(true);

	int rv = CC3501E_HW_OK;
	if (len == 0u) {
		/* len 0 is a pure CS poke -- with cs_hold clear it is the standalone
		 * deassert that saves this family a fourth opcode.  Never hand the driver a
		 * zero-frame transaction. */
	} else if (tx != NULL) {
		rv = spi1_xfer(tx, rx, len) ? CC3501E_HW_OK : CC3501E_HW_ERR_STATE;
	} else {
		/* NO_TX: clock `len` copies of tx_fill through the small window.  The fill
		 * byte matters -- 0xFF and 0x00 are not interchangeable on a part that
		 * decodes MOSI during a read -- so it is refreshed per call, never assumed. */
		memset(fill_chunk, tx_fill, sizeof(fill_chunk));
		uint16_t left = len;
		while (left > 0u) {
			const uint16_t n =
			    (left > (uint16_t)sizeof(fill_chunk)) ? (uint16_t)sizeof(fill_chunk) : left;
			if (!spi1_xfer(fill_chunk, rx, n)) {
				rv = CC3501E_HW_ERR_STATE;
				break;
			}
			if (rx != NULL) {
				rx += n;
			}
			left -= n;
		}
	}

	/* Release CS when the host said so -- and ALSO on failure, whatever it asked
	 * for.  A failed chunk means the far-end device's transaction is already
	 * unrecoverable; holding CS on top of that would strand the bus until the host
	 * noticed and issued RELEASE, and would block the CONFIGURE it needs to
	 * re-establish known state. */
	if (!cs_hold || (rv != CC3501E_HW_OK)) {
		cs_drive(false);
	}
	return rv;
}

int cc3501e_hw_spi1_release(void)
{
	/* The escape hatch: it must never fail on state, so a host that lost track of
	 * a CS_HOLD chain always has a way back to a clean bus.  Nothing open is
	 * success, not an error. */
	if (cs_held) {
		cs_drive(false);
	}
	if (spi1 != NULL) {
		SPI_close(spi1);
		spi1 = NULL;
	}
	return CC3501E_HW_OK;
}
