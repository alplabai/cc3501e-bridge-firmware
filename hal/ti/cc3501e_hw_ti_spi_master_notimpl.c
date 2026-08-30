/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * cc3501e-bridge HAL: TI backend -- SPI1 host-passthrough NOT-SUPPORTED
 * stand-in, linked in place of cc3501e_hw_ti_spi_master.c for the DEFAULT
 * (non -WifiHostDriver) bench build.
 *
 * The E1M connector's SPI1 bus only gets a CONFIG_SPI_1 controller instance
 * in ti/cc3501e_aen_wifi.syscfg (see that file's "SPI1 controller (master)"
 * block); the silicon-validated default board file (ti/cc3501e_aen.syscfg)
 * declares no second SPI instance, so cc3501e_hw_ti_spi_master.c's own
 * #ifndef CONFIG_SPI_1 #error refuses to build against it.  worker.c still
 * calls cc3501e_hw_spi1_configure/transfer/release unconditionally (protocol
 * v6's CMD_SPI1_CONFIGURE/TRANSFER/RELEASE are always dispatched), so the
 * default image needs SOME definition of all three to link.
 *
 * configure() and transfer() answer CC3501E_HW_ERR_NOTIMPL, which hw_to_resp()
 * maps to RESP_ERR_NOT_READY -- the same "not wired on this build" answer the
 * silicon-free stub backend (hal/cc3501e_hw_stub.c) gives before its first
 * CONFIGURE, and exactly the case <alp/protocol/cc3501e.h> documents for
 * cc3501e_hw_spi1_transfer(): "the backend has no SPI1 at all".  release()
 * does NOT follow that pattern: it answers CC3501E_HW_OK, because the escape
 * hatch must never fail on state and "no bus was ever open" already IS the
 * released state.  Build with -WifiHostDriver for a working configure/transfer.
 */

#include <stdbool.h>
#include <stdint.h>

#include "../cc3501e_hw.h"

int cc3501e_hw_spi1_configure(uint32_t  freq_hz,
                              uint8_t   mode,
                              uint8_t   bits_per_word,
                              uint8_t   cs,
                              uint32_t *actual_freq_hz_out)
{
	(void)freq_hz;
	(void)mode;
	(void)bits_per_word;
	(void)cs;
	(void)actual_freq_hz_out;
	return CC3501E_HW_ERR_NOTIMPL;
}

int cc3501e_hw_spi1_transfer(const uint8_t *tx,
                             uint8_t       *rx,
                             uint16_t       len,
                             uint8_t        tx_fill,
                             bool           cs_hold)
{
	(void)tx;
	(void)rx;
	(void)len;
	(void)tx_fill;
	(void)cs_hold;
	return CC3501E_HW_ERR_NOTIMPL;
}

int cc3501e_hw_spi1_release(void)
{
	/* The escape hatch must NEVER fail on state -- <alp/chips/cc3501e/core.h>
	 * documents cc3501e_spi1_release() as unable to fail on state, the console
	 * text says the same, and hal/cc3501e_hw_stub.c (the silicon-free backend)
	 * answers OK here too.  NOTIMPL would contradict all three: on THIS build
	 * there was never a bus to release, which is success, not a failure to
	 * report.  Leave NOTIMPL to configure()/transfer(), the two bodies that
	 * genuinely have no answer without CONFIG_SPI_1. */
	return CC3501E_HW_OK;
}
