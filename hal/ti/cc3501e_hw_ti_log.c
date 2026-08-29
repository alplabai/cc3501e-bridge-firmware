/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * cc3501e-bridge HAL: TI backend -- log level + diagnostics (GET_DIAG_INFO
 * sources: reset cause, uptime, free heap).
 *
 * Split by hardware subsystem out of cc3501e_hw_ti.c (issue #703, #461
 * Phase B).  cc3501e_hw_ti.c keeps platform lifecycle + the deferred-reboot
 * latch; see cc3501e_hw_ti_internal.h for the cross-TU seam.
 *
 * Built ONLY for CC3501E_HAL_BACKEND=ti (the bench build), against TI's
 * SimpleLink CC35xx SDK.  CI builds the stub backend instead, so this file
 * is never on the SDK-free path.
 */

#include <stdbool.h>
#include <stdint.h>

/* PowerWFF3_getResetReason() -- the SUPPORTED accessor for the reset cause.
 * Do NOT read PRCM_SCRATCHPAD RSTCAUS directly: see the note on
 * cc3501e_hw_reset_cause() below for what that cost. */
#include <ti/drivers/power/PowerWFF3.h>

#include <ti/drivers/dpl/ClockP.h> /* uptime source for GET_DIAG_INFO (no radio needed) */

#include "alp/protocol/cc3501e.h"

#include "../cc3501e_hw.h"

int cc3501e_hw_set_log_level(uint8_t level)
{
	(void)level;
	return CC3501E_HW_OK;
}

/* Reset cause, via the SDK's supported accessor.
 *
 * SWRU626 7.2.1: "CC35xx HW keeps information on the reset cause ... The reset
 * cause information is also transferred to application FW."  This used to
 * hardcode ALP_CC3501E_RESET_UNKNOWN, so a wedged run could not be told apart
 * from a clean cold boot -- and that distinction is the point, because #5's
 * residual wedge (~1 run in 12) has now survived five separate fixes.
 *
 * DO NOT READ PRCM_SCRATCHPAD RSTCAUS DIRECTLY.  I tried, at
 * PRCM_SCRATCHPAD_BASE + PRCM_SCRATCHPAD_O_RSTCAUS, and it BROKE GET_DIAG_INFO
 * on silicon: `alp companion ver` still answered "CC3501E protocol v5" while
 * `alp companion diag info` returned -5 every time, because the handler runs
 * from the SPI dispatch callback and the faulting read killed the reply.
 *
 * The reason is that hw_memmap.h's PRCM_SCRATCHPAD_BASE (0x4109F000) does not
 * agree with the address the SDK's own working code uses: psa_fwu.c defines
 * PRCM_SCRATCHPAD__PRCM_SCPAD2__ADDR as 0x41099000, and with LINE2 at offset
 * 0x1000 that puts the bank at 0x41098000.  The header constant is not the
 * address the silicon answers on.
 *
 * PowerWFF3_getResetReason() is the supported accessor and lives in
 * drivers_cc35xx.a, which this image already links.  Its PowerWFF3_ResetReason
 * values are RSTCAUS bit COMBINATIONS -- every real cause also carries
 * RSTLINE|POR -- so they are matched as masks, most specific first, not
 * compared for equality against a single bit. */
uint8_t cc3501e_hw_reset_cause(void)
{
	static uint8_t latched;
	static bool    have_latched;

	if (have_latched) {
		return latched;
	}

	const PowerWFF3_ResetReason reason = PowerWFF3_getResetReason();

	/* Order is deliberate: the SDK's enum values OVERLAP (PowerWFF3_RESET_M33WD
	 * is RSTLINE|POR|M33WD, so it also matches RESET_PIN_POR's bits).  Test the
	 * specific causes BEFORE the generic pin/POR one, or a watchdog reset reports
	 * as a power-on -- which is exactly the confusion this function exists to
	 * remove. */
	switch (reason) {
	case PowerWFF3_RESET_M33WD:
		latched = (uint8_t)ALP_CC3501E_RESET_WATCHDOG;
		break;
	case PowerWFF3_RESET_BOD:
	case PowerWFF3_RESET_RVML:
	case PowerWFF3_RESET_RVMH:
		/* Table 7-4's rail monitors and brownout detect: the host only needs
		 * "the rails moved". */
		latched = (uint8_t)ALP_CC3501E_RESET_BROWNOUT;
		break;
	case PowerWFF3_RESET_CPU:
	case PowerWFF3_RESET_RFCORE:
		/* A CPU/RF-core reset the device asked for -- what the deferred
		 * CMD_RESET path and an OTA install produce. */
		latched = (uint8_t)ALP_CC3501E_RESET_SOFT;
		break;
	case PowerWFF3_RESET_DSSM:
		/* Debug-subsystem request: a probe reset, not a fault.  No wire code
		 * for it, and calling it POWER_ON would be a lie. */
		latched = (uint8_t)ALP_CC3501E_RESET_UNKNOWN;
		break;
	case PowerWFF3_RESET_PIN_POR:
		/* Pin reset and POR are indistinguishable here -- the SDK folds them
		 * into one value.  Report POWER_ON: on this bench every such reset is
		 * a DPS-150 cold cycle. */
		latched = (uint8_t)ALP_CC3501E_RESET_POWER_ON;
		break;
	case PowerWFF3_RESET_UNKNOWN:
	default:
		latched = (uint8_t)ALP_CC3501E_RESET_UNKNOWN;
		break;
	}

	have_latched = true;
	return latched;
}

uint32_t cc3501e_hw_uptime_ms(void)
{
	/* Real uptime from the DPL clock (TI Drivers, RTOS-backed -- no radio
	 * needed).  getSystemTicks() is a 32-bit tick count; getSystemTickPeriod()
	 * is microseconds-per-tick.  Compute in 64-bit to avoid the ticks*us
	 * overflow, then return milliseconds (wraps after ~49 days, documented). */
	const uint64_t ticks     = (uint64_t)ClockP_getSystemTicks();
	const uint64_t period_us = (uint64_t)ClockP_getSystemTickPeriod();
	return (uint32_t)((ticks * period_us) / 1000u);
}

/* FreeRTOS heap-accounting API (configurability diag source).  Declared extern
 * so this TU does not pull in the kernel headers; it resolves at link time iff
 * the SysConfig FreeRTOS aggregate links a heap_N implementation (dynamic
 * allocation enabled, which this image uses for the scheduler + SwiP path). */
extern size_t xPortGetFreeHeapSize(void);

uint32_t cc3501e_hw_free_heap_bytes(void)
{
	return (uint32_t)xPortGetFreeHeapSize();
}
