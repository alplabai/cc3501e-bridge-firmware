/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * cc3501e-bridge HAL: TI backend -- GPIO proxy (v0.4), camera-enable LDOs,
 * and the bridge READY/host-IRQ line.
 *
 * Split by hardware subsystem out of cc3501e_hw_ti.c (issue #703, #461
 * Phase B).  cc3501e_hw_ti.c keeps platform lifecycle + the deferred-reboot
 * latch; see cc3501e_hw_ti_internal.h for the cross-TU seam the split uses.
 *
 * Built ONLY for CC3501E_HAL_BACKEND=ti (the bench build), against TI's
 * SimpleLink CC35xx SDK.  CI builds the stub backend instead, so this file
 * is never on the SDK-free path.
 */

#include <stdbool.h>
#include <stdint.h>

#include "ti_drivers_config.h" /* GPIO_pinUpperBound (SysConfig board file) */

#include <ti/drivers/GPIO.h>

#include "alp/protocol/cc3501e.h"

#include "../cc3501e_hw.h"
#include "transport.h" /* cc3501e_bridge_busy/ready prototypes (worker's weak hooks) */

/* --------------------------------------------------------------- */
/* GPIO proxy (v0.4) -- real CC3501E pad I/O via TI Drivers GPIO.    */
/*                                                                   */
/* The CC35xx TI Drivers GPIO layer is PIN-INDEXED: gpioPinConfigs[] */
/* (ti_drivers_config.c) is indexed by the GPIO pad number directly  */
/* (GPIO0..GPIO37, GPIO_pinUpperBound=37) and GPIO_setConfig/write/  */
/* read(index,...) take that pad number.  So the protocol's raw      */
/* cc3501e_gpio index drives the pad 1:1 -- NO logical IO11/IO13..    */
/* -> pad map is needed in firmware (the host owns the logical map,  */
/* metadata/e1m_modules/aen/from-cc3501e.tsv).  The guard below       */
/* refuses the pads the bridge itself owns (the CONFIG_SPI_0 lines +  */
/* the CONFIG_UART2_0 console glue) and the pads not bonded on        */
/* CC35X1E, so a stray host GPIO command can never tear down the      */
/* inter-chip link mid-transfer. */
static bool gpio_pad_reserved(uint8_t pad)
{
	switch (pad) {
	/* CONFIG_SPI_0 inter-chip bridge: CSN=16, SCLK=27, POCI=28, PICO=29. */
	case 16u:
	case 27u:
	case 28u:
	case 29u:
	/* GPIO17 (E1M IO16) = the bridge READY/host-IRQ line, firmware-owned (rev-1
	 * host-IRQ wire: CC35 GPIO17 -> Alif P2_6; GPIO17 is not SPI0-CS-capable,
	 * GPIO16 is the CS).  Reserve it so a host GPIO-proxy command can't clobber
	 * the flow-control line. */
	case 17u:
	/* CONFIG_UART2_0 console glue: TX=5, RX=6. */
	case 5u:
	case 6u:
	/* 7/8/9 are NOT "not bonded" -- TI documents them as SWDIO (pin 40), SWCLK
	 * (pin 39) and LOGGER (pin 38).  They are unavailable to the GPIO proxy
	 * because the debug/logging functions own them, which is a different and
	 * much better reason: driving SWDIO or SWCLK from the host would cut the
	 * debug probe off mid-session.  gpioPinConfigs marking them unavailable is
	 * the mechanism, not the justification.  Issue #20. */
	case 7u:
	case 8u:
	case 9u:
		return true;
	default:
		return false;
	}
}

static bool gpio_pad_ok(uint8_t pad)
{
	return (pad <= GPIO_pinUpperBound) && !gpio_pad_reserved(pad);
}

/* GPIO interrupt handler.  This only clears the pending edge -- the host never
 * learns the pad moved.  That is a MISSING PRODUCER, not missing hardware: #130 /
 * alp-sdk#1721 multiplexed the slave->master attention pulse onto the rev-1 READY
 * wire (CC35 GPIO17 -> Alif P2_6), so delivering EVT_GPIO_INTERRUPT is an
 * event_ring_push() call here (which pulses attention on its own).  The HW arming
 * in cc3501e_hw_gpio_set_interrupt() is already real. */
static void gpio_irq_cb(uint_least8_t index)
{
	GPIO_clearInt(index);
}

/* ---- Open-drain emulation state (#21) -------------------------------------- *
 * The CC35xx GPIOWFF3 controller has no true open-drain output
 * (GPIO_CFG_OUTPUT_OPEN_DRAIN_INTERNAL is NOT_SUPPORTED), so it is emulated.
 * This USED to be emulated as a push-pull output idling HIGH, which is not
 * open-drain at all: it actively drives the line high, so on any net with a
 * second driver pulling low the two fight and the pad sources current into it.
 * The old comment said as much -- "NOT safe on a line with another active
 * driver" -- which is an accurate description of a latent short, not a
 * mitigation.  A host that asks for OPEN_DRAIN is asking for the safe behaviour
 * precisely BECAUSE it expects other drivers on the net.
 *
 * The pad has the mechanism to do it properly: its output can be disabled
 * (GPIOnCFG.OUTDIS), which is exactly what the READY pad's reset state relies on
 * below.  So emulate the real thing: assert = drive LOW, release = Hi-Z and let
 * the net's pull-up decide.  Never drive high.
 *
 * Tracked per pad because cc3501e_hw_gpio_write() has to know whether a 1 means
 * "drive high" (push-pull) or "let go" (open-drain).  GPIO_pinUpperBound is well
 * under 32 on this part; the bit index is bounds-checked before every use. */
static uint32_t od_pads;  /* bit N set: pad N configured OPEN_DRAIN */
static uint32_t od_pulls; /* bit N set: that pad wants the internal pull-up */

static bool pad_is_open_drain(uint8_t pad)
{
	return (pad < 32u) && ((od_pads & ((uint32_t)1u << pad)) != 0u);
}

/* Release an open-drain pad: output disabled, so the net floats to whatever the
 * external pull-up (or another driver) decides. */
static int od_release(uint8_t pad)
{
	const GPIO_PinConfig pull = ((od_pulls & ((uint32_t)1u << pad)) != 0u)
	                                ? GPIO_CFG_PULL_UP_INTERNAL
	                                : GPIO_CFG_PULL_NONE_INTERNAL;
	return (GPIO_setConfig(pad, GPIO_CFG_INPUT_INTERNAL | pull | GPIO_CFG_IN_INT_NONE) == 0)
	           ? CC3501E_HW_OK
	           : CC3501E_HW_ERR_IO;
}

int cc3501e_hw_gpio_configure(uint8_t pad, uint8_t dir, uint8_t pull)
{
	if (!gpio_pad_ok(pad)) {
		return CC3501E_HW_ERR_INVAL;
	}
	GPIO_PinConfig pull_cfg = (pull == ALP_CC3501E_GPIO_PULL_UP)     ? GPIO_CFG_PULL_UP_INTERNAL
	                          : (pull == ALP_CC3501E_GPIO_PULL_DOWN) ? GPIO_CFG_PULL_DOWN_INTERNAL
	                                                                 : GPIO_CFG_PULL_NONE_INTERNAL;
	GPIO_PinConfig cfg;
	switch (dir) {
	case ALP_CC3501E_GPIO_DIR_OUTPUT:
		/* push-pull, start low; host sets the level with GPIO_WRITE. */
		cfg = GPIO_CFG_OUTPUT_INTERNAL | pull_cfg | GPIO_CFG_OUT_LOW;
		od_pads &= ~((uint32_t)1u << (pad & 31u));
		break;
	case ALP_CC3501E_GPIO_DIR_OPEN_DRAIN:
		/* Emulated -- see the od_pads note above.  Register the pad and leave
		 * it RELEASED (Hi-Z), the deasserted state of an open-drain line.
		 * cc3501e_hw_gpio_write() drives low to assert and returns to Hi-Z to
		 * release; it never drives the line high. */
		if (pad >= 32u) {
			return CC3501E_HW_ERR_INVAL; /* od_pads is a 32-bit mask */
		}
		od_pads |= ((uint32_t)1u << pad);
		if (pull == ALP_CC3501E_GPIO_PULL_UP) {
			od_pulls |= ((uint32_t)1u << pad);
		} else {
			od_pulls &= ~((uint32_t)1u << pad);
		}
		return od_release(pad);
	case ALP_CC3501E_GPIO_DIR_INPUT:
	default:
		cfg = GPIO_CFG_INPUT_INTERNAL | pull_cfg | GPIO_CFG_IN_INT_NONE;
		od_pads &= ~((uint32_t)1u << (pad & 31u));
		break;
	}
	return (GPIO_setConfig(pad, cfg) == 0) ? CC3501E_HW_OK : CC3501E_HW_ERR_IO;
}

int cc3501e_hw_gpio_write(uint8_t pad, uint8_t level)
{
	if (!gpio_pad_ok(pad)) {
		return CC3501E_HW_ERR_INVAL;
	}
	if (pad_is_open_drain(pad)) {
		/* Open-drain: a 1 RELEASES the line (Hi-Z, the net's pull-up wins); a
		 * 0 asserts by driving low.  Driving high here is precisely the
		 * contention this emulation exists to avoid. */
		if (level != 0u) {
			return od_release(pad);
		}
		if (GPIO_setConfig(pad,
		                   GPIO_CFG_OUTPUT_INTERNAL | GPIO_CFG_PULL_NONE_INTERNAL |
		                       GPIO_CFG_OUT_LOW) != 0) {
			return CC3501E_HW_ERR_IO;
		}
		GPIO_write(pad, 0u);
		return CC3501E_HW_OK;
	}
	GPIO_write(pad, (level != 0u) ? 1u : 0u);
	return CC3501E_HW_OK;
}

/* ---- Bridge READY/host-IRQ line: CC35 GPIO17 (E1M IO16) -> Alif P2_6 ------- *
 * Strong overrides of the worker's weak cc3501e_bridge_busy/ready() hooks.  The
 * line is flow-control to the host master: LOW = the bridge is BUSY (a radio op
 * is running and the SPI-slave DMA is dead, do not clock), HIGH = the slave is
 * armed and the host may clock a transaction.  Configured push-pull and driven
 * LOW (busy) from cc3501e_hw_init(), immediately after Board_init()'s GPIO_init()
 * -- NOT lazily on first use.  The pad's RESET state is output-disabled with an
 * internal pull-UP (SWRS343A Table 5-1 pin 29 "PU"; SWRU626 Table 16-72
 * GPIO17PCTL.CTL reset 1h "Pull up"; Table 16-71 GPIO17CFG.OUTDIS reset 1h), so
 * an undriven line reads HIGH at the host -- "armed" -- through the whole of
 * boot.  ready_ensure_init() below stays as an idempotent guard.  Issue #17.
 * GPIO17 = the silicon-legal sibling of the SPI0 CS pair (it is not CS-capable,
 * GPIO16 is the CS); reserved from the host GPIO proxy in gpio_pad_reserved(). */
#define CC3501E_READY_GPIO 17u

static bool ready_inited;

/* Last level this file drove.  The attention pulse below must only fire while the
 * line is HIGH (bridge idle/armed); pulsing it during a BUSY window would tell the
 * host "armed" in the middle of a radio or flash blackout, which is precisely the
 * lie that desyncs the link. */
static bool ready_level_high;

static void ready_ensure_init(void)
{
	if (!ready_inited) {
		(void)GPIO_setConfig(CC3501E_READY_GPIO,
		                     GPIO_CFG_OUTPUT_INTERNAL | GPIO_CFG_PULL_NONE_INTERNAL |
		                         GPIO_CFG_OUT_LOW);
		ready_inited = true;
	}
}

void cc3501e_bridge_busy(void)
{
	ready_ensure_init();
	GPIO_write(CC3501E_READY_GPIO, 0u); /* LOW = busy */
	ready_level_high = false;
}

void cc3501e_bridge_ready(void)
{
	ready_ensure_init();
	GPIO_write(CC3501E_READY_GPIO, 1u); /* HIGH = ready */
	ready_level_high = true;
}

#if defined(CC3501E_ATTN_PULSE) && CC3501E_ATTN_PULSE
/* Attention pulse for async events (#130).
 *
 * There is ONE slave->master wire, and it already means READY: HIGH = armed and
 * idle, LOW = busy.  An event queued while the bridge is already idle produces no
 * transition, so a host waiting on an edge would never learn about it -- which is
 * why the host has had to poll GET_PENDING_EVENTS on a timer.
 *
 * Give it an edge without inventing a second meaning: drop the line and raise it
 * again.  The host's attention ISR fires on the rising edge and schedules a drain;
 * a host that does not use the interrupt sees a momentary BUSY->READY, which is a
 * transition it already handles on every re-arm.
 *
 * ONLY while HIGH.  If the line is LOW the bridge is genuinely busy (radio op or
 * flash blackout) and the host is deliberately held off -- raising it there would
 * invite the host to clock into a dead slave.  The event simply waits; the next
 * cc3501e_bridge_ready() at the end of that op supplies the rising edge anyway.
 *
 * Spurious drains are harmless by construction: GET_PENDING_EVENTS on an empty
 * ring returns an empty list, so a host may drain on ANY rising edge -- including
 * every ordinary re-arm -- without needing to tell attention from flow control.
 * That is what makes one wire enough. */
/* A bounded LOW pulse -- wider than the host pad synchroniser, far too short to
 * look like a busy/flow-control drop.
 *
 * The width is NOT the "~2-4 us" this comment used to assert.  At the 160 MHz
 * core clock a 300-iteration volatile spin is on the order of a few microseconds
 * only if each iteration costs ~1-2 clocks, which it does not; the real figure
 * depends on the compiled loop body and has never been measured on a scope here.
 * Treat 300 as an empirically-chosen constant, not a derived timing.  If the
 * pulse width ever matters, measure it -- and note that the READY line is an
 * open connection on the current E1M-AEN rev, so it cannot be measured from the
 * host side at all.  Issue #20. */
#ifndef CC3501E_ATTN_PULSE_LOW_SPINS
#define CC3501E_ATTN_PULSE_LOW_SPINS 300u
#endif

void cc3501e_bridge_attn_pulse(void)
{
	if (!ready_inited || !ready_level_high) {
		return;
	}
	/* The LOW phase must be wide enough for the host to SEE.  Two back-to-back
	 * GPIO_write() calls give a LOW of tens of nanoseconds; the Alif pad has an
	 * input synchroniser ahead of its edge detector, so a glitch that narrow is
	 * never sampled and no rising edge is generated.
	 *
	 * A bounded spin rather than ClockP_usleep(): this runs from the ring-push
	 * path, which can be reached from an ISR, where a DPL sleep is not safe.
	 * The caller already checked the line is idle-HIGH, so a few microseconds
	 * LOW cannot be mistaken for flow control. */
	GPIO_write(CC3501E_READY_GPIO, 0u);
	for (volatile uint32_t spin = 0u; spin < CC3501E_ATTN_PULSE_LOW_SPINS; spin++) {
		__asm__ volatile("nop");
	}
	GPIO_write(CC3501E_READY_GPIO, 1u);
}
#endif /* CC3501E_ATTN_PULSE */

int cc3501e_hw_gpio_read(uint8_t pad, uint8_t *level_out)
{
	if (!gpio_pad_ok(pad)) {
		if (level_out != 0) {
			*level_out = 0u;
		}
		return CC3501E_HW_ERR_INVAL;
	}
	uint8_t lvl = (GPIO_read(pad) != 0u) ? 1u : 0u;
	if (level_out != 0) {
		*level_out = lvl;
	}
	return CC3501E_HW_OK;
}

int cc3501e_hw_gpio_set_interrupt(uint8_t pad, uint8_t edge, uint8_t enabled)
{
	if (!gpio_pad_ok(pad)) {
		return CC3501E_HW_ERR_INVAL;
	}
	if (enabled == 0u || edge == ALP_CC3501E_GPIO_EDGE_NONE) {
		/* Disable: back to a plain interrupt-free input. */
		return (GPIO_setConfig(pad, GPIO_CFG_INPUT_INTERNAL | GPIO_CFG_IN_INT_NONE) == 0)
		           ? CC3501E_HW_OK
		           : CC3501E_HW_ERR_IO;
	}
	GPIO_PinConfig edge_cfg;
	switch (edge) {
	case ALP_CC3501E_GPIO_EDGE_RISING:
		edge_cfg = GPIO_CFG_IN_INT_RISING;
		break;
	case ALP_CC3501E_GPIO_EDGE_FALLING:
		edge_cfg = GPIO_CFG_IN_INT_FALLING;
		break;
	case ALP_CC3501E_GPIO_EDGE_BOTH:
		/* GPIOWFF3 has no both-edges trigger (NOT_SUPPORTED) -- reject so
		 * the host arms a single edge instead of getting a silent no-op. */
		return CC3501E_HW_ERR_INVAL;
	default:
		return CC3501E_HW_ERR_INVAL;
	}
	/* Register the latch callback BEFORE enabling the edge (INT_ENABLE in
	 * the config self-enables the line -- no separate GPIO_enableInt). */
	GPIO_setCallback(pad, gpio_irq_cb);
	return (GPIO_setConfig(pad, GPIO_CFG_INPUT_INTERNAL | edge_cfg | GPIO_CFG_INT_ENABLE) == 0)
	           ? CC3501E_HW_OK
	           : CC3501E_HW_ERR_IO;
}

int cc3501e_hw_cam_enable(uint8_t which, uint8_t on)
{
	/* CAM_EN_LDO0 -> GPIO_1, CAM_EN_LDO1 -> GPIO_0 -- per the E1M-AEN (BDE-BW35N
	 * module U4) netlist: pin54 GPIO_1 = R_CAM_EN_LDO0, pin55 GPIO_0 = R_CAM_EN_LDO1.
	 * (Earlier code/header had this REVERSED; the SWRU626-era note was wrong.)
	 * Push-pull, default-off at boot. */
	uint8_t pad;
	switch (which) {
	case 0u:
		pad = 1u; /* CAM_EN_LDO0 -> GPIO_1 */
		break;
	case 1u:
		pad = 0u; /* CAM_EN_LDO1 -> GPIO_0 */
		break;
	default:
		return CC3501E_HW_ERR_INVAL;
	}
	GPIO_PinConfig cfg = GPIO_CFG_OUTPUT_INTERNAL | GPIO_CFG_PULL_NONE_INTERNAL |
	                     ((on != 0u) ? GPIO_CFG_OUT_HIGH : GPIO_CFG_OUT_LOW);
	if (GPIO_setConfig(pad, cfg) != 0) {
		return CC3501E_HW_ERR_IO;
	}
	GPIO_write(pad, (on != 0u) ? 1u : 0u);
	return CC3501E_HW_OK;
}
