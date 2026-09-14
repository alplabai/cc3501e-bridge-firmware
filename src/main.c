/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * cc3501e-bridge firmware -- entry point.
 *
 * The CC3501E's Cortex-M application core runs this firmware as the
 * SPI-slave (or, on SDIO-routed boards, SDIO-slave) parser that fronts
 * TI's SimpleLink Wi-Fi + BLE stacks for the Alif host.  The runtime
 * model is interrupt-driven: the active transport's slave peripheral
 * ISR hands complete request frames to protocol_dispatch() (via
 * protocol_build_reply) and stages the reply; the main loop just idles
 * in WFI between interrupts (the gd32-bridge model).
 *
 * Selectable host-control transport (CC3501E_CONTROL_TRANSPORT, default
 * spi).  SPI is the always-available baseline/fallback; SDIO is opt-in
 * for boards that dedicate the Alif's single SDIO controller to the
 * CC3501E (no SD card).  Exactly one is active.  See transport.h +
 * docs/cc3501e-bridge.md.
 *
 * Backends: CC3501E_HAL_BACKEND=stub keeps everything hardware-free for
 * host-side protocol tests (PING / GET_VERSION round-trip; HW-touching
 * ops report NOTIMPL).  CC3501E_HAL_BACKEND=ti drives real silicon
 * against TI's SimpleLink CC33xx SDK (built on the bench).
 */

#include "transport.h"
#include "worker.h"
#include "event_ring.h"
#include "../hal/cc3501e_hw.h"

#if defined(CC3501E_RTOS_FREERTOS)
/* ----------------------------------------------------------------- *
 * ti bench build (FreeRTOS).  In NORMAL mode the bridge SPI slave is  *
 * fully event-driven -- there is NO poll task: transport_spi_init()   *
 * opens the SPI peripheral in SPI_MODE_CALLBACK with DMA on host-DMA  *
 * channels 12 (RX) / 13 (TX), and the driver's transfer-complete      *
 * callback (on_transfer, hal/ti/transport_hw_ti_spi.c) advances the   *
 * request-header -> request-payload -> reply-header -> reply-payload  *
 * hardware-SS0 phases, dispatching each complete frame from callback  *
 * context.                                                            *
 * This bring-up task only DRAINS the async worker (the seconds-long   *
 * Wlan_* radio ops run here, off the SPI callback) and runs the       *
 * housekeeping tick every 10 ms.                                      *
 *                                                                     *
 * ONE image, TWO boot modes.  If the host armed OTA UPDATE MODE (wire  *
 * opcode 0x47) the previous boot, bridge_transport_spi_polled() is     *
 * true for the whole of THIS boot: the same transport_spi_init() opens *
 * the slave POLLED (SPI_MODE_BLOCKING / SPI_WAIT_FOREVER) instead, and *
 * THIS task runs the update-mode loop -- one polled frame, then the    *
 * OTA/flash pump, forever.  Still ONE task; there is NO build switch   *
 * for the bridge mode -- see the block above the update-mode loop.     *
 * (The native/stub build keeps the bare WFI loop below.)              *
 * ----------------------------------------------------------------- */
#include <FreeRTOS.h>
#include <task.h>

/* 2048 words = 8 KB.  Was 512 (2 KB), which OVERFLOWED on the Wi-Fi build: this
 * task runs cc3501e_hw_wifi_boot_start -> Wlan_Start -> InitHostDriver ->
 * init_device -> FW download, a deep CC35xx host-driver chain.  TI's
 * network_terminal runs Wlan_Start on a 6048-byte thread (main_freertos.c
 * THREADSTACKSIZE); 8 KB gives margin.  A 2 KB overflow faulted the task before
 * Wlan_Start returned, leaving the bridge SPI dead from boot (ping_ok=0,
 * reqhdr_rx=0x00000000) -- bench-proven.  (The transport + FW-event threads
 * Wlan_Start spawns get their own 4096+1200-byte stacks from the FreeRTOS heap.) */
/* 32 KB, not 8 KB.  This task runs the psa_fwu flash path, and psa_fwu_start
 * VERIFIES THE MANIFEST -- SHA + ECDSA P-256 -- on this stack; PSA crypto call
 * chains carry multi-KB frames and large locals.  configCHECK_FOR_STACK_OVERFLOW
 * is 2 with no vApplicationStackOverflowHook defined here, so an overflow lands
 * in the kernel default and the chip stops serving ANYTHING until reset -- which
 * is exactly the observed OTA signature: BEGIN never returns, the device is
 * unreachable for the rest of the session (81 s / 181 s / 361 s all identical --
 * a fault does not care how long you wait), and a cold power cycle always
 * recovers it, while radio/BLE/PING (no crypto, no flash lock) stay perfect. */
#define CC3501E_BRINGUP_STACK_WORDS 8192u
static StackType_t bringup_stack[CC3501E_BRINGUP_STACK_WORDS];
/* Below the TI driver/NWP tasks so they can release flash locks -- see the
 * priority-inversion note at xTaskCreateStatic below. */
#define CC3501E_BRINGUP_TASK_PRIO (configMAX_PRIORITIES - 4)

static StaticTask_t bringup_tcb;

/* #142: dedicated link-healer task.  cc3501e_hw_tick() (drained on
 * bringup_task above) can be frozen for up to ~70 s inside a
 * WIFI_CONNECT/Wlan_Start body -- Wlan_Start/Wlan_Connect/Wlan_RoleUp all
 * block the calling task -- which froze the four SPI self-heals that used
 * to run from it too (hal/ti/cc3501e_hw_ti.c's own comment at
 * cc3501e_hw_tick()'s old call site has the root-cause writeup).  Moving
 * JUST those checks to their own task, independent of whatever the bring-up
 * task is blocked inside, is the fix; see hal/ti/cc3501e_hw_ti.c's
 * cc3501e_hw_link_tick() for what actually runs.
 *
 * STACK SIZING (NOT bench-verified -- this task has never run on silicon as
 * of this change): the call graph is shallow next to bringup_task's own --
 * on an idle tick, cc3501e_hw_link_tick() only calls bridge_transport_spi_
 * is_dead()/_phase_stalled()/_reply_armed()/_quiet_ms()/_phase() (a handful
 * of instructions each, no locals of note); on a self-heal firing it
 * additionally calls through bridge_transport_spi_hw_reinit() ->
 * spi_open_and_arm() (own frame 0x24/36 B by direct disassembly --
 * tiarmobjdump on the built .o shows `sub sp, #0x24` at that function's
 * entry) -> SPI_open()/SPI_close() (TI driver internals, UNMEASURED -- not
 * rebuilt with -fstack-usage here) plus ClockP_usleep().  No Wlan_* NWP
 * host-driver chain, no PSA crypto (psa_fwu_start's SHA/ECDSA verification
 * is what actually drove bringup_task's own 8 KB/32 KB sizing, see the
 * comments above) -- 1024 words is generous headroom over what this graph
 * needs, not a measured minimum, same shape as bringup_task's own sizing.
 *
 * PLACEMENT: TCM (.bss.link_stack, ti/build_ti.sh's linker.cmd patch #5),
 * NOT the default DRAM_NON_SECURE every other static in this file lands in.
 * ti/build_ti.sh --wifi --ble (the ship config this whole fix targets) was
 * ALREADY within ~431 B of exhausting DRAM_NON_SECURE before this change --
 * that number is not a guess, it is build_ti.sh's own pre-existing comment
 * on the .bss.sock_ring TCM move ("DRAM is full ... 431 bytes spare") --
 * and this task's stack alone needs several times that.  Confirmed on the
 * real linker: with this array left in its default DRAM section, EVERY
 * stack size from 512 words down to 80 words either failed placement (512
 * words: short by 1695 B; 128 words: short by 159 B; 96 words: short by
 * 31 B; 88 words: short by 8 B, in the smaller GROUP_5 rather than GROUP_4
 * once GROUP_4 itself finally had room) or left CC3501E_WEDGE_PROBE (which
 * adds its own DRAM-resident diagnostic state, hal/ti/transport_hw_ti_spi.c)
 * with no room at all.  TCM_DRAM_NON_SECURE is a SEPARATE ~128 KB region
 * (origin 0x20000000, the vendor board's own linker.cmd) that
 * .bss.sock_ring already proves has room for a 16 KB buffer -- moving 4 KB
 * of task stack there costs this design nothing (the link task never DMAs
 * out of its own stack) and gives DRAM_NON_SECURE its ~431 B back, which is
 * what actually lets 1024 words -- and CC3501E_WEDGE_PROBE on top of it --
 * link clean.  Verify in cc3501e-bridge.map that link_stack resolves at
 * 0x200xxxxx, NOT 0x28xxxxxx, same check build_ti.sh's own comment already
 * prescribes for rx_ring.
 *
 * configCHECK_FOR_STACK_OVERFLOW == 2 (see bringup_task's own PSA-crypto
 * comment) still catches an overrun at runtime with no application hook
 * defined -- an overflow here halts the bridge rather than silently
 * corrupting an adjacent task's memory, the same fail-safe bringup_task
 * relies on.  FIRST BENCH ACTION for this task should still be
 * uxTaskGetStackHighWaterMark() on link_tcb after a soak (idle and through
 * at least one quiet-rearm heal and one desync/arm-fail/stall recovery) to
 * confirm the generous-headroom assumption above, now that DRAM is no
 * longer the constraint forcing a smaller number. */
#define CC3501E_LINK_TASK_STACK_WORDS 1024u
/* See the placement paragraph above -- ti/build_ti.sh patches
 * cc3501e_vendor.cmd to route .bss.link_stack to TCM_DRAM_NON_SECURE; a
 * build that skips that patch (or a linker.cmd that no longer has the
 * anchor the patch keys off) fails loudly at link time instead of silently
 * landing this back in the already-exhausted DRAM bank. */
static StackType_t link_stack[CC3501E_LINK_TASK_STACK_WORDS]
    __attribute__((section(".bss.link_stack")));
static StaticTask_t link_tcb;

/* 10 ms period -- matches the bring-up task's own housekeeping cadence
 * (cc3501e_hw_tick()'s vTaskDelay below), which is also the period the
 * pre-#142 self-heals ran at before they were ever moved off it. */
#define CC3501E_LINK_TASK_PERIOD_MS 10u

static void link_task(void *arg)
{
	(void)arg;
	for (;;) {
		cc3501e_hw_link_tick();
		vTaskDelay(pdMS_TO_TICKS(CC3501E_LINK_TASK_PERIOD_MS));
	}
}

static void bringup_task(void *arg)
{
	(void)arg;

	/* Read-and-CLEAR the persisted OTA-update-mode flag before ANYTHING blocks.
	 * The FIRST call to bridge_transport_spi_polled() consumes the flag; every
	 * later call on this boot returns the same latched answer, so the whole boot
	 * agrees on one mode (transport open mode, the no-op teardown guards, and the
	 * OTA pump all key off this one predicate).
	 *
	 * Clearing on READ is what stops a wedged update-mode boot from becoming a
	 * PERMANENT update-mode boot loop: update mode has no self-heal, so the host's
	 * only escape is a WIFI_EN/nRESET cold cycle -- and that must always land in
	 * NORMAL mode. */
#if defined(CC3501E_CONTROL_TRANSPORT_SDIO)
	/* SDIO builds have NO polled path: the flag is a property of the SPI bridge
	 * (it exists only to keep SPI_open out of SPI_MODE_CALLBACK), and the
	 * update-mode loop below lives in the SPI half of this #if.  Reading the flag
	 * here would let a stray 0x47 -- protocol_ota.c and hal/ti/transport_hw_ti_spi.c
	 * are BOTH compiled in an SDIO build, so it can be armed -- suppress
	 * cc3501e_hw_net_init() on a boot that then falls straight through to the
	 * NORMAL loop, silently costing that boot its whole lwIP stack.  Constant-fold
	 * it to false instead: this build is always normal mode. */
	const bool update_mode = false;
#else
	const bool update_mode = bridge_transport_spi_polled();
#endif

	if (!update_mode) {
		/* Bring up the lwIP TCP/IP core FIRST -- before transport_spi_init() arms the
		 * callback-driven bridge slave and before the radio is lazy-started.  tcpip_init
		 * waits for the lwIP thread to start; doing it later (from the worker drain, after
		 * the transport + Wlan_Start) HUNG the worker on the bench (the lwIP thread never
		 * came up + the radio ate the heap its 16 KB stack needs).  No-op on a build without
		 * Wi-Fi/lwIP.  The Wi-Fi connect path's STA netif (network_stack_add_if_sta) needs
		 * this core to exist.
		 *
		 * DELIBERATELY SKIPPED in update mode: nothing there touches the network
		 * stack (only PING / OTA_* / GET_DIAG_INFO / RESET are serviceable), and
		 * lwIP's 16 KB thread is heap this boot has no reason to spend. */
		cc3501e_hw_net_init();
	}

#if defined(CC3501E_CONTROL_TRANSPORT_SDIO)
	transport_sdio_init(); /* SDIO builds have no polled path -- normal loop only */
#else
	transport_spi_init(); /* opens POLLED or CALLBACK per the same latched flag */

	if (update_mode) {
		/* ===== OTA UPDATE MODE =====
		 * A SPI_MODE_CALLBACK (DMA) SPI_open PERMANENTLY prevents psa_fwu_start()
		 * and psa_fwu_write() from returning, and SPI_close() does not undo the
		 * claim (silicon 2026-08-21: start returns when called before the transport
		 * or after a POLLED open, and never returns after a callback open).  So this
		 * boot never opens the slave in callback mode at all.
		 *
		 * ONE TASK, and it must stay one.  A polled peripheral only listens while it
		 * is inside SPI_transfer, and the flash pump (psa_fwu_write, hundreds of ms;
		 * a slot erase 22-41 s) does not fit inside one.  Nothing on the psa_fwu path
		 * yields -- FlashWFF3's uDMA waits and XMEMWFF3's deadline polls are bare
		 * spins -- so a SECOND, lower-priority task parked in SPI_transfer would be
		 * off-CPU for the whole op just the same, and TI caps what an armed-but-
		 * unserviced slave can absorb at 32 BYTES of RX FIFO before it silently
		 * overruns (SWRU626 ch.18 gives 32 LOCATIONS and this transport configures
		 * an 8-bit data size; SPIWFF3DMA.h:84-86, :97).  #41 corrected this figure
		 * from "8 frames" in transport_hw_ti_spi.c and left the identical claim
		 * here, where it is load-bearing for the single-task OTA loop below (#65).  A HIGHER-priority poll task cannot
		 * exist either: spiPollingTransfer is a busy-spin with SPI_WAIT_FOREVER (a
		 * peripheral only gets the polling branch when transferTimeout IS
		 * SPI_WAIT_FOREVER, SPIWFF3DMA.c:774-777) and configUSE_TIME_SLICING is 0, so
		 * it would starve the pump and deadlock OTA outright.  Splitting also turns a
		 * BETWEEN-frames gap -- the one place the FIFO can be safely reset -- into a
		 * preemption at an arbitrary mid-frame phase.  So: keep the pump between
		 * frames, and make the gap harmless instead of trying to abolish it.  That is
		 * spi_fifo_reset() in hal/ti/transport_hw_ti_spi.c, which starts every frame
		 * from empty RX+TX FIFOs; read its comment before touching this loop.
		 *
		 * Run the accept one-shot BEFORE the loop: poll_service blocks FOREVER waiting
		 * for the host, and an unconfirmed TRIAL image that never gets accepted is
		 * REVERTED by the cold BL2/MCUboot on the next boot (the same SHIP-CRITICAL
		 * ordering as the normal path below). */
		cc3501e_hw_tick();

		for (;;) {
			/* ONE whole frame (all four SS0 phases), blocking, then the pump.  The
			 * FULL tick is safe here only because _hw_reinit / _hw_release /
			 * _hw_suspend / _hw_quiesce are no-ops while polled (see
			 * hal/ti/transport_hw_ti_spi.c).  It is also REQUIRED: cc3501e_hw_ota_pump()
			 * (the psa_fwu flash) and BOTH deferred reboots -- NVIC_SystemReset
			 * (CMD_RESET, the abort escape) and psa_fwu_request_reboot (the FINISH
			 * swap) -- fire only from inside cc3501e_hw_tick().  Without it FINISH
			 * acks, stages, and then hangs at STAGED forever.
			 *
			 * poll_service returns false only when nothing is armed at all (dead
			 * handle) -- do not hot-spin on that; the tick owns the dead-slave
			 * re-open, so give it a tick to run. */
			const bool serviced = bridge_transport_spi_poll_service();
			cc3501e_hw_tick();
			if (!serviced) {
				vTaskDelay(pdMS_TO_TICKS(1));
			}
		}
	}
#endif

	/* ===== NORMAL (DMA/callback) MODE from here down ===== */

	/* Bridge armed + idle -> drive the READY/host-IRQ line HIGH so the host may
	 * clock.  The radio is brought up lazily (boot_start is NOT called), so without
	 * this the line would idle LOW (busy) forever and gate every command.  The
	 * worker then drops it LOW around each radio op (cc3501e_bridge_busy/ready).
	 *
	 * NORMAL mode ONLY -- in update mode READY has exactly one owner,
	 * bridge_transport_spi_poll_service(), which raises it around the transfer it
	 * has actually armed.  Raising it here as well would advertise "clock me" with
	 * no polled transfer in flight, which is precisely the lie that produced
	 * ~26 B/s with every WRITE returning -5. */
	cc3501e_bridge_ready();

	/* #142: create the link-healer task HERE, not in main() before the
	 * scheduler starts.  bridge_transport_spi_polled()'s read-and-clear
	 * (this function's very first statement, above the update_mode branch)
	 * and transport_spi_init()'s spi_open_and_arm() (which creates the
	 * reinit/release/suspend mutex -- see hal/ti/transport_hw_ti_spi.c's
	 * bridge_transport_spi_hw_init()) must both complete before a SECOND
	 * task can safely call into any of that state.  Creating link_task from
	 * main(), before vTaskStartScheduler(), cannot guarantee that ordering:
	 * once the scheduler starts, task start order is not guaranteed, so
	 * link_task could call cc3501e_hw_link_tick() -> bridge_transport_spi_
	 * polled() before bringup_task ever reaches its own first call -- a
	 * non-volatile, non-atomic check-then-set race on that function's OWN
	 * g_boot_read/g_polled latch (transport_hw_ti_spi.c), which was never a
	 * hazard before this change because exactly one task ever called it.
	 * Reaching this line proves both preconditions already happened. */
	(void)xTaskCreateStatic(
	    link_task,
	    "cc3501e_link",
	    CC3501E_LINK_TASK_STACK_WORDS,
	    NULL,
	    CC3501E_BRINGUP_TASK_PRIO, /* same priority as bring-up -- see link_task's own comment */
	    link_stack,
	    &link_tcb);

	/* Confirm the MCUboot/PSA-FWU image FIRST (psa_fwu_accept, run by the first
	 * cc3501e_hw_tick) -- BEFORE anything that might block.  SHIP-CRITICAL
	 * ordering: a freshly-programmed vendor image boots as an unconfirmed TRIAL
	 * and is REVERTED by the cold BL2/MCUboot on the next boot unless the running
	 * app accepts it.  If a later step hangs (e.g. the radio bring-up) before the
	 * accept runs, the trial is never confirmed -> every subsequent cold boot
	 * reverts it -> the vendor image stops launching (host reads
	 * reqhdr_rx=0xFFFFFFFF) -- bench-proven 2026-06-18.  Accepting here, right
	 * after the bridge is armed, makes the image permanent regardless of what the
	 * radio does. */
	cc3501e_hw_tick();

	/* Radio is brought up LAZILY on the first Wi-Fi op (from the worker drain),
	 * NOT at boot.  Wlan_Start can be slow/blocking (host-driver + NWP FW
	 * download); running it here would stall the bridge before any PING and, if it
	 * ever hangs, take the whole link down.  Deferring keeps the PING / IO /
	 * config link rock-solid and isolates the radio to the GET_MAC / Wi-Fi path
	 * (which re-opens the bridge SPI after the op -- see cc3501e_hw_ti.c).
	 * cc3501e_hw_wifi_boot_start() is left available but intentionally not called. */

	for (;;) {
		/* DRAIN the async worker OUTSIDE the SPI ISR: a QUEUED job (e.g.
		 * GET_MAC submitted from the ISR's poll) runs its blocking HAL body
		 * HERE, on the task, for as long as it needs (Wi-Fi init takes
		 * seconds).  Meanwhile the SPI ISR keeps answering the host's poll
		 * re-issues from the worker's shared state -- the whole point of the
		 * submit/poll seam (P0-4). */
		worker_run_pending();
		cc3501e_hw_tick();
		vTaskDelay(pdMS_TO_TICKS(10));
	}
}

int main(void)
{
	cc3501e_hw_init();
	worker_init();
	event_ring_init();
	(void)xTaskCreateStatic(bringup_task,
	                        "cc3501e_bringup",
	                        CC3501E_BRINGUP_STACK_WORDS,
	                        NULL,
	                        /* NOT the top priority.  The psa_fwu flash path locks
	                         * XMEMWFF3's writeMutex (MutexP), and the Wi-Fi/NWP
	                         * stack and NVOCMP take flash locks from their own,
	                         * LOWER-priority tasks.  At configMAX_PRIORITIES - 1
	                         * this task could never be preempted, so if a lower
	                         * task held that mutex it could never run to release
	                         * it -- a priority inversion that HANGS the OTA
	                         * forever while leaving radio ops (which do not touch
	                         * the flash mutex) working perfectly.  Bench
	                         * 2026-08-21: BEGIN never returned at 181 s and 361 s
	                         * with the device unreachable throughout, and a cold
	                         * cycle always recovered it. */
	                        CC3501E_BRINGUP_TASK_PRIO,
	                        bringup_stack,
	                        &bringup_tcb);
	vTaskStartScheduler();
	for (;;) {
	}
	/* unreachable */
}

#else /* bare-loop: native/stub build (gd32-bridge model) */

/* The Cortex-M intrinsic; weakly defined here so the scaffold compiles
 * under hosted toolchains where __WFI() is missing. */
#ifndef __WFI
__attribute__((weak)) void __WFI(void)
{
}
#endif

int main(void)
{
	cc3501e_hw_init();
	worker_init();
	event_ring_init();

#if defined(CC3501E_CONTROL_TRANSPORT_SDIO)
	/* Opt-in: the board routes the Alif SDIO controller to the CC3501E
     * (no SD card present).  Falls back to SPI by simply not defining
     * this at build time. */
	transport_sdio_init();
#else
	transport_spi_init(); /* default + always-available baseline */
#endif

	for (;;) {
		__WFI();
		/* Drain any QUEUED async job (the submit/poll seam) on this idle
		 * wakeup, then run housekeeping.  On the silicon-free build the
		 * drain is a no-op (the stub runs jobs synchronously at submit). */
		worker_run_pending();
		cc3501e_hw_tick();
	}
	/* unreachable */
	return 0;
}

#endif /* CC3501E_RTOS_FREERTOS */
