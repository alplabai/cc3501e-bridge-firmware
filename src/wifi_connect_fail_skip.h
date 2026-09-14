/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * Silicon-free wait/decision behind the CONNECT-failure-exit drain-reinit
 * skip (hal/ti/cc3501e_hw_ti_wifi.c's cc3501e_hw_wifi_connect_sta(), every
 * ALP_CC3501E_WIFI_CONN_FAILED exit).  Split out so the one DECISION below
 * -- which has no dependency on CC35xx/Wlan_* state at all -- is exercisable
 * by the host unit-test build, not just readable in a file that only ever
 * compiles against the vendored TI SDK.  Mirrors src/wifi_retry.c's split.
 *
 * BACKGROUND (advisor analysis, moderate confidence -- NOT bench-proven):
 * a WIFI_CONNECT that fails association currently pays src/worker.c's drain
 * reinit (SPI_close/SPI_open) unconditionally on its failure exit, the same
 * destructive-no-op-on-a-live-slave hazard #106 already measured for RSSI
 * and WIFI_DISCONNECT (run7: a 1 ms host poll gap wedged 7/7 associated
 * boots; skipping the reinit took that to 0/5).
 *
 * WHAT THE TRACE ACTUALLY COVERS, and what it does NOT:
 *   - Wlan_Connect() is SYNCHRONOUS but only queues a message: wlan_if.c
 *     ~1272-1296 (Wlan_Connect) -> wlan_connect_internal() -> CME_WlanConnect()
 *     (cme.c ~1517-1624), which builds a cmeMsg_t and calls pushMsg2Queue()
 *     (cme.c ~1624) and returns.  The actual 802.11 association work --
 *     scan, auth, assoc, the whole handshake -- runs LATER, asynchronously,
 *     on the CME task that drains that queue.
 *   - HIFInit() (the call that actually re-points the shared host-DMA + NWP
 *     IRQ and disrupts the bridge slave) is reachable from exactly one path
 *     in this SDK: Wlan_Start() (wlan_if.c:879) -> InitHostDriver()
 *     (init_host.c:106) -> bus_sendInitCommand() (init_host.c ~196) ->
 *     HIFInit() (bus_hif.c:96).  Wlan_Start()'s only caller in this firmware
 *     is cc3501e_hw_wifi_lazy_start() (hal/ti/cc3501e_hw_ti_wifi.c ~776),
 *     which runs ONCE per boot (wifi_started guards it) -- never again on a
 *     connect retry pass.
 *   - DMAWFF3_initHw() (DMAWFF3.c:109), the SDK's one GLOBAL DMA hardware
 *     reset, is called from exactly one site, DMAWFF3_init() (DMAWFF3.c:82),
 *     itself called from exactly one site, SPIWFF3DMA.c:1647 (initDMA(),
 *     the BRIDGE's own SPI driver init) -- never from any Wi-Fi source path.
 *   - What is NOT traced: the asynchronous CME association work itself --
 *     the actual scan/auth/assoc HIF traffic that runs on channel 11
 *     (HOSTDMA_DRIVER_CH_HIF) after CME_WlanConnect()'s message is drained.
 *     Nothing above shows that traffic never disturbs the bridge's own
 *     channels 12/13, only that the TWO functions known to touch the shared
 *     DMA/IRQ hardware directly (HIFInit, DMAWFF3_initHw) are unreachable
 *     from the connect path.  The retry pass's own wifi_clear_stale_assoc()
 *     -> Wlan_Disconnect() -> CME_WlanDisconnect() is traced the same way as
 *     the unconditional WIFI_DISCONNECT skip (src/worker.c's wifi_disconnect
 *     group, sourced against worker.c ~1192-1212) -- a synchronous message-
 *     queue post, no DMA/SPI/interrupt-mask touched on that call either.
 *
 * BECAUSE the asynchronous association work is untraced, "the slave answered
 * a host frame since the body's role-up reinit" is NOT good evidence by
 * itself -- that reinit ran BEFORE Wlan_Connect, before the retry pass,
 * before the whole association wait, and before all of that untraced async
 * work.  An earlier version of this fix sampled its baseline there and
 * fired the skip on frames served by ANY host poll in that whole window,
 * including ones that happened before a LATER Wlan_Connect or the
 * asynchronous association work killed the slave -- the skip fired anyway,
 * and the drain's reinit (the only thing that could have recovered it)
 * never ran.  The host polls WIFI_STATUS every 50 ms, so that early
 * baseline had almost always already advanced by the time any failure exit
 * ran, regardless of whether the slave was still alive at the moment of
 * failure (host review of 580f748).
 *
 * THE FIX: sample a FRESH baseline at the failure exit itself -- after
 * Wlan_Connect, the retry pass, the wait, and the trailing
 * wifi_clear_stale_assoc(), i.e. after everything untraced has already had
 * its chance to disturb the slave -- and POLL forward from there for up to
 * ~3 host WIFI_STATUS poll gaps (150 ms) for a frame to land.  A slave still
 * being serviced answers within that window; a dead one does not, and the
 * caller falls through to the unconditional reinit exactly as before this
 * whole fix -- the built-in falsifier.  See wifi_wait_host_frame() below.
 *
 * The caller is TI-only (built for CC3501E_HAL_BACKEND=ti only, against the
 * vendored SDK) and is bench-covered by the ship-bar validation in
 * bringing-up-the-cc3501e-wifi-ble-bridge (cold-cycle, `wifi connect`, then
 * `ver` still answering) -- this header covers only the pure wait, not
 * silicon behaviour.
 */

#ifndef CC3501E_BRIDGE_WIFI_CONNECT_FAIL_SKIP_H
#define CC3501E_BRIDGE_WIFI_CONNECT_FAIL_SKIP_H

#include <stdbool.h>
#include <stdint.h>

/* Poll @p count_fn for up to @p window_ms (in @p step_ms increments, via
 * @p sleep_ms_fn) for it to report a value different from the sample taken
 * at ENTRY to this call -- i.e. the baseline is THIS call's own first read,
 * never one supplied by the caller.  That is deliberate, not an oversight:
 * see this header's own BACKGROUND section for why an externally-supplied,
 * earlier baseline is exactly the bug this function exists to not repeat.
 *
 * Returns true the moment the count differs from that baseline (a host
 * frame completed -- the slave is demonstrably still armed; stop polling
 * immediately, do not wait out the rest of the window).  Returns false if
 * @p window_ms elapses with no change (no evidence the slave is alive; the
 * caller must reinit).
 *
 * @p count_fn and @p sleep_ms_fn are injected so this is testable without
 * the vendored TI SDK or a real clock -- the real caller (hal/ti/
 * cc3501e_hw_ti_wifi.c) passes cc3501e_hw_host_txn_count() and a thin
 * ClockP_usleep() wrapper; tests/unit/wifi_connect_fail_skip/ passes fakes.
 * @p window_ms and @p step_ms are parameters (not baked in) for the same
 * testability reason -- the real call site's own constants
 * (CC3501E_WIFI_CONNECT_FAIL_SKIP_WINDOW_MS / _STEP_MS) live next to that
 * call site, not here. */
bool wifi_wait_host_frame(uint32_t (*count_fn)(void),
                          void (*sleep_ms_fn)(uint32_t),
                          uint32_t window_ms,
                          uint32_t step_ms);

#endif /* CC3501E_BRIDGE_WIFI_CONNECT_FAIL_SKIP_H */
