/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * Silicon-free decision behind the CONNECT-failure-exit drain-reinit skip
 * (hal/ti/cc3501e_hw_ti_wifi.c's cc3501e_hw_wifi_connect_sta(), the
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
 * boots; skipping the reinit took that to 0/5).  A TI SDK source audit
 * (worker.c ~1192-1212, hal/ti/transport_hw_ti_spi.c ~27-100) argues
 * Wlan_Connect/Wlan_Disconnect never touch the bridge's own DMA channels
 * 12/13, so a slave the CONNECT body armed before Wlan_Connect stays armed
 * through the whole association attempt and its own failure-path
 * Wlan_Disconnect cleanup -- UNLESS the association attempt itself somehow
 * disturbed it by a mechanism the audit missed.  "The slave answered at
 * least one host frame since the body's last reinit" is offered as
 * behavioural evidence for "the slave is still armed", cheaper than a bench
 * run and self-falsifying: if the slave went dead partway through (say,
 * after Wlan_Start's HIFInit ran again on a retry pass), the count will NOT
 * have advanced and the caller must still reinit as before this fix.
 *
 * The caller is TI-only (built for CC3501E_HAL_BACKEND=ti only, against the
 * vendored SDK) and is bench-covered by the ship-bar validation in
 * bringing-up-the-cc3501e-wifi-ble-bridge (cold-cycle, `wifi connect`, then
 * `ver` still answering) -- this header covers only the pure comparison, not
 * silicon behaviour.
 */

#ifndef CC3501E_BRIDGE_WIFI_CONNECT_FAIL_SKIP_H
#define CC3501E_BRIDGE_WIFI_CONNECT_FAIL_SKIP_H

#include <stdbool.h>
#include <stdint.h>

/* May the drain skip its own reinit on a WIFI_CONNECT_STA failure exit?
 *
 * @p txn_count_at_reinit is cc3501e_hw_host_txn_count() sampled right after
 * the connect body's own last bridge_transport_spi_hw_reinit() call (the
 * role-up reinit, hal/ti/cc3501e_hw_ti_wifi.c ~1924-1926 -- the ONLY reinit
 * this function body performs before its failure exits).
 * @p txn_count_now is the same counter sampled at the failure exit, right
 * before deciding.
 *
 * True (skip) iff the two differ: the slave answered at least one host frame
 * -- proof it is still armed -- since that reinit, so paying a second one is
 * the destructive no-op #106 already measured.  False (do not skip, caller
 * must still reinit) when they are equal: no frame completed since the
 * reinit, so the slave may have gone dead partway through this attempt (the
 * built-in falsifier the header comment above describes) and the caller
 * cannot assume it is still live.
 *
 * A plain inequality, not a bounded/modular subtraction, is deliberate and
 * still correct across a hypothetical counter wrap: g_host_txn_count itself
 * SATURATES rather than wraps (cc3501e_hw_ti.c's cc3501e_hw_notify_reply_
 * sent()), so no wrap can occur in practice, but even if the counter type
 * changed to a wrapping one, "the two values differ" is exactly "the slave
 * served a frame between the two samples" regardless of which direction the
 * wrap went, with no modulus arithmetic needed. */
bool wifi_connect_fail_skip_reinit(uint32_t txn_count_at_reinit, uint32_t txn_count_now);

#endif /* CC3501E_BRIDGE_WIFI_CONNECT_FAIL_SKIP_H */
