/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * Silicon-free retry-decision logic behind cc3501e_hw_wifi_connect_sta()'s
 * bounded reason-30 retry (hal/ti/cc3501e_hw_ti_wifi.c).  Split out, like
 * src/sock_recv_commit.{c,h} split the SOCK_RECV lazy-commit arithmetic out
 * of a TI-SDK-dependent HAL body, so the two DECISIONS below -- which have no
 * dependency on CC35xx/Wlan_* state at all -- are exercisable by the host
 * unit-test build (tests/unit/wifi_retry/), not just readable in a file that
 * only ever compiles against the vendored TI SDK.
 *
 * 1. wifi_retry_delay_ms(): which delay a reason-30 retry waits before
 *    re-issuing Wlan_Connect, keyed on WHICH vendor event actually recorded
 *    the reason on the pass that just ended -- an AUTHENTICATION_REJECTED(30)
 *    deny-lists the BSSID in OUR OWN driver for ~10 s (drv_ti_internal.h:370),
 *    but an ASSOCIATION_REJECTED(30) carrying a comeback-time IE deny-lists
 *    only for the AP's own (typically ~1 s) comeback interval
 *    (drv_ti_mlme.c:1293-1327) -- the driver itself resends up to
 *    ASSOC_MAX_TRIES(3) meanwhile (drv_ti_sta_specific.c:710-738).  The event
 *    must be snapshotted at the SAME instant (the SAME guard) as the
 *    wifi_last_reason write it accompanies, not read back after the retry
 *    site's own wait wakes -- a prior version of this fix picked the delay
 *    that way and it was unsound (a further vendor event landing in the gap
 *    between "wifi_last_reason recorded" and "wait wakes" could select the
 *    wrong one) -- see the caller for where that snapshot happens.
 *
 * 2. wifi_retry_should_restore_first_pass(): whether a RETRY-PASS terminal
 *    reason must be replaced by the FIRST pass's real REJECTED/30 rather than
 *    published as-is -- see its own comment.
 */

#ifndef CC3501E_BRIDGE_WIFI_RETRY_H
#define CC3501E_BRIDGE_WIFI_RETRY_H

#include <stdbool.h>
#include <stdint.h>

/* Which vendor event most recently wrote wifi_last_reason for the CURRENT
 * connect attempt.  NOT the vendor's own WlanEventId_e ordinals (wlan_if.h,
 * CC3501E_WIFI-only, not includable from this silicon-free header) -- the
 * CC3501E_WIFI call site translates the three write sites it cares about
 * (wifi_event_cb()'s DISCONNECT / ASSOCIATION_REJECTED / AUTHENTICATION_
 * REJECTED cases) into these values under the exact same guard as each
 * wifi_last_reason write. */
typedef enum {
	WIFI_RETRY_EVENT_NONE = 0, /* nothing recorded yet for this attempt */
	WIFI_RETRY_EVENT_ASSOCIATION_REJECTED,
	WIFI_RETRY_EVENT_AUTHENTICATION_REJECTED,
	WIFI_RETRY_EVENT_DISCONNECT,
} wifi_retry_event_t;

/* Delay (ms) a reason-30 retry must wait before re-issuing Wlan_Connect.
 *
 * comeback_delay_ms applies ONLY to WIFI_RETRY_EVENT_ASSOCIATION_REJECTED --
 * the one shape whose local deny-list entry (if the comeback IE even added
 * one) expires on the AP's own short comeback interval.
 *
 * denylist_delay_ms is the conservative default for every OTHER value,
 * including WIFI_RETRY_EVENT_AUTHENTICATION_REJECTED (whose deny-list entry
 * genuinely needs the full ~10 s), WIFI_RETRY_EVENT_DISCONNECT (a DISCONNECT's
 * ReasonCode landing on 30 by coincidence is an un-excluded shape this must
 * not under-wait), and WIFI_RETRY_EVENT_NONE (should not occur at the one call
 * site that reads this -- retry eligibility already required a recorded
 * reason of exactly 30 -- but the safe default costs nothing if it ever does). */
uint32_t wifi_retry_delay_ms(wifi_retry_event_t event,
                             uint32_t           comeback_delay_ms,
                             uint32_t           denylist_delay_ms);

/* Change 2 (the reason-3 aliasing fix): does a RETRY-PASS terminal reason need
 * to be replaced with the FIRST pass's real REJECTED/30 rather than published
 * as-is?
 *
 * True for:
 *   - reason 3 (WLAN_REASON_DEAUTH_LEAVING).  TWO vendor writers can produce
 *     this on a retry pass with no per-attempt reset in between: our own
 *     pre-retry wifi_clear_stale_assoc() -> Wlan_Disconnect() ->
 *     cmeWlanDisconnect(WLAN_REASON_DEAUTH_LEAVING) (cme_station_flow.c:521-548,
 *     cme_connection_mng.c:2721-2722), which sets pDrv->deauthReason to 3 and
 *     is echoed back on the very next DISCONNECT this cb sees; and hostap's
 *     own SME give-up timer, sme_deauth() (sme.c:2228-2245), which writes a
 *     FRESH generic 3 when an auth/assoc timeout or a failed SAE exchange
 *     ends the retry pass WITHOUT writing any new deauthReason of its own
 *     (sme_event_auth_timed_out/sme_event_assoc_timed_out, sme.c:2291-2306;
 *     the SAE failure path, sme.c:1580-1592).  Either way, 3 on a retry pass
 *     carries strictly less information than the first pass's real AP-issued
 *     30, and DISCONNECT's ReasonCode source, pDrv->deauthReason
 *     (driver_ti_wifi.c:1592-1603, off CME_NotifyStaConnectionState, itself
 *     called from cme.c:4159-4164's WPA_SUPP_DISCONNECTED handler), has no
 *     per-attempt reset that would otherwise clear a stale 3 between passes.
 *   - reason 0 (no vendor reason recorded THIS pass at all) -- the retry's
 *     own Wlan_Connect-refused (KICK) and wait-timed-out (TIMEOUT) exits
 *     already restore the first pass's outcome at their own call sites; this
 *     covers the remaining case, a genuine terminal event arriving with no
 *     reason attached.
 *
 * False for any other value: a real AP-issued reject code on the retry pass
 * is strictly the retry's OWN outcome and must be published as-is.  No second
 * retry follows either way -- this function only decides what gets
 * PUBLISHED, not whether to retry again. */
bool wifi_retry_should_restore_first_pass(int16_t retry_pass_reason);

#endif /* CC3501E_BRIDGE_WIFI_RETRY_H */
