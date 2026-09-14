/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * Silicon-free retry-decision logic behind cc3501e_hw_wifi_connect_sta()'s
 * bounded reason-30 retry (hal/ti/cc3501e_hw_ti_wifi.c).  Split out so the
 * two DECISIONS below -- which have no dependency on CC35xx/Wlan_* state at
 * all -- are exercisable by the host unit-test build (tests/unit/wifi_retry/),
 * not just readable in a file that only ever compiles against the vendored
 * TI SDK.
 *
 * 1. wifi_retry_delay_ms(): which delay a reason-30 retry waits before
 *    re-issuing Wlan_Connect, keyed on WHICH vendor event actually recorded
 *    the reason on the pass that just ended -- an AUTHENTICATION_REJECTED(30)
 *    deny-lists the BSSID in OUR OWN driver for ~10 s (drv_ti_internal.h:370),
 *    but an ASSOCIATION_REJECTED(30) carrying a comeback-time IE deny-lists
 *    only for the AP's own (typically ~1 s) comeback interval
 *    (drv_ti_mlme.c:1293-1327) -- the driver itself resends up to
 *    ASSOC_MAX_TRIES(3) meanwhile (drv_ti_sta_specific.c:710-738).  The event
 *    is packed into the SAME word as the reason it accompanies
 *    (wifi_last_reason_tag, hal/ti/cc3501e_hw_ti_wifi.c) and the pair is
 *    loaded ONCE at the retry site, not read back piecemeal after the retry
 *    site's own wait wakes -- a prior version of this fix picked the delay
 *    that way and it was unsound (a further vendor event landing in the gap
 *    between the two separate reads could pair a reason from one event with
 *    the event tag from another) -- see wifi_last_reason_tag's own
 *    declaration comment for the full mechanism.
 *
 * 2. wifi_retry_should_restore_first_pass(): whether a RETRY-PASS terminal
 *    reason should be replaced by the FIRST pass's real REJECTED/30 rather
 *    than published as-is -- see its own comment, including the residual
 *    this heuristic accepts.
 */

#ifndef CC3501E_BRIDGE_WIFI_RETRY_H
#define CC3501E_BRIDGE_WIFI_RETRY_H

#include <stdbool.h>
#include <stdint.h>

/* Which vendor event most recently wrote the reason half of
 * wifi_last_reason_tag for the CURRENT connect attempt.  NOT the vendor's
 * own WlanEventId_e ordinals (wlan_if.h, CC3501E_WIFI-only, not includable
 * from this silicon-free header) -- the CC3501E_WIFI call site translates
 * the three write sites it cares about (wifi_event_cb()'s DISCONNECT /
 * ASSOCIATION_REJECTED / AUTHENTICATION_REJECTED cases) into these values,
 * packed into the same word as the reason at the same instant it is
 * written. */
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
 *   - reason 3 (WLAN_REASON_DEAUTH_LEAVING).  SEVERAL vendor paths can
 *     produce this on a retry pass with no per-attempt reset in between --
 *     NOT an exhaustive list, and not always our own cleanup echoing back:
 *     (a) our own pre-retry wifi_clear_stale_assoc() -> Wlan_Disconnect() ->
 *     cmeWlanDisconnect(WLAN_REASON_DEAUTH_LEAVING) (cme_station_flow.c:521-548,
 *     cme_connection_mng.c:2721-2722), which sets pDrv->deauthReason to 3 and
 *     is echoed back on the very next DISCONNECT this cb sees; (b) hostap's
 *     SME give-up timers, sme_auth_timer/sme_assoc_timer -> sme_deauth()
 *     (sme.c:2325-2345); (c) sme_event_assoc_reject() -> sme_deauth()
 *     (sme.c:2287) when an ASSOCIATION_REJECTED arrives WITHOUT the
 *     comeback-time IE (a real AP rejection, unconditionally re-deauthed);
 *     and (d) sme_event_disassoc() (sme.c:2322) calling
 *     wpa_drv_deauthenticate(..., WLAN_REASON_DEAUTH_LEAVING) directly on a
 *     stray disassociation.  (b)-(d) all reach ti_driver_deauthenticate()'s
 *     `pDrv->deauthReason = aReasonCode` (drv_ti_sta_specific.c:400) with
 *     WLAN_REASON_DEAUTH_LEAVING.  An auth/assoc timeout that instead lands
 *     via sme_event_auth_timed_out/sme_event_assoc_timed_out
 *     (sme.c:2291-2306) or the SAE failure path (sme.c:1580-1592) does NOT
 *     call sme_deauth and writes no new deauthReason at all -- it leaves
 *     whatever value was already there STALE, which could be an earlier
 *     real code, not necessarily 3.
 *   - reason 0 (no vendor reason recorded THIS pass at all) -- the retry's
 *     own Wlan_Connect-refused (KICK) and wait-timed-out (TIMEOUT) exits
 *     already restore the first pass's outcome at their own call sites; this
 *     covers the remaining case, a genuine terminal event arriving with no
 *     reason attached.
 *
 * RESIDUAL, stated plainly: this heuristic can relabel a real retry-pass
 * failure as the first pass's stale reason.  Reason 3 is not exclusively
 * self-inflicted -- an AP's OWN deauth or disassoc frame can legitimately
 * carry ReasonCode 3 (ti_drv_rxDeauthPacket / ti_drv_rxDisassocPacket,
 * drv_ti_mlme.c:1476 / 1521, both copy the frame's own reason_code verbatim
 * into pDrv->deauthReason), and a retry pass that reaches AUTHENTICATION_
 * REJECTED(30) again -- e.g. because the retry's own passphrase is wrong,
 * which the FIRST pass's auth-level 30 never actually checked -- would also
 * be wrongly relabeled if it happened to end via a reason-3 DISCONNECT
 * rather than a fresh AUTHENTICATION_REJECTED event.  Restoring is still the
 * better default (a self-inflicted 3 is the common case this closes, per
 * cme_station_flow.c's own pre-retry cleanup above), but it is a heuristic on
 * an ambiguous signal, not a proof that the retry pass's own outcome was
 * uninformative.
 *
 * False for any other value: a real AP-issued reject code on the retry pass
 * is published as-is.  No second retry follows either way -- this function
 * only decides what gets PUBLISHED, not whether to retry again. */
bool wifi_retry_should_restore_first_pass(int16_t retry_pass_reason);

/* 3. wifi_retry_sanitize_reason(): is a reason value about to be PUBLISHED at
 * a CONNECTING-window terminal transition (issue #144, the "stale deauth
 * reason 3" bug) actually the wire's own verdict, or OUR OWN earlier cleanup
 * leaking through?
 *
 * THE MECHANISM (SDK-traced, hal/ti/cc3501e_hw_ti_wifi.c's own call sites):
 * every Wlan_Disconnect() THIS firmware issues -- wifi_clear_stale_assoc()'s
 * post-failure cleanup (#1437) and cc3501e_hw_wifi_disconnect()'s host-
 * requested teardown alike, both `Wlan_Disconnect(WLAN_ROLE_STA, NULL)` --
 * resolves (when the STA state machine is not already idle) through
 * CmeStationDisconnectKick() -> cmeWlanDisconnect(WLAN_REASON_DEAUTH_LEAVING)
 * (cc3501e-bridge-firmware vendor trace: cme_station_flow.c:525-548) into
 * ti_driver_deauthenticate()'s `pDrv->deauthReason = aReasonCode`
 * (drv_ti_sta_specific.c:400) -- unconditionally 3, with NO per-attempt
 * reset.  A SUBSEQUENT connect attempt that fails through a path which never
 * itself writes a fresh deauthReason then republishes that stale 3 as if it
 * were this attempt's own verdict.
 *
 * THE WRONG-PASSPHRASE WPA3-SAE SHAPE THIS CLOSES: a bad SAE Confirm is
 * detected PURELY LOCALLY, by hash comparison, with no over-the-air
 * deauth/disassoc frame involved at all -- hostap's sme_sae_auth()
 * (auth_transaction==2 branch, third_party/hostap/wpa_supplicant/sme.c:1457)
 * returns -1 straight from `sae_check_confirm() < 0`, and the caller,
 * sme_event_auth() (sme.c:1583-1588), takes that failure through
 * `wpas_connection_failed(); wpa_supplicant_set_state(wpa_s,
 * WPA_DISCONNECTED);` -- NEITHER call takes or forwards a reason code, and
 * NEITHER calls sme_deauth() (contrast sme_auth_timer/sme_assoc_timer/
 * sme_event_assoc_reject, sme.c:2228-2287, which DO call sme_deauth() and so
 * DO write a fresh 3 of their own -- a genuine, if generic, give-up code,
 * not a stale one).  driver_ti_wifi.c's ti_driver_state_changed(), watching
 * this exact WPA_AUTHENTICATING -> WPA_DISCONNECTED transition, republishes
 * whatever pDrv->deauthReason ALREADY held via
 * `CME_NotifyStaConnectionState(pDrv->roleId, LINK_CONNECTION_STATE_
 * DISCONNECTED, pDrv->deauthReason)` (driver_ti_wifi.c:1594-1603) --
 * verbatim, stale or not.  Bench-confirmed (run13 P4, e1m-aen-evk-01,
 * WPA3-SAE): a SECOND `wifi connect` with the same wrong passphrase fails in
 * 4.0-5.5 s (far too fast to be a wire round trip's worth of waiting) with
 * reason 3, on every boot -- consistent with THIS attempt inheriting the
 * PRIOR attempt's own #1437 cleanup value, not a fresh one.
 *
 * NO MORE INFORMATIVE SIGNAL IS AVAILABLE ON THIS PATH -- checked, not
 * assumed: (a) no real SAE/auth status reaches here -- sme.c:1583-1588 pass
 * no status of any kind, so there is nothing more specific to record
 * instead; (b) WlanEventDisconnect_t's own IsStaIsDiscnctInitiator field
 * (wlan_if.h) cannot distinguish a received frame from this stale echo
 * either -- traced to its one real-reason writer, cmeStaSendDisConnectedEventToApp()
 * (cme_station_flow.c:867-877), which hardcodes it to 0 unconditionally for
 * every non-200-reason WLAN_EVENT_DISCONNECT it ever dispatches (the ONLY
 * site where it reads 1 pairs exclusively with ReasonCode ==
 * WLAN_DISCONNECT_USER_INITIATED/200, cme.c:3291-3313/3337, which this
 * mechanism already never records -- see wifi_event_cb()'s DISCONNECT case).
 * So this is the documented, coarser fallback: treat reason 3 as
 * uninformative (publish 0 instead) once this firmware has EVER issued its
 * own Wlan_Disconnect(), rather than trying to prove which specific past
 * cleanup is the one still sitting in pDrv->deauthReason.
 *
 * RESIDUAL, stated plainly: this can ALSO blank a genuine AP-sent
 * ReasonCode 3 (a real deauth/disassoc, ti_drv_rxDeauthPacket /
 * ti_drv_rxDisassocPacket, drv_ti_mlme.c:1476/1521, copy the frame's own
 * reason_code verbatim) arriving on any attempt AFTER the first ever
 * Wlan_Disconnect() this boot -- there is no cheaper, precise way to tell
 * the two apart with what the SDK exposes on this path (see above).  Once
 * `own_disconnect_issued` goes true it is never expected to go false again
 * (mirrors "since boot" in the issue's own accepted design) -- a boot-scoped
 * over-approximation, not a per-attempt one, same tradeoff wifi_retry_
 * should_restore_first_pass() above already accepts for the SAME reason
 * value in the narrower retry-pass case.
 *
 * Applies ONLY to reason 3 -- every other value (including 0, 30, and 200,
 * which is filtered upstream in wifi_event_cb() before it ever reaches here)
 * is real, specific information this function must not touch. */
int16_t wifi_retry_sanitize_reason(int16_t reason, bool own_disconnect_issued);

#endif /* CC3501E_BRIDGE_WIFI_RETRY_H */
