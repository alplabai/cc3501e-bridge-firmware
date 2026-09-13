/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * cc3501e-bridge HAL: TI backend -- Wi-Fi (v0.2, real CC35xx SimpleLink
 * host integration) + the factory-MAC read + the boot-time radio/lwIP
 * bring-up (radio<->SPI coexistence fix; see cc3501e_hw.h).
 *
 * Split by hardware subsystem out of cc3501e_hw_ti.c (issue #703, #461
 * Phase B).  cc3501e_hw_wifi_lazy_start() below is shared with
 * cc3501e_hw_ti_ble.c (BLE shares the HIF, so BLE_ENABLE lazy-starts Wi-Fi
 * first) -- see cc3501e_hw_ti_internal.h for that cross-TU seam.
 * cc3501e_hw_ti.c keeps platform lifecycle + the deferred-reboot latch.
 *
 * Built ONLY for CC3501E_HAL_BACKEND=ti (the bench build), against TI's
 * SimpleLink CC35xx SDK.  CI builds the stub backend instead, so this file
 * is never on the SDK-free path.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h> /* memcpy (scan-result cache / AP SSID staging) */

/* Async-event ring (src/event_ring.h, on the firmware CMake include path): the
 * Wi-Fi connect/disconnect path pushes EVT_WIFI_* here for the host to drain via
 * CMD_GET_PENDING_EVENTS.  Silicon-free; always linked. */
#include "event_ring.h"

#ifdef CC3501E_WIFI
/* CC35xx Wi-Fi host API (SDK 10.10 moved off the classic <.../simplelink.h>):
 * Wlan_Get / WlanMacAddress_t / WLAN_GET_MACADDRESS / WLAN_ROLE_STA / Wlan_Start /
 * Wlan_RoleUp / RoleUpStaCmd_t.  Impl in wifi_stack.a (pulls the OSI port ->
 * osi_dpl.c); the Wlan_* refs here prove the P0-5 link. */
#include <wlan_if.h>
/* OSI sync-obj rendezvous (osi_SyncObj* / OSI_WAIT_FOREVER): the worker thread
 * Clear()s + waits, the Wi-Fi event cb (host-driver thread) Signal()s.  Impl in
 * wifi_platform_cc35xx.a via osi_dpl.c.  See WIFI_BLE_INTEGRATION.md rendezvous. */
#include <osi_kernel.h>
/* lwIP bring-up app-source (compiled from the demo into this image -- see
 * build_ti.ps1 $WifiHostDriver $sources): network_set_up / network_stack_add_if_ap
 * / network_stack_get_if_ip for the DHCP + AP + GET_IP bodies.  WIFI_GET_IP has
 * NO async event (per the integration recipe) -- it polls lwIP for a non-zero ip. */
#include <network_lwip.h>
/* The demo's network_lwip.c references the app control block app_CB (status bits
 * + the connect/dhcp OSI sync objs) which the demo defines in network_terminal.c
 * -- a TU we do NOT link (it carries the console main()).  Provide app_CB here,
 * zero-initialised: network_lwip.c only reads app_CB.Status and guards every
 * app_CB.CON_CB.*SyncObj use against NULL, so a zeroed block is safe (those
 * optional signals are simply skipped on this headless bridge). */
#include <network_terminal.h>
appControlBlock app_CB;
/* uptime source for cc3501e_hw_wifi_connect_sta's bounded DHCP-lease poll
 * (ClockP_usleep between tries so the tcpip thread runs DHCP). */
#include <ti/drivers/dpl/ClockP.h>
/* InitTerm() -- see the call in cc3501e_hw_net_init() for why a headless bridge
 * with an unrouted UART still has to initialise the vendor console. */
#include <uart_term.h>
/* netif_is_up / netif_is_link_up / netif_dhcp_data + struct dhcp, for the
 * GET_DIAG_INFO DHCP-state bytes (cc3501e_hw_wifi_dhcp_diag below). */
#include <lwip/netif.h>
#include <lwip/dhcp.h>
/* LOCK_TCPIP_CORE / UNLOCK_TCPIP_CORE for the stalled-DHCP restart in
 * cc3501e_hw_wifi_connect_sta(); network_set_up() takes the same lock from this
 * same worker context, so the precedent is the vendor's own. */
#include <lwip/tcpip.h>
#endif

#include "alp/protocol/cc3501e.h"

#include "../cc3501e_hw.h"
#include "cc3501e_hw_ti_internal.h" /* cc3501e_hw_wifi_lazy_start (shared with cc3501e_hw_ti_ble.c) */
#include "transport.h" /* bridge_transport_spi_hw_reinit/suspend, cc3501e_bridge_busy/ready */

/* Last 802.11 reason/status code the cb actually saw -- see the accessor
 * cc3501e_hw_wifi_last_reason() (both build variants, further down).  0 = none
 * recorded.  Declared UNCONDITIONALLY (unlike wifi_sta_role_up and friends,
 * which stay inside the #ifdef CC3501E_WIFI branch below): the connect-status
 * latch's cc3501e_hw_wifi_mark_connecting() (itself unconditional, further
 * down, so both ti sub-builds link) reads and clears it directly, so it must
 * be visible in the non-Wi-Fi ti build too, not just the real one that writes
 * it from wifi_event_cb().  wifi_conn_set(), the other reader, stays
 * CC3501E_WIFI-only -- unlike mark_connecting(), it has no reason to run (or
 * even exist) without a real connect body to call it, and every one of its
 * call sites is itself inside that same #ifdef. */
static volatile int16_t wifi_last_reason;

/* Is a connect attempt currently OPEN -- mark_connecting() has run and the
 * terminal wifi_conn_set() has not (yet) published CONNECTED/CONN_FAILED/
 * DISCONNECTED?  Forward-declared here so wifi_event_cb() (right below, well
 * before g_wifi_conn's own declaration further down) can call it; defined
 * next to g_wifi_conn, which it reads.
 *
 * SAFE TO READ FROM wifi_event_cb()'S CONTEXT (the host-driver thread, a third
 * context alongside the SPI-ISR/protocol context that calls mark_connecting()
 * and the worker/task context that calls wifi_conn_set()): g_wifi_conn.state
 * is a single `volatile uint8_t` -- a one-byte load the compiler always
 * re-reads from memory (no cached/stale value across the call) and that the
 * CPU cannot tear, so a concurrent write from either other context can only
 * make this return the OLD state or the NEW one, never a corrupt in-between
 * value.  A single stale read (seeing the old state for one more event) is
 * exactly the race this admits, and is what the "RESIDUAL" note on the
 * DISCONNECT case below and on g_wifi_conn's own comment describes -- nothing
 * worse.
 *
 * Declared inside the #ifdef CC3501E_WIFI block right below, not here: an
 * unguarded declaration with no matching call in the non-Wi-Fi ti build warns
 * as unused there too, not just an unguarded definition, and wifi_event_cb()
 * (its only caller) is itself inside that same block. */

#ifdef CC3501E_WIFI
static bool wifi_conn_is_connecting(void);

/* --------------------------------------------------------------- *
 * Lazy Wi-Fi bring-up (P0-6) -- the CC35xx host stack is started ONCE,    *
 * on first use, from the async worker's drain (NOT the SPI ISR).  Per     *
 * WIFI_BLE_INTEGRATION.md "Init order": Wlan_Start(eventCB) ->            *
 * Wlan_RoleUp(WLAN_ROLE_STA, ...).  Both block (RoleUp can take seconds), *
 * which is exactly why this is gated behind the worker.                   *
 * --------------------------------------------------------------- */

/* One-time guard: the stack is started exactly once for the process
 * lifetime; subsequent GET_MAC / Wi-Fi ops skip straight to the op. */
static bool wifi_started;

/* One-time STA role-up guard.  Wlan_Start (lazy_start) is enough for the factory
 * MAC read, but SCAN and CONNECT need the STA ROLE up.  RoleUp is brought up once
 * (bounded timeout, NOT WLAN_WAIT_FOREVER -- a stuck role-up must never hang the
 * worker / the bridge SPI re-open).  Shared by scan + connect so RoleUp runs once. */
static bool wifi_sta_role_up;

/* Role-up/down budget, shared by the STA and AP paths so they cannot drift.
 * 10000 ms is the value the STA path has used since bring-up. */
#define CC3501E_WIFI_ROLE_TIMEOUT_MS 10000u

/* AP role-up latch, mirroring wifi_sta_role_up.  Set on a successful
 * Wlan_RoleUp(AP), cleared on a successful Wlan_RoleDown(AP), and reported via
 * cc3501e_hw_radio_role() -> GET_DIAG_INFO's `role` field.  This is firmware
 * BOOKKEEPING, not a radio query: it says "we brought the AP role up and have
 * not torn it down", which is precisely the claim #1562 needs to compare
 * against an AP that has stopped advertising.  If the host ever observes
 * role == WIFI_AP while a second radio sees no beacon, the NWP dropped the AP
 * underneath us without the firmware asking -- a different fault from the
 * firmware tearing it down, and the two are indistinguishable today. */
static bool wifi_ap_role_up;

/* ---- Worker <-> Wi-Fi-event rendezvous (WIFI_BLE_INTEGRATION.md) ----------
 * Async Wlan_* ops (Scan / Connect) complete on the host-driver thread via the
 * event cb, NOT on the worker thread that issued them.  The worker Clear()s the
 * sync obj, issues the op, then Wait()s; the cb copies the result into the cache
 * below and Signal()s.  Created once at first lazy-start (osi services are up by
 * then).  A single rendezvous is sufficient: the worker is single-in-flight
 * (one Wi-Fi op at a time), matching src/worker.c's single-job seam.
 *
 * Scope note (architecture, not a TODO): src/worker.c routes only argument-free
 * getters one at a time, and the WIFI protocol handlers in src/protocol.c are
 * direct synchronous calls (NOT poll-by-repeat like GET_MAC) -- the native_sim
 * suite locks single-frame NOT_READY for SCAN/CONNECT.  These bodies therefore
 * block in their caller's context; on silicon the seconds-long ones must be
 * reached from the worker drain (as GET_MAC is).  Wiring SCAN/CONNECT through
 * the worker needs an arg-carrying multi-job worker + poll-by-repeat handlers
 * (a wire/seam redesign) -- out of scope here (see RETURN notes). */
static OsiSyncObj_t wifi_event_sync;  /* signalled by the event cb        */
static volatile int wifi_last_status; /* WLAN event Status (<0 = fail)    */

/* Scan-result cache: the whole AP list arrives in ONE WLAN_EVENT_SCAN_RESULT.
 * The cb snapshots it here; a SCAN body copies/packs it after the wait. */
#define WIFI_SCAN_CACHE_MAX WLAN_MAX_SCAN_COUNT
static volatile uint32_t  wifi_scan_count;
static WlanNetworkEntry_t wifi_scan_cache[WIFI_SCAN_CACHE_MAX];

/* Wi-Fi event callback -- runs on the host-driver task (NOT the SPI ISR, NOT
 * the worker).  Snapshots the result the issuing op is waiting on, then signals
 * the rendezvous.  GET_MAC takes no event (blocking get) so it is not handled
 * here.  osi_SyncObjSignal (not *FromISR): the cb is thread context. */
/* DIAG: count EVERY Wi-Fi event the host-driver task delivers + record the last
 * Id, so a scan that never returns SCAN_RESULT can be split: NWP fired NO event
 * at all (RX/scan-engine stall -> antenna/HW) vs fired OTHER events (cb gap). */
static volatile uint32_t wifi_cb_event_count;
static volatile uint32_t wifi_cb_last_id;

/* IEEE 802.11 status code wifi_event_cb() acts on below (ASSOCIATION_REJECTED
 * temporary-reject handling -- see that case; status 17,
 * AP_UNABLE_TO_HANDLE_NEW_STA, gets no vendor retry and stays terminal, so it
 * is discussed there by numeric value only, not given its own define here).
 * Named locally rather than #include'd: the canonical definition is hostap's
 * ieee802_11_defs.h (source/third_party/hostap/src/common/ieee802_11_defs.h:135),
 * but hostap ships this SDK as a PREBUILT archive (hostap.a, see
 * ti/build_ti.sh's link list) with no -I for its src/common, so that header is
 * not on this TU's include path. */
#define CC3501E_WLAN_STATUS_ASSOC_REJECTED_TEMPORARILY 30u /* ieee802_11_defs.h:135 */

static void wifi_event_cb(WlanEvent_t *event)
{
	if (event == NULL) {
		return;
	}
	wifi_cb_event_count++;
	wifi_cb_last_id = (uint32_t)event->Id;
	switch (event->Id) {
	case WLAN_EVENT_SCAN_RESULT: {
		uint32_t n = event->Data.ScanResult.NetworkListResultLen;
		if (n > WIFI_SCAN_CACHE_MAX) {
			n = WIFI_SCAN_CACHE_MAX;
		}
		for (uint32_t i = 0; i < n; i++) {
			wifi_scan_cache[i] = event->Data.ScanResult.NetworkListResult[i];
		}
		wifi_scan_count  = n;
		wifi_last_status = 0;
		osi_SyncObjSignal(&wifi_event_sync);
		break;
	}
	case WLAN_EVENT_EXTENDED_SCAN_RESULT: {
		/* The NWP delivers scan results in the EXTENDED format -- a pointer to a
		 * separate list -- not WLAN_EVENT_SCAN_RESULT.  Its entries share the
		 * same leading fields as the normal WlanNetworkEntry_t, so copy those
		 * into the same cache the wire-packer reads.  WITHOUT this case the cb
		 * dropped the only event the scan fired and the host saw -4 (proven by
		 * the event-count diag + the TI SDK two-event model). */
		WlanEventExtendedScanResult_t *ext = event->Data.pExtendedScanResult;
		const int ext_null = (ext == NULL || ext->NetworkListResult == NULL) ? 1 : 0;
		uint32_t  n        = ext_null ? 0u : ext->NetworkListResultLen;
		if (n > WIFI_SCAN_CACHE_MAX) {
			n = WIFI_SCAN_CACHE_MAX;
		}
		for (uint32_t i = 0; i < n; i++) {
			const WlanNetworkEntryExtended_t *e = &ext->NetworkListResult[i];
			memcpy(wifi_scan_cache[i].Ssid, e->Ssid, WLAN_SSID_MAX_LENGTH);
			memcpy(wifi_scan_cache[i].Bssid, e->Bssid, WLAN_BSSID_LENGTH);
			wifi_scan_cache[i].SsidLen      = e->SsidLen;
			wifi_scan_cache[i].Rssi         = e->Rssi;
			wifi_scan_cache[i].SecurityInfo = e->SecurityInfo;
			wifi_scan_cache[i].Channel      = e->Channel;
		}
		wifi_scan_count = n;
		wifi_last_status =
		    ext_null ? -77 : 0; /* DIAG: -77 = EXTENDED event had a NULL data pointer */
		osi_SyncObjSignal(&wifi_event_sync);
		break;
	}
	case WLAN_EVENT_CONNECT:
		/* WlanEventConnect_t::Status is NOT a general result code with a
		 * negative-on-failure convention: this SDK's only two writers
		 * (cme_connection_mng.c ~8025/~8454) set it to
		 * SL_WLAN_CONNECT_EVENT_STATUS_SUCCESS or _ALREADY_CONNECTED, both
		 * non-negative -- a failed connect arrives as one of the three events
		 * below instead, never as a negative CONNECT.Status.  An earlier
		 * version of this cb stashed Status when negative; that branch never
		 * fired on this SDK and a truncated int32 would not have decoded to
		 * anything meaningful anyway, so it was removed rather than kept as
		 * dead code. */
		wifi_last_status = (int)event->Data.Connect.Status;
		osi_SyncObjSignal(&wifi_event_sync);
		break;
	case WLAN_EVENT_DISCONNECT:
		/* A connect attempt that the FW rejects arrives as one of these three
		 * rather than a negative CONNECT.Status; surface it as a failure. */
		wifi_last_status = -1;
		/* Record the reason ONLY while an attempt is OPEN (mark_connecting()
		 * ran, wifi_conn_set() has not yet published a terminal state -- see
		 * wifi_conn_is_connecting()'s own comment for the exact contract and
		 * why reading it here is safe) -- and NEVER record 200
		 * (WLAN_DISCONNECT_USER_INITIATED): the vendor hardcodes that value
		 * whenever a disconnect is issued while its OWN station state machine
		 * happens to be idle (cme.c ~3298/~3335), and it was never a real
		 * 802.11 reason regardless of who caused it.
		 *
		 * This replaces an earlier firmware-owned "we asked for this
		 * disconnect" flag (wifi_own_disconnect_pending) that tried to track
		 * our own Wlan_Disconnect() calls directly.  That flag had real gaps:
		 * cc3501e_hw_wifi_disconnect() could return an error WITHOUT clearing
		 * it (the vendor returns an error and emits NO event with no STA/P2P
		 * role up, cme.c ~1645-1654, or when its pending-op check rejects the
		 * call, wlan_if.c ~191-200/~1324-1329; a role switch can also return
		 * OK with NO event, cme.c ~1667-1672) -- any of those left the flag
		 * armed to wrongly swallow a LATER, unrelated event's reason.  A
		 * vendor-generated event unrelated to our own disconnect (e.g.
		 * AUTHENTICATION_REJECTED's underlying WPA_SUPP_DISCONNECTED,
		 * drv_ti_mlme.c ~1177) could also consume it.  Gating on OUR OWN
		 * connecting-state latch instead needs no bookkeeping that a vendor
		 * error path can leave stuck.
		 *
		 * RESIDUAL, stated plainly (this is an observability byte, not a
		 * safety one): mark_connecting() clears wifi_last_reason at SUBMIT
		 * (SPI-ISR/protocol context), and cc3501e_hw_wifi_connect_sta() clears
		 * it AGAIN right before Wlan_Connect (see there) -- but that second
		 * reset is still not the same instant the vendor actually begins
		 * processing the new connect.  A late event from the PREVIOUS attempt
		 * (still in flight on this host-driver thread -- e.g. a disconnect WE
		 * issued while actually connected or mid-association, host
		 * WIFI_DISCONNECT or the #1437 cleanup, whose vendor DISCONNECT
		 * carries REASON 3 / WLAN_REASON_DEAUTH_LEAVING, a REAL 802.11 code,
		 * not 200) landing in that tiny remaining window can still be recorded
		 * against the NEW attempt.  Not scoped to reason 3 specifically any
		 * more (that was the wider pre-second-reset window's shape) -- ANY
		 * late event from the prior attempt that lands there is recorded as
		 * if it belonged to the new one.  See hal/cc3501e_hw.h's
		 * cc3501e_hw_wifi_last_reason() contract, which states this plainly
		 * for the host too.
		 *
		 * FIRST REAL CODE WINS: only write when wifi_last_reason == 0 (still
		 * nothing recorded for this attempt).  Needed because ASSOCIATION_REJECTED's
		 * non-terminal handling (see that case) lets a genuine rejection --
		 * status 30 -- sit recorded while the vendor's own comeback retry runs;
		 * a retry sequence that ultimately fails ends in exactly THIS event,
		 * with a generic, self-inflicted reason (3, WLAN_REASON_DEAUTH_LEAVING,
		 * from hostap's own give-up path, sme_deauth() at sme.c ~2228-2245)
		 * that carries far less information than the 30 already recorded.
		 * Without this guard, that generic 3 would silently replace the real
		 * rejection reason right as the attempt goes terminal, which is the
		 * one moment a host is guaranteed to actually read this byte.  Does
		 * NOT change the residual above: that scenario also starts from
		 * wifi_last_reason == 0 (a fresh attempt's own reset), so it still
		 * writes exactly as documented.  Does NOT apply to
		 * ASSOCIATION_REJECTED / AUTHENTICATION_REJECTED themselves -- each of
		 * those is always itself a specific, real status worth recording, so a
		 * second one differing from a first (however that happened) is not
		 * the same "generic close-out overwrites a real reason" problem this
		 * guard exists for. */
		if (wifi_conn_is_connecting() && wifi_last_reason == 0 &&
		    event->Data.Disconnect.ReasonCode != (int16_t)WLAN_DISCONNECT_USER_INITIATED) {
			wifi_last_reason = event->Data.Disconnect.ReasonCode;
		}
		osi_SyncObjSignal(&wifi_event_sync);
		break;
	case WLAN_EVENT_ASSOCIATION_REJECTED: {
		/* AssocStatusCode is a real 802.11 status code (driver/drv_ti/
		 * drv_ti_mlme.c ~1287, wlanDispatcherSendEvent(..., sizeof(uint16_t))
		 * off apMngPack->u.assoc_resp.status_code) -- worth the same low-byte
		 * publication a DISCONNECT's ReasonCode gets.  Same CONNECTING-only
		 * gate as DISCONNECT above. */
		const uint16_t status = event->Data.AssocStatusCode;

		/* BENCH (Run8, e1m-aen-evk-01, GPE 0.254.8.0, alp-console connect-first
		 * WPA3, cold power cycle ~1.5 min apart): the connect result strictly
		 * ALTERNATED pass/fail across consecutive boots (P1: A R A R A R T A R
		 * A R A; P3 scan-first: A R A R).  Every rejection printed
		 * `fail: 2` (REJECTED) with WIFI_STATUS last_reason 30
		 * (CC3501E_WLAN_STATUS_ASSOC_REJECTED_TEMPORARILY) after ~4-6 s, last
		 * vendor event WLAN_EVENT_DISCONNECT.  INFERENCE (not proven by the
		 * SDK source, only by the alternating pattern): WPA3 requires PMF, and
		 * a cold power cycle with no deauth leaves the AP still holding the
		 * PREVIOUS boot's security association, so it rejects temporarily
		 * with an Association Comeback Time; the NEXT boot then finds that SA
		 * expired and succeeds -- hence the strict alternation.
		 *
		 * THE ACTUAL BUG this fix closes does not depend on that inference:
		 * on status 30 WITH a comeback-time IE (WLAN_TIMEOUT_ASSOC_COMEBACK),
		 * the vendor driver ITSELF already retries the association --
		 * ti_drv_rxAssocRespPacket() (drv_ti_mlme.c ~1291-1345) registers
		 * ti_drv_AssocTimeout via eloop_register_timeout(), armed for the AP's
		 * own comeback duration -- and, on firing, ti_drv_AssocTimeout()
		 * (drv_ti_sta_specific.c ~708-733) re-sends the association request
		 * (ti_drv_txAssocReqPacket()) for up to ASSOC_MAX_TRIES(3) attempts
		 * total (the initial try2associate() plus 2 internal driver resends)
		 * before giving up and reporting EVENT_ASSOC_TIMED_OUT to the
		 * supplicant.  Before this fix, THIS cb treated the FIRST rejection
		 * as terminal -- set wifi_last_status=-1 and signalled -- so
		 * cc3501e_hw_wifi_connect_sta() published FAILED/REJECTED immediately
		 * and ran wifi_clear_stale_assoc() -> Wlan_Disconnect(), aborting the
		 * vendor's own in-flight comeback retry out from under it -- exactly
		 * the alternating pass/fail pattern the bench recorded, independent
		 * of whether the PMF/stale-SA inference above is the right
		 * explanation for WHY the AP rejects.
		 *
		 * STATUS 17 (AP_UNABLE_TO_HANDLE_NEW_STA) IS NOT THE SAME SHAPE AND
		 * STAYS TERMINAL, per drv_ti_mlme.c:71,1325-1343: it cancels any
		 * pending ti_drv_AssocTimeout, then registers a NEW one only
		 * `if (ASSOC_TIMEOUT_MSECS < msecs)` with ASSOC_TIMEOUT_MSECS==200 and
		 * msecs==200 hardcoded right above it -- `200 < 200` is never true, so
		 * NO retry timer is ever actually armed for 17.  It still returns
		 * RX_MGMT_NONE (the frame never reaches the supplicant), so treating
		 * it as non-terminal here would only DELAY the eventual failure by
		 * however long this cb's caller waits, buying nothing.  Status 30
		 * WITHOUT the comeback-time IE is the same story by a different route
		 * (drv_ti_mlme.c ~1293-1295's `if` requires the IE; skipping it falls
		 * through to destroyAssocData() and a real RX_MGMT_ASSOC return) --
		 * ends terminally via the supplicant, no driver-level retry at all.
		 *
		 * THE REAL RETRY BOUND, traced beyond the driver: this SDK sets
		 * WPA_DRIVER_FLAGS_SME (driver_ti_wifi.c:3926), so hostap's SME layer
		 * (not the driver) owns the overall association clock. Starting an
		 * association arms SME_ASSOC_TIMEOUT (5, i.e. 5 s -- sme.c:35,
		 * eloop_register_timeout() at sme.c:2192) ONCE, for the FIRST
		 * association request.  The driver's own comeback retries above
		 * return RX_MGMT_NONE, so the frame never reaches wpa_supplicant and
		 * NOTHING re-arms or cancels that 5 s SME timer on the comeback path
		 * -- sme_assoc_timer() -> sme_deauth() (sme.c:2338-2345) fires 5 s
		 * after the FIRST association request if nothing terminal happened by
		 * then, regardless of how many driver-level retries are still
		 * in-flight.  So the vendor's own retry sequence for a genuine
		 * comeback (status 30 + IE) runs for AT MOST 5 s total, not up to the
		 * full ASSOC_MAX_TRIES(3)/comeback-interval math in isolation --
		 * hostapd's own default comeback time caps at 1000 TU (~1.024 s), so
		 * in practice retries land at roughly the 1 s and 2 s marks inside
		 * that 5 s window, comfortably inside it.
		 *
		 * FIX: 30 alone is NOT terminal here (17 is excluded -- see above).
		 * Record the status into the live reason (still gated on
		 * wifi_conn_is_connecting(), same rule as every other case) so a host
		 * polling WIFI_STATUS mid-retry sees it, but do NOT set
		 * wifi_last_status or signal -- the connect body simply keeps waiting
		 * on its EXISTING osi_SyncObjWait(&wifi_event_sync, 30s) in
		 * cc3501e_hw_wifi_connect_sta(), for the vendor's own retry to either
		 * succeed (WLAN_EVENT_CONNECT) or -- within its own ~5 s SME ceiling,
		 * well inside our 30 s wait -- report a REAL terminal event (a
		 * DISCONNECT with reason 3, WLAN_REASON_DEAUTH_LEAVING, from
		 * sme_deauth(); see that case's own comment for how this cb keeps the
		 * more informative 30 instead of letting that generic 3 overwrite
		 * it).  Any other rejected status is unaffected and stays terminal.
		 *
		 * TIMEOUT BUDGET: unaffected, and now doubly so given the ~5 s real
		 * ceiling just traced (comfortably inside the pre-existing 30 s
		 * association wait either way).  This does not add a wait; it only
		 * stops short-circuiting the association wait BEFORE the vendor's own
		 * retry (which was already going to run regardless, and was
		 * previously being aborted mid-flight by our own Wlan_Disconnect())
		 * gets to finish.  Role-up (bounded 10 s, CC3501E_WIFI_ROLE_TIMEOUT_MS)
		 * + this 30 s association wait + the DHCP poll's 20 s budget
		 * (CC3501E_STA_DHCP_TRIES(100) * CC3501E_STA_DHCP_POLL_US(200 ms), see
		 * below) sums to the SAME 60 s deepest path against the host apps'
		 * 75000 ms CONNECT timeout budget as before -- no new headroom is
		 * spent, and 15 s of that budget was already unused margin.
		 *
		 * AUTHENTICATION_REJECTED with the SAME statuses is deliberately left
		 * terminal below, NOT given this treatment: traced in the SDK
		 * (destroyAuthData(), drv_ti_sta_specific.c ~1112-1124) that the
		 * driver CANCELS ti_drv_AuthTimeout (eloop_cancel_timeout()) for
		 * exactly this rejection path (drv_ti_mlme.c ~1183-1191, the
		 * DENY_LIST_EN branch for 17/30/DENIED_INSUFFICIENT_BANDWIDTH/
		 * REQUEST_DECLINED) and returns RX_MGMT_NONE -- no retry timer is
		 * registered (contrast the ASSOC-comeback path's
		 * eloop_register_timeout() above), and RX_MGMT_NONE means
		 * wpa_supplicant's own SME layer never even sees the frame, so it has
		 * no basis to retry either.  Nothing in this SDK resumes an auth
		 * rejected with 17/30 once the deny-list timer is armed; the
		 * ASSOC-comeback retry is genuinely a different, unique mechanism, not
		 * one that also covers AUTH. */
		if (status == CC3501E_WLAN_STATUS_ASSOC_REJECTED_TEMPORARILY) {
			if (wifi_conn_is_connecting()) {
				wifi_last_reason = (int16_t)status;
			}
			break; /* deliberately NO wifi_last_status write, NO signal --
			        * this is not terminal; see the comment above.  Status 17
			        * is NOT included here -- it gets no vendor retry either,
			        * so treating it as non-terminal would only delay the
			        * failure; it falls through to the terminal path below. */
		}

		wifi_last_status = -1;
		if (wifi_conn_is_connecting()) {
			wifi_last_reason = (int16_t)status;
		}
		osi_SyncObjSignal(&wifi_event_sync);
		break;
	}
	case WLAN_EVENT_AUTHENTICATION_REJECTED:
		wifi_last_status = -1;
		/* Same as ASSOCIATION_REJECTED above: AuthStatusCode is a real 802.11
		 * status code (drv_ti_mlme.c ~1177, off apMngPack->u.auth.status_code).
		 * Kept fully terminal, including for status 17/30 -- see the
		 * ASSOCIATION_REJECTED case's comment for why nothing in the vendor
		 * SDK retries an auth rejection the way it retries an association
		 * rejection. */
		if (wifi_conn_is_connecting()) {
			wifi_last_reason = (int16_t)event->Data.AuthStatusCode;
		}
		osi_SyncObjSignal(&wifi_event_sync);
		break;
	default:
		break; /* CONNECTING / peer / P2P / etc. -- not awaited here */
	}
}

/* DIAG: last Wi-Fi event Id the cb saw (any type) + the running count, surfaced
 * via GET_DIAG_INFO so the host can identify which event the scan actually fires
 * when neither SCAN_RESULT nor EXTENDED_SCAN_RESULT signals. */
uint32_t cc3501e_hw_wifi_last_event_id(void)
{
	return wifi_cb_last_id;
}

/* See the contract on the declaration in hal/cc3501e_hw.h. */
void cc3501e_hw_wifi_dhcp_diag(uint8_t *state_out, uint8_t *flags_out)
{
	if (state_out != 0) *state_out = 0u;
	if (flags_out != 0) *flags_out = 0u;

	/* network_stack_init() has not run on a boot that never brought lwIP up
	 * (update mode skips cc3501e_hw_net_init entirely), so the netif pointer is
	 * the thing to guard on, not wifi_started: the netif is registered at BOOT,
	 * long before any radio op. */
	struct netif *nif = (struct netif *)network_get_sta_if();
	if (nif == 0) {
		return;
	}

	if (flags_out != 0) {
		uint8_t f = 0u;
		if (netif_is_up(nif)) f |= 0x01u;
		if (netif_is_link_up(nif)) f |= 0x02u;
		*flags_out = f;
	}

	/* NULL until dhcp_start() runs, which is exactly the case worth telling
	 * apart -- it leaves state_out at 0, "not reported", rather than
	 * fabricating a DHCP_STATE_OFF that would look like a started-then-stopped
	 * client. */
	struct dhcp *d = netif_dhcp_data(nif);
	if (d == 0) {
		return;
	}
	if (state_out != 0) {
		/* +1 so 0 stays reserved for "not reported"; d->state is u8_t and the
		 * enum tops out well below 255, so this cannot wrap. */
		*state_out = (uint8_t)(d->state + 1u);
	}
	if (flags_out != 0) {
		const uint8_t tries = (d->tries > 63u) ? 63u : d->tries;
		*flags_out          = (uint8_t)((*flags_out & 0x03u) | (uint8_t)(tries << 2));
	}
}

/* Current WI-FI role for GET_DIAG_INFO (see cc3501e_hw.h).  Pure bookkeeping --
 * no Wlan_Get, so this is safe to poll while a role is up.
 *
 * AP outranks STA in the single-value wire field: the AP role is the one a host
 * polls about, and scan/connect leave wifi_sta_role_up latched for the process
 * lifetime, so reporting STA would mask every AP.
 *
 * BLE is deliberately NOT folded in (no ROLE_BLE_* / ROLE_DUAL_WIFI_BLE here):
 * cc3501e_nimble_host_is_enabled() lives behind CC3501E_BLE in a different TU,
 * and reaching for it would make this Wi-Fi TU fail to build in the Wi-Fi-only
 * configuration. The field's consumer (#1562) asks about the soft-AP. */
uint8_t cc3501e_hw_radio_role(void)
{
	if (wifi_ap_role_up) return (uint8_t)ALP_CC3501E_ROLE_WIFI_AP;
	if (wifi_sta_role_up) return (uint8_t)ALP_CC3501E_ROLE_WIFI_STA;
	return (uint8_t)ALP_CC3501E_ROLE_OFF;
}

/* See the contract on the declaration in cc3501e_hw_ti_internal.h -- the direct
 * STA-specific answer cc3501e_hw_radio_role() cannot give once AP is also up. */
bool cc3501e_hw_wifi_sta_role_up(void)
{
	return wifi_sta_role_up;
}

/* Defined below; boot_start pre-caches the STA role through it. */
static int cc3501e_hw_wifi_ensure_sta_role(void);

/* Bring the CC35xx Wi-Fi host up to STA role once.  Returns CC3501E_HW_OK
 * when the stack is up (already-started is OK), CC3501E_HW_ERR_IO on a
 * start/role failure.  Called from the worker drain before the first
 * Wlan_* data op.  Non-static (declared in cc3501e_hw_ti_internal.h):
 * cc3501e_hw_ti_ble.c's BLE_ENABLE also calls it (BLE shares the HIF, so
 * Wlan_Start must run first). */
int cc3501e_hw_wifi_lazy_start(void)
{
	if (wifi_started) {
		return CC3501E_HW_OK;
	}
	/* Create the event rendezvous before Wlan_Start so the cb can signal it. */
	if (osi_SyncObjCreate(&wifi_event_sync) != OSI_OK) {
		return CC3501E_HW_ERR_IO;
	}
	/* Wlan_Start internally runs InitHostDriver -> HIF/NWP bring-up + FW
	 * download (wlan_if.c) and reads the factory MAC into the CME cache; it is
	 * the ONLY radio init GET_MAC needs.  Do NOT Wlan_RoleUp(STA) here: RoleUp is
	 * for the connect/scan/AP roles, NOT for the factory-MAC read.
	 *
	 * === bridge-SPI recovery (SHIP-CRITICAL, proven on silicon) ===
	 * Wlan_Start's HIFInit re-points the shared host-DMA + the NWP IRQ to bring up
	 * the host<->NWP link, which DISRUPTS the bridge SPI slave (the host then reads
	 * 0x00000000 from a dead link).  This happens as a SIDE EFFECT of HIFInit
	 * REGARDLESS of whether Wlan_Start ultimately succeeds, so the bridge MUST be
	 * re-opened after the attempt EVEN ON FAILURE -- bench-proven: with reinit only
	 * on the success path, a failing/slow Wlan_Start left the SPI dead forever
	 * (ping_ok stuck 0, reqhdr_rx=0x00000000).  bridge_transport_spi_hw_reinit()
	 * does the real SPI_close + SPI_open + re-arm (transport owns the handle). */
	const int start_rv = Wlan_Start(wifi_event_cb);
	bridge_transport_spi_hw_reinit();
	if (start_rv != 0) {
		return CC3501E_HW_ERR_IO; /* radio not up; link is back, GET_MAC reports IO */
	}

	wifi_started = true;
	return CC3501E_HW_OK;
}

/* Boot-time radio bring-up (radio<->SPI coexistence fix; see cc3501e_hw.h).
 * Runs the SAME one-time lazy-start, but EARLY -- the bring-up task calls this
 * once after the SPI poll task is up and before it services any host command,
 * so the seconds-long Wlan_Start/RoleUp (during which the bridge is down) runs
 * with no host traffic to disrupt.  The wifi_started one-time guard inside
 * makes this idempotent: a later GET_MAC sees wifi_started==true and skips
 * straight to the short Wlan_Get.  cc3501e_hw_wifi_lazy_start() already re-syncs
 * the bridge slave (bridge_transport_spi_hw_reinit) after the radio is up, so the
 * link is re-armed at a clean header boundary before the first host command. */
void cc3501e_hw_wifi_boot_start(void)
{
	cc3501e_bridge_busy(); /* configure GPIO17 + hold the host off through the boot radio init */
	(void)cc3501e_hw_wifi_lazy_start();
	/* Bring the STA role up ONCE here at boot, before the host polls, so each later
	 * scan is a LIGHT Wlan_Scan (role already up) rather than carrying the role-up.
	 *
	 * CORRECTION -- an earlier version of this comment said RoleUp "needs the bridge
	 * quiesced (suspend) to kick".  Measurement refutes that:
	 * cc3501e_hw_wifi_scan_run() performs the SAME ensure_sta_role(), equally
	 * unguarded, and a scan issued as the first radio operation of a boot returns
	 * records 5 of 5 cold-booted runs (prebuilt/CHANGELOG.md).  What the role-up
	 * needs is the reinit AFTER it, not a suspend before it.  The suspend below is
	 * also INERT at this call site when spi == NULL -- it degenerates to two flag
	 * writes and never reaches SPI_transferCancel -- so it is not what holds this
	 * function together either.
	 *
	 * WHY THIS FUNCTION IS STILL NOT CALLED, since the correction above makes it
	 * look newly safe and it is not.  src/main.c leaves it out deliberately, and
	 * the blocking reason is NOT the suspend:
	 *
	 *   1. The MCUboot trial accept is gated on g_host_txn_count > 0 -- one fully
	 *      drained host reply (cc3501e_hw_ti.c) -- deliberately, so an image that
	 *      boots but wedges the bridge never becomes permanent.  Moving radio
	 *      bring-up ahead of transport_spi_init() therefore puts a possible hang
	 *      in front of an accept that cannot fire yet, and on a freshly flashed
	 *      TRIAL image that is the 2026-06-18 no-launch state.
	 *   2. This function reaches bridge_transport_spi_hw_reinit() while spi == NULL,
	 *      which opens and arms the slave; transport_spi_init() then opens it
	 *      again, and SPI_open on an already-open index returns NULL.  Nothing
	 *      closes in between.
	 *
	 * The first-radio-op wedge it would have addressed is ~2 in 16 cold boots and
	 * is handled host-side instead -- see prebuilt/CHANGELOG.md's residual list. */
	bridge_transport_spi_hw_suspend();
	(void)cc3501e_hw_wifi_ensure_sta_role();
	bridge_transport_spi_hw_reinit();
	cc3501e_bridge_ready(); /* boot radio init done, slave armed -> host may clock */
}

/* Bring up the lwIP TCP/IP core ONCE, EARLY -- called from bringup_task BEFORE
 * transport_spi_init() spawns the busy-poll bridge slave task AND before the radio
 * is lazy-started.  WHY this exact spot (root-caused on silicon 2026-06-23): tcpip_init
 * spawns the lwIP tcpip thread then sys_sem_wait()s for it to start.  Run from the
 * worker drain AFTER Wlan_Start, that wait HUNG the worker -- the bridge slave poll
 * task busy-waits the RX FIFO at priority 8 (one below bringup), which starves the
 * lwIP thread (TCPIP_THREAD_PRIO clamps to <=9) so it never signals, and the radio
 * had already consumed the FreeRTOS heap the 16 KB tcpip stack needs.  Calling it
 * here -- before that poll task exists and before any radio allocation -- lets the
 * tcpip thread start cleanly.  Once the core mutex exists, the later
 * network_stack_add_if_sta() (ensure_sta_role) is a synchronous LOCK_TCPIP_CORE +
 * netif_add (no thread wait), so it is unaffected by the busy-poll thereafter. */
void cc3501e_hw_net_init(void)
{
	/* Initialise the vendor console terminal.  This bridge has no console and the
	 * UART is not routed, so the obvious reading is that this call is pointless.  It
	 * is not: WITHOUT it the shipped image dereferences a NULL UART handle on every
	 * station connect.
	 *
	 * Read out of build/ti/cc3501e-bridge.out as shipped in v0.8.0:
	 *
	 *     link_callback:  ... bl <Report>        ; "link_callback==UP starting DHCP"
	 *                     ... bl <dhcp_start>
	 *
	 *     Report -> Message -> UART_writePolling -> putch
	 *            -> UART2_write(uartHandle, ...)
	 *     UART2_writeTimeout:  ldr r4, [r0, #0]  ; the handle, with NO null check
	 *
	 * and `InitTerm` does not appear in that image at all -- it was dead-stripped
	 * because nothing called it -- so `uartHandle` is never assigned and stays NULL
	 * from .bss.  The vendor's link_callback() (network_terminal demo's
	 * network_lwip.c, which ti/build_ti.sh compiles verbatim) calls Report() there
	 * unconditionally, and hal/ti/cc3501e_hw_ti_sock.c calls it on its own socket
	 * error paths.  Both run that dereference: on the worker task, under
	 * LOCK_TCPIP_CORE, immediately before dhcp_start().
	 *
	 * ti/cc3501e_aen_wifi.syscfg says "the bridge firmware never calls
	 * InitTerm()/Report()".  The first half is what caused this; the second half is
	 * simply wrong, and the disassembly above is why.
	 *
	 * NOT CLAIMED: that this is why the station never gets a lease.  The connect
	 * body demonstrably returns and the bridge demonstrably keeps serving BLE, GPIO
	 * and diagnostics afterwards, so whatever that dereference does on this silicon,
	 * it is survivable today.  What cannot stand is a shipped image whose Wi-Fi
	 * connect path depends on the contents of address 0 -- that is undefined
	 * behaviour whose outcome can change with any relink.
	 *
	 * Opening the unrouted UART is proven safe rather than assumed: a probe build on
	 * 2026-08-29 opened UART2_0, got a non-NULL handle and wrote 188988 bytes with
	 * UART2_STATUS_SUCCESS (both XDS110 COM ports received nothing, because
	 * GPIO5/GPIO6 do not reach the probe on this SoM -- see the syscfg).  The cost is
	 * a polled TX of a few dozen characters at 115200 on the connect path, a couple
	 * of milliseconds, and TX drains with no receiver attached.
	 *
	 * Called BEFORE network_stack_init() so the terminal is valid before any vendor
	 * callback can reach Report(). */
	InitTerm();

	network_stack_init();
	/* Register the STA netif HERE at boot too -- before transport_spi_init() spawns
	 * the busy-poll bridge task.  network_stack_add_if_sta() does LOCK_TCPIP_CORE +
	 * netif_add; done LATER from the radio path (ensure_sta_role) it DEADLOCKED the
	 * worker -- the busy-poll task (prio 8, one below bringup) starves the lwIP tcpip
	 * thread, which holds/needs the core lock, so LOCK_TCPIP_CORE never returns
	 * (silicon 2026-06-23: scan -4 / READY stuck low after adding the netif there).
	 * At boot, before that poll task exists, the tcpip thread runs and netif_add
	 * completes.  The registration is persistent (a static netif); the later
	 * Wlan_RoleUp(STA) binds it.  The STA netif's linkoutput (the WLAN tx) is what the
	 * connect EAPOL-4way / WPA3-SAE handshake flows over -- without it the NWP raises
	 * no connect event (the no-connect-event root cause). */
	network_stack_add_if_sta();
}
#else  /* !CC3501E_WIFI -- default v0.1 ti build brings up no radio */
void cc3501e_hw_wifi_boot_start(void)
{
	/* No radio linked in this build -- nothing to bring up. */
}
void cc3501e_hw_net_init(void)
{
	/* No lwIP linked in this build -- nothing to bring up. */
}
uint8_t cc3501e_hw_radio_role(void)
{
	/* No radio linked in this build -- no role can be up.  Same reason as
	 * cc3501e_hw_wifi_last_event_id below: GET_DIAG_INFO reads it
	 * unconditionally, so the non-Wi-Fi ti build must define it too. */
	return (uint8_t)ALP_CC3501E_ROLE_OFF;
}
bool cc3501e_hw_wifi_sta_role_up(void)
{
	/* No radio linked in this build -- no STA role can be up.  cc3501e_hw_ti_power.c
	 * calls this unconditionally (it links against both ti sub-builds), so the
	 * non-Wi-Fi ti build must define it too. */
	return false;
}
uint32_t cc3501e_hw_wifi_last_event_id(void)
{
	/* No radio linked in this build -- no Wi-Fi events to report.  protocol.c's
	 * GET_DIAG_INFO reads this unconditionally, so the non-Wi-Fi ti build must
	 * define it too (matches cc3501e_hw_stub.c). */
	return 0u;
}
void cc3501e_hw_wifi_dhcp_diag(uint8_t *state_out, uint8_t *flags_out)
{
	/* No lwIP linked in this build, so there is no DHCP client to report on.
	 * Zero is the wire's "not reported", which is the honest answer here --
	 * same reason cc3501e_hw_wifi_last_event_id() above must exist. */
	if (state_out != 0) *state_out = 0u;
	if (flags_out != 0) *flags_out = 0u;
}
#endif /* CC3501E_WIFI */

/* Read the CC3501E's 6-byte factory MAC (see ../cc3501e_hw.h).  Reached ONLY
 * from the async worker's drain, like the rest of this file's Wlan_* ops. */
int cc3501e_hw_get_mac(uint8_t mac[6])
{
	if (mac == 0) {
		return CC3501E_HW_ERR_INVAL;
	}
#ifdef CC3501E_WIFI
	/* Real factory-MAC read via the CC35xx Wi-Fi host API.  WLAN_GET_MACADDRESS
	 * is a blocking get (no async event).  This is reached ONLY from the
	 * async worker's drain (worker_run_pending, on bringup_task), never from
	 * the SPI ISR (P0-4/P0-6): the ISR submits the job and polls the cached
	 * result, so the seconds-long lazy Wi-Fi init + Wlan_Get below run off
	 * the ISR.  cc3501e_hw_wifi_lazy_start() brings the stack up once (per
	 * WIFI_BLE_INTEGRATION.md init order) before the first Wlan_* op. */
	const int wifi_rv = cc3501e_hw_wifi_lazy_start();
	if (wifi_rv != CC3501E_HW_OK) {
		return wifi_rv;
	}
	WlanMacAddress_t p = { .roleType = WLAN_ROLE_STA };
	if (Wlan_Get(WLAN_GET_MACADDRESS, &p) != 0) {
		return CC3501E_HW_ERR_IO;
	}
	for (int i = 0; i < 6; i++) {
		mac[i] = (uint8_t)p.pMacAddress[i];
	}
	return CC3501E_HW_OK;
#else
	/* v0.1 brings up no network processor, so the factory MAC is not
	 * readable yet -- report NOTIMPL, which the protocol layer maps to
	 * RESP_ERR_NOT_READY.  The real read lands under CC3501E_WIFI (above). */
	(void)mac;
	return CC3501E_HW_ERR_NOTIMPL;
#endif
}

/* ---- async-connect status latch (CMD_WIFI_STATUS) ------------------------- *
 * The worker-routed connect body BLOCKS for seconds on the association event, so
 * the host no longer polls it to completion (which clocked the bridge while the
 * radio op held it down -- the -4/desync wall).  Instead the firmware mirrors the
 * outcome into this latch, which the NON-blocking CMD_WIFI_STATUS reads off the
 * SPI ISR (no radio op).  Defined unconditionally (no SDK needed) so it links in
 * BOTH ti sub-builds; the connect body (CC3501E_WIFI) is the only terminal writer.
 *
 * Concurrency: written by mark_connecting() in the SPI-ISR/protocol context at
 * SUBMIT, then by the connect body on the drain thread at completion -- never both
 * at once (the worker is single-in-flight).  Read by handle_wifi_status (SPI ISR).
 * volatile byte fields; state is published LAST so a reader seeing a terminal state
 * also sees the matching fail_reason (release-style, mirrors the worker's publish
 * discipline).
 *
 * rssi is NOT part of that guarantee: it is never populated.  wifi_conn_set()
 * always sets it to 0 because this NWP cannot be asked for a beacon measurement
 * on the connect path (see the hazard note in cc3501e_hw_wifi_connect_sta), so
 * the field has only ever held 0.  The host must NOT treat it as a signal level
 * -- WIFI_GET_RSSI is the real read (issue #1387).
 *
 * reason is a FROZEN COPY of wifi_last_reason (below), taken by wifi_conn_set()
 * at the moment it publishes a terminal state -- NOT a live mirror.  Freezing
 * matters even though wifi_event_cb() also gates writes to wifi_last_reason on
 * wifi_conn_is_connecting(): that gate protects wifi_last_reason from being
 * overwritten AFTER this attempt's terminal transition, right up until the
 * NEXT mark_connecting() reopens it (and clears the live static back to 0 --
 * see mark_connecting()'s own comment).  Freezing here is what lets a host
 * still read THIS attempt's reason correctly even after that next
 * mark_connecting() has already cleared the live copy for the new attempt.
 *
 * SCOPE: this byte covers the CONNECT ATTEMPT only -- the reason or status
 * that ENDED or REJECTED that attempt (a DISCONNECT/REJECTED event that
 * TERMINATES it, i.e. arrives while state is still CONNECTING).  Once a
 * connect reaches CONNECTED, wifi_conn_set() has already frozen this field for
 * that attempt and nothing calls it again for THAT association: a spontaneous,
 * AP-initiated deauth arriving AFTER a successful CONNECTED updates the LIVE
 * wifi_last_reason (wifi_event_cb() does not know or care whether anything is
 * still waiting on it), but there is no listener left to freeze that update
 * into g_wifi_conn.reason or to transition g_wifi_conn.state out of CONNECTED
 * -- this firmware has no background watcher for a post-connect deauth today,
 * on this byte or on state/fail_reason either.  That is a pre-existing gap in
 * THIS firmware's async-latch design generally, not specific to this byte, and
 * post-connect tracking is deliberately out of scope here. */
static volatile struct {
	uint8_t state;       /* alp_cc3501e_wifi_conn_state_t   */
	uint8_t fail_reason; /* alp_cc3501e_wifi_fail_t          */
	int8_t  rssi;        /* NEVER POPULATED -- always 0      */
	int16_t reason;      /* frozen copy of wifi_last_reason at the terminal transition */
} g_wifi_conn = { (uint8_t)ALP_CC3501E_WIFI_DISCONNECTED,
	              (uint8_t)ALP_CC3501E_WIFI_FAIL_NONE,
	              0,
	              0 };

#ifdef CC3501E_WIFI
/* See the forward declaration + full contract comment above (before
 * wifi_event_cb()).  Guarded, unlike g_wifi_conn itself: wifi_event_cb(), the
 * only caller, is CC3501E_WIFI-only too, so an unguarded definition here is
 * simply unused (and warns as such) in the non-Wi-Fi ti build. */
static bool wifi_conn_is_connecting(void)
{
	return g_wifi_conn.state == (uint8_t)ALP_CC3501E_WIFI_CONNECTING;
}
#endif /* CC3501E_WIFI */

void cc3501e_hw_wifi_mark_connecting(void)
{
	g_wifi_conn.fail_reason = (uint8_t)ALP_CC3501E_WIFI_FAIL_NONE;
	g_wifi_conn.rssi        = 0;
	/* Clear the LIVE reason latch too, not just the frozen copy below: without
	 * this a new attempt that ends in TIMEOUT/KICK (no DISCONNECT/REJECTED event
	 * of its own) would have wifi_conn_set() freeze WHATEVER the previous
	 * attempt's event left in wifi_last_reason, misreporting a stale reason
	 * against this attempt.  The published byte must read 0 unless THIS attempt
	 * actually recorded one. */
	wifi_last_reason   = 0;
	g_wifi_conn.reason = 0;
	g_wifi_conn.state  = (uint8_t)ALP_CC3501E_WIFI_CONNECTING; /* publish state last */
}

int cc3501e_hw_wifi_conn_status(uint8_t *state, uint8_t *fail_reason, int8_t *rssi_dbm)
{
	if (state != 0) *state = g_wifi_conn.state;
	if (fail_reason != 0) *fail_reason = g_wifi_conn.fail_reason;
	if (rssi_dbm != 0) *rssi_dbm = g_wifi_conn.rssi;
	return CC3501E_HW_OK;
}

/* See the contract on the declaration in hal/cc3501e_hw.h.  ONE unconditional
 * definition (unlike cc3501e_hw_wifi_last_event_id()'s two, which mirror
 * wifi_started/wifi_sta_role_up's own #ifdef split): g_wifi_conn.reason is
 * itself unconditional storage (this struct, above), and the only function
 * that ever writes it -- wifi_conn_set(), from the FROZEN copy of
 * wifi_last_reason described on g_wifi_conn's comment -- is CC3501E_WIFI-only.
 * The non-Wi-Fi ti build has no wifi_conn_set() at all to write a nonzero
 * value, so this naturally reads its zero-init 0 there without a separate
 * hardcoded stub. */
int16_t cc3501e_hw_wifi_last_reason(void)
{
	return g_wifi_conn.reason;
}

/* --------------------------------------------------------------- */
/* Wi-Fi (v0.2) -- real CC35xx SimpleLink host integration.          */
/*                                                                   */
/* Under CC3501E_WIFI these route to the CC35xx Wi-Fi host (Wlan_* /  */
/* network_*), each lazy-starting the STA stack first and using the   */
/* osi_SyncObj rendezvous for the async (Scan / Connect) completions. */
/* The stub / silicon-free host build (#else) keeps every op NOTIMPL  */
/* (-> RESP_ERR_NOT_READY) so the native_sim suite stays unchanged.   */
/*                                                                   */
/* Map the host wire security byte (alp_cc3501e_wifi_connect_t.security:*/
/* 0=open, 1=WPA2-PSK, 2=WPA3-SAE) onto the SDK WLAN_SEC_TYPE_* enum. */
#ifdef CC3501E_WIFI
static char cc3501e_wifi_sec(uint8_t security)
{
	switch (security) {
	case 0u:
		return (char)WLAN_SEC_TYPE_OPEN;
	case 2u:
		/* WPA2_WPA3(16) = transition mode (WPA2 / WPA2+PMF / WPA3), NOT WPA3-only(12).
		 * Our scan labels both a pure-WPA3 and a WPA2/WPA3-transition AP as "wpa3", so
		 * the transition type is the robust choice -- it associates to either. */
		return (char)WLAN_SEC_TYPE_WPA2_WPA3;
	case 1u:
	default:
		return (char)WLAN_SEC_TYPE_WPA_WPA2; /* WPA/WPA2-PSK */
	}
}

/* Bring the STA role up once (after Wlan_Start), with a bounded timeout.  Needed
 * by SCAN + CONNECT (not GET_MAC).  Wlan_RoleUp returns the role id (>=0) on
 * success, <0 on error/timeout. */
static int cc3501e_hw_wifi_ensure_sta_role(void)
{
	if (wifi_sta_role_up) {
		return CC3501E_HW_OK;
	}
	const int wifi_rv = cc3501e_hw_wifi_lazy_start();
	if (wifi_rv != CC3501E_HW_OK) {
		return wifi_rv;
	}
	/* Program the STA PHY band BEFORE RoleUp.  THE missing step (root cause of the
	 * 0-AP scan): WLAN_SET_STA_WIFI_BAND is the ONLY call that runs
	 * l2_StorePhyConfig() to set the band the radio RX uses + inits the scan-DB
	 * wifi_band_cfg (static init leaves it 0 -> the survey captured 0 frames).
	 * BOTH TI references do this on every STA role-up (network_terminal
	 * wlan_cmd.c:676, at_commands atcmd_wlan.c:805); ours omitted it. */
	uint8_t sta_wifi_band = (uint8_t)
	    BAND_SEL_ONLY_2_4GHZ; /* our antenna/AP are 2.4 GHz; BOTH made the kick fail (5G) */
	(void)Wlan_Set(WLAN_SET_STA_WIFI_BAND, &sta_wifi_band);
	/* The STA netif is registered ONCE at boot (cc3501e_hw_net_init ->
	 * network_stack_add_if_sta) -- it must NOT be done here: from this radio-path
	 * context the busy-poll bridge task starves the lwIP tcpip thread and
	 * LOCK_TCPIP_CORE deadlocks the worker.  Wlan_RoleUp(STA) binds the already-
	 * registered netif, which is what the connect EAPOL/SAE handshake flows over. */
	RoleUpStaCmd_t staParams = { 0 };
	if (Wlan_RoleUp(WLAN_ROLE_STA, &staParams, CC3501E_WIFI_ROLE_TIMEOUT_MS) < 0) {
		return CC3501E_HW_ERR_IO;
	}
	wifi_sta_role_up = true;
	/* Apply the radio power policy SYNCHRONOUSLY, right here, before returning --
	 * NOT just latched for cc3501e_hw_tick()'s later drain.  That used to be the
	 * whole story (cc3501e_hw_power_reapply_radio() used to run here too, only
	 * setting a dirty flag for the tick to consume), and it is an ordering bug
	 * on the CONNECT-FIRST path:
	 *
	 * cc3501e_hw_wifi_connect_sta() calls THIS function and then, in the SAME
	 * worker-job body -- no return to main()'s bringup loop in between -- goes
	 * straight into Wlan_Connect and its association/EAPOL-SAE handshake and the
	 * bounded DHCP-lease poll.  cc3501e_hw_tick() (which drains the service that
	 * used to apply this) only runs AFTER worker_run_pending() returns, i.e. AFTER
	 * that whole body already ran under whatever power-save mode the radio
	 * actually held at that point.
	 *
	 * HYPOTHESIS, NOT RE-VERIFIED SINCE THIS FIX: `wifi scan` first (same
	 * ensure_sta_role(), but the scan job's tick has a chance to apply ACTIVE
	 * before a LATER connect job runs) associated 16/16 pre-fix; a bare
	 * connect-first `wifi connect` failed roughly half the time -- `fail: 2`
	 * (REJECTED) after WLAN_EVENT_DISCONNECT, or a DHCP timeout.  That was
	 * attributed to Wlan_Start leaving the radio in AUTO_PS (vendor
	 * wlan_if.c:867's devicePowerSaveMode default) through the connect body.
	 * The vendor source does not actually support that specific mechanism: on
	 * SDK 10.10.01.08, Wlan_RoleUp's own PS-mode push to firmware is commented
	 * out at both STA and AP role-up (driver_cc35xx.c ~1026 and ~1314:
	 * `//cc3xxx_cmd_set_ps_mode(...)`), so role-up itself never sends
	 * anything -- wlan_if.c:867 only sets the HOST-SIDE cached default, and the
	 * ONLY path that ever reaches firmware is an explicit Wlan_Set(POWER_SAVE)
	 * with a STA role already up, via CME_MESSAGE_ID_PS_SET (cme.c
	 * ~3020-3037).  So what the radio actually ran under, pre-fix, was
	 * whatever the LAST successful Wlan_Set(POWER_SAVE) sent -- cold-boot
	 * default or a previous attempt's leftover -- not specifically "AUTO_PS
	 * from Wlan_Start".  The fix below is still correct regardless (it is the
	 * first Wlan_Set(POWER_SAVE) that can reach firmware once THIS role is up,
	 * landing before Wlan_Connect instead of after), but the fail-mode
	 * attribution above is bench observation, not a mechanism proven from the
	 * SDK, and has not been re-run since this fix landed.
	 *
	 * cc3501e_hw_power_apply_radio_now() (cc3501e_hw_ti_power.c) shares the exact
	 * effective-policy rule cc3501e_hw_power_service() uses -- one function, not a
	 * second copy of the rule -- and is TASK CONTEXT ONLY, same constraint as
	 * pp_apply_core (#1683): Wlan_Set() blocks and must never run off the SPI-
	 * dispatch ISR.  Safe here: every caller of THIS function --
	 * cc3501e_hw_wifi_scan_run() (WIFI_SCAN_START), cc3501e_hw_wifi_connect_sta()
	 * (WIFI_CONNECT_STA), and the currently-uncalled cc3501e_hw_wifi_boot_start()
	 * -- is worker-routed off the ISR: src/worker.c's header comment states the
	 * ISR only SUBMITS a job and POLLS its cached result, while the blocking HAL
	 * body (this function included) runs in worker_run_pending(), invoked from
	 * main()'s bringup-task loop, not from SPI dispatch.
	 *
	 * Deliberately NOT also calling cc3501e_hw_power_reapply_radio() here any
	 * more: it is redundant with the synchronous apply just above (any host
	 * CMD_POWER_POLICY already sets pp_radio_dirty itself, in
	 * cc3501e_hw_set_power_policy(), cc3501e_hw_ti_power.c) and it used to be
	 * actively HARMFUL on the failed-connect path.  A failed connect's cleanup
	 * (wifi_clear_stale_assoc(), #1437) issues its own Wlan_Disconnect(), which
	 * leaves WLAN_IF_DISCONNECT_IN_PROGRESS set (vendor wlan_if.c ~2334) until
	 * that disconnect's own event dispatches.  A stray reapply here re-arms
	 * pp_radio_dirty, and if cc3501e_hw_tick()'s drain then lands inside that
	 * window, WLAN_SET_POWER_MANAGEMENT can come back WLAN_RET_OPER_IN_PROGRESS
	 * (wlan_if.c ~586-608's set_cond_in_process_wlan_set() gate) -- a real
	 * Wlan_Set() failure this HAL then reported as pp_radio_ok = false for a
	 * policy nothing was actually wrong with. */
	cc3501e_hw_power_apply_radio_now();
	return CC3501E_HW_OK;
}

/* Shared scan core: lazy-start the radio, kick a STA scan of both bands, then
 * wait the single WLAN_EVENT_SCAN_RESULT (whole list in one event) so the
 * result list is cached (wifi_scan_cache[] / wifi_scan_count) for the caller
 * to read/pack.  WLAN_MAX_SCAN_COUNT(20) records cached; MAX_PAYLOAD=512 caps
 * the eventual wire pack to ~17 records (noted upstream).  Returns CC3501E_HW_*. */
static int cc3501e_hw_wifi_scan_run(void)
{
	/* Wlan_Scan runs after just Wlan_Start (no RoleUp) -- the scan completes + the
	 * WLAN_EVENT_SCAN_RESULT fires either way; adding a 10s RoleUp here only
	 * blocked the worker + disrupted the bridge (scan timed out, bench-proven). */
	const int wifi_rv = cc3501e_hw_wifi_lazy_start();
	if (wifi_rv != CC3501E_HW_OK) {
		return wifi_rv;
	}
	/* The STA role is pre-cached at boot (cc3501e_hw_wifi_boot_start), so this
	 * ensure_sta_role returns INSTANTLY with no RoleUp radio op.  The scan is then a
	 * LIGHT Wlan_Scan -- like GET_MAC's Wlan_Get it kicks WITHOUT a bridge suspend.
	 * No suspend == no SPI_close/open churn (the suspend's close/open is what wedged
	 * the link past re-sync; GET_MAC, reinit-only, never churns).  reinit
	 * AFTER recovers the slave's DMA that Wlan_Scan tore down. */
	const int role_rv = cc3501e_hw_wifi_ensure_sta_role();

	scanCommon_t sc = { 0 };
	sc.Band         = BAND_SEL_ONLY_2_4GHZ; /* 2.4 GHz antenna; matches the role-up band-set */
	wifi_scan_count = 0u;
	osi_SyncObjClear(&wifi_event_sync);
	const uint32_t cb_before = wifi_cb_event_count; /* DIAG: did ANY wifi event fire? */
	int            scan_rv   = -1;
	if (role_rv == CC3501E_HW_OK) {
		scan_rv = Wlan_Scan(WLAN_ROLE_STA, &sc, (unsigned char)WLAN_MAX_SCAN_COUNT);
	}
	int wait_rv = OSI_OK;
	if (scan_rv == 0) {
		/* 6s (was 20s) so RoleUp(<=10s)+wait fits the host's 30s poll budget --
		 * the host then reads the REAL final code instead of a -4 timeout mask. */
		wait_rv = osi_SyncObjWait(&wifi_event_sync, 6u * OSI_WAIT_FOR_SECOND);
	}
	bridge_transport_spi_hw_reinit(); /* recover the bridge slave after the radio ops */

	if (role_rv != CC3501E_HW_OK) {
		return CC3501E_HW_ERR_INVAL; /* STA RoleUp failed -> host ALP_ERR_INVAL (-1) */
	}
	if (scan_rv != 0) {
		return CC3501E_HW_ERR_INVAL; /* Wlan_Scan kick failed -> host ALP_ERR_INVAL (-1) */
	}
	if (wait_rv != OSI_OK) {
		/* No WLAN_EVENT_SCAN_RESULT signalled.  DIAG split via the cb event count:
		 *   NO event at all   -> NOTIMPL -> host ALP_ERR_NOT_READY (-2): the NWP/scan
		 *                        engine produced nothing (scan stall / RF).
		 *   other events only -> IO      -> host ALP_ERR_IO (-5): events fired but the
		 *                        cb never saw SCAN_RESULT (a cb/event-routing gap). */
		if (wifi_cb_event_count == cb_before) {
			return CC3501E_HW_ERR_NOTIMPL;
		}
		return CC3501E_HW_ERR_IO;
	}
	return CC3501E_HW_OK;
}

/* WIFI_SCAN_START: run a scan + cache the result list.  Kept for the
 * scan-then-stop control surface; cc3501e_hw_wifi_scan() (below) runs the
 * SAME core and additionally packs the cached list onto the wire. */
int cc3501e_hw_wifi_scan_start(void)
{
	return cc3501e_hw_wifi_scan_run();
}

/* cc3501e_hw_wifi_scan: run the scan (shared core), then PACK the cached AP
 * list into @p buf in the host's wire format -- per record:
 *   bssid[6] | rssi(1) | channel(1) | security(1) | ssid_len(1) then ssid[ssid_len]
 * (the cc3501e_wifi_scan parser's CC3501E_SCAN_REC_HDR=10 layout).  Security and
 * channel are passed through RAW (no translation); rssi is the SDK's signed beacon
 * RSSI.  Records are packed until @p cap would be exceeded; *out_len = total bytes. */
int cc3501e_hw_wifi_scan(uint8_t *buf, size_t cap, size_t *out_len)
{
	if (buf == 0 || out_len == 0) {
		return CC3501E_HW_ERR_INVAL;
	}
	*out_len = 0u;

	const int rv = cc3501e_hw_wifi_scan_run();
	if (rv != CC3501E_HW_OK) {
		return rv;
	}

	const uint32_t count = wifi_scan_count;
	size_t         off   = 0u;
	for (uint32_t i = 0u; i < count; i++) {
		const WlanNetworkEntry_t *e        = &wifi_scan_cache[i];
		uint8_t                   ssid_len = (uint8_t)e->SsidLen;
		if (ssid_len > sizeof(e->Ssid)) {
			ssid_len = (uint8_t)sizeof(e->Ssid);
		}
		const size_t rec = 11u + (size_t)ssid_len; /* CC3501E_SCAN_REC_HDR + SSID */
		if (off + rec > cap) {
			break; /* stop before overflowing the wire buffer */
		}
		for (uint32_t b = 0u; b < 6u; b++) {
			buf[off + b] = (uint8_t)e->Bssid[b];
		}
		buf[off + 6u] = (uint8_t)e->Rssi;    /* signed beacon RSSI, raw */
		buf[off + 7u] = (uint8_t)e->Channel; /* raw */
		/* Raw 16-bit SecurityInfo, little-endian.  The host needs BOTH bytes:
		 * the sec-type that distinguishes open / WPA2 / WPA3 lives in the HIGH
		 * byte (WLAN_SCAN_RESULT_SEC_TYPE_BITMAP = (SecurityInfo >> 8) & 0x3f) --
		 * the old 1-byte pack truncated to the low byte (group cipher) so the
		 * host could only ever print "?". */
		buf[off + 8u]  = (uint8_t)(e->SecurityInfo & 0xFFu);
		buf[off + 9u]  = (uint8_t)((e->SecurityInfo >> 8) & 0xFFu);
		buf[off + 10u] = ssid_len;
		for (uint32_t s = 0u; s < ssid_len; s++) {
			buf[off + 11u + s] = (uint8_t)e->Ssid[s];
		}
		off += rec;
	}
	*out_len = off;
	return CC3501E_HW_OK;
}

int cc3501e_hw_wifi_scan_stop(void)
{
	/* The CC35xx scan is a one-shot that self-completes via the event; there
	 * is no standing scan to cancel.  Treat stop as a successful no-op so the
	 * host's stop after a completed scan does not error. */
	return CC3501E_HW_OK;
}

/* Publish a terminal connect outcome to the status latch (detail first, state last
 * -- a reader that observes the terminal state also observes the matching detail).
 *
 * rssi is always set to 0: this NWP cannot supply a measurement on the connect
 * path (the hazard note in cc3501e_hw_wifi_connect_sta).  Issue #1387: the host
 * must not report the latched byte as a signal level; WIFI_GET_RSSI is the only
 * real read.
 *
 * ALSO enqueue the matching async EVT_* so a host that registered an event
 * callback (via CMD_GET_PENDING_EVENTS polling) is notified: CONNECTED ->
 * EVT_WIFI_CONNECTED, a terminal FAILED/DISCONNECTED -> EVT_WIFI_DISCONNECTED.
 * Both carry no payload -- the host reads the detail (rssi / fail_reason) via
 * CMD_WIFI_STATUS.  wifi_conn_set is the single terminal-transition chokepoint
 * (mark_connecting writes the CONNECTING latch directly and is NOT terminal), so
 * exactly one event is queued per terminal outcome. */
static void wifi_conn_set(uint8_t state, uint8_t fail_reason)
{
	g_wifi_conn.fail_reason = fail_reason;
	g_wifi_conn.rssi        = 0;
	/* Freeze wifi_last_reason HERE, at the terminal transition -- see
	 * g_wifi_conn's comment above (`reason` field) for why a live mirror is
	 * wrong: every caller of this function on the FAILED paths runs
	 * wifi_clear_stale_assoc() (#1437) right after it returns, which issues its
	 * own Wlan_Disconnect() and, asynchronously, could otherwise still move
	 * wifi_last_reason again before a host ever reads it.
	 *
	 * CONNECTED is the one state that ALWAYS freezes 0, never the live
	 * wifi_last_reason -- even if a since-succeeded retry left a transient
	 * rejection status (e.g. 30, WLAN_STATUS_ASSOC_REJECTED_TEMPORARILY) sitting
	 * in it.  hal/cc3501e_hw.h's contract for this byte is "the reason or status
	 * that ENDED or REJECTED that attempt": a CONNECTED attempt was neither, so
	 * publishing a stale rejection alongside a successful CONNECTED would
	 * contradict that contract and mislead a host into reading a live
	 * association as somehow still carrying a past reject.
	 *
	 * CONNECTED also clears the LIVE wifi_last_reason itself, not just the
	 * frozen g_wifi_conn.reason above -- otherwise the two disagree from this
	 * point on: a later cc3501e_hw_wifi_disconnect() call publishes
	 * DISCONNECTED by copying the (still-stale) live value
	 * (wifi_last_reason), which would re-surface that same old rejection
	 * status as if it were the reason THIS now-clean disconnect ended,
	 * exactly the kind of stale/self-inflicted mislabeling the rest of this
	 * cb already guards against.  Clearing it here keeps "0 unless THIS
	 * attempt/session actually records one of its own" true continuously,
	 * not just at this one instant. */
	if (state == (uint8_t)ALP_CC3501E_WIFI_CONNECTED) {
		wifi_last_reason   = 0;
		g_wifi_conn.reason = 0;
	} else {
		g_wifi_conn.reason = wifi_last_reason;
	}
	g_wifi_conn.state = state;

	if (state == (uint8_t)ALP_CC3501E_WIFI_CONNECTED) {
		(void)event_ring_push((uint8_t)ALP_CC3501E_EVT_WIFI_CONNECTED, NULL, 0u);
	} else if (state == (uint8_t)ALP_CC3501E_WIFI_CONN_FAILED ||
	           state == (uint8_t)ALP_CC3501E_WIFI_DISCONNECTED) {
		(void)event_ring_push((uint8_t)ALP_CC3501E_EVT_WIFI_DISCONNECTED, NULL, 0u);
	}
}

/* Clear the NWP's stale association after a FAILED connect (#1437).
 *
 * A failed Wlan_Connect leaves association state behind, so the NEXT connect --
 * with a correct SSID and passphrase -- fails at the role-up kick with
 * ALP_CC3501E_WIFI_FAIL_KICK.  Bench-confirmed on E1M-AEN801 r1, reproduced 2/2
 * (#1435).  That was worked around host-side in #1436, but the invariant "a failed
 * connect leaves the NWP ready for the next connect" is firmware-internal state
 * hygiene and belongs here, so every host does not have to know.
 *
 * Deliberately NOT cc3501e_hw_wifi_disconnect(): that mirrors DISCONNECTED /
 * FAIL_NONE into the status latch, which would erase the very fail_reason the host
 * is about to read.
 *
 * CALLERS NOW INVOKE THIS *AFTER* THEIR wifi_conn_set(), not before.  The old
 * order was justified by "the failure latch is written last and wins", but that
 * reasoning belongs to cc3501e_hw_wifi_disconnect(), which writes the latch.
 * THIS helper does not touch it, so nothing is won by going first -- and going
 * first costs the host its verdict.
 *
 * IT DOES NOT HANG.  Settled by reading the SDK, not inferred: Wlan_Disconnect's
 * STA branch is `ret = CME_WlanDisconnect(TRUE)`, and CME_WlanDisconnect ends in
 * `pushMsg2Queue(&msg); return status;` -- it dispatches a message and returns,
 * with no blocking wait
 * (SDK source/ti/net/wifi_stack/app_entry/wlan_if.c and .../cme/cme.c).  The
 * bench agrees: on the scan-first ordering a failed association is followed by
 * BLE_ENABLE, BLE_SCAN and BLE_DISABLE all succeeding, 2 of 2, and those are
 * worker-routed so they can only drain after this body returned.  An earlier
 * version of this comment asserted the hang as the campaign's central symptom.
 * It was wrong.
 *
 * WHAT THE SDK DOES SHOW IS WORSE, AND IT IS THE #1437 MECHANISM.
 * Wlan_Disconnect first calls set_cond_in_process_wlan_discconnect(1), which
 * sets WLAN_IF_DISCONNECT_IN_PROGRESS in the SDK's g_oper_bitmap.  On the STA
 * branch it then returns WITHOUT calling set_finish_wlan_disconnect() -- only
 * the AP branch and the failure label clear it inline.  The bit is otherwise
 * cleared in exactly one place: the WLAN_EVENT_DISCONNECT handler.
 *
 * And Wlan_Connect gates on that same bitmap.  It calls
 * set_cond_in_process_wlan_connect() -> is_wlan_oper_in_progress(), whose
 * allow-mask for CONNECT is ROLE_UP_AP | ROLE_DOWN_AP | SET | GET | GET_EXCLUDE
 * | SET_EXCLUDE.  WLAN_IF_DISCONNECT_IN_PROGRESS is NOT in it.  So while that
 * bit is set, every Wlan_Connect returns RET_OPER_IN_PROGRESS immediately --
 * which this file maps to ALP_CC3501E_WIFI_FAIL_KICK.
 *
 * That is EXACTLY the #1437 symptom this helper exists to prevent: "the next
 * connect, with a correct SSID and passphrase, fails at the role-up kick with
 * FAIL_KICK".  So the helper may be causing the bug it was added to cure.
 *
 * NOT PROVEN, and the gap is specific: whether the supplicant emits
 * WLAN_EVENT_DISCONNECT when asked to disconnect a STA that never associated.
 * If it does, the bit clears and this is a non-issue.  If it does not, the bit
 * sticks for the rest of the session and every later connect dies at the kick.
 * That is one bench observation away -- a failed connect, then a second connect,
 * and read fail_reason -- and it is worth taking before changing this helper.
 *
 * Ordering the latch first is therefore cheap insurance, not the cure: it costs
 * nothing, and it means a hang ANYWHERE after it -- this call, the drain's own
 * reinit, or something else -- still leaves the host holding a verdict.
 * FAIL_KICK vs FAIL_TIMEOUT vs FAIL_REJECTED is the difference between a wrong
 * passphrase, a marginal signal, and a firmware defect.  Bounding or deferring
 * the call is deliberately NOT done here: the tree requires a busy/reinit/ready
 * bracket around a disconnect (see src/worker.c and src/protocol_wifi.c), a
 * bare tick-deferred flag would drop that, and deferring bounds nothing anyway
 * since the tick runs on the same task.
 *
 * Best-effort: the association may already be gone, so a non-zero return is not
 * itself a failure -- the caller's own error is what gets reported. */
static void wifi_clear_stale_assoc(void)
{
	if (wifi_started) {
		/* No "we asked for this" flag to arm here any more -- see
		 * wifi_event_cb()'s DISCONNECT case for why that approach was
		 * dropped.  This call always runs AFTER the caller's own
		 * wifi_conn_set(FAILED, ...), so the connecting-state gate that
		 * replaced the flag is already closed by the time this fires: see
		 * that case's RESIDUAL note for the one narrow situation (this call's
		 * own delayed DISCONNECT event landing in the tiny window between a
		 * NEW attempt's second wifi_last_reason reset -- in
		 * cc3501e_hw_wifi_connect_sta(), right before its own Wlan_Connect --
		 * and the vendor actually starting to process that new connect) where
		 * that gate can still record THIS disconnect's reason against the
		 * wrong attempt. */
		(void)Wlan_Disconnect(WLAN_ROLE_STA, NULL);
	}
}

/* STA L3 bring-up: bounded DHCP-lease poll after the L2 connect event.
 * CC3501E_STA_DHCP_TRIES * CC3501E_STA_DHCP_POLL_US = 100 * 200 ms = 20 s budget
 * (the worker drain sleeps between tries so the tcpip thread runs DHCP).
 *
 * 20 s, not the 10 s this shipped with, and the four-second difference is the
 * whole point.  lwIP retransmits DISCOVER on a doubling backoff: dhcp_discover()
 * increments dhcp->tries AFTER the send and then arms
 *
 *     msecs = (u16_t)((dhcp->tries < 6 ? 1 << dhcp->tries : 60) * 1000);
 *
 * (third_party/lwip/lwip-stack/src/core/ipv4/dhcp.c in the vendor SDK), so the
 * sends land at t = 0, 2, 6, 14, 30, 62 s.  A 10 s budget therefore emits
 * exactly THREE DISCOVERs -- the last at t=6 -- and then gives up at t=10, four
 * seconds of dead air before the fourth would have gone out.  That is the worst
 * available place to stop: inside the longest gap so far, having spent only the
 * three shortest retries.
 *
 * The association SUCCEEDS, which is what makes the lease the interesting half.
 * Bench-measured on an E1M-AEN801 with the on-board antenna: a post-fail
 * WIFI_GET_RSSI reads -75 dBm with status 0, which a radio that never associated
 * cannot produce, and the scan reports the same -75 dBm for the same AP.  The
 * connect failures returned at 14.45 s and 16.87 s -- consistent with
 * associate-then-10s-DHCP, and NOT with the 30 s association wait above, which
 * never expired.
 *
 * THIS CHANGE IS NOT A CURE, but do not read it as a no-op either -- an earlier
 * version of this comment over-retracted it, and the correction is worth having.
 *
 * MEASURED 2026-09-12 on the image that reports lwIP's own DHCP state: the
 * failure is INTERMITTENT, not absolute.  Across four runs at -78 dBm the
 * station leased once (192.168.1.194, followed by a full TCP round trip to
 * 192.168.1.1:80) and failed to lease on the others, and in a failing run the
 * diagnostic read DHCP_STATE_SELECTING with tries = 5 -- DISCOVERs leaving,
 * mostly unanswered.
 *
 * With a per-attempt success probability below one, attempts are what buy you a
 * lease, and this budget is what decides how many happen INSIDE the connect
 * call: three at 10 s (t = 0, 2, 6), four at 20 s (adding t = 14).  So the
 * change genuinely improves the odds of the connect itself succeeding.  What it
 * cannot do is make an unanswered DISCOVER answered, which is why the cause
 * still sits below this function.
 *
 * The evidence that first prompted the retraction still stands and still
 * matters.  After the failed connect the host polled
 * WIFI_GET_IP once a second for 30 s and got "no address" every single time --
 * and those answers are trustworthy rather than a dead link, because
 * GET_DIAG_INFO, BLE_ENABLE, BLE_SCAN, BLE_DISABLE and a proxied GPIO read all
 * succeeded AFTERWARDS on the same link.  Nothing here tears the association or
 * the netif down on the no-lease path (this function just latches and returns),
 * so lwIP's DHCP client kept running throughout.  Association plus the 10 s
 * in-body poll plus 30 s of host polling is roughly 40 s with no lease, which
 * spans the DISCOVERs at 0, 2, 6, 14 AND 30 s.  A fourth attempt would have
 * changed nothing.
 *
 * The budget is still wrong as written and still worth correcting: stopping four
 * seconds before a retransmit cannot be the right place to give up, whatever the
 * cause turns out to be.  Treat this as removing a confounder, not as the fix.
 *
 * 20 s covers the fourth DISCOVER at t=14 and leaves 6 s for OFFER/REQUEST/ACK.
 * Deliberately NOT 35 s to also cover the fifth at t=30, and the caller's budget
 * is why.  This image never calls cc3501e_hw_wifi_boot_start() (see src/main.c),
 * so a connect issued as the first radio op of a boot carries Wlan_Start, a
 * Wlan_Set and a 10 s Wlan_RoleUp INSIDE this body, before the 30 s association
 * wait -- prebuilt/CHANGELOG.md records that.  The association timeout and this
 * poll are mutually exclusive (a timed-out association returns above and never
 * reaches DHCP), so the deepest path that reaches here is
 * role-up + association + lease = 10 + 30 + 20 = 60 s against the 70000 ms the
 * bench apps pass.  At 35 s that same path is 75 s and the caller gives up
 * first, turning a fixable lease delay back into an unreadable host timeout.
 *
 * Measured, that path is nowhere near its worst case: the two failing runs took
 * 14.45 s and 16.87 s end to end, so role-up plus association cost roughly 4.5 s
 * and the new budget puts them at about 24.5 s.
 *
 * Where the real cause is NOT, as far as static reading can settle it: the
 * vendor netif plumbing.  cc3501e_hw_net_init() calls network_stack_add_if_sta()
 * at boot, _role_sta_up() installs both callbacks, network_set_up() does
 * netif_set_up then netif_set_link_up, status_callback() fills hwaddr from
 * Wlan_Get(WLAN_GET_MACADDRESS) (pMacAddress is a 6-byte array, so the memcpy is
 * not a truncated pointer), and link_callback() registers the receive path via
 * Wlan_EtherPacketRecvRegisterCallback() and then calls dhcp_start() because
 * sta_ip_mode initialises to IP_DHCP and isIpAcquired starts zero.  All of that
 * is in the network_terminal demo's network_lwip.c, which ti/build_ti.sh
 * compiles verbatim, and the LP_EM_CC35X1 and LP_EM_CC35X1ET copies are
 * byte-identical.
 *
 * So the next evidence is below lwIP or off-board: the NWP's STA data path, or
 * the AP declining to lease.
 *
 * It CANNOT come from the CC3501E's own console, and an earlier version of this
 * comment wrongly proposed that.  link_callback() does print "link_callback==UP
 * starting DHCP" and "DHCP is %d" through Report(), but ti/cc3501e_aen_wifi.syscfg
 * records the measurement that kills the idea: on the E1M-AEN801 the CC35 UART2
 * pins (GPIO5/GPIO6) are NOT routed to the debug probe.  A probe build opened
 * UART2_0, got a non-NULL handle and wrote 188988 bytes with
 * UART2_STATUS_SUCCESS while BOTH XDS110 COM ports received zero bytes across a
 * full cold boot.  The radio tracer pin (GPIO9) is unrouted as well, and an
 * activated part exposes no MEM-AP over SWD.  There is no console here.
 *
 * The decisive read therefore has to travel over the bridge link itself, and
 * lwIP already holds the answer: netif_dhcp_data(sta netif)->state separates the
 * two candidates outright.  DHCP_STATE_OFF means dhcp_start() never ran and the
 * fault is above the radio; SELECTING with ->tries climbing means DISCOVERs are
 * leaving and nothing is coming back, which points at the NWP data path or the
 * AP.  Reporting that state, ->tries, and netif->flags through GET_DIAG_INFO is
 * the smallest change that ends the guessing, and it needs no wiring that this
 * SoM does not have. */
#define CC3501E_STA_DHCP_TRIES 100u

/* How many times the lease poll may restart a stalled DHCP client.  Two keeps
 * the worst case bounded -- each restart costs at most the 2 s to its first
 * retransmit -- while giving a lossy link three independent runs at the
 * exchange inside one connect call. */
#define CC3501E_STA_DHCP_KICKS 2u

/* Soft-AP role-up defaults that a zero-init does NOT supply; see the block in
 * cc3501e_hw_wifi_ap_start() for why each one matters.  Values mirror TI's
 * ParseRoleUpApCmd() reference defaults for CC35xx. */
#ifndef CC3501E_AP_STA_LIMIT
/* TI clamps this to [1, 8] and defaults to 4.  Zero means no client may ever
 * associate, which is #1562. */
#define CC3501E_AP_STA_LIMIT 4u
#endif
#ifndef CC3501E_AP_SAE_PWE
/* SAE password element derivation; TI's reference default. */
#define CC3501E_AP_SAE_PWE 2u
#endif

#define CC3501E_STA_DHCP_POLL_US 200000u

int cc3501e_hw_wifi_connect_sta(const uint8_t *ssid,
                                uint8_t        ssid_len,
                                const uint8_t *psk,
                                uint8_t        psk_len,
                                uint8_t        security)
{
	if (ssid == 0 || ssid_len == 0u) {
		/* The latch was armed CONNECTING at submit (mark_connecting); a bad arg must
		 * still publish a TERMINAL outcome, else the latch stays stuck CONNECTING and
		 * the host's status poll never resolves (spins to a misleading timeout). */
		wifi_conn_set((uint8_t)ALP_CC3501E_WIFI_CONN_FAILED, (uint8_t)ALP_CC3501E_WIFI_FAIL_KICK);
		return CC3501E_HW_ERR_INVAL;
	}
	/* Did THIS call actually perform the role-up?  ensure_sta_role() returns at its
	 * wifi_sta_role_up early-return with no radio op at all when the role is already
	 * latched, and in that case the slave is live and callback-armed.  The reinit
	 * below must not run then -- see its comment. */
	const bool role_up_was_latched = wifi_sta_role_up;

	const int wifi_rv =
	    cc3501e_hw_wifi_ensure_sta_role(); /* lazy-start + bounded STA role-up (shared) */

	/* Recover the bridge slave between the role-up and Wlan_Connect, but ONLY when a
	 * role-up actually happened on this call.
	 *
	 * ensure_sta_role() runs Wlan_Set(STA_WIFI_BAND) + Wlan_RoleUp, and a radio op of
	 * that weight tears the slave's DMA down -- the same teardown lazy_start()
	 * reinits after Wlan_Start, and the same one cc3501e_hw_wifi_scan_run() reinits
	 * after its own role-up + Wlan_Scan.  This body had a reinit AFTER it, from the
	 * worker drain (src/worker.c, WIFI_CONNECT_STA is not on the exemption list), but
	 * none BETWEEN the role-up and Wlan_Connect: it went straight into a 30 s
	 * osi_SyncObjWait with the slave down.
	 *
	 * Bench-measured asymmetry, published v0.8.0.  CONNECT as the first radio op of a
	 * boot -- role-up inside this body -- times out AND every opcode after it fails
	 * until a cold cycle, so the drain's own reinit never runs either, which says the
	 * drain does not return.  SCAN first -- role-up inside scan_run(), behind its
	 * reinit -- still fails to associate but the link SURVIVES: BLE_ENABLE, BLE_SCAN,
	 * BLE_DISABLE and a proxied GPIO read all succeed after it, 2 of 2.
	 *
	 * So this reinit is NOT expected to fix the association.  What it should do is
	 * make the failure observable: a live slave answers PING and WIFI_STATUS off the
	 * ISR even while the worker is stuck, so the radio's own fail_reason becomes
	 * readable instead of timing out.  Judge it on that, not on association.
	 *
	 * GATED because reinit on a LIVE slave is its own documented hazard, not a no-op:
	 * transport_hw_ti_spi.c states its precondition is "ONLY right after a radio op",
	 * cc3501e_hw_ti_ble.c records a reinit on a no-work path taking over 120 s and
	 * repeatedly wedging the link, and src/worker.c calls that shape "the destructive
	 * no-op".  The scan-first ordering is the ONE station path that currently
	 * survives; leaving it byte-identical is deliberate.
	 *
	 * Reinit ONLY -- deliberately no bridge_transport_spi_hw_suspend() around the
	 * role-up.  That was tried, flashed, and wedges the link outright: suspend() calls
	 * SPI_transferCancel(), which this tree records three times as not returning
	 * against an armed callback transfer, and bracketing ensure_sta_role() made that
	 * call reachable in a shipped image for the first time.  See prebuilt/CHANGELOG.md.
	 * Reinit-only is the proven-safe half: it is what GET_MAC and the scan already do.
	 *
	 * The drain's post-body reinit is still required and stays: this one is BEFORE
	 * Wlan_Connect, the drain's is after the body returns. */
	if (!role_up_was_latched) {
		bridge_transport_spi_hw_reinit();
	}

	if (wifi_rv != CC3501E_HW_OK) {
		wifi_conn_set((uint8_t)ALP_CC3501E_WIFI_CONN_FAILED, (uint8_t)ALP_CC3501E_WIFI_FAIL_KICK);
		return wifi_rv;
	}
	osi_SyncObjClear(&wifi_event_sync);
	wifi_last_status = 0;
	/* mark_connecting() (SPI-ISR/protocol context, at submit) already cleared
	 * wifi_last_reason once for this attempt, but that was BEFORE the worker
	 * body reached this point -- everything from ensure_sta_role() through the
	 * reinit above runs in between, and a late event from the PREVIOUS attempt
	 * (still in flight on the host-driver thread) can land in that gap and get
	 * recorded there.  Two real shapes, not just one: a supplicant DISCONNECT
	 * carrying any real reason arriving after an AUTHENTICATION_REJECTED
	 * already closed out the old attempt, or a late ASSOCIATION_REJECTED /
	 * AUTHENTICATION_REJECTED arriving after the old attempt was already
	 * declared a TIMEOUT.  Clear it again HERE too, right next to
	 * wifi_last_status's own reset and for the same reason -- immediately
	 * before Wlan_Connect, as close to the new attempt's real start as this
	 * body gets. */
	wifi_last_reason = 0;
	/* Wlan_Connect(ssid,len,bssid=NULL,secType,pass,passlen,flags=0).  Open
	 * networks pass a NULL/zero-length password. */
	if (Wlan_Connect((const signed char *)ssid,
	                 (int)ssid_len,
	                 NULL,
	                 cc3501e_wifi_sec(security),
	                 (const char *)psk,
	                 (char)psk_len,
	                 0) != 0) {
		wifi_conn_set((uint8_t)ALP_CC3501E_WIFI_CONN_FAILED, (uint8_t)ALP_CC3501E_WIFI_FAIL_KICK);
		wifi_clear_stale_assoc(); /* #1437: leave the NWP ready for the next connect */
		return CC3501E_HW_ERR_IO;
	}
	/* BOUNDED wait for the connect event.  This op is WORKER-ROUTED (see protocol.c
	 * wifi_join -> worker), so the wait pends off the SPI ISR.
	 *
	 * NOTE this comment used to add "the READY/host-IRQ line is held BUSY for the
	 * duration ... so the host never clocks into the dead SPI-slave DMA".  That is no
	 * longer true on the first-op-is-CONNECT path: the gated reinit above re-arms the
	 * slave and raises READY before this wait, on purpose, so the host CAN clock
	 * WIFI_STATUS in and read the radio's verdict.  It is also not true on this bench
	 * regardless -- READY (CC35 GPIO17 -> Alif P2_6) is an open connection on the
	 * E1M-AEN801 unit, 0 edges in 20000 samples, so it holds nobody off.  The SPI
	 * framing itself is hardware SS0.  The L2 association completes on
	 * silicon.  WPA2 associates ~15s in, but WPA3-SAE is SLOWER (the extra SAE
	 * commit/confirm exchange + PMF), so a 15s wait raced the WLAN_EVENT_CONNECT and
	 * timed out on a WPA3 AP even though the association was in progress -- bench-seen
	 * on "Alp Electronix" (wpa3), 2026-07-05.  30s covers WPA2 and WPA3-SAE with margin
	 * (the bridge is BUSY for the wait, but a connect is a deliberate, infrequent op).
	 * The OUTCOME is mirrored into the status latch below; the host collects it
	 * NON-blocking via CMD_WIFI_STATUS.  DHCP/IP is brought up right after (L3). */
	if (osi_SyncObjWait(&wifi_event_sync, 30u * OSI_WAIT_FOR_SECOND) != OSI_OK) {
		/* No connect event within the wait -- TERMINAL timeout (was masked as a
		 * retryable IO that looped the host's poll-by-repeat -> -4). */
		wifi_conn_set((uint8_t)ALP_CC3501E_WIFI_CONN_FAILED,
		              (uint8_t)ALP_CC3501E_WIFI_FAIL_TIMEOUT);
		wifi_clear_stale_assoc(); /* #1437: leave the NWP ready for the next connect */
		return CC3501E_HW_ERR_IO;
	}
	if (wifi_last_status < 0) {
		/* FW rejected the association/auth (WLAN_EVENT_CONNECT Status<0, or a
		 * DISCONNECT/ASSOCIATION_REJECTED/AUTHENTICATION_REJECTED event) -- TERMINAL. */
		wifi_conn_set((uint8_t)ALP_CC3501E_WIFI_CONN_FAILED,
		              (uint8_t)ALP_CC3501E_WIFI_FAIL_REJECTED);
		wifi_clear_stale_assoc(); /* #1437: leave the NWP ready for the next connect */
		return CC3501E_HW_ERR_IO;
	}
	/* L2 ASSOCIATED.  Bring the STA netif UP at L3 + start DHCP, MIRRORING the AP path
	 * (cc3501e_hw_wifi_ap_start -> network_set_up(network_get_ap_if())).  Without this
	 * the netif stays link-down, lwIP has no route, and lwip_socket/connect fail -- the
	 * L3 gap the bench hit.  network_set_up() does netif_set_up + netif_set_link_up under
	 * LOCK_TCPIP_CORE; the STA netif's link_callback (registered at boot by
	 * network_stack_add_if_sta -> _role_sta_up) then runs dhcp_start FROM THE TCPIP
	 * CONTEXT (sta_ip_mode defaults to IP_DHCP, STATIC_IP undefined).  The old
	 * "network_set_up()/DHCP would hang the worker" caveat here PREDATED the busy-poll
	 * transport that starved the tcpip thread; the slave is now DMA-callback driven, so
	 * LOCK_TCPIP_CORE no longer deadlocks the drain (confirmed 2026-07-05).
	 *
	 * Do NOT read the RSSI here: a Wlan_Get(WLAN_GET_RSSI) immediately after associate
	 * BLOCKS on this NWP (the link is not yet settled for a beacon measurement) and hangs
	 * the worker body -- the host fetches it separately via WIFI_GET_RSSI once settled. */
	network_set_up(network_get_sta_if());

	/* GATE "CONNECTED" ON L3-UP: bounded, NON-blocking poll of the netif IP (a light read,
	 * no radio op) with a short task-sleep between tries so DHCP proceeds on the tcpip
	 * thread -- NOT a blocking semaphore wait in the drain.  Publishing CONNECTED only
	 * once a nonzero IP is leased means a host that observes CONNECTED can open sockets
	 * immediately (the netif has a route). */
	uint32_t ip = 0u, mask = 0u, gw = 0u, dhcp = 0u;
	unsigned kicks = 0u;
	for (unsigned i = 0u; i < CC3501E_STA_DHCP_TRIES; ++i) {
		if (network_stack_get_if_ip(WLAN_ROLE_STA, &ip, &mask, &gw, &dhcp) == 0 && ip != 0u) {
			break;
		}
		ip = 0u;

		/* RESTART A STALLED CLIENT rather than wait out its backoff.
		 *
		 * lwIP arms `msecs = (tries < 6 ? 1 << tries : 60) * 1000` in both
		 * dhcp_discover() and dhcp_select(), so the gap between attempts doubles:
		 * t = 0, 2, 6, 14, 30, 62 s.  By tries = 3 the client is transmitting for
		 * a few milliseconds and then idle for 8, then 16, then 32 seconds.  Most
		 * of this poll's budget is spent in that dead air.
		 *
		 * Bench-measured at -80 dBm on the image that reports lwIP's own state: a
		 * failing attempt sits in DHCP_STATE_REQUESTING with tries = 5 -- the
		 * OFFER arrived and the REQUEST went out, and the ACK did not come back.
		 * That is a lossy link, not a broken path, and the answer to a lossy link
		 * is more attempts per second, not a longer wait.
		 *
		 * Restarting resets tries to 0 and begins again at 2 s spacing.  Capped at
		 * CC3501E_STA_DHCP_KICKS so a genuinely absent server still terminates,
		 * and gated on tries >= 3 so a healthy exchange in progress is never
		 * interrupted -- a client that is about to be answered must be left alone.
		 *
		 * LOCK_TCPIP_CORE because dhcp_release_and_stop/dhcp_start are core
		 * functions; network_set_up() takes the same lock from this same worker
		 * context a few lines above, so this adds no new locking hazard. */
		if (kicks < CC3501E_STA_DHCP_KICKS) {
			struct netif *nif = (struct netif *)network_get_sta_if();
			struct dhcp  *d   = (nif != 0) ? netif_dhcp_data(nif) : 0;
			if (d != 0 && d->tries >= 3u) {
				LOCK_TCPIP_CORE();
				dhcp_release_and_stop(nif);
				(void)dhcp_start(nif);
				UNLOCK_TCPIP_CORE();
				++kicks;
			}
		}

		ClockP_usleep(CC3501E_STA_DHCP_POLL_US);
	}
	if (ip == 0u) {
		/* Associated at L2 but no DHCP lease within the budget -- TERMINAL (there is no
		 * usable IP, so a "connected" report would mislead the host into failing socket
		 * ops).  The host reads this as CONN_FAILED/TIMEOUT via CMD_WIFI_STATUS. */
		wifi_conn_set((uint8_t)ALP_CC3501E_WIFI_CONN_FAILED,
		              (uint8_t)ALP_CC3501E_WIFI_FAIL_TIMEOUT);
		return CC3501E_HW_ERR_IO;
	}
	wifi_conn_set((uint8_t)ALP_CC3501E_WIFI_CONNECTED, (uint8_t)ALP_CC3501E_WIFI_FAIL_NONE);
	return CC3501E_HW_OK;
}

int cc3501e_hw_wifi_disconnect(void)
{
	if (!wifi_started) {
		return CC3501E_HW_OK; /* nothing to disconnect */
	}
	/* No "we asked for this" flag to arm here any more -- see
	 * wifi_event_cb()'s DISCONNECT case for why.  Nothing to gate here either:
	 * a host WIFI_DISCONNECT is only ever issued while state is CONNECTED (the
	 * only state a host would sensibly send one from), so the connecting-state
	 * gate that replaced the flag is already closed the whole time this
	 * function runs -- the vendor's own async DISCONNECT event for THIS call
	 * cannot land inside a CONNECTING window it never opened. */
	if (Wlan_Disconnect(WLAN_ROLE_STA, NULL) != 0) {
		return CC3501E_HW_ERR_IO;
	}
	/* Host-requested teardown succeeded: mirror the state into the latch and
	 * queue an async EVT_WIFI_DISCONNECTED (wifi_conn_set does both). */
	wifi_conn_set((uint8_t)ALP_CC3501E_WIFI_DISCONNECTED, (uint8_t)ALP_CC3501E_WIFI_FAIL_NONE);
	return CC3501E_HW_OK;
}

int cc3501e_hw_wifi_ap_start(const uint8_t *ssid,
                             uint8_t        ssid_len,
                             const uint8_t *psk,
                             uint8_t        psk_len,
                             uint8_t        security)
{
	if (ssid == 0 || ssid_len == 0u || ssid_len >= WLAN_SSID_MAX_LENGTH) {
		return CC3501E_HW_ERR_INVAL;
	}
	/* The stack must be started (Wlan_Start) before any RoleUp; lazy-start
	 * brings it up as STA first, then we role-up the AP interface. */
	const int wifi_rv = cc3501e_hw_wifi_lazy_start();
	if (wifi_rv != CC3501E_HW_OK) {
		return wifi_rv;
	}
	network_stack_add_if_ap();

	/* RoleUpApCmd_t.ssid is NUL-terminated in the SDK; copy + terminate. */
	uint8_t ssid_buf[WLAN_SSID_MAX_LENGTH];
	memcpy(ssid_buf, ssid, ssid_len);
	ssid_buf[ssid_len] = 0u;

	/* Force ALWAYS-ACTIVE power mode BEFORE the role-up.  Wlan_Start leaves the
	 * NWP in ELP (root-caused against the SDK 2026-06-22 -- see the identical
	 * call in cc3501e_hw_ble_enable, which needs it because BLE-controller init
	 * never completes from ELP).  Until now that was the ONLY
	 * WLAN_SET_POWER_MANAGEMENT call in this HAL, so an AP started without BLE
	 * ran with whatever mode Wlan_Start left behind.
	 *
	 * A soft-AP has to beacon continuously; it is the one role that cannot
	 * tolerate the NWP dozing.  This is the leading hypothesis for #1562, where
	 * the AP starts, advertises for ~100 s and then stops with ap_start long
	 * since returned -- the instrumented run pinned the failure DOWNSTREAM of
	 * Wlan_RoleUp (stage saturated at 7), which is where a power-mode death
	 * lives and where the stage counter could no longer see.
	 *
	 * Deliberately scoped to the AP path, NOT lazy_start: always-active raises
	 * idle draw for every radio user, and scan/connect are demonstrably fine
	 * from ELP (scan returned 5 APs during the #1562 bench runs). Do not widen
	 * this without measuring the power cost.
	 *
	 * Best-effort by design (return ignored, as in the BLE path): if the NWP
	 * rejects the set, the AP should still come up -- just possibly with the
	 * #1562 lifetime. A hard failure here would turn a degraded AP into no AP.
	 *
	 * AUDITED for the STA path's connect-first ordering bug (this file's
	 * cc3501e_hw_wifi_ensure_sta_role()): AP does NOT have it.  Unlike STA, the
	 * AP role-up path never went through the latch/drain
	 * (cc3501e_hw_power_service()) to reach a safe default in the first place --
	 * this Wlan_Set() call already runs SYNCHRONOUSLY, inline, BEFORE
	 * Wlan_RoleUp(AP) below, so there is no window where beaconing could start
	 * under a stale ELP/AUTO_PS setting.
	 *
	 * Two related bugs WERE found here, one level up, in cc3501e_hw_ti_power.c:
	 *   1. Its effective-policy rule used to exclude AP from the
	 *      unconfigured-default override by testing cc3501e_hw_radio_role() !=
	 *      WIFI_AP -- and that role getter reports AP over STA whenever BOTH
	 *      are up.  So a scan/connect that brought STA up WHILE this AP was
	 *      already running read as "AP" there, took the pp_policy_latched
	 *      (BALANCED) branch instead of the unconfigured default.  Fixed by
	 *      gating on the STA role being up directly
	 *      (cc3501e_hw_wifi_sta_role_up()) instead of reading
	 *      cc3501e_hw_radio_role().
	 *   2. Even with (1) fixed, an EXPLICIT host CMD_POWER_POLICY of BALANCED /
	 *      LOW_POWER / DEEP_SLEEP still applied verbatim -- pm is device-wide
	 *      (ctrlCmdFw_SetSleepAuth has no per-role scoping), so that pulled
	 *      THIS role's forced ALWAYS_ACTIVE back to ELP the next time
	 *      cc3501e_hw_tick() drained it.  pp_apply_radio() now forces pm back
	 *      to ALWAYS_ACTIVE unconditionally whenever AP is up, regardless of
	 *      which policy (or who chose it) is otherwise being applied -- see
	 *      its header comment.  ps (the STA-specific half) is unaffected and
	 *      still follows the host's policy.
	 * With AP up and STA never coming up, cc3501e_hw_tick()'s drain never runs
	 * at all either way (pp_apply_radio_effective() requires an STA role up
	 * before calling pp_apply_radio()), so a host CMD_POWER_POLICY sent while
	 * only this AP is running does not reach the radio until a STA role also
	 * exists -- this Wlan_Set() call is what keeps an AP-only bridge on
	 * ALWAYS_ACTIVE in that window regardless. */
	WlanPowerManagement_e pm = POWER_MANAGEMENT_ALWAYS_ACTIVE_MODE; /* = 0 */
	(void)Wlan_Set(WLAN_SET_POWER_MANAGEMENT, (void *)&pm);

	/* A ZEROED RoleUpApCmd_t IS NOT A USABLE AP.  Several of its fields have a
	 * meaningful zero that is not "leave at default", and TI's own reference
	 * filler sets every one of them before Wlan_RoleUp
	 * (ParseRoleUpApCmd(), demos/network_terminal/cmd_parser.c in the CC35xx
	 * SDK).  This code used to pass the zero-init struct with only
	 * ssid/channel/secParams filled, which is the root cause of #1562:
	 *
	 *   sta_limit = 0 -- the field "limits the number of stations that the AP
	 *     has", so zero permits ZERO clients.  TI defaults it to 4 and clamps
	 *     anything outside [1, 8] back to 4, i.e. 0 is not a legal value at
	 *     all -- it is the uninitialised value reaching the NWP.  BENCH
	 *     (2026-08-28, two radios, the measurement that finally separated
	 *     "AP is down" from "AP refuses clients"): the AP beaconed
	 *     CONTINUOUSLY for 275 s at 33-95% signal on a second radio while 16
	 *     association attempts from an Intel AX200 all stalled in
	 *     `associating` and never reached `connected` -- identically for WPA2
	 *     and for an OPEN AP, which rules the security path out -- and the
	 *     firmware's WLAN event counter never moved off the 3 role-up events,
	 *     so the NWP never even reported an association attempt.
	 *   countryDomain = {0,0,0} -- TI sets the "00" world regulatory domain.
	 *   sae_anticlogging_threshold = 0 -- that is SAE_ANTI_CLOGGING_ALWAYS,
	 *     a real setting, not "unset"; TI uses SAE_ANTI_CLOGGING_DEFAULT (5).
	 *   sae_pwe = 0 -- TI uses 2.  Both SAE fields matter only for the WPA3
	 *     security type, but they are wrong in the same way for it.
	 *
	 * hidden = FALSE and tx_pow = 0 (= max) already match TI's defaults, so
	 * they stay implicit in the zero-init rather than being restated. */
	RoleUpApCmd_t ap              = { 0 };
	ap.ssid                       = ssid_buf;
	ap.channel                    = 6u; /* common 2.4 GHz default */
	ap.sta_limit                  = CC3501E_AP_STA_LIMIT;
	ap.countryDomain[0]           = '0';
	ap.countryDomain[1]           = '0';
	ap.countryDomain[2]           = '\0';
	ap.sae_pwe                    = CC3501E_AP_SAE_PWE;
	ap.sae_anticlogging_threshold = (uint8_t)SAE_ANTI_CLOGGING_DEFAULT;
	ap.secParams.Type             = (uint8_t)cc3501e_wifi_sec(security);
	ap.secParams.Key              = (int8_t *)psk;
	ap.secParams.KeyLen           = psk_len;
	/* Bounded, like the STA role-up above and for the same reason this file
	 * already states at the wifi_sta_role_up latch: "a stuck role-up must
	 * never hang the worker / the bridge SPI re-open".  This call was the
	 * one place that ignored that rule.
	 *
	 * WIFI_AP_START is worker-routed, and worker_run_pending() holds
	 * cc3501e_bridge_busy() (READY LOW) across worker_execute().  An AP
	 * RoleUp the NWP never command-completes therefore blocks the drain
	 * FOREVER: the post-op bridge_transport_spi_hw_reinit() and the READY
	 * raise are never reached, the SPI slave is never re-armed, and every
	 * later host command fails until a power cycle.  Same 10000 ms budget as
	 * the STA path; a timeout maps to ERR_IO so the drain always returns and
	 * re-arms the bridge.  Issue #5. */
	if (Wlan_RoleUp(WLAN_ROLE_AP, &ap, CC3501E_WIFI_ROLE_TIMEOUT_MS) != 0) {
		return CC3501E_HW_ERR_IO;
	}
	wifi_ap_role_up = true;
	network_set_up(network_get_ap_if());
	return CC3501E_HW_OK;
}

int cc3501e_hw_wifi_ap_stop(void)
{
	if (!wifi_started) {
		return CC3501E_HW_OK;
	}
	/* Bounded for the same reason as the role-up.  This one is the more
	 * dangerous of the two in practice: BRINGUP_STATUS.md warns "do NOT use
	 * `ap-stop` to reset, it wedges the bridge (alp-sdk#1564)", and an
	 * unbounded RoleDown inside the worker drain is exactly a mechanism that
	 * produces that.  Issue #5. */
	if (Wlan_RoleDown(WLAN_ROLE_AP, CC3501E_WIFI_ROLE_TIMEOUT_MS) != 0) {
		return CC3501E_HW_ERR_IO;
	}
	wifi_ap_role_up = false;
	/* The AP role-up path forces device-wide pm to ALWAYS_ACTIVE, both inline
	 * (cc3501e_hw_wifi_ap_start()'s own Wlan_Set() before RoleUp) and via
	 * pp_apply_radio()'s AP-up override on every later apply.  Neither is
	 * self-undoing, so re-apply now to re-derive the host's actual latched
	 * policy with the AP no longer forcing anything.
	 *
	 * THIS ONLY ACTUALLY FIXES IT WHEN A STA ROLE IS ALSO UP.  Safe to call
	 * unconditionally either way -- cc3501e_hw_power_apply_radio_now() ->
	 * pp_apply_radio_effective() no-ops cleanly with no STA role up (#5's
	 * guard) -- but that guard means an AP-ONLY bridge (no STA role ever
	 * brought up) is NOT fixed by this call: it stays ALWAYS_ACTIVE
	 * regardless, until a STA role comes up or the next POWER_POLICY lands
	 * with one already up.  That residual costs idle power only, not
	 * correctness -- ALWAYS_ACTIVE is always a safe (if wasteful) state,
	 * never a wrong one.  TASK CONTEXT ONLY, and safe here because
	 * WIFI_AP_STOP is worker-routed (src/worker.c, src/protocol_wifi.c's
	 * handle_wifi_ap_stop() -> handle_worker_routed()), so this body -- like
	 * cc3501e_hw_wifi_ensure_sta_role() -- always runs in worker_run_pending()
	 * on the bringup task, never the SPI-dispatch ISR. */
	cc3501e_hw_power_apply_radio_now();
	return CC3501E_HW_OK;
}

/* A received 802.11 signal is negative and above roughly -100 dBm.  Values at the
 * int8 floor (-127/-128) are the NWP's "not measured yet" state, and 0 is the
 * zero-init value; neither is a reading.  See cc3501e_hw_wifi_get_rssi (#1438). */
static bool rssi_plausible(int8_t dbm)
{
	return dbm < 0 && dbm > -127;
}

int cc3501e_hw_wifi_get_rssi(int8_t *rssi_dbm_out)
{
	if (rssi_dbm_out == 0) {
		return CC3501E_HW_ERR_INVAL;
	}
	const int wifi_rv = cc3501e_hw_wifi_lazy_start();
	if (wifi_rv != CC3501E_HW_OK) {
		return wifi_rv;
	}
	WlanBeaconRssi_t r = { 0 }; /* role_id 0 = STA */
	if (Wlan_Get(WLAN_GET_RSSI, &r) != 0) {
		return CC3501E_HW_ERR_IO;
	}

	/* VALIDATE before reporting (#1438).  rssi_beacon was passed through raw, so a
	 * read taken before the NWP has measured a beacon -- which is exactly where the
	 * `wifi connect` result line reads it, moments after association -- surfaced the
	 * unmeasured floor as a real dBm figure: `rssi=-127 dBm` for an AP that a scan
	 * minutes earlier put at -55, with `wifi status` reading -55 correctly a second
	 * later.  A number that wrong is worse than no number, because it looks like a
	 * measurement.
	 *
	 * No SDK constant names the floor, so bound it physically instead: a received
	 * 802.11 signal is negative and above roughly -100 dBm, so anything at or below
	 * -127 (the int8 floor region) or at/above 0 is not a measurement.  Zero is
	 * rejected for the same reason #1376 rejected it on the host -- it is the
	 * zero-init value and physically implausible.
	 *
	 * WlanBeaconRssi_t also carries rssi_data, the data-frame measurement, which is
	 * populated on a link carrying traffic even when no beacon has been sampled yet.
	 * Prefer the beacon reading, fall back to it, and only then report NOT_READY --
	 * which the console already renders as `rssi=unavailable`. */
	const int8_t beacon = r.rssi_beacon;
	const int8_t data   = r.rssi_data;
	const int8_t rssi   = rssi_plausible(beacon) ? beacon : data;

	if (!rssi_plausible(rssi)) {
		return CC3501E_HW_ERR_NOTIMPL; /* -> RESP_ERR_NOT_READY, "unavailable" */
	}
	*rssi_dbm_out = rssi;
	return CC3501E_HW_OK;
}

int cc3501e_hw_wifi_get_ip(uint8_t iface, uint8_t ip_out[4])
{
	if (ip_out == 0) {
		return CC3501E_HW_ERR_INVAL;
	}
	if (iface != (uint8_t)ALP_CC3501E_WIFI_IFACE_STA &&
	    iface != (uint8_t)ALP_CC3501E_WIFI_IFACE_AP) {
		return CC3501E_HW_ERR_INVAL;
	}
	if (!wifi_started) {
		return CC3501E_HW_ERR_IO; /* stack not up -> neither interface has an address */
	}
	/* The AP interface only exists once WIFI_AP_START has rolled the role up;
	 * asking before that is a host sequencing error, not a transient, so say
	 * NOTIMPL (-> RESP_ERR_NOT_READY) rather than IO (-> RESP_ERR_RADIO, which
	 * the host retries for its whole budget). */
	if (iface == (uint8_t)ALP_CC3501E_WIFI_IFACE_AP && !wifi_ap_role_up) {
		return CC3501E_HW_ERR_NOTIMPL;
	}
	const int role = (iface == (uint8_t)ALP_CC3501E_WIFI_IFACE_AP) ? WLAN_ROLE_AP : WLAN_ROLE_STA;
	uint32_t  ip = 0u, mask = 0u, gw = 0u, dhcp = 0u;
	if (network_stack_get_if_ip(role, &ip, &mask, &gw, &dhcp) != 0) {
		return CC3501E_HW_ERR_IO;
	}
	if (ip == 0u) {
		return CC3501E_HW_ERR_IO; /* DHCP not yet acquired -> NOT-ready-ish (RADIO) */
	}
	/* Emit the u32 MSB-first.  network_stack_get_if_ip hands back the lwIP
	 * netif address, which is a NETWORK-order u32, so on this little-endian core
	 * this extraction puts the LAST octet in ip_out[0] -- the wire order is
	 * reversed, and alp-sdk's cc3501e_wifi_get_ip() reverses it back (see the
	 * byte-order note there).  Both interfaces go through this same extraction,
	 * so the AP address needs no separate host-side handling. */
	ip_out[0] = (uint8_t)((ip >> 24) & 0xFFu);
	ip_out[1] = (uint8_t)((ip >> 16) & 0xFFu);
	ip_out[2] = (uint8_t)((ip >> 8) & 0xFFu);
	ip_out[3] = (uint8_t)(ip & 0xFFu);
	return CC3501E_HW_OK;
}
#else  /* !CC3501E_WIFI -- stub / silicon-free host build */
int cc3501e_hw_wifi_scan_start(void)
{
	return CC3501E_HW_ERR_NOTIMPL;
}

int cc3501e_hw_wifi_scan_stop(void)
{
	return CC3501E_HW_ERR_NOTIMPL;
}

int cc3501e_hw_wifi_scan(uint8_t *buf, size_t cap, size_t *out_len)
{
	(void)buf;
	(void)cap;
	if (out_len != 0) *out_len = 0u;
	return CC3501E_HW_ERR_NOTIMPL;
}

int cc3501e_hw_wifi_connect_sta(const uint8_t *ssid,
                                uint8_t        ssid_len,
                                const uint8_t *psk,
                                uint8_t        psk_len,
                                uint8_t        security)
{
	(void)ssid;
	(void)ssid_len;
	(void)psk;
	(void)psk_len;
	(void)security;
	return CC3501E_HW_ERR_NOTIMPL;
}

int cc3501e_hw_wifi_disconnect(void)
{
	return CC3501E_HW_ERR_NOTIMPL;
}

int cc3501e_hw_wifi_ap_start(const uint8_t *ssid,
                             uint8_t        ssid_len,
                             const uint8_t *psk,
                             uint8_t        psk_len,
                             uint8_t        security)
{
	(void)ssid;
	(void)ssid_len;
	(void)psk;
	(void)psk_len;
	(void)security;
	return CC3501E_HW_ERR_NOTIMPL;
}

int cc3501e_hw_wifi_ap_stop(void)
{
	return CC3501E_HW_ERR_NOTIMPL;
}

int cc3501e_hw_wifi_get_rssi(int8_t *rssi_dbm_out)
{
	if (rssi_dbm_out != 0) *rssi_dbm_out = 0;
	return CC3501E_HW_ERR_NOTIMPL;
}

int cc3501e_hw_wifi_get_ip(uint8_t iface, uint8_t ip_out[4])
{
	(void)iface;
	(void)ip_out;
	return CC3501E_HW_ERR_NOTIMPL;
}
#endif /* CC3501E_WIFI */
