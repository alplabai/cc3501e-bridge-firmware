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
#include "transport.h"  /* bridge_transport_spi_hw_reinit/suspend, cc3501e_bridge_busy/ready */
#include "wifi_retry.h" /* wifi_retry_delay_ms / wifi_retry_should_restore_first_pass -- the
                          * silicon-free retry decisions the reason-30 retry site (further
                          * down) calls into.  #include'd unconditionally, like
                          * wifi_last_reason_tag just below: wifi_retry_event_t has no TI-SDK
                          * dependency, so this compiles in both ti sub-builds. */
#include "wifi_connect_fail_skip.h" /* wifi_wait_host_frame() -- the silicon-free "poll for a
                                      * served host frame" wait every CONNECT failure exit
                                      * calls into below, via wifi_connect_fail_mark_skip().
                                      * Same unconditional-include reasoning as wifi_retry.h
                                      * just above. */

/* RUN12 MECHANISM (referenced by name from every other site in this file that
 * touches a reason/event pair -- this is the one place it is explained in
 * full).
 *
 * Last 802.11 reason/status code the cb actually saw, PAIRED with WHICH
 * vendor event recorded it -- see the accessor cc3501e_hw_wifi_last_reason()
 * (both build variants, further down) for the reason half's external
 * contract, and wifi_retry.h's own comment on wifi_retry_event_t for the
 * event half's.  The two used to be separate statics (wifi_last_reason +
 * wifi_last_reason_event), each written and read independently -- RUN12
 * replaces them with ONE `volatile uint32_t`, bits[31:16] the event tag,
 * bits[15:0] the reason's raw bit pattern, because separate statics are not
 * actually a matched pair under concurrent access: wifi_event_cb() runs on
 * the vendor event-callback thread, which SDK config sets to PRIORITY 8
 * (control_cmd_fw.h:52), strictly above the worker thread's PRIORITY 6 that
 * runs cc3501e_hw_wifi_connect_sta() -- the higher-priority cb thread can
 * preempt the worker BETWEEN two separate loads (or two separate stores),
 * so a reader that loads the reason and the event as two instructions could
 * observe the reason from one vendor event paired with the event tag from a
 * DIFFERENT, later one.  A single 32-bit store/load is naturally atomic on
 * this Cortex-M33 (no tearing, aligned word access), so every write below
 * packs BOTH halves into one store, and every read that needs the pair loads
 * this word ONCE into a local before decoding either half -- the tag moves
 * only together with the reason, never one without the other.
 *
 * 0 = none recorded (reason 0, event NONE): the boot default, the value on
 * the stub / silicon-free build (which never sees a real WLAN event), and
 * what a fresh attempt reads until it records one of its own.
 *
 * Declared UNCONDITIONALLY (unlike wifi_sta_role_up and friends, which stay
 * inside the #ifdef CC3501E_WIFI branch below): the connect-status latch's
 * cc3501e_hw_wifi_mark_connecting() (itself unconditional, further down, so
 * both ti sub-builds link) clears it directly, so it must be visible in the
 * non-Wi-Fi ti build too, not just the real one that writes it from
 * wifi_event_cb().  wifi_conn_set(), the other reader, stays CC3501E_WIFI-only
 * -- unlike mark_connecting(), it has no reason to run (or even exist)
 * without a real connect body to call it, and every one of its call sites is
 * itself inside that same #ifdef.  wifi_retry_event_t itself has no TI-SDK
 * dependency either, so none of this costs the non-Wi-Fi build anything. */
static volatile uint32_t wifi_last_reason_tag;

/* Pack/unpack helpers, guarded (unlike wifi_last_reason_tag itself, just
 * above): every CALLER of these three -- wifi_event_cb()'s write sites and
 * cc3501e_hw_wifi_connect_sta()'s retry site -- is itself CC3501E_WIFI-only.
 * mark_connecting() (unconditional) clears the tag with a bare `= 0` literal
 * rather than wifi_reason_tag_pack(WIFI_RETRY_EVENT_NONE, 0), so it needs
 * none of these; leaving them unguarded would just warn as unused in the
 * non-Wi-Fi ti build.
 *
 * Pack (event, reason) into the single word wifi_last_reason_tag holds.  The
 * (uint16_t) cast on `reason` is the round-trip step: C requires int-to-
 * unsigned conversion to preserve the bit pattern (mod 2^16), and the
 * matching (int16_t) cast in wifi_reason_tag_reason() below reverses it --
 * implementation-defined by the standard, but the identity on every
 * two's-complement toolchain this firmware targets (ticlang, arm-none-eabi-gcc). */
#ifdef CC3501E_WIFI
static inline uint32_t wifi_reason_tag_pack(wifi_retry_event_t event, int16_t reason)
{
	return ((uint32_t)(uint16_t)event << 16) | (uint32_t)(uint16_t)reason;
}

/* Decode the reason half of a tag word already loaded into a local -- callers
 * must load wifi_last_reason_tag ONCE and pass that local here, not read the
 * volatile again, or they reintroduce the exact two-separate-loads race
 * RUN12 exists to close. */
static inline int16_t wifi_reason_tag_reason(uint32_t tag)
{
	return (int16_t)(tag & 0xFFFFu);
}

/* Decode the event half -- same one-load-then-decode contract as
 * wifi_reason_tag_reason() above. */
static inline wifi_retry_event_t wifi_reason_tag_event(uint32_t tag)
{
	return (wifi_retry_event_t)(tag >> 16);
}
#endif /* CC3501E_WIFI */

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

/* Issue #144 (the persistent stale-deauth-reason-3 fix): has THIS firmware
 * ever issued its own Wlan_Disconnect() -- wifi_clear_stale_assoc()'s
 * post-failure cleanup below, or cc3501e_hw_wifi_disconnect()'s host-
 * requested teardown -- since boot?  Both resolve to the SAME vendor call
 * (`Wlan_Disconnect(WLAN_ROLE_STA, NULL)`), which (when the STA state
 * machine is not already idle) leaves pDrv->deauthReason == 3
 * (WLAN_REASON_DEAUTH_LEAVING) with no per-attempt reset -- see
 * wifi_retry_sanitize_reason()'s own comment (src/wifi_retry.h) for the
 * full SDK trace and the residual this coarse, boot-scoped flag accepts.
 * Set at both Wlan_Disconnect() call sites below, read only by
 * wifi_conn_set() right before it freezes a terminal reason. */
static bool wifi_own_disconnect_issued;

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

/* RUN10/RUN11 OBSERVABILITY (why there is no first-pass-wake-event static
 * here): a bench operator has no direct way to read back, after the fact,
 * whether cc3501e_hw_wifi_connect_sta()'s bounded retry fired for a given
 * connect call.  wifi_cb_last_id above cannot serve that purpose -- by the
 * time a connect call returns, the retry's own #1437 cleanup (if it ran) has
 * already overwritten it with that cleanup's own DISCONNECT, the SAME
 * contamination that made Run9's "last event id 2" reading inconclusive
 * (see the ASSOCIATION_REJECTED case's own comment).  A prior version of
 * this fix added two module-static "SWD-readable" mirrors for this, but on
 * an ACTIVATED CC3501E there is no SWD path to read them from: issue #21
 * records OpenOCD reporting "Could not find MEM-AP to control the core" and
 * AP 1 returning a constant 0x00080025 at every address -- unlike
 * wifi_cb_last_id, which IS observable, but only because it goes out over
 * the wire in GET_DIAG_INFO, not via SWD.  Removed as write-only dead state
 * rather than kept as statics nothing can read.
 *
 * The retry IS observable today, just indirectly, through connect timing:
 * since RUN12 (see wifi_last_reason_tag's own declaration comment, top of
 * file, and wifi_retry_delay_ms()'s comment) a retried connect attempt no
 * longer pays the SAME delay for every shape (RUN11's compromise; see
 * CC3501E_WIFI_RETRY_DENYLIST_DELAY_MS's own comment for why that compromise
 * existed and what closes it here).  For a comeback-IE shape the total is
 * roughly 8-12 s end to end: the first pass's ~4-5 s (bench-measured, see
 * RUN9/RUN10's own timing notes) plus up to CC3501E_WIFI_RETRY_DISCONNECT_
 * WAIT_MS(2 s) plus CC3501E_WIFI_RETRY_COMEBACK_DELAY_MS(1 s) plus the
 * retry's own association -- NOT the 1-2 s an earlier version of this
 * comment claimed, which counted only the fixed delays and left out the
 * two association attempts themselves.  For a deny-list (auth, or any
 * other) shape, run11's bench (the always-11 s-delay predecessor of this
 * mechanism) measured 19.6-34.5 s for its 12 successes -- RUN12 changes
 * WHICH events get which delay, not the deny-list delay itself, so that
 * shape's total is expected to land in the same range, not yet re-measured
 * under RUN12 specifically.  Those 19.6-34.5 s totals are END-TO-END
 * connect-call time, NOT the association phase alone: they also include
 * role-up (bounded 10 s, CC3501E_WIFI_ROLE_TIMEOUT_MS) and the DHCP lease
 * poll (bounded 20 s, CC3501E_STA_DHCP_TRIES * CC3501E_STA_DHCP_POLL_US)
 * that follow a successful association.  What IS capped at 30 s from the
 * first Wlan_Connect is the association phase itself (first attempt + this
 * retry's own overhead + its own wait), enforced directly by the retry's
 * own deadline-arithmetic check -- not the whole connect call, which these
 * bench totals measure. A
 * wire field carrying the first-pass wake event id directly (GET_DIAG_INFO's
 * 18-byte reply and CMD_WIFI_STATUS's alp_cc3501e_wifi_status_t are BOTH
 * already fully packed, protocol_diag.c / protocol_wifi.c) stays a
 * documented follow-up, not done here since it needs an actual wire-format
 * change (a new opcode field or a version bump), out of scope for this fix. */

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
		/* CORRECTED: this comment used to claim WlanEventConnect_t::Status
		 * "never fired negative on this SDK", citing cme_connection_mng.c
		 * ~8025 as one of only two writers, both non-negative.  BOTH of that
		 * claim's citations are dead code: ~8025 sits inside the `#if 0` block
		 * at cme_connection_mng.c ~7966-8136, and the OTHER site this comment
		 * used to cite (~8454, the ALREADY_CONNECTED write) is ALSO dead, inside
		 * a SEPARATE `#if 0` block at cme_connection_mng.c ~8420-8512.  Neither
		 * ever compiles.  The claim itself is false regardless: a real writer,
		 * cme_station_flow.c ~664-676 (CmeScanDone), sets Status = -1
		 * ("connection failed, probably timeout") and dispatches it directly as
		 * WLAN_EVENT_CONNECT.  It is reached from cme.c ~2907-2919
		 * (CME_MESSAGE_ID_SCAN_DONE) -> CmeStationFlowSM(CME_STA_SCAN_DONE, ...)
		 * -> CmeScanDone, via the state-machine dispatch table at
		 * cme_station_flow.c:118 (`{CmeScanDone, ...}` for CME_STA_SCAN_DONE).
		 * The LIVE success writer (the real analogue of the dead ~8025 site) is
		 * cme.c ~4037, `pArgs->Status = 0;`, also dispatched as WLAN_EVENT_CONNECT.
		 * The plain assignment below already handles a negative Status correctly
		 * (it is stored and read back as the signed int it is, no masking or
		 * unsigned reinterpretation) -- nothing here needs to change, only
		 * the claim that the negative path was unreachable. */
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
		 * safety one): mark_connecting() clears wifi_last_reason_tag at SUBMIT
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
		 * FIRST REAL CODE WINS: only write when the tag's reason half is still
		 * 0 (nothing recorded for this attempt).  Needed because
		 * ASSOCIATION_REJECTED's non-terminal handling (see that case) lets a
		 * genuine rejection -- status 30 -- sit recorded while the vendor's own
		 * comeback retry runs; a retry sequence that ultimately fails ends in
		 * exactly THIS event, with a generic, self-inflicted reason (3,
		 * WLAN_REASON_DEAUTH_LEAVING, from hostap's own give-up path,
		 * sme_deauth() at sme.c ~2228-2245) that carries far less information
		 * than the 30 already recorded.  Without this guard, that generic 3
		 * would silently replace the real rejection reason right as the
		 * attempt goes terminal, which is the one moment a host is guaranteed
		 * to actually read this byte.  Does NOT change the residual above:
		 * that scenario also starts from a freshly-reset tag (a fresh
		 * attempt's own reset), so it still writes exactly as documented.
		 * Does NOT apply to ASSOCIATION_REJECTED / AUTHENTICATION_REJECTED
		 * themselves -- each of those is always itself a specific, real status
		 * worth recording, so a second one differing from a first (however
		 * that happened) is not the same "generic close-out overwrites a real
		 * reason" problem this guard exists for. */
		if (wifi_conn_is_connecting() && wifi_reason_tag_reason(wifi_last_reason_tag) == 0 &&
		    event->Data.Disconnect.ReasonCode != (int16_t)WLAN_DISCONNECT_USER_INITIATED) {
			wifi_last_reason_tag = wifi_reason_tag_pack(WIFI_RETRY_EVENT_DISCONNECT,
			                                            event->Data.Disconnect.ReasonCode);
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
		 * Record the status into the live tag (still gated on
		 * wifi_conn_is_connecting(), same rule as every other case) so THIS
		 * pass's reason survives to be read at the retry-eligibility check --
		 * NOT so a host can observe it mid-retry: CMD_WIFI_STATUS reads
		 * g_wifi_conn.reason, the FROZEN copy wifi_conn_set() writes only at
		 * the terminal transition, and that stays whatever it was before this
		 * attempt (typically 0) for the whole CONNECTING window -- a polling
		 * host sees nothing new until this attempt actually ends.  Do NOT set
		 * wifi_last_status or signal here -- the connect body simply keeps waiting
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
		 * RUN9 UPDATE: this branch is CORRECT per the vendor code traced
		 * above and stays -- but Run9's bench (GPE 0.254.9.0, e1m-aen-evk-01)
		 * still showed the SAME alternating pattern with this fix applied,
		 * every rejection recording reason 30, landing at 3.9-4.2 s -- so
		 * SOMETHING about this bench's shape of the 30 is not what this `if`
		 * was written for.  NOT ESTABLISHED which: the bench's own "last
		 * vendor event id 2" (WLAN_EVENT_DISCONNECT) does NOT prove the
		 * reject itself arrived as a DISCONNECT -- the failed attempt's own
		 * #1437 cleanup (wifi_clear_stale_assoc() -> Wlan_Disconnect())
		 * always fires right after ANY REJECTED outcome and generates its
		 * own trailing DISCONNECT event, so "last event id 2" reads that way
		 * regardless of what actually ended the attempt.  Two CANDIDATE
		 * explanations, both consistent with the bench data, neither
		 * confirmed: (a) this cb's own comeback-IE branch never fired
		 * because the AP's rejection frame did not carry
		 * WLAN_TIMEOUT_ASSOC_COMEBACK -- ASSOCIATION_REJECTED(30) WITHOUT
		 * the IE skips the `if` above and falls through to
		 * destroyAssocData() (drv_ti_mlme.c ~1366-1372), ending via a real
		 * RX_MGMT_ASSOC the supplicant then disassociates on -- landing
		 * inside 5 s easily; or (b) the reject was actually
		 * AUTHENTICATION_REJECTED(30) the whole time (see that case below --
		 * it uses the SAME status constant on a different event, and the
		 * DISCONNECT case only records a reason when none is recorded yet,
		 * so either candidate is equally capable of being "the" 30 a host
		 * reads).  cc3501e_hw_wifi_connect_sta()'s own bounded retry (see
		 * the "RUN9 BOUNDED RETRY" comment there) is deliberately keyed on
		 * the RECORDED REASON CODE (30), not on which event carried it, so
		 * it covers both candidates without needing to prove which one is
		 * real -- and now (see wifi_retry_delay_ms()'s own comment) it picks
		 * the delay from the event half of wifi_last_reason_tag, recorded
		 * right here and in the AUTHENTICATION_REJECTED case below (RUN12
		 * mechanism, see wifi_last_reason_tag's own declaration comment, top
		 * of file) -- candidate (a) gets the short comeback delay this case
		 * records, candidate (b) the long deny-list one that case records
		 * (see RUN10 UPDATE there for why (b) is the one this bench actually
		 * hits).
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
				wifi_last_reason_tag =
				    wifi_reason_tag_pack(WIFI_RETRY_EVENT_ASSOCIATION_REJECTED, (int16_t)status);
			}
			break; /* deliberately NO wifi_last_status write, NO signal --
			        * this is not terminal; see the comment above.  Status 17
			        * is NOT included here -- it gets no vendor retry either,
			        * so treating it as non-terminal would only delay the
			        * failure; it falls through to the terminal path below. */
		}

		wifi_last_status = -1;
		if (wifi_conn_is_connecting()) {
			wifi_last_reason_tag =
			    wifi_reason_tag_pack(WIFI_RETRY_EVENT_ASSOCIATION_REJECTED, (int16_t)status);
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
		 * rejection.
		 *
		 * RUN9/RUN10/RUN11: this is also why cc3501e_hw_wifi_connect_sta()'s
		 * bounded retry needs a delay MUCH longer than Run9's original ~1.5 s
		 * one when a reason-30 rejection could be THIS shape -- traced in
		 * drv_ti_mlme.c:1177 (event dispatch) then :1187 (deny-list add),
		 * an auth rejection adds the AP's BSSID to OUR OWN station-side
		 * driver's local deny list for DENY_LIST_DEFAULT_EXPIRY_TIME (10*1000,
		 * i.e. 10 s, drv_ti_internal.h:370) -- NOT something the AP itself
		 * does -- and that SAME local list is what filters a still-denied
		 * BSSID OUT of scan results (driver_osprey_mx_scan.c:734), so
		 * re-issuing Wlan_Connect before that local entry expires cannot
		 * find the AP at all, regardless of what the AP itself would have
		 * done.  RUN10 picked a per-event delay by reading the MOST RECENT
		 * vendor event back at wake, which was unsound (see
		 * CC3501E_WIFI_RETRY_DENYLIST_DELAY_MS's own RUN11 comment); RUN11
		 * then papered over that by making EVERY reason-30 retry pay this
		 * longer delay regardless of shape.  RUN12 removes that compromise
		 * instead of keeping it: this records
		 * WIFI_RETRY_EVENT_AUTHENTICATION_REJECTED right here, packed into
		 * the same word as the reason two lines down (RUN12 mechanism, see
		 * wifi_last_reason_tag's own declaration comment, top of file) --
		 * the retry site (wifi_retry_delay_ms()) can once again pick the
		 * short comeback delay for the OTHER shape without risking picking
		 * it for THIS one. */
		if (wifi_conn_is_connecting()) {
			wifi_last_reason_tag = wifi_reason_tag_pack(WIFI_RETRY_EVENT_AUTHENTICATION_REJECTED,
			                                            (int16_t)event->Data.AuthStatusCode);
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
 * reason is a FROZEN COPY of the reason half of wifi_last_reason_tag (top of
 * file), NOT a live mirror.  As of RUN12, wifi_conn_set()'s caller resolves
 * and passes this value explicitly as its own `reason` argument rather than
 * wifi_conn_set() re-reading the live tag itself -- see that function's own
 * comment for why a late vendor event landing between the caller's decision
 * and the call would otherwise be able to publish an unintended value.
 * Freezing matters even so: wifi_event_cb() also gates writes to the tag on
 * wifi_conn_is_connecting(), which protects the tag from being overwritten
 * AFTER this attempt's terminal transition, right up until the NEXT
 * mark_connecting() reopens it (and clears the live tag back to 0 -- see
 * mark_connecting()'s own comment).  Freezing here is what lets a host still
 * read THIS attempt's reason correctly even after that next
 * mark_connecting() has already cleared the live tag for the new attempt.
 *
 * SCOPE: this byte covers the CONNECT ATTEMPT only -- the reason or status
 * that ENDED or REJECTED that attempt (a DISCONNECT/REJECTED event that
 * TERMINATES it, i.e. arrives while state is still CONNECTING).  Once a
 * connect reaches CONNECTED, wifi_conn_set() has already frozen this field for
 * that attempt and nothing calls it again for THAT association.  CORRECTED
 * (this used to claim a post-CONNECTED deauth "updates the LIVE
 * wifi_last_reason" -- it does not): every wifi_event_cb() case that writes
 * the tag gates that write on wifi_conn_is_connecting() first, and state is
 * no longer CONNECTING once a connect has reached CONNECTED, so a
 * spontaneous, AP-initiated deauth arriving AFTER that point is simply
 * DROPPED by this cb -- neither the live tag nor
 * g_wifi_conn.reason/state moves.  This firmware has no background watcher
 * for a post-connect deauth today: g_wifi_conn.state stays CONNECTED until
 * something else changes it (a host WIFI_DISCONNECT, or the next
 * mark_connecting() reopening the gate for a fresh attempt).  That absence of
 * a watcher is a pre-existing gap in THIS firmware's async-latch design
 * generally, not specific to this byte, and post-connect tracking is
 * deliberately out of scope here. */
static volatile struct {
	uint8_t state;       /* alp_cc3501e_wifi_conn_state_t   */
	uint8_t fail_reason; /* alp_cc3501e_wifi_fail_t          */
	int8_t  rssi;        /* NEVER POPULATED -- always 0      */
	int16_t reason;      /* frozen copy of the reason half of wifi_last_reason_tag        */
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

/* Handoff to src/worker.c's drain (issue #106, connect flavour): whether THIS
 * run's SUCCESS exit already re-armed the slave itself, and whether that
 * reinit actually armed it.  Set together, immediately before the
 * SUCCESS-path wifi_conn_set(CONNECTED) below; read-and-cleared once by
 * cc3501e_hw_wifi_connect_sta_take_reinit().  Both run on the SAME thread
 * (the worker drain) with nothing else touching either field in between --
 * worker_run_pending() calls worker_execute() (which calls this body to
 * completion) and only then reads the flag, so plain (non-volatile) access is
 * enough; there is no ISR side to this one, unlike g_wifi_conn above. */
static bool g_connect_reinit_pending;
static bool g_connect_reinit_armed;

bool cc3501e_hw_wifi_connect_sta_take_reinit(bool *armed_out)
{
	const bool pending = g_connect_reinit_pending;
	if (pending) {
		if (armed_out != 0) {
			*armed_out = g_connect_reinit_armed;
		}
		g_connect_reinit_pending = false; /* one-shot: consumed */
	}
	return pending;
}

/* Handoff to src/worker.c's drain (issue #106, RSSI flavour): whether THIS
 * run of cc3501e_hw_wifi_get_rssi() may have its drain reinit skipped.  Set
 * unconditionally near the top of that function, before either radio call in
 * it can fail, so every exit (success or IO error) reports the same fact.
 * Read-and-cleared once by cc3501e_hw_wifi_get_rssi_take_reinit_skip(); same
 * same-thread, no-ISR reasoning as the connect handoff above applies. */
static bool g_rssi_reinit_skip_pending;
static bool g_rssi_reinit_skip_ok;

bool cc3501e_hw_wifi_get_rssi_take_reinit_skip(bool *skip_ok_out)
{
	const bool pending = g_rssi_reinit_skip_pending;
	if (pending) {
		if (skip_ok_out != 0) {
			*skip_ok_out = g_rssi_reinit_skip_ok;
		}
		g_rssi_reinit_skip_pending = false; /* one-shot: consumed */
	}
	return pending;
}

/* Handoff to src/worker.c's drain (connect-FAILURE flavour, advisor
 * analysis): whether the FAILURE exit of THIS run of
 * cc3501e_hw_wifi_connect_sta() may have its drain reinit skipped.  Set by
 * wifi_connect_fail_mark_skip() below, called at EVERY failure exit
 * immediately before that exit's own wifi_conn_set(FAILED) -- see that
 * helper's own comment for why the sample has to be taken freshly THERE,
 * not at some earlier point in this function.  Read-and-cleared once by
 * cc3501e_hw_wifi_connect_sta_take_fail_skip(); same same-thread, no-ISR
 * reasoning as the SUCCESS-exit handoff above applies (both run on the
 * worker drain thread, nothing else touches either field in between). */
static bool g_connect_fail_skip_pending;
static bool g_connect_fail_skip_ok;

bool cc3501e_hw_wifi_connect_sta_take_fail_skip(bool *skip_ok_out)
{
	const bool pending = g_connect_fail_skip_pending;
	if (pending) {
		if (skip_ok_out != 0) {
			*skip_ok_out = g_connect_fail_skip_ok;
		}
		g_connect_fail_skip_pending = false; /* one-shot: consumed */
	}
	return pending;
}

/* Guarded, unlike the plain-bool handoff above: both of these call into
 * CC3501E_WIFI-only state (ClockP_usleep needs <ti/drivers/dpl/ClockP.h>,
 * #include'd only under CC3501E_WIFI further down, and the only caller of
 * either, cc3501e_hw_wifi_connect_sta(), is itself CC3501E_WIFI-only) -- an
 * unguarded definition here is simply unused (and warns as such, plus fails
 * to compile on ClockP_usleep) in the non-Wi-Fi ti build.  Same pattern as
 * wifi_conn_is_connecting() above. */
#ifdef CC3501E_WIFI
static void wifi_connect_fail_skip_sleep_ms(uint32_t ms)
{
	ClockP_usleep(ms * 1000u);
}

/* CORRECTED (host review of 580f748, the first version of this fix): that
 * version sampled its "has the slave served a frame" baseline right after
 * the role-up reinit, BEFORE Wlan_Connect, the retry pass, the association
 * wait, and all of the asynchronous CME work that actually performs the
 * 802.11 handshake.  Wlan_Connect() only QUEUES a message (see src/
 * wifi_connect_fail_skip.h's full citation trail) -- the radio work runs
 * LATER, on the CME task, outside this function's synchronous window
 * entirely.  The host polls WIFI_STATUS every 50 ms, so that early baseline
 * had almost always already advanced by the time ANY failure exit ran,
 * REGARDLESS of whether the slave was still alive when the failure actually
 * happened: the skip fired even when a LATER Wlan_Connect or the untraced
 * asynchronous association work had killed the slave's DMA, and the drain's
 * reinit -- the only thing that could have recovered it -- never ran.
 *
 * THE FIX: sample fresh, HERE, at the failure exit itself -- after
 * Wlan_Connect, the retry pass, and the association wait have already had
 * their chance to disturb the slave -- and POLL for up to
 * CC3501E_WIFI_CONNECT_FAIL_SKIP_WINDOW_MS (three host WIFI_STATUS poll
 * gaps) for a frame to land.  A slave still being serviced answers within
 * that window; a dead one does not, and the caller falls through to the
 * unconditional reinit exactly as before this whole fix -- the built-in
 * falsifier.  See wifi_wait_host_frame() (src/wifi_connect_fail_skip.h) for
 * the pure wait this wraps.
 *
 * NOT covered: the sample is taken BEFORE that exit's own trailing
 * wifi_clear_stale_assoc() runs, so neither the Wlan_Disconnect() it queues
 * nor any asynchronous CME work still in flight once the window ends is
 * observed.  Not a regression -- the PREVIOUS unconditional drain reinit
 * never covered that tail either, it just ran once regardless.  See src/
 * wifi_connect_fail_skip.h's own header for the full statement of what this
 * poll does and does not prove.
 *
 * Call this LAST at EVERY failure exit of cc3501e_hw_wifi_connect_sta(),
 * immediately before that exit's own wifi_conn_set(FAILED) -- including the
 * role-up-fail exit (where the slave may genuinely be dead after a failed
 * Wlan_Start, in which case no frame arrives and the reinit correctly still
 * runs) and the no-DHCP-lease exit, not just the terminal REJECTED/TIMEOUT
 * one. */
#define CC3501E_WIFI_CONNECT_FAIL_SKIP_WINDOW_MS 150u
#define CC3501E_WIFI_CONNECT_FAIL_SKIP_STEP_MS   10u

static void wifi_connect_fail_mark_skip(void)
{
	g_connect_fail_skip_ok      = wifi_wait_host_frame(cc3501e_hw_host_txn_count,
	                                                   wifi_connect_fail_skip_sleep_ms,
	                                                   CC3501E_WIFI_CONNECT_FAIL_SKIP_WINDOW_MS,
	                                                   CC3501E_WIFI_CONNECT_FAIL_SKIP_STEP_MS);
	g_connect_fail_skip_pending = true;
}
#endif /* CC3501E_WIFI */

void cc3501e_hw_wifi_mark_connecting(void)
{
	g_wifi_conn.fail_reason = (uint8_t)ALP_CC3501E_WIFI_FAIL_NONE;
	g_wifi_conn.rssi        = 0;
	/* Clear the LIVE reason/event tag too, not just the frozen copy below: this
	 * new attempt's own wifi_event_cb() writes gate on the tag's reason half
	 * being 0 ("first real code wins", see the DISCONNECT case) and the retry
	 * site's eligibility check reads it fresh -- both need a clean 0 baseline
	 * for THIS attempt, not whatever the previous one left behind.  (The
	 * FROZEN copy, g_wifi_conn.reason below, is a separate concern: every
	 * terminal caller of wifi_conn_set() now passes its own resolved reason
	 * explicitly -- RUN12, see that function's own comment -- rather than
	 * having it re-read the live tag, so clearing the tag here no longer
	 * protects that freeze the way an earlier version of this comment said;
	 * it protects the NEXT attempt's own recording instead.) */
	wifi_last_reason_tag = 0;
	g_wifi_conn.reason   = 0;
	g_wifi_conn.state    = (uint8_t)ALP_CC3501E_WIFI_CONNECTING; /* publish state last */
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
 * that ever writes it -- wifi_conn_set(), from the caller-resolved reason
 * described on g_wifi_conn's comment -- is CC3501E_WIFI-only.
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
 * `reason` (RUN12): the caller resolves this value BEFORE calling in, rather
 * than this function reading the live wifi_last_reason_tag itself.  An
 * earlier version did the latter, and every FAILED-path caller runs
 * wifi_clear_stale_assoc() (#1437) right after this returns, which issues
 * its own Wlan_Disconnect() -- if a caller had already decided to RESTORE
 * the first pass's reason (see cc3501e_hw_wifi_connect_sta()'s KICK/TIMEOUT/
 * terminal-REJECTED sites) by writing that value back into the live tag, a
 * late vendor event landing in the narrow window between that write and
 * this function's own read of it could silently replace the restored value
 * before it was ever frozen -- wifi_event_cb()'s ASSOCIATION_REJECTED /
 * AUTHENTICATION_REJECTED cases write unconditionally whenever
 * wifi_conn_is_connecting() is true (no "first real code wins" gate the way
 * DISCONNECT has), and state is STILL CONNECTING in that window, since this
 * function has not yet published the terminal state.  Passing the resolved
 * value as a parameter removes that window entirely: nothing this function
 * does can be raced by a concurrent tag write, because it never reads the
 * tag.
 *
 * ALSO enqueue the matching async EVT_* so a host that registered an event
 * callback (via CMD_GET_PENDING_EVENTS polling) is notified: CONNECTED ->
 * EVT_WIFI_CONNECTED, a terminal FAILED/DISCONNECTED -> EVT_WIFI_DISCONNECTED.
 * Both carry no payload -- the host reads the detail (rssi / fail_reason) via
 * CMD_WIFI_STATUS.  wifi_conn_set is the single terminal-transition chokepoint
 * (mark_connecting writes the CONNECTING latch directly and is NOT terminal), so
 * exactly one event is queued per terminal outcome. */
static void wifi_conn_set(uint8_t state, uint8_t fail_reason, int16_t reason)
{
	g_wifi_conn.fail_reason = fail_reason;
	g_wifi_conn.rssi        = 0;
	/* CONNECTED is the one state that ALWAYS freezes 0, ignoring `reason`
	 * entirely -- every CONNECTED caller already passes 0 (see below), but
	 * enforcing it here too keeps the invariant true regardless of the
	 * caller: hal/cc3501e_hw.h's contract for this byte is "the reason or
	 * status that ENDED or REJECTED that attempt", and a CONNECTED attempt
	 * was neither, so publishing a stale rejection alongside a successful
	 * CONNECTED would contradict that contract and mislead a host into
	 * reading a live association as somehow still carrying a past reject.
	 *
	 * CONNECTED also clears the LIVE wifi_last_reason_tag itself, not just
	 * the frozen g_wifi_conn.reason above, purely as hygiene for the NEXT
	 * connect attempt -- NOT because anything downstream of THIS attempt
	 * still reads the live tag: cc3501e_hw_wifi_disconnect() passes
	 * g_wifi_conn.reason itself, not the live tag (see that function's own
	 * comment), specifically so a later WIFI_DISCONNECT republishes
	 * whatever this call just froze (0, right here) regardless of what the
	 * live tag does or does not hold by then.  mark_connecting() clears the
	 * tag again anyway at the START of the next attempt, so this clear is
	 * redundant with that one in practice -- kept for the same reason
	 * mark_connecting()'s own comment gives: a live tag that reads 0
	 * whenever no attempt is in flight is a simpler invariant to reason
	 * about than one that is sometimes stale between attempts. */
	if (state == (uint8_t)ALP_CC3501E_WIFI_CONNECTED) {
		wifi_last_reason_tag = 0;
		g_wifi_conn.reason   = 0;
	} else {
		/* Issue #144: a reason of exactly 3 (WLAN_REASON_DEAUTH_LEAVING) can
		 * be OUR OWN earlier Wlan_Disconnect() cleanup leaking through
		 * rather than this attempt's own wire verdict -- see
		 * wifi_retry_sanitize_reason()'s own comment (src/wifi_retry.h) for
		 * the full trace.  Applies to both CONN_FAILED (a fresh terminal
		 * freeze) and DISCONNECTED (cc3501e_hw_wifi_disconnect()'s
		 * republish of an already-frozen value) alike -- idempotent either
		 * way, since sanitizing an already-sanitized value is a no-op. */
		g_wifi_conn.reason = wifi_retry_sanitize_reason(reason, wifi_own_disconnect_issued);
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
		 * NEW attempt's second wifi_last_reason_tag reset -- in
		 * cc3501e_hw_wifi_connect_sta(), right before its own Wlan_Connect --
		 * and the vendor actually starting to process that new connect) where
		 * that gate can still record THIS disconnect's reason against the
		 * wrong attempt. */
		(void)Wlan_Disconnect(WLAN_ROLE_STA, NULL);
		/* Issue #144: this is one of the two write sites that can leave
		 * pDrv->deauthReason == 3 for a LATER attempt to inherit -- see
		 * wifi_own_disconnect_issued's own declaration comment.  Set
		 * unconditionally, not gated on a return value: the vendor call
		 * dispatches its CME message and returns before any of this can be
		 * observed either way (best-effort, per this function's own top
		 * comment). */
		wifi_own_disconnect_issued = true;
	}
}

/* STA L3 bring-up: bounded DHCP-lease poll after the L2 connect event.
 * CC3501E_STA_DHCP_TRIES * CC3501E_STA_DHCP_POLL_US = 150 * 200 ms = 30 s budget
 * (the worker drain sleeps between tries so the tcpip thread runs DHCP).
 *
 * WIDENED 20 s -> 30 s on measurement, not on principle.  With the station held
 * ACTIVE and a stalled client restarted, the leases that still missed the call
 * were arriving about 1 SECOND after it gave up -- 3 of 16 in one bench run and
 * 2 of 16 in the next, each reported as a connect failure that the very next
 * host read then contradicted by handing back an address.  Those are not radio
 * failures; they are the call giving up immediately before the answer.
 *
 * 30 s is the largest value the caller budgets allow.  The deepest path that
 * reaches this poll is role-up + association + lease = 10 + 30 + 30 = 70 s,
 * against the 75000 ms the bench apps pass since their budgets were re-derived
 * (alp-sdk#2079).  At 35 s that path is 75 s and the caller times out first,
 * which is exactly the failure this change removes.
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
 * role-up + association + lease = 10 + 30 + 30 = 70 s against the 75000 ms the
 * bench apps pass since their budgets were re-derived (alp-sdk#2079).  At 35 s
 * that same path is 75 s and the caller gives up first, turning a fixable lease
 * delay back into an unreadable host timeout -- which is why 30 s is the ceiling
 * here and not a round number chosen for comfort.
 *
 * Measured, that path is nowhere near its worst case: two early failing runs
 * took 14.45 s and 16.87 s end to end, so role-up plus association cost roughly
 * 4.5 s, leaving the lease poll the overwhelming majority of the budget.
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
#define CC3501E_STA_DHCP_TRIES 150u

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

/* The 30 s association wait itself, named so the retry budget below can do
 * arithmetic against it instead of restating the literal.  Matches the
 * `30u * OSI_WAIT_FOR_SECOND` this function already waited with (OSI_WAIT_FOR_SECOND
 * == 1000, osi_kernel.h:357, so this is already in the same units,
 * milliseconds). */
#define CC3501E_WIFI_ASSOC_WAIT_MS 30000u

/* RUN9 BOUNDED RETRY (bench evidence + mechanism below; see the long comment
 * on the retry site in cc3501e_hw_wifi_connect_sta() for the full story).
 *
 * RUN11 (bench c872b94's review): RUN9 used a ~1.5 s PMF/stale-SA delay;
 * RUN10 (bench 16af840) found that too short for a second candidate shape
 * and added a SECOND, longer delay, picked by reading which vendor event
 * ended the pass (event_id_this_pass) at the moment the body woke.  That
 * pick was itself unsound: the read happened AFTER wake, so it returned the
 * MOST RECENT event, not necessarily the one that RECORDED the reason-30
 * this retry keys on -- a further event landing in the gap between the two
 * could select the SHORTER delay while our own driver still had the AP's
 * BSSID deny-listed (see below), making the retry re-issue Wlan_Connect too
 * early and end in an uninformative TIMEOUT/0 instead of ever finding the
 * AP.  RUN11's fix was to always wait out the LONGER of the two delays
 * regardless of which event ended the pass -- provably safe (never too
 * short for any of the three rejection shapes below), but it gave up the
 * whole point of RUN10's per-event pick: a comeback-IE retry that could have
 * re-issued Wlan_Connect in ~1 s now always pays the full ~11 s instead.
 *
 * RUN12 removes that compromise rather than keeping it, by fixing what was
 * actually unsound about RUN10's pick -- not WHERE the value came from
 * (which event), but WHEN it was read.  wifi_event_cb() now packs the event
 * into the SAME word as the reason (wifi_last_reason_tag, declared above) in
 * one store, so the tag moves only together with the reason -- not read back
 * piecemeal after this function's own wait wakes, which is what let a later
 * event pair a stale event with a fresh reason (or vice versa) before.
 * wifi_retry_delay_ms() (src/wifi_retry.h) then picks the delay from that
 * single loaded pair: the short comeback delay for
 * WIFI_RETRY_EVENT_ASSOCIATION_REJECTED, the long deny-list delay for every
 * other value (AUTHENTICATION_REJECTED, DISCONNECT, or NONE) -- see that
 * function's own comment for why the conservative default is right for the
 * shapes RUN11's data could not distinguish.
 *
 * Checked directly against the SDK for all three rejection shapes this `if`
 * and its neighbours discuss: the comeback-IE path (ASSOCIATION_REJECTED
 * WITH the IE) deny-lists only for its own comeback duration (typically
 * ~1 s -- `msecs = tu * 1024 / 1000` off the AP's own IE,
 * drv_ti_mlme.c:1293-1327; the driver itself resends up to ASSOC_MAX_TRIES(3)
 * meanwhile, drv_ti_sta_specific.c:710-738); the no-IE path deny-lists
 * nothing at all (falls straight to destroyAssocData(), drv_ti_mlme.c
 * ~1366-1372, no DenyList_addElement call on that path); and
 * AUTHENTICATION_REJECTED deny-lists for the full
 * CC3501E_WIFI_RETRY_DENYLIST_EXPIRY_MS below.  NOT YET RE-RUN ON THE BENCH:
 * this closes the mechanism gap RUN11 knowingly left open, but no run number
 * is claimed for it -- see the CC3501E_WIFI_RETRY_COMEBACK_DELAY_MS comment
 * for what a future bench run would need to show. */

/* CC3501E_WIFI_RETRY_DENYLIST_EXPIRY_MS: DENY_LIST_DEFAULT_EXPIRY_TIME
 * itself, read verbatim from the SDK, not guessed -- `#define
 * DENY_LIST_DEFAULT_EXPIRY_TIME 10*1000 // 10 seconds in milliseconds`
 * (drv_ti_internal.h:370), already milliseconds.
 *
 * WHOSE list, precisely (corrected -- an earlier version of this comment
 * called it "the AP's deny list", which is backwards): both call sites,
 * `DenyList_addElement(&(apDrv->denyList), ..., DENY_LIST_DEFAULT_EXPIRY_TIME
 * + osi_GetTimeMS())` (drv_ti_mlme.c:1187) and the comeback-IE one
 * (drv_ti_mlme.c:1315), add to `apDrv->denyList` -- apDrv is THIS
 * station's OWN driver state, so this is OUR OWN local list, entered
 * against the AP's BSSID as the key.  DenyList_elementExists()
 * (drv_ti_sta_specific.c ~1804-1850) consults that SAME local list to
 * filter scan/connect candidates (driver_osprey_mx_scan.c:734) -- it is OUR
 * OWN driver refusing to re-offer a BSSID it locally deny-listed, never
 * something the AP itself does, knows about, or enforces.
 *
 * Event order matters for the margin below: wlanDispatcherSendEvent
 * (WLAN_EVENT_AUTHENTICATION_REJECTED, ...) (drv_ti_mlme.c:1177) runs
 * BEFORE DenyList_addElement(..., DENY_LIST_DEFAULT_EXPIRY_TIME +
 * osi_GetTimeMS()) (drv_ti_mlme.c:1187) -- a few instructions apart, not a
 * meaningful span of wall-clock time -- so the local entry's expiry lands
 * for practical purposes DENY_LIST_DEFAULT_EXPIRY_TIME AFTER wake, meaning
 * the margin below is real margin, not slack already eaten by event
 * ordering. */
#define CC3501E_WIFI_RETRY_DENYLIST_EXPIRY_MS 10000u /* drv_ti_internal.h:370 */

/* +1 s margin for scheduling/RTT slop on top of the ~10 s expiry above --
 * not a vendor number.  Used for the AUTHENTICATION_REJECTED shape (and, per
 * wifi_retry_delay_ms()'s conservative default, every shape other than the
 * comeback-IE one below). */
#define CC3501E_WIFI_RETRY_DENYLIST_DELAY_MS (CC3501E_WIFI_RETRY_DENYLIST_EXPIRY_MS + 1000u)

/* Delay a retry waits when the FIRST pass's reason-30 was recorded by
 * ASSOCIATION_REJECTED (WIFI_RETRY_EVENT_ASSOCIATION_REJECTED) -- the
 * comeback-IE shape, whose local deny-list entry (if the IE even added one)
 * expires on the AP's OWN comeback interval, not our driver's fixed ~10 s
 * one: `msecs = tu * 1024 / 1000` off the AP's Timeout-Interval IE
 * (drv_ti_mlme.c:1293-1327), typically ~1 s for hostapd's own default
 * comeback time (1000 TU); the driver itself resends the association up to
 * ASSOC_MAX_TRIES(3) meanwhile (drv_ti_sta_specific.c:710-738), so by the
 * time THIS cb sees a terminal ASSOCIATION_REJECTED(30) recorded at all, that
 * comeback window has already been running.  1000u (1 s) is a round number
 * at least as long as the typical 1000 TU (~1.024 s) case -- not a vendor
 * constant, a chosen margin the same way CC3501E_WIFI_RETRY_DENYLIST_DELAY_MS's
 * own +1 s is.  UNVERIFIED ON THE BENCH as of this fix (see the RUN9 BOUNDED
 * RETRY comment's own note): a comeback IE advertising a longer interval than
 * this would make the retry re-issue Wlan_Connect while our own driver's
 * internal comeback timer (or the AP's SA) is still live, most likely
 * degrading to an uninformative TIMEOUT/0 rather than corrupting anything --
 * the next bench run against a real WPA3-SAE AP is what would confirm this
 * margin is generous enough in practice. */
#define CC3501E_WIFI_RETRY_COMEBACK_DELAY_MS 1000u

/* CC3501E_WIFI_RETRY_DISCONNECT_WAIT_MS: bounded wait for wifi_clear_stale_assoc()'s
 * own Wlan_Disconnect() to actually DISPATCH its WLAN_EVENT_DISCONNECT before
 * re-issuing Wlan_Connect.  Needed because the vendor SDK refuses a new
 * Wlan_Connect while WLAN_IF_DISCONNECT_IN_PROGRESS is set in its internal
 * oper-bitmap, and that bit clears ONLY when the disconnect event dispatches
 * (wlan_if.c ~2334) -- re-issuing too early is the SAME sticky-bit trap
 * cc3501e_hw_wifi_connect_sta()'s existing FAIL_KICK handling exists for
 * elsewhere in this file.  Generous relative to how fast a dispatch of an
 * already-completed operation should be; best-effort -- if it times out this
 * function still proceeds to re-issue Wlan_Connect, which then either
 * succeeds or reports FAIL_KICK exactly as any other oper-bitmap collision
 * would. */
#define CC3501E_WIFI_RETRY_DISCONNECT_WAIT_MS 2000u

/* Upper bound on the time this function spends BEFORE it can even re-issue
 * Wlan_Connect for the retry, used ONLY by the retry-eligibility budget check
 * below (elapsed + this + CC3501E_WIFI_RETRY_MIN_WAIT_MS <=
 * CC3501E_WIFI_ASSOC_WAIT_MS) -- NOT the actual per-retry sleep any more (that
 * is wifi_retry_delay_ms()'s per-event pick, which can be the shorter
 * CC3501E_WIFI_RETRY_COMEBACK_DELAY_MS).  Deliberately still built from
 * CC3501E_WIFI_RETRY_DENYLIST_DELAY_MS, the LONGER of the two per-event
 * delays, not a per-event value: the event IS already known by the time
 * the eligibility check runs (it was packed into wifi_last_reason_tag when
 * the reason was recorded, before this check ever reads it) -- but pinning
 * the budget check to a SINGLE, per-event-independent value is simpler
 * than branching this arithmetic on which delay wifi_retry_delay_ms() will
 * end up returning, and choosing the WORST-CASE value keeps that
 * simplification conservative -- eligibility can only under-promise the
 * time available, never over-promise it, regardless of which delay this
 * retry ends up sleeping. */
#define CC3501E_WIFI_RETRY_OVERHEAD_MS \
	(CC3501E_WIFI_RETRY_DISCONNECT_WAIT_MS + CC3501E_WIFI_RETRY_DENYLIST_DELAY_MS)

/* Floor on what must remain of the association budget AFTER paying that
 * overhead for the retry to be worth attempting at all -- a retry whose own
 * Wlan_Connect+wait would have only a sliver of the budget left is not worth
 * running (WPA3-SAE alone can take real time; see this function's own wait
 * comment).  Not tied to any specific vendor number -- a simple, generous
 * floor chosen so the retry only fires when there is genuinely enough of the
 * 30 s window left for it to have a fair shot.
 *
 * THE REAL BOUND on the whole association phase (first attempt + delay +
 * retry) is, and remains, CC3501E_WIFI_ASSOC_WAIT_MS (30 s) -- the
 * retry-eligibility check below enforces that DIRECTLY (elapsed +
 * CC3501E_WIFI_RETRY_OVERHEAD_MS + this floor <= CC3501E_WIFI_ASSOC_WAIT_MS),
 * not via a derived constant.  For CONTEXT, not as the actual limit: Run10's
 * typical numbers (rejects landing at ~3.9-5.2 s) put elapsed at ~4.2 s,
 * which is Run10's typical case, not the bound -- the check itself only
 * lets the retry fire while elapsed <= ~14000 ms (30000 - 13000 - 3000), and
 * its worst case is 14000 + 13000 = 27000 ms of wall-clock time before the
 * retry's OWN Wlan_Connect is even re-issued, still inside the 30 s window
 * with this 3 s floor left for that re-issued connect's own wait. */
#define CC3501E_WIFI_RETRY_MIN_WAIT_MS 3000u

/* #142 (host review of b3dc1e2): the SPI link self-heals (dead handle,
 * resync burst, arm failure, reply stall, quiet-armed) froze for the whole
 * duration of this function's own waits -- up to ~70 s -- because the ONLY
 * place they ran, cc3501e_hw_tick(), is drained on the SAME bring-up task
 * this function itself blocks.  A background task calling the heals
 * independently was tried and REJECTED (see src/link_quiet_rearm.h's top
 * comment for the full writeup) -- it let a reinit fire inside windows the
 * rest of this codebase guarantees are reinit-free.  The fix instead is to
 * call cc3501e_hw_link_heal(true) FROM INSIDE this function's own wait
 * points, sliced to <= CC3501E_WIFI_HEAL_SLICE_MS, in the THREE windows
 * below where this function itself already establishes the slave is safe to
 * touch: the host is known to be polling WIFI_STATUS every 50 ms (state is
 * CONNECTING the whole time) and no cc3501e_bridge_busy()-bracketed section
 * of THIS body is open.
 *
 * READY (CORRECTED, #142 item 6, host review of dfd5280): this used to also
 * claim READY is already known-good by this point on every pass.  That is
 * NOT true on the very first pass when role_up_was_latched is true (no
 * reinit ran in THIS call at all): the worker drain's own busy()/reinit()
 * bracket around the WHOLE job (src/worker.c, ~877) holds READY LOW from
 * before this function was even entered, and nothing in this body raises it
 * again until the SPI ISR's own re-arm cycle does so on the host's NEXT
 * serviced request.  Harmless regardless: the host's poll-by-repeat simply
 * retries until that re-arm lands, exactly as it already tolerates any other
 * READY-low stretch, and the heals called from these wait points do not
 * themselves depend on READY being high (bridge_transport_spi_hw_reinit()
 * raises it on success; on a genuine wedge dropping busy() at the top of
 * this whole job already asserted it low, which is the correct state to be
 * in either way).
 *
 * NEVER call cc3501e_hw_link_heal(true) between role-up and this body's own
 * post-role-up reinit (bridge_transport_spi_hw_reinit() above, gated on
 * !role_up_was_latched), and NEVER inside a busy()/reinit() bracket anywhere
 * in this file. */
#define CC3501E_WIFI_HEAL_SLICE_MS 100u

/* Slice an osi_SyncObjWait() into <= CC3501E_WIFI_HEAL_SLICE_MS chunks,
 * calling cc3501e_hw_link_heal(true) between them, so a long association
 * wait no longer freezes the link heals for its whole duration.
 *
 * SLICED AGAINST AN ABSOLUTE DEADLINE (#142 item 5, host review of dfd5280),
 * not a decrementing nominal-slice counter: a heal call can itself take real
 * wall-clock time (the dead-handle heal's SPI_open retry budget is ~123 ms
 * worst case, hal/ti/transport_hw_ti_spi.c's spi_open_and_arm() comment) --
 * decrementing @p total_ms by the NOMINAL slice size ignored that cost, so a
 * heal-heavy wait could overrun @p total_ms by seconds and push this
 * function's whole association phase past its documented ~70 s budget
 * (examples/aen/aen-cc3501e-wedge-postmortem assumes a 75 s outer ceiling).
 * Measuring elapsed_ms against start_ms with cc3501e_hw_uptime_ms() (the
 * SAME idiom this file already uses for assoc_wait_start_ms elsewhere) keeps
 * the WHOLE loop -- osi_SyncObjWait time AND heal time both -- bounded to
 * @p total_ms, same as an unsliced single call would have been.
 *
 * SEMAPHORE-PERSISTENCE PRESERVED: @p sync's own signalled-until-cleared
 * behaviour is untouched by this change -- only the PER-SLICE timeout
 * argument is computed differently; if @p sync was already signalled when a
 * later slice starts, that slice's osi_SyncObjWait() still returns OSI_OK
 * immediately, same as always.
 *
 * Returns OSI_OK the moment @p sync signals (matching plain osi_SyncObjWait's
 * own contract); returns the LAST slice's non-OK return value once
 * @p total_ms elapses with no signal -- callers here only ever test
 * `!= OSI_OK`, so which specific non-OK code survives does not matter. */
static OsiReturnVal_e wifi_assoc_wait_sliced(OsiSyncObj_t *sync, uint32_t total_ms)
{
	const uint32_t start_ms = cc3501e_hw_uptime_ms();
	OsiReturnVal_e rv       = OSI_OK;

	for (;;) {
		const uint32_t elapsed_ms = (uint32_t)(cc3501e_hw_uptime_ms() - start_ms);
		if (elapsed_ms >= total_ms) {
			return rv; /* budget exhausted in real wall-clock time, heal cost included */
		}
		const uint32_t remaining_ms = total_ms - elapsed_ms;
		const uint32_t slice =
		    (remaining_ms < CC3501E_WIFI_HEAL_SLICE_MS) ? remaining_ms : CC3501E_WIFI_HEAL_SLICE_MS;
		rv = osi_SyncObjWait(sync, slice);
		if (rv == OSI_OK) {
			return OSI_OK; /* the event signalled -- stop slicing immediately */
		}
		cc3501e_hw_link_heal(true);
	}
}

/* Slice a ClockP_usleep() delay into <= CC3501E_WIFI_HEAL_SLICE_MS chunks,
 * calling cc3501e_hw_link_heal(true) between them -- same reasoning as
 * wifi_assoc_wait_sliced() above (including the absolute-deadline fix, #142
 * item 5), for the retry's own deny-list/comeback delay (up to
 * CC3501E_WIFI_RETRY_DENYLIST_DELAY_MS, ~11 s). */
static void wifi_sleep_sliced(uint32_t total_ms)
{
	const uint32_t start_ms = cc3501e_hw_uptime_ms();

	for (;;) {
		const uint32_t elapsed_ms = (uint32_t)(cc3501e_hw_uptime_ms() - start_ms);
		if (elapsed_ms >= total_ms) {
			return;
		}
		const uint32_t remaining_ms = total_ms - elapsed_ms;
		const uint32_t slice =
		    (remaining_ms < CC3501E_WIFI_HEAL_SLICE_MS) ? remaining_ms : CC3501E_WIFI_HEAL_SLICE_MS;
		ClockP_usleep(slice * 1000u);
		cc3501e_hw_link_heal(true);
	}
}

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
		wifi_conn_set(
		    (uint8_t)ALP_CC3501E_WIFI_CONN_FAILED, (uint8_t)ALP_CC3501E_WIFI_FAIL_KICK, 0);
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
	 * worker drain (src/worker.c; on a FAILURE exit it still does -- see
	 * cc3501e_hw_wifi_connect_sta_take_reinit() below for the SUCCESS exit, which
	 * as of #106 re-arms the slave itself and skips the drain's copy), but none
	 * BETWEEN the role-up and Wlan_Connect: it went straight into a 30 s
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
	 * A FAILURE exit below used to always need a reinit from somewhere and get it
	 * from the drain, unconditionally.  This one is BEFORE Wlan_Connect; the drain's
	 * runs after worker_execute() returns.  The SUCCESS exit is still different: it
	 * pays a SECOND reinit itself, right before publishing CONNECTED, and that one --
	 * not this one -- is what the drain skips via cc3501e_hw_wifi_connect_sta_take_
	 * reinit() (#106).  EVERY FAILURE exit's drain reinit is now CONDITIONALLY
	 * skipped too (advisor analysis, not bench-proven): see
	 * wifi_connect_fail_mark_skip() above (called at each failure exit, including
	 * the role-up-fail one right below) and cc3501e_hw_wifi_connect_sta_take_
	 * fail_skip(). */
	if (!role_up_was_latched) {
		/* #142 item 4: drop READY BEFORE the reinit -- a role-up is a radio
		 * op like any other and tears the slave's DMA down, so the host must
		 * see the line go LOW across that window instead of clocking a
		 * live-looking READY into a slave whose reinit has not finished
		 * re-arming yet.  SOME other reinit call sites do the same (this
		 * function's own SUCCESS-exit reinit ~2725, src/worker.c's drain
		 * ~1284) -- CORRECTED (#142 item 8, host review of dfd5280): this is
		 * NOT "every other reinit call site", though; cc3501e_hw_wifi_lazy_
		 * start() (~784) and cc3501e_hw_wifi_scan_run() (~1402) reinit with
		 * no preceding busy() at all. */
		cc3501e_bridge_busy();
		bridge_transport_spi_hw_reinit();
	}

	/* #142 item 2: reset the quiet-armed detector's per-call arm gate + fire
	 * cap for THIS connect attempt, right after the point above where this
	 * body's own reinit (if any) last touched the slave -- see that
	 * function's own comment (hal/cc3501e_hw.h) for what it resets and why
	 * here. */
	cc3501e_hw_link_heal_begin_connect();

	if (wifi_rv != CC3501E_HW_OK) {
		/* This exit is reached ONLY when role_up_was_latched was false:
		 * cc3501e_hw_wifi_ensure_sta_role() returns CC3501E_HW_OK immediately
		 * at its own wifi_sta_role_up early-return (above, ~1273) whenever the
		 * role was already up, so a non-OK wifi_rv here means that branch was
		 * NOT taken -- the reinit just above (`if (!role_up_was_latched)`)
		 * always ran right before this exit, unconditionally.  So this is not
		 * a case of "expect no frame": wifi_connect_fail_mark_skip() polls the
		 * same as every other failure exit, and if THAT reinit's own arm
		 * succeeded, host polls land within the window and the skip correctly
		 * fires; if it failed (the slave genuinely dead after a failed
		 * Wlan_Start), no frame lands and the drain's reinit still runs,
		 * unchanged from before this whole fix. */
		wifi_connect_fail_mark_skip();
		wifi_conn_set(
		    (uint8_t)ALP_CC3501E_WIFI_CONN_FAILED, (uint8_t)ALP_CC3501E_WIFI_FAIL_KICK, 0);
		return wifi_rv;
	}
	/* RUN9 BOUNDED RETRY -- outer state shared across the (at most 2) passes of
	 * the loop below.  Bench evidence (Run9, GPE 0.254.9.0, e1m-aen-evk-01,
	 * alp-console connect-first): the earlier ASSOCIATION_REJECTED(30)
	 * non-terminal fix (812649d/074079e) did NOT fix the observed pattern --
	 * connect-first still alternated pass/fail 8 of 16, EVERY rejection
	 * `fail 2 / reason 30`, last vendor event id 2 (WLAN_EVENT_DISCONNECT).
	 * NOT PROOF the reject itself arrived as a DISCONNECT, though: the
	 * failed attempt's own #1437 cleanup issues a Wlan_Disconnect() right
	 * after, which generates a trailing DISCONNECT event that would make
	 * "last event id 2" true regardless of what actually ended the
	 * attempt -- see the ASSOCIATION_REJECTED case's own "RUN9 UPDATE" for
	 * the two candidate explanations this data cannot distinguish between.
	 * CONTROL (this part IS a direct observation, not an inference):
	 * running a clean `wifi disconnect` on the boot BEFORE a power-off made
	 * the NEXT boot associate first try, 2 of 2, then 4 boots in a row
	 * associated -- isolating an UNCLEAN power-off (no deauth sent) as the
	 * trigger.  INFERENCE, not proven from the SDK: the AP still holds this
	 * STA's stale security association from before the power-off and runs a
	 * PMF SA-Query against it, rejecting new associations with status 30
	 * until that check times out -- independent of exactly which vendor
	 * event this cb observed the 30 on.
	 *
	 * So the retry lives HERE, in the connect body, keyed only on "did THIS
	 * attempt's recorded reason end up being 30", rather than in
	 * wifi_event_cb() keyed on a specific event type -- it fires the same way
	 * whichever candidate explanation is the real one, and (see
	 * wifi_retry_delay_ms()'s own comment) picks its wait from
	 * first_pass_reason_event below, snapshotted at the SAME instant as
	 * first_pass_reason_code, so it can wait the shorter comeback delay for
	 * the ASSOCIATION_REJECTED shape without needing to pick one per event
	 * from a stale, already-woken read.
	 *
	 * RUN10 UPDATE: bench 16af840 (GPE 0.254.10.0) showed the FIRST version
	 * of this retry (which excluded AUTHENTICATION_REJECTED entirely,
	 * treating it as a dead end rather than a delay problem) made NO
	 * difference -- same 8-of-16 alternation, rejects still landing at
	 * 3.9-5.2 s, meaning the retry never actually fired.  INFERENCE, from
	 * that timing (unchanged from before the retry existed) plus the code
	 * path (the AUTHENTICATION_REJECTED exclusion was the ONLY thing standing
	 * between "eligible" and "skipped" for a reason-30 rejection): THIS
	 * bench's reject is the AUTHENTICATION_REJECTED(30) candidate, not the
	 * ASSOCIATION_REJECTED-without-the-IE one -- still not a captured trace,
	 * but narrower than Run9's "two candidates, can't tell" state.  The retry
	 * now covers AUTHENTICATION_REJECTED too, waiting out its deny-list
	 * instead of skipping it.
	 *
	 * RUN11 UPDATE: RUN10's fix picked which delay to wait by reading the
	 * MOST RECENT vendor event at wake, which is not necessarily the one
	 * that RECORDED the reason-30 this retry keys on -- see the RUN9 BOUNDED
	 * RETRY comment (above CC3501E_WIFI_RETRY_DENYLIST_EXPIRY_MS) for the
	 * unsound case that opened.  RUN11 closed it by always waiting the ONE,
	 * longer, provably-safe delay instead of picking per event at all.
	 *
	 * RUN12 replaces event_id_this_pass with wifi_last_reason_tag (declared top
	 * of file, see its own comment for the full mechanism): the event is
	 * packed into the SAME word as the reason it accompanies, one store, so
	 * the tag moves only together with the reason -- rather than read back
	 * piecemeal here after the wait wakes.  first_pass_reason_event below
	 * freezes that pair's event half the same way first_pass_reason_code
	 * freezes its reason half (both decoded from ONE load, see the retry
	 * site's own comment further down), so the delay pick is correct again
	 * without giving up the shorter comeback wait RUN11 sacrificed.
	 *
	 * `retried`: this function retries AT MOST ONCE.  `assoc_wait_start_ms`:
	 * uptime at the FIRST Wlan_Connect, the deadline base for EVERY wait this
	 * loop does (including the retry's own), so retrying can never push the
	 * total association phase past CC3501E_WIFI_ASSOC_WAIT_MS -- see the
	 * retry-eligibility check below for exactly how. */
	bool           retried             = false;
	const uint32_t assoc_wait_start_ms = cc3501e_hw_uptime_ms();
	/* Outcome of the loop, published once after it exits (see below) --
	 * consolidates what used to be three separate wifi_conn_set() +
	 * wifi_clear_stale_assoc() call sites (KICK / TIMEOUT / REJECTED) into
	 * one, since every failure exit needs the exact same pair of calls.
	 * connect_reason_code (RUN12) is the value that same call passes as
	 * wifi_conn_set()'s explicit `reason` argument -- resolved into this
	 * local at EACH break site below, never left for wifi_conn_set() to
	 * re-read off the live wifi_last_reason_tag itself (see that function's
	 * own comment for why). */
	uint8_t connect_fail_reason = (uint8_t)ALP_CC3501E_WIFI_FAIL_NONE;
	int16_t connect_reason_code = 0;
	int     connect_rv          = CC3501E_HW_OK;
	/* First pass's outcome, saved ONLY if the retry actually fires -- so a
	 * refused retry (its own Wlan_Connect rejected, see the KICK site below)
	 * can restore and publish THIS instead of manufacturing a fresh KICK/0
	 * that would erase the only real diagnostic the attempt produced. */
	uint8_t first_pass_fail_reason = (uint8_t)ALP_CC3501E_WIFI_FAIL_NONE;
	int16_t first_pass_reason_code = 0;
	/* WHICH event recorded first_pass_reason_code, snapshotted alongside it
	 * (see the retry-eligibility check below) so wifi_retry_delay_ms() can
	 * pick the correct wait -- see that function's own comment. */
	wifi_retry_event_t first_pass_reason_event = WIFI_RETRY_EVENT_NONE;

	for (;;) {
		osi_SyncObjClear(&wifi_event_sync);
		wifi_last_status = 0;
		/* mark_connecting() (SPI-ISR/protocol context, at submit) already cleared
		 * wifi_last_reason_tag once for this attempt, but that was BEFORE the
		 * worker body reached this point -- everything from ensure_sta_role()
		 * through the reinit above runs in between, and a late event from the
		 * PREVIOUS attempt (still in flight on the host-driver thread) can land
		 * in that gap and get recorded there.  Two real shapes, not just one: a
		 * supplicant DISCONNECT carrying any real reason arriving after an
		 * AUTHENTICATION_REJECTED already closed out the old attempt, or a late
		 * ASSOCIATION_REJECTED / AUTHENTICATION_REJECTED arriving after the old
		 * attempt was already declared a TIMEOUT.  Clear it again HERE too,
		 * right next to wifi_last_status's own reset and for the same reason --
		 * immediately before Wlan_Connect, as close to the new attempt's real
		 * start as this body gets.  On the RUN9 RETRY pass this is also step 3
		 * of "clean state before retrying": the recorded reason/event pair
		 * starts this pass at exactly what a fresh, non-retried attempt would
		 * see. */
		wifi_last_reason_tag = 0;
		/* Wlan_Connect(ssid,len,bssid=NULL,secType,pass,passlen,flags=0).  Open
		 * networks pass a NULL/zero-length password. */
		if (Wlan_Connect((const signed char *)ssid,
		                 (int)ssid_len,
		                 NULL,
		                 cc3501e_wifi_sec(security),
		                 (const char *)psk,
		                 (char)psk_len,
		                 0) != 0) {
			/* On the RETRY pass (retried already true here -- it is only ever
			 * set right before the `continue` that starts that pass) this is
			 * almost certainly the SDK's oper-bitmap refusing a new
			 * Wlan_Connect because WLAN_IF_DISCONNECT_IN_PROGRESS was still
			 * set -- CC3501E_WIFI_RETRY_DISCONNECT_WAIT_MS's own wait for the
			 * cleanup's DISCONNECT is best-effort and can time out first
			 * (wlan_if.c ~2334: the bit clears only when that event
			 * dispatches).  That is OPER_IN_PROGRESS, not a fresh KICK, and
			 * publishing FAILED/KICK/0 here would DESTROY the first pass's
			 * real outcome (REJECTED/30) with a less informative one for no
			 * reason -- restore and publish that instead. */
			if (retried) {
				connect_fail_reason = first_pass_fail_reason;
				connect_reason_code = first_pass_reason_code;
			} else {
				connect_fail_reason = (uint8_t)ALP_CC3501E_WIFI_FAIL_KICK;
				connect_reason_code = 0;
			}
			connect_rv = CC3501E_HW_ERR_IO;
			break;
		}
		/* BOUNDED wait for the connect event, against the DEADLINE this whole
		 * association phase (first attempt AND retry) shares -- see
		 * assoc_wait_start_ms above.  This op is WORKER-ROUTED (see protocol.c
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
		 * NON-blocking via CMD_WIFI_STATUS.  DHCP/IP is brought up right after (L3).
		 *
		 * DEADLINE MATH: elapsed_ms is time-since-first-Wlan_Connect (unsigned
		 * subtraction, matching this file's existing cc3501e_hw_uptime_ms()
		 * deltas elsewhere -- e.g. transport_hw_ti_spi.c -- correct as long as
		 * the actual elapsed time is far under the ~49-day wrap).  On the first
		 * pass this is ~0, so wait_ms is the full budget, unchanged from
		 * before.  On the retry pass it is whatever the first reject + the
		 * retry's own overhead already spent, so the retry's wait can only be
		 * SHORTER, never longer, than what remains of the ORIGINAL 30 s --
		 * this is what makes "the deadline is computed at the first
		 * Wlan_Connect" true in code, not just in the comment. */
		{
			const uint32_t elapsed_ms = cc3501e_hw_uptime_ms() - assoc_wait_start_ms;
			const uint32_t wait_ms    = (elapsed_ms < CC3501E_WIFI_ASSOC_WAIT_MS)
			                                ? (CC3501E_WIFI_ASSOC_WAIT_MS - elapsed_ms)
			                                : 0u;
			/* #142: sliced (wifi_assoc_wait_sliced(), <= CC3501E_WIFI_HEAL_SLICE_MS
			 * per slice) instead of one blocking osi_SyncObjWait(), so the link
			 * heals -- cc3501e_hw_link_heal(true), called between slices -- keep
			 * running through the whole wait instead of being frozen for it (up
			 * to 30 s).  Safe here: the host is polling WIFI_STATUS every 50 ms
			 * the whole time and no busy()/reinit() bracket is open across this
			 * wait -- see this function's own top comment (CORRECTED, #142
			 * item 6) for why READY itself is NOT claimed known-good on every
			 * pass, and why that is still harmless. */
			if (wait_ms == 0u || wifi_assoc_wait_sliced(&wifi_event_sync, wait_ms) != OSI_OK) {
				/* No connect event within what was left of the budget.  On the
				 * FIRST pass this is a genuine TERMINAL timeout (was masked as a
				 * retryable IO that looped the host's poll-by-repeat -> -4).  On
				 * the RETRY pass (retried already true here) a timeout is a LESS
				 * informative outcome than the first pass's real REJECTED/30 --
				 * same reasoning, and same restore, as the Wlan_Connect-refused
				 * KICK site above: publish the first pass's result instead of a
				 * fresh TIMEOUT/0 that would erase it. */
				if (retried) {
					connect_fail_reason = first_pass_fail_reason;
					connect_reason_code = first_pass_reason_code;
				} else {
					connect_fail_reason = (uint8_t)ALP_CC3501E_WIFI_FAIL_TIMEOUT;
					/* NOT forced to 0 (e92d626 read the live value here too): a
					 * timed-out wait means no event satisfied THIS wait, but the tag
					 * can still hold whatever the cb last wrote under the
					 * connecting-state gate (a late REJECTED/DISCONNECT that missed
					 * the signal, or a spurious write outside the wait window) --
					 * a single load, decoded once. */
					connect_reason_code = wifi_reason_tag_reason(wifi_last_reason_tag);
				}
				connect_rv = CC3501E_HW_ERR_IO;
				break;
			}
		}
		if (wifi_last_status >= 0) {
			break; /* L2 ASSOCIATED -- fall through to L3/DHCP below. */
		}
		/* FW rejected the association/auth (WLAN_EVENT_CONNECT Status<0, or a
		 * DISCONNECT/ASSOCIATION_REJECTED/AUTHENTICATION_REJECTED event).
		 *
		 * RUN12: load the (reason, event) pair ONCE here, into `reason_tag`,
		 * before either the eligibility check or the terminal-restore decision
		 * further down use it -- see wifi_last_reason_tag's own declaration
		 * comment (top of file) for why decoding both halves from a single
		 * already-loaded local, rather than reading the volatile multiple
		 * times across this whole section, is what keeps them a matched pair
		 * even though the event-cb thread runs at a higher priority and can
		 * preempt between what would otherwise be separate reads. */
		const uint32_t reason_tag = wifi_last_reason_tag;

		/* RUN9/RUN10/RUN11 RETRY-ELIGIBILITY CHECK: for a recorded reason of
		 * exactly 30 (WLAN_STATUS_ASSOC_REJECTED_TEMPORARILY -- "first real
		 * code wins" in wifi_event_cb() already ensured the tag's reason half
		 * holds the FIRST real code THIS attempt saw, not whatever
		 * terminated it), only once per connect call.  The BUDGET check
		 * below still uses CC3501E_WIFI_RETRY_OVERHEAD_MS's worst case
		 * regardless of shape -- see that macro's own comment for why a
		 * SINGLE conservative bound, not a per-event one, is the simpler and
		 * still-correct choice; only the ACTUAL sleep two steps down is
		 * per-event. */
		if (!retried && wifi_reason_tag_reason(reason_tag) ==
		                    (int16_t)CC3501E_WLAN_STATUS_ASSOC_REJECTED_TEMPORARILY) {
			const uint32_t elapsed_ms = cc3501e_hw_uptime_ms() - assoc_wait_start_ms;
			const uint32_t budget_needed_ms =
			    CC3501E_WIFI_RETRY_OVERHEAD_MS + CC3501E_WIFI_RETRY_MIN_WAIT_MS;
			if (elapsed_ms + budget_needed_ms <= CC3501E_WIFI_ASSOC_WAIT_MS) {
				retried                 = true;
				first_pass_fail_reason  = (uint8_t)ALP_CC3501E_WIFI_FAIL_REJECTED;
				first_pass_reason_code  = wifi_reason_tag_reason(reason_tag);
				first_pass_reason_event = wifi_reason_tag_event(reason_tag);
				/* Step 3, "account for the oper-bitmap and disconnect-in-progress
				 * rules": run the SAME #1437 cleanup every OTHER failure exit
				 * uses, then wait (bounded, best-effort) for ITS OWN
				 * WLAN_EVENT_DISCONNECT to dispatch -- see
				 * CC3501E_WIFI_RETRY_DISCONNECT_WAIT_MS's comment for why that is
				 * the SDK's own precondition for a new Wlan_Connect to be
				 * accepted at all, not merely tidiness. */
				osi_SyncObjClear(&wifi_event_sync);
				wifi_clear_stale_assoc();
				(void)osi_SyncObjWait(&wifi_event_sync, CC3501E_WIFI_RETRY_DISCONNECT_WAIT_MS);
				/* Step 2's comeback delay -- picked from the pair already loaded
				 * into `reason_tag` above (RUN12): the short AP-comeback delay
				 * for WIFI_RETRY_EVENT_ASSOCIATION_REJECTED, the long local
				 * deny-list expiry for every other shape (see
				 * wifi_retry_delay_ms()'s own comment for why that is the safe
				 * default) -- long enough for OUR OWN driver's deny-list entry
				 * against this BSSID (if any was even added) to have expired,
				 * so the next Wlan_Connect's scan can consider this BSSID
				 * again. */
				const uint32_t retry_delay_ms =
				    wifi_retry_delay_ms(first_pass_reason_event,
				                        CC3501E_WIFI_RETRY_COMEBACK_DELAY_MS,
				                        CC3501E_WIFI_RETRY_DENYLIST_DELAY_MS);
				/* #142: sliced (wifi_sleep_sliced()) instead of one blocking
				 * ClockP_usleep(), same reasoning as the association wait's own
				 * slicing above -- this delay can run up to
				 * CC3501E_WIFI_RETRY_DENYLIST_DELAY_MS (~11 s).  Safe for the same
				 * reason: the disconnect cleanup above already ran, the host is
				 * still polling WIFI_STATUS (state is still CONNECTING), and no
				 * busy()/reinit() bracket is open -- see this function's own top
				 * comment (CORRECTED, #142 item 6) for why READY is not claimed
				 * known-good here either, and why that is still harmless. */
				wifi_sleep_sliced(retry_delay_ms);
				continue; /* re-issue Wlan_Connect; loop top clears state again. */
			}
		}
		/* Terminal REJECTED -- either not the retry shape, already retried
		 * once, or too little budget left.  Step 4: if this WAS the retry
		 * pass, the tag/wifi_last_status were freshly reset at this pass's
		 * own top and wifi_event_cb()'s existing "first real code wins" rule
		 * applied to THIS pass's own events -- so in the COMMON case this
		 * publishes the RETRY's own outcome, exactly as if it were a fresh
		 * attempt (because from wifi_event_cb()'s perspective, gated only on
		 * wifi_conn_is_connecting() and the tag's reason half being 0, it
		 * was).
		 *
		 * BUT one specific retry-pass outcome is NOT published as-is: see
		 * wifi_retry_should_restore_first_pass()'s own comment (src/
		 * wifi_retry.h) for the full mechanism AND its stated residual -- in
		 * short, reason 3 (WLAN_REASON_DEAUTH_LEAVING) on a retry pass is
		 * USUALLY either OUR OWN pre-retry wifi_clear_stale_assoc()'s
		 * Wlan_Disconnect() echoing back through pDrv->deauthReason (which
		 * has no per-attempt reset, driver_ti_wifi.c:1592-1603 off
		 * cme.c:4159-4164) or hostap's SME give-up timers reaching
		 * sme_deauth() -> deauthenticate (drv_ti_sta_specific.c:400 writes a
		 * fresh 3) after an auth/assoc timeout with nothing else having ended
		 * the attempt first -- either way less informative than the FIRST
		 * pass's real, AP-issued 30, so restore it instead, same pattern as
		 * the refused-retry (KICK) and timed-out-retry sites above.  Reason 0
		 * (nothing recorded this pass at all) restores for the same reason.
		 * NOT ALWAYS CORRECT, though (see wifi_retry.h's RESIDUAL): an AP's
		 * OWN deauth/disassoc can legitimately carry ReasonCode 3
		 * (drv_ti_mlme.c:1476/1521), and a retry pass whose OWN passphrase
		 * the first pass's AUTH-level 30 never actually checked could
		 * genuinely re-fail with a real reason this restore would still
		 * overwrite.  Any OTHER retry-pass reason is a real AP reject code
		 * and is published as-is -- no second retry follows either way. */
		if (retried && wifi_retry_should_restore_first_pass(wifi_reason_tag_reason(reason_tag))) {
			connect_fail_reason = first_pass_fail_reason;
			connect_reason_code = first_pass_reason_code;
		} else {
			connect_fail_reason = (uint8_t)ALP_CC3501E_WIFI_FAIL_REJECTED;
			connect_reason_code = wifi_reason_tag_reason(reason_tag);
		}
		connect_rv = CC3501E_HW_ERR_IO;
		break;
	}

	if (connect_rv != CC3501E_HW_OK) {
		/* wifi_connect_fail_mark_skip() FIRST -- before wifi_conn_set(FAILED)
		 * publishes below, while the host is (or may still be) polling
		 * WIFI_STATUS -- see that helper's own comment for the ordering this
		 * fixes.  On a RETRIED pass this samples its baseline AFTER the retry
		 * branch's OWN mid-loop wifi_clear_stale_assoc() has already run
		 * (further up this function), i.e. after everything the association
		 * attempt itself could do to the slave -- NOT after this block's own
		 * trailing wifi_clear_stale_assoc() a few lines down, which has not run
		 * yet when this is called.  That trailing call's own Wlan_Disconnect()
		 * safety rests on the SAME synchronous-message-queue trace as the
		 * unconditional WIFI_DISCONNECT skip (src/worker.c's wifi_disconnect
		 * group, sourced against worker.c ~1192-1212), not on frames served --
		 * it is not itself covered by this poll. */
		wifi_connect_fail_mark_skip();
		wifi_conn_set(
		    (uint8_t)ALP_CC3501E_WIFI_CONN_FAILED, connect_fail_reason, connect_reason_code);
		/* #1437: leave the NWP ready for the next connect.  Harmless best-effort
		 * double-call on the retry-then-fail path (the retry branch above
		 * already ran this once for the SAME reason it always does): Wlan_Disconnect()
		 * on an already-clear association is exactly the no-op wifi_clear_stale_assoc()'s
		 * own comment already documents. */
		wifi_clear_stale_assoc();
		return connect_rv;
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
		/* #142: one heal per DHCP poll iteration -- CORRECTED (#142 item 7,
		 * host review of dfd5280): this loop's own cadence is
		 * CC3501E_STA_DHCP_POLL_US == 200 ms, which is NOT "well under"
		 * CC3501E_WIFI_HEAL_SLICE_MS (100 ms) -- it is roughly double it.
		 * Still no FURTHER sub-slicing is needed here, though: 200 ms is
		 * itself already a short, fixed, bounded gap (unlike the multi-second
		 * association wait / retry sleep this same heal is sliced for
		 * above), so one heal call per iteration keeps the heals running at
		 * a comparable cadence without the added complexity of slicing an
		 * already-short sleep.  Safe for the same reason as the
		 * association-wait/retry-sleep call sites above: L2 is already
		 * associated (this loop only runs after that), the host is still
		 * polling WIFI_STATUS, and no busy()/reinit() bracket is open across
		 * this loop. */
		cc3501e_hw_link_heal(true);
	}
	if (ip == 0u) {
		/* Associated at L2 but no DHCP lease within the budget -- TERMINAL (there is no
		 * usable IP, so a "connected" report would mislead the host into failing socket
		 * ops).  The host reads this as CONN_FAILED/TIMEOUT via CMD_WIFI_STATUS.
		 *
		 * reason is NOT hardcoded 0 here: state is STILL CONNECTING throughout this
		 * whole DHCP poll (nothing has published a terminal state or CONNECTED yet),
		 * so wifi_event_cb() can still record a real reason into the live tag during
		 * it -- a spontaneous AP deauth/disassoc arriving mid-poll, or a leftover
		 * ASSOCIATION_REJECTED(30) from a comeback the vendor's own retry ultimately
		 * WON (see that case's non-terminal handling) that this L2-success path never
		 * cleared.  Freeze whatever is actually there with a single load -- TAKEN
		 * HERE, BEFORE wifi_connect_fail_mark_skip()'s own poll below, not after:
		 * state is STILL CONNECTING for the ENTIRE duration of that poll too (the
		 * same reason this snapshot has to be a single load in the first place),
		 * so a late event landing inside the poll's window must not be allowed to
		 * change the reason this exit publishes -- matching how the terminal
		 * REJECTED/TIMEOUT exit above already resolves its own connect_reason_code
		 * before doing anything else. */
		const int16_t no_dhcp_reason_code = wifi_reason_tag_reason(wifi_last_reason_tag);
		/* This is the exit run10 P2-01's death most likely took (host review of
		 * 580f748): a WIFI_STATUS verdict of CONN_FAILED/FAIL_TIMEOUT read at
		 * 50.4 s, the link alive at that point, then silence.  The FIRST version
		 * of this fix never called wifi_connect_fail_mark_skip() here at all --
		 * only the terminal REJECTED exit further down did -- so this exit
		 * always paid the drain's unconditional reinit, unchanged.  Apply the
		 * SAME helper here, same ordering (before wifi_conn_set(FAILED)): if a
		 * host frame lands within the window the drain skips its reinit; if not,
		 * it runs as before. */
		wifi_connect_fail_mark_skip();
		wifi_conn_set((uint8_t)ALP_CC3501E_WIFI_CONN_FAILED,
		              (uint8_t)ALP_CC3501E_WIFI_FAIL_TIMEOUT,
		              no_dhcp_reason_code);
		return CC3501E_HW_ERR_IO;
	}

	/* #106 connect flavour: re-arm the SPI slave HERE, before CONNECTED
	 * publishes below, instead of leaving it to src/worker.c's post-body
	 * drain reinit.  wifi_conn_set(CONNECTED) is what the host's WIFI_STATUS
	 * poll is watching for -- a FIXED 50 ms cadence, CC3501E_WIFI_STATUS_POLL_GAP_MS
	 * in chips/cc3501e/cc3501e_wifi.c (NOT the CONFIG_ALP_SDK_CC3501E_POLL_GAP_MIN_MS
	 * knob -- that is the 1 ms-default floor for a DIFFERENT poll, the
	 * poll_by_repeat backoff an already-issued op like WIFI_GET_RSSI retries
	 * on).  Observed on boot 05A (wedged at read 0, on the RSSI the console
	 * performs right after "wifi connected"), and matching run6's S25 / SF07 /
	 * SF09: this function used to publish CONNECTED from below BEFORE the
	 * drain's reinit could run (the drain cannot start it until
	 * worker_execute() -- i.e. this whole function -- returns), so the host
	 * saw CONNECTED, dropped straight to its dense post-connect WIFI_GET_RSSI
	 * poll, and clocked into a slave still down.
	 *
	 * CORRECTED: this used to blame "Wlan_Connect's own DMA teardown" for that
	 * down slave.  That contradicts what src/wifi_connect_fail_skip.h's later
	 * trace establishes: Wlan_Connect() is synchronous but only queues a
	 * message (wlan_if.c ~1272-1296 -> CME_WlanConnect, cme.c ~1517-1624,
	 * ending in pushMsg2Queue()) -- it does not itself touch the bridge's DMA.
	 * The actual disruption mechanism (if any) is in the ASYNCHRONOUS CME
	 * association work that runs afterward on the CME task, which remains
	 * UNTRACED -- see that header for the full citation trail.  Whatever the
	 * mechanism, the SECOND reinit below empirically fixed the observed RSSI
	 * wedge; this comment no longer claims to know why.
	 *
	 * Placed after the LAST radio (Wlan_*) op on this path.  Wlan_Connect and
	 * the association wait above are the last ones.  network_set_up() and the
	 * DHCP-kick loop above are lwIP/netif calls, not Wlan_* NWP calls; INFERRED
	 * (not separately measured) to leave the slave's DMA alone, by analogy with
	 * src/worker.c's socket_control / socket-data skips, which measured that
	 * ARP/SYN/FIN/RST traffic on the SAME transmit path does not need a
	 * reinit -- DHCP traffic reaches the radio over that same host interface,
	 * but nobody has bench-isolated the DHCP-kick loop on its own the way the
	 * socket ops were isolated.
	 *
	 * Re-assert busy() immediately before the reinit, matching how every BLE
	 * HAL body re-asserts it before ITS reinit (src/worker.c's drain comment
	 * explains why: the drain's own busy() bracket around worker_execute() may
	 * already have been cancelled by something else's ready(), though nothing
	 * else in THIS function raises ready() before this point).
	 *
	 * CORRECTED (2026-09-13, post-ae381bc review): this used to say removing
	 * an explicit cc3501e_bridge_ready() call from here (a version of this fix
	 * briefly had one) was necessary to keep worker_reset() ordered before
	 * READY.  That is not what is actually happening, on two counts:
	 *
	 *   1. bridge_transport_spi_hw_reinit() already raises READY itself, as a
	 *      side effect, when the arm succeeds: it calls spi_open_and_arm() ->
	 *      arm_request_header() -> arm_transfer(), and arm_transfer() calls
	 *      cc3501e_bridge_ready() directly on a successful SPI_transfer() queue
	 *      (hal/ti/transport_hw_ti_spi.c ~572), independently of anything this
	 *      function does afterward.  So READY is already HIGH by the time
	 *      wifi_conn_set(CONNECTED) below runs -- REGARDLESS of whether this
	 *      function also calls cc3501e_bridge_ready() explicitly.  The explicit
	 *      call this fix removed was always redundant with what the reinit call
	 *      above already does; removing it changed no observable behaviour.
	 *      Because the pulse in event_ring_push() (off wifi_conn_set's
	 *      EVT_WIFI_CONNECTED push) self-suppresses only while READY reads LOW,
	 *      the attention pulse FIRES on this CONNECTED push either way -- it was
	 *      never suppressed by removing the explicit call.
	 *   2. READY is not held low across this whole function to begin with, so
	 *      there is no "before worker_reset()" window this could have closed.
	 *      The FIRST reinit above (between role-up and Wlan_Connect) already
	 *      raises READY the same way, well before the association wait even
	 *      starts -- that is the whole point of the "so the host CAN clock
	 *      WIFI_STATUS in" comment on that reinit.  From there on READY tracks
	 *      the SPI ISR's own per-transaction arm/re-arm cycle (on_transfer's
	 *      re-arm on every SERVICED request) continuously through the
	 *      association wait and the DHCP loop, not something held low until
	 *      the drain's worker_reset() runs.  A host CONNECT landing in that
	 *      window and being collected against a stale result is therefore a
	 *      possible race that PREDATES this whole #106 change and is not
	 *      opened or closed by it.  If it ever needs closing, the fix is in
	 *      worker_execute()'s own publish critical section (src/worker.c) --
	 *      publish CONNECT/AP_START as IDLE there directly instead of DONE/ERR,
	 *      not by sequencing this body's or the drain's cc3501e_bridge_ready()
	 *      relative to worker_reset().
	 *
	 * cc3501e_hw_wifi_connect_sta_take_reinit() still earns its keep for a
	 * narrower, correct reason: it tells the drain NOT to call
	 * bridge_transport_spi_hw_reinit() a SECOND time for the same event (a real
	 * second SPI_close/SPI_open cycle, not just a redundant GPIO write) -- see
	 * src/worker.c's body_already_reinit / wifi_connect_body_reinit.  The
	 * drain's own subsequent cc3501e_bridge_ready()-or-not (gated on the armed
	 * outcome this handoff carries) is consistent with, and redundant to, what
	 * arm_transfer() already did above; it is not what makes READY correct.
	 * Every FAILURE exit above (bad SSID, role-up fail, Wlan_Connect reject,
	 * association timeout, the no-DHCP-lease exit just above this one) never
	 * reaches here, so take_reinit() (the SUCCESS-flavour handoff) reports
	 * false for them, same as before this whole fix.  That does NOT mean the
	 * drain's post-body reinit runs unconditionally for those exits any more,
	 * though: each of them now calls wifi_connect_fail_mark_skip() of its own
	 * (a SEPARATE, FAILURE-flavour handoff -- cc3501e_hw_wifi_connect_sta_
	 * take_fail_skip(), src/worker.c's connect_fail_skip group), which can
	 * report the drain's reinit skippable too, on its own per-run evidence.
	 * See that helper's own comment above for the full argument. */
	cc3501e_bridge_busy();
	g_connect_reinit_armed   = bridge_transport_spi_hw_reinit();
	g_connect_reinit_pending = true;

	wifi_conn_set((uint8_t)ALP_CC3501E_WIFI_CONNECTED, (uint8_t)ALP_CC3501E_WIFI_FAIL_NONE, 0);
	return CC3501E_HW_OK;
}

int cc3501e_hw_wifi_disconnect(void)
{
	if (!wifi_started) {
		return CC3501E_HW_OK; /* nothing to disconnect */
	}
	/* No "we asked for this" flag to arm here any more -- see
	 * wifi_event_cb()'s DISCONNECT case for why.  Nothing here actually ENFORCES
	 * that a host only calls this from CONNECTED (protocol_wifi.c:87-104 /
	 * worker.c:307-314 dispatch WIFI_DISCONNECT unconditionally) -- passing
	 * g_wifi_conn.reason below, rather than a hardcoded 0, is what makes this
	 * correct regardless: that field is 0 whenever state is CONNECTED (wifi_conn_set()
	 * clears it there) and the last FROZEN reason otherwise, so a WIFI_DISCONNECT
	 * republishes exactly what was already true, no matter which state it is
	 * actually called from. */
	const int disconnect_rv = Wlan_Disconnect(WLAN_ROLE_STA, NULL);
	/* Issue #144: this is the OTHER write site that can leave
	 * pDrv->deauthReason == 3 for a LATER connect attempt to inherit -- see
	 * wifi_own_disconnect_issued's own declaration comment.  Set regardless
	 * of disconnect_rv, same reasoning as wifi_clear_stale_assoc()'s own
	 * call: a non-zero return here means the SDK's own
	 * set_cond_in_process_wlan_discconnect() found a disconnect ALREADY
	 * in-flight and refused this one, not that no disconnect happened at
	 * all -- the in-flight one still runs and can still write the stale 3. */
	wifi_own_disconnect_issued = true;
	if (disconnect_rv != 0) {
		return CC3501E_HW_ERR_IO;
	}
	/* Host-requested teardown succeeded: mirror the state into the latch and
	 * queue an async EVT_WIFI_DISCONNECTED (wifi_conn_set does both).  reason is
	 * g_wifi_conn.reason itself (the FROZEN value from whatever terminal
	 * wifi_conn_set() call last ran), NOT the live tag and NOT a hardcoded 0 --
	 * see this function's own top comment for why, and hal/cc3501e_hw.h's
	 * cc3501e_hw_wifi_last_reason() contract for what a host is meant to read
	 * out of this republish. */
	wifi_conn_set((uint8_t)ALP_CC3501E_WIFI_DISCONNECTED,
	              (uint8_t)ALP_CC3501E_WIFI_FAIL_NONE,
	              g_wifi_conn.reason);
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
	/* #106 skip-gating fix: capture wifi_started BEFORE lazy_start() below can
	 * flip it -- see cc3501e_hw_wifi_get_rssi_take_reinit_skip() / worker.c's
	 * rssi_read for what this decides.  Only when Wi-Fi was ALREADY started
	 * does this call's radio op reduce to JUST the short Wlan_Get below, with
	 * no reinit anywhere in this body -- the shape the #106 run6/run7 evidence
	 * describes.  If Wi-Fi was NOT yet started, lazy_start() runs Wlan_Start()
	 * and ITS OWN reinit (this file, cc3501e_hw_wifi_lazy_start(), ~line 308),
	 * then THROWS AWAY that reinit's armed/not-armed return value -- so nothing
	 * here knows whether the slave came back up, and letting the drain skip
	 * its reinit on top of that would raise READY unconditionally over an
	 * unknown state (the #1133 condition).  Set the handoff unconditionally
	 * here (before either radio call below can fail) so every exit of this
	 * function -- success or CC3501E_HW_ERR_IO -- reports the same
	 * already-started fact to the drain. */
	const bool wifi_was_already_started = wifi_started;
	g_rssi_reinit_skip_ok               = wifi_was_already_started;
	g_rssi_reinit_skip_pending          = true;

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
