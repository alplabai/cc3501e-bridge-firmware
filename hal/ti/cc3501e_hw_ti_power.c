/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * cc3501e-bridge HAL: TI backend -- power policy (CMD_POWER_POLICY, 0x62)
 * via the CC35xx (WFF3) Power driver.
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

#include <ti/drivers/Power.h> /* Power_setConstraint/Policy (pulls PowerWFF3.h via DeviceFamily_CC35XX) */

#ifdef CC3501E_WIFI
#include <wlan_if.h> /* Wlan_Set -- the RADIO half of the power policy */
#endif

#include "alp/protocol/cc3501e.h"

#include "../cc3501e_hw.h"
#include "cc3501e_hw_ti_internal.h" /* cc3501e_hw_wifi_sta_role_up (cross-TU seam, see #6/#5's guard) */

/* --------------------------------------------------------------- */
/* Power policy (CMD_POWER_POLICY, 0x62) -- real CC35xx Power driver. */
/*                                                                   */
/* The host's coarse policy preset maps onto the CC35xx (WFF3) Power  */
/* manager: the idle-loop Power policy function (Power_setPolicy +    */
/* Power_enablePolicy) plus a balanced set of operational constraints */
/* (Power_setConstraint / Power_releaseConstraint).  Grounded in      */
/* <ti/drivers/Power.h> (Power_setConstraint/releaseConstraint/       */
/* setPolicy/enablePolicy) and <ti/drivers/power/PowerWFF3.h>         */
/* (PowerWFF3_DISALLOW_SLEEP/_IDLE, PowerWFF3_doWFI,                  */
/* PowerWFF3_sleepPolicy) -- the latter is auto-included by Power.h    */
/* under DeviceFamily_CC35XX.                                         */
/*                                                                   */
/* Constraints are REFERENCE-COUNTED per id by the Power manager       */
/* (PowerWFF3.c constraintCounts[]), and the bridge SPI driver itself  */
/* transiently sets/releases PowerWFF3_DISALLOW_SLEEP around each      */
/* transfer -- so this HAL owns AT MOST ONE long-lived reference per   */
/* constraint id and tracks it in pp_constraints_held, releasing the   */
/* previous policy's references before declaring the new ones.  That   */
/* keeps our count balanced (Power_releaseConstraint asserts a         */
/* non-zero count) and never disturbs the SPI driver's own count. */

/* Bitmask (1 << PowerWFF3_DISALLOW_*) of the constraints this HAL currently
 * holds on the host policy's behalf.  Starts empty (boot default = whatever the
 * SysConfig PowerWFF3_Config selected); each apply re-derives the desired set. */
static uint8_t pp_constraints_held;

/* Declare @id (a PowerWFF3_DISALLOW_* constraint) on the policy's behalf if not
 * already held; idempotent so re-applying the same policy is a no-op. */
static void pp_hold_constraint(uint8_t id)
{
	const uint8_t bit = (uint8_t)(1u << id);
	if ((pp_constraints_held & bit) == 0u) {
		Power_setConstraint(id); /* WFF3: always Power_SOK */
		pp_constraints_held |= bit;
	}
}

/* Release @id if this HAL holds it (balanced against pp_hold_constraint so the
 * Power manager's per-id count never underflows -- it asserts count != 0). */
static void pp_release_constraint(uint8_t id)
{
	const uint8_t bit = (uint8_t)(1u << id);
	if ((pp_constraints_held & bit) != 0u) {
		Power_releaseConstraint(id);
		pp_constraints_held &= (uint8_t)~bit;
	}
}

/* ---- Radio power save -------------------------------------------------------
 *
 * The Power_setPolicy() work below only governs the WFF3 CORE.  On a Wi-Fi part
 * the dominant term is the RADIO: an associated station that never enters power
 * save keeps its receiver up continuously, which costs orders of magnitude more
 * than anything the core's WFI/SLEEP state can save.  Nothing in this firmware
 * ever called Wlan_Set(), so every policy -- including DEEP_SLEEP -- left the
 * radio fully awake and the presets differed only in core state.
 *
 * Three independent knobs, all via Wlan_Set (see wlan_if.h):
 *   WLAN_SET_POWER_SAVE       station PS mode (ACTIVE / AUTO / POWER_SAVE)
 *   WLAN_SET_POWER_MANAGEMENT sleep authorisation (ALWAYS_ACTIVE / ELP)
 *   WLAN_SET_LSI              long sleep interval -- wake every Nth DTIM
 *
 * LSI is what finally separates LOW_POWER from DEEP_SLEEP.  Before this the two
 * presets were documented as sharing one reachable state because WFF3 exposes a
 * single core SLEEP tier; on the radio side they are genuinely different.
 *
 * LATENCY IS THE TRADE.  Waking only every Nth DTIM means inbound frames queue
 * at the AP until the next wake, so DEEP_SLEEP adds hundreds of ms of inbound
 * latency and will cut throughput hard.  That is the point of the preset, and it
 * is why BALANCED -- not a low-power mode -- stays the default. */

/* Latched so a policy set BEFORE the radio exists still lands: Wlan_Set only
 * works once a role is up, and the host may configure power at any time. */
static uint8_t  pp_policy_latched = ALP_CC3501E_PP_BALANCED;
static uint32_t pp_idle_ms_latched;
/* Has the HOST ever set a policy?  Until it does, a STA role runs ACTIVE rather
 * than the BALANCED default -- see pp_apply_radio_effective()'s comment for the
 * measurement behind that, and note the host's own policy still wins the moment
 * it sends one. */
static bool pp_host_set_policy;
/* Set by the SPI-dispatch ISR, consumed by the TASK -- see cc3501e_hw_power_service(). */
static volatile bool pp_radio_dirty;
/* Separate from pp_radio_dirty: the CORE half runs ONLY on an explicit host
 * CMD_POWER_POLICY.  A Wi-Fi role-up used to reach it too -- cc3501e_hw_ti_wifi.c
 * used to call a reapply helper unconditionally after a successful
 * Wlan_RoleUp(STA) that set pp_radio_dirty, and the service ran pp_apply_core()
 * on the BALANCED default that pp_policy_latched initialises to.  A plain
 * `wifi connect`, with no POWER_POLICY ever sent, therefore flipped the part from
 * "the core never idled" to "the sleep policy runs on every idle".  Issue #14.
 * (The STA role-up path no longer touches pp_radio_dirty at all -- see
 * cc3501e_hw_power_apply_radio_now() -- so this specific trigger is gone, but
 * pp_core_dirty stays separate from pp_radio_dirty regardless, since the core
 * half must still run ONLY for an explicit host policy.) */
static volatile bool pp_core_dirty;
/* Result of the last TASK-side apply, surfaced via cc3501e_hw_power_radio_ok()
 * because POWER_POLICY has no async-result opcode of its own. */
static volatile bool pp_radio_ok = true;

/* Map idle_ms_before_sleep onto a DTIM count.  A DTIM period is typically ~100 ms
 * (beacon 102.4 ms, DTIM 1); treat the host's idle budget as "how long may we
 * stay asleep" and clamp to the field's uint8_t range.  This finally gives
 * idle_ms_before_sleep a meaning -- it was previously accepted and discarded
 * because PowerWFF3 has no idle-hysteresis setter. */
static uint8_t pp_idle_ms_to_dtims(uint32_t idle_ms)
{
	uint32_t dtims = idle_ms / 100u;

	if (dtims < 2u) {
		dtims = 2u; /* N_DTIM below 2 is just DTIM */
	}
	if (dtims > 255u) {
		dtims = 255u;
	}
	return (uint8_t)dtims;
}

/* Apply the CORE half of @p policy.  TASK CONTEXT ONLY.
 *
 * Power_setPolicy() swaps the function pointer the idle loop runs and
 * Power_enablePolicy() re-arms it.  Those are configuration calls, not runtime
 * ones: driving them from the SPI-dispatch ISR races the idle loop that may be
 * executing the very policy being replaced.
 *
 * That race is issue #1683.  With BLE enabled, applying a preset intermittently
 * wedged the whole device -- the policy call timed out and the bridge went to
 * PING -> -5, unrecoverable without a reset, at a point that moved between runs.
 * It is NOT the radio half: the wedge survived skipping every Wlan_Set() call.
 * BLE does not cause it either, it just loads the shared HIF enough to lose the
 * race reliably -- which is why a Wi-Fi-only build looked fine for so long.
 *
 * The constraint helpers were never the problem: they are balanced by
 * pp_constraints_held, and Power_setConstraint/releaseConstraint are ISR-safe. */
static void pp_apply_core(uint8_t policy)
{
	/* Enable the configured policy ONCE.  PowerWFF3_config already names
	 * PowerWFF3_sleepPolicy as policyFxn, but nothing enables it at init, so
	 * before the first POWER_POLICY the core never idled at all. */
	static bool policy_enabled;

	if (!policy_enabled) {
		Power_enablePolicy();
		policy_enabled = true;
	}

	/* CONSTRAINTS ONLY -- deliberately no Power_setPolicy() here.
	 *
	 * PowerWFF3_sleepPolicy already expresses all four presets on its own: it
	 * "considers active constraints ... the first goal is to enter SLEEP; if that
	 * is not appropriate ... the secondary goal is the IDLE state; if that is
	 * disallowed ... the policy will fallback and simply invoke WFI"
	 * (PowerWFF3.h).  Holding DISALLOW_SLEEP + DISALLOW_IDLE therefore gives
	 * exactly what PowerWFF3_doWFI gives, with no policy swap.
	 *
	 * Swapping the policy function at runtime is what made #1683 intermittent.
	 * Power_setPolicy() replaces the pointer the idle loop invokes, so it races
	 * whatever the idle task is doing -- moving the call from the SPI-dispatch ISR
	 * onto this task narrowed that window but could not close it, because the idle
	 * loop is concurrent with EVERY task.  Not calling it at all closes it by
	 * construction.  Power_setConstraint/releaseConstraint are safe to call
	 * anytime and are the mechanism the policy is documented to read. */
	switch (policy) {
	case ALP_CC3501E_PP_PERFORMANCE:
		/* Lowest latency: forbid SLEEP and IDLE, so the policy falls all the way
		 * back to WFI -- any peripheral IRQ (the bridge SPI CS, a GPIO edge)
		 * wakes it immediately. */
		pp_hold_constraint(PowerWFF3_DISALLOW_SLEEP);
		pp_hold_constraint(PowerWFF3_DISALLOW_IDLE);
		break;
	case ALP_CC3501E_PP_BALANCED:
	case ALP_CC3501E_PP_LOW_POWER:
	case ALP_CC3501E_PP_DEEP_SLEEP:
		/* Release IDLE only.  DISALLOW_SLEEP is held for EVERY preset, because the
		 * CC35xx SLEEP state is one this bridge cannot come back from (#14):
		 *
		 *  - SWRU626 7.1.2: "The MCU domain is powered off ... Modules without
		 *    retention are reset and need to be reconfigured when exiting SLEEP",
		 *    and Table 7-1's Host M33 Sleep row: "All switchable power domains are
		 *    OFF (Core/AAOD)".  SPI0 and the uDMA live in AAOD.
		 *  - SWRU626 3.8.1.1 enumerates every selectable wake publisher in
		 *    HOSTMCU_AON.CFGWICSNS.VAL[17:0] -- ELP timer, GPIO wake src 0/1,
		 *    doorbell 0..7, nab_host_irq, ble_rfc_gpo_8_irq, RTC, two DebugSS
		 *    sources, secured_error_irq, core wdt irq.  THERE IS NO SPI PUBLISHER.
		 *
		 * The CC3501E is the SPI SLAVE and can never initiate a transfer, so if
		 * SLEEP is entered the only wire the host owns cannot wake it: asserting
		 * SS0 and clocking SCLK is lost silently, with no NAK and no slave-side
		 * timeout.  Recovery is a WIFI_EN/nRESET cold cycle -- Table 7-3's
		 * 20 ms + 380 ms + 500 ms plus the Wi-Fi association, BLE links, socket
		 * state and any OTA session.
		 *
		 * IDLE is the deepest state the bridge survives.  Table 7-1 Host M33 Idle:
		 * "all supplies and clocks are enabled ... the host domain is enabled and
		 * initialized including the peripherals however the Host M33 clock is
		 * gated" -- so SPI0 stays clocked and SPI0_IRQ wakes the M33 through the
		 * NVIC.
		 *
		 * DEEP_SLEEP is therefore a RADIO-ONLY preset now: it still takes the
		 * N-DTIM long sleep interval in pp_apply_radio(), which is the term with a
		 * published saving (SWRS343A 6.11, 975 uA at DTIM=1 against 6.10's 60 mA
		 * continuous listen).  The core-SLEEP saving it used to reach for
		 * (6.13: 520 uA vs 22 mA) is not reachable safely, and TI publishes no
		 * IDLE current at all, so the trade it was making was never quantified.
		 * The wire contract in alp-sdk's <alp/protocol/cc3501e.h> should say so;
		 * that header lives in the other repo. */
		pp_hold_constraint(PowerWFF3_DISALLOW_SLEEP);
		pp_release_constraint(PowerWFF3_DISALLOW_IDLE);
		break;
	default:
		return;
	}
}

/* Apply the radio half of @p policy.  Best-effort: returns 0 when the radio is
 * not up yet (the caller latches and re-applies after role-up) and never fails
 * the whole policy call, so a host that sets power before Wi-Fi still gets the
 * core-side policy applied.
 *
 * ps (WLAN_SET_POWER_SAVE -> CME_WlanSetPSMode(), wlan_if.c ~1716) and pm
 * (WLAN_SET_POWER_MANAGEMENT -> ctrlCmdFw_SetSleepAuth(), wlan_if.c ~1722)
 * are NOT the same knob and are NOT coupled in the vendor SDK: ps is the
 * STA's OWN power-save mode (ACTIVE / AUTO_PS / POWER_SAVE) and only matters
 * once that STA is associated as a client; pm is DEVICE-WIDE sleep
 * authorisation (ctrlCmdFw_SetSleepAuth sends ACX_SLEEP_AUTH straight to
 * firmware, control_cmd_fw.c ~1771, with no per-role scoping at all) -- it is
 * what actually gates whether the NWP may doze between EITHER role's duty
 * cycles.  A soft-AP has to beacon continuously and cannot tolerate that
 * doze (see cc3501e_hw_wifi_ap_start()'s own ALWAYS_ACTIVE force before its
 * Wlan_RoleUp(AP)), so pm is what THIS function must never hand a sleeping
 * value to while an AP is up -- ps, being STA-scoped, is independent and may
 * still follow the host's chosen preset. */
#ifdef CC3501E_WIFI
static bool pp_apply_radio(uint8_t policy, uint32_t idle_ms)
{
	uint8_t               ps;
	WlanPowerManagement_e pm;
	bool                  want_lsi = false;

	switch (policy) {
	case ALP_CC3501E_PP_PERFORMANCE:
		ps = (uint8_t)WLAN_STATION_ACTIVE_MODE;
		pm = POWER_MANAGEMENT_ALWAYS_ACTIVE_MODE;
		break;
	case ALP_CC3501E_PP_BALANCED:
		ps = (uint8_t)WLAN_STATION_AUTO_PS_MODE;
		pm = POWER_MANAGEMENT_ELP_MODE;
		break;
	case ALP_CC3501E_PP_LOW_POWER:
		/* Sleep between DTIMs but wake on EVERY one: still responsive to
		 * downlink traffic within one beacon period. */
		ps = (uint8_t)WLAN_STATION_POWER_SAVE_MODE;
		pm = POWER_MANAGEMENT_ELP_MODE;
		break;
	case ALP_CC3501E_PP_DEEP_SLEEP:
		ps       = (uint8_t)WLAN_STATION_POWER_SAVE_MODE;
		pm       = POWER_MANAGEMENT_ELP_MODE;
		want_lsi = true;
		break;
	default:
		return false;
	}

	/* OVERRIDE, regardless of @p policy or who chose it (including an
	 * explicit host CMD_POWER_POLICY -- power management is device-wide, so a
	 * host asking for BALANCED/LOW_POWER/DEEP_SLEEP is asking to sleep the
	 * WHOLE NWP, and this HAL cannot honour that verbatim while a soft-AP is
	 * relying on the NWP staying awake to keep beaconing (#1562)).  cc3501e_hw_
	 * wifi_ap_start()'s own inline force only covers the moment RoleUp(AP)
	 * happens; without this, a LATER policy apply here -- the tick draining an
	 * explicit host policy, or the STA role-up path's synchronous apply while
	 * AP is also up -- could still pull pm back to ELP.  ps is untouched: it
	 * is STA-scoped (see this function's header comment) and stays whatever
	 * @p policy chose. */
	if (cc3501e_hw_radio_role() == (uint8_t)ALP_CC3501E_ROLE_WIFI_AP) {
		pm = POWER_MANAGEMENT_ALWAYS_ACTIVE_MODE;
	}

	bool ok = (Wlan_Set(WLAN_SET_POWER_SAVE, &ps) >= 0);

	ok = (Wlan_Set(WLAN_SET_POWER_MANAGEMENT, &pm) >= 0) && ok;

	if (want_lsi || policy == ALP_CC3501E_PP_LOW_POWER) {
		WlanLongSleepInterval lsi = { 0 };

		lsi.WakeUpEvent    = want_lsi ? (uint8_t)WAKE_UP_EVENT_N_DTIM : (uint8_t)WAKE_UP_EVENT_DTIM;
		lsi.ListenInterval = want_lsi ? pp_idle_ms_to_dtims(idle_ms) : 1u;
		ok                 = (Wlan_Set(WLAN_SET_LSI, &lsi) >= 0) && ok;
	}
	return ok;
}
#else  /* !CC3501E_WIFI -- no Wi-Fi host driver, so no Wlan_Set to call */
/* Without this the whole ti build fails to link when built without
 * -WifiHostDriver / -Ble: `undefined symbol Wlan_Set` out of this file.  Found
 * while fixing the same class for the socket seams (#7), which left this as the
 * last remaining undefined in that configuration.  The CORE half above is
 * radio-independent and still applies; only the radio half is absent. */
static bool pp_apply_radio(uint8_t policy, uint32_t idle_ms)
{
	(void)policy;
	(void)idle_ms;
	return true; /* nothing to apply -> not a failure */
}
#endif /* CC3501E_WIFI */

/* Compute the effective radio policy and apply it via pp_apply_radio(), then
 * publish pp_radio_ok.  ONE copy of the effective-policy rule, shared by:
 *   - cc3501e_hw_power_apply_radio_now(), the SYNCHRONOUS caller (STA
 *     role-up, before Wlan_Connect starts the DHCP-critical window), and
 *   - cc3501e_hw_power_service()'s TASK-context drain (an explicit host
 *     CMD_POWER_POLICY set before or after a role is up).
 *
 * TASK CONTEXT ONLY, same rule as pp_apply_core (#1683): Wlan_Set() is a
 * blocking vendor radio call and must never run off the SPI-dispatch ISR.
 *
 * NO STA ROLE UP -> DO NOT CALL pp_apply_radio() AT ALL (vendor-confirmed,
 * SDK 10.10.01.08).  WLAN_SET_POWER_SAVE routes to CME_WlanSetPSMode()
 * (wlan_if.c ~1716), which returns SUCCESS in BOTH of two ways that never
 * touch firmware: its cached mode already matches the request (cme.c ~1131,
 * queues nothing), or it queues CME_MESSAGE_ID_PS_SET, whose handler silently
 * drops the whole message when no STA interface is up (cme.c ~3020-3037,
 * "STA Role is not up").  pp_apply_radio()'s Wlan_Set() return-value check
 * cannot see either case, so calling it with no STA role up would mark
 * pp_radio_ok true for a policy that silently never reached the radio.
 *
 * Skipping the call here does NOT leave pp_radio_dirty set for a later drain
 * to retry -- cc3501e_hw_power_service() (below) already clears it, in its own
 * critical section, before ever calling this function, so by the time this
 * guard runs the flag is gone either way.  The real reason a policy set before
 * any role exists is not lost: cc3501e_hw_wifi_ensure_sta_role() calls
 * cc3501e_hw_power_apply_radio_now() -- this same function -- UNCONDITIONALLY
 * on every STA role-up (not gated on pp_radio_dirty at all), re-reading
 * pp_policy_latched / pp_host_set_policy fresh each time.  So the first STA
 * role-up after a policy was set with no role up always re-applies it for
 * real, regardless of what this early-return did or did not leave behind.
 *
 * One optimistic-read consequence, PRE-DATING this guard (the old
 * `radio_up ? ok : true` did the same): pp_radio_ok reads true for a policy
 * sent while no STA role exists yet, even though nothing was actually applied
 * -- "not attempted" and "applied and confirmed" are not distinguished on the
 * wire.  Not new here; noted, not fixed. */
static void pp_apply_radio_effective(void)
{
	if (!cc3501e_hw_wifi_sta_role_up()) {
		pp_radio_ok = true; /* nothing attempted -- not a failure */
		return;
	}

	/* An un-configured role defaults to PERFORMANCE/ACTIVE, not the BALANCED
	 * default.  BALANCED maps to WLAN_STATION_AUTO_PS_MODE, and applying that
	 * before Wlan_Connect puts the station to sleep exactly when it has to hear
	 * a DHCP OFFER.  The AP buffers broadcast and multicast until a DTIM
	 * beacon, and a sleeping station on a marginal link misses beacons and
	 * therefore misses DTIMs: measured on e1m-aen-evk-01 at -78 dBm, the
	 * station associated every time, its DISCOVERs left every time
	 * (DHCP_STATE_SELECTING, tries = 5), and it leased on roughly one attempt
	 * in four.
	 *
	 * The host's power API stays authoritative: once it sends a POWER_POLICY,
	 * pp_host_set_policy latches and that policy is applied verbatim, including
	 * BALANCED.  This only changes what an un-configured station defaults to,
	 * where the alternative is a bridge that cannot reliably get an address.
	 *
	 * This used to also test cc3501e_hw_radio_role() != WIFI_AP, to exclude a
	 * running AP from the override.  That test broke the moment BOTH roles were
	 * up: cc3501e_hw_radio_role() reports AP over STA by design ("AP outranks
	 * STA" -- see its own comment), so a scan/connect bringing STA up alongside
	 * an already-running AP read as "AP", took the pp_policy_latched (BALANCED)
	 * branch instead of the unconfigured default -- and, before pp_apply_radio()
	 * grew its OWN unconditional AP-up override (below it, its header comment),
	 * that also meant the AP's forced ALWAYS_ACTIVE was pulled back to
	 * BALANCED's ELP.  Reaching this line already proves a STA role is up (the
	 * guard above), so testing STA-up directly -- rather than radio_role()'s
	 * AP-outranks-STA summary -- is what "an STA role is being brought up or
	 * is up" (not "cc3501e_hw_radio_role() alone") means here.  AP safety no
	 * longer depends on which branch @p eff takes: pp_apply_radio() forces pm
	 * to ALWAYS_ACTIVE whenever AP is up regardless of the policy passed in, so
	 * this function only has to pick the right ps/pm PRESET for the STA side
	 * -- the AP-safe pm floor is enforced one layer down. */
	const uint8_t eff =
	    pp_host_set_policy ? pp_policy_latched : (uint8_t)ALP_CC3501E_PP_PERFORMANCE;
	const bool ok = pp_apply_radio(eff, pp_idle_ms_latched);

	pp_radio_ok = ok;
}

/* SYNCHRONOUS radio-policy apply -- called from cc3501e_hw_wifi_ensure_sta_role()
 * right after Wlan_RoleUp(STA) succeeds and before it returns, so the effective
 * policy (PERFORMANCE/ACTIVE for an unconfigured STA) is in place BEFORE
 * Wlan_Connect starts association + the DHCP lease poll -- not after, once
 * cc3501e_hw_tick() next drains cc3501e_hw_power_service().  See the ordering
 * bug this closes in cc3501e_hw_wifi_ensure_sta_role()'s comment.
 *
 * TASK CONTEXT ONLY (see pp_apply_radio_effective()'s #1683 note) -- verified
 * safe: every caller of ensure_sta_role() is worker-routed off the SPI-dispatch
 * ISR (src/worker.c: the ISR only submits/polls a job; the blocking HAL body
 * runs in worker_run_pending(), called from main()'s bringup-task loop), so
 * this always runs on the task, never the ISR. */
void cc3501e_hw_power_apply_radio_now(void)
{
	pp_apply_radio_effective();
}

/* worker_critical_enter/exit: declared here rather than pulled from a shared
 * header, matching hal/ti/cc3501e_nimble_host.c's own local prototype for the
 * same pair -- there is no header that owns them.  worker.c declares WEAK
 * no-op defaults (correct for the native/stub build); hal/ti/cc3501e_hw_ti.c
 * (this same TU's platform-lifecycle sibling) provides the STRONG override
 * that actually masks interrupts (PRIMASK save/restore) on real silicon --
 * see its comment for why PRIMASK save/restore, not a bare enable. */
unsigned long worker_critical_enter(void);
void          worker_critical_exit(unsigned long key);

/* TASK-context drain for the latched radio policy.  Called from cc3501e_hw_tick().
 * Applying with no STA role up is not an error -- pp_apply_radio_effective()'s
 * own guard (#5) makes that a clean early-return.  This function still clears
 * pp_radio_dirty below regardless (the flag does NOT survive an early-return),
 * but that is harmless: cc3501e_hw_wifi_ensure_sta_role() re-applies the
 * latched policy unconditionally on the first STA role-up either way -- see
 * pp_apply_radio_effective()'s comment. */
void cc3501e_hw_power_service(void)
{
	/* Read-and-clear pp_core_dirty/pp_radio_dirty as ONE atomic step (#7).  Two
	 * separate unprotected reads/clears raced cc3501e_hw_set_power_policy(),
	 * which runs in SPI-DISPATCH (ISR) context and can set either flag between
	 * this function's two statements -- a POWER_POLICY landing in that exact
	 * window had its pp_core_dirty=true observed here (do_core reads true) but
	 * then WIPED by this function's own `pp_core_dirty = false;` before
	 * pp_apply_core() ever ran for it, losing the core half of that policy
	 * silently.  worker_critical_enter()/worker_critical_exit() is the TREE'S
	 * EXISTING masking primitive for exactly this ISR-vs-task publish/read
	 * race (src/worker.c's own header comment; also used by event_ring.c) --
	 * short enough to hold masked (two volatile reads + two writes), unlike
	 * pp_apply_core()/pp_apply_radio() below, which must NOT run masked (#1683,
	 * this function's own header comment). */
	const unsigned long key = worker_critical_enter();
	if (!pp_radio_dirty && !pp_core_dirty) {
		worker_critical_exit(key);
		return;
	}
	const bool do_core  = pp_core_dirty;
	const bool do_radio = pp_radio_dirty;
	pp_radio_dirty      = false;
	pp_core_dirty       = false;
	worker_critical_exit(key);

	/* Core FIRST, then radio -- both on this task, never in the ISR (#1683).
	 * The core half runs ONLY for an explicit host policy, never because a radio
	 * role came up (#14). */
	if (do_core) {
		pp_apply_core(pp_policy_latched);
	}
	if (!do_radio) {
		return;
	}

	pp_apply_radio_effective();
}

bool cc3501e_hw_power_radio_ok(void)
{
	return pp_radio_ok;
}

int cc3501e_hw_set_power_policy(uint8_t policy, uint8_t wake_events, uint32_t idle_ms_before_sleep)
{
	/* Validate per the header contract: an all-zero wake_events bitmap is only
	 * meaningful for the non-sleeping presets (PERFORMANCE / BALANCED); a
	 * low-power preset with NO declared wake source would idle the device with
	 * no way back, so reject it up front (the host must keep at least
	 * ALP_CC3501E_WAKE_HOST_SPI for a low-power policy). */
	if (wake_events == ALP_CC3501E_WAKE_NONE &&
	    (policy == ALP_CC3501E_PP_LOW_POWER || policy == ALP_CC3501E_PP_DEEP_SLEEP)) {
		return CC3501E_HW_ERR_INVAL;
	}

	/* wake_events: the routed sources (HOST_SPI / GPIO_IRQ / BLE / Wi-Fi) wake
	 * the core through their own still-clocked peripheral interrupts while in the
	 * WFF3 SLEEP state -- the validation above is the load-bearing use of the
	 * bitmap.  A per-source SLEEP wake-MASK has no public SDK surface: the WFF3
	 * Power driver hardwires its sleep wake sources (RTC + CSYSPWRUPREQ) in
	 * Power_init/PowerWFF3_sleepPolicy and neither <ti/drivers/Power.h> nor
	 * PowerWFF3.h exposes a Power_setWakeup()/configure-wake API (GPIO.h offers
	 * only GPIO_CFG_SHUTDOWN_WAKE_*, a per-pin SHUTDOWN -- not SLEEP -- knob
	 * applied at GPIO config time, not here). */
	/* deferred: per-bit wake_events -> HW SLEEP wake mask -- no PowerWFF3 wake-source API.
	 * (Still true for the CORE.  The RADIO's wake behaviour IS now configured,
	 * via WLAN_SET_LSI's WakeUpEvent -- see pp_apply_radio().) */

	/* idle_ms_before_sleep: PowerWFF3_sleepPolicy derives the sleep decision from
	 * the time until the next scheduled ClockP event vs the SLEEP transition
	 * latency (PowerWFF3_TOTALTIMESLEEP); a value of 0 means "use that policy
	 * default", which is exactly what running the stock policy does.  A nonzero
	 * minimum-idle threshold cannot be programmed: PowerWFF3.h exposes no
	 * idle-hysteresis setter, only the fixed latency constants. */
	/* Core policy is set; now the radio -- the dominant term.  Latch first so a
	 * policy set before Wlan_RoleUp() is re-applied once the STA role comes up
	 * -- either by cc3501e_hw_wifi_ensure_sta_role()'s own synchronous
	 * cc3501e_hw_power_apply_radio_now() call, or by the next
	 * cc3501e_hw_power_service() drain if pp_radio_dirty (set just below) is
	 * still true when the role finally does exist. */
	pp_policy_latched  = policy;
	pp_host_set_policy = true;
	pp_idle_ms_latched = idle_ms_before_sleep;

	/* Latch the radio half for the TASK.  This function runs in SPI-DISPATCH (ISR)
	 * context, and Wlan_Set() is a blocking vendor radio call -- the same reason
	 * handle_sock_recv() cannot call lwip_recv() and the worker seam exists at all.
	 *
	 * Neither half may run here.  Wlan_Set() is a blocking vendor radio call, and
	 * Power_setPolicy()/Power_enablePolicy() race the idle loop (#1683).  Both are
	 * deferred to cc3501e_hw_power_service(), which the bringup task drains via
	 * cc3501e_hw_tick().  Bench-measured with either inline: every preset returned
	 * -4 and the bridge itself went to PING -> -5.
	 *
	 * CONSEQUENCE FOR THE WIRE CONTRACT: a RESP_OK to POWER_POLICY means QUEUED,
	 * not APPLIED -- the same semantic OTA_BEGIN turned out to have.  A host that
	 * needs the realised state polls it; see cc3501e_hw_power_radio_ok(). */
	pp_core_dirty  = true; /* explicit host request -- the ONLY path to the core half */
	pp_radio_dirty = true;

	return CC3501E_HW_OK;
}
