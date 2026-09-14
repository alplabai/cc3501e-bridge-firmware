/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * cc3501e-bridge firmware: the quiet-armed re-arm DECISION behind #142's
 * connect-body link heal.
 *
 * ===================== WHY THIS EXISTS =====================
 * #142's root cause: an SPI slave armed in PH_REQ_HEADER whose DMA never
 * actually completes (an "armed-but-deaf" slave) is invisible to every
 * OTHER self-heal in hal/ti/cc3501e_hw_ti.c's cc3501e_hw_link_heal() --
 * g_resync_count does not move (no header is being misframed),
 * g_arm_fail_count does not move (the last arm succeeded),
 * bridge_transport_spi_is_dead() is false (the handle is fine), and
 * bridge_transport_spi_phase_stalled() is false (it only watches an ARMED
 * reply/payload phase -- PH_REQ_HEADER is deliberately excluded there
 * because it is the legitimate "waiting for the host" idle state that can
 * wait forever).  That last exclusion is exactly the blind spot this file
 * closes: PARKED-AND-UNARMED for an unusually long time, WHILE THE HOST IS
 * KNOWN TO BE ACTIVELY POLLING (see the safety paragraph below -- that
 * qualifier is load-bearing), is itself evidence worth acting on.
 *
 * FIRST DESIGN (host review of b3dc1e2, REJECTED): an independent
 * background task called this every 10 ms unconditionally.  That let a
 * reinit fire inside windows the rest of the codebase guarantees are
 * reinit-free -- mid BLE_SCAN, inside Wlan_Start's host-off bracket, right
 * after an OTA erase's quiesce(false) (inheriting a last-xfer stamp that
 * could be tens of seconds stale from BEFORE the flash op), or simply on an
 * ordinarily-idle host/console session that never trips any OTHER self-heal.
 * The "#106 idle-boundary safe point" framing that comment used to justify
 * this was FALSE: PH_REQ_HEADER alone does not prove the moment is safe --
 * only WHO is calling and WHEN does.  See cc3501e_hw_ti.c's own top comment
 * and hal/ti/cc3501e_hw_ti_wifi.c's wait-point call sites for the actual
 * fix: this detector is called ONLY from cc3501e_hw_wifi_connect_sta()'s own
 * wait points, sliced to <= 100 ms, in windows that function itself
 * establishes are reinit-safe (READY high, host polling WIFI_STATUS every
 * 50 ms, no body-owned reinit pending, not inside a busy() bracket) -- NEVER
 * from the unconditional idle tick.  The idle tick still runs the other four
 * heals (evidence-based: dead handle, resync burst, arm failure, reply
 * stall); it does NOT evaluate this one, so a quiet console/idle host can
 * never trip it.
 *
 * The once-per-episode LATCH: fires ONE bridge_transport_spi_hw_reinit()
 * the first time the slave has been parked at PH_REQ_HEADER, quiet, for
 * >= CC3501E_LINK_QUIET_REARM_MS -- then does not re-fire on the same
 * episode (a reinit storm would itself be a new failure mode:
 * bridge_transport_spi_hw_release()'s own comment already documents that
 * SPI_close without a preceding SPI_transferCancel carries a possible DMA
 * leak).  The latch clears once a NEW phase has genuinely completed since
 * the heal fired -- proven without needing the raw g_last_xfer_ms timestamp
 * (which never leaves hal/ti/transport_hw_ti_spi.c): if quiet_ms has become
 * SMALLER than the wall-clock time elapsed since the heal fired, the
 * underlying last-xfer timestamp must have advanced past the heal.
 * g_last_xfer_ms is stamped on every on_transfer() callback entry AND on
 * every spi_open_and_arm()/hw_release()/hw_suspend() (a re-arm or a
 * deliberate teardown both reset the quiet clock), so this reinit itself
 * never inherits a stale quiet_ms from before it ran.
 *
 * PURE ARITHMETIC, SILICON-FREE, same split as sock_prefetch_arm.h /
 * wifi_connect_fail_skip.h: cc3501e_hw_link_heal() (TI-SDK-only, needs
 * bridge_transport_spi_quiet_ms()/_phase() and cc3501e_hw_uptime_ms(), never
 * linked into a host build) is the sole real caller; this header owns only
 * the once-per-episode DECISION so it is unit-testable where the real HAL
 * cannot be linked at all.
 * ============================================================
 */

#ifndef CC3501E_BRIDGE_LINK_QUIET_REARM_H
#define CC3501E_BRIDGE_LINK_QUIET_REARM_H

#include <stdbool.h>
#include <stdint.h>

/* One quiet-rearm episode's latch state.  Zero-initialise (a plain
 * `= {0}` / static storage) -- healed_pending starts false, so the first
 * call behaves exactly like every later "not currently latched" call. */
typedef struct {
	bool     healed_pending;   /* a heal already fired for this episode */
	uint32_t heal_fired_at_ms; /* cc3501e_hw_uptime_ms() when it fired    */
} link_quiet_rearm_state_t;

/*
 * link_quiet_rearm_tick -- should the caller fire ONE
 * bridge_transport_spi_hw_reinit() right now?
 *
 *   @p st             this detector's latch state; mutated in place.
 *   @p at_idle_header  true iff the slave is parked in PH_REQ_HEADER (the
 *                      idle boundary; see bridge_transport_spi_phase()).
 *                      Not itself the safety argument (see this header's
 *                      own top comment) -- a defense-in-depth check so this
 *                      function never fires mid-frame even if a caller ever
 *                      gets the window wrong: PH_REQ_HEADER never coexists
 *                      with an armed reply/payload phase (on_transfer()
 *                      only sets phase away from PH_REQ_HEADER in the SAME
 *                      step it arms the next phase), so this one check
 *                      already covers what a separate reply_armed input
 *                      would have (dropped -- it was dead weight).
 *   @p quiet_ms        bridge_transport_spi_quiet_ms(): milliseconds since
 *                      the slave last heard ANYTHING from the host, OR last
 *                      re-armed/tore down its own handle (spi_open_and_arm/
 *                      hw_release/hw_suspend all stamp it too).
 *   @p now_ms          cc3501e_hw_uptime_ms(), sampled once per caller call
 *                      (passed in, not read here, so this stays pure).
 *   @p rearm_ms        the quiet threshold (CC3501E_LINK_QUIET_REARM_MS at
 *                      the real call site; a parameter here for testability).
 *
 * The caller decides WHETHER to call this at all -- see this header's own
 * top comment: only from cc3501e_hw_wifi_connect_sta()'s wait points, never
 * from the unconditional idle tick.  Returns true (and ARMS the latch) the
 * first time @p at_idle_header && @p quiet_ms >= @p rearm_ms, with the latch
 * not already pending.  Returns false every other call, INCLUDING every call
 * of the same wedge episode after the first fire (the latch), until
 * @p quiet_ms drops below the wall-clock time elapsed since the heal fired --
 * which only happens once a phase has genuinely completed, or the slave has
 * been re-armed/torn down, since then (see this header's own top comment for
 * the arithmetic). */
static inline bool link_quiet_rearm_tick(link_quiet_rearm_state_t *st,
                                         bool                      at_idle_header,
                                         uint32_t                  quiet_ms,
                                         uint32_t                  now_ms,
                                         uint32_t                  rearm_ms)
{
	if (st->healed_pending) {
		const uint32_t since_heal = (uint32_t)(now_ms - st->heal_fired_at_ms);
		if (quiet_ms < since_heal) {
			st->healed_pending = false; /* a phase completed since the heal fired */
		} else {
			return false; /* same episode -- do not re-fire */
		}
	}

	if (!at_idle_header) {
		return false; /* not the idle shape this detector watches */
	}
	if (quiet_ms < rearm_ms) {
		return false; /* not quiet long enough yet */
	}

	st->healed_pending   = true;
	st->heal_fired_at_ms = now_ms;
	return true;
}

#endif /* CC3501E_BRIDGE_LINK_QUIET_REARM_H */
