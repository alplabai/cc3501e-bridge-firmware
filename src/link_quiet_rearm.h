/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * cc3501e-bridge firmware: the quiet-armed re-arm DECISION behind #142's
 * link-healer task.
 *
 * ===================== WHY THIS EXISTS =====================
 * #142's root cause: an SPI slave armed in PH_REQ_HEADER whose DMA never
 * actually completes (an "armed-but-deaf" slave) is invisible to every
 * OTHER self-heal in hal/ti/cc3501e_hw_ti.c's cc3501e_hw_link_tick() --
 * g_resync_count does not move (no header is being misframed),
 * g_arm_fail_count does not move (the last arm succeeded),
 * bridge_transport_spi_is_dead() is false (the handle is fine), and
 * bridge_transport_spi_phase_stalled() is false (it only watches an ARMED
 * reply/payload phase -- PH_REQ_HEADER is deliberately excluded there
 * because it is the legitimate "waiting for the host" idle state that can
 * wait forever).  That last exclusion is exactly the blind spot this file
 * closes: PARKED-AND-UNARMED for an unusually long time, on a link that is
 * not merely idle (a healthy idle host still gets serviced on its next
 * poll; a wedged one never will), is itself evidence worth acting on IF the
 * wait is long enough to rule out an ordinarily-quiet host.
 *
 * FIX: cc3501e_hw_link_tick() calls link_quiet_rearm_tick() every 10 ms with
 * the observed phase/reply_armed/quiet-duration state.  It returns true
 * (fire a bridge_transport_spi_hw_reinit()) the FIRST time the slave has
 * been parked at PH_REQ_HEADER, unarmed, for >= CC3501E_LINK_QUIET_REARM_MS
 * continuously -- then LATCHES so it does not fire again every following
 * 10 ms tick while the wedge persists (a reinit storm would itself be a new
 * failure mode: bridge_transport_spi_hw_release()'s own comment already
 * documents that SPI_close without a preceding SPI_transferCancel carries a
 * possible DMA leak, so re-triggering every 10 ms on a wedge that does not
 * clear is not a safe substitute for firing once).  The latch only clears
 * once a NEW phase has genuinely completed since the heal fired -- proven
 * without needing the raw g_last_xfer_ms timestamp (which never leaves
 * hal/ti/transport_hw_ti_spi.c): if quiet_ms has become SMALLER than the
 * wall-clock time elapsed since the heal fired, the underlying last-xfer
 * timestamp must have advanced past the heal, i.e. a phase completed.
 *
 * MUST NOT BREAK #106: a reinit on a slave that is mid-frame (armed) is
 * hazardous.  This detector only ever fires with reply_armed == false AND
 * at_idle_header == true -- the same idle frame BOUNDARY every other
 * self-heal in this file already treats as the one safe point to tear the
 * slave down, never mid-transaction.
 *
 * PURE ARITHMETIC, SILICON-FREE, same split as sock_prefetch_arm.h /
 * wifi_connect_fail_skip.h: cc3501e_hw_link_tick() (TI-SDK-only, needs
 * bridge_transport_spi_quiet_ms()/_phase()/_reply_armed() and
 * cc3501e_hw_uptime_ms(), never linked into a host build) is the sole real
 * caller; this header owns only the once-per-episode DECISION so it is
 * unit-testable where the real link task cannot be linked at all.
 * ============================================================
 */

#ifndef CC3501E_BRIDGE_LINK_QUIET_REARM_H
#define CC3501E_BRIDGE_LINK_QUIET_REARM_H

#include <stdbool.h>
#include <stdint.h>

/* One quiet-rearm episode's latch state.  Zero-initialise (a plain
 * `= {0}` / static storage) -- healed_pending starts false, so the first
 * tick behaves exactly like every later "not currently latched" tick. */
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
 *   @p reply_armed     true iff a reply/payload phase is currently armed
 *                      (bridge_transport_spi_reply_armed()) -- mutually
 *                      exclusive with @p at_idle_header in practice, both
 *                      passed so the pure function does not have to assume
 *                      that invariant.
 *   @p quiet_ms        bridge_transport_spi_quiet_ms(): milliseconds since
 *                      the slave last heard ANYTHING from the host.
 *   @p now_ms          cc3501e_hw_uptime_ms(), sampled once per caller tick
 *                      (passed in, not read here, so this stays pure).
 *   @p rearm_ms        the quiet threshold (CC3501E_LINK_QUIET_REARM_MS at
 *                      the real call site; a parameter here for testability).
 *   @p gated           true while polled (whole-boot OTA update mode) or
 *                      quiesced (an OTA flush owns the slave in NORMAL
 *                      mode) -- the caller must not touch the slave, and
 *                      this function leaves the latch untouched too, so a
 *                      long OTA session does not itself masquerade as "a
 *                      phase completed" when gating lifts.
 *
 * Returns true (and ARMS the latch) the first time @p at_idle_header &&
 * !@p reply_armed && @p quiet_ms >= @p rearm_ms, with the latch not already
 * pending.  Returns false every other tick, INCLUDING every tick of the
 * same wedge episode after the first fire (the latch), until @p quiet_ms
 * drops below the wall-clock time elapsed since the heal fired -- which
 * only happens once a phase has genuinely completed since then (see this
 * header's own top comment for the arithmetic). */
static inline bool link_quiet_rearm_tick(link_quiet_rearm_state_t *st,
                                         bool                      at_idle_header,
                                         bool                      reply_armed,
                                         uint32_t                  quiet_ms,
                                         uint32_t                  now_ms,
                                         uint32_t                  rearm_ms,
                                         bool                      gated)
{
	if (gated) {
		return false;
	}

	if (st->healed_pending) {
		const uint32_t since_heal = (uint32_t)(now_ms - st->heal_fired_at_ms);
		if (quiet_ms < since_heal) {
			st->healed_pending = false; /* a phase completed since the heal fired */
		} else {
			return false; /* same episode -- do not re-fire */
		}
	}

	if (!at_idle_header || reply_armed) {
		return false; /* not the idle-and-unarmed shape this detector watches */
	}
	if (quiet_ms < rearm_ms) {
		return false; /* not quiet long enough yet */
	}

	st->healed_pending   = true;
	st->heal_fired_at_ms = now_ms;
	return true;
}

#endif /* CC3501E_BRIDGE_LINK_QUIET_REARM_H */
