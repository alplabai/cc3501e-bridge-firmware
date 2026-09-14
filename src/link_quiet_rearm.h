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
 * KNOWN TO BE ACTIVELY POLLING (see cc3501e_hw_ti_wifi.c's wait-point call
 * sites for that qualifier -- it is load-bearing), is itself evidence worth
 * acting on.
 *
 * FIRST DESIGN (host review of b3dc1e2, REJECTED): an independent
 * background task called this every 10 ms unconditionally.  That let a
 * reinit fire inside windows the rest of the codebase guarantees are
 * reinit-free.  See cc3501e_hw_ti.c's own top comment and hal/ti/
 * cc3501e_hw_ti_wifi.c's wait-point call sites for the fix: this detector is
 * called ONLY from cc3501e_hw_wifi_connect_sta()'s own wait points, sliced
 * to <= 100 ms -- NEVER from the unconditional idle tick, which still runs
 * the other four, evidence-based heals but never evaluates this one.
 *
 * SECOND DESIGN (dfd5280, host review, ALSO A BUG): the once-per-episode
 * latch below used to clear on "quiet_ms has dropped below the wall-clock
 * time elapsed since the heal fired" -- reasoning that bridge_transport_spi_
 * quiet_ms() resetting proves a phase completed.  It does not: quiet_ms
 * ALSO resets when the heal's OWN reinit calls spi_open_and_arm() (by
 * design -- see that stamp's own comment in hal/ti/transport_hw_ti_spi.c,
 * needed so a stale pre-OTA-flash stamp is never inherited).  So the heal's
 * own reinit satisfied the latch-clear condition against ITSELF: fire,
 * re-arm (quiet_ms -> ~0), wait rearm_ms, "quiet_ms dropped" reads true
 * again (nothing but the SAME re-arm ever happened), fire again.  Measured
 * against a genuinely deaf slave: 9 fires in 30 s.
 *
 * THE FIX: the latch clears ONLY on @p xfer_count moving --
 * bridge_transport_spi_xfer_count(), bumped ONLY in on_transfer() (real
 * host-driven transfer completions), never in spi_open_and_arm()/
 * _hw_release()/_hw_suspend() the way the quiet-ms stamp is.  quiet_ms
 * still decides WHEN to fire (so a stale pre-flash stamp still never causes
 * a false fire); xfer_count alone decides when the ALREADY-FIRED latch may
 * clear.  The two signals are deliberately different because they answer
 * different questions -- see bridge_transport_spi_xfer_count()'s own
 * comment (src/transport.h) for the fuller argument.
 *
 * REMAINING RISK (host review of dfd5280, item 2 -- NOT fully closed here):
 * 3 s of quiet_ms does not by itself prove a DEAF slave -- the host can
 * simply stop polling mid-body (examples/aen/aen-cc3501e-wedge-postmortem
 * does exactly this: a 2000 ms connect timeout, then deliberate silence, to
 * capture a wedge for forensics).  This header cannot see "why" the host
 * stopped polling; it only sees quiet_ms and xfer_count.  The caller
 * (cc3501e_hw_wifi_connect_sta(), hal/ti/cc3501e_hw_ti_wifi.c) closes MOST
 * of that gap with two call-site gates this pure function does not (and
 * cannot) enforce on its own: (a) the detector may not even be consulted
 * until at least one REAL transfer has landed since this connect body's own
 * reinit, and (b) at most one fire is allowed per connect() call.  See that
 * function's own comments for the residual this still does not close.
 *
 * PURE ARITHMETIC, SILICON-FREE, same split as sock_prefetch_arm.h /
 * wifi_connect_fail_skip.h: cc3501e_hw_link_heal() (TI-SDK-only, needs
 * bridge_transport_spi_quiet_ms()/_at_idle_header()/_xfer_count() and
 * cc3501e_hw_uptime_ms(), never linked into a host build) is the sole real
 * caller; this header owns only the once-per-episode DECISION so it is
 * unit-testable where the real HAL cannot be linked at all.
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
	bool     healed_pending;        /* a heal already fired for this episode  */
	uint32_t heal_fired_xfer_count; /* xfer_count snapshot AT the fire       */
} link_quiet_rearm_state_t;

/*
 * link_quiet_rearm_tick -- should the caller fire ONE
 * bridge_transport_spi_hw_reinit() right now?
 *
 *   @p st             this detector's latch state; mutated in place.
 *   @p at_idle_header  true iff the slave is parked in PH_REQ_HEADER (the
 *                      idle boundary; see bridge_transport_spi_
 *                      at_idle_header()).  Not itself the safety argument
 *                      (see this header's own top comment) -- a
 *                      defense-in-depth check so this function never fires
 *                      mid-frame even if a caller ever gets the window
 *                      wrong: PH_REQ_HEADER never coexists with an armed
 *                      reply/payload phase (on_transfer() only sets phase
 *                      away from PH_REQ_HEADER in the SAME step it arms the
 *                      next phase), so this one check already covers what a
 *                      separate reply_armed input would have (dropped -- it
 *                      was dead weight).
 *   @p quiet_ms        bridge_transport_spi_quiet_ms(): milliseconds since
 *                      the slave last heard ANYTHING from the host, OR last
 *                      re-armed/tore down its own handle.  Decides WHEN to
 *                      fire; NOT used to decide when the latch may clear
 *                      (see this header's own top comment for why).
 *   @p rearm_ms        the quiet threshold (CC3501E_LINK_QUIET_REARM_MS at
 *                      the real call site; a parameter here for testability).
 *   @p xfer_count       bridge_transport_spi_xfer_count(): count of REAL
 *                      host-driven transfer completions.  Decides when an
 *                      already-fired latch may clear -- ONLY when this has
 *                      moved since the fire, never on quiet_ms alone (see
 *                      this header's own top comment for the bug that
 *                      shipped without this).
 *
 * The caller decides WHETHER to call this at all -- see this header's own
 * top comment: only from cc3501e_hw_wifi_connect_sta()'s wait points, never
 * from the unconditional idle tick.  Returns true (and ARMS the latch) the
 * first time @p at_idle_header && @p quiet_ms >= @p rearm_ms, with the latch
 * not already pending.  Returns false every other call, INCLUDING every call
 * of the same wedge episode after the first fire, until @p xfer_count
 * differs from its value AT the fire -- i.e. until a REAL host transfer has
 * landed since then. */
static inline bool link_quiet_rearm_tick(link_quiet_rearm_state_t *st,
                                         bool                      at_idle_header,
                                         uint32_t                  quiet_ms,
                                         uint32_t                  rearm_ms,
                                         uint32_t                  xfer_count)
{
	if (st->healed_pending) {
		if (xfer_count != st->heal_fired_xfer_count) {
			st->healed_pending = false; /* a REAL transfer landed since the heal fired */
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

	st->healed_pending        = true;
	st->heal_fired_xfer_count = xfer_count;
	return true;
}

#endif /* CC3501E_BRIDGE_LINK_QUIET_REARM_H */
