/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * cc3501e-bridge firmware: the AP-role publish latch behind
 * cc3501e_hw_wifi_ap_start() / _ap_role_publish() / _ap_stop()
 * (hal/ti/cc3501e_hw_ti_wifi.c).  Pure bookkeeping so the ordering rule is
 * host-testable: a successful ap_start only marks the role PENDING; the worker
 * drain publishes it (up) after its post-body SPI reinit; ap_stop clears both;
 * a failed ap_start changes nothing.  Rationale and bench numbers live on
 * cc3501e_hw_wifi_ap_role_publish() in hal/cc3501e_hw.h.
 */
#ifndef CC3501E_AP_ROLE_LATCH_H
#define CC3501E_AP_ROLE_LATCH_H

#include <stdbool.h>

typedef struct {
	bool pending; /* ap_start succeeded, reinit not yet confirmed */
	bool up;      /* published: what GET_DIAG_INFO may report */
} ap_role_latch_t;

static inline void ap_role_latch_started(ap_role_latch_t *l, bool ok)
{
	if (ok) {
		l->pending = true;
	}
}

static inline void ap_role_latch_publish(ap_role_latch_t *l)
{
	if (l->pending) {
		l->pending = false;
		l->up      = true;
	}
}

static inline void ap_role_latch_stopped(ap_role_latch_t *l)
{
	l->pending = false;
	l->up      = false;
}

#endif /* CC3501E_AP_ROLE_LATCH_H */
