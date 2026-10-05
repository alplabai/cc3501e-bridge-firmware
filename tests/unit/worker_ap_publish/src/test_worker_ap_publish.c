/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * AP_START ordering (v0.9.1, finding 3): the AP role must be published only
 * AFTER the worker drain's post-body bridge_transport_spi_hw_reinit().
 * GET_DIAG_INFO reports the role from the ISR, and the host fires its next
 * request the moment it reads role == AP -- publishing from inside
 * cc3501e_hw_wifi_ap_start() put that request into the reinit window
 * (bench E1M-AEN803 v0.9.0: SOCK_OPEN rc=-4 in 3-5/20 at 0 ms, 0/20 at 200+ ms).
 *
 * Built with CC3501E_WIFI (real-firmware shape: submit only queues, the drain
 * runs the body) and --wrap on the three calls whose ORDER is the assertion.
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/ztest.h>

#include "alp/protocol/cc3501e.h"
#include "worker.h"

enum { EV_AP_START = 1, EV_REINIT = 2, EV_PUBLISH = 3 };
static int events[8];
static int n_events;

static void note(int ev)
{
	if (n_events < 8) {
		events[n_events++] = ev;
	}
}

int __wrap_cc3501e_hw_wifi_ap_start(const uint8_t *ssid,
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
	note(EV_AP_START);
	return 0; /* CC3501E_HW_OK */
}

bool __wrap_bridge_transport_spi_hw_reinit(void)
{
	note(EV_REINIT);
	return true;
}

void __wrap_cc3501e_hw_wifi_ap_role_publish(void)
{
	note(EV_PUBLISH);
}

ZTEST(cc3501e_worker_ap_publish, test_role_published_after_drain_reinit)
{
	uint8_t req[sizeof(alp_cc3501e_wifi_connect_t) + 3] = { 0 };
	req[0]                                              = 1u; /* ssid_len */
	req[1]                                              = 1u; /* psk_len */
	req[2]                                              = 1u; /* WPA2 */
	req[4]                                              = 'a';
	req[5]                                              = 'b';

	zassert_equal(worker_submit_payload(ALP_CC3501E_CMD_WIFI_AP_START, req, (uint16_t)sizeof req),
	              1,
	              "submit accepts IDLE -> QUEUED");
	worker_run_pending();

	zassert_equal(n_events, 3, "ap_start, reinit, publish -- exactly once each");
	zassert_equal(events[0], EV_AP_START, "body runs first");
	zassert_equal(events[1], EV_REINIT, "drain reinit completes BEFORE the role is published");
	zassert_equal(events[2], EV_PUBLISH, "role published last");
}

ZTEST(cc3501e_worker_ap_publish, test_other_ops_never_publish_ap_role)
{
	zassert_equal(worker_submit_payload(ALP_CC3501E_CMD_WIFI_AP_STOP, NULL, 0u), 1, "submit");
	worker_run_pending();
	for (int i = 0; i < n_events; i++) {
		zassert_true(events[i] != EV_PUBLISH, "only AP_START publishes the AP role");
	}
}

static void reset(void *fixture)
{
	(void)fixture;
	worker_init();
	n_events = 0;
	memset(events, 0, sizeof events);
}

ZTEST_SUITE(cc3501e_worker_ap_publish, NULL, NULL, reset, NULL, NULL);
