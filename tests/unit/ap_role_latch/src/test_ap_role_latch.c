/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * ap_role_latch.h: pending -> up only via publish; ap_stop clears pending; a
 * failed ap_start never publishes.  (The drain ordering itself is covered by
 * test_worker_ap_publish.)
 */
#include <zephyr/ztest.h>

#include "ap_role_latch.h"

ZTEST_SUITE(ap_role_latch, NULL, NULL, NULL, NULL, NULL);

ZTEST(ap_role_latch, test_success_is_pending_not_up_until_publish)
{
	ap_role_latch_t l = { 0 };
	ap_role_latch_started(&l, true);
	zassert_true(l.pending, "pending after a successful start");
	zassert_false(l.up, "NOT up before the drain's reinit");
	ap_role_latch_publish(&l);
	zassert_true(l.up, "up after publish");
	zassert_false(l.pending, "pending consumed");
}

ZTEST(ap_role_latch, test_failed_start_never_publishes)
{
	ap_role_latch_t l = { 0 };
	ap_role_latch_started(&l, false);
	ap_role_latch_publish(&l);
	zassert_false(l.up, "a failed ap_start must not publish");
	zassert_false(l.pending, "nor leave anything pending");
}

ZTEST(ap_role_latch, test_stop_clears_pending_so_late_publish_is_noop)
{
	ap_role_latch_t l = { 0 };
	ap_role_latch_started(&l, true);
	ap_role_latch_stopped(&l);
	ap_role_latch_publish(&l);
	zassert_false(l.up, "stop before publish: the role never comes up");
}

ZTEST(ap_role_latch, test_stop_clears_up)
{
	ap_role_latch_t l = { 0 };
	ap_role_latch_started(&l, true);
	ap_role_latch_publish(&l);
	ap_role_latch_stopped(&l);
	zassert_false(l.up, "stop takes the role down");
}

ZTEST(ap_role_latch, test_publish_without_start_is_noop)
{
	ap_role_latch_t l = { 0 };
	ap_role_latch_publish(&l);
	zassert_false(l.up, "nothing pending -> nothing published");
}
