/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * Unit tests for sock_prefetch_arm.h's sock_prefetch_should_arm() -- the
 * MAJOR 4 fix (host review of 1118c99): hal/ti/cc3501e_hw_ti_sock.c's
 * cc3501e_hw_sock_prefetch(handle, true) used to re-arm the single prefetch
 * ring for a NEW handle unconditionally, even while a DIFFERENT handle was
 * already armed and holding pumped-but-unserved bytes -- silently dropping
 * them.  This is the pure arm/no-arm DECISION split out of that TI-SDK-only
 * (lwIP, CC3501E_WIFI) file, the same way sock_recv_commit.h's tail/
 * uncommitted arithmetic is split out of it, so it is host-testable where
 * the ring itself cannot be linked at all. */

#include <stdint.h>
#include <zephyr/ztest.h>

#include "sock_prefetch_arm.h"

ZTEST_SUITE(sock_prefetch_arm, NULL, NULL, NULL, NULL, NULL);

ZTEST(sock_prefetch_arm, test_nothing_armed_may_arm)
{
	zassert_true(sock_prefetch_should_arm(0u, 100u), "ring free -> may arm for any handle");
}

ZTEST(sock_prefetch_arm, test_same_handle_may_rearm)
{
	zassert_true(sock_prefetch_should_arm(100u, 100u),
	             "already armed for THIS handle -> legitimate no-op re-arm");
}

ZTEST(sock_prefetch_arm, test_different_handle_may_not_steal_the_ring)
{
	zassert_false(sock_prefetch_should_arm(100u, 200u),
	              "a DIFFERENT handle already holds the ring -> must not re-arm for it");
}

ZTEST(sock_prefetch_arm, test_disarmed_then_new_handle_may_arm)
{
	/* Mirrors cc3501e_hw_sock_close()'s own sequence: disarm (fd_plus1 -> 0)
	 * frees the ring for a LATER connect on a different handle. */
	zassert_true(sock_prefetch_should_arm(0u, 200u),
	             "after a disarm (armed == 0), a later connect on ANY handle may arm");
}
