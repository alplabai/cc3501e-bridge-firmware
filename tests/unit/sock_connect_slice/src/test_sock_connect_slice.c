/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * sock_connect_slice_ms(): the slice arithmetic behind the non-blocking
 * SOCK_CONNECT wait (finding 4, v0.9.1).  The lwIP loop itself is TI-SDK-only;
 * covered by the bench, not here.
 */
#include <stdint.h>
#include <zephyr/ztest.h>

#include "sock_connect_slice.h"

ZTEST_SUITE(sock_connect_slice, NULL, NULL, NULL, NULL, NULL);

ZTEST(sock_connect_slice, test_full_slice_while_budget_remains)
{
	zassert_equal(sock_connect_slice_ms(0u, 21000u, 100u), 100u, "first slice is a full slice");
	zassert_equal(sock_connect_slice_ms(20800u, 21000u, 100u), 100u, "exactly one slice left");
}

ZTEST(sock_connect_slice, test_last_slice_is_clamped_to_budget)
{
	zassert_equal(sock_connect_slice_ms(20950u, 21000u, 100u), 50u, "never overshoots the budget");
}

ZTEST(sock_connect_slice, test_zero_when_budget_spent)
{
	zassert_equal(sock_connect_slice_ms(21000u, 21000u, 100u), 0u, "at budget -> give up");
	zassert_equal(sock_connect_slice_ms(99999u, 21000u, 100u), 0u, "past budget -> give up");
}

ZTEST(sock_connect_slice, test_slices_cover_budget_with_heal_between_each)
{
	uint32_t elapsed = 0u, slices = 0u, s;
	while ((s = sock_connect_slice_ms(elapsed, 21000u, 100u)) != 0u) {
		elapsed += s; /* worst case: every select() times out */
		slices++;     /* one cc3501e_hw_link_heal(false) per timed-out slice */
	}
	zassert_equal(elapsed, 21000u, "loop terminates exactly at the budget");
	zassert_equal(slices, 210u, "heals run every 100 ms, not once per 12-21 s");
}
