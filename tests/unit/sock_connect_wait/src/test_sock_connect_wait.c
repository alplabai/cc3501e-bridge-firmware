/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * sock_connect_run(): every branch of the non-blocking SOCK_CONNECT loop behind
 * a fake lwIP.  Bench-only remainder: the real lwip_* calls and lwIP's own SYN
 * give-up timing.
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/ztest.h>

#include "sock_connect_wait.h"

#define NB 0x4

typedef struct {
	int      flags, get_rc, set_rc_first, set_rc_restore, connect_rc;
	bool     in_progress;
	int      timeouts_before_ready; /* wait_writable returns 0 this many times first */
	int      wait_result;           /* then this */
	int      so_err, so_rc;
	uint32_t now, now_step;
	int      heals, waits, set_calls, last_set;
} fake_t;

static int f_get(void *c)
{
	return ((fake_t *)c)->get_rc < 0 ? -1 : ((fake_t *)c)->flags;
}
static int f_set(void *c, int fl)
{
	fake_t *f = c;
	f->set_calls++;
	f->last_set = fl;
	if (f->set_calls == 1) {
		return f->set_rc_first;
	}
	return f->set_rc_restore;
}
static int f_conn(void *c)
{
	return ((fake_t *)c)->connect_rc;
}
static bool f_inprog(void *c)
{
	return ((fake_t *)c)->in_progress;
}
static int f_wait(void *c, uint32_t ms)
{
	fake_t *f = c;
	(void)ms;
	f->waits++;
	f->now += f->now_step;
	return f->waits <= f->timeouts_before_ready ? 0 : f->wait_result;
}
static int f_soerr(void *c, int *e)
{
	*e = ((fake_t *)c)->so_err;
	return ((fake_t *)c)->so_rc;
}
static uint32_t f_now(void *c)
{
	return ((fake_t *)c)->now;
}
static void f_heal(void *c)
{
	((fake_t *)c)->heals++;
}

static const sock_connect_ops_t ops = { f_get,   f_set, f_conn, f_inprog, f_wait,
	                                    f_soerr, f_now, f_heal, NB };

static fake_t base(void)
{
	fake_t f;
	memset(&f, 0, sizeof f);
	f.flags       = 0x2;
	f.connect_rc  = -1;
	f.in_progress = true;
	f.wait_result = 1;
	f.now_step    = 100u;
	return f;
}

ZTEST_SUITE(sock_connect_wait, NULL, NULL, NULL, NULL, NULL);

ZTEST(sock_connect_wait, test_immediate_connect_restores_flags)
{
	fake_t f     = base();
	f.connect_rc = 0;
	zassert_equal(sock_connect_run(&ops, &f), 0, "ok");
	zassert_equal(f.waits, 0, "no wait");
	zassert_equal(f.last_set, 0x2, "original flags restored");
}

ZTEST(sock_connect_wait, test_non_einprogress_error_fails_and_restores)
{
	fake_t f      = base();
	f.in_progress = false;
	zassert_equal(sock_connect_run(&ops, &f), -1, "fail");
	zassert_equal(f.last_set, 0x2, "restored");
}

ZTEST(sock_connect_wait, test_heal_runs_between_timed_out_slices_then_connects)
{
	fake_t f                = base();
	f.timeouts_before_ready = 3;
	zassert_equal(sock_connect_run(&ops, &f), 0, "ok");
	zassert_equal(f.heals, 3, "one heal per timed-out slice");
	zassert_equal(f.last_set, 0x2, "restored");
}

ZTEST(sock_connect_wait, test_so_error_nonzero_fails)
{
	fake_t f = base();
	f.so_err = 111;
	zassert_equal(sock_connect_run(&ops, &f), -1, "refused");
	zassert_equal(f.last_set, 0x2, "restored");
}

ZTEST(sock_connect_wait, test_so_error_getsockopt_failure_fails)
{
	fake_t f = base();
	f.so_rc  = -1;
	zassert_equal(sock_connect_run(&ops, &f), -1, "fail");
	zassert_equal(f.last_set, 0x2, "restored");
}

ZTEST(sock_connect_wait, test_select_error_fails_and_restores)
{
	fake_t f      = base();
	f.wait_result = -1;
	zassert_equal(sock_connect_run(&ops, &f), -1, "fail");
	zassert_equal(f.last_set, 0x2, "restored");
}

ZTEST(sock_connect_wait, test_safety_cap_expires_and_restores)
{
	fake_t f                = base();
	f.timeouts_before_ready = 1 << 30; /* never ready */
	zassert_equal(sock_connect_run(&ops, &f), -1, "gives up at the cap");
	zassert_equal(f.waits, 600, "60 s / 100 ms slices");
	zassert_equal(f.last_set, 0x2, "restored");
}

ZTEST(sock_connect_wait, test_waits_past_lwip_syn_limit)
{
	/* 25 s of timeouts (> lwIP's 12-21 s SYN give-up) must NOT be abandoned. */
	fake_t f                = base();
	f.timeouts_before_ready = 250;
	zassert_equal(sock_connect_run(&ops, &f), 0, "late SYN-ACK still accepted");
}

ZTEST(sock_connect_wait, test_set_nonblock_failure_returns_io_without_connecting)
{
	fake_t f       = base();
	f.set_rc_first = -1;
	f.connect_rc   = 0;
	zassert_equal(sock_connect_run(&ops, &f), -1, "no blocking fallback");
	zassert_equal(f.set_calls, 1, "nothing to restore");
}

ZTEST(sock_connect_wait, test_get_flags_failure_returns_io)
{
	fake_t f = base();
	f.get_rc = -1;
	zassert_equal(sock_connect_run(&ops, &f), -1, "fail");
	zassert_equal(f.set_calls, 0, "never touched flags");
}

ZTEST(sock_connect_wait, test_restore_failure_is_io_even_when_connected)
{
	fake_t f         = base();
	f.connect_rc     = 0;
	f.set_rc_restore = -1;
	zassert_equal(sock_connect_run(&ops, &f), -1, "non-blocking socket must not leak");
}

ZTEST(sock_connect_wait, test_slice_clamps_to_cap)
{
	zassert_equal(sock_connect_slice_ms(0u), 100u, "full slice");
	zassert_equal(sock_connect_slice_ms(59950u), 50u, "clamped");
	zassert_equal(sock_connect_slice_ms(60000u), 0u, "spent");
}

ZTEST(sock_connect_wait, test_cap_survives_now_ms_wraparound)
{
	/* t0 just below UINT32_MAX: now - t0 is unsigned-modular, so the 60 s cap
	 * must still land after 600 slices, not instantly or never. */
	fake_t f                = base();
	f.now                   = UINT32_MAX - 150u;
	f.timeouts_before_ready = 1 << 30;
	zassert_equal(sock_connect_run(&ops, &f), -1, "gives up at the cap");
	zassert_equal(f.waits, 600, "60 s / 100 ms slices across the wrap");
	zassert_equal(f.last_set, 0x2, "restored");
}
