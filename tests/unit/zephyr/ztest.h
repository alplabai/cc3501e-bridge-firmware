/* SPDX-License-Identifier: Apache-2.0 */
/*
 * Satisfies the suites' `#include <zephyr/ztest.h>` so they stay byte-identical
 * to the alp-sdk originals they were moved from.  The macros live in
 * ../ztest_shim.h; see that header for why this repo does not pull in Zephyr.
 */
#ifndef ZEPHYR_ZTEST_H_SHIM
#define ZEPHYR_ZTEST_H_SHIM
#include "../ztest_shim.h"
#endif
