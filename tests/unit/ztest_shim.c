/* SPDX-License-Identifier: Apache-2.0 */
/* Runner for the ztest shim -- see ztest_shim.h for why this exists. */
#include "ztest_shim.h"

#include <stdarg.h>

static struct ztest_case      *cases;
static struct ztest_suite_rec *suites;
jmp_buf                        ztest_jmp;
const char                    *ztest_current = "(none)";
int                            ztest_failures;

/*
 * Append rather than push-front so cases run in registration order.
 * Constructor order within one translation unit is source order, which keeps
 * the output reading top-to-bottom like the suite file.
 */
void ztest_register(struct ztest_case *c)
{
	struct ztest_case **tail = &cases;

	while (*tail != NULL) {
		tail = &(*tail)->next;
	}
	*tail   = c;
	c->next = NULL;
}

void ztest_register_suite(struct ztest_suite_rec *s)
{
	struct ztest_suite_rec **tail = &suites;

	while (*tail != NULL) {
		tail = &(*tail)->next;
	}
	*tail   = s;
	s->next = NULL;
}

static struct ztest_suite_rec *suite_for(const char *name)
{
	for (struct ztest_suite_rec *s = suites; s != NULL; s = s->next) {
		if (strcmp(s->name, name) == 0) {
			return s;
		}
	}
	return NULL;
}

void ztest_fail(const char *file, int line, const char *detail, ...)
{
	va_list ap;

	ztest_failures++;
	fprintf(stderr, "  FAIL %s (%s:%d): ", ztest_current, file, line);
	va_start(ap, detail);
	vfprintf(stderr, detail, ap);
	va_end(ap);
	fputc('\n', stderr);
}

void ztest_abort(void)
{
	longjmp(ztest_jmp, 1);
}

void ztest_note(const char *fmt, ...)
{
	va_list ap;

	if (fmt == NULL || fmt[0] == '\0') {
		return;
	}
	fputs("       note: ", stderr);
	va_start(ap, fmt);
	vfprintf(stderr, fmt, ap);
	va_end(ap);
	fputc('\n', stderr);
}

int main(void)
{
	int total        = 0;
	int failed_cases = 0;

	for (struct ztest_suite_rec *s = suites; s != NULL; s = s->next) {
		if (s->setup != NULL) {
			s->fixture = s->setup();
		}
	}

	for (struct ztest_case *c = cases; c != NULL; c = c->next) {
		struct ztest_suite_rec *s      = suite_for(c->suite);
		int                     before = ztest_failures;

		/*
		 * A case whose suite never registered would silently run with
		 * no before/after hook -- exactly the order-dependence the
		 * hooks exist to prevent -- so refuse instead.
		 */
		if (s == NULL) {
			fprintf(stderr, "  FAIL %s: no ZTEST_SUITE registered for '%s'\n", c->name, c->suite);
			ztest_failures++;
			failed_cases++;
			total++;
			continue;
		}

		ztest_current = c->name;
		total++;
		if (s->before != NULL) {
			s->before(s->fixture);
		}
		if (setjmp(ztest_jmp) == 0) {
			c->fn();
		}
		/* after runs even when the case aborted, mirroring ztest. */
		if (s->after != NULL) {
			s->after(s->fixture);
		}

		if (ztest_failures > before) {
			failed_cases++;
		} else {
			printf("  PASS %s\n", c->name);
		}
	}

	for (struct ztest_suite_rec *s = suites; s != NULL; s = s->next) {
		if (s->teardown != NULL) {
			s->teardown(s->fixture);
		}
	}

	printf("%d/%d cases passed\n", total - failed_cases, total);
	/*
	 * Zero registered cases is a FAILURE, not a pass: it means the
	 * constructors never ran (the linker dropped the objects, or the suite
	 * was excluded from the build).  A harness that reports success when it
	 * ran nothing is exactly how coverage disappears unnoticed.
	 */
	if (total == 0) {
		fprintf(stderr, "no test cases registered -- harness is broken\n");
		return 2;
	}
	return failed_cases == 0 ? 0 : 1;
}
