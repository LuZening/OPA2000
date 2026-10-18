/**
 * @file    test_harness.c
 * @brief   Test runner: walks every suite table and executes its cases.
 *
 * @details Build modes:
 *   - HOST_TEST_NATIVE : linked against the platform libc (MSVC / MinGW /
 *     Clang).  Produces a runnable executable; this is the mode used by
 *     docs/GLOBAL_RULES.md rule 0 ("compile locally, run, verify").
 *   - otherwise        : freestanding (-nostdlib).  test_log_print() falls
 *     back to a tiny built-in formatter writing into a RAM buffer so the
 *     same test sources can also be linked into an on-target test build.
 *
 * @note    To add a new test suite: declare `g_suite_<name>[]` in the test
 *          file (terminated with TEST_SUITE_END) and add it to s_suites
 *          below.
 */

#include "test_harness.h"

#include <stdarg.h>

#if defined(HOST_TEST_NATIVE)

#include <stdio.h>
#include <stdlib.h>

void test_log_print(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
}

static void test_out(const char *s) { fputs(s, stderr); }

static void test_report(uint32_t pass, uint32_t fail, uint32_t cases,
                        uint32_t asserts)
{
    fprintf(stderr,
            "\n=== %lu passed, %lu failed, %lu cases, %lu assertions ===\n",
            (unsigned long)pass, (unsigned long)fail,
            (unsigned long)cases, (unsigned long)asserts);
}

static int test_platform_exit(int fail) { return fail ? 1 : 0; }

#else  /* freestanding cross build */

#ifndef TEST_LOG_BUF_SIZE
#define TEST_LOG_BUF_SIZE 8192U
#endif

static char     s_log_buf[TEST_LOG_BUF_SIZE];
static uint32_t s_log_len = 0U;

static void log_putc(char c)
{
    if (s_log_len + 1U < TEST_LOG_BUF_SIZE) {
        s_log_buf[s_log_len++] = c;
    }
}

static void log_puts(const char *s)
{
    while (*s) log_putc(*s++);
}

/* Supports %s %c %d %u %x %lld %llu %lu %% - enough for the harness. */
void test_log_print(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    while (*fmt) {
        if (*fmt != '%') { log_putc(*fmt++); continue; }
        fmt++;
        int lng = 0;
        while (*fmt == 'l') { lng++; fmt++; }
        char c = *fmt++;
        char tmp[24];
        int  n = 0;
        switch (c) {
        case 's': {
            const char *p = va_arg(ap, const char *);
            log_puts(p ? p : "(null)");
            break;
        }
        case 'c': log_putc((char)va_arg(ap, int)); break;
        case 'd': {
            long long v = lng ? va_arg(ap, long long)
                              : (long long)va_arg(ap, int);
            if (v < 0) { log_putc('-'); v = -v; }
            if (v == 0) { log_putc('0'); break; }
            while (v > 0 && n < (int)sizeof tmp) {
                tmp[n++] = (char)('0' + (v % 10));
                v /= 10;
            }
            while (n--) log_putc(tmp[n]);
            break;
        }
        case 'u': {
            unsigned long long v = lng ? va_arg(ap, unsigned long long)
                                       : (unsigned long long)va_arg(ap, unsigned int);
            if (v == 0) { log_putc('0'); break; }
            while (v > 0 && n < (int)sizeof tmp) {
                tmp[n++] = (char)('0' + (v % 10));
                v /= 10;
            }
            while (n--) log_putc(tmp[n]);
            break;
        }
        case 'x': {
            unsigned int v = va_arg(ap, unsigned int);
            const char *hex = "0123456789abcdef";
            if (v == 0) { log_putc('0'); break; }
            while (v > 0 && n < (int)sizeof tmp) {
                tmp[n++] = hex[v & 0xFU];
                v >>= 4;
            }
            while (n--) log_putc(tmp[n]);
            break;
        }
        case '%': log_putc('%'); break;
        default:  log_putc('%'); log_putc(c); break;
        }
    }
    va_end(ap);
}

static void test_out(const char *s) { log_puts(s); }

static void test_report(uint32_t pass, uint32_t fail, uint32_t cases,
                        uint32_t asserts)
{
    test_log_print("\n=== %lu passed, %lu failed, %lu cases, %lu assertions ===\n",
                   (unsigned long)pass, (unsigned long)fail,
                   (unsigned long)cases, (unsigned long)asserts);
}

static int test_platform_exit(int fail) { (void)fail; return 0; }

#endif /* HOST_TEST_NATIVE */

/*============================================================================
 *  Suite table
 *============================================================================*/

extern const test_entry_t g_suite_opa2000[];
extern const test_entry_t g_suite_user_adc[];
extern const test_entry_t g_suite_adc_calibration[];
extern const test_entry_t g_suite_power_control[];
extern const test_entry_t g_suite_protection[];
extern const test_entry_t g_suite_radio_band[];
extern const test_entry_t g_suite_freq_detect[];
extern const test_entry_t g_suite_cmsis_os_shim[];
extern const test_entry_t g_suite_config_mutex[];
extern const test_entry_t g_suite_boot[];
extern const test_entry_t g_suite_flash_eeprom[];
extern const test_entry_t g_suite_config_layout[];
extern const test_entry_t g_suite_fan_tach[];

static const test_entry_t *const s_suites[] = {
    g_suite_opa2000,
    g_suite_user_adc,
    g_suite_adc_calibration,
    g_suite_power_control,
    g_suite_protection,
    g_suite_radio_band,
    g_suite_freq_detect,
    g_suite_cmsis_os_shim,
    g_suite_config_mutex,
    g_suite_boot,
    g_suite_flash_eeprom,
    g_suite_config_layout,
    g_suite_fan_tach,
};

#define SUITE_COUNT (sizeof s_suites / sizeof s_suites[0])

uint32_t g_test_assert_count = 0U;
uint32_t g_test_assert_fail   = 0U;

/*============================================================================
 *  main()
 *============================================================================*/

int main(void)
{
    uint32_t pass = 0U, fail = 0U, cases = 0U, total_asserts = 0U;

    test_out("=== OPA2000 H7 unit tests ===\n");

    for (size_t s = 0U; s < SUITE_COUNT; s++) {
        const test_entry_t *e = s_suites[s];
        while (e->name != NULL && e->fn != NULL) {
            g_test_assert_count = 0U;
            g_test_assert_fail   = 0U;
            cases++;

            test_log_print("[ RUN  ] %s\n", e->name);
            e->fn();
            total_asserts += g_test_assert_count;

            if (g_test_assert_fail == 0U) {
                test_log_print("[  OK  ] %s (%lu assertion(s))\n",
                               e->name, (unsigned long)g_test_assert_count);
                pass++;
            } else {
                test_log_print("[ FAIL ] %s (%lu of %lu assertion(s))\n",
                               e->name, (unsigned long)g_test_assert_fail,
                               (unsigned long)g_test_assert_count);
                fail++;
            }
            e++;
        }
    }

    test_report(pass, fail, cases, total_asserts);
    return test_platform_exit((int)fail);
}