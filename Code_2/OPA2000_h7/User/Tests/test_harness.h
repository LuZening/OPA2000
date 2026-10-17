/**
 * @file    test_harness.h
 * @brief   Lightweight portable unit-test harness (no external deps).
 *
 * @details Usage:
 *
 *            TEST_CASE(my_test_name)
 *            {
 *                ASSERT_EQ(a, b);
 *            }
 *
 *            const test_entry_t g_suite_my_module[] = {
 *                TEST_ENTRY(my_test_name),
 *                { NULL, NULL }          // mandatory terminator
 *            };
 *
 *          The suite array is then referenced from test_harness.c.
 *          Registration deliberately avoids linker-section tricks so the
 *          same test sources build with GCC, Clang and MSVC.
 *
 * @version 2.0
 * @date    2026-10
 * @author  Control Agent (wt-control branch)
 */

#ifndef TEST_HARNESS_H
#define TEST_HARNESS_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 *  Assertion macros
 *============================================================================*/

extern uint32_t g_test_assert_count;
extern uint32_t g_test_assert_fail;

void test_log_print(const char *fmt, ...);

/* Print at most the first 10 failures per test case so a single broken
 * expectation inside a loop cannot flood the log. */
#define TEST_FAIL_PRINT(fmt, ...)                                          \
    do {                                                                  \
        g_test_assert_fail++;                                             \
        if (g_test_assert_fail <= 10U) {                                  \
            test_log_print("[FAIL] %s:%d  " fmt "\n",                     \
                           __FILE__, __LINE__, ##__VA_ARGS__);             \
        }                                                                 \
    } while (0)

#define ASSERT_TRUE(cond)                                                 \
    do { if (!(cond)) TEST_FAIL_PRINT("ASSERT_TRUE(%s)", #cond); } while (0)

#define ASSERT_FALSE(cond)                                                \
    do { if ((cond)) TEST_FAIL_PRINT("ASSERT_FALSE(%s)", #cond); } while (0)

#define ASSERT_EQ(a, b)                                                   \
    do {                                                                  \
        long long _a = (long long)(a);                                    \
        long long _b = (long long)(b);                                    \
        if (_a != _b) {                                                   \
            TEST_FAIL_PRINT("ASSERT_EQ(%s,%s): got %lld want %lld",        \
                            #a, #b, _a, _b);                              \
        } else { g_test_assert_count++; }                                  \
    } while (0)

#define ASSERT_NE(a, b)                                                   \
    do {                                                                  \
        long long _a = (long long)(a);                                    \
        long long _b = (long long)(b);                                    \
        if (_a == _b) {                                                   \
            TEST_FAIL_PRINT("ASSERT_NE(%s,%s): both %lld",                \
                            #a, #b, _a);                                  \
        } else { g_test_assert_count++; }                                  \
    } while (0)

#define ASSERT_LT(a, b)                                                   \
    do {                                                                  \
        long long _a = (long long)(a);                                    \
        long long _b = (long long)(b);                                    \
        if (!(_a < _b)) {                                                 \
            TEST_FAIL_PRINT("ASSERT_LT(%s,%s): %lld not < %lld",          \
                            #a, #b, _a, _b);                              \
        } else { g_test_assert_count++; }                                  \
    } while (0)

#define ASSERT_LE(a, b)                                                   \
    do {                                                                  \
        long long _a = (long long)(a);                                    \
        long long _b = (long long)(b);                                    \
        if (!(_a <= _b)) {                                                \
            TEST_FAIL_PRINT("ASSERT_LE(%s,%s): %lld not <= %lld",         \
                            #a, #b, _a, _b);                              \
        } else { g_test_assert_count++; }                                  \
    } while (0)

#define ASSERT_GT(a, b)                                                   \
    do {                                                                  \
        long long _a = (long long)(a);                                    \
        long long _b = (long long)(b);                                    \
        if (!(_a > _b)) {                                                 \
            TEST_FAIL_PRINT("ASSERT_GT(%s,%s): %lld not > %lld",          \
                            #a, #b, _a, _b);                              \
        } else { g_test_assert_count++; }                                  \
    } while (0)

#define ASSERT_GE(a, b)                                                   \
    do {                                                                  \
        long long _a = (long long)(a);                                    \
        long long _b = (long long)(b);                                    \
        if (!(_a >= _b)) {                                                \
            TEST_FAIL_PRINT("ASSERT_GE(%s,%s): %lld not >= %lld",         \
                            #a, #b, _a, _b);                              \
        } else { g_test_assert_count++; }                                  \
    } while (0)

#define ASSERT_NULL(p)                                                    \
    do { if ((p) != NULL) TEST_FAIL_PRINT("ASSERT_NULL(%s)", #p); } while (0)

#define ASSERT_NOT_NULL(p)                                                \
    do { if ((p) == NULL) TEST_FAIL_PRINT("ASSERT_NOT_NULL(%s)", #p); } while (0)

#define ASSERT_MEM_EQ(a, b, n)                                            \
    do {                                                                  \
        if (memcmp((a), (b), (n)) != 0) {                                \
            TEST_FAIL_PRINT("ASSERT_MEM_EQ(%s,%s,%u)",                    \
                            #a, #b, (unsigned)(n));                       \
        } else { g_test_assert_count++; }                                 \
    } while (0)

#define ASSERT_STR_EQ(a, b)                                               \
    do {                                                                  \
        if (strcmp((a), (b)) != 0) {                                     \
            TEST_FAIL_PRINT("ASSERT_STR_EQ(%s,%s): got \"%s\" want \"%s\"",\
                            #a, #b, (a), (b));                            \
        } else { g_test_assert_count++; }                                 \
    } while (0)

/*============================================================================
 *  Test declaration + registration
 *============================================================================*/

typedef void (*test_fn_t)(void);

typedef struct {
    const char *name;
    test_fn_t    fn;
} test_entry_t;

#define TEST_CASE(name)  static void test_##name(void)
#define TEST_ENTRY(name) { #name, test_##name }
#define TEST_SUITE_END  { NULL, NULL }

#ifdef __cplusplus
}
#endif

#endif /* TEST_HARNESS_H */