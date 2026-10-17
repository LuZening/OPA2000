/**
 * @file    test_radio_band.c
 * @brief   Unit tests for radio_band.c — CAT identification, framing, polling.
 *
 * @details radio_band.c is pure logic (no UART, no DMA, no ISR), so it is
 *          compiled straight into the host test binary and driven through
 *          the public API.
 *
 *          The properties that matter most here:
 *            - identification by wire signature never confuses the three
 *              protocols with each other
 *            - a single valid frame takes effect immediately (no debounce)
 *            - a malformed frame never moves the band
 *            - the poll accumulator fires every 100 ms of inactivity and is
 *              reset by both a send and a receive
 */

#include "test_harness.h"
#include "radio_band.h"
#include "stm32h7xx_hal_stub.h"

#include <stddef.h>
#include <string.h>

/*============================================================================
 *  Test doubles for the hardware hooks
 *==========================================================================*/

static uint8_t  g_tx_buf[32];
static uint16_t g_tx_len;
static uint32_t g_tx_count;

static void rb_capture_tx(const uint8_t *d, uint16_t n)
{
    g_tx_count++;
    g_tx_len = (n > sizeof g_tx_buf) ? (uint16_t)sizeof g_tx_buf : n;
    memcpy(g_tx_buf, d, g_tx_len);
}

static void rb_reset(void)
{
    test_wall_clock_reset();
    g_radio_freq_hz = 0U;
    g_tx_len = 0U;
    g_tx_count = 0U;
    memset(g_tx_buf, 0, sizeof g_tx_buf);
    radio_band_init(RADIO_TYPE_UNKNOWN);
    radio_band_set_tx_hook(rb_capture_tx);
    radio_band_set_rx_hook(NULL);
}

/*============================================================================
 *  Wire-format fixtures
 *==========================================================================*/

/* ICOM CIV: FE FE <to> <from> <cmd=00> <5xBCD> FD.  14.074200 MHz. */
static const uint8_t k_civ_14m[] = {
    0xFE, 0xFE, 0x94, 0xE0, 0x00,
    0x00, 0x01, 0x40, 0x74, 0x20,
    0xFD
};

/* Kenwood: "IF;+001407420;" — 14.074200 MHz in 10 Hz steps. */
static const uint8_t k_ken_14m[] = {
    'I', 'F', ';', '+', '0', '0', '1', '4', '0', '7', '4', '2', '0', ';'
};

/* Yaesu: 5 raw BCD bytes, 14.074200 MHz. */
static const uint8_t k_yaesu_14m[] = { 0x00, 0x01, 0x40, 0x74, 0x20 };

/*============================================================================
 *  Pure parsers
 *==========================================================================*/

TEST_CASE(civ_parses_the_frequency_response)
{
    uint32_t f10 = 0U;
    ASSERT_TRUE(radio_band_parse_icom_civ(k_civ_14m,
                (uint16_t)sizeof k_civ_14m, &f10));
    ASSERT_EQ(f10, 1407420U);
}

TEST_CASE(civ_accepts_the_rx_frequency_command)
{
    uint8_t f[sizeof k_civ_14m];
    memcpy(f, k_civ_14m, sizeof f);
    f[4] = 0x01;                       /* cmd 01 = RX frequency */
    uint32_t f10 = 0U;
    ASSERT_TRUE(radio_band_parse_icom_civ(f, (uint16_t)sizeof f, &f10));
    ASSERT_EQ(f10, 1407420U);
}

TEST_CASE(civ_rejects_a_non_frequency_command)
{
    /* cmd 03 (read mode) also ends in FD but carries no frequency. */
    uint8_t f[sizeof k_civ_14m];
    memcpy(f, k_civ_14m, sizeof f);
    f[4] = 0x03;
    uint32_t f10 = 0U;
    ASSERT_FALSE(radio_band_parse_icom_civ(f, (uint16_t)sizeof f, &f10));
}

TEST_CASE(civ_rejects_a_missing_tail_byte)
{
    uint32_t f10 = 0U;
    /* Without the FD the frame is not complete. */
    ASSERT_FALSE(radio_band_parse_icom_civ(k_civ_14m, 10U, &f10));
}

TEST_CASE(civ_rejects_an_illegal_bcd_nibble)
{
    uint8_t f[sizeof k_civ_14m];
    memcpy(f, k_civ_14m, sizeof f);
    f[8] = 0x7A;                       /* high nibble 7 > 9 */
    uint32_t f10 = 0U;
    ASSERT_FALSE(radio_band_parse_icom_civ(f, (uint16_t)sizeof f, &f10));
}

TEST_CASE(kenwood_parses_the_if_response)
{
    uint32_t f10 = 0U;
    ASSERT_TRUE(radio_band_parse_kenwood(k_ken_14m,
                (uint16_t)sizeof k_ken_14m, &f10));
    ASSERT_EQ(f10, 1407420U);
}

TEST_CASE(kenwood_accepts_the_receive_sign)
{
    /* '-' is the receive indicator, not part of the number.
     * 3.500000 MHz = 3500000 Hz = 350000 at the 10 Hz step. */
    static const uint8_t f[] = {'I','F',';','-','3','5','0','0','0','0',';'};
    uint32_t f10 = 0U;
    ASSERT_TRUE(radio_band_parse_kenwood(f, (uint16_t)sizeof f, &f10));
    ASSERT_EQ(f10, 350000U);
}

TEST_CASE(kenwood_rejects_a_frame_without_the_terminator)
{
    uint8_t f[sizeof k_ken_14m];
    memcpy(f, k_ken_14m, sizeof f);
    f[sizeof f - 1U] = '0';
    uint32_t f10 = 0U;
    ASSERT_FALSE(radio_band_parse_kenwood(f, (uint16_t)(sizeof f - 1U), &f10));
}

TEST_CASE(kenwood_rejects_another_command)
{
    static const uint8_t f[] = {'F','A',';','0','0','1','4','0','7','4','2','0','0',';'};
    uint32_t f10 = 0U;
    ASSERT_FALSE(radio_band_parse_kenwood(f, (uint16_t)sizeof f, &f10));
}

TEST_CASE(yaesu_parses_the_bcd_frequency)
{
    uint32_t f10 = 0U;
    ASSERT_TRUE(radio_band_parse_yaesu(k_yaesu_14m,
                (uint16_t)sizeof k_yaesu_14m, &f10));
    ASSERT_EQ(f10, 1407420U);
}

TEST_CASE(yaesu_ignores_the_band_flag_in_the_top_nibble)
{
    uint8_t f[sizeof k_yaesu_14m];
    memcpy(f, k_yaesu_14m, sizeof f);
    f[0] = 0x30;                       /* band flag set, low nibble still 0 */
    uint32_t f10 = 0U;
    ASSERT_TRUE(radio_band_parse_yaesu(f, (uint16_t)sizeof f, &f10));
    ASSERT_EQ(f10, 1407420U);
}

TEST_CASE(yaesu_accepts_the_six_byte_form)
{
    uint8_t f[6] = { 0xF5, 0x00, 0x01, 0x40, 0x74, 0x20 };
    uint32_t f10 = 0U;
    ASSERT_TRUE(radio_band_parse_yaesu(f, (uint16_t)sizeof f, &f10));
    ASSERT_EQ(f10, 1407420U);
}

TEST_CASE(yaesu_rejects_wrong_lengths_and_bad_bcd)
{
    uint32_t f10 = 0U;
    ASSERT_FALSE(radio_band_parse_yaesu(k_yaesu_14m, 4U, &f10));
    ASSERT_FALSE(radio_band_parse_yaesu(k_yaesu_14m, 7U, &f10));

    uint8_t bad[sizeof k_yaesu_14m];
    memcpy(bad, k_yaesu_14m, sizeof bad);
    bad[3] = 0xAB;
    ASSERT_FALSE(radio_band_parse_yaesu(bad, (uint16_t)sizeof bad, &f10));
}

/*============================================================================
 *  Identification by wire signature
 *==========================================================================*/

TEST_CASE(each_protocol_is_identified_by_its_own_signature)
{
    struct { const uint8_t *f; uint16_t n; RadioType want; } cases[] = {
        { k_civ_14m,   (uint16_t)sizeof k_civ_14m,   RADIO_TYPE_ICOM_CIV },
        { k_ken_14m,   (uint16_t)sizeof k_ken_14m,   RADIO_TYPE_KENWOOD  },
        { k_yaesu_14m, (uint16_t)sizeof k_yaesu_14m, RADIO_TYPE_YAESU   },
    };
    for (unsigned i = 0U; i < sizeof cases / sizeof cases[0]; i++) {
        rb_reset();
        radio_band_feed(cases[i].f, cases[i].n);
        ASSERT_EQ(radio_band_get_type(), cases[i].want);
        ASSERT_EQ(radio_band_get_freq(), 14074200U);
    }
}

TEST_CASE(the_civ_leader_is_what_separates_icom_from_the_others)
{
    /* Same BCD payload, but wrapped in the CIV leader/trailer.  The leading
     * FE FE is the discriminator: without it these 5 bytes would parse as
     * Yaesu, with it the frame must be recognised as ICOM. */
    rb_reset();
    radio_band_feed(k_civ_14m, (uint16_t)sizeof k_civ_14m);
    ASSERT_EQ(radio_band_get_type(), RADIO_TYPE_ICOM_CIV);

    rb_reset();
    radio_band_feed(k_yaesu_14m, (uint16_t)sizeof k_yaesu_14m);
    ASSERT_EQ(radio_band_get_type(), RADIO_TYPE_YAESU);
}

TEST_CASE(kenwood_ascii_is_never_mistaken_for_raw_bcd)
{
    /* "IF;+001407420;" contains ASCII bytes well outside 0x30..0x39; a raw
     * BCD reader must reject it.  This is what keeps the two apart. */
    uint32_t f10 = 0U;
    ASSERT_FALSE(radio_band_parse_yaesu(k_ken_14m,
                (uint16_t)sizeof k_ken_14m, &f10));
}

TEST_CASE(a_locked_hint_rejects_the_other_protocols)
{
    test_wall_clock_reset();
    radio_band_init(RADIO_TYPE_KENWOOD);
    g_radio_freq_hz = 0U;

    /* A CIV frame must not move the reading when Kenwood is pinned. */
    radio_band_feed(k_civ_14m, (uint16_t)sizeof k_civ_14m);
    ASSERT_EQ(radio_band_get_freq(), 0U);
    ASSERT_EQ(radio_band_get_type(), RADIO_TYPE_KENWOOD);

    /* The pinned protocol still works. */
    radio_band_feed(k_ken_14m, (uint16_t)sizeof k_ken_14m);
    ASSERT_EQ(radio_band_get_freq(), 14074200U);
}

/*============================================================================
 *  No debounce: one valid frame acts immediately
 *==========================================================================*/

TEST_CASE(a_single_valid_frame_takes_effect_at_once)
{
    rb_reset();

    radio_band_feed(k_ken_14m, (uint16_t)sizeof k_ken_14m);

    /* No polling, no second frame, no waiting — the band must already be
     * correct.  A debounce design would still read 0 here. */
    ASSERT_EQ(radio_band_get_freq(), 14074200U);
    ASSERT_EQ(g_radio_freq_hz, 14074200U);
    ASSERT_EQ(radio_band_freq_to_band(radio_band_get_freq()), BAND_20M_17M);
}

TEST_CASE(a_frequency_change_is_followed_immediately)
{
    rb_reset();
    radio_band_feed(k_ken_14m, (uint16_t)sizeof k_ken_14m);
    ASSERT_EQ(radio_band_get_freq(), 14074200U);

    /* One CIV frame at 7.074 MHz (40 m) — again no second confirmation. */
    static const uint8_t civ_7m[] = {
        0xFE, 0xFE, 0x94, 0xE0, 0x00,
        0x00, 0x00, 0x70, 0x74, 0x00,
        0xFD
    };
    /* The type is still Kenwood from the previous frame, so feed Kenwood. */
    static const uint8_t ken_7m[] = {
        'I','F',';','+','0','0','0','7','0','7','4','0','0',';'
    };
    (void)civ_7m;
    radio_band_feed(ken_7m, (uint16_t)sizeof ken_7m);

    ASSERT_EQ(radio_band_get_freq(), 7074000U);
    ASSERT_EQ(radio_band_freq_to_band(radio_band_get_freq()), BAND_40M_30M);
}

/*============================================================================
 *  Malformed input must never move the band
 *==========================================================================*/

TEST_CASE(a_malformed_frame_leaves_the_band_untouched)
{
    rb_reset();
    radio_band_feed(k_ken_14m, (uint16_t)sizeof k_ken_14m);
    uint32_t good = radio_band_get_freq();
    ASSERT_EQ(good, 14074200U);

    /* Truncated frame (no ';'), illegal BCD, and out-of-range frequency in
     * turn.  None may disturb the value already latched. */
    uint8_t truncated[sizeof k_ken_14m];
    memcpy(truncated, k_ken_14m, sizeof truncated);
    truncated[sizeof truncated - 1U] = '0';
    radio_band_feed(truncated, (uint16_t)(sizeof truncated - 1U));

    uint8_t bad_bcd[sizeof k_civ_14m];
    memcpy(bad_bcd, k_civ_14m, sizeof bad_bcd);
    bad_bcd[8] = 0x7A;
    radio_band_feed(bad_bcd, (uint16_t)sizeof bad_bcd);

    /* 0.500000 MHz, below the HF floor. */
    static const uint8_t civ_low[] = {
        0xFE, 0xFE, 0x94, 0xE0, 0x00,
        0x00, 0x00, 0x00, 0x05, 0x00,
        0xFD
    };
    radio_band_feed(civ_low, (uint16_t)sizeof civ_low);

    ASSERT_EQ(radio_band_get_freq(), good);
    ASSERT_EQ(g_radio_freq_hz, good);

    RadioBandStatsType st;
    radio_band_get_stats(&st);
    ASSERT_GT(st.frames_bad, 0U);
}

TEST_CASE(a_frequency_outside_the_hf_range_is_refused)
{
    rb_reset();

    /* 0.500000 MHz = 500000 Hz, well below the 1.8 MHz floor. */
    static const uint8_t civ_low[] = {
        0xFE, 0xFE, 0x94, 0xE0, 0x00,
        0x00, 0x00, 0x00, 0x05, 0x00,
        0xFD
    };
    radio_band_feed(civ_low, (uint16_t)sizeof civ_low);

    ASSERT_EQ(radio_band_get_freq(), 0U);
}

TEST_CASE(a_run_of_random_noise_cannot_move_the_band)
{
    rb_reset();
    radio_band_feed(k_ken_14m, (uint16_t)sizeof k_ken_14m);
    uint32_t good = radio_band_get_freq();

    /* Bytes that are neither ASCII nor valid BCD: every one of them must be
     * rejected outright, so nothing can be mistaken for a frame. */
    uint8_t noise[256];
    for (unsigned i = 0U; i < sizeof noise; i++) {
        noise[i] = (uint8_t)(0x80U | (i & 0x7FU));
    }
    radio_band_feed(noise, (uint16_t)sizeof noise);

    ASSERT_EQ(radio_band_get_freq(), good);
}

TEST_CASE(a_run_of_junk_cannot_wedge_the_buffer)
{
    rb_reset();

    /* More bytes than the reassembly buffer holds, all invalid BCD. */
    uint8_t junk[200];
    for (unsigned i = 0U; i < sizeof junk; i++) {
        junk[i] = (uint8_t)(0xA0U | (i & 0x0FU));
    }
    radio_band_feed(junk, (uint16_t)sizeof junk);

    RadioBandStatsType st;
    radio_band_get_stats(&st);
    ASSERT_EQ(st.rx_bytes, sizeof junk);

    /* Still able to lock afterwards. */
    radio_band_feed(k_ken_14m, (uint16_t)sizeof k_ken_14m);
    ASSERT_EQ(radio_band_get_freq(), 14074200U);
}

/*============================================================================
 *  Framing / stream handling
 *==========================================================================*/

TEST_CASE(a_frame_split_byte_by_byte_is_reassembled)
{
    rb_reset();

    for (uint16_t i = 0U; i < (uint16_t)sizeof k_civ_14m; i++) {
        radio_band_feed(&k_civ_14m[i], 1U);
    }
    /* The final byte completed the frame, and with no debounce that is
     * already enough. */
    ASSERT_EQ(radio_band_get_freq(), 14074200U);
}

TEST_CASE(two_frames_in_one_buffer_are_both_consumed)
{
    rb_reset();

    uint8_t stream[2U * sizeof k_civ_14m];
    for (unsigned i = 0U; i < 2U; i++) {
        memcpy(&stream[i * sizeof k_civ_14m], k_civ_14m, sizeof k_civ_14m);
    }
    radio_band_feed(stream, (uint16_t)sizeof stream);

    RadioBandStatsType st;
    radio_band_get_stats(&st);
    ASSERT_EQ(st.frames_ok, 2U);
}

TEST_CASE(leading_noise_before_a_frame_is_skipped)
{
    rb_reset();

    static const uint8_t junk[] = { 0x00, 0xFF, 0xFE, 0x00, 0x13, 0x37 };
    uint8_t stream[sizeof junk + sizeof k_civ_14m];
    memcpy(stream, junk, sizeof junk);
    memcpy(&stream[sizeof junk], k_civ_14m, sizeof k_civ_14m);

    radio_band_feed(stream, (uint16_t)sizeof stream);

    ASSERT_EQ(radio_band_get_freq(), 14074200U);
}

TEST_CASE(a_half_frame_times_out_and_is_dropped)
{
    rb_reset();

    /* FE FE then nothing — a frame that will never complete. */
    static const uint8_t half[] = { 0xFE, 0xFE, 0x94, 0xE0, 0x00 };
    radio_band_feed(half, (uint16_t)sizeof half);
    ASSERT_EQ(radio_band_get_freq(), 0U);

    /* Past RADIO_BAND_IDLE_MS the fragment is discarded and counted. */
    test_wall_clock_advance(RADIO_BAND_IDLE_MS + 50U);
    radio_band_poll();

    RadioBandStatsType st;
    radio_band_get_stats(&st);
    ASSERT_GE(st.timeouts, 1U);

    /* The link recovers. */
    radio_band_feed(k_civ_14m, (uint16_t)sizeof k_civ_14m);
    ASSERT_EQ(radio_band_get_freq(), 14074200U);
}

TEST_CASE(find_frame_reports_how_much_it_consumed)
{
    const uint8_t *fr;
    uint16_t fl, used;

    rb_reset();
    uint8_t two[2U * sizeof k_civ_14m];
    memcpy(two, k_civ_14m, sizeof k_civ_14m);
    memcpy(&two[sizeof k_civ_14m], k_civ_14m, sizeof k_civ_14m);

    ASSERT_TRUE(radio_band_find_frame(two, (uint16_t)sizeof two,
                       &fr, &fl, &used, RADIO_TYPE_UNKNOWN));
    ASSERT_EQ(fl, sizeof k_civ_14m);
    ASSERT_EQ(used, sizeof k_civ_14m);

    /* The second frame is then found at the same offset. */
    ASSERT_TRUE(radio_band_find_frame(&two[used],
                       (uint16_t)(sizeof two - used), &fr, &fl, &used,
                       RADIO_TYPE_UNKNOWN));
    ASSERT_EQ(used, sizeof k_civ_14m);
}

/*============================================================================
 *  ISR ring
 *==========================================================================*/

TEST_CASE(isr_push_does_not_block_when_full)
{
    rb_reset();

    uint32_t accepted = 0U;
    for (uint32_t i = 0U; i < 1000U; i++) {
        if (radio_band_push_isr((uint8_t)i)) {
            accepted++;
        }
    }
    ASSERT_GT(accepted, 0U);
    ASSERT_LT(accepted, 1000U);

    RadioBandStatsType st;
    radio_band_get_stats(&st);
    ASSERT_GT(st.overrun, 0U);
}

TEST_CASE(bytes_queued_from_the_isr_reach_the_parser)
{
    rb_reset();

    for (uint16_t i = 0U; i < (uint16_t)sizeof k_ken_14m; i++) {
        ASSERT_TRUE(radio_band_push_isr(k_ken_14m[i]));
    }
    radio_band_poll();

    ASSERT_EQ(radio_band_get_freq(), 14074200U);
    ASSERT_EQ(radio_band_get_type(), RADIO_TYPE_KENWOOD);
}

/*============================================================================
 *  Poll accumulator
 *==========================================================================*/

TEST_CASE(the_poll_fires_only_after_a_full_period_of_silence)
{
    rb_reset();

    /* One tick short of the period: nothing sent. */
    for (uint32_t i = 0U; i < RADIO_BAND_POLL_PERIOD_MS - 1U; i++) {
        radio_band_tick();
    }
    radio_band_poll();
    ASSERT_EQ(g_tx_count, 0U);

    /* The last tick completes the period. */
    radio_band_tick();
    radio_band_poll();
    ASSERT_EQ(g_tx_count, 1U);
}

TEST_CASE(sending_resets_the_accumulator)
{
    rb_reset();

    /* Fire once. */
    for (uint32_t i = 0U; i < RADIO_BAND_POLL_PERIOD_MS; i++) {
        radio_band_tick();
    }
    radio_band_poll();
    ASSERT_EQ(g_tx_count, 1U);

    /* Nine more ticks must not produce a second poll: the send zeroed the
     * accumulator, so the next one is a full period away. */
    for (uint32_t i = 0U; i < RADIO_BAND_POLL_PERIOD_MS - 1U; i++) {
        radio_band_tick();
    }
    radio_band_poll();
    ASSERT_EQ(g_tx_count, 1U);

    /* The tenth completes it. */
    radio_band_tick();
    radio_band_poll();
    ASSERT_EQ(g_tx_count, 2U);
}

TEST_CASE(receiving_a_frame_resets_the_accumulator)
{
    rb_reset();

    /* Nine ticks of silence, then the radio speaks. */
    for (uint32_t i = 0U; i < RADIO_BAND_POLL_PERIOD_MS - 1U; i++) {
        radio_band_tick();
    }
    radio_band_feed(k_ken_14m, (uint16_t)sizeof k_ken_14m);
    radio_band_poll();
    ASSERT_EQ(g_tx_count, 0U);

    /* The frame counts as activity, so the interval starts over. */
    for (uint32_t i = 0U; i < RADIO_BAND_POLL_PERIOD_MS - 1U; i++) {
        radio_band_tick();
    }
    radio_band_poll();
    ASSERT_EQ(g_tx_count, 0U);

    radio_band_tick();
    radio_band_poll();
    ASSERT_EQ(g_tx_count, 1U);
}

TEST_CASE(bytes_from_the_isr_also_reset_the_accumulator)
{
    rb_reset();

    for (uint32_t i = 0U; i < RADIO_BAND_POLL_PERIOD_MS - 1U; i++) {
        radio_band_tick();
    }
    for (uint16_t i = 0U; i < (uint16_t)sizeof k_ken_14m; i++) {
        radio_band_push_isr(k_ken_14m[i]);
    }
    radio_band_poll();

    ASSERT_EQ(g_tx_count, 0U);        /* the frame restarted the interval */
    ASSERT_EQ(radio_band_get_freq(), 14074200U);
}

TEST_CASE(the_query_matches_the_identified_protocol)
{
    /* Kenwood / not yet identified -> "IF;" */
    rb_reset();
    for (uint32_t i = 0U; i < RADIO_BAND_POLL_PERIOD_MS; i++) {
        radio_band_tick();
    }
    radio_band_poll();
    ASSERT_EQ(g_tx_len, 3U);
    ASSERT_EQ(g_tx_buf[0], 'I');
    ASSERT_EQ(g_tx_buf[1], 'F');
    ASSERT_EQ(g_tx_buf[2], ';');

    /* ICOM -> the CIV read-frequency frame. */
    rb_reset();
    radio_band_feed(k_civ_14m, (uint16_t)sizeof k_civ_14m);
    ASSERT_EQ(radio_band_get_type(), RADIO_TYPE_ICOM_CIV);
    g_tx_count = 0U;
    for (uint32_t i = 0U; i < RADIO_BAND_POLL_PERIOD_MS; i++) {
        radio_band_tick();
    }
    radio_band_poll();
    ASSERT_EQ(g_tx_len, 7U);
    ASSERT_EQ(g_tx_buf[0], 0xFE);
    ASSERT_EQ(g_tx_buf[1], 0xFE);
    ASSERT_EQ(g_tx_buf[6], 0xFD);
}

TEST_CASE(an_explicit_poll_bypasses_the_accumulator)
{
    rb_reset();

    radio_band_send_poll();

    ASSERT_EQ(g_tx_count, 1U);
}

TEST_CASE(no_tx_hook_means_no_crash)
{
    /* The module must tolerate being used without the hardware wired up.
     * rb_reset() clears the capture counters first, so a non-zero count
     * here could only come from an actual send attempt. */
    rb_reset();
    radio_band_set_tx_hook(NULL);

    for (uint32_t i = 0U; i < RADIO_BAND_POLL_PERIOD_MS; i++) {
        radio_band_tick();
    }
    radio_band_poll();                 /* must not fault */

    ASSERT_EQ(g_tx_count, 0U);
}

/*============================================================================
 *  Band mapping
 *==========================================================================*/

TEST_CASE(every_merged_band_maps_correctly)
{
    struct { uint32_t hz; BandType want; } cases[] = {
        {  1830000U, BAND_160M    },   /* 1.800 - 2.100 MHz   */
        {  3830000U, BAND_80M     },   /* 3.500 - 3.900 MHz   */
        {  7074000U, BAND_40M_30M },   /* 7.000 - 10.150 MHz  */
        { 14074000U, BAND_20M_17M },   /* 14.000 - 18.168 MHz */
        { 21000000U, BAND_15M_10M },   /* 21.000 - 29.700 MHz */
        { 29700000U, BAND_15M_10M },
    };
    for (unsigned i = 0U; i < sizeof cases / sizeof cases[0]; i++) {
        ASSERT_EQ(radio_band_freq_to_band(cases[i].hz), cases[i].want);
    }
}

TEST_CASE(a_zero_frequency_maps_to_no_band)
{
    ASSERT_EQ(radio_band_freq_to_band(0U), BAND_NONE);
}

TEST_CASE(the_published_frequency_and_the_global_agree)
{
    rb_reset();
    radio_band_feed(k_civ_14m, (uint16_t)sizeof k_civ_14m);
    /* power_update_band() reads g_radio_freq_hz; it must not drift. */
    ASSERT_EQ(g_radio_freq_hz, radio_band_get_freq());
}

/*============================================================================
 *  set_hint / TX 守卫
 *==========================================================================*/

TEST_CASE(set_hint_after_auto_lock_drops_to_unknown)
{
    rb_reset();
    /* 让自动识别先跑通（Kenwood 帧最简单） */
    radio_band_feed(k_ken_14m, (uint16_t)sizeof k_ken_14m);
    ASSERT_EQ(radio_band_get_type(), RADIO_TYPE_KENWOOD);

    /* 用户在 GUI 切到一个不同协议 -> 已识别状态被清回 UNKNOWN */
    radio_band_set_hint(RADIO_TYPE_ICOM_CIV);
    ASSERT_EQ(radio_band_get_type(), RADIO_TYPE_UNKNOWN);
}

TEST_CASE(set_hint_to_explicit_protocol_overrides_auto_lock)
{
    rb_reset();
    radio_band_feed(k_ken_14m, (uint16_t)sizeof k_ken_14m);
    /* 与已识别一致：保持不动 */
    radio_band_set_hint(RADIO_TYPE_KENWOOD);
    ASSERT_EQ(radio_band_get_type(), RADIO_TYPE_KENWOOD);

    /* 不一致：清回 UNKNOWN，下一帧会重新识别 */
    radio_band_set_hint(RADIO_TYPE_YAESU);
    ASSERT_EQ(radio_band_get_type(), RADIO_TYPE_UNKNOWN);
}

TEST_CASE(set_hint_unknown_keeps_current_identified_protocol)
{
    rb_reset();
    radio_band_feed(k_ken_14m, (uint16_t)sizeof k_ken_14m);
    RadioType before = radio_band_get_type();
    /* 用户没指定协议，识别结果保留 */
    radio_band_set_hint(RADIO_TYPE_UNKNOWN);
    ASSERT_EQ(radio_band_get_type(), before);
}

TEST_CASE(tx_active_suppresses_frequency_updates)
{
    rb_reset();
    /* 标识好协议后再发一个帧 -> 频率被采纳 */
    radio_band_feed(k_ken_14m, (uint16_t)sizeof k_ken_14m);
    ASSERT_EQ(g_radio_freq_hz, 14074200U);

    /* 操作员按了 PTT */
    g_tx_active = true;
    /* 电台扫到 7.074 MHz 再发一帧 */
    static const uint8_t ken_7m[] = {
        'I', 'F', ';', '+', '0', '0', '0', '7', '0', '7', '4', '0', '0', ';'
    };
    radio_band_feed(ken_7m, (uint16_t)sizeof ken_7m);
    /* 频率没变，freq_detect 不会被这次帧误导 */
    ASSERT_EQ(g_radio_freq_hz, 14074200U);

    /* TX 结束，下一帧恢复 */
    g_tx_active = false;
    radio_band_feed(ken_7m, (uint16_t)sizeof ken_7m);
    /* Kenwood carries the frequency in 10 Hz steps: 000707400 -> 7074000 Hz. */
    ASSERT_EQ(g_radio_freq_hz, 7074000U);
}

TEST_CASE(frames_held_tx_counter_increments_when_tx_active)
{
    rb_reset();
    RadioBandStatsType st = {0};
    radio_band_get_stats(&st);
    uint32_t before = st.frames_held_tx;

    g_tx_active = true;
    radio_band_feed(k_ken_14m, (uint16_t)sizeof k_ken_14m);

    radio_band_get_stats(&st);
    ASSERT_EQ(st.frames_held_tx, before + 1U);
    g_tx_active = false;
}

/*============================================================================
 *  Suite table
 *==========================================================================*/

const test_entry_t g_suite_radio_band[] = {
    TEST_ENTRY(civ_parses_the_frequency_response),
    TEST_ENTRY(civ_accepts_the_rx_frequency_command),
    TEST_ENTRY(civ_rejects_a_non_frequency_command),
    TEST_ENTRY(civ_rejects_a_missing_tail_byte),
    TEST_ENTRY(civ_rejects_an_illegal_bcd_nibble),
    TEST_ENTRY(kenwood_parses_the_if_response),
    TEST_ENTRY(kenwood_accepts_the_receive_sign),
    TEST_ENTRY(kenwood_rejects_a_frame_without_the_terminator),
    TEST_ENTRY(kenwood_rejects_another_command),
    TEST_ENTRY(yaesu_parses_the_bcd_frequency),
    TEST_ENTRY(yaesu_ignores_the_band_flag_in_the_top_nibble),
    TEST_ENTRY(yaesu_accepts_the_six_byte_form),
    TEST_ENTRY(yaesu_rejects_wrong_lengths_and_bad_bcd),
    TEST_ENTRY(each_protocol_is_identified_by_its_own_signature),
    TEST_ENTRY(the_civ_leader_is_what_separates_icom_from_the_others),
    TEST_ENTRY(kenwood_ascii_is_never_mistaken_for_raw_bcd),
    TEST_ENTRY(a_locked_hint_rejects_the_other_protocols),
    TEST_ENTRY(a_single_valid_frame_takes_effect_at_once),
    TEST_ENTRY(a_frequency_change_is_followed_immediately),
    TEST_ENTRY(a_malformed_frame_leaves_the_band_untouched),
    TEST_ENTRY(a_frequency_outside_the_hf_range_is_refused),
    TEST_ENTRY(a_run_of_random_noise_cannot_move_the_band),
    TEST_ENTRY(a_run_of_junk_cannot_wedge_the_buffer),
    TEST_ENTRY(a_frame_split_byte_by_byte_is_reassembled),
    TEST_ENTRY(two_frames_in_one_buffer_are_both_consumed),
    TEST_ENTRY(leading_noise_before_a_frame_is_skipped),
    TEST_ENTRY(a_half_frame_times_out_and_is_dropped),
    TEST_ENTRY(find_frame_reports_how_much_it_consumed),
    TEST_ENTRY(isr_push_does_not_block_when_full),
    TEST_ENTRY(bytes_queued_from_the_isr_reach_the_parser),
    TEST_ENTRY(the_poll_fires_only_after_a_full_period_of_silence),
    TEST_ENTRY(sending_resets_the_accumulator),
    TEST_ENTRY(receiving_a_frame_resets_the_accumulator),
    TEST_ENTRY(bytes_from_the_isr_also_reset_the_accumulator),
    TEST_ENTRY(the_query_matches_the_identified_protocol),
    TEST_ENTRY(an_explicit_poll_bypasses_the_accumulator),
    TEST_ENTRY(no_tx_hook_means_no_crash),
    TEST_ENTRY(every_merged_band_maps_correctly),
    TEST_ENTRY(a_zero_frequency_maps_to_no_band),
    TEST_ENTRY(the_published_frequency_and_the_global_agree),
    TEST_ENTRY(set_hint_after_auto_lock_drops_to_unknown),
    TEST_ENTRY(set_hint_to_explicit_protocol_overrides_auto_lock),
    TEST_ENTRY(set_hint_unknown_keeps_current_identified_protocol),
    TEST_ENTRY(tx_active_suppresses_frequency_updates),
    TEST_ENTRY(frames_held_tx_counter_increments_when_tx_active),
    TEST_SUITE_END
};