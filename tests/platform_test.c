#ifndef _WIN32
#define _XOPEN_SOURCE 700
#define _DEFAULT_SOURCE
#define _DARWIN_C_SOURCE
#endif

#include "../platform.h"

#include <assert.h>
#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>

#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#define make_pipe(descriptors) _pipe(descriptors, 1 << 16, _O_BINARY)
#define close_descriptor _close
#else
#include <unistd.h>
#define make_pipe(descriptors) pipe(descriptors)
#define close_descriptor close
#endif

/*
 * The conversions the Windows console input is built from are plain functions
 * of plain integers, so they are tested here on every system, with the numbers a
 * console record would hold.
 */

static size_t utf8_of(uint16_t *high, unsigned unit, char out[4]) {
    return platform_utf8_from_utf16(high, unit, out);
}

static void test_a_key_character_becomes_utf8(void) {
    uint16_t high = 0;
    char out[4];

    assert(utf8_of(&high, 'q', out) == 1 && out[0] == 'q');
    assert(utf8_of(&high, 0x1b, out) == 1 && out[0] == '\033');
    assert(utf8_of(&high, '\t', out) == 1 && out[0] == '\t');
    assert(utf8_of(&high, 0x7f, out) == 1 && (unsigned char)out[0] == 0x7f);
    /* e with an acute accent, two bytes. */
    assert(utf8_of(&high, 0xE9, out) == 2);
    assert((unsigned char)out[0] == 0xC3 && (unsigned char)out[1] == 0xA9);
    /* The euro sign, three. */
    assert(utf8_of(&high, 0x20AC, out) == 3);
    assert((unsigned char)out[0] == 0xE2 && (unsigned char)out[1] == 0x82 &&
           (unsigned char)out[2] == 0xAC);
    /* The last unit before the surrogates and the first after. */
    assert(utf8_of(&high, 0xD7FF, out) == 3 && (unsigned char)out[0] == 0xED);
    assert(utf8_of(&high, 0xE000, out) == 3 && (unsigned char)out[0] == 0xEE);
    /* The braille blank cbirds draws with, so the reverse of its output is proven. */
    assert(utf8_of(&high, 0x2800, out) == 3);
    assert((unsigned char)out[0] == 0xE2 && (unsigned char)out[1] == 0xA0 &&
           (unsigned char)out[2] == 0x80);
    assert(high == 0);
}

static void test_a_nul_is_nothing(void) {
    uint16_t high = 0;
    char out[4];
    /* Modifier keys on their own arrive as key records with no character. */
    assert(utf8_of(&high, 0, out) == 0);
}

static void test_a_surrogate_pair_becomes_one_character(void) {
    uint16_t high = 0;
    char out[4];

    /* U+1F426, a bird, is D83D DC26; its UTF-8 is F0 9F 90 A6. */
    assert(utf8_of(&high, 0xD83D, out) == 0);
    assert(high == 0xD83D);
    assert(utf8_of(&high, 0xDC26, out) == 4);
    assert((unsigned char)out[0] == 0xF0 && (unsigned char)out[1] == 0x9F &&
           (unsigned char)out[2] == 0x90 && (unsigned char)out[3] == 0xA6);
    assert(high == 0);
    /* The first and the last character outside the plane: U+10000, U+10FFFF. */
    assert(utf8_of(&high, 0xD800, out) == 0 && utf8_of(&high, 0xDC00, out) == 4);
    assert((unsigned char)out[0] == 0xF0 && (unsigned char)out[1] == 0x90 &&
           (unsigned char)out[2] == 0x80 && (unsigned char)out[3] == 0x80);
    assert(utf8_of(&high, 0xDBFF, out) == 0 && utf8_of(&high, 0xDFFF, out) == 4);
    assert((unsigned char)out[0] == 0xF4 && (unsigned char)out[1] == 0x8F &&
           (unsigned char)out[2] == 0xBF && (unsigned char)out[3] == 0xBF);
}

static void test_a_lone_surrogate_is_dropped_and_forgotten(void) {
    uint16_t high = 0;
    char out[4];

    /* A low half with nothing before it. */
    assert(utf8_of(&high, 0xDC26, out) == 0);
    /* A high half answered by an ordinary character: the character survives and
     * the half is not carried to the next pair. */
    assert(utf8_of(&high, 0xD83D, out) == 0);
    assert(utf8_of(&high, 'x', out) == 1 && out[0] == 'x');
    assert(high == 0);
    assert(utf8_of(&high, 0xDC26, out) == 0);
    /* Two highs in a row: the second is the one that pairs. */
    assert(utf8_of(&high, 0xD83D, out) == 0);
    assert(utf8_of(&high, 0xD83D, out) == 0);
    assert(utf8_of(&high, 0xDC26, out) == 4 && (unsigned char)out[3] == 0xA6);
}

/* The console's button bits: 1 left, 2 right, 4 middle. The record's flags: 1
 * moved, 2 double click, 4 wheel, 8 horizontal wheel. */
static void test_a_mouse_record_becomes_an_sgr_report(void) {
    unsigned held = 0;
    char out[128];

    /* A move with nothing held: button 3 plus the motion bit. */
    assert(platform_sgr_mouse(&held, 0, 1, 0, 0, 0, out, sizeof(out)) == strlen("\033[<35;1;1M"));
    assert(strcmp(out, "\033[<35;1;1M") == 0);
    /* Cells are counted from one in the report and from zero in the record. */
    platform_sgr_mouse(&held, 0, 1, 0, 9, 4, out, sizeof(out));
    assert(strcmp(out, "\033[<35;10;5M") == 0);

    /* The left button goes down where it is... */
    platform_sgr_mouse(&held, 1, 0, 0, 9, 4, out, sizeof(out));
    assert(strcmp(out, "\033[<0;10;5M") == 0 && held == 1);
    /* ...is dragged, which carries the button and the motion bit... */
    platform_sgr_mouse(&held, 1, 1, 0, 10, 4, out, sizeof(out));
    assert(strcmp(out, "\033[<32;11;5M") == 0);
    /* ...and comes up, with a lower case final. */
    platform_sgr_mouse(&held, 0, 0, 0, 10, 4, out, sizeof(out));
    assert(strcmp(out, "\033[<0;11;5m") == 0 && held == 0);

    /* Right is 2 and middle is 1 in a report, whatever order the console has. */
    platform_sgr_mouse(&held, 2, 0, 0, 0, 0, out, sizeof(out));
    assert(strcmp(out, "\033[<2;1;1M") == 0);
    platform_sgr_mouse(&held, 0, 0, 0, 0, 0, out, sizeof(out));
    assert(strcmp(out, "\033[<2;1;1m") == 0);
    platform_sgr_mouse(&held, 4, 0, 0, 0, 0, out, sizeof(out));
    assert(strcmp(out, "\033[<1;1;1M") == 0);
    platform_sgr_mouse(&held, 0, 0, 0, 0, 0, out, sizeof(out));
    assert(strcmp(out, "\033[<1;1;1m") == 0);

    /* Two buttons changing in one record are two reports. */
    platform_sgr_mouse(&held, 3, 0, 0, 2, 2, out, sizeof(out));
    assert(strcmp(out, "\033[<0;3;3M\033[<2;3;3M") == 0 && held == 3);

    /* A double click is a press like any other. */
    held = 0;
    platform_sgr_mouse(&held, 1, 2, 0, 0, 0, out, sizeof(out));
    assert(strcmp(out, "\033[<0;1;1M") == 0);
}

static void test_the_wheel_is_sixty_four_and_sixty_five(void) {
    unsigned held = 0;
    char out[128];

    platform_sgr_mouse(&held, 0, 4, 120, 5, 5, out, sizeof(out)); /* Forward, away. */
    assert(strcmp(out, "\033[<64;6;6M") == 0);
    platform_sgr_mouse(&held, 0, 4, -120, 5, 5, out, sizeof(out));
    assert(strcmp(out, "\033[<65;6;6M") == 0);
    platform_sgr_mouse(&held, 0, 8, 120, 5, 5, out, sizeof(out));
    assert(strcmp(out, "\033[<67;6;6M") == 0);
    platform_sgr_mouse(&held, 0, 8, -120, 5, 5, out, sizeof(out));
    assert(strcmp(out, "\033[<66;6;6M") == 0);
    assert(held == 0);
}

static void test_a_record_that_changed_nothing_says_nothing(void) {
    unsigned held = 1;
    char out[128];
    /* The same button state and no movement, as a focus-only record would be. */
    assert(platform_sgr_mouse(&held, 1, 0, 0, 3, 3, out, sizeof(out)) == 0);
    assert(out[0] == '\0');
    /* And a buffer too small for a report is not overrun. */
    char tiny[4];
    held = 0;
    assert(platform_sgr_mouse(&held, 1, 0, 0, 3, 3, tiny, sizeof(tiny)) == 0);
    assert(strlen(tiny) < sizeof(tiny));
}

/*
 * The parts that are the system's: a clock that runs forwards, a sleep that is
 * as good as the pace needs, and a closed pipe that is an error and not a death.
 */

static double seconds_between(const platform_time_t *start, const platform_time_t *end) {
    return (double)(end->tv_sec - start->tv_sec) + (double)(end->tv_nsec - start->tv_nsec) / 1e9;
}

static void test_the_clock_runs_forwards(void) {
    platform_time_t a, b;
    platform_now(&a);
    platform_now(&b);
    assert(seconds_between(&a, &b) >= 0.0);
    assert(a.tv_nsec >= 0 && a.tv_nsec < 1000000000L);
    platform_sleep_microseconds(20000);
    platform_now(&b);
    assert(seconds_between(&a, &b) >= 0.019);
}

/* A frame is 16.7 ms. Windows rounds an ordinary sleep up to its 15.6 ms tick,
 * which makes that 31 ms and the program thirty frames a second; this measures
 * what the pace is made of, and prints it so the CI log shows the number. */
static void test_a_sleep_of_a_frame_is_about_a_frame(void) {
    enum { SLEEPS = 30, FRAME_MICROSECONDS = 16667 };
    platform_time_t start, end;
    platform_now(&start);
    for (int i = 0; i < SLEEPS; i++) platform_sleep_microseconds(FRAME_MICROSECONDS);
    platform_now(&end);
    double mean_ms = seconds_between(&start, &end) * 1000.0 / SLEEPS;
    printf("a sleep of 16.667 ms took %.3f ms on average over %d\n", mean_ms, SLEEPS);
    assert(mean_ms >= 16.5);
    assert(mean_ms < 20.0);
}

static void test_a_pipe_with_no_reader_is_an_error_not_a_death(void) {
    int descriptors[2];
    char byte = 'x';

#ifndef _WIN32
    signal(SIGPIPE, SIG_IGN); /* As the program does. */
#endif
    assert(make_pipe(descriptors) == 0);
    assert(platform_write(descriptors[1], &byte, 1) == 1);
    close_descriptor(descriptors[0]);
    errno = 0;
    assert(platform_write(descriptors[1], &byte, 1) == -1);
    assert(errno == EPIPE);
    close_descriptor(descriptors[1]);
    errno = 0;
    assert(platform_write(descriptors[1], &byte, 1) == -1);
}

static void test_a_long_write_arrives_in_whole(void) {
    /* Larger than the pieces a console is given, with a multi byte character
     * where a piece would end, written until it is all there. */
    enum { LENGTH = 100000 };
    static char data[LENGTH], back[LENGTH];
    int descriptors[2];
    for (int i = 0; i < LENGTH; i++) data[i] = (i % 3 == 0) ? (char)0xA0 : 'a';
    assert(make_pipe(descriptors) == 0);
    size_t sent = 0, got = 0;
    /* The pipe holds 64 KB; write what fits, read it, go on. */
    while (sent < LENGTH) {
        size_t chunk = LENGTH - sent > 20000 ? 20000 : LENGTH - sent;
        ssize_t wrote = platform_write(descriptors[1], data + sent, chunk);
        assert(wrote > 0);
        sent += (size_t)wrote;
        while (got < sent) {
#ifdef _WIN32
            int n = _read(descriptors[0], back + got, (unsigned)(sent - got));
#else
            ssize_t n = read(descriptors[0], back + got, sent - got);
#endif
            assert(n > 0);
            got += (size_t)n;
        }
    }
    assert(memcmp(data, back, LENGTH) == 0);
    close_descriptor(descriptors[0]);
    close_descriptor(descriptors[1]);
}

int main(void) {
    test_a_key_character_becomes_utf8();
    test_a_nul_is_nothing();
    test_a_surrogate_pair_becomes_one_character();
    test_a_lone_surrogate_is_dropped_and_forgotten();
    test_a_mouse_record_becomes_an_sgr_report();
    test_the_wheel_is_sixty_four_and_sixty_five();
    test_a_record_that_changed_nothing_says_nothing();
    test_the_clock_runs_forwards();
    test_a_sleep_of_a_frame_is_about_a_frame();
    test_a_pipe_with_no_reader_is_an_error_not_a_death();
    test_a_long_write_arrives_in_whole();
    return 0;
}
