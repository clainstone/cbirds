#define _XOPEN_SOURCE 700 /* M_PI. */

#include "../sign.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "../font.h"

static void test_the_text_is_cleaned_down_to_what_the_font_can_draw(void) {
    char out[SIGN_TEXT_MAX];

    /* Lower case maps to upper case: the font has one case. */
    assert(sign_clean("Hello, world!", out, sizeof(out)) == 12);
    assert(strcmp(out, "HELLO, WORLD!") == 0);

    /* Characters the font lacks are skipped, not turned into spaces: a word with
     * an accent in it is a word with a letter missing and no more. */
    assert(sign_clean("caf\xc3\xa9 \xe2\x82\xac"
                      "5",
                      out, sizeof(out)) == 4);
    assert(strcmp(out, "CAF 5") == 0);
    /* The font carries the rest of printable ASCII as well, so a tilde is drawn;
     * it is the control characters and the bytes past ASCII that it lacks. */
    assert(sign_clean("a~b", out, sizeof(out)) == 3);
    assert(strcmp(out, "A~B") == 0);
    assert(sign_clean("a\x01\x7f" "b", out, sizeof(out)) == 2);
    assert(strcmp(out, "AB") == 0);

    /* Spaces: one between words, none at the ends, and a new line or a tab is a
     * space. */
    assert(sign_clean("  back \t in\n\n five  ", out, sizeof(out)) == 10);
    assert(strcmp(out, "BACK IN FIVE") == 0);

    /* Nothing left: said as zero, and the caller says so on stderr. */
    assert(sign_clean("", out, sizeof(out)) == 0 && out[0] == '\0');
    assert(sign_clean("   ", out, sizeof(out)) == 0 && out[0] == '\0');
    assert(sign_clean("\x01\x02\xff", out, sizeof(out)) == 0 && out[0] == '\0');
    assert(sign_clean(NULL, out, sizeof(out)) == 0);

    /* It never writes past what it was given room for. */
    char small[8];
    assert(sign_clean("abcdefghijklmnop", small, sizeof(small)) == 7);
    assert(strcmp(small, "ABCDEFG") == 0);
    char guarded[4 + SIGN_TEXT_MAX];
    memset(guarded, '#', sizeof(guarded));
    char long_text[SIGN_TEXT_MAX * 3];
    memset(long_text, 'x', sizeof(long_text) - 1);
    long_text[sizeof(long_text) - 1] = '\0';
    sign_clean(long_text, guarded, SIGN_TEXT_MAX);
    assert(strlen(guarded) == SIGN_TEXT_MAX - 1);
    assert(guarded[SIGN_TEXT_MAX] == '#');
}

static void test_a_long_text_wraps_at_its_spaces_as_large_as_fits(void) {
    sign_lines_t lines;
    double cell;

    /* A word on its own is one line, as large as the width allows. */
    assert(sign_fit("BOIDS", 0, 290, 400, 1000, &lines, &cell) == 1);
    assert(strcmp(lines.line[0], "BOIDS") == 0);
    assert(fabs(cell - 290.0 / sign_columns("BOIDS")) < 1e-9);

    /* Two words that fit side by side in a wide box stay on one line... */
    assert(sign_fit("GOOD DAY", 0, 800, 120, 1000, &lines, &cell) == 1);
    /* ...and in a narrow one are two, because the letters come out larger. */
    assert(sign_fit("GOOD DAY", 0, 200, 300, 1000, &lines, &cell) == 2);
    assert(strcmp(lines.line[0], "GOOD") == 0 && strcmp(lines.line[1], "DAY") == 0);
    assert(cell > 200.0 / sign_columns("GOOD DAY"));

    /* Never more than three lines, and a word is never split. */
    const char *long_text = "THE QUICK BROWN FOX JUMPS OVER THE LAZY DOG AND KEEPS GOING";
    int count = sign_fit(long_text, 0, 600, 400, 1000, &lines, &cell);
    assert(count >= 2 && count <= SIGN_MAX_LINES && count == lines.count);
    char joined[SIGN_TEXT_MAX] = "";
    for (int l = 0; l < count; l++) {
        assert(lines.line[l][0] != ' ' && lines.line[l][strlen(lines.line[l]) - 1] != ' ');
        if (l > 0) strcat(joined, " ");
        strcat(joined, lines.line[l]);
    }
    assert(strcmp(joined, long_text) == 0); /* The same words in the same order. */

    /* The lines fit the box they were fitted to, in both directions. */
    int widest = 0;
    for (int l = 0; l < count; l++)
        if (sign_columns(lines.line[l]) > widest) widest = sign_columns(lines.line[l]);
    assert(widest * cell <= 600 + 1e-9);
    assert(sign_rows(count) * cell <= 400 + 1e-9);
    /* And it is as large as any way of breaking it would have been, worked out the
     * slow way: every place a line can break, for one, two and three lines. A tie
     * goes to fewer lines, so it may be a hair under the best and never over. */
    int word_length[16], words = 0;
    for (const char *c = long_text; *c != '\0';) {
        const char *end = strchr(c, ' ');
        word_length[words++] = end != NULL ? (int)(end - c) : (int)strlen(c);
        c = end != NULL ? end + 1 : c + strlen(c);
    }
    double best = 0;
    for (int first = 1; first <= words; first++) {
        for (int second = first == words ? words : first + 1; second <= words; second++) {
            int breaks[3] = {first, second, words};
            int line_total = first == words ? 1 : (second == words ? 2 : 3);
            int start = 0, widest_line = 0;
            for (int l = 0; l < line_total; l++) {
                int letters = 0;
                for (int w = start; w < breaks[l]; w++) letters += word_length[w] + (w > start);
                if (letters * FONT_ADVANCE - 1 > widest_line)
                    widest_line = letters * FONT_ADVANCE - 1;
                start = breaks[l];
            }
            double size = 600.0 / widest_line;
            if (400.0 / sign_rows(line_total) < size) size = 400.0 / sign_rows(line_total);
            if (size > best) best = size;
        }
    }
    assert(cell <= best + 1e-9 && cell >= best / 1.03 - 1e-9);

    /* A long word makes the lines no narrower than itself. */
    assert(sign_fit("A SUPERCALIFRAGILISTIC DAY", 0, 900, 600, 1000, &lines, &cell) >= 1);
    int saw_the_word = 0;
    for (int l = 0; l < lines.count; l++)
        if (strstr(lines.line[l], "SUPERCALIFRAGILISTIC") != NULL) saw_the_word = 1;
    assert(saw_the_word);

    /* Too long for the room at one pixel a cell, there is no sign. */
    assert(sign_fit("A WORD THAT IS FAR TOO LONG FOR THE ROOM", 0, 30, 30, 1000, &lines, &cell) ==
           0);
    assert(lines.count == 0);
    assert(sign_fit("", 0, 800, 600, 1000, &lines, &cell) == 0);
    assert(sign_fit("HI", 0, 0, 600, 1000, &lines, &cell) == 0);
}

static void test_a_cell_is_never_larger_than_the_largest(void) {
    sign_lines_t lines;
    double cell;
    assert(sign_fit("HI", 0, 1000, 1000, 40, &lines, &cell) == 1);
    assert(cell == 40);
    assert(sign_fit("HI", 0, 100, 1000, 40, &lines, &cell) == 1);
    assert(cell < 40);
}

static void test_a_reference_width_keeps_a_clock_the_same_size(void) {
    sign_lines_t lines;
    double cell_wide, cell_narrow, cell_unreferenced;
    int reference = sign_columns("00:00");
    assert(reference == 5 * FONT_ADVANCE - 1);

    /* 10:09 and 1:09 are different widths, and a clock that changed size at ten
     * o'clock would be a clock that jumped. */
    assert(sign_fit("10:09", reference, 500, 300, 1000, &lines, &cell_wide) == 1);
    assert(sign_fit("1:09", reference, 500, 300, 1000, &lines, &cell_narrow) == 1);
    assert(cell_wide == cell_narrow);
    assert(sign_fit("1:09", 0, 500, 300, 1000, &lines, &cell_unreferenced) == 1);
    assert(cell_unreferenced > cell_narrow);
}

static void test_the_clock_text_follows_the_convention(void) {
    struct tm when;
    char out[16];
    memset(&when, 0, sizeof(when));

    /* 24 hour: two digits each. */
    when.tm_hour = 0;
    when.tm_min = 5;
    sign_clock_text(&when, 0, out, sizeof(out));
    assert(strcmp(out, "00:05") == 0);
    when.tm_hour = 13;
    when.tm_min = 7;
    sign_clock_text(&when, 0, out, sizeof(out));
    assert(strcmp(out, "13:07") == 0);
    when.tm_hour = 23;
    when.tm_min = 59;
    sign_clock_text(&when, 0, out, sizeof(out));
    assert(strcmp(out, "23:59") == 0);

    /* 12 hour: no AM and no PM, midnight and noon are twelve, and no zero in front
     * of the hour. */
    when.tm_hour = 0;
    when.tm_min = 5;
    sign_clock_text(&when, 1, out, sizeof(out));
    assert(strcmp(out, "12:05") == 0);
    when.tm_hour = 12;
    when.tm_min = 0;
    sign_clock_text(&when, 1, out, sizeof(out));
    assert(strcmp(out, "12:00") == 0);
    when.tm_hour = 13;
    when.tm_min = 7;
    sign_clock_text(&when, 1, out, sizeof(out));
    assert(strcmp(out, "1:07") == 0);
    when.tm_hour = 9;
    when.tm_min = 30;
    sign_clock_text(&when, 1, out, sizeof(out));
    assert(strcmp(out, "9:30") == 0);
    when.tm_hour = 23;
    when.tm_min = 59;
    sign_clock_text(&when, 1, out, sizeof(out));
    assert(strcmp(out, "11:59") == 0);

    /* Every minute of the day reads as time in both, and fits the font. */
    for (int minute = 0; minute < 24 * 60; minute++) {
        when.tm_hour = minute / 60;
        when.tm_min = minute % 60;
        for (int twelve = 0; twelve < 2; twelve++) {
            sign_clock_text(&when, twelve, out, sizeof(out));
            char clean[16];
            assert(sign_clean(out, clean, sizeof(clean)) == (int)strlen(out));
            assert(strlen(out) <= 5 && strchr(out, ':') != NULL);
        }
    }
}

static void test_the_locale_says_which_clock(void) {
    /* What nl_langinfo(T_FMT) gives in the locales people use: en_US is %r, most
     * of the rest are %T or %H:%M:%S. */
    assert(sign_wants_twelve_hours("%r"));
    assert(sign_wants_twelve_hours("%I:%M:%S %p"));
    assert(sign_wants_twelve_hours("%l:%M:%S %p"));
    assert(sign_wants_twelve_hours("%-I:%M"));
    assert(sign_wants_twelve_hours("%_l:%M"));
    assert(sign_wants_twelve_hours("%EI:%M"));
    assert(sign_wants_twelve_hours("at %I"));
    assert(!sign_wants_twelve_hours("%H:%M:%S"));
    assert(!sign_wants_twelve_hours("%T"));
    assert(!sign_wants_twelve_hours("%R"));
    assert(!sign_wants_twelve_hours("%k:%M"));
    assert(!sign_wants_twelve_hours("%-H.%M"));
    assert(!sign_wants_twelve_hours(""));
    assert(!sign_wants_twelve_hours(NULL));
    /* A literal percent sign followed by an I is not a conversion. */
    assert(!sign_wants_twelve_hours("%%I %H"));
    assert(!sign_wants_twelve_hours("100%"));
}

static void test_a_hovering_bird_stays_within_its_loop(void) {
    const double radius = 5.0;
    double widest_seen = 0;

    for (unsigned id = 0; id < 2000; id++) {
        double first_x = 0, first_y = 0, last_x = 0, last_y = 0;
        for (int step = 0; step <= 600; step++) {
            double dx, dy;
            sign_hover(id, step / 60.0, radius, &dx, &dy);
            double away = sqrt(dx * dx + dy * dy);
            assert(away <= radius + 1e-9);
            if (away > widest_seen) widest_seen = away;
            /* It moves smoothly: a bird that hopped from one side of its loop to
             * the other between frames would not be hovering. Fastest it can go is
             * a loop of the largest size at the quickest pace. */
            if (step > 0) {
                double hop = hypot(dx - last_x, dy - last_y);
                assert(hop <= 2 * M_PI * 1.4 * radius / 60.0 + 1e-9);
            } else {
                first_x = dx;
                first_y = dy;
            }
            last_x = dx;
            last_y = dy;
        }
        (void)first_x;
        (void)first_y;
    }
    /* And it does use the room it is given: some bird comes near the radius. */
    assert(widest_seen > 0.9 * radius);
}

static void test_every_bird_has_a_loop_of_its_own(void) {
    double x_a, y_a, x_b, y_b;
    int different = 0;

    /* The same bird at the same moment is the same place. */
    sign_hover(17, 3.25, 5, &x_a, &y_a);
    sign_hover(17, 3.25, 5, &x_b, &y_b);
    assert(x_a == x_b && y_a == y_b);

    /* Birds differ in phase and pace: at one moment they are not all in one place,
     * and as time goes on they do not stay in step. */
    for (unsigned id = 1; id < 200; id++) {
        sign_hover(0, 1.0, 5, &x_a, &y_a);
        sign_hover(id, 1.0, 5, &x_b, &y_b);
        if (hypot(x_a - x_b, y_a - y_b) > 0.1) different++;
    }
    assert(different > 190);
    int pace_differs = 0;
    for (unsigned id = 1; id < 200; id++) {
        /* A loop is closed when it comes back: the time it takes is its pace. Find
         * where the bird is again after one second and after ten, and count the
         * birds that are not in step with bird zero over the ten. */
        double ax, ay, bx, by, cx, cy, dx, dy;
        sign_hover(0, 0, 5, &ax, &ay);
        sign_hover(0, 10, 5, &bx, &by);
        sign_hover(id, 0, 5, &cx, &cy);
        sign_hover(id, 10, 5, &dx, &dy);
        if (fabs((hypot(ax - cx, ay - cy)) - hypot(bx - dx, by - dy)) > 0.1) pace_differs++;
    }
    assert(pace_differs > 150);
    /* And at no radius, no loop. */
    sign_hover(3, 2.0, 0, &x_a, &y_a);
    assert(x_a == 0 && y_a == 0);
}

static void test_a_sign_is_held_long_and_let_go_short(void) {
    double hold_least = 1e9, hold_most = 0, flight_least = 1e9, flight_most = 0;
    int hold_varies = 0;

    for (unsigned cycle = 0; cycle < 500; cycle++) {
        double hold = sign_hold_seconds(cycle), flight = sign_flight_seconds(cycle);
        if (hold < hold_least) hold_least = hold;
        if (hold > hold_most) hold_most = hold;
        if (flight < flight_least) flight_least = flight;
        if (flight > flight_most) flight_most = flight;
        /* The same call, the same answer: at any frame rate. */
        assert(hold == sign_hold_seconds(cycle) && flight == sign_flight_seconds(cycle));
        if (hold != sign_hold_seconds(cycle + 1)) hold_varies = 1;
    }
    /* Thirty to forty-five seconds held, ten to fifteen flown. */
    assert(hold_least >= 30 && hold_most <= 45);
    assert(flight_least >= 8 && flight_most <= 12);
    assert(hold_most - hold_least > 10 && flight_most - flight_least > 2);
    assert(hold_varies);
}

static void test_the_colon_breathes_once_a_second(void) {
    assert(fabs(sign_breath(0.0)) < 1e-12);
    assert(fabs(sign_breath(0.5) - 1.0) < 1e-12);
    assert(fabs(sign_breath(1.0)) < 1e-12);
    for (int i = 0; i <= 100; i++) {
        double breath = sign_breath(i / 100.0);
        assert(breath >= -1e-12 && breath <= 1 + 1e-12);
        /* Up to the middle of the second, and back down after it. */
        if (i > 0 && i <= 50) assert(breath >= sign_breath((i - 1) / 100.0));
        if (i > 50) assert(breath <= sign_breath((i - 1) / 100.0));
    }
}

static void test_the_sign_is_a_box_the_rest_of_the_flock_keeps_out_of(void) {
    sign_box_t box = {100, 100, 300, 200};
    double push_x, push_y;

    /* Nothing from further off than the band. */
    assert(!sign_box_push(&box, 50, 40, 150, &push_x, &push_y));
    assert(push_x == 0 && push_y == 0);
    assert(!sign_box_push(&box, 50, 350.5, 150, &push_x, &push_y));
    assert(!sign_box_push(&box, 50, 200, 40, &push_x, &push_y));
    assert(!sign_box_push(&box, 50, 0, 0, &push_x, &push_y));

    /* Away from the box, and harder the nearer. */
    assert(sign_box_push(&box, 50, 90, 150, &push_x, &push_y));
    assert(push_x < 0 && fabs(push_y) < 1e-12);
    double far_push = push_x;
    assert(sign_box_push(&box, 50, 99, 150, &push_x, &push_y));
    assert(push_x < far_push);
    assert(sign_box_push(&box, 50, 310, 150, &push_x, &push_y));
    assert(push_x > 0);
    assert(sign_box_push(&box, 50, 200, 90, &push_x, &push_y));
    assert(push_y < 0 && fabs(push_x) < 1e-12);
    assert(sign_box_push(&box, 50, 200, 215, &push_x, &push_y));
    assert(push_y > 0);
    /* Off a corner, along the line from it. */
    assert(sign_box_push(&box, 50, 90, 90, &push_x, &push_y));
    assert(push_x < 0 && push_y < 0 && fabs(push_x - push_y) < 1e-9);
    /* At the edge it is one, and never more than one on the outside. */
    assert(sign_box_push(&box, 50, 99.999, 150, &push_x, &push_y));
    assert(fabs(push_x) > 0.99 && fabs(push_x) <= 1.0);

    /* Inside, out through the nearest side, harder further in. */
    assert(sign_box_push(&box, 50, 110, 150, &push_x, &push_y));
    assert(push_x < 0 && push_y == 0 && fabs(push_x) >= 1);
    double near_the_edge = fabs(push_x);
    assert(sign_box_push(&box, 50, 150, 150, &push_x, &push_y));
    assert(fabs(push_x) > 0 || fabs(push_y) > 0);
    assert(hypot(push_x, push_y) >= near_the_edge);
    assert(sign_box_push(&box, 50, 290, 150, &push_x, &push_y));
    assert(push_x > 0);
    assert(sign_box_push(&box, 50, 200, 195, &push_x, &push_y));
    assert(push_y > 0);
    assert(hypot(push_x, push_y) <= 2.0 + 1e-9);
}

static void test_a_unit_belongs_to_its_id(void) {
    for (unsigned id = 0; id < 1000; id++) {
        double unit = sign_unit(id);
        assert(unit >= 0 && unit < 1);
        assert(unit == sign_unit(id));
    }
    assert(sign_unit(1) != sign_unit(2));
}

int main(void) {
    test_the_text_is_cleaned_down_to_what_the_font_can_draw();
    test_a_long_text_wraps_at_its_spaces_as_large_as_fits();
    test_a_cell_is_never_larger_than_the_largest();
    test_a_reference_width_keeps_a_clock_the_same_size();
    test_the_clock_text_follows_the_convention();
    test_the_locale_says_which_clock();
    test_a_hovering_bird_stays_within_its_loop();
    test_every_bird_has_a_loop_of_its_own();
    test_a_sign_is_held_long_and_let_go_short();
    test_the_colon_breathes_once_a_second();
    test_the_sign_is_a_box_the_rest_of_the_flock_keeps_out_of();
    test_a_unit_belongs_to_its_id();
    return 0;
}
