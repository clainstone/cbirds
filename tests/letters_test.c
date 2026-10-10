/* Feature test macros must precede every include. */
#define _XOPEN_SOURCE 700
#define _DEFAULT_SOURCE
#define _DARWIN_C_SOURCE

#include "../letters.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

enum { CW = 8, CH = 16 };

/* A generator of the tests' own, so that nothing here depends on the C library. */
static unsigned long long rng_state = 88172645463325252ull;
static double test_random(void) {
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 7;
    rng_state ^= rng_state << 17;
    return (double)(rng_state >> 11) / (double)(1ull << 53);
}

static void feed(vt_t *vt, const char *text) {
    vt_feed(vt, text, strlen(text));
}

static int build(letters_t *letters, vt_t *vt, const char *text, int cols, int rows) {
    assert(vt_init(vt, cols, rows) == 0);
    feed(vt, text);
    return letters_build(letters, vt, CW, CH, 100000, test_random);
}

/* The letters' positions as the simulation would hold them: a parallel array that
 * the tests move by hand. */
typedef struct {
    double x[8192], y[8192];
    int shade[8192];
} poses_t;

static void read_pose(const void *context, int index, letters_pose_t *pose) {
    const poses_t *poses = context;
    pose->x = poses->x[index];
    pose->y = poses->y[index];
    pose->shade = poses->shade[index];
}

static void rest_at_home(const letters_t *letters, poses_t *poses) {
    for (int i = 0; i < letters->count; i++) {
        poses->x[i] = letters->letter[i].home_x;
        poses->y[i] = letters->letter[i].home_y;
        poses->shade[i] = 0;
    }
}

static const double STEP = 1.0 / 60.0;

/* Cells to compare: what a paint produced, copied out. */
typedef struct {
    int cols, rows;
    cell_t cells[64 * 64];
} snapshot_t;

static void paint_now(const letters_t *letters, cells_t *cells, const poses_t *poses,
                      const letters_look_t *look, snapshot_t *out) {
    letters_paint(letters, cells, read_pose, poses, look);
    out->cols = cells->cols;
    out->rows = cells->rows;
    memcpy(out->cells, cells->now, (size_t)cells->cols * (size_t)cells->rows * sizeof(cell_t));
}

static int same_snapshot(const snapshot_t *a, const snapshot_t *b) {
    return a->cols == b->cols && a->rows == b->rows &&
           memcmp(a->cells, b->cells, (size_t)a->cols * (size_t)a->rows * sizeof(cell_t)) == 0;
}

static const cell_t *snap(const snapshot_t *s, int col, int row) {
    return &s->cells[row * s->cols + col];
}

static void test_every_glyph_cell_becomes_a_letter_with_its_home_in_the_middle_of_it(void) {
    letters_t letters;
    vt_t vt;
    int count = build(&letters, &vt, "ab  c\n d", 10, 4);
    assert(count == 4 && letters.count == 4 && letters.cells_with_glyphs == 4);
    assert(letters.letter[0].glyph == 'a' && letters.letter[0].col == 0 &&
           letters.letter[0].row == 0);
    assert(letters.letter[0].home_x == CW * 0.5 && letters.letter[0].home_y == CH * 0.5);
    assert(letters.letter[2].glyph == 'c' && letters.letter[2].col == 4);
    assert(letters.letter[2].home_x == CW * 4.5);
    assert(letters.letter[3].glyph == 'd' && letters.letter[3].row == 1 &&
           letters.letter[3].col == 1);
    assert(letters.letter[3].home_y == CH * 1.5);
    assert(letters.at[0] == 0 && letters.at[2] == -1 && letters.at[10 + 1] == 3);
    assert(letters.phase == LETTERS_AT_REST && letters.perched == 4);
    for (int i = 0; i < count; i++) assert(letters.letter[i].state == LETTER_PERCHED);
    letters_destroy(&letters);
    vt_destroy(&vt);
}

static void test_a_letter_carries_the_style_the_command_gave_it(void) {
    letters_t letters;
    vt_t vt;
    build(&letters, &vt, "\033[1;31mr\033[0m\033[48;5;20mb\033[38;2;1;2;3mx", 10, 2);
    assert(letters.count == 3);
    assert(letters.letter[0].style.fg.kind == VT_COLOUR_ANSI &&
           letters.letter[0].style.fg.value[0] == 1);
    assert(letters.letter[0].style.attributes & VT_BOLD);
    assert(letters.letter[1].style.bg.kind == VT_COLOUR_INDEXED);
    assert(letters.letter[2].style.fg.kind == VT_COLOUR_RGB);
    letters_destroy(&letters);
    vt_destroy(&vt);
}

static void test_blanks_are_not_letters_but_a_background_is_part_of_the_picture(void) {
    letters_t letters;
    vt_t vt;
    int count = build(&letters, &vt,
                      "a \033[44m  \033[0m b\xC2\xA0"
                      "c",
                      12, 2);
    /* a, b and c: spaces, a blue space, and a no-break space are not birds. */
    assert(count == 3);
    assert(letters.at[2] == -1 && letters.at[3] == -1);
    /* The blue spaces are part of the screen, drawn at rest and when everyone has left. */
    cells_t cells;
    assert(cells_init(&cells, 1) == CELLS_OK && cells_resize(&cells, 12, 2) == CELLS_OK);
    static poses_t poses;
    rest_at_home(&letters, &poses);
    snapshot_t at_rest, up;
    paint_now(&letters, &cells, &poses, NULL, &at_rest);
    assert(snap(&at_rest, 2, 0)->has_bg && snap(&at_rest, 2, 0)->bg_kind == CELLS_COLOUR_ANSI);
    assert(snap(&at_rest, 2, 0)->bg[0] == 4 && snap(&at_rest, 3, 0)->bg[0] == 4);
    assert(snap(&at_rest, 2, 0)->glyph == 0);
    assert(!snap(&at_rest, 4, 0)->has_bg);
    for (int i = 0; i < letters.count; i++) letters.letter[i].state = LETTER_FLYING;
    for (int i = 0; i < letters.count; i++) poses.y[i] = -100; /* Off the screen. */
    paint_now(&letters, &cells, &poses, NULL, &up);
    assert(snap(&up, 2, 0)->has_bg && snap(&up, 2, 0)->bg[0] == 4);
    assert(snap(&up, 0, 0)->glyph == 0);
    cells_destroy(&cells);
    letters_destroy(&letters);
    vt_destroy(&vt);
}

static void test_a_wide_letter_is_one_letter_over_two_cells(void) {
    letters_t letters;
    vt_t vt;
    int count = build(&letters, &vt,
                      "a\xE4\xB8\xAD"
                      "b\xE3\x80\x80"
                      "c",
                      10, 2);
    /* a, the wide character, b and c: the ideographic space is wide and blank. */
    assert(count == 4);
    assert(letters.letter[1].width == 2 && letters.letter[1].glyph == 0x4E2D);
    assert(letters.letter[1].home_x == 2.0 * CW); /* Between its two cells, 1 and 2. */
    assert(letters.at[1] == 1 && letters.at[2] == 1);
    assert(letters.at[4] == -1 && letters.at[5] == -1); /* The wide blank is two narrow blanks. */
    assert(letters.grid[4].width == 1 && letters.grid[5].width == 1);
    assert(letters.letter[3].glyph == 'c' && letters.letter[3].col == 6);
    letters_destroy(&letters);
    vt_destroy(&vt);
}

static void test_a_screen_with_nothing_on_it_has_nothing_to_fly(void) {
    letters_t letters;
    vt_t vt;
    assert(build(&letters, &vt, "", 10, 4) == 0);
    letters_destroy(&letters);
    vt_destroy(&vt);
    assert(build(&letters, &vt, "   \n\n \033[41m   \033[0m", 10, 4) == 0);
    letters_destroy(&letters);
    vt_destroy(&vt);
    /* And the calls that would have used them do nothing. */
    letters_t none;
    memset(&none, 0, sizeof(none));
    letters_advance(&none, 1.0, NULL, 0);
    letters_poke(&none);
    letters_land(&none, 0);
    letters_destroy(NULL);
    assert(letters_build(&none, NULL, 8, 16, 10, NULL) == -1);
}

static void test_letters_beyond_the_cap_stay_where_they_are(void) {
    letters_t letters;
    vt_t vt;
    assert(vt_init(&vt, 10, 2) == 0);
    feed(&vt, "abcdefghij\nklmnopqrst");
    assert(letters_build(&letters, &vt, CW, CH, 7, test_random) == 7);
    assert(letters.cells_with_glyphs == 20);
    assert(letters.letter[6].glyph == 'g');
    assert(letters.at[7] == -1 && letters.at[15] == -1);
    /* Everyone up: the capped ones are still on the screen, at rest forever. */
    cells_t cells;
    assert(cells_init(&cells, 1) == CELLS_OK && cells_resize(&cells, 10, 2) == CELLS_OK);
    static poses_t poses;
    for (int i = 0; i < letters.count; i++) {
        letters.letter[i].state = LETTER_FLYING;
        poses.x[i] = poses.y[i] = -50;
    }
    snapshot_t snapshot;
    paint_now(&letters, &cells, &poses, NULL, &snapshot);
    assert(snap(&snapshot, 6, 0)->glyph == 0);
    assert(snap(&snapshot, 7, 0)->glyph == 'h');
    assert(snap(&snapshot, 9, 1)->glyph == 't');
    cells_destroy(&cells);
    letters_destroy(&letters);
    vt_destroy(&vt);
}

static void test_at_rest_the_screen_is_exactly_the_text(void) {
    letters_t letters;
    vt_t vt;
    build(&letters, &vt,
          "\033[1;34mdir\033[0m  \033[4mname\033[0m\n\033[7m rev \033[0m \033[38;2;9;8;7mrgb\033[0m"
          "\033[102m \033[0m\xE4\xB8\xAD\n\033[31mred\033[39m plain",
          24, 4);
    cells_t cells;
    assert(cells_init(&cells, 1) == CELLS_OK && cells_resize(&cells, 24, 4) == CELLS_OK);
    static poses_t poses;
    rest_at_home(&letters, &poses);
    snapshot_t at_rest;
    paint_now(&letters, &cells, &poses, NULL, &at_rest);
    /* Cell for cell what the emulator holds. */
    for (int row = 0; row < 4; row++)
        for (int col = 0; col < 24; col++) {
            cell_t expected;
            letters_cell_from_vt(vt_cell(&vt, col, row), &expected);
            assert(memcmp(&expected, snap(&at_rest, col, row), sizeof(cell_t)) == 0);
        }
    assert(snap(&at_rest, 0, 0)->fg_kind == CELLS_COLOUR_ANSI && snap(&at_rest, 0, 0)->fg[0] == 4);
    assert(snap(&at_rest, 0, 0)->attributes == CELLS_BOLD);
    assert(snap(&at_rest, 5, 0)->attributes == CELLS_UNDERLINE);
    assert(snap(&at_rest, 1, 1)->attributes == CELLS_REVERSE &&
           snap(&at_rest, 0, 1)->attributes == CELLS_REVERSE);
    assert(snap(&at_rest, 6, 1)->fg_kind == CELLS_COLOUR_EXACT && snap(&at_rest, 6, 1)->fg[2] == 7);
    assert(snap(&at_rest, 10, 1)->wide == CELLS_WIDE_HEAD &&
           snap(&at_rest, 11, 1)->wide == CELLS_WIDE_TAIL);
    assert(snap(&at_rest, 9, 1)->has_bg && snap(&at_rest, 9, 1)->bg[0] == 10 &&
           snap(&at_rest, 9, 1)->glyph == 0);
    /* Painting does not depend on the cells' old contents. */
    snapshot_t again;
    memset(cells.now, 0xA5, (size_t)cells.cols * (size_t)cells.rows * sizeof(cell_t));
    paint_now(&letters, &cells, &poses, NULL, &again);
    assert(same_snapshot(&at_rest, &again));
    cells_destroy(&cells);
    letters_destroy(&letters);
    vt_destroy(&vt);
}

/* A full screen of one glyph, for the timings. */
static void fill_screen(char *text, size_t size, int cols, int rows) {
    size_t at = 0;
    for (int row = 0; row < rows; row++) {
        for (int col = 0; col < cols; col++) text[at++] = 'x';
        if (row + 1 < rows) text[at++] = '\n';
    }
    assert(at < size);
    text[at] = '\0';
}

/* Runs the cycle for `seconds` of letters' time with nobody landing, noting the
 * frame each letter left on. */
static void run_until_everyone_is_up(letters_t *letters, int *left_on) {
    for (int i = 0; i < letters->count; i++) left_on[i] = -1;
    for (int frame = 0; frame < 60 * 30; frame++) {
        letters_advance(letters, STEP, NULL, 0);
        for (int k = 0; k < letters->launched_count; k++) left_on[letters->launched[k]] = frame;
        if (letters->phase == LETTERS_IN_FLIGHT) return;
    }
}

static void test_the_text_waits_a_few_seconds_and_then_a_wave_begins_with_one_letter(void) {
    letters_t letters;
    vt_t vt;
    static char text[4096];
    fill_screen(text, sizeof(text), 40, 10);
    build(&letters, &vt, text, 40, 10);
    double waited = 0;
    int first = -1;
    while (first < 0 && waited < 20) {
        letters_advance(&letters, STEP, NULL, 0);
        waited += STEP;
        if (letters.launched_count > 0) first = letters.launched[0];
    }
    assert(first >= 0);
    /* A few seconds, long enough to see it is only the command's output. */
    assert(waited >= LETTERS_FIRST_REST - 0.05 && waited <= LETTERS_FIRST_REST + 0.1);
    assert(letters.launched_count == 1);
    assert(letters.phase == LETTERS_TAKING_OFF && letters.origin == first);
    /* Nothing else has moved; the rest are startled and waiting where they are. */
    int startled = 0;
    for (int i = 0; i < letters.count; i++)
        if (letters.letter[i].state == LETTER_STARTLED) startled++;
    assert(startled > 0 && startled <= (2 * 12 + 1) * (2 * 4 + 1));
    letters_destroy(&letters);
    vt_destroy(&vt);
}

static void test_each_letter_leaves_after_one_beside_it_has(void) {
    letters_t letters;
    vt_t vt;
    static char text[4096];
    static int left_on[8192];
    fill_screen(text, sizeof(text), 40, 10);
    build(&letters, &vt, text, 40, 10);
    run_until_everyone_is_up(&letters, left_on);
    assert(letters.phase == LETTERS_IN_FLIGHT && letters.perched == 0);
    for (int i = 0; i < letters.count; i++) {
        assert(letters.letter[i].state == LETTER_FLYING && !letters.letter[i].solo);
        assert(left_on[i] >= 0);
    }
    /* Everyone but the first had a neighbour that left before it did, within the
     * reach of a startle, and not on the same frame. */
    int origin_count = 0;
    for (int i = 0; i < letters.count; i++) {
        int explained = 0;
        for (int j = 0; j < letters.count && !explained; j++) {
            if (j == i || left_on[j] >= left_on[i]) continue;
            int dx = abs(letters.letter[j].col - letters.letter[i].col);
            int dy = abs(letters.letter[j].row - letters.letter[i].row);
            if (dx <= 12 && dy <= 4) explained = 1;
        }
        if (!explained) origin_count++;
    }
    assert(origin_count == 1);
    letters_destroy(&letters);
    vt_destroy(&vt);
}

static void test_the_wave_takes_about_a_second_to_cross_a_screenful(void) {
    double worst = 0, best = 100, total = 0;
    int runs = 20;
    for (int run = 0; run < runs; run++) {
        letters_t letters;
        vt_t vt;
        static char text[4096];
        static int left_on[8192];
        fill_screen(text, sizeof(text), 80, 24);
        rng_state = 1234567ull + (unsigned long long)run * 7919ull;
        for (int warm = 0; warm < 50; warm++)
            test_random(); /* A seed is not random until it has been stirred. */
        build(&letters, &vt, text, 80, 24);
        run_until_everyone_is_up(&letters, left_on);
        int first = 1 << 30, last = -1;
        for (int i = 0; i < letters.count; i++) {
            if (left_on[i] < first) first = left_on[i];
            if (left_on[i] > last) last = left_on[i];
        }
        double seconds = (last - first) * STEP;
        if (seconds > worst) worst = seconds;
        if (seconds < best) best = seconds;
        total += seconds;
        letters_destroy(&letters);
        vt_destroy(&vt);
    }
    /* From a corner it crosses the whole screen, from the middle half of it: the
     * brief's second is the full crossing. Measured, so tuning it is a number. */
    if (getenv("CBIRDS_TEST_NUMBERS"))
        fprintf(stderr, "wave over 80x24: %.2f s mean, %.2f to %.2f s\n", total / runs, best,
                worst);
    assert(worst < 1.8 && best > 0.4);
}

static void test_a_word_beyond_every_reach_is_startled_across_the_gap(void) {
    letters_t letters;
    vt_t vt;
    static int left_on[8192];
    /* Two words with forty cells between them: further than any letter is startled
     * from, so the wave has to cross the gap, and takes as long as sound would. */
    build(&letters, &vt, "hello                                        world", 60, 2);
    run_until_everyone_is_up(&letters, left_on);
    assert(letters.phase == LETTERS_IN_FLIGHT);
    for (int i = 0; i < letters.count; i++) assert(left_on[i] >= 0);
    int first = 1 << 30, last = -1;
    for (int i = 0; i < letters.count; i++) {
        if (left_on[i] < first) first = left_on[i];
        if (left_on[i] > last) last = left_on[i];
    }
    double seconds = (last - first) * STEP;
    assert(seconds >= 0.4 && seconds <= 1.2);
    letters_destroy(&letters);
    vt_destroy(&vt);
}

static void test_a_wave_that_is_taking_too_long_startles_the_rest_at_once(void) {
    letters_t letters;
    vt_t vt;
    /* Words far apart in a long row: the wave would cross each gap in turn. */
    build(&letters, &vt,
          "aa                                                                    "
          "                                                                      "
          "bb                                                                    "
          "                                                                      "
          "cc",
          200, 2);
    letters_poke(&letters);
    letters_advance(&letters, STEP, NULL, 0);
    assert(letters.phase == LETTERS_TAKING_OFF && letters.perched > 0);
    /* Pretend the wave has been going for its whole allowance. */
    letters.wave_began -= LETTERS_WAVE_DEADLINE;
    int frames = 0;
    while (letters.phase == LETTERS_TAKING_OFF && frames < 60) {
        letters_advance(&letters, STEP, NULL, 0);
        frames++;
    }
    assert(letters.phase == LETTERS_IN_FLIGHT);
    assert(frames * STEP <= 0.6);
    letters_destroy(&letters);
    vt_destroy(&vt);
}

static void test_flight_lasts_fifteen_to_twenty_five_seconds_and_then_they_are_called_home(void) {
    for (int run = 0; run < 5; run++) {
        letters_t letters;
        vt_t vt;
        static int left_on[8192];
        rng_state = 99ull + (unsigned long long)run;
        build(&letters, &vt, "some text\nand more", 20, 3);
        run_until_everyone_is_up(&letters, left_on);
        double flown = 0;
        while (letters.phase == LETTERS_IN_FLIGHT && flown < 40) {
            letters_advance(&letters, STEP, NULL, 0);
            flown += STEP;
        }
        assert(letters.phase == LETTERS_COMING_HOME);
        assert(flown >= 15.0 && flown <= 25.0);
        for (int i = 0; i < letters.count; i++) assert(letters.letter[i].state == LETTER_HOMING);
        letters_destroy(&letters);
        vt_destroy(&vt);
    }
}

static void test_the_text_rests_again_once_the_last_letter_is_home(void) {
    letters_t letters;
    vt_t vt;
    static int left_on[8192];
    build(&letters, &vt, "some text\nand more", 20, 3);
    run_until_everyone_is_up(&letters, left_on);
    letters_poke(&letters); /* Enter: home. */
    assert(letters.phase == LETTERS_COMING_HOME);
    /* They land one at a time; the text is not at rest until the last has. */
    for (int i = 0; i < letters.count; i++) {
        assert(letters.phase == LETTERS_COMING_HOME);
        letters_advance(&letters, STEP, NULL, 0);
        letters_land(&letters, i);
    }
    letters_advance(&letters, STEP, NULL, 0);
    assert(letters.phase == LETTERS_AT_REST && letters.perched == letters.count);
    /* A pause, longer than the first one, and then the wave again. */
    assert(LETTERS_REST > LETTERS_FIRST_REST);
    double waited = 0;
    while (letters.phase == LETTERS_AT_REST && waited < 30) {
        letters_advance(&letters, STEP, NULL, 0);
        waited += STEP;
    }
    assert(waited >= LETTERS_REST - 0.1 && waited <= LETTERS_REST + 0.1);
    assert(letters.phase == LETTERS_TAKING_OFF && letters.cycles == 2);
    letters_destroy(&letters);
    vt_destroy(&vt);
}

static void test_enter_sends_the_letters_off_and_then_calls_them_home(void) {
    letters_t letters;
    vt_t vt;
    build(&letters, &vt, "some text\nand more", 20, 3);
    letters_advance(&letters, STEP, NULL, 0);
    assert(letters.phase == LETTERS_AT_REST);
    letters_poke(&letters);
    letters_advance(&letters, STEP, NULL, 0);
    assert(letters.phase == LETTERS_TAKING_OFF && letters.launched_count == 1);

    /* Called home in the middle of the wave: the letters that had not left stay. */
    letters_poke(&letters);
    assert(letters.phase == LETTERS_COMING_HOME);
    int airborne = 0, perched = 0;
    for (int i = 0; i < letters.count; i++) {
        if (letters.letter[i].state == LETTER_HOMING) airborne++;
        if (letters.letter[i].state == LETTER_PERCHED) perched++;
        assert(letters.letter[i].state != LETTER_STARTLED);
    }
    assert(airborne == 1 && perched == letters.count - 1 && letters.perched == perched);
    /* Enter while they are coming home does nothing. */
    letters_poke(&letters);
    assert(letters.phase == LETTERS_COMING_HOME);
    letters_land(&letters, letters.origin);
    letters_advance(&letters, STEP, NULL, 0);
    assert(letters.phase == LETTERS_AT_REST);
    letters_destroy(&letters);
    vt_destroy(&vt);
}

static letters_disturbance_t pointer_at(double x, double y) {
    letters_disturbance_t pointer = {x, y, 20, 26, 60, 60};
    return pointer;
}

static void test_the_pointer_scatters_the_letters_it_touches_and_they_come_home_after_it(void) {
    letters_t letters;
    vt_t vt;
    static char text[4096];
    fill_screen(text, sizeof(text), 40, 10);
    build(&letters, &vt, text, 40, 10);
    double x = 20.5 * CW, y = 5.5 * CH;
    letters_disturbance_t pointer = pointer_at(x, y);
    letters_advance(&letters, STEP, &pointer, 1);
    int touched = letters.launched_count;
    assert(touched > 3 && touched < 40);
    for (int k = 0; k < touched; k++) {
        const letter_t *letter = &letters.letter[letters.launched[k]];
        assert(letter->state == LETTER_FLYING && letter->solo);
        double dx = letter->home_x - x, dy = letter->home_y - y;
        assert((dx * dx) / (20.0 * 20.0) + (dy * dy) / (26.0 * 26.0) <= 1.0);
    }
    assert(letters.phase == LETTERS_AT_REST);
    assert(letters.last_touched >= 0);
    /* The letters just outside the ellipse are untouched. */
    int untouched = letters.perched;
    assert(untouched == letters.count - touched);

    /* While the pointer is still near their homes they stay up, and they stay up
     * for a little in any case. */
    for (int frame = 0; frame < 120; frame++) letters_advance(&letters, STEP, &pointer, 1);
    int still_up = 0;
    for (int i = 0; i < letters.count; i++)
        if (letters.letter[i].state == LETTER_FLYING) still_up++;
    assert(still_up == touched);

    /* The pointer moves off; now they are called home, one by one, not before the
     * time they stay up for. */
    letters_disturbance_t gone = pointer_at(-500, -500);
    int homing_at_once = 0;
    letters_advance(&letters, STEP, &gone, 1);
    for (int i = 0; i < letters.count; i++)
        if (letters.letter[i].state == LETTER_HOMING) homing_at_once++;
    assert(homing_at_once == touched); /* Already up for two seconds. */
    for (int i = 0; i < letters.count; i++)
        if (letters.letter[i].state == LETTER_HOMING) letters_land(&letters, i);
    assert(letters.perched == letters.count);
    letters_destroy(&letters);
    vt_destroy(&vt);
}

static void test_a_letter_scattered_a_moment_ago_stays_up_a_moment_even_if_the_pointer_has_gone(
    void) {
    letters_t letters;
    vt_t vt;
    build(&letters, &vt, "abcdefgh", 20, 2);
    letters_disturbance_t pointer = pointer_at(3.5 * CW, 0.5 * CH);
    letters_advance(&letters, STEP, &pointer, 1);
    assert(letters.launched_count > 0);
    letters_disturbance_t gone = pointer_at(-500, -500);
    for (int frame = 0; frame < 30; frame++) letters_advance(&letters, STEP, &gone, 1);
    for (int i = 0; i < letters.count; i++) assert(letters.letter[i].state != LETTER_HOMING);
    for (int frame = 0; frame < 60; frame++) letters_advance(&letters, STEP, &gone, 1);
    int homing = 0;
    for (int i = 0; i < letters.count; i++)
        if (letters.letter[i].state == LETTER_HOMING) homing++;
    assert(homing > 0);
    letters_destroy(&letters);
    vt_destroy(&vt);
}

static void test_the_wave_starts_where_the_pointer_last_touched(void) {
    letters_t letters;
    vt_t vt;
    static char text[4096];
    fill_screen(text, sizeof(text), 40, 10);
    build(&letters, &vt, text, 40, 10);
    letters_disturbance_t pointer = pointer_at(30.5 * CW, 7.5 * CH);
    letters_advance(&letters, STEP, &pointer, 1);
    int touched = letters.last_touched;
    assert(touched >= 0);
    /* Everything it touched comes home first, then the wave begins. */
    letters_disturbance_t gone = pointer_at(-500, -500);
    for (int frame = 0; frame < 120; frame++) letters_advance(&letters, STEP, &gone, 1);
    for (int i = 0; i < letters.count; i++) letters_land(&letters, i);
    letters.rest_left = 0.01;
    letters_advance(&letters, STEP, &gone, 1);
    assert(letters.phase == LETTERS_TAKING_OFF);
    assert(letters.origin == touched);
    letters_destroy(&letters);
    vt_destroy(&vt);
}

static void test_the_letters_in_the_air_join_the_cycle_when_the_wave_comes(void) {
    letters_t letters;
    vt_t vt;
    build(&letters, &vt, "abcdefgh\nabcdefgh", 20, 3);
    letters_disturbance_t pointer = pointer_at(3.5 * CW, 0.5 * CH);
    letters_advance(&letters, STEP, &pointer, 1);
    assert(letters.launched_count > 0);
    letters.rest_left = 0.001;
    letters_advance(&letters, STEP, &pointer, 1);
    assert(letters.phase == LETTERS_TAKING_OFF);
    for (int i = 0; i < letters.count; i++) assert(!letters.letter[i].solo);
    letters_destroy(&letters);
    vt_destroy(&vt);
}

static void test_a_hawk_over_the_text_scatters_it_too(void) {
    letters_t letters;
    vt_t vt;
    build(&letters, &vt, "abcdefghijklmnop", 20, 2);
    letters_disturbance_t hawk = {8 * CW, 0.5 * CH, 40, 40, 80, 80};
    letters_advance(&letters, STEP, &hawk, 1);
    assert(letters.launched_count >= 6);
    letters_destroy(&letters);
    vt_destroy(&vt);
}

/* The flock as the program is: it moves a letter only if it was told, in the list of
 * who left, that it had; and a letter that has come home is landed. */
static void fly_what_was_told(letters_t *letters, int *told) {
    for (int k = 0; k < letters->launched_count; k++) told[letters->launched[k]] = 1;
    for (int i = 0; i < letters->count; i++)
        if (letters->letter[i].state == LETTER_HOMING && told[i]) {
            letters_land(letters, i);
            told[i] = 0;
        }
}

static void test_letters_launched_in_the_step_the_rest_ends_are_told_to_the_flock(void) {
    letters_t letters;
    vt_t vt;
    build(&letters, &vt, "hello world, this is some text\r\nsecond line of text here\r\nthird line",
          40, 5);
    int count = letters.count;
    /* A pointer over the first letters, in the very step the rest runs out. */
    letters.rest_left = 0.01;
    letters_disturbance_t pointer = {4 + CW * 3, CH / 2, 40, 40, 80, 80};
    letters_advance(&letters, STEP, &pointer, 1);
    assert(letters.phase == LETTERS_TAKING_OFF || letters.phase == LETTERS_IN_FLIGHT);
    int airborne = 0, once = 1;
    for (int i = 0; i < count; i++) {
        if (!letter_is_airborne(&letters.letter[i])) continue;
        airborne++;
        int listed = 0;
        for (int k = 0; k < letters.launched_count; k++) listed += letters.launched[k] == i;
        if (listed != 1) once = 0;
    }
    /* The ones the pointer touched and the first of the wave: every one in the air is
     * on the list, once, and nothing is on it that is not in the air. */
    assert(airborne > 1 && once && letters.launched_count == airborne);
    assert(letters.perched == count - airborne);

    /* And the cycle goes on: a flock that moves only who it was told of lands them
     * all, the text rests, and the wave comes again. */
    static int told[8192];
    memset(told, 0, sizeof(told));
    fly_what_was_told(&letters, told);
    int rested = 0;
    for (int step = 0; step < 60 * 120; step++) {
        letters_advance(&letters, STEP, NULL, 0);
        fly_what_was_told(&letters, told);
        for (int i = 0; i < count; i++) assert(!letter_is_airborne(&letters.letter[i]) || told[i]);
        if (letters.phase == LETTERS_AT_REST) rested = 1;
    }
    assert(rested && letters.cycles >= 3);
    letters_destroy(&letters);
    vt_destroy(&vt);
}

static void test_a_pointer_far_beyond_the_text_or_not_a_number_touches_nothing(void) {
    static const double FAR[] = {1e9, 3e9, -3e9, 1e18, 1e300, -1e300, HUGE_VAL, -HUGE_VAL, NAN};
    for (size_t x = 0; x < sizeof(FAR) / sizeof(*FAR); x++)
        for (size_t y = 0; y < sizeof(FAR) / sizeof(*FAR); y++) {
            letters_t letters;
            vt_t vt;
            build(&letters, &vt, "abcdefghij\nklmnopqrst", 10, 2);
            letters_disturbance_t pointer = {FAR[x], FAR[y], 40, 40, 80, 80};
            letters_advance(&letters, STEP, &pointer, 1);
            /* A pointer a screen or more away is not over the text. */
            assert(letters.launched_count == 0 && letters.perched == letters.count);
            letters_destroy(&letters);
            vt_destroy(&vt);
        }
    /* Beyond one edge and level with the text along the other: the letters in line
     * with it are touched, as they are for a pointer a cell away. */
    letters_t letters;
    vt_t vt;
    build(&letters, &vt, "abcdefghij\nklmnopqrst", 10, 2);
    letters_disturbance_t pointer = {5 * CW, CH, 3e9, 3e9, 80, 80};
    letters_advance(&letters, STEP, &pointer, 1);
    assert(letters.launched_count == letters.count);
    letters_destroy(&letters);
    vt_destroy(&vt);
}

static void test_a_clock_that_is_not_a_time_does_not_stop_the_cycle(void) {
    letters_t letters;
    vt_t vt;
    build(&letters, &vt, "some text\nand more", 20, 3);
    letters_advance(&letters, NAN, NULL, 0);
    letters_advance(&letters, -1.0, NULL, 0);
    letters_advance(&letters, INFINITY, NULL, 0);
    assert(isfinite(letters.clock) && isfinite(letters.rest_left));
    /* And it still runs: the first rest ends. */
    double waited = 0;
    while (letters.phase == LETTERS_AT_REST && waited < 30) {
        letters_advance(&letters, STEP, NULL, 0);
        waited += STEP;
    }
    assert(letters.phase == LETTERS_TAKING_OFF);
    letters_destroy(&letters);
    vt_destroy(&vt);
}

static void test_a_long_pause_in_the_clock_does_not_skip_the_wave(void) {
    letters_t letters;
    vt_t vt;
    static char text[4096];
    fill_screen(text, sizeof(text), 40, 10);
    build(&letters, &vt, text, 40, 10);
    letters_advance(&letters, 100.0, NULL, 0); /* A terminal suspended for a minute and a half. */
    assert(letters.phase == LETTERS_AT_REST);
    letters.rest_left = 0.0;
    letters_advance(&letters, 100.0, NULL, 0);
    assert(letters.phase == LETTERS_TAKING_OFF && letters.perched > 0);
    letters_destroy(&letters);
    vt_destroy(&vt);
}

/* --- Painting letters in the air ------------------------------------------------ */

static void test_a_letter_in_the_air_is_drawn_where_it_is_and_its_home_is_left_empty(void) {
    letters_t letters;
    vt_t vt;
    build(&letters, &vt, "\033[1;31mab\033[0m\n\033[44m \033[0m", 10, 4);
    cells_t cells;
    assert(cells_init(&cells, 1) == CELLS_OK && cells_resize(&cells, 10, 4) == CELLS_OK);
    static poses_t poses;
    rest_at_home(&letters, &poses);
    letters.letter[1].state = LETTER_FLYING; /* The b. */
    letters.perched--;
    poses.x[1] = 5.5 * CW;
    poses.y[1] = 2.5 * CH;
    snapshot_t s;
    paint_now(&letters, &cells, &poses, NULL, &s);
    assert(snap(&s, 0, 0)->glyph == 'a');
    assert(snap(&s, 1, 0)->glyph == 0); /* Home, vacated. */
    assert(snap(&s, 5, 2)->glyph == 'b');
    assert(snap(&s, 5, 2)->fg_kind == CELLS_COLOUR_ANSI && snap(&s, 5, 2)->fg[0] == 1);
    assert(snap(&s, 5, 2)->attributes == CELLS_BOLD);
    /* Over the blue space it takes the blue for a background. */
    poses.x[1] = 0.5 * CW;
    poses.y[1] = 1.5 * CH;
    paint_now(&letters, &cells, &poses, NULL, &s);
    assert(snap(&s, 0, 1)->glyph == 'b' && snap(&s, 0, 1)->has_bg && snap(&s, 0, 1)->bg[0] == 4);
    /* Drawn over a letter at rest, it hides it, and the letter is back when it moves on. */
    poses.x[1] = 0.5 * CW;
    poses.y[1] = 0.5 * CH;
    paint_now(&letters, &cells, &poses, NULL, &s);
    assert(snap(&s, 0, 0)->glyph == 'b');
    poses.x[1] = 3.5 * CW;
    paint_now(&letters, &cells, &poses, NULL, &s);
    assert(snap(&s, 0, 0)->glyph == 'a');
    cells_destroy(&cells);
    letters_destroy(&letters);
    vt_destroy(&vt);
}

static void test_a_background_stays_when_the_letter_on_it_leaves(void) {
    letters_t letters;
    vt_t vt;
    build(&letters, &vt, "\033[43;30m hi \033[0m", 10, 2);
    cells_t cells;
    assert(cells_init(&cells, 1) == CELLS_OK && cells_resize(&cells, 10, 2) == CELLS_OK);
    static poses_t poses;
    rest_at_home(&letters, &poses);
    letters.letter[0].state = LETTER_FLYING;
    letters.perched--;
    poses.x[0] = poses.y[0] = -10;
    snapshot_t s;
    paint_now(&letters, &cells, &poses, NULL, &s);
    /* The h is gone and the yellow it sat on is not. */
    assert(snap(&s, 1, 0)->glyph == 0 && snap(&s, 1, 0)->has_bg && snap(&s, 1, 0)->bg[0] == 3);
    assert(!snap(&s, 1, 0)->has_fg);
    assert(snap(&s, 2, 0)->glyph == 'i' && snap(&s, 2, 0)->fg[0] == 0);
    cells_destroy(&cells);
    letters_destroy(&letters);
    vt_destroy(&vt);
}

static void test_attributes_belong_to_the_letter_wherever_it_is(void) {
    letters_t letters;
    vt_t vt;
    build(&letters, &vt, "\033[7;4;1mR\033[0m", 10, 2);
    cells_t cells;
    assert(cells_init(&cells, 1) == CELLS_OK && cells_resize(&cells, 10, 2) == CELLS_OK);
    static poses_t poses;
    rest_at_home(&letters, &poses);
    letters.letter[0].state = LETTER_FLYING;
    letters.perched--;
    poses.x[0] = 6.5 * CW;
    poses.y[0] = 1.5 * CH;
    snapshot_t s;
    paint_now(&letters, &cells, &poses, NULL, &s);
    assert(snap(&s, 6, 1)->glyph == 'R');
    assert(snap(&s, 6, 1)->attributes == (CELLS_REVERSE | CELLS_UNDERLINE | CELLS_BOLD));
    /* Its home has none of them: a reversed cell's colour was the letter's. */
    assert(snap(&s, 0, 0)->glyph == 0 && snap(&s, 0, 0)->attributes == 0 &&
           !snap(&s, 0, 0)->has_bg);
    cells_destroy(&cells);
    letters_destroy(&letters);
    vt_destroy(&vt);
}

static void test_a_letter_with_no_colour_of_its_own_takes_the_flocks_in_the_air(void) {
    letters_t letters;
    vt_t vt;
    build(&letters, &vt, "a\033[32mb\033[0m", 10, 2);
    cells_t cells;
    assert(cells_init(&cells, 1) == CELLS_OK && cells_resize(&cells, 10, 2) == CELLS_OK);
    static poses_t poses;
    rest_at_home(&letters, &poses);
    for (int i = 0; i < 2; i++) {
        letters.letter[i].state = LETTER_FLYING;
        poses.x[i] = (4.5 + i) * CW;
        poses.y[i] = 0.5 * CH;
        poses.shade[i] = 2;
    }
    letters.perched = 0;
    static const uint8_t ramp[3][3] = {{255, 0, 0}, {0, 255, 0}, {0, 0, 255}};
    letters_look_t look = {ramp, 3};
    snapshot_t s;
    paint_now(&letters, &cells, &poses, &look, &s);
    assert(snap(&s, 4, 0)->has_fg && snap(&s, 4, 0)->fg_kind == CELLS_COLOUR_RGB);
    assert(snap(&s, 4, 0)->fg[2] == 255 && snap(&s, 4, 0)->fg[0] == 0);
    /* The green one keeps the green the command gave it. */
    assert(snap(&s, 5, 0)->fg_kind == CELLS_COLOUR_ANSI && snap(&s, 5, 0)->fg[0] == 2);
    /* Without a ramp it has none, and is plain. */
    paint_now(&letters, &cells, &poses, NULL, &s);
    assert(!snap(&s, 4, 0)->has_fg);
    /* A shade past the end wraps rather than reading beyond the ramp. */
    poses.shade[0] = 7;
    paint_now(&letters, &cells, &poses, &look, &s);
    assert(snap(&s, 4, 0)->fg[1] == 255);
    cells_destroy(&cells);
    letters_destroy(&letters);
    vt_destroy(&vt);
}

/* A colour of the 256 is the command's own, kept as its index at rest and in the
 * air, before and behind: a ramp is for letters that have none. */
static void test_a_256_colour_is_kept_as_its_index_at_rest_and_in_the_air(void) {
    letters_t letters;
    vt_t vt;
    build(&letters, &vt, "\033[38;5;196mR\033[48;5;21mB\033[0m", 10, 2);
    cells_t cells;
    assert(cells_init(&cells, 1) == CELLS_OK && cells_resize(&cells, 10, 2) == CELLS_OK);
    static poses_t poses;
    rest_at_home(&letters, &poses);
    static const uint8_t ramp[1][3] = {{0, 255, 0}};
    letters_look_t look = {ramp, 1};
    snapshot_t s;
    paint_now(&letters, &cells, &poses, &look, &s);
    assert(snap(&s, 0, 0)->has_fg && snap(&s, 0, 0)->fg_kind == CELLS_COLOUR_INDEXED &&
           snap(&s, 0, 0)->fg[0] == 196);
    assert(snap(&s, 1, 0)->has_bg && snap(&s, 1, 0)->bg_kind == CELLS_COLOUR_INDEXED &&
           snap(&s, 1, 0)->bg[0] == 21);
    letters.letter[0].state = LETTER_FLYING;
    letters.perched--;
    poses.x[0] = 5.5 * CW;
    poses.y[0] = 1.5 * CH;
    paint_now(&letters, &cells, &poses, &look, &s);
    assert(snap(&s, 5, 1)->glyph == 'R');
    assert(snap(&s, 5, 1)->fg_kind == CELLS_COLOUR_INDEXED && snap(&s, 5, 1)->fg[0] == 196);
    cells_destroy(&cells);
    letters_destroy(&letters);
    vt_destroy(&vt);
}

static void test_two_in_one_cell_show_the_one_nearer_home_and_the_lower_on_a_tie(void) {
    letters_t letters;
    vt_t vt;
    build(&letters, &vt, "a                  b", 30, 2);
    cells_t cells;
    assert(cells_init(&cells, 1) == CELLS_OK && cells_resize(&cells, 30, 2) == CELLS_OK);
    static poses_t poses;
    rest_at_home(&letters, &poses);
    for (int i = 0; i < 2; i++) letters.letter[i].state = LETTER_FLYING;
    letters.perched = 0;
    /* a is 8 pixels from home, b is 40 away. */
    poses.x[0] = 9.5 * CW;
    poses.y[0] = 0.5 * CH;
    poses.x[1] = 9.5 * CW + 3;
    poses.y[1] = 0.5 * CH;
    snapshot_t s;
    paint_now(&letters, &cells, &poses, NULL, &s);
    assert(snap(&s, 9, 0)->glyph == 'a');
    /* Nearer to b's home: b is drawn. */
    poses.x[0] = 12.5 * CW;
    poses.x[1] = 12.5 * CW + 3;
    paint_now(&letters, &cells, &poses, NULL, &s);
    assert(snap(&s, 12, 0)->glyph == 'b');
    /* The same distance from their own homes: the first. */
    poses.x[0] = letters.letter[0].home_x + 20 * CW;
    poses.x[1] =
        letters.letter[1].home_x - (letters.letter[1].home_x - letters.letter[0].home_x) + 20 * CW;
    poses.x[1] = poses.x[0];
    double d0 = fabs(poses.x[0] - letters.letter[0].home_x);
    double d1 = fabs(poses.x[1] - letters.letter[1].home_x);
    assert(d0 != d1); /* Not a tie after all: b is nearer. */
    paint_now(&letters, &cells, &poses, NULL, &s);
    assert(snap(&s, 20, 0)->glyph == 'b');
    cells_destroy(&cells);
    letters_destroy(&letters);
    vt_destroy(&vt);
}

static void test_a_wide_letter_in_the_air_covers_two_cells_and_never_half_of_another(void) {
    letters_t letters;
    vt_t vt;
    build(&letters, &vt, "\xE4\xB8\xAD\xE6\x96\x87", 10, 2);
    cells_t cells;
    assert(cells_init(&cells, 1) == CELLS_OK && cells_resize(&cells, 10, 2) == CELLS_OK);
    static poses_t poses;
    rest_at_home(&letters, &poses);
    letters.letter[0].state = LETTER_FLYING;
    letters.perched--;
    /* Over the second letter's first cell: both of its cells are the flier's, and
     * the letter it covers loses its glyph and its tail together. */
    poses.x[0] = 3.0 * CW;
    poses.y[0] = 0.5 * CH;
    snapshot_t s;
    paint_now(&letters, &cells, &poses, NULL, &s);
    assert(snap(&s, 2, 0)->glyph == 0x4E2D && snap(&s, 2, 0)->wide == CELLS_WIDE_HEAD);
    assert(snap(&s, 3, 0)->wide == CELLS_WIDE_TAIL);
    assert(snap(&s, 0, 0)->wide == CELLS_NARROW && snap(&s, 1, 0)->wide == CELLS_NARROW);
    assert(snap(&s, 0, 0)->glyph == 0);
    /* Half over the second letter: it must not be left with a tail and no head. */
    poses.x[0] = 4.0 * CW;
    paint_now(&letters, &cells, &poses, NULL, &s);
    assert(snap(&s, 3, 0)->wide == CELLS_WIDE_HEAD && snap(&s, 4, 0)->wide == CELLS_WIDE_TAIL);
    assert(snap(&s, 2, 0)->wide == CELLS_NARROW && snap(&s, 2, 0)->glyph == 0);
    /* At the right margin there is no room for two cells, and it is not drawn. */
    poses.x[0] = 9.5 * CW;
    paint_now(&letters, &cells, &poses, NULL, &s);
    for (int col = 0; col < 10; col++) assert(snap(&s, col, 0)->glyph != 0x4E2D);
    /* Nor outside the screen at all. */
    poses.x[0] = -30;
    poses.y[0] = 500;
    paint_now(&letters, &cells, &poses, NULL, &s);
    cells_destroy(&cells);
    letters_destroy(&letters);
    vt_destroy(&vt);
}

static void test_a_paint_into_cells_of_another_size_is_clipped(void) {
    letters_t letters;
    vt_t vt;
    build(&letters, &vt, "abcdefghij\nklmnopqrst", 10, 2);
    cells_t smaller;
    assert(cells_init(&smaller, 1) == CELLS_OK && cells_resize(&smaller, 6, 1) == CELLS_OK);
    static poses_t poses;
    rest_at_home(&letters, &poses);
    letters_paint(&letters, &smaller, read_pose, &poses, NULL);
    assert(cells_at(&smaller, 5, 0)->glyph == 'f');
    cells_t bigger;
    assert(cells_init(&bigger, 1) == CELLS_OK && cells_resize(&bigger, 14, 4) == CELLS_OK);
    letters_paint(&letters, &bigger, read_pose, &poses, NULL);
    assert(cells_at(&bigger, 9, 0)->glyph == 'j' && cells_at(&bigger, 10, 0)->glyph == 0);
    assert(cells_at(&bigger, 0, 3)->glyph == 0);
    cells_destroy(&bigger);
    cells_destroy(&smaller);
    letters_destroy(&letters);
    vt_destroy(&vt);
}

/* --- Reading ---------------------------------------------------------------------- */

static double clock_seconds(void) {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (double)now.tv_sec + (double)now.tv_nsec / 1e9;
}

static void test_input_that_ends_is_read_to_its_end(void) {
    int fds[2];
    assert(pipe(fds) == 0);
    assert(write(fds[1], "hello\nworld\n", 12) == 12);
    close(fds[1]);
    vt_t vt;
    assert(vt_init(&vt, 20, 4) == 0);
    letters_reading_t reading = {1.0, 1.0, 5.0, 1 << 20};
    size_t bytes = 0;
    double started = clock_seconds();
    assert(letters_read(&vt, fds[0], &reading, &bytes, NULL) == LETTERS_READ_ENDED);
    assert(clock_seconds() - started < 0.5); /* Not by waiting for a timeout. */
    assert(bytes == 12);
    assert(vt_cell(&vt, 0, 0)->glyph == 'h' && vt_cell(&vt, 0, 1)->glyph == 'w');
    close(fds[0]);
    vt_destroy(&vt);
}

static void test_input_that_goes_quiet_is_not_waited_for_for_ever(void) {
    int fds[2];
    assert(pipe(fds) == 0);
    assert(write(fds[1], "tail -f\n", 8) == 8);
    vt_t vt;
    assert(vt_init(&vt, 20, 4) == 0);
    letters_reading_t reading = {5.0, 0.3, 5.0, 1 << 20};
    size_t bytes = 0;
    double started = clock_seconds();
    assert(letters_read(&vt, fds[0], &reading, &bytes, NULL) == LETTERS_READ_QUIET);
    double took = clock_seconds() - started;
    assert(took >= 0.25 && took < 1.5);
    assert(bytes == 8 && vt_cell(&vt, 0, 0)->glyph == 't');
    close(fds[0]);
    close(fds[1]);
    vt_destroy(&vt);
}

static void test_input_that_never_comes_ends_the_wait(void) {
    int fds[2];
    assert(pipe(fds) == 0);
    vt_t vt;
    assert(vt_init(&vt, 20, 4) == 0);
    letters_reading_t reading = {0.3, 0.3, 5.0, 1 << 20};
    size_t bytes = 99;
    double started = clock_seconds();
    assert(letters_read(&vt, fds[0], &reading, &bytes, NULL) == LETTERS_READ_NOTHING);
    double took = clock_seconds() - started;
    assert(took >= 0.25 && took < 1.5 && bytes == 0);
    close(fds[0]);
    close(fds[1]);
    vt_destroy(&vt);
}

/* The patience is from the moment the writer is writing, which the test waits for:
 * on a Mac with the address sanitizer a fork took longer than the patience, and
 * fewer lines than five had come by the time it ran out. And a line every fifty
 * milliseconds is a line every hundred on a machine whose sleeps run long, so it is
 * three lines at least that it must have read, not five. */
static void test_input_that_keeps_trickling_is_cut_off_by_patience(void) {
    int fds[2], writing[2];
    assert(pipe(fds) == 0 && pipe(writing) == 0);
    pid_t child = fork();
    assert(child >= 0);
    if (child == 0) {
        close(fds[0]);
        close(writing[0]);
        if (write(writing[1], "", 1) != 1) _exit(1);
        close(writing[1]);
        for (int i = 0; i < 100; i++) {
            if (write(fds[1], "line\n", 5) < 0) _exit(1);
            usleep(50000);
        }
        _exit(0);
    }
    close(fds[1]);
    close(writing[1]);
    char running;
    assert(read(writing[0], &running, 1) == 1);
    close(writing[0]);
    vt_t vt;
    assert(vt_init(&vt, 20, 4) == 0);
    letters_reading_t reading = {5.0, 0.5, 0.6, 1 << 20};
    size_t bytes = 0;
    double started = clock_seconds();
    assert(letters_read(&vt, fds[0], &reading, &bytes, NULL) == LETTERS_READ_IMPATIENT);
    double took = clock_seconds() - started;
    assert(took >= 0.5 && took < 1.5);
    assert(bytes >= 3 * 5 && bytes < 100 * 5);
    close(fds[0]);
    int status = 0;
    waitpid(child, &status, 0);
    vt_destroy(&vt);
}

static void test_a_flood_stops_at_the_byte_limit_and_keeps_the_last_screenful(void) {
    int fds[2];
    assert(pipe(fds) == 0);
    pid_t child = fork();
    assert(child >= 0);
    if (child == 0) {
        close(fds[0]);
        char line[] = "y\n";
        for (;;)
            if (write(fds[1], line, 2) < 0) _exit(0);
    }
    close(fds[1]);
    vt_t vt;
    assert(vt_init(&vt, 20, 4) == 0);
    letters_reading_t reading = {5.0, 5.0, 10.0, 100000};
    size_t bytes = 0;
    double started = clock_seconds();
    assert(letters_read(&vt, fds[0], &reading, &bytes, NULL) == LETTERS_READ_LIMIT);
    assert(bytes == 100000);
    assert(clock_seconds() - started < 3.0);
    assert(vt_cell(&vt, 0, 0)->glyph == 'y' && vt.scrolled > 40000);
    close(fds[0]);
    int status = 0;
    waitpid(child, &status, 0);
    vt_destroy(&vt);
}

static void test_input_cut_in_the_middle_of_a_character_is_finished(void) {
    int fds[2];
    assert(pipe(fds) == 0);
    assert(write(fds[1], "ab\xE2\x82", 4) == 4);
    close(fds[1]);
    vt_t vt;
    assert(vt_init(&vt, 20, 4) == 0);
    letters_reading_t reading = {1.0, 1.0, 5.0, 1 << 20};
    assert(letters_read(&vt, fds[0], &reading, NULL, NULL) == LETTERS_READ_ENDED);
    assert(vt_cell(&vt, 2, 0)->glyph == 0xFFFD);
    close(fds[0]);
    vt_destroy(&vt);
}

static void test_a_descriptor_that_is_no_use_is_an_error_and_an_empty_one_is_empty(void) {
    vt_t vt;
    assert(vt_init(&vt, 20, 4) == 0);
    letters_reading_t reading = {0.5, 0.5, 1.0, 1 << 20};
    size_t bytes = 5;
    assert(letters_read(&vt, 987, &reading, &bytes, NULL) == LETTERS_READ_ERROR && bytes == 0);
    int fds[2];
    assert(pipe(fds) == 0);
    close(fds[1]);
    assert(letters_read(&vt, fds[0], &reading, &bytes, NULL) == LETTERS_READ_ENDED && bytes == 0);
    close(fds[0]);
    vt_destroy(&vt);
}

static void test_the_bytes_can_be_kept_to_lay_the_text_out_again(void) {
    int fds[2];
    assert(pipe(fds) == 0);
    static char big[200000];
    for (size_t i = 0; i < sizeof(big); i++) big[i] = (char)('a' + i % 26);
    pid_t child = fork();
    assert(child >= 0);
    if (child == 0) {
        close(fds[0]);
        size_t at = 0;
        while (at < sizeof(big)) {
            ssize_t put = write(fds[1], big + at, sizeof(big) - at);
            if (put <= 0) _exit(1);
            at += (size_t)put;
        }
        _exit(0);
    }
    close(fds[1]);
    vt_t vt;
    assert(vt_init(&vt, 20, 4) == 0);
    letters_reading_t reading = {2.0, 1.0, 5.0, 150000};
    size_t bytes = 0;
    uint8_t *raw = NULL;
    assert(letters_read(&vt, fds[0], &reading, &bytes, &raw) == LETTERS_READ_LIMIT);
    /* Exactly what was fed, no more than the limit, and the same screen again from it. */
    assert(raw != NULL && bytes == 150000 && memcmp(raw, big, bytes) == 0);
    vt_t again;
    assert(vt_init(&again, 20, 4) == 0);
    vt_feed(&again, raw, bytes);
    vt_finish(&again);
    assert(memcmp(vt.storage, again.storage, 20 * 4 * sizeof(vt_cell_t)) == 0);
    free(raw);
    close(fds[0]);
    int status = 0;
    waitpid(child, &status, 0);
    vt_destroy(&again);
    vt_destroy(&vt);
}

static void test_the_last_line_feed_does_not_scroll_a_screen_that_the_text_fills(void) {
    /* Three lines on a screen of three, each ended as a command ends it. A terminal
     * would push the first one off to make room for the cursor; here there is no
     * cursor to make room for, and the first line stays. */
    static const char *const endings[] = {"\n", "\r\n"};
    for (int e = 0; e < 2; e++) {
        char text[64];
        snprintf(text, sizeof(text), "a%sb%sc%s", endings[e], endings[e], endings[e]);
        int fds[2];
        assert(pipe(fds) == 0);
        assert(write(fds[1], text, strlen(text)) == (ssize_t)strlen(text));
        close(fds[1]);
        vt_t vt;
        assert(vt_init(&vt, 10, 3) == 0);
        letters_reading_t reading = {1.0, 1.0, 5.0, 1 << 20};
        size_t bytes = 0;
        assert(letters_read(&vt, fds[0], &reading, &bytes, NULL) == LETTERS_READ_ENDED);
        assert(bytes == strlen(text)); /* All of it was read, and not all fed. */
        assert(vt_cell(&vt, 0, 0)->glyph == 'a' && vt_cell(&vt, 0, 2)->glyph == 'c' &&
               vt.scrolled == 0);
        close(fds[0]);
        vt_destroy(&vt);
    }
    /* The line feed is held, not lost: more text after it, and it was a line feed. */
    int fds[2];
    assert(pipe(fds) == 0);
    pid_t child = fork();
    assert(child >= 0);
    if (child == 0) {
        close(fds[0]);
        if (write(fds[1], "a\n", 2) != 2) _exit(1);
        usleep(300000);
        if (write(fds[1], "b\nc\n", 4) != 4) _exit(1);
        _exit(0);
    }
    close(fds[1]);
    vt_t vt;
    assert(vt_init(&vt, 10, 3) == 0);
    letters_reading_t reading = {1.0, 2.0, 5.0, 1 << 20};
    assert(letters_read(&vt, fds[0], &reading, NULL, NULL) == LETTERS_READ_ENDED);
    assert(vt_cell(&vt, 0, 0)->glyph == 'a' && vt_cell(&vt, 0, 1)->glyph == 'b' &&
           vt_cell(&vt, 0, 2)->glyph == 'c');
    close(fds[0]);
    int status = 0;
    waitpid(child, &status, 0);
    vt_destroy(&vt);
    /* The helper, on its own. */
    assert(letters_text_end((const uint8_t *)"ab\n", 3) == 2);
    assert(letters_text_end((const uint8_t *)"ab\r\n", 4) == 2);
    assert(letters_text_end((const uint8_t *)"ab\n\n", 4) == 3);
    assert(letters_text_end((const uint8_t *)"ab", 2) == 2 &&
           letters_text_end((const uint8_t *)"", 0) == 0);
    assert(letters_text_end((const uint8_t *)"\n", 1) == 0);
}

int main(void) {
    test_every_glyph_cell_becomes_a_letter_with_its_home_in_the_middle_of_it();
    test_a_letter_carries_the_style_the_command_gave_it();
    test_blanks_are_not_letters_but_a_background_is_part_of_the_picture();
    test_a_wide_letter_is_one_letter_over_two_cells();
    test_a_screen_with_nothing_on_it_has_nothing_to_fly();
    test_letters_beyond_the_cap_stay_where_they_are();
    test_at_rest_the_screen_is_exactly_the_text();
    test_the_text_waits_a_few_seconds_and_then_a_wave_begins_with_one_letter();
    test_each_letter_leaves_after_one_beside_it_has();
    test_the_wave_takes_about_a_second_to_cross_a_screenful();
    test_a_word_beyond_every_reach_is_startled_across_the_gap();
    test_a_wave_that_is_taking_too_long_startles_the_rest_at_once();
    test_flight_lasts_fifteen_to_twenty_five_seconds_and_then_they_are_called_home();
    test_the_text_rests_again_once_the_last_letter_is_home();
    test_enter_sends_the_letters_off_and_then_calls_them_home();
    test_the_pointer_scatters_the_letters_it_touches_and_they_come_home_after_it();
    test_a_letter_scattered_a_moment_ago_stays_up_a_moment_even_if_the_pointer_has_gone();
    test_the_wave_starts_where_the_pointer_last_touched();
    test_the_letters_in_the_air_join_the_cycle_when_the_wave_comes();
    test_a_hawk_over_the_text_scatters_it_too();
    test_a_pointer_far_beyond_the_text_or_not_a_number_touches_nothing();
    test_letters_launched_in_the_step_the_rest_ends_are_told_to_the_flock();
    test_a_long_pause_in_the_clock_does_not_skip_the_wave();
    test_a_clock_that_is_not_a_time_does_not_stop_the_cycle();
    test_a_letter_in_the_air_is_drawn_where_it_is_and_its_home_is_left_empty();
    test_a_background_stays_when_the_letter_on_it_leaves();
    test_attributes_belong_to_the_letter_wherever_it_is();
    test_a_letter_with_no_colour_of_its_own_takes_the_flocks_in_the_air();
    test_a_256_colour_is_kept_as_its_index_at_rest_and_in_the_air();
    test_two_in_one_cell_show_the_one_nearer_home_and_the_lower_on_a_tie();
    test_a_wide_letter_in_the_air_covers_two_cells_and_never_half_of_another();
    test_a_paint_into_cells_of_another_size_is_clipped();
    test_input_that_ends_is_read_to_its_end();
    test_input_that_goes_quiet_is_not_waited_for_for_ever();
    test_input_that_never_comes_ends_the_wait();
    test_input_that_keeps_trickling_is_cut_off_by_patience();
    test_a_flood_stops_at_the_byte_limit_and_keeps_the_last_screenful();
    test_input_cut_in_the_middle_of_a_character_is_finished();
    test_the_bytes_can_be_kept_to_lay_the_text_out_again();
    test_the_last_line_feed_does_not_scroll_a_screen_that_the_text_fills();
    test_a_descriptor_that_is_no_use_is_an_error_and_an_empty_one_is_empty();
    return 0;
}
