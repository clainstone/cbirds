/* Feature test macros must precede every include. */
#define _XOPEN_SOURCE 700
#define _DEFAULT_SOURCE
#define _DARWIN_C_SOURCE

#include "letters.h"

#include <errno.h>
#include <math.h>
#include <poll.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

/*
 * The cycle, in seconds.
 *
 * The first rest is long enough to see that this is only the command's output and
 * nothing has happened to it; the later ones are longer, because the second time
 * round nobody needs convincing and the text wants to be read. Flight is the
 * murmuration: under about fifteen seconds it is over before the eye has found
 * the shape, and over twenty-five somebody starts wondering where the text went.
 */
const double LETTERS_FIRST_REST = 4.0;
const double LETTERS_REST = 8.0;
const double LETTERS_FLIGHT_LEAST = 16.0;
const double LETTERS_FLIGHT_MOST = 22.0;
/* Homing is a promise: whatever the flock is doing, every letter is back inside
 * this long after the call. Up to the relax time they bank like birds; after it
 * the limits come off, and by the deadline the last step is the distance left. */
const double LETTERS_HOMING_DEADLINE = 4.0;
const double LETTERS_HOMING_RELAX = 2.0;
/* A letter nobody's near enough to startle (a word alone on a far line) is startled
 * by the noise instead, at the latest this long after the wave began. */
const double LETTERS_WAVE_DEADLINE = 3.0;

enum {
    /* Birds on a wire leave when the bird beside them does. Beside is a good many
     * cells along the line and a few lines either way, because a letter is not
     * startled by the one touching it alone: it is startled by whatever near it has
     * moved, and the nearer the sooner. */
    WAVE_REACH_COLS = 12,
    WAVE_REACH_ROWS = 4,
    READ_CHUNK = 65536
};
/* How long a letter takes to react to one that has left: a moment of its own, a
 * little more for each cell between them, and a random part, so that the front is a
 * ripple and not a ruler. At these numbers the front moves at about eighty cells a
 * second through dense text, which is a screenful in the second the brief asks for;
 * measured on full 80 by 24 screens and on the samples, in the tests. */
static const double WAVE_REACTION = 0.02, WAVE_PER_CELL = 0.012, WAVE_JITTER = 0.03;
/* A letter that was scattered stays up at least this long, so that being touched
 * is seen to do something even when the pointer has already gone. */
static const double SOLO_STAY = 0.9;
/* The longest step of the clock taken at once: a terminal that was suspended for a
 * minute does not skip the wave. */
static const double LONGEST_STEP = 0.25;

static double unit(const letters_t *letters) {
    double value = letters->random ? letters->random() : 0.5;
    return value < 0 ? 0 : (value > 1 ? 1 : value);
}

/* --- Cells from the screen ----------------------------------------------------- */

static int is_blank_glyph(uint32_t glyph) {
    return glyph == 0 || glyph == ' ' || glyph == 0xA0 || glyph == 0x3000;
}

static void set_colour(const vt_colour_t *colour, uint8_t *kind, uint8_t rgb[3], uint8_t *has) {
    rgb[0] = rgb[1] = rgb[2] = 0;
    *kind = CELLS_COLOUR_RGB;
    *has = colour->kind != VT_COLOUR_DEFAULT;
    if (colour->kind == VT_COLOUR_ANSI) {
        *kind = CELLS_COLOUR_ANSI;
        rgb[0] = colour->value[0];
    } else if (colour->kind == VT_COLOUR_INDEXED) {
        *kind = CELLS_COLOUR_INDEXED;
        rgb[0] = colour->value[0];
    } else if (colour->kind == VT_COLOUR_RGB) {
        /* The command asked for this exact colour, so the terminal is asked for it
         * again whatever COLORTERM says: it was shown 24 bit once already. */
        *kind = CELLS_COLOUR_EXACT;
        memcpy(rgb, colour->value, 3);
    }
}

static uint8_t attributes_of(const vt_style_t *style) {
    uint8_t out = 0;
    if (style->attributes & VT_BOLD) out |= CELLS_BOLD;
    if (style->attributes & VT_DIM) out |= CELLS_DIM;
    if (style->attributes & VT_ITALIC) out |= CELLS_ITALIC;
    if (style->attributes & VT_UNDERLINE) out |= CELLS_UNDERLINE;
    if (style->attributes & VT_REVERSE) out |= CELLS_REVERSE;
    return out;
}

void letters_cell_from_vt(const vt_cell_t *source, cell_t *out) {
    memset(out, 0, sizeof(*out));
    out->glyph = is_blank_glyph(source->glyph) ? 0 : source->glyph;
    set_colour(&source->style.fg, &out->fg_kind, out->fg, &out->has_fg);
    set_colour(&source->style.bg, &out->bg_kind, out->bg, &out->has_bg);
    out->attributes = attributes_of(&source->style);
    out->wide = source->width == 2 ? CELLS_WIDE_HEAD : (source->width == 0 ? CELLS_WIDE_TAIL : 0);
    /* A blank shows its background, an underline in the colour of its text, and a
     * reversed one as a block of that colour; its text colour and its boldness show
     * nothing. They are left off, so that a line of spaces that a command wrote in
     * bold red is as quiet to send as a line of spaces. */
    if (out->glyph == 0 && !(out->attributes & (CELLS_UNDERLINE | CELLS_REVERSE))) {
        out->has_fg = 0;
        out->fg_kind = 0;
        memset(out->fg, 0, 3);
        out->attributes = 0;
    }
}

/* What stays behind when a letter leaves: the background it sat on and nothing
 * else. Its colour, its bold, its underline and its reverse go with it, because
 * they were the letter's; a reversed cell's background is the letter's own colour,
 * so there is none left to keep. */
static void scenery_of(const vt_cell_t *source, cell_t *out) {
    memset(out, 0, sizeof(*out));
    if (source->style.attributes & VT_REVERSE) return;
    set_colour(&source->style.bg, &out->bg_kind, out->bg, &out->has_bg);
}

/* --- Building ---------------------------------------------------------------- */

void letters_destroy(letters_t *letters) {
    if (letters == NULL) return;
    free(letters->grid);
    free(letters->at);
    free(letters->letter);
    free(letters->pending);
    free(letters->launched);
    free(letters->claim);
    free(letters->claim_distance);
    memset(letters, 0, sizeof(*letters));
}

int letters_build(letters_t *letters, const vt_t *vt, int cell_width, int cell_height,
                  int max_letters, double (*random)(void)) {
    if (letters == NULL) return -1;
    memset(letters, 0, sizeof(*letters));
    if (vt == NULL || vt->line == NULL || cell_width < 1 || cell_height < 1) return -1;
    size_t cells = (size_t)vt->cols * (size_t)vt->rows;
    letters->cols = vt->cols;
    letters->rows = vt->rows;
    letters->cell_width = cell_width;
    letters->cell_height = cell_height;
    letters->random = random;
    letters->last_touched = letters->origin = letters->last_left = -1;
    letters->grid = malloc(cells * sizeof(*letters->grid));
    letters->at = malloc(cells * sizeof(*letters->at));
    letters->claim = malloc(cells * sizeof(*letters->claim));
    letters->claim_distance = malloc(cells * sizeof(*letters->claim_distance));
    if (!letters->grid || !letters->at || !letters->claim || !letters->claim_distance) goto failed;

    for (int row = 0; row < vt->rows; row++)
        memcpy(letters->grid + (size_t)row * (size_t)vt->cols, vt->line[row],
               (size_t)vt->cols * sizeof(*letters->grid));
    for (size_t i = 0; i < cells; i++) letters->at[i] = -1;

    /* A blank is not a letter, however wide, and a wide blank (the ideographic
     * space) is two narrow ones so that nothing downstream has to ask. */
    int glyph_cells = 0;
    for (size_t i = 0; i < cells; i++) {
        vt_cell_t *cell = &letters->grid[i];
        if (cell->width == 0) continue;
        if (is_blank_glyph(cell->glyph)) {
            if (cell->width == 2 && i + 1 < cells) letters->grid[i + 1].width = 1;
            cell->width = 1;
            cell->glyph = 0;
        } else {
            glyph_cells++;
        }
    }
    letters->cells_with_glyphs = glyph_cells;

    int wanted = glyph_cells < max_letters ? glyph_cells : max_letters;
    if (wanted <= 0) {
        letters_destroy(letters);
        return 0;
    }
    letters->letter = calloc((size_t)wanted, sizeof(*letters->letter));
    letters->pending = malloc((size_t)wanted * sizeof(*letters->pending));
    letters->launched = malloc((size_t)wanted * sizeof(*letters->launched));
    if (!letters->letter || !letters->pending || !letters->launched) goto failed;

    for (int row = 0; row < vt->rows && letters->count < wanted; row++)
        for (int col = 0; col < vt->cols && letters->count < wanted; col++) {
            const vt_cell_t *cell = &letters->grid[(size_t)row * (size_t)vt->cols + (size_t)col];
            if (cell->width == 0 || is_blank_glyph(cell->glyph)) continue;
            int index = letters->count++;
            letter_t *letter = &letters->letter[index];
            letter->glyph = cell->glyph;
            letter->style = cell->style;
            letter->col = col;
            letter->row = row;
            letter->width = cell->width;
            letter->home_x = (col + cell->width / 2.0) * cell_width;
            letter->home_y = (row + 0.5) * cell_height;
            letter->state = LETTER_PERCHED;
            letters->at[(size_t)row * (size_t)vt->cols + (size_t)col] = index;
            if (cell->width == 2)
                letters->at[(size_t)row * (size_t)vt->cols + (size_t)col + 1] = index;
        }
    letters->perched = letters->count;
    letters->phase = LETTERS_AT_REST;
    letters->rest_left = LETTERS_FIRST_REST;
    return letters->count;

failed:
    letters_destroy(letters);
    return -1;
}

/* --- The cycle ----------------------------------------------------------------- */

static double heading_away(const letters_t *letters, double from_x, double from_y, double home_x,
                           double home_y) {
    double ax = home_x - from_x, ay = home_y - from_y;
    double length = sqrt(ax * ax + ay * ay);
    if (length < 1e-9) {
        ax = 0;
        ay = 0;
    } else {
        ax /= length;
        ay /= length;
    }
    /* Mostly up, as a bird off a wire goes, and a little away from what frightened
     * it, with a spread so that a row of them does not go off in parallel. */
    double angle = atan2(-1.0 + 0.5 * ay, 0.5 * ax) + (unit(letters) - 0.5) * 0.7;
    while (angle < 0) angle += 2 * M_PI;
    while (angle >= 2 * M_PI) angle -= 2 * M_PI;
    return angle;
}

static void startle(letters_t *letters, int index, double wake_at) {
    letter_t *letter = &letters->letter[index];
    letter->state = LETTER_STARTLED;
    letter->wake_at = wake_at;
    letters->pending[letters->pending_count++] = index;
}

static void launch(letters_t *letters, int index, double fear_x, double fear_y, int solo) {
    letter_t *letter = &letters->letter[index];
    letter->state = LETTER_FLYING;
    letter->airborne = 0;
    letter->homing_for = 0;
    letter->solo = solo;
    letter->launch_direction =
        heading_away(letters, fear_x, fear_y, letter->home_x, letter->home_y);
    letters->perched--;
    letters->launched[letters->launched_count++] = index;
    if (!solo) letters->last_left = index;
}

/* The time a letter takes to react to one `cells` away (in cell widths) that left at
 * `left_at`. */
static double reaction_time(const letters_t *letters, double left_at, double cells) {
    return left_at + WAVE_REACTION + WAVE_PER_CELL * cells + WAVE_JITTER * unit(letters);
}

static void startle_neighbours(letters_t *letters, int index) {
    const letter_t *from = &letters->letter[index];
    for (int dy = -WAVE_REACH_ROWS; dy <= WAVE_REACH_ROWS; dy++) {
        int row = from->row + dy;
        if (row < 0 || row >= letters->rows) continue;
        for (int dx = -WAVE_REACH_COLS; dx <= WAVE_REACH_COLS + from->width - 1; dx++) {
            int col = from->col + dx;
            if (col < 0 || col >= letters->cols) continue;
            int other = letters->at[(size_t)row * (size_t)letters->cols + (size_t)col];
            if (other < 0 || other == index || letters->letter[other].state != LETTER_PERCHED)
                continue;
            double across = (letters->letter[other].home_x - from->home_x) / letters->cell_width;
            double down = (letters->letter[other].home_y - from->home_y) / letters->cell_width;
            startle(letters, other,
                    reaction_time(letters, from->wake_at, sqrt(across * across + down * down)));
        }
    }
}

/* A wave that has run out of letters near enough to startle, with some still at
 * home: a gap wider than any reach, between the logo and the text beside it or
 * between two columns of a table. The nearest to the last that left is startled,
 * and it takes as long as it takes sound to cross the gap. */
static int startle_across_a_gap(letters_t *letters) {
    if (letters->last_left < 0) return 0;
    const letter_t *from = &letters->letter[letters->last_left];
    int best = -1;
    double best_squared = 0;
    for (int i = 0; i < letters->count; i++) {
        const letter_t *letter = &letters->letter[i];
        if (letter->state != LETTER_PERCHED) continue;
        double dx = letter->home_x - from->home_x, dy = letter->home_y - from->home_y;
        double squared = dx * dx + dy * dy;
        if (best < 0 || squared < best_squared) {
            best = i;
            best_squared = squared;
        }
    }
    if (best < 0) return 0;
    startle(letters, best,
            reaction_time(letters, from->wake_at, sqrt(best_squared) / letters->cell_width));
    return 1;
}

static void begin_wave(letters_t *letters) {
    letters->phase = LETTERS_TAKING_OFF;
    letters->wave_began = letters->clock;
    letters->cycles++;
    letters->launched_count = 0;
    letters->origin = letters->last_left = -1;
    /* Whatever was scattered joins the cycle: a letter on its way back from the
     * pointer turns round, and is part of the flock for the flight. */
    for (int i = 0; i < letters->count; i++) {
        letter_t *letter = &letters->letter[i];
        if (letter->state == LETTER_HOMING) letter->state = LETTER_FLYING;
        letter->solo = 0;
    }
    if (letters->perched == 0) return;

    int origin = -1;
    if (letters->last_touched >= 0 &&
        letters->letter[letters->last_touched].state == LETTER_PERCHED)
        origin = letters->last_touched;
    for (int tries = 0; origin < 0 && tries < 64; tries++) {
        int candidate = (int)(unit(letters) * letters->count) % letters->count;
        if (letters->letter[candidate].state == LETTER_PERCHED) origin = candidate;
    }
    for (int i = 0; origin < 0 && i < letters->count; i++)
        if (letters->letter[i].state == LETTER_PERCHED) origin = i;
    letters->origin = origin;
    startle(letters, origin, letters->clock);
}

static void run_wave(letters_t *letters) {
    int origin = letters->origin;
    double fear_x = origin >= 0 ? letters->letter[origin].home_x : 0;
    double fear_y = origin >= 0 ? letters->letter[origin].home_y : 0;
    if (origin < 0) {
        fear_x = letters->cols * letters->cell_width / 2.0;
        fear_y = letters->rows * letters->cell_height / 2.0;
    }
    /* The noise reaches the ones nobody was near enough to startle. */
    if (letters->clock - letters->wave_began >= LETTERS_WAVE_DEADLINE)
        for (int i = 0; i < letters->count; i++)
            if (letters->letter[i].state == LETTER_PERCHED)
                startle(letters, i, letters->clock + 0.3 * unit(letters));

    int progressed;
    do {
        progressed = 0;
        for (int p = 0; p < letters->pending_count;) {
            int index = letters->pending[p];
            if (letters->letter[index].wake_at > letters->clock) {
                p++;
                continue;
            }
            letters->pending[p] = letters->pending[--letters->pending_count];
            launch(letters, index, fear_x, fear_y, 0);
            startle_neighbours(letters, index);
            progressed = 1;
        }
        /* Nobody left to startle and the wave not over: it crosses the gap. */
        if (letters->pending_count == 0 && letters->perched > 0 && startle_across_a_gap(letters))
            progressed = 1;
    } while (progressed);
}

/* One step of the wave, and the end of it: when no letter is waiting to be startled
 * the flight proper begins. */
static void step_taking_off(letters_t *letters) {
    run_wave(letters);
    if (letters->perched == 0) {
        letters->phase = LETTERS_IN_FLIGHT;
        letters->flight_left =
            LETTERS_FLIGHT_LEAST + (LETTERS_FLIGHT_MOST - LETTERS_FLIGHT_LEAST) * unit(letters);
    }
}

static void begin_home(letters_t *letters) {
    letters->phase = LETTERS_COMING_HOME;
    letters->pending_count = 0;
    for (int i = 0; i < letters->count; i++) {
        letter_t *letter = &letters->letter[i];
        if (letter->state == LETTER_STARTLED) {
            letter->state = LETTER_PERCHED; /* It had not left, and now it need not. */
        } else if (letter->state == LETTER_FLYING) {
            letter->state = LETTER_HOMING;
            letter->homing_for = 0;
        }
        letter->solo = 0;
    }
}

void letters_poke(letters_t *letters) {
    if (letters == NULL || letters->count == 0) return;
    if (letters->phase == LETTERS_AT_REST)
        begin_wave(letters);
    else if (letters->phase == LETTERS_TAKING_OFF || letters->phase == LETTERS_IN_FLIGHT)
        begin_home(letters);
}

static int inside_ellipse(double dx, double dy, double radius_x, double radius_y) {
    if (radius_x <= 0 || radius_y <= 0) return 0;
    return (dx * dx) / (radius_x * radius_x) + (dy * dy) / (radius_y * radius_y) <= 1.0;
}

/* The pointer or a hawk, at rest: whatever it touches goes up on its own, and
 * whatever it has left alone for long enough comes down. */
static void disturb(letters_t *letters, const letters_disturbance_t *disturbances, int count) {
    for (int d = 0; d < count; d++) {
        const letters_disturbance_t *at = &disturbances[d];
        int first_col = (int)floor((at->x - at->touch_x) / letters->cell_width);
        int last_col = (int)floor((at->x + at->touch_x) / letters->cell_width);
        int first_row = (int)floor((at->y - at->touch_y) / letters->cell_height);
        int last_row = (int)floor((at->y + at->touch_y) / letters->cell_height);
        if (first_col < 0) first_col = 0;
        if (first_row < 0) first_row = 0;
        if (last_col >= letters->cols) last_col = letters->cols - 1;
        if (last_row >= letters->rows) last_row = letters->rows - 1;
        for (int row = first_row; row <= last_row; row++)
            for (int col = first_col; col <= last_col; col++) {
                int index = letters->at[(size_t)row * (size_t)letters->cols + (size_t)col];
                if (index < 0) continue;
                letter_t *letter = &letters->letter[index];
                if (letter->state != LETTER_PERCHED) continue;
                if (!inside_ellipse(letter->home_x - at->x, letter->home_y - at->y, at->touch_x,
                                    at->touch_y))
                    continue;
                launch(letters, index, at->x, at->y, 1);
                letters->last_touched = index;
            }
    }
    for (int i = 0; i < letters->count; i++) {
        letter_t *letter = &letters->letter[i];
        if (letter->state != LETTER_FLYING || !letter->solo || letter->airborne < SOLO_STAY)
            continue;
        int clear = 1;
        for (int d = 0; d < count && clear; d++)
            if (inside_ellipse(letter->home_x - disturbances[d].x,
                               letter->home_y - disturbances[d].y, disturbances[d].clear_x,
                               disturbances[d].clear_y))
                clear = 0;
        if (clear) {
            letter->state = LETTER_HOMING;
            letter->homing_for = 0;
        }
    }
}

void letters_advance(letters_t *letters, double seconds, const letters_disturbance_t *disturbances,
                     int disturbance_count) {
    if (letters == NULL || letters->count == 0) return;
    if (seconds < 0) seconds = 0;
    if (seconds > LONGEST_STEP) seconds = LONGEST_STEP;
    letters->clock += seconds;
    letters->launched_count = 0;
    for (int i = 0; i < letters->count; i++) {
        letter_t *letter = &letters->letter[i];
        if (!letter_is_airborne(letter)) continue;
        letter->airborne += seconds;
        if (letter->state == LETTER_HOMING) letter->homing_for += seconds;
    }

    switch (letters->phase) {
        case LETTERS_AT_REST:
            disturb(letters, disturbances, disturbance_count);
            letters->rest_left -= seconds;
            if (letters->rest_left <= 0) begin_wave(letters);
            /* The wave starts this very step, with its first letter. */
            if (letters->phase == LETTERS_TAKING_OFF) step_taking_off(letters);
            break;
        case LETTERS_TAKING_OFF:
            step_taking_off(letters);
            break;
        case LETTERS_IN_FLIGHT:
            letters->flight_left -= seconds;
            if (letters->flight_left <= 0) begin_home(letters);
            break;
        case LETTERS_COMING_HOME:
            if (letters->perched == letters->count) {
                letters->phase = LETTERS_AT_REST;
                letters->rest_left = LETTERS_REST;
                letters->last_touched = -1;
            }
            break;
    }
}

void letters_release(letters_t *letters, int index) {
    if (letters == NULL || index < 0 || index >= letters->count) return;
    letter_t *letter = &letters->letter[index];
    if (letter_is_airborne(letter)) return;
    letter->state = LETTER_FLYING;
    letter->airborne = 0;
    letter->solo = 0;
    letters->perched--;
}

void letters_land(letters_t *letters, int index) {
    if (letters == NULL || index < 0 || index >= letters->count) return;
    letter_t *letter = &letters->letter[index];
    if (!letter_is_airborne(letter)) return;
    letter->state = LETTER_PERCHED;
    letter->airborne = letter->homing_for = 0;
    letter->solo = 0;
    letters->perched++;
}

/* --- Painting -------------------------------------------------------------------- */

/* The column a letter in the air is drawn in. Its position is the middle of the
 * glyph, so a wide one starts half its width to the left of it; the nearest whole
 * column to that, which for a narrow letter is the cell it is in. */
static int column_of(const letters_t *letters, const letter_t *letter, double x) {
    double left = x - letter->width * letters->cell_width / 2.0;
    return (int)floor(left / letters->cell_width + 0.5);
}

/* Writes what stays behind at a position, breaking whatever wide glyph it cuts. */
static void put_scenery(const letters_t *letters, cells_t *cells, int col, int row) {
    cell_t *cell = cells_at(cells, col, row);
    if (cell == NULL) return;
    scenery_of(&letters->grid[(size_t)row * (size_t)letters->cols + (size_t)col], cell);
}

/* A cell is about to be written over: if it is half of a wide glyph, the other
 * half goes back to the scenery. */
static void release(const letters_t *letters, cells_t *cells, int col, int row) {
    cell_t *cell = cells_at(cells, col, row);
    if (cell == NULL) return;
    if (cell->wide == CELLS_WIDE_HEAD)
        put_scenery(letters, cells, col + 1, row);
    else if (cell->wide == CELLS_WIDE_TAIL)
        put_scenery(letters, cells, col - 1, row);
}

static void draw_flier(const letters_t *letters, cells_t *cells, const letter_t *letter, int col,
                       int row, int shade, const letters_look_t *look) {
    cell_t drawn;
    memset(&drawn, 0, sizeof(drawn));
    drawn.glyph = letter->glyph;
    drawn.attributes = attributes_of(&letter->style);
    if (letter->style.fg.kind == VT_COLOUR_DEFAULT && look != NULL && look->ramp != NULL &&
        look->ramp_shades > 0) {
        /* A letter with no colour of its own wears the flock's, by heading. */
        int at = shade < 0 ? 0 : shade % look->ramp_shades;
        memcpy(drawn.fg, look->ramp[at], 3);
        drawn.fg_kind = CELLS_COLOUR_RGB;
        drawn.has_fg = 1;
    } else {
        set_colour(&letter->style.fg, &drawn.fg_kind, drawn.fg, &drawn.has_fg);
    }
    for (int part = 0; part < letter->width; part++) {
        release(letters, cells, col + part, row);
        cell_t *cell = cells_at(cells, col + part, row);
        if (cell == NULL) continue;
        cell_t here = drawn;
        if (part > 0) {
            here.glyph = 0;
            here.has_fg = 0;
            here.attributes = drawn.attributes;
        }
        if (letter->style.attributes & VT_REVERSE) {
            /* Reverse is the letter's own: its colours swap wherever it is. */
            set_colour(&letter->style.bg, &here.bg_kind, here.bg, &here.has_bg);
        } else {
            /* Anything else sits on the background of the place it flies over. */
            cell_t scenery;
            scenery_of(&letters->grid[(size_t)row * (size_t)letters->cols + (size_t)(col + part)],
                       &scenery);
            memcpy(here.bg, scenery.bg, 3);
            here.bg_kind = scenery.bg_kind;
            here.has_bg = scenery.has_bg;
        }
        here.wide = letter->width == 2 ? (part == 0 ? CELLS_WIDE_HEAD : CELLS_WIDE_TAIL) : 0;
        *cell = here;
    }
}

void letters_paint(const letters_t *letters, cells_t *cells, letters_pose_reader_t read,
                   const void *context, const letters_look_t *look) {
    if (letters == NULL || cells == NULL || cells->now == NULL) return;
    cells_clear(cells);

    /* The screen as it was printed, with a gap wherever a letter has gone. */
    for (int row = 0; row < letters->rows; row++)
        for (int col = 0; col < letters->cols; col++) {
            cell_t *cell = cells_at(cells, col, row);
            if (cell == NULL) continue;
            size_t at = (size_t)row * (size_t)letters->cols + (size_t)col;
            int index = letters->at[at];
            if (index >= 0 && letter_is_airborne(&letters->letter[index]))
                scenery_of(&letters->grid[at], cell);
            else
                letters_cell_from_vt(&letters->grid[at], cell);
        }

    /* Who is in which cell. The nearer its own home wins, and the lower index when
     * they tie: both are stable from one frame to the next as the letters move, and
     * a letter about to land is not hidden by one passing over it. */
    size_t grid_cells = (size_t)letters->cols * (size_t)letters->rows;
    for (size_t i = 0; i < grid_cells; i++) letters->claim[i] = -1;
    for (int i = 0; i < letters->count; i++) {
        const letter_t *letter = &letters->letter[i];
        if (!letter_is_airborne(letter)) continue;
        letters_pose_t pose;
        read(context, i, &pose);
        int col = column_of(letters, letter, pose.x);
        int row = (int)floor(pose.y / letters->cell_height);
        if (col < 0 || row < 0 || col + letter->width > letters->cols || row >= letters->rows)
            continue;
        double dx = pose.x - letter->home_x, dy = pose.y - letter->home_y;
        double distance = dx * dx + dy * dy;
        for (int part = 0; part < letter->width; part++) {
            size_t at = (size_t)row * (size_t)letters->cols + (size_t)(col + part);
            if (letters->claim[at] < 0 || distance < letters->claim_distance[at]) {
                letters->claim[at] = i;
                letters->claim_distance[at] = distance;
            }
        }
    }
    for (int i = 0; i < letters->count; i++) {
        const letter_t *letter = &letters->letter[i];
        if (!letter_is_airborne(letter)) continue;
        letters_pose_t pose;
        read(context, i, &pose);
        int col = column_of(letters, letter, pose.x);
        int row = (int)floor(pose.y / letters->cell_height);
        if (col < 0 || row < 0 || col + letter->width > letters->cols || row >= letters->rows)
            continue;
        int won = 1;
        for (int part = 0; part < letter->width; part++)
            if (letters->claim[(size_t)row * (size_t)letters->cols + (size_t)(col + part)] != i)
                won = 0;
        if (won) draw_flier(letters, cells, letter, col, row, pose.shade, look);
    }
}

/* --- Reading the text ------------------------------------------------------------ */

static double seconds_now(void) {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (double)now.tv_sec + (double)now.tv_nsec / 1e9;
}

letters_read_end_t letters_read(vt_t *vt, int fd, const letters_reading_t *reading, size_t *bytes,
                                uint8_t **raw) {
    static char chunk[READ_CHUNK];
    size_t total = 0, kept_capacity = 0;
    if (raw != NULL) *raw = NULL;
    double started = seconds_now(), first = 0, last = 0;
    letters_read_end_t end = LETTERS_READ_ENDED;

    for (;;) {
        double now = seconds_now();
        double left;
        if (total == 0) {
            left = reading->first_byte_wait - (now - started);
        } else {
            left = reading->quiet - (now - last);
            double patient = reading->patience - (now - first);
            if (patient < left) left = patient;
        }
        if (left <= 0) {
            end = total == 0 ? LETTERS_READ_NOTHING
                             : (reading->quiet - (now - last) <= 0 ? LETTERS_READ_QUIET
                                                                   : LETTERS_READ_IMPATIENT);
            break;
        }
        struct pollfd wait = {.fd = fd, .events = POLLIN};
        int ready = poll(&wait, 1, (int)ceil(left * 1000.0));
        if (ready < 0) {
            if (errno == EINTR) continue;
            end = LETTERS_READ_ERROR;
            break;
        }
        if (ready == 0) continue; /* The deadline test at the top decides which. */
        if (wait.revents & POLLNVAL) {
            end = LETTERS_READ_ERROR;
            break;
        }
        ssize_t got = read(fd, chunk, sizeof(chunk));
        if (got < 0) {
            if (errno == EINTR || errno == EAGAIN) continue;
            end = LETTERS_READ_ERROR;
            break;
        }
        if (got == 0) {
            end = LETTERS_READ_ENDED;
            break;
        }
        size_t take = (size_t)got;
        if (reading->limit > 0 && total + take > reading->limit) take = reading->limit - total;
        vt_feed(vt, chunk, take);
        if (raw != NULL) {
            /* The bytes as they came, for laying the text out again on another size of
             * screen. Bounded by the limit like everything else. */
            if (total + take > kept_capacity) {
                size_t wanted = kept_capacity ? kept_capacity * 2 : 4 * READ_CHUNK;
                while (wanted < total + take) wanted *= 2;
                uint8_t *grown = realloc(*raw, wanted);
                if (grown == NULL) {
                    free(*raw);
                    *raw = NULL;
                    raw = NULL;
                } else {
                    *raw = grown;
                    kept_capacity = wanted;
                }
            }
            if (raw != NULL) memcpy(*raw + total, chunk, take);
        }
        total += take;
        last = seconds_now();
        if (first == 0) first = last;
        if (reading->limit > 0 && total >= reading->limit) {
            end = LETTERS_READ_LIMIT;
            break;
        }
    }
    vt_finish(vt);
    if (bytes != NULL) *bytes = total;
    return end;
}
