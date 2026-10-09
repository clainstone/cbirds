/*
 * Text as a flock: which letter is where, and when each one leaves.
 *
 * A screenful of text, as vt.c lays it out, becomes a letter for every cell that
 * has a glyph in it. Each has a home, the middle of its cell, and carries the
 * glyph and the style the command gave it. This module keeps no positions: where
 * a letter is while it flies belongs to the simulation, which is the flock's own.
 * What lives here is everything that is about text rather than about flying:
 *
 *   - the cycle: the text at rest, a wave that startles it letter by letter from
 *     one point, a flight, a call home, and rest again;
 *   - who has left and who has landed, and the pointer or a hawk scattering the
 *     letters it touches in between;
 *   - what the screen shows for any arrangement of them, as cells: the text
 *     exactly as it was printed while every letter is at home, and otherwise the
 *     scenery that stays put and the letters that do not.
 *
 * Time is the caller's: letters_advance is given the seconds that passed, so a
 * paused simulation stands still and a recording is the same every time.
 */

#ifndef LETTERS_H
#define LETTERS_H

#include <stddef.h>
#include <stdint.h>

#include "cells.h"
#include "vt.h"

typedef enum {
    LETTER_PERCHED,  /* At home and still, and drawn as the command printed it. */
    LETTER_STARTLED, /* Has been startled and has not left yet: still at home. */
    LETTER_FLYING,   /* Off, with the flock. */
    LETTER_HOMING    /* On its way back, to land exactly. */
} letter_state_t;

typedef enum {
    LETTERS_AT_REST,    /* The text, as it was printed. */
    LETTERS_TAKING_OFF, /* A wave runs through it. */
    LETTERS_IN_FLIGHT,  /* All of them up, flocking. */
    LETTERS_COMING_HOME /* Called home, landing. */
} letters_phase_t;

typedef struct {
    uint32_t glyph;
    vt_style_t style;
    int col, row;
    int width;             /* Cells: 1, or 2 for a wide glyph. */
    double home_x, home_y; /* The middle of its cell, or of its two cells, in pixels. */
    letter_state_t state;
    double wake_at;          /* STARTLED: the time on the letters' clock at which it leaves. */
    double airborne;         /* Seconds since it left. */
    double homing_for;       /* Seconds since it began to come home. */
    double launch_direction; /* Radians, the way it sets off: up and away from the fright. */
    int solo;                /* Left because something touched it, not because the cycle came. */
} letter_t;

typedef struct {
    int cols, rows;
    int cell_width, cell_height; /* Pixels a cell, as the simulation measures them. */
    vt_cell_t *grid;             /* The screen as printed: cols * rows. */
    int *at;                     /* Per cell, the letter whose home it is part of, or -1. */
    letter_t *letter;
    int count;
    int cells_with_glyphs; /* Glyph cells in the text, those past the cap included. */

    letters_phase_t phase;
    double clock;     /* Seconds the cycle has run. */
    double rest_left; /* AT_REST: seconds until the wave. */
    double flight_left;
    double wave_began;
    int cycles;       /* Waves so far. */
    int perched;      /* Letters that are PERCHED or STARTLED. */
    int last_touched; /* The letter a pointer or a hawk last scattered, or -1. */
    int origin;       /* The letter the wave started from, or -1. */
    int last_left;    /* The letter the wave sent off last, or -1: where a gap is crossed from. */

    int *pending; /* STARTLED letters, in no order. */
    int pending_count;
    int *launched; /* Letters that left during the last advance. */
    int launched_count;

    int *claim; /* Scratch for painting: per cell, the letter drawn there. */
    double *claim_distance;

    double (*random)(void); /* Uniform in [0, 1]. The simulation's own, so a seed holds. */
} letters_t;

/* What can disturb a perched letter: the pointer, a hawk. A letter within the touch
 * ellipse leaves on its own; one that has left comes home when its home is outside
 * the clear ellipse of every disturbance, which is wider so that it does not land
 * to be scattered again. Radii in pixels. */
typedef struct {
    double x, y;
    double touch_x, touch_y;
    double clear_x, clear_y;
} letters_disturbance_t;

/* What the painter needs to know about a letter in the air. */
typedef struct {
    double x, y;
    int shade; /* Which colour of the ramp, from its heading. */
} letters_pose_t;

typedef void (*letters_pose_reader_t)(const void *context, int index, letters_pose_t *pose);

typedef struct {
    const uint8_t (*ramp)[3]; /* The colours a letter that has none of its own takes in flight. */
    int ramp_shades;
} letters_look_t;

/* The cycle's timing, in seconds, for the tests and for anyone tuning it. */
extern const double LETTERS_FIRST_REST, LETTERS_REST, LETTERS_FLIGHT_LEAST, LETTERS_FLIGHT_MOST,
    LETTERS_HOMING_DEADLINE, LETTERS_HOMING_RELAX, LETTERS_WAVE_DEADLINE;

/* Builds the letters from a screen. The first max_letters glyph cells, in reading
 * order, become letters; any more are part of the scenery and never move. Returns
 * the number of letters, zero if the screen has no glyph on it (that is an empty
 * input, and there is nothing to fly), or -1 when memory runs out. `random` is
 * kept. The cycle begins at rest. */
int letters_build(letters_t *letters, const vt_t *vt, int cell_width, int cell_height,
                  int max_letters, double (*random)(void));
void letters_destroy(letters_t *letters);

/* The wave, now, from the last letter touched or else from one at random; or, when
 * the letters are up, call them home. This is what Enter does. */
void letters_poke(letters_t *letters);

/* Advances the cycle by `seconds`. The letters that left are in letters->launched
 * until the next call; the simulation puts them in the air. */
void letters_advance(letters_t *letters, double seconds, const letters_disturbance_t *disturbances,
                     int disturbance_count);

/* A letter that has reached home. */
void letters_land(letters_t *letters, int index);

/* A letter sent off at once, whatever it was doing: the way out at the end. */
void letters_release(letters_t *letters, int index);

/* Whether the simulation moves it: it is in the air. */
static inline int letter_is_airborne(const letter_t *letter) {
    return letter->state == LETTER_FLYING || letter->state == LETTER_HOMING;
}

/* Fills the cells, which must be the size of the screen, with what is to be seen:
 * the text as printed for every letter at home, the scenery where one has left,
 * and each letter in the air in the cell that holds it. Two in one cell: the one
 * nearer its own home is drawn, and the lower index when they tie, so that the
 * picture does not flicker between them. */
void letters_paint(const letters_t *letters, cells_t *cells, letters_pose_reader_t read,
                   const void *context, const letters_look_t *look);

/* Converts a cell of the screen as printed to a cell to be drawn. */
void letters_cell_from_vt(const vt_cell_t *source, cell_t *out);

/* --- Reading the text ------------------------------------------------------ */

typedef struct {
    double first_byte_wait; /* Seconds to wait for anything at all. */
    double quiet;           /* Seconds of silence, after something, that end it. */
    double patience;        /* Seconds in all, after the first byte, that end it. */
    size_t limit;           /* Bytes to read at most. */
} letters_reading_t;

/* How much of the text is fed to the emulator: all of it but the one line feed that
 * ends it (and the carriage return before that), so that a command's last newline
 * does not scroll a full screen. Used by the reader and for laying the same bytes
 * out again. */
size_t letters_text_end(const uint8_t *bytes, size_t length);

typedef enum {
    LETTERS_READ_ENDED,     /* End of input. */
    LETTERS_READ_QUIET,     /* Silent for the quiet time. */
    LETTERS_READ_LIMIT,     /* The byte limit. */
    LETTERS_READ_IMPATIENT, /* The time allowed, with the input still coming. */
    LETTERS_READ_NOTHING,   /* Not a byte in the first-byte wait. */
    LETTERS_READ_ERROR      /* The descriptor is no use. */
} letters_read_end_t;

/* Reads from a descriptor into the emulator until one of the conditions in
 * `reading` holds, never blocking past them. The input is fed as it comes, so a
 * flood costs no more memory than a screen, and what is kept is the last screenful
 * of it, as a terminal would keep. Returns why it stopped, and the bytes read in
 * *bytes. The emulator is finished before it returns. If `raw` is not NULL the bytes
 * themselves are kept there, in memory the caller frees, so that the text can be laid
 * out again on a screen of another size; NULL if there was no memory for them. */
letters_read_end_t letters_read(vt_t *vt, int fd, const letters_reading_t *reading, size_t *bytes,
                                uint8_t **raw);

#endif
