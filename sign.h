/*
 * What a flock needs to know to be a sign, with nothing of the flock in it.
 *
 * The words: cleaned down to what the font can draw, and wrapped onto the
 * fewest lines that make the letters largest. The time: HH:MM from a struct tm,
 * in the convention the locale asks for. The hover: the small loop a bird flies
 * around its place in a letter while a sign is held. The rhythm of hold and
 * flight. And the box the rest of the flock keeps out of. All of it is a
 * function of its arguments, so all of it can be tested without a flock.
 */

#ifndef SIGN_H
#define SIGN_H

#include <stddef.h>
#include <time.h>

enum {
    SIGN_MAX_LINES = 3,
    SIGN_TEXT_MAX = 160, /* Characters kept of what was asked for, with the end. */
    /* Rows of blank between two lines of letters: two, so that a line reads as a
     * line and the letters are as large as the room allows. */
    SIGN_LINE_GAP = 2
};

typedef struct {
    int count;
    char line[SIGN_MAX_LINES][SIGN_TEXT_MAX];
} sign_lines_t;

/* The text as the font can draw it: capitals, one space between words and none
 * at the ends, and no character the font lacks. Returns how many letters, digits
 * and marks are left (spaces do not count), zero when there is nothing to draw. */
int sign_clean(const char *text, char *out, size_t size);

/* How many rows of cells the lines take, and how many columns the widest line. */
int sign_rows(int lines);
int sign_columns(const char *line);

/* Wraps cleaned text at its spaces onto one line, two or three, whichever makes
 * the cell largest in a box of width by height pixels, and returns the number of
 * lines (zero if even one pixel a cell does not fit). The cell is the side of
 * one lit square of a letter; it is never more than `largest`. A reference width
 * in columns, when it is wider than the text, sizes the cell instead, so that a
 * clock does not change size between 1:09 and 10:09. Lines are centred on each
 * other by the caller. */
int sign_fit(const char *clean, int reference_columns, double width, double height, double largest,
             sign_lines_t *out, double *cell);

/* 12 hour or 24: whether a strftime format, as nl_langinfo(T_FMT) gives it, shows
 * the hour on a 12 hour clock (%I, %l or %r). */
int sign_wants_twelve_hours(const char *time_format);

/* "13:05", or "1:05" on a 12 hour clock; no AM and no PM, and no zero in front of
 * the hour there, which would make it look like the other convention. */
void sign_clock_text(const struct tm *when, int twelve_hours, char *out, size_t size);

/* Where in its loop a bird is, relative to the middle of it. `id` gives the bird
 * its own phase, pace, direction, tilt and size; the loop is never wider than
 * `radius` from the middle. */
void sign_hover(unsigned id, double seconds, double radius, double *dx, double *dy);

/* A number from 0 up to but not including 1 that belongs to `id`, the same each
 * time: for the things that must differ from bird to bird and be the same at any
 * frame rate. */
double sign_unit(unsigned id);

/* How long a sign is held, and how long the flock flies before writing it again,
 * for the nth time round: a little different each time, the same at any frame
 * rate. */
double sign_hold_seconds(unsigned cycle);
double sign_flight_seconds(unsigned cycle);

/* The colon's lift, zero to one and back to zero, once a second: given the part
 * of the second that has gone. */
double sign_breath(double fraction_of_a_second);

/* A rectangle the rest of the flock keeps out of, and the push that does it. */
typedef struct {
    double left, top, right, bottom;
} sign_box_t;

/* The push out of the box on a bird at (x, y): zero further than `band` pixels
 * away, one at the box's edge and growing to two at the middle. Returns 0 when
 * there is none. */
int sign_box_push(const sign_box_t *box, double band, double x, double y, double *push_x,
                  double *push_y);

#endif
