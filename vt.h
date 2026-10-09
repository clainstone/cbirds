/*
 * A small terminal emulator: bytes in, a grid of styled cells out.
 *
 * It exists so that text piped into cbirds can be laid out the way the command
 * meant it. fastfetch and neofetch do not print lines, they place things: a
 * logo, then a cursor that goes back up and across to write the information
 * beside it. Reading that as plain lines would give a staircase. Fed the same
 * bytes, this keeps what a terminal would have on its screen when the command
 * finished, and nothing else.
 *
 * It understands what such tools use and ignores the rest without harm: UTF-8
 * with wide characters, the C0 controls a printer cares about, cursor movement
 * and erasing, SGR colours and attributes, saving the cursor, and autowrap. Every
 * other sequence is read to its end and thrown away, so a stray escape cannot
 * leave the parser stuck in the middle of a string. The state is a struct the
 * caller owns, and feeding it never allocates.
 */

#ifndef VT_H
#define VT_H

#include <stddef.h>
#include <stdint.h>

typedef enum {
    VT_COLOUR_DEFAULT = 0, /* The terminal's own foreground or background. */
    VT_COLOUR_ANSI,        /* One of the sixteen, 0 to 15: SGR 30-37, 90-97, 40-47, 100-107. */
    VT_COLOUR_INDEXED,     /* 16 to 255 of the 256 colour palette: SGR 38;5;n. */
    VT_COLOUR_RGB          /* 24 bit: SGR 38;2;r;g;b. */
} vt_colour_kind_t;

typedef struct {
    uint8_t kind;     /* A vt_colour_kind_t. */
    uint8_t value[3]; /* The index in value[0] for ANSI and INDEXED; red, green, blue for RGB. */
} vt_colour_t;

enum { VT_BOLD = 1, VT_DIM = 2, VT_ITALIC = 4, VT_UNDERLINE = 8, VT_REVERSE = 16 };

typedef struct {
    vt_colour_t fg, bg;
    uint8_t attributes; /* VT_BOLD and friends, or'd. */
} vt_style_t;

typedef struct {
    /* A code point. Zero is a cell nothing was printed in, and the second cell of
     * a wide character. A printed space is a space. */
    uint32_t glyph;
    vt_style_t style;
    /* One cell for most characters, two for East Asian wide ones and emoji, which
     * mark the first cell with two and the second with zero. */
    uint8_t width;
} vt_cell_t;

enum { VT_MAX_PARAMS = 16 };

typedef struct {
    int cols, rows;
    vt_cell_t *storage;
    vt_cell_t **line; /* rows pointers into storage; a scroll turns them, never copies cells. */

    int cursor_col, cursor_row;
    int wrap_pending; /* The last column has been written; the next glyph wraps first. */
    int autowrap;     /* DEC mode 7, which neofetch turns off while it draws. */
    vt_style_t pen;
    uint32_t last_glyph; /* For REP. */

    int saved, saved_col, saved_row, saved_wrap_pending; /* ESC 7, CSI s. */
    vt_style_t saved_pen;

    /* The parser. */
    int state;
    int params[VT_MAX_PARAMS];
    uint8_t colon_after[VT_MAX_PARAMS]; /* The separator after this parameter was a colon. */
    int param_count, params_seen;
    int marker;       /* '?', '>', '<' or '=' at the start of a CSI, else 0. */
    int intermediate; /* A byte from 0x20 to 0x2f was seen. */
    int utf8_need;    /* Continuation bytes still to come. */
    uint32_t utf8_code, utf8_least;

    long scrolled; /* Lines that have gone off the top. */
    long printed;  /* Glyphs written, wide ones once. */
} vt_t;

/* A blank screen of cols by rows, the cursor at the top left. Returns 0, or -1 for
 * a size that is not at least one by one or memory that is not there. */
int vt_init(vt_t *vt, int cols, int rows);
void vt_destroy(vt_t *vt);

/* Interprets the bytes as a terminal would, in the order they come. Chunks may end
 * anywhere, in the middle of a sequence or a UTF-8 character. */
void vt_feed(vt_t *vt, const void *bytes, size_t length);

/* The input has ended: a UTF-8 character that was cut short becomes U+FFFD, and a
 * sequence that never finished is dropped. */
void vt_finish(vt_t *vt);

/* The cell at a column and row, counted from zero, or NULL outside the screen. */
const vt_cell_t *vt_cell(const vt_t *vt, int col, int row);

/* How many columns a code point takes: 0 for a combining mark or other format
 * character, 2 for an East Asian wide one or an emoji, 1 for everything else. */
int vt_glyph_width(uint32_t glyph);

/* Whether nothing visible is in the cell: never printed in, or a space. */
int vt_cell_is_blank(const vt_cell_t *cell);

#endif
