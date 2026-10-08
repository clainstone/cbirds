/*
 * A frame as a grid of terminal cells, for terminals that draw no images.
 *
 * The simulation is rendered into a pixel canvas exactly as it is for a
 * recording, and the canvas is then read back as cells: eight braille dots a
 * cell, or two half blocks, each cell wearing the colour of whatever bird is in
 * it. What comes out is the escape text that puts that grid on a screen — and
 * only the cells that changed since the last frame, because a terminal with no
 * graphics protocol is usually also a terminal with no spare bandwidth.
 *
 * Nothing here knows about birds. It knows RGBA pixels, cell sizes, and the
 * three sequences every terminal since the VT100 has understood: move, colour,
 * print.
 *
 * A cell can also be filled in by hand, glyph and style, for text that was never
 * pixels: the colour then says how it is to be written, as one of the terminal's
 * own sixteen or 256 so that it shows in the user's theme, or as 24 bit, and the
 * glyph may be wide. Cells read from a canvas are all plain 24 bit colour, and
 * what is sent for them has not changed.
 */

#ifndef CELLS_H
#define CELLS_H

#include <stddef.h>
#include <stdint.h>

#include "png.h"

typedef enum { CELLS_OK = 0, CELLS_ERR_ARGUMENT, CELLS_ERR_MEMORY } cells_status_t;

typedef enum {
    CELLS_BRAILLE,  /* Two by four dots a cell: the finest thing text can do. */
    CELLS_SEXTANTS, /* Two by three solid blocks a cell: bolder, nearly as fine. */
    CELLS_BLOCKS,   /* Two half blocks a cell: coarser, and colour on every pixel. */
    CELLS_TEXT      /* Glyphs of text, put there by hand; cells_read does not make them. */
} cells_style_t;

/* How a colour is written. RGB is the one cells_read makes: 24 bit when the
 * terminal has it and the nearest of the 256 colour cube when it has not. The
 * other three are for colours somebody chose and the terminal must show as they
 * are: the sixteen it themes, a 256 palette index, or 24 bit whatever COLORTERM
 * says. For the first two the colour is in [0] and the other bytes are not read. */
typedef enum {
    CELLS_COLOUR_RGB = 0,
    CELLS_COLOUR_ANSI,
    CELLS_COLOUR_INDEXED,
    CELLS_COLOUR_EXACT
} cells_colour_t;

enum { CELLS_BOLD = 1, CELLS_DIM = 2, CELLS_ITALIC = 4, CELLS_UNDERLINE = 8, CELLS_REVERSE = 16 };

/* A glyph two columns wide is a head, drawn once and moving the cursor on by two,
 * and a tail, which only holds the place and is never sent. */
enum { CELLS_NARROW = 0, CELLS_WIDE_HEAD = 1, CELLS_WIDE_TAIL = 2 };

typedef struct {
    uint32_t glyph; /* A code point; zero is an empty cell. */
    uint8_t fg[3];
    uint8_t bg[3];
    uint8_t has_fg, has_bg;
    uint8_t fg_kind, bg_kind; /* A cells_colour_t. */
    uint8_t attributes;       /* CELLS_BOLD and friends, or'd. */
    uint8_t wide;             /* CELLS_NARROW, CELLS_WIDE_HEAD or CELLS_WIDE_TAIL. */
} cell_t;

typedef struct {
    int cols, rows;
    cell_t *now, *before;
    int draw_everything;      /* The next emit redraws every cell, blanks included. */
    int keep_cols, keep_rows; /* A top left rectangle that belongs to somebody else. */
    int truecolor;
    char *text; /* What cells_emit produced, NUL terminated. */
    size_t length, capacity;
} cells_t;

/* truecolor picks 24 bit SGR; otherwise the nearest of the 256 colour cube. */
cells_status_t cells_init(cells_t *cells, int truecolor);
void cells_destroy(cells_t *cells);

/* Sizing, and the corner to leave alone (the parameter panel draws itself). */
cells_status_t cells_resize(cells_t *cells, int cols, int rows);
void cells_keep_out_of(cells_t *cells, int cols, int rows);
void cells_invalidate(cells_t *cells); /* Something else touched the screen. */

/* The grid the next cells_emit will send, for a caller that fills it in by hand
 * instead of reading a canvas: empty it, then set what is there. NULL outside it. */
void cells_clear(cells_t *cells);
cell_t *cells_at(cells_t *cells, int col, int row);

/* Reads the canvas into the current grid. The canvas is cols*cell_width by
 * rows*cell_height pixels or larger, with alpha marking ink: transparent is sky.
 * A pixel's colour is read straight, so the canvas should hold what the
 * terminal ought to show, not what a picture with a ground would. */
void cells_read(cells_t *cells, cells_style_t style, const png_image_t *canvas, int cell_width,
                int cell_height);

/* Produces the escape text that turns the previous frame into this one, into
 * cells->text, and makes this frame the previous one. */
cells_status_t cells_emit(cells_t *cells);

/* Paints the grid the way a terminal would show it — dots or half blocks in
 * their colours on a ground — into an image cell_width by cell_height pixels a
 * cell, so that a snapshot of a text terminal is a picture of what was on it and
 * not of the pixels it was read from. The cells painted are the last emitted.
 *
 * With CELLS_TEXT the glyphs are drawn with the 5 by 7 font, whole pixels at the
 * largest scale that fits, and the sixteen colours are given a palette of their
 * own, since a picture has no theme to borrow: the terminal's default colours are
 * a light grey on the ground. Glyphs the font does not carry that are blocks, box
 * drawing or braille are drawn as such; accented Latin letters lose their accent;
 * anything else is a hollow box. */
cells_status_t cells_paint(const cells_t *cells, cells_style_t style, png_image_t *out,
                           int cell_width, int cell_height, const uint8_t ground[3]);

/* The braille code point for a two by four dot pattern: bit (column + row * 2)
 * for each lit dot, column 0..1, row 0..3. Exposed for the tests. */
uint32_t cells_braille(unsigned dots);

/* The sextant code point for a two by three block pattern, bit (column + row * 2)
 * for each filled block, row 0..2: U+1FB00 onwards in order of the pattern's
 * value, except the four patterns that already had characters of their own. */
uint32_t cells_sextant(unsigned blocks);

const char *cells_status_string(cells_status_t status);

#endif
