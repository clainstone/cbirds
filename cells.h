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
    CELLS_BLOCKS    /* Two half blocks a cell: coarser, and colour on every pixel. */
} cells_style_t;

typedef struct {
    uint32_t glyph; /* A code point; zero is an empty cell. */
    uint8_t fg[3];
    uint8_t bg[3];
    uint8_t has_fg, has_bg;
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
 * not of the pixels it was read from. The cells painted are the last emitted. */
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
