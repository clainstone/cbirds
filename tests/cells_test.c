#define _POSIX_C_SOURCE 200809L
#include "../cells.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* A canvas of cols*8 by rows*16 transparent pixels, with a paint brush. */
static png_image_t blank(int cols, int rows) {
    png_image_t canvas = {0, 0, NULL};
    assert(png_image_alloc(&canvas, cols * 8, rows * 16) == PNG_OK);
    return canvas;
}

static void paint(png_image_t *canvas, int x, int y, int w, int h, uint8_t r, uint8_t g,
                  uint8_t b) {
    for (int yy = y; yy < y + h; yy++)
        for (int xx = x; xx < x + w; xx++) {
            uint8_t *px = &canvas->pixels[((size_t)yy * (size_t)canvas->width + (size_t)xx) * 4];
            px[0] = r;
            px[1] = g;
            px[2] = b;
            px[3] = 255;
        }
}

static const cell_t *at(const cells_t *cells, int row, int col) {
    /* After an emit the frame just read is in `before`. */
    return &cells->before[(size_t)row * (size_t)cells->cols + (size_t)col];
}

static void test_braille_dot_numbering(void) {
    /* Braille numbers its dots 1 2 3 down the left, 4 5 6 down the right, then
     * 7 and 8 along the bottom; U+2800 plus the bits. */
    assert(cells_braille(0) == 0x2800);
    assert(cells_braille(1u << 0) == 0x2801); /* Top left: dot 1. */
    assert(cells_braille(1u << 1) == 0x2808); /* Top right: dot 4. */
    assert(cells_braille(1u << 2) == 0x2802); /* Second row left: dot 2. */
    assert(cells_braille(1u << 6) == 0x2840); /* Bottom left: dot 7. */
    assert(cells_braille(1u << 7) == 0x2880); /* Bottom right: dot 8. */
    assert(cells_braille(0xFF) == 0x28FF);    /* Every dot. */
}

static void test_ink_becomes_dots_in_the_ink_colour(void) {
    cells_t cells;
    assert(cells_init(&cells, 1) == CELLS_OK);
    assert(cells_resize(&cells, 4, 2) == CELLS_OK);
    png_image_t canvas = blank(4, 2);

    /* A red square filling the top-left dot of cell (row 1, col 2) exactly:
     * a dot is 4 by 4 pixels of an 8 by 16 cell. */
    paint(&canvas, 2 * 8, 1 * 16, 4, 4, 255, 0, 0);
    cells_read(&cells, CELLS_BRAILLE, &canvas, 8, 16);
    assert(cells_emit(&cells) == CELLS_OK);

    const cell_t *cell = at(&cells, 1, 2);
    assert(cell->glyph == 0x2801);
    assert(cell->has_fg && cell->fg[0] == 255 && cell->fg[1] == 0 && cell->fg[2] == 0);
    assert(!cell->has_bg);
    /* And the other cells are empty. */
    assert(at(&cells, 0, 0)->glyph == 0);
    assert(at(&cells, 1, 3)->glyph == 0);

    /* The text that was emitted positions each row once — a first frame draws
     * every cell, blanks as spaces, so the cursor runs along the row on its own —
     * sets the colour and prints the glyph. */
    assert(strstr(cells.text, "\033[2;1H") != NULL);
    assert(strstr(cells.text, "\033[2;3H") == NULL);
    assert(strstr(cells.text, "\033[38;2;255;0;0m") != NULL);
    assert(strstr(cells.text, "\xe2\xa0\x81") != NULL); /* U+2801 in UTF-8. */
    assert(strstr(cells.text, "\033[0m") != NULL);

    png_image_free(&canvas);
    cells_destroy(&cells);
}

static void test_a_faint_fringe_is_not_a_dot(void) {
    cells_t cells;
    assert(cells_init(&cells, 1) == CELLS_OK);
    assert(cells_resize(&cells, 1, 1) == CELLS_OK);
    png_image_t canvas = blank(1, 1);
    /* One pixel of sixteen in the dot's patch: anti-aliasing, not a bird. */
    paint(&canvas, 0, 0, 1, 1, 255, 255, 255);
    cells_read(&cells, CELLS_BRAILLE, &canvas, 8, 16);
    assert(cells_emit(&cells) == CELLS_OK);
    assert(at(&cells, 0, 0)->glyph == 0);
    /* Half of it, and it is. */
    paint(&canvas, 0, 0, 4, 2, 255, 255, 255);
    cells_read(&cells, CELLS_BRAILLE, &canvas, 8, 16);
    assert(cells_emit(&cells) == CELLS_OK);
    assert(at(&cells, 0, 0)->glyph == 0x2801);
    png_image_free(&canvas);
    cells_destroy(&cells);
}

static void test_only_what_changed_is_emitted(void) {
    cells_t cells;
    assert(cells_init(&cells, 1) == CELLS_OK);
    assert(cells_resize(&cells, 40, 10) == CELLS_OK);
    png_image_t canvas = blank(40, 10);
    paint(&canvas, 0, 0, 8, 16, 0, 255, 0);
    cells_read(&cells, CELLS_BRAILLE, &canvas, 8, 16);
    assert(cells_emit(&cells) == CELLS_OK);
    size_t first = cells.length;

    /* The same frame again: nothing to say. */
    cells_read(&cells, CELLS_BRAILLE, &canvas, 8, 16);
    assert(cells_emit(&cells) == CELLS_OK);
    assert(cells.length == 0);

    /* The bird moves one cell right: the old cell is blanked, the new one drawn,
     * and that is all — a fraction of the first frame. */
    memset(canvas.pixels, 0, (size_t)canvas.width * (size_t)canvas.height * 4);
    paint(&canvas, 8, 0, 8, 16, 0, 255, 0);
    cells_read(&cells, CELLS_BRAILLE, &canvas, 8, 16);
    assert(cells_emit(&cells) == CELLS_OK);
    assert(cells.length > 0 && cells.length < first / 4);
    assert(strstr(cells.text, "\033[1;1H") != NULL); /* The blanked cell. */
    assert(strstr(cells.text, "\033[38;2;0;255;0m") != NULL);
    /* Two adjacent cells changed, so the cursor was positioned once. */
    assert(strstr(cells.text, "\033[1;2H") == NULL);

    /* Told the screen was touched, it redraws everything again. */
    cells_invalidate(&cells);
    cells_read(&cells, CELLS_BRAILLE, &canvas, 8, 16);
    assert(cells_emit(&cells) == CELLS_OK);
    assert(cells.length >= first / 2);

    png_image_free(&canvas);
    cells_destroy(&cells);
}

static void test_the_corner_left_to_the_panel_is_never_written(void) {
    cells_t cells;
    assert(cells_init(&cells, 1) == CELLS_OK);
    assert(cells_resize(&cells, 6, 4) == CELLS_OK);
    cells_keep_out_of(&cells, 3, 2);
    png_image_t canvas = blank(6, 4);
    /* Ink everywhere. */
    paint(&canvas, 0, 0, 6 * 8, 4 * 16, 200, 200, 200);
    cells_read(&cells, CELLS_BRAILLE, &canvas, 8, 16);
    assert(cells_emit(&cells) == CELLS_OK);
    for (int row = 0; row < 2; row++)
        for (int col = 0; col < 3; col++) assert(at(&cells, row, col)->glyph == 0);
    assert(at(&cells, 0, 3)->glyph == 0x28FF);
    assert(at(&cells, 2, 0)->glyph == 0x28FF);
    /* And no cursor move lands inside the corner. */
    assert(strstr(cells.text, "\033[1;1H") == NULL);
    assert(strstr(cells.text, "\033[2;2H") == NULL);
    assert(strstr(cells.text, "\033[1;4H") != NULL);
    png_image_free(&canvas);
    cells_destroy(&cells);
}

static void test_half_blocks_never_paint_the_sky(void) {
    cells_t cells;
    assert(cells_init(&cells, 1) == CELLS_OK);
    assert(cells_resize(&cells, 3, 1) == CELLS_OK);
    png_image_t canvas = blank(3, 1);
    paint(&canvas, 0, 0, 8, 8, 255, 0, 0);  /* Cell 0: top half only. */
    paint(&canvas, 8, 8, 8, 8, 0, 0, 255);  /* Cell 1: bottom half only. */
    paint(&canvas, 16, 0, 8, 8, 255, 0, 0); /* Cell 2: both. */
    paint(&canvas, 16, 8, 8, 8, 0, 0, 255);
    cells_read(&cells, CELLS_BLOCKS, &canvas, 8, 16);
    assert(cells_emit(&cells) == CELLS_OK);

    /* Top only: the upper half block in the top's colour, background untouched. */
    assert(at(&cells, 0, 0)->glyph == 0x2580 && at(&cells, 0, 0)->has_fg &&
           !at(&cells, 0, 0)->has_bg);
    assert(at(&cells, 0, 0)->fg[0] == 255);
    /* Bottom only: the lower half block, foreground, background untouched. */
    assert(at(&cells, 0, 1)->glyph == 0x2584 && !at(&cells, 0, 1)->has_bg);
    assert(at(&cells, 0, 1)->fg[2] == 255);
    /* Both: upper block, top colour in front, bottom colour behind. */
    assert(at(&cells, 0, 2)->glyph == 0x2580 && at(&cells, 0, 2)->has_bg);
    assert(at(&cells, 0, 2)->fg[0] == 255 && at(&cells, 0, 2)->bg[2] == 255);
    assert(strstr(cells.text, "\033[48;2;0;0;255m") != NULL);
    png_image_free(&canvas);
    cells_destroy(&cells);
}

/* A hop along the row the cursor is already on is a cursor forward, not a fresh
 * absolute position: half the bytes, over a frame of scattered changes. */
static void test_a_hop_along_the_row_is_a_cursor_forward(void) {
    cells_t cells;
    assert(cells_init(&cells, 1) == CELLS_OK);
    assert(cells_resize(&cells, 12, 2) == CELLS_OK);
    png_image_t canvas = blank(12, 2);
    paint(&canvas, 0, 0, 8, 16, 9, 9, 9);
    paint(&canvas, 5 * 8, 0, 8, 16, 9, 9, 9);
    paint(&canvas, 3 * 8, 16, 8, 16, 9, 9, 9);
    cells_read(&cells, CELLS_BRAILLE, &canvas, 8, 16);
    assert(cells_emit(&cells) == CELLS_OK); /* Everything, first time. */
    /* Now move the two birds on the top row one cell right each. */
    memset(canvas.pixels, 0, (size_t)canvas.width * (size_t)canvas.height * 4);
    paint(&canvas, 8, 0, 8, 16, 9, 9, 9);
    paint(&canvas, 6 * 8, 0, 8, 16, 9, 9, 9);
    paint(&canvas, 3 * 8, 16, 8, 16, 9, 9, 9);
    cells_read(&cells, CELLS_BRAILLE, &canvas, 8, 16);
    assert(cells_emit(&cells) == CELLS_OK);
    /* Row one: an absolute move to the start, then cells 0 and 1 in a run, then
     * a cursor forward of three to cell 5, then cells 5 and 6. Row two: nothing. */
    assert(strstr(cells.text, "\033[1;1H") != NULL);
    assert(strstr(cells.text, "\033[3C") != NULL);
    assert(strstr(cells.text, "\033[1;6H") == NULL);
    assert(strstr(cells.text, "\033[2;") == NULL);
    png_image_free(&canvas);
    cells_destroy(&cells);
}

static void test_the_pen_is_not_reset_between_cells_of_one_colour(void) {
    cells_t cells;
    assert(cells_init(&cells, 1) == CELLS_OK);
    assert(cells_resize(&cells, 8, 1) == CELLS_OK);
    png_image_t canvas = blank(8, 1);
    paint(&canvas, 0, 0, 8 * 8, 16, 10, 20, 30);
    cells_read(&cells, CELLS_BRAILLE, &canvas, 8, 16);
    assert(cells_emit(&cells) == CELLS_OK);
    /* Eight cells, one colour: the SGR appears once and the move once. */
    int colours = 0, moves = 0;
    for (const char *p = cells.text; (p = strstr(p, "\033[38;2;")) != NULL; p++) colours++;
    for (const char *p = cells.text; (p = strstr(p, "H")) != NULL; p++) moves++;
    assert(colours == 1);
    assert(moves == 1);
    png_image_free(&canvas);
    cells_destroy(&cells);
}

static void test_without_truecolor_the_cube_is_used(void) {
    cells_t cells;
    assert(cells_init(&cells, 0) == CELLS_OK);
    assert(cells_resize(&cells, 1, 1) == CELLS_OK);
    png_image_t canvas = blank(1, 1);
    paint(&canvas, 0, 0, 8, 16, 255, 0, 0);
    cells_read(&cells, CELLS_BRAILLE, &canvas, 8, 16);
    assert(cells_emit(&cells) == CELLS_OK);
    assert(strstr(cells.text, "\033[38;5;196m") != NULL); /* Pure red in the cube. */
    assert(strstr(cells.text, ";2;") == NULL);
    png_image_free(&canvas);
    cells_destroy(&cells);
}

static void test_a_resize_redraws_everything(void) {
    cells_t cells;
    assert(cells_init(&cells, 1) == CELLS_OK);
    assert(cells_resize(&cells, 4, 1) == CELLS_OK);
    png_image_t canvas = blank(4, 1);
    cells_read(&cells, CELLS_BRAILLE, &canvas, 8, 16);
    assert(cells_emit(&cells) == CELLS_OK);
    assert(cells_resize(&cells, 4, 1) == CELLS_OK); /* Same size: nothing changes. */
    assert(!cells.draw_everything);
    assert(cells_resize(&cells, 5, 2) == CELLS_OK);
    assert(cells.draw_everything);
    assert(cells.cols == 5 && cells.rows == 2);
    png_image_free(&canvas);
    cells_destroy(&cells);
}

static void test_painting_shows_what_the_terminal_showed(void) {
    cells_t cells;
    assert(cells_init(&cells, 1) == CELLS_OK);
    assert(cells_resize(&cells, 2, 1) == CELLS_OK);
    png_image_t canvas = blank(2, 1);
    paint(&canvas, 0, 0, 4, 4, 255, 0, 0); /* Top left dot of cell 0, red. */
    cells_read(&cells, CELLS_BRAILLE, &canvas, 8, 16);
    assert(cells_emit(&cells) == CELLS_OK);

    static const uint8_t ground[3] = {1, 2, 3};
    png_image_t picture = {0, 0, NULL};
    assert(cells_paint(&cells, CELLS_BRAILLE, &picture, 8, 16, ground) == CELLS_OK);
    assert(picture.width == 16 && picture.height == 16);
    /* The dot's patch is red inside its one pixel margin, the ground elsewhere. */
    const uint8_t *inside = &picture.pixels[(1 * 16 + 1) * 4];
    const uint8_t *corner = &picture.pixels[(0 * 16 + 0) * 4];
    const uint8_t *elsewhere = &picture.pixels[(10 * 16 + 12) * 4];
    assert(inside[0] == 255 && inside[1] == 0 && inside[2] == 0);
    assert(corner[0] == 1 && corner[1] == 2 && corner[2] == 3);
    assert(elsewhere[0] == 1 && elsewhere[1] == 2 && elsewhere[2] == 3);
    png_image_free(&picture);

    /* Blocks: the upper half block fills the top half of the cell. */
    memset(canvas.pixels, 0, (size_t)canvas.width * (size_t)canvas.height * 4);
    paint(&canvas, 8, 0, 8, 8, 0, 255, 0);
    cells_read(&cells, CELLS_BLOCKS, &canvas, 8, 16);
    assert(cells_emit(&cells) == CELLS_OK);
    assert(cells_paint(&cells, CELLS_BLOCKS, &picture, 8, 16, ground) == CELLS_OK);
    const uint8_t *top = &picture.pixels[(3 * 16 + 12) * 4];
    const uint8_t *bottom = &picture.pixels[(12 * 16 + 12) * 4];
    assert(top[1] == 255);
    assert(bottom[1] == 2); /* Ground: the sky is never painted. */
    png_image_free(&picture);
    png_image_free(&canvas);
    cells_destroy(&cells);
}

/* Two birds in one cell: the cell wears the colour of the one that fills more of
 * it, not a blend that is neither and different in every cell. */
static void test_a_shared_cell_wears_the_bigger_bird(void) {
    cells_t cells;
    assert(cells_init(&cells, 1) == CELLS_OK);
    assert(cells_resize(&cells, 1, 1) == CELLS_OK);
    png_image_t canvas = blank(1, 1);
    paint(&canvas, 0, 0, 8, 10, 200, 0, 0); /* Red, ten rows of sixteen. */
    paint(&canvas, 0, 10, 8, 6, 0, 0, 200); /* Blue, six. */
    cells_read(&cells, CELLS_BRAILLE, &canvas, 8, 16);
    assert(cells_emit(&cells) == CELLS_OK);
    const cell_t *cell = &cells.before[0];
    assert(cell->fg[0] == 200 && cell->fg[1] == 0 && cell->fg[2] == 0);
    /* Weighted by how much of each pixel is ink, not by pixels. */
    for (int y = 0; y < 10; y++)
        for (int x = 0; x < 8; x++) canvas.pixels[(y * 8 + x) * 4 + 3] = 60; /* Faint red. */
    cells_read(&cells, CELLS_BRAILLE, &canvas, 8, 16);
    assert(cells_emit(&cells) == CELLS_OK);
    assert(cells.before[0].fg[2] == 200); /* Six solid rows of blue outweigh ten faint of red. */
    png_image_free(&canvas);
    cells_destroy(&cells);
}

/* Sextants: two by three solid blocks a cell, laid out by Unicode 13 in order of
 * their pattern with the four that already existed left out. */
static void test_sextant_code_points(void) {
    assert(cells_sextant(0) == ' ');
    assert(cells_sextant(0x01) == 0x1FB00); /* Top left alone: SEXTANT-1. */
    assert(cells_sextant(0x02) == 0x1FB01); /* SEXTANT-2. */
    assert(cells_sextant(0x03) == 0x1FB02); /* SEXTANT-12. */
    assert(cells_sextant(0x14) == 0x1FB13); /* SEXTANT-35, the last before the left half. */
    assert(cells_sextant(0x15) == 0x258C);  /* SEXTANT-135 is the left half block. */
    assert(cells_sextant(0x16) == 0x1FB14); /* And the count skips it. */
    assert(cells_sextant(0x2A) == 0x2590);  /* SEXTANT-246 is the right half block. */
    assert(cells_sextant(0x2B) == 0x1FB28); /* Skipping both. */
    assert(cells_sextant(0x3E) == 0x1FB3B); /* SEXTANT-23456, the last in the block. */
    assert(cells_sextant(0x3F) == 0x2588);  /* Everything is the full block. */
    /* Every pattern gets its own character, and the block is exactly filled. */
    int seen[0x40] = {0};
    for (unsigned p = 1; p < 0x3F; p++) {
        uint32_t code = cells_sextant(p);
        if (code >= 0x1FB00 && code <= 0x1FB3B) seen[code - 0x1FB00]++;
    }
    for (int i = 0; i <= 0x3B; i++) assert(seen[i] == 1);
}

static void test_ink_becomes_sextants_too(void) {
    cells_t cells;
    assert(cells_init(&cells, 1) == CELLS_OK);
    assert(cells_resize(&cells, 2, 1) == CELLS_OK);
    png_image_t canvas = blank(2, 1);
    /* The top third of cell 1, both columns: rows of 16 split 5/5/6. */
    paint(&canvas, 8, 0, 8, 5, 0, 200, 0);
    cells_read(&cells, CELLS_SEXTANTS, &canvas, 8, 16);
    assert(cells_emit(&cells) == CELLS_OK);
    assert(cells.before[1].glyph == 0x1FB02); /* SEXTANT-12. */
    assert(cells.before[1].fg[1] == 200);
    assert(cells.before[0].glyph == 0);
    /* The whole cell is the full block, which is not in the sextant block. */
    paint(&canvas, 8, 0, 8, 16, 0, 200, 0);
    cells_read(&cells, CELLS_SEXTANTS, &canvas, 8, 16);
    assert(cells_emit(&cells) == CELLS_OK);
    assert(cells.before[1].glyph == 0x2588);
    /* And a painting of it fills the cell. */
    static const uint8_t ground[3] = {1, 2, 3};
    png_image_t picture = {0, 0, NULL};
    assert(cells_paint(&cells, CELLS_SEXTANTS, &picture, 8, 16, ground) == CELLS_OK);
    assert(picture.pixels[(15 * 16 + 15) * 4 + 1] == 200);
    assert(picture.pixels[(15 * 16 + 2) * 4 + 1] == 2);
    png_image_free(&picture);
    png_image_free(&canvas);
    cells_destroy(&cells);
}

/* --- Cells filled in by hand ------------------------------------------------- */

static cell_t *put(cells_t *cells, int col, int row, uint32_t glyph) {
    cell_t *cell = cells_at(cells, col, row);
    assert(cell != NULL);
    memset(cell, 0, sizeof(*cell));
    cell->glyph = glyph;
    return cell;
}

static void colour(cell_t *cell, int background, uint8_t kind, uint8_t a, uint8_t b, uint8_t c) {
    uint8_t *rgb = background ? cell->bg : cell->fg;
    rgb[0] = a;
    rgb[1] = b;
    rgb[2] = c;
    if (background) {
        cell->has_bg = 1;
        cell->bg_kind = kind;
    } else {
        cell->has_fg = 1;
        cell->fg_kind = kind;
    }
}

static void test_cells_can_be_filled_in_by_hand(void) {
    cells_t cells;
    assert(cells_init(&cells, 1) == CELLS_OK);
    assert(cells_resize(&cells, 3, 2) == CELLS_OK);
    assert(cells_at(&cells, 3, 0) == NULL && cells_at(&cells, 0, 2) == NULL);
    assert(cells_at(&cells, -1, 0) == NULL && cells_at(NULL, 0, 0) == NULL);
    put(&cells, 1, 1, 'x');
    assert(cells_at(&cells, 1, 1)->glyph == 'x');
    cells_clear(&cells);
    assert(cells_at(&cells, 1, 1)->glyph == 0);
    cells_clear(NULL);
    cells_destroy(&cells);
}

static void test_a_palette_colour_is_written_as_the_terminals_own(void) {
    cells_t cells;
    assert(cells_init(&cells, 1) == CELLS_OK);
    assert(cells_resize(&cells, 8, 1) == CELLS_OK);
    colour(put(&cells, 0, 0, 'a'), 0, CELLS_COLOUR_ANSI, 1, 0, 0);
    colour(put(&cells, 1, 0, 'b'), 0, CELLS_COLOUR_ANSI, 7, 0, 0);
    colour(put(&cells, 2, 0, 'c'), 0, CELLS_COLOUR_ANSI, 12, 0, 0);
    cell_t *both = put(&cells, 3, 0, 'd');
    colour(both, 0, CELLS_COLOUR_ANSI, 15, 0, 0);
    colour(both, 1, CELLS_COLOUR_ANSI, 4, 9, 9);
    colour(put(&cells, 4, 0, 'e'), 1, CELLS_COLOUR_ANSI, 9, 0, 0);
    assert(cells_emit(&cells) == CELLS_OK);
    /* 30 to 37 and 90 to 97, 40 to 47 and 100 to 107: no 24 bit colour anywhere,
     * so the terminal's theme decides what they look like. */
    assert(strstr(cells.text, "\033[31ma") != NULL);
    assert(strstr(cells.text, "\033[37mb") != NULL);
    assert(strstr(cells.text, "\033[94mc") != NULL);
    assert(strstr(cells.text, "\033[97m\033[44md") != NULL);
    assert(strstr(cells.text, "\033[101me") != NULL);
    assert(strstr(cells.text, "38;2") == NULL && strstr(cells.text, "38;5") == NULL);
    cells_destroy(&cells);
}

static void test_a_256_colour_index_is_written_as_an_index(void) {
    cells_t cells;
    assert(cells_init(&cells, 0) == CELLS_OK); /* A terminal with no 24 bit colour. */
    assert(cells_resize(&cells, 4, 1) == CELLS_OK);
    cell_t *a = put(&cells, 0, 0, 'a');
    colour(a, 0, CELLS_COLOUR_INDEXED, 208, 0, 0);
    colour(a, 1, CELLS_COLOUR_INDEXED, 17, 0, 0);
    assert(cells_emit(&cells) == CELLS_OK);
    assert(strstr(cells.text, "\033[38;5;208m\033[48;5;17ma") != NULL);
    cells_destroy(&cells);
}

static void test_exact_colour_is_24_bit_whatever_the_terminal_admits_to(void) {
    cells_t cells;
    assert(cells_init(&cells, 0) == CELLS_OK);
    assert(cells_resize(&cells, 4, 1) == CELLS_OK);
    colour(put(&cells, 0, 0, 'a'), 0, CELLS_COLOUR_EXACT, 10, 20, 30);
    colour(put(&cells, 1, 0, 'b'), 0, CELLS_COLOUR_RGB, 10, 20, 30);
    assert(cells_emit(&cells) == CELLS_OK);
    /* Input that used 24 bit asked for it, and the terminal that was shown it is
     * shown it again; the colours we made up ourselves follow what COLORTERM says. */
    assert(strstr(cells.text, "\033[38;2;10;20;30ma") != NULL);
    assert(strstr(cells.text, "\033[38;5;") != NULL);
    cells_destroy(&cells);
}

static void test_attributes_are_added_without_a_reset_and_dropped_with_one(void) {
    cells_t cells;
    assert(cells_init(&cells, 1) == CELLS_OK);
    assert(cells_resize(&cells, 6, 1) == CELLS_OK);
    put(&cells, 0, 0, 'a')->attributes = CELLS_BOLD;
    put(&cells, 1, 0, 'b')->attributes = CELLS_BOLD | CELLS_UNDERLINE;
    put(&cells, 2, 0, 'c')->attributes = CELLS_BOLD | CELLS_UNDERLINE;
    put(&cells, 3, 0, 'd')->attributes = CELLS_REVERSE | CELLS_ITALIC | CELLS_DIM;
    put(&cells, 4, 0, 'e');
    assert(cells_emit(&cells) == CELLS_OK);
    /* Bold, then underline added on top of it, then nothing for the same again. */
    assert(strstr(cells.text, "\033[1ma\033[4mbc") != NULL);
    /* Dropping bold and underline is a reset and then what the next one wants. */
    assert(strstr(cells.text, "c\033[0m\033[2;3;7md") != NULL);
    assert(strstr(cells.text, "d\033[0me") != NULL);
    cells_destroy(&cells);
}

static void test_a_cell_that_only_changes_attribute_is_sent_again(void) {
    cells_t cells;
    assert(cells_init(&cells, 1) == CELLS_OK);
    assert(cells_resize(&cells, 3, 1) == CELLS_OK);
    put(&cells, 1, 0, 'x');
    assert(cells_emit(&cells) == CELLS_OK);
    put(&cells, 1, 0, 'x')->attributes = CELLS_UNDERLINE;
    assert(cells_emit(&cells) == CELLS_OK);
    assert(strstr(cells.text, "\033[4mx") != NULL);
    /* The same again, nothing. */
    put(&cells, 1, 0, 'x')->attributes = CELLS_UNDERLINE;
    assert(cells_emit(&cells) == CELLS_OK);
    assert(cells.length == 0);
    /* A palette index differs by its index and not by the bytes around it. */
    cell_t *y = put(&cells, 1, 0, 'x');
    y->attributes = CELLS_UNDERLINE;
    colour(y, 0, CELLS_COLOUR_ANSI, 2, 99, 99);
    assert(cells_emit(&cells) == CELLS_OK);
    assert(cells.length > 0);
    y = put(&cells, 1, 0, 'x');
    y->attributes = CELLS_UNDERLINE;
    colour(y, 0, CELLS_COLOUR_ANSI, 2, 7, 7);
    assert(cells_emit(&cells) == CELLS_OK);
    assert(cells.length == 0);
    cells_destroy(&cells);
}

static void test_a_wide_glyph_is_sent_once_and_its_tail_never(void) {
    cells_t cells;
    assert(cells_init(&cells, 1) == CELLS_OK);
    assert(cells_resize(&cells, 6, 1) == CELLS_OK);
    put(&cells, 0, 0, 'a');
    put(&cells, 1, 0, 0x4E2D)->wide = CELLS_WIDE_HEAD;
    put(&cells, 2, 0, 0)->wide = CELLS_WIDE_TAIL;
    put(&cells, 3, 0, 'b');
    assert(cells_emit(&cells) == CELLS_OK);
    /* a, the wide character, b: the cursor needs no move between them, because the
     * terminal advanced two columns by itself. */
    assert(strstr(cells.text,
                  "a\xE4\xB8\xAD"
                  "b") != NULL);
    cells_destroy(&cells);
}

static void test_a_wide_glyph_is_redrawn_whole_when_half_of_it_changes(void) {
    cells_t cells;
    assert(cells_init(&cells, 1) == CELLS_OK);
    assert(cells_resize(&cells, 6, 1) == CELLS_OK);
    put(&cells, 1, 0, 0x4E2D)->wide = CELLS_WIDE_HEAD;
    put(&cells, 2, 0, 0)->wide = CELLS_WIDE_TAIL;
    assert(cells_emit(&cells) == CELLS_OK);

    /* Nothing changed: nothing sent. */
    put(&cells, 1, 0, 0x4E2D)->wide = CELLS_WIDE_HEAD;
    put(&cells, 2, 0, 0)->wide = CELLS_WIDE_TAIL;
    assert(cells_emit(&cells) == CELLS_OK);
    assert(cells.length == 0);

    /* Only its second cell is different, in colour: the character is drawn again. */
    put(&cells, 1, 0, 0x4E2D)->wide = CELLS_WIDE_HEAD;
    colour(put(&cells, 2, 0, 0), 1, CELLS_COLOUR_ANSI, 1, 0, 0);
    cells_at(&cells, 2, 0)->wide = CELLS_WIDE_TAIL;
    assert(cells_emit(&cells) == CELLS_OK);
    assert(strstr(cells.text, "\xE4\xB8\xAD") != NULL);

    /* Something narrow lands on its second cell: the old glyph is gone from the
     * grid, so both cells of it are written, the first as what it now is. */
    put(&cells, 1, 0, 'x');
    put(&cells, 2, 0, 'y');
    assert(cells_emit(&cells) == CELLS_OK);
    assert(strstr(cells.text, "xy") != NULL);
    cells_destroy(&cells);
}

static void test_the_default_bytes_have_not_changed(void) {
    /* The shape of everything cells_read makes, which is plain 24 bit colour and
     * no attributes: what a terminal was sent before styles existed. */
    cells_t cells;
    assert(cells_init(&cells, 1) == CELLS_OK);
    assert(cells_resize(&cells, 3, 1) == CELLS_OK);
    png_image_t canvas = blank(3, 1);
    paint(&canvas, 0, 0, 4, 4, 255, 0, 0);
    paint(&canvas, 8, 0, 4, 4, 0, 255, 0);
    cells_read(&cells, CELLS_BRAILLE, &canvas, 8, 16);
    assert(cells_emit(&cells) == CELLS_OK);
    assert(strcmp(cells.text,
                  "\033[1;1H\033[38;2;255;0;0m\xE2\xA0\x81\033[38;2;0;255;0m\xE2\xA0\x81"
                  "\033[0m \033[0m") == 0);
    png_image_free(&canvas);
    cells_destroy(&cells);
}

/* --- Painting text ------------------------------------------------------------ */

static const uint8_t *pixel(const png_image_t *picture, int x, int y) {
    return &picture->pixels[((size_t)y * (size_t)picture->width + (size_t)x) * 4];
}

static int lit_pixels(const png_image_t *picture, int x0, int y0, int width, int height,
                      const uint8_t ground[3]) {
    int lit = 0;
    for (int y = y0; y < y0 + height; y++)
        for (int x = x0; x < x0 + width; x++)
            if (memcmp(pixel(picture, x, y), ground, 3) != 0) lit++;
    return lit;
}

static void emit_and_paint(cells_t *cells, png_image_t *picture, const uint8_t ground[3]) {
    assert(cells_emit(cells) == CELLS_OK);
    assert(cells_paint(cells, CELLS_TEXT, picture, 12, 20, ground) == CELLS_OK);
}

static void test_text_is_painted_with_the_font_in_its_colour(void) {
    static const uint8_t ground[3] = {10, 10, 10};
    cells_t cells;
    assert(cells_init(&cells, 1) == CELLS_OK);
    assert(cells_resize(&cells, 3, 1) == CELLS_OK);
    colour(put(&cells, 0, 0, 'A'), 0, CELLS_COLOUR_EXACT, 200, 10, 10);
    put(&cells, 1, 0, 'B'); /* The default colour. */
    png_image_t picture = {0, 0, NULL};
    emit_and_paint(&cells, &picture, ground);
    assert(picture.width == 36 && picture.height == 20);

    /* An A has ink in the first row of its glyph, at the two pixels the font puts
     * there, doubled: the cell is 12 by 20 and the glyph 5 by 7 at scale 2. */
    int ink = lit_pixels(&picture, 0, 0, 12, 20, ground);
    assert(ink > 40 && ink < 200);
    int found_red = 0;
    for (int y = 0; y < 20; y++)
        for (int x = 0; x < 12; x++) {
            const uint8_t *p = pixel(&picture, x, y);
            if (memcmp(p, ground, 3) != 0) {
                assert(p[0] == 200 && p[1] == 10 && p[2] == 10);
                found_red = 1;
            }
        }
    assert(found_red);
    /* The B beside it is the default light grey, and nothing spills across. */
    assert(lit_pixels(&picture, 12, 0, 12, 20, ground) > 40);
    const uint8_t *grey = NULL;
    for (int x = 12; x < 24 && grey == NULL; x++)
        if (memcmp(pixel(&picture, x, 10), ground, 3) != 0) grey = pixel(&picture, x, 10);
    assert(grey != NULL && grey[0] == 204 && grey[1] == 204 && grey[2] == 204);
    assert(lit_pixels(&picture, 24, 0, 12, 20, ground) == 0);
    png_image_free(&picture);
    cells_destroy(&cells);
}

static void test_painted_text_has_lower_case_and_the_rest_of_ascii(void) {
    static const uint8_t ground[3] = {0, 0, 0};
    cells_t cells;
    assert(cells_init(&cells, 1) == CELLS_OK);
    assert(cells_resize(&cells, 1, 1) == CELLS_OK);
    png_image_t picture = {0, 0, NULL};
    int sizes[2];
    uint32_t pair[2] = {'a', 'A'};
    for (int i = 0; i < 2; i++) {
        cells_clear(&cells);
        put(&cells, 0, 0, pair[i]);
        emit_and_paint(&cells, &picture, ground);
        sizes[i] = lit_pixels(&picture, 0, 0, 12, 20, ground);
        png_image_free(&picture);
    }
    /* An a is not an A: lower case has its own glyph, with its own ink. */
    assert(sizes[0] > 0 && sizes[1] > 0 && sizes[0] != sizes[1]);
    /* Every printable ASCII character has a picture. */
    for (uint32_t c = '!'; c <= '~'; c++) {
        cells_clear(&cells);
        put(&cells, 0, 0, c);
        emit_and_paint(&cells, &picture, ground);
        assert(lit_pixels(&picture, 0, 0, 12, 20, ground) > 0);
        png_image_free(&picture);
    }
    /* A space has none. */
    cells_clear(&cells);
    put(&cells, 0, 0, ' ');
    emit_and_paint(&cells, &picture, ground);
    assert(lit_pixels(&picture, 0, 0, 12, 20, ground) == 0);
    png_image_free(&picture);
    cells_destroy(&cells);
}

static void test_painted_attributes(void) {
    static const uint8_t ground[3] = {10, 20, 30};
    cells_t cells;
    assert(cells_init(&cells, 1) == CELLS_OK);
    assert(cells_resize(&cells, 5, 1) == CELLS_OK);
    put(&cells, 0, 0, 'l');
    put(&cells, 1, 0, 'l')->attributes = CELLS_BOLD;
    put(&cells, 2, 0, 'l')->attributes = CELLS_UNDERLINE;
    put(&cells, 3, 0, 'l')->attributes = CELLS_REVERSE;
    put(&cells, 4, 0, 'l')->attributes = CELLS_DIM;
    png_image_t picture = {0, 0, NULL};
    emit_and_paint(&cells, &picture, ground);
    int plain = lit_pixels(&picture, 0, 0, 12, 20, ground);
    int bold = lit_pixels(&picture, 12, 0, 12, 20, ground);
    int underlined = lit_pixels(&picture, 24, 0, 12, 20, ground);
    assert(bold > plain);
    assert(underlined == plain + 12);
    /* Reverse paints the whole cell in the foreground, and the glyph in the ground. */
    assert(lit_pixels(&picture, 36, 0, 12, 20, ground) > 12 * 20 / 2);
    assert(memcmp(pixel(&picture, 36, 0), (uint8_t[3]){204, 204, 204}, 3) == 0);
    /* Dim is the foreground halfway to the background. */
    const uint8_t *dim = NULL;
    for (int y = 0; y < 20 && dim == NULL; y++)
        for (int x = 48; x < 60; x++)
            if (memcmp(pixel(&picture, x, y), ground, 3) != 0) {
                dim = pixel(&picture, x, y);
                break;
            }
    assert(dim != NULL && dim[0] == (204 + 10) / 2 && dim[1] == (204 + 20) / 2);
    png_image_free(&picture);
    cells_destroy(&cells);
}

static void test_painted_palette_colours_are_the_pictures_own(void) {
    static const uint8_t ground[3] = {0, 0, 0};
    cells_t cells;
    assert(cells_init(&cells, 1) == CELLS_OK);
    assert(cells_resize(&cells, 3, 1) == CELLS_OK);
    colour(put(&cells, 0, 0, 'l'), 0, CELLS_COLOUR_ANSI, 1, 0, 0);
    cell_t *bold = put(&cells, 1, 0, 'l');
    colour(bold, 0, CELLS_COLOUR_ANSI, 1, 0, 0);
    bold->attributes = CELLS_BOLD;
    colour(put(&cells, 2, 0, 'l'), 0, CELLS_COLOUR_INDEXED, 232, 0, 0);
    png_image_t picture = {0, 0, NULL};
    emit_and_paint(&cells, &picture, ground);
    const uint8_t *red = NULL, *bright = NULL, *grey = NULL;
    for (int y = 0; y < 20; y++)
        for (int x = 0; x < 12; x++) {
            if (!red && memcmp(pixel(&picture, x, y), ground, 3) != 0) red = pixel(&picture, x, y);
            if (!bright && memcmp(pixel(&picture, 12 + x, y), ground, 3) != 0)
                bright = pixel(&picture, 12 + x, y);
            if (!grey && memcmp(pixel(&picture, 24 + x, y), ground, 3) != 0)
                grey = pixel(&picture, 24 + x, y);
        }
    assert(red && bright && grey);
    assert(red[0] == 205 && red[1] == 49);       /* Red, in the picture's palette. */
    assert(bright[0] == 241 && bright[1] == 76); /* Bold makes it the bright red. */
    assert(grey[0] == 8 && grey[1] == 8);        /* The first grey of the 256. */
    png_image_free(&picture);
    cells_destroy(&cells);
}

static void test_painted_blocks_boxes_and_braille_fill_the_cell(void) {
    static const uint8_t ground[3] = {0, 0, 0};
    cells_t cells;
    assert(cells_init(&cells, 1) == CELLS_OK);
    assert(cells_resize(&cells, 6, 1) == CELLS_OK);
    put(&cells, 0, 0, 0x2588); /* Full block. */
    put(&cells, 1, 0, 0x2580); /* Upper half. */
    put(&cells, 2, 0, 0x2502); /* Vertical line. */
    put(&cells, 3, 0, 0x253C); /* Cross. */
    put(&cells, 4, 0, 0x28FF); /* Every braille dot. */
    put(&cells, 5, 0, 0x2593); /* Dark shade. */
    png_image_t picture = {0, 0, NULL};
    emit_and_paint(&cells, &picture, ground);
    assert(lit_pixels(&picture, 0, 0, 12, 20, ground) == 12 * 20);
    assert(lit_pixels(&picture, 12, 0, 12, 20, ground) == 12 * 10);
    assert(lit_pixels(&picture, 12, 0, 12, 10, ground) == 12 * 10);
    /* A vertical line touches the top and bottom edges, so a column of them joins. */
    assert(memcmp(pixel(&picture, 24 + 5, 0), ground, 3) != 0);
    assert(memcmp(pixel(&picture, 24 + 5, 19), ground, 3) != 0);
    assert(memcmp(pixel(&picture, 24 + 0, 10), ground, 3) == 0);
    /* A cross reaches all four edges. */
    assert(memcmp(pixel(&picture, 36 + 5, 0), ground, 3) != 0);
    assert(memcmp(pixel(&picture, 36 + 5, 19), ground, 3) != 0);
    assert(memcmp(pixel(&picture, 36 + 0, 9), ground, 3) != 0);
    assert(memcmp(pixel(&picture, 36 + 11, 9), ground, 3) != 0);
    assert(lit_pixels(&picture, 48, 0, 12, 20, ground) > 40);
    int dark = lit_pixels(&picture, 60, 0, 12, 20, ground);
    assert(dark > 12 * 20 / 2 && dark < 12 * 20);
    png_image_free(&picture);
    cells_destroy(&cells);
}

static void test_painted_accents_fall_back_and_the_unknown_is_a_box(void) {
    static const uint8_t ground[3] = {0, 0, 0};
    cells_t cells;
    assert(cells_init(&cells, 1) == CELLS_OK);
    assert(cells_resize(&cells, 4, 1) == CELLS_OK);
    put(&cells, 0, 0, 'e');
    put(&cells, 1, 0, 0xE9);                           /* é, which is an e in a 5 by 7. */
    put(&cells, 2, 0, 0x4E2D)->wide = CELLS_WIDE_HEAD; /* CJK, which the font cannot draw. */
    put(&cells, 3, 0, 0)->wide = CELLS_WIDE_TAIL;
    png_image_t picture = {0, 0, NULL};
    emit_and_paint(&cells, &picture, ground);
    int plain = lit_pixels(&picture, 0, 0, 12, 20, ground);
    assert(plain > 0 && lit_pixels(&picture, 12, 0, 12, 20, ground) == plain);
    /* The box is hollow and spans both cells. */
    assert(lit_pixels(&picture, 24, 0, 24, 20, ground) > 40);
    assert(memcmp(pixel(&picture, 24 + 12, 10), ground, 3) == 0);
    assert(memcmp(pixel(&picture, 24 + 12, 3), ground, 3) != 0);
    png_image_free(&picture);
    cells_destroy(&cells);
}

int main(void) {
    test_sextant_code_points();
    test_ink_becomes_sextants_too();
    test_painting_shows_what_the_terminal_showed();
    test_a_shared_cell_wears_the_bigger_bird();
    test_braille_dot_numbering();
    test_ink_becomes_dots_in_the_ink_colour();
    test_a_faint_fringe_is_not_a_dot();
    test_only_what_changed_is_emitted();
    test_the_corner_left_to_the_panel_is_never_written();
    test_half_blocks_never_paint_the_sky();
    test_a_hop_along_the_row_is_a_cursor_forward();
    test_the_pen_is_not_reset_between_cells_of_one_colour();
    test_without_truecolor_the_cube_is_used();
    test_a_resize_redraws_everything();
    test_cells_can_be_filled_in_by_hand();
    test_a_palette_colour_is_written_as_the_terminals_own();
    test_a_256_colour_index_is_written_as_an_index();
    test_exact_colour_is_24_bit_whatever_the_terminal_admits_to();
    test_attributes_are_added_without_a_reset_and_dropped_with_one();
    test_a_cell_that_only_changes_attribute_is_sent_again();
    test_a_wide_glyph_is_sent_once_and_its_tail_never();
    test_a_wide_glyph_is_redrawn_whole_when_half_of_it_changes();
    test_the_default_bytes_have_not_changed();
    test_text_is_painted_with_the_font_in_its_colour();
    test_painted_text_has_lower_case_and_the_rest_of_ascii();
    test_painted_attributes();
    test_painted_palette_colours_are_the_pictures_own();
    test_painted_blocks_boxes_and_braille_fill_the_cell();
    test_painted_accents_fall_back_and_the_unknown_is_a_box();
    return 0;
}
