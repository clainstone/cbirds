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
    return 0;
}
