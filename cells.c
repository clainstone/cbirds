#include "cells.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* How much of a dot's patch of pixels has to be ink before the dot lights: a
 * quarter. Less and every anti-aliased fringe is a dot and the birds are fat
 * blobs; more and the wing tips vanish and the birds are commas. */
enum { INK_THRESHOLD = 64 };

/* Braille numbers its dots down the left column then down the right, with the
 * fourth row bolted on afterwards: 1 2 3 7 on the left, 4 5 6 8 on the right. */
static const unsigned BRAILLE_BIT[4][2] = {{0x01, 0x08}, {0x02, 0x10}, {0x04, 0x20}, {0x40, 0x80}};

uint32_t cells_braille(unsigned dots) {
    unsigned pattern = 0;
    for (int row = 0; row < 4; row++)
        for (int column = 0; column < 2; column++)
            if (dots & (1u << (column + row * 2))) pattern |= BRAILLE_BIT[row][column];
    return 0x2800 + pattern;
}

const char *cells_status_string(cells_status_t status) {
    switch (status) {
        case CELLS_OK:
            return "ok";
        case CELLS_ERR_ARGUMENT:
            return "invalid argument";
        case CELLS_ERR_MEMORY:
            return "out of memory";
    }
    return "unknown";
}

cells_status_t cells_init(cells_t *cells, int truecolor) {
    if (cells == NULL) return CELLS_ERR_ARGUMENT;
    memset(cells, 0, sizeof(*cells));
    cells->truecolor = truecolor;
    cells->draw_everything = 1;
    return CELLS_OK;
}

void cells_destroy(cells_t *cells) {
    if (cells == NULL) return;
    free(cells->now);
    free(cells->before);
    free(cells->text);
    memset(cells, 0, sizeof(*cells));
}

cells_status_t cells_resize(cells_t *cells, int cols, int rows) {
    if (cells == NULL || cols <= 0 || rows <= 0) return CELLS_ERR_ARGUMENT;
    if (cols == cells->cols && rows == cells->rows) return CELLS_OK;
    size_t count = (size_t)cols * (size_t)rows;
    cell_t *now = calloc(count, sizeof(*now));
    cell_t *before = calloc(count, sizeof(*before));
    if (now == NULL || before == NULL) {
        free(now);
        free(before);
        return CELLS_ERR_MEMORY;
    }
    free(cells->now);
    free(cells->before);
    cells->now = now;
    cells->before = before;
    cells->cols = cols;
    cells->rows = rows;
    cells->draw_everything = 1;
    return CELLS_OK;
}

void cells_keep_out_of(cells_t *cells, int cols, int rows) {
    if (cells == NULL) return;
    if (cols != cells->keep_cols || rows != cells->keep_rows) cells->draw_everything = 1;
    cells->keep_cols = cols;
    cells->keep_rows = rows;
}

void cells_invalidate(cells_t *cells) {
    if (cells != NULL) cells->draw_everything = 1;
}

/* The ink in one rectangle of the canvas: how much of it there is, and what
 * colour it is. The colour is the one that fills most of the patch, not the
 * average: two birds of different shades sharing a cell used to average into a
 * colour that was neither, that changed a little every frame as they moved, and
 * that was different from every other cell's — ten thousand distinct colours a
 * recording, each costing a colour sequence. The sprites are flat tints, so the
 * colours in a patch are few and exact; a small table sorts them. */
enum { PATCH_COLOURS = 8 };

typedef struct {
    double coverage; /* 0 to 255, mean alpha. */
    uint8_t rgb[3];
} patch_t;

static patch_t read_patch(const png_image_t *canvas, int x0, int y0, int width, int height) {
    patch_t patch = {0, {0, 0, 0}};
    uint8_t colour[PATCH_COLOURS][3];
    double weight[PATCH_COLOURS] = {0};
    int colours = 0;
    double alpha_sum = 0, red = 0, green = 0, blue = 0;
    long counted = 0;
    for (int y = y0; y < y0 + height; y++) {
        if (y < 0 || y >= canvas->height) continue;
        for (int x = x0; x < x0 + width; x++) {
            if (x < 0 || x >= canvas->width) continue;
            const uint8_t *px =
                &canvas->pixels[((size_t)y * (size_t)canvas->width + (size_t)x) * 4];
            double a = px[3];
            counted++;
            alpha_sum += a;
            if (a == 0) continue;
            red += px[0] * a;
            green += px[1] * a;
            blue += px[2] * a;
            int slot = 0;
            while (slot < colours && memcmp(colour[slot], px, 3) != 0) slot++;
            if (slot == colours && colours < PATCH_COLOURS) memcpy(colour[colours++], px, 3);
            if (slot < colours) weight[slot] += a;
        }
    }
    if (counted == 0 || alpha_sum == 0) return patch;
    patch.coverage = alpha_sum / (double)counted;
    int best = -1;
    for (int slot = 0; slot < colours; slot++)
        if (best < 0 || weight[slot] > weight[best]) best = slot;
    if (best >= 0 && colours < PATCH_COLOURS) {
        memcpy(patch.rgb, colour[best], 3);
    } else {
        /* More colours than the table holds: a sprite of somebody's own, with
         * shading of its own. The mean is the honest answer for that. */
        patch.rgb[0] = (uint8_t)(red / alpha_sum + 0.5);
        patch.rgb[1] = (uint8_t)(green / alpha_sum + 0.5);
        patch.rgb[2] = (uint8_t)(blue / alpha_sum + 0.5);
    }
    return patch;
}

static void read_braille_cell(cell_t *cell, const png_image_t *canvas, int x0, int y0,
                              int cell_width, int cell_height) {
    /* Dots are the cell divided two by four; a cell narrower than two pixels or
     * shorter than four gets what it gets. */
    unsigned dots = 0;
    for (int row = 0; row < 4; row++)
        for (int column = 0; column < 2; column++) {
            int dx0 = x0 + column * cell_width / 2;
            int dx1 = x0 + (column + 1) * cell_width / 2;
            int dy0 = y0 + row * cell_height / 4;
            int dy1 = y0 + (row + 1) * cell_height / 4;
            if (dx1 <= dx0) dx1 = dx0 + 1;
            if (dy1 <= dy0) dy1 = dy0 + 1;
            patch_t dot = read_patch(canvas, dx0, dy0, dx1 - dx0, dy1 - dy0);
            if (dot.coverage >= INK_THRESHOLD) dots |= 1u << (column + row * 2);
        }
    memset(cell, 0, sizeof(*cell));
    if (dots == 0) return;
    /* The colour is the whole cell's, so two dots of one bird agree. */
    patch_t whole = read_patch(canvas, x0, y0, cell_width, cell_height);
    cell->glyph = cells_braille(dots);
    memcpy(cell->fg, whole.rgb, 3);
    cell->has_fg = 1;
}

static void read_block_cell(cell_t *cell, const png_image_t *canvas, int x0, int y0, int cell_width,
                            int cell_height) {
    int half = cell_height / 2;
    if (half < 1) half = 1;
    patch_t top = read_patch(canvas, x0, y0, cell_width, half);
    patch_t bottom = read_patch(canvas, x0, y0 + half, cell_width, cell_height - half);
    int top_ink = top.coverage >= INK_THRESHOLD, bottom_ink = bottom.coverage >= INK_THRESHOLD;
    memset(cell, 0, sizeof(*cell));
    if (!top_ink && !bottom_ink) return;
    /* Never paint the sky: an empty half is the terminal's own background, so
     * the upper half block is used when the top has ink and the lower when only
     * the bottom does, and the background colour is set only when both do. */
    if (top_ink) {
        cell->glyph = 0x2580; /* Upper half block. */
        memcpy(cell->fg, top.rgb, 3);
        cell->has_fg = 1;
        if (bottom_ink) {
            memcpy(cell->bg, bottom.rgb, 3);
            cell->has_bg = 1;
        }
    } else {
        cell->glyph = 0x2584; /* Lower half block. */
        memcpy(cell->fg, bottom.rgb, 3);
        cell->has_fg = 1;
    }
}

void cells_read(cells_t *cells, cells_style_t style, const png_image_t *canvas, int cell_width,
                int cell_height) {
    if (cells == NULL || canvas == NULL || canvas->pixels == NULL || cells->now == NULL) return;
    if (cell_width < 1) cell_width = 1;
    if (cell_height < 1) cell_height = 1;
    for (int row = 0; row < cells->rows; row++)
        for (int col = 0; col < cells->cols; col++) {
            cell_t *cell = &cells->now[(size_t)row * (size_t)cells->cols + (size_t)col];
            if (col < cells->keep_cols && row < cells->keep_rows) {
                memset(cell, 0, sizeof(*cell));
                continue;
            }
            int x0 = col * cell_width, y0 = row * cell_height;
            if (style == CELLS_BRAILLE)
                read_braille_cell(cell, canvas, x0, y0, cell_width, cell_height);
            else
                read_block_cell(cell, canvas, x0, y0, cell_width, cell_height);
        }
}

/* --- Painting --------------------------------------------------------------- */

static void paint_rect(png_image_t *out, int x, int y, int width, int height,
                       const uint8_t rgb[3]) {
    for (int yy = y; yy < y + height; yy++) {
        if (yy < 0 || yy >= out->height) continue;
        for (int xx = x; xx < x + width; xx++) {
            if (xx < 0 || xx >= out->width) continue;
            uint8_t *px = &out->pixels[((size_t)yy * (size_t)out->width + (size_t)xx) * 4];
            px[0] = rgb[0];
            px[1] = rgb[1];
            px[2] = rgb[2];
            px[3] = 255;
        }
    }
}

cells_status_t cells_paint(const cells_t *cells, cells_style_t style, png_image_t *out,
                           int cell_width, int cell_height, const uint8_t ground[3]) {
    if (cells == NULL || out == NULL || cells->before == NULL) return CELLS_ERR_ARGUMENT;
    if (cell_width < 2 || cell_height < 4) return CELLS_ERR_ARGUMENT;
    png_status_t allocated =
        png_image_alloc(out, cells->cols * cell_width, cells->rows * cell_height);
    if (allocated != PNG_OK) return CELLS_ERR_MEMORY;
    paint_rect(out, 0, 0, out->width, out->height, ground);

    int dot_w = cell_width / 2, dot_h = cell_height / 4;
    /* A dot is drawn a pixel in from its patch on every side, which is roughly
     * how a font draws one: round, with air between it and its neighbours. */
    int gap_w = dot_w > 2 ? 1 : 0, gap_h = dot_h > 2 ? 1 : 0;
    for (int row = 0; row < cells->rows; row++)
        for (int col = 0; col < cells->cols; col++) {
            const cell_t *cell = &cells->before[(size_t)row * (size_t)cells->cols + (size_t)col];
            if (cell->glyph == 0) continue;
            int x0 = col * cell_width, y0 = row * cell_height;
            if (style == CELLS_BRAILLE) {
                unsigned bits = cell->glyph - 0x2800;
                for (int r = 0; r < 4; r++)
                    for (int c = 0; c < 2; c++)
                        if (bits & BRAILLE_BIT[r][c])
                            paint_rect(out, x0 + c * dot_w + gap_w, y0 + r * dot_h + gap_h,
                                       dot_w - 2 * gap_w, dot_h - 2 * gap_h, cell->fg);
            } else {
                int half = cell_height / 2;
                if (cell->glyph == 0x2580) {
                    paint_rect(out, x0, y0, cell_width, half, cell->fg);
                    if (cell->has_bg)
                        paint_rect(out, x0, y0 + half, cell_width, cell_height - half, cell->bg);
                } else {
                    paint_rect(out, x0, y0 + half, cell_width, cell_height - half, cell->fg);
                }
            }
        }
    return CELLS_OK;
}

/* --- Emission --------------------------------------------------------------- */

static cells_status_t reserve(cells_t *cells, size_t extra) {
    size_t needed = cells->length + extra + 1;
    if (needed <= cells->capacity) return CELLS_OK;
    size_t capacity = cells->capacity ? cells->capacity : 8192;
    while (capacity < needed) capacity *= 2;
    char *grown = realloc(cells->text, capacity);
    if (grown == NULL) return CELLS_ERR_MEMORY;
    cells->text = grown;
    cells->capacity = capacity;
    return CELLS_OK;
}

static cells_status_t put(cells_t *cells, const char *bytes, size_t length) {
    cells_status_t status = reserve(cells, length);
    if (status != CELLS_OK) return status;
    memcpy(cells->text + cells->length, bytes, length);
    cells->length += length;
    cells->text[cells->length] = '\0';
    return CELLS_OK;
}

static cells_status_t put_format(cells_t *cells, const char *format, int a, int b, int c) {
    char scratch[48];
    int length = snprintf(scratch, sizeof(scratch), format, a, b, c);
    if (length < 0) return CELLS_ERR_ARGUMENT;
    return put(cells, scratch, (size_t)length);
}

static cells_status_t put_glyph(cells_t *cells, uint32_t glyph) {
    char utf8[4];
    size_t length;
    if (glyph == 0) glyph = ' ';
    if (glyph < 0x80) {
        utf8[0] = (char)glyph;
        length = 1;
    } else if (glyph < 0x800) {
        utf8[0] = (char)(0xC0 | (glyph >> 6));
        utf8[1] = (char)(0x80 | (glyph & 0x3F));
        length = 2;
    } else if (glyph < 0x10000) {
        utf8[0] = (char)(0xE0 | (glyph >> 12));
        utf8[1] = (char)(0x80 | ((glyph >> 6) & 0x3F));
        utf8[2] = (char)(0x80 | (glyph & 0x3F));
        length = 3;
    } else {
        utf8[0] = (char)(0xF0 | (glyph >> 18));
        utf8[1] = (char)(0x80 | ((glyph >> 12) & 0x3F));
        utf8[2] = (char)(0x80 | ((glyph >> 6) & 0x3F));
        utf8[3] = (char)(0x80 | (glyph & 0x3F));
        length = 4;
    }
    return put(cells, utf8, length);
}

/* The nearest entry of the 6x6x6 cube, for a terminal that has no 24 bit colour. */
static int cube_index(const uint8_t rgb[3]) {
    int levels[3];
    for (int c = 0; c < 3; c++) levels[c] = (rgb[c] + 25) / 51;
    return 16 + 36 * levels[0] + 6 * levels[1] + levels[2];
}

typedef struct {
    int has_fg, has_bg;
    uint8_t fg[3], bg[3];
} pen_t;

static cells_status_t set_colour(cells_t *cells, int background, const uint8_t rgb[3]) {
    int which = background ? 48 : 38;
    if (cells->truecolor) {
        char scratch[32];
        int length = snprintf(scratch, sizeof(scratch), "\033[%d;2;%d;%d;%dm", which, rgb[0],
                              rgb[1], rgb[2]);
        return put(cells, scratch, (size_t)length);
    }
    return put_format(cells, "\033[%d;5;%dm", which, cube_index(rgb), 0);
}

/* Brings the terminal's pen to what the cell wants, emitting as little as it
 * can: a reset only when something has to go back to default, a colour only
 * when it differs from the one already set. */
static cells_status_t dress(cells_t *cells, pen_t *pen, const cell_t *cell) {
    cells_status_t status = CELLS_OK;
    int drop_fg = pen->has_fg && !cell->has_fg, drop_bg = pen->has_bg && !cell->has_bg;
    if (drop_fg || drop_bg) {
        status = put(cells, "\033[0m", 4);
        pen->has_fg = pen->has_bg = 0;
    }
    if (status == CELLS_OK && cell->has_fg && (!pen->has_fg || memcmp(pen->fg, cell->fg, 3) != 0)) {
        status = set_colour(cells, 0, cell->fg);
        memcpy(pen->fg, cell->fg, 3);
        pen->has_fg = 1;
    }
    if (status == CELLS_OK && cell->has_bg && (!pen->has_bg || memcmp(pen->bg, cell->bg, 3) != 0)) {
        status = set_colour(cells, 1, cell->bg);
        memcpy(pen->bg, cell->bg, 3);
        pen->has_bg = 1;
    }
    return status;
}

static int same_cell(const cell_t *a, const cell_t *b) {
    if (a->glyph != b->glyph || a->has_fg != b->has_fg || a->has_bg != b->has_bg) return 0;
    if (a->has_fg && memcmp(a->fg, b->fg, 3) != 0) return 0;
    if (a->has_bg && memcmp(a->bg, b->bg, 3) != 0) return 0;
    return 1;
}

cells_status_t cells_emit(cells_t *cells) {
    if (cells == NULL || cells->now == NULL) return CELLS_ERR_ARGUMENT;
    cells->length = 0;
    if (cells->text != NULL) cells->text[0] = '\0';

    pen_t pen = {0, 0, {0, 0, 0}, {0, 0, 0}};
    cells_status_t status = CELLS_OK;
    int emitted = 0;
    for (int row = 0; row < cells->rows && status == CELLS_OK; row++) {
        int cursor_col = -1; /* Where the terminal's cursor is on this row, if known. */
        for (int col = 0; col < cells->cols && status == CELLS_OK; col++) {
            size_t at = (size_t)row * (size_t)cells->cols + (size_t)col;
            const cell_t *cell = &cells->now[at];
            if (col < cells->keep_cols && row < cells->keep_rows) {
                cursor_col = -1;
                continue;
            }
            if (!cells->draw_everything && same_cell(cell, &cells->before[at])) continue;
            /* Position only when the cursor is not already here: a run of changed
             * cells costs one move, not one per cell — and a short hop along the
             * same row is a cursor forward, four bytes, rather than an absolute
             * move at eight or nine. */
            if (cursor_col >= 0 && col > cursor_col && col - cursor_col < 100)
                status = put_format(cells, "\033[%dC", col - cursor_col, 0, 0);
            else if (cursor_col != col)
                status = put_format(cells, "\033[%d;%dH", row + 1, col + 1, 0);
            if (status == CELLS_OK) status = dress(cells, &pen, cell);
            if (status == CELLS_OK) status = put_glyph(cells, cell->glyph);
            cursor_col = col + 1;
            emitted++;
        }
    }
    if (status == CELLS_OK && (pen.has_fg || pen.has_bg || emitted > 0))
        status = put(cells, "\033[0m", 4);
    if (status != CELLS_OK) return status;

    /* This frame is now the one on the screen. */
    cell_t *swap = cells->before;
    cells->before = cells->now;
    cells->now = swap;
    cells->draw_everything = 0;
    return CELLS_OK;
}
