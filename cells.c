#include "cells.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "font.h"

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

/* Unicode 13 laid the sextants out by the value of their pattern, top left the
 * lowest bit, and left out the patterns that already existed as characters: the
 * left and right halves, nothing, and everything. */
uint32_t cells_sextant(unsigned blocks) {
    blocks &= 0x3F;
    if (blocks == 0) return ' ';
    if (blocks == 0x3F) return 0x2588; /* Full block. */
    if (blocks == 0x15) return 0x258C; /* Left half: rows one, two and three, left. */
    if (blocks == 0x2A) return 0x2590; /* Right half. */
    uint32_t code = 0x1FB00 + blocks - 1;
    if (blocks > 0x15) code--;
    if (blocks > 0x2A) code--;
    return code;
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

void cells_clear(cells_t *cells) {
    if (cells == NULL || cells->now == NULL) return;
    memset(cells->now, 0, (size_t)cells->cols * (size_t)cells->rows * sizeof(*cells->now));
}

cell_t *cells_at(cells_t *cells, int col, int row) {
    if (cells == NULL || cells->now == NULL || col < 0 || row < 0 || col >= cells->cols ||
        row >= cells->rows)
        return NULL;
    return &cells->now[(size_t)row * (size_t)cells->cols + (size_t)col];
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

static void read_sextant_cell(cell_t *cell, const png_image_t *canvas, int x0, int y0,
                              int cell_width, int cell_height) {
    unsigned blocks = 0;
    for (int row = 0; row < 3; row++)
        for (int column = 0; column < 2; column++) {
            int bx0 = x0 + column * cell_width / 2;
            int bx1 = x0 + (column + 1) * cell_width / 2;
            int by0 = y0 + row * cell_height / 3;
            int by1 = y0 + (row + 1) * cell_height / 3;
            if (bx1 <= bx0) bx1 = bx0 + 1;
            if (by1 <= by0) by1 = by0 + 1;
            patch_t block = read_patch(canvas, bx0, by0, bx1 - bx0, by1 - by0);
            if (block.coverage >= INK_THRESHOLD) blocks |= 1u << (column + row * 2);
        }
    memset(cell, 0, sizeof(*cell));
    if (blocks == 0) return;
    patch_t whole = read_patch(canvas, x0, y0, cell_width, cell_height);
    cell->glyph = cells_sextant(blocks);
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
    if (style == CELLS_TEXT) return; /* Text is put there by hand. */
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
            else if (style == CELLS_SEXTANTS)
                read_sextant_cell(cell, canvas, x0, y0, cell_width, cell_height);
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

/* --- Painting text ---------------------------------------------------------- */

/* The terminal's sixteen, for a picture, which has no theme to borrow them from.
 * Chosen to be read on the dark ground the pictures have: the stock VGA blue is
 * nearly the colour of that ground, so this is the palette editors ship for dark
 * themes. */
static const uint8_t PICTURE_ANSI[16][3] = {
    {0, 0, 0},       {205, 49, 49},   {13, 188, 121}, {229, 229, 16},
    {36, 114, 200},  {188, 63, 188},  {17, 168, 205}, {229, 229, 229},
    {102, 102, 102}, {241, 76, 76},   {35, 209, 139}, {245, 245, 67},
    {59, 142, 234},  {214, 112, 214}, {41, 184, 219}, {255, 255, 255},
};
static const uint8_t PICTURE_FOREGROUND[3] = {204, 204, 204};

static void text_colour(uint8_t kind, const uint8_t value[3], int bold, uint8_t out[3]) {
    if (kind == CELLS_COLOUR_ANSI || (kind == CELLS_COLOUR_INDEXED && value[0] < 16)) {
        /* Bold in the first eight is the bright colour, as most terminals draw it. */
        int index = value[0] & 15;
        if (bold && index < 8) index += 8;
        memcpy(out, PICTURE_ANSI[index], 3);
    } else if (kind == CELLS_COLOUR_INDEXED) {
        int index = value[0];
        if (index >= 232) {
            out[0] = out[1] = out[2] = (uint8_t)(8 + 10 * (index - 232));
        } else {
            static const uint8_t LEVEL[6] = {0, 95, 135, 175, 215, 255};
            index -= 16;
            out[0] = LEVEL[index / 36];
            out[1] = LEVEL[index / 6 % 6];
            out[2] = LEVEL[index % 6];
        }
    } else {
        memcpy(out, value, 3);
    }
}

static void mix_colour(const uint8_t a[3], const uint8_t b[3], int share_of_b, uint8_t out[3]) {
    for (int c = 0; c < 3; c++)
        out[c] = (uint8_t)((a[c] * (100 - share_of_b) + b[c] * share_of_b) / 100);
}

/* Which arms a box drawing character has: left 1, right 2, up 4, down 8, a
 * nibble for each of U+2500 to U+257F. Heavy and double lines are drawn as light
 * ones, and the diagonals (U+2571 to U+2573) are zero and drawn on their own. */
static const char BOX_ARMS[] =
    "33CC33CC33CCAAAA"
    "999966665555EEEE"
    "EEEEDDDDDDDDBBBB"
    "BBBB77777777FFFF"
    "FFFFFFFFFFFF33CC"
    "3CAAA999666555EE"
    "EDDDBBB777FFFA95"
    "6000142814283C3C";

static void paint_dither(png_image_t *out, int x, int y, int width, int height, int level,
                         const uint8_t rgb[3]) {
    for (int yy = y; yy < y + height; yy++)
        for (int xx = x; xx < x + width; xx++) {
            int on = level == 1
                         ? ((xx & 1) == 0 && (yy & 1) == 0)
                         : (level == 2 ? ((xx + yy) & 1) == 0 : !((xx & 1) == 0 && (yy & 1) == 0));
            if (on) paint_rect(out, xx, yy, 1, 1, rgb);
        }
}

static void paint_line(png_image_t *out, int x0, int y0, int x1, int y1, int thickness,
                       const uint8_t rgb[3]) {
    int steps = abs(x1 - x0) > abs(y1 - y0) ? abs(x1 - x0) : abs(y1 - y0);
    if (steps == 0) steps = 1;
    for (int i = 0; i <= steps; i++)
        paint_rect(out, x0 + (x1 - x0) * i / steps, y0 + (y1 - y0) * i / steps, thickness,
                   thickness, rgb);
}

/* A block element, a box drawing character or a braille pattern, drawn to the
 * whole cell as a terminal draws them, so that a logo made of █ and a tree made of
 * ├── meet at the edges instead of floating in the font's margins. Returns whether
 * it was one. */
static int paint_cell_graphic(png_image_t *out, int x, int y, int width, int height, uint32_t glyph,
                              const uint8_t rgb[3]) {
    if (glyph >= 0x2580 && glyph <= 0x259F) {
        int half_w = width / 2, half_h = height / 2;
        if (glyph == 0x2580) {
            paint_rect(out, x, y, width, half_h, rgb);
        } else if (glyph >= 0x2581 && glyph <= 0x2588) { /* Lower one eighth to the full block. */
            int rows = (int)(glyph - 0x2580) * height / 8;
            paint_rect(out, x, y + height - rows, width, rows, rgb);
        } else if (glyph >= 0x2589 && glyph <= 0x258F) { /* Left seven eighths to one. */
            int columns = (int)(0x2590 - glyph) * width / 8;
            paint_rect(out, x, y, columns, height, rgb);
        } else if (glyph == 0x2590) {
            paint_rect(out, x + half_w, y, width - half_w, height, rgb);
        } else if (glyph >= 0x2591 && glyph <= 0x2593) {
            paint_dither(out, x, y, width, height, (int)(glyph - 0x2590), rgb);
        } else if (glyph == 0x2594) {
            paint_rect(out, x, y, width, height / 8 > 0 ? height / 8 : 1, rgb);
        } else if (glyph == 0x2595) {
            int columns = width / 8 > 0 ? width / 8 : 1;
            paint_rect(out, x + width - columns, y, columns, height, rgb);
        } else {
            /* Quadrants: upper left 1, upper right 2, lower left 4, lower right 8. */
            static const uint8_t QUADRANTS[8] = {4, 8, 1, 13, 9, 7, 11, 2};
            unsigned bits = glyph - 0x2596 < 8 ? QUADRANTS[glyph - 0x2596]
                            : glyph == 0x259E  ? 6
                                               : 14;
            if (bits & 1) paint_rect(out, x, y, half_w, half_h, rgb);
            if (bits & 2) paint_rect(out, x + half_w, y, width - half_w, half_h, rgb);
            if (bits & 4) paint_rect(out, x, y + half_h, half_w, height - half_h, rgb);
            if (bits & 8)
                paint_rect(out, x + half_w, y + half_h, width - half_w, height - half_h, rgb);
        }
        return 1;
    }
    if (glyph >= 0x2500 && glyph <= 0x257F) {
        char hex = BOX_ARMS[glyph - 0x2500];
        unsigned arms = hex >= 'A' ? (unsigned)(hex - 'A' + 10) : (unsigned)(hex - '0');
        int thick = width / 6 > 0 ? width / 6 : 1;
        int cx = x + (width - thick) / 2, cy = y + (height - thick) / 2;
        if (glyph >= 0x2571 && glyph <= 0x2573) {
            if (glyph != 0x2572) paint_line(out, x, y + height - 1, x + width - 1, y, thick, rgb);
            if (glyph != 0x2571) paint_line(out, x, y, x + width - 1, y + height - 1, thick, rgb);
            return 1;
        }
        if (arms & 1) paint_rect(out, x, cy, cx - x + thick, thick, rgb);
        if (arms & 2) paint_rect(out, cx, cy, x + width - cx, thick, rgb);
        if (arms & 4) paint_rect(out, cx, y, thick, cy - y + thick, rgb);
        if (arms & 8) paint_rect(out, cx, cy, thick, y + height - cy, rgb);
        return 1;
    }
    if (glyph >= 0x2800 && glyph <= 0x28FF) {
        int dot_w = width / 2, dot_h = height / 4;
        int gap_w = dot_w > 2 ? 1 : 0, gap_h = dot_h > 2 ? 1 : 0;
        for (int r = 0; r < 4; r++)
            for (int c = 0; c < 2; c++)
                if ((glyph - 0x2800) & BRAILLE_BIT[r][c])
                    paint_rect(out, x + c * dot_w + gap_w, y + r * dot_h + gap_h, dot_w - 2 * gap_w,
                               dot_h - 2 * gap_h, rgb);
        return 1;
    }
    return 0;
}

static void paint_bitmap(png_image_t *out, int x, int y, int width, int height, const char *glyph,
                         int bold, const uint8_t rgb[3]) {
    int scale_x = width / (FONT_ADVANCE) > 0 ? width / FONT_ADVANCE : 1;
    int scale_y = height / (FONT_HEIGHT + 1) > 0 ? height / (FONT_HEIGHT + 1) : 1;
    int origin_x = x + (width - FONT_WIDTH * scale_x) / 2;
    int origin_y = y + (height - FONT_HEIGHT * scale_y) / 2;
    for (int row = 0; row < FONT_HEIGHT; row++)
        for (int column = 0; column < FONT_WIDTH; column++)
            if (glyph[row * FONT_WIDTH + column] == '#')
                paint_rect(out, origin_x + column * scale_x, origin_y + row * scale_y,
                           scale_x + (bold ? 1 : 0), scale_y, rgb);
}

/* A glyph the font has no picture of: a hollow box, so a gap in the text reads as
 * a missing character and not as a space. */
static void paint_missing(png_image_t *out, int x, int y, int width, int height,
                          const uint8_t rgb[3]) {
    int inset_x = width / 6 > 1 ? width / 6 : 1, inset_y = height / 6 > 1 ? height / 6 : 1;
    int w = width - 2 * inset_x, h = height - 2 * inset_y;
    if (w < 2 || h < 2) return;
    paint_rect(out, x + inset_x, y + inset_y, w, 1, rgb);
    paint_rect(out, x + inset_x, y + inset_y + h - 1, w, 1, rgb);
    paint_rect(out, x + inset_x, y + inset_y, 1, h, rgb);
    paint_rect(out, x + inset_x + w - 1, y + inset_y, 1, h, rgb);
}

static void paint_text_cell(png_image_t *out, const cell_t *cell, int x, int y, int cell_width,
                            int cell_height, const uint8_t ground[3]) {
    int width = cell->wide == CELLS_WIDE_HEAD ? cell_width * 2 : cell_width;
    int bold = (cell->attributes & CELLS_BOLD) != 0;
    uint8_t fg[3], bg[3];
    if (cell->has_fg)
        text_colour(cell->fg_kind, cell->fg, bold, fg);
    else
        memcpy(fg, PICTURE_FOREGROUND, 3);
    if (cell->has_bg)
        text_colour(cell->bg_kind, cell->bg, 0, bg);
    else
        memcpy(bg, ground, 3);
    if (cell->attributes & CELLS_REVERSE) {
        uint8_t swap[3];
        memcpy(swap, fg, 3);
        memcpy(fg, bg, 3);
        memcpy(bg, swap, 3);
    }
    if (cell->attributes & CELLS_DIM) {
        uint8_t faded[3];
        mix_colour(fg, bg, 50, faded);
        memcpy(fg, faded, 3);
    }
    if (memcmp(bg, ground, 3) != 0) paint_rect(out, x, y, width, cell_height, bg);

    uint32_t glyph = cell->glyph;
    if (glyph != 0 && glyph != ' ') {
        const char *bitmap = font_glyph_for(glyph);
        if (bitmap == NULL && font_plain_letter(glyph) != 0)
            bitmap = font_glyph_for(font_plain_letter(glyph));
        if (paint_cell_graphic(out, x, y, width, cell_height, glyph, fg)) {
            /* Drawn whole. */
        } else if (bitmap != NULL) {
            paint_bitmap(out, x, y, width, cell_height, bitmap, bold, fg);
        } else if (font_plain_letter(glyph) != ' ') {
            paint_missing(out, x, y, width, cell_height, fg);
        }
    }
    if (cell->attributes & CELLS_UNDERLINE) {
        int sy = cell_height / (FONT_HEIGHT + 1) > 0 ? cell_height / (FONT_HEIGHT + 1) : 1;
        int line = y + (cell_height - FONT_HEIGHT * sy) / 2 + FONT_HEIGHT * sy + 1;
        if (line >= y + cell_height) line = y + cell_height - 1;
        paint_rect(out, x, line, width, 1, fg);
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

    if (style == CELLS_TEXT) {
        for (int row = 0; row < cells->rows; row++)
            for (int col = 0; col < cells->cols; col++) {
                const cell_t *cell =
                    &cells->before[(size_t)row * (size_t)cells->cols + (size_t)col];
                if (cell->wide == CELLS_WIDE_TAIL) continue;
                paint_text_cell(out, cell, col * cell_width, row * cell_height, cell_width,
                                cell_height, ground);
            }
        return CELLS_OK;
    }

    int dot_w = cell_width / 2, dot_h = cell_height / 4;
    /* A dot is drawn a pixel in from its patch on every side, which is roughly
     * how a font draws one: round, with air between it and its neighbours. */
    int gap_w = dot_w > 2 ? 1 : 0, gap_h = dot_h > 2 ? 1 : 0;
    for (int row = 0; row < cells->rows; row++)
        for (int col = 0; col < cells->cols; col++) {
            const cell_t *cell = &cells->before[(size_t)row * (size_t)cells->cols + (size_t)col];
            if (cell->glyph == 0) continue;
            int x0 = col * cell_width, y0 = row * cell_height;
            if (style == CELLS_SEXTANTS) {
                /* Back from the code point to the pattern, the way it was made. */
                unsigned blocks = 0;
                for (unsigned candidate = 1; candidate < 0x40; candidate++)
                    if (cells_sextant(candidate) == cell->glyph) blocks = candidate;
                int third = cell_height / 3;
                for (int r = 0; r < 3; r++)
                    for (int c = 0; c < 2; c++)
                        if (blocks & (1u << (c + r * 2)))
                            paint_rect(out, x0 + c * dot_w, y0 + r * third, dot_w,
                                       r == 2 ? cell_height - 2 * third : third, cell->fg);
            } else if (style == CELLS_BRAILLE) {
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
    uint8_t fg_kind, bg_kind, attributes;
} pen_t;

/* Two colours are the same if they are written the same: a palette index is only
 * its first byte, and the rest of the three is whatever was lying there. */
static int same_colour(uint8_t kind_a, const uint8_t a[3], uint8_t kind_b, const uint8_t b[3]) {
    if (kind_a != kind_b) return 0;
    if (kind_a == CELLS_COLOUR_ANSI || kind_a == CELLS_COLOUR_INDEXED) return a[0] == b[0];
    return memcmp(a, b, 3) == 0;
}

static cells_status_t set_colour(cells_t *cells, int background, uint8_t kind,
                                 const uint8_t rgb[3]) {
    int which = background ? 48 : 38;
    if (kind == CELLS_COLOUR_ANSI) {
        /* The sixteen are SGR 30-37 and 90-97 (40-47 and 100-107 for a background):
         * the terminal's own, whatever theme it wears. */
        int code = (rgb[0] < 8 ? 30 : 90 - 8) + rgb[0] + (background ? 10 : 0);
        return put_format(cells, "\033[%dm", code, 0, 0);
    }
    if (kind == CELLS_COLOUR_INDEXED) return put_format(cells, "\033[%d;5;%dm", which, rgb[0], 0);
    if (cells->truecolor || kind == CELLS_COLOUR_EXACT) {
        char scratch[32];
        int length = snprintf(scratch, sizeof(scratch), "\033[%d;2;%d;%d;%dm", which, rgb[0],
                              rgb[1], rgb[2]);
        return put(cells, scratch, (size_t)length);
    }
    return put_format(cells, "\033[%d;5;%dm", which, cube_index(rgb), 0);
}

/* The attributes the pen lacks, in one sequence. */
static cells_status_t add_attributes(cells_t *cells, unsigned wanted) {
    static const struct {
        unsigned bit;
        char code;
    } SGR[] = {{CELLS_BOLD, '1'},
               {CELLS_DIM, '2'},
               {CELLS_ITALIC, '3'},
               {CELLS_UNDERLINE, '4'},
               {CELLS_REVERSE, '7'}};
    char scratch[16] = "\033[";
    size_t length = 2;
    for (size_t i = 0; i < sizeof(SGR) / sizeof(*SGR); i++) {
        if (!(wanted & SGR[i].bit)) continue;
        if (length > 2) scratch[length++] = ';';
        scratch[length++] = SGR[i].code;
    }
    scratch[length++] = 'm';
    return put(cells, scratch, length);
}

/* Brings the terminal's pen to what the cell wants, emitting as little as it
 * can: a reset only when something has to go back to default, a colour only
 * when it differs from the one already set. */
static cells_status_t dress(cells_t *cells, pen_t *pen, const cell_t *cell) {
    cells_status_t status = CELLS_OK;
    int drop_fg = pen->has_fg && !cell->has_fg, drop_bg = pen->has_bg && !cell->has_bg;
    int drop_attributes = (pen->attributes & ~cell->attributes) != 0;
    if (drop_fg || drop_bg || drop_attributes) {
        status = put(cells, "\033[0m", 4);
        pen->has_fg = pen->has_bg = 0;
        pen->attributes = 0;
    }
    if (status == CELLS_OK && (cell->attributes & ~pen->attributes)) {
        status = add_attributes(cells, (unsigned)(cell->attributes & ~pen->attributes));
        pen->attributes |= cell->attributes;
    }
    if (status == CELLS_OK && cell->has_fg &&
        (!pen->has_fg || !same_colour(pen->fg_kind, pen->fg, cell->fg_kind, cell->fg))) {
        status = set_colour(cells, 0, cell->fg_kind, cell->fg);
        memcpy(pen->fg, cell->fg, 3);
        pen->fg_kind = cell->fg_kind;
        pen->has_fg = 1;
    }
    if (status == CELLS_OK && cell->has_bg &&
        (!pen->has_bg || !same_colour(pen->bg_kind, pen->bg, cell->bg_kind, cell->bg))) {
        status = set_colour(cells, 1, cell->bg_kind, cell->bg);
        memcpy(pen->bg, cell->bg, 3);
        pen->bg_kind = cell->bg_kind;
        pen->has_bg = 1;
    }
    return status;
}

static int same_cell(const cell_t *a, const cell_t *b) {
    if (a->glyph != b->glyph || a->has_fg != b->has_fg || a->has_bg != b->has_bg) return 0;
    if (a->attributes != b->attributes || a->wide != b->wide) return 0;
    if (a->has_fg && !same_colour(a->fg_kind, a->fg, b->fg_kind, b->fg)) return 0;
    if (a->has_bg && !same_colour(a->bg_kind, a->bg, b->bg_kind, b->bg)) return 0;
    return 1;
}

cells_status_t cells_emit(cells_t *cells) {
    if (cells == NULL || cells->now == NULL) return CELLS_ERR_ARGUMENT;
    cells->length = 0;
    if (cells->text != NULL) cells->text[0] = '\0';

    pen_t pen = {0};
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
            /* The second cell of a wide glyph is never sent: its head covers it. */
            if (cell->wide == CELLS_WIDE_TAIL) continue;
            int changed = cells->draw_everything || !same_cell(cell, &cells->before[at]);
            /* A wide glyph is one thing: if either of its cells is new, it is drawn
             * again whole, since a terminal that finds half of one overwritten
             * shows damage rather than the other half. */
            if (!changed && cell->wide == CELLS_WIDE_HEAD && col + 1 < cells->cols)
                changed = !same_cell(&cells->now[at + 1], &cells->before[at + 1]);
            if (!changed) continue;
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
            cursor_col = col + (cell->wide == CELLS_WIDE_HEAD ? 2 : 1);
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
