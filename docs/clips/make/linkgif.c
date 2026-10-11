/* linkgif DIR COLS ROWS FPS FROM TO OUT.gif [TITLE [STYLE]]: three terminals side by side,
 * as they were. DIR holds window-0.log to window-2.log, which capture_link.py
 * wrote: what each `cbirds --link` window sent its terminal, and when. Each is
 * played into cbirds' own terminal emulator up to the time of a frame, and its
 * screen is painted in braille dots, in a window with a title bar, on a desktop.
 * Nothing is drawn that the windows did not draw. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cells.h"
#include "font.h"
#include "gif.h"
#include "png.h"
#include "vt.h"

enum { WINDOWS = 3, CW = 8, CH = 16, TITLE = 26, GAP = 26, MARGIN = 22, RADIUS = 9 };

static const uint8_t DESKTOP[3] = {13, 17, 23};
static const uint8_t BAR[3] = {38, 40, 50};
static const uint8_t GROUND[3] = {18, 18, 24};
static const uint8_t TITLE_INK[3] = {150, 154, 170};
static const uint8_t LIGHTS[3][3] = {{255, 95, 86}, {255, 189, 46}, {39, 201, 63}};

typedef struct {
    double *at;
    unsigned char **bytes;
    size_t *length;
    size_t count, next;
    vt_t vt;
    cells_t cells;
} window_t;

static int hex(int c) {
    return c <= '9' ? c - '0' : (c | 32) - 'a' + 10;
}

static void load(window_t *w, const char *path) {
    FILE *f = fopen(path, "r");
    if (f == NULL) {
        perror(path);
        exit(1);
    }
    size_t capacity = 0;
    static char line[1 << 22];
    while (fgets(line, sizeof(line), f)) {
        char *space = strchr(line, ' ');
        if (space == NULL) continue;
        if (w->count == capacity) {
            capacity = capacity ? capacity * 2 : 1024;
            w->at = realloc(w->at, capacity * sizeof(*w->at));
            w->bytes = realloc(w->bytes, capacity * sizeof(*w->bytes));
            w->length = realloc(w->length, capacity * sizeof(*w->length));
        }
        w->at[w->count] = strtod(line, NULL);
        size_t n = strcspn(space + 1, "\n") / 2;
        unsigned char *b = malloc(n ? n : 1);
        for (size_t i = 0; i < n; i++) b[i] = (unsigned char)(hex(space[1 + 2 * i]) * 16 + hex(space[2 + 2 * i]));
        w->bytes[w->count] = b;
        w->length[w->count] = n;
        w->count++;
    }
    fclose(f);
}

static void fill(png_image_t *img, int x, int y, int w, int h, const uint8_t rgb[3]) {
    for (int yy = y; yy < y + h; yy++)
        for (int xx = x; xx < x + w; xx++) {
            if (xx < 0 || yy < 0 || xx >= img->width || yy >= img->height) continue;
            uint8_t *p = img->pixels + ((size_t)yy * (size_t)img->width + (size_t)xx) * 4;
            p[0] = rgb[0];
            p[1] = rgb[1];
            p[2] = rgb[2];
            p[3] = 255;
        }
}

static void disc(png_image_t *img, int cx, int cy, int r, const uint8_t rgb[3]) {
    for (int y = -r; y <= r; y++)
        for (int x = -r; x <= r; x++)
            if (x * x + y * y <= r * r) fill(img, cx + x, cy + y, 1, 1, rgb);
}

static void text(png_image_t *img, int x, int y, const char *s, const uint8_t rgb[3]) {
    for (; *s; s++, x += FONT_ADVANCE) {
        const char *g = font_glyph_for((unsigned char)*s);
        if (g == NULL) continue;
        for (int r = 0; r < FONT_HEIGHT; r++)
            for (int c = 0; c < FONT_WIDTH; c++)
                if (g[r * FONT_WIDTH + c] == '#') fill(img, x + c, y + r, 1, 1, rgb);
    }
}

/* The window's corners, rounded: the desktop is put back outside a quarter circle. */
static void round_corners(png_image_t *img, int x, int y, int w, int h) {
    for (int cy = 0; cy < RADIUS; cy++)
        for (int cx = 0; cx < RADIUS; cx++) {
            double dx = RADIUS - cx - 0.5, dy = RADIUS - cy - 0.5;
            if (dx * dx + dy * dy <= (double)RADIUS * RADIUS) continue;
            fill(img, x + cx, y + cy, 1, 1, DESKTOP);
            fill(img, x + w - 1 - cx, y + cy, 1, 1, DESKTOP);
            fill(img, x + cx, y + h - 1 - cy, 1, 1, DESKTOP);
            fill(img, x + w - 1 - cx, y + h - 1 - cy, 1, 1, DESKTOP);
        }
}

int main(int argc, char **argv) {
    if (argc < 8) {
        fprintf(stderr, "linkgif DIR COLS ROWS FPS FROM TO OUT.gif [TITLE]\n");
        return 2;
    }
    const char *dir = argv[1], *out = argv[7], *title = argc > 8 ? argv[8] : "cbirds --link";
    const char *style_name = argc > 9 ? argv[9] : "braille";
    cells_style_t style = strcmp(style_name, "sextants") == 0 ? CELLS_SEXTANTS
                          : strcmp(style_name, "blocks") == 0 ? CELLS_BLOCKS
                                                              : CELLS_BRAILLE;
    int cols = atoi(argv[2]), rows = atoi(argv[3]), fps = atoi(argv[4]);
    double from = atof(argv[5]), to = atof(argv[6]);
    window_t windows[WINDOWS];
    memset(windows, 0, sizeof(windows));
    for (int n = 0; n < WINDOWS; n++) {
        char path[1024];
        snprintf(path, sizeof(path), "%s/window-%d.log", dir, n);
        load(&windows[n], path);
        if (vt_init(&windows[n].vt, cols, rows) != 0 || cells_init(&windows[n].cells, 1) != CELLS_OK ||
            cells_resize(&windows[n].cells, cols, rows) != CELLS_OK)
            return 1;
    }
    int ww = cols * CW, wh = rows * CH + TITLE;
    int width = 2 * MARGIN + WINDOWS * ww + (WINDOWS - 1) * GAP, height = 2 * MARGIN + wh;
    png_image_t canvas = {0, 0, NULL};
    if (png_image_alloc(&canvas, width, height) != PNG_OK) return 1;
    gif_writer_t *gif = NULL;
    if (gif_open(&gif, out, width, height, 100 / fps) != GIF_OK) return 1;

    int frames = 0;
    for (double t = from; t < to; t += 1.0 / fps, frames++) {
        fill(&canvas, 0, 0, width, height, DESKTOP);
        for (int n = 0; n < WINDOWS; n++) {
            window_t *w = &windows[n];
            while (w->next < w->count && w->at[w->next] <= t) {
                vt_feed(&w->vt, w->bytes[w->next], w->length[w->next]);
                w->next++;
            }
            for (int r = 0; r < rows; r++)
                for (int c = 0; c < cols; c++) {
                    const vt_cell_t *v = vt_cell(&w->vt, c, r);
                    cell_t *cell = cells_at(&w->cells, c, r);
                    memset(cell, 0, sizeof(*cell));
                    if (style == CELLS_BRAILLE ? v->glyph < 0x2801 || v->glyph > 0x28FF
                                               : v->glyph <= ' ')
                        continue; /* Only what the renderer draws. */
                    cell->glyph = v->glyph;
                    cell->has_fg = 1;
                    cell->fg_kind = CELLS_COLOUR_RGB;
                    if (v->style.fg.kind == VT_COLOUR_RGB) {
                        memcpy(cell->fg, v->style.fg.value, 3);
                    } else {
                        cell->fg[0] = cell->fg[1] = cell->fg[2] = 220;
                    }
                }
            cells_emit(&w->cells);
            png_image_t screen = {0, 0, NULL};
            if (cells_paint(&w->cells, style, &screen, CW, CH, GROUND) != CELLS_OK) return 1;
            int x = MARGIN + n * (ww + GAP), y = MARGIN;
            fill(&canvas, x, y, ww, TITLE, BAR);
            for (int l = 0; l < 3; l++) disc(&canvas, x + 16 + l * 18, y + TITLE / 2, 5, LIGHTS[l]);
            int tw = (int)strlen(title) * FONT_ADVANCE;
            text(&canvas, x + (ww - tw) / 2, y + (TITLE - FONT_HEIGHT) / 2, title, TITLE_INK);
            for (int r = 0; r < screen.height; r++)
                memcpy(canvas.pixels + ((size_t)(y + TITLE + r) * (size_t)width + (size_t)x) * 4,
                       screen.pixels + (size_t)r * (size_t)screen.width * 4, (size_t)screen.width * 4);
            png_image_free(&screen);
            round_corners(&canvas, x, y, ww, wh);
        }
        if (gif_add_frame(gif, &canvas) != GIF_OK) return 1;
    }
    size_t written = 0;
    if (gif_close(gif, &written, NULL) != GIF_OK) return 1;
    printf("%s: %d frames, %dx%d, %.1f KB\n", out, frames, width, height, written / 1024.0);
    return 0;
}
