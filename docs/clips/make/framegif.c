/* framegif IN.gif OUT.gif TITLE: the same frames in a terminal window, with a title
 * bar, three lights and rounded corners, on the dark of a GitHub page. The frames
 * themselves are copied pixel for pixel; the table is made from the first, which
 * in a recording of text is the text at rest, every colour it has. */
#define main gifpeek_main
#include "gifpeek.c"
#undef main

#include "gif.h"

enum { TITLE = 26, MARGIN = 14, RADIUS = 9 };
static const uint8_t DESKTOP[3] = {13, 17, 23};
static const uint8_t BAR[3] = {38, 40, 50};
static const uint8_t TITLE_INK[3] = {150, 154, 170};
static const uint8_t LIGHTS[3][3] = {{255, 95, 86}, {255, 189, 46}, {39, 201, 63}};

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
    if (argc < 4) {
        fprintf(stderr, "framegif IN.gif OUT.gif TITLE\n");
        return 2;
    }
    gif_t g;
    memset(&g, 0, sizeof(g));
    load_file(argv[1], &g);
    const char *fatal = scan_gif(&g);
    if (fatal != NULL) die(1, "%s", fatal);
    int ww = g.width, wh = g.height + TITLE;
    int width = ww + 2 * MARGIN, height = wh + 2 * MARGIN;
    png_image_t canvas = {0, 0, NULL};
    if (png_image_alloc(&canvas, width, height) != PNG_OK) return 1;
    gif_writer_t *out = NULL;
    if (gif_open(&out, argv[2], width, height, g.frames[0].delay) != GIF_OK) return 1;
    player_t p;
    player_init(&p, &g);
    for (int f = 0; f < g.count; f++) {
        if (!player_step(&p)) return 1;
        fill(&canvas, 0, 0, width, height, DESKTOP);
        fill(&canvas, MARGIN, MARGIN, ww, TITLE, BAR);
        for (int l = 0; l < 3; l++) disc(&canvas, MARGIN + 16 + l * 18, MARGIN + TITLE / 2, 5, LIGHTS[l]);
        int tw = (int)strlen(argv[3]) * FONT_ADVANCE;
        text(&canvas, MARGIN + (ww - tw) / 2, MARGIN + (TITLE - FONT_HEIGHT) / 2, argv[3], TITLE_INK);
        for (int y = 0; y < g.height; y++)
            for (int x = 0; x < g.width; x++) {
                const uint8_t *src = p.canvas + ((size_t)y * (size_t)g.width + (size_t)x) * 4;
                uint8_t *dst = canvas.pixels +
                               ((size_t)(MARGIN + TITLE + y) * (size_t)width + (size_t)(MARGIN + x)) * 4;
                memcpy(dst, src, 3);
                dst[3] = 255;
            }
        round_corners(&canvas, MARGIN, MARGIN, ww, wh);
        if (gif_add_frame(out, &canvas) != GIF_OK) return 1;
    }
    size_t written = 0;
    if (gif_close(out, &written, NULL) != GIF_OK) return 1;
    printf("%s: %d frames, %dx%d, %.1f KB\n", argv[2], g.count, width, height, written / 1024.0);
    player_free(&p);
    return 0;
}
