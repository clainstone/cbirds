#define _POSIX_C_SOURCE 200809L
#include "../sixel.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* A small sixel decoder, so the encoder is tested by what a terminal would make
 * of its output and not by string matching. Fills `out` with palette indices;
 * returns the number of colour registers defined. */
static int decode(const char *text, uint8_t *out, int width, int height, int fill) {
    memset(out, (unsigned char)fill, (size_t)width * (size_t)height);
    const char *p = strstr(text, "q");
    assert(p != NULL);
    p++;
    int colour = 0, x = 0, band = 0, registers = 0;
    long repeat = 1;
    while (*p != '\0') {
        if (*p == '\033') break;
        if (*p == '"') { /* Raster attributes: skip the four numbers. */
            p++;
            for (int n = 0; n < 4; n++) {
                while (*p >= '0' && *p <= '9') p++;
                if (*p == ';') p++;
            }
            continue;
        }
        if (*p == '#') {
            p++;
            colour = (int)strtol(p, (char **)&p, 10);
            if (*p == ';') { /* A definition, not a selection. */
                registers++;
                for (int n = 0; n < 4; n++) {
                    p++;
                    while (*p >= '0' && *p <= '9') p++;
                }
            }
            continue;
        }
        if (*p == '!') {
            p++;
            repeat = strtol(p, (char **)&p, 10);
            continue;
        }
        if (*p == '$') {
            x = 0;
            p++;
            continue;
        }
        if (*p == '-') {
            x = 0;
            band++;
            p++;
            continue;
        }
        assert(*p >= '?' && *p <= '~');
        int six = *p - '?';
        for (long r = 0; r < repeat; r++, x++)
            for (int dy = 0; dy < 6; dy++) {
                int y = band * 6 + dy;
                if ((six & (1 << dy)) && x < width && y < height)
                    out[y * width + x] = (uint8_t)colour;
            }
        repeat = 1;
        p++;
    }
    return registers;
}

static const uint8_t PALETTE[][3] = {
    {18, 18, 24},   /* 0: the ground */
    {255, 176, 66}, /* 1: an ember */
    {96, 226, 255}, /* 2: the hawk */
};

static png_image_t picture(int w, int h) {
    png_image_t image = {0, 0, NULL};
    assert(png_image_alloc(&image, w, h) == PNG_OK);
    for (int i = 0; i < w * h; i++) {
        memcpy(&image.pixels[i * 4], PALETTE[0], 3);
        image.pixels[i * 4 + 3] = 255;
    }
    return image;
}

static void dab(png_image_t *image, int x, int y, int w, int h, const uint8_t rgb[3]) {
    for (int yy = y; yy < y + h; yy++)
        for (int xx = x; xx < x + w; xx++)
            memcpy(&image->pixels[(yy * image->width + xx) * 4], rgb, 3);
}

static void test_round_trip(void) {
    sixel_t sixel;
    assert(sixel_init(&sixel) == SIXEL_OK);
    png_image_t image = picture(40, 14); /* Not a multiple of six: a ragged last band. */
    dab(&image, 3, 2, 5, 4, PALETTE[1]);
    dab(&image, 20, 9, 7, 5, PALETTE[2]);
    dab(&image, 39, 13, 1, 1, PALETTE[1]); /* The very last pixel. */
    assert(sixel_encode(&sixel, &image, PALETTE, 3) == SIXEL_OK);

    /* It is a DCS with the raster attributes and three colour registers. */
    assert(strncmp(sixel.text, "\033P0;1;0q\"1;1;40;14", 18) == 0);
    assert(strstr(sixel.text, "#1;2;100;69;25") != NULL); /* 255,176,66 in percent. */
    assert(sixel.text[sixel.length - 2] == '\033' && sixel.text[sixel.length - 1] == '\\');

    /* And decoded, every pixel is what went in. */
    uint8_t decoded[40 * 14];
    int registers = decode(sixel.text, decoded, 40, 14, 99);
    assert(registers == 3);
    for (int y = 0; y < 14; y++)
        for (int x = 0; x < 40; x++) {
            int expected = 0;
            if (x >= 3 && x < 8 && y >= 2 && y < 6) expected = 1;
            if (x >= 20 && x < 27 && y >= 9 && y < 14) expected = 2;
            if (x == 39 && y == 13) expected = 1;
            assert(decoded[y * 40 + x] == expected);
        }
    png_image_free(&image);
    sixel_destroy(&sixel);
}

static void test_off_palette_colours_snap_to_the_nearest(void) {
    sixel_t sixel;
    assert(sixel_init(&sixel) == SIXEL_OK);
    png_image_t image = picture(6, 6);
    static const uint8_t nearly_ember[3] = {240, 170, 80}, nearly_ground[3] = {30, 28, 40};
    dab(&image, 0, 0, 3, 6, nearly_ember);
    dab(&image, 3, 0, 3, 6, nearly_ground);
    assert(sixel_encode(&sixel, &image, PALETTE, 3) == SIXEL_OK);
    uint8_t decoded[36];
    decode(sixel.text, decoded, 6, 6, 99);
    assert(decoded[0] == 1 && decoded[5] == 0);
    /* A blend half way to the ground is nearer the ground than the tint. */
    png_image_free(&image);
    sixel_destroy(&sixel);
}

static void test_runs_are_counted_and_a_ground_band_is_cheap(void) {
    sixel_t sixel;
    assert(sixel_init(&sixel) == SIXEL_OK);
    png_image_t image = picture(800, 6); /* One band, all ground. */
    assert(sixel_encode(&sixel, &image, PALETTE, 3) == SIXEL_OK);
    /* Eight hundred ground columns are one counted run, not eight hundred glyphs. */
    assert(strstr(sixel.text, "#0!800~") != NULL);
    assert(sixel.length < 120);
    png_image_free(&image);
    sixel_destroy(&sixel);
}

static void test_the_memo_forgets_an_old_palette(void) {
    sixel_t sixel;
    assert(sixel_init(&sixel) == SIXEL_OK);
    png_image_t image = picture(6, 6);
    dab(&image, 0, 0, 6, 6, PALETTE[2]);
    assert(sixel_encode(&sixel, &image, PALETTE, 3) == SIXEL_OK);
    uint8_t decoded[36];
    decode(sixel.text, decoded, 6, 6, 99);
    assert(decoded[0] == 2);
    /* The same pixels against a palette in a different order. */
    static const uint8_t reordered[][3] = {{96, 226, 255}, {18, 18, 24}};
    assert(sixel_encode(&sixel, &image, reordered, 2) == SIXEL_OK);
    decode(sixel.text, decoded, 6, 6, 99);
    assert(decoded[0] == 0);
    png_image_free(&image);
    sixel_destroy(&sixel);
}

static void test_bad_arguments_are_refused(void) {
    sixel_t sixel;
    assert(sixel_init(&sixel) == SIXEL_OK);
    png_image_t image = picture(2, 2);
    assert(sixel_encode(NULL, &image, PALETTE, 3) == SIXEL_ERR_ARGUMENT);
    assert(sixel_encode(&sixel, &image, PALETTE, 0) == SIXEL_ERR_ARGUMENT);
    assert(sixel_encode(&sixel, &image, PALETTE, 257) == SIXEL_ERR_ARGUMENT);
    png_image_free(&image);
    sixel_destroy(&sixel);
}

int main(void) {
    test_round_trip();
    test_off_palette_colours_snap_to_the_nearest();
    test_runs_are_counted_and_a_ground_band_is_cheap();
    test_the_memo_forgets_an_old_palette();
    test_bad_arguments_are_refused();
    return 0;
}
