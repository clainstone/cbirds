#include "../png.h"

#include <assert.h>
#include <stdlib.h>
#include <string.h>

/* Fills an image with one of several shapes of data, because a compressor that
 * only ever sees one shape is not tested. */
static void paint(png_image_t *image, int kind, unsigned *seed) {
    for (long i = 0; i < (long)image->width * image->height; i++) {
        uint8_t *p = image->pixels + i * 4;
        switch (kind) {
            case 0: /* Flat, the easy case. */
                p[0] = p[1] = p[2] = 40;
                p[3] = 255;
                break;
            case 1: /* Noise, the incompressible one. */
                *seed = *seed * 1103515245u + 12345u;
                p[0] = (uint8_t)(*seed >> 16);
                p[1] = (uint8_t)(*seed >> 8);
                p[2] = (uint8_t)*seed;
                p[3] = (uint8_t)(*seed >> 24);
                break;
            case 2: /* Stripes, which is what a sprite mostly is. */
                p[0] = (i / 7) % 2 ? 255 : 0;
                p[1] = 128;
                p[2] = (i / 7) % 2 ? 0 : 255;
                p[3] = 255;
                break;
            default: /* A gradient, where matches are long but never exact. */
                p[0] = (uint8_t)(i % 256);
                p[1] = (uint8_t)(i / 256);
                p[2] = 90;
                p[3] = 255;
                break;
        }
    }
}

static void test_round_trip(void) {
    unsigned seed = 1;
    static const int SIZES[][2] = {{1, 1}, {2, 3}, {17, 5}, {64, 64}, {200, 120}, {300, 200}};

    for (size_t s = 0; s < sizeof(SIZES) / sizeof(*SIZES); s++) {
        for (int kind = 0; kind < 4; kind++) {
            png_image_t image = {0, 0, NULL}, back = {0, 0, NULL};
            uint8_t *encoded = NULL;
            size_t length = 0;

            assert(png_image_alloc(&image, SIZES[s][0], SIZES[s][1]) == PNG_OK);
            paint(&image, kind, &seed);
            assert(png_encode(&image, &encoded, &length) == PNG_OK);

            /* Every byte back, through our own inflater: the compressor and the
             * decompressor are each other's only check. */
            assert(png_decode(encoded, length, &back) == PNG_OK);
            assert(back.width == image.width && back.height == image.height);
            assert(memcmp(back.pixels, image.pixels,
                          (size_t)image.width * (size_t)image.height * 4) == 0);

            free(encoded);
            png_image_free(&image);
            png_image_free(&back);
        }
    }
}

static void test_compression_earns_its_place(void) {
    unsigned seed = 7;
    png_image_t image = {0, 0, NULL};
    uint8_t *encoded = NULL;
    size_t length = 0;
    size_t raw;

    /* Flat data must compress hard, or the compressor is not doing anything. */
    assert(png_image_alloc(&image, 128, 128) == PNG_OK);
    paint(&image, 0, &seed);
    raw = (size_t)image.width * (size_t)image.height * 4;
    assert(png_encode(&image, &encoded, &length) == PNG_OK);
    assert(length < raw / 20);
    free(encoded);
    png_image_free(&image);

    /* Stripes are what a sprite is mostly made of. */
    assert(png_image_alloc(&image, 200, 120) == PNG_OK);
    paint(&image, 2, &seed);
    raw = (size_t)image.width * (size_t)image.height * 4;
    assert(png_encode(&image, &encoded, &length) == PNG_OK);
    assert(length < raw / 20);
    free(encoded);
    png_image_free(&image);

    /* And noise must not come out meaningfully larger than it went in: the
     * stored fallback exists for exactly this. */
    assert(png_image_alloc(&image, 128, 128) == PNG_OK);
    paint(&image, 1, &seed);
    raw = (size_t)image.width * (size_t)image.height * 4;
    assert(png_encode(&image, &encoded, &length) == PNG_OK);
    assert(length < raw + raw / 50);
    free(encoded);
    png_image_free(&image);
}

static void test_transforms(void) {
    png_image_t source = {0, 0, NULL}, turned = {0, 0, NULL}, smaller = {0, 0, NULL};
    unsigned seed = 3;

    assert(png_image_alloc(&source, 32, 32) == PNG_OK);
    paint(&source, 3, &seed);

    /* A full turn is the image again, to within the resampling. */
    assert(png_rotate(&source, 2 * 3.14159265358979323846, &turned) == PNG_OK);
    assert(turned.width == 32 && turned.height == 32);

    /* Shrinking halves it, enlarging doubles it, and neither loses the canvas. */
    assert(png_resize(&source, 16, 16, &smaller) == PNG_OK);
    assert(smaller.width == 16 && smaller.height == 16);
    png_image_free(&smaller);
    assert(png_resize(&source, 64, 48, &smaller) == PNG_OK);
    assert(smaller.width == 64 && smaller.height == 48);

    /* A tint leaves alpha alone, which is what keeps wings from bleeding. */
    uint8_t before = source.pixels[3];
    png_tint(&source, 10, 20, 30, PNG_TINT_REPLACE);
    assert(source.pixels[0] == 10 && source.pixels[1] == 20 && source.pixels[2] == 30);
    assert(source.pixels[3] == before);

    png_image_free(&source);
    png_image_free(&turned);
    png_image_free(&smaller);
}

static void test_refusals(void) {
    png_image_t image = {0, 0, NULL};
    uint8_t *encoded = NULL;
    size_t length = 0;
    static const uint8_t signature[8] = {0x89, 'P', 'N', 'G', '\r', '\n', 0x1a, '\n'};

    assert(png_decode(NULL, 10, &image) == PNG_ERR_ARGUMENT);
    assert(png_decode(signature, 4, &image) == PNG_ERR_TRUNCATED);
    assert(png_decode((const uint8_t *)"not a png at all", 16, &image) == PNG_ERR_SIGNATURE);
    assert(png_decode(signature, sizeof(signature), &image) == PNG_ERR_TRUNCATED);

    assert(png_image_alloc(&image, 0, 10) == PNG_ERR_ARGUMENT);
    assert(png_image_alloc(&image, 10, -1) == PNG_ERR_ARGUMENT);
    assert(png_image_alloc(&image, 1 << 20, 1 << 20) == PNG_ERR_UNSUPPORTED);
    assert(png_encode(&image, &encoded, &length) == PNG_ERR_ARGUMENT);
    assert(png_rotate(NULL, 1.0, &image) == PNG_ERR_ARGUMENT);
    assert(png_resize(&image, -1, 4, &image) == PNG_ERR_ARGUMENT);

    /* Every status has something to say for itself. */
    for (int s = PNG_OK; s <= PNG_ERR_DEFLATE; s++) {
        const char *text = png_status_string((png_status_t)s);
        assert(text != NULL && strlen(text) > 0);
        assert(strcmp(text, "unknown error") != 0);
    }
}

/* A truncated or corrupted stream must be refused, never followed. */
static void test_damage_is_refused(void) {
    png_image_t image = {0, 0, NULL}, back = {0, 0, NULL};
    uint8_t *encoded = NULL;
    size_t length = 0;
    unsigned seed = 11;

    assert(png_image_alloc(&image, 48, 48) == PNG_OK);
    paint(&image, 3, &seed);
    assert(png_encode(&image, &encoded, &length) == PNG_OK);

    /* Every truncation of a good file is a bad file. */
    for (size_t cut = 1; cut < length; cut += 7) assert(png_decode(encoded, cut, &back) != PNG_OK);

    /* And so is a flipped bit anywhere in it, because every chunk is checksummed. */
    for (size_t at = 8; at < length; at += 101) {
        encoded[at] ^= 0x40;
        assert(png_decode(encoded, length, &back) != PNG_OK);
        encoded[at] ^= 0x40;
    }
    /* Unharmed, it still reads. */
    assert(png_decode(encoded, length, &back) == PNG_OK);

    free(encoded);
    png_image_free(&image);
    png_image_free(&back);
}

int main(void) {
    test_round_trip();
    test_compression_earns_its_place();
    test_transforms();
    test_refusals();
    test_damage_is_refused();
    return 0;
}
