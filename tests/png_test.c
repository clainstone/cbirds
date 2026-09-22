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

/* Everything below reads files built here byte by byte: scanlines packed and
 * filtered, stored deflate blocks, the zlib header and Adler checksum, the chunk
 * CRCs. None of it goes through png.c, so the two cannot share a mistake. */

typedef struct {
    uint8_t *data;
    size_t length, capacity;
} bytes_t;

static void put_byte(bytes_t *b, unsigned value) {
    if (b->length == b->capacity) {
        b->capacity = b->capacity ? b->capacity * 2 : 256;
        b->data = (uint8_t *)realloc(b->data, b->capacity);
        assert(b->data != NULL);
    }
    b->data[b->length++] = (uint8_t)value;
}

static void put_bytes(bytes_t *b, const uint8_t *data, size_t length) {
    for (size_t i = 0; i < length; i++) put_byte(b, data[i]);
}

static void put_be32(bytes_t *b, uint32_t value) {
    for (int shift = 24; shift >= 0; shift -= 8) put_byte(b, (value >> shift) & 0xff);
}

/* Bit by bit, without a table: slow, and plainly the one in the specification. */
static uint32_t crc_of(const uint8_t *data, size_t length) {
    uint32_t c = 0xffffffffu;
    for (size_t i = 0; i < length; i++) {
        c ^= data[i];
        for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xedb88320u & (0u - (c & 1u)));
    }
    return ~c;
}

static uint32_t adler_of(const uint8_t *data, size_t length) {
    uint32_t a = 1, b = 0;
    for (size_t i = 0; i < length; i++) {
        a = (a + data[i]) % 65521u;
        b = (b + a) % 65521u;
    }
    return (b << 16) | a;
}

static void put_chunk(bytes_t *png, const char *type, const uint8_t *body, size_t length) {
    put_be32(png, (uint32_t)length);
    size_t start = png->length;
    put_bytes(png, (const uint8_t *)type, 4);
    put_bytes(png, body, length);
    put_be32(png, crc_of(png->data + start, length + 4));
}

/* Signature and IHDR, whatever they say: what follows is up to the caller. */
static void put_header(bytes_t *png, int width, int height, int depth, int color_type,
                       int interlace) {
    static const uint8_t signature[8] = {0x89, 'P', 'N', 'G', '\r', '\n', 0x1a, '\n'};
    uint8_t ihdr[13] = {0};

    for (int i = 0; i < 4; i++) {
        ihdr[i] = (uint8_t)((unsigned)width >> (24 - 8 * i));
        ihdr[4 + i] = (uint8_t)((unsigned)height >> (24 - 8 * i));
    }
    ihdr[8] = (uint8_t)depth;
    ihdr[9] = (uint8_t)color_type;
    ihdr[12] = (uint8_t)interlace;
    put_bytes(png, signature, sizeof(signature));
    put_chunk(png, "IHDR", ihdr, sizeof(ihdr));
}

/* A zlib stream of stored blocks: no compression, every byte where it can be seen. */
static void put_zlib_stored(bytes_t *zlib, const uint8_t *raw, size_t length) {
    size_t at = 0;

    put_byte(zlib, 0x78);
    put_byte(zlib, 0x01);
    do {
        size_t n = length - at > 65535 ? 65535 : length - at;
        put_byte(zlib, at + n == length); /* Final block or not. */
        put_byte(zlib, (unsigned)(n & 0xff));
        put_byte(zlib, (unsigned)(n >> 8));
        put_byte(zlib, (unsigned)(~n & 0xff));
        put_byte(zlib, (unsigned)((~n >> 8) & 0xff));
        put_bytes(zlib, raw + at, n);
        at += n;
    } while (at < length);
    put_be32(zlib, adler_of(raw, length));
}

/* An image as the file holds it: samples at their own depth, before anything is
 * brought to 8 bits. */
typedef struct {
    int width, height, depth, color_type, interlaced;
    const uint8_t *palette; /* The PLTE body, or NULL for none. */
    size_t palette_length;
    const uint8_t *trns; /* The tRNS body, or NULL for none. */
    size_t trns_length;
    unsigned *samples; /* width * height * channels, row major. */
} picture_t;

static int channels_of(int color_type) {
    return color_type == 2 ? 3 : color_type == 4 ? 2 : color_type == 6 ? 4 : 1;
}

/* Samples below `limit`, with both ends of the range in. */
static unsigned *make_samples(const picture_t *pic, unsigned limit, unsigned *seed) {
    size_t count = (size_t)pic->width * (size_t)pic->height * (size_t)channels_of(pic->color_type);
    unsigned *samples = (unsigned *)malloc(count * sizeof(*samples));

    assert(samples != NULL);
    for (size_t i = 0; i < count; i++) {
        *seed = *seed * 1103515245u + 12345u;
        samples[i] = (*seed >> 8) % limit;
    }
    samples[0] = 0;
    samples[count - 1] = limit - 1;
    return samples;
}

/* Adam7, from the specification: first column, first row, and the two steps. */
static const int ADAM7[7][4] = {{0, 0, 8, 8}, {4, 0, 8, 8}, {0, 4, 4, 8}, {2, 0, 4, 4},
                                {0, 2, 2, 4}, {1, 0, 2, 2}, {0, 1, 1, 2}};
static const int NOT_INTERLACED[1][4] = {{0, 0, 1, 1}};

static int predict(int filter, int a, int b, int c) {
    int p = a + b - c, pa = abs(p - a), pb = abs(p - b), pc = abs(p - c);

    switch (filter) {
        case 1:
            return a;
        case 2:
            return b;
        case 3:
            return (a + b) / 2;
        case 4:
            return pa <= pb && pa <= pc ? a : pb <= pc ? b : c;
        default:
            return 0;
    }
}

/* The raw scanlines, pass by pass. Row y of pass p takes filter (y + p) % 5, so
 * every filter meets every depth, sub byte and 16 bit ones included, and the
 * filters look back one pixel, or one byte when pixels are smaller. */
static void put_scanlines(bytes_t *raw, const picture_t *pic) {
    int channels = channels_of(pic->color_type);
    int bits = channels * pic->depth;
    size_t unit = bits >= 8 ? (size_t)bits / 8 : 1;
    const int(*passes)[4] = pic->interlaced ? ADAM7 : NOT_INTERLACED;

    for (int p = 0; p < (pic->interlaced ? 7 : 1); p++) {
        int x0 = passes[p][0], y0 = passes[p][1], dx = passes[p][2], dy = passes[p][3];
        int width = pic->width > x0 ? (pic->width - x0 + dx - 1) / dx : 0;
        int height = pic->height > y0 ? (pic->height - y0 + dy - 1) / dy : 0;
        if (width == 0 || height == 0) continue; /* Not even a filter byte. */

        size_t stride = ((size_t)width * (size_t)bits + 7) / 8;
        uint8_t *line = (uint8_t *)calloc(stride, 1), *above = (uint8_t *)calloc(stride, 1);
        assert(line != NULL && above != NULL);
        for (int y = 0; y < height; y++) {
            memset(line, 0, stride);
            for (int x = 0; x < width; x++) {
                size_t pixel = (size_t)(y0 + y * dy) * (size_t)pic->width + (size_t)(x0 + x * dx);
                for (int c = 0; c < channels; c++) {
                    unsigned value = pic->samples[pixel * (size_t)channels + (size_t)c];
                    size_t index = (size_t)x * (size_t)channels + (size_t)c;
                    if (pic->depth == 16) {
                        line[index * 2] = (uint8_t)(value >> 8);
                        line[index * 2 + 1] = (uint8_t)value;
                    } else if (pic->depth == 8) {
                        line[index] = (uint8_t)value;
                    } else { /* Packed from the most significant bit. */
                        size_t bit = index * (size_t)pic->depth;
                        line[bit / 8] |= (uint8_t)(value << (8 - (size_t)pic->depth - bit % 8));
                    }
                }
            }
            int filter = (y + p) % 5;
            put_byte(raw, (unsigned)filter);
            for (size_t i = 0; i < stride; i++) {
                int a = i >= unit ? line[i - unit] : 0;
                int c = i >= unit ? above[i - unit] : 0; /* The row above starts as zeros. */
                put_byte(raw, (unsigned)(line[i] - predict(filter, a, above[i], c)) & 0xff);
            }
            memcpy(above, line, stride);
        }
        free(line);
        free(above);
    }
}

/* The zlib stream of the picture, its raster longer or shorter by `extra` bytes. */
static void put_picture_zlib(bytes_t *zlib, const picture_t *pic, int extra) {
    bytes_t raw = {NULL, 0, 0};

    put_scanlines(&raw, pic);
    if (extra < 0) raw.length -= (size_t)-extra;
    for (int i = 0; i < extra; i++) put_byte(&raw, 0);
    put_zlib_stored(zlib, raw.data, raw.length);
    free(raw.data);
}

/* The file, its chunks in the given order: X a text chunk the decoder must skip,
 * P PLTE, T tRNS, D the image data split over two IDAT, as encoders may do. IEND
 * closes it whatever the order. */
static void put_picture(bytes_t *png, const picture_t *pic, int extra, const char *order) {
    bytes_t zlib = {NULL, 0, 0};

    put_picture_zlib(&zlib, pic, extra);
    put_header(png, pic->width, pic->height, pic->depth, pic->color_type, pic->interlaced);
    for (const char *o = order; *o; o++) {
        if (*o == 'X') put_chunk(png, "tEXt", (const uint8_t *)"Comment\0by hand", 15);
        if (*o == 'P' && pic->palette) put_chunk(png, "PLTE", pic->palette, pic->palette_length);
        if (*o == 'T' && pic->trns) put_chunk(png, "tRNS", pic->trns, pic->trns_length);
        if (*o == 'D') {
            put_chunk(png, "IDAT", zlib.data, zlib.length / 2);
            put_chunk(png, "IDAT", zlib.data + zlib.length / 2, zlib.length - zlib.length / 2);
        }
    }
    put_chunk(png, "IEND", NULL, 0);
    free(zlib.data);
}

static png_status_t decode_picture(const picture_t *pic, int extra, const char *order,
                                   png_image_t *image) {
    bytes_t png = {NULL, 0, 0};

    put_picture(&png, pic, extra, order);
    png_status_t status = png_decode(png.data, png.length, image);
    free(png.data);
    return status;
}

/* What the decoder must give back, from the specification alone: small samples
 * stretched to 0..255, 16 bit ones cut to their high byte, a tRNS key matched at
 * the full depth, and palette indices past PLTE transparent black. */
static void expected_rgba(const picture_t *pic, uint8_t *rgba) {
    int channels = channels_of(pic->color_type);
    unsigned top = (1u << pic->depth) - 1u, key[3] = {0, 0, 0};
    int keyed = pic->trns != NULL && (pic->color_type == 0 || pic->color_type == 2);

    for (int c = 0; keyed && c < channels; c++)
        key[c] = ((unsigned)pic->trns[2 * c] << 8) | pic->trns[2 * c + 1];

    for (size_t i = 0; i < (size_t)pic->width * (size_t)pic->height; i++) {
        const unsigned *s = pic->samples + i * (size_t)channels;
        uint8_t *q = rgba + i * 4, v[4] = {0, 0, 0, 0};

        for (int c = 0; c < channels; c++)
            v[c] = (uint8_t)(pic->depth == 16 ? s[c] >> 8 : s[c] * 255u / top);
        switch (pic->color_type) {
            case 0:
                q[0] = q[1] = q[2] = v[0];
                q[3] = keyed && s[0] == key[0] ? 0 : 255;
                break;
            case 2:
                memcpy(q, v, 3);
                q[3] = keyed && s[0] == key[0] && s[1] == key[1] && s[2] == key[2] ? 0 : 255;
                break;
            case 3:
                memset(q, 0, 4);
                if (s[0] < pic->palette_length / 3) {
                    memcpy(q, pic->palette + 3 * s[0], 3);
                    q[3] = pic->trns != NULL && s[0] < pic->trns_length ? pic->trns[s[0]] : 255;
                }
                break;
            case 4:
                q[0] = q[1] = q[2] = v[0];
                q[3] = v[1];
                break;
            default:
                memcpy(q, v, 4);
                break;
        }
    }
}

/* Decodes, and must match the expected pixels exactly. */
static void assert_decodes(const picture_t *pic) {
    size_t size = (size_t)pic->width * (size_t)pic->height * 4;
    uint8_t *want = (uint8_t *)malloc(size);
    png_image_t image = {0, 0, NULL};

    assert(want != NULL);
    expected_rgba(pic, want);
    assert(decode_picture(pic, 0, "XPTD", &image) == PNG_OK);
    assert(image.width == pic->width && image.height == pic->height);
    assert(memcmp(image.pixels, want, size) == 0);
    free(want);
    png_image_free(&image);
}

/* The header says how long the raster is. A stream that inflates to more is a
 * decompression bomb, and must be stopped at the first byte too many rather than
 * inflated and judged afterwards. */
static void test_bombs_are_refused(void) {
    png_image_t image = {0, 0, NULL};
    bytes_t zlib = {NULL, 0, 0}, png = {NULL, 0, 0};
    size_t size = (size_t)4 << 20;
    uint8_t *zeros = (uint8_t *)calloc(size, 1);

    /* A 1x1 RGBA image is five bytes of raster; these are four megabytes. */
    assert(zeros != NULL);
    put_zlib_stored(&zlib, zeros, size);
    put_header(&png, 1, 1, 8, 6, 0);
    put_chunk(&png, "IDAT", zlib.data, zlib.length);
    put_chunk(&png, "IEND", NULL, 0);
    assert(png_decode(png.data, png.length, &image) == PNG_ERR_DEFLATE);
    assert(image.pixels == NULL);
    free(zeros);

    /* The real thing, fixed Huffman codes: a literal zero, then matches of 258
     * bytes at distance 1, thirteen bits each. About 420 KB that inflate to
     * 64 MB, and a correct Adler checksum, so the size is the only fault. */
    size_t produced = 1, target = (size_t)64 << 20;
    uint32_t bits = 0;
    int count = 0;
    zlib.length = png.length = 0;
    put_byte(&zlib, 0x78);
    put_byte(&zlib, 0x01);
    bits = 1 | 1 << 1; /* Final block, fixed codes. */
    count = 3;
    bits |= 0x0cu << count; /* Literal 0 is 00110000, sent from its first bit. */
    count += 8;
    for (; produced + 258 <= target; produced += 258) {
        bits |= 0xa3u << count; /* Length 258 is 11000101, reversed; distance 1 is five zeros. */
        count += 13;
        while (count >= 8) {
            put_byte(&zlib, bits & 0xff);
            bits >>= 8;
            count -= 8;
        }
    }
    count += 7; /* End of block is seven zeros. */
    while (count > 0) {
        put_byte(&zlib, bits & 0xff);
        bits >>= 8;
        count -= 8;
    }
    put_be32(&zlib, (uint32_t)(produced % 65521) << 16 | 1); /* Adler of zeros. */
    assert(zlib.length < 450000);
    put_header(&png, 1, 1, 8, 6, 0);
    put_chunk(&png, "IDAT", zlib.data, zlib.length);
    put_chunk(&png, "IEND", NULL, 0);
    assert(png_decode(png.data, png.length, &image) == PNG_ERR_DEFLATE);
    assert(image.pixels == NULL);

    free(zlib.data);
    free(png.data);
}

/* One byte short is refused, and so is one byte over: the raster is exact. */
static void test_raster_length_is_exact(void) {
    unsigned seed = 5;

    for (int interlaced = 0; interlaced < 2; interlaced++) {
        picture_t pic = {3, 2, 8, 2, interlaced, NULL, 0, NULL, 0, NULL};
        png_image_t image = {0, 0, NULL};

        pic.samples = make_samples(&pic, 256, &seed);
        assert(decode_picture(&pic, -1, "D", &image) == PNG_ERR_TRUNCATED);
        assert(decode_picture(&pic, 1, "D", &image) == PNG_ERR_DEFLATE);
        assert(image.pixels == NULL);
        assert_decodes(&pic);
        free(pic.samples);
    }
}

/* What every optimizer writes: an index per pixel, often under 8 bits, often
 * with fewer colors than the depth allows, often with some of them see through. */
static void test_palettes(void) {
    uint8_t palette[257 * 3], alpha[256];
    unsigned seed = 21;
    /* Width, height, depth, PLTE entries, tRNS entries. Odd widths end rows in the
     * middle of a byte. */
    static const int CASES[][5] = {{13, 6, 1, 2, 1},   {7, 5, 2, 3, 2},     {5, 7, 4, 11, 6},
                                   {9, 6, 8, 200, 17}, {9, 6, 8, 256, 256}, {10, 5, 8, 7, 0},
                                   {17, 5, 4, 16, 16}, {3, 5, 1, 2, 0}};

    for (int i = 0; i < 257 * 3; i++) palette[i] = (uint8_t)(i * 37 + 11);
    for (int i = 0; i < 256; i++) alpha[i] = (uint8_t)(i * 53 + 7);

    for (size_t k = 0; k < sizeof(CASES) / sizeof(*CASES); k++) {
        for (int interlaced = 0; interlaced < 2; interlaced++) {
            const int *c = CASES[k];
            picture_t pic = {c[0], c[1], c[2], 3, interlaced, palette, 0, NULL, 0, NULL};
            pic.palette_length = (size_t)c[3] * 3;
            pic.trns = c[4] ? alpha : NULL;
            pic.trns_length = (size_t)c[4];
            pic.samples = make_samples(&pic, (unsigned)c[3], &seed);
            assert_decodes(&pic);
            free(pic.samples);
        }
    }

    /* An index past PLTE is an error in the file: it reads as transparent black. */
    picture_t pic = {5, 7, 4, 3, 0, palette, 5 * 3, alpha, 2, NULL};
    png_image_t image = {0, 0, NULL};
    pic.samples = make_samples(&pic, 16, &seed);
    pic.samples[1] = 15;
    pic.samples[2] = 4;
    assert_decodes(&pic);
    assert(decode_picture(&pic, 0, "PTD", &image) == PNG_OK);
    static const uint8_t HOLE[4] = {0, 0, 0, 0};
    assert(memcmp(image.pixels + 4, HOLE, 4) == 0);
    assert(memcmp(image.pixels + 8, palette + 12, 3) == 0 && image.pixels[11] == 255);
    png_image_free(&image);

    /* No palette, an empty one, one not made of triples, one too long. */
    pic.palette = NULL;
    assert(decode_picture(&pic, 0, "PD", &image) == PNG_ERR_CHUNK);
    pic.palette = palette;
    pic.palette_length = 0;
    assert(decode_picture(&pic, 0, "PD", &image) == PNG_ERR_CHUNK);
    pic.palette_length = 16;
    assert(decode_picture(&pic, 0, "PD", &image) == PNG_ERR_CHUNK);
    pic.palette_length = 257 * 3;
    assert(decode_picture(&pic, 0, "PD", &image) == PNG_ERR_CHUNK);

    /* More alphas than colors, and chunks out of order: PLTE after the data, tRNS
     * before PLTE or after the data, either one twice. */
    pic.palette_length = 5 * 3;
    pic.trns_length = 6;
    assert(decode_picture(&pic, 0, "PTD", &image) == PNG_ERR_CHUNK);
    pic.trns_length = 5;
    assert(decode_picture(&pic, 0, "PTD", &image) == PNG_OK);
    png_image_free(&image);
    assert(decode_picture(&pic, 0, "DP", &image) == PNG_ERR_CHUNK);
    assert(decode_picture(&pic, 0, "PDP", &image) == PNG_ERR_CHUNK);
    assert(decode_picture(&pic, 0, "TPD", &image) == PNG_ERR_CHUNK);
    assert(decode_picture(&pic, 0, "PDT", &image) == PNG_ERR_CHUNK);
    assert(decode_picture(&pic, 0, "PPD", &image) == PNG_ERR_CHUNK);
    assert(decode_picture(&pic, 0, "PTTD", &image) == PNG_ERR_CHUNK);
    assert(image.pixels == NULL);
    free(pic.samples);
}

static void test_gray(void) {
    unsigned seed = 33;
    static const int DEPTHS[] = {1, 2, 4, 8, 16};
    png_image_t image = {0, 0, NULL};

    for (size_t d = 0; d < sizeof(DEPTHS) / sizeof(*DEPTHS); d++) {
        for (int interlaced = 0; interlaced < 2; interlaced++) {
            picture_t pic = {11, 6, DEPTHS[d], 0, interlaced, NULL, 0, NULL, 0, NULL};
            pic.samples = make_samples(&pic, 1u << DEPTHS[d], &seed);
            assert_decodes(&pic);
            free(pic.samples);
        }
    }

    /* A key at 2 bits: every pixel of value 2 is transparent. */
    static const uint8_t KEY2[2] = {0, 2};
    picture_t pic = {9, 5, 2, 0, 0, NULL, 0, KEY2, 2, NULL};
    pic.samples = make_samples(&pic, 4, &seed);
    assert_decodes(&pic);
    free(pic.samples);

    /* A key at 16 bits is matched at 16 bits: 0x12ff reads as the same gray as
     * the key 0x1234, and stays opaque. */
    static const uint8_t KEY16[2] = {0x12, 0x34};
    pic = (picture_t){8, 6, 16, 0, 0, NULL, 0, KEY16, 2, NULL};
    pic.samples = make_samples(&pic, 65536, &seed);
    for (int i = 0; i < 8 * 6; i += 3) {
        pic.samples[i] = 0x1234;
        pic.samples[i + 1] = 0x12ff;
    }
    assert_decodes(&pic);
    assert(decode_picture(&pic, 0, "TD", &image) == PNG_OK);
    assert(image.pixels[0] == 0x12 && image.pixels[3] == 0);
    assert(image.pixels[4] == 0x12 && image.pixels[7] == 255);
    png_image_free(&image);

    /* A key is two bytes for gray, never more or fewer. */
    pic.trns_length = 1;
    assert(decode_picture(&pic, 0, "TD", &image) == PNG_ERR_CHUNK);
    pic.trns = (const uint8_t *)"\x12\x34\x56\x78\x9a\xbc";
    pic.trns_length = 6;
    assert(decode_picture(&pic, 0, "TD", &image) == PNG_ERR_CHUNK);
    free(pic.samples);

    /* Gray with alpha, at both depths it comes in. */
    for (int depth = 8; depth <= 16; depth += 8) {
        pic = (picture_t){7, 6, depth, 4, 0, NULL, 0, NULL, 0, NULL};
        pic.samples = make_samples(&pic, 1u << depth, &seed);
        assert_decodes(&pic);
        free(pic.samples);
    }
    assert(image.pixels == NULL);
}

static void test_sixteen_bits(void) {
    unsigned seed = 44;
    png_image_t image = {0, 0, NULL};

    /* An RGB key at 16 bits: pixels differing from it in one low byte read as the
     * same color and must stay opaque. */
    static const uint8_t KEY[6] = {0x12, 0x34, 0xab, 0xcd, 0x00, 0x01};
    picture_t pic = {7, 5, 16, 2, 0, NULL, 0, KEY, 6, NULL};
    pic.samples = make_samples(&pic, 65536, &seed);
    for (int i = 0; i + 1 < 7 * 5; i += 4) {
        unsigned *key = pic.samples + i * 3, *almost = key + 3;
        key[0] = almost[0] = 0x1234;
        key[1] = almost[1] = 0xabcd;
        key[2] = 0x0001;
        almost[2] = 0x0002;
    }
    assert_decodes(&pic);
    assert(decode_picture(&pic, 0, "TD", &image) == PNG_OK);
    static const uint8_t KEYED[4] = {0x12, 0xab, 0x00, 0}, ALMOST[4] = {0x12, 0xab, 0x00, 255};
    assert(memcmp(image.pixels, KEYED, 4) == 0 && memcmp(image.pixels + 4, ALMOST, 4) == 0);
    png_image_free(&image);

    /* The key is six bytes for RGB. */
    pic.trns_length = 2;
    assert(decode_picture(&pic, 0, "TD", &image) == PNG_ERR_CHUNK);
    free(pic.samples);

    /* The same at 8 bits, where the key is still written with two bytes a sample. */
    static const uint8_t KEY8[6] = {0, 10, 0, 20, 0, 30};
    pic = (picture_t){6, 6, 8, 2, 0, NULL, 0, KEY8, 6, NULL};
    pic.samples = make_samples(&pic, 256, &seed);
    pic.samples[3] = 10;
    pic.samples[4] = 20;
    pic.samples[5] = 30;
    assert_decodes(&pic);
    free(pic.samples);

    /* RGBA at 16 bits, eight bytes a pixel, with a palette the decoder ignores. */
    pic = (picture_t){6, 5, 16, 6, 0, (const uint8_t *)"\1\2\3", 3, NULL, 0, NULL};
    pic.samples = make_samples(&pic, 65536, &seed);
    assert_decodes(&pic);
    free(pic.samples);
    assert(image.pixels == NULL);
}

/* Adam7: seven passes, each its own small image with its own filtered rows,
 * scattered back into the whole. */
static void test_interlaced(void) {
    unsigned seed = 55;
    uint8_t palette[4 * 3] = {255, 0, 0, 0, 255, 0, 0, 0, 255, 9, 9, 9};
    static const uint8_t ALPHA[3] = {0, 128, 255}, KEY[6] = {0x80, 0x00, 0x00, 0x00, 0xff, 0xff};
    /* 13x7: passes of one or two columns, and rows that end in the middle of a
     * byte at every depth under 8. */
    picture_t cases[] = {
        {13, 7, 8, 6, 1, NULL, 0, NULL, 0, NULL},
        {13, 7, 16, 2, 1, NULL, 0, KEY, 6, NULL},
        {13, 7, 2, 3, 1, palette, 12, ALPHA, 3, NULL},
        {13, 7, 1, 0, 1, NULL, 0, NULL, 0, NULL},
        {13, 7, 16, 4, 1, NULL, 0, NULL, 0, NULL},
        {13, 7, 4, 0, 1, NULL, 0, NULL, 0, NULL},
        /* 1x1: passes 2 to 7 are empty, and send nothing at all. */
        {1, 1, 8, 6, 1, NULL, 0, NULL, 0, NULL},
        {1, 1, 1, 0, 1, NULL, 0, NULL, 0, NULL},
    };

    for (size_t k = 0; k < sizeof(cases) / sizeof(*cases); k++) {
        picture_t *pic = &cases[k];
        pic->samples = make_samples(pic, pic->color_type == 3 ? 4 : 1u << pic->depth, &seed);
        if (pic->color_type == 2) { /* One pixel on the key. */
            pic->samples[3] = 0x8000;
            pic->samples[4] = 0;
            pic->samples[5] = 0xffff;
        }
        assert_decodes(pic);
        free(pic->samples);
    }

    /* Every small size, where passes come and go. */
    for (int width = 1; width <= 10; width++) {
        for (int height = 1; height <= 10; height++) {
            picture_t gray = {width, height, 2, 0, 1, NULL, 0, NULL, 0, NULL};
            picture_t rgba = {width, height, 8, 6, 1, NULL, 0, NULL, 0, NULL};
            gray.samples = make_samples(&gray, 4, &seed);
            rgba.samples = make_samples(&rgba, 256, &seed);
            assert_decodes(&gray);
            assert_decodes(&rgba);
            free(gray.samples);
            free(rgba.samples);
        }
    }
}

static void test_unsupported_variants(void) {
    png_image_t image = {0, 0, NULL};
    bytes_t zlib = {NULL, 0, 0};
    static const uint8_t RAW[5] = {0, 1, 2, 3, 4};
    /* Depth and color type pairs outside the specification, then an interlace
     * method that does not exist. */
    static const int CASES[][3] = {{3, 0, 0}, {16, 3, 0}, {4, 2, 0}, {1, 4, 0},
                                   {2, 6, 0}, {8, 1, 0},  {8, 5, 0}, {8, 7, 0},
                                   {0, 0, 0}, {32, 6, 0}, {8, 6, 2}};

    put_zlib_stored(&zlib, RAW, sizeof(RAW));
    for (size_t k = 0; k < sizeof(CASES) / sizeof(*CASES); k++) {
        bytes_t png = {NULL, 0, 0};
        put_header(&png, 1, 1, CASES[k][0], CASES[k][1], CASES[k][2]);
        put_chunk(&png, "IDAT", zlib.data, zlib.length);
        put_chunk(&png, "IEND", NULL, 0);
        assert(png_decode(png.data, png.length, &image) == PNG_ERR_UNSUPPORTED);
        free(png.data);
    }

    /* A filter type past Paeth. */
    static const uint8_t BAD_FILTER[5] = {5, 1, 2, 3, 4};
    bytes_t png = {NULL, 0, 0};
    zlib.length = 0;
    put_zlib_stored(&zlib, BAD_FILTER, sizeof(BAD_FILTER));
    put_header(&png, 1, 1, 8, 6, 0);
    put_chunk(&png, "IDAT", zlib.data, zlib.length);
    put_chunk(&png, "IEND", NULL, 0);
    assert(png_decode(png.data, png.length, &image) == PNG_ERR_CHUNK);
    assert(image.pixels == NULL);
    free(png.data);
    free(zlib.data);
}

int main(void) {
    test_round_trip();
    test_compression_earns_its_place();
    test_transforms();
    test_refusals();
    test_damage_is_refused();
    test_bombs_are_refused();
    test_raster_length_is_exact();
    test_palettes();
    test_gray();
    test_sixteen_bits();
    test_interlaced();
    test_unsupported_variants();
    return 0;
}
