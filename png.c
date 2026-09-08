#include "png.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define PNG_MAX_DIMENSION 16384
#define PNG_MAX_PIXELS (1 << 26) /*64 M pixels, 256 MB once expanded to RGBA*/
#define DEFLATE_MAX_BLOCK 65535

const char *png_status_string(png_status_t status) {
    switch (status) {
        case PNG_OK:
            return "ok";
        case PNG_ERR_MEMORY:
            return "out of memory";
        case PNG_ERR_ARGUMENT:
            return "invalid argument";
        case PNG_ERR_TRUNCATED:
            return "truncated file";
        case PNG_ERR_SIGNATURE:
            return "not a PNG file";
        case PNG_ERR_CHUNK:
            return "malformed chunk";
        case PNG_ERR_CRC:
            return "chunk checksum mismatch";
        case PNG_ERR_UNSUPPORTED:
            return "unsupported PNG variant";
        case PNG_ERR_DEFLATE:
            return "corrupted compressed data";
    }
    return "unknown error";
}

/*============================== Checksums ==================================*/

static uint32_t crc_table[256];
static int crc_table_ready = 0;

static void crc_table_init() {
    for (uint32_t n = 0; n < 256; n++) {
        uint32_t c = n;
        for (int k = 0; k < 8; k++) c = (c & 1) ? 0xedb88320u ^ (c >> 1) : c >> 1;
        crc_table[n] = c;
    }
    crc_table_ready = 1;
}

static uint32_t crc32_of(const uint8_t *data, size_t length) {
    uint32_t c = 0xffffffffu;
    if (!crc_table_ready) crc_table_init();
    for (size_t i = 0; i < length; i++) c = crc_table[(c ^ data[i]) & 0xff] ^ (c >> 8);
    return c ^ 0xffffffffu;
}

static uint32_t adler32_of(const uint8_t *data, size_t length) {
    uint32_t a = 1, b = 0;
    for (size_t i = 0; i < length; i++) {
        a = (a + data[i]) % 65521u;
        b = (b + a) % 65521u;
    }
    return (b << 16) | a;
}

/*=============================== Inflate ===================================
 *
 * DEFLATE decompressor (RFC 1951). Supports the three block types : stored,
 * fixed Huffman and dynamic Huffman. Speed is not a concern here, the base
 * image is decompressed exactly once at startup, so the straightforward bit
 * by bit canonical Huffman decoding is used.
 */

typedef struct {
    const uint8_t *src;
    size_t src_len;
    size_t src_pos;
    uint32_t bit_buf;
    int bit_count;
    uint8_t *out;
    size_t out_len;
    size_t out_cap;
    int failed;
} inflate_t;

typedef struct {
    short counts[16];   /*number of codes per length*/
    short symbols[288]; /*symbols ordered by code*/
} huffman_t;

static int inflate_bits(inflate_t *s, int need) {
    uint32_t val = s->bit_buf;

    while (s->bit_count < need) {
        if (s->src_pos >= s->src_len) {
            s->failed = 1;
            return 0;
        }
        val |= (uint32_t)s->src[s->src_pos++] << s->bit_count;
        s->bit_count += 8;
    }
    s->bit_buf = val >> need;
    s->bit_count -= need;
    return (int)(val & ((1u << need) - 1u));
}

static int inflate_reserve(inflate_t *s, size_t extra) {
    size_t needed = s->out_len + extra;
    if (needed <= s->out_cap) return 1;

    size_t cap = s->out_cap ? s->out_cap : 4096;
    while (cap < needed) {
        if (cap > (size_t)-1 / 2) return 0;
        cap *= 2;
    }
    uint8_t *grown = (uint8_t *)realloc(s->out, cap);
    if (grown == NULL) return 0;
    s->out = grown;
    s->out_cap = cap;
    return 1;
}

static void huffman_build(huffman_t *h, const uint8_t *lengths, int count) {
    short offsets[16];

    memset(h->counts, 0, sizeof(h->counts));
    for (int i = 0; i < count; i++) h->counts[lengths[i]]++;
    h->counts[0] = 0;

    offsets[0] = 0;
    offsets[1] = 0;
    for (int len = 1; len < 15; len++) offsets[len + 1] = (short)(offsets[len] + h->counts[len]);
    for (int i = 0; i < count; i++)
        if (lengths[i]) h->symbols[offsets[lengths[i]]++] = (short)i;
}

static int huffman_decode(inflate_t *s, const huffman_t *h) {
    int code = 0, first = 0, index = 0;

    for (int len = 1; len <= 15; len++) {
        code |= inflate_bits(s, 1);
        if (s->failed) return -1;
        int count = h->counts[len];
        if (code - first < count) return h->symbols[index + (code - first)];
        index += count;
        first = (first + count) << 1;
        code <<= 1;
    }
    return -1;
}

static int inflate_stored(inflate_t *s) {
    s->bit_buf = 0;
    s->bit_count = 0;
    if (s->src_pos + 4 > s->src_len) return 0;

    unsigned len = s->src[s->src_pos] | ((unsigned)s->src[s->src_pos + 1] << 8);
    unsigned nlen = s->src[s->src_pos + 2] | ((unsigned)s->src[s->src_pos + 3] << 8);
    s->src_pos += 4;
    if (len != (~nlen & 0xffffu)) return 0;
    if (s->src_pos + len > s->src_len) return 0;
    if (!inflate_reserve(s, len)) return 0;

    memcpy(s->out + s->out_len, s->src + s->src_pos, len);
    s->out_len += len;
    s->src_pos += len;
    return 1;
}

static const short length_base[29] = {3,  4,  5,  6,   7,   8,   9,   10,  11, 13,
                                      15, 17, 19, 23,  27,  31,  35,  43,  51, 59,
                                      67, 83, 99, 115, 131, 163, 195, 227, 258};
static const short length_extra[29] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2,
                                       2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 0};
static const short dist_base[30] = {1,    2,    3,    4,    5,    7,    9,    13,    17,    25,
                                    33,   49,   65,   97,   129,  193,  257,  385,   513,   769,
                                    1025, 1537, 2049, 3073, 4097, 6145, 8193, 12289, 16385, 24577};
static const short dist_extra[30] = {0, 0, 0, 0, 1, 1, 2, 2,  3,  3,  4,  4,  5,  5,  6,
                                     6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13};

static int inflate_block(inflate_t *s, const huffman_t *lencode, const huffman_t *distcode) {
    for (;;) {
        int symbol = huffman_decode(s, lencode);
        if (symbol < 0) return 0;

        if (symbol < 256) {
            if (!inflate_reserve(s, 1)) return 0;
            s->out[s->out_len++] = (uint8_t)symbol;
            continue;
        }
        if (symbol == 256) return 1; /*end of block*/

        symbol -= 257;
        if (symbol >= 29) return 0;
        int len = length_base[symbol] + inflate_bits(s, length_extra[symbol]);

        symbol = huffman_decode(s, distcode);
        if (symbol < 0 || symbol >= 30) return 0;
        size_t dist = (size_t)dist_base[symbol] + (size_t)inflate_bits(s, dist_extra[symbol]);
        if (s->failed || dist > s->out_len) return 0;
        if (!inflate_reserve(s, (size_t)len)) return 0;

        for (int i = 0; i < len; i++) {
            s->out[s->out_len] = s->out[s->out_len - dist];
            s->out_len++;
        }
    }
}

static void inflate_fixed_tables(huffman_t *lencode, huffman_t *distcode) {
    uint8_t lengths[288];

    for (int i = 0; i < 144; i++) lengths[i] = 8;
    for (int i = 144; i < 256; i++) lengths[i] = 9;
    for (int i = 256; i < 280; i++) lengths[i] = 7;
    for (int i = 280; i < 288; i++) lengths[i] = 8;
    huffman_build(lencode, lengths, 288);

    for (int i = 0; i < 30; i++) lengths[i] = 5;
    huffman_build(distcode, lengths, 30);
}

static int inflate_dynamic_tables(inflate_t *s, huffman_t *lencode, huffman_t *distcode) {
    static const uint8_t order[19] = {16, 17, 18, 0, 8,  7, 9,  6, 10, 5,
                                      11, 4,  12, 3, 13, 2, 14, 1, 15};
    uint8_t lengths[288 + 30];
    huffman_t codelen;

    int nlen = inflate_bits(s, 5) + 257;
    int ndist = inflate_bits(s, 5) + 1;
    int ncode = inflate_bits(s, 4) + 4;
    if (s->failed || nlen > 286 || ndist > 30) return 0;

    memset(lengths, 0, 19);
    for (int i = 0; i < ncode; i++) lengths[order[i]] = (uint8_t)inflate_bits(s, 3);
    if (s->failed) return 0;
    huffman_build(&codelen, lengths, 19);

    int index = 0;
    while (index < nlen + ndist) {
        int symbol = huffman_decode(s, &codelen);
        if (symbol < 0) return 0;

        if (symbol < 16) {
            lengths[index++] = (uint8_t)symbol;
            continue;
        }

        int repeat;
        uint8_t value = 0;
        if (symbol == 16) {
            if (index == 0) return 0;
            value = lengths[index - 1];
            repeat = 3 + inflate_bits(s, 2);
        } else if (symbol == 17) {
            repeat = 3 + inflate_bits(s, 3);
        } else {
            repeat = 11 + inflate_bits(s, 7);
        }
        if (s->failed || index + repeat > nlen + ndist) return 0;
        while (repeat--) lengths[index++] = value;
    }
    if (lengths[256] == 0) return 0; /*no end of block code*/

    huffman_build(lencode, lengths, nlen);
    huffman_build(distcode, lengths + nlen, ndist);
    return 1;
}

/*Inflates a raw DEFLATE stream, *out must be freed by the caller*/
static png_status_t inflate_raw(const uint8_t *data, size_t length, uint8_t **out,
                                size_t *out_length, size_t *consumed) {
    inflate_t s;
    huffman_t lencode, distcode;
    int final;

    memset(&s, 0, sizeof(s));
    s.src = data;
    s.src_len = length;

    do {
        final = inflate_bits(&s, 1);
        int type = inflate_bits(&s, 2);
        int ok;

        if (s.failed) {
            free(s.out);
            return PNG_ERR_TRUNCATED;
        }
        if (type == 0) {
            ok = inflate_stored(&s);
        } else if (type == 1) {
            inflate_fixed_tables(&lencode, &distcode);
            ok = inflate_block(&s, &lencode, &distcode);
        } else if (type == 2) {
            ok = inflate_dynamic_tables(&s, &lencode, &distcode) &&
                 inflate_block(&s, &lencode, &distcode);
        } else {
            ok = 0;
        }
        if (!ok || s.failed) {
            free(s.out);
            return PNG_ERR_DEFLATE;
        }
    } while (!final);

    *out = s.out;
    *out_length = s.out_len;
    if (consumed) *consumed = s.src_pos;
    return PNG_OK;
}

/*Inflates a zlib stream (RFC 1950) verifying header and Adler checksum*/
static png_status_t inflate_zlib(const uint8_t *data, size_t length, uint8_t **out,
                                 size_t *out_length) {
    if (length < 6) return PNG_ERR_TRUNCATED;
    if ((data[0] & 0x0f) != 8) return PNG_ERR_UNSUPPORTED; /*not deflate*/
    if (data[1] & 0x20) return PNG_ERR_UNSUPPORTED;        /*preset dictionary*/
    if ((((unsigned)data[0] << 8) | data[1]) % 31u) return PNG_ERR_DEFLATE;

    size_t consumed = 0;
    png_status_t status = inflate_raw(data + 2, length - 2, out, out_length, &consumed);
    if (status != PNG_OK) return status;

    if (length - 2 - consumed >= 4) {
        const uint8_t *tail = data + 2 + consumed;
        uint32_t stored = ((uint32_t)tail[0] << 24) | ((uint32_t)tail[1] << 16) |
                          ((uint32_t)tail[2] << 8) | tail[3];
        if (stored != adler32_of(*out, *out_length)) {
            free(*out);
            *out = NULL;
            return PNG_ERR_DEFLATE;
        }
    }
    return PNG_OK;
}

/*============================ Image helpers ================================*/

png_status_t png_image_alloc(png_image_t *image, int width, int height) {
    if (image == NULL || width <= 0 || height <= 0) return PNG_ERR_ARGUMENT;
    if (width > PNG_MAX_DIMENSION || height > PNG_MAX_DIMENSION) return PNG_ERR_UNSUPPORTED;
    if ((long)width * height > PNG_MAX_PIXELS) return PNG_ERR_UNSUPPORTED;

    image->pixels = (uint8_t *)calloc((size_t)width * (size_t)height, 4);
    if (image->pixels == NULL) return PNG_ERR_MEMORY;
    image->width = width;
    image->height = height;
    return PNG_OK;
}

void png_image_free(png_image_t *image) {
    if (image == NULL) return;
    free(image->pixels);
    image->pixels = NULL;
    image->width = 0;
    image->height = 0;
}

/*Color channels are scaled by alpha so that filtering never drags the color
 * of fully transparent pixels into the visible ones*/
static void premultiply(uint8_t *pixels, size_t count) {
    for (size_t i = 0; i < count; i++) {
        uint8_t *p = pixels + i * 4;
        unsigned a = p[3];
        p[0] = (uint8_t)((p[0] * a + 127) / 255);
        p[1] = (uint8_t)((p[1] * a + 127) / 255);
        p[2] = (uint8_t)((p[2] * a + 127) / 255);
    }
}

static void unpremultiply(uint8_t *pixels, size_t count) {
    for (size_t i = 0; i < count; i++) {
        uint8_t *p = pixels + i * 4;
        unsigned a = p[3];
        if (a == 0) {
            p[0] = p[1] = p[2] = 0;
            continue;
        }
        for (int c = 0; c < 3; c++) {
            unsigned v = (p[c] * 255u + a / 2) / a;
            p[c] = (uint8_t)(v > 255 ? 255 : v);
        }
    }
}

/*Samples the premultiplied buffer, everything outside the image is transparent*/
static void sample_bilinear(const uint8_t *pixels, int width, int height, double x, double y,
                            uint8_t *out) {
    double fx = x - 0.5, fy = y - 0.5;
    int x0 = (int)floor(fx), y0 = (int)floor(fy);
    double wx = fx - x0, wy = fy - y0;

    for (int c = 0; c < 4; c++) out[c] = 0;

    double acc[4] = {0, 0, 0, 0};
    for (int dy = 0; dy < 2; dy++) {
        for (int dx = 0; dx < 2; dx++) {
            int sx = x0 + dx, sy = y0 + dy;
            if (sx < 0 || sy < 0 || sx >= width || sy >= height) continue;
            double weight = (dx ? wx : 1.0 - wx) * (dy ? wy : 1.0 - wy);
            const uint8_t *p = pixels + ((size_t)sy * width + sx) * 4;
            for (int c = 0; c < 4; c++) acc[c] += weight * p[c];
        }
    }
    for (int c = 0; c < 4; c++) {
        double v = acc[c] + 0.5;
        out[c] = (uint8_t)(v < 0 ? 0 : (v > 255 ? 255 : v));
    }
}

/*=============================== Transforms ================================*/

png_status_t png_rotate(const png_image_t *src, double radians, png_image_t *out) {
    if (src == NULL || src->pixels == NULL || out == NULL) return PNG_ERR_ARGUMENT;

    size_t count = (size_t)src->width * (size_t)src->height;
    uint8_t *work = (uint8_t *)malloc(count * 4);
    if (work == NULL) return PNG_ERR_MEMORY;
    memcpy(work, src->pixels, count * 4);
    premultiply(work, count);

    png_status_t status = png_image_alloc(out, src->width, src->height);
    if (status != PNG_OK) {
        free(work);
        return status;
    }

    double cx = src->width / 2.0, cy = src->height / 2.0;
    double cs = cos(radians), sn = sin(radians);

    for (int y = 0; y < out->height; y++) {
        for (int x = 0; x < out->width; x++) {
            double dx = x + 0.5 - cx, dy = y + 0.5 - cy;
            /*inverse rotation : where does this destination pixel come from*/
            double sx = cx + dx * cs + dy * sn;
            double sy = cy - dx * sn + dy * cs;
            sample_bilinear(work, src->width, src->height, sx, sy,
                            out->pixels + ((size_t)y * out->width + x) * 4);
        }
    }
    unpremultiply(out->pixels, count);
    free(work);
    return PNG_OK;
}

png_status_t png_resize(const png_image_t *src, int width, int height, png_image_t *out) {
    if (src == NULL || src->pixels == NULL || out == NULL) return PNG_ERR_ARGUMENT;
    if (width <= 0 || height <= 0) return PNG_ERR_ARGUMENT;

    size_t src_count = (size_t)src->width * (size_t)src->height;
    uint8_t *work = (uint8_t *)malloc(src_count * 4);
    if (work == NULL) return PNG_ERR_MEMORY;
    memcpy(work, src->pixels, src_count * 4);
    premultiply(work, src_count);

    png_status_t status = png_image_alloc(out, width, height);
    if (status != PNG_OK) {
        free(work);
        return status;
    }

    double scale_x = (double)src->width / width;
    double scale_y = (double)src->height / height;
    int shrinking = (width <= src->width && height <= src->height);

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            uint8_t *dst = out->pixels + ((size_t)y * width + x) * 4;

            if (shrinking) {
                /*box filter : average of every source pixel falling in the cell*/
                int x0 = (int)floor(x * scale_x), x1 = (int)ceil((x + 1) * scale_x);
                int y0 = (int)floor(y * scale_y), y1 = (int)ceil((y + 1) * scale_y);
                if (x1 <= x0) x1 = x0 + 1;
                if (y1 <= y0) y1 = y0 + 1;
                if (x1 > src->width) x1 = src->width;
                if (y1 > src->height) y1 = src->height;

                uint32_t acc[4] = {0, 0, 0, 0};
                uint32_t samples = 0;
                for (int sy = y0; sy < y1; sy++) {
                    for (int sx = x0; sx < x1; sx++) {
                        const uint8_t *p = work + ((size_t)sy * src->width + sx) * 4;
                        for (int c = 0; c < 4; c++) acc[c] += p[c];
                        samples++;
                    }
                }
                if (samples == 0) samples = 1;
                for (int c = 0; c < 4; c++) dst[c] = (uint8_t)((acc[c] + samples / 2) / samples);
            } else {
                sample_bilinear(work, src->width, src->height, (x + 0.5) * scale_x,
                                (y + 0.5) * scale_y, dst);
            }
        }
    }
    unpremultiply(out->pixels, (size_t)width * (size_t)height);
    free(work);
    return PNG_OK;
}

png_status_t png_rotate_resize(const png_image_t *src, double radians, int width, int height,
                               png_image_t *out) {
    png_image_t rotated = {0, 0, NULL};
    png_status_t status = png_rotate(src, radians, &rotated);

    if (status != PNG_OK) return status;
    status = png_resize(&rotated, width, height, out);
    png_image_free(&rotated);
    return status;
}

void png_tint(png_image_t *image, uint8_t r, uint8_t g, uint8_t b, png_tint_mode_t mode) {
    if (image == NULL || image->pixels == NULL) return;

    size_t count = (size_t)image->width * (size_t)image->height;
    const uint8_t color[3] = {r, g, b};

    for (size_t i = 0; i < count; i++) {
        uint8_t *p = image->pixels + i * 4;
        for (int c = 0; c < 3; c++)
            p[c] = mode == PNG_TINT_REPLACE ? color[c] : (uint8_t)((p[c] * color[c] + 127) / 255);
    }
}

/*============================== PNG decoding ===============================*/

static const uint8_t png_signature[8] = {0x89, 'P', 'N', 'G', '\r', '\n', 0x1a, '\n'};

static uint32_t read_be32(const uint8_t *p) {
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}

static void write_be32(uint8_t *p, uint32_t value) {
    p[0] = (uint8_t)(value >> 24);
    p[1] = (uint8_t)(value >> 16);
    p[2] = (uint8_t)(value >> 8);
    p[3] = (uint8_t)value;
}

static int paeth_predictor(int a, int b, int c) {
    int p = a + b - c;
    int pa = abs(p - a), pb = abs(p - b), pc = abs(p - c);

    if (pa <= pb && pa <= pc) return a;
    if (pb <= pc) return b;
    return c;
}

/*Undoes the per scanline filters, in place, on the raw (still packed) raster*/
static int png_unfilter(uint8_t *raster, int width, int height, int channels) {
    size_t stride = (size_t)width * channels;
    uint8_t *previous = NULL;
    uint8_t *row = raster;

    for (int y = 0; y < height; y++) {
        int filter = row[0];
        uint8_t *current = row + 1;

        for (size_t i = 0; i < stride; i++) {
            int a = i >= (size_t)channels ? current[i - channels] : 0;
            int b = previous ? previous[i] : 0;
            int c = (previous && i >= (size_t)channels) ? previous[i - channels] : 0;
            int value = current[i];

            switch (filter) {
                case 0:
                    break;
                case 1:
                    value += a;
                    break;
                case 2:
                    value += b;
                    break;
                case 3:
                    value += (a + b) / 2;
                    break;
                case 4:
                    value += paeth_predictor(a, b, c);
                    break;
                default:
                    return 0;
            }
            current[i] = (uint8_t)value;
        }
        previous = current;
        row += stride + 1;
    }
    return 1;
}

/*Expands the unfiltered raster into straight RGBA*/
static void png_expand(const uint8_t *raster, int width, int height, int color_type, int channels,
                       uint8_t *rgba) {
    size_t stride = (size_t)width * channels;

    for (int y = 0; y < height; y++) {
        const uint8_t *src = raster + (stride + 1) * y + 1;
        uint8_t *dst = rgba + (size_t)y * width * 4;

        for (int x = 0; x < width; x++) {
            const uint8_t *p = src + (size_t)x * channels;
            uint8_t *q = dst + (size_t)x * 4;

            switch (color_type) {
                case 0:
                    q[0] = q[1] = q[2] = p[0];
                    q[3] = 255;
                    break;
                case 4:
                    q[0] = q[1] = q[2] = p[0];
                    q[3] = p[1];
                    break;
                case 2:
                    q[0] = p[0];
                    q[1] = p[1];
                    q[2] = p[2];
                    q[3] = 255;
                    break;
                default:
                    q[0] = p[0];
                    q[1] = p[1];
                    q[2] = p[2];
                    q[3] = p[3];
                    break;
            }
        }
    }
}

png_status_t png_decode(const uint8_t *data, size_t length, png_image_t *out) {
    if (data == NULL || out == NULL) return PNG_ERR_ARGUMENT;
    if (length < sizeof(png_signature)) return PNG_ERR_TRUNCATED;
    if (memcmp(data, png_signature, sizeof(png_signature)) != 0) return PNG_ERR_SIGNATURE;

    int width = 0, height = 0, color_type = -1, channels = 0;
    uint8_t *idat = NULL;
    size_t idat_len = 0, idat_cap = 0;
    size_t pos = sizeof(png_signature);
    png_status_t status = PNG_ERR_CHUNK;
    int seen_end = 0;

    while (pos + 8 <= length) {
        uint32_t chunk_len = read_be32(data + pos);
        const uint8_t *type = data + pos + 4;

        if (chunk_len > 0x7fffffffu || pos + 12 + chunk_len > length) {
            status = PNG_ERR_TRUNCATED;
            goto fail;
        }
        const uint8_t *body = data + pos + 8;
        if (crc32_of(type, chunk_len + 4) != read_be32(body + chunk_len)) {
            status = PNG_ERR_CRC;
            goto fail;
        }

        if (memcmp(type, "IHDR", 4) == 0) {
            if (chunk_len != 13 || color_type >= 0) goto fail;
            width = (int)read_be32(body);
            height = (int)read_be32(body + 4);
            int bit_depth = body[8];
            color_type = body[9];

            if (width <= 0 || height <= 0 || width > PNG_MAX_DIMENSION ||
                height > PNG_MAX_DIMENSION || (long)width * height > PNG_MAX_PIXELS) {
                status = PNG_ERR_UNSUPPORTED;
                goto fail;
            }
            if (bit_depth != 8 || body[10] != 0 || body[11] != 0 || body[12] != 0) {
                status = PNG_ERR_UNSUPPORTED; /*only 8 bit, deflate, no interlace*/
                goto fail;
            }
            switch (color_type) {
                case 0:
                    channels = 1;
                    break;
                case 2:
                    channels = 3;
                    break;
                case 4:
                    channels = 2;
                    break;
                case 6:
                    channels = 4;
                    break;
                default:
                    status = PNG_ERR_UNSUPPORTED;
                    goto fail; /*palette*/
            }
        } else if (memcmp(type, "IDAT", 4) == 0) {
            if (color_type < 0) goto fail;
            if (idat_len + chunk_len > idat_cap) {
                size_t cap = idat_cap ? idat_cap : 8192;
                while (cap < idat_len + chunk_len) cap *= 2;
                uint8_t *grown = (uint8_t *)realloc(idat, cap);
                if (grown == NULL) {
                    status = PNG_ERR_MEMORY;
                    goto fail;
                }
                idat = grown;
                idat_cap = cap;
            }
            memcpy(idat + idat_len, body, chunk_len);
            idat_len += chunk_len;
        } else if (memcmp(type, "IEND", 4) == 0) {
            seen_end = 1;
            break;
        }
        pos += 12 + chunk_len;
    }

    if (color_type < 0 || idat_len == 0 || !seen_end) {
        status = seen_end ? PNG_ERR_CHUNK : PNG_ERR_TRUNCATED;
        goto fail;
    }

    uint8_t *raster = NULL;
    size_t raster_len = 0;
    status = inflate_zlib(idat, idat_len, &raster, &raster_len);
    free(idat);
    idat = NULL;
    if (status != PNG_OK) return status;

    size_t expected = ((size_t)width * channels + 1) * height;
    if (raster_len < expected) {
        free(raster);
        return PNG_ERR_TRUNCATED;
    }
    if (!png_unfilter(raster, width, height, channels)) {
        free(raster);
        return PNG_ERR_CHUNK;
    }

    status = png_image_alloc(out, width, height);
    if (status != PNG_OK) {
        free(raster);
        return status;
    }
    png_expand(raster, width, height, color_type, channels, out->pixels);
    free(raster);
    return PNG_OK;

fail:
    free(idat);
    return status;
}

/*============================== PNG encoding ===============================
 *
 * The zlib stream is written with stored (uncompressed) DEFLATE blocks : it
 * is a perfectly valid PNG that any decoder reads, and it keeps the encoder
 * down to a handful of lines. Sprites are a few hundred bytes each, so the
 * size that is given up does not matter here.
 */

static uint8_t *write_chunk(uint8_t *p, const char *type, const uint8_t *body, size_t length) {
    write_be32(p, (uint32_t)length);
    memcpy(p + 4, type, 4);
    if (length) memcpy(p + 8, body, length);
    write_be32(p + 8 + length, crc32_of(p + 4, length + 4));
    return p + 12 + length;
}

png_status_t png_encode(const png_image_t *image, uint8_t **out_data, size_t *out_length) {
    if (image == NULL || image->pixels == NULL || out_data == NULL || out_length == NULL)
        return PNG_ERR_ARGUMENT;
    if (image->width <= 0 || image->height <= 0) return PNG_ERR_ARGUMENT;

    size_t stride = (size_t)image->width * 4 + 1; /*one filter byte per scanline*/
    size_t raw_len = stride * (size_t)image->height;

    uint8_t *raw = (uint8_t *)malloc(raw_len);
    if (raw == NULL) return PNG_ERR_MEMORY;
    for (int y = 0; y < image->height; y++) {
        raw[stride * y] = 0; /*filter : none*/
        memcpy(raw + stride * y + 1, image->pixels + (size_t)y * image->width * 4,
               (size_t)image->width * 4);
    }

    size_t blocks = (raw_len + DEFLATE_MAX_BLOCK - 1) / DEFLATE_MAX_BLOCK;
    if (blocks == 0) blocks = 1;
    size_t zlib_len = 2 + blocks * 5 + raw_len + 4;
    size_t total = sizeof(png_signature) + (12 + 13) + (12 + zlib_len) + 12;

    uint8_t *png = (uint8_t *)malloc(total);
    if (png == NULL) {
        free(raw);
        return PNG_ERR_MEMORY;
    }
    uint8_t *zlib = (uint8_t *)malloc(zlib_len);
    if (zlib == NULL) {
        free(raw);
        free(png);
        return PNG_ERR_MEMORY;
    }

    zlib[0] = 0x78; /*deflate, 32k window*/
    zlib[1] = 0x01; /*no dictionary, checksum of the two bytes is a multiple of 31*/
    size_t zp = 2, offset = 0;
    for (size_t i = 0; i < blocks; i++) {
        size_t chunk = raw_len - offset;
        if (chunk > DEFLATE_MAX_BLOCK) chunk = DEFLATE_MAX_BLOCK;

        zlib[zp++] = (i + 1 == blocks) ? 1 : 0; /*stored block, final flag*/
        zlib[zp++] = (uint8_t)(chunk & 0xff);
        zlib[zp++] = (uint8_t)(chunk >> 8);
        zlib[zp++] = (uint8_t)(~chunk & 0xff);
        zlib[zp++] = (uint8_t)((~chunk >> 8) & 0xff);
        memcpy(zlib + zp, raw + offset, chunk);
        zp += chunk;
        offset += chunk;
    }
    write_be32(zlib + zp, adler32_of(raw, raw_len));
    zp += 4;

    uint8_t header[13];
    write_be32(header, (uint32_t)image->width);
    write_be32(header + 4, (uint32_t)image->height);
    header[8] = 8;  /*bit depth*/
    header[9] = 6;  /*RGBA*/
    header[10] = 0; /*deflate*/
    header[11] = 0; /*adaptive filtering*/
    header[12] = 0; /*no interlace*/

    uint8_t *p = png;
    memcpy(p, png_signature, sizeof(png_signature));
    p += sizeof(png_signature);
    p = write_chunk(p, "IHDR", header, sizeof(header));
    p = write_chunk(p, "IDAT", zlib, zp);
    p = write_chunk(p, "IEND", NULL, 0);

    free(zlib);
    free(raw);
    *out_data = png;
    *out_length = (size_t)(p - png);
    return PNG_OK;
}
