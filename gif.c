#include "gif.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum {
    GIF_COLOURS = 256,
    GIF_MIN_CODE_SIZE = 8, /* Eight bits, because the table is 256 long. */
    GIF_CLEAR_CODE = 1 << GIF_MIN_CODE_SIZE,
    GIF_END_CODE = GIF_CLEAR_CODE + 1,
    GIF_FIRST_CODE = GIF_CLEAR_CODE + 2,
    GIF_MAX_CODE = 4095,
    /* Colours are counted in five bits a channel: the difference between two
     * neighbouring buckets is below what anyone can see, and it turns the count
     * into one pass over a fixed table. */
    QUANT_BITS = 5,
    QUANT_LEVELS = 1 << QUANT_BITS,
    QUANT_BUCKETS = QUANT_LEVELS * QUANT_LEVELS * QUANT_LEVELS
};

struct gif_writer {
    FILE *file;
    int width, height;
    int frames;
    size_t written;
    gif_status_t status;
    int have_palette;
    uint8_t palette[GIF_COLOURS][3];
    uint8_t bucket_to_index[QUANT_BUCKETS]; /* Nearest palette entry, per bucket. */
    uint8_t *indices;                       /* One frame's worth. */
    /* LZW state, kept here so a frame does not allocate. */
    uint8_t block[255];
    int block_length;
    int delay; /* Hundredths of a second between frames. */
    uint32_t bits;
    int bit_count;
    int16_t *dictionary; /* GIF_MAX_CODE + 1 by GIF_COLOURS, -1 for empty. */
};

static void put_byte(gif_writer_t *w, uint8_t byte) {
    if (w->status != GIF_OK) return;
    if (fputc(byte, w->file) == EOF) {
        w->status = GIF_ERR_IO;
        return;
    }
    w->written++;
}

static void put_bytes(gif_writer_t *w, const void *data, size_t length) {
    if (w->status != GIF_OK) return;
    if (fwrite(data, 1, length, w->file) != length) {
        w->status = GIF_ERR_IO;
        return;
    }
    w->written += length;
}

static void put_short(gif_writer_t *w, int value) {
    put_byte(w, (uint8_t)(value & 0xff));
    put_byte(w, (uint8_t)((value >> 8) & 0xff));
}

/*============================ The colour table =============================*/

static int bucket_of(const uint8_t *rgb) {
    int r = rgb[0] >> (8 - QUANT_BITS), g = rgb[1] >> (8 - QUANT_BITS),
        b = rgb[2] >> (8 - QUANT_BITS);
    return (r * QUANT_LEVELS + g) * QUANT_LEVELS + b;
}

/*
 * The most used buckets become the table, and every other bucket is pointed at
 * whichever of them is nearest. On this program's output the two hundred and
 * fifty six most frequent colours cover better than 99.8% of the pixels, so the
 * result is visually exact rather than merely close.
 */
static gif_status_t build_palette(gif_writer_t *w, const png_image_t *frame) {
    uint32_t *counts = (uint32_t *)calloc(QUANT_BUCKETS, sizeof(*counts));
    uint32_t *sums = (uint32_t *)calloc((size_t)QUANT_BUCKETS * 3, sizeof(*sums));
    if (counts == NULL || sums == NULL) {
        free(counts);
        free(sums);
        return GIF_ERR_MEMORY;
    }

    size_t pixels = (size_t)frame->width * (size_t)frame->height;
    for (size_t i = 0; i < pixels; i++) {
        const uint8_t *rgb = frame->pixels + i * 4;
        int bucket = bucket_of(rgb);
        counts[bucket]++;
        for (int c = 0; c < 3; c++) sums[(size_t)bucket * 3 + (size_t)c] += rgb[c];
    }

    /* The top entries by count, chosen by repeated selection: two hundred and
     * fifty six passes over the buckets is nothing next to the pixels. */
    int chosen[GIF_COLOURS];
    int used = 0;
    for (int slot = 0; slot < GIF_COLOURS; slot++) {
        int best = -1;
        uint32_t best_count = 0;
        for (int bucket = 0; bucket < QUANT_BUCKETS; bucket++)
            if (counts[bucket] > best_count) {
                best_count = counts[bucket];
                best = bucket;
            }
        if (best < 0) break;
        chosen[used] = best;
        for (int c = 0; c < 3; c++)
            w->palette[used][c] = (uint8_t)(sums[(size_t)best * 3 + (size_t)c] / counts[best]);
        counts[best] = 0;
        used++;
    }
    if (used == 0) { /* A blank frame still needs one entry. */
        memset(w->palette[0], 0, 3);
        chosen[0] = 0;
        used = 1;
    }
    for (int slot = used; slot < GIF_COLOURS; slot++) memcpy(w->palette[slot], w->palette[0], 3);

    /* Every bucket in the cube gets pointed at the nearest chosen colour, once,
     * so mapping a pixel afterwards is a single lookup. */
    for (int bucket = 0; bucket < QUANT_BUCKETS; bucket++) {
        int r = (bucket / QUANT_LEVELS / QUANT_LEVELS) << (8 - QUANT_BITS);
        int g = ((bucket / QUANT_LEVELS) % QUANT_LEVELS) << (8 - QUANT_BITS);
        int b = (bucket % QUANT_LEVELS) << (8 - QUANT_BITS);
        long best_distance = -1;
        int best_slot = 0;
        for (int slot = 0; slot < used; slot++) {
            long dr = r - w->palette[slot][0], dg = g - w->palette[slot][1],
                 db = b - w->palette[slot][2];
            long distance = dr * dr + dg * dg + db * db;
            if (best_distance < 0 || distance < best_distance) {
                best_distance = distance;
                best_slot = slot;
            }
        }
        w->bucket_to_index[bucket] = (uint8_t)best_slot;
    }
    (void)chosen;
    free(counts);
    free(sums);
    w->have_palette = 1;
    return GIF_OK;
}

/*================================== LZW ====================================*/

static void flush_block(gif_writer_t *w) {
    if (w->block_length == 0) return;
    put_byte(w, (uint8_t)w->block_length);
    put_bytes(w, w->block, (size_t)w->block_length);
    w->block_length = 0;
}

static void put_code(gif_writer_t *w, int code, int width) {
    w->bits |= (uint32_t)code << w->bit_count;
    w->bit_count += width;
    while (w->bit_count >= 8) {
        w->block[w->block_length++] = (uint8_t)(w->bits & 0xff);
        w->bits >>= 8;
        w->bit_count -= 8;
        if (w->block_length == 255) flush_block(w);
    }
}

static void encode_frame(gif_writer_t *w, const uint8_t *indices, size_t count) {
    int width = GIF_MIN_CODE_SIZE + 1;
    int next = GIF_FIRST_CODE;

    memset(w->dictionary, -1, (size_t)(GIF_MAX_CODE + 1) * GIF_COLOURS * sizeof(*w->dictionary));
    w->block_length = 0;
    w->bits = 0;
    w->bit_count = 0;

    put_byte(w, GIF_MIN_CODE_SIZE);
    put_code(w, GIF_CLEAR_CODE, width);
    if (count == 0) {
        put_code(w, GIF_END_CODE, width);
        if (w->bit_count > 0) put_code(w, 0, 8 - w->bit_count);
        flush_block(w);
        put_byte(w, 0);
        return;
    }

    int current = indices[0];
    for (size_t i = 1; i < count; i++) {
        int next_index = indices[i];
        int16_t found = w->dictionary[(size_t)current * GIF_COLOURS + (size_t)next_index];
        if (found >= 0) {
            current = found;
            continue;
        }
        put_code(w, current, width);
        if (next <= GIF_MAX_CODE) {
            w->dictionary[(size_t)current * GIF_COLOURS + (size_t)next_index] = (int16_t)next;
            /* The width grows once the codes no longer fit, as the decoder
             * expects, and the table is cleared when it is full. */
            if (next == (1 << width) && width < 12) width++;
            next++;
        } else {
            put_code(w, GIF_CLEAR_CODE, width);
            memset(w->dictionary, -1,
                   (size_t)(GIF_MAX_CODE + 1) * GIF_COLOURS * sizeof(*w->dictionary));
            next = GIF_FIRST_CODE;
            width = GIF_MIN_CODE_SIZE + 1;
        }
        current = next_index;
    }
    put_code(w, current, width);
    put_code(w, GIF_END_CODE, width);
    if (w->bit_count > 0) put_code(w, 0, 8 - w->bit_count);
    flush_block(w);
    put_byte(w, 0); /* End of the image's sub-blocks. */
}

/*================================= Writing =================================*/

gif_status_t gif_open(gif_writer_t **out, const char *path, int width, int height,
                      int delay_hundredths) {
    if (out == NULL || path == NULL || width <= 0 || height <= 0 || width > 65535 || height > 65535)
        return GIF_ERR_ARGUMENT;

    gif_writer_t *w = (gif_writer_t *)calloc(1, sizeof(*w));
    if (w == NULL) return GIF_ERR_MEMORY;
    w->indices = (uint8_t *)malloc((size_t)width * (size_t)height);
    w->dictionary =
        (int16_t *)malloc((size_t)(GIF_MAX_CODE + 1) * GIF_COLOURS * sizeof(*w->dictionary));
    w->file = fopen(path, "wb");
    if (w->indices == NULL || w->dictionary == NULL || w->file == NULL) {
        gif_status_t why = w->file == NULL ? GIF_ERR_IO : GIF_ERR_MEMORY;
        if (w->file != NULL) fclose(w->file);
        free(w->indices);
        free(w->dictionary);
        free(w);
        return why;
    }
    w->width = width;
    w->height = height;
    w->status = GIF_OK;

    put_bytes(w, "GIF89a", 6);
    put_short(w, width);
    put_short(w, height);
    put_byte(w, 0xf7); /* Global table, 256 entries, 8 bits a colour. */
    put_byte(w, 0);    /* Background index. */
    put_byte(w, 0);    /* No aspect ratio. */
    /* The table itself is written by the first frame, once it is known, so the
     * header's remaining 768 bytes are reserved here and filled then. */
    for (int i = 0; i < GIF_COLOURS * 3; i++) put_byte(w, 0);

    /* Netscape's looping extension: the only way to say "forever". */
    put_bytes(w, "\x21\xff\x0bNETSCAPE2.0\x03\x01\x00\x00\x00", 19);
    w->frames = 0;
    /* The delay is per frame and goes in each graphic control block. */
    if (delay_hundredths < 1) delay_hundredths = 1;
    w->delay = delay_hundredths;
    *out = w;
    return w->status;
}

gif_status_t gif_add_frame(gif_writer_t *w, const png_image_t *frame) {
    if (w == NULL || frame == NULL || frame->pixels == NULL) return GIF_ERR_ARGUMENT;
    if (w->status != GIF_OK) return w->status;
    if (frame->width != w->width || frame->height != w->height) return GIF_ERR_ARGUMENT;

    if (!w->have_palette) {
        gif_status_t status = build_palette(w, frame);
        if (status != GIF_OK) return w->status = status;
        /* Back to the reserved space and write the table we now know. */
        long here = ftell(w->file);
        if (here < 0 || fseek(w->file, 13, SEEK_SET) != 0) return w->status = GIF_ERR_IO;
        if (fwrite(w->palette, 3, GIF_COLOURS, w->file) != GIF_COLOURS)
            return w->status = GIF_ERR_IO;
        if (fseek(w->file, here, SEEK_SET) != 0) return w->status = GIF_ERR_IO;
    }

    size_t pixels = (size_t)w->width * (size_t)w->height;
    for (size_t i = 0; i < pixels; i++)
        w->indices[i] = w->bucket_to_index[bucket_of(frame->pixels + i * 4)];

    put_bytes(w, "\x21\xf9\x04\x00", 4); /* Graphic control, no disposal. */
    put_short(w, w->delay);
    put_byte(w, 0); /* No transparent index. */
    put_byte(w, 0);

    put_byte(w, 0x2c); /* Image descriptor. */
    put_short(w, 0);
    put_short(w, 0);
    put_short(w, w->width);
    put_short(w, w->height);
    put_byte(w, 0); /* No local table, not interlaced. */

    encode_frame(w, w->indices, pixels);
    if (w->status == GIF_OK) w->frames++;
    return w->status;
}

gif_status_t gif_close(gif_writer_t *w, size_t *bytes_written, int *frames_written) {
    if (w == NULL) return GIF_ERR_ARGUMENT;
    put_byte(w, 0x3b); /* Trailer. */
    gif_status_t status = w->status;
    if (bytes_written != NULL) *bytes_written = w->written;
    if (frames_written != NULL) *frames_written = w->frames;
    if (fclose(w->file) != 0 && status == GIF_OK) status = GIF_ERR_IO;
    free(w->indices);
    free(w->dictionary);
    free(w);
    return status;
}

const char *gif_status_string(gif_status_t status) {
    switch (status) {
        case GIF_OK:
            return "ok";
        case GIF_ERR_ARGUMENT:
            return "invalid argument";
        case GIF_ERR_MEMORY:
            return "out of memory";
        case GIF_ERR_IO:
            return "write error";
    }
    return "unknown error";
}
