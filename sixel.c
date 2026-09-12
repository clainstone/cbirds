#include "sixel.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

const char *sixel_status_string(sixel_status_t status) {
    switch (status) {
        case SIXEL_OK:
            return "ok";
        case SIXEL_ERR_ARGUMENT:
            return "invalid argument";
        case SIXEL_ERR_MEMORY:
            return "out of memory";
    }
    return "unknown";
}

sixel_status_t sixel_init(sixel_t *sixel) {
    if (sixel == NULL) return SIXEL_ERR_ARGUMENT;
    memset(sixel, 0, sizeof(*sixel));
    sixel->memo = malloc(32768 * sizeof(*sixel->memo));
    if (sixel->memo == NULL) return SIXEL_ERR_MEMORY;
    memset(sixel->memo, -1, 32768 * sizeof(*sixel->memo));
    return SIXEL_OK;
}

void sixel_destroy(sixel_t *sixel) {
    if (sixel == NULL) return;
    free(sixel->text);
    free(sixel->index);
    free(sixel->memo);
    memset(sixel, 0, sizeof(*sixel));
}

static sixel_status_t reserve(sixel_t *sixel, size_t extra) {
    size_t needed = sixel->length + extra + 1;
    if (needed <= sixel->capacity) return SIXEL_OK;
    size_t capacity = sixel->capacity ? sixel->capacity : 16384;
    while (capacity < needed) capacity *= 2;
    char *grown = realloc(sixel->text, capacity);
    if (grown == NULL) return SIXEL_ERR_MEMORY;
    sixel->text = grown;
    sixel->capacity = capacity;
    return SIXEL_OK;
}

static sixel_status_t put(sixel_t *sixel, const char *bytes, size_t length) {
    sixel_status_t status = reserve(sixel, length);
    if (status != SIXEL_OK) return status;
    memcpy(sixel->text + sixel->length, bytes, length);
    sixel->length += length;
    sixel->text[sixel->length] = '\0';
    return SIXEL_OK;
}

static sixel_status_t put_number(sixel_t *sixel, const char *prefix, long number) {
    char scratch[32];
    int length = snprintf(scratch, sizeof(scratch), "%s%ld", prefix, number);
    return put(sixel, scratch, (size_t)length);
}

/* The palette entry a pixel is: an exact match when there is one, the nearest by
 * squared distance when there is not, remembered per 15 bit colour so the search
 * happens once per colour a frame ever contains rather than once per pixel. */
static int nearest(sixel_t *sixel, const uint8_t *px, const uint8_t (*palette)[3], int colours) {
    int key = ((px[0] >> 3) << 10) | ((px[1] >> 3) << 5) | (px[2] >> 3);
    if (sixel->memo[key] >= 0) return sixel->memo[key];
    int best = 0;
    long best_distance = -1;
    for (int c = 0; c < colours; c++) {
        long dr = (long)px[0] - palette[c][0], dg = (long)px[1] - palette[c][1],
             db = (long)px[2] - palette[c][2];
        long distance = dr * dr + dg * dg + db * db;
        if (best_distance < 0 || distance < best_distance) {
            best_distance = distance;
            best = c;
        }
    }
    sixel->memo[key] = (int16_t)best;
    return best;
}

/* A run of one sixel character: written out when it changes. Runs of three or
 * fewer are cheaper spelled out than counted. */
static sixel_status_t put_run(sixel_t *sixel, int six, long run) {
    if (run <= 0) return SIXEL_OK;
    char glyph = (char)('?' + six);
    if (run <= 3) {
        char spelled[3] = {glyph, glyph, glyph};
        return put(sixel, spelled, (size_t)run);
    }
    sixel_status_t status = put_number(sixel, "!", run);
    if (status == SIXEL_OK) status = put(sixel, &glyph, 1);
    return status;
}

sixel_status_t sixel_encode(sixel_t *sixel, const png_image_t *image, const uint8_t (*palette)[3],
                            int colours) {
    if (sixel == NULL || image == NULL || image->pixels == NULL || palette == NULL || colours < 1 ||
        colours > SIXEL_COLOURS_MAX || image->width < 1 || image->height < 1)
        return SIXEL_ERR_ARGUMENT;

    /* A new palette forgets what the old one taught. */
    if (colours != sixel->memo_colours) {
        memset(sixel->memo, -1, 32768 * sizeof(*sixel->memo));
        sixel->memo_colours = colours;
    }
    size_t pixels = (size_t)image->width * (size_t)image->height;
    if (pixels > sixel->index_capacity) {
        uint8_t *grown = realloc(sixel->index, pixels);
        if (grown == NULL) return SIXEL_ERR_MEMORY;
        sixel->index = grown;
        sixel->index_capacity = pixels;
    }
    for (size_t i = 0; i < pixels; i++)
        sixel->index[i] = (uint8_t)nearest(sixel, &image->pixels[i * 4], palette, colours);

    sixel->length = 0;
    /* P2 = 1: pixels a pass leaves at zero keep what is under them. Every pixel is
     * in some colour's pass here, ground included, so nothing is left to chance;
     * the flag only means a partial redraw would not blank what it skipped. */
    sixel_status_t status = put(sixel, "\033P0;1;0q", 8);
    if (status == SIXEL_OK) {
        char raster[48];
        int length = snprintf(raster, sizeof(raster), "\"1;1;%d;%d", image->width, image->height);
        status = put(sixel, raster, (size_t)length);
    }
    for (int c = 0; c < colours && status == SIXEL_OK; c++) {
        char entry[48];
        int length = snprintf(entry, sizeof(entry), "#%d;2;%d;%d;%d", c, palette[c][0] * 100 / 255,
                              palette[c][1] * 100 / 255, palette[c][2] * 100 / 255);
        status = put(sixel, entry, (size_t)length);
    }

    /* Per band of six rows: which colours are in it and where each starts and
     * stops, in one pass over the pixels, then one run length pass per colour
     * that only walks the columns it occupies. */
    int *first = malloc((size_t)colours * sizeof(int));
    int *last = malloc((size_t)colours * sizeof(int));
    uint8_t *column_bits = malloc((size_t)image->width);
    if (first == NULL || last == NULL || column_bits == NULL) status = SIXEL_ERR_MEMORY;

    for (int y0 = 0; y0 < image->height && status == SIXEL_OK; y0 += 6) {
        for (int c = 0; c < colours; c++) {
            first[c] = image->width;
            last[c] = -1;
        }
        int rows = image->height - y0 < 6 ? image->height - y0 : 6;
        for (int dy = 0; dy < rows; dy++) {
            const uint8_t *row = sixel->index + (size_t)(y0 + dy) * (size_t)image->width;
            for (int x = 0; x < image->width; x++) {
                int c = row[x];
                if (x < first[c]) first[c] = x;
                if (x > last[c]) last[c] = x;
            }
        }
        for (int c = 0; c < colours && status == SIXEL_OK; c++) {
            if (last[c] < 0) continue;
            /* This colour's six bit column pattern across its span. */
            for (int x = first[c]; x <= last[c]; x++) column_bits[x] = 0;
            for (int dy = 0; dy < rows; dy++) {
                const uint8_t *row = sixel->index + (size_t)(y0 + dy) * (size_t)image->width;
                for (int x = first[c]; x <= last[c]; x++)
                    if (row[x] == c) column_bits[x] |= (uint8_t)(1 << dy);
            }
            status = put_number(sixel, "#", c);
            if (status == SIXEL_OK && first[c] > 0) status = put_run(sixel, 0, first[c]);
            int six = -1;
            long run = 0;
            for (int x = first[c]; x <= last[c] && status == SIXEL_OK; x++) {
                if (column_bits[x] == six) {
                    run++;
                    continue;
                }
                status = put_run(sixel, six, run);
                six = column_bits[x];
                run = 1;
            }
            if (status == SIXEL_OK) status = put_run(sixel, six, run);
            if (status == SIXEL_OK) status = put(sixel, "$", 1); /* Back to the left. */
        }
        if (status == SIXEL_OK) status = put(sixel, "-", 1); /* Next band. */
    }
    if (status == SIXEL_OK) status = put(sixel, "\033\\", 2);
    free(first);
    free(last);
    free(column_bits);
    return status;
}
