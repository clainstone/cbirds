#include "../gif.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
 * A GIF reader, written here from the specification rather than borrowed from
 * the encoder, because an encoder checked against its own assumptions is not
 * checked at all. It handles exactly what gif.c emits: one global table, no
 * interlacing, no local tables.
 */
typedef struct {
    int width, height, frames, loops, delay;
    uint8_t palette[256][3];
    uint8_t *indices; /* Of the last frame read. */
    size_t pixels;
} reading_t;

static int read_lzw(const uint8_t *data, size_t length, int min_code, uint8_t *out,
                    size_t expected) {
    enum { MAX = 4096 };
    static uint8_t entry[MAX][64];
    static int entry_length[MAX];
    int clear = 1 << min_code, end = clear + 1;
    int width = min_code + 1, next = clear + 2, previous = -1;
    size_t at = 0, bit = 0, total = length * 8;

    for (int i = 0; i < clear; i++) {
        entry[i][0] = (uint8_t)i;
        entry_length[i] = 1;
    }
    while (bit + (size_t)width <= total) {
        size_t byte = bit / 8;
        int offset = (int)(bit % 8);
        uint32_t chunk = (uint32_t)data[byte];
        if (byte + 1 < length) chunk |= (uint32_t)data[byte + 1] << 8;
        if (byte + 2 < length) chunk |= (uint32_t)data[byte + 2] << 16;
        int code = (int)((chunk >> offset) & (uint32_t)((1 << width) - 1));
        bit += (size_t)width;

        if (code == clear) {
            width = min_code + 1;
            next = clear + 2;
            previous = -1;
            continue;
        }
        if (code == end) break;

        const uint8_t *source;
        int source_length;
        if (code < next && (code < clear || entry_length[code] > 0)) {
            source = entry[code];
            source_length = entry_length[code];
        } else if (previous >= 0) {
            source = entry[previous];
            source_length = entry_length[previous];
        } else {
            return 0;
        }
        if (at + (size_t)source_length > expected) return 0;
        memcpy(out + at, source, (size_t)source_length);
        size_t written_at = at;
        at += (size_t)source_length;
        /* The self referring code repeats its own first byte. */
        if (code >= next && previous >= 0) {
            if (at >= expected) return 0;
            out[at++] = source[0];
        }

        if (previous >= 0 && next < MAX) {
            int take = entry_length[previous];
            if (take > 63) take = 63;
            memcpy(entry[next], entry[previous], (size_t)take);
            entry[next][take] = out[written_at];
            entry_length[next] = take + 1;
            next++;
            if (next == (1 << width) && width < 12) width++;
        }
        previous = code < next ? code : next - 1;
        if (code >= clear + 2 && code >= next) previous = next - 1;
    }
    return at == expected;
}

static int read_gif(const char *path, reading_t *out) {
    static uint8_t file[1 << 22];
    FILE *f = fopen(path, "rb");
    if (f == NULL) return 0;
    size_t length = fread(file, 1, sizeof(file), f);
    fclose(f);
    if (length < 14 || memcmp(file, "GIF89a", 6) != 0) return 0;

    memset(out, 0, sizeof(*out));
    out->loops = -1;
    out->width = file[6] | (file[7] << 8);
    out->height = file[8] | (file[9] << 8);
    if (!(file[10] & 0x80)) return 0;
    int colours = 2 << (file[10] & 7);
    if (colours != 256) return 0;
    memcpy(out->palette, file + 13, 768);

    out->pixels = (size_t)out->width * (size_t)out->height;
    out->indices = (uint8_t *)malloc(out->pixels);
    if (out->indices == NULL) return 0;

    size_t at = 13 + 768;
    static uint8_t payload[1 << 22];
    while (at < length && file[at] != 0x3b) {
        if (file[at] == 0x21) {
            int label = file[at + 1];
            at += 2;
            if (label == 0xf9) {
                out->delay = file[at + 2] | (file[at + 3] << 8);
                at += 1 + file[at];
                at += 1;
            } else if (label == 0xff) {
                size_t n = file[at];
                int netscape = memcmp(file + at + 1, "NETSCAPE", 8) == 0;
                at += 1 + n;
                if (netscape) out->loops = file[at + 2] | (file[at + 3] << 8);
                while (file[at]) at += 1 + file[at];
                at++;
            } else {
                while (file[at]) at += 1 + file[at];
                at++;
            }
            continue;
        }
        if (file[at] != 0x2c) return 0;
        at += 10;
        int min_code = file[at++];
        size_t payload_length = 0;
        while (file[at]) {
            memcpy(payload + payload_length, file + at + 1, file[at]);
            payload_length += file[at];
            at += 1 + file[at];
        }
        at++;
        if (!read_lzw(payload, payload_length, min_code, out->indices, out->pixels)) return 0;
        out->frames++;
    }
    return 1;
}

static void paint(png_image_t *frame, int step) {
    for (int y = 0; y < frame->height; y++)
        for (int x = 0; x < frame->width; x++) {
            uint8_t *p = frame->pixels + ((size_t)y * (size_t)frame->width + (size_t)x) * 4;
            int on = ((x + step * 3) / 6 + y / 5) % 2;
            p[0] = on ? 240 : 20;
            p[1] = on ? 90 : 20;
            p[2] = on ? 40 : 30;
            p[3] = 255;
        }
}

static void test_round_trip(void) {
    enum { W = 64, H = 40, N = 6 };
    const char *path = "/tmp/cbirds_gif_test.gif";
    gif_writer_t *writer = NULL;
    png_image_t frame = {0, 0, NULL};
    reading_t read;
    size_t bytes = 0;
    int frames = 0;

    assert(gif_open(&writer, path, W, H, 5) == GIF_OK);
    assert(png_image_alloc(&frame, W, H) == PNG_OK);
    for (int f = 0; f < N; f++) {
        paint(&frame, f);
        assert(gif_add_frame(writer, &frame) == GIF_OK);
    }
    assert(gif_close(writer, &bytes, &frames) == GIF_OK);
    assert(frames == N && bytes > 0);

    /* Read it back with a reader written from the spec, not from the encoder. */
    assert(read_gif(path, &read));
    assert(read.width == W && read.height == H);
    assert(read.frames == N);
    assert(read.delay == 5);
    assert(read.loops == 0); /* Forever. */

    /* The last frame's pixels, through the palette, must be what went in: two
     * colours in, two colours out, in the right places. */
    paint(&frame, N - 1);
    for (size_t i = 0; i < read.pixels; i++) {
        const uint8_t *want = frame.pixels + i * 4;
        const uint8_t *got = read.palette[read.indices[i]];
        for (int c = 0; c < 3; c++) assert(got[c] > want[c] - 8 && got[c] < want[c] + 8);
    }
    free(read.indices);
    png_image_free(&frame);
    remove(path);
}

static void test_refusals(void) {
    gif_writer_t *writer = NULL;
    png_image_t frame = {0, 0, NULL};

    assert(gif_open(NULL, "/tmp/x.gif", 4, 4, 5) == GIF_ERR_ARGUMENT);
    assert(gif_open(&writer, NULL, 4, 4, 5) == GIF_ERR_ARGUMENT);
    assert(gif_open(&writer, "/tmp/x.gif", 0, 4, 5) == GIF_ERR_ARGUMENT);
    assert(gif_open(&writer, "/nowhere/at/all/x.gif", 4, 4, 5) == GIF_ERR_IO);

    assert(gif_open(&writer, "/tmp/cbirds_gif_refuse.gif", 8, 8, 5) == GIF_OK);
    assert(gif_add_frame(writer, NULL) == GIF_ERR_ARGUMENT);
    /* A frame of the wrong size is not this animation's frame. */
    assert(png_image_alloc(&frame, 9, 8) == PNG_OK);
    assert(gif_add_frame(writer, &frame) == GIF_ERR_ARGUMENT);
    png_image_free(&frame);
    assert(gif_close(writer, NULL, NULL) == GIF_OK);
    remove("/tmp/cbirds_gif_refuse.gif");

    for (int s = GIF_OK; s <= GIF_ERR_IO; s++) {
        const char *text = gif_status_string((gif_status_t)s);
        assert(text != NULL && strcmp(text, "unknown error") != 0);
    }
}

/* Long runs of one colour are what this program's frames are made of, so they
 * had better compress. */
static void test_flat_frames_compress(void) {
    enum { W = 320, H = 200, N = 4 };
    const char *path = "/tmp/cbirds_gif_flat.gif";
    gif_writer_t *writer = NULL;
    png_image_t frame = {0, 0, NULL};
    size_t bytes = 0;

    assert(gif_open(&writer, path, W, H, 4) == GIF_OK);
    assert(png_image_alloc(&frame, W, H) == PNG_OK);
    for (size_t i = 0; i < (size_t)W * H; i++) {
        frame.pixels[i * 4 + 0] = 18;
        frame.pixels[i * 4 + 1] = 18;
        frame.pixels[i * 4 + 2] = 24;
        frame.pixels[i * 4 + 3] = 255;
    }
    for (int f = 0; f < N; f++) assert(gif_add_frame(writer, &frame) == GIF_OK);
    assert(gif_close(writer, &bytes, NULL) == GIF_OK);
    /* Four flat frames of 64000 pixels each, in well under a tenth of that. */
    assert(bytes < (size_t)W * H / 10);
    png_image_free(&frame);
    remove(path);
}

int main(void) {
    test_round_trip();
    test_refusals();
    test_flat_frames_compress();
    return 0;
}
