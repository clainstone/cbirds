/*
 * gifpeek: turn frames of a GIF into PNG files, so that something which can
 * look at a PNG but cannot play a GIF can see what an animation looks like.
 *
 * Build, from this directory:
 *   cc -std=c99 -O2 -I../baseline gifpeek.c ../baseline/png.c ../baseline/font.c -lm -o gifpeek
 *
 * The file is read twice. The first pass walks the blocks and decodes every
 * frame's pixel data without keeping it, which is what makes the frame count
 * and the delays trustworthy before anything is drawn: a frame counts only if
 * all of it was read. The second pass composes the frames in order onto a
 * canvas, as a player would, and copies out the ones that were asked for.
 */

#include <ctype.h>
#include <limits.h>
#include <math.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "font.h"
#include "png.h"

enum {
    MAX_FILE_BYTES = 1 << 30,
    /* The PNG encoder's own limits, repeated so a refusal can say why. */
    MAX_SIDE = 16384,
    MAX_PIXELS = 1 << 26,
    LZW_CODES = 4096,
    SHEET_GAP = 4,
    LABEL_SCALE = 2,
    LABEL_PAD = 2
};

static const uint8_t GAP_GREY[3] = {80, 80, 80};

/*================================= Utilities ===============================*/

static void die(int status, const char *fmt, ...) {
    va_list args;
    fputs("gifpeek: ", stderr);
    va_start(args, fmt);
    vfprintf(stderr, fmt, args);
    va_end(args);
    fputc('\n', stderr);
    exit(status);
}

static void *must_alloc(size_t count, size_t size, int zeroed) {
    void *p = NULL;
    if (size != 0 && count > (size_t)-1 / size) die(1, "out of memory");
    p = zeroed ? calloc(count ? count : 1, size ? size : 1) : malloc(count * size + 1);
    if (p == NULL) die(1, "out of memory");
    return p;
}

/* Digits only: no sign, no blanks, no overflow. */
static int parse_uint(const char **cursor, long *value) {
    const char *p = *cursor;
    long v = 0;
    if (!isdigit((unsigned char)*p)) return 0;
    while (isdigit((unsigned char)*p)) {
        if (v > (LONG_MAX - 9) / 10) return 0;
        v = v * 10 + (*p - '0');
        p++;
    }
    *cursor = p;
    *value = v;
    return 1;
}

/* A plain decimal number: strtod would also take blanks, nan, inf and hex. */
static int parse_number(const char **cursor, double *value) {
    const char *p = *cursor;
    char *end = NULL;
    double v;
    if (!isdigit((unsigned char)*p) && *p != '.') return 0;
    for (const char *c = p; *c != '\0' && *c != ','; c++)
        if (!(isdigit((unsigned char)*c) || *c == '.' || *c == 'e' || *c == 'E' || *c == '+' ||
              *c == '-'))
            return 0;
    v = strtod(p, &end);
    if (end == p || !isfinite(v)) return 0;
    *cursor = end;
    *value = v;
    return 1;
}

/*=============================== The GIF file ==============================*/

typedef struct {
    int x, y, w, h;
    int interlaced;
    int delay;       /* Hundredths of a second. */
    int disposal;    /* 0 or 1 leave it, 2 clear it, 3 put back what was there. */
    int transparent; /* Palette index left alone, or -1. */
    int pal_size;    /* Entries in the frame's own table, 0 for none. */
    size_t pal_at;
    size_t lzw_at; /* First byte after the minimum code size. */
    int min_code;
    long start; /* Hundredths of a second since the first frame began. */
} frame_t;

typedef struct {
    uint8_t *data;
    size_t size;
    int width, height;
    int background;
    int gct_size; /* Entries in the global table, 0 for none. */
    size_t gct_at;
    frame_t *frames;
    int count;
    int cap;
    long total;     /* All the delays, in hundredths. */
    char stop[200]; /* Why reading ended before the trailer, or empty. */
} gif_t;

static void note_stop(gif_t *g, const char *fmt, ...) {
    va_list args;
    if (g->stop[0] != '\0') return;
    va_start(args, fmt);
    vsnprintf(g->stop, sizeof(g->stop), fmt, args);
    va_end(args);
}

static int u16(const uint8_t *p) {
    return p[0] | (p[1] << 8);
}

static void load_file(const char *path, gif_t *g) {
    FILE *f = fopen(path, "rb");
    size_t cap = 1 << 16, len = 0;
    uint8_t *buf;
    if (f == NULL) die(1, "cannot open %s", path);
    buf = (uint8_t *)must_alloc(cap, 1, 0);
    for (;;) {
        size_t got;
        if (len == cap) {
            uint8_t *bigger;
            if (cap >= MAX_FILE_BYTES) die(1, "%s is too large", path);
            cap *= 2;
            bigger = (uint8_t *)realloc(buf, cap);
            if (bigger == NULL) die(1, "out of memory");
            buf = bigger;
        }
        got = fread(buf + len, 1, cap - len, f);
        len += got;
        if (got == 0) break;
    }
    if (ferror(f)) die(1, "cannot read %s", path);
    fclose(f);
    g->data = buf;
    g->size = len;
}

/*================================== LZW ====================================*/

typedef struct {
    const uint8_t *d;
    size_t size;
    size_t pos;       /* Next byte to hand out. */
    size_t block_end; /* End of the current sub-block's data. */
    int done;         /* The terminator, or the end of the file, was reached. */
    int cut;          /* The end of the file came first. */
} blocks_t;

/* One byte of the image data, hopping from sub-block to sub-block. */
static int next_data_byte(blocks_t *b) {
    while (b->pos >= b->block_end) {
        size_t n;
        if (b->done) return -1;
        if (b->pos >= b->size) {
            b->done = b->cut = 1;
            return -1;
        }
        n = b->d[b->pos++];
        if (n == 0) {
            b->done = 1;
            return -1;
        }
        if (b->size - b->pos < n) {
            b->block_end = b->size; /* Hand out what is there, then report it cut. */
            b->cut = 1;
        } else {
            b->block_end = b->pos + n;
        }
        if (b->pos >= b->block_end) {
            b->done = 1;
            return -1;
        }
    }
    return b->d[b->pos++];
}

/* Moves past whatever is left of the sub-blocks. False if the file ends first. */
static int finish_blocks(blocks_t *b) {
    if (b->done) return !b->cut;
    if (b->cut) return 0;
    b->pos = b->block_end;
    for (;;) {
        size_t n;
        if (b->pos >= b->size) return 0;
        n = b->d[b->pos++];
        if (n == 0) return 1;
        if (b->size - b->pos < n) return 0;
        b->pos += n;
    }
}

typedef enum { LZW_OK, LZW_CUT, LZW_BAD, LZW_SHORT } lzw_status_t;

/*
 * Decodes one frame's pixel indices into out[0..pixels), in the order the file
 * holds them (interlacing is undone later). Stops as soon as the frame is
 * full, whatever the stream still says. On LZW_OK *after is the offset of the
 * byte past the frame's terminator block.
 */
static lzw_status_t lzw_decode(const gif_t *g, size_t at, int min_code, uint8_t *out, size_t pixels,
                               size_t *after) {
    uint16_t prefix[LZW_CODES];
    uint8_t suffix[LZW_CODES];
    uint8_t stack[LZW_CODES + 1];
    blocks_t b;
    int clear = 1 << min_code, end_code = clear + 1;
    int next = clear + 2, width = min_code + 1;
    int prev = -1, first = 0;
    uint32_t acc = 0;
    int have = 0;
    size_t filled = 0;

    b.d = g->data;
    b.size = g->size;
    b.pos = b.block_end = at;
    b.done = b.cut = 0;
    for (int i = 0; i < clear; i++) {
        suffix[i] = (uint8_t)i;
        prefix[i] = 0;
    }

    while (filled < pixels) {
        int code, cur, sp = 0;
        while (have < width) {
            int byte = next_data_byte(&b);
            if (byte < 0) return b.cut ? LZW_CUT : LZW_SHORT;
            acc |= (uint32_t)byte << have;
            have += 8;
        }
        code = (int)(acc & ((1u << width) - 1));
        acc >>= width;
        have -= width;

        if (code == clear) {
            next = clear + 2;
            width = min_code + 1;
            prev = -1;
            continue;
        }
        if (code == end_code) return LZW_SHORT;

        if (prev < 0) {
            if (code >= clear) return LZW_BAD; /* Only a pixel can follow a clear. */
            out[filled++] = (uint8_t)code;
            first = code;
            prev = code;
            continue;
        }

        cur = code;
        if (cur >= next) {
            if (cur > next) return LZW_BAD;
            stack[sp++] = (uint8_t)first; /* The code being defined right now. */
            cur = prev;
        }
        while (cur >= clear) {
            stack[sp++] = suffix[cur];
            cur = prefix[cur];
        }
        stack[sp++] = (uint8_t)cur;
        first = cur;
        while (sp > 0 && filled < pixels) out[filled++] = stack[--sp];

        if (next < LZW_CODES) {
            prefix[next] = (uint16_t)prev;
            suffix[next] = (uint8_t)first;
            next++;
            if (next == (1 << width) && width < 12) width++;
        }
        prev = code;
    }
    if (!finish_blocks(&b)) return LZW_CUT;
    *after = b.pos;
    return LZW_OK;
}

/*========================= Pass one: the structure =========================*/

static int skip_blocks(const gif_t *g, size_t *pos) {
    for (;;) {
        size_t n;
        if (*pos >= g->size) return 0;
        n = g->data[(*pos)++];
        if (n == 0) return 1;
        if (g->size - *pos < n) return 0;
        *pos += n;
    }
}

static void add_frame(gif_t *g, const frame_t *f) {
    if (g->count == g->cap) {
        int cap = g->cap ? g->cap * 2 : 64;
        frame_t *bigger = (frame_t *)realloc(g->frames, (size_t)cap * sizeof(*bigger));
        if (bigger == NULL) die(1, "out of memory");
        g->frames = bigger;
        g->cap = cap;
    }
    g->frames[g->count] = *f;
    g->frames[g->count].start = g->total;
    g->total += f->delay;
    g->count++;
}

/*
 * Reads the header and walks every block. Frames are added as long as they are
 * whole; the first thing that is not (a cut, a bad code, a stray byte) ends
 * the walk with a note of why, and what was read before it stands. Returns
 * NULL, or a message when not even a header could be read.
 */
static const char *scan_gif(gif_t *g) {
    const uint8_t *d = g->data;
    size_t pos, pixels_max = (size_t)MAX_PIXELS, indices_cap = 0;
    uint8_t *indices = NULL;
    int delay = 0, disposal = 0, transparent = -1;

    if (g->size < 6 || memcmp(d, "GIF8", 4) != 0 || (d[4] != '7' && d[4] != '9') || d[5] != 'a')
        return "not a GIF file";
    if (g->size < 13) return "the header is cut short";
    g->width = u16(d + 6);
    g->height = u16(d + 8);
    g->background = d[11];
    if (g->width == 0 || g->height == 0) return "the screen has no size";
    if ((size_t)g->width * (size_t)g->height > pixels_max) return "the screen is too large";
    pos = 13;
    if (d[10] & 0x80) {
        size_t entries = (size_t)2 << (d[10] & 7);
        if (g->size - pos < entries * 3) return "the global colour table is cut short";
        g->gct_size = (int)entries;
        g->gct_at = pos;
        pos += entries * 3;
    }

    for (;;) {
        int block;
        if (pos >= g->size) {
            note_stop(g, "the file ends without a trailer");
            break;
        }
        block = d[pos++];
        if (block == 0x3b) break;

        if (block == 0x21) {
            int label;
            if (pos >= g->size) {
                note_stop(g, "the file is cut short in an extension after %d frames", g->count);
                break;
            }
            label = d[pos++];
            if (label == 0xf9 && pos < g->size && d[pos] >= 4 && g->size - pos >= 5) {
                int flags = d[pos + 1];
                delay = u16(d + pos + 2);
                transparent = (flags & 1) ? d[pos + 4] : -1;
                disposal = (flags >> 2) & 7;
                if (disposal > 3) disposal = 0;
            }
            if (!skip_blocks(g, &pos)) {
                note_stop(g, "the file is cut short in an extension after %d frames", g->count);
                break;
            }
            continue;
        }

        if (block == 0x2c) {
            frame_t f;
            size_t pixels, after = 0;
            lzw_status_t status;
            int flags;
            if (g->size - pos < 9) {
                note_stop(g, "the file is cut short in the header of frame %d", g->count);
                break;
            }
            memset(&f, 0, sizeof(f));
            f.x = u16(d + pos);
            f.y = u16(d + pos + 2);
            f.w = u16(d + pos + 4);
            f.h = u16(d + pos + 6);
            flags = d[pos + 8];
            pos += 9;
            f.interlaced = (flags & 0x40) != 0;
            if (flags & 0x80) {
                size_t entries = (size_t)2 << (flags & 7);
                if (g->size - pos < entries * 3) {
                    note_stop(g, "the file is cut short in the colour table of frame %d", g->count);
                    break;
                }
                f.pal_size = (int)entries;
                f.pal_at = pos;
                pos += entries * 3;
            }
            if (pos >= g->size) {
                note_stop(g, "the file is cut short at the data of frame %d", g->count);
                break;
            }
            f.min_code = d[pos++];
            f.lzw_at = pos;
            f.delay = delay;
            f.disposal = disposal;
            f.transparent = transparent;
            delay = 0;
            disposal = 0;
            transparent = -1;

            if (f.min_code < 1 || f.min_code > 8) {
                note_stop(g, "frame %d has an invalid LZW code size (%d)", g->count, f.min_code);
                break;
            }
            pixels = (size_t)f.w * (size_t)f.h;
            if (pixels > pixels_max) {
                note_stop(g, "frame %d is too large (%dx%d)", g->count, f.w, f.h);
                break;
            }
            if (pixels > indices_cap) {
                free(indices);
                indices = (uint8_t *)must_alloc(pixels, 1, 0);
                indices_cap = pixels;
            }
            status = lzw_decode(g, f.lzw_at, f.min_code, indices, pixels, &after);
            if (status != LZW_OK) {
                note_stop(g,
                          status == LZW_CUT   ? "the file is cut short in frame %d"
                          : status == LZW_BAD ? "frame %d holds a corrupt LZW stream"
                                              : "frame %d has less image data than its size",
                          g->count);
                break;
            }
            add_frame(g, &f);
            pos = after;
            continue;
        }

        note_stop(g, "unexpected byte 0x%02x at offset %lu after %d frames", block,
                  (unsigned long)(pos - 1), g->count);
        break;
    }
    free(indices);
    return NULL;
}

/*========================= Pass two: composing frames ======================*/

typedef struct {
    const gif_t *g;
    uint8_t *canvas; /* RGBA, alpha 0 where nothing has been drawn. */
    uint8_t *saved;  /* The canvas as it was before a frame that is to be undone. */
    uint8_t *indices;
    size_t indices_cap;
    int next; /* The frame the next step composes. */
    int saved_valid;
} player_t;

static void player_init(player_t *p, const gif_t *g) {
    memset(p, 0, sizeof(*p));
    p->g = g;
    p->canvas = (uint8_t *)must_alloc((size_t)g->width * (size_t)g->height, 4, 1);
}

static void player_free(player_t *p) {
    free(p->canvas);
    free(p->saved);
    free(p->indices);
}

/* Row i of the data in the file is row interlace_row(i) of the picture. */
static int interlace_row(int i, int h) {
    int n1 = (h + 7) / 8, n2 = (h + 3) / 8, n3 = (h + 1) / 4;
    if (i < n1) return i * 8;
    i -= n1;
    if (i < n2) return 4 + i * 8;
    i -= n2;
    if (i < n3) return 2 + i * 4;
    i -= n3;
    return 1 + i * 2;
}

static void clear_rect(player_t *p, const frame_t *f) {
    int cw = p->g->width, ch = p->g->height;
    for (int y = f->y; y < f->y + f->h && y < ch; y++)
        for (int x = f->x; x < f->x + f->w && x < cw; x++)
            memset(p->canvas + ((size_t)y * (size_t)cw + (size_t)x) * 4, 0, 4);
}

/* Composes frame p->next. False if its data no longer decodes, which cannot
 * happen to a frame the first pass accepted. */
static int player_step(player_t *p) {
    const gif_t *g = p->g;
    const frame_t *f = &g->frames[p->next];
    const uint8_t *pal = NULL;
    int pal_size = 0, cw = g->width, ch = g->height;
    size_t pixels = (size_t)f->w * (size_t)f->h, after = 0;

    if (p->next > 0) {
        const frame_t *last = &g->frames[p->next - 1];
        if (last->disposal == 2) clear_rect(p, last);
        if (last->disposal == 3 && p->saved_valid)
            memcpy(p->canvas, p->saved, (size_t)cw * (size_t)ch * 4);
    }
    if (f->disposal == 3) {
        if (p->saved == NULL) p->saved = (uint8_t *)must_alloc((size_t)cw * (size_t)ch, 4, 0);
        memcpy(p->saved, p->canvas, (size_t)cw * (size_t)ch * 4);
        p->saved_valid = 1;
    }

    if (pixels > p->indices_cap) {
        free(p->indices);
        p->indices = (uint8_t *)must_alloc(pixels, 1, 0);
        p->indices_cap = pixels;
    }
    if (lzw_decode(g, f->lzw_at, f->min_code, p->indices, pixels, &after) != LZW_OK) return 0;

    if (f->pal_size > 0) {
        pal = g->data + f->pal_at;
        pal_size = f->pal_size;
    } else if (g->gct_size > 0) {
        pal = g->data + g->gct_at;
        pal_size = g->gct_size;
    }

    for (int row = 0; row < f->h; row++) {
        int y = f->y + (f->interlaced ? interlace_row(row, f->h) : row);
        const uint8_t *src = p->indices + (size_t)row * (size_t)f->w;
        uint8_t *dst;
        if (y >= ch) continue;
        dst = p->canvas + ((size_t)y * (size_t)cw + (size_t)f->x) * 4;
        for (int col = 0; col < f->w && f->x + col < cw; col++, dst += 4) {
            int index = src[col];
            if (index == f->transparent) continue;
            if (pal == NULL) {
                dst[0] = dst[1] = dst[2] = (uint8_t)index;
            } else if (index < pal_size) {
                memcpy(dst, pal + index * 3, 3);
            } else {
                dst[0] = dst[1] = dst[2] = 0;
            }
            dst[3] = 255;
        }
    }
    p->next++;
    return 1;
}

/*============================ Drawing the output ===========================*/

/* Nearest neighbour: the source column or row each output one takes from. */
static int *make_map(int out, int in) {
    int *map = (int *)must_alloc((size_t)out, sizeof(int), 0);
    for (int i = 0; i < out; i++) {
        long long at = ((2LL * i + 1) * in) / (2LL * out);
        map[i] = at >= in ? in - 1 : (int)at;
    }
    return map;
}

typedef struct {
    int sw, sh;       /* One scaled frame. */
    int *xmap, *ymap; /* Source column and row per output column and row. */
    uint8_t backdrop[3];
    int label;
} drawing_t;

/* Pixels nothing was drawn on show the GIF's background colour. */
static void backdrop_of(const gif_t *g, uint8_t out[3]) {
    out[0] = out[1] = out[2] = 0;
    if (g->gct_size > 0 && g->background < g->gct_size)
        memcpy(out, g->data + g->gct_at + g->background * 3, 3);
}

static void stamp_label(uint8_t *dst, size_t stride, int w, int h, const char *text) {
    int box_w = font_text_width(text) * LABEL_SCALE + 2 * LABEL_PAD;
    int box_h = FONT_HEIGHT * LABEL_SCALE + 2 * LABEL_PAD;
    int pen = LABEL_PAD;
    for (int y = 0; y < box_h && y < h; y++)
        for (int x = 0; x < box_w && x < w; x++) {
            uint8_t *px = dst + ((size_t)y * stride + (size_t)x) * 4;
            px[0] = px[1] = px[2] = 0;
            px[3] = 255;
        }
    for (const char *c = text; *c != '\0'; c++) {
        const char *glyph = font_glyph(*c);
        if (glyph == NULL) continue;
        for (int gy = 0; gy < FONT_HEIGHT; gy++)
            for (int gx = 0; gx < FONT_WIDTH; gx++) {
                if (glyph[gy * FONT_WIDTH + gx] != '#') continue;
                for (int sy = 0; sy < LABEL_SCALE; sy++)
                    for (int sx = 0; sx < LABEL_SCALE; sx++) {
                        int x = pen + gx * LABEL_SCALE + sx;
                        int y = LABEL_PAD + gy * LABEL_SCALE + sy;
                        uint8_t *px;
                        if (x >= w || y >= h) continue;
                        px = dst + ((size_t)y * stride + (size_t)x) * 4;
                        px[0] = px[1] = px[2] = px[3] = 255;
                    }
            }
        pen += FONT_ADVANCE * LABEL_SCALE;
    }
}

/* Draws the player's canvas, scaled, into a slot at dst; stride is in pixels. */
static void draw_frame(const player_t *p, const drawing_t *dr, int index, uint8_t *dst,
                       size_t stride) {
    const gif_t *g = p->g;
    for (int y = 0; y < dr->sh; y++) {
        const uint8_t *row = p->canvas + (size_t)dr->ymap[y] * (size_t)g->width * 4;
        uint8_t *out = dst + (size_t)y * stride * 4;
        for (int x = 0; x < dr->sw; x++, out += 4) {
            const uint8_t *px = row + (size_t)dr->xmap[x] * 4;
            if (px[3] != 0)
                memcpy(out, px, 3);
            else
                memcpy(out, dr->backdrop, 3);
            out[3] = 255;
        }
    }
    if (dr->label) {
        char text[64];
        long start = g->frames[index].start;
        snprintf(text, sizeof(text), "F%d %ld.%02ldS", index, start / 100, start % 100);
        stamp_label(dst, stride, dr->sw, dr->sh, text);
    }
}

/*============================ Which frames to show =========================*/

static int *resolve_spec(const char *spec, const gif_t *g, int *count, char *err, size_t err_size) {
    int n = g->count, k = 0;
    int *pick = NULL;
    const char *p;

#define FAIL(...)                             \
    do {                                      \
        snprintf(err, err_size, __VA_ARGS__); \
        free(pick);                           \
        return NULL;                          \
    } while (0)

    if (strncmp(spec, "every:", 6) == 0) {
        long step;
        p = spec + 6;
        if (!parse_uint(&p, &step) || *p != '\0' || step < 1)
            FAIL("every:N wants a whole number of at least 1, got \"%s\"", spec + 6);
        pick = (int *)must_alloc((size_t)(n / step) + 1, sizeof(int), 0);
        for (long i = 0; i < n; i += step) pick[k++] = (int)i;
    } else if (strncmp(spec, "spread:", 7) == 0) {
        long want;
        p = spec + 7;
        if (!parse_uint(&p, &want) || *p != '\0' || want < 1)
            FAIL("spread:K wants a whole number of at least 1, got \"%s\"", spec + 7);
        if (want > n) {
            fprintf(stderr,
                    "gifpeek: spread:%ld asks for more frames than the %d read, using all\n", want,
                    n);
            want = n;
        }
        pick = (int *)must_alloc((size_t)want, sizeof(int), 0);
        for (long j = 0; j < want; j++)
            pick[k++] = want == 1 ? 0 : (int)((j * (n - 1) * 2 + (want - 1)) / (2 * (want - 1)));
    } else if (strncmp(spec, "range:", 6) == 0) {
        long a, b;
        p = spec + 6;
        if (!parse_uint(&p, &a) || *p != '-')
            FAIL("range:A-B wants two frame numbers, got \"%s\"", spec + 6);
        p++;
        if (!parse_uint(&p, &b) || *p != '\0')
            FAIL("range:A-B wants two frame numbers, got \"%s\"", spec + 6);
        if (a > b) FAIL("range:%ld-%ld runs backwards", a, b);
        if (b >= n)
            FAIL("range:%ld-%ld is past the last frame; the file has %d frames (0-%d)", a, b, n,
                 n - 1);
        pick = (int *)must_alloc((size_t)(b - a) + 1, sizeof(int), 0);
        for (long i = a; i <= b; i++) pick[k++] = (int)i;
    } else if (strncmp(spec, "times:", 6) == 0) {
        int commas = 1;
        for (p = spec + 6; *p != '\0'; p++) commas += *p == ',';
        pick = (int *)must_alloc((size_t)commas, sizeof(int), 0);
        p = spec + 6;
        for (;;) {
            double t;
            int at = 0;
            if (!parse_number(&p, &t) || t < 0 || (*p != ',' && *p != '\0'))
                FAIL("times: wants seconds separated by commas, e.g. times:1,2.5,4");
            if (t * 100 > (double)g->total + 1e-6)
                FAIL("time %g is past the end of the GIF (%ld.%02ld s)", t, g->total / 100,
                     g->total % 100);
            while (at + 1 < n && (double)g->frames[at + 1].start <= t * 100 + 1e-6) at++;
            pick[k++] = at;
            if (*p == '\0') break;
            p++;
        }
    } else {
        int commas = 1;
        for (p = spec; *p != '\0'; p++) commas += *p == ',';
        pick = (int *)must_alloc((size_t)commas, sizeof(int), 0);
        p = spec;
        for (;;) {
            long v;
            if (!parse_uint(&p, &v) || (*p != ',' && *p != '\0'))
                FAIL(
                    "cannot read SPEC \"%s\": use 0,25,50 or every:N, times:S,S, spread:K, "
                    "range:A-B",
                    spec);
            if (v >= n)
                FAIL("frame %ld is past the last frame; the file has %d frames (0-%d)", v, n,
                     n - 1);
            pick[k++] = (int)v;
            if (*p == '\0') break;
            p++;
        }
    }
#undef FAIL
    *count = k;
    return pick;
}

/*============================ Output file names ============================*/

/* The end of a "%[flags][width]d" starting at s (which points at the '%'), or NULL. */
static const char *number_token(const char *s) {
    const char *p = s + 1;
    int digits = 0;
    while (*p == '-' || *p == '+' || *p == '0' || *p == ' ') p++;
    while (isdigit((unsigned char)*p)) {
        p++;
        digits++;
    }
    return (*p == 'd' && digits <= 3) ? p + 1 : NULL;
}

static int has_number(const char *pattern) {
    for (const char *p = pattern; *p != '\0'; p++) {
        if (*p != '%') continue;
        if (p[1] == '%') {
            p++;
            continue;
        }
        if (number_token(p) != NULL) return 1;
    }
    return 0;
}

/* The pattern with each %d filled in. Only used on a pattern that has_number. */
static int expand(const char *pattern, int number, char *out, size_t size) {
    size_t n = 0;
    for (const char *p = pattern; *p != '\0';) {
        const char *end;
        char piece[64];
        size_t len;
        if (*p == '%' && p[1] == '%') {
            piece[0] = '%';
            len = 1;
            p += 2;
        } else if (*p == '%' && (end = number_token(p)) != NULL) {
            char format[16];
            size_t flen = (size_t)(end - p);
            memcpy(format, p, flen);
            format[flen] = '\0';
            /* The format is "%", flags, up to three digits and "d": nothing else. */
            len = (size_t)snprintf(piece, sizeof(piece), format, number);
            p = end;
        } else {
            piece[0] = *p++;
            len = 1;
        }
        if (len >= sizeof(piece) || n + len >= size) return 0;
        memcpy(out + n, piece, len);
        n += len;
    }
    out[n] = '\0';
    return 1;
}

static void write_png(const char *path, const png_image_t *image) {
    uint8_t *bytes = NULL;
    size_t length = 0;
    FILE *f;
    png_status_t status = png_encode(image, &bytes, &length);
    if (status != PNG_OK) die(1, "cannot encode %s: %s", path, png_status_string(status));
    f = fopen(path, "wb");
    if (f == NULL) die(1, "cannot create %s", path);
    if (fwrite(bytes, 1, length, f) != length) die(1, "cannot write %s", path);
    if (fclose(f) != 0) die(1, "cannot write %s", path);
    free(bytes);
}

/*================================== Main ===================================*/

static void usage(FILE *to) {
    fputs(
        "usage: gifpeek --info IN.gif\n"
        "       gifpeek IN.gif OUT.png SPEC [--cols N] [--scale S] [--label]\n"
        "\n"
        "SPEC picks the frames:\n"
        "  0,25,50      frame numbers, from 0\n"
        "  every:N      every Nth frame\n"
        "  times:1,2.5  the frames showing at those seconds\n"
        "  spread:K     K frames spread from the first to the last\n"
        "  range:A-B    frames A to B, inclusive\n"
        "One frame is written as OUT.png; several make a contact sheet, --cols across\n"
        "(default 2). --scale S resizes each frame first (0.5 halves), --label stamps\n"
        "frame number and time on each. An OUT with %d in it (out-%03d.png) gets one\n"
        "file per frame instead, numbered by frame.\n",
        to);
}

static void print_info(const gif_t *g) {
    printf("width: %d\n", g->width);
    printf("height: %d\n", g->height);
    printf("frames: %d\n", g->count);
    printf("delay: %d (first frame, hundredths of a second)\n", g->frames[0].delay);
    printf("duration: %ld.%02ld s\n", g->total / 100, g->total % 100);
}

int main(int argc, char **argv) {
    const char *positional[4];
    int npos = 0, info = 0, label = 0;
    long cols = 2;
    double scale = 1;
    gif_t g;
    const char *fatal;

    for (int i = 1; i < argc; i++) {
        const char *a = argv[i];
        if (strcmp(a, "-h") == 0 || strcmp(a, "--help") == 0) {
            usage(stdout);
            return 0;
        } else if (strcmp(a, "--info") == 0) {
            info = 1;
        } else if (strcmp(a, "--label") == 0) {
            label = 1;
        } else if (strcmp(a, "--cols") == 0 || strcmp(a, "--scale") == 0) {
            const char *value = i + 1 < argc ? argv[++i] : NULL;
            const char *p = value;
            if (value == NULL) die(2, "%s needs a value", a);
            if (a[2] == 'c') {
                if (!parse_uint(&p, &cols) || *p != '\0' || cols < 1 || cols > 100000)
                    die(2, "--cols wants a whole number of at least 1, got \"%s\"", value);
            } else if (!parse_number(&p, &scale) || *p != '\0' || scale <= 0 || scale > 64) {
                die(2, "--scale wants a number above 0 and up to 64, got \"%s\"", value);
            }
        } else if (a[0] == '-' && a[1] == '-') {
            die(2, "unknown option %s (try --help)", a);
        } else {
            if (npos == 3) die(2, "too many arguments (try --help)");
            positional[npos++] = a;
        }
    }
    if (info ? npos != 1 : npos != 3) {
        usage(stderr);
        return 2;
    }

    memset(&g, 0, sizeof(g));
    load_file(positional[0], &g);
    fatal = scan_gif(&g);
    if (fatal != NULL) die(1, "%s: %s", positional[0], fatal);
    if (g.count == 0)
        die(1, "%s: no frames could be read%s%s%s", positional[0], g.stop[0] ? " (" : "", g.stop,
            g.stop[0] ? ")" : "");
    if (g.stop[0] != '\0')
        fprintf(stderr, "gifpeek: warning: %s: %s; %d frame%s read\n", positional[0], g.stop,
                g.count, g.count == 1 ? "" : "s");

    if (info) {
        print_info(&g);
    } else {
        const char *out_path = positional[1];
        char err[256], name[1024];
        int selected = 0, many_files = has_number(out_path);
        int *pick = resolve_spec(positional[2], &g, &selected, err, sizeof(err));
        int ncols, nrows, *needed;
        long long sheet_w, sheet_h;
        double fw, fh;
        drawing_t dr;
        png_image_t sheet;
        player_t player;
        int last = 0;

        if (pick == NULL) die(1, "%s", err);
        if (selected < 1) die(1, "SPEC selects no frames");

        fw = floor(g.width * scale + 0.5);
        fh = floor(g.height * scale + 0.5);
        if (fw < 1) fw = 1;
        if (fh < 1) fh = 1;
        if (fw > MAX_SIDE || fh > MAX_SIDE)
            die(1, "a frame would be %.0fx%.0f; use a smaller --scale", fw, fh);
        dr.sw = (int)fw;
        dr.sh = (int)fh;
        dr.label = label;

        ncols = many_files ? 1 : (int)(cols < selected ? cols : selected);
        nrows = many_files ? 1 : (selected + ncols - 1) / ncols;
        sheet_w = (long long)ncols * dr.sw + (long long)(ncols - 1) * SHEET_GAP;
        sheet_h = (long long)nrows * dr.sh + (long long)(nrows - 1) * SHEET_GAP;
        if (sheet_w > MAX_SIDE || sheet_h > MAX_SIDE || sheet_w * sheet_h > MAX_PIXELS)
            die(1,
                "the sheet would be %lldx%lld, over what a PNG here can hold; "
                "use a smaller --scale, more --cols or fewer frames",
                sheet_w, sheet_h);
        if (png_image_alloc(&sheet, (int)sheet_w, (int)sheet_h) != PNG_OK) die(1, "out of memory");
        for (size_t i = 0; i < (size_t)sheet_w * (size_t)sheet_h; i++) {
            memcpy(sheet.pixels + i * 4, GAP_GREY, 3);
            sheet.pixels[i * 4 + 3] = 255;
        }

        dr.xmap = make_map(dr.sw, g.width);
        dr.ymap = make_map(dr.sh, g.height);
        backdrop_of(&g, dr.backdrop);

        needed = (int *)must_alloc((size_t)g.count, sizeof(int), 1);
        for (int j = 0; j < selected; j++) {
            needed[pick[j]] = 1;
            if (pick[j] > last) last = pick[j];
        }

        player_init(&player, &g);
        for (int i = 0; i <= last; i++) {
            if (!player_step(&player)) die(1, "frame %d no longer decodes", i);
            if (!needed[i]) continue;
            if (many_files) {
                if (!expand(out_path, i, name, sizeof(name)))
                    die(1, "the file name for frame %d is too long", i);
                draw_frame(&player, &dr, i, sheet.pixels, (size_t)sheet_w);
                write_png(name, &sheet);
                printf("%s: frame %d, %dx%d\n", name, i, dr.sw, dr.sh);
                continue;
            }
            for (int j = 0; j < selected; j++) {
                size_t x, y;
                if (pick[j] != i) continue;
                x = (size_t)(j % ncols) * (size_t)(dr.sw + SHEET_GAP);
                y = (size_t)(j / ncols) * (size_t)(dr.sh + SHEET_GAP);
                draw_frame(&player, &dr, i, sheet.pixels + (y * (size_t)sheet_w + x) * 4,
                           (size_t)sheet_w);
            }
        }
        if (!many_files) {
            write_png(out_path, &sheet);
            printf("%s: %d frame%s, %lldx%lld\n", out_path, selected, selected == 1 ? "" : "s",
                   sheet_w, sheet_h);
        }

        player_free(&player);
        png_image_free(&sheet);
        free(dr.xmap);
        free(dr.ymap);
        free(needed);
        free(pick);
    }

    free(g.frames);
    free(g.data);
    return 0;
}
