#include "picture.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

enum {
    /* The picture is sampled on a grid no finer than this a side, whatever size it
     * lands at: sixty five thousand cells of a few pixels each are plenty to cut
     * into the four thousand pieces the most birds ever ask for, and the work is
     * done again whenever the window changes size. */
    WORK_MAX = 256,
    /* And its colours are counted on a copy this small: nine thousand pixels say
     * what a picture is made of as well as nine million do. */
    COLOUR_WORK_MAX = 96
};

picture_fit_t picture_fit(const png_image_t *image, double left, double top, double width,
                          double height) {
    picture_fit_t fit = {left, top, 0, 0, 0};
    if (image == NULL || image->width <= 0 || image->height <= 0 || width <= 0 || height <= 0)
        return fit;
    double scale = width / image->width;
    if (height / image->height < scale) scale = height / image->height;
    fit.scale = scale;
    fit.width = image->width * scale;
    fit.height = image->height * scale;
    fit.left = left + (width - fit.width) / 2;
    fit.top = top + (height - fit.height) / 2;
    return fit;
}

/* The same small hash as the sign's, for the same reason: a point's jitter is
 * a function of the seed and which point it is, and nothing else. */
static uint32_t mix(uint32_t x) {
    x ^= x >> 16;
    x *= 0x7feb352dU;
    x ^= x >> 15;
    x *= 0x846ca68bU;
    x ^= x >> 16;
    return x;
}

static double unit_of(uint32_t x) {
    return (double)(x >> 8) / 16777216.0;
}

/* A cell's place in the order along an axis, ties broken by its number, so that
 * the order is total and the cut is the same cut every time. */
static int before(int a, int b, int axis, int columns) {
    int key_a = axis == 0 ? a % columns : a / columns;
    int key_b = axis == 0 ? b % columns : b / columns;
    return key_a < key_b || (key_a == key_b && a < b);
}

/* Moves the cell that belongs at `nth` in the order to `nth`, with everything
 * before it in front and everything after behind: Hoare's selection, which is
 * all a cut in half needs and is linear where sorting is not. */
static void select_nth(int *cells, int lo, int hi, int nth, int axis, int columns) {
    int left = lo, right = hi - 1;
    while (left < right) {
        int pivot = cells[left + (right - left) / 2];
        int i = left, j = right;
        while (i <= j) {
            while (before(cells[i], pivot, axis, columns)) i++;
            while (before(pivot, cells[j], axis, columns)) j--;
            if (i <= j) {
                int swap = cells[i];
                cells[i] = cells[j];
                cells[j] = swap;
                i++;
                j--;
            }
        }
        if (nth <= j)
            right = j;
        else if (nth >= i)
            left = i;
        else
            break;
    }
}

typedef struct {
    const png_image_t *work;
    const picture_fit_t *fit;
    unsigned seed;
    picture_point_t *out;
} sampling_t;

/* One point for one piece of the picture: near its middle, off it by a share of
 * the piece's own size, and on a cell that is ink. */
static void pick(const sampling_t *sampling, const int *cells, int count, int piece) {
    int columns = sampling->work->width;
    double sum_x = 0, sum_y = 0;
    int low_x = columns, high_x = 0, low_y = sampling->work->height, high_y = 0;
    for (int i = 0; i < count; i++) {
        int x = cells[i] % columns, y = cells[i] / columns;
        sum_x += x + 0.5;
        sum_y += y + 0.5;
        if (x < low_x) low_x = x;
        if (x > high_x) high_x = x;
        if (y < low_y) low_y = y;
        if (y > high_y) high_y = y;
    }
    uint32_t hash = mix(sampling->seed * 0x9e3779b1U + (uint32_t)piece * 0x85ebca6bU + 1u);
    /* A seventh of the piece either way is enough that the points are not a lattice
     * and little enough that two neighbours are never close: at a third either way
     * the nearest two of four hundred points were a third of the spacing apart,
     * and at a seventh they are a half of it, while no point is further from its
     * nearest neighbour than a spacing and a twentieth. */
    double aim_x = sum_x / count + (unit_of(hash) - 0.5) * 0.3 * (high_x - low_x + 1);
    double aim_y = sum_y / count + (unit_of(mix(hash + 1u)) - 0.5) * 0.3 * (high_y - low_y + 1);
    int best = cells[0];
    double best_gap = 1e300;
    for (int i = 0; i < count; i++) {
        double dx = cells[i] % columns + 0.5 - aim_x, dy = cells[i] / columns + 0.5 - aim_y;
        double gap = dx * dx + dy * dy;
        if (gap < best_gap) {
            best_gap = gap;
            best = cells[i];
        }
    }
    int cell_x = best % columns, cell_y = best / columns;
    /* Somewhere in that cell, not its middle, so that two pieces that had to share
     * a cell (more points than cells) still do not share a place. */
    double at_x = cell_x + 0.5 + (unit_of(mix(hash + 2u)) - 0.5) * 0.9;
    double at_y = cell_y + 0.5 + (unit_of(mix(hash + 3u)) - 0.5) * 0.9;
    picture_point_t *point = &sampling->out[piece];
    point->x = sampling->fit->left + at_x / columns * sampling->fit->width;
    point->y = sampling->fit->top + at_y / sampling->work->height * sampling->fit->height;
    const uint8_t *pixel = sampling->work->pixels + (size_t)best * 4;
    memcpy(point->rgb, pixel, 3);
}

/* Cuts the cells in half along their longer side, in proportion to the pieces
 * each half is to make, until there is one piece to a cell range. */
static void divide(const sampling_t *sampling, int *cells, int lo, int hi, int first, int pieces) {
    if (pieces == 1) {
        pick(sampling, cells + lo, hi - lo, first);
        return;
    }
    int columns = sampling->work->width;
    int low_x = columns, high_x = 0, low_y = sampling->work->height, high_y = 0;
    for (int i = lo; i < hi; i++) {
        int x = cells[i] % columns, y = cells[i] / columns;
        if (x < low_x) low_x = x;
        if (x > high_x) high_x = x;
        if (y < low_y) low_y = y;
        if (y > high_y) high_y = y;
    }
    int axis = high_x - low_x >= high_y - low_y ? 0 : 1;
    int left_pieces = pieces / 2;
    /* At least a cell to each piece on both sides, which the caller made sure of
     * by never asking for more pieces than there are cells. */
    int split = lo + (int)((long)(hi - lo) * left_pieces / pieces);
    select_nth(cells, lo, hi, split, axis, columns);
    divide(sampling, cells, lo, split, first, left_pieces);
    divide(sampling, cells, split, hi, first + left_pieces, pieces - left_pieces);
}

int picture_sample(const png_image_t *image, const picture_fit_t *fit, int count, unsigned seed,
                   picture_point_t *out) {
    if (image == NULL || image->pixels == NULL || fit == NULL || out == NULL || count <= 0 ||
        fit->width <= 0 || fit->height <= 0)
        return 0;
    double longest = fit->width > fit->height ? fit->width : fit->height;
    double factor = longest > WORK_MAX ? WORK_MAX / longest : 1.0;
    int columns = (int)(fit->width * factor + 0.5), rows = (int)(fit->height * factor + 0.5);
    if (columns < 1) columns = 1;
    if (rows < 1) rows = 1;

    png_image_t work = {0, 0, NULL};
    if (png_resize(image, columns, rows, &work) != PNG_OK) return 0;
    int opaque = 0;
    for (int i = 0; i < columns * rows; i++) opaque += work.pixels[(size_t)i * 4 + 3] > 127;
    if (opaque == 0) {
        png_image_free(&work);
        return 0;
    }
    /* A cell for every piece: a picture with less ink than there are birds shares
     * its cells round, and the jitter inside the cell tells the birds apart. */
    int total = opaque > count ? opaque : count;
    int *cells = malloc(sizeof(*cells) * (size_t)total);
    if (cells == NULL) {
        png_image_free(&work);
        return 0;
    }
    int made = 0;
    for (int i = 0; i < columns * rows; i++)
        if (work.pixels[(size_t)i * 4 + 3] > 127) cells[made++] = i;
    for (int i = opaque; i < total; i++) cells[i] = cells[i % opaque];

    sampling_t sampling = {&work, fit, seed, out};
    divide(&sampling, cells, 0, total, 0, count);

    /* The pieces come out in order of place, the left of the picture first, and a
     * bird sent to the nth would be a bird sent to a place by its number. Dealt
     * out instead, by the seed. */
    for (int i = count - 1; i > 0; i--) {
        int j = (int)(mix(seed * 0x2545f491U + (uint32_t)i) % (uint32_t)(i + 1));
        picture_point_t swap = out[i];
        out[i] = out[j];
        out[j] = swap;
    }
    free(cells);
    png_image_free(&work);
    return count;
}

typedef struct {
    uint8_t rgb[3];
} pixel_t;

static int by_red(const void *a, const void *b) {
    return (int)((const pixel_t *)a)->rgb[0] - (int)((const pixel_t *)b)->rgb[0];
}
static int by_green(const void *a, const void *b) {
    return (int)((const pixel_t *)a)->rgb[1] - (int)((const pixel_t *)b)->rgb[1];
}
static int by_blue(const void *a, const void *b) {
    return (int)((const pixel_t *)a)->rgb[2] - (int)((const pixel_t *)b)->rgb[2];
}

typedef struct {
    int lo, hi; /* A range of the pixels. */
    int channel, range;
} box_t;

/* Which channel varies most in the box, and by how much. */
static void measure_box(const pixel_t *pixels, box_t *box) {
    int low[3] = {255, 255, 255}, high[3] = {0, 0, 0};
    for (int i = box->lo; i < box->hi; i++)
        for (int c = 0; c < 3; c++) {
            if (pixels[i].rgb[c] < low[c]) low[c] = pixels[i].rgb[c];
            if (pixels[i].rgb[c] > high[c]) high[c] = pixels[i].rgb[c];
        }
    box->channel = 0;
    box->range = high[0] - low[0];
    for (int c = 1; c < 3; c++)
        if (high[c] - low[c] > box->range) {
            box->range = high[c] - low[c];
            box->channel = c;
        }
}

double picture_luminance(const uint8_t rgb[3]) {
    return (0.2126 * rgb[0] + 0.7152 * rgb[1] + 0.0722 * rgb[2]) / 255.0;
}

int picture_quantise(const png_image_t *image, int most, uint8_t colours[][3]) {
    if (image == NULL || image->pixels == NULL || most <= 0 || colours == NULL) return 0;
    int longest = image->width > image->height ? image->width : image->height;
    int width = image->width, height = image->height;
    if (longest > COLOUR_WORK_MAX) {
        width = (int)((double)image->width * COLOUR_WORK_MAX / longest + 0.5);
        height = (int)((double)image->height * COLOUR_WORK_MAX / longest + 0.5);
        if (width < 1) width = 1;
        if (height < 1) height = 1;
    }
    png_image_t work = {0, 0, NULL};
    const png_image_t *source = image;
    if (width != image->width || height != image->height) {
        if (png_resize(image, width, height, &work) != PNG_OK) return 0;
        source = &work;
    }
    pixel_t *pixels = malloc(sizeof(*pixels) * (size_t)width * (size_t)height);
    box_t *boxes = malloc(sizeof(*boxes) * (size_t)most);
    if (pixels == NULL || boxes == NULL) {
        free(pixels);
        free(boxes);
        png_image_free(&work);
        return 0;
    }
    int count = 0;
    for (int i = 0; i < width * height; i++) {
        const uint8_t *pixel = source->pixels + (size_t)i * 4;
        if (pixel[3] <= 127) continue;
        memcpy(pixels[count++].rgb, pixel, 3);
    }
    png_image_free(&work);
    if (count == 0) {
        free(pixels);
        free(boxes);
        return 0;
    }

    int boxes_made = 1;
    boxes[0] = (box_t){0, count, 0, 0};
    measure_box(pixels, &boxes[0]);
    while (boxes_made < most) {
        /* Cut the box that has the most to say: the widest range of one channel
         * among those with a colour to split off. */
        int widest = -1;
        for (int b = 0; b < boxes_made; b++)
            if (boxes[b].range > 0 && (widest < 0 || boxes[b].range > boxes[widest].range))
                widest = b;
        if (widest < 0) break; /* Every box is one colour: that is the picture. */
        box_t *box = &boxes[widest];
        int (*order)(const void *, const void *) =
            box->channel == 0 ? by_red : (box->channel == 1 ? by_green : by_blue);
        qsort(pixels + box->lo, (size_t)(box->hi - box->lo), sizeof(*pixels), order);
        /* Cut where the colour changes nearest the middle, not through the middle of
         * a run of one colour: that puts the same colour in two boxes. */
        int middle = box->lo + (box->hi - box->lo) / 2, cut = -1;
        for (int away = 0; cut < 0 && away < box->hi - box->lo; away++) {
            for (int side = -1; side <= 1; side += 2) {
                int at = middle + side * away;
                if (at <= box->lo || at >= box->hi) continue;
                if (pixels[at].rgb[box->channel] != pixels[at - 1].rgb[box->channel]) {
                    cut = at;
                    break;
                }
            }
        }
        box_t upper = {cut, box->hi, 0, 0};
        box->hi = cut;
        measure_box(pixels, box);
        measure_box(pixels, &upper);
        boxes[boxes_made++] = upper;
    }

    int made = 0;
    for (int b = 0; b < boxes_made; b++) {
        long sum[3] = {0, 0, 0};
        for (int i = boxes[b].lo; i < boxes[b].hi; i++)
            for (int c = 0; c < 3; c++) sum[c] += pixels[i].rgb[c];
        long size = boxes[b].hi - boxes[b].lo;
        for (int c = 0; c < 3; c++) colours[made][c] = (uint8_t)((sum[c] + size / 2) / size);
        made++;
    }
    free(pixels);
    free(boxes);
    /* Lightest first, which is the way every ramp in the program runs. */
    for (int i = 1; i < made; i++)
        for (int j = i; j > 0 && picture_luminance(colours[j]) > picture_luminance(colours[j - 1]);
             j--) {
            for (int c = 0; c < 3; c++) {
                uint8_t swap = colours[j][c];
                colours[j][c] = colours[j - 1][c];
                colours[j - 1][c] = swap;
            }
        }
    return made;
}

int picture_nearest(const uint8_t colours[][3], int count, const uint8_t rgb[3]) {
    int best = 0;
    double best_gap = 1e300;
    for (int i = 0; i < count; i++) {
        /* The usual weighted distance: green counts for most, and red for more
         * the redder the pair. Good enough to choose between a handful. */
        double mean_red = (colours[i][0] + rgb[0]) / 2.0;
        double dr = (double)colours[i][0] - rgb[0], dg = (double)colours[i][1] - rgb[1];
        double db = (double)colours[i][2] - rgb[2];
        double gap =
            (2 + mean_red / 256) * dr * dr + 4 * dg * dg + (2 + (255 - mean_red) / 256) * db * db;
        if (gap < best_gap) {
            best_gap = gap;
            best = i;
        }
    }
    return best;
}
