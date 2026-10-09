#include "../picture.h"

#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

static png_image_t make_image(int width, int height) {
    png_image_t image = {0, 0, NULL};
    assert(png_image_alloc(&image, width, height) == PNG_OK);
    return image;
}

static void set_pixel(png_image_t *image, int x, int y, int r, int g, int b, int a) {
    uint8_t *pixel = image->pixels + ((size_t)y * (size_t)image->width + (size_t)x) * 4;
    pixel[0] = (uint8_t)r;
    pixel[1] = (uint8_t)g;
    pixel[2] = (uint8_t)b;
    pixel[3] = (uint8_t)a;
}

static png_image_t solid_square(int side) {
    png_image_t image = make_image(side, side);
    for (int y = 0; y < side; y++)
        for (int x = 0; x < side; x++) set_pixel(&image, x, y, 200, 120, 40, 255);
    return image;
}

/* A disc of ink on nothing: the shape of a picture with a cut out edge. */
static png_image_t disc(int side, double radius) {
    png_image_t image = make_image(side, side);
    for (int y = 0; y < side; y++)
        for (int x = 0; x < side; x++)
            if (hypot(x + 0.5 - side / 2.0, y + 0.5 - side / 2.0) <= radius)
                set_pixel(&image, x, y, 30, 200, 90, 255);
    return image;
}

static void test_a_picture_lands_in_its_box_with_its_own_proportions(void) {
    png_image_t wide = make_image(40, 10);
    picture_fit_t fit = picture_fit(&wide, 100, 50, 400, 300);
    /* As wide as the box, a quarter as tall, in the middle of it. */
    assert(fit.scale == 10 && fit.width == 400 && fit.height == 100);
    assert(fit.left == 100 && fit.top == 50 + 100);
    png_image_t tall = make_image(10, 40);
    fit = picture_fit(&tall, 100, 50, 400, 300);
    assert(fit.scale == 7.5 && fit.height == 300 && fit.width == 75);
    assert(fit.left == 100 + (400 - 75) / 2.0 && fit.top == 50);
    /* Nothing to fit, or nowhere to fit it. */
    fit = picture_fit(&tall, 0, 0, 0, 300);
    assert(fit.width == 0 && fit.scale == 0);
    png_image_free(&wide);
    png_image_free(&tall);
}

static void test_as_many_points_as_asked_inside_the_opaque_area(void) {
    png_image_t image = disc(200, 80);
    picture_fit_t fit = picture_fit(&image, 50, 20, 600, 300);
    enum { COUNT = 800 };
    static picture_point_t points[COUNT];

    assert(picture_sample(&image, &fit, COUNT, 5, points) == COUNT);
    for (int i = 0; i < COUNT; i++) {
        /* On the disc, in the picture's own pixels: the point is inside the fitted
         * box, and mapped back to the image it is within a cell of ink. The cell is
         * the sampling grid's own, which is a pixel or three of the screen. */
        assert(points[i].x >= fit.left && points[i].x <= fit.left + fit.width);
        assert(points[i].y >= fit.top && points[i].y <= fit.top + fit.height);
        double image_x = (points[i].x - fit.left) / fit.scale;
        double image_y = (points[i].y - fit.top) / fit.scale;
        double from_the_middle = hypot(image_x - 100, image_y - 100);
        assert(from_the_middle <= 80 + 3.0 / fit.scale * 2);
        /* Its colour, give or take what a box filter and its rounding make of an edge. */
        assert(abs(points[i].rgb[0] - 30) <= 4 && abs(points[i].rgb[1] - 200) <= 4 &&
               abs(points[i].rgb[2] - 90) <= 4);
    }
    /* And they use the whole of it: the disc is not sampled from one side. */
    double least_x = 1e9, most_x = 0, least_y = 1e9, most_y = 0;
    for (int i = 0; i < COUNT; i++) {
        if (points[i].x < least_x) least_x = points[i].x;
        if (points[i].x > most_x) most_x = points[i].x;
        if (points[i].y < least_y) least_y = points[i].y;
        if (points[i].y > most_y) most_y = points[i].y;
    }
    assert(most_x - least_x > 0.9 * 160 * fit.scale);
    assert(most_y - least_y > 0.9 * 160 * fit.scale);
    png_image_free(&image);
}

static void test_the_same_seed_gives_the_same_points(void) {
    png_image_t image = disc(120, 55);
    picture_fit_t fit = picture_fit(&image, 0, 0, 500, 300);
    enum { COUNT = 300 };
    static picture_point_t first[COUNT], again[COUNT], other[COUNT];

    assert(picture_sample(&image, &fit, COUNT, 7, first) == COUNT);
    assert(picture_sample(&image, &fit, COUNT, 7, again) == COUNT);
    assert(memcmp(first, again, sizeof(first)) == 0);
    assert(picture_sample(&image, &fit, COUNT, 8, other) == COUNT);
    int moved = 0;
    for (int i = 0; i < COUNT; i++) moved += first[i].x != other[i].x || first[i].y != other[i].y;
    assert(moved > COUNT / 2); /* Another seed, another jitter. */
    png_image_free(&image);
}

static void test_the_points_are_spread_evenly(void) {
    png_image_t image = solid_square(100);
    picture_fit_t fit = picture_fit(&image, 0, 0, 400, 400);
    enum { COUNT = 400 };
    static picture_point_t points[COUNT];
    assert(picture_sample(&image, &fit, COUNT, 3, points) == COUNT);

    /* Four hundred points on 400 by 400: one to every 20 by 20. No two are close
     * and none is far from its nearest neighbour. */
    double spacing = 20, least = 1e9, most = 0;
    for (int i = 0; i < COUNT; i++) {
        double nearest = 1e9;
        for (int j = 0; j < COUNT; j++) {
            if (i == j) continue;
            double gap = hypot(points[i].x - points[j].x, points[i].y - points[j].y);
            if (gap < nearest) nearest = gap;
        }
        if (nearest < least) least = nearest;
        if (nearest > most) most = nearest;
    }
    assert(least > 0.4 * spacing);
    assert(most < 1.3 * spacing);

    /* Equal shares of the picture have equal shares of the points: sixteen tiles
     * of a hundred pixels, each with twenty five points give or take three. */
    int tiles[16] = {0};
    for (int i = 0; i < COUNT; i++) {
        int column = (int)(points[i].x / 100), row = (int)(points[i].y / 100);
        if (column > 3) column = 3;
        if (row > 3) row = 3;
        tiles[row * 4 + column]++;
    }
    for (int t = 0; t < 16; t++) assert(tiles[t] >= 22 && tiles[t] <= 28);
    png_image_free(&image);
}

static void test_the_points_follow_where_the_picture_is_and_what_colour_it_is(void) {
    /* Red on the left half, blue on the right, and nothing in the top quarter. */
    png_image_t image = make_image(80, 40);
    for (int y = 10; y < 40; y++)
        for (int x = 0; x < 80; x++)
            if (x < 40)
                set_pixel(&image, x, y, 220, 30, 30, 255);
            else
                set_pixel(&image, x, y, 30, 30, 220, 255);
    picture_fit_t fit = picture_fit(&image, 0, 0, 800, 400);
    enum { COUNT = 500 };
    static picture_point_t points[COUNT];
    assert(picture_sample(&image, &fit, COUNT, 1, points) == COUNT);
    int red = 0, blue = 0;
    for (int i = 0; i < COUNT; i++) {
        assert(points[i].y >= 100 - 5); /* None in the part with no ink. */
        if (points[i].x < 400) {
            assert(points[i].rgb[0] == 220 && points[i].rgb[2] == 30);
            red++;
        } else {
            assert(points[i].rgb[2] == 220 && points[i].rgb[0] == 30);
            blue++;
        }
    }
    assert(red > 230 && blue > 230);
    png_image_free(&image);
}

static void test_only_what_is_more_than_half_opaque_is_ink(void) {
    png_image_t image = make_image(60, 60);
    for (int y = 0; y < 60; y++)
        for (int x = 0; x < 60; x++) set_pixel(&image, x, y, 255, 255, 255, x < 30 ? 127 : 128);
    picture_fit_t fit = picture_fit(&image, 0, 0, 240, 240);
    static picture_point_t points[200];
    assert(picture_sample(&image, &fit, 200, 1, points) == 200);
    for (int i = 0; i < 200; i++) assert(points[i].x > 120 - 5); /* 128 is ink, 127 is not. */

    /* Nothing opaque at all: no points, and the caller says so. */
    for (int i = 0; i < 60 * 60; i++) image.pixels[i * 4 + 3] = 0;
    assert(picture_sample(&image, &fit, 200, 1, points) == 0);
    png_image_free(&image);
}

static void test_more_birds_than_ink_share_it_out(void) {
    /* Sixteen opaque pixels and a hundred birds: every bird has a place, and no
     * two of them have the same one. */
    png_image_t image = solid_square(4);
    picture_fit_t fit = picture_fit(&image, 0, 0, 40, 40);
    enum { COUNT = 100 };
    static picture_point_t points[COUNT];
    assert(picture_sample(&image, &fit, COUNT, 2, points) == COUNT);
    for (int i = 0; i < COUNT; i++) {
        assert(points[i].x >= 0 && points[i].x <= 40 && points[i].y >= 0 && points[i].y <= 40);
        for (int j = 0; j < i; j++)
            assert(points[i].x != points[j].x || points[i].y != points[j].y);
    }
    /* One bird, and a thousand. */
    assert(picture_sample(&image, &fit, 1, 2, points) == 1);
    static picture_point_t many[4096];
    assert(picture_sample(&image, &fit, 4096, 2, many) == 4096);
    png_image_free(&image);

    /* A big picture, at the largest the work grid goes to, and every bird of the
     * most there are. */
    png_image_t big = solid_square(1000);
    fit = picture_fit(&big, 0, 0, 1500, 900);
    assert(picture_sample(&big, &fit, 4096, 9, many) == 4096);
    for (int i = 0; i < 4096; i++)
        assert(many[i].x >= fit.left && many[i].x <= fit.left + fit.width && many[i].y >= fit.top &&
               many[i].y <= fit.top + fit.height);
    png_image_free(&big);
}

static void test_a_picture_is_cut_down_to_a_few_colours(void) {
    uint8_t colours[8][3];

    /* A picture of three colours is three colours, lightest first. */
    png_image_t image = make_image(30, 10);
    for (int y = 0; y < 10; y++)
        for (int x = 0; x < 30; x++) {
            if (x < 10)
                set_pixel(&image, x, y, 250, 250, 240, 255);
            else if (x < 20)
                set_pixel(&image, x, y, 200, 40, 40, 255);
            else
                set_pixel(&image, x, y, 20, 20, 90, 255);
        }
    assert(picture_quantise(&image, 8, colours) == 3);
    assert(colours[0][0] == 250 && colours[1][0] == 200 && colours[2][2] == 90);
    assert(picture_luminance(colours[0]) > picture_luminance(colours[1]));
    assert(picture_luminance(colours[1]) > picture_luminance(colours[2]));
    /* Fewer asked for, fewer given. */
    assert(picture_quantise(&image, 2, colours) == 2);
    assert(picture_quantise(&image, 1, colours) == 1);
    png_image_free(&image);

    /* A gradient of two hundred and fifty six greys is as many as were asked for
     * and no more, and they run the length of it. */
    image = make_image(256, 4);
    for (int y = 0; y < 4; y++)
        for (int x = 0; x < 256; x++) set_pixel(&image, x, y, x, x, x, 255);
    assert(picture_quantise(&image, 8, colours) == 8);
    for (int i = 1; i < 8; i++)
        assert(picture_luminance(colours[i - 1]) > picture_luminance(colours[i]));
    assert(colours[0][0] > 200 && colours[7][0] < 55);
    png_image_free(&image);

    /* What is not ink does not count: a red wall behind a blue picture is blue. */
    image = make_image(20, 20);
    for (int y = 0; y < 20; y++)
        for (int x = 0; x < 20; x++)
            set_pixel(&image, x, y, x < 10 ? 255 : 20, 0, x < 10 ? 0 : 255, x < 10 ? 0 : 255);
    assert(picture_quantise(&image, 8, colours) == 1);
    assert(colours[0][2] == 255 && colours[0][0] == 20);
    for (int i = 0; i < 400; i++) image.pixels[i * 4 + 3] = 0;
    assert(picture_quantise(&image, 8, colours) == 0);
    png_image_free(&image);
}

static void test_a_colour_finds_the_nearest_of_a_few(void) {
    static const uint8_t palette[4][3] = {{250, 250, 240}, {200, 40, 40}, {20, 20, 90}, {0, 0, 0}};
    const uint8_t pale[3] = {240, 240, 230}, red[3] = {180, 60, 50}, navy[3] = {30, 30, 100};
    const uint8_t dark[3] = {5, 5, 5};
    assert(picture_nearest(palette, 4, pale) == 0);
    assert(picture_nearest(palette, 4, red) == 1);
    assert(picture_nearest(palette, 4, navy) == 2);
    assert(picture_nearest(palette, 4, dark) == 3);
    assert(picture_nearest(palette, 3, dark) == 2); /* Of the three it is given. */
    assert(picture_nearest(palette, 1, red) == 0);
}

static void test_light_is_light(void) {
    const uint8_t black[3] = {0, 0, 0}, white[3] = {255, 255, 255};
    const uint8_t red[3] = {255, 0, 0}, green[3] = {0, 255, 0}, blue[3] = {0, 0, 255};
    assert(picture_luminance(black) == 0);
    assert(fabs(picture_luminance(white) - 1) < 1e-9);
    assert(picture_luminance(green) > picture_luminance(red));
    assert(picture_luminance(red) > picture_luminance(blue));
}

int main(void) {
    test_a_picture_lands_in_its_box_with_its_own_proportions();
    test_as_many_points_as_asked_inside_the_opaque_area();
    test_the_same_seed_gives_the_same_points();
    test_the_points_are_spread_evenly();
    test_the_points_follow_where_the_picture_is_and_what_colour_it_is();
    test_only_what_is_more_than_half_opaque_is_ink();
    test_more_birds_than_ink_share_it_out();
    test_a_picture_is_cut_down_to_a_few_colours();
    test_a_colour_finds_the_nearest_of_a_few();
    test_light_is_light();
    return 0;
}
