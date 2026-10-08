/*
 * A picture, as something a flock can be told to draw: where to send each bird
 * so that together they cover the picture evenly, and what colours they wear.
 *
 * The picture is a PNG, and only the opaque part of it counts: a pixel with an
 * alpha above half is ink, the rest is sky. Nothing here knows about birds. It
 * knows an image, a box to fit it into, and how many points are wanted.
 */

#ifndef PICTURE_H
#define PICTURE_H

#include <stdint.h>

#include "png.h"

/* Where the picture lands: its own proportions, as large as the box allows, in
 * the middle of it. */
typedef struct {
    double left, top, width, height; /* In the box's units: pixels of the screen. */
    double scale;                    /* One pixel of the image is this many of those. */
} picture_fit_t;

picture_fit_t picture_fit(const png_image_t *image, double left, double top, double width,
                          double height);

typedef struct {
    double x, y;
    uint8_t rgb[3]; /* The picture's colour where the point is. */
} picture_point_t;

/* As many points as asked for, spread evenly over the opaque pixels of the image
 * as it lands in `fit`, and the colour of the picture under each. The opaque
 * area is cut into as many pieces of equal area as there are points, and each
 * point is a little off the middle of its own piece, by an amount the seed picks:
 * so no two points are close, none is far from its neighbours, and the same seed
 * gives the same points. Returns how many were made: all of them, or none if
 * there is nothing opaque. The points come in no order of place. */
int picture_sample(const png_image_t *image, const picture_fit_t *fit, int count, unsigned seed,
                   picture_point_t *out);

/* At most `most` colours that stand for the opaque part of the image, by median
 * cut, lightest first. Fewer if the picture has fewer; none if nothing is opaque.
 * Returns how many. */
int picture_quantise(const png_image_t *image, int most, uint8_t colours[][3]);

/* The one of colours[0..count) that looks most like rgb. */
int picture_nearest(const uint8_t colours[][3], int count, const uint8_t rgb[3]);

/* How light a colour is, from 0 to 1. */
double picture_luminance(const uint8_t rgb[3]);

#endif
