/*
 * Minimal self contained PNG library.
 *
 * Handles what this project needs and nothing more : decoding of 8 bit
 * non interlaced PNG files (grayscale, RGB, with or without alpha), a few
 * geometric and color transformations, and encoding back to a PNG kept in
 * memory. No external dependency, zlib included : the DEFLATE decompressor
 * and the CRC/Adler checksums are implemented here.
 */

#ifndef PNG_H
#define PNG_H

#include <stddef.h>
#include <stdint.h>

/*8 bit RGBA image with straight (not premultiplied) alpha*/
typedef struct {
    int width;
    int height;
    uint8_t *pixels; /*width * height * 4 bytes, row major*/
} png_image_t;

typedef enum {
    PNG_OK = 0,
    PNG_ERR_MEMORY,
    PNG_ERR_ARGUMENT,
    PNG_ERR_TRUNCATED,
    PNG_ERR_SIGNATURE,
    PNG_ERR_CHUNK,
    PNG_ERR_CRC,
    PNG_ERR_UNSUPPORTED,
    PNG_ERR_DEFLATE
} png_status_t;

typedef enum {
    PNG_TINT_MULTIPLY, /*color is scaled by the given one*/
    PNG_TINT_REPLACE   /*color is replaced, alpha is kept*/
} png_tint_mode_t;

const char *png_status_string(png_status_t status);

/*Allocates a fully transparent image*/
png_status_t png_image_alloc(png_image_t *image, int width, int height);
void png_image_free(png_image_t *image);

/*Decodes an in memory PNG file into a RGBA image*/
png_status_t png_decode(const uint8_t *data, size_t length, png_image_t *out);

/*Encodes a RGBA image into an in memory PNG file, *out_data must be freed*/
png_status_t png_encode(const png_image_t *image, uint8_t **out_data, size_t *out_length);

/*Rotates around the center keeping the canvas size : whatever falls outside
 * is cropped, whatever enters is transparent. Positive angles turn clockwise
 * on screen (y grows downwards).*/
png_status_t png_rotate(const png_image_t *src, double radians, png_image_t *out);

/*Box filtered when shrinking, bilinear when enlarging*/
png_status_t png_resize(const png_image_t *src, int width, int height, png_image_t *out);

/*Rotation at the source resolution followed by the resize, which is what
 * keeps the small sprites clean*/
png_status_t png_rotate_resize(const png_image_t *src, double radians, int width, int height,
                               png_image_t *out);

/*Recolors the image in place, alpha is never touched*/
void png_tint(png_image_t *image, uint8_t r, uint8_t g, uint8_t b, png_tint_mode_t mode);

#endif
