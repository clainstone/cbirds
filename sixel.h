/*
 * A sixel encoder, for the terminals that draw pixels but not Kitty's: xterm,
 * foot, mlterm, contour, Konsole, mintty, Windows Terminal. Six rows at a time,
 * one pass per colour per band, run length coded, against a palette the caller
 * chooses — the flock is five tints, a hawk and a ground, and every pixel is
 * snapped to the nearest of those rather than to whatever a quantiser finds,
 * because a small palette is what keeps a frame small enough to send thirty
 * times a second.
 */

#ifndef SIXEL_H
#define SIXEL_H

#include <stddef.h>
#include <stdint.h>

#include "png.h"

enum { SIXEL_COLOURS_MAX = 256 };

typedef enum { SIXEL_OK = 0, SIXEL_ERR_ARGUMENT, SIXEL_ERR_MEMORY } sixel_status_t;

typedef struct {
    char *text; /* The DCS sequence, NUL terminated, from the last encode. */
    size_t length, capacity;
    uint8_t *index; /* Scratch: one palette index a pixel. */
    size_t index_capacity;
    int16_t *memo;    /* 15 bit colour to palette index, filled as colours are met. */
    int memo_colours; /* How many palette entries the memo was built for. */
} sixel_t;

sixel_status_t sixel_init(sixel_t *sixel);
void sixel_destroy(sixel_t *sixel);

/* Encodes the image against the palette into sixel->text. Alpha is ignored:
 * the caller composes onto a ground first, and the ground is in the palette. */
sixel_status_t sixel_encode(sixel_t *sixel, const png_image_t *image, const uint8_t (*palette)[3],
                            int colours);

const char *sixel_status_string(sixel_status_t status);

#endif
