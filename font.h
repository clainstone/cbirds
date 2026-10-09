/*
 * A 5 by 7 bitmap font, just wide enough for a flock to spell with.
 *
 * Glyphs are authored as seven rows of five characters, '#' where a bird goes,
 * so the data can be read and corrected in place rather than decoded from hex.
 * For the flock, lower case maps to upper case: at five pixels wide there is no
 * room for two cases, and a flock writing in capitals is what you want anyway. A
 * picture of a screenful of text wants both, and has them through font_glyph_for.
 */

#ifndef FONT_H
#define FONT_H

#include <stdint.h>

enum { FONT_WIDTH = 5, FONT_HEIGHT = 7, FONT_ADVANCE = FONT_WIDTH + 1 };

/* The 35 cells of a glyph, row major, '#' set and anything else clear. Returns
 * NULL for a character the font does not carry. Space returns all clear. */
const char *font_glyph(char character);

/* The same for a code point, and without folding case: lower case letters have
 * glyphs of their own, so does the rest of printable ASCII, and anything past
 * ASCII is NULL. */
const char *font_glyph_for(uint32_t glyph);

/* The plain letter or sign that stands in for an accented or typographic one, or
 * zero: é is e, a curly quote is a straight one. */
uint32_t font_plain_letter(uint32_t glyph);

/* Cells that would be set if the text were laid out: what a caller has to make
 * room for. */
int font_text_cells(const char *text);

/* Width in glyph cells of the text, blanks included. */
int font_text_width(const char *text);

#endif
