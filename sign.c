#include "sign.h"

#include <ctype.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "font.h"

/* M_PI is not C99, and this file asks for no feature macros. */
static const double PI = 3.14159265358979323846;

/* The next character of UTF-8 text and the bytes it takes. A byte that begins
 * nothing well formed is one byte and U+FFFD, which the font has no letter for. */
static uint32_t next_code_point(const char *c, int *length) {
    const unsigned char *at = (const unsigned char *)c;
    int more = at[0] >= 0xC2 && at[0] <= 0xDF   ? 1
               : at[0] >= 0xE0 && at[0] <= 0xEF ? 2
               : at[0] >= 0xF0 && at[0] <= 0xF4 ? 3
                                                : 0;
    uint32_t point = more == 1 ? at[0] & 0x1Fu : more == 2 ? at[0] & 0x0Fu : at[0] & 0x07u;
    *length = 1;
    if (more == 0) return 0xFFFD;
    for (int i = 1; i <= more; i++) {
        if ((at[i] & 0xC0) != 0x80) return 0xFFFD; /* Cut short, or not UTF-8 at all. */
        point = (point << 6) | (at[i] & 0x3Fu);
    }
    *length = more + 1;
    return point;
}

/* What a character past ASCII is written as: its plain letter, the font's own
 * word for it where there is one (é is E, a curly quote a straight one), or two
 * letters where one would be a lie: ß is SS, as it is when a German capitalises
 * it, and the ligatures are AE and OE. Nothing for what the font cannot say. */
static int plain_letters(uint32_t point, char letters[2]) {
    switch (point) {
        case 0xDF: /* ß */
            letters[0] = letters[1] = 'S';
            return 2;
        case 0xC6: /* Æ */
        case 0xE6: /* æ */
            letters[0] = 'A';
            letters[1] = 'E';
            return 2;
        case 0x152: /* Œ */
        case 0x153: /* œ */
            letters[0] = 'O';
            letters[1] = 'E';
            return 2;
        default:
            letters[0] = (char)font_plain_letter(point);
            return letters[0] != '\0' ? 1 : 0;
    }
}

int sign_clean(const char *text, char *out, size_t size) {
    size_t at = 0;
    int kept = 0;

    if (size == 0) return 0;
    out[0] = '\0';
    if (text == NULL) return 0;
    for (const char *c = text; *c != '\0' && at + 1 < size && at + 1 < SIGN_TEXT_MAX;) {
        char letters[2] = {*c, '\0'};
        int count = 1, length = 1;
        if ((unsigned char)*c >= 0x80) {
            count = plain_letters(next_code_point(c, &length), letters);
        }
        c += length;
        for (int i = 0; i < count && at + 1 < size && at + 1 < SIGN_TEXT_MAX; i++) {
            char wanted = letters[i];
            /* A new line in the middle of a sentence is a space, not a missing letter. */
            if (wanted == '\t' || wanted == '\n' || wanted == '\r') wanted = ' ';
            if (wanted == ' ') {
                if (at > 0 && out[at - 1] != ' ') out[at++] = ' ';
                continue;
            }
            if (font_glyph(wanted) == NULL) continue; /* Skipped, as if it were not there. */
            out[at++] = (char)toupper((unsigned char)wanted);
            kept++;
        }
    }
    while (at > 0 && out[at - 1] == ' ') at--;
    out[at] = '\0';
    return kept;
}

int sign_rows(int lines) {
    return lines <= 0 ? 0 : lines * FONT_HEIGHT + (lines - 1) * SIGN_LINE_GAP;
}

int sign_columns(const char *line) {
    size_t glyphs = strlen(line);
    return glyphs == 0 ? 0 : (int)glyphs * FONT_ADVANCE - 1;
}

enum { MAX_WORDS = SIGN_TEXT_MAX / 2 + 1 };

typedef struct {
    int count;
    const char *start[MAX_WORDS];
    int length[MAX_WORDS];
} words_t;

static void split_words(const char *clean, words_t *words) {
    words->count = 0;
    for (const char *c = clean; *c != '\0';) {
        if (*c == ' ') {
            c++;
            continue;
        }
        const char *end = c;
        while (*end != '\0' && *end != ' ') end++;
        if (words->count < MAX_WORDS) {
            words->start[words->count] = c;
            words->length[words->count] = (int)(end - c);
            words->count++;
        }
        c = end;
    }
}

/* Greedy: as many words to a line as fit in `limit` letters. Fills `lines` and
 * returns how many it took, or zero if it needed more than `most`. */
static int wrap_at(const words_t *words, int limit, int most, sign_lines_t *lines) {
    int count = 0, used = 0;
    memset(lines, 0, sizeof(*lines));
    for (int w = 0; w < words->count; w++) {
        int length = words->length[w];
        if (count > 0 && used > 0 && used + 1 + length <= limit) {
            lines->line[count - 1][used] = ' ';
            memcpy(lines->line[count - 1] + used + 1, words->start[w], (size_t)length);
            used += 1 + length;
            lines->line[count - 1][used] = '\0';
            continue;
        }
        if (count == most) return 0;
        memcpy(lines->line[count], words->start[w], (size_t)length);
        lines->line[count][length] = '\0';
        used = length;
        count++;
    }
    lines->count = count;
    return count;
}

int sign_fit(const char *clean, int reference_columns, double width, double height, double largest,
             sign_lines_t *out, double *cell) {
    words_t words;
    int best_count = 0;
    double best_cell = 0;

    memset(out, 0, sizeof(*out));
    *cell = 0;
    if (clean == NULL || width <= 0 || height <= 0) return 0;
    split_words(clean, &words);
    if (words.count == 0) return 0;

    int longest = 0, total = 0;
    for (int w = 0; w < words.count; w++) {
        if (words.length[w] > longest) longest = words.length[w];
        total += words.length[w] + 1;
    }
    for (int allowed = 1; allowed <= SIGN_MAX_LINES && allowed <= words.count; allowed++) {
        /* The narrowest the lines can be and still take no more than `allowed` of
         * them: the widest line is then as short as the words let it be, which is
         * what makes the letters as large as they can be. */
        sign_lines_t lines;
        int count = 0;
        for (int limit = longest; limit <= total && count == 0; limit++)
            count = wrap_at(&words, limit, allowed, &lines);
        if (count == 0) continue;

        int columns = reference_columns;
        for (int l = 0; l < count; l++)
            if (sign_columns(lines.line[l]) > columns) columns = sign_columns(lines.line[l]);
        double size = width / columns;
        double by_height = height / sign_rows(count);
        if (by_height < size) size = by_height;
        if (size > largest) size = largest;
        /* A tie goes to fewer lines: a second one has to buy a real gain in size,
         * or a short sentence is broken in two for nothing. */
        if (best_count == 0 || size > best_cell * 1.03) {
            best_count = count;
            best_cell = size;
            *out = lines;
        }
    }
    if (best_count == 0 || best_cell < 1.0) {
        memset(out, 0, sizeof(*out));
        return 0;
    }
    *cell = best_cell;
    return best_count;
}

/* The conversions that print the hour on a twelve hour clock, past whatever a
 * glibc flag, a width or an E or O modifier puts in front of them. */
int sign_wants_twelve_hours(const char *time_format) {
    if (time_format == NULL) return 0;
    for (const char *c = time_format; *c != '\0'; c++) {
        if (*c != '%') continue;
        c++;
        if (*c == '%') continue;
        while (*c != '\0' && (strchr("-_0^#", *c) != NULL || (*c >= '1' && *c <= '9'))) c++;
        while (*c == 'E' || *c == 'O') c++;
        if (*c == '\0') break;
        if (*c == 'I' || *c == 'l' || *c == 'r') return 1;
    }
    return 0;
}

void sign_clock_text(const struct tm *when, int twelve_hours, int seconds, char *out,
                     size_t size) {
    int hour = when->tm_hour, minute = when->tm_min, second = when->tm_sec;
    if (twelve_hours) {
        hour %= 12;
        if (hour == 0) hour = 12;
    }
    if (seconds && twelve_hours)
        snprintf(out, size, "%d:%02d:%02d", hour, minute, second);
    else if (seconds)
        snprintf(out, size, "%02d:%02d:%02d", hour, minute, second);
    else if (twelve_hours)
        snprintf(out, size, "%d:%02d", hour, minute);
    else
        snprintf(out, size, "%02d:%02d", hour, minute);
}

/* A small integer hash (lowbias32): every bit of the input reaches every bit of
 * the output, which is all a bird's loop needs of a random number. Drawn from
 * the bird's number and not from the flock's own generator, so that a loop is the
 * same loop at any frame rate, and holding a sign changes nothing about where the
 * flock's random numbers would have taken it. */
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

void sign_hover(unsigned id, double seconds, double radius, double *dx, double *dy) {
    uint32_t base = mix(id + 0x9e3779b9U);
    double size = 0.6 + 0.4 * unit_of(mix(base + 1 * 0x85ebca6bU));
    double hertz = 0.5 + 0.9 * unit_of(mix(base + 2 * 0x85ebca6bU));
    double phase = 2 * PI * unit_of(mix(base + 3 * 0x85ebca6bU));
    double flatness = 0.5 + 0.5 * unit_of(mix(base + 4 * 0x85ebca6bU));
    double tilt = PI * unit_of(mix(base + 5 * 0x85ebca6bU));
    double sense = (mix(base + 6 * 0x85ebca6bU) & 1u) ? 1.0 : -1.0;

    double angle = phase + sense * 2 * PI * hertz * seconds;
    double along = size * radius * cos(angle);
    double across = size * radius * flatness * sin(angle);
    *dx = along * cos(tilt) - across * sin(tilt);
    *dy = along * sin(tilt) + across * cos(tilt);
}

double sign_unit(unsigned id) {
    return unit_of(mix(id * 0x9e3779b1U + 0x7f4a7c15U));
}

double sign_hold_seconds(unsigned cycle) {
    return 30.0 + 15.0 * unit_of(mix(cycle * 2u + 1u));
}

/* Eight to twelve seconds, not ten to fifteen, which was too long to wait for the
 * text on a small screen. A murmuration needs about a second to be one: on 800
 * birds, from a second after the release the flock is one group on 120 by 34
 * cells and up, 91 to 100% of the birds linked when they are within a bird and a
 * half of each other, and on 96 by 26 and 80 by 24, where they are crowded, it is
 * a few clouds that wheel and join, the largest of them holding two fifths. So
 * the shortest flight is seven seconds of murmuration, and the writers are home
 * between four tenths of a second and nine tenths after the text is written. */
double sign_flight_seconds(unsigned cycle) {
    return 8.0 + 4.0 * unit_of(mix(cycle * 2u + 2u));
}

double sign_breath(double fraction_of_a_second) {
    return 0.5 - 0.5 * cos(2 * PI * fraction_of_a_second);
}

int sign_box_push(const sign_box_t *box, double band, double x, double y, double *push_x,
                  double *push_y) {
    *push_x = *push_y = 0;
    double nearest_x = x < box->left ? box->left : (x > box->right ? box->right : x);
    double nearest_y = y < box->top ? box->top : (y > box->bottom ? box->bottom : y);
    double dx = x - nearest_x, dy = y - nearest_y;
    double distance = sqrt(dx * dx + dy * dy);

    if (distance > 0) {
        if (distance >= band) return 0;
        /* Squared, so it is felt late and firmly: a bird at the end of the band
         * is not turned at all, and one at the box is turned hard. */
        double strength = (1 - distance / band) * (1 - distance / band);
        *push_x = strength * dx / distance;
        *push_y = strength * dy / distance;
        return 1;
    }
    /* Inside, or exactly on its edge: out through the nearest side, harder the
     * further in, so a bird that gets to the middle has no doubt about the way. */
    double to_left = x - box->left, to_right = box->right - x;
    double to_top = y - box->top, to_bottom = box->bottom - y;
    double least = to_left;
    double out_x = -1, out_y = 0;
    if (to_right < least) {
        least = to_right;
        out_x = 1;
    }
    if (to_top < least) {
        least = to_top;
        out_x = 0;
        out_y = -1;
    }
    if (to_bottom < least) {
        least = to_bottom;
        out_x = 0;
        out_y = 1;
    }
    double half = (box->right - box->left) / 2;
    if ((box->bottom - box->top) / 2 < half) half = (box->bottom - box->top) / 2;
    double depth = half > 0 ? least / half : 0;
    if (depth > 1) depth = 1;
    *push_x = out_x * (1 + depth);
    *push_y = out_y * (1 + depth);
    return 1;
}
