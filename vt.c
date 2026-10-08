#include "vt.h"

#include <stdlib.h>
#include <string.h>

enum {
    GROUND,
    ESCAPE,
    ESCAPE_SKIP, /* After ESC and an intermediate byte: one more byte ends it. */
    CSI,
    CSI_IGNORE, /* A sequence that broke its own grammar, read to its final byte. */
    OSC,        /* Ends at BEL or ST. */
    STRING      /* DCS, SOS, PM and APC: ends at ST. */
};

enum {
    /* Large enough for any cursor move a tool will ask for (neofetch moves left by
     * nine million to be sure of column zero) and small enough that the products
     * below cannot overflow an int. */
    PARAM_LIMIT = 1 << 24,
    TAB_WIDTH = 8,
    REPLACEMENT = 0xFFFD
};

typedef struct {
    uint32_t first, last;
} range_t;

/* Marks that take no cell of their own: the combining blocks, the joiners and
 * direction controls, variation selectors, and the Hangul vowels and finals that
 * sit on the consonant before them. The ranges are those of wcwidth's table that
 * a terminal is ever shown; the many small Indic and Arabic marks are left out,
 * and print as one cell, which is what a terminal without the tables does too. */
/* clang-format off */
static const range_t ZERO_WIDTH[] = {
    {0x0300, 0x036F},   {0x0483, 0x0489},   {0x0591, 0x05BD},   {0x05BF, 0x05BF},
    {0x05C1, 0x05C2},   {0x05C4, 0x05C5},   {0x05C7, 0x05C7},   {0x0610, 0x061A},
    {0x064B, 0x065F},   {0x0670, 0x0670},   {0x06D6, 0x06DC},   {0x06DF, 0x06E4},
    {0x06E7, 0x06E8},   {0x06EA, 0x06ED},   {0x0E31, 0x0E31},   {0x0E34, 0x0E3A},
    {0x0E47, 0x0E4E},   {0x1160, 0x11FF},   {0x1AB0, 0x1AFF},   {0x1DC0, 0x1DFF},
    {0x200B, 0x200F},   {0x202A, 0x202E},   {0x2060, 0x2064},   {0x2066, 0x206F},
    {0x20D0, 0x20FF},   {0x3099, 0x309A},   {0xFE00, 0xFE0F},   {0xFE20, 0xFE2F},
    {0xFEFF, 0xFEFF},   {0xE0100, 0xE01EF},
};
/* clang-format on */

/* East Asian Wide and Fullwidth, and the emoji that default to two cells. Sorted,
 * for a binary search. */
/* clang-format off */
static const range_t WIDE[] = {
    {0x1100, 0x115F},   {0x231A, 0x231B},   {0x2329, 0x232A},   {0x23E9, 0x23EC},
    {0x23F0, 0x23F0},   {0x23F3, 0x23F3},   {0x25FD, 0x25FE},   {0x2614, 0x2615},
    {0x2648, 0x2653},   {0x267F, 0x267F},   {0x2693, 0x2693},   {0x26A1, 0x26A1},
    {0x26AA, 0x26AB},   {0x26BD, 0x26BE},   {0x26C4, 0x26C5},   {0x26CE, 0x26CE},
    {0x26D4, 0x26D4},   {0x26EA, 0x26EA},   {0x26F2, 0x26F3},   {0x26F5, 0x26F5},
    {0x26FA, 0x26FA},   {0x26FD, 0x26FD},   {0x2705, 0x2705},   {0x270A, 0x270B},
    {0x2728, 0x2728},   {0x274C, 0x274C},   {0x274E, 0x274E},   {0x2753, 0x2755},
    {0x2757, 0x2757},   {0x2795, 0x2797},   {0x27B0, 0x27B0},   {0x27BF, 0x27BF},
    {0x2B1B, 0x2B1C},   {0x2B50, 0x2B50},   {0x2B55, 0x2B55},   {0x2E80, 0x2E99},
    {0x2E9B, 0x2EF3},   {0x2F00, 0x2FD5},   {0x2FF0, 0x2FFB},   {0x3000, 0x303E},
    {0x3041, 0x3096},   {0x309B, 0x30FF},   {0x3105, 0x312F},   {0x3131, 0x318E},
    {0x3190, 0x31E3},   {0x31F0, 0x321E},   {0x3220, 0x3247},   {0x3250, 0x4DBF},
    {0x4E00, 0xA48C},   {0xA490, 0xA4C6},   {0xA960, 0xA97C},   {0xAC00, 0xD7A3},
    {0xF900, 0xFAFF},   {0xFE10, 0xFE19},   {0xFE30, 0xFE52},   {0xFE54, 0xFE66},
    {0xFE68, 0xFE6B},   {0xFF01, 0xFF60},   {0xFFE0, 0xFFE6},   {0x16FE0, 0x16FE4},
    {0x17000, 0x187F7}, {0x18800, 0x18CD5}, {0x1B000, 0x1B2FB}, {0x1F004, 0x1F004},
    {0x1F0CF, 0x1F0CF}, {0x1F18E, 0x1F18E}, {0x1F191, 0x1F19A}, {0x1F200, 0x1F202},
    {0x1F210, 0x1F23B}, {0x1F240, 0x1F248}, {0x1F250, 0x1F251}, {0x1F260, 0x1F265},
    {0x1F300, 0x1F320}, {0x1F32D, 0x1F335}, {0x1F337, 0x1F37C}, {0x1F37E, 0x1F393},
    {0x1F3A0, 0x1F3CA}, {0x1F3CF, 0x1F3D3}, {0x1F3E0, 0x1F3F0}, {0x1F3F4, 0x1F3F4},
    {0x1F3F8, 0x1F43E}, {0x1F440, 0x1F440}, {0x1F442, 0x1F4FC}, {0x1F4FF, 0x1F53D},
    {0x1F54B, 0x1F54E}, {0x1F550, 0x1F567}, {0x1F57A, 0x1F57A}, {0x1F595, 0x1F596},
    {0x1F5A4, 0x1F5A4}, {0x1F5FB, 0x1F64F}, {0x1F680, 0x1F6C5}, {0x1F6CC, 0x1F6CC},
    {0x1F6D0, 0x1F6D2}, {0x1F6D5, 0x1F6D7}, {0x1F6EB, 0x1F6EC}, {0x1F6F4, 0x1F6FC},
    {0x1F7E0, 0x1F7EB}, {0x1F90C, 0x1F93A}, {0x1F93C, 0x1F945}, {0x1F947, 0x1F9FF},
    {0x1FA70, 0x1FAFF}, {0x20000, 0x2FFFD}, {0x30000, 0x3FFFD},
};
/* clang-format on */

static int in_table(const range_t *table, size_t count, uint32_t glyph) {
    size_t low = 0, high = count;
    while (low < high) {
        size_t middle = low + (high - low) / 2;
        if (glyph < table[middle].first)
            high = middle;
        else if (glyph > table[middle].last)
            low = middle + 1;
        else
            return 1;
    }
    return 0;
}

int vt_glyph_width(uint32_t glyph) {
    if (glyph < 0x300) return 1; /* Latin, which is nearly everything printed. */
    if (in_table(ZERO_WIDTH, sizeof(ZERO_WIDTH) / sizeof(*ZERO_WIDTH), glyph)) return 0;
    if (in_table(WIDE, sizeof(WIDE) / sizeof(*WIDE), glyph)) return 2;
    return 1;
}

int vt_cell_is_blank(const vt_cell_t *cell) {
    return cell == NULL || cell->glyph == 0 || cell->glyph == ' ';
}

/* --- The screen ------------------------------------------------------------- */

static vt_cell_t blank_cell(const vt_t *vt) {
    vt_cell_t cell;
    memset(&cell, 0, sizeof(cell));
    cell.width = 1;
    /* Erasing paints the background in use and nothing else, as a terminal with
     * background colour erase does: `\e[41m\e[K` is how a status bar is filled. */
    cell.style.bg = vt->pen.bg;
    return cell;
}

static void reset_pen(vt_style_t *pen) {
    memset(pen, 0, sizeof(*pen));
}

int vt_init(vt_t *vt, int cols, int rows) {
    if (vt == NULL || cols < 1 || rows < 1 || (size_t)cols > 100000 || (size_t)rows > 100000)
        return -1;
    memset(vt, 0, sizeof(*vt));
    vt->storage = calloc((size_t)cols * (size_t)rows, sizeof(*vt->storage));
    vt->line = malloc((size_t)rows * sizeof(*vt->line));
    if (vt->storage == NULL || vt->line == NULL) {
        free(vt->storage);
        free(vt->line);
        memset(vt, 0, sizeof(*vt));
        return -1;
    }
    vt->cols = cols;
    vt->rows = rows;
    vt->autowrap = 1;
    for (int row = 0; row < rows; row++) {
        vt->line[row] = vt->storage + (size_t)row * (size_t)cols;
        for (int col = 0; col < cols; col++) vt->line[row][col].width = 1;
    }
    return 0;
}

void vt_destroy(vt_t *vt) {
    if (vt == NULL) return;
    free(vt->storage);
    free(vt->line);
    memset(vt, 0, sizeof(*vt));
}

const vt_cell_t *vt_cell(const vt_t *vt, int col, int row) {
    if (vt == NULL || vt->line == NULL || col < 0 || row < 0 || col >= vt->cols || row >= vt->rows)
        return NULL;
    return &vt->line[row][col];
}

static int clamp(int value, int low, int high) {
    return value < low ? low : (value > high ? high : value);
}

/* A wide character is two cells that live and die together. After anything that
 * can cut one in half, whichever half is left alone becomes a blank. */
static void repair_line(vt_t *vt, int row) {
    vt_cell_t *line = vt->line[row];
    for (int col = 0; col < vt->cols; col++) {
        int orphan = 0;
        if (line[col].width == 2)
            orphan = col + 1 >= vt->cols || line[col + 1].width != 0;
        else if (line[col].width == 0)
            orphan = col == 0 || line[col - 1].width != 2;
        if (orphan) {
            line[col].glyph = 0;
            line[col].width = 1;
        }
    }
}

static void erase_cells(vt_t *vt, int row, int from, int to) {
    vt_cell_t blank = blank_cell(vt);
    from = clamp(from, 0, vt->cols);
    to = clamp(to, 0, vt->cols);
    for (int col = from; col < to; col++) vt->line[row][col] = blank;
    repair_line(vt, row);
}

static void erase_lines(vt_t *vt, int from, int to) {
    for (int row = clamp(from, 0, vt->rows); row < clamp(to, 0, vt->rows); row++)
        erase_cells(vt, row, 0, vt->cols);
}

/* Lines [first, last) turn by `count`: the first `count` go to the end, blank. A
 * scroll is turning pointers, so a flood of output costs one line a line and not a
 * whole screen of cells. */
static void turn_lines(vt_t *vt, int first, int last, int count, int upwards) {
    int span = last - first;
    if (span <= 0 || count <= 0) return;
    if (count > span) count = span;
    vt_cell_t **block = vt->line + first;
    vt_cell_t *kept[64];
    vt_cell_t **moved = count <= 64 ? kept : malloc((size_t)count * sizeof(*moved));
    if (moved == NULL) { /* No memory for the bookkeeping: the lines are only erased. */
        for (int row = first; row < last; row++) erase_cells(vt, row, 0, vt->cols);
        return;
    }
    if (upwards) {
        memcpy(moved, block, (size_t)count * sizeof(*moved));
        memmove(block, block + count, (size_t)(span - count) * sizeof(*block));
        memcpy(block + (span - count), moved, (size_t)count * sizeof(*moved));
        for (int row = last - count; row < last; row++) erase_cells(vt, row, 0, vt->cols);
    } else {
        memcpy(moved, block + (span - count), (size_t)count * sizeof(*moved));
        memmove(block + count, block, (size_t)(span - count) * sizeof(*block));
        memcpy(block, moved, (size_t)count * sizeof(*moved));
        for (int row = first; row < first + count; row++) erase_cells(vt, row, 0, vt->cols);
    }
    if (moved != kept) free(moved);
}

static void scroll_up(vt_t *vt, int count) {
    turn_lines(vt, 0, vt->rows, count, 1);
    vt->scrolled += count < vt->rows ? count : vt->rows;
}

static void scroll_down(vt_t *vt, int count) {
    turn_lines(vt, 0, vt->rows, count, 0);
}

/* Down a line, scrolling at the bottom. */
static void index_down(vt_t *vt) {
    if (vt->cursor_row >= vt->rows - 1)
        scroll_up(vt, 1);
    else
        vt->cursor_row++;
}

static void index_up(vt_t *vt) {
    if (vt->cursor_row <= 0)
        scroll_down(vt, 1);
    else
        vt->cursor_row--;
}

/* --- Printing --------------------------------------------------------------- */

/* The cell is about to be written over: if it is half of a wide character, the
 * other half goes blank. */
static void release_cell(vt_t *vt, int row, int col) {
    vt_cell_t *line = vt->line[row];
    if (line[col].width == 2 && col + 1 < vt->cols) {
        line[col + 1].glyph = 0;
        line[col + 1].width = 1;
    } else if (line[col].width == 0 && col > 0) {
        line[col - 1].glyph = 0;
        line[col - 1].width = 1;
    }
}

static void print_glyph(vt_t *vt, uint32_t glyph, int width) {
    if (width == 2 && vt->cols < 2) return;
    if (vt->wrap_pending) {
        vt->wrap_pending = 0;
        if (vt->autowrap) {
            vt->cursor_col = 0;
            index_down(vt);
        }
    }
    if (width == 2 && vt->cursor_col == vt->cols - 1) {
        /* No room for both halves: wrap first, leaving this cell empty, as xterm
         * does; with wrapping off the character has nowhere to go. */
        if (!vt->autowrap) return;
        release_cell(vt, vt->cursor_row, vt->cursor_col);
        vt->line[vt->cursor_row][vt->cursor_col] = blank_cell(vt);
        vt->cursor_col = 0;
        index_down(vt);
    }

    vt_cell_t *line = vt->line[vt->cursor_row];
    int col = vt->cursor_col;
    release_cell(vt, vt->cursor_row, col);
    if (width == 2) release_cell(vt, vt->cursor_row, col + 1);
    line[col].glyph = glyph;
    line[col].style = vt->pen;
    line[col].width = (uint8_t)width;
    if (width == 2) {
        line[col + 1].glyph = 0;
        line[col + 1].style = vt->pen;
        line[col + 1].width = 0;
    }
    vt->last_glyph = glyph;
    vt->printed++;

    col += width;
    if (col >= vt->cols) {
        vt->cursor_col = vt->cols - 1;
        vt->wrap_pending = vt->autowrap;
    } else {
        vt->cursor_col = col;
    }
}

static void print_code_point(vt_t *vt, uint32_t glyph) {
    if (glyph >= 0x80 && glyph < 0xA0) return; /* C1 controls draw nothing. */
    int width = vt_glyph_width(glyph);
    if (width == 0) return;
    print_glyph(vt, glyph, width);
}

/* --- Controls and sequences -------------------------------------------------- */

static void move_to(vt_t *vt, int col, int row) {
    vt->cursor_col = clamp(col, 0, vt->cols - 1);
    vt->cursor_row = clamp(row, 0, vt->rows - 1);
    vt->wrap_pending = 0;
}

static void control(vt_t *vt, unsigned char byte) {
    switch (byte) {
        case 0x08: /* BS */
            move_to(vt, vt->cursor_col - 1, vt->cursor_row);
            break;
        case 0x09: /* HT: the next multiple of eight, or the last column. */
            move_to(vt, (vt->cursor_col / TAB_WIDTH + 1) * TAB_WIDTH, vt->cursor_row);
            break;
        case 0x0A: /* LF, as a terminal with onlcr shows it: a new line at column 0. */
            vt->cursor_col = 0;
            vt->wrap_pending = 0;
            index_down(vt);
            break;
        case 0x0B: /* VT and FF go down a line and stay in their column. */
        case 0x0C:
            vt->wrap_pending = 0;
            index_down(vt);
            break;
        case 0x0D: /* CR */
            move_to(vt, 0, vt->cursor_row);
            break;
        default: /* BEL, SO, SI, and the rest draw nothing. */
            break;
    }
}

static void save_cursor(vt_t *vt) {
    vt->saved = 1;
    vt->saved_col = vt->cursor_col;
    vt->saved_row = vt->cursor_row;
    vt->saved_wrap_pending = vt->wrap_pending;
    vt->saved_pen = vt->pen;
}

static void restore_cursor(vt_t *vt) {
    if (!vt->saved) {
        move_to(vt, 0, 0);
        reset_pen(&vt->pen);
        return;
    }
    vt->cursor_col = clamp(vt->saved_col, 0, vt->cols - 1);
    vt->cursor_row = clamp(vt->saved_row, 0, vt->rows - 1);
    vt->wrap_pending = vt->saved_wrap_pending;
    vt->pen = vt->saved_pen;
}

/* ESC c: back to how it was switched on. */
static void full_reset(vt_t *vt) {
    reset_pen(&vt->pen);
    erase_lines(vt, 0, vt->rows);
    vt->cursor_col = vt->cursor_row = 0;
    vt->wrap_pending = 0;
    vt->autowrap = 1;
    vt->saved = 0;
}

static vt_colour_t colour_of(int index) {
    vt_colour_t colour;
    memset(&colour, 0, sizeof(colour));
    index = clamp(index, 0, 255);
    colour.kind = index < 16 ? VT_COLOUR_ANSI : VT_COLOUR_INDEXED;
    colour.value[0] = (uint8_t)index;
    return colour;
}

static vt_colour_t colour_of_rgb(int red, int green, int blue) {
    vt_colour_t colour;
    colour.kind = VT_COLOUR_RGB;
    colour.value[0] = (uint8_t)clamp(red, 0, 255);
    colour.value[1] = (uint8_t)clamp(green, 0, 255);
    colour.value[2] = (uint8_t)clamp(blue, 0, 255);
    return colour;
}

/* SGR 38 and 48 take their colour from what follows: 5;n or 2;r;g;b with
 * semicolons, 5:n or 2:r:g:b with colons (and the form with an empty colour space
 * id, 2::r:g:b, that some tools write). Returns how many parameters after the
 * 38 were consumed. `target` is NULL for 58, the underline colour, which is read
 * the same way and thrown away. */
static int read_extended_colour(const vt_t *vt, int at, vt_colour_t *target) {
    int count = vt->param_count < VT_MAX_PARAMS ? vt->param_count : VT_MAX_PARAMS;
    const int *p = vt->params;
    if (at + 1 >= count) return count - at - 1;

    if (vt->colon_after[at]) {
        /* Everything joined to it by colons is one group. */
        int end = at + 1;
        while (end < count && vt->colon_after[end - 1]) end++;
        int members = end - at - 1;
        if (p[at + 1] == 5 && members >= 2) {
            if (target) *target = colour_of(p[at + 2]);
        } else if (p[at + 1] == 2 && members >= 4) {
            /* r:g:b, or colourspace:r:g:b with the colour space usually empty. */
            int first = members >= 5 ? at + 3 : at + 2;
            if (target) *target = colour_of_rgb(p[first], p[first + 1], p[first + 2]);
        }
        return members;
    }
    if (p[at + 1] == 5) {
        if (at + 2 >= count) return count - at - 1;
        if (target) *target = colour_of(p[at + 2]);
        return 2;
    }
    if (p[at + 1] == 2) {
        if (at + 4 >= count) return count - at - 1; /* Cut short: drop what is there. */
        if (target) *target = colour_of_rgb(p[at + 2], p[at + 3], p[at + 4]);
        return 4;
    }
    return 1; /* Not a form we know; the mode number goes with it. */
}

static void select_graphic_rendition(vt_t *vt) {
    int count = vt->param_count < VT_MAX_PARAMS ? vt->param_count : VT_MAX_PARAMS;
    for (int i = 0; i < count; i++) {
        int code = vt->params[i];
        vt_style_t *pen = &vt->pen;
        if (code == 0) {
            reset_pen(pen);
        } else if (code == 1) {
            pen->attributes |= VT_BOLD;
        } else if (code == 2) {
            pen->attributes |= VT_DIM;
        } else if (code == 3) {
            pen->attributes |= VT_ITALIC;
        } else if (code == 4 || code == 21) {
            pen->attributes |= VT_UNDERLINE;
        } else if (code == 7) {
            pen->attributes |= VT_REVERSE;
        } else if (code == 22) {
            pen->attributes &= (uint8_t) ~(VT_BOLD | VT_DIM);
        } else if (code == 23) {
            pen->attributes &= (uint8_t)~VT_ITALIC;
        } else if (code == 24) {
            pen->attributes &= (uint8_t)~VT_UNDERLINE;
        } else if (code == 27) {
            pen->attributes &= (uint8_t)~VT_REVERSE;
        } else if (code >= 30 && code <= 37) {
            pen->fg = colour_of(code - 30);
        } else if (code >= 90 && code <= 97) {
            pen->fg = colour_of(code - 90 + 8);
        } else if (code >= 40 && code <= 47) {
            pen->bg = colour_of(code - 40);
        } else if (code >= 100 && code <= 107) {
            pen->bg = colour_of(code - 100 + 8);
        } else if (code == 39) {
            memset(&pen->fg, 0, sizeof(pen->fg));
        } else if (code == 49) {
            memset(&pen->bg, 0, sizeof(pen->bg));
        } else if (code == 38) {
            i += read_extended_colour(vt, i, &pen->fg);
        } else if (code == 48) {
            i += read_extended_colour(vt, i, &pen->bg);
        } else if (code == 58) {
            i += read_extended_colour(vt, i, NULL);
        }
        /* Blink, conceal, strikethrough, overline, fonts: not kept. */
    }
}

/* A parameter that is missing or zero means the default, which for a count is
 * one. */
static int count_param(const vt_t *vt, int at) {
    if (at >= vt->param_count || at >= VT_MAX_PARAMS || vt->params[at] <= 0) return 1;
    return vt->params[at];
}

static int plain_param(const vt_t *vt, int at) {
    if (at >= vt->param_count || at >= VT_MAX_PARAMS) return 0;
    return vt->params[at];
}

static void erase_in_display(vt_t *vt) {
    int mode = plain_param(vt, 0);
    if (mode == 0) {
        erase_cells(vt, vt->cursor_row, vt->cursor_col, vt->cols);
        erase_lines(vt, vt->cursor_row + 1, vt->rows);
    } else if (mode == 1) {
        erase_lines(vt, 0, vt->cursor_row);
        erase_cells(vt, vt->cursor_row, 0, vt->cursor_col + 1);
    } else if (mode == 2 || mode == 3) {
        erase_lines(vt, 0, vt->rows);
    }
}

static void erase_in_line(vt_t *vt) {
    int mode = plain_param(vt, 0);
    if (mode == 0)
        erase_cells(vt, vt->cursor_row, vt->cursor_col, vt->cols);
    else if (mode == 1)
        erase_cells(vt, vt->cursor_row, 0, vt->cursor_col + 1);
    else if (mode == 2)
        erase_cells(vt, vt->cursor_row, 0, vt->cols);
}

static void insert_cells(vt_t *vt, int count) {
    vt_cell_t *line = vt->line[vt->cursor_row];
    int room = vt->cols - vt->cursor_col;
    if (count > room) count = room;
    memmove(line + vt->cursor_col + count, line + vt->cursor_col,
            (size_t)(room - count) * sizeof(*line));
    vt->wrap_pending = 0;
    erase_cells(vt, vt->cursor_row, vt->cursor_col, vt->cursor_col + count);
}

static void delete_cells(vt_t *vt, int count) {
    vt_cell_t *line = vt->line[vt->cursor_row];
    int room = vt->cols - vt->cursor_col;
    if (count > room) count = room;
    memmove(line + vt->cursor_col, line + vt->cursor_col + count,
            (size_t)(room - count) * sizeof(*line));
    vt->wrap_pending = 0;
    erase_cells(vt, vt->cursor_row, vt->cols - count, vt->cols);
}

static void set_private_modes(vt_t *vt, int on) {
    int count = vt->param_count < VT_MAX_PARAMS ? vt->param_count : VT_MAX_PARAMS;
    for (int i = 0; i < count; i++)
        if (vt->params[i] == 7) {
            vt->autowrap = on;
            if (!on) vt->wrap_pending = 0;
        }
}

static void dispatch_csi(vt_t *vt, unsigned char final) {
    int n;
    if (vt->intermediate) return; /* DECSCUSR, DECSTR and the like: nothing for us. */
    if (vt->marker != 0) {
        /* ?h and ?l carry DECAWM; the rest of the private sequences, mouse and
         * cursor modes, the kitty keyboard protocol that reuses `u`, are not ours. */
        if (vt->marker == '?' && (final == 'h' || final == 'l'))
            set_private_modes(vt, final == 'h');
        return;
    }
    switch (final) {
        case 'A': /* CUU */
            move_to(vt, vt->cursor_col, vt->cursor_row - count_param(vt, 0));
            break;
        case 'B': /* CUD */
        case 'e': /* VPR */
            move_to(vt, vt->cursor_col, vt->cursor_row + count_param(vt, 0));
            break;
        case 'C': /* CUF */
        case 'a': /* HPR */
            move_to(vt, vt->cursor_col + count_param(vt, 0), vt->cursor_row);
            break;
        case 'D': /* CUB */
            move_to(vt, vt->cursor_col - count_param(vt, 0), vt->cursor_row);
            break;
        case 'E': /* CNL */
            move_to(vt, 0, vt->cursor_row + count_param(vt, 0));
            break;
        case 'F': /* CPL */
            move_to(vt, 0, vt->cursor_row - count_param(vt, 0));
            break;
        case 'G': /* CHA */
        case '`': /* HPA */
            move_to(vt, count_param(vt, 0) - 1, vt->cursor_row);
            break;
        case 'd': /* VPA */
            move_to(vt, vt->cursor_col, count_param(vt, 0) - 1);
            break;
        case 'H': /* CUP */
        case 'f': /* HVP */
            move_to(vt, count_param(vt, 1) - 1, count_param(vt, 0) - 1);
            break;
        case 'J':
            erase_in_display(vt);
            break;
        case 'K':
            erase_in_line(vt);
            break;
        case 'L': /* IL */
            n = count_param(vt, 0);
            turn_lines(vt, vt->cursor_row, vt->rows, n, 0);
            vt->cursor_col = 0;
            vt->wrap_pending = 0;
            break;
        case 'M': /* DL */
            n = count_param(vt, 0);
            turn_lines(vt, vt->cursor_row, vt->rows, n, 1);
            vt->cursor_col = 0;
            vt->wrap_pending = 0;
            break;
        case '@': /* ICH */
            insert_cells(vt, count_param(vt, 0));
            break;
        case 'P': /* DCH */
            delete_cells(vt, count_param(vt, 0));
            break;
        case 'X': /* ECH */
            n = count_param(vt, 0);
            erase_cells(vt, vt->cursor_row, vt->cursor_col, vt->cursor_col + n);
            break;
        case 'S': /* SU */
            scroll_up(vt, count_param(vt, 0));
            break;
        case 'T': /* SD */
            scroll_down(vt, count_param(vt, 0));
            break;
        case 'b': /* REP */
            if (vt->last_glyph != 0) {
                n = count_param(vt, 0);
                if (n > vt->cols * vt->rows) n = vt->cols * vt->rows;
                for (int i = 0; i < n; i++) print_code_point(vt, vt->last_glyph);
            }
            break;
        case 'm':
            select_graphic_rendition(vt);
            break;
        case 's':
            if (vt->param_count <= 1 && !vt->params_seen) save_cursor(vt);
            break;
        case 'u':
            restore_cursor(vt);
            break;
        default: /* Modes, scrolling regions, reports, window ops: read and dropped. */
            break;
    }
}

/* --- The parser ------------------------------------------------------------- */

static void start_csi(vt_t *vt) {
    vt->state = CSI;
    vt->param_count = 1;
    memset(vt->params, 0, sizeof(vt->params));
    memset(vt->colon_after, 0, sizeof(vt->colon_after));
    vt->params_seen = 0;
    vt->marker = 0;
    vt->intermediate = 0;
}

static void feed_escape(vt_t *vt, unsigned char byte) {
    if (byte == 0x1B) return; /* ESC ESC: still one escape. */
    if (byte == 0x18 || byte == 0x1A) {
        vt->state = GROUND;
        return;
    }
    if (byte < 0x20) {
        control(vt, byte); /* A control in the middle of an escape is carried out. */
        return;
    }
    if (byte >= 0x20 && byte <= 0x2F) {
        vt->state = ESCAPE_SKIP;
        return;
    }
    vt->state = GROUND;
    switch (byte) {
        case '[':
            start_csi(vt);
            break;
        case ']':
            vt->state = OSC;
            break;
        case 'P': /* DCS */
        case 'X': /* SOS */
        case '^': /* PM */
        case '_': /* APC */
            vt->state = STRING;
            break;
        case '7':
            save_cursor(vt);
            break;
        case '8':
            restore_cursor(vt);
            break;
        case 'c':
            full_reset(vt);
            break;
        case 'D': /* IND */
            vt->wrap_pending = 0;
            index_down(vt);
            break;
        case 'E': /* NEL */
            vt->cursor_col = 0;
            vt->wrap_pending = 0;
            index_down(vt);
            break;
        case 'M': /* RI */
            vt->wrap_pending = 0;
            index_up(vt);
            break;
        default: /* ST on its own, keypad modes, and every other escape. */
            break;
    }
}

static void feed_csi(vt_t *vt, unsigned char byte) {
    if (byte == 0x1B) {
        vt->state = ESCAPE;
        return;
    }
    if (byte == 0x18 || byte == 0x1A) {
        vt->state = GROUND;
        return;
    }
    if (byte < 0x20) {
        control(vt, byte);
        return;
    }
    if (byte >= 0x80) {
        /* Not part of any sequence: the sequence is abandoned and the byte is read
         * as what it is, the start of a UTF-8 character or a stray one. */
        vt->state = GROUND;
        vt_feed(vt, &byte, 1);
        return;
    }
    if (byte >= '0' && byte <= '9') {
        if (vt->intermediate) {
            vt->state = CSI_IGNORE;
            return;
        }
        vt->params_seen = 1;
        int at = vt->param_count - 1;
        if (at < VT_MAX_PARAMS) {
            int value = vt->params[at] * 10 + (byte - '0');
            vt->params[at] = value > PARAM_LIMIT ? PARAM_LIMIT : value;
        }
        return;
    }
    if (byte == ';' || byte == ':') {
        if (vt->intermediate) {
            vt->state = CSI_IGNORE;
            return;
        }
        vt->params_seen = 1;
        int at = vt->param_count - 1;
        if (byte == ':' && at < VT_MAX_PARAMS) vt->colon_after[at] = 1;
        if (vt->param_count < 1000) vt->param_count++;
        return;
    }
    if (byte >= '<' && byte <= '?') {
        if (vt->params_seen || vt->marker || vt->intermediate)
            vt->state = CSI_IGNORE;
        else
            vt->marker = byte;
        return;
    }
    if (byte >= 0x20 && byte <= 0x2F) {
        vt->intermediate = 1;
        return;
    }
    if (byte >= 0x40 && byte <= 0x7E) {
        vt->state = GROUND;
        dispatch_csi(vt, byte);
        return;
    }
    /* DEL, which a terminal ignores wherever it comes. */
}

static void feed_ground(vt_t *vt, unsigned char byte) {
    if (vt->utf8_need > 0) {
        if ((byte & 0xC0) == 0x80) {
            vt->utf8_code = (vt->utf8_code << 6) | (byte & 0x3Fu);
            if (--vt->utf8_need == 0) {
                uint32_t code = vt->utf8_code;
                /* Overlong forms, surrogates and anything past U+10FFFF are not
                 * characters, and a terminal must not take them for some. */
                if (code < vt->utf8_least || (code >= 0xD800 && code <= 0xDFFF) || code > 0x10FFFF)
                    code = REPLACEMENT;
                print_code_point(vt, code);
            }
            return;
        }
        /* Cut short by something that is not a continuation: that character is
         * lost, and this byte is read afresh. */
        vt->utf8_need = 0;
        print_code_point(vt, REPLACEMENT);
    }
    if (byte == 0x1B) {
        vt->state = ESCAPE;
    } else if (byte < 0x20) {
        control(vt, byte);
    } else if (byte < 0x7F) {
        print_code_point(vt, byte);
    } else if (byte == 0x7F) {
        /* DEL draws nothing. */
    } else if (byte >= 0xC2 && byte <= 0xDF) {
        vt->utf8_need = 1;
        vt->utf8_code = byte & 0x1Fu;
        vt->utf8_least = 0x80;
    } else if (byte >= 0xE0 && byte <= 0xEF) {
        vt->utf8_need = 2;
        vt->utf8_code = byte & 0x0Fu;
        vt->utf8_least = 0x800;
    } else if (byte >= 0xF0 && byte <= 0xF4) {
        vt->utf8_need = 3;
        vt->utf8_code = byte & 0x07u;
        vt->utf8_least = 0x10000;
    } else {
        print_code_point(vt, REPLACEMENT); /* A stray continuation, C0, C1, F5 to FF. */
    }
}

static void feed_byte(vt_t *vt, unsigned char byte) {
    switch (vt->state) {
        case GROUND:
            feed_ground(vt, byte);
            break;
        case ESCAPE:
            feed_escape(vt, byte);
            break;
        case ESCAPE_SKIP:
            if (byte == 0x1B)
                vt->state = ESCAPE;
            else if (byte == 0x18 || byte == 0x1A)
                vt->state = GROUND;
            else if (byte < 0x20)
                control(vt, byte);
            else if (byte >= 0x30)
                vt->state = GROUND;
            break;
        case CSI:
            feed_csi(vt, byte);
            break;
        case CSI_IGNORE:
            if (byte == 0x1B)
                vt->state = ESCAPE;
            else if (byte == 0x18 || byte == 0x1A || (byte >= 0x40 && byte <= 0x7E))
                vt->state = GROUND;
            else if (byte < 0x20)
                control(vt, byte);
            break;
        case OSC:
        case STRING:
            if (byte == 0x1B)
                vt->state = ESCAPE; /* ESC \ is ST; anything else starts a new escape. */
            else if (byte == 0x18 || byte == 0x1A || (byte == 0x07 && vt->state == OSC))
                vt->state = GROUND;
            break;
        default:
            vt->state = GROUND;
            break;
    }
}

void vt_feed(vt_t *vt, const void *bytes, size_t length) {
    const unsigned char *data = bytes;
    if (vt == NULL || vt->line == NULL || data == NULL) return;
    for (size_t i = 0; i < length; i++) feed_byte(vt, data[i]);
}

void vt_finish(vt_t *vt) {
    if (vt == NULL || vt->line == NULL) return;
    if (vt->utf8_need > 0) {
        vt->utf8_need = 0;
        print_code_point(vt, REPLACEMENT);
    }
    vt->state = GROUND;
}
