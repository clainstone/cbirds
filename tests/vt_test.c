#include "../vt.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* A screen of the size asked for, fed a string. */
static vt_t make(int cols, int rows) {
    vt_t vt;
    assert(vt_init(&vt, cols, rows) == 0);
    return vt;
}

static void feed(vt_t *vt, const char *text) {
    vt_feed(vt, text, strlen(text));
}

static const vt_cell_t *cell(const vt_t *vt, int col, int row) {
    const vt_cell_t *c = vt_cell(vt, col, row);
    assert(c != NULL);
    return c;
}

static size_t encode(uint32_t glyph, char *out) {
    if (glyph < 0x80) {
        out[0] = (char)glyph;
        return 1;
    }
    if (glyph < 0x800) {
        out[0] = (char)(0xC0 | (glyph >> 6));
        out[1] = (char)(0x80 | (glyph & 0x3F));
        return 2;
    }
    if (glyph < 0x10000) {
        out[0] = (char)(0xE0 | (glyph >> 12));
        out[1] = (char)(0x80 | ((glyph >> 6) & 0x3F));
        out[2] = (char)(0x80 | (glyph & 0x3F));
        return 3;
    }
    out[0] = (char)(0xF0 | (glyph >> 18));
    out[1] = (char)(0x80 | ((glyph >> 12) & 0x3F));
    out[2] = (char)(0x80 | ((glyph >> 6) & 0x3F));
    out[3] = (char)(0x80 | (glyph & 0x3F));
    return 4;
}

/* A row as UTF-8 text: blanks as spaces, the second half of a wide character
 * skipped, trailing spaces cut. Enough to read a screen in an assert. */
static void row_text(const vt_t *vt, int row, char *out, size_t size) {
    size_t at = 0;
    for (int col = 0; col < vt->cols; col++) {
        const vt_cell_t *c = cell(vt, col, row);
        if (c->width == 0) continue;
        char bytes[4];
        size_t length = encode(c->glyph ? c->glyph : ' ', bytes);
        assert(at + length + 1 < size);
        memcpy(out + at, bytes, length);
        at += length;
    }
    while (at > 0 && out[at - 1] == ' ') at--;
    out[at] = '\0';
}

static int row_is(const vt_t *vt, int row, const char *expected) {
    char text[1024];
    row_text(vt, row, text, sizeof(text));
    return strcmp(text, expected) == 0;
}

/* The invariants no input may break. */
static void check_the_screen_is_sound(const vt_t *vt) {
    assert(vt->cursor_col >= 0 && vt->cursor_col < vt->cols);
    assert(vt->cursor_row >= 0 && vt->cursor_row < vt->rows);
    for (int row = 0; row < vt->rows; row++)
        for (int col = 0; col < vt->cols; col++) {
            const vt_cell_t *c = cell(vt, col, row);
            assert(c->width <= 2);
            if (c->width == 2) {
                assert(col + 1 < vt->cols);
                assert(cell(vt, col + 1, row)->width == 0);
                assert(c->glyph != 0);
            }
            if (c->width == 0) {
                assert(col > 0 && cell(vt, col - 1, row)->width == 2);
                assert(c->glyph == 0);
            }
        }
}

static void test_plain_text_lands_where_a_cleared_screen_would_put_it(void) {
    vt_t vt = make(20, 4);
    feed(&vt, "hello");
    assert(row_is(&vt, 0, "hello"));
    assert(vt.cursor_col == 5 && vt.cursor_row == 0);
    assert(cell(&vt, 0, 0)->glyph == 'h' && cell(&vt, 0, 0)->width == 1);
    /* Nothing was printed past it, and the style is the default one. */
    assert(cell(&vt, 5, 0)->glyph == 0);
    assert(cell(&vt, 0, 0)->style.fg.kind == VT_COLOUR_DEFAULT);
    assert(cell(&vt, 0, 0)->style.bg.kind == VT_COLOUR_DEFAULT);
    assert(cell(&vt, 0, 0)->style.attributes == 0);
    vt_destroy(&vt);
}

static void test_a_newline_is_a_new_line_at_column_zero(void) {
    vt_t vt = make(20, 4);
    /* A pipe carries bare line feeds; the pty a command would have written to turns
     * each into a return and a line feed, and so does this. */
    feed(&vt, "one\ntwo\n\nfour");
    assert(row_is(&vt, 0, "one"));
    assert(row_is(&vt, 1, "two"));
    assert(row_is(&vt, 2, ""));
    assert(row_is(&vt, 3, "four"));
    vt_destroy(&vt);
}

static void test_a_return_goes_back_and_overwrites(void) {
    vt_t vt = make(20, 2);
    feed(&vt, "progress 10%\rprogress 99%");
    assert(row_is(&vt, 0, "progress 99%"));
    feed(&vt, "\r\nab\r");
    feed(&vt, "X");
    assert(row_is(&vt, 1, "Xb"));
    vt_destroy(&vt);
}

static void test_backspace_steps_back_and_stops_at_the_margin(void) {
    vt_t vt = make(10, 2);
    feed(&vt, "abc\b\bX");
    assert(row_is(&vt, 0, "aXc"));
    feed(&vt, "\r\b\b\bY");
    assert(row_is(&vt, 0, "YXc"));
    vt_destroy(&vt);
}

static void test_a_tab_goes_to_the_next_multiple_of_eight(void) {
    vt_t vt = make(20, 2);
    feed(&vt, "a\tb\tc");
    assert(cell(&vt, 0, 0)->glyph == 'a');
    assert(cell(&vt, 8, 0)->glyph == 'b');
    assert(cell(&vt, 16, 0)->glyph == 'c');
    /* Past the last stop it ends at the last column, and does not wrap. */
    feed(&vt, "\t\t\tZ");
    assert(vt.cursor_row == 0);
    assert(cell(&vt, 19, 0)->glyph == 'Z');
    vt_destroy(&vt);
}

static void test_a_line_longer_than_the_screen_wraps(void) {
    vt_t vt = make(5, 3);
    feed(&vt, "abcdefghijkl");
    assert(row_is(&vt, 0, "abcde"));
    assert(row_is(&vt, 1, "fghij"));
    assert(row_is(&vt, 2, "kl"));
    vt_destroy(&vt);
}

static void test_a_line_exactly_as_wide_as_the_screen_wraps_only_when_more_comes(void) {
    vt_t vt = make(5, 4);
    /* The cursor waits in the last column, so the newline after a full line is
     * the only line break and no blank line appears between them. */
    feed(&vt, "abcde\nfghij\nk");
    assert(row_is(&vt, 0, "abcde"));
    assert(row_is(&vt, 1, "fghij"));
    assert(row_is(&vt, 2, "k"));
    assert(vt.scrolled == 0);
    vt_destroy(&vt);
}

static void test_text_taller_than_the_screen_keeps_the_last_screenful(void) {
    vt_t vt = make(10, 3);
    feed(&vt, "1\n2\n3\n4\n5\n6");
    assert(row_is(&vt, 0, "4"));
    assert(row_is(&vt, 1, "5"));
    assert(row_is(&vt, 2, "6"));
    assert(vt.scrolled == 3);
    check_the_screen_is_sound(&vt);

    /* A great deal of it, as `yes` would send, costs nothing and ends the same. */
    vt_t big = make(80, 24);
    char line[] = "the quick brown fox jumps over the lazy dog\n";
    for (int i = 0; i < 20000; i++) vt_feed(&big, line, sizeof(line) - 1);
    assert(row_is(&big, 0, "the quick brown fox jumps over the lazy dog"));
    assert(row_is(&big, 23, ""));
    assert(big.cursor_row == 23);
    assert(big.scrolled == 20000 - 23);
    vt_destroy(&big);
    vt_destroy(&vt);
}

static void test_utf8_is_one_cell_for_each_code_point(void) {
    vt_t vt = make(20, 2);
    feed(&vt, "caf\xC3\xA9 \xE2\x82\xAC \xE2\x96\x88\xE2\x96\x88 \xF0\x9F\x90\xA6");
    assert(cell(&vt, 3, 0)->glyph == 0xE9);
    assert(cell(&vt, 3, 0)->width == 1);
    assert(cell(&vt, 5, 0)->glyph == 0x20AC);
    assert(cell(&vt, 7, 0)->glyph == 0x2588 && cell(&vt, 8, 0)->glyph == 0x2588);
    assert(cell(&vt, 10, 0)->glyph == 0x1F426);
    assert(cell(&vt, 10, 0)->width == 2);
    assert(cell(&vt, 11, 0)->width == 0);
    check_the_screen_is_sound(&vt);
    vt_destroy(&vt);
}

static void test_invalid_utf8_becomes_the_replacement_character(void) {
    static const struct {
        const char *name;
        const char *bytes;
    } bad[] = {
        {"a lone continuation byte", "a\x80z"},
        {"a byte that is never valid", "a\xFFz"},
        {"an overlong slash", "a\xC0\xAFz"},
        {"an overlong three byte form", "a\xE0\x80\xAFz"},
        {"a surrogate", "a\xED\xA0\x80z"},
        {"past the last code point", "a\xF4\x90\x80\x80z"},
        {"a lead byte too far", "a\xF8\x88\x80\x80\x80z"},
        {"a lead byte cut short by ASCII", "a\xE2\x82z"},
        {"a lead byte cut short by another lead byte", "a\xE2\xC3\xA9z"},
    };
    for (size_t i = 0; i < sizeof(bad) / sizeof(*bad); i++) {
        vt_t vt = make(20, 2);
        feed(&vt, bad[i].bytes);
        int seen = 0;
        for (int col = 0; col < 20; col++)
            if (cell(&vt, col, 0)->glyph == 0xFFFD) seen++;
        if (seen == 0) fprintf(stderr, "no replacement for %s\n", bad[i].name);
        assert(seen >= 1);
        assert(cell(&vt, 0, 0)->glyph == 'a');
        /* The ASCII after the damage is not lost with it. */
        int z = 0;
        for (int col = 0; col < 20; col++)
            if (cell(&vt, col, 0)->glyph == 'z') z++;
        assert(z == 1);
        check_the_screen_is_sound(&vt);
        vt_destroy(&vt);
    }
    /* Cut short by another lead byte keeps that one whole. */
    vt_t vt = make(20, 2);
    feed(&vt, "\xE2\xC3\xA9");
    assert(cell(&vt, 0, 0)->glyph == 0xFFFD);
    assert(cell(&vt, 1, 0)->glyph == 0xE9);
    vt_destroy(&vt);
}

static void test_utf8_cut_short_at_the_end_is_finished_as_a_replacement(void) {
    vt_t vt = make(10, 2);
    feed(&vt, "ab\xE2\x82");
    assert(row_is(&vt, 0, "ab"));
    vt_finish(&vt);
    assert(cell(&vt, 2, 0)->glyph == 0xFFFD);
    /* And a complete character at the end needs no finishing. */
    vt_t whole = make(10, 2);
    feed(&whole, "ab\xE2\x82\xAC");
    vt_finish(&whole);
    assert(cell(&whole, 2, 0)->glyph == 0x20AC);
    assert(cell(&whole, 3, 0)->glyph == 0);
    vt_destroy(&whole);
    vt_destroy(&vt);
}

static void test_utf8_may_arrive_in_pieces(void) {
    vt_t vt = make(10, 2);
    static const char bytes[] = "x\xE2\x82\xAC\xF0\x9F\x90\xA6y";
    for (size_t i = 0; i + 1 < sizeof(bytes); i++) vt_feed(&vt, bytes + i, 1);
    assert(cell(&vt, 0, 0)->glyph == 'x');
    assert(cell(&vt, 1, 0)->glyph == 0x20AC);
    assert(cell(&vt, 2, 0)->glyph == 0x1F426 && cell(&vt, 2, 0)->width == 2);
    assert(cell(&vt, 4, 0)->glyph == 'y');
    vt_destroy(&vt);
}

static void test_wide_characters_take_two_cells(void) {
    vt_t vt = make(12, 2);
    feed(&vt,
         "a\xE4\xB8\xAD\xE6\x96\x87"
         "b");
    /* a, then two wide characters of two cells each, then b. */
    assert(cell(&vt, 1, 0)->glyph == 0x4E2D && cell(&vt, 1, 0)->width == 2);
    assert(cell(&vt, 2, 0)->width == 0 && cell(&vt, 2, 0)->glyph == 0);
    assert(cell(&vt, 3, 0)->glyph == 0x6587 && cell(&vt, 3, 0)->width == 2);
    assert(cell(&vt, 5, 0)->glyph == 'b');
    assert(vt.cursor_col == 6);
    /* Both halves wear the pen. */
    vt_t styled = make(8, 1);
    feed(&styled, "\033[31m\xE4\xB8\xAD");
    assert(cell(&styled, 0, 0)->style.fg.kind == VT_COLOUR_ANSI);
    assert(cell(&styled, 1, 0)->style.fg.kind == VT_COLOUR_ANSI);
    vt_destroy(&styled);
    vt_destroy(&vt);
}

static void test_a_wide_character_at_the_right_margin_wraps_whole(void) {
    vt_t vt = make(5, 3);
    feed(&vt,
         "abcd\xE4\xB8\xAD"
         "e");
    /* Four cells are taken and the last cannot hold both halves: it stays empty and
     * the character begins the next line. */
    assert(row_is(&vt, 0, "abcd"));
    assert(cell(&vt, 4, 0)->glyph == 0);
    assert(cell(&vt, 0, 1)->glyph == 0x4E2D && cell(&vt, 0, 1)->width == 2);
    assert(cell(&vt, 2, 1)->glyph == 'e');
    check_the_screen_is_sound(&vt);

    /* With wrapping off there is nowhere for it to go, and it is dropped. */
    vt_t nowrap = make(5, 3);
    feed(&nowrap,
         "\033[?7labcd\xE4\xB8\xAD"
         "e");
    assert(cell(&nowrap, 4, 0)->glyph == 'e');
    assert(nowrap.cursor_row == 0);
    check_the_screen_is_sound(&nowrap);

    /* A wide character that ends exactly at the margin is an ordinary one. */
    vt_t fits = make(6, 3);
    feed(&fits, "abcd\xE4\xB8\xAD");
    assert(cell(&fits, 4, 0)->glyph == 0x4E2D && cell(&fits, 4, 0)->width == 2);
    assert(fits.cursor_row == 0 && fits.wrap_pending == 1);
    vt_destroy(&fits);
    vt_destroy(&nowrap);
    vt_destroy(&vt);
}

static void test_a_wide_character_written_over_in_half_leaves_no_orphan(void) {
    vt_t vt = make(8, 2);
    feed(&vt, "\xE4\xB8\xAD\xE6\x96\x87");
    feed(&vt, "\rX"); /* Over the first half of the first character. */
    check_the_screen_is_sound(&vt);
    assert(cell(&vt, 0, 0)->glyph == 'X');
    assert(cell(&vt, 1, 0)->glyph == 0 && cell(&vt, 1, 0)->width == 1);
    assert(cell(&vt, 2, 0)->glyph == 0x6587);
    feed(&vt, "\033[4GY"); /* Over the second half of the second. */
    check_the_screen_is_sound(&vt);
    assert(cell(&vt, 2, 0)->glyph == 0 && cell(&vt, 3, 0)->glyph == 'Y');
    /* Erasing, inserting and deleting can cut one in two as well. */
    feed(&vt, "\r\033[2K\xE4\xB8\xAD\xE4\xB8\xAD\033[2G\033[X");
    check_the_screen_is_sound(&vt);
    feed(&vt, "\r\033[2K\xE4\xB8\xAD\xE4\xB8\xAD\033[2G\033[P");
    check_the_screen_is_sound(&vt);
    feed(&vt, "\r\033[2K\xE4\xB8\xAD\xE4\xB8\xAD\033[2G\033[@");
    check_the_screen_is_sound(&vt);
    feed(&vt, "\r\033[2K\xE4\xB8\xAD\xE4\xB8\xAD\033[1;3H\033[1K");
    check_the_screen_is_sound(&vt);
    vt_destroy(&vt);
}

static void test_combining_marks_and_joiners_take_no_cell(void) {
    vt_t vt = make(10, 2);
    /* e and a combining acute, a zero width joiner, a variation selector. */
    feed(&vt,
         "e\xCC\x81z\xE2\x80\x8D"
         "w\xEF\xB8\x8F"
         "q");
    assert(row_is(&vt, 0, "ezwq"));
    assert(vt.cursor_col == 4);
    vt_destroy(&vt);
}

static void test_glyph_widths(void) {
    assert(vt_glyph_width('a') == 1);
    assert(vt_glyph_width(0xE9) == 1);
    assert(vt_glyph_width(0x2588) == 1);  /* Block elements are narrow. */
    assert(vt_glyph_width(0x2500) == 1);  /* And box drawing. */
    assert(vt_glyph_width(0x28FF) == 1);  /* And braille. */
    assert(vt_glyph_width(0x4E2D) == 2);  /* CJK. */
    assert(vt_glyph_width(0x3042) == 2);  /* Hiragana. */
    assert(vt_glyph_width(0xAC00) == 2);  /* Hangul syllables. */
    assert(vt_glyph_width(0xFF21) == 2);  /* Fullwidth A. */
    assert(vt_glyph_width(0xFF71) == 1);  /* Halfwidth katakana. */
    assert(vt_glyph_width(0x3000) == 2);  /* Ideographic space. */
    assert(vt_glyph_width(0x1F600) == 2); /* Emoji. */
    assert(vt_glyph_width(0x1F426) == 2);
    assert(vt_glyph_width(0x2764) == 1); /* A heart, text presentation. */
    assert(vt_glyph_width(0x0301) == 0); /* Combining acute. */
    assert(vt_glyph_width(0x200D) == 0); /* Zero width joiner. */
    assert(vt_glyph_width(0xFE0F) == 0); /* Variation selector 16. */
    assert(vt_glyph_width(0x20000) == 2);
    assert(vt_glyph_width(0x10FFFF) == 1);
}

static void test_the_cursor_moves_and_stays_on_the_screen(void) {
    vt_t vt = make(10, 6);
    feed(&vt, "\033[3;4H");
    assert(vt.cursor_row == 2 && vt.cursor_col == 3);
    feed(&vt, "\033[A");
    assert(vt.cursor_row == 1);
    feed(&vt, "\033[2B");
    assert(vt.cursor_row == 3);
    feed(&vt, "\033[3C");
    assert(vt.cursor_col == 6);
    feed(&vt, "\033[2D");
    assert(vt.cursor_col == 4);
    /* A zero count means one. */
    feed(&vt, "\033[0A\033[0C");
    assert(vt.cursor_row == 2 && vt.cursor_col == 5);
    /* And no count at all, with the semicolon before it empty. */
    feed(&vt, "\033[;2H");
    assert(vt.cursor_row == 0 && vt.cursor_col == 1);
    /* Too far goes to the edge, as neofetch's nine million relies on. */
    feed(&vt, "\033[99A");
    assert(vt.cursor_row == 0);
    feed(&vt, "\033[9999999B");
    assert(vt.cursor_row == 5);
    feed(&vt, "\033[9999999D");
    assert(vt.cursor_col == 0);
    feed(&vt, "\033[9999999C");
    assert(vt.cursor_col == 9);
    feed(&vt, "\033[99999999999999999999999C\033[99999999999999999999999D");
    assert(vt.cursor_col == 0);
    feed(&vt, "\033[99;99H");
    assert(vt.cursor_row == 5 && vt.cursor_col == 9);
    check_the_screen_is_sound(&vt);
    vt_destroy(&vt);
}

static void test_columns_and_rows_are_counted_from_one(void) {
    vt_t vt = make(10, 6);
    feed(&vt, "\033[5G");
    assert(vt.cursor_col == 4);
    feed(&vt, "\033[4d");
    assert(vt.cursor_row == 3);
    feed(&vt, "\033[2;3f");
    assert(vt.cursor_row == 1 && vt.cursor_col == 2);
    feed(&vt, "\033[H");
    assert(vt.cursor_row == 0 && vt.cursor_col == 0);
    feed(&vt, "\033[3E");
    assert(vt.cursor_row == 3 && vt.cursor_col == 0);
    feed(&vt, "abc\033[2F");
    assert(vt.cursor_row == 1 && vt.cursor_col == 0);
    vt_destroy(&vt);
}

static void test_erase_in_line(void) {
    vt_t vt = make(10, 3);
    feed(&vt, "0123456789\033[1;5H\033[K");
    assert(row_is(&vt, 0, "0123"));
    feed(&vt, "\033[2;1H0123456789\033[2;5H\033[1K");
    assert(row_is(&vt, 1, "     56789"));
    feed(&vt, "\033[3;1H0123456789\033[3;5H\033[2K");
    assert(row_is(&vt, 2, ""));
    assert(vt.cursor_col == 4 && vt.cursor_row == 2);
    vt_destroy(&vt);
}

static void test_erase_in_display(void) {
    vt_t vt = make(6, 3);
    feed(&vt, "aaaaaa\nbbbbbb\ncccccc");
    feed(&vt, "\033[2;3H\033[J");
    assert(row_is(&vt, 0, "aaaaaa"));
    assert(row_is(&vt, 1, "bb"));
    assert(row_is(&vt, 2, ""));

    feed(&vt, "\033[1;1Haaaaaa\033[2;1Hbbbbbb\033[3;1Hcccccc\033[2;3H\033[1J");
    assert(row_is(&vt, 0, ""));
    assert(row_is(&vt, 1, "   bbb"));
    assert(row_is(&vt, 2, "cccccc"));

    feed(&vt, "\033[2J");
    assert(row_is(&vt, 0, "") && row_is(&vt, 1, "") && row_is(&vt, 2, ""));
    assert(vt.cursor_row == 1 && vt.cursor_col == 2); /* ED leaves the cursor where it was. */
    feed(&vt, "x\033[3J");
    assert(row_is(&vt, 1, ""));
    vt_destroy(&vt);
}

static void test_erasing_paints_the_background_in_use(void) {
    vt_t vt = make(8, 2);
    /* A status bar: set the background, then erase to the end of the line. */
    feed(&vt, "\033[44mbar\033[K\033[0m");
    for (int col = 0; col < 8; col++) {
        assert(cell(&vt, col, 0)->style.bg.kind == VT_COLOUR_ANSI);
        assert(cell(&vt, col, 0)->style.bg.value[0] == 4);
    }
    assert(cell(&vt, 5, 0)->glyph == 0);
    assert(cell(&vt, 5, 0)->style.fg.kind == VT_COLOUR_DEFAULT);
    /* Back to the default: erasing leaves no colour. */
    feed(&vt, "\033[2;1Hxyz\033[K");
    assert(cell(&vt, 5, 1)->style.bg.kind == VT_COLOUR_DEFAULT);
    vt_destroy(&vt);
}

static void test_sgr_sets_the_sixteen_colours(void) {
    vt_t vt = make(40, 2);
    feed(&vt, "\033[31ma\033[92mb\033[44mc\033[103md\033[39me\033[49mf");
    assert(cell(&vt, 0, 0)->style.fg.kind == VT_COLOUR_ANSI &&
           cell(&vt, 0, 0)->style.fg.value[0] == 1);
    assert(cell(&vt, 1, 0)->style.fg.kind == VT_COLOUR_ANSI &&
           cell(&vt, 1, 0)->style.fg.value[0] == 10);
    assert(cell(&vt, 2, 0)->style.bg.kind == VT_COLOUR_ANSI &&
           cell(&vt, 2, 0)->style.bg.value[0] == 4);
    assert(cell(&vt, 2, 0)->style.fg.value[0] == 10);
    assert(cell(&vt, 3, 0)->style.bg.value[0] == 11);
    assert(cell(&vt, 4, 0)->style.fg.kind == VT_COLOUR_DEFAULT);
    assert(cell(&vt, 4, 0)->style.bg.value[0] == 11);
    assert(cell(&vt, 5, 0)->style.bg.kind == VT_COLOUR_DEFAULT);
    vt_destroy(&vt);
}

static void test_sgr_sets_256_and_24_bit_colours(void) {
    vt_t vt = make(40, 2);
    feed(&vt, "\033[38;5;208ma\033[48;5;17mb\033[38;2;10;20;30mc\033[48;2;255;128;0md");
    assert(cell(&vt, 0, 0)->style.fg.kind == VT_COLOUR_INDEXED &&
           cell(&vt, 0, 0)->style.fg.value[0] == 208);
    assert(cell(&vt, 1, 0)->style.bg.kind == VT_COLOUR_INDEXED &&
           cell(&vt, 1, 0)->style.bg.value[0] == 17);
    assert(cell(&vt, 2, 0)->style.fg.kind == VT_COLOUR_RGB);
    assert(cell(&vt, 2, 0)->style.fg.value[0] == 10 && cell(&vt, 2, 0)->style.fg.value[1] == 20 &&
           cell(&vt, 2, 0)->style.fg.value[2] == 30);
    assert(cell(&vt, 3, 0)->style.bg.kind == VT_COLOUR_RGB);
    assert(cell(&vt, 3, 0)->style.bg.value[0] == 255 && cell(&vt, 3, 0)->style.bg.value[1] == 128);
    /* The first sixteen of the 256 are the same colours as the sixteen. */
    feed(&vt, "\033[0m\033[38;5;9me");
    assert(cell(&vt, 4, 0)->style.fg.kind == VT_COLOUR_ANSI &&
           cell(&vt, 4, 0)->style.fg.value[0] == 9);
    /* Out of range clamps rather than wraps. */
    feed(&vt, "\033[38;2;300;0;0mf\033[38;5;999mg");
    assert(cell(&vt, 5, 0)->style.fg.value[0] == 255);
    assert(cell(&vt, 6, 0)->style.fg.value[0] == 255);
    vt_destroy(&vt);
}

static void test_sgr_colons_are_read_like_semicolons(void) {
    vt_t vt = make(40, 2);
    feed(&vt, "\033[38:5:99ma\033[38:2:1:2:3mb\033[48:2::4:5:6mc\033[38;2;7;8;9;1md");
    assert(cell(&vt, 0, 0)->style.fg.kind == VT_COLOUR_INDEXED &&
           cell(&vt, 0, 0)->style.fg.value[0] == 99);
    assert(cell(&vt, 1, 0)->style.fg.kind == VT_COLOUR_RGB &&
           cell(&vt, 1, 0)->style.fg.value[2] == 3);
    assert(cell(&vt, 2, 0)->style.bg.kind == VT_COLOUR_RGB &&
           cell(&vt, 2, 0)->style.bg.value[0] == 4 && cell(&vt, 2, 0)->style.bg.value[2] == 6);
    /* And what follows a colour is still read: that 1 is bold. */
    assert(cell(&vt, 3, 0)->style.fg.value[0] == 7);
    assert(cell(&vt, 3, 0)->style.attributes & VT_BOLD);
    /* An underline style with a colon, and an underline colour that is skipped. */
    feed(&vt, "\033[0m\033[4:3;58;5;196me");
    assert(cell(&vt, 4, 0)->style.attributes & VT_UNDERLINE);
    assert(cell(&vt, 4, 0)->style.fg.kind == VT_COLOUR_DEFAULT);
    vt_destroy(&vt);
}

static void test_sgr_sets_and_clears_attributes(void) {
    vt_t vt = make(40, 2);
    feed(&vt, "\033[1ma\033[2mb\033[3mc\033[4md\033[7me");
    assert(cell(&vt, 0, 0)->style.attributes == VT_BOLD);
    assert(cell(&vt, 1, 0)->style.attributes == (VT_BOLD | VT_DIM));
    assert(cell(&vt, 4, 0)->style.attributes ==
           (VT_BOLD | VT_DIM | VT_ITALIC | VT_UNDERLINE | VT_REVERSE));
    feed(&vt, "\033[22mf\033[23mg\033[24mh\033[27mi");
    assert(cell(&vt, 5, 0)->style.attributes == (VT_ITALIC | VT_UNDERLINE | VT_REVERSE));
    assert(cell(&vt, 6, 0)->style.attributes == (VT_UNDERLINE | VT_REVERSE));
    assert(cell(&vt, 7, 0)->style.attributes == VT_REVERSE);
    assert(cell(&vt, 8, 0)->style.attributes == 0);
    /* One sequence may do several things, and an empty one resets. */
    feed(&vt, "\033[1;31;4mj\033[mk");
    assert(cell(&vt, 9, 0)->style.attributes == (VT_BOLD | VT_UNDERLINE));
    assert(cell(&vt, 9, 0)->style.fg.value[0] == 1);
    assert(cell(&vt, 10, 0)->style.attributes == 0);
    assert(cell(&vt, 10, 0)->style.fg.kind == VT_COLOUR_DEFAULT);
    feed(&vt, "\033[1;32m\033[0ml\033[;1mm");
    assert(cell(&vt, 11, 0)->style.attributes == 0 &&
           cell(&vt, 11, 0)->style.fg.kind == VT_COLOUR_DEFAULT);
    assert(cell(&vt, 12, 0)->style.attributes == VT_BOLD);
    vt_destroy(&vt);
}

static void test_a_cut_short_extended_colour_is_dropped_safely(void) {
    vt_t vt = make(40, 2);
    feed(&vt, "\033[38;2;1;2mx\033[38;5mx\033[38mx\033[48;2mx\033[38;9;1mx");
    check_the_screen_is_sound(&vt);
    for (int col = 0; col < 5; col++) assert(cell(&vt, col, 0)->style.fg.kind == VT_COLOUR_DEFAULT);
    assert(cell(&vt, 0, 0)->style.bg.kind == VT_COLOUR_DEFAULT);
    /* And whatever was set before is not lost with them. */
    feed(&vt, "\033[1m\033[38;2;1;2m y");
    assert(cell(&vt, 6, 0)->style.attributes & VT_BOLD);
    vt_destroy(&vt);
}

static void test_ls_style_output_keeps_each_name_in_its_colour(void) {
    vt_t vt = make(40, 5);
    /* What `ls --color=always -F` sends: reset, colour, name, reset. */
    feed(&vt,
         "\033[0m\033[01;34mbin\033[0m\n\033[0m\033[01;32mrun\033[0m*\n"
         "\033[0mplain.txt\033[0m\n\033[01;36mlink\033[0m -> \033[0m\033[01;34mbin\033[0m\n");
    assert(row_is(&vt, 0, "bin"));
    assert(cell(&vt, 0, 0)->style.fg.value[0] == 4 &&
           (cell(&vt, 0, 0)->style.attributes & VT_BOLD));
    assert(row_is(&vt, 1, "run*"));
    assert(cell(&vt, 0, 1)->style.fg.value[0] == 2);
    assert(cell(&vt, 3, 1)->style.fg.kind == VT_COLOUR_DEFAULT);
    assert(cell(&vt, 3, 1)->style.attributes == 0);
    assert(row_is(&vt, 2, "plain.txt"));
    assert(cell(&vt, 0, 2)->style.fg.kind == VT_COLOUR_DEFAULT);
    assert(row_is(&vt, 3, "link -> bin"));
    assert(cell(&vt, 0, 3)->style.fg.value[0] == 6);
    assert(cell(&vt, 5, 3)->style.fg.kind == VT_COLOUR_DEFAULT);
    assert(cell(&vt, 8, 3)->style.fg.value[0] == 4);
    vt_destroy(&vt);
}

static void test_a_neofetch_like_logo_gets_its_information_beside_it(void) {
    vt_t vt = make(60, 8);
    /* How neofetch draws: the logo first, line by line; then it goes back up by the
     * logo's height and a long way left and writes each information line after
     * moving right past the logo, going down a line as it goes. */
    feed(&vt,
         "\033[?25l\033[?7l"
         "\033[1;34m  .-.   \033[0m\n"
         "\033[1;34m (o o)  \033[0m\n"
         "\033[1;34m | O \\  \033[0m\n"
         "\033[1;34m  \\   \\ \033[0m\n"
         "\033[1;34m   `~~~'\033[0m\n"
         "\033[5A\033[9999999D"
         "\033[10C\033[1;34muser\033[0m@\033[1;34mhost\033[0m\n"
         "\033[10C-----------\n"
         "\033[10C\033[1;34mOS\033[0m: Linux\n"
         "\033[10C\033[1;34mShell\033[0m: sh\n"
         "\033[10C\033[1;34mCPU\033[0m: some chip\n"
         "\033[?25h\033[?7h");
    assert(row_is(&vt, 0, "  .-.     user@host"));
    assert(row_is(&vt, 1, " (o o)    -----------"));
    assert(row_is(&vt, 2, " | O \\    OS: Linux"));
    assert(row_is(&vt, 3, "  \\   \\   Shell: sh"));
    assert(row_is(&vt, 4, "   `~~~'  CPU: some chip"));
    /* The logo kept its colour and the information has its own. */
    assert(cell(&vt, 3, 0)->style.fg.value[0] == 4);
    assert(cell(&vt, 10, 0)->style.fg.value[0] == 4);
    assert(cell(&vt, 14, 0)->style.fg.kind == VT_COLOUR_DEFAULT && cell(&vt, 14, 0)->glyph == '@');
    assert(cell(&vt, 15, 0)->style.fg.value[0] == 4);
    assert(vt.cursor_row == 5 && vt.cursor_col == 0);
    assert(vt.autowrap == 1);
    check_the_screen_is_sound(&vt);
    vt_destroy(&vt);
}

static void test_a_fastfetch_like_logo_is_followed_by_saved_cursor_moves(void) {
    vt_t vt = make(50, 6);
    /* The shape fastfetch emits: it prints the logo, saves the cursor, then moves
     * to the top, writes the module lines with CUF, and comes back. */
    feed(&vt,
         "\033[34mAAAAAA\033[0m\nBBBBBB\nCCCCCC\n\033[3A"
         "\033[8C\033[1muser\033[0m\n"
         "\033[8C\033[1mos\033[0m: x\n"
         "\033[8C\033[1mcpu\033[0m: y\n");
    assert(row_is(&vt, 0, "AAAAAA  user"));
    assert(row_is(&vt, 1, "BBBBBB  os: x"));
    assert(row_is(&vt, 2, "CCCCCC  cpu: y"));
    assert(cell(&vt, 0, 0)->style.fg.value[0] == 4);
    assert(cell(&vt, 8, 0)->style.attributes & VT_BOLD);
    vt_destroy(&vt);
}

static void test_a_figlet_banner_is_kept_exactly_as_printed(void) {
    vt_t vt = make(40, 8);
    feed(&vt,
         " _          _ _       \n"
         "| |__   ___| | | ___  \n"
         "| '_ \\ / _ \\ | |/ _ \\ \n"
         "| | | |  __/ | | (_) |\n"
         "|_| |_|\\___|_|_|\\___/ \n");
    assert(row_is(&vt, 0, " _          _ _"));
    assert(row_is(&vt, 1, "| |__   ___| | | ___"));
    assert(row_is(&vt, 2, "| '_ \\ / _ \\ | |/ _ \\"));
    assert(row_is(&vt, 3, "| | | |  __/ | | (_) |"));
    assert(row_is(&vt, 4, "|_| |_|\\___|_|_|\\___/"));
    vt_destroy(&vt);
}

static void test_the_cursor_can_be_saved_and_restored_both_ways(void) {
    vt_t vt = make(20, 4);
    feed(&vt, "ab\0337\033[3;5H\033[31mXY\0338Z");
    /* ESC 8 is back at the third column, and in the colours of that moment. */
    assert(cell(&vt, 2, 0)->glyph == 'Z');
    assert(cell(&vt, 2, 0)->style.fg.kind == VT_COLOUR_DEFAULT);
    assert(cell(&vt, 4, 2)->glyph == 'X' && cell(&vt, 4, 2)->style.fg.value[0] == 1);

    feed(&vt, "\033[1;1H\033[1m\033[s\033[4;1Hmmm\033[0m\033[uQ");
    assert(cell(&vt, 0, 0)->glyph == 'Q');
    assert(cell(&vt, 0, 0)->style.attributes & VT_BOLD);
    /* A restore with nothing saved goes home. */
    vt_t fresh = make(20, 4);
    feed(&fresh, "\n\n  \033[u!");
    assert(cell(&fresh, 0, 0)->glyph == '!');
    /* The kitty keyboard protocol's CSI > 1 u and CSI < u are not a restore. */
    vt_t kitty = make(20, 4);
    feed(&kitty, "ab\033[s\033[1;1H\033[>1u\033[<ucd\033[u!");
    assert(cell(&kitty, 0, 0)->glyph == 'c' && cell(&kitty, 1, 0)->glyph == 'd');
    assert(cell(&kitty, 2, 0)->glyph == '!');
    vt_destroy(&kitty);
    vt_destroy(&fresh);
    vt_destroy(&vt);
}

static void test_sequences_we_do_not_use_are_read_whole_and_dropped(void) {
    vt_t vt = make(40, 3);
    feed(&vt,
         "a"
         "\033]0;a window title\007" /* OSC ended by BEL. */
         "b"
         "\033]8;;https://example.org\033\\" /* OSC ended by ST: a hyperlink. */
         "c"
         "\033]8;;\033\\"
         "d"
         "\033P1$r0m\033\\" /* DCS. */
         "e"
         "\033_Gi=1,a=q;AAAA\033\\" /* APC, as kitty's graphics. */
         "f"
         "\033[?25l\033[?1049h\033[?2004h\033[?1000;1006h"
         "g"
         "\033[5n\033[6n\033[c\033[>c\033[=c\033[18t\033[3 q\033[!p\033[1;24r"
         "h"
         "\033(B\033)0\033#8\033=\033>"
         "i"
         "\007\016\017\177"
         "j");
    assert(row_is(&vt, 0, "abcdefghij"));
    check_the_screen_is_sound(&vt);
    vt_destroy(&vt);
}

static void test_a_sequence_that_never_ends_cannot_swallow_what_follows_a_new_escape(void) {
    vt_t vt = make(40, 3);
    /* ESC after the start of a string ends it, and what follows is read afresh. */
    feed(&vt, "a\033]0;title that never ends\033[1;31mb");
    assert(row_is(&vt, 0, "ab"));
    assert(cell(&vt, 1, 0)->style.fg.value[0] == 1);
    /* CAN and SUB cancel a sequence. */
    vt_t cancel = make(40, 3);
    feed(&cancel, "a\033[31\030b\033[32\032c\033]0;t\030d");
    assert(row_is(&cancel, 0, "abcd"));
    assert(cell(&cancel, 1, 0)->style.fg.kind == VT_COLOUR_DEFAULT);
    /* An escape inside a CSI starts over. */
    vt_t again = make(40, 3);
    feed(&again, "\033[3\033[31mx");
    assert(cell(&again, 0, 0)->style.fg.value[0] == 1 && cell(&again, 0, 0)->glyph == 'x');
    /* A control inside a CSI is carried out, and the sequence goes on. */
    vt_t inside = make(40, 3);
    feed(&inside, "a\033[3\n1mb");
    assert(cell(&inside, 0, 1)->glyph == 'b');
    assert(cell(&inside, 0, 1)->style.fg.value[0] == 1);
    vt_destroy(&inside);
    vt_destroy(&again);
    vt_destroy(&cancel);
    vt_destroy(&vt);
}

static void test_a_broken_sequence_is_read_to_its_final_byte_and_ignored(void) {
    vt_t vt = make(40, 3);
    /* A digit after an intermediate, a marker in the middle, too many parameters:
     * none of it is allowed to act, and the text after is plain text. */
    feed(&vt, "a\033[ 1mb\033[1?mc\033[1;2;3;4;5;6;7;8;9;10;11;12;13;14;15;16;17;18;19;20mdef");
    assert(row_is(&vt, 0, "abcdef"));
    assert(cell(&vt, 1, 0)->style.attributes == 0);
    assert(cell(&vt, 2, 0)->style.attributes == 0);
    check_the_screen_is_sound(&vt);
    vt_destroy(&vt);
}

static void test_a_sequence_cut_off_at_the_end_does_nothing(void) {
    vt_t vt = make(40, 3);
    feed(&vt, "ab\033");
    vt_finish(&vt);
    assert(row_is(&vt, 0, "ab"));
    vt_t csi = make(40, 3);
    feed(&csi, "ab\033[31");
    vt_finish(&csi);
    assert(row_is(&csi, 0, "ab"));
    assert(csi.pen.fg.kind == VT_COLOUR_DEFAULT);
    vt_t osc = make(40, 3);
    feed(&osc, "ab\033]0;never closed");
    vt_finish(&osc);
    assert(row_is(&osc, 0, "ab"));
    vt_destroy(&osc);
    vt_destroy(&csi);
    vt_destroy(&vt);
}

static void test_bytes_above_ascii_inside_a_sequence_end_it(void) {
    vt_t vt = make(40, 3);
    feed(&vt, "\033[1\xC3\xA9x");
    /* The sequence is abandoned and the character is read as itself. */
    assert(cell(&vt, 0, 0)->glyph == 0xE9);
    assert(cell(&vt, 1, 0)->glyph == 'x');
    vt_destroy(&vt);
}

static void test_wrapping_can_be_turned_off_and_on(void) {
    vt_t vt = make(5, 3);
    feed(&vt, "\033[?7labcdefgh");
    /* Everything past the last column lands in it, and the last glyph wins. */
    assert(row_is(&vt, 0, "abcdh"));
    assert(vt.cursor_row == 0);
    feed(&vt, "\033[?7h\r\nabcdefg");
    assert(row_is(&vt, 1, "abcde"));
    assert(row_is(&vt, 2, "fg"));
    vt_destroy(&vt);
}

static void test_characters_can_be_inserted_deleted_and_erased(void) {
    vt_t vt = make(10, 3);
    feed(&vt, "0123456789\033[1;4H\033[2@");
    assert(row_is(&vt, 0, "012  34567"));
    feed(&vt, "\033[3P");
    assert(row_is(&vt, 0, "0124567"));
    feed(&vt, "\033[2X");
    assert(row_is(&vt, 0, "012  67"));
    /* A count too large for the line stops at the line. */
    feed(&vt, "\033[99@\033[99P\033[99X");
    assert(row_is(&vt, 0, "012"));
    vt_destroy(&vt);
}

static void test_lines_can_be_inserted_deleted_and_scrolled(void) {
    vt_t vt = make(4, 4);
    feed(&vt, "a\nb\nc\nd\033[2;1H\033[L");
    assert(row_is(&vt, 0, "a") && row_is(&vt, 1, "") && row_is(&vt, 2, "b") && row_is(&vt, 3, "c"));
    feed(&vt, "\033[M");
    assert(row_is(&vt, 1, "b") && row_is(&vt, 2, "c") && row_is(&vt, 3, ""));
    feed(&vt, "\033[S");
    assert(row_is(&vt, 0, "b") && row_is(&vt, 1, "c") && row_is(&vt, 2, ""));
    feed(&vt, "\033[T");
    assert(row_is(&vt, 0, "") && row_is(&vt, 1, "b") && row_is(&vt, 2, "c"));
    /* Reverse index at the top scrolls down; index at the bottom scrolls up. */
    feed(&vt, "\033[1;1H\033M");
    assert(row_is(&vt, 1, "") && row_is(&vt, 2, "b") && row_is(&vt, 3, "c"));
    feed(&vt, "\033[4;1H\033D");
    assert(row_is(&vt, 1, "b") && row_is(&vt, 2, "c") && row_is(&vt, 3, ""));
    /* A scroll that asks for more than the screen holds clears it and is safe. */
    feed(&vt, "\033[999S\033[999T\033[999L\033[999M");
    assert(row_is(&vt, 0, "") && row_is(&vt, 3, ""));
    check_the_screen_is_sound(&vt);
    vt_destroy(&vt);
}

static void test_a_repeat_repeats_the_last_glyph(void) {
    vt_t vt = make(10, 2);
    feed(&vt, "ab\033[3b");
    assert(row_is(&vt, 0, "abbbb"));
    vt_t none = make(10, 2);
    feed(&none, "\033[3bz");
    assert(row_is(&none, 0, "z"));
    vt_destroy(&none);
    vt_destroy(&vt);
}

static void test_a_full_reset_clears_the_screen_and_the_pen(void) {
    vt_t vt = make(10, 3);
    feed(&vt, "\033[31mabc\033[?7l\ndef\033c");
    assert(row_is(&vt, 0, "") && row_is(&vt, 1, ""));
    assert(vt.cursor_col == 0 && vt.cursor_row == 0);
    assert(vt.pen.fg.kind == VT_COLOUR_DEFAULT);
    assert(vt.autowrap == 1);
    vt_destroy(&vt);
}

static void test_the_result_does_not_depend_on_how_the_bytes_are_chunked(void) {
    static const char sample[] =
        "\033[1;34m  .-.   \033[0m\n (o o)  \xE2\x96\x88\xE4\xB8\xAD\xE6\x96\x87\n"
        "\033[2A\033[10Cx\033[38;2;1;2;3mc\033[48:5:7m\xF0\x9F\x90\xA6\033[0m\n"
        "\033]0;t\007\033P1$r\033\\tab\there\r\n\xE2\x82\xAC\033[K\033[1;1H\033[2J"
        "again \033[31;42;1mred\033[m \033[?7l"
        "a very long line that does not fit the narrow screen at all, not even close\n";
    vt_t whole = make(20, 5);
    vt_feed(&whole, sample, sizeof(sample) - 1);
    vt_finish(&whole);
    for (size_t step = 1; step <= 7; step += 2) {
        vt_t pieces = make(20, 5);
        for (size_t at = 0; at < sizeof(sample) - 1; at += step) {
            size_t length = sizeof(sample) - 1 - at < step ? sizeof(sample) - 1 - at : step;
            vt_feed(&pieces, sample + at, length);
        }
        vt_finish(&pieces);
        assert(memcmp(whole.storage, pieces.storage,
                      (size_t)whole.cols * (size_t)whole.rows * sizeof(vt_cell_t)) == 0);
        assert(pieces.cursor_col == whole.cursor_col && pieces.cursor_row == whole.cursor_row);
        vt_destroy(&pieces);
    }
    vt_destroy(&whole);
}

static void test_a_screen_one_column_wide_survives_anything(void) {
    vt_t vt = make(1, 3);
    feed(&vt,
         "abc\xE4\xB8\xAD"
         "d\033[5C\033[3Pe\n\tf");
    check_the_screen_is_sound(&vt);
    vt_t tiny = make(1, 1);
    feed(&tiny, "abc\n\xE4\xB8\xAD\033[2J\033[L\033[M\033[S\033[T\033[@\033[P\033[X");
    check_the_screen_is_sound(&tiny);
    vt_destroy(&tiny);
    vt_destroy(&vt);
}

static void test_a_screen_has_to_have_a_size(void) {
    vt_t vt;
    assert(vt_init(&vt, 0, 10) == -1);
    assert(vt_init(&vt, 10, 0) == -1);
    assert(vt_init(&vt, -1, -1) == -1);
    assert(vt_init(NULL, 10, 10) == -1);
    assert(vt_cell(NULL, 0, 0) == NULL);
    /* And an empty one reads as nothing at all. */
    assert(vt_init(&vt, 3, 2) == 0);
    assert(vt_cell(&vt, 3, 0) == NULL && vt_cell(&vt, 0, 2) == NULL && vt_cell(&vt, -1, 0) == NULL);
    assert(vt_cell_is_blank(cell(&vt, 0, 0)));
    feed(&vt, " x");
    assert(vt_cell_is_blank(cell(&vt, 0, 0)) && !vt_cell_is_blank(cell(&vt, 1, 0)));
    vt_feed(&vt, NULL, 0);
    vt_feed(NULL, "x", 1);
    vt_destroy(&vt);
    vt_destroy(&vt); /* A second destroy of a destroyed screen is harmless. */
}

/* Whatever bytes arrive, the screen stays a sound screen. A few million of them,
 * mostly the ones a parser is most likely to trip on, run under the sanitizers. */
static void test_a_flood_of_hostile_bytes_leaves_the_screen_sound(void) {
    static const char favourites[] =
        "\033[];:?<=>!\"#$%&'()*+,-./0123456789ABCDHJKLMPSTXabcdefghlmnrsu`@\\^_\r\n\t\b\030\032 "
        "\177";
    unsigned long long state = 88172645463325252ull;
    vt_t vt = make(23, 7);
    for (int round = 0; round < 40; round++) {
        char chunk[4096];
        for (size_t i = 0; i < sizeof(chunk); i++) {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            if (state % 3 != 0)
                chunk[i] = favourites[(state >> 8) % (sizeof(favourites) - 1)];
            else
                chunk[i] = (char)(state >> 16);
        }
        vt_feed(&vt, chunk, sizeof(chunk));
        check_the_screen_is_sound(&vt);
    }
    vt_finish(&vt);
    check_the_screen_is_sound(&vt);
    vt_destroy(&vt);
}

int main(void) {
    test_plain_text_lands_where_a_cleared_screen_would_put_it();
    test_a_newline_is_a_new_line_at_column_zero();
    test_a_return_goes_back_and_overwrites();
    test_backspace_steps_back_and_stops_at_the_margin();
    test_a_tab_goes_to_the_next_multiple_of_eight();
    test_a_line_longer_than_the_screen_wraps();
    test_a_line_exactly_as_wide_as_the_screen_wraps_only_when_more_comes();
    test_text_taller_than_the_screen_keeps_the_last_screenful();
    test_utf8_is_one_cell_for_each_code_point();
    test_invalid_utf8_becomes_the_replacement_character();
    test_utf8_cut_short_at_the_end_is_finished_as_a_replacement();
    test_utf8_may_arrive_in_pieces();
    test_wide_characters_take_two_cells();
    test_a_wide_character_at_the_right_margin_wraps_whole();
    test_a_wide_character_written_over_in_half_leaves_no_orphan();
    test_combining_marks_and_joiners_take_no_cell();
    test_glyph_widths();
    test_the_cursor_moves_and_stays_on_the_screen();
    test_columns_and_rows_are_counted_from_one();
    test_erase_in_line();
    test_erase_in_display();
    test_erasing_paints_the_background_in_use();
    test_sgr_sets_the_sixteen_colours();
    test_sgr_sets_256_and_24_bit_colours();
    test_sgr_colons_are_read_like_semicolons();
    test_sgr_sets_and_clears_attributes();
    test_a_cut_short_extended_colour_is_dropped_safely();
    test_ls_style_output_keeps_each_name_in_its_colour();
    test_a_neofetch_like_logo_gets_its_information_beside_it();
    test_a_fastfetch_like_logo_is_followed_by_saved_cursor_moves();
    test_a_figlet_banner_is_kept_exactly_as_printed();
    test_the_cursor_can_be_saved_and_restored_both_ways();
    test_sequences_we_do_not_use_are_read_whole_and_dropped();
    test_a_sequence_that_never_ends_cannot_swallow_what_follows_a_new_escape();
    test_a_broken_sequence_is_read_to_its_final_byte_and_ignored();
    test_a_sequence_cut_off_at_the_end_does_nothing();
    test_bytes_above_ascii_inside_a_sequence_end_it();
    test_wrapping_can_be_turned_off_and_on();
    test_characters_can_be_inserted_deleted_and_erased();
    test_lines_can_be_inserted_deleted_and_scrolled();
    test_a_repeat_repeats_the_last_glyph();
    test_a_full_reset_clears_the_screen_and_the_pen();
    test_the_result_does_not_depend_on_how_the_bytes_are_chunked();
    test_a_screen_one_column_wide_survives_anything();
    test_a_screen_has_to_have_a_size();
    test_a_flood_of_hostile_bytes_leaves_the_screen_sound();
    return 0;
}
