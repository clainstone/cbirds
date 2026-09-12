#include "../options.h"

#include <assert.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

static int birds, quiet, mono, palette;
static double weight;
static const char *label;

static const char *const PALETTES[] = {"mono", "flame", "ice", NULL};

static const option_t TABLE[] = {
    {'n', "birds", OPTION_INT, &birds, 1, 4096, NULL, "COUNT", "how many boids", "Flock"},
    {'w', "weight", OPTION_DOUBLE, &weight, 0.0, 1.0, NULL, "VALUE", "a weight", "Flock"},
    {'q', "quiet", OPTION_FLAG, &quiet, 0, 0, NULL, NULL, "say less", "Output"},
    {'m', "mono", OPTION_FLAG, &mono, 0, 0, NULL, NULL, "one colour", "Output"},
    {'P', "palette", OPTION_ENUM, &palette, 0, 0, PALETTES, "NAME", "colour scheme", "Output"},
    {0, "label", OPTION_STRING, &label, 0, 0, NULL, "TEXT", "a caption", "Output"},
};
enum { COUNT = sizeof(TABLE) / sizeof(*TABLE) };

static void reset(void) {
    birds = 800;
    weight = 0.5;
    quiet = mono = palette = 0;
    label = NULL;
}

static options_status_t parse(char *error, size_t size, ...) {
    char *argv[16] = {(char *)"cbirds"};
    int argc = 1;
    va_list arguments;
    va_start(arguments, size);
    for (char *a = va_arg(arguments, char *); a != NULL; a = va_arg(arguments, char *))
        argv[argc++] = a;
    va_end(arguments);
    return options_parse(TABLE, COUNT, argc, argv, error, size);
}

static void test_forms(void) {
    char error[128];

    /* The four ways of giving a value all mean the same thing. */
    reset();
    assert(parse(error, sizeof(error), "-n", "1500", NULL) == OPTIONS_OK && birds == 1500);
    reset();
    assert(parse(error, sizeof(error), "-n1500", NULL) == OPTIONS_OK && birds == 1500);
    reset();
    assert(parse(error, sizeof(error), "--birds", "1500", NULL) == OPTIONS_OK && birds == 1500);
    reset();
    assert(parse(error, sizeof(error), "--birds=1500", NULL) == OPTIONS_OK && birds == 1500);

    reset();
    assert(parse(error, sizeof(error), "--weight=0.25", NULL) == OPTIONS_OK && weight == 0.25);
    reset();
    assert(parse(error, sizeof(error), "--label", "hello world", NULL) == OPTIONS_OK);
    assert(strcmp(label, "hello world") == 0);
}

static void test_flags_cluster(void) {
    char error[128];

    reset();
    assert(parse(error, sizeof(error), "-qm", NULL) == OPTIONS_OK && quiet && mono);
    /* A cluster may end in a value taking option. */
    reset();
    assert(parse(error, sizeof(error), "-qn200", NULL) == OPTIONS_OK && quiet && birds == 200);
    reset();
    assert(parse(error, sizeof(error), "-qn", "200", NULL) == OPTIONS_OK && quiet && birds == 200);

    /* --no-NAME turns a flag back off, whatever set it. */
    reset();
    assert(parse(error, sizeof(error), "--quiet", "--no-quiet", NULL) == OPTIONS_OK && !quiet);
    reset();
    quiet = 1;
    assert(parse(error, sizeof(error), "--no-quiet", NULL) == OPTIONS_OK && !quiet);
}

static void test_enumerations(void) {
    char error[128];

    reset();
    assert(parse(error, sizeof(error), "--palette", "ice", NULL) == OPTIONS_OK && palette == 2);
    reset();
    assert(parse(error, sizeof(error), "-Pflame", NULL) == OPTIONS_OK && palette == 1);

    reset();
    assert(parse(error, sizeof(error), "--palette", "chartreuse", NULL) == OPTIONS_ERROR);
    assert(strstr(error, "mono") && strstr(error, "flame") && strstr(error, "ice"));
}

static void test_refusals(void) {
    char error[128];

    reset();
    assert(parse(error, sizeof(error), "--birds", "0", NULL) == OPTIONS_ERROR);
    assert(strstr(error, "between 1 and 4096") != NULL);
    reset();
    assert(parse(error, sizeof(error), "--birds", "5000", NULL) == OPTIONS_ERROR);
    reset();
    assert(parse(error, sizeof(error), "--birds", "12.5", NULL) == OPTIONS_ERROR);
    assert(strstr(error, "whole number") != NULL);
    reset();
    assert(parse(error, sizeof(error), "--birds", "many", NULL) == OPTIONS_ERROR);
    assert(strstr(error, "wants a number") != NULL);
    reset();
    assert(parse(error, sizeof(error), "--birds", NULL) == OPTIONS_ERROR);
    assert(strstr(error, "wants a value") != NULL);
    reset();
    assert(parse(error, sizeof(error), "--quiet=1", NULL) == OPTIONS_ERROR);
    assert(strstr(error, "takes no value") != NULL);
    reset();
    assert(parse(error, sizeof(error), "--birdz", "10", NULL) == OPTIONS_ERROR);
    assert(strstr(error, "unknown option '--birdz'") != NULL);
    reset();
    assert(parse(error, sizeof(error), "-z", NULL) == OPTIONS_ERROR);
    assert(strstr(error, "unknown option '-z'") != NULL);

    /* This program takes no positional arguments, and says so. */
    reset();
    assert(parse(error, sizeof(error), "flock.png", NULL) == OPTIONS_ERROR);
    assert(strstr(error, "unexpected argument 'flock.png'") != NULL);
    reset();
    assert(parse(error, sizeof(error), "--", "flock.png", NULL) == OPTIONS_ERROR);
    reset();
    assert(parse(error, sizeof(error), "--", NULL) == OPTIONS_OK);
}

static void test_help_and_version(void) {
    char error[128];
    reset();
    assert(parse(error, sizeof(error), "-h", NULL) == OPTIONS_HELP);
    assert(parse(error, sizeof(error), "--help", NULL) == OPTIONS_HELP);
    assert(parse(error, sizeof(error), "-V", NULL) == OPTIONS_VERSION);
    assert(parse(error, sizeof(error), "--version", NULL) == OPTIONS_VERSION);
    /* Asked for anywhere, it wins over whatever else is on the line. */
    assert(parse(error, sizeof(error), "-n", "10", "--help", NULL) == OPTIONS_HELP);
}

static void test_usage_is_aligned(void) {
    static const char *const EXAMPLES[] = {"cbirds -n 1500", NULL};
    char buffer[4096] = {0};
    FILE *out = fmemopen(buffer, sizeof(buffer), "w");
    assert(out != NULL);
    options_usage(out, "cbirds", "A flock in your terminal.", EXAMPLES, TABLE, COUNT);
    fclose(out);

    assert(strstr(buffer, "A flock in your terminal.") == buffer);
    assert(strstr(buffer, "Usage: cbirds [OPTIONS]") != NULL);
    /* Groups appear once each, in table order. */
    const char *flock = strstr(buffer, "\nFlock\n");
    const char *output = strstr(buffer, "\nOutput\n");
    const char *general = strstr(buffer, "\nGeneral\n");
    assert(flock && output && general && flock < output && output < general);
    assert(strstr(buffer, "-n, --birds COUNT") != NULL);
    assert(strstr(buffer, "    --label TEXT") != NULL); /* No shorthand, still aligned. */
    assert(strstr(buffer, "-h, --help") != NULL);
    assert(strstr(buffer, "-V, --version") != NULL);
    assert(strstr(buffer, "Examples") != NULL);
    assert(strstr(buffer, "cbirds -n 1500") != NULL);

    /* Every help text starts at the same column. */
    size_t at = 0, column = 0;
    for (char *line = strtok(buffer, "\n"); line != NULL; line = strtok(NULL, "\n"), at++) {
        if (line[0] != ' ' || line[2] != '-') continue;
        const char *help = strstr(line, "  ");
        while (help && help[2] == ' ') help += 1;
        size_t here = (size_t)(help + 2 - line);
        if (column == 0) column = here;
        assert(here == column);
    }
    assert(column > 0);
}

int main(void) {
    test_forms();
    test_flags_cluster();
    test_enumerations();
    test_refusals();
    test_help_and_version();
    test_usage_is_aligned();
    return 0;
}
