/*
 * Table driven command line options.
 *
 * A tool with forty switches cannot keep them in a hand written loop and stay
 * readable, and its --help cannot stay aligned by hand. Declare the options
 * once, as data, and both the parser and the help text come off the same table.
 */

#ifndef OPTIONS_H
#define OPTIONS_H

#include <stddef.h>
#include <stdio.h>

typedef enum {
    OPTION_FLAG,   /* No argument, sets its int target to 1. */
    OPTION_INT,    /* Bounded integer. */
    OPTION_DOUBLE, /* Bounded double. */
    OPTION_ENUM,   /* One of names[], the index goes into the int target. */
    OPTION_STRING  /* Kept as a pointer into argv. */
} option_kind_t;

typedef struct {
    char shorthand;   /* 0 when the option is long only. */
    const char *name; /* Long name without the dashes, never NULL. */
    option_kind_t kind;
    void *target;
    double minimum, maximum;  /* INT and DOUBLE only. */
    const char *const *names; /* ENUM only, NULL terminated. */
    const char *metavar;      /* "COUNT", "FPS", shown in --help. */
    const char *help;         /* One line, lower case, no trailing stop. */
    const char *group;        /* Section heading in --help. */
} option_t;

typedef enum {
    OPTIONS_OK = 0,
    OPTIONS_HELP,    /* --help was asked for, print usage and exit zero. */
    OPTIONS_VERSION, /* --version, likewise. */
    OPTIONS_ERROR    /* Message written into the caller's buffer. */
} options_status_t;

/*
 * Accepts, for an option named "birds" with shorthand 'n':
 *     -n 800   -n800   --birds 800   --birds=800
 * Flags cluster, so -qv is -q -v, and --no-NAME clears a flag. A bare "--"
 * ends option parsing. Anything left over is an error: this program takes no
 * positional arguments.
 */
options_status_t options_parse(const option_t *table, size_t count, int argc, char **argv,
                               char *error, size_t error_size);

/* Groups in table order, columns aligned to the widest option. */
void options_usage(FILE *out, const char *program, const char *tagline, const char *const *examples,
                   const option_t *table, size_t count);

const char *options_status_string(options_status_t status);

#endif
