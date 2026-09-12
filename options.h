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
    char shorthand;    /* 0 when the option is long only. */
    const char *name;  /* Long name without the dashes, never NULL. */
    const char *alias; /* An older name, accepted but never advertised. */
    option_kind_t kind;
    void *target;
    double minimum, maximum;  /* INT and DOUBLE only. */
    const char *const *names; /* ENUM only, NULL terminated. */
    const char *metavar;      /* "COUNT", "FPS", shown in --help. */
    const char *help;         /* One line, lower case, no trailing stop. */
    const char *group;        /* Section heading in --help. */
    int essential;            /* Shown by -h as well as by --help. */
} option_t;

typedef enum {
    OPTIONS_OK = 0,
    OPTIONS_HELP,       /* -h: the one screen version. */
    OPTIONS_HELP_FULL,  /* --help: everything, grouped. */
    OPTIONS_VERSION,    /* --version. */
    OPTIONS_COMPLETION, /* --completion SHELL, the shell left in the buffer. */
    OPTIONS_ERROR       /* Message written into the caller's buffer. */
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

/* One worked line of the help: what to type, and what it gets you. Kept as two
 * strings rather than one pre-padded line so the columns are aligned by the same
 * arithmetic that aligns the options, and stay aligned when one is edited. */
typedef struct {
    const char *command;
    const char *what;
} option_example_t;

/*
 * Groups in table order, columns aligned to the widest option. With everything
 * false only the rows marked essential are shown, which is what -h is for: one
 * screen a newcomer can read, against the full list for someone looking for a
 * particular switch.
 */
void options_usage(FILE *out, const char *program, const char *tagline,
                   const option_example_t *examples, const option_t *table, size_t count,
                   int everything);

/* Completions for bash, zsh or fish, off the same table. */
int options_completion(FILE *out, const char *shell, const char *program, const option_t *table,
                       size_t count);

const char *options_status_string(options_status_t status);

#endif
