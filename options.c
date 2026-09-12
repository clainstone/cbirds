#include "options.h"

#include <errno.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

static void fail(char *error, size_t size, const char *format, ...) {
    va_list arguments;
    va_start(arguments, format);
    vsnprintf(error, size, format, arguments);
    va_end(arguments);
}

static const option_t *find_long(const option_t *table, size_t count, const char *name,
                                 size_t length) {
    for (size_t i = 0; i < count; i++)
        if (strlen(table[i].name) == length && strncmp(table[i].name, name, length) == 0)
            return &table[i];
    return NULL;
}

static const option_t *find_short(const option_t *table, size_t count, char shorthand) {
    for (size_t i = 0; i < count; i++)
        if (table[i].shorthand && table[i].shorthand == shorthand) return &table[i];
    return NULL;
}

/* Long names read better in messages, so errors quote those even for -n. */
static options_status_t assign(const option_t *option, const char *text, char *error,
                               size_t error_size) {
    if (option->kind == OPTION_FLAG) {
        *(int *)option->target = 1;
        return OPTIONS_OK;
    }
    if (option->kind == OPTION_STRING) {
        *(const char **)option->target = text;
        return OPTIONS_OK;
    }
    if (option->kind == OPTION_ENUM) {
        for (int i = 0; option->names[i] != NULL; i++)
            if (strcmp(option->names[i], text) == 0) {
                *(int *)option->target = i;
                return OPTIONS_OK;
            }
        size_t at = (size_t)snprintf(error, error_size, "--%s must be one of", option->name);
        for (int i = 0; option->names[i] != NULL && at < error_size; i++)
            at += (size_t)snprintf(error + at, error_size - at, "%s %s", i ? "," : "",
                                   option->names[i]);
        return OPTIONS_ERROR;
    }

    char *end;
    errno = 0;
    double value = strtod(text, &end);
    if (errno == ERANGE || end == text || *end != '\0') {
        fail(error, error_size, "--%s wants a number, not '%s'", option->name, text);
        return OPTIONS_ERROR;
    }
    if (value < option->minimum || value > option->maximum) {
        if (option->kind == OPTION_INT)
            fail(error, error_size, "--%s must be between %d and %d", option->name,
                 (int)option->minimum, (int)option->maximum);
        else
            fail(error, error_size, "--%s must be between %g and %g", option->name, option->minimum,
                 option->maximum);
        return OPTIONS_ERROR;
    }
    if (option->kind == OPTION_INT) {
        if (value != (double)(int)value) {
            fail(error, error_size, "--%s wants a whole number, not '%s'", option->name, text);
            return OPTIONS_ERROR;
        }
        *(int *)option->target = (int)value;
    } else {
        *(double *)option->target = value;
    }
    return OPTIONS_OK;
}

options_status_t options_parse(const option_t *table, size_t count, int argc, char **argv,
                               char *error, size_t error_size) {
    if (table == NULL || argv == NULL || error == NULL || error_size == 0) return OPTIONS_ERROR;
    error[0] = '\0';

    for (int i = 1; i < argc; i++) {
        const char *argument = argv[i];

        if (strcmp(argument, "--") == 0) {
            if (i + 1 < argc) {
                fail(error, error_size, "unexpected argument '%s'", argv[i + 1]);
                return OPTIONS_ERROR;
            }
            return OPTIONS_OK;
        }
        if (strcmp(argument, "-h") == 0 || strcmp(argument, "--help") == 0) return OPTIONS_HELP;
        if (strcmp(argument, "-V") == 0 || strcmp(argument, "--version") == 0)
            return OPTIONS_VERSION;

        if (argument[0] != '-' || argument[1] == '\0') {
            fail(error, error_size, "unexpected argument '%s'", argument);
            return OPTIONS_ERROR;
        }

        if (argument[1] == '-') { /* Long form. */
            const char *name = argument + 2;
            const char *equals = strchr(name, '=');
            size_t length = equals ? (size_t)(equals - name) : strlen(name);

            /* --no-NAME clears a flag, which is how the off switches read. */
            if (length > 3 && strncmp(name, "no-", 3) == 0) {
                const option_t *option = find_long(table, count, name + 3, length - 3);
                if (option != NULL && option->kind == OPTION_FLAG) {
                    if (equals) {
                        fail(error, error_size, "--%.*s takes no value", (int)length, name);
                        return OPTIONS_ERROR;
                    }
                    *(int *)option->target = 0;
                    continue;
                }
            }

            const option_t *option = find_long(table, count, name, length);
            if (option == NULL) {
                fail(error, error_size, "unknown option '--%.*s'", (int)length, name);
                return OPTIONS_ERROR;
            }
            if (option->kind == OPTION_FLAG) {
                if (equals) {
                    fail(error, error_size, "--%s takes no value", option->name);
                    return OPTIONS_ERROR;
                }
                *(int *)option->target = 1;
                continue;
            }
            const char *text = equals ? equals + 1 : (++i < argc ? argv[i] : NULL);
            if (text == NULL) {
                fail(error, error_size, "--%s wants a value", option->name);
                return OPTIONS_ERROR;
            }
            options_status_t status = assign(option, text, error, error_size);
            if (status != OPTIONS_OK) return status;
            continue;
        }

        /* Short form, and flags cluster: -qv, -qn800, -qn 800. */
        for (const char *c = argument + 1; *c != '\0'; c++) {
            const option_t *option = find_short(table, count, *c);
            if (option == NULL) {
                fail(error, error_size, "unknown option '-%c'", *c);
                return OPTIONS_ERROR;
            }
            if (option->kind == OPTION_FLAG) {
                *(int *)option->target = 1;
                continue;
            }
            const char *text = c[1] != '\0' ? c + 1 : (++i < argc ? argv[i] : NULL);
            if (text == NULL) {
                fail(error, error_size, "-%c wants a value", *c);
                return OPTIONS_ERROR;
            }
            options_status_t status = assign(option, text, error, error_size);
            if (status != OPTIONS_OK) return status;
            break; /* The rest of the cluster was the value. */
        }
    }
    return OPTIONS_OK;
}

/* Exactly as wide as render_option writes it, so the two cannot disagree: two
 * of indent, four for the shorthand slot whether or not there is one, the two
 * dashes, the name, and the metavar with its space. */
static size_t option_width(const option_t *option) {
    size_t width = 2 + 4 + 2 + strlen(option->name);
    if (option->kind != OPTION_FLAG && option->metavar != NULL)
        width += 1 + strlen(option->metavar);
    return width;
}

static void render_option(FILE *out, size_t column, char shorthand, const char *name,
                          const char *metavar, const char *help) {
    char left[96];
    int at = snprintf(left, sizeof(left), "  ");
    if (shorthand)
        at += snprintf(left + at, sizeof(left) - (size_t)at, "-%c, ", shorthand);
    else
        at += snprintf(left + at, sizeof(left) - (size_t)at, "    ");
    at += snprintf(left + at, sizeof(left) - (size_t)at, "--%s", name);
    if (metavar != NULL) snprintf(left + at, sizeof(left) - (size_t)at, " %s", metavar);
    fprintf(out, "%-*s%s\n", (int)column, left, help);
}

void options_usage(FILE *out, const char *program, const char *tagline, const char *const *examples,
                   const option_t *table, size_t count) {
    size_t column = strlen("  -V, --version");
    for (size_t i = 0; i < count; i++) {
        size_t width = option_width(&table[i]);
        if (width > column) column = width;
    }
    column += 2;

    if (tagline != NULL) fprintf(out, "%s\n\n", tagline);
    fprintf(out, "Usage: %s [OPTIONS]\n", program);

    const char *group = NULL;
    for (size_t i = 0; i <= count; i++) {
        const option_t *option = i < count ? &table[i] : NULL;
        const char *next = option ? option->group : "General";
        if (group == NULL || strcmp(group, next) != 0) {
            fprintf(out, "\n%s\n", next);
            group = next;
        }
        if (option == NULL) break;

        render_option(out, column, option->shorthand, option->name,
                      option->kind == OPTION_FLAG ? NULL : option->metavar, option->help);
    }
    render_option(out, column, 'h', "help", NULL, "show this help and exit");
    render_option(out, column, 'V', "version", NULL, "show the version and exit");

    if (examples != NULL) {
        fprintf(out, "\nExamples\n");
        for (size_t i = 0; examples[i] != NULL; i++) fprintf(out, "  %s\n", examples[i]);
    }
}

const char *options_status_string(options_status_t status) {
    switch (status) {
        case OPTIONS_OK:
            return "ok";
        case OPTIONS_HELP:
            return "help requested";
        case OPTIONS_VERSION:
            return "version requested";
        case OPTIONS_ERROR:
            return "invalid arguments";
    }
    return "unknown error";
}
