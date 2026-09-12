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

static int same_name(const char *candidate, const char *name, size_t length) {
    return candidate != NULL && strlen(candidate) == length &&
           strncmp(candidate, name, length) == 0;
}

static const option_t *find_long(const option_t *table, size_t count, const char *name,
                                 size_t length) {
    for (size_t i = 0; i < count; i++)
        if (same_name(table[i].name, name, length) || same_name(table[i].alias, name, length))
            return &table[i];
    return NULL;
}

/* Levenshtein, small and iterative, so a typo can be answered with the name the
 * user probably meant instead of a bare refusal. */
static size_t edit_distance(const char *a, const char *b) {
    size_t la = strlen(a), lb = strlen(b);
    size_t previous[64], current[64];
    if (lb + 1 > 64) return 64;
    for (size_t j = 0; j <= lb; j++) previous[j] = j;
    for (size_t i = 1; i <= la; i++) {
        current[0] = i;
        for (size_t j = 1; j <= lb; j++) {
            size_t cost = a[i - 1] == b[j - 1] ? 0 : 1;
            size_t best = previous[j] + 1;
            if (current[j - 1] + 1 < best) best = current[j - 1] + 1;
            if (previous[j - 1] + cost < best) best = previous[j - 1] + cost;
            current[j] = best;
        }
        memcpy(previous, current, (lb + 1) * sizeof(*previous));
    }
    return previous[lb];
}

static const char *nearest_name(const option_t *table, size_t count, const char *name,
                                size_t length) {
    char wanted[64];
    const char *best = NULL;
    size_t best_distance = 3; /* Further than two edits away is a different word. */
    if (length + 1 > sizeof(wanted)) return NULL;
    memcpy(wanted, name, length);
    wanted[length] = '\0';
    for (size_t i = 0; i < count; i++) {
        size_t distance = edit_distance(wanted, table[i].name);
        if (distance < best_distance) {
            best_distance = distance;
            best = table[i].name;
        }
    }
    return best;
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
        if (strcmp(argument, "-h") == 0) return OPTIONS_HELP;
        if (strcmp(argument, "--help") == 0) return OPTIONS_HELP_FULL;
        if (strcmp(argument, "--completion") == 0) {
            if (i + 1 >= argc) {
                fail(error, error_size, "--completion wants bash, zsh or fish");
                return OPTIONS_ERROR;
            }
            snprintf(error, error_size, "%s", argv[i + 1]);
            return OPTIONS_COMPLETION;
        }
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
                const char *meant = nearest_name(table, count, name, length);
                if (meant != NULL)
                    fail(error, error_size, "unknown option '--%.*s', did you mean '--%s'?",
                         (int)length, name, meant);
                else
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

void options_usage(FILE *out, const char *program, const char *tagline,
                   const option_example_t *examples, const option_t *table, size_t count,
                   int everything) {
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
        if (option != NULL && !everything && !option->essential) continue;
        const char *next = option ? option->group : "General";
        if (group == NULL || strcmp(group, next) != 0) {
            fprintf(out, "\n%s\n", next);
            group = next;
        }
        if (option == NULL) break;

        render_option(out, column, option->shorthand, option->name,
                      option->kind == OPTION_FLAG ? NULL : option->metavar, option->help);
    }
    if (everything) {
        render_option(out, column, 'h', "help", NULL, "the one screen help");
        render_option(out, column, 0, "completion", "SHELL", "completions for bash, zsh or fish");
        render_option(out, column, 'V', "version", NULL, "show the version and exit");
    } else {
        render_option(out, column, 0, "help", NULL, "every option, grouped");
        render_option(out, column, 'V', "version", NULL, "show the version and exit");
    }

    if (examples != NULL && everything) {
        size_t widest = 0;
        for (size_t i = 0; examples[i].command != NULL; i++) {
            size_t length = strlen(examples[i].command);
            if (length > widest) widest = length;
        }
        fprintf(out, "\nExamples\n");
        for (size_t i = 0; examples[i].command != NULL; i++)
            fprintf(out, "  %-*s  %s\n", (int)widest, examples[i].command, examples[i].what);
    }
}

int options_completion(FILE *out, const char *shell, const char *program, const option_t *table,
                       size_t count) {
    if (shell == NULL) return 0;

    if (strcmp(shell, "bash") == 0) {
        fprintf(out, "# %s completions for bash\ncomplete -W \"", program);
        for (size_t i = 0; i < count; i++) fprintf(out, "--%s ", table[i].name);
        fprintf(out, "--help --version --completion\" %s\n", program);
        return 1;
    }
    if (strcmp(shell, "zsh") == 0) {
        fprintf(out, "#compdef %s\n_arguments \\\n", program);
        for (size_t i = 0; i < count; i++)
            fprintf(out, "  '--%s[%s]%s' \\\n", table[i].name, table[i].help,
                    table[i].kind == OPTION_FLAG ? "" : ":value:");
        fprintf(out, "  '--help[every option, grouped]' \\\n  '--version[show the version]'\n");
        return 1;
    }
    if (strcmp(shell, "fish") == 0) {
        for (size_t i = 0; i < count; i++) {
            fprintf(out, "complete -c %s -l %s", program, table[i].name);
            if (table[i].shorthand) fprintf(out, " -s %c", table[i].shorthand);
            if (table[i].kind != OPTION_FLAG) fprintf(out, " -r");
            if (table[i].kind == OPTION_ENUM && table[i].names != NULL) {
                fprintf(out, " -a \"");
                for (int k = 0; table[i].names[k] != NULL; k++)
                    fprintf(out, "%s%s", k ? " " : "", table[i].names[k]);
                fprintf(out, "\"");
            }
            fprintf(out, " -d \"%s\"\n", table[i].help);
        }
        fprintf(out, "complete -c %s -l help -d \"every option, grouped\"\n", program);
        return 1;
    }
    return 0;
}

const char *options_status_string(options_status_t status) {
    switch (status) {
        case OPTIONS_OK:
            return "ok";
        case OPTIONS_HELP:
        case OPTIONS_HELP_FULL:
            return "help requested";
        case OPTIONS_COMPLETION:
            return "completions requested";
        case OPTIONS_VERSION:
            return "version requested";
        case OPTIONS_ERROR:
            return "invalid arguments";
    }
    return "unknown error";
}
