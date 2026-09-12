/* Feature test macros must precede every include. */
#define _XOPEN_SOURCE 700
#define _DEFAULT_SOURCE
#define _DARWIN_C_SOURCE

#include <errno.h>
#include <math.h>
#include <poll.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include "font.h"
#include "kitty_graphics.h"
#include "options.h"
#include "png.h"
#include "spatial_grid.h"
#include "sprite_png.h"

enum {
    ROTATION_FRAMES = 90,
    MAX_PALETTE_SHADES = 8,
    MAX_FLOCKS = 5,
    COLOUR_BY_FIXED = 0,
    COLOUR_BY_HEADING,
    COLOUR_BY_DENSITY,
    COLOUR_BY_FLOCK,
    MOUSE_FLEE = 0,
    MOUSE_FOLLOW,
    MOUSE_CAT,
    MOUSE_OFF,
    DEFAULT_MOUSE_REACH = 120,
    DEFAULT_VISION_NOTCH = 6,
    /* Only every sixteenth bird leaves one, because a tail behind all of them is
     * three times the bandwidth for a picture that reads as mud. Fifty comets in
     * a flock of eight hundred is what says "moving" in a still frame. */
    TRAIL_EVERY = 16,
    TRAIL_LENGTH = 4,
    INTRO_SECONDS = 3,
    MAX_HAWKS = 8,
    HAWK_REACH = 180,
    AUTOPILOT_PERIOD = 4, /* Seconds between one slider moving and the next. */
    AUTOPILOT_YIELD = 3,  /* Seconds it keeps its hands off after a keypress. */
    OUTRO_FRAMES = 40,
    FRAME_ANGLE = 360 / ROTATION_FRAMES,
    SPRITE_SUPERSAMPLE = 8,
    SPRITE_WORK_MAX = 256,
    MIN_BIRD_SIZE = 4,
    MAX_BIRD_SIZE = 64,
    MAX_BIRDS = 4096,
    INPUT_BUFFER_SIZE = 100,
    DEFAULT_COLS = 80,
    DEFAULT_ROWS = 24,
    DEFAULT_CELL_WIDTH = 8,
    DEFAULT_CELL_HEIGHT = 16,
    /* Edge bands the flock turns away from, as a fraction of the viewport: a
     * third on the sides and the top, half of that at the bottom. */
    TURN_BAND_DIVISOR = 3,
    BOTTOM_BAND_DIVISOR = 6,
    MIN_FRAME_RATE = 30,
    DEFAULT_FRAME_RATE = 60,
    MAX_FRAME_RATE = 120,
    FRAME_RATE_STEP = 5,
    DEFAULT_SPEED = 40,
    DEFAULT_BIRD_SIZE = 15,
    SPATIAL_CELL_SIZE = 12,
    /* Perception is tuned as a radius in pixels rather than in whole grid cells,
     * which is what lets it share the twelve notch travel: the cell scan derives
     * from it, and the distance test was always the exact radius anyway. Sixty
     * pixels is the five cell ceiling it had before. */
    MIN_VISION_RADIUS = 12,
    MAX_VISION_RADIUS = 60,
    DEFAULT_VISION_RADIUS = 36,
    MAX_VISION_CELLS = MAX_VISION_RADIUS / SPATIAL_CELL_SIZE,
    /* The parameter panel, anchored to the top left corner. Its size in cells is
     * fixed: it follows the longest parameter name and the bar, never the
     * terminal. Below the minimum viewport it is dropped and the flock keeps
     * everything; the minimums leave a corridor to the right of the panel and
     * one underneath it. */
    LEGEND_COLUMNS = 38,
    LEGEND_ROWS = 10,
    /* One notch a keypress, so this is also the number of steps every parameter
     * travels through, from its floor to its ceiling. */
    LEGEND_BAR_CELLS = 12,
    LEGEND_NAME_WIDTH = 10,
    LEGEND_VALUE_WIDTH = 5,
    LEGEND_MIN_COLS = 50,
    LEGEND_MIN_ROWS = 14,
    LEGEND_LINE_MAX = 128,
    SPAWN_ATTEMPTS = 32
};

/* The same scale as the original bottom edge turn: large enough that it settles
 * the direction on its own, whatever the flocking terms are doing. */
static const double LEGEND_PUSH = 100000.0;
/* Heavy enough to bend a flock that is busy flocking, light enough that it bends
 * rather than shatters. */
static const double MOUSE_WEIGHT = 4.0;
static const double HAWK_WEIGHT = 6.0;
static const double HAWK_SPEED = 1.15;
static const double CAT_PERIOD = 6.0; /* Seconds of one stalk and pounce. */
static const double CAT_STALK = 4.5;  /* Of which this much is holding still. */
static const double CAT_POUNCE = 3.0; /* And then this much harder. */

/* Needed in the config initializer, so macros rather than constants. */
#define DEFAULT_SEPARATION_W 0.005
#define DEFAULT_ALIGNMENT_W 1.5
#define DEFAULT_COHESION_W 0.01
#define DEFAULT_BOUNDARY_W 0.2

static const double BOUNDARY_MIN = 0.01;
static const double SEPARATION_MIN = 0.001;
static const double COHESION_MIN = 0.002;
static const double ALIGNMENT_MIN = 0.1;

/* Each ceiling is placed so that the default lands exactly on the fourth of
 * twelve notches, a third along the bar. There are no step constants any more:
 * a step is one notch, which is a twelfth of the travel by construction, and
 * that is what makes a keypress worth exactly one cell of bar. */
#define DEFAULT_NOTCH 4
#define NOTCH_CEILING(minimum, default_value) ((minimum) + 3 * ((default_value) - (minimum)))
static const double BOUNDARY_MAX = NOTCH_CEILING(0.01, DEFAULT_BOUNDARY_W);
static const double SEPARATION_MAX = NOTCH_CEILING(0.001, DEFAULT_SEPARATION_W);
static const double COHESION_MAX = NOTCH_CEILING(0.002, DEFAULT_COHESION_W);
static const double ALIGNMENT_MAX = NOTCH_CEILING(0.1, DEFAULT_ALIGNMENT_W);

#define ALT_SCREEN_ON "\033[?1049h"
#define ALT_SCREEN_OFF "\033[?1049l"
#define CURSOR_HIDE "\033[?25l"
#define CURSOR_SHOW "\033[?25h"
#define SYNC_UPDATE_END "\033[?2026l"
/* Any event tracking plus SGR coordinates: 1003 reports plain motion as well as
 * clicks, and 1006 lifts the 223 column ceiling of the original encoding. */
#define MOUSE_REPORT_ON "\033[?1003h\033[?1006h"
#define MOUSE_REPORT_OFF "\033[?1006l\033[?1003l"

typedef struct {
    double x, y;
} vector_t;

typedef struct {
    double x, y, direction;
    int frame;
    int shade; /* Index into the palette, and half of the image id. */
    int flock; /* Which flock it reads: separation ignores this, the rest does not. */
    double trail_x[TRAIL_LENGTH], trail_y[TRAIL_LENGTH];
    int trail_at, trail_held;
} bird_t;

typedef struct {
    uint8_t *data;
    size_t length;
} image_frame_t;

typedef struct {
    int width, height, cols, rows;
    int cell_width, cell_height, turn_x, turn_y, turn_bottom;
    int legend_width, legend_height; /* The panel in pixels, zero when there is none. */
} screen_t;

typedef struct {
    int birds, frame_rate, bird_size, palette, flocks, colour_by;
    int mouse_mode, mouse_reach;
    int wrap, trails, hawks;
    double speed;
    int vision_cells, vision_radius, vision_radius_squared;
    double separation, alignment, cohesion, boundary;
    /* Notch positions, zero to LEGEND_BAR_CELLS. These are the state the keys
     * move; every value above is derived from them, which is what makes one
     * keypress exactly one notch of bar rather than nearly one. */
    int boundary_notch, separation_notch, cohesion_notch, alignment_notch;
    int vision_notch, rate_notch;
} config_t;

static config_t config = {
    .birds = 800,
    .frame_rate = DEFAULT_FRAME_RATE,
    .speed = DEFAULT_SPEED,
    .bird_size = DEFAULT_BIRD_SIZE,
    .palette = 0,
    .flocks = 1,
    .colour_by = COLOUR_BY_HEADING,
    .mouse_mode = MOUSE_FLEE,
    .mouse_reach = DEFAULT_MOUSE_REACH,
    .vision_cells = DEFAULT_VISION_RADIUS / SPATIAL_CELL_SIZE,
    .vision_radius = DEFAULT_VISION_RADIUS,
    .vision_radius_squared = DEFAULT_VISION_RADIUS * DEFAULT_VISION_RADIUS,
    .separation = DEFAULT_SEPARATION_W,
    .alignment = DEFAULT_ALIGNMENT_W,
    .cohesion = DEFAULT_COHESION_W,
    .boundary = DEFAULT_BOUNDARY_W,
    .boundary_notch = DEFAULT_NOTCH,
    .separation_notch = DEFAULT_NOTCH,
    .cohesion_notch = DEFAULT_NOTCH,
    .alignment_notch = DEFAULT_NOTCH,
    /* Twelve to sixty pixels in steps of four: thirty six is the sixth notch. */
    .vision_notch = DEFAULT_VISION_NOTCH,
    .rate_notch = DEFAULT_NOTCH,
};
static screen_t screen;
static int legend_enabled = 1; /* Cleared by --no-legend, never at runtime. */
static int mouse_enabled = 1;  /* Cleared by --no-mouse, never at runtime. */
static int force_graphics;     /* Set by --force: skip the protocol probe. */

/* Where the pointer is, in pixels, and whether it has ever been seen. The
 * terminal reports cells, so the position is the middle of the cell it names:
 * that is as precise as the protocol gets. */
static struct {
    int present;
    double x, y;
} mouse;

/* A monotonic clock for everything that animates on its own: the frame counter
 * for anything that wants to act every so many frames, the seconds for anything
 * that has to look the same whatever the frame rate. */
static struct {
    long frame;
    double seconds;
} clock_state;

/* Paused holds the simulation still but keeps drawing and reading keys, so the
 * panel still answers and a single step is possible. Stepping is one frame of
 * motion granted while paused. */
static int paused;
static int step_once;
static int population_changed;
/* Whether the panel is currently on screen. The panel is anchored at the origin
 * and constant in cells, so it never leaves text behind by moving: the only row
 * ever needing an erase is one it occupied before being switched off. Clearing
 * the whole screen would take the uploaded sprites with it. */
static int legend_drawn;
static struct termios saved_termios;
static volatile sig_atomic_t terminal_is_raw;
static volatile sig_atomic_t terminal_restored;
static volatile sig_atomic_t alt_screen_is_on;

static void write_all(const void *data, size_t length) {
    const char *bytes = data;
    while (length > 0) {
        ssize_t written = write(STDOUT_FILENO, bytes, length);
        if (written < 0) {
            if (errno == EINTR) continue;
            return;
        }
        if (written == 0) return;
        bytes += written;
        length -= (size_t)written;
    }
}

static void restore_terminal(void) {
    if (terminal_restored) return;
    terminal_restored = 1;
    if (terminal_is_raw) {
        tcsetattr(STDIN_FILENO, TCSAFLUSH, &saved_termios);
        terminal_is_raw = 0;
    }
    if (!alt_screen_is_on) return; /* The probe failed before we took the screen. */
    write_all(MOUSE_REPORT_OFF, sizeof(MOUSE_REPORT_OFF) - 1);
    write_all(SYNC_UPDATE_END, sizeof(SYNC_UPDATE_END) - 1);
    write_all(CURSOR_SHOW, sizeof(CURSOR_SHOW) - 1);
    write_all(ALT_SCREEN_OFF, sizeof(ALT_SCREEN_OFF) - 1);
}

static void signal_handler(int signal_number) {
    restore_terminal();
    _exit(128 + signal_number);
}

static void install_signal_handlers(void) {
    static const int signals[] = {SIGINT,  SIGTERM, SIGHUP, SIGQUIT,
                                  SIGSEGV, SIGFPE,  SIGBUS, SIGABRT};
    struct sigaction action;
    memset(&action, 0, sizeof(action));
    action.sa_handler = signal_handler;
    action.sa_flags = (int)SA_RESETHAND;
    sigemptyset(&action.sa_mask);
    for (size_t i = 0; i < sizeof(signals) / sizeof(*signals); i++)
        sigaction(signals[i], &action, NULL);
}

/* Sends a request and collects whatever comes back until a terminator or the
 * deadline, whichever is first. Both the graphics probe and the colour queries
 * are the same shape, asking the terminal a question that it may simply not
 * answer, and neither may hang the startup path waiting for a reply that is
 * never coming. Raw mode has to be on already, or the reply would be echoed and
 * held until a newline. */
static size_t terminal_query(const char *request, size_t request_length, char *reply,
                             size_t reply_size, int milliseconds) {
    struct timespec start, now;
    size_t length = 0;

    if (reply_size == 0) return 0;
    reply[0] = '\0';
    write_all(request, request_length);
    clock_gettime(CLOCK_MONOTONIC, &start);

    for (;;) {
        clock_gettime(CLOCK_MONOTONIC, &now);
        long spent = (now.tv_sec - start.tv_sec) * 1000L + (now.tv_nsec - start.tv_nsec) / 1000000L;
        if (spent >= milliseconds) break;

        struct pollfd wait = {.fd = STDIN_FILENO, .events = POLLIN};
        int ready = poll(&wait, 1, (int)(milliseconds - spent));
        if (ready < 0) {
            if (errno == EINTR) continue;
            break;
        }
        if (ready == 0) break;

        ssize_t got = read(STDIN_FILENO, reply + length, reply_size - 1 - length);
        if (got <= 0) break;
        length += (size_t)got;
        reply[length] = '\0';
        /* Every reply this program asks for ends one of these three ways. */
        if (memchr(reply, '\a', length) != NULL || strstr(reply, "\033\\") != NULL ||
            memchr(reply, 'c', length) != NULL)
            break;
        if (length + 1 >= reply_size) break;
    }
    return length;
}

/*
 * Ask whether the terminal speaks the graphics protocol, rather than drawing to
 * a terminal that cannot show it and leaving a stranger with a black screen and
 * a bad first impression. A one pixel image is offered for query only, which
 * uploads nothing and displays nothing; a terminal that understands answers
 * with an APC reply, and one that does not ignores it silently. The Primary
 * Device Attributes request that follows is the control: every terminal answers
 * that, so an answer to the second with none to the first is a clear no rather
 * than a timeout.
 */
static int terminal_speaks_graphics(void) {
    static const char probe[] = "\033_Gi=31,s=1,v=1,a=q,t=d,f=24;AAAA\033\\\033[c";
    char reply[128];
    size_t length = terminal_query(probe, sizeof(probe) - 1, reply, sizeof(reply), 250);
    if (length == 0) return 0; /* Answered nothing at all: assume the worst. */
    return strstr(reply, "\033_G") != NULL;
}

static void enter_alt_screen(void) {
    write_all(ALT_SCREEN_ON, sizeof(ALT_SCREEN_ON) - 1);
    write_all(CURSOR_HIDE, sizeof(CURSOR_HIDE) - 1);
    if (mouse_enabled) write_all(MOUSE_REPORT_ON, sizeof(MOUSE_REPORT_ON) - 1);
    alt_screen_is_on = 1;
}

static int enter_terminal(void) {
    struct termios raw;
    if (tcgetattr(STDIN_FILENO, &raw) < 0) return -1;
    saved_termios = raw;
    raw.c_iflag &= (tcflag_t) ~(tcflag_t)(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
    raw.c_oflag &= (tcflag_t) ~(tcflag_t)OPOST;
    raw.c_cflag |= CS8;
    raw.c_lflag &= (tcflag_t) ~(tcflag_t)(ECHO | ICANON | IEXTEN);
    raw.c_cc[VSUSP] = _POSIX_VDISABLE;
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw) < 0) return -1;
    terminal_is_raw = 1;
    return 0;
}

/* Kept proportional to the viewport: a fixed pixel distance covers a short
 * terminal entirely and pins the whole flock against one edge. */
static void update_turn_distances(void) {
    screen.turn_x = screen.width / TURN_BAND_DIVISOR;
    screen.turn_y = screen.height / TURN_BAND_DIVISOR;
    screen.turn_bottom = screen.height / BOTTOM_BAND_DIVISOR;
    if (screen.turn_x < 1) screen.turn_x = 1;
    if (screen.turn_y < 1) screen.turn_y = 1;
    if (screen.turn_bottom < 1) screen.turn_bottom = 1;
}

/* The panel takes a corner rather than a row, so the flyable area stays an L and
 * no other derivation has to shrink: the flock keeps the full width below the
 * panel and the full height beside it, and is kept out of the corner by a force
 * instead of by a bound. */
static void measure_legend(void) {
    screen.legend_width = screen.legend_height = 0;
    if (!legend_enabled) return;
    if (screen.cols < LEGEND_MIN_COLS || screen.rows < LEGEND_MIN_ROWS) return;
    screen.legend_width = LEGEND_COLUMNS * screen.cell_width;
    screen.legend_height = LEGEND_ROWS * screen.cell_height;
}

/* Split out of the ioctl query so the tests drive the real derivation. */
static void apply_screen_size(int cols, int rows, int pixel_width, int pixel_height) {
    screen.cols = cols > 0 ? cols : DEFAULT_COLS;
    screen.rows = rows > 0 ? rows : DEFAULT_ROWS;
    screen.width = pixel_width;
    screen.height = pixel_height;
    if (screen.width <= 0 || screen.height <= 0) {
        screen.width = screen.cols * DEFAULT_CELL_WIDTH;
        screen.height = screen.rows * DEFAULT_CELL_HEIGHT;
    }
    screen.cell_width = screen.width / screen.cols;
    screen.cell_height = screen.height / screen.rows;
    if (screen.cell_width < 1) screen.cell_width = 1;
    if (screen.cell_height < 1) screen.cell_height = 1;
    measure_legend();
    update_turn_distances();
}

static void update_screen_dimensions(void) {
    struct winsize size;
    memset(&size, 0, sizeof(size));
    if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &size) < 0) memset(&size, 0, sizeof(size));
    apply_screen_size(size.ws_col, size.ws_row, size.ws_xpixel, size.ws_ypixel);
}

/*
 * The terminal's own colours, asked for rather than guessed.
 *
 * OSC 4 names a palette entry, OSC 10 and 11 the foreground and background. The
 * answer comes back as rgb:RRRR/GGGG/BBBB, four hex digits a channel, and a
 * terminal that does not implement the query simply says nothing. A ramp is then
 * built between the most saturated answer and the background, which is what
 * makes a screenshot match the poster's own setup: the single thing that decides
 * whether a terminal toy looks native or looks imported.
 */
static uint8_t theme_tints[5][3];
static int theme_is_known;

static int parse_osc_colour(const char *reply, uint8_t rgb[3]) {
    const char *at = strstr(reply, "rgb:");
    unsigned r, g, b;
    if (at == NULL) return 0;
    if (sscanf(at + 4, "%4x/%4x/%4x", &r, &g, &b) != 3) return 0;
    /* Four hex digits a channel is the usual answer, but some terminals send
     * two; scale whichever came back down to a byte. */
    const char *slash = strchr(at + 4, '/');
    int digits = slash ? (int)(slash - (at + 4)) : 4;
    int shift = digits >= 4 ? 8 : 0;
    rgb[0] = (uint8_t)(r >> shift);
    rgb[1] = (uint8_t)(g >> shift);
    rgb[2] = (uint8_t)(b >> shift);
    return 1;
}

static int ask_colour(const char *request, uint8_t rgb[3]) {
    char reply[128];
    if (terminal_query(request, strlen(request), reply, sizeof(reply), 60) == 0) return 0;
    return parse_osc_colour(reply, rgb);
}

static int saturation_of(const uint8_t rgb[3]) {
    int high = rgb[0] > rgb[1] ? rgb[0] : rgb[1];
    int low = rgb[0] < rgb[1] ? rgb[0] : rgb[1];
    if (rgb[2] > high) high = rgb[2];
    if (rgb[2] < low) low = rgb[2];
    return high - low;
}

/* Five steps from the accent to the background, so the ramp ends where the
 * screen does and the far end of the flock reads as distance. */
static void ramp_between(const uint8_t from[3], const uint8_t to[3]) {
    for (int i = 0; i < 5; i++)
        for (int c = 0; c < 3; c++)
            theme_tints[i][c] = (uint8_t)(from[c] + (to[c] - from[c]) * i / 6);
}

static int learn_the_theme(void) {
    uint8_t accent[3] = {0, 0, 0}, background[3] = {0, 0, 0};
    int best = -1;

    /* Entries one to six are the terminal's own reds through cyans, which is
     * where a colour scheme keeps its character. */
    for (int entry = 1; entry <= 6; entry++) {
        char request[32];
        uint8_t rgb[3];
        snprintf(request, sizeof(request), "\033]4;%d;?\033\\", entry);
        if (!ask_colour(request, rgb)) continue;
        int saturation = saturation_of(rgb);
        if (saturation > best) {
            best = saturation;
            memcpy(accent, rgb, sizeof(accent));
        }
    }
    if (best < 0) return 0;
    if (!ask_colour("\033]11;?\033\\", background)) {
        /* No background either: fade towards black, which is the common case. */
        background[0] = background[1] = background[2] = 0;
    }
    ramp_between(accent, background);
    theme_is_known = 1;
    return 1;
}

/*
 * A palette is a list of tints applied to the one embedded sprite. The first
 * entry of every palette is the sprite untouched, so a bird with no shade of
 * its own looks exactly as it always did.
 */
typedef struct {
    const char *name;
    const char *help;
    int shades;
    const uint8_t (*tints)[3];
    png_tint_mode_t mode;
} palette_t;

static const uint8_t EMBER_TINTS[][3] = {
    {255, 214, 138}, {255, 176, 66}, {247, 122, 41}, {224, 74, 39}, {173, 44, 51},
};
static const uint8_t ICE_TINTS[][3] = {
    {226, 246, 255}, {160, 220, 250}, {96, 176, 236}, {58, 122, 206}, {44, 74, 158},
};
static const uint8_t ACID_TINTS[][3] = {
    {238, 255, 176}, {186, 244, 96}, {118, 214, 74}, {54, 176, 108}, {26, 122, 106},
};
static const uint8_t PAPER_TINTS[][3] = {
    {248, 246, 240}, {206, 202, 192}, {158, 154, 146}, {104, 102, 98}, {48, 48, 46},
};

static const palette_t PALETTES[] = {
    {"theme", "the terminal's own colours, asked for at startup", 5,
     (const uint8_t (*)[3])theme_tints, PNG_TINT_REPLACE},
    {"original", "the sprite as it was drawn", 1, NULL, PNG_TINT_MULTIPLY},
    {"ember", "embers, pale gold to deep red", 5, EMBER_TINTS, PNG_TINT_REPLACE},
    {"ice", "ice, white through to deep blue", 5, ICE_TINTS, PNG_TINT_REPLACE},
    {"acid", "acid, lime through to teal", 5, ACID_TINTS, PNG_TINT_REPLACE},
    {"paper", "paper, five greys", 5, PAPER_TINTS, PNG_TINT_REPLACE},
};
enum { PALETTE_COUNT = sizeof(PALETTES) / sizeof(*PALETTES) };

static const char *PALETTE_NAMES[PALETTE_COUNT + 1];
static const char *const COLOUR_BY_NAMES[] = {"fixed", "heading", "density", "flock", NULL};
static const char *const MOUSE_NAMES[] = {"flee", "follow", "cat", "off", NULL};

/* Named rather than numbered: the table's order is a presentation choice and
 * should not be load bearing. */
static int palette_named(const char *name) {
    for (int i = 0; i < PALETTE_COUNT; i++)
        if (strcmp(PALETTES[i].name, name) == 0) return i;
    return 0;
}

static int palette_follows_the_theme(void) {
    return strcmp(PALETTES[config.palette].name, "theme") == 0;
}

#define FALLBACK_PALETTE palette_named("ember")

static void name_the_palettes(void) {
    for (int i = 0; i < PALETTE_COUNT; i++) PALETTE_NAMES[i] = PALETTES[i].name;
    PALETTE_NAMES[PALETTE_COUNT] = NULL;
}

static const palette_t *palette(void) {
    return &PALETTES[config.palette];
}

static int palette_shades(void) {
    return palette()->shades;
}

/* Shade zero of a tinted palette is still a tint: the list is the whole ramp. */
static void palette_tint(png_image_t *image, int shade) {
    const palette_t *chosen = palette();
    if (chosen->tints == NULL) return;
    if (shade < 0) shade = 0;
    if (shade >= chosen->shades) shade = chosen->shades - 1;
    png_tint(image, chosen->tints[shade][0], chosen->tints[shade][1], chosen->tints[shade][2],
             chosen->mode);
}

/* Kitty has no per placement tint, so a colour is a second set of images. The
 * rotation is the expensive half and it does not depend on the colour, so each
 * angle is rotated once and then tinted and encoded per shade: the second shade
 * costs a few hundred microseconds, not another fifty milliseconds. */
static void build_rotation_frames(image_frame_t *frames, int shades, int size) {
    png_image_t source = {0, 0, NULL}, canvas = {0, 0, NULL};
    png_status_t status = png_decode(sprite_png, sprite_png_len, &source);
    if (status != PNG_OK) {
        fprintf(stderr, "Cannot decode the embedded sprite: %s\n", png_status_string(status));
        exit(EXIT_FAILURE);
    }
    int canvas_size = size * SPRITE_SUPERSAMPLE;
    if (canvas_size > SPRITE_WORK_MAX) canvas_size = SPRITE_WORK_MAX;
    if (canvas_size > source.width) canvas_size = source.width;
    status = png_resize(&source, canvas_size, canvas_size, &canvas);
    png_image_free(&source);
    if (status != PNG_OK) {
        fprintf(stderr, "Cannot scale the embedded sprite: %s\n", png_status_string(status));
        exit(EXIT_FAILURE);
    }
    for (int i = 0; i < ROTATION_FRAMES; i++) {
        png_image_t frame = {0, 0, NULL};
        double radians = i * FRAME_ANGLE * M_PI / 180.0;
        status = png_rotate_resize(&canvas, radians, size, size, &frame);
        for (int shade = 0; status == PNG_OK && shade < shades; shade++) {
            png_image_t tinted = {0, 0, NULL};
            uint8_t *encoded = NULL;
            size_t encoded_length = 0;
            status = png_image_alloc(&tinted, frame.width, frame.height);
            if (status == PNG_OK) {
                memcpy(tinted.pixels, frame.pixels, (size_t)frame.width * (size_t)frame.height * 4);
                palette_tint(&tinted, shade);
                status = png_encode(&tinted, &encoded, &encoded_length);
            }
            png_image_free(&tinted);
            if (status != PNG_OK) break;
            frames[shade * ROTATION_FRAMES + i].data = encoded;
            frames[shade * ROTATION_FRAMES + i].length = encoded_length;
        }
        png_image_free(&frame);
        if (status != PNG_OK) {
            fprintf(stderr, "Cannot build the rotation frames: %s\n", png_status_string(status));
            exit(EXIT_FAILURE);
        }
    }
    png_image_free(&canvas);
}

/* Image ids run shade major: shade * ROTATION_FRAMES + frame + 1, so a bird's
 * image is a multiply and an add away from its heading and its shade. */
static uint32_t sprite_image_id(int shade, int frame) {
    return (uint32_t)(shade * ROTATION_FRAMES + frame) + 1;
}

/* The hawks' own set, uploaded after the flock's, so one arithmetic rule covers
 * every placement the program makes. */
static uint32_t hawk_image_id(int frame) {
    return (uint32_t)(palette_shades() * ROTATION_FRAMES + frame) + 1;
}

static kitty_graphics_status_t upload_rotation_frames(kitty_graphics_t *graphics,
                                                      const image_frame_t *frames, int shades) {
    for (int i = 0; i < ROTATION_FRAMES * shades; i++) {
        kitty_graphics_status_t status =
            kitty_graphics_upload_png(graphics, (uint32_t)i + 1, frames[i].data, frames[i].length);
        if (status != KITTY_GRAPHICS_OK) return status;
    }
    kitty_graphics_status_t status = kitty_graphics_delete_all_placements(graphics);
    return status == KITTY_GRAPHICS_OK ? kitty_graphics_flush(graphics) : status;
}

static void free_rotation_frames(image_frame_t *frames, int shades) {
    for (int i = 0; i < ROTATION_FRAMES * shades; i++) {
        free(frames[i].data);
        frames[i].data = NULL;
        frames[i].length = 0;
    }
}

static double random_unit(void) {
    return (double)rand() / RAND_MAX;
}

static int direction_frame(double radians) {
    int degrees = (int)(radians * 180.0 / M_PI);
    return ((degrees % 360 + 360) % 360) / FRAME_ANGLE;
}

/* Where the force acts: the panel grown by one frame of travel. A bird just
 * outside this rectangle lands at worst one epsilon inside the panel edge, which
 * is still outside the panel itself, and by then the push is on. That is what
 * makes the panel unreachable rather than merely unwelcoming, and the margin
 * follows the frame rate because speed does. */
static int legend_turn_zone(double x, double y) {
    return screen.legend_width > 0 && x < screen.legend_width + config.speed &&
           y < screen.legend_height + config.speed;
}

/* Out through the nearer of the two open sides. The panel sits in a corner, so
 * the only ways out are right and down, and the push never aims at a screen
 * edge. The magnitude settles the direction by itself. */
static int legend_repels(const bird_t *bird, vector_t *boundary) {
    if (!legend_turn_zone(bird->x, bird->y)) return 0;
    double escape_x = screen.legend_width + config.speed - bird->x;
    double escape_y = screen.legend_height + config.speed - bird->y;
    if (escape_x <= escape_y)
        boundary->x = LEGEND_PUSH;
    else
        boundary->y = LEGEND_PUSH;
    return 1;
}

/* Spread over the region no turn band covers, so no bird starts by fleeing an
 * edge and the flock does not begin stacked on a single point. */
/*
 * The flock writes.
 *
 * A target per lit cell of the text, laid out in the rectangle the panel leaves
 * free, and a bird per target round robin so a cell with several birds on it
 * reads as a thick stroke. While it is writing, a bird steers at its target and
 * moves the smaller of its speed and the distance left, which is what lets it
 * land exactly instead of orbiting: the letters come out crisp and then breathe,
 * because the flocking terms are still there underneath.
 *
 * Targets never fall inside the panel's turn zone, because the force that keeps
 * birds off the panel is unanswerable and a target in there could never be
 * reached.
 */
enum { SPELL_MAX_TARGETS = 2048 };

static struct {
    int count;
    double x[SPELL_MAX_TARGETS];
    double y[SPELL_MAX_TARGETS];
    double until; /* Seconds on the clock at which to let go; negative is never. */
    int writing;
} spell;

static void spell_clear(void) {
    spell.count = 0;
    spell.writing = 0;
}

/* Lays the text out and returns how many targets it made, zero if it will not
 * fit or the text has nothing to draw. */
static int spell_layout(const char *text) {
    double pad = config.bird_size * 2.0;
    double left = screen.legend_width > 0 ? screen.legend_width + config.speed + pad : pad;
    double top = pad, right = screen.width - pad, bottom = screen.height - pad;
    int columns = font_text_width(text);

    spell_clear();
    if (columns <= 0 || right - left < columns || bottom - top < FONT_HEIGHT) return 0;

    /* One cell is as large as both dimensions allow, so the text fills the space
     * it has without being stretched out of shape. */
    double cell = (right - left) / columns;
    double by_height = (bottom - top) / FONT_HEIGHT;
    if (by_height < cell) cell = by_height;

    double width = columns * cell, height = FONT_HEIGHT * cell;
    double origin_x = left + (right - left - width) / 2;
    double origin_y = top + (bottom - top - height) / 2;

    int column = 0;
    for (const char *c = text; *c != '\0'; c++) {
        const char *glyph = font_glyph(*c);
        if (glyph == NULL) continue;
        for (int row = 0; row < FONT_HEIGHT; row++) {
            for (int x = 0; x < FONT_WIDTH; x++) {
                if (glyph[row * FONT_WIDTH + x] != '#') continue;
                if (spell.count >= SPELL_MAX_TARGETS) break;
                double px = origin_x + (column + x + 0.5) * cell;
                double py = origin_y + (row + 0.5) * cell;
                if (legend_turn_zone(px, py)) continue;
                spell.x[spell.count] = px;
                spell.y[spell.count] = py;
                spell.count++;
            }
        }
        column += FONT_ADVANCE;
    }
    spell.writing = spell.count > 0;
    return spell.count;
}

static int spell_target_of(int index, double *x, double *y) {
    if (!spell.writing || spell.count == 0) return 0;
    *x = spell.x[index % spell.count];
    *y = spell.y[index % spell.count];
    return 1;
}

static double normalized_angle(double y, double x) {
    double angle = atan2(y, x);
    return angle < 0 ? angle + 2 * M_PI : angle;
}

/*
 * Hawks.
 *
 * A predator is what gives a clip a story: the flock splits, streams around it
 * and closes again behind, which is the part people loop. A hawk is not a boid.
 * It has no neighbours, obeys none of the three rules, and is kept out of the
 * grid entirely, so the flocking maths is untouched by its existence: it simply
 * chases the nearest bird and every bird flees it.
 */
typedef struct {
    double x, y, direction;
    int frame;
} hawk_t;

static hawk_t hawks[MAX_HAWKS];
/* The hawks' images always go up, so k can summon one at any time. */
static int hawk_sets_built;

static void place_hawks(void) {
    for (int i = 0; i < config.hawks; i++) {
        hawks[i].x = screen.width * (i + 1.0) / (config.hawks + 1.0);
        hawks[i].y = screen.height * (i % 2 ? 0.75 : 0.25);
        hawks[i].direction = 2 * M_PI * random_unit();
        hawks[i].frame = direction_frame(hawks[i].direction);
    }
}

/* Brute force over the flock, because eight hawks against four thousand birds is
 * a few tens of thousands of comparisons: less than one percent of a frame, and
 * it needs no grid of its own. */
static void hunt(const bird_t *birds) {
    for (int i = 0; i < config.hawks; i++) {
        double best = -1, best_x = 0, best_y = 0;
        for (int b = 0; b < config.birds; b++) {
            double dx = birds[b].x - hawks[i].x, dy = birds[b].y - hawks[i].y;
            double squared = dx * dx + dy * dy;
            if (best < 0 || squared < best) {
                best = squared;
                best_x = birds[b].x;
                best_y = birds[b].y;
            }
        }
        if (best >= 0) {
            double to_x = best_x - hawks[i].x, to_y = best_y - hawks[i].y;
            if (to_x * to_x + to_y * to_y > 1e-9) hawks[i].direction = normalized_angle(to_y, to_x);
        }
        /* Faster than its prey, or it would never catch up and the chase would
         * never look like one. Turned back at the edges like everything else. */
        hawks[i].x += config.speed * HAWK_SPEED * cos(hawks[i].direction);
        hawks[i].y += config.speed * HAWK_SPEED * sin(hawks[i].direction);
        if (hawks[i].x < 0) hawks[i].x = 0;
        if (hawks[i].x > screen.width) hawks[i].x = screen.width;
        if (hawks[i].y < 0) hawks[i].y = 0;
        if (hawks[i].y > screen.height) hawks[i].y = screen.height;
        hawks[i].frame = direction_frame(hawks[i].direction);
    }
}

/* Every bird flees every hawk in reach, hardest when it is closest. */
static vector_t hawk_vector(const bird_t *bird) {
    vector_t force = {0, 0};
    for (int i = 0; i < config.hawks; i++) {
        double dx = bird->x - hawks[i].x, dy = bird->y - hawks[i].y;
        double squared = dx * dx + dy * dy;
        if (squared >= HAWK_REACH * HAWK_REACH || squared < 1e-9) continue;
        double distance = sqrt(squared);
        double strength = (HAWK_REACH - distance) / HAWK_REACH;
        force.x += strength * dx / distance;
        force.y += strength * dy / distance;
    }
    return force;
}

/* One bird, so that growing the flock at runtime places only the new ones. */
static void place_one_bird(bird_t *bird, int index) {
    double min_x = screen.turn_x, max_x = screen.width - screen.turn_x;
    double min_y = screen.turn_y, max_y = screen.height - screen.turn_bottom;
    if (max_x <= min_x) min_x = max_x = screen.width / 2.0;
    if (max_y <= min_y) min_y = max_y = screen.height / 2.0;

    /* Rejected rather than clamped, so the whole free region stays in play
     * instead of the flock piling up along one edge of the panel. Bounded, with a
     * deterministic fallback below the panel, because on a small viewport the
     * free region can be almost entirely covered. */
    for (int attempt = 0;; attempt++) {
        bird->x = min_x + (max_x - min_x) * random_unit();
        bird->y = min_y + (max_y - min_y) * random_unit();
        if (!legend_turn_zone(bird->x, bird->y)) break;
        if (attempt + 1 >= SPAWN_ATTEMPTS) {
            bird->y = screen.legend_height + config.speed + 1;
            if (bird->y > max_y) bird->x = screen.legend_width + config.speed + 1;
            break;
        }
    }
    bird->direction = 2 * M_PI * random_unit();
    bird->frame = direction_frame(bird->direction);
    /* Flocks are handed out round robin so they come out even, and when there is
     * more than one the palette follows them: a colour per flock is what makes
     * two of them legible as two. */
    bird->flock = index % config.flocks;
    bird->shade = config.flocks > 1 ? bird->flock % palette_shades()
                                    : (int)(random_unit() * palette_shades()) % palette_shades();
}

static void initialize_birds(bird_t *birds) {
    for (int i = 0; i < config.birds; i++) place_one_bird(&birds[i], i);
}

/*
 * The pointer, as a thing in the world.
 *
 * flee is the default because it is the one that explains itself: the flock
 * parts around the cursor within a second of the viewer moving it, and nobody
 * has to be told what happened. follow is the opposite and makes the pointer a
 * feeder. cat holds still and waits, the flock creeps back, and then it pounces:
 * this is kitty, after all.
 *
 * The force falls off with distance rather than being a hard wall like the
 * panel's, because the flock has to bend around the pointer and close again
 * behind it, not bounce off it.
 */
static vector_t pointer_vector(const bird_t *bird) {
    vector_t force = {0, 0};
    if (!mouse.present || config.mouse_mode == MOUSE_OFF) return force;

    double dx = bird->x - mouse.x, dy = bird->y - mouse.y;
    double squared = dx * dx + dy * dy;
    double reach = config.mouse_reach;
    if (squared >= reach * reach || squared < 1e-9) return force;

    double distance = sqrt(squared);
    /* One at the pointer, nothing at the edge of its reach. */
    double strength = (reach - distance) / reach;
    double pull = config.mouse_mode == MOUSE_FOLLOW ? -1.0 : 1.0;

    if (config.mouse_mode == MOUSE_CAT) {
        /* Still for a while, then a pounce: the flock has time to come back and
         * be surprised. The pounce is a stronger flee over a wider reach. */
        double phase = fmod(clock_state.seconds, CAT_PERIOD);
        if (phase < CAT_STALK) return force;
        strength *= CAT_POUNCE;
    }
    force.x = pull * strength * dx / distance;
    force.y = pull * strength * dy / distance;
    return force;
}

static vector_t boundary_vector(const bird_t *bird) {
    vector_t boundary = {0, 0};
    if (legend_repels(bird, &boundary)) return boundary;
    if (config.wrap) return boundary; /* A door, so no wall. */
    if (bird->x < screen.turn_x)
        boundary.x = 1;
    else if (bird->x > screen.width - screen.turn_x)
        boundary.x = -1;
    if (bird->y < screen.turn_y)
        boundary.y = 1;
    else if (bird->y > screen.height - screen.turn_bottom)
        boundary.y = -1;
    return boundary;
}

static void read_bird_position(const void *context, int index, double *x, double *y) {
    const bird_t *birds = context;
    *x = birds[index].x;
    *y = birds[index].y;
}

static double flock_direction(const bird_t *birds, const spatial_grid_t *grid, int target_index,
                              int *crowd) {
    const bird_t *target = &birds[target_index];
    double want_x, want_y;
    /* Writing overrules flocking while it lasts: a bird with a target steers at
     * it and nothing else, which is what makes a letter a letter. */
    if (spell_target_of(target_index, &want_x, &want_y)) {
        double to_x = want_x - target->x, to_y = want_y - target->y;
        if (crowd != NULL) *crowd = 0;
        if (to_x * to_x + to_y * to_y > 1e-9) return normalized_angle(to_y, to_x);
        return target->direction;
    }
    vector_t separation = {0, 0}, alignment = {0, 0}, cohesion = {0, 0};
    vector_t boundary = boundary_vector(target);
    vector_t pointer = pointer_vector(target);
    vector_t hawk = hawk_vector(target);
    int neighbors = 0, kin = 0;
    int center_x, center_y;
    spatial_grid_cell_for_position(grid, target->x, target->y, &center_x, &center_y);
    int min_x = center_x - config.vision_cells;
    int max_x = center_x + config.vision_cells;
    int min_y = center_y - config.vision_cells;
    int max_y = center_y + config.vision_cells;
    if (min_x < 0) min_x = 0;
    if (min_y < 0) min_y = 0;
    if (max_x >= grid->columns) max_x = grid->columns - 1;
    if (max_y >= grid->rows) max_y = grid->rows - 1;

    for (int cell_y = min_y; cell_y <= max_y; cell_y++) {
        for (int cell_x = min_x; cell_x <= max_x; cell_x++) {
            int cell = cell_y * grid->columns + cell_x;
            for (int slot = grid->offsets[cell]; slot < grid->offsets[cell + 1]; slot++) {
                int i = grid->indices[slot];
                if (i == target_index) continue;
                const bird_t *other = &birds[i];
                double dx = target->x - other->x, dy = target->y - other->y;
                if (dx * dx + dy * dy >= config.vision_radius_squared) continue;
                /* Separation is physical and applies to every bird in reach.
                 * Alignment and cohesion are social, and a bird only reads its
                 * own flock: two flocks pass through each other, swirl, and
                 * refuse to merge. With one flock every neighbour is kin and the
                 * arithmetic is exactly what it was. */
                separation.x += dx;
                separation.y += dy;
                neighbors++;
                if (other->flock != target->flock) continue;
                alignment.x += cos(other->direction);
                alignment.y += sin(other->direction);
                cohesion.x += other->x;
                cohesion.y += other->y;
                kin++;
            }
        }
    }
    if (crowd != NULL) *crowd = neighbors;
    if (neighbors) {
        if (kin) {
            alignment.x /= kin;
            alignment.y /= kin;
            cohesion.x = cohesion.x / kin - target->x;
            cohesion.y = cohesion.y / kin - target->y;
        }
        double x = separation.x * config.separation + alignment.x * config.alignment +
                   cohesion.x * config.cohesion + boundary.x * config.boundary +
                   pointer.x * MOUSE_WEIGHT + hawk.x * HAWK_WEIGHT;
        double y = separation.y * config.separation + alignment.y * config.alignment +
                   cohesion.y * config.cohesion + boundary.y * config.boundary +
                   pointer.y * MOUSE_WEIGHT + hawk.y * HAWK_WEIGHT;
        return x == 0 && y == 0 ? target->direction : normalized_angle(y, x);
    }
    boundary.x = boundary.x * config.boundary + pointer.x * MOUSE_WEIGHT + hawk.x * HAWK_WEIGHT;
    boundary.y = boundary.y * config.boundary + pointer.y * MOUSE_WEIGHT + hawk.y * HAWK_WEIGHT;
    if (boundary.x != 0 || boundary.y != 0) {
        double x = cos(target->direction) + boundary.x;
        double y = sin(target->direction) + boundary.y;
        if (x != 0 || y != 0) return normalized_angle(y, x);
    }
    return target->direction;
}

/* What decides a bird's shade within the ramp. Heading is the striking one: a
 * turn runs a ripple of colour through the whole flock, because neighbours that
 * agree on a heading agree on a colour. */
static int shade_for(const bird_t *bird, int crowd) {
    int shades = palette_shades();
    if (shades <= 1) return 0;
    switch (config.colour_by) {
        case COLOUR_BY_HEADING: {
            double turns =
                normalized_angle(sin(bird->direction), cos(bird->direction)) / (2 * M_PI);
            int shade = (int)(turns * shades);
            return shade >= shades ? shades - 1 : shade;
        }
        case COLOUR_BY_DENSITY: {
            /* Eight neighbours is a crowd at the default radius, so the ramp is
             * spent by then and the densest knots read as the far end of it. */
            int shade = crowd * shades / 9;
            return shade >= shades ? shades - 1 : shade;
        }
        case COLOUR_BY_FLOCK:
            return bird->flock % shades;
        default:
            return bird->shade;
    }
}

/* Off one edge and back on the other. The edges stop pushing back when this is
 * on, because a wall and a door in the same place is neither. */
static void wrap_position(bird_t *bird) {
    double width = screen.width, height = screen.height;
    if (bird->x < 0) bird->x += width;
    if (bird->x >= width) bird->x -= width;
    if (bird->y < 0) bird->y += height;
    if (bird->y >= height) bird->y -= height;
}

static void update_birds(bird_t *birds, const bird_t *snapshot, const spatial_grid_t *grid) {
    for (int i = 0; i < config.birds; i++) {
        int crowd = 0;
        double direction = flock_direction(snapshot, grid, i, &crowd);
        birds[i].direction = direction;
        /* Never past the target: the last step is the distance left, which is
         * what makes a letter crisp instead of a cloud orbiting one. */
        double step = config.speed;
        double want_x, want_y;
        if (spell_target_of(i, &want_x, &want_y)) {
            double dx = want_x - birds[i].x, dy = want_y - birds[i].y;
            double remaining = sqrt(dx * dx + dy * dy);
            if (remaining < step) step = remaining;
        }
        birds[i].x += step * cos(direction);
        birds[i].y += step * sin(direction);
        if (config.wrap) wrap_position(&birds[i]);
        birds[i].shade = shade_for(&birds[i], crowd);
        if (config.trails && i % TRAIL_EVERY == 0) {
            /* Where it was, not where it is: a tail behind, never under. */
            birds[i].trail_x[birds[i].trail_at] = snapshot[i].x;
            birds[i].trail_y[birds[i].trail_at] = snapshot[i].y;
            birds[i].trail_at = (birds[i].trail_at + 1) % TRAIL_LENGTH;
            if (birds[i].trail_held < TRAIL_LENGTH) birds[i].trail_held++;
        }
    }
}

static int bird_placement(const bird_t *bird, kitty_graphics_placement_t *placement) {
    if (bird->x < 0 || bird->y < 0) return 0;

    int pixel_x = (int)bird->x;
    int pixel_y = (int)bird->y;
    int column = pixel_x / screen.cell_width;
    int row = pixel_y / screen.cell_height;
    if (column >= screen.cols || row >= screen.rows) return 0;

    *placement = (kitty_graphics_placement_t){
        .image_id = sprite_image_id(bird->shade, bird->frame),
        .placement_id = 0,
        .row = row,
        .column = column,
        .x_offset = pixel_x % screen.cell_width,
        .y_offset = pixel_y % screen.cell_height,
        .z_index = 0,
    };
    return 1;
}

/* One slider row: the name, the bar, and the two keys that move it, the lowering
 * one first because that is the end of the bar it works from. The filled length
 * is the notch itself, not a value scaled into cells, so a keypress moves the bar
 * by exactly one cell and nothing rounds. No number: the bar is the readout. */
static void legend_slider(char *line, size_t size, const char *name, int notch, const char *value,
                          char lower, char raise) {
    char bar[LEGEND_BAR_CELLS * 3 + 1];
    size_t at = 0;

    for (int cell = 0; cell < LEGEND_BAR_CELLS; cell++) {
        const char *glyph = cell < notch ? "\u2593" : "\u2591";
        memcpy(bar + at, glyph, 3);
        at += 3;
    }
    bar[at] = '\0';
    /* The value is right aligned in a column of its own, so the numbers line up
     * under each other and the keys stay where the eye already looks for them. */
    snprintf(line, size, "\u2502 %-*s %s %*s  %c/%c \u2502", LEGEND_NAME_WIDTH, name, bar,
             LEGEND_VALUE_WIDTH, value, lower, raise);
}

/* Enough decimals to tell one notch from the next, and no more: separation and
 * cohesion step in thousandths, the other two in hundredths. */
static void legend_number(char *out, size_t size, double value, int decimals) {
    snprintf(out, size, "%.*f", decimals, value);
}

/* The panel, ten rows of it, anchored to the top left corner. */
static void build_legend(char lines[LEGEND_ROWS][LEGEND_LINE_MAX]) {
    int inner = LEGEND_COLUMNS - 2;
    size_t at = 0;

    memcpy(lines[0], "\u256d", 3);
    at = 3;
    for (int i = 0; i < inner; i++, at += 3) memcpy(lines[0] + at, "\u2500", 3);
    memcpy(lines[0] + at, "\u256e", 4);

    char value[LEGEND_VALUE_WIDTH + 8];
    legend_number(value, sizeof(value), config.boundary, 2);
    legend_slider(lines[1], LEGEND_LINE_MAX, "boundary", config.boundary_notch, value, 'b', 'B');
    legend_number(value, sizeof(value), config.separation, 3);
    legend_slider(lines[2], LEGEND_LINE_MAX, "separation", config.separation_notch, value, 's',
                  'S');
    legend_number(value, sizeof(value), config.cohesion, 3);
    legend_slider(lines[3], LEGEND_LINE_MAX, "cohesion", config.cohesion_notch, value, 'c', 'C');
    legend_number(value, sizeof(value), config.alignment, 2);
    legend_slider(lines[4], LEGEND_LINE_MAX, "alignment", config.alignment_notch, value, 'a', 'A');
    /* Pixels and frames a second are whole numbers, so they print as such. */
    snprintf(value, sizeof(value), "%dpx", config.vision_radius);
    legend_slider(lines[5], LEGEND_LINE_MAX, "perception", config.vision_notch, value, 'p', 'P');
    snprintf(value, sizeof(value), "%d", config.frame_rate);
    legend_slider(lines[6], LEGEND_LINE_MAX, "rate", config.rate_notch, value, 'r', 'R');

    snprintf(lines[7], LEGEND_LINE_MAX, "\u2502 %*s \u2502", inner - 2, "");
    snprintf(lines[8], LEGEND_LINE_MAX, "\u2502 %-*s q%*s \u2502", LEGEND_NAME_WIDTH, "quit",
             inner - LEGEND_NAME_WIDTH - 4, "");

    memcpy(lines[9], "\u2570", 3);
    at = 3;
    for (int i = 0; i < inner; i++, at += 3) memcpy(lines[9] + at, "\u2500", 3);
    memcpy(lines[9] + at, "\u256f", 4);
}

static kitty_graphics_status_t queue_legend(kitty_graphics_t *graphics) {
    char lines[LEGEND_ROWS][LEGEND_LINE_MAX];

    if (screen.legend_width == 0) {
        /* Switched off by a viewport that shrank under it. The panel never moves
         * and never changes size, so these are the only rows that can ever hold
         * stale text, and they are erased one line at a time: an erase of the
         * whole screen would delete the uploaded sprites and leave every later
         * placement pointing at nothing. */
        if (!legend_drawn) return KITTY_GRAPHICS_OK;
        for (int row = 0; row < LEGEND_ROWS; row++) {
            kitty_graphics_status_t status = kitty_graphics_write_text(graphics, row, 0, "\033[K");
            if (status != KITTY_GRAPHICS_OK) return status;
        }
        legend_drawn = 0;
        return KITTY_GRAPHICS_OK;
    }

    build_legend(lines);
    for (int row = 0; row < LEGEND_ROWS; row++) {
        kitty_graphics_status_t status = kitty_graphics_write_text(graphics, row, 0, lines[row]);
        if (status != KITTY_GRAPHICS_OK) return status;
    }
    legend_drawn = 1;
    return KITTY_GRAPHICS_OK;
}

static kitty_graphics_status_t queue_render_frame(kitty_graphics_t *graphics, const bird_t *birds) {
    kitty_graphics_status_t status = kitty_graphics_begin_synchronized_update(graphics);
    if (status == KITTY_GRAPHICS_OK) status = kitty_graphics_delete_all_placements(graphics);
    if (config.trails && palette_shades() > 1) {
        int faint = palette_shades() - 1; /* The far end of the ramp. */
        for (int i = 0; status == KITTY_GRAPHICS_OK; i += TRAIL_EVERY) {
            if (i >= config.birds) break;
            for (int step = 0; step < birds[i].trail_held; step++) {
                bird_t ghost = birds[i];
                ghost.x = birds[i].trail_x[step];
                ghost.y = birds[i].trail_y[step];
                ghost.shade = faint;
                kitty_graphics_placement_t placement;
                if (bird_placement(&ghost, &placement))
                    status = kitty_graphics_place(graphics, &placement);
                if (status != KITTY_GRAPHICS_OK) break;
            }
        }
    }
    for (int i = 0; status == KITTY_GRAPHICS_OK && i < config.birds; i++) {
        kitty_graphics_placement_t placement;
        if (bird_placement(&birds[i], &placement))
            status = kitty_graphics_place(graphics, &placement);
    }
    for (int i = 0; status == KITTY_GRAPHICS_OK && i < config.hawks; i++) {
        bird_t as_bird = {.x = hawks[i].x, .y = hawks[i].y, .frame = hawks[i].frame};
        kitty_graphics_placement_t placement;
        if (bird_placement(&as_bird, &placement)) {
            placement.image_id = hawk_image_id(hawks[i].frame);
            status = kitty_graphics_place(graphics, &placement);
        }
    }
    if (status == KITTY_GRAPHICS_OK) status = queue_legend(graphics);
    if (status == KITTY_GRAPHICS_OK) status = kitty_graphics_end_synchronized_update(graphics);
    return status;
}

static kitty_graphics_status_t render_frame(kitty_graphics_t *graphics, bird_t *birds,
                                            const bird_t *snapshot, const spatial_grid_t *grid) {
    kitty_graphics_status_t status = queue_render_frame(graphics, birds);
    if (status != KITTY_GRAPHICS_OK) return status;

    /* Paused still draws and still reads keys, so the panel answers and the
     * sliders can be explored on a still frame. A step grants one frame of
     * motion and then stands still again. */
    if (paused && !step_once) return KITTY_GRAPHICS_OK;
    step_once = 0;
    update_birds(birds, snapshot, grid);
    for (int i = 0; i < config.birds; i++) birds[i].frame = direction_frame(birds[i].direction);
    return KITTY_GRAPHICS_OK;
}

static void update_speed(void) {
    config.speed = (double)DEFAULT_SPEED * DEFAULT_FRAME_RATE / config.frame_rate;
}

static double notch_value(int notch, double minimum, double maximum) {
    return minimum + (maximum - minimum) * notch / LEGEND_BAR_CELLS;
}

/* The inverse: which notch a real value belongs to. */
static int notch_for_integer(int value, int minimum, int maximum) {
    return ((value - minimum) * LEGEND_BAR_CELLS + (maximum - minimum) / 2) / (maximum - minimum);
}

static int notch_integer(int notch, int minimum, int maximum) {
    return minimum + ((maximum - minimum) * notch + LEGEND_BAR_CELLS / 2) / LEGEND_BAR_CELLS;
}

/* Derives every tunable from its notch. Called after any key that moves one, so
 * the values and the bars can never disagree. */
static void apply_notches(void) {
    config.boundary = notch_value(config.boundary_notch, BOUNDARY_MIN, BOUNDARY_MAX);
    config.separation = notch_value(config.separation_notch, SEPARATION_MIN, SEPARATION_MAX);
    config.cohesion = notch_value(config.cohesion_notch, COHESION_MIN, COHESION_MAX);
    config.alignment = notch_value(config.alignment_notch, ALIGNMENT_MIN, ALIGNMENT_MAX);

    config.vision_radius = notch_integer(config.vision_notch, MIN_VISION_RADIUS, MAX_VISION_RADIUS);
    config.vision_radius_squared = config.vision_radius * config.vision_radius;
    /* The block of cells the search sweeps has to cover the radius, so it rounds
     * up: a radius that is not a whole number of cells still needs the cell it
     * reaches into. */
    config.vision_cells = (config.vision_radius + SPATIAL_CELL_SIZE - 1) / SPATIAL_CELL_SIZE;

    config.frame_rate = notch_integer(config.rate_notch, MIN_FRAME_RATE, MAX_FRAME_RATE);
    update_speed();
}

static int requested_frame_rate = DEFAULT_FRAME_RATE;
static const char *requested_spell;
static int spell_hold = 6;
static int show_intro = 1;
static int show_outro = 1;
static int autopilot;
static int idle_seconds = 60;
static int screensaver;
static int clock_mode;
static int frame_limit;
static char spell_buffer[256];
static int requested_perception = DEFAULT_VISION_RADIUS;
static int requested_seed = -1;
static int requested_preset = -1;

/*
 * A preset is the six notches together, because the interesting settings are
 * combinations rather than single values, and naming one is how a look gets
 * shared. The order is boundary, separation, cohesion, alignment, perception,
 * rate, which is the order the panel shows them in.
 */
typedef struct {
    const char *name;
    const char *help;
    int notch[6];
} preset_t;

static const preset_t PRESETS[] = {
    {"murmuration", "one great restless body, the starling look", {4, 3, 7, 9, 8, 4}},
    {"swarm", "tight, fast and nervous, like insects", {5, 8, 9, 3, 3, 8}},
    {"school", "wide and slow, fish over a reef", {3, 5, 6, 7, 11, 2}},
    {"storm", "loose and violent, thrown about", {9, 10, 2, 2, 6, 9}},
    {"calm", "a gentle drift, almost still", {2, 4, 4, 5, 6, 1}},
};
enum { PRESET_COUNT = sizeof(PRESETS) / sizeof(*PRESETS) };
static const char *PRESET_NAMES[PRESET_COUNT + 1];

static void name_the_presets(void) {
    for (int i = 0; i < PRESET_COUNT; i++) PRESET_NAMES[i] = PRESETS[i].name;
    PRESET_NAMES[PRESET_COUNT] = NULL;
}

static void apply_preset(int which) {
    const preset_t *preset = &PRESETS[which];
    config.boundary_notch = preset->notch[0];
    config.separation_notch = preset->notch[1];
    config.cohesion_notch = preset->notch[2];
    config.alignment_notch = preset->notch[3];
    config.vision_notch = preset->notch[4];
    config.rate_notch = preset->notch[5];
    apply_notches();
}

/* The option table: the parser and the help text both come off this, so adding a
 * switch is one row and never a second place to keep in step. */
static const option_t OPTIONS[] = {
    {'n', "birds", OPTION_INT, &config.birds, 1, MAX_BIRDS, NULL, "COUNT",
     "how many boids to fly (default 800)", "Flock"},
    {'s', "size", OPTION_INT, &config.bird_size, MIN_BIRD_SIZE, MAX_BIRD_SIZE, NULL, "PIXELS",
     "sprite size in pixels (default 15)", "Flock"},
    {'k', "flocks", OPTION_INT, &config.flocks, 1, MAX_FLOCKS, NULL, "COUNT",
     "split into this many flocks that will not merge (default 1)", "Flock"},
    {'c', "color", OPTION_ENUM, &config.palette, 0, 0, PALETTE_NAMES, "RAMP",
     "theme, original, ember, ice, acid, paper (default theme)", "Colour"},
    {0, "color-by", OPTION_ENUM, &config.colour_by, 0, 0, COLOUR_BY_NAMES, "MODE",
     "what picks a bird's shade: heading, density, flock, fixed", "Colour"},
    {0, "preset", OPTION_ENUM, &requested_preset, 0, 0, PRESET_NAMES, "NAME",
     "murmuration, swarm, school, storm, calm", "Flock"},
    {0, "seed", OPTION_INT, &requested_seed, 0, 2147483647, NULL, "N",
     "the same seed gives the same flock", "Flock"},
    {0, "hawks", OPTION_INT, &config.hawks, 0, MAX_HAWKS, NULL, "COUNT",
     "predators hunting the flock (default 0)", "Flock"},
    {0, "spell", OPTION_STRING, &requested_spell, 0, 0, NULL, "TEXT",
     "the flock writes TEXT, then lets go; - reads stdin", "World"},
    {0, "spell-hold", OPTION_INT, &spell_hold, 0, 600, NULL, "SECONDS",
     "how long it holds the writing (default 6, 0 forever)", "World"},
    {0, "clock", OPTION_FLAG, &clock_mode, 0, 0, NULL, NULL,
     "the flock is the time, re-formed on the minute", "World"},
    {0, "intro", OPTION_FLAG, &show_intro, 0, 0, NULL, NULL,
     "open by writing the name, on by default", "Modes"},
    {0, "outro", OPTION_FLAG, &show_outro, 0, 0, NULL, NULL,
     "fly away on q instead of vanishing, on by default", "Modes"},
    {'a', "auto", OPTION_FLAG, &autopilot, 0, 0, NULL, NULL, "the sliders wander by themselves",
     "Modes"},
    {0, "idle", OPTION_INT, &idle_seconds, 0, 3600, NULL, "SECONDS",
     "autopilot after this long untouched (default 60, 0 off)", "Modes"},
    {0, "screensaver", OPTION_FLAG, &screensaver, 0, 0, NULL, NULL,
     "no panel, autopilot, any key or movement quits", "Modes"},
    {0, "frames", OPTION_INT, &frame_limit, 0, 1000000, NULL, "N",
     "quit after N frames, for recording", "Output"},
    {0, "boundary", OPTION_INT, &config.boundary_notch, 0, LEGEND_BAR_CELLS, NULL, "NOTCH",
     "how hard the edges push back, 0 to 12 (default 4)", "Sliders"},
    {0, "separation", OPTION_INT, &config.separation_notch, 0, LEGEND_BAR_CELLS, NULL, "NOTCH",
     "how much a bird keeps its distance (default 4)", "Sliders"},
    {0, "cohesion", OPTION_INT, &config.cohesion_notch, 0, LEGEND_BAR_CELLS, NULL, "NOTCH",
     "how much it seeks the crowd (default 4)", "Sliders"},
    {0, "alignment", OPTION_INT, &config.alignment_notch, 0, LEGEND_BAR_CELLS, NULL, "NOTCH",
     "how much it matches its neighbours (default 4)", "Sliders"},
    {0, "perception", OPTION_INT, &requested_perception, MIN_VISION_RADIUS, MAX_VISION_RADIUS, NULL,
     "PIXELS", "how far it sees, 12 to 60 (default 36)", "Sliders"},
    {'f', "fps", OPTION_INT, &requested_frame_rate, MIN_FRAME_RATE, MAX_FRAME_RATE, NULL, "RATE",
     "frames a second, snapped to a notch (default 60)", "Sliders"},
    {'l', "legend", OPTION_FLAG, &legend_enabled, 0, 0, NULL, NULL,
     "show the parameter panel, on by default", "Display"},
    {'w', "wrap", OPTION_FLAG, &config.wrap, 0, 0, NULL, NULL,
     "leave one edge, arrive from the other", "World"},
    {'e', "trails", OPTION_FLAG, &config.trails, 0, 0, NULL, NULL, "faint tails behind the flock",
     "World"},
    {'m', "mouse", OPTION_ENUM, &config.mouse_mode, 0, 0, MOUSE_NAMES, "MODE",
     "the pointer is: flee, follow, cat, off (default flee)", "Interaction"},
    {0, "mouse-reach", OPTION_INT, &config.mouse_reach, 8, 600, NULL, "PIXELS",
     "how far the pointer reaches (default 120)", "Interaction"},
    {0, "mouse-reporting", OPTION_FLAG, &mouse_enabled, 0, 0, NULL, NULL,
     "ask the terminal for pointer positions, on by default", "Interaction"},
    {0, "force", OPTION_FLAG, &force_graphics, 0, 0, NULL, NULL,
     "draw without asking the terminal whether it can", "General"},
};
enum { OPTION_COUNT = sizeof(OPTIONS) / sizeof(*OPTIONS) };

static const char *const EXAMPLES[] = {
    "cbirds                      a flock, and nothing to read",
    "cbirds -n 2000 -f 120       more of them, faster",
    "cbirds --no-legend          hide the panel, the flock keeps the corner",
    NULL,
};

/* Back to the shipped look, which is what someone reaches for after pressing
 * every key to see what it does. */
static void apply_preset_defaults(void) {
    config.boundary_notch = DEFAULT_NOTCH;
    config.separation_notch = DEFAULT_NOTCH;
    config.cohesion_notch = DEFAULT_NOTCH;
    config.alignment_notch = DEFAULT_NOTCH;
    config.vision_notch = DEFAULT_VISION_NOTCH;
    config.rate_notch = DEFAULT_NOTCH;
    apply_notches();
}

/* CSI < button ; column ; row M or m, one based, as mode 1006 sends it. */
static void read_mouse_report(const char *sequence) {
    int button, column, row;
    if (sequence[0] != '<') return;
    if (sscanf(sequence + 1, "%d;%d;%d", &button, &column, &row) != 3) return;
    if (column < 1 || row < 1) return;
    mouse.x = (column - 0.5) * screen.cell_width;
    mouse.y = (row - 0.5) * screen.cell_height;
    mouse.present = 1;
}

/*
 * Autopilot.
 *
 * A terminal left open on a second monitor becomes the demo for whoever walks
 * past the chair, which is how a thing like this actually spreads. One notch
 * every few seconds, of one slider at a time, so the change is always legible as
 * a change rather than a new program. It backs off for a moment after a keypress,
 * because fighting the user for a slider is worse than not moving it.
 */
static double last_key_at;
static double last_drift_at;

static void drift_a_slider(void) {
    int *notches[] = {&config.boundary_notch, &config.separation_notch, &config.cohesion_notch,
                      &config.alignment_notch, &config.vision_notch};
    int which = (int)(random_unit() * 5) % 5;
    int *notch = notches[which];
    int step = random_unit() < 0.5 ? -1 : 1;

    /* Turned back at the ends rather than stuck against them. */
    if (*notch + step < 0 || *notch + step > LEGEND_BAR_CELLS) step = -step;
    *notch += step;
    apply_notches();
}

/* Autopilot when asked for, and after a while untouched when not. */
static int flying_itself(void) {
    if (autopilot || screensaver) return 1;
    if (idle_seconds <= 0) return 0;
    return clock_state.seconds - last_key_at >= idle_seconds;
}

static void maybe_drift(void) {
    if (!flying_itself()) return;
    if (clock_state.seconds - last_key_at < AUTOPILOT_YIELD) return;
    if (clock_state.seconds - last_drift_at < AUTOPILOT_PERIOD) return;
    last_drift_at = clock_state.seconds;
    drift_a_slider();
}

/* The flock is the time, re-formed on the minute. */
static void maybe_tell_the_time(void) {
    static int minute_shown = -1;
    time_t now = time(NULL);
    struct tm parts;
    char text[16];

    if (!clock_mode) return;
    localtime_r(&now, &parts);
    if (parts.tm_min == minute_shown && spell.writing) return;
    minute_shown = parts.tm_min;
    snprintf(text, sizeof(text), "%02d:%02d", parts.tm_hour, parts.tm_min);
    if (spell_layout(text)) spell.until = -1.0; /* Held until the minute turns. */
}

static int handle_input(void) {
    enum { INPUT_NORMAL, INPUT_ESCAPE, INPUT_SEQUENCE };
    static int input_state = INPUT_NORMAL;
    static char sequence[32];
    static size_t sequence_length;
    char input[INPUT_BUFFER_SIZE];
    ssize_t length = read(STDIN_FILENO, input, sizeof(input));
    if (length > 0) last_key_at = clock_state.seconds;
    for (ssize_t i = 0; i < length; i++) {
        unsigned char key = (unsigned char)input[i];
        int *notch = NULL, step = 0;
        /* A screensaver is dismissed by whatever the passer by pressed. */
        if (screensaver) return 0;
        if (input_state == INPUT_ESCAPE) {
            if (key == '[' || key == 'O') {
                input_state = INPUT_SEQUENCE;
                sequence_length = 0;
            } else if (key != '\033') {
                input_state = INPUT_NORMAL;
            }
            continue;
        }
        if (input_state == INPUT_SEQUENCE) {
            /* A final byte ends the sequence; everything before it is its body.
             * Anything longer than the buffer is not a report we know. */
            if (key >= 0x40 && key <= 0x7e) {
                input_state = INPUT_NORMAL;
                sequence[sequence_length] = '\0';
                if (key == 'M' || key == 'm') read_mouse_report(sequence);
                continue;
            }
            if (sequence_length + 1 < sizeof(sequence)) sequence[sequence_length++] = (char)key;
            continue;
        }
        if (key == '\033') {
            input_state = INPUT_ESCAPE;
            continue;
        }

        switch (key) {
            case 'q':
                return 0;
            case ' ':
                paused = !paused;
                continue;
            case '.':
                step_once = 1; /* One frame of motion, then still again. */
                continue;
            case '0':
                apply_preset_defaults();
                continue;
            case '+':
            case '=':
                if (config.birds < MAX_BIRDS) {
                    config.birds += config.birds / 4 + 1;
                    if (config.birds > MAX_BIRDS) config.birds = MAX_BIRDS;
                    population_changed = 1;
                }
                continue;
            case '-':
                if (config.birds > 1) {
                    config.birds -= config.birds / 5 + 1;
                    if (config.birds < 1) config.birds = 1;
                    population_changed = 1;
                }
                continue;
            case 'h':
                legend_enabled = !legend_enabled;
                measure_legend();
                update_turn_distances();
                continue;
            case 'k':
                if (config.hawks < hawk_sets_built * MAX_HAWKS && config.hawks < MAX_HAWKS) {
                    config.hawks++;
                    place_hawks();
                }
                continue;
            case 'K':
                if (config.hawks > 0) config.hawks--;
                continue;
            case 'w':
                config.wrap = !config.wrap;
                continue;
            case 'e':
                config.trails = !config.trails;
                continue;
            case 'M':
                config.mouse_mode = (config.mouse_mode + 1) % 4;
                continue;
            case 'L':
                config.colour_by = (config.colour_by + 1) % 4;
                continue;
            case '\t':
                requested_preset = (requested_preset + 1) % PRESET_COUNT;
                apply_preset(requested_preset);
                continue;
            case 'B':
                notch = &config.boundary_notch;
                step = 1;
                break;
            case 'b':
                notch = &config.boundary_notch;
                step = -1;
                break;
            case 'S':
                notch = &config.separation_notch;
                step = 1;
                break;
            case 's':
                notch = &config.separation_notch;
                step = -1;
                break;
            case 'C':
                notch = &config.cohesion_notch;
                step = 1;
                break;
            case 'c':
                notch = &config.cohesion_notch;
                step = -1;
                break;
            case 'A':
                notch = &config.alignment_notch;
                step = 1;
                break;
            case 'a':
                notch = &config.alignment_notch;
                step = -1;
                break;
            case 'P':
                notch = &config.vision_notch;
                step = 1;
                break;
            case 'p':
                notch = &config.vision_notch;
                step = -1;
                break;
            case 'R':
                notch = &config.rate_notch;
                step = 1;
                break;
            case 'r':
                notch = &config.rate_notch;
                step = -1;
                break;
            default:
                continue;
        }
        *notch += step;
        if (*notch < 0) *notch = 0;
        if (*notch > LEGEND_BAR_CELLS) *notch = LEGEND_BAR_CELLS;
        apply_notches();
    }
    return 1;
}

static int wait_for_terminal_io(void) {
    struct pollfd descriptors[] = {
        {.fd = STDIN_FILENO, .events = POLLIN},
        {.fd = STDOUT_FILENO, .events = POLLOUT},
    };
    int result;
    do {
        result = poll(descriptors, sizeof(descriptors) / sizeof(*descriptors), -1);
    } while (result < 0 && errno == EINTR);
    return result < 0 ? -1 : 0;
}

#define CBIRDS_VERSION "1.0.0"

static void usage(FILE *out, const char *program) {
    options_usage(out, program, "cbirds \u2014 a flock of birds in your terminal.", EXAMPLES,
                  OPTIONS, OPTION_COUNT);
}

/* "-" means stdin, so that fortune | cbirds --spell - works. Read before raw
 * mode, because after it a pipe and a terminal are told apart differently. */
static const char *read_spell_text(const char *given) {
    if (given == NULL || strcmp(given, "-") != 0) return given;

    size_t at = 0;
    int c;
    while (at + 1 < sizeof(spell_buffer) && (c = getchar()) != EOF) {
        if (c == '\n' || c == '\r' || c == '\t') c = ' ';
        spell_buffer[at++] = (char)c;
    }
    while (at > 0 && spell_buffer[at - 1] == ' ') at--;
    spell_buffer[at] = '\0';

    /* stdin was the pipe, and the keyboard still has to work afterwards: this is
     * what makes `fortune | cbirds --spell -` more than a nice idea. If there is
     * no controlling terminal to go back to, raw mode will say so in a moment. */
    if (!isatty(STDIN_FILENO)) {
        if (freopen("/dev/tty", "r", stdin) == NULL) {
            fprintf(stderr, "cbirds: read the text from stdin but found no terminal to run in\n");
            exit(EXIT_FAILURE);
        }
    }
    return at > 0 ? spell_buffer : NULL;
}

static void read_options(int argc, char **argv) {
    char error[160];
    name_the_palettes();
    name_the_presets();
    options_status_t status =
        options_parse(OPTIONS, OPTION_COUNT, argc, argv, error, sizeof(error));

    if (status == OPTIONS_HELP) {
        usage(stdout, argv[0]);
        exit(EXIT_SUCCESS);
    }
    if (status == OPTIONS_VERSION) {
        printf("cbirds %s\n", CBIRDS_VERSION);
        exit(EXIT_SUCCESS);
    }
    if (status != OPTIONS_OK) {
        fprintf(stderr, "%s: %s\n", argv[0], error);
        fprintf(stderr, "Try '%s --help'.\n", argv[0]);
        exit(EXIT_FAILURE);
    }
    /* A preset is expanded first so that a slider given after it still wins: the
     * table cannot express that order, so the parser's left to right reading is
     * honoured by putting the broad stroke before the fine ones. */
    if (requested_preset >= 0) {
        int boundary = config.boundary_notch, separation = config.separation_notch;
        int cohesion = config.cohesion_notch, alignment = config.alignment_notch;
        apply_preset(requested_preset);
        if (boundary != DEFAULT_NOTCH) config.boundary_notch = boundary;
        if (separation != DEFAULT_NOTCH) config.separation_notch = separation;
        if (cohesion != DEFAULT_NOTCH) config.cohesion_notch = cohesion;
        if (alignment != DEFAULT_NOTCH) config.alignment_notch = alignment;
        if (requested_perception != DEFAULT_VISION_RADIUS)
            config.vision_notch =
                notch_for_integer(requested_perception, MIN_VISION_RADIUS, MAX_VISION_RADIUS);
        if (requested_frame_rate != DEFAULT_FRAME_RATE)
            config.rate_notch =
                notch_for_integer(requested_frame_rate, MIN_FRAME_RATE, MAX_FRAME_RATE);
    } else {
        /* The notch is the state the keys move, so a value off that grid could
         * not be one: snap what was asked for to the nearest. */
        config.rate_notch = notch_for_integer(requested_frame_rate, MIN_FRAME_RATE, MAX_FRAME_RATE);
        config.vision_notch =
            notch_for_integer(requested_perception, MIN_VISION_RADIUS, MAX_VISION_RADIUS);
    }
    apply_notches();
    requested_spell = read_spell_text(requested_spell);
    /* A screensaver has one job and no panel, and anything at all ends it. */
    if (screensaver) {
        legend_enabled = 0;
        show_intro = 0;
        idle_seconds = 0;
    }
}

static long elapsed_microseconds(const struct timespec *start, const struct timespec *end) {
    return (end->tv_sec - start->tv_sec) * 1000000L + (end->tv_nsec - start->tv_nsec) / 1000L;
}

int main(int argc, char **argv) {
    image_frame_t frames[ROTATION_FRAMES * (MAX_PALETTE_SHADES + 1)] = {0};
    kitty_graphics_t graphics;
    spatial_grid_t grid;
    struct timespec frame_start, frame_end;
    read_options(argc, argv);
    install_signal_handlers();
    atexit(restore_terminal);

    /* The terminal is asked its questions before anything is built for it: can
     * you draw this at all, and what colours do you use? The sprites are then
     * built once, in the answers. */
    if (enter_terminal() < 0) {
        perror("Can't enable raw mode");
        exit(EXIT_FAILURE);
    }
    if (!force_graphics && !terminal_speaks_graphics()) {
        restore_terminal();
        fprintf(stderr,
                "cbirds draws with the Kitty graphics protocol, and this terminal did not\n"
                "answer for it. Kitty, WezTerm, Ghostty and recent Konsole all do.\n"
                "Run it under one of those, or pass --force to try anyway.\n");
        exit(EXIT_FAILURE);
    }
    if (palette_follows_the_theme() && !learn_the_theme()) config.palette = FALLBACK_PALETTE;

    spatial_grid_status_t grid_status = spatial_grid_init(&grid, SPATIAL_CELL_SIZE);
    if (grid_status != SPATIAL_GRID_OK) {
        fprintf(stderr, "Cannot initialize spatial grid: %s\n",
                spatial_grid_status_string(grid_status));
        exit(EXIT_FAILURE);
    }
    /* A named seed makes a run repeatable, which is what lets a look be shared
     * and a bug report be reproduced. */
    srand(requested_seed >= 0 ? (unsigned)requested_seed : (unsigned)time(NULL));
    update_screen_dimensions();
    grid_status = spatial_grid_prepare(&grid, screen.width, screen.height, config.birds);
    if (grid_status != SPATIAL_GRID_OK) {
        fprintf(stderr, "Cannot prepare spatial grid: %s\n",
                spatial_grid_status_string(grid_status));
        exit(EXIT_FAILURE);
    }
    build_rotation_frames(frames, palette_shades(), config.bird_size);
    /* A hawk has to read as a bigger bird at a glance, so it gets its own set at
     * twice the size, one shade, uploaded straight after the flock's. */
    build_rotation_frames(
        frames + palette_shades() * ROTATION_FRAMES, 1,
        config.bird_size * 2 > MAX_BIRD_SIZE ? MAX_BIRD_SIZE : config.bird_size * 2);
    hawk_sets_built = 1;

    bird_t *birds = calloc((size_t)config.birds, sizeof(*birds));
    bird_t *snapshot = malloc(sizeof(*snapshot) * (size_t)config.birds);
    if (!birds || !snapshot) {
        perror("Out of memory");
        exit(EXIT_FAILURE);
    }
    kitty_graphics_status_t graphics_status = kitty_graphics_init(&graphics, STDOUT_FILENO);
    if (graphics_status != KITTY_GRAPHICS_OK) {
        fprintf(stderr, "Cannot initialize Kitty graphics: %s\n",
                kitty_graphics_status_string(graphics_status));
        exit(EXIT_FAILURE);
    }

    enter_alt_screen();
    write_all("\x1b[J", sizeof("\x1b[J") - 1);
    update_screen_dimensions();
    initialize_birds(birds);
    place_hawks();
    /* An explicit text wins over the intro: someone who asked for a word does
     * not want to be told the program's name first. */
    if (requested_spell != NULL) {
        if (spell_layout(requested_spell)) spell.until = spell_hold > 0 ? (double)spell_hold : -1.0;
    } else if (show_intro && spell_layout("CBIRDS")) {
        spell.until = INTRO_SECONDS;
    }
    int sprite_sets = palette_shades() + 1;
    graphics_status = upload_rotation_frames(&graphics, frames, sprite_sets);
    free_rotation_frames(frames, sprite_sets);
    if (graphics_status != KITTY_GRAPHICS_OK) {
        fprintf(stderr, "Cannot upload Kitty graphics: %s\n",
                kitty_graphics_status_string(graphics_status));
        exit(EXIT_FAILURE);
    }

    struct timespec started;
    clock_gettime(CLOCK_MONOTONIC, &started);
    int live_birds = config.birds;
    int running = 1;
    int leaving = 0; /* Frames left of the flight out. */
    while (running) {
        if (!handle_input() && !leaving) {
            /* Asked to quit: fly off the top first, so the last thing seen is
             * the flock leaving rather than the screen blinking out. */
            if (!show_outro) break;
            leaving = OUTRO_FRAMES;
            spell_clear();
        }
        if (leaving && --leaving == 0) break;

        clock_gettime(CLOCK_MONOTONIC, &frame_start);
        clock_state.frame++;
        clock_state.seconds = (double)(frame_start.tv_sec - started.tv_sec) +
                              (double)(frame_start.tv_nsec - started.tv_nsec) / 1e9;
        /* Writing lets go when its hold is up, and the flock takes over again
         * from wherever the letters left it, which is the nicest part to watch. */
        if (spell.writing && spell.until >= 0 && clock_state.seconds >= spell.until) spell_clear();
        maybe_tell_the_time();
        maybe_drift();
        update_screen_dimensions();
        grid_status = spatial_grid_prepare(&grid, screen.width, screen.height, config.birds);
        if (grid_status != SPATIAL_GRID_OK) {
            fprintf(stderr, "Cannot resize spatial grid: %s\n",
                    spatial_grid_status_string(grid_status));
            exit(EXIT_FAILURE);
        }
        if (population_changed) {
            /* Grown or shrunk by a keypress. The birds already flying carry on;
             * only the new ones need placing, and the grid needs room for them. */
            population_changed = 0;
            bird_t *grown = realloc(birds, sizeof(*birds) * (size_t)config.birds);
            bird_t *grown_snapshot = realloc(snapshot, sizeof(*snapshot) * (size_t)config.birds);
            if (grown != NULL) birds = grown;
            if (grown_snapshot != NULL) snapshot = grown_snapshot;
            if (grown == NULL || grown_snapshot == NULL) {
                config.birds = live_birds; /* Keep what we have rather than lose it. */
            } else {
                for (int i = live_birds; i < config.birds; i++) place_one_bird(&birds[i], i);
                live_birds = config.birds;
            }
            grid_status = spatial_grid_prepare(&grid, screen.width, screen.height, config.birds);
            if (grid_status != SPATIAL_GRID_OK) {
                fprintf(stderr, "Cannot resize spatial grid: %s\n",
                        spatial_grid_status_string(grid_status));
                exit(EXIT_FAILURE);
            }
        }
        memcpy(snapshot, birds, sizeof(*birds) * (size_t)config.birds);
        grid_status = spatial_grid_build(&grid, config.birds, read_bird_position, snapshot);
        if (grid_status != SPATIAL_GRID_OK) {
            fprintf(stderr, "Cannot build spatial grid: %s\n",
                    spatial_grid_status_string(grid_status));
            exit(EXIT_FAILURE);
        }
        if (leaving) {
            /* Straight up, every one of them, and nothing else steering. */
            for (int i = 0; i < config.birds; i++) {
                birds[i].direction = 3 * M_PI / 2;
                birds[i].y -= config.speed;
                birds[i].frame = direction_frame(birds[i].direction);
            }
        }
        if (!paused || step_once) hunt(snapshot);
        graphics_status = leaving ? queue_render_frame(&graphics, birds)
                                  : render_frame(&graphics, birds, snapshot, &grid);
        if (graphics_status != KITTY_GRAPHICS_OK) {
            fprintf(stderr, "Cannot render Kitty graphics: %s\n",
                    kitty_graphics_status_string(graphics_status));
            exit(EXIT_FAILURE);
        }
        while (running && graphics.length > 0) {
            graphics_status = kitty_graphics_flush_nonblocking(&graphics);
            if (graphics_status == KITTY_GRAPHICS_AGAIN) {
                if (wait_for_terminal_io() < 0) {
                    perror("Cannot wait for terminal output");
                    exit(EXIT_FAILURE);
                }
                running = handle_input();
                continue;
            }
            if (graphics_status != KITTY_GRAPHICS_OK) {
                fprintf(stderr, "Cannot flush Kitty graphics: %s\n",
                        kitty_graphics_status_string(graphics_status));
                exit(EXIT_FAILURE);
            }
        }
        if (!running) break;
        if (frame_limit > 0 && clock_state.frame >= frame_limit) break;
        clock_gettime(CLOCK_MONOTONIC, &frame_end);
        long remaining =
            1000000L / config.frame_rate - elapsed_microseconds(&frame_start, &frame_end);
        if (remaining > 0) {
            struct timespec delay = {remaining / 1000000L, (remaining % 1000000L) * 1000L};
            nanosleep(&delay, NULL);
        }
    }
    spatial_grid_destroy(&grid);
    kitty_graphics_destroy(&graphics);
    free(snapshot);
    free(birds);
    return EXIT_SUCCESS;
}
