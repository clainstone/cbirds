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

#include "cells.h"
#include "font.h"
#include "gif.h"
#include "kitty_graphics.h"
#include "options.h"
#include "png.h"
#include "spatial_grid.h"
#include "sprite_png.h"

enum {
    ROTATION_FRAMES = 90,
    MAX_PALETTE_SHADES = 8,
    /* Three is as many as a five step ramp can tell apart: at four the two nearest
     * shades are closer to each other than two birds are wide. */
    MAX_FLOCKS = 3,
    MOUSE_FLEE = 0,
    MOUSE_FOLLOW,
    MOUSE_OFF,
    MOUSE_MODE_COUNT,
    DEFAULT_MOUSE_REACH = 120,
    DEFAULT_VISION_NOTCH = 6,
    DEFAULT_TURNING_NOTCH = 8,
    /* Only every sixteenth bird leaves one, because a tail behind all of them is
     * three times the bandwidth for a picture that reads as mud. Fifty comets in
     * a flock of eight hundred is what says "moving" in a still frame. */
    TRAIL_EVERY = 16,
    TRAIL_LENGTH = 4,
    INTRO_SECONDS = 3,
    /* Four is as many as the sky holds: past that they overlap each other, crowd
     * the flock into the edges and stop reading as separate animals. */
    MAX_HAWKS = 4,
    HAWK_REACH = 150,
    HAWK_COMMITMENT = 40, /* Frames it stays after one bird before reconsidering. */
    HAWK_GIVE_UP = 200,   /* Pixels past which a reconsidered chase is dropped. */
    HAWK_STALK = 340,     /* And how far off it looks for the next bird. */
    HAWK_PASS = 14,       /* Frames it flies straight after a pass. */
    HAWK_SPACING = 150,   /* Pixels two hawks try to keep between them. */
    /* A GIF's delay is in hundredths of a second, so the rates it can express are
     * 100/1, 100/2, 100/3 and so on. Viewers also clamp anything under two
     * hundredths up to a tenth of a second, which puts the real ceiling at fifty:
     * sixty is simply not a rate a GIF has. */
    MAX_RECORD_FPS = 50,
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
    /* The panel is 38 by 10 cells and the flock may not enter it. At the smallest
     * terminal it used to appear in it covered half the screen, and half the flock
     * was squeezed off the edges of what was left: it needs to be a fifth of the
     * room, not a half, so it waits for a window it fits inside. */
    LEGEND_MIN_COLS = 76,
    LEGEND_MIN_ROWS = 22,
    LEGEND_LINE_MAX = 128,
    SPAWN_ATTEMPTS = 32
};

/* The same scale as the original bottom edge turn: large enough that it settles
 * the direction on its own, whatever the flocking terms are doing. */
static const double LEGEND_PUSH = 100000.0;
/* How hard an edge pushes once a bird reaches the screen's own edge, against a
 * flocking sum of one to four. Twelve turns them well inside the band and still
 * leaves them the whole screen to use; much more and the flock plays in a box. */
static const double EDGE_FIRM = 12.0;
/* And how much harder each further band-width of straying costs. */
static const double ESCAPE_PENALTY = 8.0;
/* Heavy enough to bend a flock that is busy flocking, light enough that it bends
 * rather than shatters. */
static const double MOUSE_WEIGHT = 4.0;
/* A hawk frightens a bird; it does not get to throw it off the screen. At six
 * the flee beat the edge even at the screen's own edge, and four birds in fifty
 * were outside the frame at any moment with four hawks up. */
static const double HAWK_WEIGHT = 3.0;
/* A shade slower than the flock when it is only cruising, so it has to dive to
 * catch anything: a hawk that outruns the birds at rest never has to commit, and
 * the moment it commits is the moment worth watching. */
static const double HAWK_SPEED = 0.90;
/* Sharper than a bird's bank, because a raptor is more agile, but a limit all the
 * same: without one it turned forty degrees a frame and read as a glitch. A fifth
 * of a radian looked calm and never caught anything: the turning circle was wider
 * than the flock, so every miss became a long trip to a wall and back. Half a
 * radian still only landed one chase in eight; at a whole one it lands half of
 * them, and the wall is reached a third as often. */
static const double HAWK_TURN = 1.0;
/* That is per frame at sixty a second. At any other rate it has to be rescaled or
 * the hawk is a different animal: half a radian a frame is 1718 degrees a second
 * at sixty and 3437 at a hundred and twenty. */
#define HAWK_TURN_PER_FRAME() (HAWK_TURN * DEFAULT_FRAME_RATE / (double)config.frame_rate)
static const double HAWK_LEAD = 3.0; /* Frames ahead of the prey it aims. */
/* Inside the dive it accelerates and stops leading: a bird that flees is only a
 * tenth slower than a cruising hawk, so without this the chase never closes and
 * there is no moment to watch. */
static const double HAWK_DIVE = 90.0;
static const double HAWK_DIVE_SPEED = 1.45;
/* How much of the flee is sideways rather than straight away. Purely radial and
 * the flock bursts like a firework and is gone; with a curl to it the birds peel
 * around the hawk and close up behind, which is the shape people watch for. */
static const double HAWK_SWIRL = 0.9;
/* What one hawk's company is worth to another, against a chase of one. */
static const double HAWK_APART = 1.2;
/* And what a wall is worth: more than the chase, or it follows a bird into the
 * edge and bounces off it. */
static const double HAWK_WALL = 2.5;
/* Flocking is local — a bird sees sixty pixels at most — so nothing in the three
 * rules keeps a flock together as a body across a whole screen, and three flocks
 * left to themselves spread until they are one cloud in three colours. The leash
 * is the missing long range term: nothing at all within a flock's own width of
 * its centre, and a pull that grows outside it. */
static const int FLOCK_LEASH = 150;
static const double LEASH_WEIGHT = 2.5;
/* A breeze the whole flock leans into. Enough to shape it, not enough to carry
 * it off: at the top notch it is about a third of the alignment weight. */
static const double WIND_WEIGHT = 0.5;

/* Needed in the config initializer, so macros rather than constants. */
#define DEFAULT_SEPARATION_W 0.005
#define DEFAULT_ALIGNMENT_W 1.5
/* Not a slider any more. Dragging it from end to end moved the flock's own
 * measure of itself — neighbours within sixty pixels — by four and a half, where
 * simply removing the term moves it by eleven: the force does something, the
 * knob did not, and it was costing a row of a six row panel and two of the
 * fourteen keys. */
#define COHESION_W 0.01
#define DEFAULT_BOUNDARY_W 0.2

static const double BOUNDARY_MIN = 0.01;
static const double SEPARATION_MIN = 0.001;
static const double ALIGNMENT_MIN = 0.1;

/* Each ceiling is placed so that the default lands exactly on the fourth of
 * twelve notches, a third along the bar. There are no step constants any more:
 * a step is one notch, which is a twelfth of the travel by construction, and
 * that is what makes a keypress worth exactly one cell of bar. */
#define DEFAULT_NOTCH 4
#define NOTCH_CEILING(minimum, default_value) ((minimum) + 3 * ((default_value) - (minimum)))
static const double BOUNDARY_MAX = NOTCH_CEILING(0.01, DEFAULT_BOUNDARY_W);
static const double SEPARATION_MAX = NOTCH_CEILING(0.001, DEFAULT_SEPARATION_W);
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
    int birds, frame_rate, bird_size, palette, flocks;
    int mouse_mode, mouse_reach;
    int trails, hawks, shape;
    int turning_notch;
    double speed;
    int vision_cells, vision_radius, vision_radius_squared;
    double separation, alignment, boundary;
    /* Notch positions, zero to LEGEND_BAR_CELLS. These are the state the keys
     * move; every value above is derived from them, which is what makes one
     * keypress exactly one notch of bar rather than nearly one. */
    int boundary_notch, separation_notch, alignment_notch;
    int vision_notch, rate_notch;
} config_t;

static config_t config = {
    .birds = 800,
    .frame_rate = DEFAULT_FRAME_RATE,
    .speed = DEFAULT_SPEED,
    .bird_size = DEFAULT_BIRD_SIZE,
    .palette = 0,
    .flocks = 1,
    .mouse_mode = MOUSE_FLEE,
    .mouse_reach = DEFAULT_MOUSE_REACH,
    .turning_notch = DEFAULT_TURNING_NOTCH,
    .vision_cells = DEFAULT_VISION_RADIUS / SPATIAL_CELL_SIZE,
    .vision_radius = DEFAULT_VISION_RADIUS,
    .vision_radius_squared = DEFAULT_VISION_RADIUS * DEFAULT_VISION_RADIUS,
    .separation = DEFAULT_SEPARATION_W,
    .alignment = DEFAULT_ALIGNMENT_W,
    .boundary = DEFAULT_BOUNDARY_W,
    .boundary_notch = DEFAULT_NOTCH,
    .separation_notch = DEFAULT_NOTCH,
    .alignment_notch = DEFAULT_NOTCH,
    /* Twelve to sixty pixels in steps of four: thirty six is the sixth notch. */
    .vision_notch = DEFAULT_VISION_NOTCH,
    .rate_notch = DEFAULT_NOTCH,
};
static screen_t screen;
static int legend_enabled = 1; /* Cleared by --no-legend, never at runtime. */
static int mouse_enabled = 1;  /* Cleared by --no-mouse, never at runtime. */
/*
 * How the frame reaches the screen.
 *
 * Kitty's graphics protocol draws real sprites, and is asked for first. A
 * terminal that does not answer for it gets the same flock as text: the frame
 * is rendered to pixels exactly as it is for a recording, and the pixels are
 * read back as braille — eight dots a cell, the finest thing text can do — or
 * as half blocks, two a cell, each cell in the colour of the bird in it. So
 * every terminal that can show colour gets a flock; the only question is how
 * fine.
 */
typedef enum {
    RENDER_AUTO = 0, /* Kitty if the terminal answers for it, braille if not. */
    RENDER_KITTY,
    RENDER_BRAILLE,
    RENDER_BLOCKS,
} render_mode_t;
static const char *const RENDER_NAMES[] = {"auto", "kitty", "braille", "blocks", NULL};
static int render_mode = RENDER_AUTO;

static int drawing_with_text(void) {
    return render_mode == RENDER_BRAILLE || render_mode == RENDER_BLOCKS;
}

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
/* Defined with the other notch arithmetic; needed here because the step a bird
 * takes is capped by the size of the screen it is taking it on. */
static void update_speed(void);

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
    /* Before the distances, because the panel's turn zone is the panel grown by
     * one frame of travel and the frame of travel depends on the screen. */
    update_speed();
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

/* The relative luminance the WCAG contrast ratio is built on. Used to keep the
 * flock off the terminal's own background, which is the one colour a bird must
 * never be. */
static double luminance_of(const uint8_t rgb[3]) {
    double channel[3];
    for (int c = 0; c < 3; c++) {
        double v = rgb[c] / 255.0;
        channel[c] = v <= 0.04045 ? v / 12.92 : pow((v + 0.055) / 1.055, 2.4);
    }
    return 0.2126 * channel[0] + 0.7152 * channel[1] + 0.0722 * channel[2];
}

static double contrast_between(const uint8_t a[3], const uint8_t b[3]) {
    double high = luminance_of(a), low = luminance_of(b);
    if (high < low) {
        double swap = high;
        high = low;
        low = swap;
    }
    return (high + 0.05) / (low + 0.05);
}

/* Five steps from the accent towards the background, stopping well short of it.
 * Over four steps the last shade *was* the background, byte for byte: a fifth of
 * the flock was painted in the colour of the sky and simply did not exist, and
 * with three flocks up a whole flock went missing. Over seven, the far end still
 * reads as distance and the worst case across the common colour schemes keeps a
 * contrast of 1.8 against the ground instead of 1.0. */
static void ramp_between(const uint8_t from[3], const uint8_t to[3]) {
    for (int i = 0; i < 5; i++)
        for (int c = 0; c < 3; c++)
            theme_tints[i][c] = (uint8_t)(from[c] + (to[c] - from[c]) * i / 7);
}

static int learn_the_theme(void) {
    uint8_t accent[3] = {0, 0, 0}, background[3] = {0, 0, 0};
    double best = -1;

    /* The background first, because the accent is chosen against it. */
    if (!ask_colour("\033]11;?\033\\", background)) {
        /* No background: fade towards black, which is the common case. */
        background[0] = background[1] = background[2] = 0;
    }
    /* Entries one to six are the terminal's own reds through cyans, which is
     * where a colour scheme keeps its character. The most saturated of them is
     * not always the one to take: on stock xterm that is pure blue on black,
     * which is both the dimmest colour on the screen and the ugliest. Saturation
     * times contrast picks the colour with character that can also be seen. */
    for (int entry = 1; entry <= 6; entry++) {
        char request[32];
        uint8_t rgb[3];
        snprintf(request, sizeof(request), "\033]4;%d;?\033\\", entry);
        if (!ask_colour(request, rgb)) continue;
        double score = saturation_of(rgb) * contrast_between(rgb, background);
        if (score > best) {
            best = score;
            memcpy(accent, rgb, sizeof(accent));
        }
    }
    if (best < 0) return 0;
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
static const uint8_t MATRIX_TINTS[][3] = {
    {198, 255, 198}, {120, 246, 120}, {54, 210, 70}, {26, 150, 48}, {12, 92, 30},
};
/* What the embedded sprite is actually painted, for the palettes that leave it
 * alone: a hawk still has to stand off that. */
static const uint8_t SPRITE_OWN_COLOUR[3] = {237, 28, 36};

static const palette_t PALETTES[] = {
    {"theme", "the terminal's own colours, asked for at startup", 5,
     (const uint8_t (*)[3])theme_tints, PNG_TINT_REPLACE},
    {"ember", "embers, pale gold to deep red", 5, EMBER_TINTS, PNG_TINT_REPLACE},
    {"ice", "ice, white through to deep blue", 5, ICE_TINTS, PNG_TINT_REPLACE},
    {"acid", "acid, lime through to teal", 5, ACID_TINTS, PNG_TINT_REPLACE},
    {"matrix", "the green of the film it is named after", 5, MATRIX_TINTS, PNG_TINT_REPLACE},
};
enum { PALETTE_COUNT = sizeof(PALETTES) / sizeof(*PALETTES) };

static const char *PALETTE_NAMES[PALETTE_COUNT + 1];
static const char *const MOUSE_NAMES[] = {"flee", "follow", "off", NULL};

/* A PNG of somebody's own, if they gave one: it keeps its own colours, which is
 * the whole reason for drawing one. Every palette replaces the colour it is
 * given, so the flock used to come out flat ember whatever the artwork was. */
static const char *sprite_path;

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
    if (sprite_path != NULL) return 1; /* One set, untinted, as it was drawn. */
    return palette()->shades;
}

/* A hawk is not one of the flock's shades. What has to read instantly is that
 * this one is different.
 *
 * It used to be a dark silhouette, which is what a hawk looks like against the
 * sky and what nothing looks like against a black terminal: barely a twentieth of
 * a stop above the background, invisible in every recording. Scarlet fixed that
 * everywhere except the warm ramps, where scarlet is just another ember — against
 * ember the contrast was 1.16, which is no contrast at all. So there are two
 * colours and the palette picks: whichever of them stands furthest from the
 * nearest thing the flock is wearing. */
static const uint8_t HAWK_COLOURS[][3] = {
    {255, 60, 72},   /* Scarlet, for every cold or grey ramp. */
    {255, 246, 210}, /* A hot near-white, against a single dark colour. */
    {96, 226, 255},  /* And an electric cyan, for the ramps that are already fire. */
};
enum { HAWK_COLOUR_COUNT = sizeof(HAWK_COLOURS) / sizeof(*HAWK_COLOURS) };

/* How far apart two colours look, which is not how far apart their brightnesses
 * are: scarlet and pale ice blue are a stone's throw apart by luminance and could
 * not be more different to look at. The usual weighted RGB distance, good enough
 * for choosing between two candidates. */
static double colour_distance(const uint8_t a[3], const uint8_t b[3]) {
    double mean_red = (a[0] + b[0]) / 2.0;
    double dr = a[0] - b[0], dg = a[1] - b[1], db = a[2] - b[2];
    return sqrt((2 + mean_red / 256) * dr * dr + 4 * dg * dg +
                (2 + (255 - mean_red) / 256) * db * db);
}

static const uint8_t *hawk_colour(void) {
    const palette_t *chosen = palette();
    /* Against artwork nobody has looked at, scarlet and no cleverness. */
    if (sprite_path != NULL) return HAWK_COLOURS[0];
    int best = 0;
    double best_gap = -1;
    for (int candidate = 0; candidate < HAWK_COLOUR_COUNT; candidate++) {
        double gap = 1e9;
        for (int shade = 0; shade < chosen->shades; shade++) {
            const uint8_t *tint = chosen->tints != NULL ? chosen->tints[shade] : SPRITE_OWN_COLOUR;
            double against = colour_distance(HAWK_COLOURS[candidate], tint);
            if (against < gap) gap = against;
        }
        if (gap > best_gap) {
            best_gap = gap;
            best = candidate;
        }
    }
    return HAWK_COLOURS[best];
}

static void hawk_tint(png_image_t *image) {
    const uint8_t *colour = hawk_colour();
    png_tint(image, colour[0], colour[1], colour[2], PNG_TINT_REPLACE);
}

/* Shade zero of a tinted palette is still a tint: the list is the whole ramp. */
static void palette_tint(png_image_t *image, int shade) {
    const palette_t *chosen = palette();
    if (sprite_path != NULL) return; /* Somebody's own artwork, left alone. */
    if (chosen->tints == NULL) return;
    if (shade < 0) shade = 0;
    if (shade >= chosen->shades) shade = chosen->shades - 1;
    png_tint(image, chosen->tints[shade][0], chosen->tints[shade][1], chosen->tints[shade][2],
             chosen->mode);
}

/* Twice a bird, clamped: a hawk has to read as the bigger thing at a glance. */
static int hawk_sprite_size(void) {
    int size = config.bird_size * 2;
    return size > MAX_BIRD_SIZE ? MAX_BIRD_SIZE : size;
}

/* A bird is drawn from its top left corner; a hawk from its middle, so that the
 * point the flock flees and the silhouette on the screen are the same place. One
 * function, because the live frame and the recorder both have to agree. */
static int hawk_draw_offset(void) {
    return hawk_sprite_size() / 2;
}

/*
 * The bird, or something else.
 *
 * A shape is a handful of triangles in a unit square, pointing along +x because
 * that is what frame zero means, rasterised with four samples a pixel so the
 * edges survive the rotation and the shrink. Drawing them rather than embedding
 * them means five more sprites for no more bytes, and the only external file the
 * program will ever read is one the user chose.
 */
typedef struct {
    double x[3], y[3];
} triangle_t;

typedef struct {
    const char *name;
    int count;
    const triangle_t *triangles;
    double roundness; /* Radius of a filled circle to union in, zero for none. */
} shape_t;

static const triangle_t ARROW_TRIANGLES[] = {
    {{0.05, 0.95, 0.05}, {0.15, 0.50, 0.85}},
    {{0.05, 0.45, 0.05}, {0.35, 0.50, 0.65}},
};
static const triangle_t PLANE_TRIANGLES[] = {
    {{0.10, 0.95, 0.10}, {0.44, 0.50, 0.56}}, /* Fuselage. */
    {{0.30, 0.55, 0.20}, {0.48, 0.50, 0.08}}, /* Upper wing. */
    {{0.30, 0.55, 0.20}, {0.52, 0.50, 0.92}}, /* Lower wing. */
    {{0.08, 0.22, 0.08}, {0.30, 0.50, 0.70}}, /* Tail. */
};

static const shape_t SHAPES[] = {
    {"bird", 0, NULL, 0.0}, /* The embedded drawing, not a shape at all. */
    {"arrow", 2, ARROW_TRIANGLES, 0.0},
    {"plane", 4, PLANE_TRIANGLES, 0.0},
    {"dot", 0, NULL, 0.40},
};
enum { SHAPE_COUNT = sizeof(SHAPES) / sizeof(*SHAPES) };
static const char *SHAPE_NAMES[SHAPE_COUNT + 1];

static void name_the_shapes(void) {
    for (int i = 0; i < SHAPE_COUNT; i++) SHAPE_NAMES[i] = SHAPES[i].name;
    SHAPE_NAMES[SHAPE_COUNT] = NULL;
}

static int inside_triangle(const triangle_t *t, double x, double y) {
    double d1 = (x - t->x[1]) * (t->y[0] - t->y[1]) - (t->x[0] - t->x[1]) * (y - t->y[1]);
    double d2 = (x - t->x[2]) * (t->y[1] - t->y[2]) - (t->x[1] - t->x[2]) * (y - t->y[2]);
    double d3 = (x - t->x[0]) * (t->y[2] - t->y[0]) - (t->x[2] - t->x[0]) * (y - t->y[0]);
    return (d1 >= 0 && d2 >= 0 && d3 >= 0) || (d1 <= 0 && d2 <= 0 && d3 <= 0);
}

static int inside_shape(const shape_t *shape, double x, double y) {
    if (shape->roundness > 0) {
        double dx = x - 0.5, dy = y - 0.5;
        if (dx * dx + dy * dy <= shape->roundness * shape->roundness) return 1;
    }
    for (int i = 0; i < shape->count; i++)
        if (inside_triangle(&shape->triangles[i], x, y)) return 1;
    return 0;
}

static png_status_t draw_shape(int which, int size, png_image_t *out) {
    const shape_t *shape = &SHAPES[which];
    png_status_t status = png_image_alloc(out, size, size);
    if (status != PNG_OK) return status;

    for (int py = 0; py < size; py++) {
        for (int px = 0; px < size; px++) {
            int hits = 0;
            /* Four samples a pixel: enough of an edge to survive a rotation and a
             * shrink, and cheap enough to do once at startup. */
            for (int sy = 0; sy < 2; sy++)
                for (int sx = 0; sx < 2; sx++)
                    hits += inside_shape(shape, (px + 0.25 + sx * 0.5) / size,
                                         (py + 0.25 + sy * 0.5) / size);
            uint8_t *pixel = out->pixels + ((size_t)py * (size_t)size + (size_t)px) * 4;
            pixel[0] = pixel[1] = pixel[2] = 255;
            pixel[3] = (uint8_t)(hits * 255 / 4);
        }
    }
    return PNG_OK;
}

static const char *program_name = "cbirds"; /* As invoked, for every message. */

/* Whichever the user asked for: a PNG of their own, one of the drawn shapes, or
 * the drawing compiled into the binary. */
static png_status_t load_sprite(png_image_t *out) {
    if (sprite_path != NULL) {
        FILE *file = fopen(sprite_path, "rb");
        if (file == NULL) {
            fprintf(stderr, "%s: cannot open %s\n", program_name, sprite_path);
            exit(EXIT_FAILURE);
        }
        static uint8_t buffer[1 << 22]; /* Four megabytes of PNG is a generous bird. */
        size_t length = fread(buffer, 1, sizeof(buffer), file);
        fclose(file);
        return png_decode(buffer, length, out);
    }
    if (config.shape != 0) return draw_shape(config.shape, SPRITE_WORK_MAX, out);
    return png_decode(sprite_png, sprite_png_len, out);
}

/* Kitty has no per placement tint, so a colour is a second set of images. The
 * rotation is the expensive half and it does not depend on the colour, so each
 * angle is rotated once and then tinted and encoded per shade: the second shade
 * costs a few hundred microseconds, not another fifty milliseconds. */
static void build_rotation_frames(image_frame_t *frames, int shades, int size, int hawk) {
    png_image_t source = {0, 0, NULL}, canvas = {0, 0, NULL};
    png_status_t status = load_sprite(&source);
    if (status != PNG_OK) {
        fprintf(stderr, "cbirds: cannot read the sprite: %s\n", png_status_string(status));
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
                if (hawk)
                    hawk_tint(&tinted);
                else
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
 * Wind.
 *
 * A slow wandering breeze, which is what keeps a flock off centre and stops a
 * long run looking like it has settled. The direction takes a small random step
 * each frame, so it drifts rather than jumping, and the strength is a notch like
 * everything else.
 */
/* The rain falls, and nothing else has a wind. As a slider it did not do the one
 * thing it was for — it was meant to keep a flock off centre, and at full strength
 * the flock sat 34% off centre against 33% with no wind at all — while tripling
 * the share of birds outside the frame. What is left of it is the downdraught
 * that makes --matrix rain. */
static int the_rain_is_falling;

static vector_t wind_vector(void) {
    vector_t force = {0, 0};
    if (!the_rain_is_falling) return force;
    force.y = 1; /* Straight down, and it stays there. */
    return force;
}

/*
 * Turning inertia.
 *
 * A bird that can turn any amount in one frame moves like a particle: the flock
 * comes out as a blob that changes shape instantly. Cap the turn and it banks
 * instead, which gives the flock curved fronts, a leading edge, and the look of
 * something with mass. This is the single change that makes it read as birds
 * rather than as points, which is why the default is eight of twelve rather than
 * the twelve it used to effectively be.
 *
 * Writing is exempt. It already overrules the flocking rules, and a bird that
 * cannot turn sharply cannot land on a letter: it would circle one instead, and
 * the crispness of the letters is the whole point of them.
 */
static double turn_towards(double from, double to, double most) {
    double delta = atan2(sin(to - from), cos(to - from));
    if (delta > most) delta = most;
    if (delta < -most) delta = -most;
    double turned = from + delta;
    if (turned < 0) turned += 2 * M_PI;
    if (turned >= 2 * M_PI) turned -= 2 * M_PI;
    return turned;
}

/* Twelve is instant, zero is as near a straight line as a bird gets — near, and
 * not exactly, because a bird that cannot turn at all cannot be turned back by
 * the edges either: at a true zero the whole flock flew out of the frame within a
 * second and the screen stayed black until something else was pressed.
 *
 * Scaled by the frame rate, like the speed is, so that both are really per second
 * and a bird's turning circle is the same number of pixels whatever the rate. Left
 * per frame, a bird at thirty covered twice the ground per frame and was allowed
 * the same turn for it: the flock could not come round inside the edge band any
 * more, and one bird in eight was off the screen at --fps 30 against one in
 * thirty at sixty. */
/* What the notch itself means, before the frame rate has its say: this is what
 * the panel shows, because the panel is about notches. */
static double turning_notch_radians(void) {
    if (config.turning_notch >= LEGEND_BAR_CELLS) return 2 * M_PI;
    /* The whole bar is a setting somebody might want: from a sixth of a turn a
     * frame, which is a long lazy bank that swings wide, up to a quarter.
     * Measured from zero instead, the bottom third of the travel put between a
     * tenth and all of the flock outside the screen — at the very bottom no bird
     * could turn at all, so the edges could not turn them back either and the
     * whole flock left inside a second — and the bar was half poison, with the
     * README recommending a value from the poisoned half. */
    return M_PI / 6 + (M_PI / 2 - M_PI / 6) * config.turning_notch / LEGEND_BAR_CELLS;
}

static double turn_limit(void) {
    if (config.turning_notch >= LEGEND_BAR_CELLS) return 2 * M_PI;
    double scaled = turning_notch_radians() * DEFAULT_FRAME_RATE / (double)config.frame_rate;
    return scaled > 2 * M_PI ? 2 * M_PI : scaled;
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
    int prey;       /* Index of the bird it is chasing, negative for none. */
    int commitment; /* Frames left before it is allowed to change its mind. */
    int passing;    /* Frames left of a straight run out of the flock. */
} hawk_t;

static hawk_t hawks[MAX_HAWKS];
/* The hawks' images always go up, so k can summon one at any time. */
static int hawk_sets_built;

/* One hawk, so that summoning one with k leaves the hawks already hunting exactly
 * where they are instead of teleporting the lot of them. */
static void place_one_hawk(int i) {
    hawks[i].x = screen.width * (i + 1.0) / (config.hawks + 1.0);
    hawks[i].y = screen.height * (i % 2 ? 0.75 : 0.25);
    hawks[i].direction = 2 * M_PI * random_unit();
    hawks[i].frame = direction_frame(hawks[i].direction);
    hawks[i].prey = -1;
    hawks[i].commitment = 0;
    hawks[i].passing = 0;
}

static void place_hawks(void) {
    for (int i = 0; i < config.hawks; i++) place_one_hawk(i);
}

/* The nearest bird, by brute force: eight hawks against four thousand birds is a
 * few tens of thousands of comparisons, well under a percent of a frame, and it
 * needs no structure of its own. A hawk another hawk has already chosen is passed
 * over when `unclaimed` is asked for: two silhouettes converging on one bird read
 * as one hawk with a rendering fault, and they arrive on top of each other. */
static int nearest_bird(const bird_t *birds, double x, double y, int self, int unclaimed,
                        double no_nearer_than) {
    int best = -1;
    double best_distance = 0;
    double floor_squared = no_nearer_than * no_nearer_than;
    for (int b = 0; b < config.birds; b++) {
        if (unclaimed) {
            int taken = 0;
            for (int h = 0; h < config.hawks && !taken; h++)
                if (h != self && hawks[h].prey == b) taken = 1;
            if (taken) continue;
        }
        double dx = birds[b].x - x, dy = birds[b].y - y;
        double distance = dx * dx + dy * dy;
        if (distance < floor_squared) continue;
        if (best < 0 || distance < best_distance) {
            best_distance = distance;
            best = b;
        }
    }
    return best;
}

static double distance_to_bird(const bird_t *birds, const hawk_t *hawk, int bird) {
    double dx = birds[bird].x - hawk->x, dy = birds[bird].y - hawk->y;
    return sqrt(dx * dx + dy * dy);
}

/* How near the hawk will pass its bird over the whole of this frame's travel, not
 * merely where it stands now. At forty pixels a frame a hawk can pass clean
 * through a bird between one frame and the next; the nearest point of the segment
 * it is about to fly is what it actually reaches. */
static double reach_along_the_step(const bird_t *birds, const hawk_t *hawk, int bird) {
    double step = config.speed * HAWK_DIVE_SPEED;
    double dx = birds[bird].x - hawk->x, dy = birds[bird].y - hawk->y;
    double along = dx * cos(hawk->direction) + dy * sin(hawk->direction);
    if (along < 0) along = 0;
    if (along > step) along = step;
    double near_x = hawk->x + along * cos(hawk->direction);
    double near_y = hawk->y + along * sin(hawk->direction);
    dx = birds[bird].x - near_x;
    dy = birds[bird].y - near_y;
    return sqrt(dx * dx + dy * dy);
}

/* Pick, or keep. A hawk holds on to its bird for HAWK_COMMITMENT frames whatever
 * else wanders past, and only when that runs out will it trade up, and then only
 * for a bird a clear quarter closer than the one it is on. Without the hold it
 * re-picked the nearest bird every frame, which in a dense flock means a different
 * bird most frames: it turned on the spot and looked broken rather than hungry. */
static void choose_prey(hawk_t *hawk, const bird_t *birds, int self) {
    if (hawk->prey >= config.birds) hawk->prey = -1;
    if (hawk->prey >= 0 && hawk->commitment > 0) return; /* Committed: hold on. */
    /* And with the commitment spent, a chase that has not closed by now is over:
     * the bird outran it, and there are five hundred others. */
    if (hawk->prey >= 0 && distance_to_bird(birds, hawk, hawk->prey) > HAWK_GIVE_UP)
        hawk->prey = -1;

    /* And it picks its bird from outside the flock's own alarm radius, so there is
     * a chase to watch: the nearest bird to a hawk that has just flown through the
     * middle of a flock is one already beside it, and a strike with no approach is
     * over before anybody sees it start. */
    int candidate = nearest_bird(birds, hawk->x, hawk->y, self, 1, HAWK_STALK);
    if (candidate < 0) candidate = nearest_bird(birds, hawk->x, hawk->y, self, 1, 0);
    if (candidate < 0) candidate = nearest_bird(birds, hawk->x, hawk->y, self, 0, 0);
    if (candidate < 0) return;
    if (hawk->prey >= 0 &&
        distance_to_bird(birds, hawk, candidate) > 0.75 * distance_to_bird(birds, hawk, hawk->prey))
        return; /* Not enough of an improvement to be worth changing its mind. */
    hawk->prey = candidate;
    hawk->commitment = HAWK_COMMITMENT;
}

static double hawk_turn_limit(void) {
    double limit = HAWK_TURN_PER_FRAME();
    /* And never so lazy that it cannot turn inside the room it has: on a small
     * terminal a turning circle wider than a third of the screen means the hawk
     * is committed to the glass before it can see it, and it spends the run
     * bouncing from wall to wall. */
    double shorter = screen.width < screen.height ? screen.width : screen.height;
    if (shorter > 0) {
        double needed = config.speed * HAWK_DIVE_SPEED / (shorter / 3.0);
        if (needed > limit) limit = needed;
    }
    return limit > M_PI ? M_PI : limit;
}

/* How tight a circle it can fly. */
static double hawk_turning_radius(void) {
    return config.speed * HAWK_DIVE_SPEED / hawk_turn_limit();
}

/* And how far off it has to see a wall: a whole diameter, not a radius. A radius
 * is the bare minimum to turn ninety degrees, and since the push starts at
 * nothing at the band's edge the hawk was always committed before the push was
 * worth anything — it reflected off the glass about once a second. Three gives it
 * room to lean out of the turn instead of hauling on it. */
static double hawk_wall_band(void) {
    return hawk_turning_radius() * 3;
}

/* A wall, seen a turning circle ahead. Without this the hawk flew into the edge
 * and reflected: a hundred and eighty degrees in one frame, three times a second,
 * which is exactly what "buggy" looks like. Now it banks away like the flock does,
 * and the reflection below is a safety net that almost never fires. */
static vector_t hawk_wall_vector(const hawk_t *hawk) {
    vector_t wall = {0, 0};
    double band = hawk_wall_band();
    /* Never more than a third of the room there is, or on a small terminal the two
     * sides of the same axis would overlap and the nearer one would win
     * everywhere: the hawk would be pushed one way from every position on the
     * screen and end up pinned against the far edge. */
    double band_x = band < screen.width / 3.0 ? band : screen.width / 3.0;
    double band_y = band < screen.height / 3.0 ? band : screen.height / 3.0;
    if (band_x >= 1) {
        if (hawk->x < band_x)
            wall.x = (band_x - hawk->x) / band_x;
        else if (hawk->x > screen.width - band_x)
            wall.x = -(hawk->x - (screen.width - band_x)) / band_x;
    }
    if (band_y >= 1) {
        if (hawk->y < band_y)
            wall.y = (band_y - hawk->y) / band_y;
        else if (hawk->y > screen.height - band_y)
            wall.y = -(hawk->y - (screen.height - band_y)) / band_y;
    }
    /* The panel is a wall as well. Every bird is forbidden from it and a hawk was
     * not, so it flew over the sliders in a sixth of all frames: the one thing on
     * the screen that is not sky, with something flying through it. It is steered
     * around rather than forbidden, because a hawk that stopped dead at an edge
     * nobody can see would look stranger than one that crosses it. */
    if (screen.legend_width > 0 && hawk->x < screen.legend_width + band_x &&
        hawk->y < screen.legend_height + band_y) {
        double out_right = screen.legend_width + band_x - hawk->x;
        double out_below = screen.legend_height + band_y - hawk->y;
        if (out_right / band_x < out_below / band_y)
            wall.x += out_right / band_x;
        else
            wall.y += out_below / band_y;
    }
    return wall;
}

/* Room for the other hawks: a nudge away from any hawk closer than HAWK_SPACING,
 * so eight of them share a flock instead of flying as one thick smear. */
static vector_t hawk_spacing(int self) {
    vector_t apart = {0, 0};
    for (int i = 0; i < config.hawks; i++) {
        if (i == self) continue;
        double dx = hawks[self].x - hawks[i].x, dy = hawks[self].y - hawks[i].y;
        double squared = dx * dx + dy * dy;
        if (squared >= (double)HAWK_SPACING * HAWK_SPACING || squared < 1e-9) continue;
        double distance = sqrt(squared);
        /* Unanswerable at contact rather than merely firm: a push that fades to
         * nothing as they touch is a push that lets two hawks overlap, which is
         * the one thing this rule exists to prevent. */
        double strength = HAWK_SPACING / distance - 1;
        apart.x += strength * dx / distance;
        apart.y += strength * dy / distance;
    }
    return apart;
}

/*
 * The chase.
 *
 * A hawk aims where its bird is going rather than where it is, which is what makes
 * the pursuit look intelligent instead of trailing. It banks: a limit on the turn
 * per frame, looser than a bird's because a raptor is more agile, but a limit all
 * the same. Inside HAWK_DIVE it stops leading, goes straight at the bird and
 * accelerates, because a fleeing bird is only a tenth slower than a cruising hawk
 * and without that last push the gap never closes and there is nothing to watch.
 *
 * Then it is through them, and flies straight for a few frames before turning
 * back: that exit is the half of a stoop that makes the flock close up behind.
 */
static void hunt(const bird_t *birds) {
    for (int i = 0; i < config.hawks; i++) {
        hawk_t *hawk = &hawks[i];
        if (hawk->commitment > 0) hawk->commitment--;

        if (hawk->passing > 0) {
            hawk->passing--;
            hawk->prey = -1;
        } else {
            /* A pass is being among them, not catching the one it set out after:
             * the bird it chose is fleeing, so what it actually flies through is
             * whichever birds are there when it arrives. */
            /* How near it has to get: two silhouettes, and no wider. Widening it
             * to a frame's travel made the strike distance depend on the frame
             * rate — six times as many strikes at --fps 30 as at 60 — so instead
             * the whole of this frame's travel is tested below, which is the same
             * thing without the arithmetic depending on how fast the clock runs. */
            double arrived = config.bird_size * 2;
            /* Its own bird, and only its own bird. Counting whatever else it
             * happened to pass close to ended nine chases in ten before they were
             * chases: the strike fired the frame the dive began, sixty pixels from
             * the bird it had chosen, and the commitment, the lead and the dive
             * were all dead letters. */
            int struck = hawk->prey >= 0 && hawk->prey < config.birds &&
                         reach_along_the_step(birds, hawk, hawk->prey) < arrived;
            if (struck) {
                hawk->prey = -1;
                hawk->commitment = 0;
                hawk->passing = HAWK_PASS; /* Through and out the other side. */
            } else {
                choose_prey(hawk, birds, i);
            }
        }

        double pace = HAWK_SPEED;
        vector_t apart = hawk_spacing(i);
        vector_t wall = hawk_wall_vector(hawk);
        double want_x = apart.x * HAWK_APART + wall.x * HAWK_WALL;
        double want_y = apart.y * HAWK_APART + wall.y * HAWK_WALL;
        if (hawk->prey >= 0) {
            const bird_t *prey = &birds[hawk->prey];
            double gap = distance_to_bird(birds, hawk, hawk->prey);
            if (gap < HAWK_DIVE) pace = HAWK_DIVE_SPEED;
            /* Aim where the bird will be when the hawk gets there, not a fixed
             * distance ahead: close in, that is almost no lead at all, and a fixed
             * one had it cutting across in front of the bird and out the far side,
             * round and round. Never further ahead than HAWK_LEAD frames, because
             * past that the guess is worth less than the chase. */
            double frames = gap / (config.speed * pace);
            if (frames > HAWK_LEAD) frames = HAWK_LEAD;
            double lead = config.speed * frames;
            double to_x = prey->x + cos(prey->direction) * lead - hawk->x;
            double to_y = prey->y + sin(prey->direction) * lead - hawk->y;
            double reach = sqrt(to_x * to_x + to_y * to_y);
            if (reach > 1e-9) {
                want_x += to_x / reach;
                want_y += to_y / reach;
            }
        }
        if (want_x * want_x + want_y * want_y > 1e-9)
            hawk->direction =
                turn_towards(hawk->direction, normalized_angle(want_y, want_x), hawk_turn_limit());

        hawk->x += config.speed * pace * cos(hawk->direction);
        hawk->y += config.speed * pace * sin(hawk->direction);

        /* Turned back at the walls rather than pinned against them: a clamp left
         * it sliding along an edge for a quarter of every run. The margin is half
         * its own silhouette, so it turns while it is still wholly on the screen —
         * clamping to the screen edge instead put the sprite's far half outside it,
         * and a placement that does not fit is a placement the terminal drops, so
         * every wall cost a one frame blink. */
        double margin = hawk_draw_offset();
        double last_x = screen.width - 1 - margin, last_y = screen.height - 1 - margin;
        if (last_x < margin) last_x = margin;
        if (last_y < margin) last_y = margin;
        if (hawk->x < margin || hawk->x > last_x) {
            hawk->x = hawk->x < margin ? margin : last_x;
            hawk->direction = normalized_angle(sin(hawk->direction), -cos(hawk->direction));
            hawk->prey = -1; /* Whatever it was after, it is not that way now. */
            hawk->commitment = 0;
            hawk->passing = 0;
        }
        if (hawk->y < margin || hawk->y > last_y) {
            hawk->y = hawk->y < margin ? margin : last_y;
            hawk->direction = normalized_angle(-sin(hawk->direction), cos(hawk->direction));
            hawk->prey = -1;
            hawk->commitment = 0;
            hawk->passing = 0;
        }
        hawk->frame = direction_frame(hawk->direction);
    }
}

/* How far a hawk's alarm carries. Never more than a third of the shorter side of
 * the screen: eight hawks with a hundred and fifty pixel reach on a terminal three
 * hundred pixels wide leave the flock nowhere at all to be, and it spends its time
 * pressed against the edges. */
static double hawk_reach(void) {
    double shorter = screen.width < screen.height ? screen.width : screen.height;
    double fits = shorter / 3.0;
    return HAWK_REACH < fits ? HAWK_REACH : fits;
}

/* Every bird flees every hawk in reach, hardest when it is closest, and not
 * straight away from it: part of the flee is sideways, around the hawk, on
 * whichever side the bird is already heading. Straight away and the flock bursts
 * open and is simply gone; around, and it opens, streams past and closes behind,
 * which is the shape worth recording. */
static vector_t hawk_vector(const bird_t *bird) {
    vector_t force = {0, 0};
    for (int i = 0; i < config.hawks; i++) {
        double reach = hawk_reach();
        double dx = bird->x - hawks[i].x, dy = bird->y - hawks[i].y;
        double squared = dx * dx + dy * dy;
        if (squared >= reach * reach || squared < 1e-9) continue;
        double distance = sqrt(squared);
        double strength = (reach - distance) / reach;
        double away_x = dx / distance, away_y = dy / distance;
        /* One of the two ways round; the one the bird is already turning. */
        double side_x = -away_y, side_y = away_x;
        if (cos(bird->direction) * side_x + sin(bird->direction) * side_y < 0) {
            side_x = -side_x;
            side_y = -side_y;
        }
        force.x += strength * (away_x + HAWK_SWIRL * side_x);
        force.y += strength * (away_y + HAWK_SWIRL * side_y);
    }
    return force;
}

/* Each flock flies at its own pace, the first at full speed and the last at
 * seven eighths of it. Two flocks at identical speeds pass through each other and
 * come out symmetrical; a little apart and they shear, overtake and tangle, which
 * is the part that looks alive. Small enough that nobody reads it as a bug. */
static double flock_pace(int flock) {
    if (config.flocks <= 1) return 1.0;
    return 1.0 - 0.125 * flock / (config.flocks - 1);
}

/* Where each flock is, measured once a frame off the same snapshot every bird
 * reads, so every bird in a flock agrees about where its flock is. */
static double flock_center_x[MAX_FLOCKS], flock_center_y[MAX_FLOCKS];
/* Where each flock is told to be: its own centre, shoved clear of the others. */
static double flock_home_x[MAX_FLOCKS], flock_home_y[MAX_FLOCKS];

/* A flock's shade: the ramp's ends first, then the space between them, so two
 * flocks come out as far apart as the palette allows instead of as shade zero and
 * shade one, which on any five step ramp are nearly the same colour. */
static int shade_for_flock(int flock) {
    int shades = palette_shades();
    if (shades <= 1 || config.flocks <= 1) return 0;
    int spread = flock * (shades - 1) / (config.flocks - 1);
    return spread >= shades ? shades - 1 : spread;
}

/* One bird, so that growing the flock at runtime places only the new ones. */
static void place_one_bird(bird_t *bird, int index) {
    double min_x = screen.turn_x, max_x = screen.width - screen.turn_x;
    double min_y = screen.turn_y, max_y = screen.height - screen.turn_bottom;
    if (max_x <= min_x) min_x = max_x = screen.width / 2.0;
    if (max_y <= min_y) min_y = max_y = screen.height / 2.0;

    /* Flocks are handed out round robin so they come out even. Each one starts in
     * its own column of the screen, because scattered evenly they begin as one
     * speckled cloud and take half a minute to sort themselves out; started apart
     * they read as separate flocks from the first frame and then go and meet. */
    bird->flock = index % config.flocks;
    if (config.flocks > 1) {
        double band = (max_x - min_x) / config.flocks;
        min_x += band * bird->flock;
        max_x = min_x + band;
        /* Unless that flock is already flying somewhere, in which case a bird
         * added with + joins it there rather than appearing in the column it
         * started in a minute ago and being dragged across the screen. */
        if (flock_home_x[bird->flock] != 0 || flock_home_y[bird->flock] != 0) {
            min_x = flock_home_x[bird->flock] - FLOCK_LEASH / 2.0;
            max_x = min_x + FLOCK_LEASH;
            min_y = flock_home_y[bird->flock] - FLOCK_LEASH / 2.0;
            max_y = min_y + FLOCK_LEASH;
        }
    }

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
    /* When there is more than one flock the palette follows them: a colour per
     * flock is what makes two of them legible as two. */
    bird->shade = config.flocks > 1 ? shade_for_flock(bird->flock)
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
 * feeder.
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

    force.x = pull * strength * dx / distance;
    force.y = pull * strength * dy / distance;
    return force;
}

/*
 * How hard an edge pushes, given how far into its band a bird has gone.
 *
 * This used to be a flat 1 anywhere in the band, which is why most of the flock
 * ended up off screen: a constant push loses to a strong enough alignment, and
 * once it has lost there is nothing stronger further out to win it back. It now
 * grows, gently through the band and then steeply past the edge of the screen,
 * so leaving is always possible and staying away never is.
 */
static double edge_push(double past, double band) {
    if (past <= 0) return 0;
    double depth = past / band; /* Zero at the band's inner edge, one at the screen's. */
    if (depth <= 1) return EDGE_FIRM * depth * depth;
    /* Past the screen itself the weight stops having a vote: the softest boundary
     * is still a boundary. Dividing it out here means the push a bird feels once
     * it is out of sight is the same at notch 0 as at notch 12, so a low boundary
     * makes them turn late rather than leave. Carrying the weight in the first
     * term keeps the whole thing continuous at the edge: without it the push
     * jumped by a factor of a hundred at notch 0 and a bird was slapped rather
     * than turned. */
    return EDGE_FIRM * (config.boundary + (depth - 1) * ESCAPE_PENALTY) / config.boundary;
}

static vector_t boundary_vector(const bird_t *bird) {
    vector_t boundary = {0, 0};
    if (legend_repels(bird, &boundary)) return boundary;
    if (the_rain_is_falling) return boundary; /* A door, so no wall. */
    if (bird->x < screen.turn_x)
        boundary.x = edge_push(screen.turn_x - bird->x, screen.turn_x);
    else if (bird->x > screen.width - screen.turn_x)
        boundary.x = -edge_push(bird->x - (screen.width - screen.turn_x), screen.turn_x);
    if (bird->y < screen.turn_y)
        boundary.y = edge_push(screen.turn_y - bird->y, screen.turn_y);
    else if (bird->y > screen.height - screen.turn_bottom)
        boundary.y = -edge_push(bird->y - (screen.height - screen.turn_bottom), screen.turn_bottom);
    return boundary;
}

/* How much room one flock asks of another: two leashes, or nearly the whole of
 * the shorter side of the screen when that is less, so that on a small viewport
 * the flocks are not all shoved into the corners. It is an ask, not a promise:
 * five flocks cannot all be three hundred pixels apart on a screen four hundred
 * pixels tall, and what they settle at is as far apart as there is room for. */
static double flock_room(void) {
    double room = 2.0 * FLOCK_LEASH;
    double shorter = screen.width < screen.height ? screen.width : screen.height;
    double fits = 0.9 * shorter;
    return room < fits ? room : fits;
}

static void measure_flocks(const bird_t *birds) {
    int counted[MAX_FLOCKS] = {0};
    for (int f = 0; f < MAX_FLOCKS; f++) {
        flock_center_x[f] = flock_center_y[f] = 0;
        flock_home_x[f] = flock_home_y[f] = 0;
    }
    if (config.flocks <= 1) return;
    for (int i = 0; i < config.birds; i++) {
        int flock = birds[i].flock;
        if (flock < 0 || flock >= MAX_FLOCKS) continue;
        flock_center_x[flock] += birds[i].x;
        flock_center_y[flock] += birds[i].y;
        counted[flock]++;
    }
    for (int f = 0; f < MAX_FLOCKS; f++)
        if (counted[f] > 0) {
            flock_center_x[f] /= counted[f];
            flock_center_y[f] /= counted[f];
        }

    /* Left to themselves the three centres of gravity drift to the middle of the
     * screen and sit on top of each other, and three flocks become one cloud in
     * three colours. So each flock is sent home to its own centre moved clear of
     * every other flock's: they shoulder each other apart, find their own corners
     * of the sky and keep them, without any of it being on rails. */
    double room = flock_room();
    for (int f = 0; f < config.flocks && f < MAX_FLOCKS; f++) {
        flock_home_x[f] = flock_center_x[f];
        flock_home_y[f] = flock_center_y[f];
        if (counted[f] == 0) continue;
        double shove_x = 0, shove_y = 0;
        int crowding = 0;
        for (int g = 0; g < config.flocks && g < MAX_FLOCKS; g++) {
            if (g == f || counted[g] == 0) continue;
            double dx = flock_center_x[f] - flock_center_x[g];
            double dy = flock_center_y[f] - flock_center_y[g];
            double distance = sqrt(dx * dx + dy * dy);
            if (distance >= room) continue;
            if (distance < 1e-9) {
                /* Exactly on top of one another: pick a direction rather than
                 * dividing by nothing, one for each flock, and let them unfold. */
                double angle = 2 * M_PI * f / config.flocks;
                dx = cos(angle);
                dy = sin(angle);
                distance = 1;
            }
            shove_x += (room - distance) * dx / distance;
            shove_y += (room - distance) * dy / distance;
            crowding++;
        }
        /* The sum of the shoves, but never further than one room's worth of it:
         * four flocks all pushing the middle one used to throw its home two
         * hundred pixels clear off the screen, where the leash drags the flock
         * into an edge that is pushing back just as hard. Averaging instead was
         * worse — in a row of three the two outer shoves on the middle flock
         * oppose each other, and an average of opposites is nothing at all. */
        if (crowding > 0) {
            double shove = sqrt(shove_x * shove_x + shove_y * shove_y);
            if (shove > room) {
                shove_x = shove_x * room / shove;
                shove_y = shove_y * room / shove;
            }
            flock_home_x[f] += shove_x;
            flock_home_y[f] += shove_y;
        }
        /* And home is somewhere a flock can actually be. */
        if (flock_home_x[f] < 0) flock_home_x[f] = 0;
        if (flock_home_y[f] < 0) flock_home_y[f] = 0;
        if (flock_home_x[f] > screen.width) flock_home_x[f] = screen.width;
        if (flock_home_y[f] > screen.height) flock_home_y[f] = screen.height;
    }
}

/* The leash: nothing within FLOCK_LEASH of its own flock's centre, and outside
 * that a pull home that grows with the distance. */
static vector_t leash_vector(const bird_t *bird) {
    vector_t pull = {0, 0};
    if (config.flocks <= 1 || bird->flock < 0 || bird->flock >= MAX_FLOCKS) return pull;
    double dx = flock_home_x[bird->flock] - bird->x;
    double dy = flock_home_y[bird->flock] - bird->y;
    double distance = sqrt(dx * dx + dy * dy);
    if (distance <= FLOCK_LEASH || distance < 1e-9) return pull;
    double strength = (distance - FLOCK_LEASH) / FLOCK_LEASH;
    if (strength > 1) strength = 1;
    pull.x = strength * dx / distance;
    pull.y = strength * dy / distance;
    return pull;
}

static void read_bird_position(const void *context, int index, double *x, double *y) {
    const bird_t *birds = context;
    *x = birds[index].x;
    *y = birds[index].y;
}

static double flock_direction(const bird_t *birds, const spatial_grid_t *grid, int target_index) {
    const bird_t *target = &birds[target_index];
    double want_x, want_y;
    /* Writing overrules flocking while it lasts: a bird with a target steers at
     * it and nothing else, which is what makes a letter a letter. */
    if (spell_target_of(target_index, &want_x, &want_y)) {
        double to_x = want_x - target->x, to_y = want_y - target->y;
        if (to_x * to_x + to_y * to_y > 1e-9) return normalized_angle(to_y, to_x);
        return target->direction;
    }
    vector_t separation = {0, 0}, alignment = {0, 0}, cohesion = {0, 0};
    vector_t boundary = boundary_vector(target);
    vector_t leash = leash_vector(target);
    vector_t pointer = pointer_vector(target);
    vector_t hawk = hawk_vector(target);
    vector_t wind = wind_vector();
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
    if (neighbors) {
        if (kin) {
            alignment.x /= kin;
            alignment.y /= kin;
            cohesion.x = cohesion.x / kin - target->x;
            cohesion.y = cohesion.y / kin - target->y;
        }
        double x = separation.x * config.separation + alignment.x * config.alignment +
                   cohesion.x * COHESION_W + boundary.x * config.boundary + leash.x * LEASH_WEIGHT +
                   pointer.x * MOUSE_WEIGHT + hawk.x * HAWK_WEIGHT + wind.x * WIND_WEIGHT;
        double y = separation.y * config.separation + alignment.y * config.alignment +
                   cohesion.y * COHESION_W + boundary.y * config.boundary + leash.y * LEASH_WEIGHT +
                   pointer.y * MOUSE_WEIGHT + hawk.y * HAWK_WEIGHT + wind.y * WIND_WEIGHT;
        return x == 0 && y == 0 ? target->direction : normalized_angle(y, x);
    }
    boundary.x = boundary.x * config.boundary + leash.x * LEASH_WEIGHT + pointer.x * MOUSE_WEIGHT +
                 hawk.x * HAWK_WEIGHT + wind.x * WIND_WEIGHT;
    boundary.y = boundary.y * config.boundary + leash.y * LEASH_WEIGHT + pointer.y * MOUSE_WEIGHT +
                 hawk.y * HAWK_WEIGHT + wind.y * WIND_WEIGHT;
    if (boundary.x != 0 || boundary.y != 0) {
        double x = cos(target->direction) + boundary.x;
        double y = sin(target->direction) + boundary.y;
        if (x != 0 || y != 0) return normalized_angle(y, x);
    }
    return target->direction;
}

/*
 * What decides a bird's shade within the ramp, and it is not a setting: one flock
 * is coloured by heading, and more than one by flock, because those are the two
 * answers anybody wanted.
 *
 * There used to be a --color-by with four modes and a key to cycle them. Two of
 * them were the same thing — colouring by flock is what a flock is given at birth
 * anyway — one painted two thirds of a default flock in the single darkest shade
 * of the ramp and got worse the more birds there were, and the fourth is this.
 *
 * Heading is the striking one: a turn runs a ripple of colour through the whole
 * flock, because neighbours that agree on a heading agree on a colour.
 */
static int shade_for(const bird_t *bird) {
    int shades = palette_shades();
    if (shades <= 1) return 0;
    if (config.flocks > 1) return shade_for_flock(bird->flock);

    double turns = normalized_angle(sin(bird->direction), cos(bird->direction)) / (2 * M_PI);
    /* Folded at the half turn, because a heading is a circle and a ramp is a line:
     * laid straight on to it, two birds a degree apart either side of due east got
     * the two ends of the palette, and the flock came out salted with dark speckle
     * that no turn of it explained. Folded, the two ways round meet in the middle
     * and the colour runs smoothly with the heading; opposite headings share a
     * shade, which nobody can see, and the seam, which everybody could, is gone. */
    double folded = turns < 0.5 ? turns * 2 : (1 - turns) * 2;
    int shade = (int)(folded * shades);
    return shade >= shades ? shades - 1 : shade;
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
    measure_flocks(snapshot);
    for (int i = 0; i < config.birds; i++) {
        double direction = flock_direction(snapshot, grid, i);
        /* Banking is for flocking. Two things are not flocking and are exempt: a
         * bird writing a letter, which has to be able to land on it, and a bird
         * inside the panel's turn zone, whose push is a constraint rather than a
         * force. Limiting that one would break the panel's unreachability, which
         * is proved on the assumption that a bird can turn away at once. */
        double unused_x, unused_y;
        if (!spell_target_of(i, &unused_x, &unused_y) &&
            !legend_turn_zone(snapshot[i].x, snapshot[i].y))
            direction = turn_towards(snapshot[i].direction, direction, turn_limit());
        birds[i].direction = direction;
        /* Never past the target: the last step is the distance left, which is
         * what makes a letter crisp instead of a cloud orbiting one. */
        double step = config.speed * flock_pace(snapshot[i].flock);
        double want_x, want_y;
        if (spell_target_of(i, &want_x, &want_y)) {
            double dx = want_x - birds[i].x, dy = want_y - birds[i].y;
            double remaining = sqrt(dx * dx + dy * dy);
            if (remaining < step) step = remaining;
        }
        birds[i].x += step * cos(direction);
        birds[i].y += step * sin(direction);
        if (the_rain_is_falling) wrap_position(&birds[i]);
        birds[i].shade = shade_for(&birds[i]);
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

static int frame_limit;
static int bench_frames;
static const char *snapshot_path;
static const char *record_path;
static int record_fps = 25;
static int record_seconds = 6;
/* The size of the recording, in cells, as one thing: two flags for one idea, and
 * nobody was ever going to reach for four hundred by a hundred and twenty. */
static int record_columns = 96;
static int record_rows = 26;
static const char *requested_record_size;

/* What the panel's stats row reports, averaged over the last second so the
 * numbers are readable rather than flickering. */
static struct {
    double frame_ms;
    double bytes;
    double rate;
    long counted;
    double window_started;
    double window_ms;
    double window_bytes;
} stats;

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
     * under each other and the keys stay where the eye already looks for them.
     * Padded by cells rather than by bytes: a degree sign is two bytes and one
     * cell, and printf counts the bytes, so that row came out a cell short and
     * the panel had a bite out of its right hand side. */
    size_t bytes = strlen(value), glyphs = 0;
    for (const unsigned char *c = (const unsigned char *)value; *c != '\0'; c++)
        if ((*c & 0xc0) != 0x80) glyphs++;
    int column = LEGEND_VALUE_WIDTH + (int)(bytes - glyphs);
    snprintf(line, size, "\u2502 %-*s %s %*s  %c/%c \u2502", LEGEND_NAME_WIDTH, name, bar, column,
             value, lower, raise);
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
    legend_number(value, sizeof(value), config.alignment, 2);
    legend_slider(lines[3], LEGEND_LINE_MAX, "alignment", config.alignment_notch, value, 'a', 'A');
    /* On the panel because it has keys: t and T used to change the banking with
     * nothing on the screen to say what they had done, which is how somebody could
     * turn it to nothing and be left looking at an empty sky. */
    snprintf(value, sizeof(value), "%.0f\u00b0", turning_notch_radians() * 180 / M_PI);
    legend_slider(lines[4], LEGEND_LINE_MAX, "turning", config.turning_notch, value, 't', 'T');
    /* Pixels and frames a second are whole numbers, so they print as such. */
    snprintf(value, sizeof(value), "%dpx", config.vision_radius);
    legend_slider(lines[5], LEGEND_LINE_MAX, "perception", config.vision_notch, value, 'p', 'P');
    snprintf(value, sizeof(value), "%d", config.frame_rate);
    legend_slider(lines[6], LEGEND_LINE_MAX, "rate", config.rate_notch, value, 'r', 'R');

    /* What the frame costs, always, rather than behind a flag: it was a switch
     * that did nothing at all unless the panel was up, and the row it wrote into
     * was otherwise blank. Built as text and padded to the same inner width as
     * every other row — written straight into the row it came out four cells short
     * and the panel had a notch in its right hand side. The units are on the
     * numbers, because 2.1 30 60 is three numbers and not a sentence. */
    char measured[LEGEND_LINE_MAX / 2];
    snprintf(measured, sizeof(measured), "%-*s %5.1fms %5.0fKB %3.0ffps", LEGEND_NAME_WIDTH,
             "frame", stats.frame_ms, stats.bytes / 1024.0, stats.rate);
    snprintf(lines[7], LEGEND_LINE_MAX, "\u2502 %-*s \u2502", inner - 2, measured);
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

static kitty_graphics_status_t queue_text_frame(kitty_graphics_t *graphics, const bird_t *birds);

static kitty_graphics_status_t queue_render_frame(kitty_graphics_t *graphics, const bird_t *birds) {
    if (drawing_with_text()) return queue_text_frame(graphics, birds);
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
        bird_t as_bird = {.x = hawks[i].x - hawk_draw_offset(),
                          .y = hawks[i].y - hawk_draw_offset(),
                          .frame = hawks[i].frame};
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

/*
 * The text renderer's own state: the sprites as pixels, a canvas the size of
 * the screen, and the grid of cells that is diffed against the last frame.
 */
static png_status_t rasterise_sprites(png_image_t *frames);
static void compose_onto(png_image_t *canvas, const png_image_t *frames, const bird_t *birds,
                         int with_ground);

static png_image_t text_sprites[ROTATION_FRAMES * (MAX_PALETTE_SHADES + 1)];
static png_image_t text_canvas;
static cells_t text_cells;
static int text_legend_was_drawn;

/* Whether the terminal can take a 24 bit colour. COLORTERM is the convention,
 * and nearly every terminal that can sets it; one that cannot gets the nearest
 * of the 256 colour cube, which is coarser and still a flock. */
static int terminal_has_truecolor(void) {
    const char *colorterm = getenv("COLORTERM");
    if (colorterm == NULL) return 0;
    return strcmp(colorterm, "truecolor") == 0 || strcmp(colorterm, "24bit") == 0;
}

static int prepare_text_renderer(void) {
    if (rasterise_sprites(text_sprites) != PNG_OK) return 0;
    if (cells_init(&text_cells, terminal_has_truecolor()) != CELLS_OK) return 0;
    return 1;
}

/* Sized to the screen, and resized with it. */
static int text_renderer_fits_the_screen(void) {
    if (text_canvas.width != screen.width || text_canvas.height != screen.height) {
        png_image_free(&text_canvas);
        if (png_image_alloc(&text_canvas, screen.width, screen.height) != PNG_OK) return 0;
    }
    return cells_resize(&text_cells, screen.cols, screen.rows) == CELLS_OK;
}

static kitty_graphics_status_t queue_text_frame(kitty_graphics_t *graphics, const bird_t *birds) {
    if (!text_renderer_fits_the_screen()) return KITTY_GRAPHICS_ERR_MEMORY;
    kitty_graphics_status_t status = kitty_graphics_begin_synchronized_update(graphics);
    /* The panel first, so that when it is switched off the rows it erases are
     * redrawn by the cells that follow in the same frame, and when it is on the
     * cells leave its corner alone rather than drawing over its text. */
    if (status == KITTY_GRAPHICS_OK) status = queue_legend(graphics);
    if (legend_drawn != text_legend_was_drawn) {
        cells_invalidate(&text_cells);
        text_legend_was_drawn = legend_drawn;
    }
    cells_keep_out_of(&text_cells, legend_drawn ? LEGEND_COLUMNS : 0,
                      legend_drawn ? LEGEND_ROWS : 0);

    compose_onto(&text_canvas, text_sprites, birds, 0);
    cells_read(&text_cells, render_mode == RENDER_BLOCKS ? CELLS_BLOCKS : CELLS_BRAILLE,
               &text_canvas, screen.cell_width, screen.cell_height);
    if (cells_emit(&text_cells) != CELLS_OK) return KITTY_GRAPHICS_ERR_MEMORY;
    if (status == KITTY_GRAPHICS_OK)
        status = kitty_graphics_write_raw(graphics, text_cells.text, text_cells.length);
    if (status == KITTY_GRAPHICS_OK) status = kitty_graphics_end_synchronized_update(graphics);
    return status;
}

/* Move first, then draw, and move both the flock and the hawks before drawing
 * either. Drawing first meant the frame on the screen held the birds from before
 * the step and the hawks from after it — a hawk a whole step, forty pixels, ahead
 * of the hole it had just made, which is not what the recorder drew and not what
 * the physics said. Paused still draws and still reads keys, so the panel answers
 * and the sliders can be explored on a still frame; a step grants one frame of
 * motion and then stands still again. */
static kitty_graphics_status_t render_frame(kitty_graphics_t *graphics, bird_t *birds,
                                            const bird_t *snapshot, const spatial_grid_t *grid) {
    if (!paused || step_once) {
        step_once = 0;
        hunt(snapshot);
        update_birds(birds, snapshot, grid);
        for (int i = 0; i < config.birds; i++) birds[i].frame = direction_frame(birds[i].direction);
    }
    return queue_render_frame(graphics, birds);
}

static void update_speed(void) {
    config.speed = (double)DEFAULT_SPEED * DEFAULT_FRAME_RATE / config.frame_rate;
    /* And never more than a tenth of the shorter side in one frame. The step is
     * fixed in pixels a second, which on a full screen is a bird crossing in half
     * a second and on a small one is a bird crossing in an eighth: it cannot turn
     * inside a band it clears in two frames, and one bird in six was off the
     * screen at forty by fourteen. Above about four hundred and fifty pixels tall
     * — which is most terminals — this changes nothing. */
    double shorter = screen.width < screen.height ? screen.width : screen.height;
    if (shorter > 0 && config.speed > shorter / 10.0) config.speed = shorter / 10.0;
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
/* Left alone for this long, the sliders start wandering by themselves. It was
 * two flags — one to switch it on and one to set the seconds — for a behaviour
 * whose only two useful settings are this and --screensaver. */
enum { IDLE_SECONDS = 60 };
static int screensaver;
static int matrix_mode;
static int clock_mode;
static char spell_buffer[256];
static int requested_perception = DEFAULT_VISION_RADIUS;
static int requested_seed = -1;
static int requested_preset = -1;

/*
 * A preset is the six notches together, because the interesting settings are
 * combinations rather than single values, and naming one is how a look gets
 * shared. The order is boundary, separation, alignment, perception, rate, which
 * is the order the panel shows them in.
 */
typedef struct {
    const char *name;
    const char *help;
    int notch[5];
} preset_t;

static const preset_t PRESETS[] = {
    /* No preset takes the boundary below 7 or the frame rate below the default,
     * whatever else it does. Under either, a preset is not a look, it is a flock
     * that spends its time outside the frame or jumping five body lengths between
     * one frame and the next: school had one bird in nine off the screen and calm,
     * of all of them, was the choppiest thing in the program. */
    {"murmuration", "one great restless body, the starling look", {7, 3, 9, 8, 6}},
    {"swarm", "tight, fast and nervous, like insects", {7, 8, 3, 3, 8}},
    {"storm", "loose and violent, thrown about", {9, 10, 2, 6, 9}},
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
    config.alignment_notch = preset->notch[2];
    config.vision_notch = preset->notch[3];
    config.rate_notch = preset->notch[4];
    apply_notches();
}

/* The option table: the parser and the help text both come off this, so adding a
 * switch is one row and never a second place to keep in step. */
static const option_t OPTIONS[] = {
    /* short, long, alias, kind, target, min, max, names, metavar, help, group, on -h */
    {'n', "birds", NULL, OPTION_INT, &config.birds, 1, MAX_BIRDS, NULL, "COUNT",
     "how many birds (default 800)", "Flock", 1},
    {'s', "size", NULL, OPTION_INT, &config.bird_size, MIN_BIRD_SIZE, MAX_BIRD_SIZE, NULL, "PIXELS",
     "sprite size in pixels (default 15)", "Flock", 1},
    /* k is the hawk key in the panel, so k is the hawk flag on the line: -k 3 used
     * to mean three flocks, which is a trap laid by the program's own help. */
    {'g', "flocks", "groups", OPTION_INT, &config.flocks, 1, MAX_FLOCKS, NULL, "COUNT",
     "flocks that keep to their own kind (default 1)", "Flock", 1},
    {'k', "hawks", NULL, OPTION_INT, &config.hawks, 0, MAX_HAWKS, NULL, "COUNT",
     "predators hunting the flock (default 0)", "Flock", 1},
    {0, "preset", NULL, OPTION_ENUM, &requested_preset, 0, 0, PRESET_NAMES, "NAME",
     "murmuration, swarm, storm", "Flock", 1},
    {0, "seed", NULL, OPTION_INT, &requested_seed, 0, 2147483647, NULL, "N",
     "the same seed gives the same flock", "Flock", 0},

    {0, "boundary", NULL, OPTION_INT, &config.boundary_notch, 0, LEGEND_BAR_CELLS, NULL, "NOTCH",
     "how hard the edges push back (default 4)", "Sliders   0 to 12, as the panel shows them", 0},
    {0, "separation", NULL, OPTION_INT, &config.separation_notch, 0, LEGEND_BAR_CELLS, NULL,
     "NOTCH", "how much a bird keeps its distance (default 4)",
     "Sliders   0 to 12, as the panel shows them", 0},
    {0, "alignment", NULL, OPTION_INT, &config.alignment_notch, 0, LEGEND_BAR_CELLS, NULL, "NOTCH",
     "how much it matches its neighbours (default 4)", "Sliders   0 to 12, as the panel shows them",
     0},
    {0, "turning", NULL, OPTION_INT, &config.turning_notch, 0, LEGEND_BAR_CELLS, NULL, "NOTCH",
     "sharpest turn a frame, 12 is instant (default 8)",
     "Sliders   0 to 12, as the panel shows them", 0},
    {0, "perception", NULL, OPTION_INT, &requested_perception, MIN_VISION_RADIUS, MAX_VISION_RADIUS,
     NULL, "PIXELS", "how far it sees, 12 to 60 (default 36)",
     "Sliders   0 to 12, as the panel shows them", 0},
    {'f', "fps", NULL, OPTION_INT, &requested_frame_rate, MIN_FRAME_RATE, MAX_FRAME_RATE, NULL,
     "RATE", "frames a second, 30 to 120 (default 60)",
     "Sliders   0 to 12, as the panel shows them", 1},

    {'c', "color", "palette", OPTION_ENUM, &config.palette, 0, 0, PALETTE_NAMES, "RAMP",
     "theme, ember, ice, acid, matrix", "Look", 1},
    {0, "shape", NULL, OPTION_ENUM, &config.shape, 0, 0, SHAPE_NAMES, "NAME",
     "bird, arrow, plane, dot", "Look", 1},
    {0, "sprite", NULL, OPTION_STRING, &sprite_path, 0, 0, NULL, "FILE",
     "a PNG of your own, kept in its own colours", "Look", 0},
    {'e', "trails", NULL, OPTION_FLAG, &config.trails, 0, 0, NULL, NULL,
     "faint tails behind the flock", "Look", 0},
    {'l', "no-panel", "no-legend", OPTION_OFF, &legend_enabled, 0, 0, NULL, NULL,
     "hide the sliders in the corner", "Look", 1},

    {0, "spell", NULL, OPTION_STRING, &requested_spell, 0, 0, NULL, "TEXT",
     "the flock writes TEXT; - reads stdin", "World", 1},
    {0, "spell-hold", NULL, OPTION_INT, &spell_hold, 0, 600, NULL, "SECONDS",
     "how long it holds the writing (default 6, 0 forever)", "World", 0},
    {0, "clock", NULL, OPTION_FLAG, &clock_mode, 0, 0, NULL, NULL,
     "the flock is the time, re-formed on the minute", "World", 0},

    {'m', "mouse", NULL, OPTION_ENUM, &config.mouse_mode, 0, 0, MOUSE_NAMES, "MODE",
     "the pointer is: flee, follow, off (default flee)", "Input", 1},
    {0, "mouse-reach", NULL, OPTION_INT, &config.mouse_reach, 40, 280, NULL, "PIXELS",
     "how far the pointer reaches, 40 to 280 (default 120)", "Input", 0},
    {0, "no-mouse-reporting", NULL, OPTION_OFF, &mouse_enabled, 0, 0, NULL, NULL,
     "do not ask the terminal for the pointer", "Input", 0},

    {0, "screensaver", NULL, OPTION_FLAG, &screensaver, 0, 0, NULL, NULL,
     "no panel, autopilot, any key quits", "Modes", 1},
    {0, "no-intro", NULL, OPTION_OFF, &show_intro, 0, 0, NULL, NULL,
     "do not open by writing the name", "Modes", 0},
    {0, "no-outro", NULL, OPTION_OFF, &show_outro, 0, 0, NULL, NULL, "do not fly away on q",
     "Modes", 0},

    {0, "matrix", NULL, OPTION_FLAG, &matrix_mode, 0, 0, NULL, NULL, "it is raining birds",
     "Oddities", 0},

    {0, "bench", NULL, OPTION_INT, &bench_frames, 0, 1000000, NULL, "N",
     "run N frames with no terminal, print the numbers, quit", "Output", 0},
    {0, "frames", NULL, OPTION_INT, &frame_limit, 0, 1000000, NULL, "N",
     "quit after N frames, for recording", "Output", 0},
    {0, "snapshot", NULL, OPTION_STRING, &snapshot_path, 0, 0, NULL, "FILE",
     "write the last frame as a PNG", "Output", 0},
    {0, "record", NULL, OPTION_STRING, &record_path, 0, 0, NULL, "FILE",
     "record an animated GIF with no terminal, and quit", "Output", 0},
    {0, "record-fps", NULL, OPTION_INT, &record_fps, 2, MAX_RECORD_FPS, NULL, "RATE",
     "frames a second in the GIF, up to 50 (default 25)", "Output", 0},
    {0, "record-seconds", NULL, OPTION_INT, &record_seconds, 1, 120, NULL, "SECONDS",
     "how long the GIF runs (default 6)", "Output", 0},
    {0, "record-size", NULL, OPTION_STRING, &requested_record_size, 0, 0, NULL, "COLSxROWS",
     "the size to record at, in cells (default 96x26)", "Output", 0},

    {0, "render", NULL, OPTION_ENUM, &render_mode, 0, 0, RENDER_NAMES, "HOW",
     "kitty, braille or blocks; auto asks the terminal (default auto)", "Look", 1},
};
enum { OPTION_COUNT = sizeof(OPTIONS) / sizeof(*OPTIONS) };

/* The panel teaches the slider keys, so this only has to list the rest. */
#define KEYS_HELP                                                      \
    "\nKeys   b/B s/S a/A t/T p/P r/R   one notch down / up\n"         \
    "       space pause   . step   0 reset   +/- birds   Tab preset\n" \
    "       h panel   e trails   k/K hawks   M mouse\n"                \
    "       q quit\n"

enum { EXIT_USAGE = 2 }; /* A mistyped command is not a run that went wrong. */

static const option_example_t EXAMPLES[] = {
    {"cbirds", "a flock, and nothing to read"},
    {"cbirds --preset murmuration", "the starling look"},
    {"cbirds --hawks 2 --color ember", "something to watch"},
    {"cbirds --flocks 3 --color ice", "three of them, keeping to their own"},
    {"fortune | cbirds --spell -", "the flock writes whatever is piped in"},
    {"cbirds --record flock.gif", "a GIF, with no terminal in the way"},
    {"cbirds --screensaver", "for a terminal left open"},
    {NULL, NULL},
};

/* Back to the shipped look, which is what someone reaches for after pressing
 * every key to see what it does. */
static void apply_preset_defaults(void) {
    config.boundary_notch = DEFAULT_NOTCH;
    config.separation_notch = DEFAULT_NOTCH;
    config.alignment_notch = DEFAULT_NOTCH;
    config.vision_notch = DEFAULT_VISION_NOTCH;
    config.rate_notch = DEFAULT_NOTCH;
    /* These two as well. They are not on the panel, so somebody who has turned
     * the banking down with t and cannot see what they did has nothing else to
     * undo it with, and "back to the defaults" left them where they were. */
    config.turning_notch = DEFAULT_TURNING_NOTCH;
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
    int *notches[] = {&config.boundary_notch, &config.separation_notch, &config.alignment_notch,
                      &config.vision_notch};
    enum { NOTCH_COUNT = sizeof(notches) / sizeof(*notches) };
    int which = (int)(random_unit() * NOTCH_COUNT) % NOTCH_COUNT;
    int *notch = notches[which];
    int step = random_unit() < 0.5 ? -1 : 1;

    /* Turned back at the ends rather than stuck against them. */
    if (*notch + step < 0 || *notch + step > LEGEND_BAR_CELLS) step = -step;
    *notch += step;
    apply_notches();
}

/* A screensaver flies itself from the first frame; anything else waits a minute
 * to be sure nobody is watching. */
static int flying_itself(void) {
    if (screensaver) return 1;
    return clock_state.seconds - last_key_at >= IDLE_SECONDS;
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

/*
 * Up up down down left right left right b a.
 *
 * Listed in --help rather than hidden, because an easter egg nobody finds is
 * wasted and the line itself is a screenshot. The arrows arrive as CSI final
 * bytes, which the sequence reader already has in hand; b and a are the same
 * keys that move the boundary and alignment sliders, so those move too, and
 * nothing is lost by that.
 */
static const char KONAMI[] = "AABBDCDCba";
enum { KONAMI_LENGTH = sizeof(KONAMI) - 1 };
static char konami_seen[KONAMI_LENGTH];
static int konami_at;

static void konami_note(char key) {
    /* The last ten keys, compared as a whole. A ring rather than a running match
     * because people mash arrows, and a stutter should not throw the sequence
     * away: AAABBDCDCba has the code in it and ought to count. */
    konami_seen[konami_at % KONAMI_LENGTH] = key;
    konami_at++;
    if (konami_at < KONAMI_LENGTH) return;
    for (int i = 0; i < KONAMI_LENGTH; i++)
        if (konami_seen[(konami_at + i) % KONAMI_LENGTH] != KONAMI[i]) return;

    konami_at = 0;
    memset(konami_seen, 0, sizeof(konami_seen));
    config.hawks = MAX_HAWKS;
    place_hawks();
    if (spell_layout("NICE")) spell.until = clock_state.seconds + 3.0;
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
                if (key == 'M' || key == 'm')
                    read_mouse_report(sequence);
                else if (sequence_length == 0)
                    konami_note((char)key); /* A bare arrow, not a modified one. */
                continue;
            }
            if (sequence_length + 1 < sizeof(sequence)) sequence[sequence_length++] = (char)key;
            continue;
        }
        if (key == '\033') {
            input_state = INPUT_ESCAPE;
            continue;
        }

        if (key == 'b' || key == 'a') konami_note((char)key);
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
                if (hawk_sets_built && config.hawks < MAX_HAWKS) {
                    config.hawks++;
                    place_one_hawk(config.hawks - 1);
                }
                continue;
            case 'K':
                if (config.hawks > 0) config.hawks--;
                continue;
            case 'T':
                if (config.turning_notch < LEGEND_BAR_CELLS) config.turning_notch++;
                continue;
            case 't':
                if (config.turning_notch > 0) config.turning_notch--;
                continue;
            case 'e':
                config.trails = !config.trails;
                continue;
            case 'M':
                config.mouse_mode = (config.mouse_mode + 1) % MOUSE_MODE_COUNT;
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

/*
 * The program writes its own PNG, with its own encoder.
 *
 * Compositing is the one thing the renderer never has to do, because Kitty does
 * it: so a snapshot rebuilds the rotated sprites as pixels, alpha blends every
 * bird into one canvas, and hands it to png_encode. Fifty milliseconds and a few
 * megabytes for a still, which is a fair price for a picture that the README can
 * honestly say the program drew of itself.
 */
static void blend_sprite(png_image_t *canvas, const png_image_t *sprite, int at_x, int at_y) {
    for (int y = 0; y < sprite->height; y++) {
        int cy = at_y + y;
        if (cy < 0 || cy >= canvas->height) continue;
        for (int x = 0; x < sprite->width; x++) {
            int cx = at_x + x;
            if (cx < 0 || cx >= canvas->width) continue;
            const uint8_t *src =
                sprite->pixels + ((size_t)y * (size_t)sprite->width + (size_t)x) * 4;
            uint8_t *dst = canvas->pixels + ((size_t)cy * (size_t)canvas->width + (size_t)cx) * 4;
            unsigned alpha = src[3];
            if (alpha == 0) continue;
            for (int c = 0; c < 3; c++)
                dst[c] = (uint8_t)((src[c] * alpha + dst[c] * (255 - alpha)) / 255);
            dst[3] = (uint8_t)(alpha + dst[3] * (255 - alpha) / 255);
        }
    }
}

/* The rotated sprites as pixels, which is what compositing needs and the
 * renderer never does, because Kitty does it. */
/*
 * Every shade, and the hawks' bigger silhouette after them, laid out exactly as
 * the image ids are: shade major, then the hawk set. A snapshot that rasterised
 * one shade and used it for every bird was a picture of a flock that does not
 * exist, which is what the README's stills and the demo were until now.
 */
static png_status_t rasterise_sprites(png_image_t *frames) {
    png_image_t source = {0, 0, NULL};
    png_status_t status = load_sprite(&source);
    if (status != PNG_OK) return status;

    int hawk_size = hawk_sprite_size();
    int sets = palette_shades() + 1;
    for (int set = 0; set < sets && status == PNG_OK; set++) {
        int hawk = set == palette_shades();
        int size = hawk ? hawk_size : config.bird_size;
        png_image_t canvas_sprite = {0, 0, NULL};
        int work = size * SPRITE_SUPERSAMPLE;
        if (work > SPRITE_WORK_MAX) work = SPRITE_WORK_MAX;
        if (work > source.width) work = source.width;
        status = png_resize(&source, work, work, &canvas_sprite);

        for (int i = 0; i < ROTATION_FRAMES && status == PNG_OK; i++) {
            png_image_t *frame = &frames[set * ROTATION_FRAMES + i];
            status = png_rotate_resize(&canvas_sprite, i * FRAME_ANGLE * M_PI / 180.0, size, size,
                                       frame);
            if (status != PNG_OK) break;
            if (hawk)
                hawk_tint(frame);
            else
                palette_tint(frame, set);
        }
        png_image_free(&canvas_sprite);
    }
    png_image_free(&source);
    return status;
}

static void free_sprites(png_image_t *frames) {
    for (int i = 0; i < ROTATION_FRAMES * (MAX_PALETTE_SHADES + 1); i++) png_image_free(&frames[i]);
}

/* The ground, opaque, so a picture looks like the terminal it was taken in
 * rather than like a cut out. */
static void fill_ground(png_image_t *canvas) {
    for (size_t i = 0; i < (size_t)canvas->width * (size_t)canvas->height; i++) {
        canvas->pixels[i * 4 + 0] = 18;
        canvas->pixels[i * 4 + 1] = 18;
        canvas->pixels[i * 4 + 2] = 24;
        canvas->pixels[i * 4 + 3] = 255;
    }
}

/* The same order the live renderer places in: tails, then the flock, then the
 * hawks over the top. On a ground for a picture; on nothing at all for a text
 * terminal, whose own background is the sky and whose cells must not paint it. */
static void compose_onto(png_image_t *canvas, const png_image_t *frames, const bird_t *birds,
                         int with_ground) {
    int shades = palette_shades();
    if (with_ground)
        fill_ground(canvas);
    else
        memset(canvas->pixels, 0, (size_t)canvas->width * (size_t)canvas->height * 4);

    if (config.trails && shades > 1) {
        for (int i = 0; i < config.birds; i += TRAIL_EVERY)
            for (int step = 0; step < birds[i].trail_held; step++) {
                const png_image_t *sprite =
                    &frames[(shades - 1) * ROTATION_FRAMES + birds[i].frame % ROTATION_FRAMES];
                if (sprite->pixels != NULL)
                    blend_sprite(canvas, sprite, (int)birds[i].trail_x[step],
                                 (int)birds[i].trail_y[step]);
            }
    }
    for (int i = 0; i < config.birds; i++) {
        int shade = birds[i].shade % shades;
        const png_image_t *sprite =
            &frames[shade * ROTATION_FRAMES + birds[i].frame % ROTATION_FRAMES];
        if (sprite->pixels == NULL) continue;
        blend_sprite(canvas, sprite, (int)birds[i].x, (int)birds[i].y);
    }
    for (int i = 0; i < config.hawks; i++) {
        const png_image_t *sprite =
            &frames[shades * ROTATION_FRAMES + hawks[i].frame % ROTATION_FRAMES];
        if (sprite->pixels == NULL) continue;
        blend_sprite(canvas, sprite, (int)hawks[i].x - hawk_draw_offset(),
                     (int)hawks[i].y - hawk_draw_offset());
    }
}

/* A picture of what is on the screen. Under Kitty that is the sprites on a
 * ground; on a text terminal it is the dots or the blocks, painted the way the
 * terminal shows them, because a snapshot of the pixels the cells were read from
 * would be a picture of something nobody saw. */
static int write_snapshot(const char *path, const bird_t *birds) {
    png_image_t canvas = {0, 0, NULL};
    static png_image_t frames[ROTATION_FRAMES * (MAX_PALETTE_SHADES + 1)];
    uint8_t *encoded = NULL;
    size_t encoded_length = 0;
    int written = 0;
    static const uint8_t ground[3] = {18, 18, 24};

    png_status_t status = PNG_OK;
    if (drawing_with_text()) {
        if (cells_paint(&text_cells, render_mode == RENDER_BLOCKS ? CELLS_BLOCKS : CELLS_BRAILLE,
                        &canvas, screen.cell_width, screen.cell_height, ground) != CELLS_OK)
            status = PNG_ERR_MEMORY;
    } else {
        status = rasterise_sprites(frames);
        if (status == PNG_OK) status = png_image_alloc(&canvas, screen.width, screen.height);
        if (status == PNG_OK) compose_onto(&canvas, frames, birds, 1);
    }
    if (status == PNG_OK) {
        status = png_encode(&canvas, &encoded, &encoded_length);
    }
    free_sprites(frames);
    png_image_free(&canvas);

    if (status == PNG_OK) {
        FILE *out = fopen(path, "wb");
        if (out != NULL) {
            written = fwrite(encoded, 1, encoded_length, out) == encoded_length;
            fclose(out);
        }
    }
    free(encoded);
    return written;
}

static void usage(FILE *out, const char *program, int everything) {
    options_usage(out, program, "cbirds \u2014 a flock of birds in your terminal.", EXAMPLES,
                  OPTIONS, OPTION_COUNT, everything);
    if (everything) fputs(KEYS_HELP, out);
}

/* "-" means stdin, so that fortune | cbirds --spell - works. */
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
            fprintf(stderr, "%s: read the text from stdin but found no terminal to run in\n",
                    program_name);
            exit(EXIT_FAILURE);
        }
    }
    return at > 0 ? spell_buffer : NULL;
}

static void read_options(int argc, char **argv) {
    char error[160];
    if (argc > 0 && argv[0] != NULL) program_name = argv[0];
    name_the_palettes();
    name_the_presets();
    name_the_shapes();
    options_status_t status =
        options_parse(OPTIONS, OPTION_COUNT, argc, argv, error, sizeof(error));

    if (status == OPTIONS_HELP || status == OPTIONS_HELP_FULL) {
        usage(stdout, program_name, status == OPTIONS_HELP_FULL);
        exit(EXIT_SUCCESS);
    }
    if (status == OPTIONS_COMPLETION) {
        if (!options_completion(stdout, error, "cbirds", OPTIONS, OPTION_COUNT)) {
            fprintf(stderr, "%s: --completion wants bash, zsh or fish\n", program_name);
            exit(EXIT_USAGE);
        }
        exit(EXIT_SUCCESS);
    }
    if (status == OPTIONS_VERSION) {
        printf("cbirds %s\n", CBIRDS_VERSION);
        exit(EXIT_SUCCESS);
    }
    if (status != OPTIONS_OK) {
        /* Told what was wrong, and where to look, and exiting two rather than one
         * so a script can tell a mistyped command from a run that went wrong. */
        fprintf(stderr, "%s: %s\n", program_name, error);
        fprintf(stderr, "Try '%s --help'.\n", program_name);
        exit(EXIT_USAGE);
    }
    /* A preset is expanded first so that a slider given after it still wins: the
     * table cannot express that order, so the parser's left to right reading is
     * honoured by putting the broad stroke before the fine ones. */
    if (requested_preset >= 0) {
        int boundary = config.boundary_notch, separation = config.separation_notch;
        int alignment = config.alignment_notch;
        apply_preset(requested_preset);
        if (boundary != DEFAULT_NOTCH) config.boundary_notch = boundary;
        if (separation != DEFAULT_NOTCH) config.separation_notch = separation;
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
    /* Checked here rather than where it is used, so every mode reports a bad
     * sprite the same way and none of them gets halfway into a run first. */
    if (sprite_path != NULL) {
        png_image_t probe = {0, 0, NULL};
        png_status_t sprite_status = load_sprite(&probe);
        if (sprite_status != PNG_OK) {
            fprintf(stderr, "%s: %s: %s\n", program_name, sprite_path,
                    png_status_string(sprite_status));
            exit(EXIT_FAILURE);
        }
        png_image_free(&probe);
    }
    if (requested_record_size != NULL) {
        int columns = 0, rows = 0;
        char extra = 0;
        if (sscanf(requested_record_size, "%dx%d%c", &columns, &rows, &extra) != 2 ||
            columns < 40 || columns > 400 || rows < 14 || rows > 120) {
            fprintf(stderr, "%s: --record-size wants COLUMNSxROWS, 40x14 to 400x120, not '%s'\n",
                    program_name, requested_record_size);
            exit(EXIT_USAGE);
        }
        record_columns = columns;
        record_rows = rows;
    }
    requested_spell = read_spell_text(requested_spell);
    /* A palette with one colour in it cannot tell the flocks apart, which is worth
     * saying out loud rather than letting somebody wonder where their three flocks
     * went. */
    if (config.flocks > 1 && palette_shades() <= 1)
        fprintf(stderr, "%s: %s has one colour, so the %d flocks will look like one\n",
                program_name, palette()->name, config.flocks);
    /* It is raining birds: green, falling, wrapping, with tails. Every part of it
     * is a switch that already existed, which is the whole joke. */
    if (matrix_mode) {
        config.palette = palette_named("matrix");
        config.trails = 1;
        config.alignment_notch = LEGEND_BAR_CELLS;
        the_rain_is_falling = 1;
        apply_notches();
    }
    /* A screensaver has one job and no panel, and anything at all ends it. */
    if (screensaver) {
        legend_enabled = 0;
        show_intro = 0;
    }
}

static long elapsed_microseconds(const struct timespec *start, const struct timespec *end) {
    return (end->tv_sec - start->tv_sec) * 1000000L + (end->tv_nsec - start->tv_nsec) / 1000L;
}

/*
 * The numbers, with no terminal in the way.
 *
 * What the README claims about this program should be measurable by anyone who
 * clones it, so the measurement is a flag rather than a paragraph. No terminal is
 * opened and nothing is drawn: the frame is built into the graphics buffer and
 * its length counted, which is exactly what a real frame would put on the wire.
 */
/*
 * Hundredths of a second a frame, for a rate asked for in frames a second.
 *
 * A GIF carries its delay as whole hundredths, so the only rates it has are
 * 100/1, 100/2, 100/3 and so on, and viewers clamp anything under two hundredths
 * up to a tenth of a second. Fifty is therefore the ceiling and sixty is not a
 * rate a GIF has at all: whatever is asked for is rounded to one it does.
 */
static int record_delay_for(int fps) {
    int best = 100 / MAX_RECORD_FPS;
    double error = -1;
    /* Chosen by the error in the rate, not by rounding the delay: the two are not
     * the same, because the rate is a hundred over the delay. Rounding gives 50
     * for 41 frames a second where 33 is nearer to it. */
    for (int delay = 100 / MAX_RECORD_FPS; delay <= 100; delay++) {
        double mistake = 100.0 / delay - fps;
        if (mistake < 0) mistake = -mistake;
        if (error < 0 || mistake < error) {
            error = mistake;
            best = delay;
        }
    }
    return best;
}

/*
 * Recording.
 *
 * Headless, like the benchmark, and for the same reason: the demo in the README
 * has to be regenerable by anyone who clones this, with one command, and nothing
 * about it should depend on a terminal being attached or on how fast one happens
 * to be. Every frame is simulated, every --record-every'th is composited and
 * handed to the GIF writer.
 */
/* Headless there is no terminal to ask what colours it uses, so the palette that
 * follows the terminal has nothing in it: black birds on a black ground, which is
 * what `cbirds --record flock.gif` — the example in the help — recorded. Both the
 * modes that never open a terminal fall back to the shipped ramp. */
static void settle_the_palette_without_a_terminal(void) {
    if (palette_follows_the_theme()) config.palette = FALLBACK_PALETTE;
}

static int run_recording(void) {
    static png_image_t frames[ROTATION_FRAMES * (MAX_PALETTE_SHADES + 1)];
    png_image_t canvas = {0, 0, NULL};
    spatial_grid_t grid;
    gif_writer_t *gif = NULL;
    size_t bytes = 0;
    int written = 0;
    /* The delay is whole hundredths, so the rate asked for is rounded to one the
     * format can carry and the rate actually achieved is reported rather than
     * claimed. Every simulated frame is recorded, and the simulation steps at the
     * recording rate, so the motion in the GIF runs at life speed. */
    settle_the_palette_without_a_terminal();
    int delay = record_delay_for(record_fps);
    /* The rate a hundredth-of-a-second delay really gives, which is not always a
     * whole number: a delay of 17 plays at 5.88 a second, not 5. */
    double actual_fps = 100.0 / delay;
    /* Counted on that rate, so --record-seconds means the seconds it lasts rather
     * than the seconds it was meant to. */
    int total = (int)(actual_fps * record_seconds + 0.5);

    /* Same formula as update_speed, at the recording rate instead of the display
     * one, so a bird covers the same ground per second whatever the rate is. */
    config.speed = (double)DEFAULT_SPEED * DEFAULT_FRAME_RATE / actual_fps;

    /* A GIF has no panel in it: the panel is terminal text, and compose() draws
     * birds. Left enabled it would still reserve its corner and keep the flock
     * out of it, and every recording would have an unexplained empty rectangle in
     * the top left. */
    legend_enabled = 0;
    apply_screen_size(record_columns, record_rows, record_columns * DEFAULT_CELL_WIDTH,
                      record_rows * DEFAULT_CELL_HEIGHT);
    if (spatial_grid_init(&grid, SPATIAL_CELL_SIZE) != SPATIAL_GRID_OK) return EXIT_FAILURE;
    if (spatial_grid_prepare(&grid, screen.width, screen.height, config.birds) != SPATIAL_GRID_OK)
        return EXIT_FAILURE;
    if (rasterise_sprites(frames) != PNG_OK) {
        fprintf(stderr, "%s: cannot build the sprites to record with\n", program_name);
        return EXIT_FAILURE;
    }
    if (png_image_alloc(&canvas, screen.width, screen.height) != PNG_OK) return EXIT_FAILURE;

    gif_status_t gif_status = gif_open(&gif, record_path, screen.width, screen.height, delay);
    if (gif_status != GIF_OK) {
        fprintf(stderr, "%s: %s: %s\n", program_name, record_path, gif_status_string(gif_status));
        return EXIT_FAILURE;
    }

    bird_t *birds = calloc((size_t)config.birds, sizeof(*birds));
    bird_t *snapshot = malloc(sizeof(*snapshot) * (size_t)config.birds);
    if (birds == NULL || snapshot == NULL) return EXIT_FAILURE;
    srand(requested_seed >= 0 ? (unsigned)requested_seed : 1u);
    initialize_birds(birds);
    place_hawks();
    if (requested_spell != NULL && spell_layout(requested_spell))
        spell.until = spell_hold > 0 ? (double)spell_hold : -1.0;

    for (int frame = 0; frame < total && gif_status == GIF_OK; frame++) {
        /* The clock the features read has to advance, or nothing that animates
         * on its own terms would animate at all. */
        clock_state.frame = frame;
        clock_state.seconds = (double)frame / actual_fps;
        if (spell.writing && spell.until >= 0 && clock_state.seconds >= spell.until) spell_clear();
        maybe_tell_the_time();
        maybe_drift();

        memcpy(snapshot, birds, sizeof(*birds) * (size_t)config.birds);
        spatial_grid_build(&grid, config.birds, read_bird_position, snapshot);
        hunt(snapshot);
        update_birds(birds, snapshot, &grid);
        for (int i = 0; i < config.birds; i++) birds[i].frame = direction_frame(birds[i].direction);

        compose_onto(&canvas, frames, birds, 1);
        gif_status = gif_add_frame(gif, &canvas);
    }

    gif_status_t closed = gif_close(gif, &bytes, &written);
    if (gif_status == GIF_OK) gif_status = closed;
    free_sprites(frames);
    png_image_free(&canvas);
    spatial_grid_destroy(&grid);
    free(snapshot);
    free(birds);

    if (gif_status != GIF_OK) {
        fprintf(stderr, "%s: %s: %s\n", program_name, record_path, gif_status_string(gif_status));
        return EXIT_FAILURE;
    }
    printf("%s: %d frames, %dx%d, %.4g fps, %.1fs, %.1f KB\n", record_path, written, screen.width,
           screen.height, actual_fps, written / actual_fps, (double)bytes / 1024.0);
    if (delay != record_delay_for(record_fps) || (int)(actual_fps + 0.5) != record_fps)
        fprintf(stderr,
                "%s: asked for %d fps, recorded at %.4g. A GIF's delay between frames is\n"
                "whole hundredths of a second, so the only rates it has are 100/1, 100/2,\n"
                "100/3 and so on, and viewers clamp anything under two hundredths up to a\n"
                "tenth. %.4g is the nearest rate this format can actually carry.\n",
                program_name, record_fps, actual_fps, actual_fps);
    return EXIT_SUCCESS;
}

static int run_benchmark(void) {
    kitty_graphics_t graphics;
    spatial_grid_t grid;
    struct timespec start, finish;

    settle_the_palette_without_a_terminal();
    apply_screen_size(200, 50, 1600, 800);
    if (spatial_grid_init(&grid, SPATIAL_CELL_SIZE) != SPATIAL_GRID_OK) return EXIT_FAILURE;
    if (spatial_grid_prepare(&grid, screen.width, screen.height, config.birds) != SPATIAL_GRID_OK)
        return EXIT_FAILURE;
    if (kitty_graphics_init(&graphics, STDOUT_FILENO) != KITTY_GRAPHICS_OK) return EXIT_FAILURE;

    bird_t *birds = calloc((size_t)config.birds, sizeof(*birds));
    bird_t *snapshot = malloc(sizeof(*snapshot) * (size_t)config.birds);
    if (birds == NULL || snapshot == NULL) return EXIT_FAILURE;
    srand(requested_seed >= 0 ? (unsigned)requested_seed : 1u);
    initialize_birds(birds);
    place_hawks();

    double bytes = 0;
    clock_gettime(CLOCK_MONOTONIC, &start);
    for (int frame = 0; frame < bench_frames; frame++) {
        memcpy(snapshot, birds, sizeof(*birds) * (size_t)config.birds);
        spatial_grid_build(&grid, config.birds, read_bird_position, snapshot);
        hunt(snapshot);
        graphics.length = 0;
        render_frame(&graphics, birds, snapshot, &grid);
        bytes += (double)graphics.length;
    }
    clock_gettime(CLOCK_MONOTONIC, &finish);

    double seconds =
        (double)(finish.tv_sec - start.tv_sec) + (double)(finish.tv_nsec - start.tv_nsec) / 1e9;
    double per_frame = seconds / bench_frames;
    printf("birds        %d\n", config.birds);
    printf("flocks       %d\n", config.flocks);
    printf("hawks        %d\n", config.hawks);
    printf("viewport     %dx%d px\n", screen.width, screen.height);
    printf("frames       %d\n", bench_frames);
    printf("frame time   %.3f ms\n", per_frame * 1000.0);
    printf("ceiling      %.0f fps\n", 1.0 / per_frame);
    printf("bytes/frame  %.0f (%.1f KB)\n", bytes / bench_frames, bytes / bench_frames / 1024.0);
    printf("at %d fps    %.1f MB/s\n", config.frame_rate,
           bytes / bench_frames * config.frame_rate / 1e6);

    kitty_graphics_destroy(&graphics);
    spatial_grid_destroy(&grid);
    free(snapshot);
    free(birds);
    return EXIT_SUCCESS;
}

int main(int argc, char **argv) {
    image_frame_t frames[ROTATION_FRAMES * (MAX_PALETTE_SHADES + 1)] = {0};
    kitty_graphics_t graphics;
    spatial_grid_t grid;
    struct timespec frame_start, frame_end;
    read_options(argc, argv);
    if (bench_frames > 0) return run_benchmark();
    if (record_path != NULL) return run_recording();
    install_signal_handlers();
    atexit(restore_terminal);

    /* The terminal is asked its questions before anything is built for it: can
     * you draw this at all, and what colours do you use? The sprites are then
     * built once, in the answers. */
    if (enter_terminal() < 0) {
        perror("Can't enable raw mode");
        exit(EXIT_FAILURE);
    }
    /* Kitty's protocol if the terminal answers for it, and the same flock in
     * braille if it does not: there is no terminal this refuses to run in. */
    if (render_mode == RENDER_AUTO)
        render_mode = terminal_speaks_graphics() ? RENDER_KITTY : RENDER_BRAILLE;
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
    if (drawing_with_text()) {
        /* The sprites stay pixels, to be read back as cells every frame. */
        if (!prepare_text_renderer()) {
            fprintf(stderr, "%s: cannot build the sprites to draw with\n", program_name);
            exit(EXIT_FAILURE);
        }
    } else {
        build_rotation_frames(frames, palette_shades(), config.bird_size, 0);
        /* A hawk has to read as a bigger bird at a glance, so it gets its own set
         * at twice the size, one shade, uploaded straight after the flock's. */
        build_rotation_frames(frames + palette_shades() * ROTATION_FRAMES, 1, hawk_sprite_size(),
                              1);
    }
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
    if (!drawing_with_text()) {
        int sprite_sets = palette_shades() + 1;
        graphics_status = upload_rotation_frames(&graphics, frames, sprite_sets);
        free_rotation_frames(frames, sprite_sets);
        if (graphics_status != KITTY_GRAPHICS_OK) {
            fprintf(stderr, "Cannot upload Kitty graphics: %s\n",
                    kitty_graphics_status_string(graphics_status));
            exit(EXIT_FAILURE);
        }
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
        graphics_status = leaving ? queue_render_frame(&graphics, birds)
                                  : render_frame(&graphics, birds, snapshot, &grid);
        if (graphics_status != KITTY_GRAPHICS_OK) {
            fprintf(stderr, "Cannot render Kitty graphics: %s\n",
                    kitty_graphics_status_string(graphics_status));
            exit(EXIT_FAILURE);
        }
        size_t frame_bytes = graphics.length;
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
        /* Averaged over a second, because a number that changes sixty times a
         * second is decoration rather than information. */
        stats.window_ms += (double)elapsed_microseconds(&frame_start, &frame_end) / 1000.0;
        stats.window_bytes += (double)frame_bytes;
        stats.counted++;
        if (clock_state.seconds - stats.window_started >= 1.0) {
            double span = clock_state.seconds - stats.window_started;
            stats.frame_ms = stats.window_ms / (double)stats.counted;
            stats.bytes = stats.window_bytes / (double)stats.counted;
            stats.rate = (double)stats.counted / span;
            stats.window_started = clock_state.seconds;
            stats.window_ms = stats.window_bytes = 0;
            stats.counted = 0;
        }
        long remaining =
            1000000L / config.frame_rate - elapsed_microseconds(&frame_start, &frame_end);
        if (remaining > 0) {
            struct timespec delay = {remaining / 1000000L, (remaining % 1000000L) * 1000L};
            nanosleep(&delay, NULL);
        }
    }
    if (snapshot_path != NULL) {
        restore_terminal();
        if (write_snapshot(snapshot_path, birds))
            fprintf(stderr, "%s: wrote %s\n", program_name, snapshot_path);
        else
            fprintf(stderr, "%s: could not write %s\n", program_name, snapshot_path);
    }
    spatial_grid_destroy(&grid);
    kitty_graphics_destroy(&graphics);
    free(snapshot);
    free(birds);
    return EXIT_SUCCESS;
}
