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

#include "kitty_graphics.h"
#include "options.h"
#include "png.h"
#include "spatial_grid.h"
#include "sprite_png.h"

enum {
    ROTATION_FRAMES = 90,
    MAX_PALETTE_SHADES = 8,
    MAX_FLOCKS = 5,
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
#define MOUSE_ON "\033[?1003h\033[?1006h"
#define MOUSE_OFF "\033[?1006l\033[?1003l"

typedef struct {
    double x, y;
} vector_t;

typedef struct {
    double x, y, direction;
    int frame;
    int shade; /* Index into the palette, and half of the image id. */
    int flock; /* Which flock it reads: separation ignores this, the rest does not. */
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
    .vision_notch = 6,
    .rate_notch = DEFAULT_NOTCH,
};
static screen_t screen;
static int legend_enabled = 1; /* Cleared by --no-legend, never at runtime. */
static int mouse_enabled = 1;  /* Cleared by --no-mouse, never at runtime. */

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
/* Whether the panel is currently on screen. The panel is anchored at the origin
 * and constant in cells, so it never leaves text behind by moving: the only row
 * ever needing an erase is one it occupied before being switched off. Clearing
 * the whole screen would take the uploaded sprites with it. */
static int legend_drawn;
static struct termios saved_termios;
static volatile sig_atomic_t terminal_is_raw;
static volatile sig_atomic_t terminal_restored;

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
    write_all(MOUSE_OFF, sizeof(MOUSE_OFF) - 1);
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

static int enter_terminal(void) {
    struct termios raw;
    write_all(ALT_SCREEN_ON, sizeof(ALT_SCREEN_ON) - 1);
    write_all(CURSOR_HIDE, sizeof(CURSOR_HIDE) - 1);
    if (mouse_enabled) write_all(MOUSE_ON, sizeof(MOUSE_ON) - 1);
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
    {"original", "the sprite as it was drawn", 1, NULL, PNG_TINT_MULTIPLY},
    {"ember", "embers, pale gold to deep red", 5, EMBER_TINTS, PNG_TINT_REPLACE},
    {"ice", "ice, white through to deep blue", 5, ICE_TINTS, PNG_TINT_REPLACE},
    {"acid", "acid, lime through to teal", 5, ACID_TINTS, PNG_TINT_REPLACE},
    {"paper", "paper, five greys", 5, PAPER_TINTS, PNG_TINT_REPLACE},
};
enum { PALETTE_COUNT = sizeof(PALETTES) / sizeof(*PALETTES) };

static const char *PALETTE_NAMES[PALETTE_COUNT + 1];

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
static void build_rotation_frames(image_frame_t *frames, int shades) {
    png_image_t source = {0, 0, NULL}, canvas = {0, 0, NULL};
    png_status_t status = png_decode(sprite_png, sprite_png_len, &source);
    if (status != PNG_OK) {
        fprintf(stderr, "Cannot decode the embedded sprite: %s\n", png_status_string(status));
        exit(EXIT_FAILURE);
    }
    int canvas_size = config.bird_size * SPRITE_SUPERSAMPLE;
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
        status = png_rotate_resize(&canvas, radians, config.bird_size, config.bird_size, &frame);
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
static void initialize_birds(bird_t *birds) {
    double min_x = screen.turn_x, max_x = screen.width - screen.turn_x;
    double min_y = screen.turn_y, max_y = screen.height - screen.turn_bottom;
    if (max_x <= min_x) min_x = max_x = screen.width / 2.0;
    if (max_y <= min_y) min_y = max_y = screen.height / 2.0;

    for (int i = 0; i < config.birds; i++) {
        bird_t *bird = &birds[i];
        /* Rejected rather than clamped, so the whole free region stays in play
         * instead of the flock piling up along one edge of the panel. Bounded,
         * with a deterministic fallback below the panel, because on a small
         * viewport the free region can be almost entirely covered. */
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
        /* Flocks are handed out round robin so they come out even, and when there
         * is more than one the palette follows them: a colour per flock is what
         * makes two of them legible as two. */
        bird->flock = i % config.flocks;
        bird->shade = config.flocks > 1
                          ? bird->flock % palette_shades()
                          : (int)(random_unit() * palette_shades()) % palette_shades();
    }
}

static double normalized_angle(double y, double x) {
    double angle = atan2(y, x);
    return angle < 0 ? angle + 2 * M_PI : angle;
}

static vector_t boundary_vector(const bird_t *bird) {
    vector_t boundary = {0, 0};
    if (legend_repels(bird, &boundary)) return boundary;
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

static double flock_direction(const bird_t *birds, const spatial_grid_t *grid, int target_index) {
    const bird_t *target = &birds[target_index];
    vector_t separation = {0, 0}, alignment = {0, 0}, cohesion = {0, 0};
    vector_t boundary = boundary_vector(target);
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
                   cohesion.x * config.cohesion + boundary.x * config.boundary;
        double y = separation.y * config.separation + alignment.y * config.alignment +
                   cohesion.y * config.cohesion + boundary.y * config.boundary;
        return x == 0 && y == 0 ? target->direction : normalized_angle(y, x);
    }
    boundary.x *= config.boundary;
    boundary.y *= config.boundary;
    if (boundary.x != 0 || boundary.y != 0) {
        double x = cos(target->direction) + boundary.x;
        double y = sin(target->direction) + boundary.y;
        if (x != 0 || y != 0) return normalized_angle(y, x);
    }
    return target->direction;
}

static void update_birds(bird_t *birds, const bird_t *snapshot, const spatial_grid_t *grid) {
    for (int i = 0; i < config.birds; i++) {
        double direction = flock_direction(snapshot, grid, i);
        birds[i].direction = direction;
        birds[i].x += config.speed * cos(direction);
        birds[i].y += config.speed * sin(direction);
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
    for (int i = 0; status == KITTY_GRAPHICS_OK && i < config.birds; i++) {
        kitty_graphics_placement_t placement;
        if (bird_placement(&birds[i], &placement))
            status = kitty_graphics_place(graphics, &placement);
    }
    if (status == KITTY_GRAPHICS_OK) status = queue_legend(graphics);
    if (status == KITTY_GRAPHICS_OK) status = kitty_graphics_end_synchronized_update(graphics);
    return status;
}

static kitty_graphics_status_t render_frame(kitty_graphics_t *graphics, bird_t *birds,
                                            const bird_t *snapshot, const spatial_grid_t *grid) {
    kitty_graphics_status_t status = queue_render_frame(graphics, birds);
    if (status != KITTY_GRAPHICS_OK) return status;

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

static int handle_input(void) {
    enum { INPUT_NORMAL, INPUT_ESCAPE, INPUT_SEQUENCE };
    static int input_state = INPUT_NORMAL;
    static char sequence[32];
    static size_t sequence_length;
    char input[INPUT_BUFFER_SIZE];
    ssize_t length = read(STDIN_FILENO, input, sizeof(input));
    for (ssize_t i = 0; i < length; i++) {
        unsigned char key = (unsigned char)input[i];
        int *notch = NULL, step = 0;
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

/* The option table: the parser and the help text both come off this, so adding a
 * switch is one row and never a second place to keep in step. */
static int requested_frame_rate = DEFAULT_FRAME_RATE;

static const option_t OPTIONS[] = {
    {'n', "birds", OPTION_INT, &config.birds, 1, MAX_BIRDS, NULL, "COUNT",
     "how many boids to fly (default 800)", "Flock"},
    {'s', "size", OPTION_INT, &config.bird_size, MIN_BIRD_SIZE, MAX_BIRD_SIZE, NULL, "PIXELS",
     "sprite size in pixels (default 15)", "Flock"},
    {'k', "flocks", OPTION_INT, &config.flocks, 1, MAX_FLOCKS, NULL, "COUNT",
     "split into this many flocks that will not merge (default 1)", "Flock"},
    {'c', "palette", OPTION_ENUM, &config.palette, 0, 0, PALETTE_NAMES, "NAME",
     "colour the flock: original, ember, ice, acid, paper", "Flock"},
    {'f', "fps", OPTION_INT, &requested_frame_rate, MIN_FRAME_RATE, MAX_FRAME_RATE, NULL, "RATE",
     "frames a second, snapped to a notch (default 60)", "Display"},
    {'l', "legend", OPTION_FLAG, &legend_enabled, 0, 0, NULL, NULL,
     "show the parameter panel, on by default", "Display"},
    {'m', "mouse", OPTION_FLAG, &mouse_enabled, 0, 0, NULL, NULL,
     "follow the pointer, on by default", "Interaction"},
};
enum { OPTION_COUNT = sizeof(OPTIONS) / sizeof(*OPTIONS) };

static const char *const EXAMPLES[] = {
    "cbirds                      a flock, and nothing to read",
    "cbirds -n 2000 -f 120       more of them, faster",
    "cbirds --no-legend          hide the panel, the flock keeps the corner",
    NULL,
};

static void usage(FILE *out, const char *program) {
    options_usage(out, program, "cbirds \u2014 a flock of birds in your terminal.", EXAMPLES,
                  OPTIONS, OPTION_COUNT);
}

static void read_options(int argc, char **argv) {
    char error[160];
    name_the_palettes();
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
    /* The notch is the state the keys move, so a rate off that grid could not be
     * one: snap what was asked for to the nearest. */
    config.rate_notch = ((requested_frame_rate - MIN_FRAME_RATE) * LEGEND_BAR_CELLS +
                         (MAX_FRAME_RATE - MIN_FRAME_RATE) / 2) /
                        (MAX_FRAME_RATE - MIN_FRAME_RATE);
    apply_notches();
}

static long elapsed_microseconds(const struct timespec *start, const struct timespec *end) {
    return (end->tv_sec - start->tv_sec) * 1000000L + (end->tv_nsec - start->tv_nsec) / 1000L;
}

int main(int argc, char **argv) {
    image_frame_t frames[ROTATION_FRAMES * MAX_PALETTE_SHADES] = {0};
    kitty_graphics_t graphics;
    spatial_grid_t grid;
    struct timespec frame_start, frame_end;
    read_options(argc, argv);
    spatial_grid_status_t grid_status = spatial_grid_init(&grid, SPATIAL_CELL_SIZE);
    if (grid_status != SPATIAL_GRID_OK) {
        fprintf(stderr, "Cannot initialize spatial grid: %s\n",
                spatial_grid_status_string(grid_status));
        exit(EXIT_FAILURE);
    }
    srand((unsigned)time(NULL));
    update_screen_dimensions();
    grid_status = spatial_grid_prepare(&grid, screen.width, screen.height, config.birds);
    if (grid_status != SPATIAL_GRID_OK) {
        fprintf(stderr, "Cannot prepare spatial grid: %s\n",
                spatial_grid_status_string(grid_status));
        exit(EXIT_FAILURE);
    }
    build_rotation_frames(frames, palette_shades());

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

    install_signal_handlers();
    atexit(restore_terminal);
    if (enter_terminal() < 0) {
        perror("Can't enable raw mode");
        exit(EXIT_FAILURE);
    }
    write_all("\x1b[J", sizeof("\x1b[J") - 1);
    update_screen_dimensions();
    initialize_birds(birds);
    graphics_status = upload_rotation_frames(&graphics, frames, palette_shades());
    free_rotation_frames(frames, palette_shades());
    if (graphics_status != KITTY_GRAPHICS_OK) {
        fprintf(stderr, "Cannot upload Kitty graphics: %s\n",
                kitty_graphics_status_string(graphics_status));
        exit(EXIT_FAILURE);
    }

    struct timespec started;
    clock_gettime(CLOCK_MONOTONIC, &started);
    int running = 1;
    while (running) {
        running = handle_input();
        if (!running) break;

        clock_gettime(CLOCK_MONOTONIC, &frame_start);
        clock_state.frame++;
        clock_state.seconds = (double)(frame_start.tv_sec - started.tv_sec) +
                              (double)(frame_start.tv_nsec - started.tv_nsec) / 1e9;
        update_screen_dimensions();
        grid_status = spatial_grid_prepare(&grid, screen.width, screen.height, config.birds);
        if (grid_status != SPATIAL_GRID_OK) {
            fprintf(stderr, "Cannot resize spatial grid: %s\n",
                    spatial_grid_status_string(grid_status));
            exit(EXIT_FAILURE);
        }
        memcpy(snapshot, birds, sizeof(*birds) * (size_t)config.birds);
        grid_status = spatial_grid_build(&grid, config.birds, read_bird_position, snapshot);
        if (grid_status != SPATIAL_GRID_OK) {
            fprintf(stderr, "Cannot build spatial grid: %s\n",
                    spatial_grid_status_string(grid_status));
            exit(EXIT_FAILURE);
        }
        graphics_status = render_frame(&graphics, birds, snapshot, &grid);
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
