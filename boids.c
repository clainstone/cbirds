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
#include "png.h"
#include "spatial_grid.h"
#include "sprite_png.h"

enum {
    ROTATION_FRAMES = 90,
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
    DEFAULT_VISION_CELLS = 3,
    MIN_VISION_CELLS = 1,
    MAX_VISION_CELLS = 5,
    /* Legend widths, in columns: below the narrowest the bar is dropped and the
     * flock keeps the whole viewport. */
    LEGEND_NARROW_COLS = 44,
    LEGEND_MEDIUM_COLS = 74,
    LEGEND_WIDE_COLS = 100,
    LEGEND_MIN_ROWS = 6,
    LEGEND_TEXT_MAX = 256,
    LEGEND_LINE_MAX = LEGEND_TEXT_MAX + 16
};

/* Needed in the config initializer, so macros rather than constants. */
#define DEFAULT_SEPARATION_W 0.005
#define DEFAULT_ALIGNMENT_W 1.5
#define DEFAULT_COHESION_W 0.01
#define DEFAULT_BOUNDARY_W 0.2

static const double BOUNDARY_STEP = 0.02;
static const double BOUNDARY_MIN = 0.01;
static const double SEPARATION_STEP = 0.001;
static const double SEPARATION_MIN = 0.001;
static const double COHESION_STEP = 0.002;
static const double COHESION_MIN = 0.002;
static const double ALIGNMENT_STEP = 0.1;
static const double ALIGNMENT_MIN = 0.1;

/* Three times the default each. The weights used to rise without a limit, which
 * left a slider nothing to fill against; this also puts every default at a third
 * of its travel and keeps a keypress worth between two and six cells of bar. */
static const double BOUNDARY_MAX = 3 * DEFAULT_BOUNDARY_W;
static const double SEPARATION_MAX = 3 * DEFAULT_SEPARATION_W;
static const double COHESION_MAX = 3 * DEFAULT_COHESION_W;
static const double ALIGNMENT_MAX = 3 * DEFAULT_ALIGNMENT_W;

#define ALT_SCREEN_ON "\033[?1049h"
#define ALT_SCREEN_OFF "\033[?1049l"
#define CURSOR_HIDE "\033[?25l"
#define CURSOR_SHOW "\033[?25h"
#define SYNC_UPDATE_END "\033[?2026l"

typedef struct {
    double x, y;
} vector_t;

typedef struct {
    double x, y, direction;
    int frame;
} bird_t;

typedef struct {
    uint8_t *data;
    size_t length;
} image_frame_t;

typedef struct {
    int width, height, cols, rows;
    int cell_width, cell_height, turn_x, turn_y, turn_bottom;
    int legend_row; /* Zero-based row of the legend, negative when there is none. */
} screen_t;

typedef struct {
    int birds, frame_rate, bird_size;
    double speed;
    int vision_cells, vision_radius, vision_radius_squared;
    double separation, alignment, cohesion, boundary;
} config_t;

static config_t config = {
    .birds = 800,
    .frame_rate = DEFAULT_FRAME_RATE,
    .speed = DEFAULT_SPEED,
    .bird_size = DEFAULT_BIRD_SIZE,
    .vision_cells = DEFAULT_VISION_CELLS,
    .vision_radius = DEFAULT_VISION_CELLS * SPATIAL_CELL_SIZE,
    .vision_radius_squared =
        DEFAULT_VISION_CELLS * SPATIAL_CELL_SIZE * DEFAULT_VISION_CELLS * SPATIAL_CELL_SIZE,
    .separation = DEFAULT_SEPARATION_W,
    .alignment = DEFAULT_ALIGNMENT_W,
    .cohesion = DEFAULT_COHESION_W,
    .boundary = DEFAULT_BOUNDARY_W,
};
static screen_t screen;
/* Where the legend was last drawn, so a resize can erase that row and nothing
 * else. Clearing the whole screen would take the uploaded sprites with it. */
static int drawn_legend_row = -1;
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

/* The legend is text, and a Kitty placement is not clipped to its cell, so the
 * flock gives up the last row plus the sprite height that would spill into it.
 * Every later derivation works off the reduced height, which is what keeps the
 * bands, the grid and the placement bounds consistent with it. */
static void reserve_legend_row(void) {
    screen.legend_row = -1;
    if (screen.rows < LEGEND_MIN_ROWS || screen.cols < LEGEND_NARROW_COLS) return;

    int height = (screen.rows - 1) * screen.cell_height - config.bird_size;
    if (height < screen.cell_height) return; /* Not enough left to fly in. */
    screen.legend_row = screen.rows - 1;
    screen.rows--;
    screen.height = height;
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
    reserve_legend_row();
    update_turn_distances();
}

static void update_screen_dimensions(void) {
    struct winsize size;
    memset(&size, 0, sizeof(size));
    if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &size) < 0) memset(&size, 0, sizeof(size));
    apply_screen_size(size.ws_col, size.ws_row, size.ws_xpixel, size.ws_ypixel);
}

static void build_rotation_frames(image_frame_t frames[ROTATION_FRAMES]) {
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
        uint8_t *encoded = NULL;
        size_t encoded_length = 0;
        double radians = i * FRAME_ANGLE * M_PI / 180.0;
        status = png_rotate_resize(&canvas, radians, config.bird_size, config.bird_size, &frame);
        if (status == PNG_OK) status = png_encode(&frame, &encoded, &encoded_length);
        png_image_free(&frame);
        if (status != PNG_OK) {
            fprintf(stderr, "Cannot build the rotation frames: %s\n", png_status_string(status));
            exit(EXIT_FAILURE);
        }
        frames[i].data = encoded;
        frames[i].length = encoded_length;
    }
    png_image_free(&canvas);
}

static kitty_graphics_status_t upload_rotation_frames(kitty_graphics_t *graphics,
                                                      const image_frame_t frames[ROTATION_FRAMES]) {
    for (int i = 0; i < ROTATION_FRAMES; i++) {
        kitty_graphics_status_t status =
            kitty_graphics_upload_png(graphics, (uint32_t)i + 1, frames[i].data, frames[i].length);
        if (status != KITTY_GRAPHICS_OK) return status;
    }
    kitty_graphics_status_t status = kitty_graphics_delete_all_placements(graphics);
    return status == KITTY_GRAPHICS_OK ? kitty_graphics_flush(graphics) : status;
}

static void free_rotation_frames(image_frame_t frames[ROTATION_FRAMES]) {
    for (int i = 0; i < ROTATION_FRAMES; i++) {
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

/* Spread over the region no turn band covers, so no bird starts by fleeing an
 * edge and the flock does not begin stacked on a single point. */
static void initialize_birds(bird_t *birds) {
    double min_x = screen.turn_x, max_x = screen.width - screen.turn_x;
    double min_y = screen.turn_y, max_y = screen.height - screen.turn_bottom;
    if (max_x <= min_x) min_x = max_x = screen.width / 2.0;
    if (max_y <= min_y) min_y = max_y = screen.height / 2.0;

    for (int i = 0; i < config.birds; i++) {
        bird_t *bird = &birds[i];
        bird->x = min_x + (max_x - min_x) * random_unit();
        bird->y = min_y + (max_y - min_y) * random_unit();
        bird->direction = 2 * M_PI * random_unit();
        bird->frame = direction_frame(bird->direction);
    }
}

static double normalized_angle(double y, double x) {
    double angle = atan2(y, x);
    return angle < 0 ? angle + 2 * M_PI : angle;
}

static vector_t boundary_vector(const bird_t *bird) {
    vector_t boundary = {0, 0};
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
    int neighbors = 0;
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
                separation.x += dx;
                separation.y += dy;
                alignment.x += cos(other->direction);
                alignment.y += sin(other->direction);
                cohesion.x += other->x;
                cohesion.y += other->y;
                neighbors++;
            }
        }
    }
    if (neighbors) {
        alignment.x /= neighbors;
        alignment.y /= neighbors;
        cohesion.x = cohesion.x / neighbors - target->x;
        cohesion.y = cohesion.y / neighbors - target->y;
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
        .image_id = (uint32_t)bird->frame + 1,
        .placement_id = 0,
        .row = row,
        .column = column,
        .x_offset = pixel_x % screen.cell_width,
        .y_offset = pixel_y % screen.cell_height,
        .z_index = 0,
    };
    return 1;
}

static int weights_are_default(void) {
    return config.boundary == DEFAULT_BOUNDARY_W && config.separation == DEFAULT_SEPARATION_W &&
           config.cohesion == DEFAULT_COHESION_W && config.alignment == DEFAULT_ALIGNMENT_W;
}

/* An Emacs mode line. The sigil follows the convention for a modified buffer:
 * dashes while the weights sit at their defaults, stars once one is touched.
 * Three widths, because truncating in the middle of a field reads as a glitch. */
static void build_legend(char *line, size_t size) {
    const char *sigil = weights_are_default() ? "-:---" : "-:**-";
    char text[LEGEND_TEXT_MAX];

    if (screen.cols >= LEGEND_WIDE_COLS)
        snprintf(text, sizeof(text),
                 "%s cbirds  %d boids  %dfps  (Boids)  b/B %.2f  s/S %.3f  c/C %.3f  a/A %.1f  "
                 "p/P %d  q quit",
                 sigil, config.birds, config.frame_rate, config.boundary, config.separation,
                 config.cohesion, config.alignment, config.vision_cells);
    else if (screen.cols >= LEGEND_MEDIUM_COLS)
        snprintf(text, sizeof(text),
                 "%s b/B %.2f  s/S %.3f  c/C %.3f  a/A %.1f  p/P %d  r/R %d  q quit", sigil,
                 config.boundary, config.separation, config.cohesion, config.alignment,
                 config.vision_cells, config.frame_rate);
    else
        snprintf(text, sizeof(text), "%s b%.2f s%.3f c%.3f a%.1f p%d r%d q", sigil, config.boundary,
                 config.separation, config.cohesion, config.alignment, config.vision_cells,
                 config.frame_rate);

    /* Never wider than the viewport: a wrapped mode line would scroll the flock
     * off the top of the screen. The erase to end of line paints the rest of the
     * bar in the reversed background, so there is nothing to pad. */
    if (strlen(text) > (size_t)screen.cols) text[screen.cols] = '\0';
    snprintf(line, size, "\033[7m\033[K%s\033[0m", text);
}

static kitty_graphics_status_t queue_legend(kitty_graphics_t *graphics) {
    char line[LEGEND_LINE_MAX];

    /* A resize moves the bar, and text is not swept away by the per frame
     * placement clear: the row it used to sit on has to be erased by hand.
     * Only that row, an erase of the whole screen would delete the uploaded
     * sprites along with it and leave every later placement pointing at
     * nothing. */
    if (drawn_legend_row >= 0 && drawn_legend_row != screen.legend_row) {
        kitty_graphics_status_t status =
            kitty_graphics_write_text(graphics, drawn_legend_row, 0, "\033[K");
        if (status != KITTY_GRAPHICS_OK) return status;
        drawn_legend_row = -1;
    }
    if (screen.legend_row < 0) return KITTY_GRAPHICS_OK;

    build_legend(line, sizeof(line));
    kitty_graphics_status_t status =
        kitty_graphics_write_text(graphics, screen.legend_row, 0, line);
    if (status == KITTY_GRAPHICS_OK) drawn_legend_row = screen.legend_row;
    return status;
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

static void update_vision_radius(void) {
    config.vision_radius = config.vision_cells * SPATIAL_CELL_SIZE;
    config.vision_radius_squared = config.vision_radius * config.vision_radius;
}

static int handle_input(void) {
    enum { INPUT_NORMAL, INPUT_ESCAPE, INPUT_SEQUENCE };
    static int input_state = INPUT_NORMAL;
    char input[INPUT_BUFFER_SIZE];
    ssize_t length = read(STDIN_FILENO, input, sizeof(input));
    for (ssize_t i = 0; i < length; i++) {
        unsigned char key = (unsigned char)input[i];
        if (input_state == INPUT_ESCAPE) {
            if (key == '[' || key == 'O')
                input_state = INPUT_SEQUENCE;
            else if (key != '\033')
                input_state = INPUT_NORMAL;
            continue;
        }
        if (input_state == INPUT_SEQUENCE) {
            if (key >= 0x40 && key <= 0x7e) input_state = INPUT_NORMAL;
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
                if (config.boundary < BOUNDARY_MAX) {
                    config.boundary += BOUNDARY_STEP;
                    if (config.boundary > BOUNDARY_MAX) config.boundary = BOUNDARY_MAX;
                }
                break;
            case 'b':
                if (config.boundary > BOUNDARY_MIN) {
                    config.boundary -= BOUNDARY_STEP;
                    if (config.boundary < BOUNDARY_MIN) config.boundary = BOUNDARY_MIN;
                }
                break;
            case 'S':
                if (config.separation < SEPARATION_MAX) {
                    config.separation += SEPARATION_STEP;
                    if (config.separation > SEPARATION_MAX) config.separation = SEPARATION_MAX;
                }
                break;
            case 's':
                if (config.separation > SEPARATION_MIN) {
                    config.separation -= SEPARATION_STEP;
                    if (config.separation < SEPARATION_MIN) config.separation = SEPARATION_MIN;
                }
                break;
            case 'C':
                if (config.cohesion < COHESION_MAX) {
                    config.cohesion += COHESION_STEP;
                    if (config.cohesion > COHESION_MAX) config.cohesion = COHESION_MAX;
                }
                break;
            case 'c':
                if (config.cohesion > COHESION_MIN) {
                    config.cohesion -= COHESION_STEP;
                    if (config.cohesion < COHESION_MIN) config.cohesion = COHESION_MIN;
                }
                break;
            case 'A':
                if (config.alignment < ALIGNMENT_MAX) {
                    config.alignment += ALIGNMENT_STEP;
                    if (config.alignment > ALIGNMENT_MAX) config.alignment = ALIGNMENT_MAX;
                }
                break;
            case 'a':
                if (config.alignment > ALIGNMENT_MIN) {
                    config.alignment -= ALIGNMENT_STEP;
                    if (config.alignment < ALIGNMENT_MIN) config.alignment = ALIGNMENT_MIN;
                }
                break;
            case 'R':
                if (config.frame_rate < MAX_FRAME_RATE) {
                    config.frame_rate += FRAME_RATE_STEP;
                    if (config.frame_rate > MAX_FRAME_RATE) config.frame_rate = MAX_FRAME_RATE;
                    update_speed();
                }
                break;
            case 'r':
                if (config.frame_rate > MIN_FRAME_RATE) {
                    config.frame_rate -= FRAME_RATE_STEP;
                    if (config.frame_rate < MIN_FRAME_RATE) config.frame_rate = MIN_FRAME_RATE;
                    update_speed();
                }
                break;
            case 'P':
                if (config.vision_cells < MAX_VISION_CELLS) config.vision_cells++;
                break;
            case 'p':
                if (config.vision_cells > MIN_VISION_CELLS) config.vision_cells--;
                break;
            default:
                continue;
        }
        update_vision_radius();
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

static void usage(const char *program) {
    fprintf(stderr,
            "Usage: %s [-n BIRDS] [-f FPS] [-s SIZE]\n"
            "  -n NUMBER    number of boids (default 800, max %d)\n"
            "  -f FPS       frame rate (default %d, from %d to %d)\n"
            "  -s SIZE      bird size in pixels (default %d, from %d to %d)\n"
            "  -h           show this help\n",
            program, MAX_BIRDS, DEFAULT_FRAME_RATE, MIN_FRAME_RATE, MAX_FRAME_RATE,
            DEFAULT_BIRD_SIZE, MIN_BIRD_SIZE, MAX_BIRD_SIZE);
}

static void read_options(int argc, char **argv) {
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) {
            usage(argv[0]);
            exit(EXIT_SUCCESS);
        }
        if (strlen(argv[i]) != 2 || argv[i][0] != '-' || !strchr("nfs", argv[i][1])) {
            fprintf(stderr, "Unknown option: %s\n", argv[i]);
            usage(argv[0]);
            exit(EXIT_FAILURE);
        }
        char option = argv[i][1];
        if (++i >= argc) {
            fprintf(stderr, "Missing value for -%c\n", option);
            usage(argv[0]);
            exit(EXIT_FAILURE);
        }
        char *end;
        errno = 0;
        long value = strtol(argv[i], &end, 10);
        if (errno == ERANGE || end == argv[i] || *end || value <= 0) {
            fprintf(stderr, "Invalid value for -%c: %s\n", option, argv[i]);
            exit(EXIT_FAILURE);
        }
        if (option == 'n') {
            if (value > MAX_BIRDS) {
                fprintf(stderr, "Birds number capped to %d\n", MAX_BIRDS);
                value = MAX_BIRDS;
            }
            config.birds = (int)value;
        } else if (option == 'f') {
            if (value < MIN_FRAME_RATE || value > MAX_FRAME_RATE) {
                fprintf(stderr, "Frame rate must be between %d and %d\n", MIN_FRAME_RATE,
                        MAX_FRAME_RATE);
                exit(EXIT_FAILURE);
            }
            config.frame_rate = (int)value;
            update_speed();
        } else {
            if (value < MIN_BIRD_SIZE || value > MAX_BIRD_SIZE) {
                fprintf(stderr, "Bird size must be between %d and %d\n", MIN_BIRD_SIZE,
                        MAX_BIRD_SIZE);
                exit(EXIT_FAILURE);
            }
            config.bird_size = (int)value;
        }
    }
}

static long elapsed_microseconds(const struct timespec *start, const struct timespec *end) {
    return (end->tv_sec - start->tv_sec) * 1000000L + (end->tv_nsec - start->tv_nsec) / 1000L;
}

int main(int argc, char **argv) {
    image_frame_t frames[ROTATION_FRAMES] = {0};
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
    build_rotation_frames(frames);

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
    graphics_status = upload_rotation_frames(&graphics, frames);
    free_rotation_frames(frames);
    if (graphics_status != KITTY_GRAPHICS_OK) {
        fprintf(stderr, "Cannot upload Kitty graphics: %s\n",
                kitty_graphics_status_string(graphics_status));
        exit(EXIT_FAILURE);
    }

    int running = 1;
    while (running) {
        running = handle_input();
        if (!running) break;

        clock_gettime(CLOCK_MONOTONIC, &frame_start);
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
