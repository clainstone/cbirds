/* Feature test macros must precede every include */
#define _XOPEN_SOURCE 700
#define _DEFAULT_SOURCE
#define _DARWIN_C_SOURCE

#include <errno.h>
#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#define ROTATION_FRAME 90                  /*Number of roation frame*/
#define FRAME_ANGLE (360 / ROTATION_FRAME) /*Difference in degrees from ajacents rotation frames*/
#define BASE_IMAGE_SIZE 5
#define IMAGE_SIZES 40
#define PNG_FORMAT 100 /*Kitty's protocol png escape code*/
#define DEF_TERMINAL_COLS 80
#define DEF_TERMINAL_ROWS 24
#define DEF_CELL_WIDTH 8 /*Cell size assumed when the terminal reports no pixel size*/
#define DEF_CELL_HEIGHT 16
#define X_START_OFF 20
#define Y_START_OFF 20
#define INPUT_BUF_DIM 100
#define BIRD_ESCAPE_DIM 150 /*Upper bound of a single placement escape sequence*/
#define MAX_BIRDS 200000
#define MAX_FRAME_RATE 1000
#define PATH_DIM 256

/*Alternate screen and cursor control, used instead of shelling out to tput*/
#define ALT_SCREEN_ON "\033[?1049h"
#define ALT_SCREEN_OFF "\033[?1049l"
#define CURSOR_HIDE "\033[?25l"
#define CURSOR_SHOW "\033[?25h"

/*=========================== Simulation parameters ===============================*/

const int DEF_FRAME_RATE = 60; /*Default frame rate in case no one is specified*/
const int DEF_SPEED = 40;      /*Pixels per frame at DEF_FRAME_RATE*/
const int DEF_PERCEPTION_RADIUS = 35;

/* Runtime weights modification steps*/
const double boundary_av_st = 0.02;
const double alignment_st = 0.1;
const double separation_st = 0.001;
const double cohesion_st = 0.002;
const int frame_rate_st = 5;
const int perception_radius_st = 3;

const double boundary_av_min = 0.01;
const double alignment_min = 0.1;
const double separation_min = 0.001;
const double cohesion_min = 0.002;
const int perception_radius_min = 3;

int BIRDS_N = 800;   /*Birds number*/
int FRAME_RATE = 60; /*Frames per second*/
int TURN_RADIUS_X;   /*Border distance within the bird starts to steer to avoid the collision*/
int TURN_RADIUS_Y;
int SPEED = 40;     /*Pixels increment between two frames*/
int BIRD_SIZE = 15; /*Bird size in pixels*/
int PERCEPTION_RADIUS =
    DEF_PERCEPTION_RADIUS; /*The maximum distance whereas two boids can interacts*/
int PERCEPTION_RADIUS_SQUARED = DEF_PERCEPTION_RADIUS * DEF_PERCEPTION_RADIUS;

/*Animation weights, see https://en.wikipedia.org/wiki/Boids */
double SEPARATION_W = 0.005;
double ALIGNMENT_W = 1.5;
double COHESION_W = 0.01;
double BOUNDARY_AV_W = 0.2;

/*=================================================================================*/

static enum { RESET, RAW } ttystate = RESET; /*Terminal state : RAW, NORMAL*/

typedef int rotation_frame_id_t; /*The index that defines the id of the rotation frame*/

typedef struct {
    int id, width, heigth, speed;
    double direction, x, y;
} bird_t;

typedef struct {
    double x, y;
} vector2d_t;

typedef struct {
    rotation_frame_id_t prev_id;
    rotation_frame_id_t curr_id;
    bird_t *bird_ref;
} drawn_bird_t;

/*base16 to base64 lookup*/
const char base64_chars[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

const char *resources_root = "resources"; /*Directory holding the dimNN sprite folders*/
int screen_width;
int screen_heigth;
int n_col;
int n_row;
int character_width_p;  /*character pixel width*/
int character_height_p; /*character pixel heigth*/
size_t output_buf_off =
    0; /*Offset within the outbuffer used to concatenate escape control strings*/
size_t output_buf_size = 0;
struct termios saved_termios; /*Saved termios structure to be resumed after process termination*/
static volatile sig_atomic_t terminal_restored = 0;

/*============================================================================================*/

bird_t *init_bird(int id, int width, int heigth, int screen_width, int screen_heigth);

double calculate_rules_direction(bird_t *bird, bird_t **birds, int num_birds, int screen_width,
                                 int screen_heigth);
double my_atan2(double y, double x);
double squared_distance(bird_t *b1, bird_t *b2);

int to_degrees(double radians);
int enable_raw_mode();
int my_atenter();

uint8_t *base64_encode(const uint8_t *input, size_t input_length);

vector2d_t calculate_boundary_av_direction(bird_t *bird, int screen_width, int screen_heigth);

void init_rotation_frames(uint8_t **images_data_array);
void find_resources_root();
void get_image_path(char *path, size_t path_size, int size_index, int rotation_frame_id);
void init_birds(drawn_bird_t **birds_array, uint8_t **images_data_array, int screen_width,
                int screen_heigth);
void clean_screen();
void delete_placements();
void print_bird(drawn_bird_t **birds_array, int bird_no, char *output_buf);
void update_rotation_frame_id(drawn_bird_t **birds_array);
void init(char **output_buf, uint8_t **images_data, drawn_bird_t **draw_birds, bird_t **birds,
          bird_t **birds_copy);
void send_payload_data(uint8_t **payload_data);
void get_screen_dimensions();
void fix_weights();
void update_speed();
void update_birds(bird_t **birds_copy_to_read, bird_t **birds_to_write, int screen_width,
                  int screen_height, int birds_num);
void update_direction(bird_t *bird, double next_direction);
void add_vector(vector2d_t *vector, double x, double y);
void prod_vector(vector2d_t *vector, double scalar);
void init_vector(vector2d_t *vector, double x, double y);
void close_birds(bird_t **close_birds_list, bird_t *target, bird_t **birds, int num_birds,
                 int *counter);
void restore_terminal();
void my_atexit();
void install_signal_handlers();
void refresh_screen(char **output_buf, drawn_bird_t **draw_birds, bird_t **birds,
                    bird_t **birds_copy);
void handle_key(uint8_t **images_data);
void read_input(int argc, char **argv);
void usage(const char *prog);
void change_birds_dimensions(bool increase, uint8_t **bird_images);
void full_write(const char *data, size_t len);
void copy(bird_t **original, bird_t **copy, int birds_num);
void clear();

//=======================Low level terminal handling===========================

/*Writes the whole buffer, resuming on partial writes and on EINTR*/
void full_write(const char *data, size_t len) {
    size_t written = 0;
    while (written < len) {
        ssize_t n = write(STDOUT_FILENO, data + written, len - written);
        if (n < 0) {
            if (errno == EINTR) continue;
            return;
        }
        written += (size_t)n;
    }
}

void get_screen_dimensions() {
    struct winsize w;
    memset(&w, 0, sizeof(w));
    if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &w) < 0) memset(&w, 0, sizeof(w));

    n_col = w.ws_col > 0 ? w.ws_col : DEF_TERMINAL_COLS;
    n_row = w.ws_row > 0 ? w.ws_row : DEF_TERMINAL_ROWS;
    screen_width = w.ws_xpixel;
    screen_heigth = w.ws_ypixel;

    /*Many terminals report no pixel size at all : derive it from the cell grid
     * so that the per character size never ends up being zero.*/
    if (screen_width <= 0 || screen_heigth <= 0) {
        screen_width = n_col * DEF_CELL_WIDTH;
        screen_heigth = n_row * DEF_CELL_HEIGHT;
    }

    character_width_p = screen_width / n_col;
    character_height_p = screen_heigth / n_row;
    if (character_width_p < 1) character_width_p = 1;
    if (character_height_p < 1) character_height_p = 1;

    fix_weights();
}

int my_atenter() {
    /*Enable alternate buffer and hide the cursor*/
    full_write(ALT_SCREEN_ON, strlen(ALT_SCREEN_ON));
    full_write(CURSOR_HIDE, strlen(CURSOR_HIDE));
    return enable_raw_mode();
}

/* Raw mode : 1960 magic shit */
int enable_raw_mode() {
    struct termios buf;
    if (ttystate != RESET) {
        errno = EINVAL;
        return -1;
    }
    if (tcgetattr(STDIN_FILENO, &buf) < 0) return -1;
    saved_termios = buf;
    /* input modes: no break, no CR to NL, no parity check, no strip char,
     * no start/stop output control. */
    buf.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
    /* output modes - disable post processing */
    buf.c_oflag &= ~(OPOST);
    /* control modes - set 8 bit chars */
    buf.c_cflag |= (CS8);
    /* local modes - echoing off, canonical off, no extended functions.
     * ISIG is kept on purpose so that Ctrl+C still quits : the signal handlers
     * put the terminal back in a sane state before exiting. */
    buf.c_lflag &= ~(ECHO | ICANON | IEXTEN);
    /* Ctrl+Z would suspend the process leaving the terminal in raw mode */
    buf.c_cc[VSUSP] = _POSIX_VDISABLE;
    /* non blocking read : return immediately with whatever is available */
    buf.c_cc[VMIN] = 0;
    buf.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSAFLUSH, &buf) < 0) return -1;
    ttystate = RAW;
    return 0;
}

/*Idempotent, async signal safe enough to be called from a signal handler*/
void restore_terminal() {
    if (terminal_restored) return;
    terminal_restored = 1;
    if (ttystate == RAW) {
        tcsetattr(STDIN_FILENO, TCSAFLUSH, &saved_termios);
        ttystate = RESET;
    }
    full_write(CURSOR_SHOW, strlen(CURSOR_SHOW));
    full_write(ALT_SCREEN_OFF, strlen(ALT_SCREEN_OFF));
}

void my_atexit() {
    restore_terminal();
}

static void signal_handler(int sig) {
    restore_terminal();
    _exit(128 + sig);
}

/*Without this a crash or a Ctrl+C would leave the terminal in raw mode,
 * inside the alternate screen and without echo.*/
void install_signal_handlers() {
    struct sigaction sa;
    const int signals[] = {SIGINT, SIGTERM, SIGHUP, SIGQUIT, SIGSEGV, SIGFPE, SIGBUS, SIGABRT};

    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = signal_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_RESETHAND;

    for (size_t i = 0; i < sizeof(signals) / sizeof(signals[0]); i++)
        sigaction(signals[i], &sa, NULL);
}

//========================Image data manipulation==============================

/*Picks the first directory that actually holds the sprites, so that the binary
 * can be run from the project root, from a build subdirectory or from anywhere
 * else by exporting CBIRDS_RESOURCES.*/
void find_resources_root() {
    static const char *candidates[] = {"resources", "../resources", NULL};
    const char *env = getenv("CBIRDS_RESOURCES");
    char path[PATH_DIM];

    if (env != NULL && *env != '\0') {
        resources_root = env;
        return;
    }
    for (int i = 0; candidates[i] != NULL; i++) {
        snprintf(path, sizeof(path), "%s/dim%d/bird_0.png", candidates[i], BASE_IMAGE_SIZE);
        if (access(path, R_OK) == 0) {
            resources_root = candidates[i];
            return;
        }
    }
    fprintf(stderr,
            "Cannot find the sprite directory (looked for ./resources and ../resources).\n"
            "Run cbirds from the project root or set CBIRDS_RESOURCES.\n");
    exit(EXIT_FAILURE);
}

void init_rotation_frames(uint8_t **images_data_array) {
    char path[PATH_DIM];
    int size_index;
    int bird_index;

    for (size_index = 0; size_index < IMAGE_SIZES; size_index++) {
        for (bird_index = 0; bird_index < ROTATION_FRAME; bird_index++) {
            get_image_path(path, sizeof(path), size_index + BASE_IMAGE_SIZE, bird_index);
            FILE *file = fopen(path, "rb");
            if (file == NULL) {
                perror("Error during file opening");
                exit(EXIT_FAILURE);
            }
            if (fseek(file, 0, SEEK_END) < 0) {
                perror("Error during file seeking");
                fclose(file);
                exit(EXIT_FAILURE);
            }
            long size = ftell(file);
            if (size <= 0) {
                fprintf(stderr, "Empty or unreadable image : %s\n", path);
                fclose(file);
                exit(EXIT_FAILURE);
            }
            rewind(file);

            uint8_t *buf = (uint8_t *)malloc((size_t)size);
            if (buf == NULL) {
                perror("Out of memory while loading images");
                fclose(file);
                exit(EXIT_FAILURE);
            }
            if (fread(buf, 1, (size_t)size, file) != (size_t)size) {
                perror("Error during file reading");
                free(buf);
                fclose(file);
                exit(EXIT_FAILURE);
            }

            images_data_array[size_index * ROTATION_FRAME + bird_index] =
                base64_encode(buf, (size_t)size);
            free(buf);
            fclose(file);
        }
    }
}

/*
 * Encodes input to base64 adding padding characters if necessary. The number of
 * bytes of the returned array of char is multiple of 4
 * */
uint8_t *base64_encode(const uint8_t *input, size_t input_length) {
    uint8_t char_array_3[3];
    uint8_t char_array_4[4];
    size_t output_size = ((input_length + 2) / 3) * 4;
    uint8_t *output = (uint8_t *)malloc((output_size + 1) * sizeof(uint8_t));
    int i = 0;
    int chunk_count = 0;

    if (output == NULL) {
        perror("Out of memory while encoding images");
        exit(EXIT_FAILURE);
    }

    while (input_length >= 3) {
        input_length -= 3;
        char_array_3[0] = input[i];
        char_array_3[1] = input[i + 1];
        char_array_3[2] = input[i + 2];

        char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
        char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
        char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
        char_array_4[3] = char_array_3[2] & 0x3f;

        for (size_t j = 0; j < 4; j++) {
            output[chunk_count * 4 + j] = base64_chars[char_array_4[j]];
        }
        i += 3;
        chunk_count++;
    }

    if (input_length > 0) {
        int out_idx = 4 * chunk_count;

        if (input_length == 1) {
            char_array_3[0] = input[i];
            char_array_3[1] = 0;
            char_array_3[2] = 0;
        } else {
            char_array_3[0] = input[i];
            char_array_3[1] = input[i + 1];
            char_array_3[2] = 0;
        }
        char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
        char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
        char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
        char_array_4[3] = char_array_3[2] & 0x3f;

        output[out_idx++] = base64_chars[char_array_4[0]];
        output[out_idx++] = base64_chars[char_array_4[1]];

        if (input_length == 2) {
            output[out_idx++] = base64_chars[char_array_4[2]];
            output[out_idx++] = '=';
        } else {
            output[out_idx++] = '=';
            output[out_idx++] = '=';
        }
    }
    output[output_size] = '\0';

    return output;
}

void get_image_path(char *path, size_t path_size, int size_index, int rotation_frame_id) {
    snprintf(path, path_size, "%s/dim%d/bird_%d.png", resources_root, size_index,
             rotation_frame_id);
}

/*====================Graphical protocol escapes handling=====================
 *
 * In order to send data to the terminal emulator, images needs to be encoded
 * base64 in multiple of 4 bytes. Paylod data are sent once before the main
 * loop, then position and direction updates are sent for every frame specifying
 * new parameters without sending again the entire payload. */

void send_payload_data(uint8_t **images_data) {
    int image_size_index = BIRD_SIZE - BASE_IMAGE_SIZE;
    for (int i = 0; i < ROTATION_FRAME; i++) {
        printf("\033_Ga=t,q=2,f=%d,I=%d;%s\033\\", PNG_FORMAT, i + 1,
               (char *)images_data[ROTATION_FRAME * image_size_index + i]);
    }
    clean_screen();
    fflush(stdout);
}

/* Sends only deltas about position and direction.
 * Every rotated image has an index(I), every bird is assigned to a frame index
 * defining his placement_index(p), there can be multiple birds(with different
 * placement_index) assigned to the same image index.
 * */
void print_bird(drawn_bird_t **birds_array, int bird_no, char *output_buf) {
    char buf[BIRD_ESCAPE_DIM];
    drawn_bird_t *bird = birds_array[bird_no];

    int col, row, offset_x, offset_y;

    col = (int)bird->bird_ref->x / character_width_p;
    row = (int)bird->bird_ref->y / character_height_p;
    offset_x = (int)bird->bird_ref->x % character_width_p;
    offset_y = (int)bird->bird_ref->y % character_height_p;

    if (col >= 0 && col < n_col && row >= 0 && row < n_row) {
        rotation_frame_id_t id = bird->curr_id;
        int len =
            snprintf(buf, sizeof(buf), "\033[%d;%dH\033_Ga=p,I=%d,q=2,p=%d,X=%d,Y=%d,z=%d\033\\",
                     row + 1, col + 1, id + 1, 0, offset_x, offset_y, bird_no);
        /*Every escape sequence is concatened to the outpute buffer that is
         * flushed output once a frame*/
        if (len > 0 && output_buf_off + (size_t)len < output_buf_size) {
            memcpy(output_buf + output_buf_off, buf, (size_t)len);
            output_buf_off += (size_t)len;
        }
    }
}

/*Deletes all visible placements*/
void clean_screen() {
    printf("\033_Ga=d,d=a\033\\");
}

/*Deletes every cached placement*/
void delete_placements() {
    printf("\033_Ga=d,d=A\033\\");
}

/*=======================Birds behaviour logic==========================*/

void init(char **output_buf, uint8_t **images_data, drawn_bird_t **draw_birds, bird_t **birds,
          bird_t **birds_copy) {
    get_screen_dimensions();
    output_buf_size = (size_t)BIRD_ESCAPE_DIM * (size_t)BIRDS_N + 1;
    *output_buf = (char *)malloc(output_buf_size);
    if (*output_buf == NULL) {
        perror("Out of memory");
        exit(EXIT_FAILURE);
    }
    (*output_buf)[0] = '\0';
    for (int i = 0; i < BIRDS_N; i++) {
        draw_birds[i] = (drawn_bird_t *)malloc(sizeof(drawn_bird_t));
        birds_copy[i] = (bird_t *)malloc(sizeof(bird_t));
        if (draw_birds[i] == NULL || birds_copy[i] == NULL) {
            perror("Out of memory");
            exit(EXIT_FAILURE);
        }
    }

    init_birds(draw_birds, images_data, screen_width, screen_heigth);
    for (int i = 0; i < BIRDS_N; i++) {
        birds[i] = draw_birds[i]->bird_ref;
    }
}

void init_birds(drawn_bird_t **birds_array, uint8_t **images_data_array, int screen_width,
                int screen_heigth) {
    for (int i = 0; i < BIRDS_N; i++) {
        birds_array[i]->bird_ref = init_bird(i, BIRD_SIZE, BIRD_SIZE, screen_width, screen_heigth);
        birds_array[i]->curr_id = to_degrees(birds_array[i]->bird_ref->direction) / FRAME_ANGLE;
        birds_array[i]->prev_id = birds_array[i]->curr_id;
    }
    init_rotation_frames(images_data_array);
}

/**
 * Bird constructor. Initializes bird direction, x and y coordinates as random
 * values.
 */
bird_t *init_bird(int id, int width, int heigth, int screen_width, int screen_heigth) {
    bird_t *bird = (bird_t *)malloc(sizeof(bird_t));

    if (bird == NULL) {
        perror("Out of memory");
        exit(EXIT_FAILURE);
    }

    /*Kept inside the screen : the start offset is applied to the usable range,
     * not added on top of it.*/
    bird->x = X_START_OFF + (screen_width - 2 * X_START_OFF) * ((double)rand() / RAND_MAX);
    bird->y = Y_START_OFF + (screen_heigth - 2 * Y_START_OFF) * ((double)rand() / RAND_MAX);
    bird->direction = 2 * M_PI * ((double)rand() / RAND_MAX);
    bird->id = id;
    bird->speed = SPEED;
    bird->width = width;
    bird->heigth = heigth;

    /*Avoids blocked startin position*/
    if (bird->x < TURN_RADIUS_X || bird->x > screen_width - TURN_RADIUS_X)
        bird->x = screen_width / 2;
    if (bird->y < TURN_RADIUS_Y || bird->y > screen_heigth - TURN_RADIUS_Y)
        bird->y = screen_heigth / 2;

    return bird;
}

void update_birds(bird_t **birds_copy_to_read, bird_t **birds_to_write, int screen_width,
                  int screen_height, int birds_num) {
    bird_t **close = (bird_t **)malloc(sizeof(bird_t *) * (size_t)birds_num);

    if (close == NULL) {
        perror("Out of memory");
        exit(EXIT_FAILURE);
    }
    for (int i = 0; i < birds_num; i++) {
        int counter = 0;
        close_birds(close, birds_copy_to_read[i], birds_copy_to_read, birds_num, &counter);
        /*Also called with no neighbour at all : a lonely bird still has to keep
         * flying and to steer away from the borders.*/
        double direction = calculate_rules_direction(birds_copy_to_read[i], close, counter,
                                                     screen_width, screen_height);
        update_direction(birds_to_write[i], direction);
    }
    free(close);
}

/*Updates the bird frame_id according to his new direction*/
void update_rotation_frame_id(drawn_bird_t **birds_array) {
    for (int i = 0; i < BIRDS_N; i++) {
        birds_array[i]->prev_id = birds_array[i]->curr_id;
        birds_array[i]->curr_id = to_degrees(birds_array[i]->bird_ref->direction) / FRAME_ANGLE;
    }
}

/**
 * Calculates how many birds are flying around the target between the given
 * radius,
 * **close_birds_list is then filled whit those birds
 */
void close_birds(bird_t **close_birds_list, bird_t *target, bird_t **birds, int num_birds,
                 int *counter) {
    int current_index = 0;
    for (int i = 0; i < num_birds; i++) {
        bird_t *boid = birds[i];
        if (boid->id != target->id) {
            if (squared_distance(target, boid) < PERCEPTION_RADIUS_SQUARED) {
                close_birds_list[current_index++] = boid;
            }
        }
    }
    *counter = current_index;
}

/**
 * Calculates the steering vector of the given bird for border avoidance
 * only if is closer than radius.
 */
vector2d_t calculate_boundary_av_direction(bird_t *bird, int screen_width, int screen_heigth) {
    vector2d_t boundary_av;
    int bottom_mult = 100000;
    int bottom_off = 100;

    init_vector(&boundary_av, 0, 0);
    if (bird->x < TURN_RADIUS_X) {
        add_vector(&boundary_av, 1, 0);
    } else if (bird->x > screen_width - TURN_RADIUS_X) {
        add_vector(&boundary_av, -1, 0);
    }
    if (bird->y < TURN_RADIUS_Y) {
        add_vector(&boundary_av, 0, 1);
    } else if (bird->y > screen_heigth - bottom_off) {
        add_vector(&boundary_av, 0, -1 * bottom_mult);
    }

    return boundary_av;
}

/**
 * Calculates the steering vector calculating as the sum of four different ones:
 *
 * Separation : steer vector to avoid crowding local birds
 * Alignment : steer vector that is the mean of the steer vector of local birds
 * Cohesion : steer vector used to move towards local birds
 * Border avoidance : steer vector used to remain between borders
 * */
double calculate_rules_direction(bird_t *target, bird_t **birds, int num_birds, int screen_width,
                                 int screen_heigth) {
    vector2d_t separation = {0, 0};
    vector2d_t alignment = {0, 0};
    vector2d_t cohesion = {0, 0};

    vector2d_t boundary_av_ptr =
        calculate_boundary_av_direction(target, screen_width, screen_heigth);

    int close_count = 0;  // Calculate only if there are some birds nearby

    for (int i = 0; i < num_birds; i++) {
        bird_t *boid = birds[i];

        // Before normalization: sum of vectors obtained based on criterias
        add_vector(&separation, target->x - boid->x, target->y - boid->y);
        add_vector(&alignment, cos(boid->direction), sin(boid->direction));
        add_vector(&cohesion, boid->x, boid->y);
        close_count++;
    }

    if (close_count > 0) {
        // Normalization
        alignment.x /= close_count;
        alignment.y /= close_count;
        cohesion.x /= close_count;
        cohesion.y /= close_count;

        // Now cohesion is the vector from the target to the center of mass
        cohesion.x -= target->x;
        cohesion.y -= target->y;

        // Weights refining
        prod_vector(&separation, SEPARATION_W);
        prod_vector(&alignment, ALIGNMENT_W);
        prod_vector(&cohesion, COHESION_W);
        prod_vector(&boundary_av_ptr, BOUNDARY_AV_W);

        double result_x = separation.x + alignment.x + cohesion.x + boundary_av_ptr.x;
        double result_y = separation.y + alignment.y + cohesion.y + boundary_av_ptr.y;

        if (result_x == 0 && result_y == 0) return target->direction;

        return my_atan2(result_y, result_x);
    }

    /*No bird nearby : keep the current heading unless a border is close*/
    prod_vector(&boundary_av_ptr, BOUNDARY_AV_W);
    if (boundary_av_ptr.x != 0 || boundary_av_ptr.y != 0) {
        double result_x = cos(target->direction) + boundary_av_ptr.x;
        double result_y = sin(target->direction) + boundary_av_ptr.y;
        if (result_x != 0 || result_y != 0) return my_atan2(result_y, result_x);
    }
    return target->direction;
}

void update_direction(bird_t *bird, double next_direction) {
    bird->direction = next_direction;
    bird->speed = SPEED;
    bird->x += (double)bird->speed * cos(next_direction);
    bird->y += (double)bird->speed * sin(next_direction);
}

int to_degrees(double radians) {
    int deg = (int)(radians * (180.0 / M_PI));  // Angle values are between 0 and 360 deg
    return (deg % 360 + 360) % 360;
}

void clear() {
    printf("\x1b[J");
}

void init_vector(vector2d_t *vector, double x, double y) {
    vector->x = x;
    vector->y = y;
}

void add_vector(vector2d_t *vector, double x, double y) {
    vector->x += x;
    vector->y += y;
}

void prod_vector(vector2d_t *vector, double scalar) {
    vector->x *= scalar;
    vector->y *= scalar;
}

double squared_distance(bird_t *b1, bird_t *b2) {
    return ((b1->x - b2->x) * (b1->x - b2->x) + (b1->y - b2->y) * (b1->y - b2->y));
}

double my_atan2(double y, double x) {
    double angle = atan2(y, x);
    if (angle < 0.0) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

/*
 * Copies the array to perform the update calculation on a single immutable
 * version of the state of birds.
 * */
void copy(bird_t **original, bird_t **copy, int birds_num) {
    for (int i = 0; i < birds_num; i++) {
        copy[i]->direction = original[i]->direction;
        copy[i]->heigth = original[i]->heigth;
        copy[i]->id = original[i]->id;
        copy[i]->speed = original[i]->speed;
        copy[i]->width = original[i]->width;
        copy[i]->x = original[i]->x;
        copy[i]->y = original[i]->y;
    }
}

void fix_weights() {
    const int factor = 3;
    TURN_RADIUS_X = screen_width / factor;
    TURN_RADIUS_Y = screen_heigth / factor;
}

/*Keeps the travelled distance per second constant across frame rates*/
void update_speed() {
    SPEED = (int)((double)DEF_SPEED * DEF_FRAME_RATE / FRAME_RATE);
    if (SPEED < 1) SPEED = 1;
}

/*Handles raw mode input keys*/
void handle_key(uint8_t **images_data) {
    char input_buf[INPUT_BUF_DIM];
    ssize_t size;

    size = read(STDIN_FILENO, (void *)input_buf, INPUT_BUF_DIM);
    if (size <= 0) return;

    /*Every byte of the burst is handled, not only the first one*/
    for (ssize_t i = 0; i < size; i++) {
        char c = input_buf[i];

        switch (c) {
            case 'q': /*quit*/
                exit(0);
                break;
            case '=': /*increase bird image size*/
                if (BIRD_SIZE < IMAGE_SIZES + BASE_IMAGE_SIZE - 1)
                    change_birds_dimensions(true, images_data);
                break;
            case '-': /*decrease bird image size*/
                if (BIRD_SIZE > BASE_IMAGE_SIZE) change_birds_dimensions(false, images_data);
                break;
            case 'B': /*increase boundary_av*/
                BOUNDARY_AV_W += boundary_av_st;
                break;
            case 'b': /*decrease boundary_av*/
                if (BOUNDARY_AV_W - boundary_av_st >= boundary_av_min)
                    BOUNDARY_AV_W -= boundary_av_st;
                break;
            case 'S': /*increase separation*/
                SEPARATION_W += separation_st;
                break;
            case 's': /*decrease separation*/
                if (SEPARATION_W - separation_st >= separation_min) SEPARATION_W -= separation_st;
                break;
            case 'C': /*increase cohesion*/
                COHESION_W += cohesion_st;
                break;
            case 'c': /*decrease cohesion*/
                if (COHESION_W - cohesion_st >= cohesion_min) COHESION_W -= cohesion_st;
                break;
            case 'A': /*increase alignment*/
                ALIGNMENT_W += alignment_st;
                break;
            case 'a': /*decrease alignment*/
                if (ALIGNMENT_W - alignment_st >= alignment_min) ALIGNMENT_W -= alignment_st;
                break;
            case 'R': /*increase frame rate*/
                if (FRAME_RATE + frame_rate_st <= MAX_FRAME_RATE) {
                    FRAME_RATE += frame_rate_st;
                    update_speed();
                }
                break;
            case 'r': /*decrease frame rate*/
                if (FRAME_RATE - frame_rate_st > 0) {
                    FRAME_RATE -= frame_rate_st;
                    update_speed();
                }
                break;
            case 'P': /*increase perception radius*/
                PERCEPTION_RADIUS += perception_radius_st;
                PERCEPTION_RADIUS_SQUARED = PERCEPTION_RADIUS * PERCEPTION_RADIUS;
                break;
            case 'p': /*decrease perception radius*/
                if (PERCEPTION_RADIUS - perception_radius_st >= perception_radius_min) {
                    PERCEPTION_RADIUS -= perception_radius_st;
                    PERCEPTION_RADIUS_SQUARED = PERCEPTION_RADIUS * PERCEPTION_RADIUS;
                }
                break;
        }
    }
}

/*Runtime bird dimension change*/
void change_birds_dimensions(bool increase, uint8_t **images_data) {
    if (increase)
        BIRD_SIZE++;
    else
        BIRD_SIZE--;
    delete_placements();
    send_payload_data(images_data);
}

void usage(const char *prog) {
    fprintf(stderr,
            "Usage: %s [-n BIRDS] [-f FPS]\n"
            "  -n NUMBER    number of boids (default %d, max %d)\n"
            "  -f FPS       frame rate (default %d, max %d)\n"
            "  -h           show this help\n",
            prog, 800, MAX_BIRDS, DEF_FRAME_RATE, MAX_FRAME_RATE);
}

void read_input(int argc, char **argv) {
    for (int i = 1; i < argc; i++) {
        bool is_birds = strcmp(argv[i], "-n") == 0;
        bool is_fps = strcmp(argv[i], "-f") == 0;

        if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            usage(argv[0]);
            exit(EXIT_SUCCESS);
        }
        if (!is_birds && !is_fps) {
            fprintf(stderr, "Unknown option : %s\n", argv[i]);
            usage(argv[0]);
            exit(EXIT_FAILURE);
        }
        if (i + 1 >= argc) {
            fprintf(stderr, "Missing value for %s\n", argv[i]);
            usage(argv[0]);
            exit(EXIT_FAILURE);
        }

        char *end = NULL;
        errno = 0;
        long arg = strtol(argv[++i], &end, 10);
        if (errno == ERANGE || end == argv[i] || *end != '\0' || arg <= 0) {
            fprintf(stderr, "Invalid value for %s : %s\n", argv[i - 1], argv[i]);
            exit(EXIT_FAILURE);
        }

        if (is_birds) {
            if (arg > MAX_BIRDS) {
                fprintf(stderr, "Birds number capped to %d\n", MAX_BIRDS);
                arg = MAX_BIRDS;
            }
            BIRDS_N = (int)arg;
        } else {
            if (arg > MAX_FRAME_RATE) {
                fprintf(stderr, "Frame rate capped to %d\n", MAX_FRAME_RATE);
                arg = MAX_FRAME_RATE;
            }
            FRAME_RATE = (int)arg;
            update_speed();
        }
    }
}

void refresh_screen(char **output_buf, drawn_bird_t **draw_birds, bird_t **birds,
                    bird_t **birds_copy) {
    for (int i = 0; i < BIRDS_N; i++) print_bird(draw_birds, i, *output_buf);
    copy(birds, birds_copy, BIRDS_N);
    update_rotation_frame_id(draw_birds);
    update_birds(birds_copy, birds, screen_width, screen_heigth, BIRDS_N);
    clean_screen();
    fflush(stdout);
    full_write(*output_buf, output_buf_off);
    output_buf_off = 0;
}

/*Elapsed microseconds between two monotonic timestamps*/
static long elapsed_us(struct timespec *start, struct timespec *end) {
    return (end->tv_sec - start->tv_sec) * 1000000L + (end->tv_nsec - start->tv_nsec) / 1000L;
}

int main(int argc, char *argv[]) {
    struct timespec frame_start, frame_end;

    read_input(argc, argv); /*Reads cli input data*/
    find_resources_root();
    srand((unsigned int)time(NULL));
    get_screen_dimensions();
    install_signal_handlers();
    atexit(my_atexit);      /*Defines exit callback*/
    if (my_atenter() < 0) { /*Try to enable terminal raw mode*/
        perror("Can't enable raw mode :");
        exit(EXIT_FAILURE);
    }
    clear();

    char *output_buf;
    uint8_t **images_data = (uint8_t **)malloc(sizeof(uint8_t *) * ROTATION_FRAME * IMAGE_SIZES);
    drawn_bird_t **draw_birds = (drawn_bird_t **)malloc(sizeof(drawn_bird_t *) * (size_t)BIRDS_N);
    bird_t **birds = (bird_t **)malloc(sizeof(bird_t *) * (size_t)BIRDS_N);
    bird_t **birds_copy = (bird_t **)malloc(sizeof(bird_t *) * (size_t)BIRDS_N);

    if (images_data == NULL || draw_birds == NULL || birds == NULL || birds_copy == NULL) {
        perror("Out of memory");
        exit(EXIT_FAILURE);
    }

    init(&output_buf, images_data, draw_birds, birds, birds_copy);
    send_payload_data(images_data); /*Sends png images data base64 encoded*/

    while (1) {
        clock_gettime(CLOCK_MONOTONIC, &frame_start);

        /*Refresh screen*/
        get_screen_dimensions();
        refresh_screen(&output_buf, draw_birds, birds, birds_copy);

        /*Handles input*/
        handle_key(images_data);

        /*Sleeps to comply frame rate, taking the frame cost into account*/
        clock_gettime(CLOCK_MONOTONIC, &frame_end);
        long budget_us = 1000000L / FRAME_RATE;
        long remaining_us = budget_us - elapsed_us(&frame_start, &frame_end);
        if (remaining_us > 0) {
            struct timespec sleep_time;
            sleep_time.tv_sec = remaining_us / 1000000L;
            sleep_time.tv_nsec = (remaining_us % 1000000L) * 1000L;
            nanosleep(&sleep_time, NULL);
        }
    }
}
