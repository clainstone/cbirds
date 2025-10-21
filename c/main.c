#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <pthread.h>
#include <semaphore.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/termios.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#define _XOPEN_SOURCE 600
#define ROTATION_FRAME 90                  /*Number of roation frame*/
#define FRAME_ANGLE (360 / ROTATION_FRAME) /*Difference in degrees from ajacents rotation frames*/
#define BASE_IMAGE_SIZE 5
#define IMAGE_SIZES 40
#define PNG_FORMAT 100 /*Kitty's protocol png escape code*/
#define PERIOD_MULTIPL 1000000
#define DEF_TERMINAL_WIDTH 100
#define DEF_TERMINAL_HEIGHT 100
#define X_START_OFF 20
#define Y_START_OFF 20
#define INPUT_BUF_DIM 100

/*=========================== Simulation parameters ===============================*/

const int DEF_FRAME_RATE = 60;    /*Default frame rate in case no one is specified*/
const int PERCEPTION_RADIUS = 35; /*The maximum distance whereas two boids can interacts*/
const int PERCEPTION_RADIUS_SQUARED = PERCEPTION_RADIUS * PERCEPTION_RADIUS;
const int DEF_SPEED = 60; /*Default speed in case no one is specified as input arg*/

int BIRDS_N = 800;   /*Birds number*/
int FRAME_RATE = 60; /*Frames per second*/
int TURN_RADIUS_X;   /*Border distance within the bird starts to steer to avoid the collision*/
int TURN_RADIUS_Y;
int SPEED = 40;     /*Pixels increment between two frames*/
int BIRD_SIZE = 15; /*Bird size in pixels*/

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

int max_payload_len = -1;
ssize_t screen_width;
ssize_t screen_heigth;
ssize_t n_col;
ssize_t n_row;
ssize_t character_width_p;  /*character pixel width*/
ssize_t character_height_p; /*character pixel heigth*/
int output_buf_off = 0; /*Offset within the outbuffer used to concatenate escape control strings*/
struct termios saved_termios; /*Saved termios structure to be resumed after process termination*/

/*============================================================================================*/

bird_t *init_bird(int id, int width, int heigth, int screen_width, int screen_heigth);

double calculate_rules_direction(bird_t *bird, bird_t **birds, int num_birds, int screen_width,
                                 int screen_heigth);
double my_atan2(double y, double x);

int to_degrees(double radians);
int distance(bird_t *b1, bird_t *b2);
int squared_distance(bird_t *b1, bird_t *b2);
int enable_raw_mode();
int my_atenter();

uint8_t *base64_encode(const uint8_t *input, size_t input_length);

vector2d_t calculate_boundary_av_direction(bird_t *bird, int screen_width, int screen_heigth);

void init_rotation_frames(uint8_t **images_data_array);
void get_image_path(char *base_path, int size_index, int rotation_frame_id);
void init_birds(drawn_bird_t **birds_array, uint8_t **images_data_array, int screen_width,
                int screen_heigth);
void display_birds(drawn_bird_t **birds_array, uint8_t **images_data_array, char *output_buf);
void clean_screen();
void print_bird(drawn_bird_t **birds_array, int bird_no, char *output_buf);
void update_rotation_frame_id(drawn_bird_t **birds_array);
void init(char **output_buf, uint8_t **images_data, drawn_bird_t **draw_birds, bird_t **birds,
          bird_t **birds_copy);
void send_payload_data(uint8_t **payload_data);
void get_screen_dimensions();
void fix_weights();
void update_birds(bird_t **birds_copy_to_read, bird_t **birds_to_write, int screen_width,
                  int screen_height, int birds_num);
void update_direction(bird_t *bird, double next_direction);
void add_vector(vector2d_t *vector, double x, double y);
void prod_vector(vector2d_t *vector, double scalar);
void init_vector(vector2d_t *vector, double x, double y);
void close_birds(bird_t **close_birds_list, bird_t *target, bird_t **birds, int num_birds,
                 int *counter);
void my_atexit();
void refresh_screen();
void handle_key(uint8_t **images_data);
void read_input(int argc, char **argv);
void change_birds_dimensions(bool increase, uint8_t **bird_images);

//=======================Low level terminal handling===========================

void get_screen_dimensions() {
    struct winsize w;
    ioctl(STDOUT_FILENO, TIOCGWINSZ, &w);
    screen_width = w.ws_xpixel;
    screen_heigth = w.ws_ypixel;
    n_col = w.ws_col;
    n_row = w.ws_row;
    character_width_p = (double)screen_width / n_col;
    character_height_p = (double)screen_heigth / n_row;
    if (!screen_heigth || !screen_width) {
        screen_heigth = DEF_TERMINAL_HEIGHT;
        screen_width = DEF_TERMINAL_WIDTH;
    }
    fix_weights();
}

int my_atenter() {
    /*Enable alternate buffer*/
    system("tput smcup");
    return enable_raw_mode();
}

/* Raw mode : 1960 magic shit */
int enable_raw_mode() {
    int err;
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
    /* local modes - choing off, canonical off, no extended functions,
     * no signal chars (^Z,^C) */
    buf.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);
    buf.c_cc[VMIN] = 0;
    buf.c_cc[VMIN] = 0;
    if (tcsetattr(STDIN_FILENO, TCSAFLUSH, &buf) < 0) return -1;
    ttystate = RAW;
    return 0;
}

void my_atexit() {
    /*Disable alternate buffer*/
    system("tput rmcup");
    tcsetattr(STDERR_FILENO, TCSAFLUSH, &saved_termios);
}

//========================Image data manipulation==============================

void init_rotation_frames(uint8_t **images_data_array) {
    char base_path[] = "../resources/dim";
    char *base_path_copy = (char *)malloc(strlen(base_path) + 50);
    int size_index;
    int bird_index;

    for (size_index = 0; size_index < IMAGE_SIZES; size_index++) {
        for (bird_index = 0; bird_index < ROTATION_FRAME; bird_index++) {
            strcpy(base_path_copy, base_path);
            get_image_path(base_path_copy, size_index + BASE_IMAGE_SIZE, bird_index);
            FILE *file = fopen(base_path_copy, "rb");
            if (file == NULL) {
                perror("Error during file opening");
                exit(-1);
            }
            int size = lseek(fileno(file), 0, SEEK_END);
            lseek(fileno(file), 0, SEEK_SET);
            uint8_t buf[size];
            if (fread(buf, sizeof(char), size, file) != (unsigned long)size) {
                perror("Error during file reading");
                fclose(file);
                exit(-1);
            }

            images_data_array[size_index * ROTATION_FRAME + bird_index] = base64_encode(buf, size);
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

    if (max_payload_len == -1 || (int)strlen((char *)output) > max_payload_len)
        max_payload_len = (int)strlen((char *)output) + 1;

    return output;
}

void get_image_path(char *base_path, int size_index, int rotation_frame_id) {
    char buf[20];
    sprintf(buf, "%d", size_index);
    strcat(base_path, buf);
    strcat(base_path, "/bird_");

    sprintf(buf, "%d", rotation_frame_id);
    strcat(base_path, buf);
    strcat(base_path, ".png");
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
        printf("\033_Ga=t,q=2,f=100,I=%d;%s\033\\", i + 1,
               (char *)images_data[ROTATION_FRAME * image_size_index + i]);
    }
    fflush(stdout);
    clean_screen();
}

/* Sends only deltas about position and direction.
 * Every rotated image has an index(I), every bird is assigned to a frame index
 * defining his placement_index(p), there can be multiple birds(with different
 * placement_index) assigned to the same image index.
 * */
void print_bird(drawn_bird_t **birds_array, int bird_no, char *output_buf) {
    char buf[150];
    drawn_bird_t *bird = birds_array[bird_no];

    int col, row, offset_x, offset_y;

    col = bird->bird_ref->x / character_width_p;
    row = bird->bird_ref->y / character_height_p;
    offset_x = (int)bird->bird_ref->x % character_width_p;
    offset_y = (int)bird->bird_ref->y % character_height_p;

    if (col >= 0 && col < n_col && row >= 0 && row < n_row) {
        rotation_frame_id_t id = bird->curr_id;
        sprintf(buf, "\033[%d;%dH\033_Ga=p,I=%d,q=2,p=%d,X=%d,Y=%d,z=%d\033\\", row + 1, col + 1,
                id + 1, 0, offset_x, offset_y, bird_no);
        /*Every escape sequence is concatened to the outpute buffer that is
         * flushed output once a frame*/
        memcpy(output_buf + output_buf_off, buf, strlen(buf));
        output_buf_off += strlen(buf);
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
    ssize_t buffer_offset = 300;
    get_screen_dimensions();
    *output_buf = (char *)malloc(sizeof(char) * (buffer_offset)*BIRDS_N + 1);
    *output_buf[0] = '\0';
    for (int i = 0; i < BIRDS_N; i++) {
        draw_birds[i] = (drawn_bird_t *)malloc(sizeof(drawn_bird_t));
        birds_copy[i] = (bird_t *)malloc(sizeof(bird_t));
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

    bird->x = screen_width * ((double)rand() / RAND_MAX) + X_START_OFF;
    bird->y = screen_heigth * ((double)rand() / RAND_MAX) + Y_START_OFF;
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
    for (int i = 0; i < birds_num; i++) {
        int counter = 0;
        bird_t *close[birds_num];
        close_birds(close, birds_copy_to_read[i], birds_copy_to_read, birds_num, &counter);
        if (counter > 0) {
            double direction = calculate_rules_direction(birds_copy_to_read[i], close, counter,
                                                         screen_width, screen_height);
            update_direction(birds_to_write[i], direction);
        }
    }
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
    *counter = 0;
    for (int i = 0; i < num_birds; i++) {
        if (birds[i]->id != target->id) {
            if (squared_distance(target, birds[i]) < PERCEPTION_RADIUS_SQUARED) {
                (*counter)++;
            }
        }
    }
    int current_index = 0;
    for (int i = 0; i < num_birds; i++) {
        bird_t *boid = birds[i];
        if (boid->id != target->id) {
            if (squared_distance(target, boid) < PERCEPTION_RADIUS_SQUARED) {
                close_birds_list[current_index++] = boid;
            }
        }
    }
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
        if (boid->id != target->id) {
            add_vector(&separation, target->x - boid->x, target->y - boid->y);
            add_vector(&alignment, cos(boid->direction), sin(boid->direction));
            add_vector(&cohesion, boid->x, boid->y);
            close_count++;
        }
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

        return my_atan2(result_y, result_x);
    } else {
        // If there are no birds nearby simply returns the older direction
        return target->direction;
    }
}

void update_direction(bird_t *bird, double next_direction) {
    bird->direction = next_direction;
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

int squared_distance(bird_t *b1, bird_t *b2) {
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

void handle_key(uint8_t **images_data) {
    char input_buf[INPUT_BUF_DIM];
    ssize_t size;

    size = read(STDIN_FILENO, (void *)input_buf, INPUT_BUF_DIM);
    if (size == 1) {
        char c = input_buf[0];

        switch (c) {
            case 'q': /*quit*/
                my_atexit();
                exit(0);
                break;
            case '=': /*increase bird image size*/
                if (BIRD_SIZE < IMAGE_SIZES + BASE_IMAGE_SIZE - 1)
                    change_birds_dimensions(true, images_data);
                break;
            case '-': /*decrease bird image size*/
                if (BIRD_SIZE > BASE_IMAGE_SIZE) change_birds_dimensions(false, images_data);
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

void read_input(int argc, char **argv) {
    if (argc > 1) {
        argv++;
        argc--;
        while (argc > 1) {
            if (strcmp(*argv, "-n") == 0) { /*birds number flag*/
                argv++;
                argc--;
                long arg = strtol(*argv, NULL, 10);
                if (errno == ERANGE || arg <= 0) {
                    perror("Invalid arguments for birds num");
                    exit(-1);
                }
                BIRDS_N = (int)arg;

            } else if (strcmp(*argv, "-f") == 0) {
                argv++;
                argc--;
                long arg = strtol(*argv, NULL, 10);
                if (errno == ERANGE || arg <= 0) {
                    perror("Invalid arguments for frame rate");
                    exit(-1);
                }
                FRAME_RATE = (int)arg;
                SPEED = DEF_SPEED * (double)DEF_FRAME_RATE / FRAME_RATE;
                if (SPEED == 0) SPEED = 1;
            }

            else
                break;
            argc--;
            argv++;
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
    write(STDOUT_FILENO, *output_buf, output_buf_off);
    output_buf_off = 0;
}

int main(int argc, char *argv[]) {
    read_input(argc, argv); /*Reads cli input data*/
    get_screen_dimensions();
    if (my_atenter() < 0) { /*Try to enable terminal raw mode*/
        perror("Can't enable raw mode :");
        exit(-1);
    }
    atexit(my_atexit); /*Defines exit callback*/
    clear();
    char *output_buf;
    int output_buf_len;
    uint8_t *images_data[ROTATION_FRAME * IMAGE_SIZES];
    drawn_bird_t *draw_birds[BIRDS_N];
    bird_t *birds[BIRDS_N];
    bird_t *birds_copy[BIRDS_N];

    init(&output_buf, images_data, draw_birds, birds, birds_copy);
    send_payload_data(images_data); /*Sends png images data base64 encoded*/

    while (1) {
        /*Refresh screen*/
        get_screen_dimensions();
        refresh_screen(&output_buf, draw_birds, birds, birds_copy);

        /*Handles input*/
        handle_key(images_data);

        /*Sleeps to comply frame rate*/
        usleep(1000000 / FRAME_RATE);
    }
}
