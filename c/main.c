#include "boids.h"
#include <math.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/ioctl.h>

#define ROTATION_FRAME 360
#define FRAME_ANGLE 1
#define BIRDS_NUM 100
#define OPTIMAL_CHUNK_SIZE 4096
#define SPEED 100
#define BIRD_WIDTH 20
#define BIRD_HEIGTH 20
#define FRAME_GAP 30
#define PNG_FORMAT 100
#define ESC "\033"

#define DEF_TERMINAL_WIDTH 100
#define DEF_TERMINAL_HEIGHT 100

typedef int rotation_frame_id_t;

typedef struct {
    rotation_frame_id_t prev_id;
    rotation_frame_id_t curr_id;
    bird_t *bird_ref;
} drawn_bird_t;

static const char base64_chars[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
void get_image_path(char *base_path, int rotation_frame_id);
int init_rotation_frames(uint8_t ** images_data_array,
                         drawn_bird_t ** birds_array, char *base_path);
void init_birds(drawn_bird_t ** birds_array, uint8_t ** images_data_array,
                char *base_path, int screen_width, int screen_heigth);
int to_degrees(double radians);
void display_birds(drawn_bird_t ** birds_array,
                   uint8_t ** images_data_array);
void update_rotation_frame_id(drawn_bird_t ** birds_array);
uint8_t *base64_encode(const uint8_t * input, size_t input_length);
void print_bird(drawn_bird_t ** birds_array, uint8_t ** images_data_array,
                int bird_no);
void clean_screen();
void print_unchanged_direction(int id, int x, int y);
void print_changed_direction(int id, int x, int y, uint8_t * payload);

int main(int argc, char *argv[])
{

    char *base_path = "../resources/bird_";
    uint8_t *images_data[ROTATION_FRAME];
    drawn_bird_t *draw_birds[BIRDS_NUM];
    bird_t *birds[BIRDS_NUM];
    ssize_t screen_width;
    ssize_t screen_heigth;
    struct winsize w;
    ioctl(STDOUT_FILENO, TIOCGWINSZ, &w);
    screen_width = w.ws_xpixel;
    screen_heigth = w.ws_ypixel;
    if (!screen_heigth || !screen_width) {
        screen_heigth = DEF_TERMINAL_HEIGHT;
        screen_width = DEF_TERMINAL_WIDTH;
    }
    printf("Terminal width: %zd, height: %zd\n", screen_width,
           screen_heigth);

    for (int i = 0; i < BIRDS_NUM; i++) {
        draw_birds[i] = (drawn_bird_t *) malloc(sizeof(drawn_bird_t));
    }

    init_birds(draw_birds, images_data, base_path, screen_width,
               screen_heigth);

    for (int i = 0; i < BIRDS_NUM; i++) {
        birds[i] = draw_birds[i]->bird_ref;
    }

    while (1) {
        system("clear");
        //printf("\033_Ga=d;\033\\");
        update_birds(birds, screen_width, screen_heigth, BIRDS_NUM);
        update_rotation_frame_id(draw_birds);
        display_birds(draw_birds, images_data);
        fflush(stdout);
        usleep(30000);          
    }
}

void get_image_path(char *base_path, int rotation_frame_id)
{
    char buf[20];
    sprintf(buf, "%d", rotation_frame_id);
    strcat(base_path, buf);
    strcat(base_path, ".png");
}

int init_rotation_frames(uint8_t **images_data_array,
                         drawn_bird_t **birds_array, char *base_path)
{
    for (int i = 0; i < ROTATION_FRAME; i++) {
        char base_path_copy[strlen(base_path) + 20];
        strcpy(base_path_copy, base_path);
        get_image_path(base_path_copy, i);
        FILE *file = fopen(base_path_copy, "rb");
        if (file == NULL) {
            perror("Error during file opening");
            return -1;
        }
        int size = lseek(fileno(file), 0, SEEK_END);
        lseek(fileno(file), 0, SEEK_SET);
        uint8_t *buf = (uint8_t *) malloc(sizeof(uint8_t) * size);
        if (fread(buf, sizeof(char), size, file) != size) {
            perror("Error during file reading");
            fclose(file);
            return -1;
        }
        images_data_array[i] = base64_encode(buf, size);
        fclose(file);
        free(buf);
    }
    return 0;
}

void init_birds(drawn_bird_t **birds_array, uint8_t **images_data_array,
                char *base_path, int screen_width, int screen_heigth)
{
    for (int i = 0; i < BIRDS_NUM; i++) {
        birds_array[i]->bird_ref =
            init_bird(i, SPEED, BIRD_WIDTH, BIRD_HEIGTH, screen_width,
                      screen_heigth);
        birds_array[i]->curr_id =
            to_degrees(birds_array[i]->bird_ref->direction) / FRAME_ANGLE;
        birds_array[i]->prev_id = 0;
    }
    init_rotation_frames(images_data_array, birds_array, base_path);
}

int to_degrees(double radians)
{
    int deg = (int) (radians * (180.0 / M_PI)) + 90;
    return (deg % 360 + 360) % 360;
}

void display_birds(drawn_bird_t **birds_array, uint8_t **images_data_array)
{
    for (int i = 0; i < BIRDS_NUM; i++) {
        print_bird(birds_array, images_data_array, i);
    }
}

void update_rotation_frame_id(drawn_bird_t **birds_array)
{
    for (int i = 0; i < BIRDS_NUM; i++) {
        birds_array[i]->prev_id = birds_array[i]->curr_id;
        birds_array[i]->curr_id =
            to_degrees(birds_array[i]->bird_ref->direction) / FRAME_ANGLE;
    }
}

/**
 * @return an encoded base64 string which is terminated with the null character
 */
uint8_t *base64_encode(const uint8_t *input, size_t input_length)
{

    uint8_t char_array_3[3];
    uint8_t char_array_4[4];
    size_t output_size = ((input_length + 2) / 3) * 4;
    uint8_t *output =
        (uint8_t *) malloc((output_size + 1) * sizeof(uint8_t));
    int i = 0;
    int chunk_count = 0;

    while (input_length >= 3) {
        input_length -= 3;
        char_array_3[0] = input[i];
        char_array_3[1] = input[i + 1];
        char_array_3[2] = input[i + 2];

        char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
        char_array_4[1] =
            ((char_array_3[0] & 0x03) << 4) +
            ((char_array_3[1] & 0xf0) >> 4);
        char_array_4[2] =
            ((char_array_3[1] & 0x0f) << 2) +
            ((char_array_3[2] & 0xc0) >> 6);
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
        char_array_4[1] =
            ((char_array_3[0] & 0x03) << 4) +
            ((char_array_3[1] & 0xf0) >> 4);
        char_array_4[2] =
            ((char_array_3[1] & 0x0f) << 2) +
            ((char_array_3[2] & 0xc0) >> 6);
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


void print_bird(drawn_bird_t **birds_array, uint8_t **images_data_array,
                int bird_no)
{
    drawn_bird_t *bird = birds_array[bird_no];
    uint8_t *payload = images_data_array[bird->curr_id];
    int col = bird->bird_ref->x / 10;
    int row = bird->bird_ref->y / 20;
    int offset_x = (int) bird->bird_ref->x % 10;
    int offset_y = (int) bird->bird_ref->y % 20;
    printf("\033[%d;%dH\033_Ga=T,f=100,q=1,I=%d,X=%d,Y=%d;%s\033\\",
           row, col, bird_no, offset_x, offset_y, payload);
}
