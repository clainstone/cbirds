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
#include <time.h>
#include <unistd.h>

#include "boids.h"

#define _XOPEN_SOURCE 600
#define ROTATION_FRAME 360
#define FRAME_ANGLE (360 / ROTATION_FRAME)
#define BIRDS_NUM 200
#define BIRD_WIDTH 20
#define BIRD_HEIGTH 20
#define PNG_FORMAT 100
#define FRAME_RATE 120
#define PERIOD_MULTIPL 1000000
#define DEF_TERMINAL_WIDTH 100
#define DEF_TERMINAL_HEIGHT 100
#define THREAD_DEF_NUM 1
#define POLLING_WAIT_MILLIS 1

typedef int rotation_frame_id_t;
typedef struct {
    rotation_frame_id_t prev_id;
    rotation_frame_id_t curr_id;
    bird_t *bird_ref;
} drawn_bird_t;

typedef struct {
    int first_bird_index, last_bird_index;
    bird_t **birds_copy_to_read;
    bird_t **birds_to_write;
    drawn_bird_t **draw_birds;
    char *output_buf;
    uint8_t **images_data_array;
} btask_args_t;

const char base64_chars[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

int max_payload_len = -1;
char *base_path = "../resources/bird_";
ssize_t screen_width;
ssize_t screen_heigth;

void get_image_path(char *base_path, int rotation_frame_id);
int init_rotation_frames(uint8_t **images_data_array, char *base_path);
void init_birds(drawn_bird_t **birds_array, uint8_t **images_data_array,
                char *base_path, int screen_width, int screen_heigth);
int to_degrees(double radians);
void display_birds(drawn_bird_t **birds_array, uint8_t **images_data_array,
                   char *output_buf);
uint8_t *base64_encode(const uint8_t *input, size_t input_length);
void clean_screen();
void _task(pthread_mutex_t *buf_mutex, btask_args_t *args);
void print_bird(drawn_bird_t **birds_array, int bird_no, char *output_buf);
void update_rotation_frame_id(drawn_bird_t **birds_array);
void init(char **output_buf, uint8_t **images_data, drawn_bird_t **draw_birds,
          bird_t **birds, bird_t **birds_copy);
void send_payload_data(uint8_t **payload_data);
void get_screen_dimensions();

void get_image_path(char *base_path, int rotation_frame_id) {
    char buf[20];
    sprintf(buf, "%d", rotation_frame_id);
    strcat(base_path, buf);
    strcat(base_path, ".png");
}

int init_rotation_frames(uint8_t **images_data_array, char *base_path) {
    for (int i = 0; i < ROTATION_FRAME; i++) {
        char base_path_copy[strlen(base_path) + 20];
        strcpy(base_path_copy, base_path);
        get_image_path(base_path_copy, i);
        FILE *file = fopen(base_path_copy, "rb");
        if (file == NULL) {
            perror("Error during file opening");
            exit(1);
        }
        int size = lseek(fileno(file), 0, SEEK_END);
        lseek(fileno(file), 0, SEEK_SET);
        uint8_t *buf = (uint8_t *)malloc(sizeof(uint8_t) * size);
        if (fread(buf, sizeof(char), size, file) != (unsigned long)size) {
            perror("Error during file reading");
            fclose(file);
            exit(1);
        }
        images_data_array[i] = base64_encode(buf, size);
        fclose(file);
        free(buf);
    }
    return 0;
}

void init_birds(drawn_bird_t **birds_array, uint8_t **images_data_array,
                char *base_path, int screen_width, int screen_heigth) {
    for (int i = 0; i < BIRDS_NUM; i++) {
        birds_array[i]->bird_ref =
            init_bird(i, BIRD_WIDTH, BIRD_HEIGTH, screen_width, screen_heigth);
        birds_array[i]->curr_id =
            to_degrees(birds_array[i]->bird_ref->direction) / FRAME_ANGLE;
        birds_array[i]->prev_id = birds_array[i]->curr_id;
    }
    init_rotation_frames(images_data_array, base_path);
}

int to_degrees(double radians) {
    int deg = (int)(radians * (180.0 / M_PI));
    return (deg % 360 + 360) % 360;
}

void update_rotation_frame_id(drawn_bird_t **birds_array) {
    for (int i = 0; i < BIRDS_NUM; i++) {
        birds_array[i]->prev_id = birds_array[i]->curr_id;
        birds_array[i]->curr_id =
            to_degrees(birds_array[i]->bird_ref->direction) / FRAME_ANGLE;
    }
}

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
        char_array_4[1] =
            ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
        char_array_4[2] =
            ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
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
            ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
        char_array_4[2] =
            ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
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

void print_bird(drawn_bird_t **birds_array, int bird_no, char *output_buf) {
    char buf[150];
    drawn_bird_t *bird = birds_array[bird_no];
    int col = bird->bird_ref->x / 8;
    int row = bird->bird_ref->y / 16;
    int offset_x = (int)bird->bird_ref->x % 8;
    int offset_y = (int)bird->bird_ref->y % 16;
    rotation_frame_id_t id = bird->curr_id;
    sprintf(buf, "\033[%d;%dH\033_Ga=p,I=%d,q=2,p=%d,X=%d,Y=%d,z=%d\033\\", row,
            col, id + 1, 0, offset_x, offset_y, bird_no);
    strcat(output_buf, buf);
}

void send_payload_data(uint8_t **images_data) {
    for (int i = 0; i < ROTATION_FRAME; i++) {
        printf("\033_Ga=t,q=2,f=100,I=%d;%s\033\\", i + 1,
               (char *)images_data[i]);
    }
    fflush(stdout);
    clean_screen();
}

void init(char **output_buf, uint8_t **images_data, drawn_bird_t **draw_birds,
          bird_t **birds, bird_t **birds_copy) {
    ssize_t buffer_offset = 300;

    get_screen_dimensions();

    *output_buf = (char *)malloc(sizeof(char) * (buffer_offset)*BIRDS_NUM + 1);
    *output_buf[0] = '\0';
    for (int i = 0; i < BIRDS_NUM; i++) {
        draw_birds[i] = (drawn_bird_t *)malloc(sizeof(drawn_bird_t));
        birds_copy[i] = (bird_t *)malloc(sizeof(bird_t));
    }

    init_birds(draw_birds, images_data, base_path, screen_width, screen_heigth);
    for (int i = 0; i < BIRDS_NUM; i++) {
        birds[i] = draw_birds[i]->bird_ref;
    }
}

void clean_screen() {
    printf("\033_Ga=d,d=a\033\\");
}

void get_screen_dimensions() {
    struct winsize w;
    ioctl(STDOUT_FILENO, TIOCGWINSZ, &w);
    screen_width = w.ws_xpixel;
    screen_heigth = w.ws_ypixel;
    if (!screen_heigth || !screen_width) {
        screen_heigth = DEF_TERMINAL_HEIGHT;
        screen_width = DEF_TERMINAL_WIDTH;
    }
}

int main() {
    system("clear");
    char *output_buf;
    int output_buf_len;
    uint8_t *images_data[ROTATION_FRAME];
    drawn_bird_t *draw_birds[BIRDS_NUM];
    bird_t *birds[BIRDS_NUM];
    bird_t *birds_copy[BIRDS_NUM];
    pthread_mutex_t *queue_mutex;
    pthread_cond_t *cond;

    init(&output_buf, images_data, draw_birds, birds, birds_copy);
    send_payload_data(images_data);

    while (1) {
        get_screen_dimensions();
        for (int i = 0; i < BIRDS_NUM; i++)
            print_bird(draw_birds, i, output_buf);
        copy(birds, birds_copy, BIRDS_NUM);
        update_rotation_frame_id(draw_birds);
        update_birds(birds_copy, birds, screen_width, screen_heigth, BIRDS_NUM);

        clean_screen();
        printf("\033_Gs=1;\033\\");
        printf("%s", output_buf);
        fflush(stdout);
        printf("\033_Gs=2;\033\\");
        output_buf[0] = '\0';
        usleep(1000000 / FRAME_RATE);
    }
}
