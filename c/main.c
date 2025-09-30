#include "boids.h"
#include <math.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/ioctl.h>

#define ROTATION_FRAME 72
#define FRAME_ANGLE (360/ROTATION_FRAME)
#define BIRDS_NUM 200
#define OPTIMAL_CHUNK_SIZE 4096
#define SPEED 1
#define BIRD_WIDTH 20
#define BIRD_HEIGTH 20
#define FRAME_GAP 30
#define PNG_FORMAT 100
#define ESC "\033"

#define DEF_TERMINAL_WIDTH 100
#define DEF_TERMINAL_HEIGHT 100

typedef int rotation_frame_id_t;

typedef struct {
    rotation_frame_id_t id;
    bird_t *bird_ref;
    size_t image_size;
} drawn_bird_t;

static const char base64_chars[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static int frame_id = 0;

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
void print_escape(int data_format, int not_last_chunk, int frame_id, int x,
                  int y, int s, int v, int frame_gap, uint8_t * payload);
void run_animation();
void delete_current_frame();
void print_bird(drawn_bird_t ** birds_array, uint8_t ** images_data_array,
                int not_last_chunk, int bird_no, int x, int y,
                int frame_id);
void clean_screen();

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
        birds[i] = (bird_t *) malloc(sizeof(bird_t));
        if (i < ROTATION_FRAME)
            images_data[i] = (uint8_t *) malloc(sizeof(uint8_t));
    }

    init_birds(draw_birds, images_data, base_path, screen_width,
               screen_heigth);

    for (int i = 0; i < BIRDS_NUM; i++) {
        birds[i] = draw_birds[i]->bird_ref;
    }
    clean_screen();
    run_animation();
    while (1) {
        display_birds(draw_birds, images_data);
        update_birds(birds, screen_width, screen_heigth, BIRDS_NUM);
        update_rotation_frame_id(draw_birds);
    }
}

void get_image_path(char *base_path, int rotation_frame_id)
{
    char buf[3];
    sprintf(buf, "%d", rotation_frame_id);
    strcat(base_path, buf);
    strcat(base_path, ".png");
}

int init_rotation_frames(uint8_t **images_data_array,
                         drawn_bird_t **birds_array, char *base_path)
{
    for (int i = 0; i < ROTATION_FRAME; i++) {
        char base_path_copy[strlen(base_path) + 1];
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
        birds_array[i]->image_size = size;

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
        birds_array[i]->id =
            to_degrees(birds_array[i]->bird_ref->direction) / FRAME_ANGLE;
    }
    init_rotation_frames(images_data_array, birds_array, base_path);
}

int to_degrees(double radians)
{
    return radians * (180.0 / M_PI);
}

void display_birds(drawn_bird_t **birds_array, uint8_t **images_data_array)
{
    for (int i = 0; i < BIRDS_NUM; i++) {
        int more_frame;
        if (i < BIRDS_NUM - 1)
            more_frame = 1;
        else
            more_frame = 0;
        print_bird(birds_array, images_data_array, more_frame, i,
                   birds_array[i]->bird_ref->x,
                   birds_array[i]->bird_ref->y, frame_id);
    }
    frame_id++;
    delete_current_frame();
}

void update_rotation_frame_id(drawn_bird_t **birds_array)
{
    for (int i = 0; i < BIRDS_NUM; i++) {
        birds_array[i]->id =
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
    size_t output_size = ((input_length + 2) / 3) * 4 + 1;
    uint8_t *output = (uint8_t *) malloc((output_size+1) * sizeof(uint8_t));

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

        for (size_t i = 0; i < 4; i++) {
            output[chunk_count * 4 + i] = base64_chars[char_array_4[i]];
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




void print_escape(int data_format, int not_last_chunk, int frame_id, int x,
                  int y, int s, int v, int frame_gap, uint8_t *payload)
{
    char buf[1000];
    printf
        ("\033_Ga=f,i=1,f=%d,m=%d,c=%d,x=%d,y=%d,s=%d,v=%d,z=%d;%s\033\\",
         data_format, not_last_chunk, frame_id, x, y, s, v, frame_gap,
         (char *) payload);
    fflush(stdout);
    scanf("ciao:", buf);
    fprintf(stderr, "%s", buf);
}

void run_animation()
{
    printf("\033_Ga=a,s=3;\033\\");
}

void clean_screen()
{
    system("clear");
}

void delete_current_frame()
{
    printf("\033_Ga=d,i=1,d=a;\\033");
}


void print_bird(drawn_bird_t **birds_array, uint8_t **images_data_array,
                int not_last_chunk, int bird_no, int x, int y,
                int frame_id)
{
    rotation_frame_id_t rotation_frame_id = birds_array[bird_no]->id;
    uint8_t *payload = images_data_array[rotation_frame_id];
    print_escape(PNG_FORMAT, not_last_chunk, frame_id, x, y, BIRD_WIDTH,
                 BIRD_HEIGTH, FRAME_GAP, payload);
}
