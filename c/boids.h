
#ifndef BOIDS_H
#define BOIDS_H
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <stdlib.h>

typedef struct {
    int id, width, heigth, speed;
    double direction, x, y;
} bird_t;

typedef struct {
    double x, y;
} vector2d_t;

bird_t *init_bird(int id, int width, int heigth,
                  int screen_width, int screen_heigth);

void update_birds(bird_t ** birds, int screen_width, int screen_height,
                  int num_birds);
#endif
