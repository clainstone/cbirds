

#ifndef BOIDS_H
#define BOIDS_H
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <stdlib.h>

typedef struct {
    int x, y, id, width, heigth, speed;
    double direction;
} bird_t;

typedef struct {
    int x, y;
} vector2d_t;

bird_t *init_bird(int id, int speed, int width, int heigth,
                  int screen_width, int screen_heigth);

void update_birds(bird_t ** birds, int screen_width, int screen_height,
                  int num_birds);
static void update_direction(bird_t * bird, double next_direction);
static double calculate_rules_direction(bird_t * bird, bird_t ** birds,
                                        int num_birds,
                                        double separation_weight,
                                        double alignment_weight,
                                        double cohesion_weight,
                                        double boundary_av_weight,
                                        int screen_width,
                                        int screen_heigth,
                                        int turn_radius);
static vector2d_t *calculate_boundary_av_direction(bird_t * bird,
                                                   int screen_width,
                                                   int screen_heigth,
                                                   int turn_radius);
static void add_vector(vector2d_t * vector, int x, int y);
static void prod_vector(vector2d_t * vector, double scalar);
static int distance(bird_t * b1, bird_t * b2);
static bird_t **close_birds(bird_t * target, bird_t ** birds,
                            int num_birds, int perception_radius,
                            int *counter);

#endif