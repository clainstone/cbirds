#include "boids.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#define X_START_OFF 20
#define Y_START_OFF 20

const int PERCEPTION_RADIUS = 70;
const int TURN_RADIUS = 10;
const int SPEED = 15;
const int PERCEPTION_RADIUS_SQUARED = PERCEPTION_RADIUS * PERCEPTION_RADIUS;
const double SEPARATION_WEIGHT = 0.01;
const double ALIGNMENT_WEIGHT = 20;
const double COHESION_WEIGHT = 0.01;
const double BOUNDARY_AV_WEIGHT = 100.0;

bird_t *init_bird(int id, int width, int heigth, int screen_width,
                  int screen_heigth);

void update_birds(bird_t **birds, bird_t **close, int screen_width,
                  int screen_height, int num_birds);

void update_birds_index(bird_t **birds_copy_to_read, bird_t **birds_to_write,
                        int screen_width, int screen_height, int start_index,
                        int end_index, int birds_num);
static void update_direction(bird_t *bird, double next_direction);
static double calculate_rules_direction(
    bird_t *bird, bird_t **birds, int num_birds, double separation_weight,
    double alignment_weight, double cohesion_weight, double boundary_av_weight,
    int screen_width, int screen_heigth, int turn_radius);
static vector2d_t *calculate_boundary_av_direction(bird_t *bird,
                                                   int screen_width,
                                                   int screen_heigth,
                                                   int turn_radius);
static void add_vector(vector2d_t *vector, double x, double y);
static void prod_vector(vector2d_t *vector, double scalar);
static int distance(bird_t *b1, bird_t *b2);
static double my_atan2(double y, double x);
static void close_birds(bird_t **close_birds_list, bird_t *target,
                        bird_t **birds, int num_birds, int perception_radius,
                        int *counter);
static void update_direction(bird_t *bird, double next_direction) {
    bird->direction = next_direction;
    bird->x += (double)bird->speed * cos(next_direction);
    bird->y += (double)bird->speed * sin(next_direction);
}

/**
 * @brief Initializes a bird with random position and
 * direction. The returned bird is dynamically allocated and
 * should be freed by the caller.
 */
bird_t *init_bird(int id, int width, int heigth, int screen_width,
                  int screen_heigth) {
    bird_t *bird = (bird_t *)malloc(sizeof(bird_t));

    bird->x = screen_width * ((double)rand() / RAND_MAX) + X_START_OFF;
    bird->y = screen_heigth * ((double)rand() / RAND_MAX) + Y_START_OFF;
    bird->direction = 2 * M_PI * ((double)rand() / RAND_MAX);
    bird->id = id;
    bird->speed = SPEED;
    bird->width = width;
    bird->heigth = heigth;
    return bird;
}

static void init_vector(vector2d_t *vector, double x, double y) {
    vector->x = x;
    vector->y = y;
}

static void add_vector(vector2d_t *vector, double x, double y) {
    vector->x += x;
    vector->y += y;
}

static void prod_vector(vector2d_t *vector, double scalar) {
    vector->x *= scalar;
    vector->y *= scalar;
}

/**
 * The returned vector is dynamically allocated and should
 * be freed by the caller.
 */
static vector2d_t *calculate_boundary_av_direction(bird_t *bird,
                                                   int screen_width,
                                                   int screen_heigth,
                                                   int turn_radius) {
    vector2d_t *boundary_av = (vector2d_t *)malloc(sizeof(vector2d_t));

    init_vector(boundary_av, 0, 0);

    if (bird->x < turn_radius) {
        add_vector(boundary_av, 1, 0);
    } else if (bird->x > screen_width - turn_radius) {
        add_vector(boundary_av, -1, 0);
    }
    if (bird->y < turn_radius) {
        add_vector(boundary_av, 0, 1);
    } else if (bird->y > screen_heigth - turn_radius) {
        add_vector(boundary_av, 0, -1);
    }
    return boundary_av;
}

static double calculate_rules_direction(
    bird_t *target, bird_t **birds, int num_birds, double separation_weight,
    double alignment_weight, double cohesion_weight, double boundary_av_weight,
    int screen_width, int screen_heigth, int turn_radius) {
    vector2d_t separation = {0, 0};
    vector2d_t alignment = {0, 0};
    vector2d_t cohesion = {0, 0};

    vector2d_t *boundary_av_ptr = calculate_boundary_av_direction(
        target, screen_width, screen_heigth, turn_radius);

    int close_count = 0;

    for (int i = 0; i < num_birds; i++) {
        bird_t *boid = birds[i];

        if (boid->id != target->id) {
            add_vector(&separation, target->x - boid->x, target->y - boid->y);
            add_vector(&alignment, cos(boid->direction), sin(boid->direction));
            add_vector(&cohesion, boid->x, boid->y);
            close_count++;
        }
    }

    if (close_count > 0) {
        alignment.x /= close_count;
        alignment.y /= close_count;
        cohesion.x /= close_count;
        cohesion.y /= close_count;

        cohesion.x -= target->x;
        cohesion.y -= target->y;

        prod_vector(&separation, separation_weight);
        prod_vector(&alignment, alignment_weight);
        prod_vector(&cohesion, cohesion_weight);
        prod_vector(boundary_av_ptr, boundary_av_weight);

        double result_x =
            separation.x + alignment.x + cohesion.x + boundary_av_ptr->x;
        double result_y =
            separation.y + alignment.y + cohesion.y + boundary_av_ptr->y;
        free(boundary_av_ptr);

        return my_atan2(result_y, result_x);
    } else {
        free(boundary_av_ptr);
        return target->direction;
    }
}

static int distance(bird_t *b1, bird_t *b2) {
    return sqrt((b1->x - b2->x) * (b1->x - b2->x) +
                (b1->y - b2->y) * (b1->y - b2->y));
}

static int squared_distance(bird_t *b1, bird_t *b2) {
    return ((b1->x - b2->x) * (b1->x - b2->x) +
            (b1->y - b2->y) * (b1->y - b2->y));
}

/**
 * Populate close_bird_list with every bird that is not
 * farer of radius from target. close_birds_dimension is
 * fixed at (N-1)*sizeof(bird_t*) where N is the number of
 * birds
 */
static void close_birds(bird_t **close_birds_list, bird_t *target,
                        bird_t **birds, int num_birds, int perception_radius,
                        int *counter) {
    *counter = 0;
    for (int i = 0; i < num_birds; i++) {
        if (birds[i]->id != target->id) {
            if (squared_distance(target, birds[i]) < perception_radius) {
                (*counter)++;
            }
        }
    }
    int current_index = 0;
    for (int i = 0; i < num_birds; i++) {
        bird_t *boid = birds[i];
        if (boid->id != target->id) {
            if (squared_distance(target, boid) < perception_radius) {
                close_birds_list[current_index++] = boid;
            }
        }
    }
}

void update_birds(bird_t **birds, bird_t **close, int screen_width,
                  int screen_height, int num_birds) {
    for (int i = 0; i < num_birds; i++) {
        int counter = 0;
        close_birds(close, birds[i], birds, num_birds,
                    PERCEPTION_RADIUS_SQUARED, &counter);

        if (close != NULL && counter > 0) {
            double direction = calculate_rules_direction(
                birds[i], close, counter, SEPARATION_WEIGHT, ALIGNMENT_WEIGHT,
                COHESION_WEIGHT, BOUNDARY_AV_WEIGHT, screen_width,
                screen_height, TURN_RADIUS);
            update_direction(birds[i], direction);
        }
    }
}

double my_atan2(double y, double x) {
    double angle = atan2(y, x);
    if (angle < 0.0) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

void update_birds_index(bird_t **birds_copy_to_read, bird_t **birds_to_write,
                        int screen_width, int screen_height, int start_index,
                        int end_index, int birds_num) {
    // REMEMBER:need to choose a different close_buffer
    // for every thread
    for (int i = start_index; i < end_index; i++) {
        int counter = 0;
        bird_t *close[birds_num];
        close_birds(close, birds_copy_to_read[i], birds_copy_to_read, birds_num,
                    PERCEPTION_RADIUS_SQUARED, &counter);
        if (counter > 0) {
            double direction = calculate_rules_direction(
                birds_copy_to_read[i], close, counter, SEPARATION_WEIGHT,
                ALIGNMENT_WEIGHT, COHESION_WEIGHT, BOUNDARY_AV_WEIGHT,
                screen_width, screen_height, TURN_RADIUS);
            update_direction(birds_to_write[i], direction);
        }
    }
}

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

