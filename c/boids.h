
#ifndef BOIDS_H
#define BOIDS_H
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

typedef struct {
	int id, width, heigth, speed;
	double direction, x, y;
} bird_t;

typedef struct {
	double x, y;
} vector2d_t;

bird_t *init_bird(int id, int width, int heigth, int screen_width,
		  int screen_heigth);

void update_birds(bird_t **birds, bird_t **close, int screen_width,
		  int screen_height, int num_birds);
void update_birds_index(bird_t **birds_copy_to_read, bird_t **birds_to_write,
			int screen_width, int screen_height, int start_index,
			int end_index, int birds_num);
void copy(bird_t **original, bird_t **copy, int birds_num);
#endif
