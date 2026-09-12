#include "../spatial_grid.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

enum { CELL_SIZE = 12 };

typedef struct {
    double x, y;
} point_t;

static void read_point(const void *context, int index, double *x, double *y) {
    const point_t *points = context;
    *x = points[index].x;
    *y = points[index].y;
}

static int range_contains(const spatial_grid_t *grid, int cell, int value) {
    for (int slot = grid->offsets[cell]; slot < grid->offsets[cell + 1]; slot++)
        if (grid->indices[slot] == value) return 1;
    return 0;
}

static void test_layout_and_clamping(void) {
    static const point_t points[] = {
        {0, 0}, {11.9, 11.9}, {12, 0}, {24.9, 12}, {-10, 5}, {99, 99},
    };
    spatial_grid_t grid;

    assert(spatial_grid_init(&grid, CELL_SIZE) == SPATIAL_GRID_OK);
    assert(spatial_grid_prepare(&grid, 25, 13, 6) == SPATIAL_GRID_OK);
    assert(grid.columns == 3);
    assert(grid.rows == 2);
    assert(grid.cell_count == 6);
    const int *counts = grid.counts;
    const int *offsets = grid.offsets;
    const int *indices = grid.indices;
    assert(spatial_grid_prepare(&grid, 25, 13, 6) == SPATIAL_GRID_OK);
    assert(grid.counts == counts && grid.offsets == offsets && grid.indices == indices);
    assert(spatial_grid_build(&grid, 6, read_point, points) == SPATIAL_GRID_OK);

    assert(grid.offsets[grid.cell_count] == 6);
    assert(grid.counts[0] == 3);
    assert(range_contains(&grid, 0, 0));
    assert(range_contains(&grid, 0, 1));
    assert(range_contains(&grid, 0, 4));
    assert(range_contains(&grid, 1, 2));
    assert(range_contains(&grid, 5, 3));
    assert(range_contains(&grid, 5, 5));

    int cell_x, cell_y;
    spatial_grid_cell_for_position(&grid, -INFINITY, NAN, &cell_x, &cell_y);
    assert(cell_x == 0 && cell_y == 0);
    spatial_grid_cell_for_position(&grid, INFINITY, INFINITY, &cell_x, &cell_y);
    assert(cell_x == 2 && cell_y == 1);

    assert(spatial_grid_prepare(&grid, 12, 12, 6) == SPATIAL_GRID_OK);
    assert(grid.columns == 1 && grid.rows == 1);
    assert(spatial_grid_build(&grid, 6, read_point, points) == SPATIAL_GRID_OK);
    assert(grid.counts[0] == 6);
    spatial_grid_destroy(&grid);
}

static uint32_t next_random(uint32_t *state) {
    *state = *state * 1664525u + 1013904223u;
    return *state;
}

static double random_coordinate(uint32_t *state, int span, int offset) {
    return (double)(next_random(state) % (uint32_t)span) / 10.0 - offset;
}

static void assert_neighbors_match(const spatial_grid_t *grid, const point_t *points, int count,
                                   int target_index, int vision_cells) {
    unsigned char *expected = calloc((size_t)count, 1);
    unsigned char *actual = calloc((size_t)count, 1);
    assert(expected != NULL && actual != NULL);

    double radius = vision_cells * CELL_SIZE;
    double radius_squared = radius * radius;
    const point_t *target = &points[target_index];
    for (int i = 0; i < count; i++) {
        if (i == target_index) continue;
        double dx = target->x - points[i].x;
        double dy = target->y - points[i].y;
        if (dx * dx + dy * dy < radius_squared) expected[i] = 1;
    }

    int center_x, center_y;
    spatial_grid_cell_for_position(grid, target->x, target->y, &center_x, &center_y);
    int min_x = center_x - vision_cells;
    int max_x = center_x + vision_cells;
    int min_y = center_y - vision_cells;
    int max_y = center_y + vision_cells;
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
                double dx = target->x - points[i].x;
                double dy = target->y - points[i].y;
                if (dx * dx + dy * dy >= radius_squared) continue;
                assert(actual[i] == 0);
                actual[i] = 1;
            }
        }
    }

    assert(memcmp(expected, actual, (size_t)count) == 0);
    free(expected);
    free(actual);
}

static void test_against_brute_force(void) {
    enum { POINT_COUNT = 512 };
    point_t points[POINT_COUNT];
    uint32_t random_state = 0x6c8e9cf5u;
    spatial_grid_t grid;

    for (int i = 0; i < POINT_COUNT; i++) {
        points[i].x = random_coordinate(&random_state, 9000, 130);
        points[i].y = random_coordinate(&random_state, 6000, 100);
    }
    /* Exercise exact cell boundaries and points immediately around them. */
    points[0] = (point_t){12, 12};
    points[1] = (point_t){24, 12};
    points[2] = (point_t){36, 36};
    points[3] = (point_t){-1, 20};
    points[4] = (point_t){641, 20};

    assert(spatial_grid_init(&grid, CELL_SIZE) == SPATIAL_GRID_OK);
    assert(spatial_grid_prepare(&grid, 640, 384, POINT_COUNT) == SPATIAL_GRID_OK);
    assert(spatial_grid_build(&grid, POINT_COUNT, read_point, points) == SPATIAL_GRID_OK);
    for (int vision_cells = 1; vision_cells <= 12; vision_cells++)
        for (int target = 0; target < POINT_COUNT; target++)
            assert_neighbors_match(&grid, points, POINT_COUNT, target, vision_cells);

    spatial_grid_destroy(&grid);
}

static void test_invalid_arguments(void) {
    spatial_grid_t grid;
    assert(spatial_grid_init(NULL, CELL_SIZE) == SPATIAL_GRID_ERR_ARGUMENT);
    assert(spatial_grid_init(&grid, 0) == SPATIAL_GRID_ERR_ARGUMENT);
    assert(spatial_grid_init(&grid, CELL_SIZE) == SPATIAL_GRID_OK);
    assert(spatial_grid_prepare(&grid, 0, 10, 1) == SPATIAL_GRID_ERR_ARGUMENT);
    assert(spatial_grid_build(&grid, 0, read_point, NULL) == SPATIAL_GRID_ERR_ARGUMENT);
    spatial_grid_destroy(&grid);
}

int main(void) {
    test_layout_and_clamping();
    test_against_brute_force();
    test_invalid_arguments();
    return 0;
}
