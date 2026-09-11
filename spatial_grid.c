#include "spatial_grid.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

spatial_grid_status_t spatial_grid_init(spatial_grid_t *grid, int cell_size) {
    if (grid == NULL || cell_size <= 0) return SPATIAL_GRID_ERR_ARGUMENT;
    memset(grid, 0, sizeof(*grid));
    grid->cell_size = cell_size;
    return SPATIAL_GRID_OK;
}

void spatial_grid_destroy(spatial_grid_t *grid) {
    if (grid == NULL) return;
    free(grid->counts);
    free(grid->offsets);
    free(grid->indices);
    memset(grid, 0, sizeof(*grid));
}

spatial_grid_status_t spatial_grid_prepare(spatial_grid_t *grid, int width, int height,
                                           int item_capacity) {
    if (grid == NULL || grid->cell_size <= 0 || width <= 0 || height <= 0 || item_capacity <= 0)
        return SPATIAL_GRID_ERR_ARGUMENT;

    int columns = width / grid->cell_size + (width % grid->cell_size != 0);
    int rows = height / grid->cell_size + (height % grid->cell_size != 0);
    size_t cell_count = (size_t)columns * (size_t)rows;
    if (cell_count == 0 || cell_count > INT_MAX || cell_count > SIZE_MAX / sizeof(int) - 1)
        return SPATIAL_GRID_ERR_MEMORY;

    if (grid->columns == columns && grid->rows == rows && grid->item_capacity >= item_capacity)
        return SPATIAL_GRID_OK;

    int capacity = grid->item_capacity > item_capacity ? grid->item_capacity : item_capacity;
    int *counts = calloc(cell_count, sizeof(*counts));
    int *offsets = malloc((cell_count + 1) * sizeof(*offsets));
    int *indices = malloc((size_t)capacity * sizeof(*indices));
    if (counts == NULL || offsets == NULL || indices == NULL) {
        free(counts);
        free(offsets);
        free(indices);
        return SPATIAL_GRID_ERR_MEMORY;
    }

    free(grid->counts);
    free(grid->offsets);
    free(grid->indices);
    grid->columns = columns;
    grid->rows = rows;
    grid->cell_count = (int)cell_count;
    grid->item_capacity = capacity;
    grid->counts = counts;
    grid->offsets = offsets;
    grid->indices = indices;
    return SPATIAL_GRID_OK;
}

static int coordinate_to_cell(double position, int cells, int cell_size) {
    if (isnan(position) || position <= 0) return 0;
    if (!isfinite(position) || position >= (double)cells * cell_size) return cells - 1;
    return (int)(position / cell_size);
}

void spatial_grid_cell_for_position(const spatial_grid_t *grid, double x, double y, int *cell_x,
                                    int *cell_y) {
    if (grid == NULL || grid->columns <= 0 || grid->rows <= 0) {
        if (cell_x != NULL) *cell_x = 0;
        if (cell_y != NULL) *cell_y = 0;
        return;
    }
    if (cell_x != NULL) *cell_x = coordinate_to_cell(x, grid->columns, grid->cell_size);
    if (cell_y != NULL) *cell_y = coordinate_to_cell(y, grid->rows, grid->cell_size);
}

static int cell_for_item(const spatial_grid_t *grid, spatial_grid_position_reader_t reader,
                         const void *context, int index) {
    double x, y;
    int cell_x, cell_y;
    reader(context, index, &x, &y);
    spatial_grid_cell_for_position(grid, x, y, &cell_x, &cell_y);
    return cell_y * grid->columns + cell_x;
}

spatial_grid_status_t spatial_grid_build(spatial_grid_t *grid, int item_count,
                                         spatial_grid_position_reader_t reader,
                                         const void *context) {
    if (grid == NULL || reader == NULL || item_count < 0 || item_count > grid->item_capacity ||
        grid->cell_count <= 0 || grid->counts == NULL || grid->offsets == NULL ||
        grid->indices == NULL)
        return SPATIAL_GRID_ERR_ARGUMENT;

    memset(grid->counts, 0, (size_t)grid->cell_count * sizeof(*grid->counts));
    for (int i = 0; i < item_count; i++) grid->counts[cell_for_item(grid, reader, context, i)]++;

    grid->offsets[0] = 0;
    for (int cell = 0; cell < grid->cell_count; cell++)
        grid->offsets[cell + 1] = grid->offsets[cell] + grid->counts[cell];

    memset(grid->counts, 0, (size_t)grid->cell_count * sizeof(*grid->counts));
    for (int i = 0; i < item_count; i++) {
        int cell = cell_for_item(grid, reader, context, i);
        int slot = grid->offsets[cell] + grid->counts[cell]++;
        grid->indices[slot] = i;
    }
    return SPATIAL_GRID_OK;
}

const char *spatial_grid_status_string(spatial_grid_status_t status) {
    switch (status) {
        case SPATIAL_GRID_OK:
            return "ok";
        case SPATIAL_GRID_ERR_ARGUMENT:
            return "invalid argument";
        case SPATIAL_GRID_ERR_MEMORY:
            return "out of memory";
    }
    return "unknown error";
}
