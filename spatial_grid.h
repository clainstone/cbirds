#ifndef SPATIAL_GRID_H
#define SPATIAL_GRID_H

typedef enum {
    SPATIAL_GRID_OK = 0,
    SPATIAL_GRID_ERR_ARGUMENT,
    SPATIAL_GRID_ERR_MEMORY
} spatial_grid_status_t;

typedef void (*spatial_grid_position_reader_t)(const void *context, int index, double *x,
                                               double *y);

typedef struct {
    int cell_size;
    int columns;
    int rows;
    int cell_count;
    int item_capacity;
    int *counts;
    int *offsets;
    int *indices;
} spatial_grid_t;

spatial_grid_status_t spatial_grid_init(spatial_grid_t *grid, int cell_size);
void spatial_grid_destroy(spatial_grid_t *grid);

/* Allocates only when the grid dimensions change or item_capacity grows. */
spatial_grid_status_t spatial_grid_prepare(spatial_grid_t *grid, int width, int height,
                                           int item_capacity);

/* Rebuilds the cell ranges without allocating. The reader is called twice per item. */
spatial_grid_status_t spatial_grid_build(spatial_grid_t *grid, int item_count,
                                         spatial_grid_position_reader_t reader,
                                         const void *context);

/* Positions outside the grid are mapped to the nearest border cell. */
void spatial_grid_cell_for_position(const spatial_grid_t *grid, double x, double y, int *cell_x,
                                    int *cell_y);

const char *spatial_grid_status_string(spatial_grid_status_t status);

#endif
