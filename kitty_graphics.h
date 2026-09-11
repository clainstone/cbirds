#ifndef KITTY_GRAPHICS_H
#define KITTY_GRAPHICS_H

#include <stddef.h>
#include <stdint.h>

enum { KITTY_GRAPHICS_PAYLOAD_MAX = 4096 };

typedef enum {
    KITTY_GRAPHICS_OK = 0,
    KITTY_GRAPHICS_ERR_ARGUMENT,
    KITTY_GRAPHICS_ERR_MEMORY,
    KITTY_GRAPHICS_ERR_IO
} kitty_graphics_status_t;

typedef struct {
    int output_fd;
    char *buffer;
    size_t length;
    size_t capacity;
} kitty_graphics_t;

typedef struct {
    uint32_t image_id;
    uint32_t placement_id;
    int row;      /* Zero-based terminal row. */
    int column;   /* Zero-based terminal column. */
    int x_offset; /* Pixel offset within the terminal cell. */
    int y_offset;
    int z_index;
} kitty_graphics_placement_t;

/* The context does not own output_fd and never closes it. */
kitty_graphics_status_t kitty_graphics_init(kitty_graphics_t *graphics, int output_fd);
void kitty_graphics_destroy(kitty_graphics_t *graphics);

/* PNG data is Base64 encoded and split into protocol-sized chunks internally. */
kitty_graphics_status_t kitty_graphics_upload_png(kitty_graphics_t *graphics, uint32_t image_id,
                                                  const uint8_t *png, size_t png_length);

kitty_graphics_status_t kitty_graphics_place(kitty_graphics_t *graphics,
                                             const kitty_graphics_placement_t *placement);
kitty_graphics_status_t kitty_graphics_delete_all_placements(kitty_graphics_t *graphics);
kitty_graphics_status_t kitty_graphics_delete_image(kitty_graphics_t *graphics, uint32_t image_id);

/* Writes all queued commands. Successfully written commands are removed. */
kitty_graphics_status_t kitty_graphics_flush(kitty_graphics_t *graphics);

const char *kitty_graphics_status_string(kitty_graphics_status_t status);

#endif
