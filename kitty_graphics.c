#include "kitty_graphics.h"

#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

enum {
    KITTY_PNG_FORMAT = 100,
    /* 3072 input bytes encode to exactly 4096 Base64 bytes. */
    KITTY_RAW_CHUNK_MAX = KITTY_GRAPHICS_PAYLOAD_MAX * 3 / 4,
    KITTY_INITIAL_CAPACITY = 4096
};

static const char base64_chars[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static kitty_graphics_status_t reserve(kitty_graphics_t *graphics, size_t extra) {
    if (graphics->length == SIZE_MAX || extra > SIZE_MAX - graphics->length - 1)
        return KITTY_GRAPHICS_ERR_MEMORY;

    size_t needed = graphics->length + extra + 1;
    if (needed <= graphics->capacity) return KITTY_GRAPHICS_OK;

    size_t capacity = graphics->capacity ? graphics->capacity : KITTY_INITIAL_CAPACITY;
    while (capacity < needed) {
        if (capacity > SIZE_MAX / 2) {
            capacity = needed;
            break;
        }
        capacity *= 2;
    }

    char *grown = realloc(graphics->buffer, capacity);
    if (grown == NULL) return KITTY_GRAPHICS_ERR_MEMORY;
    graphics->buffer = grown;
    graphics->capacity = capacity;
    return KITTY_GRAPHICS_OK;
}

static kitty_graphics_status_t append_bytes(kitty_graphics_t *graphics, const void *data,
                                            size_t length) {
    kitty_graphics_status_t status = reserve(graphics, length);
    if (status != KITTY_GRAPHICS_OK) return status;

    memcpy(graphics->buffer + graphics->length, data, length);
    graphics->length += length;
    graphics->buffer[graphics->length] = '\0';
    return KITTY_GRAPHICS_OK;
}

static kitty_graphics_status_t append_format(kitty_graphics_t *graphics, const char *format, ...) {
    va_list arguments, copy;
    va_start(arguments, format);
    va_copy(copy, arguments);
    int length = vsnprintf(NULL, 0, format, copy);
    va_end(copy);
    if (length < 0) {
        va_end(arguments);
        return KITTY_GRAPHICS_ERR_ARGUMENT;
    }

    kitty_graphics_status_t status = reserve(graphics, (size_t)length);
    if (status == KITTY_GRAPHICS_OK) {
        vsnprintf(graphics->buffer + graphics->length, (size_t)length + 1, format, arguments);
        graphics->length += (size_t)length;
    }
    va_end(arguments);
    return status;
}

static size_t base64_encode_chunk(const uint8_t *input, size_t length,
                                  char output[KITTY_GRAPHICS_PAYLOAD_MAX]) {
    size_t in = 0, out = 0;
    while (in < length) {
        uint32_t a = input[in++];
        uint32_t b = in < length ? input[in++] : 0;
        uint32_t c = in < length ? input[in++] : 0;
        uint32_t triple = (a << 16) | (b << 8) | c;
        output[out++] = base64_chars[(triple >> 18) & 63];
        output[out++] = base64_chars[(triple >> 12) & 63];
        output[out++] = base64_chars[(triple >> 6) & 63];
        output[out++] = base64_chars[triple & 63];
    }
    if (length % 3 == 1) output[out - 2] = '=';
    if (length % 3 != 0) output[out - 1] = '=';
    return out;
}

kitty_graphics_status_t kitty_graphics_init(kitty_graphics_t *graphics, int output_fd) {
    if (graphics == NULL || output_fd < 0) return KITTY_GRAPHICS_ERR_ARGUMENT;
    graphics->output_fd = output_fd;
    graphics->buffer = NULL;
    graphics->length = 0;
    graphics->capacity = 0;
    return KITTY_GRAPHICS_OK;
}

void kitty_graphics_destroy(kitty_graphics_t *graphics) {
    if (graphics == NULL) return;
    free(graphics->buffer);
    graphics->output_fd = -1;
    graphics->buffer = NULL;
    graphics->length = 0;
    graphics->capacity = 0;
}

kitty_graphics_status_t kitty_graphics_upload_png(kitty_graphics_t *graphics, uint32_t image_id,
                                                  const uint8_t *png, size_t png_length) {
    if (graphics == NULL || graphics->output_fd < 0 || image_id == 0 || png == NULL ||
        png_length == 0)
        return KITTY_GRAPHICS_ERR_ARGUMENT;

    size_t original_length = graphics->length;
    size_t offset = 0;
    int first = 1;
    while (offset < png_length) {
        size_t raw_length = png_length - offset;
        if (raw_length > KITTY_RAW_CHUNK_MAX) raw_length = KITTY_RAW_CHUNK_MAX;
        int more = offset + raw_length < png_length;

        char payload[KITTY_GRAPHICS_PAYLOAD_MAX];
        size_t payload_length = base64_encode_chunk(png + offset, raw_length, payload);
        kitty_graphics_status_t status;
        if (first) {
            status = append_format(graphics, "\033_Ga=t,q=2,f=%d,I=%" PRIu32 ",m=%d;",
                                   KITTY_PNG_FORMAT, image_id, more);
            first = 0;
        } else {
            status = append_format(graphics, "\033_Gm=%d,q=2;", more);
        }
        if (status == KITTY_GRAPHICS_OK) status = append_bytes(graphics, payload, payload_length);
        if (status == KITTY_GRAPHICS_OK) status = append_bytes(graphics, "\033\\", 2);
        if (status != KITTY_GRAPHICS_OK) {
            graphics->length = original_length;
            if (graphics->buffer != NULL) graphics->buffer[graphics->length] = '\0';
            return status;
        }
        offset += raw_length;
    }
    return KITTY_GRAPHICS_OK;
}

kitty_graphics_status_t kitty_graphics_place(kitty_graphics_t *graphics,
                                             const kitty_graphics_placement_t *placement) {
    if (graphics == NULL || graphics->output_fd < 0 || placement == NULL ||
        placement->image_id == 0 || placement->row < 0 || placement->column < 0 ||
        placement->x_offset < 0 || placement->y_offset < 0)
        return KITTY_GRAPHICS_ERR_ARGUMENT;

    if (placement->placement_id == 0 && placement->z_index == 0)
        return append_format(graphics, "\033[%d;%dH\033_Ga=p,I=%" PRIu32 ",q=2,X=%d,Y=%d,C=1\033\\",
                             placement->row + 1, placement->column + 1, placement->image_id,
                             placement->x_offset, placement->y_offset);
    if (placement->placement_id == 0)
        return append_format(graphics,
                             "\033[%d;%dH\033_Ga=p,I=%" PRIu32 ",q=2,X=%d,Y=%d,z=%d,C=1\033\\",
                             placement->row + 1, placement->column + 1, placement->image_id,
                             placement->x_offset, placement->y_offset, placement->z_index);
    if (placement->z_index == 0)
        return append_format(
            graphics, "\033[%d;%dH\033_Ga=p,I=%" PRIu32 ",q=2,p=%" PRIu32 ",X=%d,Y=%d,C=1\033\\",
            placement->row + 1, placement->column + 1, placement->image_id, placement->placement_id,
            placement->x_offset, placement->y_offset);
    return append_format(
        graphics, "\033[%d;%dH\033_Ga=p,I=%" PRIu32 ",q=2,p=%" PRIu32 ",X=%d,Y=%d,z=%d,C=1\033\\",
        placement->row + 1, placement->column + 1, placement->image_id, placement->placement_id,
        placement->x_offset, placement->y_offset, placement->z_index);
}

kitty_graphics_status_t kitty_graphics_delete_placement(kitty_graphics_t *graphics,
                                                        uint32_t image_id, uint32_t placement_id) {
    if (graphics == NULL || graphics->output_fd < 0 || image_id == 0 || placement_id == 0)
        return KITTY_GRAPHICS_ERR_ARGUMENT;
    return append_format(graphics, "\033_Ga=d,d=n,I=%" PRIu32 ",p=%" PRIu32 ",q=2\033\\", image_id,
                         placement_id);
}

kitty_graphics_status_t kitty_graphics_delete_all_placements(kitty_graphics_t *graphics) {
    if (graphics == NULL || graphics->output_fd < 0) return KITTY_GRAPHICS_ERR_ARGUMENT;
    return append_bytes(graphics, "\033_Ga=d,d=a\033\\", sizeof("\033_Ga=d,d=a\033\\") - 1);
}

kitty_graphics_status_t kitty_graphics_delete_image(kitty_graphics_t *graphics, uint32_t image_id) {
    if (graphics == NULL || graphics->output_fd < 0 || image_id == 0)
        return KITTY_GRAPHICS_ERR_ARGUMENT;
    /* N selects the newest image with the supplied image number and frees its data. */
    return append_format(graphics, "\033_Ga=d,d=N,I=%" PRIu32 "\033\\", image_id);
}

kitty_graphics_status_t kitty_graphics_write_text(kitty_graphics_t *graphics, int row, int column,
                                                  const char *text) {
    if (graphics == NULL || graphics->output_fd < 0 || row < 0 || column < 0 || text == NULL)
        return KITTY_GRAPHICS_ERR_ARGUMENT;
    return append_format(graphics, "\033[%d;%dH%s", row + 1, column + 1, text);
}

kitty_graphics_status_t kitty_graphics_clear_screen(kitty_graphics_t *graphics) {
    if (graphics == NULL || graphics->output_fd < 0) return KITTY_GRAPHICS_ERR_ARGUMENT;
    return append_bytes(graphics, "\033[2J", sizeof("\033[2J") - 1);
}

kitty_graphics_status_t kitty_graphics_begin_synchronized_update(kitty_graphics_t *graphics) {
    if (graphics == NULL || graphics->output_fd < 0) return KITTY_GRAPHICS_ERR_ARGUMENT;
    return append_bytes(graphics, "\033[?2026h", sizeof("\033[?2026h") - 1);
}

kitty_graphics_status_t kitty_graphics_end_synchronized_update(kitty_graphics_t *graphics) {
    if (graphics == NULL || graphics->output_fd < 0) return KITTY_GRAPHICS_ERR_ARGUMENT;
    return append_bytes(graphics, "\033[?2026l", sizeof("\033[?2026l") - 1);
}

static void discard_written_prefix(kitty_graphics_t *graphics, size_t written) {
    if (written == 0) return;
    graphics->length -= written;
    memmove(graphics->buffer, graphics->buffer + written, graphics->length);
    graphics->buffer[graphics->length] = '\0';
}

static kitty_graphics_status_t flush_buffer(kitty_graphics_t *graphics, int nonblocking) {
    if (graphics == NULL || graphics->output_fd < 0) return KITTY_GRAPHICS_ERR_ARGUMENT;

    size_t written = 0;
    while (written < graphics->length) {
        ssize_t result =
            write(graphics->output_fd, graphics->buffer + written, graphics->length - written);
        if (result < 0) {
            if (errno == EINTR) continue;
            discard_written_prefix(graphics, written);
            if (nonblocking && (errno == EAGAIN || errno == EWOULDBLOCK))
                return KITTY_GRAPHICS_AGAIN;
            return KITTY_GRAPHICS_ERR_IO;
        }
        if (result == 0) {
            errno = EIO;
            discard_written_prefix(graphics, written);
            return KITTY_GRAPHICS_ERR_IO;
        }
        written += (size_t)result;
    }
    graphics->length = 0;
    if (graphics->buffer != NULL) graphics->buffer[0] = '\0';
    return KITTY_GRAPHICS_OK;
}

kitty_graphics_status_t kitty_graphics_flush(kitty_graphics_t *graphics) {
    return flush_buffer(graphics, 0);
}

kitty_graphics_status_t kitty_graphics_flush_nonblocking(kitty_graphics_t *graphics) {
    if (graphics == NULL || graphics->output_fd < 0) return KITTY_GRAPHICS_ERR_ARGUMENT;

    int flags = fcntl(graphics->output_fd, F_GETFL);
    if (flags < 0) return KITTY_GRAPHICS_ERR_IO;
    int changed_flags = !(flags & O_NONBLOCK);
    if (changed_flags && fcntl(graphics->output_fd, F_SETFL, flags | O_NONBLOCK) < 0)
        return KITTY_GRAPHICS_ERR_IO;

    kitty_graphics_status_t status = flush_buffer(graphics, 1);
    int write_errno = errno;
    if (changed_flags && fcntl(graphics->output_fd, F_SETFL, flags) < 0)
        return KITTY_GRAPHICS_ERR_IO;
    errno = write_errno;
    return status;
}

const char *kitty_graphics_status_string(kitty_graphics_status_t status) {
    switch (status) {
        case KITTY_GRAPHICS_OK:
            return "ok";
        case KITTY_GRAPHICS_ERR_ARGUMENT:
            return "invalid argument";
        case KITTY_GRAPHICS_ERR_MEMORY:
            return "out of memory";
        case KITTY_GRAPHICS_ERR_IO:
            return "output error";
        case KITTY_GRAPHICS_AGAIN:
            return "output would block";
    }
    return "unknown error";
}
