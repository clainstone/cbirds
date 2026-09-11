#include "kitty_graphics.h"

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static void test_small_upload(void) {
    static const uint8_t png[] = {0x00, 0xff, 0x10};
    static const char expected[] = "\033_Ga=t,q=2,f=100,I=7,m=0;AP8Q\033\\";
    kitty_graphics_t graphics;

    assert(kitty_graphics_init(&graphics, STDOUT_FILENO) == KITTY_GRAPHICS_OK);
    assert(kitty_graphics_upload_png(&graphics, 7, png, sizeof(png)) == KITTY_GRAPHICS_OK);
    assert(graphics.length == sizeof(expected) - 1);
    assert(memcmp(graphics.buffer, expected, sizeof(expected) - 1) == 0);
    kitty_graphics_destroy(&graphics);
}

static void test_chunked_upload(void) {
    const size_t input_length = KITTY_GRAPHICS_PAYLOAD_MAX * 3 / 4 + 1;
    uint8_t *png = calloc(input_length, 1);
    kitty_graphics_t graphics;
    assert(png != NULL);
    assert(kitty_graphics_init(&graphics, STDOUT_FILENO) == KITTY_GRAPHICS_OK);
    assert(kitty_graphics_upload_png(&graphics, 42, png, input_length) == KITTY_GRAPHICS_OK);

    static const char first_prefix[] = "\033_Ga=t,q=2,f=100,I=42,m=1;";
    static const char last_prefix[] = "\033_Gm=0,q=2;";
    assert(memcmp(graphics.buffer, first_prefix, sizeof(first_prefix) - 1) == 0);

    const char *first_payload = graphics.buffer + sizeof(first_prefix) - 1;
    const char *first_end = strstr(first_payload, "\033\\");
    assert(first_end != NULL);
    assert((size_t)(first_end - first_payload) == KITTY_GRAPHICS_PAYLOAD_MAX);
    for (const char *p = first_payload; p < first_end; p++) assert(*p == 'A');

    const char *last = first_end + 2;
    assert(memcmp(last, last_prefix, sizeof(last_prefix) - 1) == 0);
    last += sizeof(last_prefix) - 1;
    assert(memcmp(last, "AA==\033\\", sizeof("AA==\033\\") - 1) == 0);
    assert(last + sizeof("AA==\033\\") - 1 == graphics.buffer + graphics.length);

    kitty_graphics_destroy(&graphics);
    free(png);
}

static void test_placement_and_deletion(void) {
    static const char expected[] =
        "\033_Ga=d,d=a\033\\"
        "\033[3;5H\033_Ga=p,I=9,q=2,p=12,X=3,Y=6,z=-1,C=1\033\\"
        "\033[3;5H\033_Ga=p,I=9,q=2,X=3,Y=6,C=1\033\\"
        "\033_Ga=d,d=n,I=9,p=12,q=2\033\\"
        "\033_Ga=d,d=N,I=9\033\\";
    kitty_graphics_t graphics;
    kitty_graphics_placement_t placement = {
        .image_id = 9,
        .placement_id = 12,
        .row = 2,
        .column = 4,
        .x_offset = 3,
        .y_offset = 6,
        .z_index = -1,
    };

    assert(kitty_graphics_init(&graphics, STDOUT_FILENO) == KITTY_GRAPHICS_OK);
    assert(kitty_graphics_delete_all_placements(&graphics) == KITTY_GRAPHICS_OK);
    assert(kitty_graphics_place(&graphics, &placement) == KITTY_GRAPHICS_OK);
    placement.placement_id = 0;
    placement.z_index = 0;
    assert(kitty_graphics_place(&graphics, &placement) == KITTY_GRAPHICS_OK);
    assert(kitty_graphics_delete_placement(&graphics, 9, 12) == KITTY_GRAPHICS_OK);
    assert(kitty_graphics_delete_image(&graphics, 9) == KITTY_GRAPHICS_OK);
    assert(graphics.length == sizeof(expected) - 1);
    assert(memcmp(graphics.buffer, expected, sizeof(expected) - 1) == 0);
    kitty_graphics_destroy(&graphics);
}

static void test_synchronized_update(void) {
    static const char expected[] = "\033[?2026h\033[?2026l";
    kitty_graphics_t graphics;

    assert(kitty_graphics_init(&graphics, STDOUT_FILENO) == KITTY_GRAPHICS_OK);
    assert(kitty_graphics_begin_synchronized_update(&graphics) == KITTY_GRAPHICS_OK);
    assert(kitty_graphics_end_synchronized_update(&graphics) == KITTY_GRAPHICS_OK);
    assert(graphics.length == sizeof(expected) - 1);
    assert(memcmp(graphics.buffer, expected, sizeof(expected) - 1) == 0);
    kitty_graphics_destroy(&graphics);
}

static void test_flush(void) {
    static const uint8_t png[] = {0};
    int descriptors[2];
    char output[128];
    kitty_graphics_t graphics;

    assert(pipe(descriptors) == 0);
    assert(kitty_graphics_init(&graphics, descriptors[1]) == KITTY_GRAPHICS_OK);
    assert(kitty_graphics_upload_png(&graphics, 1, png, sizeof(png)) == KITTY_GRAPHICS_OK);
    size_t expected_length = graphics.length;
    assert(kitty_graphics_flush(&graphics) == KITTY_GRAPHICS_OK);
    assert(graphics.length == 0);
    assert((size_t)read(descriptors[0], output, sizeof(output)) == expected_length);

    kitty_graphics_destroy(&graphics);
    close(descriptors[0]);
    close(descriptors[1]);
}

static void test_nonblocking_flush_backpressure(void) {
    int descriptors[2];
    char fill[4096] = {0};
    char drain[8192];
    static const uint8_t png[] = {0};
    kitty_graphics_t graphics;

    assert(pipe(descriptors) == 0);
    int flags = fcntl(descriptors[1], F_GETFL);
    assert(flags >= 0);
    assert(fcntl(descriptors[1], F_SETFL, flags | O_NONBLOCK) == 0);
    while (write(descriptors[1], fill, sizeof(fill)) > 0) {
    }
    assert(errno == EAGAIN || errno == EWOULDBLOCK);
    assert(fcntl(descriptors[1], F_SETFL, flags) == 0);

    assert(kitty_graphics_init(&graphics, descriptors[1]) == KITTY_GRAPHICS_OK);
    assert(kitty_graphics_upload_png(&graphics, 1, png, sizeof(png)) == KITTY_GRAPHICS_OK);
    size_t expected_length = graphics.length;
    assert(kitty_graphics_flush_nonblocking(&graphics) == KITTY_GRAPHICS_AGAIN);
    assert(graphics.length == expected_length);
    assert(fcntl(descriptors[1], F_GETFL) == flags);

    assert(read(descriptors[0], drain, sizeof(drain)) > 0);
    assert(kitty_graphics_flush_nonblocking(&graphics) == KITTY_GRAPHICS_OK);
    assert(graphics.length == 0);
    assert(fcntl(descriptors[1], F_GETFL) == flags);

    kitty_graphics_destroy(&graphics);
    close(descriptors[0]);
    close(descriptors[1]);
}

static void test_invalid_arguments(void) {
    kitty_graphics_t graphics;
    kitty_graphics_placement_t placement = {0};
    assert(kitty_graphics_init(NULL, STDOUT_FILENO) == KITTY_GRAPHICS_ERR_ARGUMENT);
    assert(kitty_graphics_init(&graphics, -1) == KITTY_GRAPHICS_ERR_ARGUMENT);
    assert(kitty_graphics_init(&graphics, STDOUT_FILENO) == KITTY_GRAPHICS_OK);
    assert(kitty_graphics_upload_png(&graphics, 0, NULL, 0) == KITTY_GRAPHICS_ERR_ARGUMENT);
    assert(kitty_graphics_place(&graphics, &placement) == KITTY_GRAPHICS_ERR_ARGUMENT);
    assert(kitty_graphics_delete_placement(&graphics, 0, 0) == KITTY_GRAPHICS_ERR_ARGUMENT);
    kitty_graphics_destroy(&graphics);
}

int main(void) {
    test_small_upload();
    test_chunked_upload();
    test_placement_and_deletion();
    test_synchronized_update();
    test_flush();
    test_nonblocking_flush_backpressure();
    test_invalid_arguments();
    return 0;
}
