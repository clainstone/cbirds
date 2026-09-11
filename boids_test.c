#define main cbirds_application_main
#include "boids.c"
#undef main

#include <assert.h>

static double brute_force_flock_direction(const bird_t *birds, int target_index) {
    const bird_t *target = &birds[target_index];
    vector_t separation = {0, 0}, alignment = {0, 0}, cohesion = {0, 0};
    vector_t boundary = boundary_vector(target);
    int neighbors = 0;

    for (int i = 0; i < config.birds; i++) {
        if (i == target_index) continue;
        const bird_t *other = &birds[i];
        double dx = target->x - other->x;
        double dy = target->y - other->y;
        if (dx * dx + dy * dy >= config.vision_radius_squared) continue;
        separation.x += dx;
        separation.y += dy;
        alignment.x += cos(other->direction);
        alignment.y += sin(other->direction);
        cohesion.x += other->x;
        cohesion.y += other->y;
        neighbors++;
    }

    if (neighbors) {
        alignment.x /= neighbors;
        alignment.y /= neighbors;
        cohesion.x = cohesion.x / neighbors - target->x;
        cohesion.y = cohesion.y / neighbors - target->y;
        double x = separation.x * config.separation + alignment.x * config.alignment +
                   cohesion.x * config.cohesion + boundary.x * config.boundary;
        double y = separation.y * config.separation + alignment.y * config.alignment +
                   cohesion.y * config.cohesion + boundary.y * config.boundary;
        return x == 0 && y == 0 ? target->direction : normalized_angle(y, x);
    }

    boundary.x *= config.boundary;
    boundary.y *= config.boundary;
    if (boundary.x != 0 || boundary.y != 0) {
        double x = cos(target->direction) + boundary.x;
        double y = sin(target->direction) + boundary.y;
        if (x != 0 || y != 0) return normalized_angle(y, x);
    }
    return target->direction;
}

static double angle_difference(double a, double b) {
    return fabs(atan2(sin(a - b), cos(a - b)));
}

static uint32_t test_random(uint32_t *state) {
    *state = *state * 1103515245u + 12345u;
    return *state;
}

static void initialize_test_birds(bird_t *birds, int count) {
    uint32_t state = 0x93d765b1u;
    for (int i = 0; i < count; i++) {
        birds[i].x = (double)(test_random(&state) % 7600u) / 10.0 - 60.0;
        birds[i].y = (double)(test_random(&state) % 5000u) / 10.0 - 50.0;
        birds[i].direction = (double)(test_random(&state) % 3600u) * M_PI / 1800.0;
        birds[i].frame = direction_frame(birds[i].direction);
    }
    birds[0] = (bird_t){12, 12, 0, 0};
    birds[1] = (bird_t){24, 12, M_PI / 2, 0};
    birds[2] = (bird_t){36, 36, M_PI, 0};
    birds[3] = (bird_t){-1, 20, M_PI / 4, 0};
    birds[4] = (bird_t){641, 20, 3 * M_PI / 2, 0};
}

static void test_engine_matches_brute_force(void) {
    enum { BIRD_COUNT = 256 };
    bird_t snapshot[BIRD_COUNT], optimized[BIRD_COUNT], reference[BIRD_COUNT];
    spatial_grid_t grid;

    screen.width = 640;
    screen.height = 384;
    screen.turn_x = screen.width / 3;
    screen.turn_y = screen.height / 3;
    config.birds = BIRD_COUNT;
    config.speed = 0.75;
    config.separation = 0.005;
    config.alignment = 1.5;
    config.cohesion = 0.01;
    config.boundary = 0.2;
    initialize_test_birds(snapshot, BIRD_COUNT);

    assert(spatial_grid_init(&grid, SPATIAL_CELL_SIZE) == SPATIAL_GRID_OK);
    assert(spatial_grid_prepare(&grid, screen.width, screen.height, BIRD_COUNT) == SPATIAL_GRID_OK);
    assert(spatial_grid_build(&grid, BIRD_COUNT, read_bird_position, snapshot) == SPATIAL_GRID_OK);

    for (config.vision_cells = MIN_VISION_CELLS; config.vision_cells <= MAX_VISION_CELLS;
         config.vision_cells++) {
        update_vision_radius();
        for (int i = 0; i < BIRD_COUNT; i++) {
            double expected = brute_force_flock_direction(snapshot, i);
            double actual = flock_direction(snapshot, &grid, i);
            assert(angle_difference(expected, actual) < 1e-11);
        }
    }

    config.vision_cells = DEFAULT_VISION_CELLS;
    update_vision_radius();
    memcpy(optimized, snapshot, sizeof(snapshot));
    memcpy(reference, snapshot, sizeof(snapshot));
    update_birds(optimized, snapshot, &grid);
    for (int i = 0; i < BIRD_COUNT; i++) {
        double direction = brute_force_flock_direction(snapshot, i);
        reference[i].direction = direction;
        reference[i].x += config.speed * cos(direction);
        reference[i].y += config.speed * sin(direction);
        optimized[i].frame = direction_frame(optimized[i].direction);
        reference[i].frame = direction_frame(reference[i].direction);
        assert(angle_difference(optimized[i].direction, reference[i].direction) < 1e-11);
        assert(fabs(optimized[i].x - reference[i].x) < 1e-11);
        assert(fabs(optimized[i].y - reference[i].y) < 1e-11);
        assert(optimized[i].frame == reference[i].frame);
    }

    spatial_grid_destroy(&grid);
}

static int feed_input(const char *keys) {
    int descriptors[2];
    assert(pipe(descriptors) == 0);
    int saved_stdin = dup(STDIN_FILENO);
    assert(saved_stdin >= 0);
    size_t length = strlen(keys);
    assert(write(descriptors[1], keys, length) == (ssize_t)length);
    close(descriptors[1]);
    assert(dup2(descriptors[0], STDIN_FILENO) == STDIN_FILENO);
    close(descriptors[0]);
    int result = handle_input();
    assert(dup2(saved_stdin, STDIN_FILENO) == STDIN_FILENO);
    close(saved_stdin);
    return result;
}

static void test_vision_controls(void) {
    config.vision_cells = DEFAULT_VISION_CELLS;
    update_vision_radius();
    assert(config.vision_radius == 36);
    assert(config.vision_radius_squared == 1296);

    assert(feed_input("P") == 1);
    assert(config.vision_cells == 4);
    assert(config.vision_radius == 48);
    assert(config.vision_radius_squared == 2304);

    config.vision_cells = MAX_VISION_CELLS;
    update_vision_radius();
    assert(feed_input("PP") == 1);
    assert(config.vision_cells == MAX_VISION_CELLS);
    assert(config.vision_radius == 144);

    config.vision_cells = MIN_VISION_CELLS;
    update_vision_radius();
    assert(feed_input("pp") == 1);
    assert(config.vision_cells == MIN_VISION_CELLS);
    assert(config.vision_radius == 12);
    assert(feed_input("q") == 0);
}

static void test_frame_rate_controls(void) {
    char keys[INPUT_BUFFER_SIZE + 1];

    config.frame_rate = DEFAULT_FRAME_RATE;
    update_speed();
    memset(keys, 'R', INPUT_BUFFER_SIZE);
    keys[INPUT_BUFFER_SIZE] = '\0';
    assert(feed_input(keys) == 1);
    assert(config.frame_rate == MAX_FRAME_RATE);
    assert(config.speed == 20.0);

    memset(keys, 'r', INPUT_BUFFER_SIZE);
    assert(feed_input(keys) == 1);
    assert(config.frame_rate == MIN_FRAME_RATE);
    assert(config.speed == 80.0);

    memset(keys, 'R', INPUT_BUFFER_SIZE);
    keys[INPUT_BUFFER_SIZE - 1] = 'q';
    assert(feed_input(keys) == 0);
    assert(config.frame_rate == MAX_FRAME_RATE);

    config.frame_rate = DEFAULT_FRAME_RATE;
    update_speed();
}

static void clear_graphics_buffer(kitty_graphics_t *graphics) {
    graphics->length = 0;
    if (graphics->buffer != NULL) graphics->buffer[0] = '\0';
}

static void test_flicker_free_render_queue(void) {
    static const char begin_update[] = "\033[?2026h";
    static const char end_update[] = "\033[?2026l";
    kitty_graphics_t graphics;
    bird_t bird = {.x = 9, .y = 17, .direction = 0, .frame = 3};

    config.birds = 1;
    screen.cols = 80;
    screen.rows = 24;
    screen.cell_width = 8;
    screen.cell_height = 16;
    assert(kitty_graphics_init(&graphics, STDOUT_FILENO) == KITTY_GRAPHICS_OK);

    assert(queue_render_frame(&graphics, &bird) == KITTY_GRAPHICS_OK);
    assert(memcmp(graphics.buffer, begin_update, sizeof(begin_update) - 1) == 0);
    const char *clear = strstr(graphics.buffer, "a=d,d=a");
    const char *placement = strstr(graphics.buffer, "a=p,I=4,q=2,X=1,Y=1,C=1");
    assert(clear != NULL && placement != NULL && clear < placement);
    assert(strstr(graphics.buffer, "a=d,d=n") == NULL);
    assert(strcmp(graphics.buffer + graphics.length - (sizeof(end_update) - 1), end_update) == 0);

    clear_graphics_buffer(&graphics);
    bird.x = 18;
    assert(queue_render_frame(&graphics, &bird) == KITTY_GRAPHICS_OK);
    assert(strstr(graphics.buffer, "a=p,I=4,q=2,X=2,Y=1,C=1") != NULL);
    assert(strstr(graphics.buffer, "a=d,d=a") != NULL);

    clear_graphics_buffer(&graphics);
    bird.frame = 4;
    assert(queue_render_frame(&graphics, &bird) == KITTY_GRAPHICS_OK);
    assert(strstr(graphics.buffer, "a=p,I=5,q=2,X=2,Y=1,C=1") != NULL);
    assert(strstr(graphics.buffer, "a=d,d=n") == NULL);

    clear_graphics_buffer(&graphics);
    bird.x = -1;
    assert(queue_render_frame(&graphics, &bird) == KITTY_GRAPHICS_OK);
    assert(strstr(graphics.buffer, "a=p") == NULL);
    assert(strstr(graphics.buffer, "a=d,d=a") != NULL);

    kitty_graphics_destroy(&graphics);
}

int main(void) {
    test_engine_matches_brute_force();
    test_vision_controls();
    test_frame_rate_controls();
    test_flicker_free_render_queue();
    return 0;
}
