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

static void set_test_screen(int width, int height) {
    screen.width = width;
    screen.height = height;
    update_turn_distances();
}

static void reset_test_config(void) {
    config.birds = 800;
    config.frame_rate = DEFAULT_FRAME_RATE;
    config.bird_size = DEFAULT_BIRD_SIZE;
    config.vision_cells = DEFAULT_VISION_CELLS;
    config.separation = DEFAULT_SEPARATION_W;
    config.alignment = DEFAULT_ALIGNMENT_W;
    config.cohesion = DEFAULT_COHESION_W;
    config.boundary = DEFAULT_BOUNDARY_W;
    update_speed();
    update_vision_radius();
}

/* The bar carries its own escapes: reverse video and an erase to end of line on
 * the way in, a reset on the way out. These two strip them. */
static const char *legend_text(const char *line) {
    static const char prefix[] = "\033[7m\033[K";
    assert(strncmp(line, prefix, sizeof(prefix) - 1) == 0);
    return line + sizeof(prefix) - 1;
}

static size_t legend_text_length(const char *line) {
    static const char suffix[] = "\033[0m";
    const char *text = legend_text(line);
    size_t length = strlen(text);
    assert(length >= sizeof(suffix) - 1);
    assert(strcmp(text + length - (sizeof(suffix) - 1), suffix) == 0);
    return length - (sizeof(suffix) - 1);
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

    set_test_screen(640, 384);
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

static void test_boundary_bands_follow_the_viewport(void) {
    set_test_screen(900, 600);
    assert(screen.turn_x == 300);
    assert(screen.turn_y == 200);
    assert(screen.turn_bottom == 100);

    const bird_t left = {.x = 299, .y = 300};
    const bird_t right = {.x = 601, .y = 300};
    const bird_t top = {.x = 450, .y = 199};
    const bird_t bottom = {.x = 450, .y = 501};
    const bird_t below_top = {.x = 450, .y = 201};
    const bird_t above_bottom = {.x = 450, .y = 499};
    const bird_t center = {.x = 450, .y = 300};

    vector_t left_force = boundary_vector(&left);
    vector_t right_force = boundary_vector(&right);
    vector_t top_force = boundary_vector(&top);
    vector_t bottom_force = boundary_vector(&bottom);
    vector_t below_top_force = boundary_vector(&below_top);
    vector_t above_bottom_force = boundary_vector(&above_bottom);
    vector_t center_force = boundary_vector(&center);

    /* The horizontal bands mirror each other, the vertical ones are a third of
     * the height at the top and a sixth at the bottom. */
    assert(left_force.x == 1 && left_force.y == 0);
    assert(right_force.x == -1 && right_force.y == 0);
    assert(top_force.x == 0 && top_force.y == 1);
    assert(bottom_force.x == 0 && bottom_force.y == -1);
    /* One pixel past either edge the band must already be over. */
    assert(below_top_force.x == 0 && below_top_force.y == 0);
    assert(above_bottom_force.x == 0 && above_bottom_force.y == 0);
    assert(center_force.x == 0 && center_force.y == 0);
}

/* A bottom band of a fixed 100 pixels used to swallow a short viewport whole
 * and push every bird upwards, flock pinned to the top edge. */
static void test_bottom_band_scales_on_a_short_viewport(void) {
    set_test_screen(640, 96); /* Six rows of sixteen pixels. */
    assert(screen.turn_y == 32);
    assert(screen.turn_bottom == 16);
    assert(screen.turn_y < screen.height - screen.turn_bottom); /* Never overlapping. */

    for (int y = screen.turn_y; y <= screen.height - screen.turn_bottom; y++) {
        const bird_t bird = {.x = 320, .y = y};
        vector_t force = boundary_vector(&bird);
        assert(force.x == 0 && force.y == 0);
    }
    const bird_t low = {.x = 320, .y = 95};
    const bird_t high = {.x = 320, .y = 1};
    assert(boundary_vector(&low).y == -1);
    assert(boundary_vector(&high).y == 1);
}

static void test_birds_start_spread_inside_the_free_region(void) {
    enum { BIRD_COUNT = 512 };
    bird_t birds[BIRD_COUNT];

    set_test_screen(900, 600);
    config.birds = BIRD_COUNT;
    srand(20260911u);
    initialize_birds(birds);

    int distinct = 0;
    for (int i = 0; i < BIRD_COUNT; i++) {
        vector_t force = boundary_vector(&birds[i]);
        /* No bird starts inside a band, so none opens the run fleeing an edge. */
        assert(force.x == 0 && force.y == 0);
        assert(birds[i].frame == direction_frame(birds[i].direction));
        if (birds[i].x != birds[0].x || birds[i].y != birds[0].y) distinct++;
    }
    /* And they are spread over that region instead of stacked on its middle. */
    assert(distinct > BIRD_COUNT * 3 / 4);
}

static void test_legend_fits_every_width(void) {
    char line[LEGEND_LINE_MAX];

    reset_test_config();
    apply_screen_size(120, 30, 120 * 8, 30 * 16);
    build_legend(line, sizeof(line));
    assert(legend_text_length(line) <= (size_t)screen.cols);
    assert(strstr(line, "(Boids)") != NULL); /* The major mode field. */
    assert(strstr(line, "800 boids") != NULL);
    assert(strstr(line, "q quit") != NULL);

    apply_screen_size(80, 30, 80 * 8, 30 * 16);
    build_legend(line, sizeof(line));
    assert(legend_text_length(line) <= (size_t)screen.cols);
    assert(strstr(line, "(Boids)") == NULL); /* Dropped, it does not fit. */
    assert(strstr(line, "b/B 0.20") != NULL);
    assert(strstr(line, "r/R 60") != NULL);

    apply_screen_size(48, 30, 48 * 8, 30 * 16);
    build_legend(line, sizeof(line));
    assert(legend_text_length(line) <= (size_t)screen.cols);
    assert(strstr(line, "b0.20") != NULL);
    assert(strstr(line, "b/B") == NULL);

    /* Truncated rather than wrapped: a wrap would scroll the flock away. */
    apply_screen_size(LEGEND_NARROW_COLS, 30, LEGEND_NARROW_COLS * 8, 30 * 16);
    build_legend(line, sizeof(line));
    assert(legend_text_length(line) <= (size_t)LEGEND_NARROW_COLS);
}

static void test_legend_row_is_reserved(void) {
    reset_test_config();
    apply_screen_size(80, 24, 80 * 8, 24 * 16);
    assert(screen.cell_height == 16);
    assert(screen.legend_row == 23);
    assert(screen.rows == 23); /* The flock gives up the last row. */
    /* And the sprite height that would spill out of the last row it can use. */
    assert(screen.height == 23 * 16 - DEFAULT_BIRD_SIZE);
    assert(screen.turn_y == screen.height / 3); /* The bands follow. */

    /* No reachable position can put a sprite over the bar. */
    for (int y = 0; y <= screen.height; y++) {
        const bird_t bird = {.x = 100, .y = y};
        kitty_graphics_placement_t placement;
        assert(bird_placement(&bird, &placement));
        assert(placement.row < screen.legend_row);
        assert(y + DEFAULT_BIRD_SIZE <= screen.legend_row * screen.cell_height);
    }
}

static void test_legend_absent_on_a_tiny_viewport(void) {
    reset_test_config();
    apply_screen_size(30, 24, 30 * 8, 24 * 16); /* Too narrow. */
    assert(screen.legend_row < 0);
    assert(screen.rows == 24);
    assert(screen.height == 24 * 16);

    apply_screen_size(80, 5, 80 * 8, 5 * 16); /* Too short. */
    assert(screen.legend_row < 0);
    assert(screen.rows == 5);
    assert(screen.height == 5 * 16);
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

static void test_legend_sigil_tracks_the_weights(void) {
    char line[LEGEND_LINE_MAX];

    reset_test_config();
    apply_screen_size(120, 30, 120 * 8, 30 * 16);
    build_legend(line, sizeof(line));
    assert(strncmp(legend_text(line), "-:---", 5) == 0);

    assert(feed_input("A") == 1);
    build_legend(line, sizeof(line));
    assert(strncmp(legend_text(line), "-:**-", 5) == 0);

    reset_test_config();
    build_legend(line, sizeof(line));
    assert(strncmp(legend_text(line), "-:---", 5) == 0);
}

static void test_weights_stop_at_their_ceiling(void) {
    char keys[INPUT_BUFFER_SIZE + 1];

    reset_test_config();
    memset(keys, 'B', sizeof(keys) - 1);
    keys[sizeof(keys) - 1] = '\0';
    for (int i = 0; i < 5; i++) assert(feed_input(keys) == 1);
    assert(config.boundary == BOUNDARY_MAX);

    memset(keys, 'A', sizeof(keys) - 1);
    for (int i = 0; i < 5; i++) assert(feed_input(keys) == 1);
    assert(config.alignment == ALIGNMENT_MAX);

    memset(keys, 'S', sizeof(keys) - 1);
    for (int i = 0; i < 5; i++) assert(feed_input(keys) == 1);
    assert(config.separation == SEPARATION_MAX);

    memset(keys, 'C', sizeof(keys) - 1);
    for (int i = 0; i < 5; i++) assert(feed_input(keys) == 1);
    assert(config.cohesion == COHESION_MAX);

    /* The floors still work, and the pair still meets in the middle. */
    memset(keys, 'b', sizeof(keys) - 1);
    for (int i = 0; i < 5; i++) assert(feed_input(keys) == 1);
    assert(config.boundary == BOUNDARY_MIN);
    reset_test_config();
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
    assert(config.vision_radius == MAX_VISION_CELLS * SPATIAL_CELL_SIZE);

    config.vision_cells = MIN_VISION_CELLS;
    update_vision_radius();
    assert(feed_input("pp") == 1);
    assert(config.vision_cells == MIN_VISION_CELLS);
    assert(config.vision_radius == MIN_VISION_CELLS * SPATIAL_CELL_SIZE);
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
    screen.legend_row = -1; /* The legend has its own test below. */
    drawn_legend_row = -1;
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

static void test_frame_carries_the_legend(void) {
    kitty_graphics_t graphics;
    bird_t bird = {.x = 9, .y = 17, .direction = 0, .frame = 3};

    reset_test_config();
    config.birds = 1;
    drawn_legend_row = -1;
    apply_screen_size(80, 24, 80 * 8, 24 * 16);
    assert(kitty_graphics_init(&graphics, STDOUT_FILENO) == KITTY_GRAPHICS_OK);

    assert(queue_render_frame(&graphics, &bird) == KITTY_GRAPHICS_OK);
    const char *placement = strstr(graphics.buffer, "a=p");
    const char *bar = strstr(graphics.buffer, "\033[7m\033[K");
    const char *sync_end = strstr(graphics.buffer, "\033[?2026l");
    assert(placement != NULL && bar != NULL && sync_end != NULL);
    assert(bar > placement); /* The bar goes on top of the flock. */
    assert(bar < sync_end);  /* Inside the update, so it cannot tear. */
    /* Addressed at the reserved row, one based on the wire. */
    assert(strstr(graphics.buffer, "\033[24;1H\033[7m") != NULL);
    assert(drawn_legend_row == screen.legend_row);

    /* A frame at an unchanged size repaints the bar and erases nothing: the
     * reverse video erase inside the bar already covers the whole row. */
    clear_graphics_buffer(&graphics);
    assert(queue_render_frame(&graphics, &bird) == KITTY_GRAPHICS_OK);
    assert(strstr(graphics.buffer, "\033[7m\033[K") != NULL);
    assert(strstr(graphics.buffer, "\033[24;1H\033[K") == NULL);

    kitty_graphics_destroy(&graphics);
}

/* Clearing the whole screen takes the uploaded sprites with it in Kitty, which
 * leaves every later placement pointing at an image that no longer exists: the
 * flock simply stops being drawn. Only the row the bar has left is erased. */
static void test_frame_never_erases_the_whole_screen(void) {
    kitty_graphics_t graphics;
    bird_t bird = {.x = 9, .y = 17, .direction = 0, .frame = 3};

    reset_test_config();
    config.birds = 1;
    drawn_legend_row = -1;
    assert(kitty_graphics_init(&graphics, STDOUT_FILENO) == KITTY_GRAPHICS_OK);

    apply_screen_size(80, 24, 80 * 8, 24 * 16);
    assert(queue_render_frame(&graphics, &bird) == KITTY_GRAPHICS_OK);
    assert(drawn_legend_row == 23);
    assert(strstr(graphics.buffer, "\033[2J") == NULL);
    assert(strstr(graphics.buffer, "\033[3J") == NULL);

    /* Grown: the old bar is now stranded mid screen, that one row is erased. */
    clear_graphics_buffer(&graphics);
    apply_screen_size(80, 40, 80 * 8, 40 * 16);
    assert(queue_render_frame(&graphics, &bird) == KITTY_GRAPHICS_OK);
    assert(strstr(graphics.buffer, "\033[2J") == NULL);
    assert(strstr(graphics.buffer, "\033[24;1H\033[K") != NULL);  /* The old row. */
    assert(strstr(graphics.buffer, "\033[40;1H\033[7m") != NULL); /* The new one. */
    assert(drawn_legend_row == 39);

    /* Narrowed past the threshold: the bar goes away and its row is erased. */
    clear_graphics_buffer(&graphics);
    apply_screen_size(30, 40, 30 * 8, 40 * 16);
    assert(queue_render_frame(&graphics, &bird) == KITTY_GRAPHICS_OK);
    assert(strstr(graphics.buffer, "\033[2J") == NULL);
    assert(strstr(graphics.buffer, "\033[40;1H\033[K") != NULL);
    assert(strstr(graphics.buffer, "\033[7m") == NULL);
    assert(drawn_legend_row == -1);

    /* And once erased it is not erased again every frame. */
    clear_graphics_buffer(&graphics);
    assert(queue_render_frame(&graphics, &bird) == KITTY_GRAPHICS_OK);
    assert(strstr(graphics.buffer, "\033[K") == NULL);

    kitty_graphics_destroy(&graphics);
}

int main(void) {
    test_engine_matches_brute_force();
    test_boundary_bands_follow_the_viewport();
    test_bottom_band_scales_on_a_short_viewport();
    test_birds_start_spread_inside_the_free_region();
    test_weights_stop_at_their_ceiling();
    test_vision_controls();
    test_frame_rate_controls();
    test_flicker_free_render_queue();
    test_legend_fits_every_width();
    test_legend_sigil_tracks_the_weights();
    test_legend_row_is_reserved();
    test_legend_absent_on_a_tiny_viewport();
    test_frame_carries_the_legend();
    test_frame_never_erases_the_whole_screen();
    return 0;
}
