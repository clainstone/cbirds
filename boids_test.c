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

/* Bands only: the panel has its own tests, and left standing it would answer
 * boundary_vector before the bands ever got the chance. */
static void set_test_screen(int width, int height) {
    screen.width = width;
    screen.height = height;
    screen.legend_width = screen.legend_height = 0;
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

/* The panel is drawn with multi byte glyphs, so its width is a count of cells,
 * not of bytes. Every glyph it uses is one cell wide. */
static size_t legend_cells(const char *line) {
    size_t cells = 0;
    for (const unsigned char *p = (const unsigned char *)line; *p; p++)
        if ((*p & 0xc0) != 0x80) cells++;
    return cells;
}

/* The forbidden rectangle, written out here rather than borrowed from the
 * program: a sprite is drawn from its top left corner and reaches bird_size
 * right and down, so it covers part of the panel exactly when that corner is
 * inside it. No bird may ever be here. */
static int sprite_overlaps_legend(double x, double y) {
    return screen.legend_width > 0 && x < screen.legend_width && y < screen.legend_height;
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

static void test_legend_panel_layout(void) {
    char lines[LEGEND_ROWS][LEGEND_LINE_MAX];

    reset_test_config();
    apply_screen_size(80, 24, 80 * 8, 24 * 16);
    assert(screen.legend_width == LEGEND_COLUMNS * screen.cell_width);
    assert(screen.legend_height == LEGEND_ROWS * screen.cell_height);
    build_legend(lines);

    /* A rectangle, every row exactly as wide as the panel claims to be. */
    for (int row = 0; row < LEGEND_ROWS; row++) assert(legend_cells(lines[row]) == LEGEND_COLUMNS);
    assert(strstr(lines[0], "\u256d") == lines[0]);
    assert(strstr(lines[0], "\u256e") != NULL);
    assert(strstr(lines[LEGEND_ROWS - 1], "\u2570") == lines[LEGEND_ROWS - 1]);
    assert(strstr(lines[LEGEND_ROWS - 1], "\u256f") != NULL);

    /* One slider a parameter, named, with a bar and its pair of keys. */
    static const char *names[] = {"boundary",  "separation", "cohesion",
                                  "alignment", "perception", "rate"};
    static const char *pairs[] = {"b/B", "s/S", "c/C", "a/A", "p/P", "r/R"};
    for (int i = 0; i < 6; i++) {
        assert(strstr(lines[1 + i], names[i]) != NULL);
        /* Lowercase first: the key that lowers, then the one that raises. */
        assert(strstr(lines[1 + i], pairs[i]) != NULL);
        assert(strstr(lines[1 + i], "\u2591") != NULL); /* Some empty track shows. */
    }
    assert(strstr(lines[8], "quit") != NULL);

    /* The inverted pair is the whole point, so the old order must be absent. */
    static const char *reversed[] = {"B/b", "S/s", "C/c", "A/a", "P/p", "R/r"};
    for (int row = 0; row < LEGEND_ROWS; row++)
        for (int i = 0; i < 6; i++) assert(strstr(lines[row], reversed[i]) == NULL);

    /* And no numeric value anywhere in the panel. */
    for (int row = 0; row < LEGEND_ROWS; row++)
        for (const char *c = lines[row]; *c; c++) assert(!(*c >= '0' && *c <= '9'));
}

static void test_legend_bars_span_their_travel(void) {
    char lines[LEGEND_ROWS][LEGEND_LINE_MAX];

    reset_test_config();
    apply_screen_size(80, 24, 80 * 8, 24 * 16);
    assert(bar_cells(BOUNDARY_MIN, BOUNDARY_MIN, BOUNDARY_MAX) == 0);
    assert(bar_cells(BOUNDARY_MAX, BOUNDARY_MIN, BOUNDARY_MAX) == LEGEND_BAR_CELLS);
    assert(bar_cells(ALIGNMENT_MIN, ALIGNMENT_MIN, ALIGNMENT_MAX) == 0);
    assert(bar_cells(ALIGNMENT_MAX, ALIGNMENT_MIN, ALIGNMENT_MAX) == LEGEND_BAR_CELLS);
    /* Out of range cannot overflow the bar either way. */
    assert(bar_cells(-1, BOUNDARY_MIN, BOUNDARY_MAX) == 0);
    assert(bar_cells(99, BOUNDARY_MIN, BOUNDARY_MAX) == LEGEND_BAR_CELLS);

    /* Monotone, and every default sits partway along its travel. */
    int previous = -1;
    for (double v = BOUNDARY_MIN; v <= BOUNDARY_MAX; v += BOUNDARY_STEP / 4) {
        int cells = bar_cells(v, BOUNDARY_MIN, BOUNDARY_MAX);
        assert(cells >= previous);
        previous = cells;
    }
    assert(bar_cells(DEFAULT_BOUNDARY_W, BOUNDARY_MIN, BOUNDARY_MAX) > 0);
    assert(bar_cells(DEFAULT_BOUNDARY_W, BOUNDARY_MIN, BOUNDARY_MAX) < LEGEND_BAR_CELLS);

    /* A single keypress has to move the bar, which is what the ceilings buy:
     * the coarsest parameter must not need more than six presses per cell. */
    static const struct {
        double step, min, max;
    } travel[] = {{BOUNDARY_STEP, BOUNDARY_MIN, BOUNDARY_MAX},
                  {SEPARATION_STEP, SEPARATION_MIN, SEPARATION_MAX},
                  {COHESION_STEP, COHESION_MIN, COHESION_MAX},
                  {ALIGNMENT_STEP, ALIGNMENT_MIN, ALIGNMENT_MAX}};
    for (size_t i = 0; i < sizeof(travel) / sizeof(*travel); i++) {
        double presses_per_cell =
            (travel[i].max - travel[i].min) / travel[i].step / LEGEND_BAR_CELLS;
        assert(presses_per_cell <= 6.0);
    }

    /* The rendered bar follows the value. */
    config.boundary = BOUNDARY_MIN;
    build_legend(lines);
    assert(strstr(lines[1], "\u2593") == NULL); /* Nothing filled on the floor. */
    config.boundary = BOUNDARY_MAX;
    build_legend(lines);
    assert(strstr(lines[1], "\u2591") == NULL); /* Nothing empty on the ceiling. */
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

static void test_legend_repels_towards_the_nearer_way_out(void) {
    reset_test_config();
    apply_screen_size(200, 50, 200 * 8, 50 * 16);
    double w = screen.legend_width, h = screen.legend_height, m = config.speed;

    /* Close to the right side of the zone, the way out is rightwards. */
    const bird_t right = {.x = w + m - 1, .y = h / 2};
    assert(boundary_vector(&right).x == LEGEND_PUSH);
    assert(boundary_vector(&right).y == 0);

    /* Close to the bottom of it, downwards. */
    const bird_t below = {.x = w / 2, .y = h + m - 1};
    assert(boundary_vector(&below).y == LEGEND_PUSH);
    assert(boundary_vector(&below).x == 0);

    /* Deep in the corner, along the shorter escape, which for a panel wider
     * than it is tall is downwards. */
    const bird_t corner = {.x = 1, .y = 1};
    assert(h < w);
    assert(boundary_vector(&corner).y == LEGEND_PUSH);
    assert(boundary_vector(&corner).x == 0);

    /* One step outside the zone the panel stands aside and the screen bands
     * answer exactly as they did before it existed: that point is still inside
     * the left and top bands, so it gets their unit push, not the panel's. */
    const bird_t clear = {.x = w + m + 1, .y = h + m + 1};
    vector_t force = boundary_vector(&clear);
    assert(force.x == 1 && force.y == 1);
    assert(clear.x < screen.turn_x && clear.y < screen.turn_y);

    /* And in open water there is no force at all. */
    const bird_t middle = {.x = screen.width / 2.0, .y = screen.height / 2.0};
    force = boundary_vector(&middle);
    assert(force.x == 0 && force.y == 0);

    /* With no panel there is no push at all. */
    legend_enabled = 0;
    apply_screen_size(200, 50, 200 * 8, 50 * 16);
    assert(screen.legend_width == 0);
    const bird_t origin = {.x = 1, .y = 1};
    force = boundary_vector(&origin);
    assert(force.x == 1 && force.y == 1); /* Only the screen bands, as before. */
    legend_enabled = 1;
}

static void test_legend_push_overrules_the_flock(void) {
    enum { BIRD_COUNT = 50 };
    bird_t birds[BIRD_COUNT];
    spatial_grid_t grid;

    reset_test_config();
    apply_screen_size(200, 50, 200 * 8, 50 * 16);
    config.birds = BIRD_COUNT;

    /* One bird inside the turn zone, every neighbour packed to its upper left
     * and heading that way, so separation, alignment and cohesion all pull it
     * deeper into the panel. */
    birds[0] = (bird_t){screen.legend_width + 2.0, screen.legend_height / 2.0, 0, 0};
    for (int i = 1; i < BIRD_COUNT; i++)
        birds[i] = (bird_t){birds[0].x - 4, birds[0].y - 4, M_PI, 0};

    assert(spatial_grid_init(&grid, SPATIAL_CELL_SIZE) == SPATIAL_GRID_OK);
    assert(spatial_grid_prepare(&grid, screen.width, screen.height, BIRD_COUNT) == SPATIAL_GRID_OK);
    assert(spatial_grid_build(&grid, BIRD_COUNT, read_bird_position, birds) == SPATIAL_GRID_OK);

    double with_panel = flock_direction(birds, &grid, 0);
    assert(cos(with_panel) > 0.999); /* Straight out to the right. */

    /* The same flock without the panel turns it the other way, which is what
     * makes this a statement about the push and not about the neighbours. */
    screen.legend_width = screen.legend_height = 0;
    double without_panel = flock_direction(birds, &grid, 0);
    assert(cos(without_panel) < 0);
    spatial_grid_destroy(&grid);
}

/* The test the whole design rests on. */
static void test_no_bird_ever_reaches_the_panel(void) {
    enum { FRAMES = 400, DIRECTIONS = 16 };
    spatial_grid_t grid;

    assert(spatial_grid_init(&grid, SPATIAL_CELL_SIZE) == SPATIAL_GRID_OK);
    for (int rate = MIN_FRAME_RATE; rate <= MAX_FRAME_RATE;
         rate += MAX_FRAME_RATE - MIN_FRAME_RATE) {
        reset_test_config();
        config.frame_rate = rate;
        update_speed(); /* Which moves the turn margin with it. */
        apply_screen_size(200, 50, 200 * 8, 50 * 16);
        config.birds = 1;
        assert(spatial_grid_prepare(&grid, screen.width, screen.height, 1) == SPATIAL_GRID_OK);

        double margin = config.speed;
        /* Start just outside the turn zone, all the way round its two open
         * sides, aimed in every direction. */
        for (double x = 0; x <= screen.legend_width + margin + 40; x += 17)
            for (double y = 0; y <= screen.legend_height + margin + 40; y += 13)
                for (int d = 0; d < DIRECTIONS; d++) {
                    bird_t bird = {x, y, 2 * M_PI * d / DIRECTIONS, 0};
                    if (legend_turn_zone(bird.x, bird.y)) continue; /* Not a legal start. */
                    bird_t snapshot = bird;
                    for (int frame = 0; frame < FRAMES; frame++) {
                        snapshot = bird;
                        assert(spatial_grid_build(&grid, 1, read_bird_position, &snapshot) ==
                               SPATIAL_GRID_OK);
                        update_birds(&bird, &snapshot, &grid);
                        assert(!sprite_overlaps_legend(bird.x, bird.y));
                    }
                }
    }
    spatial_grid_destroy(&grid);
    reset_test_config();
}

static void test_birds_start_clear_of_the_panel(void) {
    enum { BIRD_COUNT = 512 };
    bird_t birds[BIRD_COUNT];

    srand(20260912u);
    /* A roomy viewport, and one where the panel covers most of the free region. */
    static const int sizes[][2] = {{200, 50}, {80, 24}, {44, 15}};
    for (size_t i = 0; i < sizeof(sizes) / sizeof(*sizes); i++) {
        reset_test_config();
        apply_screen_size(sizes[i][0], sizes[i][1], sizes[i][0] * 8, sizes[i][1] * 16);
        config.birds = BIRD_COUNT;
        initialize_birds(birds);
        for (int b = 0; b < BIRD_COUNT; b++) {
            assert(!legend_turn_zone(birds[b].x, birds[b].y));
            assert(!sprite_overlaps_legend(birds[b].x, birds[b].y));
        }
    }
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
    screen.legend_width = screen.legend_height = 0; /* The panel has its own tests. */
    legend_drawn = 0;
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

static void test_frame_carries_the_panel(void) {
    kitty_graphics_t graphics;
    bird_t bird = {.x = 400, .y = 300, .direction = 0, .frame = 3};

    reset_test_config();
    config.birds = 1;
    legend_drawn = 0;
    apply_screen_size(80, 24, 80 * 8, 24 * 16);
    assert(kitty_graphics_init(&graphics, STDOUT_FILENO) == KITTY_GRAPHICS_OK);

    assert(queue_render_frame(&graphics, &bird) == KITTY_GRAPHICS_OK);
    const char *placement = strstr(graphics.buffer, "a=p");
    const char *panel = strstr(graphics.buffer, "\033[1;1H\u256d");
    const char *sync_end = strstr(graphics.buffer, "\033[?2026l");
    assert(placement != NULL && panel != NULL && sync_end != NULL);
    assert(panel > placement); /* Drawn over the flock. */
    assert(panel < sync_end);  /* Inside the update, so it cannot tear. */
    assert(legend_drawn);

    /* Ten rows, addressed one based on the wire, from the top left corner. */
    for (int row = 1; row <= LEGEND_ROWS; row++) {
        char cup[24];
        snprintf(cup, sizeof(cup), "\033[%d;1H", row);
        assert(strstr(graphics.buffer, cup) != NULL);
    }

    /* It is anchored and constant, so a second frame repaints and erases nothing. */
    clear_graphics_buffer(&graphics);
    assert(queue_render_frame(&graphics, &bird) == KITTY_GRAPHICS_OK);
    assert(strstr(graphics.buffer, "\u256d") != NULL);
    assert(strstr(graphics.buffer, "\033[K") == NULL);

    kitty_graphics_destroy(&graphics);
}

static void test_panel_switches_off_cleanly(void) {
    kitty_graphics_t graphics;
    bird_t bird = {.x = 400, .y = 300, .direction = 0, .frame = 3};

    reset_test_config();
    config.birds = 1;
    legend_drawn = 0;
    apply_screen_size(80, 24, 80 * 8, 24 * 16);
    assert(kitty_graphics_init(&graphics, STDOUT_FILENO) == KITTY_GRAPHICS_OK);
    assert(queue_render_frame(&graphics, &bird) == KITTY_GRAPHICS_OK);
    assert(legend_drawn);

    /* A viewport that shrinks under the panel takes it away, and the rows it
     * held are erased one line at a time. Never a screen erase: that would
     * delete the uploaded sprites and stop the flock being drawn at all. */
    clear_graphics_buffer(&graphics);
    apply_screen_size(30, 24, 30 * 8, 24 * 16);
    assert(screen.legend_width == 0);
    assert(queue_render_frame(&graphics, &bird) == KITTY_GRAPHICS_OK);
    assert(!legend_drawn);
    assert(strstr(graphics.buffer, "\033[2J") == NULL);
    assert(strstr(graphics.buffer, "\033[3J") == NULL);
    for (int row = 1; row <= LEGEND_ROWS; row++) {
        char erase[32];
        snprintf(erase, sizeof(erase), "\033[%d;1H\033[K", row);
        assert(strstr(graphics.buffer, erase) != NULL);
    }

    /* Erased once, not every frame after. */
    clear_graphics_buffer(&graphics);
    assert(queue_render_frame(&graphics, &bird) == KITTY_GRAPHICS_OK);
    assert(strstr(graphics.buffer, "\033[K") == NULL);

    /* Back above the threshold it draws itself again. */
    clear_graphics_buffer(&graphics);
    apply_screen_size(80, 24, 80 * 8, 24 * 16);
    assert(queue_render_frame(&graphics, &bird) == KITTY_GRAPHICS_OK);
    assert(legend_drawn);
    assert(strstr(graphics.buffer, "\u256d") != NULL);

    kitty_graphics_destroy(&graphics);
}

static void test_no_legend_leaves_the_corner_to_the_flock(void) {
    kitty_graphics_t graphics;
    bird_t bird = {.x = 400, .y = 300, .direction = 0, .frame = 3};

    reset_test_config();
    config.birds = 1;
    legend_drawn = 0;
    legend_enabled = 0;
    apply_screen_size(80, 24, 80 * 8, 24 * 16);
    assert(screen.legend_width == 0);
    assert(screen.rows == 24); /* Nothing reserved anywhere. */
    assert(screen.height == 24 * 16);

    const bird_t corner = {.x = 1, .y = 1};
    vector_t force = boundary_vector(&corner);
    assert(force.x == 1 && force.y == 1); /* Screen bands only. */

    assert(kitty_graphics_init(&graphics, STDOUT_FILENO) == KITTY_GRAPHICS_OK);
    assert(queue_render_frame(&graphics, &bird) == KITTY_GRAPHICS_OK);
    assert(strstr(graphics.buffer, "\u256d") == NULL);
    assert(strstr(graphics.buffer, "\033[K") == NULL);
    assert(strstr(graphics.buffer, "\033[2J") == NULL);
    assert(strstr(graphics.buffer, "a=p") != NULL);

    kitty_graphics_destroy(&graphics);
    legend_enabled = 1;
}

int main(void) {
    test_engine_matches_brute_force();
    test_boundary_bands_follow_the_viewport();
    test_bottom_band_scales_on_a_short_viewport();
    test_birds_start_spread_inside_the_free_region();
    test_vision_controls();
    test_frame_rate_controls();
    test_flicker_free_render_queue();
    test_legend_panel_layout();
    test_legend_bars_span_their_travel();
    test_weights_stop_at_their_ceiling();
    test_legend_repels_towards_the_nearer_way_out();
    test_legend_push_overrules_the_flock();
    test_no_bird_ever_reaches_the_panel();
    test_birds_start_clear_of_the_panel();
    test_frame_carries_the_panel();
    test_panel_switches_off_cleanly();
    test_no_legend_leaves_the_corner_to_the_flock();
    return 0;
}
