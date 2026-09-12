#define main cbirds_application_main
#include "../boids.c"
#undef main

#include <assert.h>

/* The model, written out a second time the obvious way. It knows the three rules,
 * the edges and the leash, and it deliberately does not know the pointer, the
 * hawks or the wind: the test that uses it asserts that those are all switched
 * off, so that this stays a check of the search and not a second implementation
 * to keep in step. */
static double brute_force_flock_direction(const bird_t *birds, int target_index) {
    const bird_t *target = &birds[target_index];
    vector_t separation = {0, 0}, alignment = {0, 0}, cohesion = {0, 0};
    vector_t boundary = boundary_vector(target);
    vector_t leash = leash_vector(target);
    int neighbors = 0, kin = 0;

    for (int i = 0; i < config.birds; i++) {
        if (i == target_index) continue;
        const bird_t *other = &birds[i];
        double dx = target->x - other->x;
        double dy = target->y - other->y;
        if (dx * dx + dy * dy >= config.vision_radius_squared) continue;
        separation.x += dx;
        separation.y += dy;
        neighbors++;
        if (other->flock != target->flock) continue;
        alignment.x += cos(other->direction);
        alignment.y += sin(other->direction);
        cohesion.x += other->x;
        cohesion.y += other->y;
        kin++;
    }

    if (neighbors) {
        if (kin) {
            alignment.x /= kin;
            alignment.y /= kin;
            cohesion.x = cohesion.x / kin - target->x;
            cohesion.y = cohesion.y / kin - target->y;
        }
        double x = separation.x * config.separation + alignment.x * config.alignment +
                   cohesion.x * COHESION_W + boundary.x * config.boundary + leash.x * LEASH_WEIGHT;
        double y = separation.y * config.separation + alignment.y * config.alignment +
                   cohesion.y * COHESION_W + boundary.y * config.boundary + leash.y * LEASH_WEIGHT;
        return x == 0 && y == 0 ? target->direction : normalized_angle(y, x);
    }

    boundary.x = boundary.x * config.boundary + leash.x * LEASH_WEIGHT;
    boundary.y = boundary.y * config.boundary + leash.y * LEASH_WEIGHT;
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
    config.boundary_notch = DEFAULT_NOTCH;
    config.separation_notch = DEFAULT_NOTCH;
    config.alignment_notch = DEFAULT_NOTCH;
    config.vision_notch = 6;
    config.rate_notch = DEFAULT_NOTCH;
    config.palette = 0;
    config.mouse_mode = MOUSE_FLEE;
    config.mouse_reach = DEFAULT_MOUSE_REACH;
    config.turning_notch = DEFAULT_TURNING_NOTCH;
    config.flocks = 1;
    config.trails = 0;
    config.hawks = 0;
    matrix_mode = 0;
    apply_notches();
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
    birds[0] = (bird_t){.x = 12, .y = 12};
    birds[1] = (bird_t){.x = 24, .y = 12, .direction = M_PI / 2};
    birds[2] = (bird_t){.x = 36, .y = 36, .direction = M_PI};
    birds[3] = (bird_t){.x = -1, .y = 20, .direction = M_PI / 4};
    birds[4] = (bird_t){.x = 641, .y = 20, .direction = 3 * M_PI / 2};
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
    config.boundary = 0.2;
    initialize_test_birds(snapshot, BIRD_COUNT);

    assert(spatial_grid_init(&grid, SPATIAL_CELL_SIZE) == SPATIAL_GRID_OK);
    assert(spatial_grid_prepare(&grid, screen.width, screen.height, BIRD_COUNT) == SPATIAL_GRID_OK);
    assert(spatial_grid_build(&grid, BIRD_COUNT, read_bird_position, snapshot) == SPATIAL_GRID_OK);

    /* The three terms the reference does not model must all be quiet, or it is
     * not modelling the same thing. */
    assert(config.hawks == 0);
    assert(!mouse.present || config.mouse_mode == MOUSE_OFF);
    assert(!the_rain_is_falling);

    /* Swept over the flock count too: the reference models the same social rule,
     * so a divergence would mean one of the two forgot it. */
    for (config.flocks = 1; config.flocks <= MAX_FLOCKS; config.flocks++) {
        for (int i = 0; i < BIRD_COUNT; i++) snapshot[i].flock = i % config.flocks;
        /* What update_birds does before any bird reads its flock's whereabouts. */
        measure_flocks(snapshot);
        for (config.vision_notch = 0; config.vision_notch <= LEGEND_BAR_CELLS;
             config.vision_notch++) {
            apply_notches();
            for (int i = 0; i < BIRD_COUNT; i++) {
                double expected = brute_force_flock_direction(snapshot, i);
                double actual = flock_direction(snapshot, &grid, i);
                assert(angle_difference(expected, actual) < 1e-11);
            }
        }
    }
    config.flocks = 1;
    for (int i = 0; i < BIRD_COUNT; i++) snapshot[i].flock = 0;

    config.vision_notch = 6;
    apply_notches();
    memcpy(optimized, snapshot, sizeof(snapshot));
    memcpy(reference, snapshot, sizeof(snapshot));
    update_birds(optimized, snapshot, &grid);
    for (int i = 0; i < BIRD_COUNT; i++) {
        /* The reference banks the same way the engine does, or this would be
         * comparing two different models rather than two ways of searching. */
        double direction = turn_towards(snapshot[i].direction,
                                        brute_force_flock_direction(snapshot, i), turn_limit());
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
     * the height at the top and a sixth at the bottom. The push grows with depth
     * rather than being a unit vector, so what mirrors is the sign and the size. */
    assert(left_force.x > 0 && left_force.y == 0);
    assert(right_force.x == -left_force.x && right_force.y == 0);
    assert(top_force.x == 0 && top_force.y > 0);
    assert(bottom_force.x == 0 && bottom_force.y < 0);
    /* And one pixel inside a band half as wide is twice as deep into it, so the
     * bottom pushes harder there than the top does. */
    assert(-bottom_force.y > top_force.y);
    /* One pixel past either edge the band must already be over. */
    assert(below_top_force.x == 0 && below_top_force.y == 0);
    assert(above_bottom_force.x == 0 && above_bottom_force.y == 0);
    assert(center_force.x == 0 && center_force.y == 0);
}

/* A bottom band of a fixed 100 pixels used to swallow a short viewport whole
 * and push every bird upwards, flock pinned to the top edge. */
/* The band used to push with a unit vector wherever in it a bird was, so the
 * flocking terms outvoted it everywhere except in aggregate and most of the flock
 * spent its time off the screen entirely. The push has to grow with depth, and
 * past the screen's own edge it has to stop caring what the boundary weight is. */
static void test_the_edge_pushes_harder_the_further_out_a_bird_is(void) {
    set_test_screen(900, 600);

    /* Every band, not just the left one: the bottom band is half the width of the
     * top and is the one birds actually leak through. */
    double previous = 0;
    for (int x = 299; x >= -200; x -= 10) {
        const bird_t bird = {.x = x, .y = 300};
        double push = boundary_vector(&bird).x;
        assert(push > previous);
        previous = push;
    }
    previous = 0;
    for (int x = 601; x <= 1100; x += 10) {
        const bird_t bird = {.x = x, .y = 300};
        double push = -boundary_vector(&bird).x;
        assert(push > previous);
        previous = push;
    }
    previous = 0;
    for (int y = 199; y >= -200; y -= 10) {
        const bird_t bird = {.x = 450, .y = y};
        double push = boundary_vector(&bird).y;
        assert(push > previous);
        previous = push;
    }
    previous = 0;
    for (int y = 501; y <= 900; y += 10) {
        const bird_t bird = {.x = 450, .y = y};
        double push = -boundary_vector(&bird).y;
        assert(push > previous);
        previous = push;
    }

    /* The push a bird actually feels is the band's times the boundary weight, and
     * it has no step in it at the screen's own edge: a bird crossing the line is
     * turned, not slapped. That is the one place the two halves of the formula
     * meet, and they used to differ by a factor of a hundred at notch zero. */
    for (int notch = 0; notch <= LEGEND_BAR_CELLS; notch++) {
        config.boundary_notch = notch;
        apply_notches();
        const bird_t inside = {.x = 0.001, .y = 300};
        const bird_t outside = {.x = -0.001, .y = 300};
        double in = boundary_vector(&inside).x * config.boundary;
        double out = boundary_vector(&outside).x * config.boundary;
        assert(fabs(out - in) < 1e-3);
    }

    /* And the softest boundary anyone can ask for brings a bird that has left the
     * screen back about as firmly as the firmest does: at notch zero it turns
     * late, not never. A hundred pixels out the two are within a fifth of each
     * other, while at the edge itself — where the weight is supposed to decide —
     * they differ by a factor of sixty. */
    const bird_t gone = {.x = -100, .y = 300};
    const bird_t leaving = {.x = 0, .y = 300};
    config.boundary_notch = 0;
    apply_notches();
    double softest = boundary_vector(&gone).x * config.boundary;
    double soft_edge = boundary_vector(&leaving).x * config.boundary;
    config.boundary_notch = LEGEND_BAR_CELLS;
    apply_notches();
    double firmest = boundary_vector(&gone).x * config.boundary;
    double firm_edge = boundary_vector(&leaving).x * config.boundary;
    assert(softest > firmest * 0.8);
    assert(softest > EDGE_FIRM);
    assert(soft_edge < firm_edge / 10);
    config.boundary_notch = DEFAULT_NOTCH;
    apply_notches();
}

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
    assert(boundary_vector(&low).y < 0);
    assert(boundary_vector(&high).y > 0);
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

/* Counts the filled cells of one slider row as it is actually drawn, which is
 * what the eye sees and therefore what the requirement is about. */
static int filled_cells(const char *line) {
    int filled = 0;
    for (const char *c = line; (c = strstr(c, "\u2593")) != NULL; c += 3) filled++;
    return filled;
}

static void test_legend_panel_layout(void) {
    char lines[LEGEND_ROWS][LEGEND_LINE_MAX];

    reset_test_config();
    /* The panel waits for a window it is a fifth of rather than half of: at the
     * smallest terminal it used to appear in, it covered 54% of the area and the
     * force that keeps birds out of it squeezed half the flock off the edges of
     * what was left. One column narrower and the flock was fine; one column wider
     * and it was not. */
    assert(LEGEND_COLUMNS * LEGEND_ROWS * 4 <= LEGEND_MIN_COLS * LEGEND_MIN_ROWS);
    apply_screen_size(LEGEND_MIN_COLS - 1, LEGEND_MIN_ROWS, (LEGEND_MIN_COLS - 1) * 8,
                      LEGEND_MIN_ROWS * 16);
    assert(screen.legend_width == 0);
    apply_screen_size(LEGEND_MIN_COLS, LEGEND_MIN_ROWS - 1, LEGEND_MIN_COLS * 8,
                      (LEGEND_MIN_ROWS - 1) * 16);
    assert(screen.legend_width == 0);
    apply_screen_size(LEGEND_MIN_COLS, LEGEND_MIN_ROWS, LEGEND_MIN_COLS * 8, LEGEND_MIN_ROWS * 16);
    assert(screen.legend_width > 0);

    apply_screen_size(80, 24, 80 * 8, 24 * 16);
    assert(screen.legend_width == LEGEND_COLUMNS * screen.cell_width);
    assert(screen.legend_height == LEGEND_ROWS * screen.cell_height);
    build_legend(lines);

    /* A rectangle, every row exactly as wide as the panel claims to be. */
    for (int row = 0; row < LEGEND_ROWS; row++) assert(legend_cells(lines[row]) == LEGEND_COLUMNS);

    /* Including the row that says what the frame costs. It used to come out four
     * cells short, putting a notch in the panel's right hand side, and it was
     * behind a flag that this test never set. */
    stats.frame_ms = 2.125;
    stats.bytes = 31000;
    stats.rate = 60;
    build_legend(lines);
    for (int row = 0; row < LEGEND_ROWS; row++) assert(legend_cells(lines[row]) == LEGEND_COLUMNS);
    assert(strstr(lines[LEGEND_ROWS - 3], "frame") != NULL);
    assert(strstr(lines[LEGEND_ROWS - 3], "ms") != NULL); /* With their units on. */
    assert(strstr(lines[LEGEND_ROWS - 3], "KB") != NULL);
    assert(strstr(lines[LEGEND_ROWS - 3], "fps") != NULL);
    assert(strstr(lines[0], "\u256d") == lines[0]);
    assert(strstr(lines[0], "\u256e") != NULL);
    assert(strstr(lines[LEGEND_ROWS - 1], "\u2570") == lines[LEGEND_ROWS - 1]);
    assert(strstr(lines[LEGEND_ROWS - 1], "\u256f") != NULL);

    /* Every notch the keys can reach comes back with 0, including the two the
     * panel does not show: somebody who has turned the banking down with t cannot
     * see what they did, so the one key that says "back to the defaults" has to
     * actually mean it. */
    config.turning_notch = 0;
    config.boundary_notch = 0;
    apply_preset_defaults();
    assert(config.turning_notch == DEFAULT_TURNING_NOTCH);
    assert(config.boundary_notch == DEFAULT_NOTCH);

    /* One slider a parameter, named, with a bar and its pair of keys. */
    static const char *names[] = {"boundary", "separation", "alignment",
                                  "turning",  "perception", "rate"};
    static const char *pairs[] = {"b/B", "s/S", "a/A", "t/T", "p/P", "r/R"};
    for (int i = 0; i < 6; i++) {
        assert(strstr(lines[1 + i], names[i]) != NULL);
        /* Lowercase first: the key that lowers, then the one that raises. */
        assert(strstr(lines[1 + i], pairs[i]) != NULL);
        assert(strstr(lines[1 + i], "\u2591") != NULL); /* Some empty track shows. */
    }
    assert(strstr(lines[LEGEND_ROWS - 2], "quit") != NULL);

    /* The inverted pair is the whole point, so the old order must be absent. */
    static const char *reversed[] = {"B/b", "S/s", "A/a", "T/t", "P/p", "R/r"};
    for (int row = 0; row < LEGEND_ROWS; row++)
        for (int i = 0; i < 6; i++) assert(strstr(lines[row], reversed[i]) == NULL);

    /* Each slider states its value beside its bar, right aligned in a column of
     * its own so the numbers stack. */
    static const char *values[] = {"0.20", "0.005", "1.50", "70\u00b0", "36px", "60"};
    for (int i = 0; i < 6; i++) assert(strstr(lines[1 + i], values[i]) != NULL);
}

static void test_legend_values_follow_their_notch(void) {
    char lines[LEGEND_ROWS][LEGEND_LINE_MAX];
    static const char *floors[] = {"0.01", "0.001", "0.10", "30\u00b0", "12px", "30"};
    static const char *ceilings[] = {"0.58", "0.013", "4.30", "360\u00b0", "60px", "120"};

    reset_test_config();
    apply_screen_size(80, 24, 80 * 8, 24 * 16);

    config.boundary_notch = config.separation_notch = config.alignment_notch =
        config.turning_notch = config.vision_notch = config.rate_notch = 0;
    apply_notches();
    build_legend(lines);
    for (int i = 0; i < 6; i++) {
        assert(strstr(lines[1 + i], floors[i]) != NULL);
        assert(filled_cells(lines[1 + i]) == 0);
    }

    config.boundary_notch = config.separation_notch = config.alignment_notch =
        config.turning_notch = config.vision_notch = config.rate_notch = LEGEND_BAR_CELLS;
    apply_notches();
    build_legend(lines);
    for (int i = 0; i < 6; i++) {
        assert(strstr(lines[1 + i], ceilings[i]) != NULL);
        assert(filled_cells(lines[1 + i]) == LEGEND_BAR_CELLS);
    }

    /* A press moves the number and the bar together, by construction: both come
     * off the same notch. */
    reset_test_config();
    assert(feed_input("B") == 1);
    build_legend(lines);
    assert(filled_cells(lines[1]) == 5);
    assert(strstr(lines[1], "0.25") != NULL);
    reset_test_config();
}

/* The requirement: one keypress moves the bar by exactly one cell, for every
 * parameter, everywhere along its travel. */
static void test_one_keypress_is_one_cell(void) {
    char lines[LEGEND_ROWS][LEGEND_LINE_MAX];
    static const struct {
        int row;
        char raise, lower;
    } sliders[] = {{1, 'B', 'b'}, {2, 'S', 's'}, {3, 'A', 'a'},
                   {4, 'T', 't'}, {5, 'P', 'p'}, {6, 'R', 'r'}};

    apply_screen_size(80, 24, 80 * 8, 24 * 16);
    for (size_t i = 0; i < sizeof(sliders) / sizeof(*sliders); i++) {
        char key[2] = {sliders[i].lower, '\0'};
        reset_test_config();

        /* Down to the floor, then one cell at a time all the way up. */
        for (int n = 0; n <= LEGEND_BAR_CELLS; n++) assert(feed_input(key) == 1);
        build_legend(lines);
        assert(filled_cells(lines[sliders[i].row]) == 0);

        key[0] = sliders[i].raise;
        for (int expected = 1; expected <= LEGEND_BAR_CELLS; expected++) {
            assert(feed_input(key) == 1);
            build_legend(lines);
            assert(filled_cells(lines[sliders[i].row]) == expected);
        }
        /* And the ceiling holds. */
        assert(feed_input(key) == 1);
        build_legend(lines);
        assert(filled_cells(lines[sliders[i].row]) == LEGEND_BAR_CELLS);

        /* Back down the same way, one cell a press. */
        key[0] = sliders[i].lower;
        for (int expected = LEGEND_BAR_CELLS - 1; expected >= 0; expected--) {
            assert(feed_input(key) == 1);
            build_legend(lines);
            assert(filled_cells(lines[sliders[i].row]) == expected);
        }
        assert(feed_input(key) == 1);
        build_legend(lines);
        assert(filled_cells(lines[sliders[i].row]) == 0);
    }
    reset_test_config();
}

/* The bar is 1.5 times the eight cells it started at, and every notch of it is
 * reachable: the value a notch stands for is derived from the notch, so the two
 * cannot drift apart. */
static void test_bar_spans_the_whole_travel(void) {
    reset_test_config();
    assert(LEGEND_BAR_CELLS == 12);

    for (int n = 0; n <= LEGEND_BAR_CELLS; n++) {
        config.boundary_notch = config.separation_notch = config.alignment_notch =
            config.vision_notch = config.rate_notch = n;
        apply_notches();
        if (n == 0) {
            assert(config.boundary == BOUNDARY_MIN);
            assert(config.separation == SEPARATION_MIN);
            assert(config.alignment == ALIGNMENT_MIN);
            assert(config.vision_radius == MIN_VISION_RADIUS);
            assert(config.frame_rate == MIN_FRAME_RATE);
        }
        if (n == LEGEND_BAR_CELLS) {
            assert(fabs(config.boundary - BOUNDARY_MAX) < 1e-12);
            assert(fabs(config.alignment - ALIGNMENT_MAX) < 1e-12);
            assert(config.vision_radius == MAX_VISION_RADIUS);
            assert(config.frame_rate == MAX_FRAME_RATE);
        }
        /* The scan has to reach as far as the radius does. */
        assert(config.vision_cells * SPATIAL_CELL_SIZE >= config.vision_radius);
        assert((config.vision_cells - 1) * SPATIAL_CELL_SIZE < config.vision_radius);
        assert(config.vision_cells <= MAX_VISION_CELLS);
    }

    /* Each default sits on the fourth notch, so no press is ever a fraction of a
     * cell. To within what a double can carry: the ceilings are placed to make
     * the fourth notch the default, and the arithmetic that gets there is not
     * bit exact for decimals a binary float cannot hold. */
    reset_test_config();
    assert(fabs(config.boundary - DEFAULT_BOUNDARY_W) < 1e-12);
    assert(fabs(config.separation - DEFAULT_SEPARATION_W) < 1e-12);
    assert(fabs(config.alignment - DEFAULT_ALIGNMENT_W) < 1e-12);
    assert(config.boundary_notch == DEFAULT_NOTCH);
    assert(config.alignment_notch == DEFAULT_NOTCH);
    /* The two integer parameters land exactly, being integers. */
    assert(config.vision_radius == DEFAULT_VISION_RADIUS);
    assert(config.frame_rate == DEFAULT_FRAME_RATE);
}

static void test_weights_stop_at_their_bounds(void) {
    char keys[INPUT_BUFFER_SIZE + 1];
    static const struct {
        char raise, lower;
        const double *ceiling, *floor;
        const double *value;
    } weights[] = {{'B', 'b', &BOUNDARY_MAX, &BOUNDARY_MIN, &config.boundary},
                   {'S', 's', &SEPARATION_MAX, &SEPARATION_MIN, &config.separation},
                   {'A', 'a', &ALIGNMENT_MAX, &ALIGNMENT_MIN, &config.alignment}};

    keys[INPUT_BUFFER_SIZE] = '\0';
    for (size_t i = 0; i < sizeof(weights) / sizeof(*weights); i++) {
        reset_test_config();
        memset(keys, weights[i].raise, INPUT_BUFFER_SIZE);
        assert(feed_input(keys) == 1);
        assert(fabs(*weights[i].value - *weights[i].ceiling) < 1e-12);

        memset(keys, weights[i].lower, INPUT_BUFFER_SIZE);
        assert(feed_input(keys) == 1);
        assert(fabs(*weights[i].value - *weights[i].floor) < 1e-12);
    }
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
     * the left and top bands, so it gets their push, not the panel's, and theirs
     * grows with depth and is nowhere near LEGEND_PUSH. */
    const bird_t clear = {.x = w + m + 1, .y = h + m + 1};
    vector_t force = boundary_vector(&clear);
    assert(force.x > 0 && force.x < EDGE_FIRM);
    assert(force.y > 0 && force.y < EDGE_FIRM);
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
    /* Only the screen bands, and one pixel in from the corner they are pushing
     * about as hard as they ever do, nowhere near LEGEND_PUSH. */
    assert(force.x > EDGE_FIRM * 0.9 && force.x <= EDGE_FIRM);
    assert(force.y > EDGE_FIRM * 0.9 && force.y <= EDGE_FIRM);
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
    birds[0] = (bird_t){.x = screen.legend_width + 2.0, .y = screen.legend_height / 2.0};
    for (int i = 1; i < BIRD_COUNT; i++)
        birds[i] = (bird_t){.x = birds[0].x - 4, .y = birds[0].y - 4, .direction = M_PI};

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
    /* Swept over the frame rate, which moves the margin, and over the turning
     * limit, which is what nearly broke this: a bird that cannot turn at once
     * cannot be turned away at once, so the panel's push is exempt from the limit
     * and this test is what says so. Dropping that exemption fails it. */
    static const int TURNS[] = {0, 1, DEFAULT_TURNING_NOTCH, LEGEND_BAR_CELLS};
    for (size_t turn = 0; turn < sizeof(TURNS) / sizeof(*TURNS); turn++)
        for (int rate = MIN_FRAME_RATE; rate <= MAX_FRAME_RATE;
             rate += MAX_FRAME_RATE - MIN_FRAME_RATE) {
            reset_test_config();
            config.turning_notch = TURNS[turn];
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
                        bird_t bird = {.x = x, .y = y, .direction = 2 * M_PI * d / DIRECTIONS};
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
    static const int sizes[][2] = {{200, 50}, {80, 24}, {50, 15}};
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

/* Two flocks share the space without sharing a heading. */
static void test_the_recording_rate_is_one_a_gif_has(void) {
    /* A GIF's delay is whole hundredths, so the rates it can carry are 100/n. The
     * rate asked for is rounded to one of those, never faked. */
    assert(record_delay_for(50) == 2);
    assert(record_delay_for(25) == 4);
    assert(record_delay_for(20) == 5);
    assert(record_delay_for(10) == 10);
    assert(record_delay_for(2) == 50);

    /* Sixty is not a rate a GIF has: it comes back as the ceiling. */
    assert(record_delay_for(60) == 2);
    assert(record_delay_for(120) == 2);
    assert(100 / record_delay_for(60) == MAX_RECORD_FPS);

    /* Nothing ever asks a viewer for a delay it would clamp. */
    for (int fps = 2; fps <= 120; fps++) {
        int delay = record_delay_for(fps);
        assert(delay >= 2);
        assert(100 / delay <= MAX_RECORD_FPS);
        /* And it is the nearest rate the format has, compared as rates rather
         * than as delays: no other legal delay is closer to what was asked. */
        double got = 100.0 / delay;
        for (int other = 2; other <= 100; other++) {
            double mine = fabs(got - fps), theirs = fabs(100.0 / other - fps);
            assert(mine <= theirs + 1e-9);
        }
    }
}

static void test_birds_bank_rather_than_snap(void) {
    reset_test_config();
    config.turning_notch = DEFAULT_TURNING_NOTCH;
    double most = turn_limit();
    assert(most > 0 && most < 2 * M_PI);

    /* Asked to reverse, it turns as far as it may and no further. */
    double turned = turn_towards(0.0, M_PI, most);
    assert(angle_difference(turned, most) < 1e-12);

    /* It takes the short way round, through zero rather than the long way. */
    double from_high = turn_towards(2 * M_PI - 0.05, 0.05, most);
    assert(from_high >= 0 && from_high < 2 * M_PI);
    assert(angle_difference(from_high, 0.05) < 1e-12); /* Within reach, so it arrives. */
    double clockwise = turn_towards(0.05, 2 * M_PI - 0.05, most);
    assert(angle_difference(clockwise, 2 * M_PI - 0.05) < 1e-12);

    /* Anything already within the limit is simply reached. */
    assert(angle_difference(turn_towards(1.0, 1.0 + most / 2, most), 1.0 + most / 2) < 1e-12);

    /* The result is always on the circle, from anywhere to anywhere. */
    for (int a = 0; a < 360; a += 7)
        for (int b = 0; b < 360; b += 11) {
            double got = turn_towards(a * M_PI / 180, b * M_PI / 180, most);
            assert(got >= 0 && got < 2 * M_PI);
            assert(angle_difference(got, a * M_PI / 180) <= most + 1e-12);
        }

    /* Twelve notches is instant, which is what it always used to be. */
    config.turning_notch = LEGEND_BAR_CELLS;
    assert(angle_difference(turn_towards(0.0, M_PI, turn_limit()), M_PI) < 1e-12);
    /* And the bottom notch is a long lazy bank — a sixth of a turn a frame, so a
     * bird comes round in twelve — rather than nothing at all. Nothing meant the
     * edges could not turn the flock back either, and the whole of it left the
     * screen inside a second and stayed away; the bottom third of the bar was a
     * setting nobody could want. Every notch is now one somebody might. */
    config.turning_notch = 0;
    assert(turn_limit() > M_PI / 8);
    assert(turn_limit() < M_PI / 4);
    assert(turn_towards(1.0, 3.0, turn_limit()) > 1.0);
    assert(turn_towards(1.0, 3.0, turn_limit()) < 1.0 + M_PI / 4);
    /* And the bar still climbs from end to end. */
    double previous = turn_limit();
    for (int notch = 1; notch < LEGEND_BAR_CELLS; notch++) {
        config.turning_notch = notch;
        assert(turn_limit() > previous);
        previous = turn_limit();
    }

    /* Writing is exempt, or a bird could not land on a letter: it would circle
     * one, and the crispness of the letters is the whole point of them. */
    legend_enabled = 1;
    apply_screen_size(200, 50, 1600, 800);
    config.turning_notch = 0; /* The harshest limit there is. */
    assert(spell_layout("I") > 0);
    enum { BIRD_COUNT = 4 };
    bird_t birds[BIRD_COUNT], snapshot[BIRD_COUNT];
    spatial_grid_t grid;
    config.birds = BIRD_COUNT;
    for (int i = 0; i < BIRD_COUNT; i++)
        birds[i] = (bird_t){.x = spell.x[0] - config.speed / 2, .y = spell.y[0], .direction = M_PI};
    assert(spatial_grid_init(&grid, SPATIAL_CELL_SIZE) == SPATIAL_GRID_OK);
    assert(spatial_grid_prepare(&grid, screen.width, screen.height, BIRD_COUNT) == SPATIAL_GRID_OK);
    memcpy(snapshot, birds, sizeof(birds));
    assert(spatial_grid_build(&grid, BIRD_COUNT, read_bird_position, snapshot) == SPATIAL_GRID_OK);
    update_birds(birds, snapshot, &grid);
    assert(fabs(birds[0].x - spell.x[0]) < 1e-9); /* Landed, despite the limit. */
    spatial_grid_destroy(&grid);

    spell_clear();
    reset_test_config();
}

static void test_the_konami_code(void) {
    reset_test_config();
    legend_enabled = 1;
    apply_screen_size(200, 50, 1600, 800);
    config.hawks = 0;
    konami_at = 0;
    spell_clear();

    /* Nine of the ten is nine of the ten. */
    for (const char *c = "AABBDCDCb"; *c; c++) konami_note(*c);
    assert(config.hawks == 0);
    konami_note('a');
    assert(config.hawks == MAX_HAWKS);
    assert(spell.writing); /* And it says so. */

    /* A wrong key in the middle is a wrong sequence. */
    config.hawks = 0;
    spell_clear();
    konami_at = 0;
    memset(konami_seen, 0, sizeof(konami_seen));
    for (const char *c = "AABBDCDXCba"; *c; c++) konami_note(*c);
    assert(config.hawks == 0);

    /* People mash arrows, so a stutter in front must not throw it away: the last
     * ten keys are what count, not a running match. */
    for (const char *c = "AAABBDCDCba"; *c; c++) konami_note(*c);
    assert(config.hawks == MAX_HAWKS);

    /* And rubbish in front of a correct code still counts. */
    config.hawks = 0;
    spell_clear();
    for (const char *c = "qwertyAABBDCDCba"; *c; c++) konami_note(*c);
    assert(config.hawks == MAX_HAWKS);

    config.hawks = 0;
    konami_at = 0;
    spell_clear();
    reset_test_config();
}

/* The only wind left is the one that makes the rain fall, and it falls down. */
static void test_only_the_rain_has_a_wind(void) {
    reset_test_config();
    assert(!the_rain_is_falling);
    assert(wind_vector().x == 0 && wind_vector().y == 0);

    the_rain_is_falling = 1;
    vector_t falling = wind_vector();
    assert(falling.y > 0); /* Down the screen. */
    assert(falling.x == 0);
    the_rain_is_falling = 0;
    reset_test_config();
}

static void test_autopilot_wanders_and_yields(void) {
    reset_test_config();
    screensaver = 0;
    last_key_at = 0;
    clock_state.seconds = 0;

    /* Left alone it does nothing, because nothing has idled yet. */
    assert(!flying_itself());
    /* And after the idle time it takes over. */
    clock_state.seconds = IDLE_SECONDS + 1;
    assert(flying_itself());
    /* A screensaver takes over at once, with nothing to wait for. */
    clock_state.seconds = 0;
    screensaver = 1;
    assert(flying_itself());
    screensaver = 0;
    assert(!flying_itself());

    /* A drift moves exactly one notch, and stays inside the bar. */
    screensaver = 1;
    srand(7);
    for (int step = 0; step < 400; step++) {
        int before[] = {config.boundary_notch, config.separation_notch, config.alignment_notch,
                        config.vision_notch};
        drift_a_slider();
        int after[] = {config.boundary_notch, config.separation_notch, config.alignment_notch,
                       config.vision_notch};
        int moved = 0;
        for (size_t i = 0; i < sizeof(before) / sizeof(*before); i++) {
            assert(after[i] >= 0 && after[i] <= LEGEND_BAR_CELLS);
            if (after[i] != before[i]) {
                assert(after[i] - before[i] == 1 || before[i] - after[i] == 1);
                moved++;
            }
        }
        assert(moved == 1); /* One slider at a time, so a change reads as one. */
    }

    /* It keeps its hands off for a moment after a keypress: a slider the user is
     * holding is worse to fight over than one left alone. */
    clock_state.seconds = 100;
    last_key_at = 100;
    last_drift_at = 0;
    int held = config.boundary_notch;
    maybe_drift();
    assert(config.boundary_notch == held);
    clock_state.seconds = 100 + AUTOPILOT_YIELD + AUTOPILOT_PERIOD;
    maybe_drift();
    assert(last_drift_at == clock_state.seconds); /* And then it does move one. */

    screensaver = 0;
    clock_state.seconds = 0;
    last_key_at = 0;
    last_drift_at = 0;
    reset_test_config();
}

static void test_hawks_hunt_and_the_flock_flees(void) {
    enum { BIRD_COUNT = 20 };
    bird_t birds[BIRD_COUNT];
    spatial_grid_t grid;

    reset_test_config();
    legend_enabled = 0;
    apply_screen_size(200, 50, 1600, 800);
    config.birds = BIRD_COUNT;
    config.hawks = 1;
    place_hawks();

    /* One hawk in the middle, pointing the wrong way, and the flock off to its
     * right well out of reach. The birds are spread and each has a heading of its
     * own: piled on one point they are indistinguishable, and then a test that
     * says the hawk keeps to one bird is not saying anything. */
    hawks[0].x = 800;
    hawks[0].y = 400;
    hawks[0].direction = M_PI;
    hawks[0].prey = -1;
    hawks[0].commitment = 0;
    hawks[0].passing = 0;
    for (int i = 0; i < BIRD_COUNT; i++)
        birds[i] =
            (bird_t){.x = 1100 + (i % 5) * 20, .y = 340 + (i / 5) * 30, .direction = i * 0.3};

    /* It banks rather than snapping: one frame turns it by its limit and no more,
     * which is what stopped it reading as a glitch. */
    double before = hawks[0].direction;
    hunt(birds);
    double turned = angle_difference(hawks[0].direction, before);
    assert(turned > 0);
    assert(turned <= HAWK_TURN + 1e-9);
    int first = hawks[0].prey;
    assert(first >= 0); /* And it has chosen something. */

    /* And it arrives, against birds that are flocking and fleeing rather than
     * standing still: what it strikes is the bird it chose, at the distance a
     * strike is defined to happen at, having held that one bird the whole way in.
     * The chase that did not converge at all was the complaint — a wide turn and a
     * fixed aim ahead of the bird had it cutting across in front of the flock and
     * out the far side, round and round — and the chase that ended on whatever
     * else was nearby was worse, because it looked like a chase and was not. */
    assert(spatial_grid_init(&grid, SPATIAL_CELL_SIZE) == SPATIAL_GRID_OK);
    assert(spatial_grid_prepare(&grid, screen.width, screen.height, BIRD_COUNT) == SPATIAL_GRID_OK);
    double gap_before = distance_to_bird(birds, &hawks[0], first);
    int held = first, struck = -1;
    double struck_at = 0;
    bird_t moving[BIRD_COUNT];
    memcpy(moving, birds, sizeof(birds));
    for (int frame = 0; frame < 60 && struck < 0; frame++) {
        bird_t snapshot[BIRD_COUNT];
        memcpy(snapshot, moving, sizeof(moving));
        assert(spatial_grid_build(&grid, BIRD_COUNT, read_bird_position, snapshot) ==
               SPATIAL_GRID_OK);
        update_birds(moving, snapshot, &grid);
        int before = hawks[0].prey;
        /* Measured the way the strike is measured: along the whole of the step
         * the hawk is about to fly, not from where it happens to stand. */
        double reach = before >= 0 ? reach_along_the_step(snapshot, &hawks[0], before) : 0;
        hunt(snapshot);
        if (hawks[0].passing > 0 && before >= 0) {
            struck = before;
            struck_at = reach;
        } else if (hawks[0].prey >= 0) {
            held = hawks[0].prey;
        }
    }
    spatial_grid_destroy(&grid);
    assert(struck >= 0);                      /* It got there. */
    assert(struck == held);                   /* On the bird it was chasing. */
    assert(struck_at < config.bird_size * 2); /* Actually reached it. */
    assert(distance_to_bird(moving, &hawks[0], struck) < gap_before);

    /* It sticks with one bird rather than swapping every frame: that flip
     * flopping was the whole reason it looked broken. Even with another bird put
     * directly in its path, the commitment holds. */
    hawks[0].x = 400;
    hawks[0].y = 400;
    hawks[0].passing = 0;
    hawks[0].prey = -1;
    hawks[0].commitment = 0;
    hunt(birds);
    int chosen = hawks[0].prey;
    assert(chosen >= 0);
    int bait = (chosen + 1) % BIRD_COUNT;
    hawks[0].commitment = HAWK_COMMITMENT;
    birds[bait].x = hawks[0].x + 10;
    birds[bait].y = hawks[0].y + 10;
    hunt(birds);
    assert(hawks[0].prey == chosen || hawks[0].passing > 0);

    /* And once the commitment runs out it drops a bird it has not caught and takes
     * a fresh one, chosen from a distance rather than from under its nose: a
     * strike with no approach is over before anyone has seen it start. */
    hawks[0].passing = 0;
    hawks[0].prey = chosen;
    hawks[0].commitment = 0;
    birds[chosen].x = hawks[0].x + HAWK_GIVE_UP + 40; /* Outrun it. */
    birds[chosen].y = hawks[0].y;
    birds[bait].x = hawks[0].x + 80; /* And one right beside it. */
    birds[bait].y = hawks[0].y;
    int picked_from = -1;
    {
        /* Measured before the hawk moves: afterwards it is a step closer, and a
         * test that allows for the step is a test that would pass at 260 px when
         * the rule says 340. */
        hawk_t before_move = hawks[0];
        hunt(birds);
        picked_from = hawks[0].prey;
        assert(picked_from >= 0);
        assert(distance_to_bird(birds, &before_move, picked_from) >= HAWK_STALK);
    }
    assert(hawks[0].prey != chosen);
    assert(hawks[0].prey != bait);

    /* Unless there are not enough birds to go round, in which case they do share:
     * eight hawks and two birds is a legal thing to ask for, and a hawk with no
     * prey at all would simply stop hunting. */
    {
        int flock_was = config.birds;
        config.birds = 2;
        config.hawks = 4;
        place_hawks();
        for (int i = 0; i < config.hawks; i++) {
            hawks[i].prey = -1;
            hawks[i].commitment = 0;
            hawks[i].passing = 0;
        }
        hunt(birds);
        for (int i = 0; i < config.hawks; i++) assert(hawks[i].prey >= 0);
        config.birds = flock_was;
        config.hawks = 1;
    }

    /* Two hawks never share a bird: converging on one point they arrive on top of
     * each other and read as one hawk with a rendering fault. */
    config.hawks = 2;
    place_hawks();
    /* Exactly on the same spot, so the nearest bird is the same bird for both and
     * only the rule can separate them. */
    hawks[0].x = hawks[1].x = 400;
    hawks[0].y = hawks[1].y = 400;
    hawks[0].direction = hawks[1].direction = 0;
    hawks[0].prey = hawks[1].prey = -1;
    hawks[0].commitment = hawks[1].commitment = 0;
    hawks[0].passing = hawks[1].passing = 0;
    hunt(birds);
    assert(hawks[0].prey >= 0 && hawks[1].prey >= 0);
    assert(hawks[0].prey != hawks[1].prey);
    /* And each keeps its distance from the other: a nudge away from any hawk
     * inside HAWK_SPACING, nothing at all beyond it, so eight of them share a
     * flock instead of flying as one thick smear. */
    hawks[0].x = hawks[1].x = 400;
    hawks[0].y = 380;
    hawks[1].y = 420;
    assert(hawk_spacing(0).y < 0);
    assert(hawk_spacing(1).y == -hawk_spacing(0).y);
    hawks[1].y = 380 + HAWK_SPACING;
    assert(hawk_spacing(0).x == 0 && hawk_spacing(0).y == 0);
    config.hawks = 1;
    assert(hawk_spacing(0).x == 0 && hawk_spacing(0).y == 0);

    /* Flying among them starts a pass: it stops steering and goes straight out
     * the other side, which is what a stoop looks like from outside. */
    hawks[0].x = birds[0].x;
    hawks[0].y = birds[0].y;
    hawks[0].passing = 0;
    hawks[0].prey = 0;
    hunt(birds);
    assert(hawks[0].passing > 0);
    assert(hawks[0].prey < 0);
    double heading = hawks[0].direction;
    hunt(birds);
    assert(angle_difference(hawks[0].direction, heading) < 1e-12); /* Straight. */

    /* Turned back at a wall rather than pinned against it: pinning cost it a
     * quarter of every run sliding along an edge, and it turns while its whole
     * silhouette is still on the screen, because a placement that does not fit is
     * one the terminal drops and every wall used to cost a blink. */
    hawks[0].x = screen.width - 1;
    hawks[0].y = 400;
    hawks[0].direction = 0; /* Straight at the wall. */
    hawks[0].prey = -1;
    hawks[0].commitment = 30;
    hawks[0].passing = 30;
    hunt(birds);
    assert(hawks[0].x <= screen.width - 1 - hawk_draw_offset());
    assert(cos(hawks[0].direction) < 0); /* Sent back the other way. */
    hunt(birds);
    assert(hawks[0].x < screen.width - 1 - hawk_draw_offset()); /* And actually leaving. */

    /* A wall is seen a whole turning circle off, and the panel is a wall too: a
     * hawk that only saw the glass when it arrived bounced off it about once a
     * second, and one that could not see the panel at all flew over the sliders in
     * a sixth of every run. */
    config.hawks = 1;
    legend_enabled = 1;
    measure_legend();
    update_turn_distances();
    hawk_t near_left = {.x = 2, .y = screen.height / 2.0};
    hawk_t near_bottom = {.x = screen.width / 2.0, .y = screen.height - 2};
    assert(screen.legend_width > 0);
    assert(hawk_wall_vector(&near_left).x > 0);
    assert(hawk_wall_vector(&near_bottom).y < 0);
    assert(hawk_wall_band() > hawk_turning_radius()); /* Seen in time to act on. */

    /* A place the screen's own walls cannot reach, so that what is being measured
     * is the panel and only the panel: past the left band, below the top one, and
     * still inside the panel's own. */
    double band = hawk_wall_band();
    hawk_t beside_panel = {.x = screen.legend_width + band / 2, .y = band + 10};
    assert(beside_panel.x > band);                        /* Clear of the left band. */
    assert(beside_panel.y < screen.legend_height + band); /* Inside the panel's. */
    assert(hawk_wall_vector(&beside_panel).x > 0 || hawk_wall_vector(&beside_panel).y > 0);
    legend_enabled = 0;
    measure_legend();
    update_turn_distances();
    /* And with no panel there is nothing there to steer around. */
    assert(hawk_wall_vector(&beside_panel).x == 0 && hawk_wall_vector(&beside_panel).y == 0);

    /* A hawk's alarm carries less far on a small screen, or eight of them leave
     * the flock nowhere at all to be. */
    apply_screen_size(LEGEND_MIN_COLS, LEGEND_MIN_ROWS, LEGEND_MIN_COLS * 8, LEGEND_MIN_ROWS * 16);
    assert(hawk_reach() < HAWK_REACH);
    assert(fabs(hawk_reach() - screen.height / 3.0) < 1e-9);
    apply_screen_size(200, 50, 1600, 800);
    assert(hawk_reach() == HAWK_REACH);

    /* And over a long run against a flock that is flocking, the reflection at the
     * wall — the one thing in the chase that is not a bank — stays rare. */
    config.hawks = 2;
    place_hawks();
    assert(spatial_grid_init(&grid, SPATIAL_CELL_SIZE) == SPATIAL_GRID_OK);
    assert(spatial_grid_prepare(&grid, screen.width, screen.height, BIRD_COUNT) == SPATIAL_GRID_OK);
    for (int i = 0; i < BIRD_COUNT; i++)
        birds[i] = (bird_t){.x = 400 + (i % 5) * 30, .y = 300 + (i / 5) * 30, .direction = i * 0.3};
    int reflections = 0, steps = 600;
    double before_turn[MAX_HAWKS];
    for (int i = 0; i < config.hawks; i++) before_turn[i] = hawks[i].direction;
    for (int frame = 0; frame < steps; frame++) {
        bird_t snapshot[BIRD_COUNT];
        memcpy(snapshot, birds, sizeof(birds));
        assert(spatial_grid_build(&grid, BIRD_COUNT, read_bird_position, snapshot) ==
               SPATIAL_GRID_OK);
        update_birds(birds, snapshot, &grid);
        hunt(snapshot);
        for (int i = 0; i < config.hawks; i++) {
            if (angle_difference(hawks[i].direction, before_turn[i]) > hawk_turn_limit() + 1e-9)
                reflections++;
            before_turn[i] = hawks[i].direction;
        }
    }
    spatial_grid_destroy(&grid);
    assert(reflections * 25 < steps * config.hawks); /* Under four frames in a hundred. */
    config.hawks = 1;

    /* Every bird flees every hawk in reach, hardest when closest, not at all
     * beyond it. */
    config.hawks = 1;
    hawks[0].x = 800;
    hawks[0].y = 400;
    const bird_t close = {.x = 820, .y = 400};
    const bird_t further = {.x = 800 + HAWK_REACH - 20, .y = 400};
    const bird_t clear = {.x = 800 + HAWK_REACH + 1, .y = 400};
    assert(hawk_vector(&close).x > hawk_vector(&further).x);
    assert(hawk_vector(&further).x > 0);
    assert(hawk_vector(&clear).x == 0 && hawk_vector(&clear).y == 0);
    /* Part of the flee is sideways, so the flock streams around the hawk and
     * closes behind it instead of bursting straight open. */
    assert(fabs(hawk_vector(&close).y) > 0);
    /* And on a small screen the alarm carries less far: the bird that felt it at
     * a hundred and thirty pixels on a big screen feels nothing here. */
    apply_screen_size(LEGEND_MIN_COLS, LEGEND_MIN_ROWS, LEGEND_MIN_COLS * 8, LEGEND_MIN_ROWS * 16);
    hawks[0].x = screen.width / 2.0;
    hawks[0].y = screen.height / 2.0;
    const bird_t far_on_a_small_screen = {.x = screen.width / 2.0 + HAWK_REACH - 20,
                                          .y = screen.height / 2.0};
    const bird_t near_on_a_small_screen = {.x = screen.width / 2.0 + 10, .y = screen.height / 2.0};
    assert(hawk_vector(&far_on_a_small_screen).x == 0);
    assert(hawk_vector(&near_on_a_small_screen).x > 0);
    apply_screen_size(200, 50, 1600, 800);
    hawks[0].x = 800;
    hawks[0].y = 400;
    config.hawks = 0;
    assert(hawk_vector(&close).x == 0);

    /* And a hawk overrules a flock heading into it. */
    config.hawks = 1;
    for (int i = 0; i < BIRD_COUNT; i++) birds[i] = (bird_t){.x = 830, .y = 400, .direction = M_PI};
    assert(spatial_grid_init(&grid, SPATIAL_CELL_SIZE) == SPATIAL_GRID_OK);
    assert(spatial_grid_prepare(&grid, screen.width, screen.height, BIRD_COUNT) == SPATIAL_GRID_OK);
    assert(spatial_grid_build(&grid, BIRD_COUNT, read_bird_position, birds) == SPATIAL_GRID_OK);
    assert(cos(flock_direction(birds, &grid, 0)) > 0);
    spatial_grid_destroy(&grid);

    /* However long the chase runs, every hawk stays wholly on the screen: not
     * merely inside it, but far enough in that all of its sprite is too. Tried on
     * the smallest viewport the program will run in as well, where the distance a
     * hawk needs to see a wall coming is wider than half the screen. */
    static const int VIEWPORTS[][2] = {{200, 50}, {LEGEND_MIN_COLS, LEGEND_MIN_ROWS}};
    for (size_t view = 0; view < sizeof(VIEWPORTS) / sizeof(*VIEWPORTS); view++) {
        apply_screen_size(VIEWPORTS[view][0], VIEWPORTS[view][1], VIEWPORTS[view][0] * 8,
                          VIEWPORTS[view][1] * 16);
        config.hawks = MAX_HAWKS;
        place_hawks();
        double least_x = screen.width, most_x = 0, least_y = screen.height, most_y = 0;
        for (int step = 0; step < 400; step++) {
            hunt(birds);
            for (int i = 0; i < config.hawks; i++) {
                assert(hawks[i].x >= hawk_draw_offset());
                assert(hawks[i].y >= hawk_draw_offset());
                assert(hawks[i].x <= screen.width - 1 - hawk_draw_offset());
                assert(hawks[i].y <= screen.height - 1 - hawk_draw_offset());
                if (hawks[i].x < least_x) least_x = hawks[i].x;
                if (hawks[i].x > most_x) most_x = hawks[i].x;
                if (hawks[i].y < least_y) least_y = hawks[i].y;
                if (hawks[i].y > most_y) most_y = hawks[i].y;
            }
        }
        /* In the middle of the screen there is no wall to be seen, whatever the
         * turning circle is: a band wider than half the screen would otherwise
         * reach past the centre and push the same way from everywhere. */
        hawk_t middle = {.x = screen.width / 2.0, .y = screen.height / 2.0};
        assert(hawk_wall_vector(&middle).x == 0);
        assert(hawk_wall_vector(&middle).y == 0);

        /* And they use the screen rather than being held against one side of it:
         * the distance a hawk needs to see a wall coming can be wider than half a
         * small screen, and unclamped it would be pushed the same way from every
         * position on it. */
        assert(most_x - least_x > screen.width / 2.0);
        assert(most_y - least_y > screen.height / 2.0);
    }
    apply_screen_size(200, 50, 200 * 8, 50 * 16);

    /* Summoning one leaves the others exactly where they were hunting. */
    config.hawks = 3;
    place_hawks();
    for (int step = 0; step < 10; step++) hunt(birds);
    double kept_x = hawks[0].x, kept_y = hawks[0].y;
    config.hawks = 4;
    place_one_hawk(config.hawks - 1);
    assert(hawks[0].x == kept_x && hawks[0].y == kept_y);

    config.hawks = 0;
    legend_enabled = 1;
    reset_test_config();
}

/* Both turn limits are radians a second dressed as radians a frame, so they have
 * to be divided by the frame rate: left alone, --fps 30 moved a bird twice as far
 * per frame and allowed it the same turn for it, and one bird in eight ended up
 * off the screen against one in thirty at sixty. */
static void test_the_turn_limits_follow_the_frame_rate(void) {
    reset_test_config();
    /* Roomy, so that the step is set by the frame rate alone: on a small screen
     * it is capped by the screen instead, which is the next thing asserted. */
    apply_screen_size(200, 60, 1600, 960);
    config.turning_notch = 6;

    config.frame_rate = DEFAULT_FRAME_RATE;
    update_speed();
    double bird_at_sixty = turn_limit(), hawk_at_sixty = hawk_turn_limit();
    double step_at_sixty = config.speed;

    config.frame_rate = DEFAULT_FRAME_RATE / 2;
    update_speed();
    assert(fabs(config.speed - step_at_sixty * 2) < 1e-9);      /* Twice the ground. */
    assert(fabs(turn_limit() - bird_at_sixty * 2) < 1e-9);      /* Twice the turn. */
    assert(fabs(hawk_turn_limit() - hawk_at_sixty * 2) < 1e-9); /* Both of them. */

    config.frame_rate = DEFAULT_FRAME_RATE * 2;
    update_speed();
    assert(fabs(turn_limit() - bird_at_sixty / 2) < 1e-9);
    assert(fabs(hawk_turn_limit() - hawk_at_sixty / 2) < 1e-9);

    /* And a bird never crosses more than a tenth of the shorter side in one
     * frame, however low the rate goes: it cannot turn inside a band it clears in
     * two frames, and on a forty by fourteen terminal one bird in six was off the
     * screen because of it. */
    config.frame_rate = MIN_FRAME_RATE;
    apply_screen_size(40, 14, 320, 224);
    assert(config.speed <= screen.height / 10.0 + 1e-9);
    apply_screen_size(200, 60, 1600, 960);
    assert(config.speed > screen.height / 20.0);

    /* Instant stays instant, and nothing ever exceeds a half turn a frame. */
    config.turning_notch = LEGEND_BAR_CELLS;
    config.frame_rate = MIN_FRAME_RATE;
    update_speed();
    assert(turn_limit() == 2 * M_PI);
    assert(hawk_turn_limit() <= M_PI);
    reset_test_config();
}

/* One flock is coloured by heading, more than one by flock. There is no flag: the
 * two answers anybody wanted are the only two there are. */
static void test_more_flocks_are_more_colours(void) {
    reset_test_config();
    config.palette = palette_named("ember");
    assert(palette_shades() == 5);

    /* One flock, and the colour follows the heading: two birds flying different
     * ways are different colours. */
    config.flocks = 1;
    bird_t east = {.direction = 0, .flock = 0};
    bird_t west = {.direction = M_PI, .flock = 0};
    assert(shade_for(&east) != shade_for(&west));

    /* Three flocks, and the colour follows the flock instead: every bird of one
     * is one colour whatever it is doing, and the three are three colours. */
    config.flocks = 3;
    int seen[MAX_PALETTE_SHADES] = {0};
    for (int flock = 0; flock < 3; flock++) {
        bird_t member = {.direction = 1.0, .flock = flock};
        bird_t other_way = {.direction = 4.0, .flock = flock};
        int shade = shade_for(&member);
        assert(shade == shade_for(&other_way));
        assert(shade >= 0 && shade < palette_shades());
        seen[shade] = 1;
    }
    int distinct = 0;
    for (int i = 0; i < palette_shades(); i++) distinct += seen[i];
    assert(distinct == 3); /* Three flocks, three shades. */
    /* And they are spread: the two outer flocks get the ends of the ramp, so the
     * difference between them is the widest the palette has to offer. */
    bird_t lowest = {.flock = 0}, highest = {.flock = 2};
    assert(shade_for(&lowest) == 0);
    assert(shade_for(&highest) == palette_shades() - 1);
    reset_test_config();
}

static void test_the_flock_can_be_laid_out_as_text(void) {
    reset_test_config();
    legend_enabled = 1;
    apply_screen_size(200, 50, 1600, 800);

    /* A target per lit cell of the font, minus any that would fall in the
     * panel's turn zone, where no bird could ever reach one. */
    int made = spell_layout("HELLO");
    assert(made > 0 && made <= font_text_cells("HELLO"));
    assert(spell.writing);
    for (int i = 0; i < made; i++) {
        assert(!legend_turn_zone(spell.x[i], spell.y[i]));
        assert(spell.x[i] >= 0 && spell.x[i] <= screen.width);
        assert(spell.y[i] >= 0 && spell.y[i] <= screen.height);
    }

    /* Birds share targets round robin, so a cell with several on it reads as a
     * thick stroke rather than leaving the rest of the flock idle. */
    double first_x, first_y, wrapped_x, wrapped_y;
    assert(spell_target_of(0, &first_x, &first_y));
    assert(spell_target_of(made, &wrapped_x, &wrapped_y));
    assert(first_x == wrapped_x && first_y == wrapped_y);

    /* A bird with a target steers at it and ignores its neighbours entirely. */
    enum { BIRD_COUNT = 8 };
    bird_t birds[BIRD_COUNT];
    spatial_grid_t grid;
    config.birds = BIRD_COUNT;
    for (int i = 0; i < BIRD_COUNT; i++)
        birds[i] = (bird_t){.x = spell.x[0] - 100, .y = spell.y[0], .direction = M_PI};
    assert(spatial_grid_init(&grid, SPATIAL_CELL_SIZE) == SPATIAL_GRID_OK);
    assert(spatial_grid_prepare(&grid, screen.width, screen.height, BIRD_COUNT) == SPATIAL_GRID_OK);
    assert(spatial_grid_build(&grid, BIRD_COUNT, read_bird_position, birds) == SPATIAL_GRID_OK);
    assert(angle_difference(flock_direction(birds, &grid, 0), 0.0) < 1e-12);

    /* And it lands exactly rather than orbiting: one update from a whole speed
     * away puts it on the target, not past it. */
    bird_t snapshot[BIRD_COUNT];
    birds[0].x = spell.x[0] - config.speed / 2;
    birds[0].y = spell.y[0];
    memcpy(snapshot, birds, sizeof(birds));
    update_birds(birds, snapshot, &grid);
    assert(fabs(birds[0].x - spell.x[0]) < 1e-9);
    assert(fabs(birds[0].y - spell.y[0]) < 1e-9);
    spatial_grid_destroy(&grid);

    /* Letting go hands the flock back to the flocking rules. */
    spell_clear();
    assert(!spell.writing);
    assert(!spell_target_of(0, &first_x, &first_y));

    /* Text with nothing to draw, and text that cannot fit, both decline. */
    assert(spell_layout("") == 0);
    assert(spell_layout("\x01\x02") == 0);
    apply_screen_size(44, 15, 44 * 8, 15 * 16);
    assert(spell_layout("A VERY LONG MESSAGE INDEED THAT WILL NOT FIT AT ALL") == 0);
    assert(!spell.writing);
    reset_test_config();
}

static void test_presets_set_every_notch(void) {
    reset_test_config();
    /* Each preset names a whole look, so every one of them has to move at least
     * one notch off the default, or it is not a look. */
    for (int i = 0; i < PRESET_COUNT; i++) {
        apply_preset(i);
        int notches[] = {config.boundary_notch, config.separation_notch, config.alignment_notch,
                         config.vision_notch, config.rate_notch};
        int moved = 0;
        for (size_t k = 0; k < sizeof(notches) / sizeof(*notches); k++) {
            assert(notches[k] >= 0 && notches[k] <= LEGEND_BAR_CELLS);
            if (notches[k] != DEFAULT_NOTCH) moved = 1;
        }
        assert(moved);
        /* And the derived values follow, as they do for a keypress. */
        assert(config.boundary >= BOUNDARY_MIN && config.boundary <= BOUNDARY_MAX);
        assert(config.frame_rate >= MIN_FRAME_RATE && config.frame_rate <= MAX_FRAME_RATE);
        assert(config.vision_radius >= MIN_VISION_RADIUS &&
               config.vision_radius <= MAX_VISION_RADIUS);
    }
    /* Every preset is a different look from every other. */
    for (int i = 0; i < PRESET_COUNT; i++)
        for (int j = i + 1; j < PRESET_COUNT; j++)
            assert(memcmp(PRESETS[i].notch, PRESETS[j].notch, sizeof(PRESETS[i].notch)) != 0);
    reset_test_config();
}

static void test_a_notch_survives_the_round_trip(void) {
    /* --perception and --fps take real units and snap. A value that is already on
     * the grid has to come back as itself, or a dotfile could not express one. */
    for (int notch = 0; notch <= LEGEND_BAR_CELLS; notch++) {
        int pixels = notch_integer(notch, MIN_VISION_RADIUS, MAX_VISION_RADIUS);
        assert(notch_for_integer(pixels, MIN_VISION_RADIUS, MAX_VISION_RADIUS) == notch);
        int rate = notch_integer(notch, MIN_FRAME_RATE, MAX_FRAME_RATE);
        assert(notch_for_integer(rate, MIN_FRAME_RATE, MAX_FRAME_RATE) == notch);
    }
    /* And anything between snaps to the nearer of the two. */
    assert(notch_for_integer(MIN_VISION_RADIUS, MIN_VISION_RADIUS, MAX_VISION_RADIUS) == 0);
    assert(notch_for_integer(MAX_VISION_RADIUS, MIN_VISION_RADIUS, MAX_VISION_RADIUS) ==
           LEGEND_BAR_CELLS);
    assert(notch_for_integer(100, MIN_FRAME_RATE, MAX_FRAME_RATE) == 9); /* 98 is notch nine. */
}

static void test_the_pointer_moves_the_flock(void) {
    reset_test_config();
    legend_enabled = 0;
    apply_screen_size(200, 50, 200 * 8, 50 * 16);
    mouse.present = 1;
    mouse.x = 800;
    mouse.y = 400;

    /* A bird to the right of the pointer flees further right, and is drawn
     * towards it in follow. Straight along the line between them either way. */
    const bird_t east = {.x = 800 + 40, .y = 400};
    config.mouse_mode = MOUSE_FLEE;
    vector_t away = pointer_vector(&east);
    assert(away.x > 0 && fabs(away.y) < 1e-12);
    config.mouse_mode = MOUSE_FOLLOW;
    vector_t towards = pointer_vector(&east);
    assert(towards.x < 0 && fabs(towards.y) < 1e-12);
    assert(fabs(towards.x + away.x) < 1e-12); /* Exactly opposite. */

    /* It falls off with distance and stops at its reach, so the flock bends
     * around the pointer and closes behind it rather than bouncing off. */
    config.mouse_mode = MOUSE_FLEE;
    const bird_t near = {.x = 800 + 10, .y = 400};
    const bird_t far = {.x = 800 + 100, .y = 400};
    const bird_t beyond = {.x = 800 + config.mouse_reach + 1, .y = 400};
    assert(pointer_vector(&near).x > pointer_vector(&far).x);
    assert(pointer_vector(&beyond).x == 0 && pointer_vector(&beyond).y == 0);

    /* Off means off, and so does never having seen the pointer. */
    config.mouse_mode = MOUSE_OFF;
    assert(pointer_vector(&east).x == 0);
    config.mouse_mode = MOUSE_FLEE;
    mouse.present = 0;
    assert(pointer_vector(&east).x == 0);
    mouse.present = 1;

    /* And it overrules a flock that wants to go the other way. */
    enum { BIRD_COUNT = 30 };
    bird_t birds[BIRD_COUNT];
    spatial_grid_t grid;
    config.birds = BIRD_COUNT;
    config.mouse_mode = MOUSE_FLEE;
    for (int i = 0; i < BIRD_COUNT; i++) birds[i] = (bird_t){.x = 840, .y = 400, .direction = M_PI};
    assert(spatial_grid_init(&grid, SPATIAL_CELL_SIZE) == SPATIAL_GRID_OK);
    assert(spatial_grid_prepare(&grid, screen.width, screen.height, BIRD_COUNT) == SPATIAL_GRID_OK);
    assert(spatial_grid_build(&grid, BIRD_COUNT, read_bird_position, birds) == SPATIAL_GRID_OK);
    assert(cos(flock_direction(birds, &grid, 0)) > 0); /* Away, not with them. */
    config.mouse_mode = MOUSE_OFF;
    assert(cos(flock_direction(birds, &grid, 0)) < 0); /* With them again. */
    spatial_grid_destroy(&grid);

    mouse.present = 0;
    clock_state.seconds = 0;
    legend_enabled = 1;
    reset_test_config();
}

static void test_the_shade_follows_the_heading(void) {
    reset_test_config();
    config.palette = palette_named("ember");
    int shades = palette_shades();
    assert(shades == 5);

    /* Every shade of the ramp is reachable, and the colour runs smoothly with the
     * angle — up to the half turn and back down again, because a heading is a
     * circle and a ramp is a line. Laid straight on to it there was a seam at due
     * east where one degree of turn crossed the whole palette, and the flock came
     * out salted with speckle that no turn of it explained. */
    int seen[8] = {0};
    int previous = shade_for(&(bird_t){.direction = 0});
    for (int step = 0; step < 360; step++) {
        bird_t bird = {.direction = step * M_PI / 180.0};
        int shade = shade_for(&bird);
        assert(shade >= 0 && shade < shades);
        assert(abs(shade - previous) <= 1); /* No jump anywhere, seam included. */
        previous = shade;
        seen[shade] = 1;
    }
    for (int i = 0; i < shades; i++) assert(seen[i]);
    /* And it climbs to the half turn and comes back, rather than wrapping. */
    assert(shade_for(&(bird_t){.direction = 0}) == 0);
    assert(shade_for(&(bird_t){.direction = M_PI}) == shades - 1);
    assert(shade_for(&(bird_t){.direction = 2 * M_PI - 0.001}) == 0);

    /* A palette with one shade has nothing to choose. Somebody's own sprite is
     * that case: it keeps the colours it was drawn in, so there is one set of
     * images and nothing is tinted. */
    sprite_path = "somebody.png";
    assert(palette_shades() == 1);
    bird_t any = {.direction = 2.0};
    assert(shade_for(&any) == 0);
    sprite_path = NULL;
    reset_test_config();
}

/* A hawk has to be findable in one glance, in every palette. Scarlet did that
 * everywhere except on the warm ramps, where it is just another ember. */
static void test_the_hawk_is_never_the_colour_of_the_flock(void) {
    for (config.palette = 0; config.palette < PALETTE_COUNT; config.palette++) {
        if (palette_follows_the_theme()) {
            static const uint8_t ACCENT[3] = {205, 0, 0}, GROUND[3] = {18, 18, 24};
            ramp_between(ACCENT, GROUND);
        }
        const uint8_t *hawk = hawk_colour();
        double nearest = 1e9;
        for (int shade = 0; shade < palette()->shades; shade++) {
            double gap = colour_distance(hawk, palette()->tints[shade]);
            if (gap < nearest) nearest = gap;
        }
        /* Two hundred is about the distance from scarlet to a mid grey: further
         * than any two shades of one ramp ever are from each other. */
        assert(nearest > 200);

        /* And the sprite is actually painted with it. */
        png_image_t feather = {0, 0, NULL};
        assert(png_image_alloc(&feather, 1, 1) == PNG_OK);
        feather.pixels[3] = 255;
        hawk_tint(&feather);
        assert(feather.pixels[0] == hawk[0]);
        assert(feather.pixels[1] == hawk[1]);
        assert(feather.pixels[2] == hawk[2]);
        png_image_free(&feather);

        /* And it is the best of the candidates, not merely an acceptable one. */
        for (int candidate = 0; candidate < HAWK_COLOUR_COUNT; candidate++) {
            double other = 1e9;
            for (int shade = 0; shade < palette()->shades; shade++) {
                double gap = colour_distance(HAWK_COLOURS[candidate], palette()->tints[shade]);
                if (gap < other) other = gap;
            }
            assert(other <= nearest);
        }
    }
    reset_test_config();
}

/* The one colour a bird must never be is the colour of the sky behind it. The
 * ramp used to end exactly on the terminal's background: a fifth of the flock was
 * invisible on every scheme, and with three flocks up one whole flock was. */
static void test_the_theme_ramp_never_reaches_the_background(void) {
    static const uint8_t GROUNDS[][3] = {
        {0, 0, 0}, {18, 18, 24}, {40, 42, 54}, {0, 43, 54}, {253, 246, 227},
    };
    static const uint8_t ACCENTS[][3] = {
        {205, 0, 0}, {0, 0, 238}, {189, 147, 249}, {251, 73, 52}, {42, 161, 152},
    };
    for (size_t g = 0; g < sizeof(GROUNDS) / sizeof(*GROUNDS); g++)
        for (size_t a = 0; a < sizeof(ACCENTS) / sizeof(*ACCENTS); a++) {
            ramp_between(ACCENTS[a], GROUNDS[g]);
            for (int shade = 0; shade < 5; shade++) {
                assert(memcmp(theme_tints[shade], GROUNDS[g], 3) != 0);
                /* And not merely different: the far end stops short of two thirds
                 * of the way there, so there is always some of the accent left. */
                for (int c = 0; c < 3; c++) {
                    int travelled = abs((int)theme_tints[shade][c] - (int)ACCENTS[a][c]);
                    int whole = abs((int)GROUNDS[g][c] - (int)ACCENTS[a][c]);
                    assert(travelled * 3 <= whole * 2 + 1);
                }
            }
            /* And it is still a ramp: every step is nearer the ground than the
             * one before it, or the five shades are not a ramp at all. */
            for (int shade = 1; shade < 5; shade++) {
                double near = contrast_between(theme_tints[shade - 1], GROUNDS[g]);
                double far = contrast_between(theme_tints[shade], GROUNDS[g]);
                assert(far <= near);
            }
        }

    /* On the common ground — a dark terminal — and with an accent the terminal
     * can actually show, every shade stands off it. */
    static const uint8_t DARK[3] = {18, 18, 24};
    for (size_t a = 0; a < sizeof(ACCENTS) / sizeof(*ACCENTS); a++) {
        if (contrast_between(ACCENTS[a], DARK) < 3) continue;
        ramp_between(ACCENTS[a], DARK);
        for (int shade = 0; shade < 5; shade++)
            assert(contrast_between(theme_tints[shade], DARK) > 1.3);
    }
}

static void test_theme_colours_are_parsed(void) {
    uint8_t rgb[3];

    /* Four hex digits a channel is the usual answer. */
    assert(parse_osc_colour("\033]4;1;rgb:cc24/1d1d/1f1f\033\\", rgb));
    assert(rgb[0] == 0xcc && rgb[1] == 0x1d && rgb[2] == 0x1f);

    /* Some terminals send two, and the same parse has to cope. */
    assert(parse_osc_colour("\033]11;rgb:12/34/56\033\\", rgb));
    assert(rgb[0] == 0x12 && rgb[1] == 0x34 && rgb[2] == 0x56);

    /* Anything else is simply not an answer. */
    assert(!parse_osc_colour("", rgb));
    assert(!parse_osc_colour("\033]4;1;?\033\\", rgb));
    assert(!parse_osc_colour("rgb:", rgb));
    assert(!parse_osc_colour("rgb:zz/zz/zz", rgb));

    /* The accent is the most saturated answer, so grey never wins. */
    static const uint8_t grey[3] = {128, 128, 128};
    static const uint8_t red[3] = {204, 29, 31};
    assert(saturation_of(grey) == 0);
    assert(saturation_of(red) > saturation_of(grey));

    /* The ramp starts at the accent and heads for the background. */
    static const uint8_t ground[3] = {18, 18, 24};
    ramp_between(red, ground);
    assert(theme_tints[0][0] == red[0] && theme_tints[0][1] == red[1]);
    assert(theme_tints[4][0] < theme_tints[0][0]); /* Fading that way. */
    for (int i = 1; i < 5; i++) assert(theme_tints[i][0] <= theme_tints[i - 1][0]);
}

/* Two flocks at the same speed pass through each other symmetrically and it looks
 * like one flock with two colours. A little apart, they shear. */
static void test_each_flock_flies_at_its_own_pace(void) {
    reset_test_config();
    config.flocks = 1;
    assert(flock_pace(0) == 1.0);
    config.flocks = 3;
    assert(flock_pace(0) == 1.0);
    assert(flock_pace(1) < flock_pace(0));
    assert(flock_pace(2) < flock_pace(1));
    assert(flock_pace(2) > 0.8); /* Different, not crippled. */

    /* And the difference shows up in the distance covered. */
    apply_screen_size(200, 50, 1600, 800);
    bird_t birds[2], snapshot[2];
    spatial_grid_t grid;
    config.birds = 2;
    /* Both well clear of every edge band, so what is being compared is the pace
     * and nothing else: started inside one, the band bends one of them and the
     * difference being measured is not the one the test is named after. */
    birds[0] = (bird_t){.x = 800, .y = 400, .direction = 0, .flock = 0};
    birds[1] = (bird_t){.x = 800, .y = 450, .direction = 0, .flock = 2};
    memcpy(snapshot, birds, sizeof(birds));
    assert(spatial_grid_init(&grid, SPATIAL_CELL_SIZE) == SPATIAL_GRID_OK);
    assert(spatial_grid_prepare(&grid, screen.width, screen.height, 2) == SPATIAL_GRID_OK);
    assert(spatial_grid_build(&grid, 2, read_bird_position, snapshot) == SPATIAL_GRID_OK);
    update_birds(birds, snapshot, &grid);
    assert(boundary_vector(&snapshot[0]).x == 0 && boundary_vector(&snapshot[0]).y == 0);
    assert(boundary_vector(&snapshot[1]).x == 0 && boundary_vector(&snapshot[1]).y == 0);
    assert(birds[0].x - 800 > birds[1].x - 800);
    spatial_grid_destroy(&grid);
    reset_test_config();
}

/* --matrix is the rain: green, falling, wrapping, with tails. The falling and the
 * wrapping used to be two switches of their own, which is how a flock came to be
 * able to leave the screen by one edge and arrive at the other in broad daylight
 * — and, with nothing to break the symmetry, converge on a single heading in two
 * seconds and stay there for as long as anybody watched. */
static void test_the_matrix_is_the_only_thing_that_rains(void) {
    char *argv[] = {"cbirds", "--matrix", "--flocks", "3", NULL};

    reset_test_config();
    the_rain_is_falling = 0;
    read_options(4, argv);
    assert(config.palette == palette_named("matrix"));
    assert(config.trails == 1);
    assert(the_rain_is_falling == 1);
    /* And three flocks are still three colours in it. */
    bird_t first = {.flock = 0}, last = {.flock = 2};
    assert(shade_for(&first) != shade_for(&last));
    the_rain_is_falling = 0;
    reset_test_config();
}

/* A terminal with no graphics protocol gets the same flock as text: braille by
 * default, half blocks on request, and never a Kitty command. */
static void test_a_text_terminal_gets_the_flock_in_braille(void) {
    kitty_graphics_t graphics;
    bird_t birds[3];

    reset_test_config();
    legend_enabled = 0;
    config.palette = palette_named("ember");
    config.birds = 3;
    config.hawks = 1;
    apply_screen_size(60, 20, 480, 320);
    birds[0] = (bird_t){.x = 100, .y = 100, .direction = 0.3};
    birds[1] = (bird_t){.x = 200, .y = 150, .direction = 2.0};
    birds[2] = (bird_t){.x = 300, .y = 200, .direction = 4.0};
    place_hawks();

    render_mode = RENDER_BRAILLE;
    assert(drawing_with_text());
    assert(prepare_text_renderer());
    assert(kitty_graphics_init(&graphics, STDOUT_FILENO) == KITTY_GRAPHICS_OK);
    assert(queue_render_frame(&graphics, birds) == KITTY_GRAPHICS_OK);

    /* Synchronised, coloured, in braille, and not one graphics command. */
    assert(strstr(graphics.buffer, "\033[?2026h") == graphics.buffer);
    assert(strstr(graphics.buffer, "\033_G") == NULL);
    assert(strstr(graphics.buffer, "\033[38;2;") != NULL ||
           strstr(graphics.buffer, "\033[38;5;") != NULL);
    int braille = 0;
    for (const unsigned char *c = (const unsigned char *)graphics.buffer; *c; c++)
        if (c[0] == 0xE2 && (c[1] & 0xFC) == 0xA0) braille++; /* U+2800..U+28FF. */
    assert(braille >= 3);                                     /* At least a dot cell per bird. */
    assert(strstr(graphics.buffer, "\033[2J") == NULL);       /* The rule holds here too. */

    /* The same frame again is nothing but the brackets: only changes are sent. */
    size_t first = graphics.length;
    graphics.length = 0;
    assert(queue_render_frame(&graphics, birds) == KITTY_GRAPHICS_OK);
    assert(graphics.length < first / 4);

    /* Half blocks on request. */
    render_mode = RENDER_BLOCKS;
    graphics.length = 0;
    cells_invalidate(&text_cells);
    assert(queue_render_frame(&graphics, birds) == KITTY_GRAPHICS_OK);
    assert(strstr(graphics.buffer, "\xe2\x96\x80") != NULL ||
           strstr(graphics.buffer, "\xe2\x96\x84") != NULL); /* U+2580 or U+2584. */

    /* And a snapshot under either is a picture of the cells, the size of the
     * screen, not of the pixels they were read from. */
    char path[] = "/tmp/cbirds_text_snapshot.png";
    assert(write_snapshot(path, birds));
    FILE *file = fopen(path, "rb");
    assert(file != NULL);
    static uint8_t bytes[1 << 20];
    size_t length = fread(bytes, 1, sizeof(bytes), file);
    fclose(file);
    remove(path);
    png_image_t picture = {0, 0, NULL};
    assert(png_decode(bytes, length, &picture) == PNG_OK);
    assert(picture.width == screen.cols * screen.cell_width);
    assert(picture.height == screen.rows * screen.cell_height);
    png_image_free(&picture);

    kitty_graphics_destroy(&graphics);
    cells_destroy(&text_cells);
    png_image_free(&text_canvas);
    free_sprites(text_sprites);
    render_mode = RENDER_KITTY;
    config.hawks = 0;
    legend_enabled = 1;
    reset_test_config();
}

/* A GIF has no panel in it, so it must not have a hole where one would be — and
 * it is drawn without a terminal, so the palette that asks the terminal what
 * colours it uses has to fall back to one that has colours in it. */
static void test_recording_gives_the_whole_frame_to_the_flock(void) {
    char path[] = "/tmp/cbirds_record_test.gif";
    legend_enabled = 1;
    reset_test_config();
    config.palette = palette_named("theme");
    assert(palette_follows_the_theme());
    config.birds = 40;
    record_path = path;
    record_fps = 25;
    record_seconds = 1;
    record_columns = 60;
    record_rows = 20;
    /* It reports what it wrote on stdout, which in a test run is noise. */
    fflush(stdout);
    int saved = dup(STDOUT_FILENO);
    FILE *quiet = freopen("/dev/null", "w", stdout);
    assert(quiet != NULL);
    int status = run_recording();
    fflush(stdout);
    dup2(saved, STDOUT_FILENO);
    close(saved);
    clearerr(stdout);
    assert(status == EXIT_SUCCESS);
    /* Black birds on a black ground was what `cbirds --record flock.gif` wrote:
     * the theme's ramp is learned from the terminal, and there is no terminal. */
    assert(!palette_follows_the_theme());
    assert(palette_shades() > 1);
    assert(legend_enabled == 0);
    assert(screen.legend_width == 0 && screen.legend_height == 0);
    remove(path);
    record_path = NULL;
    legend_enabled = 1;
    reset_test_config();
}

/* Two flocks that cannot see past their own noses drift through each other within
 * a couple of seconds, and what is left is one flock in three colours. The leash
 * to each flock's own centre, and the shove those centres give each other, are
 * what make a flock a body and three flocks three bodies. */
static void test_flocks_keep_to_their_own_side_of_the_sky(void) {
    enum { BIRD_COUNT = 300 };
    static bird_t birds[BIRD_COUNT], snapshot[BIRD_COUNT];
    spatial_grid_t grid;

    for (int flocks = 2; flocks <= MAX_FLOCKS; flocks++) {
        reset_test_config();
        legend_enabled = 0;
        apply_screen_size(100, 28, 800, 448);
        config.birds = BIRD_COUNT;
        config.flocks = flocks;

        /* All of them started in one heap in the middle, which is the hardest
         * case: if they sort themselves out from there they sort themselves out
         * from anywhere. */
        for (int i = 0; i < BIRD_COUNT; i++)
            birds[i] = (bird_t){.x = screen.width / 2.0 + (i % 17) - 8,
                                .y = screen.height / 2.0 + (i % 13) - 6,
                                .direction = i * 0.21,
                                .flock = i % flocks};

        assert(spatial_grid_init(&grid, SPATIAL_CELL_SIZE) == SPATIAL_GRID_OK);
        assert(spatial_grid_prepare(&grid, screen.width, screen.height, BIRD_COUNT) ==
               SPATIAL_GRID_OK);
        double gap_sum = 0;
        int measured = 0;
        for (int frame = 0; frame < 700; frame++) {
            memcpy(snapshot, birds, sizeof(birds));
            assert(spatial_grid_build(&grid, BIRD_COUNT, read_bird_position, snapshot) ==
                   SPATIAL_GRID_OK);
            update_birds(birds, snapshot, &grid);
            if (frame < 300) continue; /* Time to sort themselves out first. */

            /* Home is always somewhere a flock can be. A home off the screen puts
             * the leash and the edge in a tug of war of about equal strength, and
             * the flock parks on the glass for as long as it lasts. */
            double least = screen.width + screen.height;
            for (int f = 0; f < flocks; f++) {
                assert(flock_home_x[f] >= 0 && flock_home_x[f] <= screen.width);
                assert(flock_home_y[f] >= 0 && flock_home_y[f] <= screen.height);
                /* And home is a shove away from the flock, not a throw: four
                 * flocks all pushing the middle one used to send its home four
                 * rooms clear of where the flock actually was. */
                double shove_x = flock_home_x[f] - flock_center_x[f];
                double shove_y = flock_home_y[f] - flock_center_y[f];
                assert(sqrt(shove_x * shove_x + shove_y * shove_y) <= flock_room() + 1e-9);
                for (int g = f + 1; g < flocks; g++) {
                    double dx = flock_center_x[f] - flock_center_x[g];
                    double dy = flock_center_y[f] - flock_center_y[g];
                    double distance = sqrt(dx * dx + dy * dy);
                    if (distance < least) least = distance;
                }
            }
            gap_sum += least;
            measured++;
        }
        spatial_grid_destroy(&grid);
        assert(measured > 0);
        /* The two closest flocks, averaged over the run, keep a leash between
         * them: they cross and touch, they do not settle on top of each other. */
        assert(gap_sum / measured > FLOCK_LEASH * 0.8);

        /* And a bird's neighbours are its own kind: mixed evenly, more than half
         * of them would be strangers with five flocks up. */
        measure_flocks(birds);
        long foreign = 0, counted = 0;
        for (int i = 0; i < BIRD_COUNT; i += 7) {
            int nearest = -1;
            double best = 0;
            for (int j = 0; j < BIRD_COUNT; j++) {
                if (j == i) continue;
                double dx = birds[i].x - birds[j].x, dy = birds[i].y - birds[j].y;
                double distance = dx * dx + dy * dy;
                if (nearest < 0 || distance < best) {
                    best = distance;
                    nearest = j;
                }
            }
            counted++;
            if (birds[nearest].flock != birds[i].flock) foreign++;
        }
        assert(counted > 0);
        assert(foreign * 4 < counted); /* Under a quarter, against 1 - 1/flocks. */
    }

    /* One flock has nothing to keep away from, and pays nothing for the rule. */
    config.flocks = 1;
    for (int i = 0; i < BIRD_COUNT; i++) birds[i].flock = 0;
    measure_flocks(birds);
    for (int f = 0; f < MAX_FLOCKS; f++) assert(flock_center_x[f] == 0 && flock_center_y[f] == 0);
    assert(leash_vector(&birds[0]).x == 0 && leash_vector(&birds[0]).y == 0);
    legend_enabled = 1;
    reset_test_config();
}

static void test_flocks_do_not_align_with_each_other(void) {
    enum { BIRD_COUNT = 40 };
    bird_t birds[BIRD_COUNT];
    spatial_grid_t grid;

    reset_test_config();
    legend_enabled = 0;
    apply_screen_size(200, 50, 200 * 8, 50 * 16);
    config.birds = BIRD_COUNT;
    config.flocks = 2;

    /* One bird of flock zero heading right, and a crowd of flock one heading the
     * other way. They sit exactly on top of it, so separation contributes nothing
     * and only the social terms can move it: the test is then about those alone. */
    birds[0] = (bird_t){.x = 800, .y = 400};
    for (int i = 1; i < BIRD_COUNT; i++)
        birds[i] = (bird_t){.x = 800, .y = 400, .direction = M_PI, .flock = 1};

    assert(spatial_grid_init(&grid, SPATIAL_CELL_SIZE) == SPATIAL_GRID_OK);
    assert(spatial_grid_prepare(&grid, screen.width, screen.height, BIRD_COUNT) == SPATIAL_GRID_OK);
    assert(spatial_grid_build(&grid, BIRD_COUNT, read_bird_position, birds) == SPATIAL_GRID_OK);

    /* Not its flock, so it holds its own heading however many of them there are.
     * Its own flock is one bird, so the leash pulls it nowhere it was not already
     * going: the two flocks' centres coincide, and a flock of one is always at
     * its own centre. */
    measure_flocks(birds);
    assert(flock_direction(birds, &grid, 0) == 0.0);

    /* Put them all in one flock and the same crowd turns it right around. */
    config.flocks = 1;
    for (int i = 0; i < BIRD_COUNT; i++) birds[i].flock = 0;
    measure_flocks(birds);
    assert(angle_difference(flock_direction(birds, &grid, 0), M_PI) < 1e-12);

    spatial_grid_destroy(&grid);
    legend_enabled = 1;
    reset_test_config();
}

static void test_mouse_reports_are_parsed(void) {
    reset_test_config();
    apply_screen_size(80, 24, 80 * 8, 24 * 16);
    mouse.present = 0;

    /* CSI < button ; column ; row M, one based, as mode 1006 sends it. The
     * terminal names a cell, so the position is that cell's middle. */
    assert(feed_input("\033[<35;10;5M") == 1);
    assert(mouse.present);
    assert(mouse.x == 9.5 * screen.cell_width);
    assert(mouse.y == 4.5 * screen.cell_height);

    /* A release report moves it just the same. */
    assert(feed_input("\033[<0;20;9m") == 1);
    assert(mouse.x == 19.5 * screen.cell_width);
    assert(mouse.y == 8.5 * screen.cell_height);

    /* A key arriving in the same read is still acted on. */
    config.boundary_notch = DEFAULT_NOTCH;
    apply_notches();
    assert(feed_input("\033[<35;3;3MB") == 1);
    assert(config.boundary_notch == DEFAULT_NOTCH + 1);

    /* Sequences that are not mouse reports leave it alone, and are swallowed
     * rather than read as keystrokes: an arrow key must not nudge a weight. */
    double before_x = mouse.x;
    config.boundary_notch = DEFAULT_NOTCH;
    apply_notches();
    assert(feed_input("\033[A\033[1;2B\033OP") == 1);
    assert(mouse.x == before_x);
    assert(config.boundary_notch == DEFAULT_NOTCH);

    /* A report longer than the buffer cannot run off it. */
    char huge[64];
    memset(huge, '9', sizeof(huge) - 1);
    huge[sizeof(huge) - 1] = '\0';
    char overlong[80];
    snprintf(overlong, sizeof(overlong), "\033[<%sM", huge);
    assert(feed_input(overlong) == 1);
    assert(feed_input("q") == 0);
    reset_test_config();
}

static void test_vision_controls(void) {
    reset_test_config();
    assert(config.vision_radius == DEFAULT_VISION_RADIUS);
    assert(config.vision_radius_squared == DEFAULT_VISION_RADIUS * DEFAULT_VISION_RADIUS);
    assert(config.vision_cells == 3);

    /* One notch is four pixels of radius, a third of a grid cell, which the scan
     * has to round up to reach. */
    assert(feed_input("P") == 1);
    assert(config.vision_notch == 7);
    assert(config.vision_radius == 40);
    assert(config.vision_radius_squared == 1600);
    assert(config.vision_cells == 4);

    char keys[INPUT_BUFFER_SIZE + 1];
    memset(keys, 'P', INPUT_BUFFER_SIZE);
    keys[INPUT_BUFFER_SIZE] = '\0';
    assert(feed_input(keys) == 1);
    assert(config.vision_notch == LEGEND_BAR_CELLS);
    assert(config.vision_radius == MAX_VISION_RADIUS);
    assert(config.vision_cells == MAX_VISION_CELLS); /* The five cell ceiling. */

    memset(keys, 'p', INPUT_BUFFER_SIZE);
    assert(feed_input(keys) == 1);
    assert(config.vision_notch == 0);
    assert(config.vision_radius == MIN_VISION_RADIUS);
    assert(config.vision_cells == 1);
    assert(feed_input("q") == 0);
    reset_test_config();
}

static void test_frame_rate_controls(void) {
    char keys[INPUT_BUFFER_SIZE + 1];

    reset_test_config();
    /* Roomy, or the step is capped by the screen rather than by the rate. */
    apply_screen_size(200, 60, 1600, 960);
    assert(config.frame_rate == DEFAULT_FRAME_RATE);
    memset(keys, 'R', INPUT_BUFFER_SIZE);
    keys[INPUT_BUFFER_SIZE] = '\0';
    assert(feed_input(keys) == 1);
    assert(config.rate_notch == LEGEND_BAR_CELLS);
    assert(config.frame_rate == MAX_FRAME_RATE);
    assert(config.speed == 20.0);

    memset(keys, 'r', INPUT_BUFFER_SIZE);
    assert(feed_input(keys) == 1);
    assert(config.rate_notch == 0);
    assert(config.frame_rate == MIN_FRAME_RATE);
    assert(config.speed == 80.0);

    memset(keys, 'R', INPUT_BUFFER_SIZE);
    keys[INPUT_BUFFER_SIZE - 1] = 'q';
    assert(feed_input(keys) == 0);
    assert(config.frame_rate == MAX_FRAME_RATE);

    /* Every notch of the rate is a whole number of frames per second, even
     * though a twelfth of its range is not. */
    for (int n = 0; n <= LEGEND_BAR_CELLS; n++) {
        config.rate_notch = n;
        apply_notches();
        assert(config.frame_rate >= MIN_FRAME_RATE && config.frame_rate <= MAX_FRAME_RATE);
    }
    reset_test_config();
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
    /* Screen bands only, pushing inwards and nothing like the panel's push. */
    assert(force.x > 0 && force.x <= EDGE_FIRM);
    assert(force.y > 0 && force.y <= EDGE_FIRM);

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
    test_the_edge_pushes_harder_the_further_out_a_bird_is();
    test_bottom_band_scales_on_a_short_viewport();
    test_birds_start_spread_inside_the_free_region();
    test_the_recording_rate_is_one_a_gif_has();
    test_birds_bank_rather_than_snap();
    test_the_konami_code();
    test_only_the_rain_has_a_wind();
    test_autopilot_wanders_and_yields();
    test_hawks_hunt_and_the_flock_flees();
    test_the_turn_limits_follow_the_frame_rate();
    test_more_flocks_are_more_colours();
    test_the_flock_can_be_laid_out_as_text();
    test_presets_set_every_notch();
    test_a_notch_survives_the_round_trip();
    test_the_pointer_moves_the_flock();
    test_the_shade_follows_the_heading();
    test_the_hawk_is_never_the_colour_of_the_flock();
    test_the_theme_ramp_never_reaches_the_background();
    test_theme_colours_are_parsed();
    test_each_flock_flies_at_its_own_pace();
    test_the_matrix_is_the_only_thing_that_rains();
    test_a_text_terminal_gets_the_flock_in_braille();
    test_recording_gives_the_whole_frame_to_the_flock();
    test_flocks_keep_to_their_own_side_of_the_sky();
    test_flocks_do_not_align_with_each_other();
    test_mouse_reports_are_parsed();
    test_vision_controls();
    test_frame_rate_controls();
    test_flicker_free_render_queue();
    test_legend_panel_layout();
    test_legend_values_follow_their_notch();
    test_bar_spans_the_whole_travel();
    test_one_keypress_is_one_cell();
    test_weights_stop_at_their_bounds();
    test_legend_repels_towards_the_nearer_way_out();
    test_legend_push_overrules_the_flock();
    test_no_bird_ever_reaches_the_panel();
    test_birds_start_clear_of_the_panel();
    test_frame_carries_the_panel();
    test_panel_switches_off_cleanly();
    test_no_legend_leaves_the_corner_to_the_flock();
    return 0;
}
