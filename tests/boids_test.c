#define main cbirds_application_main
#include "../boids.c"
#undef main

#include <assert.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/wait.h>

/* A directory of the run's own for the files it writes: a fixed name in /tmp
 * collides with a second run, and may already be something else, a symlink
 * included, that a truncating open would then write through. */
static char scratch[512];

static void make_scratch(void) {
    const char *base = getenv("TMPDIR");
    snprintf(scratch, sizeof(scratch), "%s/cbirds_boids_test.XXXXXX",
             base != NULL && *base != '\0' ? base : "/tmp");
    assert(mkdtemp(scratch) != NULL);
}

static void scratch_file(char *path, size_t size, const char *name) {
    int length = snprintf(path, size, "%s/%s", scratch, name);
    assert(length > 0 && (size_t)length < size);
}

/* The model, written out a second time the obvious way. It knows the three rules,
 * the edges and the leash, and it deliberately does not know the pointer, the
 * hawks or the wind: the test that uses it asserts that those are all switched
 * off, so that this stays a check of the search and not a second implementation
 * to keep in step. */
static double brute_force_flock_direction(const bird_t *birds, int target_index) {
    const bird_t *target = &birds[target_index];
    vector_t separation = {0, 0}, alignment = {0, 0}, cohesion = {0, 0}, wary = {0, 0};
    vector_t boundary = boundary_vector(target);
    vector_t leash = leash_vector(target);
    int neighbors = 0, strangers = 0;
    double kin = 0;

    for (int i = 0; i < config.birds; i++) {
        if (i == target_index) continue;
        const bird_t *other = &birds[i];
        double dx = target->x - other->x;
        double dy = target->y - other->y;
        if (dx * dx + dy * dy >= config.vision_radius_squared) continue;
        separation.x += dx;
        separation.y += dy;
        neighbors++;
        if (other->flock != target->flock) {
            if (config.avoid_kinship > 0) {
                trig_entry_t heading = trig_lookup(other->direction);
                alignment.x += config.avoid_kinship * heading.cosine;
                alignment.y += config.avoid_kinship * heading.sine;
                cohesion.x += config.avoid_kinship * other->x;
                cohesion.y += config.avoid_kinship * other->y;
                kin += config.avoid_kinship;
            }
            double distance = sqrt(dx * dx + dy * dy);
            if (config.avoid_weight > 0 && distance > 1e-9) {
                wary.x += (1 - distance / config.vision_radius) * dx / distance;
                wary.y += (1 - distance / config.vision_radius) * dy / distance;
                strangers++;
            }
            continue;
        }
        trig_entry_t heading = trig_lookup(other->direction);
        alignment.x += heading.cosine;
        alignment.y += heading.sine;
        cohesion.x += other->x;
        cohesion.y += other->y;
        kin += 1;
    }

    if (neighbors) {
        if (kin) {
            alignment.x /= kin;
            alignment.y /= kin;
            cohesion.x = cohesion.x / kin - target->x;
            cohesion.y = cohesion.y / kin - target->y;
        }
        if (strangers) {
            wary.x /= strangers;
            wary.y /= strangers;
        }
        double x = separation.x * config.separation + alignment.x * config.alignment +
                   cohesion.x * COHESION_W + boundary.x * config.boundary + leash.x * LEASH_WEIGHT +
                   wary.x * config.avoid_weight;
        double y = separation.y * config.separation + alignment.y * config.alignment +
                   cohesion.y * COHESION_W + boundary.y * config.boundary + leash.y * LEASH_WEIGHT +
                   wary.y * config.avoid_weight;
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

/* The speed slider's two ends, written out here rather than derived, so that a
 * change to how the program counts its fifths has to agree with them. */
static const double PACE_FLOOR = 0.2, PACE_CEILING = 2.6;

/* Nobody is alarmed to begin with, and no pointer has ever been seen. */
static void reset_the_waves(void) {
    memset(waves, 0, sizeof(waves));
    waves_in_flight = 0;
    wave_task_count = 0;
    memset(&mouse, 0, sizeof(mouse));
}

static void reset_test_config(void) {
    frame_seconds = 1.0 / FRAME_RATE;
    config.birds = 800;
    config.bird_size = DEFAULT_BIRD_SIZE;
    config.boundary_notch = DEFAULT_NOTCH;
    config.separation_notch = DEFAULT_NOTCH;
    config.alignment_notch = DEFAULT_NOTCH;
    config.vision_notch = 6;
    config.pace_notch = DEFAULT_NOTCH;
    config.avoid_notch = DEFAULT_NOTCH;
    config.palette = 0;
    config.turning_notch = DEFAULT_TURNING_NOTCH;
    config.flocks = 1;
    config.trails = 0;
    config.hawks = 0;
    matrix_mode = 0;
    unlock_fps = 0;
    fireflies_mode = 0;
    fireflies_destroy(&night);
    letters_mode = 0;
    /* The space is the one thing a test may leave running, and the flat flock must
     * not find it there. */
    end_the_sky();
    sky_mode = 0;
    sky_picture_size = 0;
    ink_is_known = 0;
    apply_notches();
    reset_the_waves();
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

static void test_the_trig_lookup_covers_the_circle(void) {
    const double delta = 2 * M_PI / TRIG_LOOKUP_SIZE;
    const double tolerance = delta / 2 + 1e-6; /* Float error is far below one bin. */

    trig_entry_t zero = trig_lookup(0);
    trig_entry_t quarter = trig_lookup(M_PI / 2);
    trig_entry_t half = trig_lookup(M_PI);
    trig_entry_t three_quarters = trig_lookup(3 * M_PI / 2);
    assert(fabs(zero.cosine - 1) < 1e-6 && fabs(zero.sine) < 1e-6);
    assert(fabs(quarter.cosine) < 1e-6 && fabs(quarter.sine - 1) < 1e-6);
    assert(fabs(half.cosine + 1) < 1e-6 && fabs(half.sine) < 1e-6);
    assert(fabs(three_quarters.cosine) < 1e-6 && fabs(three_quarters.sine + 1) < 1e-6);

    /* The mask wraps in either direction, including the exact seam. */
    trig_entry_t full_turn = trig_lookup(2 * M_PI);
    assert(full_turn.cosine == zero.cosine && full_turn.sine == zero.sine);
    trig_entry_t before_zero = trig_lookup(-delta);
    trig_entry_t before_full_turn = trig_lookup(2 * M_PI - delta);
    assert(before_zero.cosine == before_full_turn.cosine);
    assert(before_zero.sine == before_full_turn.sine);

    /* Sample between table entries as well as on them, over negative and
     * positive turns. Every result must be within half a bin of the exact angle. */
    for (int i = -TRIG_LOOKUP_SIZE * 2; i <= TRIG_LOOKUP_SIZE * 2; i++) {
        double angle = (double)i * delta / 7;
        trig_entry_t got = trig_lookup(angle);
        double got_angle = atan2(got.sine, got.cosine);
        assert(angle_difference(got_angle, angle) <= tolerance);
    }
}

static void test_the_frame_rate_can_be_unlocked(void) {
    char error[128];
    char *argv[] = {"cbirds", "--unlock-fps", NULL};

    reset_test_config();
    long budget = 1000000L / FRAME_RATE;
    assert(frame_delay_after(1000) == budget - 1000);
    assert(frame_delay_after(budget) == 0);
    assert(options_parse(OPTIONS, OPTION_COUNT, 2, argv, error, sizeof(error)) == OPTIONS_OK);
    assert(unlock_fps);
    assert(frame_delay_after(0) == 0);
    assert(frame_delay_after(1000) == 0);
    reset_test_config();
}

static uint32_t test_random(uint32_t *state) {
    *state = *state * 1103515245u + 12345u;
    return *state;
}

static void initialize_test_birds(bird_t *birds, int count) {
    uint32_t state = 0x93d765b1u;
    for (int i = 0; i < count; i++) {
        /* Whole, not just the fields this test sets: the engine reads the layer,
         * the wings and the rest, and a stack array holds whatever was there. */
        birds[i] = (bird_t){0};
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
    assert(!mouse.present);
    assert(!the_rain_is_falling);

    /* Swept over the flock count too: the reference models the same social rule,
     * so a divergence would mean one of the two forgot it. */
    for (config.flocks = 1; config.flocks <= MAX_FLOCKS; config.flocks++) {
        for (int i = 0; i < BIRD_COUNT; i++) snapshot[i].flock = i % config.flocks;
        /* And over how much they avoid each other, which moves the homes as
         * well as the wariness, so the whereabouts are measured at each. */
        static const int AVOIDANCE[] = {0, 2, DEFAULT_NOTCH, 8, LEGEND_BAR_CELLS};
        for (size_t a = 0; a < sizeof(AVOIDANCE) / sizeof(*AVOIDANCE); a++) {
            config.avoid_notch = AVOIDANCE[a];
            apply_notches();
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
    }
    config.avoid_notch = DEFAULT_NOTCH;
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
    seed_random(20260911u);
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

/* A bird added with + starts from nothing, whatever the memory it was given held:
 * with trails on, its tail index is an array index on the very next frame. And
 * the flock is resized whole, keeping the birds already flying. */
static void test_a_grown_flock_starts_its_new_birds_clean(void) {
    enum { FEW = 8, MANY = 64 };
    reset_test_config();
    set_test_screen(900, 600);
    config.trails = 1;
    seed_random(11);

    bird_t poisoned;
    memset(&poisoned, 0xa5, sizeof(poisoned));
    place_one_bird(&poisoned, 0);
    assert(poisoned.trail_at == 0 && poisoned.trail_held == 0 && poisoned.gliding == 0);

    config.birds = FEW;
    bird_t *birds = calloc(FEW, sizeof(*birds));
    bird_t *snapshot = malloc(FEW * sizeof(*snapshot));
    assert(birds != NULL && snapshot != NULL);
    initialize_birds(birds);
    bird_t first = birds[0];

    config.birds = MANY;
    assert(resize_the_flock(&birds, &snapshot, FEW, MANY));
    assert(memcmp(&birds[0], &first, sizeof(first)) == 0);
    for (int i = FEW; i < MANY; i++)
        assert(birds[i].trail_at == 0 && birds[i].trail_held == 0 && birds[i].wing < WING_CYCLE);

    /* Flown with trails for a while, which reads and writes every tail. */
    spatial_grid_t grid;
    assert(spatial_grid_init(&grid, SPATIAL_CELL_SIZE) == SPATIAL_GRID_OK);
    assert(spatial_grid_prepare(&grid, screen.width, screen.height, MANY) == SPATIAL_GRID_OK);
    for (int frame = 0; frame < 2 * TRAIL_LENGTH; frame++) {
        memcpy(snapshot, birds, MANY * sizeof(*birds));
        assert(spatial_grid_build(&grid, MANY, read_bird_position, snapshot) == SPATIAL_GRID_OK);
        update_birds(birds, snapshot, &grid);
    }
    for (int i = 0; i < MANY; i += TRAIL_EVERY)
        assert(birds[i].trail_held == TRAIL_LENGTH && birds[i].trail_at < TRAIL_LENGTH);

    /* And shrunk, the survivors are the first ones, untouched. */
    first = birds[0];
    config.birds = FEW / 2;
    assert(resize_the_flock(&birds, &snapshot, MANY, FEW / 2));
    assert(memcmp(&birds[0], &first, sizeof(first)) == 0);

    spatial_grid_destroy(&grid);
    free(snapshot);
    free(birds);
    reset_test_config();
}

/* However the program ends, the terminal is put back, and a reader that goes
 * away, as head does after its first bytes, is one of the ways: SIGPIPE's default
 * used to kill it there and leave the shell raw, with no echo. The whole program
 * runs in a child on a terminal of its own, writing into a pipe that is closed
 * under it. */
static void test_a_closed_pipe_leaves_the_terminal_as_it_was(void) {
    int master = posix_openpt(O_RDWR | O_NOCTTY);
    assert(master >= 0 && grantpt(master) == 0 && unlockpt(master) == 0);
    const char *name = ptsname(master);
    assert(name != NULL);
    int terminal = open(name, O_RDWR | O_NOCTTY);
    assert(terminal >= 0);
    struct termios before, after;
    assert(tcgetattr(terminal, &before) == 0);
    int output[2];
    assert(pipe(output) == 0);
    fflush(NULL);

    pid_t child = fork();
    assert(child >= 0);
    if (child == 0) {
        alarm(20); /* A hang fails the test instead of stalling the suite. */
        int quiet = open("/dev/null", O_WRONLY);
        if (quiet < 0 || dup2(terminal, STDIN_FILENO) < 0 || dup2(output[1], STDOUT_FILENO) < 0 ||
            dup2(quiet, STDERR_FILENO) < 0)
            _exit(99);
        close(output[0]);
        /* As a fresh process has them: earlier tests leave state behind. */
        terminal_is_raw = terminal_restored = alt_screen_is_on = sprites_uploaded = 0;
        char *argv[] = {"cbirds", "--render", "braille", "-n", "50", NULL};
        exit(cbirds_application_main(5, argv));
    }
    close(output[1]);
    char byte;
    assert(read(output[0], &byte, 1) == 1);
    close(output[0]);
    int status = 0;
    assert(waitpid(child, &status, 0) == child);
    /* An error it reports and exits on, not a signal it dies of. */
    assert(WIFEXITED(status) && WEXITSTATUS(status) == EXIT_FAILURE);
    assert(tcgetattr(terminal, &after) == 0);
    assert(after.c_iflag == before.c_iflag && after.c_oflag == before.c_oflag &&
           after.c_cflag == before.c_cflag && after.c_lflag == before.c_lflag);
    assert(memcmp(after.c_cc, before.c_cc, sizeof(before.c_cc)) == 0);
    close(terminal);
    close(master);
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
    char lines[LEGEND_MAX_ROWS][LEGEND_LINE_MAX];

    reset_test_config();
    /* The panel waits for a window it is a fifth of rather than half of: at the
     * smallest terminal it used to appear in, it covered 54% of the area and the
     * force that keeps birds out of it squeezed half the flock off the edges of
     * what was left. One column narrower and the flock was fine; one column wider
     * and it was not. */
    assert(LEGEND_COLUMNS * LEGEND_MAX_ROWS * 4 <= LEGEND_MIN_COLS * LEGEND_MIN_ROWS);
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
    config.pace_notch = LEGEND_BAR_CELLS;
    apply_preset_defaults();
    assert(config.turning_notch == DEFAULT_TURNING_NOTCH);
    assert(config.boundary_notch == DEFAULT_NOTCH);
    assert(config.pace_notch == DEFAULT_PACE_NOTCH && config.pace == DEFAULT_PACE);

    /* One slider a parameter, named, with a bar and its pair of keys. */
    static const char *names[] = {"boundary", "separation", "alignment",
                                  "turning",  "perception", "speed"};
    static const char *pairs[] = {"b/B", "s/S", "a/A", "t/T", "p/P", "v/V"};
    for (int i = 0; i < 6; i++) {
        assert(strstr(lines[1 + i], names[i]) != NULL);
        /* Lowercase first: the key that lowers, then the one that raises. */
        assert(strstr(lines[1 + i], pairs[i]) != NULL);
        assert(strstr(lines[1 + i], "\u2591") != NULL); /* Some empty track shows. */
    }
    assert(strstr(lines[LEGEND_ROWS - 2], "quit") != NULL);

    /* The inverted pair is the whole point, so the old order must be absent. */
    static const char *reversed[] = {"B/b", "S/s", "A/a", "T/t", "P/p", "R/r", "V/v"};
    for (int row = 0; row < LEGEND_ROWS; row++)
        for (int i = 0; i < 7; i++) assert(strstr(lines[row], reversed[i]) == NULL);

    /* Each slider states its value beside its bar, right aligned in a column of
     * its own so the numbers stack. */
    static const char *values[] = {"0.20", "0.005", "1.50", "70\u00b0", "36px", "1.0\u00d7"};
    for (int i = 0; i < 6; i++) assert(strstr(lines[1 + i], values[i]) != NULL);
    assert(strstr(lines[LEGEND_ROWS - 3], "60fps") != NULL);
}

static void test_legend_values_follow_their_notch(void) {
    char lines[LEGEND_MAX_ROWS][LEGEND_LINE_MAX];
    static const char *floors[] = {"0.01", "0.001", "0.10", "30\u00b0", "12px", "0.2\u00d7"};
    static const char *ceilings[] = {"0.58", "0.013", "4.30", "360\u00b0", "60px", "2.6\u00d7"};

    reset_test_config();
    apply_screen_size(80, 24, 80 * 8, 24 * 16);

    config.boundary_notch = config.separation_notch = config.alignment_notch =
        config.turning_notch = config.vision_notch = config.pace_notch = 0;
    apply_notches();
    build_legend(lines);
    for (int i = 0; i < 6; i++) {
        assert(strstr(lines[1 + i], floors[i]) != NULL);
        assert(filled_cells(lines[1 + i]) == 0);
    }

    config.boundary_notch = config.separation_notch = config.alignment_notch =
        config.turning_notch = config.vision_notch = config.pace_notch = LEGEND_BAR_CELLS;
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
    char lines[LEGEND_MAX_ROWS][LEGEND_LINE_MAX];
    static const struct {
        int row;
        char raise, lower;
    } sliders[] = {{1, 'B', 'b'}, {2, 'S', 's'}, {3, 'A', 'a'},
                   {4, 'T', 't'}, {5, 'P', 'p'}, {6, 'V', 'v'}};

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
            config.vision_notch = config.pace_notch = n;
        apply_notches();
        /* The speed moves in fifths, which is what lets the panel print it whole. */
        assert(fabs(config.pace - (0.2 + 0.2 * n)) < 1e-12);
        if (n == 0) {
            assert(config.pace == PACE_FLOOR);
            assert(config.boundary == BOUNDARY_MIN);
            assert(config.separation == SEPARATION_MIN);
            assert(config.alignment == ALIGNMENT_MIN);
            assert(config.vision_radius == MIN_VISION_RADIUS);
        }
        if (n == LEGEND_BAR_CELLS) {
            assert(fabs(config.boundary - BOUNDARY_MAX) < 1e-12);
            assert(fabs(config.alignment - ALIGNMENT_MAX) < 1e-12);
            assert(config.vision_radius == MAX_VISION_RADIUS);
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
    /* The tests fly the reference flock on the fourth notch: exactly one, the
     * flock that shipped. The program itself starts slower, on DEFAULT_PACE_NOTCH. */
    assert(config.pace == 1.0);
    assert(config.boundary_notch == DEFAULT_NOTCH);
    assert(config.alignment_notch == DEFAULT_NOTCH);
    /* The integer parameter lands exactly, being an integer. */
    assert(config.vision_radius == DEFAULT_VISION_RADIUS);
}

static void test_weights_stop_at_their_bounds(void) {
    char keys[INPUT_BUFFER_SIZE + 1];
    static const struct {
        char raise, lower;
        const double *ceiling, *floor;
        const double *value;
    } weights[] = {{'B', 'b', &BOUNDARY_MAX, &BOUNDARY_MIN, &config.boundary},
                   {'S', 's', &SEPARATION_MAX, &SEPARATION_MIN, &config.separation},
                   {'A', 'a', &ALIGNMENT_MAX, &ALIGNMENT_MIN, &config.alignment},
                   {'V', 'v', &PACE_CEILING, &PACE_FLOOR, &config.pace}};

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
    /* Swept over a frame of the usual length and one that took twice as long,
     * which move the margin, and over the turning limit, which is what nearly
     * broke this: a bird that cannot turn at once cannot be turned away at once,
     * so the panel's push is exempt from the limit and this test is what says so.
     * Dropping that exemption fails it. */
    static const int TURNS[] = {0, 1, DEFAULT_TURNING_NOTCH, LEGEND_BAR_CELLS};
    static const double FRAMES_LONG[] = {1.0, 2.0};
    for (size_t turn = 0; turn < sizeof(TURNS) / sizeof(*TURNS); turn++)
        for (size_t length = 0; length < sizeof(FRAMES_LONG) / sizeof(*FRAMES_LONG); length++) {
            reset_test_config();
            config.turning_notch = TURNS[turn];
            /* Which moves the turn margin with it. */
            set_frame_seconds(FRAMES_LONG[length] / FRAME_RATE);
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

    seed_random(20260912u);
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
    assert(formation_layout("I") > 0);
    enum { BIRD_COUNT = 4 };
    bird_t birds[BIRD_COUNT], snapshot[BIRD_COUNT];
    spatial_grid_t grid;
    config.birds = BIRD_COUNT;
    for (int i = 0; i < BIRD_COUNT; i++)
        birds[i] = (bird_t){
            .x = formation.x[0] - config.speed / 2, .y = formation.y[0], .direction = M_PI};
    assert(spatial_grid_init(&grid, SPATIAL_CELL_SIZE) == SPATIAL_GRID_OK);
    assert(spatial_grid_prepare(&grid, screen.width, screen.height, BIRD_COUNT) == SPATIAL_GRID_OK);
    memcpy(snapshot, birds, sizeof(birds));
    assert(spatial_grid_build(&grid, BIRD_COUNT, read_bird_position, snapshot) == SPATIAL_GRID_OK);
    update_birds(birds, snapshot, &grid);
    assert(fabs(birds[0].x - formation.x[0]) < 1e-9); /* Landed, despite the limit. */
    spatial_grid_destroy(&grid);

    formation_clear();
    reset_test_config();
}

static void test_the_konami_code(void) {
    reset_test_config();
    legend_enabled = 1;
    apply_screen_size(200, 50, 1600, 800);
    config.hawks = 0;
    konami_at = 0;
    formation_clear();

    /* Nine of the ten is nine of the ten. */
    for (const char *c = "AABBDCDCb"; *c; c++) konami_note(*c);
    assert(config.hawks == 0);
    konami_note('a');
    assert(config.hawks == MAX_HAWKS);

    /* A wrong key in the middle is a wrong sequence. */
    config.hawks = 0;
    formation_clear();
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
    formation_clear();
    for (const char *c = "qwertyAABBDCDCba"; *c; c++) konami_note(*c);
    assert(config.hawks == MAX_HAWKS);

    config.hawks = 0;
    konami_at = 0;
    formation_clear();
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
    last_key_at = 0;
    clock_state.seconds = 0;

    /* Left alone it does nothing, because nothing has idled yet. */
    assert(!flying_itself());
    /* And after the idle time it takes over. */
    clock_state.seconds = IDLE_SECONDS + 1;
    assert(flying_itself());
    clock_state.seconds = 0;
    assert(!flying_itself());

    /* A drift moves exactly one notch, and stays inside the bar. */
    seed_random(7);
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

    /* A keypress takes the sliders back: it keeps its hands off until the user
     * has been gone a whole minute again. */
    clock_state.seconds = 100;
    last_key_at = 100;
    last_drift_at = 0;
    int held = config.boundary_notch;
    maybe_drift();
    assert(config.boundary_notch == held);
    clock_state.seconds = 100 + IDLE_SECONDS;
    maybe_drift();
    assert(last_drift_at == clock_state.seconds); /* And then it does move one. */
    /* And no more often than the period, once it is flying itself. */
    clock_state.seconds += AUTOPILOT_PERIOD - 1;
    maybe_drift();
    assert(last_drift_at == 100 + IDLE_SECONDS);
    clock_state.seconds += 1;
    maybe_drift();
    assert(last_drift_at == clock_state.seconds);

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
    hawks[0].commitment = (double)HAWK_COMMITMENT_FRAMES / FRAME_RATE;
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
    hawks[0].commitment = 0.5;
    hawks[0].passing = 0.5;
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

/* Speed and both turn limits are per-second quantities dressed as a step. The
 * step follows elapsed time, not how many times the loop happened to run. */
static void test_motion_follows_elapsed_time(void) {
    reset_test_config();
    /* Roomy, so that the step is set by elapsed time alone: on a small screen
     * it is capped by the screen instead, which is the next thing asserted. */
    apply_screen_size(200, 60, 1600, 960);
    config.turning_notch = 6;

    set_frame_seconds(1.0 / FRAME_RATE);
    double bird_at_sixty = turn_limit(), hawk_at_sixty = hawk_turn_limit();
    double step_at_sixty = config.speed;

    set_frame_seconds(2.0 / FRAME_RATE);
    assert(fabs(config.speed - step_at_sixty * 2) < 1e-9);      /* Twice the ground. */
    assert(fabs(turn_limit() - bird_at_sixty * 2) < 1e-9);      /* Twice the turn. */
    assert(fabs(hawk_turn_limit() - hawk_at_sixty * 2) < 1e-9); /* Both of them. */
    assert(fabs(config.speed * FRAME_RATE / 2 - step_at_sixty * FRAME_RATE) < 1e-9);

    set_frame_seconds(0.5 / FRAME_RATE);
    assert(fabs(config.speed - step_at_sixty / 2) < 1e-9);
    assert(fabs(turn_limit() - bird_at_sixty / 2) < 1e-9);
    assert(fabs(hawk_turn_limit() - hawk_at_sixty / 2) < 1e-9);
    assert(fabs(config.speed * FRAME_RATE * 2 - step_at_sixty * FRAME_RATE) < 1e-9);

    /* Two equal monotonic timestamps are legal: that frame advances nothing and
     * must not leave the hawk geometry dividing zero by zero. */
    set_frame_seconds(0);
    assert(config.speed == 0);
    assert(turn_limit() == 0);
    assert(hawk_turning_radius() == 0);

    /* A small screen lowers the velocity, but a late frame still covers its full
     * share of that velocity instead of slowing the flock down. */
    apply_screen_size(40, 14, 320, 224);
    set_frame_seconds(1.0 / FRAME_RATE);
    double small_step_at_sixty = config.speed;
    assert(small_step_at_sixty <= screen.height / 10.0 + 1e-9);
    set_frame_seconds(2.0 / FRAME_RATE);
    assert(fabs(config.speed - small_step_at_sixty * 2) < 1e-9);
    assert(fabs(config.speed * FRAME_RATE / 2 - small_step_at_sixty * FRAME_RATE) < 1e-9);

    /* A screen with room in it is not capped at all. */
    apply_screen_size(200, 60, 1600, 960);
    assert(config.speed > screen.height / 20.0);

    /* Instant stays instant, and nothing ever exceeds a half turn a frame. */
    config.turning_notch = LEGEND_BAR_CELLS;
    set_frame_seconds(2.0 / FRAME_RATE);
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
    int made = formation_layout("HELLO");
    assert(made > 0 && made <= font_text_cells("HELLO"));
    assert(formation.writing);
    for (int i = 0; i < made; i++) {
        assert(!legend_turn_zone(formation.x[i], formation.y[i]));
        assert(formation.x[i] >= 0 && formation.x[i] <= screen.width);
        assert(formation.y[i] >= 0 && formation.y[i] <= screen.height);
    }

    /* Birds share targets round robin, so a cell with several on it reads as a
     * thick stroke rather than leaving the rest of the flock idle. */
    double first_x, first_y, wrapped_x, wrapped_y;
    assert(formation_target_of(0, &first_x, &first_y));
    assert(formation_target_of(made, &wrapped_x, &wrapped_y));
    assert(first_x == wrapped_x && first_y == wrapped_y);

    /* A bird with a target steers at it and ignores its neighbours entirely. */
    enum { BIRD_COUNT = 8 };
    bird_t birds[BIRD_COUNT];
    spatial_grid_t grid;
    config.birds = BIRD_COUNT;
    for (int i = 0; i < BIRD_COUNT; i++)
        birds[i] = (bird_t){.x = formation.x[0] - 100, .y = formation.y[0], .direction = M_PI};
    assert(spatial_grid_init(&grid, SPATIAL_CELL_SIZE) == SPATIAL_GRID_OK);
    assert(spatial_grid_prepare(&grid, screen.width, screen.height, BIRD_COUNT) == SPATIAL_GRID_OK);
    assert(spatial_grid_build(&grid, BIRD_COUNT, read_bird_position, birds) == SPATIAL_GRID_OK);
    assert(angle_difference(flock_direction(birds, &grid, 0), 0.0) < 1e-12);

    /* And it lands exactly rather than orbiting: one update from a whole speed
     * away puts it on the target, not past it. */
    bird_t snapshot[BIRD_COUNT];
    birds[0].x = formation.x[0] - config.speed / 2;
    birds[0].y = formation.y[0];
    memcpy(snapshot, birds, sizeof(birds));
    update_birds(birds, snapshot, &grid);
    assert(fabs(birds[0].x - formation.x[0]) < 1e-9);
    assert(fabs(birds[0].y - formation.y[0]) < 1e-9);
    spatial_grid_destroy(&grid);

    /* Letting go hands the flock back to the flocking rules. */
    formation_clear();
    assert(!formation.writing);
    assert(!formation_target_of(0, &first_x, &first_y));

    /* Text with nothing to draw, and text that cannot fit, both decline. */
    assert(formation_layout("") == 0);
    assert(formation_layout("\x01\x02") == 0);
    apply_screen_size(44, 15, 44 * 8, 15 * 16);
    assert(formation_layout("A VERY LONG MESSAGE INDEED THAT WILL NOT FIT AT ALL") == 0);
    assert(!formation.writing);
    reset_test_config();
}

/*
 * The sign: --say, --clock, the pointer, the keys, and --screensaver.
 */

/* Everything a sign leaves behind in the program's globals, as a fresh process
 * has it, and a screen to write on. */
static void reset_sign_state(void) {
    reset_test_config();
    memset(&the_sign, 0, sizeof(the_sign));
    formation_clear();
    formation.sign = 0;
    say_text = NULL;
    sign_words[0] = '\0';
    clock_mode = 0;
    clock_start = NULL;
    screensaver_mode = 0;
    picture_path = NULL;
    picture_colours_in_use = 0;
    palette_was_asked_for = 0;
    picture_palette.shades = 0;
    png_image_free(&picture_image);
    sprite_path = NULL;
    mouse.present = 0;
    mouse.moved_at = 0;
    clock_state.seconds = 0;
    paused = 0;
    step_once = 0;
    legend_enabled = 0;
}

static void ask_for_a_sign(const char *text) {
    say_text = text;
    sign_clean(text, sign_words, sizeof(sign_words));
    the_sign.kind = SIGN_SAY;
}

typedef struct {
    bird_t *birds, *snapshot;
    spatial_grid_t grid;
} sign_sky_t;

/* A flock on the screen as it is, from a seed, and the same steps the recorder
 * takes a frame. */
static void open_the_world(sign_sky_t *world, int birds, int seed) {
    config.birds = birds;
    world->birds = calloc((size_t)birds, sizeof(*world->birds));
    world->snapshot = malloc(sizeof(*world->snapshot) * (size_t)birds);
    assert(world->birds != NULL && world->snapshot != NULL);
    assert(spatial_grid_init(&world->grid, SPATIAL_CELL_SIZE) == SPATIAL_GRID_OK);
    assert(spatial_grid_prepare(&world->grid, screen.width, screen.height, birds) ==
           SPATIAL_GRID_OK);
    seed_random((unsigned)seed);
    initialize_birds(world->birds);
}

static void step_the_world(sign_sky_t *world, double seconds) {
    clock_state.seconds = seconds;
    sign_advance(world->birds);
    memcpy(world->snapshot, world->birds, sizeof(bird_t) * (size_t)config.birds);
    assert(spatial_grid_build(&world->grid, config.birds, read_bird_position, world->snapshot) ==
           SPATIAL_GRID_OK);
    fly(world->birds, world->snapshot, &world->grid);
}

static void close_the_world(sign_sky_t *world) {
    spatial_grid_destroy(&world->grid);
    free(world->snapshot);
    free(world->birds);
}

/* Runs from `from` to `to` seconds at `fps`, and leaves the clock at the end. */
static void fly_the_world(sign_sky_t *world, double from, double to, double fps) {
    frame_seconds = 1.0 / fps;
    update_speed();
    for (double at = from; at < to - 1e-9; at += 1.0 / fps) step_the_world(world, at);
}

static double home_distance(const sign_sky_t *world, int bird) {
    int target = formation.slot[bird];
    assert(target >= 0);
    /* The colon of a clock rises and settles, and its place goes with it. */
    return hypot(world->birds[bird].x - formation.x[target],
                 world->birds[bird].y - formation.y[target] - formation.shift_y[target]);
}

/* The intro, written out here the way it always was: one line, the largest cell
 * the free rectangle gives it, centred. If any of this moves, so does every
 * recording that opens with BOIDS. */
static int expected_intro_targets(const char *text, double *x, double *y, int most) {
    double pad = config.bird_size * 2.0;
    double left = screen.legend_width > 0 ? screen.legend_width + config.speed + pad : pad;
    double top = pad, right = screen.width - pad, bottom = screen.height - pad;
    int columns = font_text_width(text);
    double cell = (right - left) / columns;
    if ((bottom - top) / FONT_HEIGHT < cell) cell = (bottom - top) / FONT_HEIGHT;
    double origin_x = left + (right - left - columns * cell) / 2;
    double origin_y = top + (bottom - top - FONT_HEIGHT * cell) / 2;
    int count = 0, column = 0;
    for (const char *c = text; *c != '\0'; c++) {
        const char *glyph = font_glyph(*c);
        if (glyph == NULL) continue;
        for (int row = 0; row < FONT_HEIGHT; row++)
            for (int cell_x = 0; cell_x < FONT_WIDTH; cell_x++) {
                if (glyph[row * FONT_WIDTH + cell_x] != '#') continue;
                double px = origin_x + (column + cell_x + 0.5) * cell;
                double py = origin_y + (row + 0.5) * cell;
                if (legend_turn_zone(px, py) || count >= most) continue;
                x[count] = px;
                y[count] = py;
                count++;
            }
        column += FONT_ADVANCE;
    }
    return count;
}

static void test_the_intro_is_untouched_by_signs(void) {
    static double want_x[FORMATION_MAX_TARGETS], want_y[FORMATION_MAX_TARGETS];

    for (int with_the_panel = 0; with_the_panel < 2; with_the_panel++) {
        reset_sign_state();
        legend_enabled = with_the_panel;
        apply_screen_size(200, 50, 1600, 800);
        formation.sign = 1; /* Left over from a sign: the intro puts it right. */
        begin_the_intro();

        /* BOIDS, three seconds, everybody writing, and no hover: the same targets
         * at the same places that master makes. */
        assert(formation.writing && !formation.sign && !formation.keep_out);
        assert(formation.until == INTRO_SECONDS);
        int count = expected_intro_targets("BOIDS", want_x, want_y, FORMATION_MAX_TARGETS);
        assert(count > 0 && formation.count == count);
        if (!with_the_panel) assert(count == font_text_cells("BOIDS"));
        for (int i = 0; i < count; i++) {
            assert(fabs(formation.x[i] - want_x[i]) < 1e-9);
            assert(fabs(formation.y[i] - want_y[i]) < 1e-9);
        }
        /* One line, laid out as the whole free rectangle: wider than two thirds. */
        double span = formation.x[count - 1] - formation.x[0];
        assert(span > screen.width / 2.0 - 1);

        /* Round robin over every bird, with all eight hundred of them writing. */
        for (int i = 0; i < 800; i++) {
            double x, y;
            bird_t bird = {0};
            assert(formation_target_for(&bird, i, &x, &y));
            assert(x == formation.x[i % count] && y == formation.y[i % count]);
        }
    }

    /* With nothing asked for, nothing else is: the intro is what begin_the_intro
     * does, and the sign has no kind. */
    reset_sign_state();
    assert(!a_sign_is_asked_for());
    apply_screen_size(200, 50, 1600, 800);
    begin_the_intro();
    assert(formation.writing && formation.until == INTRO_SECONDS);

    /* Asked for a sign, the flock writes that instead and the intro never starts. */
    reset_sign_state();
    apply_screen_size(200, 50, 1600, 800);
    ask_for_a_sign("hello");
    begin_the_intro();
    assert(!formation.writing && formation.sign);
    reset_sign_state();
}

static void test_a_key_ends_the_intro_and_leaves_a_sign_up(void) {
    reset_sign_state();
    apply_screen_size(200, 50, 1600, 800);
    begin_the_intro();
    assert(formation.writing);
    assert(feed_input("b") == 1);
    assert(!formation.writing); /* As it always was. */

    /* A sign is for reading: the keys are for the flock. */
    reset_sign_state();
    apply_screen_size(200, 50, 1600, 800);
    sign_sky_t world;
    ask_for_a_sign("hello");
    begin_the_intro();
    open_the_world(&world, 200, 3);
    sign_advance(world.birds);
    assert(formation.writing && the_sign.up);
    assert(feed_input("bsaApPvVhh\033[A") == 1);
    assert(formation.writing && the_sign.up);
    assert(feed_input("q") == 0); /* And q is q. */
    close_the_world(&world);
    reset_sign_state();
}

static void test_a_text_is_laid_out_as_a_sign_in_lines(void) {
    reset_sign_state();
    apply_screen_size(200, 50, 1600, 800);
    sign_sky_t world;
    const char *words = "Back in five minutes, or ten if the bus is late";
    ask_for_a_sign(words);
    begin_the_intro();
    open_the_world(&world, 800, 3);
    sign_advance(world.birds);

    /* Up, as the words in capitals, every lit cell of them a target. */
    assert(the_sign.up && formation.writing && formation.sign && formation.keep_out);
    char clean[SIGN_TEXT_MAX];
    sign_clean(words, clean, sizeof(clean));
    assert(formation.count == font_text_cells(clean));

    /* Two or three lines, as large as fits, in the middle, and two thirds of the
     * width at most. */
    double least_x = 1e9, most_x = 0, least_y = 1e9, most_y = 0;
    for (int t = 0; t < formation.count; t++) {
        assert(formation.x[t] > 0 && formation.x[t] < screen.width);
        assert(formation.y[t] > 0 && formation.y[t] < screen.height);
        if (formation.x[t] < least_x) least_x = formation.x[t];
        if (formation.x[t] > most_x) most_x = formation.x[t];
        if (formation.y[t] < least_y) least_y = formation.y[t];
        if (formation.y[t] > most_y) most_y = formation.y[t];
    }
    assert(most_x - least_x <= screen.width * SIGN_WIDTH_SHARE);
    assert(((least_x + most_x) / 2) > screen.width / 2.0 - 60 &&
           ((least_x + most_x) / 2) < screen.width / 2.0 + 60);
    assert(((least_y + most_y) / 2) > screen.height / 2.0 - 60 &&
           ((least_y + most_y) / 2) < screen.height / 2.0 + 60);
    /* Lines: the rows the targets fall in, a cell apart, with the gap between. */
    int rows = (int)((most_y - least_y) / formation.cell + 0.5) + 1;
    assert(rows == sign_rows(2) || rows == sign_rows(3));
    int lines = rows == sign_rows(2) ? 2 : 3;
    int blank_rows = 0;
    for (int row = 0; row < rows; row++) {
        int lit = 0;
        for (int t = 0; t < formation.count; t++)
            if ((int)((formation.y[t] - least_y) / formation.cell + 0.5) == row) lit = 1;
        blank_rows += !lit;
    }
    assert(blank_rows == (lines - 1) * SIGN_LINE_GAP);

    /* A few birds a cell, and a flock left to fly: the writers are a share of it,
     * each target has one or more, and a bird of the far sky never writes. */
    int writers = 0;
    int per_target[FORMATION_MAX_TARGETS] = {0};
    for (int i = 0; i < config.birds; i++) {
        if (formation.slot[i] < 0) continue;
        writers++;
        per_target[formation.slot[i]]++;
    }
    assert(writers > 0 && writers <= config.birds * SIGN_WRITER_SHARE + formation.count);
    assert(writers < config.birds);
    for (int t = 0; t < formation.count; t++)
        assert(per_target[t] >= 1 && per_target[t] <= SIGN_PER_CELL_MAX);

    /* The panel's corner is kept free, as the intro keeps it. */
    legend_enabled = 1;
    apply_screen_size(200, 50, 1600, 800);
    clock_state.seconds = 1;
    sign_advance(world.birds); /* The panel changes the screen, and the sign follows. */
    assert(the_sign.up && formation.count == font_text_cells(clean));
    for (int t = 0; t < formation.count; t++) {
        assert(!legend_turn_zone(formation.x[t], formation.y[t]));
        assert(formation.x[t] > screen.legend_width || formation.y[t] > screen.legend_height);
    }
    close_the_world(&world);

    /* On an eighty column terminal the panel is half the width, and what is beside
     * it is a sliver: the sign takes the room under it instead, and is the larger
     * for it, and is still not in the panel's corner. */
    reset_sign_state();
    legend_enabled = 1;
    apply_screen_size(80, 24, 640, 384);
    assert(screen.legend_width > 0);
    ask_for_a_sign("HI THERE");
    begin_the_intro();
    open_the_world(&world, 400, 3);
    sign_advance(world.birds);
    assert(the_sign.up);
    for (int t = 0; t < formation.count; t++) {
        assert(!legend_turn_zone(formation.x[t], formation.y[t]));
        assert(formation.y[t] > screen.legend_height);
    }
    double cell_with_the_panel = formation.cell;
    legend_enabled = 0;
    apply_screen_size(80, 24, 640, 384);
    clock_state.seconds = 1;
    sign_advance(world.birds);
    assert(the_sign.up && formation.cell > cell_with_the_panel); /* More room, larger. */
    close_the_world(&world);
    reset_sign_state();
}

static void test_a_sign_that_does_not_fit_says_so_and_the_flock_flies(void) {
    reset_sign_state();
    sign_sky_t world;
    ask_for_a_sign("a message far too long for a screen this small to hold at all, surely");
    apply_screen_size(44, 15, 44 * 8, 15 * 16);
    begin_the_intro();
    open_the_world(&world, 100, 3);
    sign_advance(world.birds);
    assert(!the_sign.up && !formation.writing);
    assert(!formation_target_of(0, &(double){0}, &(double){0}));
    /* It tries again a second on, when the window may have grown. */
    assert(the_sign.until > clock_state.seconds);
    apply_screen_size(200, 50, 1600, 800);
    config.birds = 100;
    for (double at = 0.0; at < 3.0; at += 0.25) {
        clock_state.seconds = at;
        sign_advance(world.birds);
    }
    close_the_world(&world);

    /* The flock too small for the text: fewer birds than lit cells. */
    reset_sign_state();
    apply_screen_size(200, 50, 1600, 800);
    ask_for_a_sign("hello world");
    begin_the_intro();
    open_the_world(&world, 40, 3);
    sign_advance(world.birds);
    assert(!the_sign.up && !formation.writing);
    close_the_world(&world);
    reset_sign_state();
}

static void test_a_hovering_bird_stays_within_its_loop_and_does_not_stand_still(void) {
    reset_sign_state();
    apply_screen_size(200, 50, 1600, 800);
    sign_sky_t world;
    ask_for_a_sign("HI");
    begin_the_intro();
    open_the_world(&world, 300, 4);

    /* Three seconds to land, at sixty frames a second. */
    fly_the_world(&world, 0, 3, 60);
    assert(the_sign.up);
    int writers = 0;
    for (int i = 0; i < config.birds; i++) writers += formation.slot[i] >= 0;
    assert(writers > 0);

    double most = 0;
    double travelled[300] = {0}, last_x[300], last_y[300];
    for (int i = 0; i < config.birds; i++) {
        last_x[i] = world.birds[i].x;
        last_y[i] = world.birds[i].y;
    }
    for (double at = 3; at < 8; at += 1.0 / 60) {
        step_the_world(&world, at);
        for (int i = 0; i < config.birds; i++) {
            if (formation.slot[i] < 0) continue;
            double away = home_distance(&world, i);
            assert(away <= formation.hover + 1e-6); /* Within its loop, every frame. */
            if (away > most) most = away;
            travelled[i] += hypot(world.birds[i].x - last_x[i], world.birds[i].y - last_y[i]);
            last_x[i] = world.birds[i].x;
            last_y[i] = world.birds[i].y;
        }
    }
    /* A few pixels across, and used: some bird goes most of the way out. */
    assert(formation.hover >= SIGN_HOVER_MIN && formation.hover <= SIGN_HOVER_MAX);
    assert(most > 0.8 * SIGN_HOVER_MIN);
    /* None of them stands still: in five seconds each has flown a loop or more. */
    int moving = 0;
    for (int i = 0; i < config.birds; i++)
        if (formation.slot[i] >= 0 && travelled[i] > 2 * M_PI * 0.3 * formation.hover) moving++;
    assert(moving == writers);

    /* The rest of the flock keeps out of the text: at no moment more than a few in
     * a hundred of the free birds are inside its box. */
    int inside = 0, free_birds = 0;
    for (int i = 0; i < config.birds; i++) {
        if (formation.slot[i] >= 0) continue;
        free_birds++;
        inside += world.birds[i].x > formation.box.left && world.birds[i].x < formation.box.right &&
                  world.birds[i].y > formation.box.top && world.birds[i].y < formation.box.bottom;
    }
    assert(free_birds > 0 && inside * 20 <= free_birds);
    close_the_world(&world);
    reset_sign_state();
}

static void test_a_sign_draws_no_random_numbers_and_the_flock_keeps_its_own(void) {
    reset_sign_state();
    apply_screen_size(200, 50, 1600, 800);
    sign_sky_t world;
    ask_for_a_sign("HI THERE");
    begin_the_intro();
    open_the_world(&world, 100, 4);
    /* The controller and the hover are the same at any frame rate and any seed:
     * nothing in them reads the flock's own generator. */
    uint32_t words[RANDOM_WORDS];
    memcpy(words, random_state.word, sizeof(words));
    int front = random_state.front, rear = random_state.rear;
    for (double at = 0; at < 100; at += 0.5) {
        clock_state.seconds = at;
        sign_advance(world.birds);
    }
    assert(memcmp(words, random_state.word, sizeof(words)) == 0);
    assert(front == random_state.front && rear == random_state.rear);
    close_the_world(&world);
    reset_sign_state();
}

static void test_the_sign_comes_back_after_its_flight(void) {
    reset_sign_state();
    apply_screen_size(200, 50, 1600, 800);
    sign_sky_t world;
    ask_for_a_sign("HELLO");
    begin_the_intro();
    open_the_world(&world, 300, 5);

    /* Held for as long as the rhythm says, from the first frame. */
    double hold = sign_hold_seconds(0), flight = sign_flight_seconds(0);
    int first_count = 0;
    double let_go_at = -1, written_again_at = -1;
    double fps = 25;
    frame_seconds = 1.0 / fps;
    update_speed();
    for (int frame = 0; frame < (int)((hold + flight + 6) * fps); frame++) {
        double at = frame / fps;
        step_the_world(&world, at);
        if (frame == 0) first_count = formation.count;
        if (let_go_at < 0 && !formation.writing) let_go_at = at;
        if (let_go_at >= 0 && written_again_at < 0 && formation.writing) written_again_at = at;
        if (at < hold - 1.0 / fps) assert(formation.writing && the_sign.up);
    }
    assert(first_count == font_text_cells("HELLO"));
    /* It let go when it was due, flew for as long as it was due to, and wrote
     * again, the same text in the same places. */
    assert(fabs(let_go_at - hold) <= 1.0 / fps + 1e-9);
    assert(fabs(written_again_at - (let_go_at + flight)) <= 1.0 / fps + 1e-9);
    assert(formation.writing && formation.count == first_count);
    assert(the_sign.cycle == 1);

    /* And the birds came home: by the end of the six seconds after it wrote, every
     * writer is in its own loop. */
    int home = 0, writers = 0;
    for (int i = 0; i < config.birds; i++) {
        if (formation.slot[i] < 0) continue;
        writers++;
        home += home_distance(&world, i) <= formation.hover + 1e-6;
    }
    assert(writers > 0 && home == writers);

    /* While it flew, the flock was free: no targets, no keep out, no scatter. */
    close_the_world(&world);
    reset_sign_state();
}

static void test_a_pause_holds_a_sign_too(void) {
    reset_sign_state();
    apply_screen_size(200, 50, 1600, 800);
    sign_sky_t world;
    ask_for_a_sign("HI");
    begin_the_intro();
    open_the_world(&world, 100, 5);
    sign_advance(world.birds);
    double until = the_sign.until;
    paused = 1;
    for (double at = 0.5; at <= 100; at += 0.5) {
        clock_state.seconds = at;
        sign_advance(world.birds);
        assert(formation.writing); /* Never let go while paused. */
    }
    /* A long pause, and the hold has not run out in the meantime. */
    assert(the_sign.until > until + 99);
    paused = 0;
    clock_state.seconds = 100.5;
    sign_advance(world.birds);
    assert(formation.writing);
    close_the_world(&world);
    reset_sign_state();
}

/* A local time that the clock can be set to, whatever the zone this runs in. */
static time_t local_time(int hour, int minute, int second) {
    struct tm when;
    memset(&when, 0, sizeof(when));
    when.tm_year = 126;
    when.tm_mon = 9;
    when.tm_mday = 8;
    when.tm_hour = hour;
    when.tm_min = minute;
    when.tm_sec = second;
    when.tm_isdst = -1;
    time_t made = mktime(&when);
    assert(made != (time_t)-1);
    return made;
}

/* Every lit cell of the sign has as many birds writing it as every other, and all
 * of them are in their loops: what a sign is, whichever way it came to be. */
static void assert_every_cell_has_its_writers(const sign_sky_t *world) {
    static int writers[FORMATION_MAX_TARGETS];
    for (int t = 0; t < formation.count; t++) writers[t] = 0;
    for (int i = 0; i < config.birds; i++) {
        int t = formation.slot[i];
        if (t < 0) continue;
        assert(t < formation.count);
        writers[t]++;
        assert(home_distance(world, i) <= formation.hover + 1e-6);
    }
    for (int t = 0; t < formation.count; t++) assert(writers[t] == the_sign.per_cell);
}

static void test_the_clock_tells_the_time_and_changes_it_a_letter_at_a_time(void) {
    reset_sign_state();
    apply_screen_size(200, 50, 1600, 800);
    sign_sky_t world;
    the_sign.kind = SIGN_CLOCK;
    the_sign.virtual_clock = 1;
    the_sign.origin = local_time(10, 9, 50);
    begin_the_intro();
    open_the_world(&world, 300, 6);

    sign_advance(world.birds);
    assert(the_sign.up && strcmp(the_sign.written, "10:09") == 0);
    assert(formation.count == font_text_cells("10:09"));
    double cell = formation.cell;

    /* It holds the minute it wrote, and the moment the next one begins it has
     * written that: the letters that changed are other birds' work from then, and
     * the clock is never down. */
    for (double at = 0.1; at < 9.99; at += 0.1) {
        clock_state.seconds = at;
        sign_advance(world.birds);
        assert(the_sign.up && strcmp(the_sign.written, "10:09") == 0);
    }
    clock_state.seconds = 10.0;
    sign_advance(world.birds);
    assert(the_sign.up && formation.writing && strcmp(the_sign.written, "10:10") == 0);
    assert(formation.count == font_text_cells("10:10"));
    assert(formation.cell == cell);
    /* And it does not change again for the rest of the minute. */
    for (double at = 10.5; at < 69.9; at += 0.5) {
        clock_state.seconds = at;
        sign_advance(world.birds);
        assert(the_sign.up && strcmp(the_sign.written, "10:10") == 0);
    }
    close_the_world(&world);

    /* On a twelve hour clock, one in the afternoon is 1:05 and the cell is the
     * same one as for 10:09: 1:05 is a digit narrower, and does not grow. */
    reset_sign_state();
    apply_screen_size(200, 50, 1600, 800);
    the_sign.kind = SIGN_CLOCK;
    the_sign.virtual_clock = 1;
    the_sign.twelve_hours = 1;
    the_sign.origin = local_time(13, 5, 0);
    begin_the_intro();
    open_the_world(&world, 300, 6);
    sign_advance(world.birds);
    assert(strcmp(the_sign.written, "1:05") == 0);
    assert(formation.count == font_text_cells("1:05"));
    assert(fabs(formation.cell - cell) < 1e-9);
    close_the_world(&world);
    reset_sign_state();
}

/* A clock that has landed, at 200 by 50 cells with a flock of 800: ten past ten
 * and five seconds from the next minute. */
static void open_a_clock_at(sign_sky_t *world, int hour, int minute, int second, int birds) {
    reset_sign_state();
    apply_screen_size(200, 50, 1600, 800);
    the_sign.kind = SIGN_CLOCK;
    the_sign.virtual_clock = 1;
    the_sign.origin = local_time(hour, minute, second);
    begin_the_intro();
    open_the_world(world, birds, 6);
}

static void test_a_new_minute_lets_go_of_the_letters_that_changed_and_of_nothing_else(void) {
    sign_sky_t world;
    open_a_clock_at(&world, 10, 9, 55, 800);
    /* Five seconds to the new minute, and a second of that is enough to land. */
    fly_the_world(&world, 0, 4.9, 60);
    assert(strcmp(the_sign.written, "10:09") == 0);
    assert_every_cell_has_its_writers(&world);

    /* Who writes the hour and the colon, and where, and who writes the minutes. */
    enum { KEPT_LETTERS = 3 }; /* "10:" */
    static int kept_birds[MAX_BIRDS], minute_birds[MAX_BIRDS];
    static double kept_x[MAX_BIRDS], kept_y[MAX_BIRDS];
    int kept = 0, minutes = 0;
    for (int i = 0; i < config.birds; i++) {
        int t = formation.slot[i];
        if (t < 0) continue;
        if (formation.glyph[t] < KEPT_LETTERS) {
            kept_birds[kept] = i;
            kept_x[kept] = formation.x[t];
            kept_y[kept] = formation.y[t];
            kept++;
        } else {
            minute_birds[minutes++] = i;
        }
    }
    assert(kept > 0 && minutes > 0);

    /* Across the change, frame by frame: the writers of "10:" never leave their
     * loops, never change their places and never let go. */
    int changed_at = -1;
    for (double at = 4.9; at < 8.0; at += 1.0 / 60) {
        step_the_world(&world, at);
        if (changed_at < 0 && strcmp(the_sign.written, "10:10") == 0) changed_at = (int)(at * 60);
        assert(the_sign.up && formation.writing);
        for (int k = 0; k < kept; k++) {
            int t = formation.slot[kept_birds[k]];
            assert(t >= 0 && formation.glyph[t] < KEPT_LETTERS);
            assert(formation.x[t] == kept_x[k] && formation.y[t] == kept_y[k]);
            assert(home_distance(&world, kept_birds[k]) <= formation.hover + 1e-6);
        }
    }
    assert(changed_at == (int)(5.0 * 60));

    /* The birds that wrote the old minutes are flocking, and the new ones are other
     * birds, in their loops, as many to a cell as before. */
    for (int m = 0; m < minutes; m++) assert(formation.slot[minute_birds[m]] < 0);
    assert(formation.count == font_text_cells("10:10"));
    assert_every_cell_has_its_writers(&world);
    close_the_world(&world);
    reset_sign_state();
}

static void test_the_hour_lets_go_of_the_whole_clock(void) {
    sign_sky_t world;
    open_a_clock_at(&world, 10, 59, 55, 800);
    fly_the_world(&world, 0, 4.9, 60);
    assert(strcmp(the_sign.written, "10:59") == 0);
    assert_every_cell_has_its_writers(&world);

    clock_state.seconds = 5.0;
    step_the_world(&world, 5.0);
    assert(!the_sign.up && !formation.writing);
    for (int i = 0; i < config.birds; i++) {
        double x, y;
        assert(!formation_target_of(i, &x, &y));
    }
    /* It flies for the three seconds, and then the new time is written. */
    fly_the_world(&world, 5.0 + 1.0 / 60, 5.0 + SIGN_CLOCK_FLIGHT - 0.1, 60);
    assert(!the_sign.up && !formation.writing);
    fly_the_world(&world, 5.0 + SIGN_CLOCK_FLIGHT - 0.1, 5.0 + SIGN_CLOCK_FLIGHT + 2.0, 60);
    assert(the_sign.up && strcmp(the_sign.written, "11:00") == 0);
    assert_every_cell_has_its_writers(&world);
    close_the_world(&world);
    reset_sign_state();
}

/* The same on a twelve hour clock, whose hour has no zero in front of it: "1:09" to
 * "1:10" is two letters, and "12:59" to "1:00" is the hour, of another length. */
static void test_a_twelve_hour_clock_changes_a_letter_at_a_time_too(void) {
    sign_sky_t world;
    open_a_clock_at(&world, 13, 9, 55, 800);
    the_sign.twelve_hours = 1;
    fly_the_world(&world, 0, 4.9, 60);
    assert(strcmp(the_sign.written, "1:09") == 0);
    assert_every_cell_has_its_writers(&world);
    static int kept[MAX_BIRDS];
    int held = 0;
    for (int i = 0; i < config.birds; i++)
        if (formation.slot[i] >= 0 && formation.glyph[formation.slot[i]] < 2) kept[held++] = i;
    assert(held > 0);
    fly_the_world(&world, 4.9, 7.0, 60);
    assert(the_sign.up && strcmp(the_sign.written, "1:10") == 0);
    assert(formation.count == font_text_cells("1:10"));
    for (int k = 0; k < held; k++) {
        int t = formation.slot[kept[k]];
        assert(t >= 0 && formation.glyph[t] < 2);
    }
    assert_every_cell_has_its_writers(&world);
    close_the_world(&world);

    open_a_clock_at(&world, 12, 59, 55, 800);
    the_sign.twelve_hours = 1;
    fly_the_world(&world, 0, 4.9, 60);
    assert(strcmp(the_sign.written, "12:59") == 0);
    clock_state.seconds = 5.0;
    step_the_world(&world, 5.0);
    assert(!the_sign.up && !formation.writing);
    fly_the_world(&world, 5.0 + 1.0 / 60, 5.0 + SIGN_CLOCK_FLIGHT + 2.0, 60);
    assert(the_sign.up && strcmp(the_sign.written, "1:00") == 0);
    assert_every_cell_has_its_writers(&world);
    close_the_world(&world);
    reset_sign_state();
}

/* Fresh birds for each letter that changes must not run the clock down: after an
 * hour of minutes, and the hour itself, every lit cell has as many birds as it
 * had at the start, and the same as every other cell. */
static void test_every_lit_cell_of_a_clock_keeps_its_writers_through_an_hour(void) {
    sign_sky_t world;
    open_a_clock_at(&world, 10, 0, 30, 800);
    fly_the_world(&world, 0, 3.0, 20);
    assert(strcmp(the_sign.written, "10:00") == 0);
    assert_every_cell_has_its_writers(&world);
    int per_cell = the_sign.per_cell;
    assert(per_cell >= 2); /* So that there is something to starve. */

    /* The change of each minute, and a few seconds either side of it, flown; the
     * rest of the minute is held, and need not be. Sixty one changes, which is the
     * hour at the sixtieth. */
    for (int minute = 1; minute <= 61; minute++) {
        double change = 30.0 + 60.0 * (minute - 1);
        fly_the_world(&world, change - 1.0, change + 4.5, 20);
        char want[16];
        snprintf(want, sizeof(want), "%d:%02d", 10 + (minute >= 60), minute % 60);
        assert(strcmp(the_sign.written, want) == 0);
        assert(the_sign.up && formation.count == font_text_cells(want));
        assert(the_sign.per_cell == per_cell);
        assert_every_cell_has_its_writers(&world);
    }
    close_the_world(&world);
    reset_sign_state();
}

/* The colour of a sign is a gradient along the text, from one end of the ramp to
 * the other, and the colour of the place and not of the bird. */
static void test_a_sign_is_coloured_along_its_text(void) {
    sign_sky_t world;
    int shades = 5;
    const char *texts[] = {"HELLO", "BACK IN FIVE MINUTES PLEASE", "I"};
    for (int k = 0; k < 3; k++) {
        reset_sign_state();
        apply_screen_size(200, 50, 1600, 800);
        ask_for_a_sign(texts[k]);
        begin_the_intro();
        open_the_world(&world, 600, 5);
        assert(palette_shades() == shades);
        fly_the_world(&world, 0, 2.5, 60);
        assert(the_sign.up);
        if (k == 1) assert(formation.y[formation.count - 1] > formation.y[0] + 5 * formation.cell);

        /* Every writer is the shade of its place's distance along the text, and
         * the places that are further along are never a lighter shade. */
        int lowest = shades, highest = -1, writers = 0;
        for (int i = 0; i < config.birds; i++) {
            int t = formation.slot[i];
            if (t < 0) continue;
            writers++;
            int want = (int)(formation.across[t] * shades);
            if (want >= shades) want = shades - 1;
            assert(world.birds[i].shade == want);
            if (world.birds[i].shade < lowest) lowest = world.birds[i].shade;
            if (world.birds[i].shade > highest) highest = world.birds[i].shade;
            for (int j = 0; j < config.birds; j++) {
                int u = formation.slot[j];
                if (u >= 0 && formation.across[u] < formation.across[t])
                    assert(world.birds[j].shade <= world.birds[i].shade);
            }
        }
        assert(writers > 0);
        /* A word runs the ramp from end to end, a wrapped text too, whatever its
         * lines, which are the colours of the columns they are in, and a lone
         * letter runs it across itself. */
        assert(lowest == 0 && highest == shades - 1);
        /* And it stays so while the birds hover, whichever way they are turned. */
        static int kept[MAX_BIRDS];
        for (int i = 0; i < config.birds; i++) kept[i] = world.birds[i].shade;
        fly_the_world(&world, 2.5, 4.0, 60);
        for (int i = 0; i < config.birds; i++)
            if (formation.slot[i] >= 0) assert(world.birds[i].shade == kept[i]);
        close_the_world(&world);
    }

    /* Two flocks are two colours, and a sign does not take that from them. */
    reset_sign_state();
    apply_screen_size(200, 50, 1600, 800);
    config.flocks = 2;
    ask_for_a_sign("HELLO");
    begin_the_intro();
    open_the_world(&world, 600, 5);
    fly_the_world(&world, 0, 2.5, 60);
    for (int i = 0; i < config.birds; i++)
        assert(world.birds[i].shade == shade_for_flock(world.birds[i].flock));
    close_the_world(&world);
    reset_sign_state();
}

/* Five letters, five shades: a clock of ice is a letter a colour, and a letter
 * keeps its colour when the letters round it are written again. */
static void test_the_letters_of_a_clock_are_a_shade_each_and_keep_it(void) {
    sign_sky_t world;
    open_a_clock_at(&world, 10, 9, 55, 800);
    fly_the_world(&world, 0, 4.9, 60);
    assert(palette_shades() == 5);
    for (int i = 0; i < config.birds; i++) {
        int t = formation.slot[i];
        if (t >= 0) assert(world.birds[i].shade == formation.glyph[t]);
    }
    fly_the_world(&world, 4.9, 7.0, 60);
    assert(strcmp(the_sign.written, "10:10") == 0);
    for (int i = 0; i < config.birds; i++) {
        int t = formation.slot[i];
        if (t >= 0) assert(world.birds[i].shade == formation.glyph[t]);
    }
    close_the_world(&world);
    reset_sign_state();
}

/* The time is the local time, wherever the program is: the same instant is one
 * time in one zone and another in the next. A POSIX zone written out, so that no
 * zone database has to be there. */
static void test_the_clock_tells_local_time(void) {
    const char *was = getenv("TZ");
    char kept[128] = "";
    if (was != NULL) snprintf(kept, sizeof(kept), "%s", was);

    reset_sign_state();
    the_sign.kind = SIGN_CLOCK;
    the_sign.virtual_clock = 1;
    the_sign.origin = 12 * 3600 + 34 * 60; /* 12:34 on the first of January 1970, UTC. */
    char text[SIGN_TEXT_MAX];

    setenv("TZ", "UTC0", 1);
    tzset();
    sign_text_now(text, sizeof(text));
    assert(strcmp(text, "12:34") == 0);
    setenv("TZ", "AHEAD-5", 1); /* Five hours ahead of UTC, in POSIX's backwards way. */
    tzset();
    sign_text_now(text, sizeof(text));
    assert(strcmp(text, "17:34") == 0);
    setenv("TZ", "BEHIND5", 1);
    tzset();
    sign_text_now(text, sizeof(text));
    assert(strcmp(text, "07:34") == 0);
    the_sign.twelve_hours = 1;
    sign_text_now(text, sizeof(text));
    assert(strcmp(text, "7:34") == 0);
    /* And the run's own clock moves it on, a second at a time. */
    clock_state.seconds = 66.0;
    the_sign.twelve_hours = 0;
    sign_text_now(text, sizeof(text));
    assert(strcmp(text, "07:35") == 0);

    if (was != NULL)
        setenv("TZ", kept, 1);
    else
        unsetenv("TZ");
    tzset();
    reset_sign_state();
}

static void test_the_colon_lifts_with_the_seconds(void) {
    reset_sign_state();
    apply_screen_size(200, 50, 1600, 800);
    sign_sky_t world;
    the_sign.kind = SIGN_CLOCK;
    the_sign.virtual_clock = 1;
    the_sign.origin = local_time(10, 9, 0);
    begin_the_intro();
    open_the_world(&world, 300, 6);
    clock_state.seconds = 0;
    sign_advance(world.birds);

    int lifted = 0;
    for (int t = 0; t < formation.count; t++) lifted += formation.lift[t] > 0;
    assert(lifted == font_text_cells(":")); /* The colon's birds and nobody else's. */
    /* On the tick, settled; half a second on, at the top; and back again. */
    for (int t = 0; t < formation.count; t++) assert(formation.shift_y[t] == 0);
    clock_state.seconds = 3.5;
    sign_advance(world.birds);
    for (int t = 0; t < formation.count; t++) {
        if (formation.lift[t] > 0)
            assert(fabs(formation.shift_y[t] + SIGN_BREATH_LIFT * formation.cell) < 1e-9);
        else
            assert(formation.shift_y[t] == 0);
    }
    clock_state.seconds = 4.0;
    sign_advance(world.birds);
    for (int t = 0; t < formation.count; t++) assert(fabs(formation.shift_y[t]) < 1e-9);

    /* A bird of the colon is flown to where it has risen. */
    clock_state.seconds = 3.5;
    sign_advance(world.birds);
    for (int i = 0; i < config.birds; i++) {
        int target = formation.slot[i];
        if (target < 0 || formation.lift[target] == 0) continue;
        double x, y;
        assert(formation_target_of(i, &x, &y));
        assert(y < formation.y[target] - 0.4 * formation.cell + formation.hover_y[i] + 1e-9);
        break;
    }
    close_the_world(&world);
    reset_sign_state();
}

static void test_the_pointer_scatters_a_sign_and_it_comes_back(void) {
    reset_sign_state();
    apply_screen_size(200, 50, 1600, 800);
    sign_sky_t world;
    ask_for_a_sign("HI");
    begin_the_intro();
    open_the_world(&world, 300, 7);
    fly_the_world(&world, 0, 4, 60);
    int writers = 0;
    for (int i = 0; i < config.birds; i++) writers += formation.slot[i] >= 0;
    for (int i = 0; i < config.birds; i++)
        if (formation.slot[i] >= 0) assert(home_distance(&world, i) <= formation.hover + 1e-6);

    /* A pointer that moves through the middle of the sign. */
    double middle_x = 0, middle_y = 0;
    for (int t = 0; t < formation.count; t++) {
        middle_x += formation.x[t] / formation.count;
        middle_y += formation.y[t] / formation.count;
    }
    mouse.present = 1;
    double most_away = 0;
    int scattered = 0;
    for (double at = 4; at < 6; at += 1.0 / 60) {
        mouse.x = middle_x + 20 * sin(at * 9);
        mouse.y = middle_y;
        mouse.moved_at = at;
        step_the_world(&world, at);
    }
    for (int i = 0; i < config.birds; i++) {
        int target = formation.slot[i];
        if (target < 0) continue;
        double dx = formation.x[target] - mouse.x, dy = formation.y[target] - mouse.y;
        if (dx * dx + dy * dy < (double)MOUSE_REACH * MOUSE_REACH) {
            scattered++;
            assert(world.birds[i].scattered > 0); /* Within its reach, scattered... */
            if (home_distance(&world, i) > most_away) most_away = home_distance(&world, i);
        } else if (dx * dx + dy * dy > (double)(MOUSE_REACH + 40) * (MOUSE_REACH + 40)) {
            assert(world.birds[i].scattered == 0); /* ...and no further. */
        }
    }
    assert(scattered >= 8 && scattered < writers); /* The letters it reaches, not all of them. */
    assert(most_away > 3 * formation.hover);       /* They are gone from their places. */

    /* The pointer stops and the birds come back to their own places. */
    fly_the_world(&world, 6, 14, 60);
    int home = 0;
    for (int i = 0; i < config.birds; i++) {
        if (formation.slot[i] < 0) continue;
        assert(world.birds[i].scattered == 0);
        home += home_distance(&world, i) <= formation.hover + 1e-6;
    }
    assert(home == writers);
    close_the_world(&world);

    /* A pointer that is not moving scatters nothing: the last place it was seen is
     * not a place the sign is afraid of. */
    reset_sign_state();
    apply_screen_size(200, 50, 1600, 800);
    ask_for_a_sign("HI");
    begin_the_intro();
    open_the_world(&world, 300, 7);
    mouse.present = 1;
    mouse.x = middle_x;
    mouse.y = middle_y;
    mouse.moved_at = -100;
    fly_the_world(&world, 0, 5, 60);
    for (int i = 0; i < config.birds; i++) assert(world.birds[i].scattered == 0);
    close_the_world(&world);
    reset_sign_state();
}

static void test_a_hawk_over_a_sign_scatters_the_places_it_is_over(void) {
    reset_sign_state();
    apply_screen_size(200, 50, 1600, 800);
    sign_sky_t world;
    ask_for_a_sign("HI");
    begin_the_intro();
    open_the_world(&world, 300, 7);
    fly_the_world(&world, 0, 4, 60);

    /* No hawks, nobody is scattered, whatever hawks[] has in it. */
    hawks[0].x = formation.x[0];
    hawks[0].y = formation.y[0];
    config.hawks = 0;
    for (int i = 0; i < config.birds; i++)
        if (formation.slot[i] >= 0) assert(sign_scatter_left(&world.birds[i], i) == 0);

    /* A hawk over the first place scatters the birds whose places are near it, and
     * not the others, and the ones it scatters stay away for a while. */
    config.hawks = 1;
    double reach = HAWK_SCATTER * hawk_reach();
    int near = 0, far = 0;
    for (int i = 0; i < config.birds; i++) {
        int target = formation.slot[i];
        if (target < 0) continue;
        double away = hypot(formation.x[target] - hawks[0].x, formation.y[target] - hawks[0].y);
        double left = sign_scatter_left(&world.birds[i], i);
        if (away < reach) {
            assert(left >= HAWK_SCATTER_SECONDS && left < SCATTER_SECONDS);
            near++;
        } else {
            assert(left == 0);
            far++;
        }
    }
    assert(near >= 4 && far >= 4);

    /* And the bird that was scattered comes home after the hawk has gone. */
    int scattered_bird = -1;
    for (int i = 0; i < config.birds && scattered_bird < 0; i++)
        if (formation.slot[i] == 0) scattered_bird = i;
    assert(scattered_bird >= 0);
    world.birds[scattered_bird].scattered =
        sign_scatter_left(&world.birds[scattered_bird], scattered_bird);
    config.hawks = 0;
    fly_the_world(&world, 4, 12, 60);
    assert(world.birds[scattered_bird].scattered == 0);
    assert(home_distance(&world, scattered_bird) <= formation.hover + 1e-6);
    close_the_world(&world);
    reset_sign_state();
}

static void test_the_intro_birds_are_never_scattered(void) {
    reset_sign_state();
    apply_screen_size(200, 50, 1600, 800);
    sign_sky_t world;
    begin_the_intro();
    open_the_world(&world, 100, 2);
    mouse.present = 1;
    mouse.x = screen.width / 2.0;
    mouse.y = screen.height / 2.0;
    for (double at = 0; at < 2; at += 1.0 / 60) {
        mouse.moved_at = at;
        step_the_world(&world, at);
    }
    for (int i = 0; i < config.birds; i++) assert(world.birds[i].scattered == 0);
    close_the_world(&world);
    reset_sign_state();
}

static void test_a_screensaver_quits_at_the_first_sign_of_anybody(void) {
    reset_sign_state();
    apply_screen_size(80, 24, 640, 384);

    /* Not a screensaver, a key is a key. */
    assert(feed_input("x") == 1);
    assert(feed_input("\033[<35;10;5M") == 1);

    screensaver_mode = 1;
    /* The first half second is whatever started it: read and thrown away, and the
     * program carries on. */
    clock_state.seconds = 0;
    assert(feed_input("x") == 1);
    clock_state.seconds = SCREENSAVER_GRACE - 0.01;
    assert(feed_input(" ") == 1);
    assert(feed_input("\033[<35;10;5M") == 1);
    assert(feed_input("q") == 1); /* Not even q: it is not a key for anything yet. */
    assert(!paused);

    /* After it, a key, a click, a pointer that moves, an arrow: any of them. */
    clock_state.seconds = SCREENSAVER_GRACE + 0.01;
    assert(feed_input("x") == 0);
    assert(feed_input("\r") == 0);
    assert(feed_input("\033[<0;10;5M") == 0);  /* A click. */
    assert(feed_input("\033[<35;11;5M") == 0); /* Just moving. */
    assert(feed_input("\033[A") == 0);
    assert(feed_input("\033") == 0);
    clock_state.seconds = 3600;
    assert(feed_input("z") == 0);
    /* Nothing at all is not a reason to leave. */
    assert(feed_input("") == 1);
    screensaver_mode = 0;
    reset_sign_state();
}

/* settle_the_sign says what it ignored on stderr, which is for a person and not
 * for the log of a test run. */
static void settle_quietly(void) {
    fflush(stderr);
    int kept = dup(STDERR_FILENO), quiet = open("/dev/null", O_WRONLY);
    assert(kept >= 0 && quiet >= 0 && dup2(quiet, STDERR_FILENO) == STDERR_FILENO);
    close(quiet);
    settle_the_sign();
    assert(dup2(kept, STDERR_FILENO) == STDERR_FILENO);
    close(kept);
}

/* Waits up to this many milliseconds for something to read on a descriptor. select
 * and not poll, as the program does: macOS's poll does not answer for a terminal. */
static int readable_within(int fd, int milliseconds) {
    fd_set set;
    FD_ZERO(&set);
    FD_SET(fd, &set);
    struct timeval limit = {milliseconds / 1000, (milliseconds % 1000) * 1000};
    return select(fd + 1, &set, NULL, NULL, &limit);
}

/* Runs the whole program's option handling in a child and returns how it exited. */
static int exit_status_of(int argc, char **argv) {
    fflush(NULL);
    pid_t child = fork();
    assert(child >= 0);
    if (child == 0) {
        int quiet = open("/dev/null", O_WRONLY);
        if (quiet < 0 || dup2(quiet, STDERR_FILENO) < 0 || dup2(quiet, STDOUT_FILENO) < 0)
            _exit(99);
        alarm(20);
        read_options(argc, argv);
        _exit(0);
    }
    int status = 0;
    assert(waitpid(child, &status, 0) == child);
    return WIFEXITED(status) ? WEXITSTATUS(status) : -1;
}

static void test_the_options_that_make_a_sign(void) {
    char error[160];
    int hour, minute, second;

    reset_sign_state();
    char *say[] = {"cbirds", "--say", "back in five", NULL};
    assert(options_parse(OPTIONS, OPTION_COUNT, 3, say, error, sizeof(error)) == OPTIONS_OK);
    assert(say_text != NULL && strcmp(say_text, "back in five") == 0);
    reset_sign_state();
    char *clock_flags[] = {"cbirds", "--clock", "--clock-at", "10:09:50", "--screensaver", NULL};
    assert(options_parse(OPTIONS, OPTION_COUNT, 5, clock_flags, error, sizeof(error)) ==
           OPTIONS_OK);
    assert(clock_mode && clock_start != NULL && screensaver_mode);
    reset_sign_state();

    /* The time to start the clock from: hours and minutes, and seconds if wanted. */
    assert(read_clock_start("10:09", &hour, &minute, &second) && hour == 10 && minute == 9 &&
           second == 0);
    assert(read_clock_start("23:59:58", &hour, &minute, &second) && hour == 23 && minute == 59 &&
           second == 58);
    assert(read_clock_start("0:00", &hour, &minute, &second) && hour == 0 && minute == 0);
    assert(!read_clock_start("24:00", &hour, &minute, &second));
    assert(!read_clock_start("10:60", &hour, &minute, &second));
    assert(!read_clock_start("10:09:60", &hour, &minute, &second));
    assert(!read_clock_start("10", &hour, &minute, &second));
    assert(!read_clock_start("10:", &hour, &minute, &second));
    assert(!read_clock_start("10:09:", &hour, &minute, &second));
    assert(!read_clock_start("10:09pm", &hour, &minute, &second));
    assert(!read_clock_start("ten", &hour, &minute, &second));
    assert(!read_clock_start("", &hour, &minute, &second));

    /* What is said, in the program: a text the font cannot draw is no sign. */
    say_text = "\x01 \xff";
    settle_quietly();
    assert(the_sign.kind == SIGN_NONE);
    reset_sign_state();
    say_text = "hello";
    config.birds = 800;
    settle_quietly();
    assert(the_sign.kind == SIGN_SAY && strcmp(sign_words, "HELLO") == 0);
    reset_sign_state();
    say_text = "hello world, this is a very long text";
    config.birds = 50;
    settle_quietly();
    assert(the_sign.kind == SIGN_NONE); /* The flock is too small to write it. */
    reset_sign_state();

    /* --clock-at is a clock, on the run's own time from that moment. */
    clock_start = "07:30:15";
    settle_quietly();
    assert(clock_mode && the_sign.kind == SIGN_CLOCK && the_sign.virtual_clock);
    struct tm local;
    assert(localtime_r(&the_sign.origin, &local) != NULL);
    assert(local.tm_hour == 7 && local.tm_min == 30 && local.tm_sec == 15);
    reset_sign_state();
    /* A recording and a bench run on their own clock, and say what it started at. */
    clock_mode = 1;
    record_path = "x.gif";
    settle_quietly();
    assert(the_sign.virtual_clock && labs((long)(the_sign.origin - time(NULL))) <= 2);
    record_path = NULL;
    reset_sign_state();
    clock_mode = 1;
    settle_quietly();
    assert(!the_sign.virtual_clock); /* Live, it is the wall's. */
    reset_sign_state();

    /* Two things to write at once is a mistake in the command, and says so. */
    char *both[] = {"cbirds", "--say", "hi", "--clock", NULL};
    assert(exit_status_of(4, both) == EXIT_USAGE);
    char *badly[] = {"cbirds", "--clock-at", "25:00", NULL};
    assert(exit_status_of(3, badly) == EXIT_USAGE);
    char *fine[] = {"cbirds", "--say", "hi", NULL};
    assert(exit_status_of(3, fine) == 0);
    reset_sign_state();
}

/* A picture of two colours side by side, as a PNG file in the test's own
 * directory: red on the left, blue on the right, wide, with a margin of nothing
 * round it so that only the ink is drawn. */
static void write_a_picture(const char *name, int red_on_the_left, int ink_alpha) {
    png_image_t image = {0, 0, NULL};
    assert(png_image_alloc(&image, 120, 60) == PNG_OK);
    for (int y = 0; y < 60; y++)
        for (int x = 0; x < 120; x++) {
            uint8_t *pixel = image.pixels + ((size_t)y * 120 + (size_t)x) * 4;
            int inside = x >= 10 && x < 110 && y >= 5 && y < 55;
            int left = x < 60;
            pixel[0] = (uint8_t)(left == red_on_the_left ? 220 : 30);
            pixel[1] = 30;
            pixel[2] = (uint8_t)(left == red_on_the_left ? 30 : 220);
            pixel[3] = (uint8_t)(inside ? ink_alpha : 0);
        }
    uint8_t *encoded = NULL;
    size_t length = 0;
    assert(png_encode(&image, &encoded, &length) == PNG_OK);
    char path[512];
    scratch_file(path, sizeof(path), name);
    FILE *file = fopen(path, "wb");
    assert(file != NULL && fwrite(encoded, 1, length, file) == length && fclose(file) == 0);
    free(encoded);
    png_image_free(&image);
}

static void settle_the_picture_quietly(void) {
    fflush(stderr);
    int kept = dup(STDERR_FILENO), quiet = open("/dev/null", O_WRONLY);
    assert(kept >= 0 && quiet >= 0 && dup2(quiet, STDERR_FILENO) == STDERR_FILENO);
    close(quiet);
    settle_the_sign();
    assert(dup2(kept, STDERR_FILENO) == STDERR_FILENO);
    close(kept);
}

static void test_a_picture_gives_every_bird_a_place_and_a_colour(void) {
    char path[512];
    reset_sign_state();
    write_a_picture("two.png", 1, 255);
    scratch_file(path, sizeof(path), "two.png");
    picture_path = path;
    settle_the_picture_quietly();

    /* Its own colours are the palette of the run: two of them, lightest first. */
    assert(the_sign.kind == SIGN_PICTURE && picture_colours_in_use);
    assert(picture_palette.shades == 2 && palette_shades() == 2);
    assert(palette() == &picture_palette && !palette_follows_the_theme());
    assert(picture_tints[0][0] == 220 || picture_tints[1][0] == 220);
    int red = picture_tints[0][0] == 220 ? 0 : 1;
    assert(picture_ink > 0.69 && picture_ink < 0.70); /* 100 by 50 of 120 by 60. */

    apply_screen_size(200, 50, 1600, 800);
    sign_sky_t world;
    begin_the_intro();
    open_the_world(&world, 600, 8);
    sign_advance(world.birds);
    assert(the_sign.up && formation.writing && formation.sign);

    /* As many targets as birds, one to each, in the picture's own proportions and
     * on its ink: nobody is left to flock and nobody is kept out. */
    assert(formation.count == config.birds && !formation.keep_out);
    double least_x = 1e9, most_x = 0, least_y = 1e9, most_y = 0;
    for (int i = 0; i < config.birds; i++) {
        assert(formation.slot[i] == i);
        double x = formation.x[i], y = formation.y[i];
        if (x < least_x) least_x = x;
        if (x > most_x) most_x = x;
        if (y < least_y) least_y = y;
        if (y > most_y) most_y = y;
    }
    assert(fabs((most_x - least_x) / (most_y - least_y) - 2.0) < 0.15);
    double middle = (least_x + most_x) / 2;
    int on_the_left = 0;
    for (int i = 0; i < config.birds; i++) {
        /* The colour of the picture where it is, red to the left of the middle. */
        int wants_red = formation.x[i] < middle;
        on_the_left += wants_red;
        if (fabs(formation.x[i] - middle) < 8) continue; /* On the seam. */
        assert(picture_shade[i] == (wants_red ? red : 1 - red));
    }
    assert(on_the_left > 270 && on_the_left < 330);

    /* A bird wears the colour it was given, flying in and at home. */
    int first_colour[600];
    for (int i = 0; i < config.birds; i++) first_colour[i] = picture_shade[i];
    fly_the_world(&world, 0, 4, 60);
    for (int i = 0; i < config.birds; i++) {
        assert(world.birds[i].shade == first_colour[i]);
        assert(home_distance(&world, i) <= formation.hover + 1e-6);
    }

    /* Let go, it flies and keeps its colour, and when it writes again every bird is
     * home with the colour it had. */
    double hold = sign_hold_seconds(0), flight = sign_flight_seconds(0);
    fly_the_world(&world, 4, hold + 1, 25);
    assert(!formation.writing);
    for (int i = 0; i < config.birds; i++) assert(world.birds[i].shade == first_colour[i]);
    fly_the_world(&world, hold + 1, hold + flight + 6, 25);
    assert(formation.writing && formation.count == config.birds);
    for (int i = 0; i < config.birds; i++) {
        assert(world.birds[i].shade == first_colour[i]);
        assert(home_distance(&world, i) <= formation.hover + 1e-6);
    }
    close_the_world(&world);
    reset_sign_state();
}

static void test_a_picture_wears_a_ramp_somebody_chose(void) {
    char path[512];
    reset_sign_state();
    write_a_picture("two.png", 1, 255);
    scratch_file(path, sizeof(path), "two.png");
    picture_path = path;
    palette_was_asked_for = 1;
    config.palette = palette_named("ice");
    settle_the_picture_quietly();

    /* The ramp is the one asked for, and the picture only says where on it. */
    assert(the_sign.kind == SIGN_PICTURE && !picture_colours_in_use);
    assert(palette() == &PALETTES[palette_named("ice")]);
    assert(palette_shades() == 5);
    const uint8_t light[3] = {255, 255, 255}, dark[3] = {0, 0, 0};
    const uint8_t red[3] = {220, 30, 30}, blue[3] = {30, 30, 220};
    /* Light is the light end of the ramp, dark is the dark end, and in between the
     * shade follows the lightness: the red of this picture is lighter than its blue. */
    assert(picture_shade_of(light) == 0 && picture_shade_of(dark) == palette_shades() - 1);
    assert(picture_luminance(red) > picture_luminance(blue));
    assert(picture_shade_of(red) < picture_shade_of(blue));
    int previous = 0;
    for (int grey = 255; grey >= 0; grey -= 5) {
        const uint8_t colour[3] = {(uint8_t)grey, (uint8_t)grey, (uint8_t)grey};
        int shade = picture_shade_of(colour);
        assert(shade >= previous && shade < palette_shades()); /* Darker never goes lighter. */
        previous = shade;
    }
    /* A sprite of one's own keeps its colours, and the picture is drawn in them. */
    sprite_path = path;
    assert(palette_shades() == 1 && picture_shade_of(red) == 0);
    reset_sign_state();
}

static void test_a_picture_in_black_is_not_a_picture_of_nothing(void) {
    char path[512];
    reset_sign_state();
    png_image_t image = {0, 0, NULL};
    assert(png_image_alloc(&image, 20, 20) == PNG_OK);
    for (int i = 0; i < 400; i++) {
        image.pixels[i * 4 + 0] = image.pixels[i * 4 + 1] = image.pixels[i * 4 + 2] =
            i < 200 ? 0 : 240;
        image.pixels[i * 4 + 3] = 255;
    }
    uint8_t *encoded = NULL;
    size_t length = 0;
    assert(png_encode(&image, &encoded, &length) == PNG_OK);
    png_image_free(&image);
    scratch_file(path, sizeof(path), "black.png");
    FILE *file = fopen(path, "wb");
    assert(file != NULL && fwrite(encoded, 1, length, file) == length && fclose(file) == 0);
    free(encoded);
    picture_path = path;
    settle_the_picture_quietly();

    /* The black of the picture is a colour a bird can be seen in, against the ground
     * the recordings are painted on, and the white is as it was. */
    assert(picture_palette.shades == 2);
    const uint8_t ground[3] = {PICTURE_GROUND[0], PICTURE_GROUND[1], PICTURE_GROUND[2]};
    for (int c = 0; c < 2; c++) assert(contrast_between(picture_tints[c], ground) >= 1.8);
    assert(picture_tints[0][0] == 240);
    assert(picture_tints[1][0] > 0 && picture_tints[1][0] < 120);
    /* But what is dark in the picture is still the dark end of a ramp of one's own. */
    assert(picture_dark == 0 && picture_light > 0.9);
    assert(unlink(path) == 0);
    reset_sign_state();
}

static void test_a_large_picture_is_kept_small(void) {
    char path[512];
    reset_sign_state();
    png_image_t image = {0, 0, NULL};
    assert(png_image_alloc(&image, 2400, 1200) == PNG_OK);
    for (int y = 0; y < 1200; y++)
        for (int x = 0; x < 2400; x++) {
            uint8_t *pixel = image.pixels + ((size_t)y * 2400 + (size_t)x) * 4;
            pixel[0] = (uint8_t)(x < 1200 ? 240 : 20);
            pixel[1] = 60;
            pixel[2] = (uint8_t)(x < 1200 ? 20 : 240);
            pixel[3] = (uint8_t)(y < 600 ? 255 : 0);
        }
    uint8_t *encoded = NULL;
    size_t length = 0;
    assert(png_encode(&image, &encoded, &length) == PNG_OK);
    png_image_free(&image);
    scratch_file(path, sizeof(path), "large.png");
    FILE *file = fopen(path, "wb");
    assert(file != NULL && fwrite(encoded, 1, length, file) == length && fclose(file) == 0);
    free(encoded);
    picture_path = path;
    settle_the_picture_quietly();

    /* A thousand pixels across, in the same proportions, with the same ink and the
     * same two colours. */
    assert(the_sign.kind == SIGN_PICTURE);
    assert(picture_image.width == PICTURE_KEPT_MAX && picture_image.height == PICTURE_KEPT_MAX / 2);
    assert(fabs(picture_ink - 0.5) < 0.01);
    assert(picture_palette.shades == 2);
    assert(unlink(path) == 0);
    reset_sign_state();
}

static void test_a_picture_that_cannot_be_drawn_says_so(void) {
    char good[512], broken[512], empty[512], missing[512];
    reset_sign_state();
    write_a_picture("good.png", 1, 255);
    write_a_picture("empty.png", 1, 0);
    scratch_file(good, sizeof(good), "good.png");
    scratch_file(empty, sizeof(empty), "empty.png");
    scratch_file(broken, sizeof(broken), "broken.png");
    scratch_file(missing, sizeof(missing), "missing.png");
    FILE *file = fopen(broken, "wb");
    assert(file != NULL && fputs("this is not a PNG", file) >= 0 && fclose(file) == 0);

    /* Nothing opaque: not a mistake, only nothing to draw, and the flock flies. */
    picture_path = empty;
    settle_the_picture_quietly();
    assert(the_sign.kind == SIGN_NONE && !picture_colours_in_use);
    reset_sign_state();

    /* A file that is not a PNG, and one that is not there, are errors, said the way
     * a bad --sprite is said and exiting as it does. */
    char *not_a_png[] = {"cbirds", "--picture", broken, NULL};
    assert(exit_status_of(3, not_a_png) == EXIT_FAILURE);
    char *not_there[] = {"cbirds", "--picture", missing, NULL};
    assert(exit_status_of(3, not_there) == EXIT_FAILURE);
    char *fine[] = {"cbirds", "--picture", good, NULL};
    assert(exit_status_of(3, fine) == 0);

    /* Over four megabytes, like a sprite. */
    char big[512];
    scratch_file(big, sizeof(big), "big.png");
    file = fopen(big, "wb");
    assert(file != NULL);
    static char block[1 << 16];
    for (int i = 0; i < 65; i++) assert(fwrite(block, 1, sizeof(block), file) == sizeof(block));
    assert(fclose(file) == 0);
    char *too_big[] = {"cbirds", "--picture", big, NULL};
    assert(exit_status_of(3, too_big) == EXIT_FAILURE);

    /* Three things that each take the whole sign: only one at a time. */
    char *twice[] = {"cbirds", "--picture", good, "--say", "hi", NULL};
    assert(exit_status_of(5, twice) == EXIT_USAGE);
    char *and_a_clock[] = {"cbirds", "--picture", good, "--clock", NULL};
    assert(exit_status_of(4, and_a_clock) == EXIT_USAGE);
    assert(unlink(good) == 0 && unlink(empty) == 0 && unlink(broken) == 0 && unlink(big) == 0);
    reset_sign_state();
}

static void test_a_colour_given_is_told_from_the_default(void) {
    char path[512];
    write_a_picture("two.png", 1, 255);
    scratch_file(path, sizeof(path), "two.png");

    /* Not given: the picture's own. Given, even as the default's own name: the
     * ramp that was named. */
    reset_sign_state();
    char *plain[] = {"cbirds", "--picture", path, "--seed", "4", NULL};
    read_options(5, plain);
    assert(!palette_was_asked_for && picture_colours_in_use && config.palette == 0);
    assert(the_sign.seed == 4);
    reset_sign_state();
    char *named[] = {"cbirds", "--picture", path, "--color", "ice", NULL};
    read_options(5, named);
    assert(palette_was_asked_for && !picture_colours_in_use);
    assert(PALETTES[config.palette].name[0] == 'i');
    reset_sign_state();
    char *theme[] = {"cbirds", "--picture", path, "-c", "theme", NULL};
    read_options(5, theme);
    assert(palette_was_asked_for && !picture_colours_in_use && palette_follows_the_theme());
    reset_sign_state();
    /* With no picture, none of it matters, and the default is what it was. */
    char *nothing[] = {"cbirds", NULL};
    read_options(1, nothing);
    assert(config.palette == 0 && !picture_colours_in_use && palette_follows_the_theme());
    assert(unlink(path) == 0);
    reset_sign_state();
}

/* A sign laid out as the program lays it out on a screen of this many cells: the
 * bird it would pick, the pace of this frame rate, and the writers it would send. */
static void lay_out_a_sign_on(sign_sky_t *world, int columns, int rows, int fps, const char *text,
                              int birds, int seed) {
    reset_sign_state();
    config.pace_notch = DEFAULT_PACE_NOTCH;
    apply_notches();
    apply_screen_size(columns, rows, columns * 8, rows * 16);
    frame_seconds = 1.0 / fps;
    update_speed();
    ask_for_a_sign(text);
    config.bird_size = 0;
    settle_the_bird_size();
    begin_the_intro();
    open_the_world(world, birds, seed);
    clock_state.seconds = 0;
    sign_advance(world->birds);
    assert(the_sign.up);
}

static void test_a_small_screen_leaves_the_flock_sky_and_a_roomy_one_is_as_it_was(void) {
    sign_sky_t world;

    /* How small a screen is, by its shorter side: the 96 by 26 of a recording is as
     * small as it gets, 160 by 45 is roomy, and between them it is a straight line. */
    reset_sign_state();
    double previous = 2;
    for (int rows = 20; rows <= 50; rows += 2) {
        apply_screen_size(rows * 3, rows, rows * 3 * 8, rows * 16);
        double small = sign_smallness();
        assert(small >= 0 && small <= 1 && small <= previous + 1e-12);
        previous = small;
        if (rows * 16 <= 416) assert(small == 1);
        if (rows * 16 >= 720) assert(small == 0);
    }
    apply_screen_size(120, 34, 120 * 8, 34 * 16);
    assert(sign_smallness() > 0.3 && sign_smallness() < 0.8);

    /* A roomy screen is laid out as it always was: the roomy shares of the room, the
     * writers a sixth of a flock short of three fifths, the band four steps of flight. */
    lay_out_a_sign_on(&world, 200, 50, 25, "HELLO WORLD", 800, 5);
    {
        double pad = config.bird_size * 2.0;
        sign_lines_t lines;
        double cell;
        assert(sign_fit("HELLO WORLD", 0, (1600 - 2 * pad) * SIGN_WIDTH_SHARE,
                        (800 - 2 * pad) * SIGN_HEIGHT_SHARE, SIGN_LARGEST_CELL * config.bird_size,
                        &lines, &cell) > 0);
        assert(fabs(formation.cell - cell) < 1e-9);
        assert(the_sign.per_cell == (int)(800 * SIGN_WRITER_SHARE) / formation.count);
        assert(fabs(sign_band() - SIGN_KEEP_OUT_STEPS * config.speed) < 1e-9);
    }
    close_the_world(&world);

    /* A small one gives the sign less of it, in cells that are still letters, and
     * the band is a fifth of the screen at the most, at the pace of a recording and
     * as it was at the pace of a terminal. */
    lay_out_a_sign_on(&world, 96, 26, 25, "HELLO WORLD", 800, 5);
    {
        double pad = config.bird_size * 2.0;
        sign_lines_t lines;
        double roomy_cell;
        assert(sign_fit("HELLO WORLD", 0, (768 - 2 * pad) * SIGN_WIDTH_SHARE,
                        (416 - 2 * pad) * SIGN_HEIGHT_SHARE, SIGN_LARGEST_CELL * config.bird_size,
                        &lines, &roomy_cell) > 0);
        assert(formation.cell < roomy_cell && formation.cell >= SIGN_SMALLEST_CELL);
        assert((formation.box.right - formation.box.left) < 0.5 * 768);
        assert((formation.box.bottom - formation.box.top) < 0.5 * 416);
        assert(sign_band() <= SIGN_KEEP_OUT_MOST * 416 + 1e-9);
        assert(sign_band() >= formation.band);
        assert(SIGN_KEEP_OUT_STEPS * config.speed > SIGN_KEEP_OUT_MOST * 416); /* It did bite. */
        frame_seconds = 1.0 / 60;
        update_speed();
        assert(fabs(sign_band() - SIGN_KEEP_OUT_STEPS * config.speed) < 1e-9);
    }
    close_the_world(&world);
    /* A word that is as wide as a line gets, which is bound by the width. */
    lay_out_a_sign_on(&world, 96, 26, 25, "HELLO", 800, 5);
    assert(formation.box.right - formation.box.left < 0.55 * 768);
    close_the_world(&world);

    /* More of the flock writes on a small screen, which is what makes a long text
     * strokes and not dots: three lines are two birds to a cell there and one on a
     * roomy screen, and a short text is three to a cell on both. */
    int per_cell[2][2];
    const char *texts[2] = {"HELLO WORLD", "BACK IN FIVE MINUTES"};
    for (int t = 0; t < 2; t++) {
        lay_out_a_sign_on(&world, 96, 26, 25, texts[t], 800, 5);
        per_cell[0][t] = the_sign.per_cell;
        close_the_world(&world);
        lay_out_a_sign_on(&world, 200, 50, 25, texts[t], 800, 5);
        per_cell[1][t] = the_sign.per_cell;
        close_the_world(&world);
    }
    assert(per_cell[0][0] == 3 && per_cell[1][0] == 3);
    assert(per_cell[0][1] == 2 && per_cell[1][1] == 1);

    /* A long text on a small screen is not made smaller than its letters can be: at
     * 80 by 24 its cell is the floor, or the roomy cell if that is less. */
    reset_sign_state();
    apply_screen_size(80, 24, 640, 384);
    config.bird_size = 12;
    {
        sign_lines_t lines, roomy;
        double cell, roomy_cell, width, height;
        double room_width = 640 - 2 * 24.0, room_height = 384 - 2 * 24.0;
        assert(sign_fit_in("BACK IN FIVE MINUTES", 0, room_width, room_height, 1e9, &lines, &cell,
                           &width, &height) == 3);
        assert(sign_fit("BACK IN FIVE MINUTES", 0, room_width * SIGN_WIDTH_SHARE,
                        room_height * SIGN_HEIGHT_SHARE, 1e9, &roomy, &roomy_cell) == 3);
        /* Eight pixels: under that a letter stops being one. */
        assert(roomy_cell > 8);
        assert(cell >= 8 - 1e-9 && cell <= roomy_cell + 1e-9);
    }

    /* And the free flock has sky: on 96 by 26 at 25 frames a second, with 800 birds,
     * from six seconds on, few of the free birds are hugging the sign or an edge, and
     * the sky is used. Before the shares and the band were as they were on a roomy
     * screen, 23% hugged and 1% of the sky held a bird. */
    double hugging = 0, used = 0, inside = 0;
    int samples = 0;
    for (int seed = 1; seed <= 2; seed++) {
        lay_out_a_sign_on(&world, 96, 26, 25, "HELLO WORLD", 800, seed);
        for (double at = 0.04; at < 14; at += 0.04) {
            step_the_world(&world, at);
            if (at < 6 || ((int)(at * 25 + 0.5)) % 12 != 0) continue;
            double band = sign_band();
            sign_box_t big = {formation.box.left - band, formation.box.top - band,
                              formation.box.right + band, formation.box.bottom + band};
            int free_birds = 0, near = 0, in_the_box = 0, sky_cells = 0, used_cells = 0;
            static char held[64 * 64];
            memset(held, 0, sizeof(held));
            for (int i = 0; i < config.birds; i++) {
                if (formation.slot[i] >= 0) continue;
                free_birds++;
                double x = world.birds[i].x, y = world.birds[i].y;
                double dx = x < formation.box.left
                                ? formation.box.left - x
                                : (x > formation.box.right ? x - formation.box.right : 0);
                double dy = y < formation.box.top
                                ? formation.box.top - y
                                : (y > formation.box.bottom ? y - formation.box.bottom : 0);
                double clear = hypot(dx, dy);
                if (screen.width - x < clear) clear = screen.width - x;
                if (x < clear) clear = x;
                if (y < clear) clear = y;
                if (screen.height - y < clear) clear = screen.height - y;
                near += clear < config.bird_size;
                in_the_box += dx == 0 && dy == 0;
                int cx = (int)(x / 32), cy = (int)(y / 32);
                if (cx >= 0 && cx < 64 && cy >= 0 && cy < 64) held[cy * 64 + cx] = 1;
            }
            for (int cy = 0; cy * 32 < screen.height; cy++)
                for (int cx = 0; cx * 32 < screen.width; cx++) {
                    double mx = cx * 32 + 16, my = cy * 32 + 16;
                    if (mx > big.left && mx < big.right && my > big.top && my < big.bottom)
                        continue;
                    sky_cells++;
                    used_cells += held[cy * 64 + cx];
                }
            assert(free_birds > 0 && sky_cells > 0);
            hugging += (double)near / free_birds;
            inside += (double)in_the_box / free_birds;
            used += (double)used_cells / sky_cells;
            samples++;
        }
        close_the_world(&world);
    }
    assert(samples > 10);
    assert(hugging / samples < 0.12);
    assert(used / samples > 0.15);
    assert(inside / samples < 0.02);
    reset_sign_state();
}

/* What sign_report_failure says, as a string: it speaks on stderr, which belongs to
 * the person and not to the log of a test run. */
static void what_the_sign_says(char *out, size_t size) {
    fflush(stderr);
    FILE *capture = tmpfile();
    assert(capture != NULL);
    int kept = dup(STDERR_FILENO);
    assert(kept >= 0 && dup2(fileno(capture), STDERR_FILENO) == STDERR_FILENO);
    sign_report_failure();
    fflush(stderr);
    assert(dup2(kept, STDERR_FILENO) == STDERR_FILENO);
    close(kept);
    rewind(capture);
    size_t got = fread(out, 1, size - 1, capture);
    out[got] = '\0';
    fclose(capture);
}

static void test_a_sign_that_cannot_be_laid_out_says_so_when_the_run_is_over(void) {
    char said[400];
    sign_sky_t world;
    program_name = "cbirds";

    /* A sign that has not failed says nothing. */
    reset_sign_state();
    apply_screen_size(200, 50, 1600, 800);
    ask_for_a_sign("HELLO");
    begin_the_intro();
    open_the_world(&world, 300, 3);
    sign_advance(world.birds);
    assert(the_sign.up && the_sign.failures == 0);
    what_the_sign_says(said, sizeof(said));
    assert(said[0] == '\0');
    close_the_world(&world);

    /* A text too big for the screen, found out once the run has started. */
    reset_sign_state();
    apply_screen_size(30, 8, 240, 128);
    ask_for_a_sign("ABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZ");
    begin_the_intro();
    open_the_world(&world, 1000, 3);
    for (double at = 0; at < 3; at += 0.5) {
        clock_state.seconds = at;
        sign_advance(world.birds);
    }
    assert(!the_sign.up && the_sign.failures > 0 && the_sign.why == SIGN_NO_ROOM);
    what_the_sign_says(said, sizeof(said));
    assert(strcmp(said,
                  "cbirds: --say could not be laid out on a screen of 30 by 8 cells: the text is "
                  "too big for it, so the flock flew as usual\n") == 0);
    /* Said once. */
    what_the_sign_says(said, sizeof(said));
    assert(said[0] == '\0');
    close_the_world(&world);

    /* The clock has a time, not a text. */
    reset_sign_state();
    apply_screen_size(8, 3, 64, 48);
    the_sign.kind = SIGN_CLOCK;
    the_sign.virtual_clock = 1;
    the_sign.origin = local_time(10, 9, 0);
    begin_the_intro();
    open_the_world(&world, 300, 3);
    sign_advance(world.birds);
    what_the_sign_says(said, sizeof(said));
    assert(
        strstr(said,
               "--clock could not be laid out on a screen of 8 by 3 cells: the time is too big") ==
        said + strlen("cbirds: "));
    close_the_world(&world);

    /* A flock that is too small for it says how many birds it takes. */
    reset_sign_state();
    apply_screen_size(200, 50, 1600, 800);
    ask_for_a_sign("HELLO WORLD");
    begin_the_intro();
    open_the_world(&world, 40, 3);
    sign_advance(world.birds);
    assert(!the_sign.up && the_sign.why == SIGN_TOO_FEW_BIRDS);
    what_the_sign_says(said, sizeof(said));
    char want[300];
    snprintf(want, sizeof(want),
             "cbirds: --say could not be laid out on a screen of 200 by 50 cells: it takes %d "
             "birds to write and the flock has 40 to write with, so the flock flew as usual\n",
             font_text_cells("HELLO WORLD"));
    assert(strcmp(said, want) == 0);
    close_the_world(&world);

    /* A sign that was up and lost its room says for how long it did not have it, not
     * that the flock flew as usual: it did, for a time. */
    reset_sign_state();
    apply_screen_size(200, 50, 1600, 800);
    ask_for_a_sign("HELLO WORLD");
    begin_the_intro();
    open_the_world(&world, 300, 3);
    sign_advance(world.birds);
    assert(the_sign.up);
    apply_screen_size(30, 8, 240, 128);
    clock_state.seconds = 1;
    sign_advance(world.birds);
    assert(!the_sign.up && the_sign.failures > 0 && the_sign.was_up);
    what_the_sign_says(said, sizeof(said));
    assert(
        strcmp(said,
               "cbirds: --say could not be laid out for a time on a screen of 30 by 8 cells: the "
               "text is too big for it\n") == 0);
    close_the_world(&world);
    reset_sign_state();
}

/* The whole program, on a terminal of its own that is 30 by 8 cells, with a text no
 * such screen can hold: the person at it is told when the screen has been given
 * back, and not before, because the alternate screen would hide it. */
static void test_a_text_too_big_for_the_terminal_is_said_after_the_terminal_is_given_back(void) {
    reset_sign_state();
    int master = posix_openpt(O_RDWR | O_NOCTTY);
    assert(master >= 0 && grantpt(master) == 0 && unlockpt(master) == 0);
    const char *name = ptsname(master);
    assert(name != NULL);
    int terminal = open(name, O_RDWR | O_NOCTTY);
    assert(terminal >= 0);
    struct winsize size = {.ws_row = 8, .ws_col = 30, .ws_xpixel = 240, .ws_ypixel = 128};
    assert(ioctl(terminal, TIOCSWINSZ, &size) == 0);
    fflush(NULL);
    pid_t child = fork();
    assert(child >= 0);
    if (child == 0) {
        alarm(60);
        /* The person's stderr is their terminal too. */
        if (dup2(terminal, STDIN_FILENO) < 0 || dup2(terminal, STDOUT_FILENO) < 0 ||
            dup2(terminal, STDERR_FILENO) < 0)
            _exit(99);
        terminal_is_raw = terminal_restored = alt_screen_is_on = sprites_uploaded = 0;
        char *argv[] = {
            "cbirds", "--unlock-fps", "--frames",
            "60",     "--color",      "ember",
            "-n",     "1000",         "--seed",
            "3",      "--say",        "ABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZ",
            NULL};
        _exit(cbirds_application_main(12, argv));
    }
    close(terminal);
    static char output[1 << 20];
    size_t length = 0;
    int status = 0;
    for (;;) {
        if (readable_within(master, 100) > 0) {
            ssize_t got = read(master, output + length, sizeof(output) - 1 - length);
            if (got <= 0) break;
            length += (size_t)got;
        }
        if (waitpid(child, &status, WNOHANG) == child) {
            child = -1;
            break;
        }
    }
    if (child > 0) assert(waitpid(child, &status, 0) == child);
    /* What the program had still to say when it exited. */
    while (readable_within(master, 100) > 0) {
        ssize_t got = read(master, output + length, sizeof(output) - 1 - length);
        if (got <= 0) break;
        length += (size_t)got;
    }
    output[length] = '\0';
    close(master);
    assert(WIFEXITED(status) && WEXITSTATUS(status) == EXIT_SUCCESS);

    const char *message =
        "--say could not be laid out on a screen of 30 by 8 cells: the text is too "
        "big for it, so the flock flew as usual";
    const char *told = strstr(output, message);
    const char *taken = strstr(output, ALT_SCREEN_ON);
    const char *given_back = NULL;
    for (const char *at = strstr(output, ALT_SCREEN_OFF); at != NULL;
         at = strstr(at + 1, ALT_SCREEN_OFF))
        given_back = at;
    assert(taken != NULL && given_back != NULL && told != NULL);
    assert(told > given_back && given_back > taken); /* Said once, after the screen is back. */
    assert(strstr(told + 1, message) == NULL);
    reset_sign_state();
}

static void test_a_sign_has_a_bird_as_wide_as_its_cells_unless_it_is_told(void) {
    reset_sign_state();
    /* The usual thirty on a roomy screen, and smaller on a small one, where a bird
     * of thirty pixels is a smudge on a letter of twelve. */
    apply_screen_size(200, 50, 1600, 800);
    ask_for_a_sign("HI");
    config.bird_size = 0;
    settle_the_bird_size();
    assert(config.bird_size == DEFAULT_BIRD_SIZE);
    reset_sign_state();
    apply_screen_size(80, 24, 640, 384);
    ask_for_a_sign("HELLO WORLD");
    config.bird_size = 0;
    settle_the_bird_size();
    assert(config.bird_size >= MIN_BIRD_SIZE && config.bird_size < DEFAULT_BIRD_SIZE);
    /* Told, it is what it was told; and without a sign, always thirty. */
    config.bird_size = 21;
    settle_the_bird_size();
    assert(config.bird_size == 21);
    reset_sign_state();
    apply_screen_size(80, 24, 640, 384);
    config.bird_size = 0;
    settle_the_bird_size();
    assert(config.bird_size == DEFAULT_BIRD_SIZE);
    reset_sign_state();
}

static void test_a_sign_records_in_a_gif_and_a_cast(void) {
    char gif[512], cast[512], picture[512], picture_gif[512];
    scratch_file(gif, sizeof(gif), "sign.gif");
    scratch_file(cast, sizeof(cast), "sign.cast");
    scratch_file(picture, sizeof(picture), "recorded.png");
    scratch_file(picture_gif, sizeof(picture_gif), "picture.gif");
    write_a_picture("recorded.png", 0, 255);

    /* Headless, in both formats, with the clock on its own time: the same flags the
     * README gives, run in a child because a recording is a whole run. */
    struct {
        const char *file, *option, *value;
    } runs[] = {{gif, "--say", "hi there"},
                {cast, "--clock-at", "10:09:55"},
                {picture_gif, "--picture", picture}};
    for (size_t which = 0; which < sizeof(runs) / sizeof(*runs); which++) {
        reset_sign_state();
        fflush(NULL);
        pid_t child = fork();
        assert(child >= 0);
        if (child == 0) {
            int quiet = open("/dev/null", O_WRONLY);
            if (quiet < 0 || dup2(quiet, STDOUT_FILENO) < 0 || dup2(quiet, STDERR_FILENO) < 0)
                _exit(99);
            char *argv[] = {"cbirds",
                            "--record",
                            (char *)runs[which].file,
                            "--record-seconds",
                            "2",
                            "--record-size",
                            "64x18",
                            "-n",
                            "200",
                            "--seed",
                            "3",
                            (char *)runs[which].option,
                            (char *)runs[which].value,
                            NULL};
            alarm(60);
            _exit(cbirds_application_main(13, argv));
        }
        int status = 0;
        assert(waitpid(child, &status, 0) == child);
        assert(WIFEXITED(status) && WEXITSTATUS(status) == EXIT_SUCCESS);
    }
    for (int which = 0; which < 3; which += 2) {
        FILE *file = fopen(runs[which].file, "rb");
        assert(file != NULL);
        char magic[6];
        assert(fread(magic, 1, 6, file) == 6 && memcmp(magic, "GIF89a", 6) == 0);
        fclose(file);
    }
    FILE *file = fopen(cast, "r");
    assert(file != NULL);
    char line[256];
    assert(fgets(line, sizeof(line), file) != NULL && strstr(line, "\"version\": 2") != NULL);
    int frames = 0;
    while (fgets(line, sizeof(line), file) != NULL) frames++;
    assert(frames > 40);
    fclose(file);
    assert(unlink(gif) == 0 && unlink(cast) == 0 && unlink(picture_gif) == 0 &&
           unlink(picture) == 0);
    reset_sign_state();
}

/* The whole program, on a terminal of its own, with a sign up and keys that grow
 * the flock arriving at once: a sign is laid out from every bird of the flock, and
 * it must be laid out from the flock the keys have made, not the one before them.
 * It read past the end of the old one, which only the sanitizers could see. */
static void test_a_sign_survives_the_flock_growing_under_it(void) {
    const char *runs[][5] = {{"--say", "hi", NULL}, {"--clock", NULL}, {"--picture", NULL}};
    char picture[512];
    write_a_picture("grown.png", 1, 255);
    scratch_file(picture, sizeof(picture), "grown.png");
    runs[2][1] = picture;

    for (int which = 0; which < 3; which++) {
        reset_sign_state();
        int master = posix_openpt(O_RDWR | O_NOCTTY);
        assert(master >= 0 && grantpt(master) == 0 && unlockpt(master) == 0);
        const char *name = ptsname(master);
        assert(name != NULL);
        int terminal = open(name, O_RDWR | O_NOCTTY);
        assert(terminal >= 0);
        fflush(NULL);
        pid_t child = fork();
        assert(child >= 0);
        if (child == 0) {
            alarm(60);
            int quiet = open("/dev/null", O_WRONLY);
            if (quiet < 0 || dup2(terminal, STDIN_FILENO) < 0 ||
                dup2(terminal, STDOUT_FILENO) < 0 || dup2(quiet, STDERR_FILENO) < 0)
                _exit(99);
            terminal_is_raw = terminal_restored = alt_screen_is_on = sprites_uploaded = 0;
            /* A named colour, so that no query for the terminal's own is waiting to
             * swallow the keys. */
            char *argv[] = {"cbirds",
                            "--unlock-fps",
                            "--frames",
                            "150",
                            "--color",
                            "ember",
                            "-n",
                            "60",
                            "--seed",
                            "3",
                            (char *)runs[which][0],
                            (char *)runs[which][1],
                            NULL};
            _exit(cbirds_application_main(runs[which][1] != NULL ? 12 : 11, argv));
        }
        close(terminal);
        /* The keys wait until the program has taken the screen: putting the terminal
         * in raw mode throws away what was typed before. */
        int status = 0, typed = 0;
        size_t seen = 0;
        char drain[4096], screen_taken[] = ALT_SCREEN_ON;
        for (;;) {
            if (readable_within(master, 100) > 0) {
                ssize_t got = read(master, drain, sizeof(drain));
                if (got <= 0) break;
                for (ssize_t i = 0; i < got && !typed; i++) {
                    seen = drain[i] == screen_taken[seen] ? seen + 1 : (drain[i] == '\033' ? 1 : 0);
                    if (seen == sizeof(screen_taken) - 1) {
                        assert(write(master, "+++++++-+", 9) == 9);
                        typed = 1;
                    }
                }
            }
            if (waitpid(child, &status, WNOHANG) == child) {
                child = -1;
                break;
            }
        }
        assert(typed);
        if (child > 0) assert(waitpid(child, &status, 0) == child);
        assert(WIFEXITED(status) && WEXITSTATUS(status) == EXIT_SUCCESS);
        close(master);
    }
    assert(unlink(picture) == 0);
    reset_sign_state();
}

static void test_presets_set_every_notch(void) {
    reset_test_config();
    /* Each preset names a whole look, so every one of them has to move at least
     * one notch off the default, or it is not a look. */
    for (int i = 0; i < PRESET_COUNT; i++) {
        apply_preset(i);
        int notches[] = {config.boundary_notch, config.separation_notch, config.alignment_notch,
                         config.vision_notch};
        int moved = 0;
        for (size_t k = 0; k < sizeof(notches) / sizeof(*notches); k++) {
            assert(notches[k] >= 0 && notches[k] <= LEGEND_BAR_CELLS);
            if (notches[k] != DEFAULT_NOTCH) moved = 1;
        }
        assert(moved);
        /* And the derived values follow, as they do for a keypress. */
        assert(config.boundary >= BOUNDARY_MIN && config.boundary <= BOUNDARY_MAX);
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
    /* --perception takes real units and snaps. A value that is already on the
     * grid has to come back as itself, or a dotfile could not express one. */
    for (int notch = 0; notch <= LEGEND_BAR_CELLS; notch++) {
        int pixels = notch_integer(notch, MIN_VISION_RADIUS, MAX_VISION_RADIUS);
        assert(notch_for_integer(pixels, MIN_VISION_RADIUS, MAX_VISION_RADIUS) == notch);
    }
    /* And anything between snaps to the nearer of the two. */
    assert(notch_for_integer(MIN_VISION_RADIUS, MIN_VISION_RADIUS, MAX_VISION_RADIUS) == 0);
    assert(notch_for_integer(MAX_VISION_RADIUS, MIN_VISION_RADIUS, MAX_VISION_RADIUS) ==
           LEGEND_BAR_CELLS);
    assert(notch_for_integer(50, MIN_VISION_RADIUS, MAX_VISION_RADIUS) ==
           10); /* 52 is notch ten. */
}

static void test_the_pointer_moves_the_flock(void) {
    reset_test_config();
    legend_enabled = 0;
    apply_screen_size(200, 50, 200 * 8, 50 * 16);
    mouse.present = 1;
    mouse.x = 800;
    mouse.y = 400;

    /* A bird to the right of the pointer flees further right, straight along
     * the line between them. */
    const bird_t east = {.x = 800 + 40, .y = 400};
    vector_t away = pointer_vector(&east);
    assert(away.x > 0 && fabs(away.y) < 1e-12);

    /* It falls off with distance and stops at its reach, so the flock bends
     * around the pointer and closes behind it rather than bouncing off. */
    const bird_t near = {.x = 800 + 10, .y = 400};
    const bird_t far = {.x = 800 + 100, .y = 400};
    const bird_t beyond = {.x = 800 + MOUSE_REACH + 1, .y = 400};
    assert(pointer_vector(&near).x > pointer_vector(&far).x);
    assert(pointer_vector(&beyond).x == 0 && pointer_vector(&beyond).y == 0);

    /* Never having seen the pointer is no pointer. */
    mouse.present = 0;
    assert(pointer_vector(&east).x == 0);
    mouse.present = 1;

    /* And it overrules a flock that wants to go the other way. */
    enum { BIRD_COUNT = 30 };
    bird_t birds[BIRD_COUNT];
    spatial_grid_t grid;
    config.birds = BIRD_COUNT;
    for (int i = 0; i < BIRD_COUNT; i++) birds[i] = (bird_t){.x = 840, .y = 400, .direction = M_PI};
    assert(spatial_grid_init(&grid, SPATIAL_CELL_SIZE) == SPATIAL_GRID_OK);
    assert(spatial_grid_prepare(&grid, screen.width, screen.height, BIRD_COUNT) == SPATIAL_GRID_OK);
    assert(spatial_grid_build(&grid, BIRD_COUNT, read_bird_position, birds) == SPATIAL_GRID_OK);
    assert(cos(flock_direction(birds, &grid, 0)) > 0); /* Away, not with them. */
    mouse.present = 0;
    assert(cos(flock_direction(birds, &grid, 0)) < 0); /* With them again. */
    mouse.present = 1;
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
        /* Ink is built the same way, for a dark terminal. */
        if (palette_is_ink()) {
            static const uint8_t TEXT[3] = {229, 229, 229}, GROUND[3] = {18, 18, 24};
            build_the_ink(TEXT, GROUND, SKY_DIM);
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

/* The help for --color is written out by hand, as --preset's and --shape's are,
 * so this is what keeps it the list of ramps: every name in the table, in the
 * table's order, and nothing else. Every name reaches its own ramp, which a
 * name used twice would not, because the parser takes the first it finds. And
 * every ramp has five shades, because MAX_FLOCKS was measured on five. */
static void test_the_help_names_every_ramp(void) {
    char error[160];
    const option_t *color = NULL;
    reset_test_config();
    name_the_palettes();
    for (int i = 0; i < OPTION_COUNT; i++)
        if (strcmp(OPTIONS[i].name, "color") == 0) color = &OPTIONS[i];
    assert(color != NULL && color->names == PALETTE_NAMES);

    const char *listed = color->help;
    for (int i = 0; i < PALETTE_COUNT; i++) {
        size_t length = strlen(PALETTES[i].name);
        if (i > 0) {
            assert(strncmp(listed, ", ", 2) == 0);
            listed += 2;
        }
        assert(strncmp(listed, PALETTES[i].name, length) == 0);
        listed += length;

        char *argv[] = {"cbirds", "--color", (char *)PALETTES[i].name, NULL};
        config.palette = -1;
        assert(options_parse(OPTIONS, OPTION_COUNT, 3, argv, error, sizeof(error)) == OPTIONS_OK);
        assert(config.palette == i);
        assert(PALETTES[i].shades == 5);
    }
    assert(*listed == '\0');
    reset_test_config();
}

/* A ramp may be any colour it likes, as long as none of it fades into a dark
 * terminal. The dimmest shade a ramp has shipped with is matrix's last green, at
 * 2.6 against black; paper, the greys taken out for being unreadable, ended on
 * 1.6. Nothing may go below 2.5, just under that green. */
static void test_no_ramp_fades_into_a_black_terminal(void) {
    static const uint8_t BLACK[3] = {0, 0, 0};
    for (config.palette = 0; config.palette < PALETTE_COUNT; config.palette++) {
        /* Learned from the terminal, and kept off its ground by a test of its own. */
        if (palette_follows_the_theme() || palette_is_ink()) continue;
        for (int shade = 0; shade < palette()->shades; shade++)
            assert(contrast_between(palette()->tints[shade], BLACK) >= 2.5);
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

/* Braille unless something else is asked for: no terminal is guessed at, and
 * the sprites are there for whoever runs Kitty or Ghostty and asks for them.
 * "auto" is gone, and the names are exactly the four renderers. */
static void test_braille_unless_asked(void) {
    int saved = render_mode;
    render_mode = RENDER_UNSET;
    assert(live_render_mode() == RENDER_BRAILLE);
    static const int ASKED[] = {RENDER_KITTY, RENDER_BRAILLE, RENDER_SEXTANTS, RENDER_BLOCKS};
    for (size_t i = 0; i < sizeof(ASKED) / sizeof(*ASKED); i++) {
        render_mode = ASKED[i];
        assert(live_render_mode() == ASKED[i]);
    }

    char error[160];
    render_mode = RENDER_UNSET;
    char *kitty[] = {"cbirds", "--render", "kitty", NULL};
    assert(options_parse(OPTIONS, OPTION_COUNT, 3, kitty, error, sizeof(error)) == OPTIONS_OK);
    assert(render_mode == RENDER_KITTY);
    char *automatic[] = {"cbirds", "--render", "auto", NULL};
    assert(options_parse(OPTIONS, OPTION_COUNT, 3, automatic, error, sizeof(error)) ==
           OPTIONS_ERROR);
    assert(strstr(error, "kitty, braille, sextants, blocks") != NULL);
    render_mode = saved;
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
    birds[1] =
        (bird_t){.x = 104, .y = 103, .direction = 2.9, .shade = 4}; /* On top, another shade. */
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
    /* And underneath, every pixel of ink on the canvas the cells were read from
     * is one bird's own colour: where two birds overlap the more opaque one takes
     * the pixel rather than blending, or a shared cell would be a colour that is
     * neither bird's and different in every cell. */
    for (size_t px = 0; px < (size_t)text_canvas.width * (size_t)text_canvas.height; px++) {
        const uint8_t *pixel = &text_canvas.pixels[px * 4];
        if (pixel[3] == 0) continue;
        int own = memcmp(pixel, hawk_colour(), 3) == 0;
        for (int shade = 0; shade < palette_shades() && !own; shade++)
            own = memcmp(pixel, palette()->tints[shade], 3) == 0;
        assert(own);
    }
    /* Every colour on the screen is one of the palette's own, or the hawk's: a
     * cell two birds share is not painted a third colour that is neither. */
    for (const char *at = graphics.buffer; (at = strstr(at, "\033[38;2;")) != NULL; at++) {
        int r, g, b;
        assert(sscanf(at, "\033[38;2;%d;%d;%dm", &r, &g, &b) == 3);
        uint8_t seen[3] = {(uint8_t)r, (uint8_t)g, (uint8_t)b};
        int known = memcmp(seen, hawk_colour(), 3) == 0;
        for (int shade = 0; shade < palette_shades() && !known; shade++)
            known = memcmp(seen, palette()->tints[shade], 3) == 0;
        assert(known);
    }
    assert(strstr(graphics.buffer, "\033[2J") == NULL); /* The rule holds here too. */

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
    char path[600];
    scratch_file(path, sizeof(path), "text_snapshot.png");
    assert(write_snapshot(path, birds));
    /* And a disk that is full says so, even when it only says it on close. */
    if (access("/dev/full", W_OK) == 0) assert(!write_snapshot("/dev/full", birds));
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

/* Under a text renderer the GIF is of the cells, painted as a terminal shows
 * them: dots on the ground, in the palette's colours, and nothing else. */
static void test_a_text_renderer_records_its_cells(void) {
    char path[600];
    scratch_file(path, sizeof(path), "record_braille.gif");
    reset_test_config();
    config.birds = 60;
    config.palette = palette_named("ember");
    render_mode = RENDER_BRAILLE;
    record_path = path;
    record_fps = 20;
    record_seconds = 1;
    record_columns = 60;
    record_rows = 20;
    fflush(stdout);
    int saved = dup(STDOUT_FILENO);
    assert(freopen("/dev/null", "w", stdout) != NULL);
    int status = run_recording();
    fflush(stdout);
    dup2(saved, STDOUT_FILENO);
    close(saved);
    clearerr(stdout);
    assert(status == EXIT_SUCCESS);

    FILE *file = fopen(path, "rb");
    assert(file != NULL);
    uint8_t header[10];
    assert(fread(header, 1, sizeof(header), file) == sizeof(header));
    assert(memcmp(header, "GIF89a", 6) == 0);
    assert((header[6] | header[7] << 8) == 60 * DEFAULT_CELL_WIDTH);
    assert((header[8] | header[9] << 8) == 20 * DEFAULT_CELL_HEIGHT);
    /* Every frame is a picture: twenty image descriptors for twenty frames. */
    int descriptors = 0, c;
    while ((c = fgetc(file)) != EOF)
        if (c == 0x2C) descriptors++;
    fclose(file);
    remove(path);
    assert(descriptors >= 20);

    record_path = NULL;
    record_fps = 25;
    record_seconds = 6;
    render_mode = RENDER_KITTY;
    reset_test_config();
}

/* A recording named .cast is text: an asciinema file, a JSON header and a line
 * of escape text per frame, playable in any terminal and a fraction of a GIF. */
static void test_a_cast_is_the_flock_as_text(void) {
    char path[600];
    scratch_file(path, sizeof(path), "record.cast");
    reset_test_config();
    config.birds = 60;
    config.palette = palette_named("ember");
    record_path = path;
    record_fps = 20;
    record_seconds = 1;
    record_columns = 60;
    record_rows = 20;
    fflush(stdout);
    int saved = dup(STDOUT_FILENO);
    assert(freopen("/dev/null", "w", stdout) != NULL);
    int status = run_recording();
    fflush(stdout);
    dup2(saved, STDOUT_FILENO);
    close(saved);
    clearerr(stdout);
    assert(status == EXIT_SUCCESS);

    FILE *file = fopen(path, "r");
    assert(file != NULL);
    static char line[1 << 16];
    /* The header names the version and the size of the terminal it was made for. */
    assert(fgets(line, sizeof(line), file) != NULL);
    assert(strstr(line, "{\"version\": 2, \"width\": 60, \"height\": 20,") == line);
    int events = 0, braille = 0, raw_escapes = 0;
    while (fgets(line, sizeof(line), file) != NULL) {
        events++;
        assert(line[0] == '[');
        assert(strstr(line, ", \"o\", \"") != NULL);
        /* Escape characters are spelled out for JSON; the braille is left as the
         * UTF-8 it is, which is what keeps the file readable and small. */
        assert(strstr(line, "\\u001b") != NULL);
        for (const char *c = line; *c; c++) {
            if (*c == '\033') raw_escapes++;
            if ((unsigned char)c[0] == 0xE2 && ((unsigned char)c[1] & 0xFC) == 0xA0) braille++;
        }
    }
    fclose(file);
    remove(path);
    assert(raw_escapes == 0);
    assert(events == 20 + 2); /* Twenty frames, an opening and a closing. */
    assert(braille > 60);     /* Sixty birds leave more than sixty dots behind. */

    record_path = NULL;
    record_fps = 25;
    record_seconds = 6;
    render_mode = RENDER_KITTY;
    reset_test_config();
}

/* The catalogue of sprite sets: every kind of thing drawn has its own run of
 * images, none of them overlap, and every renderer reaches the same one. */
static void test_the_sprite_catalogue_has_a_place_for_everything(void) {
    reset_test_config();
    config.palette = palette_named("ember");
    int shades = palette_shades();
    int seen[MAX_SPRITE_SETS] = {0};
    for (int shade = 0; shade < shades; shade++) {
        for (int wing = 0; wing < WING_PHASES; wing++) seen[flock_set(shade, wing, 0)]++;
        seen[flock_set(shade, 0, 1)]++;
    }
    for (int wing = 0; wing < WING_PHASES; wing++) seen[hawk_set(wing)]++;
    for (int step = 0; step < TRAIL_LENGTH; step++) seen[trail_set(step)]++;
    for (int wing = 0; wing < WING_PHASES; wing++) seen[alarm_set(wing)]++;
    for (int set = 0; set < sprite_set_count(); set++) assert(seen[set] == 1);
    assert(sprite_set_count() <= MAX_SPRITE_SETS);
    /* Ids are one based and run one set after another without a gap. */
    assert(set_image_id(0, 0) == 1);
    assert(set_image_id(1, 0) == ROTATION_FRAMES + 1);
    bird_t near = {.shade = 2, .wing = 1, .layer = 0, .frame = 7};
    bird_t far = {.shade = 2, .wing = 1, .layer = 1, .frame = 7};
    assert(sprite_image_id(&near) == set_image_id(flock_set(2, WING_SEQUENCE[1], 0), 7));
    assert(sprite_image_id(&far) == set_image_id(flock_set(2, 0, 1), 7));
    assert(sprite_image_id(&near) != sprite_image_id(&far));

    /* And rasterising fills every set: a wing phase, a far bird, a hawk phase and
     * a step of tail each have their pictures, the far and the tails smaller. */
    static png_image_t frames[ROTATION_FRAMES * MAX_SPRITE_SETS];
    assert(rasterise_sprites(frames) == PNG_OK);
    for (int set = 0; set < sprite_set_count(); set++)
        for (int frame = 0; frame < ROTATION_FRAMES; frame++)
            assert(frames[set * ROTATION_FRAMES + frame].pixels != NULL);
    assert(frames[flock_set(0, 0, 1) * ROTATION_FRAMES].width < config.bird_size);
    assert(frames[trail_set(0) * ROTATION_FRAMES].width < config.bird_size);
    assert(frames[hawk_set(0) * ROTATION_FRAMES].width == hawk_sprite_size());
    /* A folded wing has less ink than a spread one. */
    long spread = 0, folded = 0;
    const png_image_t *open = &frames[flock_set(0, 0, 0) * ROTATION_FRAMES];
    const png_image_t *shut = &frames[flock_set(0, WING_PHASES - 1, 0) * ROTATION_FRAMES];
    for (int i = 0; i < open->width * open->height; i++) spread += open->pixels[i * 4 + 3];
    for (int i = 0; i < shut->width * shut->height; i++) folded += shut->pixels[i * 4 + 3];
    assert(folded < spread * 3 / 4);
    /* Each step of tail is fainter than the one before it. */
    long ink[TRAIL_LENGTH];
    for (int step = 0; step < TRAIL_LENGTH; step++) {
        const png_image_t *ghost = &frames[trail_set(step) * ROTATION_FRAMES];
        ink[step] = 0;
        for (int i = 0; i < ghost->width * ghost->height; i++)
            ink[step] += ghost->pixels[i * 4 + 3];
        if (step > 0) assert(ink[step] < ink[step - 1]);
    }
    /* And a far bird is dimmer than a near one of the same shade. */
    const png_image_t *near_bird = &frames[flock_set(0, 0, 0) * ROTATION_FRAMES];
    const png_image_t *far_bird = &frames[flock_set(0, 0, 1) * ROTATION_FRAMES];
    int near_sum = 0, far_sum = 0, counted = 0;
    for (int i = 0; i < near_bird->width * near_bird->height && !counted; i++)
        if (near_bird->pixels[i * 4 + 3] == 255) {
            near_sum = near_bird->pixels[i * 4] + near_bird->pixels[i * 4 + 1] +
                       near_bird->pixels[i * 4 + 2];
            counted = 1;
        }
    counted = 0;
    for (int i = 0; i < far_bird->width * far_bird->height && !counted; i++)
        if (far_bird->pixels[i * 4 + 3] == 255) {
            far_sum =
                far_bird->pixels[i * 4] + far_bird->pixels[i * 4 + 1] + far_bird->pixels[i * 4 + 2];
            counted = 1;
        }
    assert(far_sum < near_sum);
    free_sprites(frames);
    reset_test_config();
}

/* Two planes: a far bird is smaller, slower, and neither sees nor is seen by a
 * near one, and the hawk hunts only the near sky. */
static void test_the_far_layer_is_another_sky(void) {
    enum { BIRD_COUNT = 30 };
    bird_t birds[BIRD_COUNT], snapshot[BIRD_COUNT];
    spatial_grid_t grid;

    reset_test_config();
    legend_enabled = 0;
    apply_screen_size(200, 50, 1600, 800);
    config.birds = BIRD_COUNT;
    /* A near bird heading east, on top of a crowd of far birds heading west. */
    birds[0] = (bird_t){.x = 800, .y = 400, .direction = 0, .layer = 0};
    for (int i = 1; i < BIRD_COUNT; i++)
        birds[i] = (bird_t){.x = 800, .y = 400, .direction = M_PI, .layer = 1};
    assert(spatial_grid_init(&grid, SPATIAL_CELL_SIZE) == SPATIAL_GRID_OK);
    assert(spatial_grid_prepare(&grid, screen.width, screen.height, BIRD_COUNT) == SPATIAL_GRID_OK);
    assert(spatial_grid_build(&grid, BIRD_COUNT, read_bird_position, birds) == SPATIAL_GRID_OK);
    measure_flocks(birds);
    assert(flock_direction(birds, &grid, 0) == 0.0); /* Not pushed, not aligned, nothing. */

    /* Slower: the same step for both planes is a different distance. */
    birds[1] = (bird_t){.x = 800, .y = 600, .direction = 0, .layer = 1};
    memcpy(snapshot, birds, sizeof(birds));
    assert(spatial_grid_build(&grid, BIRD_COUNT, read_bird_position, snapshot) == SPATIAL_GRID_OK);
    update_birds(birds, snapshot, &grid);
    double near_step = birds[0].x - 800, far_step = birds[1].x - 800;
    assert(near_step > 0 && far_step > 0);
    assert(fabs(far_step - near_step * FAR_PACE) < 1e-6);

    /* The hawk hunts only the near sky, and only the near sky fears it. */
    config.hawks = 1;
    place_hawks();
    hawks[0].x = 100;
    hawks[0].y = 400;
    birds[0] = (bird_t){.x = 700, .y = 400, .layer = 0}; /* Near, further away. */
    birds[1] = (bird_t){.x = 150, .y = 400, .layer = 1}; /* Far, right beside it. */
    for (int i = 2; i < BIRD_COUNT; i++) birds[i] = (bird_t){.x = 1500, .y = 700, .layer = 1};
    assert(nearest_bird(birds, hawks[0].x, hawks[0].y, 0, 0, 0) == 0);
    assert(hawk_vector(&birds[1]).x == 0 && hawk_vector(&birds[1]).y == 0);
    birds[1].layer = 0;
    assert(hawk_vector(&birds[1]).x > 0);

    /* One plane unless asked: under --depth a bird is born into one or the other. */
    config.hawks = 0;
    int far = 0;
    seed_random(3);
    deep_look = 1;
    for (int i = 0; i < BIRD_COUNT; i++) {
        place_one_bird(&birds[i], i);
        far += birds[i].layer;
    }
    assert(far > 0 && far < BIRD_COUNT);
    deep_look = 0;
    for (int i = 0; i < BIRD_COUNT; i++) {
        place_one_bird(&birds[i], i);
        assert(birds[i].layer == 0);
    }
    spatial_grid_destroy(&grid);
    legend_enabled = 1;
    reset_test_config();
}

/* A seed is the same flock on every system: the numbers are the program's own,
 * and they are glibc's, so what was recorded on Linux before still is. */
static void test_a_seed_draws_the_same_numbers_everywhere(void) {
    /* What glibc's rand() gives after srand(1), which is also srand(0). */
    static const uint32_t FIRST[] = {1804289383u, 846930886u, 1681692777u, 1714636915u,
                                     1957747793u, 424238335u, 719885386u,  1649760492u};
    for (unsigned seed = 0; seed <= 1; seed++) {
        seed_random(seed);
        for (size_t i = 0; i < sizeof(FIRST) / sizeof(*FIRST); i++)
            assert(next_random() == FIRST[i]);
    }
    seed_random(42);
    double lowest = 1, highest = 0;
    for (int i = 0; i < 100000; i++) {
        double unit = random_unit();
        assert(unit >= 0 && unit <= 1);
        if (unit < lowest) lowest = unit;
        if (unit > highest) highest = unit;
    }
    assert(lowest < 0.001 && highest > 0.999);
#ifdef __GLIBC__
    /* And where there is a glibc to ask, all of it, for seeds either side of
     * the one that no longer fits a signed word. */
    static const unsigned SEEDS[] = {2, 33, 20260911u, 2147483647u, 2147483648u, 4294967295u};
    for (size_t s = 0; s < sizeof(SEEDS) / sizeof(*SEEDS); s++) {
        srand(SEEDS[s]);
        seed_random(SEEDS[s]);
        for (int i = 0; i < 10000; i++) assert(next_random() == (uint32_t)rand());
    }
#endif
}

/* Wings beat at WING_HZ whatever the frame rate, out and back through the
 * sequence, and a bird sometimes stops to glide with them out. */
static void test_wings_beat_and_sometimes_glide(void) {
    reset_test_config();
    set_frame_seconds(1.0 / FRAME_RATE);
    bird_t bird = {.wing = 0, .wing_clock = 0, .gliding = 0};
    /* One beat is WING_CYCLE phases; at sixty frames a second and six beats a
     * second, that is ten frames a beat. A hundred seconds, so that what is
     * counted is the rule and not the luck of one short run. */
    int phases_seen[WING_CYCLE] = {0};
    seed_random(1);
    int frames = 0, beats = 0, glided = 0, glides = 0;
    for (frames = 0; frames < 6000; frames++) {
        int before = bird.wing;
        int was_gliding = bird.gliding > 0;
        beat_wings(&bird);
        phases_seen[bird.wing % WING_CYCLE]++;
        if (bird.gliding > 0) glided++;
        if (bird.gliding > 0 && !was_gliding) glides++;
        if (before == WING_CYCLE - 1 && bird.wing == 0) beats++;
        /* Never a jump of more than one phase in a frame. */
        assert(bird.wing == before || bird.wing == (before + 1) % WING_CYCLE ||
               (bird.gliding > 0 && bird.wing == 0));
    }
    for (int p = 0; p < WING_CYCLE; p++) assert(phases_seen[p] > 0);
    /* Six a second whenever it is not gliding: a beat every ten flapping frames.
     * A glide starts on the frame that completes a beat, so each one can shift
     * the count by a frame. */
    int flapping = frames - glided;
    assert(abs(beats * 10 - flapping) <= 10 + glides);
    assert(glides > 0);          /* It glided at some point... */
    assert(glided < frames / 2); /* ...and mostly did not. */
    /* The sequence goes out, half, folded, half: the picture for phase 3 is the
     * same as for phase 1. */
    assert(WING_SEQUENCE[1] == WING_SEQUENCE[3]);
    assert(WING_SEQUENCE[0] == 0 && WING_SEQUENCE[2] == WING_PHASES - 1);
    reset_test_config();
}

/* A GIF has no panel in it, so it must not have a hole where one would be — and
 * it is drawn without a terminal, so the palette that asks the terminal what
 * colours it uses has to fall back to one that has colours in it. */
static void test_recording_gives_the_whole_frame_to_the_flock(void) {
    char path[600];
    scratch_file(path, sizeof(path), "record.gif");
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
        /* Sent a room apart, which is the eighth notch now and was the default
         * until flocks sent home were found flying round it. */
        config.avoid_notch = 8;
        apply_notches();
        apply_screen_size(100, 28, 800, 448);
        config.birds = BIRD_COUNT;
        config.flocks = flocks;
        seed_random(1); /* The glides draw on it, so the run is pinned. */

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

/* The intro is three seconds of the flock's name, unless somebody presses a
 * key; a pointer report is not a key, or moving the mouse would end it. */
static void test_a_key_ends_the_intro(void) {
    reset_test_config();
    legend_enabled = 1;
    apply_screen_size(200, 50, 1600, 800);
    begin_the_intro();
    assert(formation.writing);
    assert(formation.until == INTRO_SECONDS);
    assert(feed_input("\033[<35;10;5M") == 1);
    assert(formation.writing);
    assert(feed_input(" ") == 1);
    assert(!formation.writing);
    paused = 0;
    mouse.present = 0;
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

/* The speed slider changes how fast the flock flies and not the shape of what it
 * flies: the step and both turn limits scale together, so a turning circle is the
 * same number of pixels at every notch. The fourth notch is the shipped flock
 * exactly, not nearly, and the way out flies at it whatever the slider says. */
static void test_the_speed_slider_flies_the_same_path_faster(void) {
    reset_test_config();
    apply_screen_size(200, 60, 1600, 960);
    set_frame_seconds(1.0 / FRAME_RATE);
    assert(config.pace == 1.0);
    assert(config.speed == DEFAULT_SPEED && config.base_speed == DEFAULT_SPEED);
    double bird_radius = config.speed / turn_limit();
    double hawk_radius = hawk_turning_radius();
    double shortest = 0, longest = 0;
    for (int n = 0; n <= LEGEND_BAR_CELLS; n++) {
        config.pace_notch = n;
        apply_notches();
        assert(config.base_speed == DEFAULT_SPEED);
        assert(fabs(config.speed - DEFAULT_SPEED * config.pace) < 1e-9);
        assert(fabs(config.speed / turn_limit() - bird_radius) < 1e-9);
        assert(fabs(hawk_turning_radius() - hawk_radius) < 1e-9);
        if (n == 0) shortest = config.speed;
        if (n == LEGEND_BAR_CELLS) longest = config.speed;
    }
    assert(shortest < DEFAULT_SPEED / 4.0 && longest > DEFAULT_SPEED * 2.5);

    /* The way out is the one thing that ignores it. */
    for (int n = 0; n <= LEGEND_BAR_CELLS; n += LEGEND_BAR_CELLS) {
        bird_t leaving = {.x = 800, .y = 500, .direction = 0};
        config.pace_notch = n;
        apply_notches();
        config.birds = 1;
        fly_away(&leaving);
        assert(leaving.y == 500 - DEFAULT_SPEED && leaving.x == 800);
        assert(fabs(leaving.direction - 3 * M_PI / 2) < 1e-12);
    }
    config.birds = 800;

    /* On a small screen the pace multiplies what the screen allows, so the top
     * of the bar is not a row of notches that do nothing. */
    apply_screen_size(40, 14, 320, 224);
    config.pace_notch = DEFAULT_NOTCH;
    apply_notches();
    double capped = config.speed;
    config.pace_notch = LEGEND_BAR_CELLS;
    apply_notches();
    assert(config.speed > capped * 2.5);

    /* The keys: one notch a press, the ends hold, 0 comes home, a preset leaves
     * it where it is because a look is not a speed. */
    int saved_preset = requested_preset;
    reset_test_config();
    assert(feed_input("V") == 1);
    assert(config.pace_notch == DEFAULT_NOTCH + 1 && fabs(config.pace - 1.2) < 1e-12);
    assert(feed_input("vv") == 1);
    assert(config.pace_notch == DEFAULT_NOTCH - 1);
    assert(feed_input("vvvvvvvvvvvvvvvv") == 1);
    assert(config.pace_notch == 0 && config.pace == PACE_FLOOR);
    assert(feed_input("0") == 1);
    assert(config.pace == DEFAULT_PACE);
    assert(feed_input("VVV\t") == 1); /* From the home notch, which 0 went back to. */
    assert(config.pace_notch == DEFAULT_PACE_NOTCH + 3);
    requested_preset = saved_preset;
    reset_test_config();
}

/* A notch on the line, like every other slider, and a preset given before it
 * does not undo it. */
/* Without --size, a bird is 30 pixels under every renderer; --size, when given,
 * is kept; and a hawk stays twice a bird at the largest birds too. */
static void test_the_default_size_follows_the_renderer(void) {
    int saved_render = render_mode;
    static const int MODES[] = {RENDER_UNSET, RENDER_KITTY, RENDER_BRAILLE, RENDER_SEXTANTS,
                                RENDER_BLOCKS};
    for (size_t k = 0; k < sizeof(MODES) / sizeof(MODES[0]); k++) {
        render_mode = MODES[k];
        config.bird_size = 0;
        settle_the_bird_size();
        assert(config.bird_size == DEFAULT_BIRD_SIZE && DEFAULT_BIRD_SIZE == 30);
        assert(hawk_sprite_size() == 2 * DEFAULT_BIRD_SIZE);
    }
    render_mode = RENDER_BRAILLE;
    config.bird_size = 60; /* as if --size 60 had been given */
    settle_the_bird_size();
    assert(config.bird_size == 60);
    config.bird_size = MAX_BIRD_SIZE;
    assert(hawk_sprite_size() == 2 * MAX_BIRD_SIZE);
    render_mode = saved_render;
    reset_test_config();
}

static void test_the_speed_is_a_flag(void) {
    char *argv[] = {"cbirds", "--preset", "storm", "--speed", "9", NULL};
    int saved_preset = requested_preset;

    reset_test_config();
    read_options(5, argv);
    assert(config.pace_notch == 9 && fabs(config.pace - 2.0) < 1e-12);
    assert(config.boundary_notch == PRESETS[2].notch[0]);
    assert(config.separation_notch == PRESETS[2].notch[1]);
    requested_preset = saved_preset;
    reset_test_config();
}

/* A hawk's hold on a chase and its run out of the flock are distances written as
 * frames, so they run on flight time: at twice the pace they are over twice as
 * soon, and cover the same ground. */
static void test_a_hawk_holds_a_chase_for_a_distance(void) {
    bird_t birds[1] = {{.x = 1500, .y = 700, .direction = 0}};
    static const int NOTCHES[] = {DEFAULT_NOTCH, 9};

    reset_test_config();
    legend_enabled = 0;
    apply_screen_size(200, 50, 1600, 800);
    config.birds = 1;
    config.hawks = 1;
    for (size_t i = 0; i < sizeof(NOTCHES) / sizeof(*NOTCHES); i++) {
        config.pace_notch = NOTCHES[i];
        apply_notches();
        set_frame_seconds(1.0 / FRAME_RATE);
        hawks[0] = (hawk_t){.x = 300, .y = 300, .prey = 0, .commitment = 0.5};
        hunt(birds);
        assert(hawks[0].prey == 0); /* Still committed, so still after it. */
        assert(fabs(hawks[0].commitment - (0.5 - config.pace / FRAME_RATE)) < 1e-12);
        hawks[0] = (hawk_t){.x = 300, .y = 300, .prey = -1, .passing = 0.2};
        hunt(birds);
        assert(fabs(hawks[0].passing - (0.2 - config.pace / FRAME_RATE)) < 1e-12);
    }
    config.hawks = 0;
    legend_enabled = 1;
    reset_test_config();
}

/* How many of the birds are outside the frame, over a run long enough for the
 * flock to find the edges. */
static double share_off_the_screen(int pace_notch, int columns, int rows, int panel) {
    enum { BIRDS = 300, SECONDS = 12 };
    spatial_grid_t grid;
    bird_t *birds = calloc(BIRDS, sizeof(*birds));
    bird_t *snapshot = malloc(sizeof(*snapshot) * BIRDS);
    long outside = 0, counted = 0;

    assert(birds != NULL && snapshot != NULL);
    reset_test_config();
    config.birds = BIRDS;
    config.pace_notch = pace_notch;
    legend_enabled = panel;
    apply_notches();
    apply_screen_size(columns, rows, columns * 8, rows * 16);
    set_frame_seconds(1.0 / FRAME_RATE);
    assert(spatial_grid_init(&grid, SPATIAL_CELL_SIZE) == SPATIAL_GRID_OK);
    assert(spatial_grid_prepare(&grid, screen.width, screen.height, BIRDS) == SPATIAL_GRID_OK);
    seed_random(7);
    initialize_birds(birds);
    for (int frame = 0; frame < SECONDS * FRAME_RATE; frame++) {
        memcpy(snapshot, birds, sizeof(*birds) * BIRDS);
        assert(spatial_grid_build(&grid, BIRDS, read_bird_position, snapshot) == SPATIAL_GRID_OK);
        fly(birds, snapshot, &grid);
        assert(frame_seconds == 1.0 / FRAME_RATE); /* Handed back as it was lent. */
        if (frame < 2 * FRAME_RATE) continue;      /* Out of where it was put first. */
        for (int i = 0; i < BIRDS; i++) {
            counted++;
            if (birds[i].x < 0 || birds[i].y < 0 || birds[i].x >= screen.width ||
                birds[i].y >= screen.height)
                outside++;
            assert(!sprite_overlaps_legend(birds[i].x, birds[i].y));
        }
    }
    spatial_grid_destroy(&grid);
    free(snapshot);
    free(birds);
    legend_enabled = 1;
    reset_test_config();
    return (double)outside / counted;
}

/* Flown in one step, the top of the bar crossed a small terminal's edge band in a
 * frame and put one bird in ten off the screen, against one in forty at pace one.
 * Flown in steps no longer than pace one's, it keeps the flock the default keeps. */
static void test_a_fast_flock_is_flown_in_steps(void) {
    spatial_grid_t grid;
    bird_t bird, snapshot;

    /* A lone bird in open sky covers the whole of a fast frame, in three steps. */
    reset_test_config();
    legend_enabled = 0;
    apply_screen_size(200, 60, 1600, 960);
    config.birds = 1;
    config.pace_notch = LEGEND_BAR_CELLS;
    apply_notches();
    set_frame_seconds(1.0 / FRAME_RATE);
    double frame_step = config.speed;
    assert(spatial_grid_init(&grid, SPATIAL_CELL_SIZE) == SPATIAL_GRID_OK);
    assert(spatial_grid_prepare(&grid, screen.width, screen.height, 1) == SPATIAL_GRID_OK);
    bird = (bird_t){.x = 800, .y = 480, .direction = 0};
    snapshot = bird;
    assert(spatial_grid_build(&grid, 1, read_bird_position, &snapshot) == SPATIAL_GRID_OK);
    fly(&bird, &snapshot, &grid);
    assert(fabs(bird.x - (800 + frame_step)) < 1e-9 && fabs(bird.y - 480) < 1e-9);
    assert(config.speed == frame_step); /* The step is back to a whole frame's. */
    spatial_grid_destroy(&grid);
    legend_enabled = 1;

    static const int SIZES[][3] = {{200, 50, 0}, {80, 24, 0}, {76, 22, 1}};
    for (size_t i = 0; i < sizeof(SIZES) / sizeof(*SIZES); i++) {
        double shipped = share_off_the_screen(DEFAULT_NOTCH, SIZES[i][0], SIZES[i][1], SIZES[i][2]);
        double slow = share_off_the_screen(0, SIZES[i][0], SIZES[i][1], SIZES[i][2]);
        double fast = share_off_the_screen(LEGEND_BAR_CELLS, SIZES[i][0], SIZES[i][1], SIZES[i][2]);
        assert(fast <= shipped * 1.5 + 0.005);
        assert(slow <= shipped * 1.5 + 0.005);
    }
    reset_test_config();
}

/* The avoidance slider is on the panel only when there is another flock to
 * avoid, and its keys do nothing when there is not. The fourth notch is the
 * shipped flocks exactly: the room they have always kept and no wariness. */
static void test_the_avoidance_slider_needs_two_flocks(void) {
    char lines[LEGEND_MAX_ROWS][LEGEND_LINE_MAX];

    reset_test_config();
    apply_screen_size(80, 24, 80 * 8, 24 * 16);
    assert(legend_rows() == LEGEND_ROWS);
    assert(screen.legend_height == LEGEND_ROWS * screen.cell_height);
    build_legend(lines);
    for (int row = 0; row < LEGEND_ROWS; row++) assert(strstr(lines[row], "avoidance") == NULL);
    assert(feed_input("GGg") == 1);
    assert(config.avoid_notch == DEFAULT_NOTCH);

    config.flocks = 3;
    apply_screen_size(80, 24, 80 * 8, 24 * 16);
    assert(legend_rows() == LEGEND_MAX_ROWS);
    assert(screen.legend_height == LEGEND_MAX_ROWS * screen.cell_height);
    build_legend(lines);
    for (int row = 0; row < LEGEND_MAX_ROWS; row++)
        assert(legend_cells(lines[row]) == LEGEND_COLUMNS);
    assert(strstr(lines[7], "avoidance") != NULL && strstr(lines[7], "g/G") != NULL);
    assert(strstr(lines[7], "1.00×") != NULL && filled_cells(lines[7]) == DEFAULT_NOTCH);
    assert(strstr(lines[LEGEND_MAX_ROWS - 3], "fps") != NULL);
    assert(strstr(lines[LEGEND_MAX_ROWS - 2], "quit") != NULL);
    assert(strstr(lines[LEGEND_MAX_ROWS - 1], "╰") == lines[LEGEND_MAX_ROWS - 1]);

    /* One notch a press, both ways, the ends hold, and 0 brings it home. */
    for (int expected = DEFAULT_NOTCH + 1; expected <= LEGEND_BAR_CELLS; expected++) {
        assert(feed_input("G") == 1);
        build_legend(lines);
        assert(filled_cells(lines[7]) == expected);
    }
    assert(feed_input("G") == 1);
    assert(config.avoid_notch == LEGEND_BAR_CELLS);
    build_legend(lines);
    assert(strstr(lines[7], "3.00×") != NULL);
    assert(feed_input("gggggggggggggggg") == 1);
    assert(config.avoid_notch == 0);
    build_legend(lines);
    assert(strstr(lines[7], "0.00×") != NULL && filled_cells(lines[7]) == 0);
    assert(feed_input("0") == 1);
    assert(config.avoid_notch == DEFAULT_NOTCH);

    /* What the notches mean: strangers kin by halves below the fourth, a room
     * and a wariness above it, growing, and none of any of it on it. */
    double room = -1, weight = -1, kinship = 2;
    for (int n = 0; n <= LEGEND_BAR_CELLS; n++) {
        config.avoid_notch = n;
        apply_notches();
        assert(config.avoid_room >= room && config.avoid_weight >= weight);
        assert(config.avoid_kinship < kinship || config.avoid_kinship == 0);
        room = config.avoid_room;
        weight = config.avoid_weight;
        kinship = config.avoid_kinship;
        if (n < DEFAULT_NOTCH)
            assert(config.avoid_kinship == ldexp(1.0, -n) && room == 0 && weight == 0);
        else
            assert(config.avoid_kinship == 0);
    }
    assert(config.avoid_room == 2.0 && config.avoid_weight == AVOID_WEIGHT_MAX);
    config.avoid_notch = DEFAULT_NOTCH;
    apply_notches();
    assert(config.avoid_room == 0 && config.avoid_weight == 0 && config.avoid_kinship == 0);
    assert(flock_room() == 0); /* Nobody is sent anywhere by default. */
    apply_screen_size(200, 60, 1600, 960);
    config.avoid_notch = 8;
    apply_notches();
    assert(flock_room() == 2.0 * FLOCK_LEASH);
    /* At the bottom the flocks are one flock: the same pace, no leash. */
    config.avoid_notch = 0;
    apply_notches();
    assert(flock_pace(2) == 1.0);
    reset_test_config();
}

static void test_the_avoidance_is_a_flag(void) {
    char *argv[] = {"cbirds", "--flocks", "3", "--avoidance", "10", NULL};
    int saved_preset = requested_preset;

    reset_test_config();
    read_options(5, argv);
    assert(config.flocks == 3 && config.avoid_notch == 10);
    assert(config.avoid_weight > 0 && config.avoid_room > 1);
    requested_preset = saved_preset;
    reset_test_config();
}

/* What three flocks do over a run: how often a bird has a stranger within its
 * sight, how far apart the two nearest flocks keep, how many birds are off the
 * screen. No bird may ever be inside the panel, which is a row taller now. */
typedef struct {
    double contact, gap, outside;
} flocks_apart_t;

static flocks_apart_t three_flocks_at(int avoid_notch, int columns, int rows, int panel) {
    enum { BIRDS = 300, SECONDS = 20 };
    spatial_grid_t grid;
    bird_t *birds = calloc(BIRDS, sizeof(*birds));
    bird_t *snapshot = malloc(sizeof(*snapshot) * BIRDS);
    long contact = 0, sampled = 0, outside = 0, counted = 0;
    double gap = 0;
    int gaps = 0;

    assert(birds != NULL && snapshot != NULL);
    reset_test_config();
    config.birds = BIRDS;
    config.flocks = 3;
    config.avoid_notch = avoid_notch;
    legend_enabled = panel;
    apply_notches();
    apply_screen_size(columns, rows, columns * 8, rows * 16);
    set_frame_seconds(1.0 / FRAME_RATE);
    for (int f = 0; f < MAX_FLOCKS; f++) flock_home_x[f] = flock_home_y[f] = 0;
    assert(spatial_grid_init(&grid, SPATIAL_CELL_SIZE) == SPATIAL_GRID_OK);
    assert(spatial_grid_prepare(&grid, screen.width, screen.height, BIRDS) == SPATIAL_GRID_OK);
    seed_random(5);
    initialize_birds(birds);
    for (int frame = 0; frame < SECONDS * FRAME_RATE; frame++) {
        memcpy(snapshot, birds, sizeof(*birds) * BIRDS);
        assert(spatial_grid_build(&grid, BIRDS, read_bird_position, snapshot) == SPATIAL_GRID_OK);
        fly(birds, snapshot, &grid);
        if (frame < 5 * FRAME_RATE) continue; /* Out of the columns they started in. */
        double least = -1;
        for (int f = 0; f < 3; f++)
            for (int g = f + 1; g < 3; g++) {
                double dx = flock_center_x[f] - flock_center_x[g];
                double dy = flock_center_y[f] - flock_center_y[g];
                double distance = sqrt(dx * dx + dy * dy);
                if (least < 0 || distance < least) least = distance;
            }
        gap += least;
        gaps++;
        for (int i = 0; i < BIRDS; i++) {
            counted++;
            if (birds[i].x < 0 || birds[i].y < 0 || birds[i].x >= screen.width ||
                birds[i].y >= screen.height)
                outside++;
            assert(!sprite_overlaps_legend(birds[i].x, birds[i].y));
        }
        if (frame % 20) continue;
        for (int i = 0; i < BIRDS; i += 3) {
            sampled++;
            for (int j = 0; j < BIRDS; j++) {
                if (birds[j].flock == birds[i].flock) continue;
                double dx = birds[i].x - birds[j].x, dy = birds[i].y - birds[j].y;
                if (dx * dx + dy * dy < config.vision_radius_squared) {
                    contact++;
                    break;
                }
            }
        }
    }
    spatial_grid_destroy(&grid);
    free(snapshot);
    free(birds);
    legend_enabled = 1;
    reset_test_config();
    return (flocks_apart_t){(double)contact / sampled, gap / gaps, (double)outside / counted};
}

/* Measured, three flocks on a roomy screen: a bird with a stranger in sight
 * 98% of the time at the bottom notch, where they are one flock, 23% shipped,
 * none at the top; the two nearest flocks 12, 164 and 481 pixels apart. And none of it by pushing
 * the flock off the screen or into the panel, on the smallest screen it opens on. */
static void test_flocks_avoid_each_other_as_much_as_asked(void) {
    flocks_apart_t mingle = three_flocks_at(0, 200, 50, 0);
    flocks_apart_t shipped = three_flocks_at(DEFAULT_NOTCH, 200, 50, 0);
    flocks_apart_t shun = three_flocks_at(LEGEND_BAR_CELLS, 200, 50, 0);
    assert(mingle.contact > shipped.contact && shipped.contact > shun.contact);
    assert(shun.contact < 0.02);
    assert(mingle.gap < shipped.gap && shipped.gap < shun.gap);
    assert(shun.outside <= shipped.outside * 3 + 0.005);

    flocks_apart_t small_shipped =
        three_flocks_at(DEFAULT_NOTCH, LEGEND_MIN_COLS, LEGEND_MIN_ROWS, 1);
    flocks_apart_t small_shun =
        three_flocks_at(LEGEND_BAR_CELLS, LEGEND_MIN_COLS, LEGEND_MIN_ROWS, 1);
    three_flocks_at(0, LEGEND_MIN_COLS, LEGEND_MIN_ROWS, 1);
    assert(small_shun.outside <= small_shipped.outside * 1.5 + 0.005);
}
/* --- The space ---------------------------------------------------------- */

/* Everything the flat flock draws on the way to a frame is a random number or a
 * float, and the floats are not the same on every compiler: fused multiply and add
 * moves the last bit on an arm64 build. The random numbers are integers. What the
 * flat flock has always drawn, in the order it drew them, is then the state the
 * generator is left in after a run, and these three came from the program as it
 * was before --3d. A new draw on any path the default takes, or one fewer, or one
 * reordered, moves it. */
static uint32_t flat_flock_leaves_the_generator_at(int flocks, int depth, int trails,
                                                   int hawk_count) {
    enum { BIRDS = 200, FRAMES = 90 };
    spatial_grid_t grid;
    bird_t *birds = calloc(BIRDS, sizeof(*birds));
    bird_t *snapshot = malloc(sizeof(*snapshot) * BIRDS);

    assert(birds != NULL && snapshot != NULL);
    reset_test_config();
    legend_enabled = 0;
    config.birds = BIRDS;
    config.palette = palette_named("ember");
    config.flocks = flocks;
    config.trails = trails;
    config.hawks = hawk_count;
    deep_look = depth;
    apply_screen_size(200, 50, 1600, 800);
    set_frame_seconds(1.0 / FRAME_RATE);
    seed_random(11);
    assert(spatial_grid_init(&grid, SPATIAL_CELL_SIZE) == SPATIAL_GRID_OK);
    assert(spatial_grid_prepare(&grid, screen.width, screen.height, BIRDS) == SPATIAL_GRID_OK);
    initialize_birds(birds);
    place_hawks();
    for (int frame = 0; frame < FRAMES; frame++) {
        memcpy(snapshot, birds, sizeof(*birds) * BIRDS);
        assert(spatial_grid_build(&grid, BIRDS, read_bird_position, snapshot) == SPATIAL_GRID_OK);
        fly(birds, snapshot, &grid);
    }
    spatial_grid_destroy(&grid);
    free(snapshot);
    free(birds);
    deep_look = 0;
    uint32_t left = next_random();
    reset_test_config();
    return left;
}

static void test_the_flat_flock_is_what_it_was(void) {
    assert(flat_flock_leaves_the_generator_at(1, 0, 0, 0) == 87498907u);
    assert(flat_flock_leaves_the_generator_at(3, 0, 1, 0) == 84104933u);
    assert(flat_flock_leaves_the_generator_at(1, 1, 1, 2) == 1926977605u);
    /* Without --3d the space is never touched: no birds in it, nothing flying. */
    assert(sky.flying == 0 && sky.world.birds == NULL && sky.views == NULL);
    /* And the flat flock's catalogue of pictures is laid out as it was. */
    reset_test_config();
    config.palette = palette_named("ember");
    assert(flock_set_count() == palette_shades() * (WING_PHASES + 1));
    assert(hawk_set(0) == 20 && trail_set(0) == 23 && alarm_set(0) == 26 &&
           sprite_set_count() == 29);
    assert(flock_set(3, 1, 0) == 3 * WING_PHASES + 1 && flock_set(3, 0, 1) == 5 * WING_PHASES + 3);
    assert(layer_count() == LAYERS);
    assert(layer_in_pass(0) == LAYERS - 1 && layer_in_pass(LAYERS - 1) == 0);
    /* The far plane under the near, and no panel to be behind. */
    kitty_graphics_placement_t placement;
    bird_t far = {.x = 5, .y = 5, .layer = 1}, near = {.x = 5, .y = 5, .layer = 0};
    apply_screen_size(80, 24, 640, 384);
    legend_enabled = 1;
    measure_legend();
    assert(bird_placement(&far, &placement) && placement.z_index == -1);
    assert(bird_placement(&near, &placement) && placement.z_index == 0);
    legend_enabled = 0;

    /* And a hawk is on the near plane with the birds, the last thing placed, and
     * not on the far one: the space puts it above its five sizes, which is a layer
     * the flat sky must never be told of. */
    kitty_graphics_t graphics;
    bird_t flock[2] = {{.x = 100, .y = 100}, {.x = 200, .y = 200, .layer = 1}};
    assert(kitty_graphics_init(&graphics, STDOUT_FILENO) == KITTY_GRAPHICS_OK);
    config.birds = 2;
    config.hawks = 1;
    hawks[0] = (hawk_t){.x = 300, .y = 300};
    assert(queue_render_frame(&graphics, flock) == KITTY_GRAPHICS_OK);
    const char *last = graphics.buffer;
    for (const char *at = strstr(last, "a=p"); at != NULL; at = strstr(at + 3, "a=p")) last = at;
    char *hawk_placement = strndup(last, (size_t)(strstr(last, "\033\\") - last));
    char hawk_id[40];
    snprintf(hawk_id, sizeof(hawk_id), "I=%u,", (unsigned)hawk_image_id(&hawks[0]));
    assert(hawk_placement != NULL && strstr(hawk_placement, hawk_id) != NULL);
    assert(strstr(hawk_placement, "z=") == NULL);
    free(hawk_placement);
    kitty_graphics_destroy(&graphics);
    memset(hawks, 0, sizeof(hawks));
    reset_test_config();
}

/* Runs the option reader in a child, as the program would run it, and says what
 * came of it: how it exited, what it printed to its error output, and the state
 * of the switches it was meant to set, as a number. */
static int options_read_in_a_child(char **argv, int argc, char *message, size_t size,
                                   int *exit_status) {
    int errors[2];
    assert(pipe(errors) == 0);
    fflush(NULL);
    pid_t child = fork();
    assert(child >= 0);
    if (child == 0) {
        alarm(20);
        close(errors[0]);
        if (dup2(errors[1], STDERR_FILENO) < 0) _exit(99);
        reset_test_config();
        read_options(argc, argv);
        _exit((sky_mode ? 1 : 0) | (deep_look ? 2 : 0) | (config.trails ? 4 : 0));
    }
    close(errors[1]);
    size_t length = 0;
    ssize_t got;
    while (length + 1 < size && (got = read(errors[0], message + length, size - 1 - length)) > 0)
        length += (size_t)got;
    message[length] = '\0';
    close(errors[0]);
    int status = 0;
    assert(waitpid(child, &status, 0) == child);
    assert(WIFEXITED(status));
    *exit_status = WEXITSTATUS(status);
    return 1;
}

/* --3d is a switch in the Look group, and it is a different sky: more birds, smaller,
 * in a dark ramp, unless something was asked for. What was asked for is kept,
 * including the number the flat sky would have given. */
static void test_three_d_has_defaults_of_its_own(void) {
    char *plain[] = {"cbirds", "--3d", NULL};
    char *asked[] = {"cbirds", "--3d", "-n", "800", "-s", "20", "--color", "ember", NULL};
    char *theme[] = {"cbirds", "--3d", "--color", "theme", NULL};

    reset_test_config();
    config.bird_size = 0; /* As a fresh process has it: not given. */
    read_options(2, plain);
    assert(sky_mode == 1);
    assert(config.birds == SKY_BIRDS && SKY_BIRDS == 2000);
    assert(config.palette == palette_named("ink"));
    assert(config.bird_size == 0); /* Settled when the renderer is known, below. */
    static const int TEXT[] = {RENDER_BRAILLE, RENDER_SEXTANTS, RENDER_BLOCKS};
    static const int SPRITES[] = {RENDER_KITTY, RENDER_UNSET};
    int saved_render = render_mode;
    for (size_t i = 0; i < sizeof(TEXT) / sizeof(*TEXT); i++) {
        render_mode = TEXT[i];
        config.bird_size = 0;
        settle_the_bird_size();
        assert(config.bird_size == SKY_TEXT_BIRD_SIZE);
    }
    for (size_t i = 0; i < sizeof(SPRITES) / sizeof(*SPRITES); i++) {
        render_mode = SPRITES[i];
        config.bird_size = 0;
        settle_the_bird_size();
        assert(config.bird_size == SKY_BIRD_SIZE);
    }
    render_mode = saved_render;

    reset_test_config();
    config.bird_size = 0;
    read_options(8, asked);
    assert(sky_mode == 1 && config.birds == 800 && config.bird_size == 20);
    assert(config.palette == palette_named("ember"));

    /* The terminal's own colours are a colour like any other, and asked for. */
    reset_test_config();
    read_options(4, theme);
    assert(config.palette == palette_named("theme"));

    /* Ink is a ramp like the rest, for either sky: asked for by name it is kept, and
     * the flat flock keeps everything else it has. The help for --3d says which
     * ramp it flies in, and it is the one it does. */
    char *ink[] = {"cbirds", "--color", "ink", NULL};
    reset_test_config();
    read_options(3, ink);
    assert(sky_mode == 0 && palette_is_ink() && config.birds == 800);
    for (int i = 0; i < OPTION_COUNT; i++)
        if (strcmp(OPTIONS[i].name, "3d") == 0) {
            assert(strstr(OPTIONS[i].help, "in ink") != NULL);
            assert(strstr(OPTIONS[i].help, "in ash") == NULL);
        }

    /* Without it nothing is different: 800 birds, the terminal's colours, thirty. */
    char *flat[] = {"cbirds", NULL};
    reset_test_config();
    config.bird_size = 0;
    read_options(1, flat);
    settle_the_bird_size();
    assert(sky_mode == 0 && config.birds == 800 && config.palette == 0 && config.bird_size == 30);
    reset_test_config();
}

/* What a space cannot do it says, once, and in the same words every time: flocks
 * are for the flat sky and so is the rain, the second sky is every bird's own
 * distance now, and there are no tails. */
static void test_three_d_says_what_it_replaces(void) {
    char message[512];
    int status;

    char *flocks[] = {"cbirds", "--3d", "--flocks", "2", NULL};
    options_read_in_a_child(flocks, 4, message, sizeof(message), &status);
    assert(status == EXIT_USAGE);
    assert(strstr(message, "--3d is one flock over one roost; --flocks is for the flat sky") !=
           NULL);

    char *rain[] = {"cbirds", "--matrix", "--3d", NULL};
    options_read_in_a_child(rain, 3, message, sizeof(message), &status);
    assert(status == EXIT_USAGE);
    assert(strstr(message, "--matrix is for the flat sky") != NULL);

    char *depth[] = {"cbirds", "--3d", "--depth", NULL};
    options_read_in_a_child(depth, 3, message, sizeof(message), &status);
    assert(status == 1); /* In the space, and the second sky cleared. */
    assert(strstr(message, "--3d replaces --depth") != NULL);

    char *tails[] = {"cbirds", "--3d", "--trails", NULL};
    options_read_in_a_child(tails, 3, message, sizeof(message), &status);
    assert(status == 1);
    assert(strstr(message, "--3d draws no tails") != NULL);

    /* Nothing is said about any of it when there is no --3d. */
    char *flat[] = {"cbirds", "--depth", "--trails", "--flocks", "2", NULL};
    options_read_in_a_child(flat, 5, message, sizeof(message), &status);
    assert(status == (2 | 4) && message[0] == '\0');

    /* And the help knows it, in the Look group, on the one screen. */
    const option_t *three = NULL;
    for (size_t i = 0; i < OPTION_COUNT; i++)
        if (strcmp(OPTIONS[i].name, "3d") == 0) three = &OPTIONS[i];
    assert(three != NULL && strcmp(three->group, "Look") == 0 && three->essential == 1);
    assert(three->kind == OPTION_FLAG && three->target == &sky_mode);
    reset_test_config();
}

/*
 * Ink: the terminal's own text colour, fading into its own ground.
 *
 * Built at run time from what the terminal answers, like theme, so the tests build
 * it from pairs they made up: the two a terminal is most likely to be, and the
 * colour schemes that are neither.
 */
typedef struct {
    const char *name;
    uint8_t foreground[3], background[3];
} ink_pair_t;

static const ink_pair_t DARK_INK = {"white on black", {229, 229, 229}, {0, 0, 0}};
static const ink_pair_t LIGHT_INK = {"black on white", {0, 0, 0}, {255, 255, 255}};
static const ink_pair_t INK_PAIRS[] = {
    {"white on black", {255, 255, 255}, {0, 0, 0}},
    {"grey on near black", {229, 229, 229}, {18, 18, 24}},
    {"black on white", {0, 0, 0}, {255, 255, 255}},
    {"solarized dark", {131, 148, 150}, {0, 43, 54}},
    {"solarized light", {101, 123, 131}, {253, 246, 227}},
    {"dracula", {248, 248, 242}, {40, 42, 54}},
    {"gruvbox light", {60, 56, 54}, {251, 241, 199}},
};

static double the_nearest_ink_contrast(const ink_pair_t *pair, int shade) {
    return contrast_between(ink_tints[shade], pair->background);
}

static void test_ink_runs_from_the_foreground_towards_the_background(void) {
    static const double PULLS[] = {SKY_DIM, FAR_DIM};
    for (size_t p = 0; p < sizeof(PULLS) / sizeof(*PULLS); p++)
        for (size_t i = 0; i < sizeof(INK_PAIRS) / sizeof(*INK_PAIRS); i++) {
            const ink_pair_t *pair = &INK_PAIRS[i];
            ink_is_known = 0;
            build_the_ink(pair->foreground, pair->background, PULLS[p]);
            assert(ink_is_known && memcmp(ink_ground, pair->background, 3) == 0);
            /* It starts on the text colour, as the terminal has it. */
            assert(memcmp(ink_tints[0], pair->foreground, 3) == 0);
            for (int shade = 1; shade < 5; shade++) {
                for (int c = 0; c < 3; c++) {
                    /* Towards the background and never past it, a step at a time. */
                    int step = (int)ink_tints[shade][c] - (int)ink_tints[shade - 1][c];
                    int way = (int)pair->background[c] - (int)pair->foreground[c];
                    assert(step * way >= 0 && abs(step) <= abs(way));
                    assert(abs((int)ink_tints[shade][c] - (int)pair->foreground[c]) <= abs(way));
                }
                /* A ramp: every shade is nearer the ground than the one before. */
                assert(the_nearest_ink_contrast(pair, shade) <=
                       the_nearest_ink_contrast(pair, shade - 1));
                assert(memcmp(ink_tints[shade], pair->background, 3) != 0);
            }
            /* Stopping well short of it: the farthest bird, as it is drawn once the
             * renderer has dimmed it, still stands off the ground by two to one. */
            uint8_t drawn[3];
            pulled_towards(ink_tints[4], pair->background, PULLS[p], drawn);
            assert(contrast_between(drawn, pair->background) >= INK_FAR_CONTRAST);
            /* And not by hugging the text colour: where the terminal has the
             * contrast for it the ramp goes a good way, so the depth shows. */
            if (the_nearest_ink_contrast(pair, 0) > 10)
                assert(the_nearest_ink_contrast(pair, 4) < the_nearest_ink_contrast(pair, 0) / 2);
        }

    /* A grey terminal makes grey ink and a white one makes black, which is the
     * whole of the idea: the same builder, opposite ramps. */
    build_the_ink(DARK_INK.foreground, DARK_INK.background, SKY_DIM);
    for (int shade = 0; shade < 5; shade++) {
        assert(ink_tints[shade][0] == ink_tints[shade][1] &&
               ink_tints[shade][1] == ink_tints[shade][2]);
        assert(ink_tints[shade][0] >= 60); /* Never the black it is written on. */
        if (shade > 0) assert(ink_tints[shade][0] < ink_tints[shade - 1][0]);
    }
    build_the_ink(LIGHT_INK.foreground, LIGHT_INK.background, SKY_DIM);
    for (int shade = 0; shade < 5; shade++) {
        assert(ink_tints[shade][0] == ink_tints[shade][1] &&
               ink_tints[shade][1] == ink_tints[shade][2]);
        assert(ink_tints[shade][0] <= 190); /* Never the white it is written on. */
        if (shade > 0) assert(ink_tints[shade][0] > ink_tints[shade - 1][0]);
    }
    assert(ink_tints[0][0] == 0);

    /* A terminal with no contrast to give has none to take away: the ramp is the
     * one shade, and the builder neither hangs nor goes past it. */
    static const uint8_t SAME[3] = {90, 90, 90}, CLOSE[3] = {100, 100, 100};
    build_the_ink(SAME, SAME, SKY_DIM);
    for (int shade = 0; shade < 5; shade++) assert(memcmp(ink_tints[shade], SAME, 3) == 0);
    build_the_ink(CLOSE, SAME, FAR_DIM);
    for (int shade = 0; shade < 5; shade++) assert(memcmp(ink_tints[shade], CLOSE, 3) == 0);
    ink_is_known = 0;
}

/* Learned from the terminal like theme, and a ramp for every ramp's check: on a
 * black terminal no shade fades into it, which is the test every other ramp has. */
static void test_ink_does_not_fade_into_a_black_terminal(void) {
    static const uint8_t BLACK[3] = {0, 0, 0};
    for (int white = 160; white <= 255; white += 5) {
        uint8_t text[3] = {(uint8_t)white, (uint8_t)white, (uint8_t)white};
        build_the_ink(text, BLACK, SKY_DIM);
        for (int shade = 0; shade < 5; shade++)
            assert(contrast_between(ink_tints[shade], BLACK) >= 2.5);
        build_the_ink(text, BLACK, FAR_DIM);
        for (int shade = 0; shade < 5; shade++)
            assert(contrast_between(ink_tints[shade], BLACK) >= 2.5);
    }
    ink_is_known = 0;
}

/* The sprites are tinted from the ramp and dimmed towards the ground, and on a
 * white ground that is towards white. */
static void test_ink_is_drawn_as_it_was_built(void) {
    static const ink_pair_t *PAIRS[] = {&DARK_INK, &LIGHT_INK};
    for (size_t i = 0; i < 2; i++) {
        const ink_pair_t *pair = PAIRS[i];
        reset_test_config();
        sky_mode = 1;
        config.palette = palette_named("ink");
        assert(palette_is_ink() && !the_ground_is_known());
        /* Until a terminal has been asked the ground is the picture's own. */
        assert(picture_ground() == PICTURE_GROUND);
        build_the_ink(pair->foreground, pair->background, SKY_DIM);
        assert(the_ground_is_known() && picture_ground() == ink_ground);
        config.palette = palette_named("ember");
        assert(picture_ground() == PICTURE_GROUND); /* Only ink asks, so only ink knows. */
        config.palette = palette_named("ink");

        double previous = 0;
        for (int bin = 0; bin < SKY_BINS; bin++) {
            png_image_t bird = {0, 0, NULL};
            assert(png_image_alloc(&bird, 1, 1) == PNG_OK);
            bird.pixels[3] = 255;
            tint_sky(&bird, bin);
            uint8_t drawn[3] = {bird.pixels[0], bird.pixels[1], bird.pixels[2]};
            double contrast = contrast_between(drawn, pair->background);
            /* Nearer is stronger, bin by bin, the whole way. */
            assert(contrast > previous);
            previous = contrast;
            if (bin == 0) assert(contrast >= 2.0 && contrast < 2.3);
            if (bin == SKY_BINS - 1) assert(memcmp(drawn, pair->foreground, 3) == 0);
            png_image_free(&bird);
        }
        /* The flat sky's far plane, built for how hard it is dimmed. */
        build_the_ink(pair->foreground, pair->background, FAR_DIM);
        for (int shade = 0; shade < 5; shade++) {
            png_image_t bird = {0, 0, NULL};
            assert(png_image_alloc(&bird, 1, 1) == PNG_OK);
            bird.pixels[3] = 255;
            far_tint(&bird, shade);
            uint8_t drawn[3] = {bird.pixels[0], bird.pixels[1], bird.pixels[2]};
            assert(contrast_between(drawn, pair->background) >= 2.0 - 1e-9);
            png_image_free(&bird);
        }
        ink_is_known = 0;
        reset_test_config();
    }
}

/* A frame of the flock composed onto a ground the colour of a white terminal: the
 * picture the snapshot and the GIF writer make, and the check that three
 * dimensions look right on paper. Every bird's strongest pixel stands off the
 * ground, the nearest is black, and the hawk is not the colour of the paper. */
static void test_a_flock_of_ink_is_seen_on_a_white_ground(void) {
    static png_image_t frames[ROTATION_FRAMES * MAX_SPRITE_SETS];
    png_image_t canvas = {0, 0, NULL};
    enum { ROWS = SKY_BINS, PER = 6 };
    bird_t birds[ROWS * PER];

    reset_test_config();
    sky_mode = 1;
    config.palette = palette_named("ink");
    config.birds = ROWS * PER;
    config.bird_size = 12;
    apply_screen_size(96, 32, 768, 512);
    build_the_ink(LIGHT_INK.foreground, LIGHT_INK.background, SKY_DIM);
    assert(rasterise_sprites(frames) == PNG_OK);
    assert(png_image_alloc(&canvas, screen.width, screen.height) == PNG_OK);
    memset(birds, 0, sizeof(birds));
    for (int bin = 0; bin < ROWS; bin++)
        for (int i = 0; i < PER; i++) {
            bird_t *bird = &birds[bin * PER + i];
            bird->layer = bin;
            bird->shape = i % SKY_SHAPES;
            bird->frame = i * 9;
            bird->x = 40 + i * 100;
            bird->y = 30 + bin * 90;
        }
    compose_onto(&canvas, frames, birds, 1);
    /* The ground is white, and not the dark one of the picture. */
    assert(memcmp(canvas.pixels, LIGHT_INK.background, 3) == 0 && canvas.pixels[3] == 255);
    for (int bin = 0; bin < ROWS; bin++) {
        double strongest = 0;
        for (int i = 0; i < PER; i++) {
            const bird_t *bird = &birds[bin * PER + i];
            double best = 0;
            for (int y = 0; y < sky_bin_size(bin); y++)
                for (int x = 0; x < sky_bin_size(bin); x++) {
                    const uint8_t *pixel =
                        canvas.pixels +
                        (((size_t)(bird->y + y)) * (size_t)canvas.width + (size_t)(bird->x + x)) *
                            4;
                    double contrast = contrast_between(pixel, LIGHT_INK.background);
                    if (contrast > best) best = contrast;
                }
            /* A bird that is seen at all, whatever its shape, even the thinnest. */
            assert(best >= 1.8);
            if (best > strongest) strongest = best;
        }
        assert(strongest >= 2.0);
        if (bin == ROWS - 1) assert(strongest >= 15); /* The nearest is black on white. */
    }
    /* A hawk against paper: scarlet, which is seen there, and not the near white
     * or the cyan, which are far from the ink and as near as can be to the sheet. */
    const uint8_t *hawk = hawk_colour();
    assert(contrast_between(hawk, LIGHT_INK.background) >= HAWK_GROUND_CONTRAST);
    assert(memcmp(hawk, HAWK_COLOURS[0], 3) == 0);
    /* On a dark ground the choice is the one it always was: nothing is ruled out. */
    build_the_ink(DARK_INK.foreground, DARK_INK.background, SKY_DIM);
    assert(contrast_between(hawk_colour(), DARK_INK.background) >= HAWK_GROUND_CONTRAST);
    config.palette = palette_named("ash");
    const uint8_t *with_ash = hawk_colour();
    config.palette = palette_named("ink");
    assert(memcmp(hawk_colour(), with_ash, 3) == 0);

    png_image_free(&canvas);
    free_sprites(frames);
    ink_is_known = 0;
    reset_test_config();
}

/* Ink is a ramp for the flat sky too, and the flat sky has waves: the light of a
 * wave is chosen to be seen on the terminal's ground, and with ink that is known,
 * so that on paper the light is dark and on a dark terminal it is what it always
 * was. Without ink asked for, the ground is the one theme learns, and nothing
 * moves. */
static void test_the_light_of_a_wave_is_seen_on_the_ground_ink_knows(void) {
    uint8_t unasked[3];

    reset_test_config();
    config.palette = palette_named("ink");
    /* Not asked of the terminal: the choice it always was. */
    ink_is_known = 0;
    memcpy(unasked, highlight_colour(), 3);

    /* White on black: seen on it. */
    build_the_ink(DARK_INK.foreground, DARK_INK.background, FAR_DIM);
    assert(contrast_between(highlight_colour(), DARK_INK.background) >= HIGHLIGHT_CONTRAST);
    /* Black on white: a light that was white would be a lit bird that is not there,
     * and the light is the dark one the list keeps for a ground that is light. */
    build_the_ink(LIGHT_INK.foreground, LIGHT_INK.background, FAR_DIM);
    assert(contrast_between(highlight_colour(), LIGHT_INK.background) >= HIGHLIGHT_CONTRAST);
    assert(memcmp(highlight_colour(), HIGHLIGHT_COLOURS[HIGHLIGHT_COLOUR_COUNT - 1], 3) == 0);

    /* And the same ramp, no longer known, is judged as ever. */
    ink_is_known = 0;
    assert(memcmp(highlight_colour(), unasked, 3) == 0);
    reset_test_config();
}

/* The terminal is asked for its background and then its foreground, with the
 * machinery the theme uses, and a terminal that does not answer, or answers half,
 * is not guessed at. The terminal is a pty with the test on the other end of it. */
typedef enum { ANSWER_BOTH, ANSWER_NOTHING, ANSWER_THE_BACKGROUND } answer_t;

static int learn_the_ink_from(answer_t answer, const char *background, const char *foreground,
                              const uint8_t expected_background[3],
                              const uint8_t expected_foreground[3], char *asked,
                              size_t asked_size) {
    int master = posix_openpt(O_RDWR | O_NOCTTY);
    assert(master >= 0 && grantpt(master) == 0 && unlockpt(master) == 0);
    const char *name = ptsname(master);
    assert(name != NULL);
    int terminal = open(name, O_RDWR | O_NOCTTY);
    assert(terminal >= 0);
    struct termios raw;
    assert(tcgetattr(terminal, &raw) == 0);
    cfmakeraw(&raw);
    assert(tcsetattr(terminal, TCSANOW, &raw) == 0);
    fflush(NULL);

    pid_t child = fork();
    assert(child >= 0);
    if (child == 0) {
        alarm(20);
        close(master);
        if (dup2(terminal, STDIN_FILENO) < 0 || dup2(terminal, STDOUT_FILENO) < 0) _exit(99);
        ink_is_known = 0;
        int learned = learn_the_ink(SKY_DIM);
        if (!learned) _exit(ink_is_known ? 98 : 1);
        if (!ink_is_known || memcmp(ink_ground, expected_background, 3) != 0 ||
            memcmp(ink_tints[0], expected_foreground, 3) != 0)
            _exit(97);
        _exit(0);
    }
    close(terminal);
    asked[0] = '\0';
    /* The terminal's side: read what is asked, answer what the case answers, until
     * the other end is gone. A read that fails is that, and the child's end of the
     * pty closing is how the test knows it has finished. */
    for (;;) {
        if (readable_within(master, 5000) <= 0) break;
        char request[64];
        ssize_t got = read(master, request, sizeof(request) - 1);
        if (got <= 0) break;
        request[got] = '\0';
        if (strlen(asked) + (size_t)got < asked_size) strcat(asked, request);
        const char *reply = NULL;
        if (strstr(request, "]11;?") != NULL && answer != ANSWER_NOTHING) reply = background;
        if (strstr(request, "]10;?") != NULL && answer == ANSWER_BOTH) reply = foreground;
        if (reply != NULL) assert(write(master, reply, strlen(reply)) == (ssize_t)strlen(reply));
    }
    int status = 0;
    assert(waitpid(child, &status, 0) == child);
    close(master);
    assert(WIFEXITED(status));
    return WEXITSTATUS(status);
}

static void test_ink_is_asked_of_the_terminal(void) {
    char asked[256];
    static const uint8_t WHITE[3] = {255, 255, 255}, BLACK[3] = {0, 0, 0},
                         GREY[3] = {0xe5, 0xe5, 0xe5};

    /* A dark terminal: four hex digits a channel, as xterm answers. */
    assert(learn_the_ink_from(ANSWER_BOTH, "\033]11;rgb:0000/0000/0000\033\\",
                              "\033]10;rgb:e5e5/e5e5/e5e5\033\\", BLACK, GREY, asked,
                              sizeof(asked)) == 0);
    /* The background first, then the text. */
    assert(strstr(asked, "]11;?") != NULL && strstr(asked, "]10;?") != NULL);
    assert(strstr(asked, "]11;?") < strstr(asked, "]10;?"));

    /* A light one, answering with two digits a channel and a bell, which some do. */
    assert(learn_the_ink_from(ANSWER_BOTH, "\033]11;rgb:ff/ff/ff\007", "\033]10;rgb:00/00/00\007",
                              WHITE, BLACK, asked, sizeof(asked)) == 0);

    /* A terminal that says nothing is asked once and not guessed at: there is no
     * foreground to ask for when there is no ground to put it on. */
    assert(learn_the_ink_from(ANSWER_NOTHING, "", "", WHITE, BLACK, asked, sizeof(asked)) == 1);
    assert(strstr(asked, "]11;?") != NULL && strstr(asked, "]10;?") == NULL);

    /* One that knows its ground and not its text is not guessed at either: a ramp
     * from a foreground that was made up is invisible on the terminals that need
     * it most. */
    assert(learn_the_ink_from(ANSWER_THE_BACKGROUND, "\033]11;rgb:ffff/ffff/ffff\033\\", "", WHITE,
                              BLACK, asked, sizeof(asked)) == 1);
    assert(strstr(asked, "]10;?") != NULL);
}

/* No terminal, no ink: a recording and a benchmark have nothing to ask, so the ramp
 * the flock wore before there was ink is what they draw, and they stay as they
 * were. */
static void test_ink_without_a_terminal_is_ash(void) {
    char path[600];
    reset_test_config();
    config.palette = palette_named("ink");
    ink_is_known = 0;
    settle_the_palette_without_a_terminal();
    assert(config.palette == palette_named("ash") && !palette_is_ink() && !the_ground_is_known());

    /* A ramp is only ink where it was asked for. */
    config.palette = palette_named("ember");
    settle_the_palette_without_a_terminal();
    assert(config.palette == palette_named("ember"));

    scratch_file(path, sizeof(path), "ink.gif");
    config.palette = palette_named("ink");
    sky_mode = 1;
    config.birds = 60;
    record_path = path;
    record_fps = 25;
    record_seconds = 1;
    record_columns = 48;
    record_rows = 16;
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
    assert(config.palette == palette_named("ash") && !the_ground_is_known());
    remove(path);
    record_path = NULL;
    reset_test_config();
}

/* A space is framed to fill the window, so its birds are a share of it, and a
 * window that settles at another size gets birds of another size: not at once, or
 * dragging a corner would rebuild them every frame, and not for a change too small
 * to see. The flat flock's are the size it was asked for, and are left alone. */
static void test_the_birds_are_rebuilt_when_the_window_settles(void) {
    kitty_graphics_t graphics;
    int quiet = open("/dev/null", O_WRONLY);
    assert(quiet >= 0 && kitty_graphics_init(&graphics, quiet) == KITTY_GRAPHICS_OK);

    reset_test_config();
    sky_mode = 1;
    config.bird_size = 8;
    config.palette = palette_named("ash");
    render_mode = RENDER_BRAILLE;
    picture_held_for = picture_last_seen = 0;
    apply_screen_size(96, 32, 768, 512);
    assert(rasterise_sprites(text_sprites) == PNG_OK);
    int built_for = sky_picture_size;
    int near_size = text_sprites[flock_set(0, 0, SKY_BINS - 1) * ROTATION_FRAMES].width;
    assert(built_for == 512 && near_size == sky_bin_size(SKY_BINS - 1));
    /* The window as it was needs nothing, however long it is looked at. */
    for (int frame = 0; frame < 3 * RESIZE_SETTLE_FRAMES; frame++)
        assert(fit_the_sprites_to_the_window(&graphics) == SPRITES_FIT);
    assert(sky_picture_size == built_for);

    /* Dragged: a new size every frame, however far it goes, is never rebuilt for. */
    for (int frame = 0; frame < 4 * RESIZE_SETTLE_FRAMES; frame++) {
        int height = 512 + 10 * (frame + 1);
        apply_screen_size(96, 32, height * 3 / 2, height);
        assert(the_sprites_are_for_another_picture() || frame < 4);
        assert(fit_the_sprites_to_the_window(&graphics) == SPRITES_FIT);
    }
    assert(sky_picture_size == built_for);

    /* Settled at twice the size: nothing for a third of a second, then a rebuild,
     * once, and the birds are twice as big and placed by what they were drawn at. */
    apply_screen_size(192, 64, 1536, 1024);
    assert(the_sprites_are_for_another_picture());
    int frames_to_wait = 0;
    while (fit_the_sprites_to_the_window(&graphics) == SPRITES_FIT) {
        assert(sky_picture_size == built_for && ++frames_to_wait <= 3 * RESIZE_SETTLE_FRAMES);
    }
    assert(frames_to_wait == RESIZE_SETTLE_FRAMES);
    assert(sky_picture_size == 1024 && !the_sprites_are_for_another_picture());
    int grown = text_sprites[flock_set(0, 0, SKY_BINS - 1) * ROTATION_FRAMES].width;
    assert(grown == sky_bin_size(SKY_BINS - 1) && grown >= near_size * 2 - 1);
    for (int bin = 0; bin < SKY_BINS; bin++)
        for (int shape = 0; shape < SKY_SHAPES; shape++)
            assert(text_sprites[flock_set(0, shape, bin) * ROTATION_FRAMES].width ==
                   sky_bin_size(bin));
    assert(hawk_size_in_layer(0) == sky_bin_size(0) * 3 &&
           text_sprites[hawk_set(0) * ROTATION_FRAMES].width == hawk_size_in_layer(0));
    for (int frame = 0; frame < 3 * RESIZE_SETTLE_FRAMES; frame++)
        assert(fit_the_sprites_to_the_window(&graphics) == SPRITES_FIT);

    /* A change of a few in a hundred is not worth the pause. */
    apply_screen_size(192, 64, 1536 * 103 / 100, 1024 * 103 / 100);
    for (int frame = 0; frame < 3 * RESIZE_SETTLE_FRAMES; frame++)
        assert(fit_the_sprites_to_the_window(&graphics) == SPRITES_FIT);
    assert(sky_picture_size == 1024);

    /* And smaller again; and under Kitty the images are sent, and freed here. */
    render_mode = RENDER_KITTY;
    apply_screen_size(96, 32, 768, 512);
    sprite_fit_t fit = SPRITES_FIT;
    for (int frame = 0; frame < 2 * RESIZE_SETTLE_FRAMES && fit == SPRITES_FIT; frame++)
        fit = fit_the_sprites_to_the_window(&graphics);
    assert(fit == SPRITES_REBUILT && sky_picture_size == 512);
    assert(text_sprites[flock_set(0, 0, 0) * ROTATION_FRAMES].pixels == NULL);
    assert(graphics.length == 0); /* Sent, not left in the buffer. */

    /* The flat flock's sprites do not depend on the window, so they are never
     * rebuilt for it. */
    free_sprites(text_sprites);
    reset_test_config();
    render_mode = RENDER_BRAILLE;
    apply_screen_size(96, 32, 768, 512);
    for (int frame = 0; frame < 3 * RESIZE_SETTLE_FRAMES; frame++) {
        apply_screen_size(96 + frame, 32, 768 + 8 * frame, 512);
        assert(fit_the_sprites_to_the_window(&graphics) == SPRITES_FIT);
    }
    assert(text_sprites[flock_set(0, 0, 0) * ROTATION_FRAMES].pixels == NULL);

    kitty_graphics_destroy(&graphics);
    close(quiet);
    picture_held_for = picture_last_seen = 0;
    render_mode = RENDER_KITTY;
    reset_test_config();
}

/* A set of pictures for every size and shape, and the hawks' and the tails' after
 * them, where the flat flock has its far plane, and last the sets of the light of a
 * wave, where they are in the flat sky too. */
static void test_a_space_has_a_picture_for_every_size_and_shape(void) {
    static png_image_t frames[ROTATION_FRAMES * MAX_SPRITE_SETS];
    int seen[MAX_SPRITE_SETS] = {0};

    reset_test_config();
    sky_mode = 1;
    config.palette = palette_named("ember");
    config.bird_size = 12;
    apply_screen_size(96, 32, 768, 512);
    assert(flock_set_count() == SKY_BINS * SKY_SHAPES);
    assert(hawk_set(0) == flock_set_count());
    assert(trail_set(0) == flock_set_count() + SKY_BINS * WING_PHASES); /* A hawk a size. */
    for (int bin = 0; bin < SKY_BINS; bin++)
        for (int shape = 0; shape < SKY_SHAPES; shape++) seen[flock_set(0, shape, bin)]++;
    for (int wing = 0; wing < SKY_BINS * WING_PHASES; wing++) seen[hawk_set(wing)]++;
    for (int step = 0; step < TRAIL_LENGTH; step++) seen[trail_set(step)]++;
    for (int wing = 0; wing < WING_PHASES; wing++) seen[alarm_set(wing)]++;
    for (int set = 0; set < sprite_set_count(); set++) assert(seen[set] == 1);
    assert(sprite_set_count() <= MAX_SPRITE_SETS);

    assert(rasterise_sprites(frames) == PNG_OK);
    long ink[SKY_BINS][SKY_SHAPES];
    int previous_width = 0;
    for (int bin = 0; bin < SKY_BINS; bin++) {
        /* Bigger the nearer, as sky_bin_size says, and square. */
        const png_image_t *first = &frames[flock_set(0, 0, bin) * ROTATION_FRAMES];
        assert(first->width == sky_bin_size(bin) && first->height == first->width);
        assert(first->width >= previous_width);
        previous_width = first->width;
        for (int shape = 0; shape < SKY_SHAPES; shape++) {
            for (int frame = 0; frame < ROTATION_FRAMES; frame++) {
                const png_image_t *image =
                    &frames[flock_set(0, shape, bin) * ROTATION_FRAMES + frame];
                assert(image->pixels != NULL && image->width == first->width);
            }
            const png_image_t *image = &frames[flock_set(0, shape, bin) * ROTATION_FRAMES];
            ink[bin][shape] = 0;
            for (int i = 0; i < image->width * image->height; i++)
                ink[bin][shape] += image->pixels[i * 4 + 3];
        }
    }
    assert(sky_bin_size(SKY_BINS - 1) > sky_bin_size(0));
    /* A bird seen edge on, or shortened, has less in it than one in full view, in
     * every size where there is room for the difference to show. */
    for (int bin = 2; bin < SKY_BINS; bin++) {
        assert(ink[bin][1] < ink[bin][0] && ink[bin][2] < ink[bin][1]);
        assert(ink[bin][SKY_ACROSS_LEVELS] < ink[bin][0]);
    }
    /* Further off is dimmer: the first opaque pixel of the nearest and the farthest. */
    int brightness[2] = {0, 0};
    for (int which = 0; which < 2; which++) {
        const png_image_t *image =
            &frames[flock_set(0, 0, which ? 0 : SKY_BINS - 1) * ROTATION_FRAMES];
        for (int i = 0; i < image->width * image->height; i++)
            if (image->pixels[i * 4 + 3] == 255) {
                brightness[which] =
                    image->pixels[i * 4] + image->pixels[i * 4 + 1] + image->pixels[i * 4 + 2];
                break;
            }
    }
    assert(brightness[0] > 0 && brightness[1] > 0 && brightness[1] < brightness[0]);
    /* The hawks are after the flock, a size and a wing phase each, and a hawk is
     * bigger than a bird of its size, and scarlet. */
    for (int bin = 0; bin < SKY_BINS; bin++)
        for (int wing = 0; wing < WING_PHASES; wing++) {
            const png_image_t *hawk = &frames[hawk_set(bin * WING_PHASES + wing) * ROTATION_FRAMES];
            assert(hawk->pixels != NULL && hawk->width > sky_bin_size(bin));
            if (wing == 0) assert(hawk->width == hawk_size_in_layer(bin));
        }
    /* No tails and no light of a wave, which a space has none of: a set that is not
     * built is not uploaded or drawn either. */
    for (int step = 0; step < TRAIL_LENGTH; step++)
        assert(frames[trail_set(step) * ROTATION_FRAMES].pixels == NULL);
    for (int wing = 0; wing < WING_PHASES; wing++)
        assert(frames[alarm_set(wing) * ROTATION_FRAMES].pixels == NULL);
    free_sprites(frames);
    reset_test_config();
}

/* How a bird is drawn: its shape follows how much of it the camera sees. */
static void test_a_birds_shape_follows_what_the_camera_sees(void) {
    sky_view_t full = {.along = 1.0f, .across = 1.0f}, head_on = {.along = 0.3f, .across = 1.0f},
               edge_on = {.along = 1.0f, .across = 0.2f}, both = {.along = 0.2f, .across = 0.2f};

    assert(sky_shape(&full, WING_SPAN[0]) == 0);
    assert(sky_shape(&full, WING_SPAN[1]) == 1); /* Wings half in, seen from above. */
    assert(sky_shape(&full, WING_SPAN[2]) == 2);
    assert(sky_shape(&head_on, WING_SPAN[0]) == SKY_ACROSS_LEVELS);
    assert(sky_shape(&edge_on, WING_SPAN[0]) == 2);
    assert(sky_shape(&both, WING_SPAN[0]) == SKY_SHAPES - 1);
    for (double a = 0; a <= 1; a += 0.05)
        for (double b = 0; b <= 1; b += 0.05) {
            sky_view_t view = {.along = (float)a, .across = (float)b};
            for (int beat = 0; beat < WING_PHASES; beat++) {
                int shape = sky_shape(&view, WING_SPAN[beat]);
                assert(shape >= 0 && shape < SKY_SHAPES);
            }
        }
}

typedef struct {
    int placed, ordered;
} frame_summary_t;

/* The z of every placement in a Kitty frame, in the order they were queued. */
static frame_summary_t summarise_the_placements(const char *frame) {
    frame_summary_t summary = {0, 1};
    int previous = -1000;
    for (const char *at = strstr(frame, "a=p"); at != NULL; at = strstr(at + 3, "a=p")) {
        const char *end = strstr(at, "\033\\");
        const char *z = strstr(at, ",z=");
        int level = z != NULL && z < end ? atoi(z + 3) : 0;
        if (level < previous) summary.ordered = 0;
        previous = level;
        summary.placed++;
    }
    return summary;
}

/* The space comes out of the same renderers as the flat flock: the birds it
 * projects are placed far to near, drawn in braille, composed onto a ground, and
 * kept out from behind the panel. */
static void test_a_space_is_drawn_by_the_renderers_of_the_flat_flock(void) {
    enum { COUNT = 400 };
    bird_t *birds = calloc(COUNT, sizeof(*birds)), *snapshot = malloc(sizeof(*snapshot) * COUNT);
    spatial_grid_t grid;
    kitty_graphics_t graphics;
    png_image_t canvas = {0, 0, NULL};

    assert(birds != NULL && snapshot != NULL);
    reset_test_config();
    sky_mode = 1;
    legend_enabled = 0;
    config.birds = COUNT;
    config.bird_size = 8;
    config.palette = palette_named("ash");
    apply_screen_size(96, 32, 768, 512);
    set_frame_seconds(1.0 / FRAME_RATE);
    seed_random(4);
    assert(spatial_grid_init(&grid, SPATIAL_CELL_SIZE) == SPATIAL_GRID_OK);
    assert(spatial_grid_prepare(&grid, screen.width, screen.height, COUNT) == SPATIAL_GRID_OK);
    initialize_birds(birds);
    assert(sky.flying == 1);
    for (int frame = 0; frame < 20; frame++) fly(birds, snapshot, &grid);

    int on_screen = 0, bins_used[SKY_BINS] = {0};
    for (int i = 0; i < COUNT; i++) {
        assert(birds[i].layer >= 0 && birds[i].layer < SKY_BINS);
        assert(birds[i].frame >= 0 && birds[i].frame < ROTATION_FRAMES);
        assert(birds[i].shape >= 0 && birds[i].shape < SKY_SHAPES);
        assert(sprite_image_id(&birds[i]) >= 1 &&
               sprite_image_id(&birds[i]) <= (uint32_t)sprite_set_count() * ROTATION_FRAMES);
        kitty_graphics_placement_t placement;
        if (bird_placement(&birds[i], &placement)) {
            on_screen++;
            assert(placement.z_index == birds[i].layer);
        }
        bins_used[birds[i].layer]++;
    }
    int bins_in_use = 0;
    for (int bin = 0; bin < SKY_BINS; bin++) bins_in_use += bins_used[bin] > 0;
    assert(bins_in_use >= 3); /* A flock has depth. */
    assert(on_screen > COUNT * 9 / 10);

    /* Sprites: one placement a bird that is on the screen, far to near. */
    assert(kitty_graphics_init(&graphics, STDOUT_FILENO) == KITTY_GRAPHICS_OK);
    assert(queue_render_frame(&graphics, birds) == KITTY_GRAPHICS_OK);
    frame_summary_t summary = summarise_the_placements(graphics.buffer);
    assert(summary.placed == on_screen && summary.ordered);

    /* Text: the same birds, as dots. */
    render_mode = RENDER_BRAILLE;
    assert(prepare_text_renderer());
    clear_graphics_buffer(&graphics);
    assert(queue_render_frame(&graphics, birds) == KITTY_GRAPHICS_OK);
    assert(strstr(graphics.buffer, "a=p") == NULL);
    int dots = 0;
    for (const unsigned char *c = (const unsigned char *)graphics.buffer; *c; c++)
        if (c[0] == 0xE2 && (c[1] & 0xFC) == 0xA0) dots++;
    assert(dots > COUNT / 4);

    /* A picture: composed onto a ground, which has birds on it of more than one
     * brightness, because a far bird is dimmer. */
    assert(png_image_alloc(&canvas, screen.width, screen.height) == PNG_OK);
    compose_onto(&canvas, text_sprites, birds, 1);
    int levels[256] = {0}, seen = 0;
    for (int i = 0; i < canvas.width * canvas.height; i++) {
        uint8_t red = canvas.pixels[i * 4];
        if (red > 60 && !levels[red]++) seen++;
    }
    assert(seen >= 3);

    /* Behind the panel is not drawn, and beside it is. */
    render_mode = RENDER_KITTY;
    legend_enabled = 1;
    apply_screen_size(96, 32, 768, 512);
    legend_drawn = 1;
    kitty_graphics_placement_t placement;
    bird_t behind = {.x = 40, .y = 40, .layer = 2}, beside = {.x = 600, .y = 40, .layer = 2};
    assert(!bird_placement(&behind, &placement));
    assert(bird_placement(&beside, &placement));
    legend_drawn = 0;

    png_image_free(&canvas);
    free_sprites(text_sprites);
    cells_destroy(&text_cells);
    png_image_free(&text_canvas);
    memset(&text_canvas, 0, sizeof(text_canvas));
    kitty_graphics_destroy(&graphics);
    spatial_grid_destroy(&grid);
    free(snapshot);
    free(birds);
    render_mode = RENDER_KITTY;
    reset_test_config();
}

/* Each row of the panel says what it does in a space, and none is longer than the
 * panel is wide: the edges are a roost that pulls, and perception is how many
 * neighbours a bird heeds. */
static void test_the_panel_says_what_its_rows_do_in_a_space(void) {
    char lines[LEGEND_MAX_ROWS][LEGEND_LINE_MAX];

    reset_test_config();
    sky_mode = 1;
    apply_screen_size(96, 32, 768, 512);
    build_legend(lines);
    assert(strstr(lines[1], "roost") != NULL && strstr(lines[1], "boundary") == NULL);
    assert(strstr(lines[2], "separation") != NULL && strstr(lines[3], "alignment") != NULL);
    assert(strstr(lines[4], "turning") != NULL && strstr(lines[4], "70\u00b0") != NULL);
    assert(strstr(lines[5], "neighbours") != NULL && strstr(lines[5], "perception") == NULL);
    assert(strstr(lines[6], "speed") != NULL);
    /* On the notches they start on, a factor of one, and seven birds. */
    for (int row = 1; row <= 3; row++) assert(strstr(lines[row], "1.00\u00d7") != NULL);
    assert(strstr(lines[5], " 7 ") != NULL);
    for (int row = 0; row < LEGEND_ROWS; row++) assert(legend_cells(lines[row]) == LEGEND_COLUMNS);
    /* The ends of the bars: a twentieth of the roost's pull to three times it, a
     * fifth of the separation to two and a half, and one neighbour to thirteen. */
    config.boundary_notch = config.separation_notch = config.alignment_notch = 0;
    config.vision_notch = 0;
    apply_notches();
    build_legend(lines);
    assert(strstr(lines[1], "0.05\u00d7") != NULL && strstr(lines[2], "0.20\u00d7") != NULL);
    assert(strstr(lines[3], "0.07\u00d7") != NULL && strstr(lines[5], " 1 ") != NULL);
    config.boundary_notch = config.separation_notch = config.alignment_notch = LEGEND_BAR_CELLS;
    config.vision_notch = LEGEND_BAR_CELLS;
    apply_notches();
    build_legend(lines);
    assert(strstr(lines[1], "2.90\u00d7") != NULL && strstr(lines[2], "2.60\u00d7") != NULL);
    assert(strstr(lines[3], "2.87\u00d7") != NULL && strstr(lines[5], "13") != NULL);
    for (int row = 0; row < LEGEND_ROWS; row++) assert(legend_cells(lines[row]) == LEGEND_COLUMNS);

    /* The flat flock's rows are what they were. */
    sky_mode = 0;
    reset_test_config();
    build_legend(lines);
    assert(strstr(lines[1], "boundary") != NULL && strstr(lines[5], "perception") != NULL);
}

/* The sliders are weights on the flight, and each is a factor on what it was tuned
 * at: the default notches are the tuned flight exactly, and the ends of each bar
 * are the ends of the factor, as the panel prints them. */
static void test_the_sliders_steer_the_flight_in_a_space(void) {
    reset_test_config();
    sky_mode = 1;
    sky_rules_t tuned = sky_default_rules();
    sky_rules_t rules = sky_rules();
    assert(rules.neighbours == 7);
    assert(fabs(rules.separation - tuned.separation) < 1e-9);
    assert(fabs(rules.alignment - tuned.alignment) < 1e-9);
    assert(fabs(rules.roost - tuned.roost) < 1e-9);
    assert(rules.current == tuned.current && rules.cruise == tuned.cruise);
    assert(fabs(rules.turn_rate - turning_notch_radians()) < 1e-12);

    config.boundary_notch = config.separation_notch = config.alignment_notch = 0;
    config.vision_notch = 0;
    config.turning_notch = 0;
    apply_notches();
    rules = sky_rules();
    assert(rules.neighbours == 1);
    assert(rules.roost < 0.1 * tuned.roost && rules.alignment < 0.1 * tuned.alignment);
    assert(rules.separation < 0.25 * tuned.separation);
    assert(fabs(rules.turn_rate * 180 / M_PI - 30) < 1e-9);

    config.boundary_notch = config.separation_notch = config.alignment_notch = LEGEND_BAR_CELLS;
    config.vision_notch = LEGEND_BAR_CELLS;
    config.turning_notch = LEGEND_BAR_CELLS;
    apply_notches();
    rules = sky_rules();
    assert(rules.neighbours == 13 && rules.neighbours <= SKY_MAX_NEIGHBOURS);
    assert(rules.roost > 2.5 * tuned.roost && rules.alignment > 2.5 * tuned.alignment);
    assert(rules.separation > 2.5 * tuned.separation);
    assert(fabs(rules.turn_rate - 2 * M_PI) < 1e-9);
    /* Each one a step up from the one before it. */
    double roost = -1;
    for (int notch = 0; notch <= LEGEND_BAR_CELLS; notch++) {
        config.boundary_notch = notch;
        apply_notches();
        assert(sky_rules().roost > roost);
        roost = sky_rules().roost;
    }
    reset_test_config();
}

/* + and - grow and shrink a flock that is flying in a space like any other: the
 * birds already in the air go on, and the new ones are born into it. */
static void test_a_space_grows_and_shrinks_with_the_keys(void) {
    bird_t *birds = calloc(100, sizeof(*birds)), *snapshot = malloc(sizeof(*snapshot) * 100);
    spatial_grid_t grid;

    assert(birds != NULL && snapshot != NULL);
    reset_test_config();
    sky_mode = 1;
    legend_enabled = 0;
    config.birds = 100;
    apply_screen_size(96, 32, 768, 512);
    seed_random(2);
    initialize_birds(birds);
    assert(spatial_grid_init(&grid, SPATIAL_CELL_SIZE) == SPATIAL_GRID_OK);
    assert(spatial_grid_prepare(&grid, screen.width, screen.height, 400) == SPATIAL_GRID_OK);
    for (int frame = 0; frame < 5; frame++) fly(birds, snapshot, &grid);
    sky_bird_t before = sky.world.birds[40];

    int from = config.birds;
    assert(feed_input("+") == 1 && population_changed && config.birds > from);
    population_changed = 0;
    assert(resize_the_flock(&birds, &snapshot, from, config.birds) == 1);
    assert(sky.world.capacity >= config.birds);
    assert(memcmp(&before, &sky.world.birds[40], sizeof(before)) == 0);
    int grown = config.birds;
    for (int frame = 0; frame < 5; frame++) fly(birds, snapshot, &grid);
    for (int i = 0; i < grown; i++)
        assert(isfinite(sky.world.birds[i].x) && birds[i].layer >= 0 && birds[i].layer < SKY_BINS);

    assert(feed_input("---") == 1 && config.birds < grown);
    for (int frame = 0; frame < 5; frame++) fly(birds, snapshot, &grid);
    /* The tails key does nothing in a space, where there are none to draw. */
    config.trails = 0;
    assert(feed_input("e") == 1 && config.trails == 0);

    /* The keys said the population had changed, and nobody is going to act on it. */
    population_changed = 0;
    spatial_grid_destroy(&grid);
    free(snapshot);
    free(birds);
    reset_test_config();
}

/* A second of the show is a second of flight at the pace it ships at, a space
 * goes slower or faster with the speed slider like the flat flock does, and a
 * paused one stands still, camera and all, until it is stepped. */
static void test_a_space_flies_at_the_pace_it_is_told(void) {
    bird_t *birds = calloc(60, sizeof(*birds)), *snapshot = malloc(sizeof(*snapshot) * 60);
    spatial_grid_t grid;
    kitty_graphics_t graphics;

    assert(birds != NULL && snapshot != NULL);
    reset_test_config();
    sky_mode = 1;
    legend_enabled = 0;
    config.birds = 60;
    apply_screen_size(96, 32, 768, 512);
    seed_random(2);
    assert(spatial_grid_init(&grid, SPATIAL_CELL_SIZE) == SPATIAL_GRID_OK);
    assert(spatial_grid_prepare(&grid, screen.width, screen.height, 60) == SPATIAL_GRID_OK);
    assert(kitty_graphics_init(&graphics, STDOUT_FILENO) == KITTY_GRAPHICS_OK);
    initialize_birds(birds);
    assert(sky.world.clock == 0); /* The warm up is before anybody is looking. */

    static const int NOTCHES[] = {0, DEFAULT_PACE_NOTCH, 4, 12};
    for (size_t i = 0; i < sizeof(NOTCHES) / sizeof(*NOTCHES); i++) {
        config.pace_notch = NOTCHES[i];
        apply_notches();
        double started = sky.world.clock;
        for (int frame = 0; frame < FRAME_RATE; frame++) fly(birds, snapshot, &grid);
        /* A second on screen is the pace's worth of seconds of flight, at the rate
         * the flight was tuned: two and a half to the pace's one fifth. */
        assert(fabs(sky.world.clock - started - config.pace * SKY_TIME) < 1e-6);
    }
    config.pace_notch = DEFAULT_PACE_NOTCH;
    apply_notches();

    paused = 1;
    step_once = 0;
    bird_t held[60];
    memcpy(held, birds, sizeof(held));
    double stopped_at = sky.world.clock;
    assert(render_frame(&graphics, birds, snapshot, &grid) == KITTY_GRAPHICS_OK);
    assert(memcmp(held, birds, sizeof(held)) == 0 && sky.world.clock == stopped_at);
    step_once = 1;
    clear_graphics_buffer(&graphics);
    assert(render_frame(&graphics, birds, snapshot, &grid) == KITTY_GRAPHICS_OK);
    assert(sky.world.clock > stopped_at && step_once == 0);
    paused = 0;

    kitty_graphics_destroy(&graphics);
    spatial_grid_destroy(&grid);
    free(snapshot);
    free(birds);
    reset_test_config();
}

/* Hawks hunt in a space: the module flies them and the flight's seam draws them
 * where the camera sees them, in front of every bird, one size to a distance. */
static void test_hawks_hunt_in_a_space_and_are_drawn_over_it(void) {
    enum { COUNT = 300 };
    bird_t *birds = calloc(COUNT, sizeof(*birds)), *snapshot = malloc(sizeof(*snapshot) * COUNT);
    spatial_grid_t grid;
    kitty_graphics_t graphics;

    assert(birds != NULL && snapshot != NULL);
    reset_test_config();
    sky_mode = 1;
    legend_enabled = 0;
    config.birds = COUNT;
    config.hawks = 2;
    config.palette = palette_named("ash");
    config.bird_size = 8;
    apply_screen_size(96, 32, 768, 512);
    set_frame_seconds(1.0 / FRAME_RATE);
    seed_random(8);
    assert(spatial_grid_init(&grid, SPATIAL_CELL_SIZE) == SPATIAL_GRID_OK);
    assert(spatial_grid_prepare(&grid, screen.width, screen.height, COUNT) == SPATIAL_GRID_OK);
    assert(kitty_graphics_init(&graphics, STDOUT_FILENO) == KITTY_GRAPHICS_OK);
    initialize_birds(birds);
    assert(sky.world.hawk_count == 2);
    for (int frame = 0; frame < 120; frame++) fly(birds, snapshot, &grid);

    int seen = 0;
    for (int i = 0; i < 2; i++) {
        assert(hawks[i].layer >= 0 && hawks[i].layer < SKY_BINS);
        assert(hawks[i].frame >= 0 && hawks[i].frame < ROTATION_FRAMES);
        assert(hawk_image_id(&hawks[i]) >= 1 &&
               hawk_image_id(&hawks[i]) <= (uint32_t)sprite_set_count() * ROTATION_FRAMES);
        /* It is where the camera sees the hawk: the module's own hawk, projected. */
        sky_camera_t camera;
        sky_view_t view;
        sky_camera(&camera);
        sky_hawk_view(&sky.world, i, &camera, &view);
        assert(view.visible && fabs(hawks[i].x - view.x) < 1e-3 &&
               fabs(hawks[i].y - view.y) < 1e-3);
        seen += hawks[i].x >= 0 && hawks[i].x < screen.width && hawks[i].y >= 0 &&
                hawks[i].y < screen.height;
    }
    assert(seen >= 1);
    /* And in the sprite frame they come after the last bird, on top of all of it. */
    assert(queue_render_frame(&graphics, birds) == KITTY_GRAPHICS_OK);
    frame_summary_t summary = summarise_the_placements(graphics.buffer);
    assert(summary.ordered && summary.placed > COUNT / 2);
    const char *last = graphics.buffer;
    for (const char *at = strstr(last, "a=p"); at != NULL; at = strstr(at + 3, "a=p")) last = at;
    char hawk_id[40];
    snprintf(hawk_id, sizeof(hawk_id), "I=%u,", (unsigned)hawk_image_id(&hawks[1]));
    const char *end = strstr(last, "\033\\");
    char *copy = strndup(last, (size_t)(end - last));
    assert(copy != NULL);
    assert(strstr(copy, hawk_id) != NULL && strstr(copy, "z=5") != NULL);
    free(copy);

    /* k summons another and K sends one away, as they do in the flat sky, and the
     * flight follows the count on the next frame. */
    hawk_sets_built = 1;
    assert(feed_input("k") == 1 && config.hawks == 3);
    fly(birds, snapshot, &grid);
    assert(sky.world.hawk_count == 3);
    assert(feed_input("KK") == 1 && config.hawks == 1);
    fly(birds, snapshot, &grid);
    assert(sky.world.hawk_count == 1);
    /* The hunt of the flat sky does not run in a space, whatever calls it. */
    hawk_t before = hawks[0];
    hunt(birds);
    assert(memcmp(&before, &hawks[0], sizeof(before)) == 0);

    kitty_graphics_destroy(&graphics);
    spatial_grid_destroy(&grid);
    free(snapshot);
    free(birds);
    config.hawks = 0;
    reset_test_config();
}

/* The pointer is a stick in the sky here too: the ray through it is the module's,
 * and the flock leaves its line. */
static void test_the_pointer_pokes_the_flock_in_a_space(void) {
    enum { COUNT = 400 };
    bird_t *birds = calloc(COUNT, sizeof(*birds)), *snapshot = malloc(sizeof(*snapshot) * COUNT);
    spatial_grid_t grid;
    double near_without = 0, near_with = 0;

    assert(birds != NULL && snapshot != NULL);
    for (int world = 0; world < 2; world++) {
        reset_test_config();
        sky_mode = 1;
        legend_enabled = 0;
        config.birds = COUNT;
        apply_screen_size(96, 32, 768, 512);
        set_frame_seconds(1.0 / FRAME_RATE);
        seed_random(12);
        mouse.present = 0;
        assert(spatial_grid_init(&grid, SPATIAL_CELL_SIZE) == SPATIAL_GRID_OK);
        assert(spatial_grid_prepare(&grid, screen.width, screen.height, COUNT) == SPATIAL_GRID_OK);
        initialize_birds(birds);
        sky_camera_t camera;
        sky_camera(&camera);
        /* The pointer in the middle of the flock's picture. */
        double middle_x = camera.centre_x, middle_y = camera.centre_y;
        for (int frame = 0; frame < 90; frame++) {
            if (world == 1) {
                mouse.present = 1;
                mouse.x = middle_x;
                mouse.y = middle_y;
            }
            fly(birds, snapshot, &grid);
        }
        /* How many birds are drawn within a bird's length of the middle: the ones the
         * stick has not cleared. */
        sky_camera(&camera);
        double origin[3], direction[3];
        sky_camera_ray(&camera, middle_x, middle_y, origin, direction);
        int near_the_line = 0;
        for (int i = 0; i < COUNT; i++) {
            const sky_bird_t *bird = &sky.world.birds[i];
            double rx = bird->x - origin[0], ry = bird->y - origin[1], rz = bird->z - origin[2];
            double along = rx * direction[0] + ry * direction[1] + rz * direction[2];
            double ox = rx - along * direction[0], oy = ry - along * direction[1],
                   oz = rz - along * direction[2];
            if (along > 0 && sqrt(ox * ox + oy * oy + oz * oz) < SKY_POKE_REACH / 2)
                near_the_line++;
        }
        if (world == 0)
            near_without = near_the_line;
        else
            near_with = near_the_line;
        spatial_grid_destroy(&grid);
        mouse.present = 0;
    }
    /* The ray is where the pointer says, and the flock is less in its way. */
    assert(near_without > 5);
    assert(near_with < near_without);
    free(snapshot);
    free(birds);
    reset_test_config();
}

/* A recording needs no terminal, in a space as anywhere: a GIF of sprites or of
 * braille, and a cast of braille, each of the size asked for, with no BOIDS written
 * before it starts. */
static void test_a_space_records_headless(void) {
    char path[600];
    static const char *NAMES[] = {"space.gif", "space.cast", "space-braille.gif"};
    static const int MODES[] = {RENDER_UNSET, RENDER_UNSET, RENDER_BRAILLE};

    for (int which = 0; which < 3; which++) {
        scratch_file(path, sizeof(path), NAMES[which]);
        reset_test_config();
        sky_mode = 1;
        config.birds = 120;
        config.bird_size = 0;
        config.palette = palette_named("ash");
        render_mode = MODES[which];
        record_path = path;
        record_fps = 20;
        record_seconds = 1;
        record_columns = 60;
        record_rows = 20;
        fflush(stdout);
        int saved = dup(STDOUT_FILENO);
        assert(freopen("/dev/null", "w", stdout) != NULL);
        int status = run_recording();
        fflush(stdout);
        dup2(saved, STDOUT_FILENO);
        close(saved);
        clearerr(stdout);
        assert(status == EXIT_SUCCESS);
        assert(formation.writing == 0);                     /* No letters: it is not a screen. */
        assert(sky.flying == 0 && sky.world.birds == NULL); /* And it cleaned up after itself. */

        FILE *file = fopen(path, "rb");
        assert(file != NULL);
        fseek(file, 0, SEEK_END);
        long length = ftell(file);
        rewind(file);
        assert(length > 1000);
        char head[8] = {0};
        assert(fread(head, 1, 6, file) == 6);
        if (which == 1) {
            assert(head[0] == '{');
            fseek(file, 0, SEEK_SET);
            static char text[1 << 16];
            int braille = 0, events = 0;
            while (fgets(text, sizeof(text), file) != NULL) {
                events++;
                for (const unsigned char *c = (const unsigned char *)text; *c; c++)
                    if (c[0] == 0xE2 && (c[1] & 0xFC) == 0xA0) braille++;
            }
            assert(events == 1 + 20 + 2 &&
                   braille > 100); /* A header, an opening, twenty frames, a closing. */
        } else {
            assert(memcmp(head, "GIF89a", 6) == 0);
        }
        fclose(file);
        remove(path);
        record_path = NULL;
    }
    render_mode = RENDER_KITTY;
    reset_test_config();
}

/* ---- Fireflies --------------------------------------------------------------- */

/* A night to run, headless, a frame at a time: what main does each frame, without
 * a terminal. */
typedef struct {
    bird_t *birds, *snapshot;
    spatial_grid_t grid;
} night_t;

static void begin_the_night(night_t *run, int columns, int rows, double fps, int seed) {
    reset_test_config();
    fireflies_mode = 1;
    config.birds = FIREFLY_COUNT;
    config.palette = palette_named("firefly");
    config.shape = shape_named("dot");
    config.bird_size = 0;
    config.pace_notch = DEFAULT_PACE_NOTCH; /* As shipped; the other tests run it faster. */
    apply_notches();
    settle_the_bird_size();
    legend_enabled = 0;
    mouse.present = 0;
    apply_screen_size(columns, rows, columns * DEFAULT_CELL_WIDTH, rows * DEFAULT_CELL_HEIGHT);
    set_frame_seconds(1.0 / fps);
    run->birds = calloc((size_t)config.birds, sizeof(*run->birds));
    run->snapshot = malloc(sizeof(*run->snapshot) * (size_t)config.birds);
    assert(run->birds != NULL && run->snapshot != NULL);
    assert(spatial_grid_init(&run->grid, SPATIAL_CELL_SIZE) == SPATIAL_GRID_OK);
    assert(spatial_grid_prepare(&run->grid, screen.width, screen.height, config.birds) ==
           SPATIAL_GRID_OK);
    seed_random((unsigned)seed);
    initialize_birds(run->birds);
}

static void one_night_frame(night_t *run) {
    memcpy(run->snapshot, run->birds, sizeof(*run->birds) * (size_t)config.birds);
    assert(spatial_grid_build(&run->grid, config.birds, read_bird_position, run->snapshot) ==
           SPATIAL_GRID_OK);
    fly(run->birds, run->snapshot, &run->grid);
}

static void end_the_night(night_t *run) {
    spatial_grid_destroy(&run->grid);
    free(run->snapshot);
    free(run->birds);
    mouse.present = 0;
    legend_enabled = 1;
    reset_test_config();
}

/* Seconds until the swarm's order passes 0.95, or -1; and what it started at. */
static double night_time_to_unison(int seed, double fps, double *start) {
    night_t run;
    begin_the_night(&run, 96, 26, fps, seed);
    one_night_frame(&run);
    *start = fireflies_order(&night);
    double when = -1;
    for (int frame = 1; frame < 90 * fps && when < 0; frame++) {
        one_night_frame(&run);
        if (fireflies_order(&night) > 0.95) when = (frame + 1) / fps;
    }
    end_the_night(&run);
    return when;
}

static void redirect_stderr_to(const char *path, int *saved) {
    fflush(stderr);
    *saved = dup(STDERR_FILENO);
    int file = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0600);
    assert(*saved >= 0 && file >= 0);
    assert(dup2(file, STDERR_FILENO) == STDERR_FILENO);
    close(file);
}

static void restore_stderr(int saved) {
    fflush(stderr);
    assert(dup2(saved, STDERR_FILENO) == STDERR_FILENO);
    close(saved);
}

static void read_text_file(const char *path, char *text, size_t size) {
    FILE *file = fopen(path, "r");
    assert(file != NULL);
    size_t length = fread(text, 1, size - 1, file);
    text[length] = '\0';
    fclose(file);
}

/* --fireflies is a switch in the Oddities, and it changes what a night wants for a
 * default and nothing else: the count, the size, the shape and the ramp, each only
 * if it was not asked for, and the ramp list has the same ten in the same order
 * with the new one after them. */
static void test_fireflies_change_the_defaults_and_nothing_else(void) {
    int saved_preset = requested_preset;
    char *plain[] = {"cbirds", NULL};
    char *night_only[] = {"cbirds", "--fireflies", NULL};
    char *asked[] = {"cbirds",  "--fireflies", "-n",      "800",   "-s", "30",
                     "--shape", "bird",        "--color", "theme", NULL};

    reset_test_config();
    config.bird_size = 0;
    config.shape = 0;
    config.palette = 0;
    read_options(1, plain);
    assert(!fireflies_mode && config.birds == 800 && config.shape == 0 && config.palette == 0);
    settle_the_bird_size();
    assert(config.bird_size == DEFAULT_BIRD_SIZE);

    reset_test_config();
    config.bird_size = 0;
    config.shape = 0;
    config.palette = 0;
    read_options(2, night_only);
    assert(fireflies_mode && config.birds == FIREFLY_COUNT);
    assert(config.shape == shape_named("dot") && strcmp(SHAPES[config.shape].name, "dot") == 0);
    assert(config.palette == palette_named("firefly"));
    settle_the_bird_size();
    assert(config.bird_size == FIREFLY_SIZE);
    /* Smaller than a bird: ten to fourteen pixels. */
    assert(FIREFLY_SIZE >= 10 && FIREFLY_SIZE <= 14 && (int)FIREFLY_SIZE < (int)DEFAULT_BIRD_SIZE);

    /* What was asked for wins, even when it is the very thing a flock ships with. */
    reset_test_config();
    config.bird_size = 0;
    config.shape = 0;
    config.palette = 0;
    read_options(10, asked);
    assert(fireflies_mode && config.birds == 800 && config.bird_size == 30);
    assert(config.shape == 0 && config.palette == 0);

    /* The ten ramps that were there are where they were, and the new ones follow:
     * firefly, then ink, which came after it. */
    static const char *const OLD[] = {"theme",  "ember", "ice",    "acid", "matrix",
                                      "aurora", "prism", "potion", "dusk", "ash"};
    assert(PALETTE_COUNT == 12);
    for (int i = 0; i < 10; i++) assert(strcmp(PALETTES[i].name, OLD[i]) == 0);
    assert(strcmp(PALETTES[10].name, "firefly") == 0 && palette_named("firefly") == 10);
    assert(strcmp(PALETTES[11].name, "ink") == 0 && palette_named("ink") == 11);
    /* Five shades, the brightest first, and none of them lost on a dark ground. */
    assert(PALETTES[10].shades == 5);
    static const uint8_t BLACK[3] = {0, 0, 0};
    for (int shade = 0; shade < 5; shade++) {
        assert(contrast_between(PALETTES[10].tints[shade], BLACK) >= 2.5);
        if (shade > 0)
            assert(luminance_of(PALETTES[10].tints[shade]) <
                   luminance_of(PALETTES[10].tints[shade - 1]));
    }
    /* Pale yellow to yellow green to dark green: red and green fall, in that order. */
    assert(PALETTES[10].tints[0][2] > 150 && PALETTES[10].tints[0][0] > 230);
    assert(PALETTES[10].tints[4][1] > PALETTES[10].tints[4][0] && PALETTES[10].tints[4][1] < 120);

    const option_t *option = NULL;
    for (int i = 0; i < OPTION_COUNT; i++)
        if (strcmp(OPTIONS[i].name, "fireflies") == 0) option = &OPTIONS[i];
    assert(option != NULL && option->kind == OPTION_FLAG && !option->essential);
    assert(strcmp(option->group, "Oddities") == 0);
    assert(strstr(option->help, "summer night") != NULL);

    requested_preset = saved_preset;
    reset_test_config();
}

/* Hawks do nothing in this mode, and say so; the same for the other switches that
 * are a flock's and not a swarm's, on the line and on the keyboard. */
static void test_a_night_leaves_the_flocks_switches_with_nothing_to_do(void) {
    char path[600], text[1024];
    int saved_preset = requested_preset;
    char *argv[] = {"cbirds",   "--fireflies", "--hawks", "2",        "--flocks", "3",
                    "--trails", "--preset",    "storm",   "--matrix", NULL};

    scratch_file(path, sizeof(path), "night_notes.txt");
    reset_test_config();
    int saved_stderr;
    redirect_stderr_to(path, &saved_stderr);
    read_options(10, argv);
    restore_stderr(saved_stderr);
    read_text_file(path, text, sizeof(text));
    remove(path);

    assert(fireflies_mode && config.hawks == 0 && config.flocks == 1 && !config.trails);
    assert(requested_preset == -1 && !matrix_mode && !the_rain_is_falling);
    assert(strstr(text, "--hawks does nothing with --fireflies") != NULL);
    assert(strstr(text, "--flocks does nothing with --fireflies") != NULL);
    assert(strstr(text, "--trails does nothing with --fireflies") != NULL);
    assert(strstr(text, "--preset does nothing with --fireflies") != NULL);
    assert(strstr(text, "--matrix does nothing with --fireflies") != NULL);
    /* Said once each, and nothing else said. */
    assert(strchr(strchr(text, '\n') + 1, '\n') != NULL);
    int lines = 0;
    for (const char *c = text; *c; c++) lines += *c == '\n';
    assert(lines == 5);

    /* Without them, silence. */
    char *quiet[] = {"cbirds", "--fireflies", NULL};
    reset_test_config();
    redirect_stderr_to(path, &saved_stderr);
    read_options(2, quiet);
    restore_stderr(saved_stderr);
    read_text_file(path, text, sizeof(text));
    remove(path);
    assert(text[0] == '\0');

    /* k does nothing, nor K, nor Tab, nor e, nor the code. */
    legend_enabled = 1;
    apply_screen_size(200, 50, 1600, 800);
    hawk_sets_built = 1;
    config.hawks = 0;
    assert(feed_input("kkK") == 1);
    assert(config.hawks == 0);
    config.trails = 0;
    assert(feed_input("e") == 1 && config.trails == 0);
    int boundary = config.boundary_notch, alignment = config.alignment_notch;
    int perception = config.vision_notch;
    assert(feed_input("\t") == 1);
    assert(config.boundary_notch == boundary && config.alignment_notch == alignment &&
           config.vision_notch == perception);
    konami_at = 0;
    memset(konami_seen, 0, sizeof(konami_seen));
    for (const char *c = "AABBDCDCba"; *c; c++) konami_note(*c);
    assert(config.hawks == 0);
    hawk_sets_built = 0;
    konami_at = 0;
    memset(konami_seen, 0, sizeof(konami_seen));

    /* Out of a night, they are what they were: the key brings a hawk. */
    fireflies_mode = 0;
    hawk_sets_built = 1;
    assert(feed_input("k") == 1 && config.hawks == 1);
    hawk_sets_built = 0;
    config.hawks = 0;
    requested_preset = saved_preset;
    reset_test_config();
}

/* The point of it: nothing in charge, and the swarm still falls into step. From a
 * random start the order is about a tenth or less, and it passes 0.95 within the
 * minute, over several seeds, with the fireflies drifting and the shipped push.
 * Measured over eight seeds at 96 by 26 cells: 17.0 to 37.7 seconds, 26.3 on
 * average, at 30 frames a second; 16.9 to 38.0, 25.6 on average, at 60. */
static void test_the_swarm_falls_into_step_with_nothing_in_charge(void) {
    double sum = 0;
    for (int seed = 1; seed <= 5; seed++) {
        double start;
        double when = night_time_to_unison(seed, 30, &start);
        assert(start < 0.15);
        assert(when > 5 && when < 60); /* Not at once, and within the minute. */
        sum += when / 5;
    }
    assert(sum > 12 && sum < 40); /* About half a minute. */

    /* And it is a climb, not a snap: a few seconds in, it is still disorder. */
    night_t run;
    begin_the_night(&run, 96, 26, 30, 1);
    for (int frame = 0; frame < 3 * 30; frame++) one_night_frame(&run);
    assert(fireflies_order(&night) < 0.6);
    /* Held once it is there: no dip below 0.9 over the next twenty seconds. */
    int frame = 3 * 30;
    while (fireflies_order(&night) < 0.97 && frame < 90 * 30) {
        one_night_frame(&run);
        frame++;
    }
    assert(fireflies_order(&night) >= 0.97);
    double least = 1;
    for (int i = 0; i < 20 * 30; i++) {
        one_night_frame(&run);
        double order = fireflies_order(&night);
        if (order < least) least = order;
    }
    assert(least > 0.9);
    end_the_night(&run);
}

/* The phases advance per second: the swarm that is the same at 30 frames a second
 * and at 60 reaches unison in the same time, to the scatter of a different step
 * each; and what moves the clocks is the clock and not the frame. */
static void test_the_swarm_keeps_time_in_seconds_not_frames(void) {
    double mean30 = 0, mean60 = 0;
    for (int seed = 1; seed <= 4; seed++) {
        double start;
        double a = night_time_to_unison(seed, 30, &start);
        double b = night_time_to_unison(seed, 60, &start);
        assert(a > 0 && b > 0);
        mean30 += a / 4;
        mean60 += b / 4;
    }
    assert(mean60 > mean30 * 0.7 && mean60 < mean30 * 1.43);

    /* With nobody to see, a firefly's phase is the time it has had over its
     * period, whatever the frame rate. */
    static const int RATES[] = {30, 60, 120};
    double phase[3];
    for (int r = 0; r < 3; r++) {
        night_t run;
        begin_the_night(&run, 96, 26, RATES[r], 2);
        config.alignment_notch = 0;
        apply_notches();
        one_night_frame(&run);
        double before = night.fly[7].phase;
        double period = night.fly[7].period;
        for (int frame = 1; frame < RATES[r] * 0.6; frame++) one_night_frame(&run);
        double seconds = (RATES[r] * 0.6 - 1) / RATES[r];
        /* With the coupling at its floor a flash is a nudge of a thousandth, and a
         * clock that saw none is a clock alone. */
        phase[r] = night.fly[7].phase - before - seconds / period;
        end_the_night(&run);
    }
    for (int r = 1; r < 3; r++) assert(fabs(phase[r] - phase[0]) < 0.05);
}

/* The pointer is a lantern: the fireflies it is held over are startled and their
 * clocks thrown, so the swarm round it falls out of step, and when it is taken
 * away the synchrony heals. */
static void test_the_lantern_scatters_the_phases_and_the_swarm_heals(void) {
    night_t run;
    begin_the_night(&run, 120, 34, 30, 3);
    one_night_frame(&run);
    /* Start in unison, so what is seen is what the lantern did. */
    for (int i = 0; i < config.birds; i++) {
        night.fly[i].phase = 0.5 + 0.002 * (i % 5);
        night.fly[i].age = night.fly[i].phase * night.fly[i].period;
    }
    assert(fireflies_order(&night) > 0.99);

    /* A run with no pointer holds. */
    for (int frame = 0; frame < 5 * 30; frame++) one_night_frame(&run);
    assert(fireflies_order(&night) > 0.95);

    /* Held over the middle of the screen for ten seconds. */
    mouse.present = 1;
    mouse.x = screen.width / 2.0;
    mouse.y = screen.height / 2.0;
    for (int frame = 0; frame < 10 * 30; frame++) one_night_frame(&run);
    double cx = 0, cy = 0, fx = 0, fy = 0;
    int under = 0, away = 0;
    for (int i = 0; i < config.birds; i++) {
        double dx = run.birds[i].x - mouse.x, dy = run.birds[i].y - mouse.y;
        double angle = 2 * M_PI * night.fly[i].phase;
        double reach = FIREFLY_LANTERN * firefly_spacing();
        if (dx * dx + dy * dy < reach * reach) {
            cx += cos(angle), cy += sin(angle);
            under++;
        } else if (dx * dx + dy * dy > 2.0 * reach * reach) {
            fx += cos(angle), fy += sin(angle);
            away++;
        }
    }
    assert(under > 0 && away > 0);
    assert(hypot(cx, cy) / under < 0.5);                  /* Under it: thrown. */
    assert(fireflies_order(&night) < 0.95);               /* So the swarm is out of step. */
    assert(hypot(fx, fy) / away > hypot(cx, cy) / under); /* And the rest less so. */

    /* The pointer goes, and the swarm falls into step again. */
    mouse.present = 0;
    double heal = -1;
    for (int frame = 0; frame < 90 * 30 && heal < 0; frame++) {
        one_night_frame(&run);
        if (fireflies_order(&night) > 0.95) heal = (frame + 1) / 30.0;
    }
    assert(heal > 0 && heal < 60);
    end_the_night(&run);
}

/* They drift: slowly, at a pace that follows the speed slider and the screen's
 * spacing, with no hawks, no leash and no alignment; they keep to the screen and
 * out of the panel, and lean towards the lower two thirds of the sky. */
static void test_the_fireflies_drift_slowly_and_keep_to_the_meadow(void) {
    night_t run;
    begin_the_night(&run, 120, 34, 30, 4);
    legend_enabled = 1;
    apply_screen_size(120, 34, 120 * 8, 34 * 16);
    assert(screen.legend_width > 0);
    initialize_birds(run.birds); /* Placed with the panel up, as a live run places them. */
    for (int i = 0; i < config.birds; i++)
        assert(!sprite_overlaps_legend(run.birds[i].x, run.birds[i].y));

    double spacing = firefly_spacing();
    double step = FIREFLY_DRIFT * spacing * (config.pace / DEFAULT_PACE) / 30.0;
    /* The same distance every frame, whichever way each is going. */
    one_night_frame(&run);
    bird_t before[FIREFLY_COUNT];
    memcpy(before, run.birds, sizeof(before));
    one_night_frame(&run);
    for (int i = 0; i < config.birds; i++) {
        double moved = hypot(run.birds[i].x - before[i].x, run.birds[i].y - before[i].y);
        assert(fabs(moved - step) < 1e-6);
    }
    /* Slow: about a spacing a second, which is a screen in half a minute. */
    assert(step * 30 < 1.5 * spacing && step * 30 > 0.5 * spacing);

    long outside = 0, low = 0, counted = 0;
    for (int frame = 0; frame < 40 * 30; frame++) {
        one_night_frame(&run);
        if (frame < 10 * 30) continue;
        for (int i = 0; i < config.birds; i++) {
            counted++;
            if (run.birds[i].x < 0 || run.birds[i].y < 0 || run.birds[i].x >= screen.width ||
                run.birds[i].y >= screen.height)
                outside++;
            if (run.birds[i].y > screen.height / 3.0) low++;
            assert(!sprite_overlaps_legend(run.birds[i].x, run.birds[i].y));
        }
    }
    assert(outside * 100 < counted); /* Under one in a hundred is off the screen. */
    /* A third of the area is the top third, and a uniform swarm would have two
     * thirds below it; the lean makes it more. */
    assert(low * 100 > counted * 70);

    /* A faster pace flies the same fireflies faster, in the same steps of time. */
    config.pace_notch = 7;
    apply_notches();
    step = FIREFLY_DRIFT * spacing * (config.pace / DEFAULT_PACE) / 30.0;
    memcpy(before, run.birds, sizeof(before));
    one_night_frame(&run);
    double moved = 0;
    for (int i = 0; i < config.birds; i++)
        moved += hypot(run.birds[i].x - before[i].x, run.birds[i].y - before[i].y) / config.birds;
    assert(fabs(moved - step) < step * 0.01);
    end_the_night(&run);
}

/* A firefly that is dark is a faint body and not nothing and not a bird: the one
 * that is lit has a shade of the ramp, and the picture shows both. */
static void test_a_dark_firefly_is_a_faint_body(void) {
    night_t run;
    static png_image_t frames[ROTATION_FRAMES * MAX_SPRITE_SETS];
    png_image_t canvas = {0, 0, NULL};

    begin_the_night(&run, 96, 26, 30, 5);
    one_night_frame(&run);
    int lit = -1, dark = -1;
    for (int i = 0; i < config.birds; i++) {
        if (run.birds[i].shade == 0 && lit < 0) lit = i; /* The flash itself. */
        if (run.birds[i].shade < 0 && dark < 0) dark = i;
    }
    assert(lit >= 0 && dark >= 0);
    /* Every shade of the ramp is on the screen in the very first frame, which is
     * what the colour table of a recording is built from. */
    int seen[5] = {0};
    for (int i = 0; i < config.birds; i++)
        if (run.birds[i].shade >= 0) seen[run.birds[i].shade]++;
    for (int shade = 0; shade < 5; shade++) assert(seen[shade] > 5);

    assert(rasterise_sprites(frames) == PNG_OK);
    assert(png_image_alloc(&canvas, screen.width, screen.height) == PNG_OK);
    compose_onto(&canvas, frames, run.birds, 1);
    /* A dark one's own pixels are not the ground and not the brightest. */
    const bird_t *body = &run.birds[dark];
    int inset = firefly_body_inset();
    int cx = (int)body->x + inset + trail_sprite_size() / 2;
    int cy = (int)body->y + inset + trail_sprite_size() / 2;
    assert(cx < screen.width && cy < screen.height);
    const uint8_t *pixel = canvas.pixels + ((size_t)cy * (size_t)canvas.width + (size_t)cx) * 4;
    int brightness = pixel[0] + pixel[1] + pixel[2];
    assert(brightness > 18 + 18 + 24); /* Something is there. */
    assert(brightness < 3 * 100);      /* And it is faint. */
    /* And its middle is greener than red: it is the ramp's last shade, faded. */
    assert(pixel[1] >= pixel[0]);
    /* The body is smaller than the dot of a flash. */
    assert(frames[trail_set(0) * ROTATION_FRAMES].width < config.bird_size);

    /* A lit one is bright at its middle. */
    const bird_t *glow = &run.birds[lit];
    cx = (int)glow->x + config.bird_size / 2;
    cy = (int)glow->y + config.bird_size / 2;
    if (cx < screen.width && cy < screen.height) {
        pixel = canvas.pixels + ((size_t)cy * (size_t)canvas.width + (size_t)cx) * 4;
        assert(pixel[0] + pixel[1] + pixel[2] > 3 * 90);
    }
    png_image_free(&canvas);
    free_sprites(frames);
    end_the_night(&run);
}

/* The panel says what its rows do here, and does it: alignment is coupling and
 * perception is sight, each does what it says, and there is a row that reads out
 * how much of the swarm is in step. Every row is as wide as the panel. */
static void test_the_panel_says_what_a_night_does(void) {
    char lines[LEGEND_MAX_ROWS][LEGEND_LINE_MAX];
    night_t run;

    begin_the_night(&run, 200, 50, 60, 6);
    legend_enabled = 1;
    apply_screen_size(200, 50, 1600, 800);
    assert(legend_rows() == LEGEND_MAX_ROWS &&
           screen.legend_height == LEGEND_MAX_ROWS * screen.cell_height);
    one_night_frame(&run);
    build_legend(lines);
    for (int row = 0; row < LEGEND_MAX_ROWS; row++)
        assert(legend_cells(lines[row]) == LEGEND_COLUMNS);
    assert(strstr(lines[1], "boundary") != NULL && strstr(lines[2], "separation") != NULL);
    assert(strstr(lines[3], "coupling") != NULL && strstr(lines[3], "a/A") != NULL);
    assert(strstr(lines[3], "alignment") == NULL);
    assert(strstr(lines[4], "turning") != NULL);
    assert(strstr(lines[5], "sight") != NULL && strstr(lines[5], "p/P") != NULL);
    assert(strstr(lines[5], "perception") == NULL);
    assert(strstr(lines[6], "speed") != NULL);
    assert(strstr(lines[7], "sync") != NULL);
    assert(strstr(lines[7], "/") == NULL); /* A reading has no keys. */
    assert(strstr(lines[8], "frame") != NULL && strstr(lines[9], "quit") != NULL);
    assert(strstr(lines[3], "1.0×") != NULL);
    char shown[16];
    snprintf(shown, sizeof(shown), "%.0fpx", firefly_law().sight);
    assert(strstr(lines[5], shown) != NULL);

    /* The sync row is the order parameter, as a bar and as a number. */
    double order = fireflies_order(&night);
    snprintf(shown, sizeof(shown), "%.2f", order);
    assert(strstr(lines[7], shown) != NULL);
    assert(filled_cells(lines[7]) == (int)(order * LEGEND_BAR_CELLS + 0.5));
    for (int i = 0; i < config.birds; i++) night.fly[i].phase = 0.3;
    build_legend(lines);
    assert(filled_cells(lines[7]) == LEGEND_BAR_CELLS && strstr(lines[7], "1.00") != NULL);

    /* A row does what it says: coupling is the push of a flash, sight is how far one
     * is seen, and each moves with its keys, a notch at a time, to both ends. */
    double push_at[LEGEND_BAR_CELLS + 1], sight_at[LEGEND_BAR_CELLS + 1];
    for (int notch = 0; notch <= LEGEND_BAR_CELLS; notch++) {
        config.alignment_notch = config.vision_notch = notch;
        apply_notches();
        push_at[notch] = firefly_law().push;
        sight_at[notch] = firefly_law().sight;
        build_legend(lines);
        assert(filled_cells(lines[3]) == notch && filled_cells(lines[5]) == notch);
        for (int row = 0; row < LEGEND_MAX_ROWS; row++)
            assert(legend_cells(lines[row]) == LEGEND_COLUMNS);
    }
    for (int notch = 1; notch <= LEGEND_BAR_CELLS; notch++) {
        assert(push_at[notch] > push_at[notch - 1] && sight_at[notch] > sight_at[notch - 1]);
    }
    /* At the default notches, the shipped push and three spacings and a bit of sight. */
    assert(fabs(push_at[DEFAULT_NOTCH] - FIREFLY_PUSH) < 1e-12);
    assert(fabs(sight_at[DEFAULT_VISION_NOTCH] - FIREFLY_SIGHT * firefly_spacing()) < 1e-9);
    assert(push_at[0] <
           0.1 * FIREFLY_PUSH); /* The floor of the bar is no coupling worth the name. */
    config.alignment_notch = DEFAULT_NOTCH;
    config.vision_notch = DEFAULT_VISION_NOTCH;
    apply_notches();

    /* The rows without a meaning here are not on it: there is no avoidance row to
     * move, and g and G do nothing with one swarm. */
    int avoid = config.avoid_notch;
    assert(feed_input("gG") == 1 && config.avoid_notch == avoid);
    /* And the keys really are the ones on the panel. */
    int notch = config.alignment_notch;
    assert(feed_input("A") == 1 && config.alignment_notch == notch + 1);
    notch = config.vision_notch;
    assert(feed_input("p") == 1 && config.vision_notch == notch - 1);

    /* On a very large screen the sight is in thousands of pixels, and still fits. */
    config.vision_notch = LEGEND_BAR_CELLS;
    apply_notches();
    apply_screen_size(1000, 250, 8000, 4000);
    build_legend(lines);
    assert(firefly_law().sight > 1000);
    for (int row = 0; row < LEGEND_MAX_ROWS; row++)
        assert(legend_cells(lines[row]) == LEGEND_COLUMNS);
    assert(strstr(lines[5], "k") != NULL);
    end_the_night(&run);

    /* And a flock's panel is the one it was: alignment, perception, avoidance. */
    reset_test_config();
    legend_enabled = 1;
    apply_screen_size(200, 50, 1600, 800);
    assert(legend_rows() == LEGEND_ROWS);
    build_legend(lines);
    assert(strstr(lines[3], "alignment") != NULL && strstr(lines[5], "perception") != NULL);
    assert(strstr(lines[7], "sync") == NULL);
}

/* The sight and the pace are measured in spacings, so the same swarm falls into step
 * in the same time on a small screen and a large one: with the sight in pixels it
 * was a second and a half at 768 by 416 and never at 3000 by 1900. */
static void test_the_swarm_is_the_same_swarm_on_any_screen(void) {
    static const int SIZES[3][2] = {{80, 24}, {200, 50}, {320, 100}};
    for (int s = 0; s < 3; s++) {
        night_t run;
        begin_the_night(&run, SIZES[s][0], SIZES[s][1], 30, 1);
        double reached = -1;
        for (int frame = 0; frame < 75 * 30 && reached < 0; frame++) {
            one_night_frame(&run);
            if (fireflies_order(&night) > 0.95) reached = (frame + 1) / 30.0;
        }
        assert(reached > 5 && reached < 75);
        /* A flash is in sight of about thirty of the others, whatever the screen. */
        double sight = firefly_law().sight;
        double neighbours =
            M_PI * sight * sight / ((double)screen.width * screen.height / config.birds);
        assert(neighbours > 28 && neighbours < 36);
        end_the_night(&run);
    }
}

/* A night records, in a GIF and in a cast, and the GIF's colour table is built
 * from its first frame: the first frame holds every shade of the ramp, so the
 * flashes come out in their own colours and not in the nearest the first frame
 * could spare. */
static void test_a_night_records_with_the_whole_ramp_in_its_colour_table(void) {
    char path[600];
    scratch_file(path, sizeof(path), "night.gif");
    reset_test_config();
    fireflies_mode = 1;
    config.birds = FIREFLY_COUNT;
    config.palette = palette_named("firefly");
    config.shape = shape_named("dot");
    config.bird_size = 0;
    record_path = path;
    record_fps = 25;
    record_seconds = 2;
    record_columns = 96;
    record_rows = 26;
    requested_seed = 7;
    fflush(stdout);
    int saved = dup(STDOUT_FILENO);
    assert(freopen("/dev/null", "w", stdout) != NULL);
    int status = run_recording();
    fflush(stdout);
    dup2(saved, STDOUT_FILENO);
    close(saved);
    clearerr(stdout);
    assert(status == EXIT_SUCCESS);

    FILE *file = fopen(path, "rb");
    assert(file != NULL);
    uint8_t header[13 + 768];
    assert(fread(header, 1, sizeof(header), file) == sizeof(header));
    assert(memcmp(header, "GIF89a", 6) == 0);
    assert((header[6] | header[7] << 8) == 96 * DEFAULT_CELL_WIDTH);
    int descriptors = 0, c;
    while ((c = fgetc(file)) != EOF)
        if (c == 0x2C) descriptors++;
    fclose(file);
    remove(path);
    assert(descriptors >= 50);
    const uint8_t(*table)[3] = (const uint8_t(*)[3])(header + 13);
    for (int shade = 0; shade < PALETTES[10].shades; shade++) {
        double nearest = 1e9;
        for (int entry = 0; entry < 256; entry++) {
            double gap = sqrt(pow(table[entry][0] - (int)PALETTES[10].tints[shade][0], 2) +
                              pow(table[entry][1] - (int)PALETTES[10].tints[shade][1], 2) +
                              pow(table[entry][2] - (int)PALETTES[10].tints[shade][2], 2));
            if (gap < nearest) nearest = gap;
        }
        assert(nearest < 12); /* Within what five bits a channel can hold apart. */
    }

    /* A cast of a night is braille, with something lit in it. */
    scratch_file(path, sizeof(path), "night.cast");
    reset_test_config();
    fireflies_mode = 1;
    config.birds = FIREFLY_COUNT;
    config.palette = palette_named("firefly");
    config.shape = shape_named("dot");
    config.bird_size = 0;
    record_path = path;
    record_fps = 20;
    record_seconds = 1;
    record_columns = 60;
    record_rows = 20;
    fflush(stdout);
    saved = dup(STDOUT_FILENO);
    assert(freopen("/dev/null", "w", stdout) != NULL);
    status = run_recording();
    fflush(stdout);
    dup2(saved, STDOUT_FILENO);
    close(saved);
    clearerr(stdout);
    assert(status == EXIT_SUCCESS);
    file = fopen(path, "r");
    assert(file != NULL);
    static char line[1 << 16];
    int events = 0, braille = 0;
    assert(fgets(line, sizeof(line), file) != NULL);
    while (fgets(line, sizeof(line), file) != NULL) {
        events++;
        for (const char *p = line; *p; p++)
            if ((unsigned char)p[0] == 0xE2 && ((unsigned char)p[1] & 0xFC) == 0xA0) braille++;
    }
    fclose(file);
    remove(path);
    assert(events == 20 + 2 && braille > 100);

    record_path = NULL;
    record_fps = 25;
    record_seconds = 6;
    record_columns = 96;
    record_rows = 26;
    requested_seed = -1;
    reset_test_config();
}

/* And it benchmarks, headless, and says how in step it got. */
static void test_a_night_benchmarks_and_reports_its_sync(void) {
    char path[600], text[2048];
    scratch_file(path, sizeof(path), "night_bench.txt");
    reset_test_config();
    fireflies_mode = 1;
    config.birds = FIREFLY_COUNT;
    config.palette = palette_named("firefly");
    config.shape = shape_named("dot");
    config.bird_size = 0;
    bench_frames = 60;
    render_mode = RENDER_UNSET;
    fflush(stdout);
    int saved = dup(STDOUT_FILENO);
    assert(freopen(path, "w", stdout) != NULL);
    int status = run_benchmark();
    fflush(stdout);
    dup2(saved, STDOUT_FILENO);
    close(saved);
    clearerr(stdout);
    assert(status == EXIT_SUCCESS);
    read_text_file(path, text, sizeof(text));
    remove(path);
    assert(strstr(text, "birds        400") != NULL);
    assert(strstr(text, "sync         0.") != NULL);
    /* The flock's bench says nothing about sync. */
    bench_frames = 0;
    render_mode = RENDER_KITTY;
    reset_test_config();
}

/* The second sky: with --depth the far fireflies are a swarm of their own, smaller
 * and dimmer, and neither sees the other's flashes. */
static void test_a_far_swarm_flashes_to_itself(void) {
    night_t run;
    deep_look = 0;
    begin_the_night(&run, 96, 26, 30, 8);
    deep_look = 1;
    for (int i = 0; i < config.birds; i++) place_one_firefly(&run.birds[i]);
    int far = 0;
    for (int i = 0; i < config.birds; i++) far += run.birds[i].layer;
    assert(far > 80 && far < 200); /* About a third of 400. */
    one_night_frame(&run);
    for (int i = 0; i < config.birds; i++) assert(night.fly[i].sky == run.birds[i].layer);
    deep_look = 0;
    end_the_night(&run);
}

/* Left alone for a minute a flock starts moving its sliders by itself; a night does
 * not, because its coupling is the thing on show. And leaving, with q, the
 * fireflies go up still flashing. */
static void test_a_night_is_left_alone_and_flies_off_still_flashing(void) {
    night_t run;
    begin_the_night(&run, 96, 26, 30, 9);
    one_night_frame(&run);

    int before[4] = {config.boundary_notch, config.separation_notch, config.alignment_notch,
                     config.vision_notch};
    last_key_at = 0;
    last_drift_at = 0;
    clock_state.seconds = 10 * IDLE_SECONDS;
    for (int i = 0; i < 20; i++) {
        clock_state.seconds += AUTOPILOT_PERIOD;
        maybe_drift();
    }
    assert(before[0] == config.boundary_notch && before[1] == config.separation_notch &&
           before[2] == config.alignment_notch && before[3] == config.vision_notch);
    fireflies_mode = 0; /* A flock, in the same minute, does move them. */
    last_drift_at = 0;
    for (int i = 0; i < 20; i++) {
        clock_state.seconds += AUTOPILOT_PERIOD;
        maybe_drift();
    }
    assert(before[0] != config.boundary_notch || before[1] != config.separation_notch ||
           before[2] != config.alignment_notch || before[3] != config.vision_notch);
    fireflies_mode = 1;
    config.boundary_notch = before[0];
    config.separation_notch = before[1];
    config.alignment_notch = before[2];
    config.vision_notch = before[3];
    apply_notches();
    clock_state.seconds = 0;

    double phase = night.fly[3].phase, y = run.birds[3].y;
    fly_away(run.birds);
    assert(run.birds[3].y < y);
    assert(night.fly[3].phase != phase);
    end_the_night(&run);
}

/* Under Kitty a lit firefly is a placement of its shade of the ramp, a dark one a
 * placement of its body, set in by half the difference of their sizes, and none is
 * left out. */
static void test_the_sprites_of_a_night_are_placed_lit_or_as_bodies(void) {
    kitty_graphics_t graphics;
    bird_t birds[3] = {{.x = 100, .y = 50, .shade = 2, .frame = 5},
                       {.x = 200, .y = 100, .shade = -1, .frame = 5},
                       {.x = 300, .y = 150, .shade = 0, .frame = 7}};
    char expected[96];

    reset_test_config();
    fireflies_mode = 1;
    config.birds = 3;
    config.palette = palette_named("firefly");
    config.bird_size = 0;
    settle_the_bird_size();
    apply_screen_size(80, 24, 640, 384);
    legend_drawn = 0;
    assert(kitty_graphics_init(&graphics, STDOUT_FILENO) == KITTY_GRAPHICS_OK);
    assert(queue_render_frame(&graphics, birds) == KITTY_GRAPHICS_OK);

    snprintf(expected, sizeof(expected), "a=p,I=%u,q=2,X=4,Y=2,C=1",
             set_image_id(flock_set(2, 0, 0), 5));
    assert(strstr(graphics.buffer, expected) != NULL);
    int inset = firefly_body_inset();
    assert(inset > 0 && trail_sprite_size() < config.bird_size);
    snprintf(expected, sizeof(expected), "a=p,I=%u,q=2,X=%d,Y=%d,C=1",
             set_image_id(trail_set(0), 5), (200 + inset) % 8, (100 + inset) % 16);
    assert(strstr(graphics.buffer, expected) != NULL);
    snprintf(expected, sizeof(expected), "a=p,I=%u,q=2,", set_image_id(flock_set(0, 0, 0), 7));
    assert(strstr(graphics.buffer, expected) != NULL);
    /* Three fireflies, three placements, and the dark one is not a bird of any shade. */
    int placements = 0;
    for (const char *at = graphics.buffer; (at = strstr(at, "a=p,")) != NULL; at++) placements++;
    assert(placements == 3);

    kitty_graphics_destroy(&graphics);
    reset_test_config();
}

/* What a night shows of three sliders, over forty seconds with the others at their
 * defaults: the share of the swarm within a few pixels of an edge, how far a
 * firefly is from its nearest neighbour, and how much it turns a second. */
typedef struct {
    double near_edge, nearest, turned;
} night_look_t;

static night_look_t look_at_a_night(int boundary, int separation, int turning) {
    night_t run;
    night_look_t look = {0, 0, 0};
    long samples = 0;

    begin_the_night(&run, 96, 26, 30, 3);
    config.boundary_notch = boundary;
    config.separation_notch = separation;
    config.turning_notch = turning;
    apply_notches();
    for (int frame = 0; frame < 40 * 30; frame++) {
        one_night_frame(&run);
        for (int i = 0; i < config.birds; i++) {
            double change = run.birds[i].direction - run.snapshot[i].direction;
            look.turned += fabs(atan2(sin(change), cos(change))) / config.birds / 40;
        }
        if (frame < 10 * 30 || frame % 30) continue;
        for (int i = 0; i < config.birds; i++) {
            const bird_t *a = &run.birds[i];
            double least = 1e18;
            for (int j = 0; j < config.birds; j++) {
                if (j == i) continue;
                double dx = a->x - run.birds[j].x, dy = a->y - run.birds[j].y;
                if (dx * dx + dy * dy < least) least = dx * dx + dy * dy;
            }
            double edge = fmin(fmin(a->x, screen.width - a->x), fmin(a->y, screen.height - a->y));
            look.near_edge += edge < 0.03 * screen.height;
            look.nearest += sqrt(least);
            samples++;
        }
    }
    look.near_edge /= samples;
    look.nearest /= samples;
    end_the_night(&run);
    return look;
}

/* A row does what it says: boundary keeps them off the edges, separation keeps them
 * apart, turning is how much they wander. The other three are in the panel test:
 * coupling and sight are the law, and the speed is the step. Measured: 2.9% of the
 * swarm at an edge with the boundary at nothing and none at the top; the nearest
 * neighbour 12.3 pixels off at nothing and 17.2 at the top; 2.3 radians of turning
 * a second and 5.6. */
static void test_every_slider_does_what_it_says_to_a_night(void) {
    night_look_t no_boundary = look_at_a_night(0, DEFAULT_NOTCH, DEFAULT_TURNING_NOTCH);
    night_look_t full_boundary =
        look_at_a_night(LEGEND_BAR_CELLS, DEFAULT_NOTCH, DEFAULT_TURNING_NOTCH);
    assert(no_boundary.near_edge > full_boundary.near_edge + 0.01);

    night_look_t no_room = look_at_a_night(DEFAULT_NOTCH, 0, DEFAULT_TURNING_NOTCH);
    night_look_t all_room = look_at_a_night(DEFAULT_NOTCH, LEGEND_BAR_CELLS, DEFAULT_TURNING_NOTCH);
    assert(all_room.nearest > no_room.nearest * 1.2);

    night_look_t lazy = look_at_a_night(DEFAULT_NOTCH, DEFAULT_NOTCH, 0);
    night_look_t sharp = look_at_a_night(DEFAULT_NOTCH, DEFAULT_NOTCH, LEGEND_BAR_CELLS);
    assert(sharp.turned > lazy.turned * 1.5);
}

/*
 * Escape waves.
 */

/* A row of birds standing still, a few pixels apart, with the first of them
 * alarmed. They are held still because what is measured is the wave: a bird
 * that flew out of sight of the next would be measuring the flock. */
enum { LINE_BIRDS = 80 };
static bird_t line[LINE_BIRDS];
static spatial_grid_t line_grid;
static double line_now; /* Seconds of flight since the alarm. */

static void set_the_line(double gap, double frame_rate, int pace_notch) {
    reset_test_config();
    reset_the_waves();
    legend_enabled = 0;
    apply_screen_size(200, 50, 1600, 800);
    config.pace_notch = pace_notch;
    apply_notches();
    set_frame_seconds(1.0 / frame_rate);
    config.birds = LINE_BIRDS;
    for (int i = 0; i < LINE_BIRDS; i++)
        line[i] = (bird_t){.x = 100 + i * gap, .y = 400, .direction = 0.1 * i};
    assert(spatial_grid_init(&line_grid, SPATIAL_CELL_SIZE) == SPATIAL_GRID_OK);
    assert(spatial_grid_prepare(&line_grid, screen.width, screen.height, LINE_BIRDS) ==
           SPATIAL_GRID_OK);
    assert(spatial_grid_build(&line_grid, LINE_BIRDS, read_bird_position, line) == SPATIAL_GRID_OK);
    /* Caught, and about to begin: the first step begins it. */
    assert(wave_catch(&waves[0], SWERVE_ANGLE, 1e-6));
    waves_in_flight = 1;
    line_now = 0;
}

/* Runs the alarm for so many steps. Says how many birds began in them, and
 * when each did, in seconds of flight since the alarm; a bird that has not has
 * a negative time. No bird may begin twice in a run. */
static int run_the_line(int steps, double began[LINE_BIRDS]) {
    int begun = 0;
    for (int i = 0; i < LINE_BIRDS; i++) began[i] = -1;
    for (int step = 0; step < steps; step++) {
        spread_the_alarm(line, &line_grid);
        for (int k = 0; k < wave_task_count; k++) {
            int bird = wave_tasks[k].bird;
            assert(began[bird] < 0);
            began[bird] = line_now + wave_tasks[k].at;
            begun++;
        }
        line_now += flight_seconds();
    }
    return begun;
}

static void put_the_line_away(void) {
    spatial_grid_destroy(&line_grid);
    reset_the_waves();
    reset_test_config();
}

/* A wave runs along a line of birds at the speed it is given, WAVE_PACE times
 * the speed a bird flies at, less a little, because the farthest bird in sight
 * is never quite at the edge of it: measured at 7.4 to 7.9 thousand pixels a
 * second of flight, 3.1 to 3.3 times the birds' 2.4 thousand. */
static void test_a_wave_crosses_a_line_of_birds_faster_than_they_fly(void) {
    double began[LINE_BIRDS];
    static const double gaps[] = {6, 9, 12};
    for (size_t g = 0; g < sizeof(gaps) / sizeof(*gaps); g++) {
        set_the_line(gaps[g], 60, DEFAULT_NOTCH);
        assert(run_the_line(120, began) == LINE_BIRDS);
        double bird_speed = config.speed / flight_seconds();
        double front = (LINE_BIRDS - 1) * gaps[g] / (began[LINE_BIRDS - 1] - began[0]);
        assert(front >= 3.0 * bird_speed);
        assert(front <= WAVE_PACE * bird_speed * 1.001);
        /* And it is a front: later is further, bird after bird. */
        for (int i = 1; i < LINE_BIRDS; i++) assert(began[i] >= began[i - 1] - 1e-12);
        put_the_line_away();
    }
    /* Further than a bird sees it does not go: a gap wider than the sight stops
     * the wave dead, which is why the sight is wider than the perception. */
    double sight = WAVE_SIGHT * config.vision_radius;
    set_the_line(sight + 1, 60, DEFAULT_NOTCH);
    assert(run_the_line(120, began) == 1);
    put_the_line_away();
    set_the_line(sight - 1, 60, DEFAULT_NOTCH);
    assert(run_the_line(240, began) == LINE_BIRDS);
    put_the_line_away();
}

/* The same pixels a second at any frame rate: the moment each bird begins is
 * the same at thirty frames a second as at sixty, and at twenty five, and at a
 * hundred and twenty, because the wave is told inside the step and not a hop to
 * a step, which would take a sixtieth of a second a hop at one rate and a
 * thirtieth at the other. And at any pace, which scales the flock's time and the
 * wave's with it: the same moments in seconds of flight. */
static void test_the_wave_covers_the_same_ground_at_thirty_and_sixty_frames_a_second(void) {
    double reference[LINE_BIRDS], other[LINE_BIRDS];
    set_the_line(6, 60, DEFAULT_NOTCH);
    assert(run_the_line(120, reference) == LINE_BIRDS);
    put_the_line_away();

    static const double rates[] = {25, 30, 33, 120};
    for (size_t r = 0; r < sizeof(rates) / sizeof(*rates); r++) {
        set_the_line(6, rates[r], DEFAULT_NOTCH);
        assert(run_the_line((int)(rates[r] * 2), other) == LINE_BIRDS);
        for (int i = 0; i < LINE_BIRDS; i++) assert(fabs(other[i] - reference[i]) < 1e-9);
        put_the_line_away();
    }
    /* Flown at 0.4 of the pace, a step is 0.4 of the flight. */
    set_the_line(6, 60, 1);
    assert(fabs(config.pace - 0.4) < 1e-9);
    assert(run_the_line(300, other) == LINE_BIRDS);
    for (int i = 0; i < LINE_BIRDS; i++) assert(fabs(other[i] - reference[i]) < 1e-9);
    put_the_line_away();
}

/* The same on a flock and not a line: settled for a few seconds, held still, and
 * one bird in the middle of it alarmed, at two frame rates. Birds are found by
 * more than one that has begun, and told by the one that began soonest, not the
 * one the step happened to look at first, so every bird begins at the same moment
 * at either rate and the same birds are reached. */
static int alarm_across_a_flock(bird_t *flock, int count, double frame_rate, double began[]) {
    spatial_grid_t grid;
    set_frame_seconds(1.0 / frame_rate);
    reset_the_waves();
    assert(spatial_grid_init(&grid, SPATIAL_CELL_SIZE) == SPATIAL_GRID_OK);
    assert(spatial_grid_prepare(&grid, screen.width, screen.height, count) == SPATIAL_GRID_OK);
    assert(spatial_grid_build(&grid, count, read_bird_position, flock) == SPATIAL_GRID_OK);
    double middle_x = 0, middle_y = 0;
    for (int i = 0; i < count; i++) {
        middle_x += flock[i].x / count;
        middle_y += flock[i].y / count;
        began[i] = -1;
    }
    int nearest = 0;
    for (int i = 1; i < count; i++)
        if (hypot(flock[i].x - middle_x, flock[i].y - middle_y) <
            hypot(flock[nearest].x - middle_x, flock[nearest].y - middle_y))
            nearest = i;
    assert(wave_catch(&waves[nearest], SWERVE_ANGLE, 1e-6));
    waves_in_flight = 1;
    double now = 0;
    int reached = 0;
    for (int step = 0; step < (int)(0.6 / flight_seconds()); step++) {
        spread_the_alarm(flock, &grid);
        for (int k = 0; k < wave_task_count; k++) {
            began[wave_tasks[k].bird] = now + wave_tasks[k].at;
            reached++;
        }
        now += flight_seconds();
    }
    spatial_grid_destroy(&grid);
    return reached;
}

static void test_a_wave_crosses_a_flock_at_the_same_moments_at_any_frame_rate(void) {
    enum { COUNT = 200 };
    static bird_t flock[COUNT], snapshot[COUNT];
    double slow[COUNT], fast[COUNT];
    spatial_grid_t grid;

    reset_test_config();
    reset_the_waves();
    legend_enabled = 0;
    apply_screen_size(96, 32, 768, 512);
    config.birds = COUNT;
    config.bird_size = 14;
    config.pace_notch = 0;
    apply_notches();
    seed_random(4);
    for (int i = 0; i < COUNT; i++) place_one_bird(&flock[i], i);
    assert(spatial_grid_init(&grid, SPATIAL_CELL_SIZE) == SPATIAL_GRID_OK);
    assert(spatial_grid_prepare(&grid, screen.width, screen.height, COUNT) == SPATIAL_GRID_OK);
    for (int frame = 0; frame < 200; frame++) {
        memcpy(snapshot, flock, sizeof(flock));
        assert(spatial_grid_build(&grid, COUNT, read_bird_position, snapshot) == SPATIAL_GRID_OK);
        fly(flock, snapshot, &grid);
    }
    spatial_grid_destroy(&grid);
    int reached = alarm_across_a_flock(flock, COUNT, 60, fast);
    assert(reached > COUNT / 2); /* A wave that reached nobody would agree with itself. */
    assert(alarm_across_a_flock(flock, COUNT, 30, slow) == reached);
    for (int i = 0; i < COUNT; i++) assert(fabs(slow[i] - fast[i]) < 1e-9);
    assert(alarm_across_a_flock(flock, COUNT, 25, slow) == reached);
    for (int i = 0; i < COUNT; i++) assert(fabs(slow[i] - fast[i]) < 1e-9);
    reset_the_waves();
    reset_test_config();
}

/* A bird alarmed by a neighbour swerves the way the neighbour does, not the way
 * the hawk would have it: copying is what makes it a wave and not a burst. */
static void test_a_bird_alarmed_by_a_neighbour_copies_its_swerve(void) {
    enum { COUNT = 5 };
    bird_t birds[COUNT];
    spatial_grid_t grid;

    reset_test_config();
    reset_the_waves();
    legend_enabled = 0;
    apply_screen_size(200, 50, 1600, 800);
    config.birds = COUNT;
    config.hawks = 1;
    place_hawks();
    /* A hawk diving east along y = 400, with an alarm radius of 90. Bird 0 is
     * under the line and bird 1 over it, both within the radius, so each swerves
     * from it to the side it is on. Bird 2 is out of the radius, below the line,
     * and in sight of bird 1 alone: a bird that had worked it out for itself would
     * swerve the way bird 0 does, and it swerves the way bird 1 does. */
    assert(ALARM_SHARE * hawk_reach() == 90);
    hawks[0] = (hawk_t){.x = 400, .y = 400, .direction = 0, .diving = 1};
    birds[0] = (bird_t){.x = 400, .y = 470, .direction = 1.0}; /* Below the line. */
    birds[1] = (bird_t){.x = 460, .y = 370, .direction = 2.0}; /* Above it. */
    birds[2] = (bird_t){.x = 500, .y = 440, .direction = 3.0}; /* Out of the hawk's reach. */
    birds[3] = (bird_t){.x = 300, .y = 700, .direction = 0.5}; /* Nowhere near anything. */
    birds[4] = (bird_t){.x = 460, .y = 160, .direction = 5.0}; /* Out of everyone's sight. */
    assert(spatial_grid_init(&grid, SPATIAL_CELL_SIZE) == SPATIAL_GRID_OK);
    assert(spatial_grid_prepare(&grid, screen.width, screen.height, COUNT) == SPATIAL_GRID_OK);
    assert(spatial_grid_build(&grid, COUNT, read_bird_position, birds) == SPATIAL_GRID_OK);
    for (int frame = 0; frame < 60; frame++) spread_the_alarm(birds, &grid);

    assert(waves[0].swerve == SWERVE_ANGLE);  /* Below the line, so clockwise from it. */
    assert(waves[1].swerve == -SWERVE_ANGLE); /* Above it, the other way. */
    assert(waves[2].swerve == waves[1].swerve);
    assert(waves[2].swerve != swerve_away_from(&birds[2], 400, 400, 0));
    assert(!wave_busy(&waves[3]) && !wave_busy(&waves[4]));
    /* The heading is the bird's own, turned by what it copied. */
    for (int i = 0; i < 3; i++) {
        double want = fmod(birds[i].direction + waves[i].swerve + 4 * M_PI, 2 * M_PI);
        assert(fabs(waves[i].heading - want) < 1e-9);
    }
    spatial_grid_destroy(&grid);
    config.hawks = 0;
    reset_the_waves();
    reset_test_config();
}

/* The refractory time holds. A bird that has swerved cannot be alarmed again
 * until it has rested, whoever tells it, and the wave that has crossed a line
 * of birds does not come back through it: run_the_line asserts that nobody
 * begins twice, over the whole of the rest. */
static void test_the_refractory_time_holds(void) {
    double began[LINE_BIRDS], again[LINE_BIRDS];
    set_the_line(6, 60, DEFAULT_NOTCH);
    double rest = WAVE_REFRACTORY * config.pace;
    int steps = (int)(rest * 60) - 60; /* A second short of it. */
    assert(run_the_line(steps, began) == LINE_BIRDS);
    for (int i = 0; i < LINE_BIRDS; i++) {
        assert(!wave_catchable(&waves[i]));
        assert(!wave_catch(&waves[i], -SWERVE_ANGLE, 0.001));
        assert(waves[i].swerve == SWERVE_ANGLE); /* And what it has is not overwritten. */
    }
    /* It is over for the first bird before it is for the last, by as long as the
     * wave took to reach it, and then they can all be caught again. */
    assert(run_the_line(240, again) == 0); /* Nothing starts it, so nothing begins. */
    for (int i = 0; i < LINE_BIRDS; i++) assert(wave_catchable(&waves[i]));
    assert(wave_catch(&waves[0], -SWERVE_ANGLE, 1e-6));
    waves_in_flight = 1;
    assert(run_the_line(120, again) == LINE_BIRDS);
    assert(waves[LINE_BIRDS - 1].swerve == -SWERVE_ANGLE); /* A second wave, the other way. */
    put_the_line_away();
}

/* A dive alarms the birds in its way and nobody else. */
static void test_a_dive_alarms_the_birds_in_its_way(void) {
    enum { COUNT = 6 };
    bird_t birds[COUNT];

    reset_test_config();
    reset_the_waves();
    legend_enabled = 0;
    apply_screen_size(200, 50, 1600, 800);
    config.birds = COUNT;
    double radius = ALARM_SHARE * hawk_reach();
    assert(radius < hawk_reach());
    birds[0] = (bird_t){.x = 800 + radius * 0.5, .y = 400};             /* Ahead of it, near. */
    birds[1] = (bird_t){.x = 800 - radius * 0.5, .y = 400};             /* Behind it, near. */
    birds[2] = (bird_t){.x = 800 + radius * 2.0, .y = 400};             /* Ahead of it, and far. */
    birds[3] = (bird_t){.x = 800, .y = 400 + radius * 0.5};             /* Beside it. */
    birds[4] = (bird_t){.x = 800 + radius * 0.5, .y = 400, .layer = 1}; /* In the far sky. */
    birds[5] = (bird_t){.x = 1500, .y = 100};

    /* Cruising, it alarms what it is heading at and nothing it is not. */
    alarm_the_birds_near(birds, 800, 400, 0, 0, radius);
    assert(wave_busy(&waves[0]));
    assert(!wave_busy(&waves[1]) && !wave_busy(&waves[3]));
    assert(!wave_busy(&waves[2]) && !wave_busy(&waves[4]) && !wave_busy(&waves[5]));

    /* Diving, everything in the radius, behind it and beside it too, but never
     * the far sky. The nearer, the sooner. */
    reset_the_waves();
    alarm_the_birds_near(birds, 800, 400, 0, 1, radius);
    assert(wave_waiting(&waves[0]) && wave_waiting(&waves[1]) && wave_waiting(&waves[3]));
    assert(!wave_busy(&waves[2]) && !wave_busy(&waves[4]) && !wave_busy(&waves[5]));
    /* Told how long to wait, by how far off it is: half of the reaction time under
     * its wing and the whole of it at the edge of its sight, which is the time a
     * wave takes to cross that at WAVE_PACE times the pace a bird flies at. */
    assert(fabs(waves[0].wait - reaction_time(radius * 0.5, radius)) < 1e-12);
    assert(waves[0].wait == waves[1].wait && waves[0].wait == waves[3].wait);
    assert(reaction_time(0, radius) < reaction_time(radius, radius));
    assert(fabs(reaction_time(radius, radius) * WAVE_PACE * flight_pixels_per_second - radius) <
           1e-9);
    reset_the_waves();
    reset_test_config();
}

/* Nothing is alarmed without a hawk, or a pointer that is whipped: a flock
 * flown for a long while with neither never has anything in its alarm state, and
 * a pointer that is slow, or still, is not a hawk. */
static void test_nothing_is_alarmed_without_hawks(void) {
    enum { COUNT = 120 };
    static bird_t birds[COUNT], snapshot[COUNT];
    spatial_grid_t grid;

    reset_test_config();
    reset_the_waves();
    legend_enabled = 0;
    apply_screen_size(100, 30, 800, 480);
    config.birds = COUNT;
    config.hawks = 0;
    seed_random(11);
    for (int i = 0; i < COUNT; i++) place_one_bird(&birds[i], i);
    assert(spatial_grid_init(&grid, SPATIAL_CELL_SIZE) == SPATIAL_GRID_OK);
    assert(spatial_grid_prepare(&grid, screen.width, screen.height, COUNT) == SPATIAL_GRID_OK);
    for (int frame = 0; frame < 300; frame++) {
        memcpy(snapshot, birds, sizeof(birds));
        assert(spatial_grid_build(&grid, COUNT, read_bird_position, snapshot) == SPATIAL_GRID_OK);
        fly(birds, snapshot, &grid);
        assert(!waves_in_flight);
        for (int i = 0; i < COUNT; i++) assert(!birds[i].alarmed);
    }
    static const wave_t still;
    for (int i = 0; i < COUNT; i++) assert(memcmp(&waves[i], &still, sizeof(still)) == 0);

    mouse.present = 1;
    mouse.x = birds[0].x;
    mouse.y = birds[0].y;
    mouse.moved_at = clock_state.seconds;
    mouse.velocity_x = POINTER_STARTLE_CELLS * screen.cell_width * 0.5;
    for (int frame = 0; frame < 30; frame++) {
        memcpy(snapshot, birds, sizeof(birds));
        assert(spatial_grid_build(&grid, COUNT, read_bird_position, snapshot) == SPATIAL_GRID_OK);
        fly(birds, snapshot, &grid);
    }
    assert(!waves_in_flight);
    spatial_grid_destroy(&grid);
    reset_the_waves();
    reset_test_config();
}

/* With hawks it is the other way round: in a flock flown for a few seconds with
 * two of them, waves run, and one crosses most of the flock at once. */
static void test_a_strike_sends_a_wave_across_the_flock(void) {
    enum { COUNT = 300 };
    static bird_t birds[COUNT], snapshot[COUNT];
    spatial_grid_t grid;

    reset_test_config();
    reset_the_waves();
    legend_enabled = 0;
    apply_screen_size(96, 32, 768, 512);
    config.birds = COUNT;
    config.bird_size = 14;
    config.hawks = 2;
    config.pace_notch = 0;
    apply_notches();
    set_frame_seconds(1.0 / 50);
    seed_random(33);
    for (int i = 0; i < COUNT; i++) place_one_bird(&birds[i], i);
    place_hawks();
    assert(spatial_grid_init(&grid, SPATIAL_CELL_SIZE) == SPATIAL_GRID_OK);
    assert(spatial_grid_prepare(&grid, screen.width, screen.height, COUNT) == SPATIAL_GRID_OK);
    int was_lit[COUNT] = {0}, begun = 0, peak = 0;
    for (int frame = 0; frame < 600; frame++) {
        memcpy(snapshot, birds, sizeof(birds));
        assert(spatial_grid_build(&grid, COUNT, read_bird_position, snapshot) == SPATIAL_GRID_OK);
        fly(birds, snapshot, &grid);
        int lit = 0;
        for (int i = 0; i < COUNT; i++) {
            if (birds[i].alarmed) lit++;
            if (birds[i].alarmed && !was_lit[i]) begun++;
            was_lit[i] = birds[i].alarmed;
        }
        if (lit > peak) peak = lit;
    }
    spatial_grid_destroy(&grid);
    assert(begun > COUNT);          /* Waves, more than one of them... */
    assert(peak * 10 >= COUNT * 6); /* ...of which one lit most of the flock at once. */
    config.hawks = 0;
    reset_the_waves();
    reset_test_config();
}

/* A swerve is a turn the banking would not allow: with the banking turned right
 * down a bird that is swerving turns further in a step than it ever banks, but
 * not further than three times as far, and only while it swerves. */
static void test_a_swerve_is_sharper_than_the_banking(void) {
    bird_t birds[2], snapshot[2];
    spatial_grid_t grid;

    reset_test_config();
    reset_the_waves();
    legend_enabled = 0;
    apply_screen_size(200, 50, 1600, 800);
    config.birds = 2;
    config.turning_notch = 0;
    double limit = turn_limit();
    assert(limit < SWERVE_ANGLE); /* Banking alone could not make it in a step. */
    birds[0] = (bird_t){.x = 600, .y = 400, .direction = 0};
    birds[1] = (bird_t){.x = 1000, .y = 400, .direction = 0};
    assert(spatial_grid_init(&grid, SPATIAL_CELL_SIZE) == SPATIAL_GRID_OK);
    assert(spatial_grid_prepare(&grid, screen.width, screen.height, 2) == SPATIAL_GRID_OK);
    assert(wave_catch(&waves[0], SWERVE_ANGLE, 1e-6));
    waves_in_flight = 1;
    memcpy(snapshot, birds, sizeof(birds));
    assert(spatial_grid_build(&grid, 2, read_bird_position, snapshot) == SPATIAL_GRID_OK);
    spread_the_alarm(snapshot, &grid);
    assert(waves[0].left > 0);
    update_birds(birds, snapshot, &grid);
    double turned = angle_difference(birds[0].direction, 0);
    assert(turned > limit + 1e-6);                /* Sharper than it would bank... */
    assert(turned <= limit * SWERVE_TURN + 1e-9); /* ...and only so much sharper. */
    assert(turned > 0.5 * SWERVE_ANGLE);          /* And it is the swerve it was told. */
    assert(birds[0].alarmed && !birds[1].alarmed);
    assert(angle_difference(birds[1].direction, 0) < 1e-9);
    spatial_grid_destroy(&grid);
    reset_the_waves();
    reset_test_config();
}

/* The far sky is never alarmed: a hawk dives through a crowd of far birds and
 * the near birds among them, and the near ones swerve and the far ones fly on as
 * if nothing had happened. */
static void test_far_birds_are_never_alarmed(void) {
    enum { COUNT = 120 };
    static bird_t birds[COUNT], snapshot[COUNT];
    spatial_grid_t grid;

    reset_test_config();
    reset_the_waves();
    legend_enabled = 0;
    apply_screen_size(100, 30, 800, 480);
    config.birds = COUNT;
    config.hawks = 1;
    for (int i = 0; i < COUNT; i++) {
        birds[i] = (bird_t){.x = 300 + (i % 12) * 16.0,
                            .y = 150 + (i / 12) * 16.0,
                            .direction = 0.4 * i,
                            .layer = i % 2};
        birds[i].frame = direction_frame(birds[i].direction);
    }
    place_hawks();
    hawks[0].x = 390;
    hawks[0].y = 220;
    hawks[0].direction = 0;
    assert(spatial_grid_init(&grid, SPATIAL_CELL_SIZE) == SPATIAL_GRID_OK);
    assert(spatial_grid_prepare(&grid, screen.width, screen.height, COUNT) == SPATIAL_GRID_OK);
    int near_lit = 0;
    for (int frame = 0; frame < 20; frame++) {
        hawks[0].diving = 1;
        memcpy(snapshot, birds, sizeof(birds));
        assert(spatial_grid_build(&grid, COUNT, read_bird_position, snapshot) == SPATIAL_GRID_OK);
        spread_the_alarm(snapshot, &grid);
        update_birds(birds, snapshot, &grid);
        for (int i = 0; i < COUNT; i++) {
            if (birds[i].layer > 0) {
                assert(!birds[i].alarmed);
                assert(!wave_busy(&waves[i]));
            } else if (birds[i].alarmed) {
                near_lit++;
            }
        }
    }
    assert(near_lit > 20); /* The near sky was alarmed, so the far one's silence is a rule. */
    spatial_grid_destroy(&grid);
    config.hawks = 0;
    reset_the_waves();
    reset_test_config();
}

/* The letters of the intro are never alarmed. */
static void test_the_letters_are_never_alarmed(void) {
    enum { COUNT = 200 };
    static bird_t birds[COUNT], snapshot[COUNT];
    spatial_grid_t grid;

    reset_test_config();
    reset_the_waves();
    legend_enabled = 0;
    apply_screen_size(100, 30, 800, 480);
    config.birds = COUNT;
    config.hawks = 2;
    seed_random(3);
    for (int i = 0; i < COUNT; i++) place_one_bird(&birds[i], i);
    place_hawks();
    begin_the_intro();
    assert(formation.writing);
    assert(spatial_grid_init(&grid, SPATIAL_CELL_SIZE) == SPATIAL_GRID_OK);
    assert(spatial_grid_prepare(&grid, screen.width, screen.height, COUNT) == SPATIAL_GRID_OK);
    for (int frame = 0; frame < 240; frame++) {
        for (int h = 0; h < config.hawks; h++) { /* The hawks among the letters, diving. */
            hawks[h].x = formation.x[(frame * 7 + h * 31) % formation.count];
            hawks[h].y = formation.y[(frame * 7 + h * 31) % formation.count];
            hawks[h].diving = 1;
        }
        memcpy(snapshot, birds, sizeof(birds));
        assert(spatial_grid_build(&grid, COUNT, read_bird_position, snapshot) == SPATIAL_GRID_OK);
        spread_the_alarm(snapshot, &grid);
        update_birds(birds, snapshot, &grid);
        for (int i = 0; i < COUNT; i++) assert(!birds[i].alarmed);
        assert(!waves_in_flight);
    }
    formation_clear();
    spatial_grid_destroy(&grid);
    config.hawks = 0;
    reset_the_waves();
    reset_test_config();
}

/* Birds of another flock are watched as far as they are kin. */
static void test_a_wave_stays_in_its_flock_unless_the_flocks_are_kin(void) {
    enum { COUNT = 40 };
    static bird_t birds[COUNT];
    spatial_grid_t grid;

    reset_test_config();
    reset_the_waves();
    legend_enabled = 0;
    apply_screen_size(200, 50, 1600, 800);
    config.birds = COUNT;
    config.flocks = 2;
    for (int i = 0; i < COUNT; i++)
        birds[i] = (bird_t){.x = 300 + i * 10.0, .y = 400, .flock = i % 2};
    assert(spatial_grid_init(&grid, SPATIAL_CELL_SIZE) == SPATIAL_GRID_OK);
    assert(spatial_grid_prepare(&grid, screen.width, screen.height, COUNT) == SPATIAL_GRID_OK);
    assert(spatial_grid_build(&grid, COUNT, read_bird_position, birds) == SPATIAL_GRID_OK);

    for (int notch = DEFAULT_NOTCH; notch >= 0; notch -= DEFAULT_NOTCH) {
        reset_the_waves();
        config.avoid_notch = notch;
        apply_notches();
        assert(wave_catch(&waves[0], SWERVE_ANGLE, 1e-6));
        waves_in_flight = 1;
        for (int frame = 0; frame < 60; frame++) spread_the_alarm(birds, &grid);
        int own = 0, strangers = 0;
        for (int i = 0; i < COUNT; i++) {
            if (!wave_busy(&waves[i])) continue;
            if (birds[i].flock == 0)
                own++;
            else
                strangers++;
        }
        assert(own == COUNT / 2);
        /* Keeping to their own at the default; one flock of two colours below it. */
        assert(strangers == (notch == DEFAULT_NOTCH ? 0 : COUNT / 2));
    }
    spatial_grid_destroy(&grid);
    reset_the_waves();
    reset_test_config();
}

/* A pointer whipped through the flock sets a wave off, and one that drifts, or
 * has stopped, or is not there, does not. */
static void test_a_pointer_whipped_through_the_flock_starts_a_wave(void) {
    enum { COUNT = 60 };
    static bird_t birds[COUNT];
    spatial_grid_t grid;

    reset_test_config();
    reset_the_waves();
    legend_enabled = 0;
    apply_screen_size(200, 50, 1600, 800);
    config.birds = COUNT;
    config.hawks = 0;
    for (int i = 0; i < COUNT; i++)
        birds[i] = (bird_t){.x = 700 + (i % 10) * 12.0, .y = 350 + (i / 10) * 12.0};
    assert(spatial_grid_init(&grid, SPATIAL_CELL_SIZE) == SPATIAL_GRID_OK);
    assert(spatial_grid_prepare(&grid, screen.width, screen.height, COUNT) == SPATIAL_GRID_OK);
    assert(spatial_grid_build(&grid, COUNT, read_bird_position, birds) == SPATIAL_GRID_OK);
    double fast = 2 * POINTER_STARTLE_CELLS * screen.cell_width;

    /* Absent, and slow, and stale: nothing. */
    spread_the_alarm(birds, &grid);
    assert(!waves_in_flight);
    mouse.present = 1;
    mouse.x = 750;
    mouse.y = 380;
    mouse.velocity_x = fast / 4;
    mouse.moved_at = clock_state.seconds;
    spread_the_alarm(birds, &grid);
    assert(!waves_in_flight);
    mouse.velocity_x = fast;
    mouse.moved_at = clock_state.seconds - 5;
    spread_the_alarm(birds, &grid);
    assert(!waves_in_flight);

    /* Whipped across, it is a hawk: the birds near it swerve, and tell the rest. */
    mouse.moved_at = clock_state.seconds;
    assert(pointer_startles());
    for (int frame = 0; frame < 120; frame++) {
        mouse.moved_at = clock_state.seconds;
        spread_the_alarm(birds, &grid);
    }
    int alarmed = 0;
    for (int i = 0; i < COUNT; i++) alarmed += wave_busy(&waves[i]);
    assert(alarmed == COUNT);

    /* And the reports that make that speed are read from the terminal: a pointer
     * fifty columns in a twentieth of a second is going fast, and goes quiet when
     * the terminal stops reporting it. */
    reset_the_waves();
    clock_state.seconds = 10;
    read_mouse_report("<35;10;5");
    clock_state.seconds = 10.05;
    read_mouse_report("<35;60;5");
    assert(mouse.velocity_x > 0 && fabs(mouse.velocity_y) < 1e-9);
    assert(pointer_startles());
    clock_state.seconds = 10.3;
    assert(!pointer_startles());
    clock_state.seconds = 11;
    read_mouse_report("<35;62;5"); /* Two columns in most of a second. */
    assert(!pointer_startles());

    clock_state.seconds = 0;
    spatial_grid_destroy(&grid);
    reset_the_waves();
    reset_test_config();
}

/* A GIF is drawn in the colours its first frame had, and the light of a wave is
 * on no first frame: a clip in which a wave can happen asks for it, and any
 * other keeps the palette it always had. A wave can happen with hawks, and not in
 * the three seconds the letters are being written. */
static int palette_of_a_recording_has(const uint8_t colour[3], int hawks, int seconds) {
    char path[600];
    scratch_file(path, sizeof(path), "light.gif");
    reset_test_config();
    config.palette = palette_named("ice");
    config.birds = 60;
    config.hawks = hawks;
    record_path = path;
    record_fps = 25;
    record_seconds = seconds;
    record_columns = 96;
    record_rows = 32;
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
    FILE *file = fopen(path, "rb");
    assert(file != NULL);
    uint8_t header[13 + 768];
    assert(fread(header, 1, sizeof(header), file) == sizeof(header));
    fclose(file);
    remove(path);
    record_path = NULL;
    record_seconds = 6;
    int found = 0;
    for (int entry = 0; entry < 256; entry++)
        if (memcmp(header + 13 + entry * 3, colour, 3) == 0) found = 1;
    reset_test_config();
    return found;
}

/* The same for a space, recorded: it has no waves, so no light is asked for, and
 * the colours it does ask for are the ones its sprites are drawn in, which are on
 * no first frame when a size of bird has not flown into view or a hawk is off the
 * screen. */
static int palette_of_a_space_recording_has(const uint8_t colour[3], int hawks) {
    char path[600];
    scratch_file(path, sizeof(path), "space_light.gif");
    reset_test_config();
    sky_mode = 1;
    config.palette = palette_named("ice");
    config.birds = 200;
    config.hawks = hawks;
    record_path = path;
    record_fps = 25;
    record_seconds = 5;
    record_columns = 64;
    record_rows = 18;
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
    FILE *file = fopen(path, "rb");
    assert(file != NULL);
    uint8_t header[13 + 768];
    assert(fread(header, 1, sizeof(header), file) == sizeof(header));
    fclose(file);
    remove(path);
    record_path = NULL;
    record_seconds = 6;
    int found = 0;
    for (int entry = 0; entry < 256; entry++)
        if (memcmp(header + 13 + entry * 3, colour, 3) == 0) found = 1;
    reset_test_config();
    return found;
}

static void test_a_recording_of_a_space_asks_for_its_sprites_and_not_for_a_wave(void) {
    reset_test_config();
    config.palette = palette_named("ice");
    uint8_t light[3], hawk[3];
    memcpy(light, highlight_colour(), 3);
    memcpy(hawk, hawk_colour(), 3);
    /* The control: in the flat sky a wave can happen, and the light is in the table. */
    assert(palette_of_a_recording_has(light, 2, 5));
    /* In a space it cannot, and the table is without it; but the hawk is in it, which
     * is a colour no first frame is sure to have. */
    assert(!palette_of_a_space_recording_has(light, 2));
    assert(palette_of_a_space_recording_has(hawk, 2));
    reset_test_config();
}

static void test_a_recording_with_hawks_has_the_light_in_its_palette(void) {
    reset_test_config();
    config.palette = palette_named("ice");
    uint8_t light[3];
    memcpy(light, highlight_colour(), 3);
    assert(palette_of_a_recording_has(light, 2, 5));  /* A wave can happen. */
    assert(!palette_of_a_recording_has(light, 0, 5)); /* No hawks, so none can. */
    assert(!palette_of_a_recording_has(light, 2, 2)); /* All of it the letters. */
    reset_test_config();
}

/* The light of a wave stands clear of the ramp, of the hawk and of the ground,
 * for every ramp, the terminal's own among them, on a dark ground and a light
 * one, and on artwork of somebody's own. */
static void test_the_light_of_a_wave_stands_clear_of_everything(void) {
    static const uint8_t ACCENT[3] = {205, 40, 40}, DARK[3] = {18, 18, 24},
                         LIGHT[3] = {246, 246, 240};
    reset_test_config();
    for (config.palette = 0; config.palette < PALETTE_COUNT; config.palette++) {
        for (int light_ground = 0; light_ground < 2; light_ground++) {
            /* Only the terminal's own ramp is ever on a ground that is not the picture's. */
            if (light_ground && !palette_follows_the_theme()) continue;
            if (palette_follows_the_theme()) ramp_between(ACCENT, light_ground ? LIGHT : DARK);
            memcpy(theme_ground, light_ground ? LIGHT : DARK, 3);
            const uint8_t *light = highlight_colour();
            assert(contrast_between(light, theme_ground) >= HIGHLIGHT_CONTRAST);
            assert(colour_distance(light, hawk_colour()) >= 100);
            for (int shade = 0; shade < palette()->shades; shade++)
                assert(colour_distance(light, palette()->tints[shade]) >= 100);

            /* And it is what the sprite is painted in. */
            png_image_t feather = {0, 0, NULL};
            assert(png_image_alloc(&feather, 1, 1) == PNG_OK);
            feather.pixels[3] = 255;
            highlight_tint(&feather);
            assert(memcmp(feather.pixels, light, 3) == 0 && feather.pixels[3] == 255);
            png_image_free(&feather);
        }
    }
    memcpy(theme_ground, PICTURE_GROUND, 3);
    config.palette = palette_named("ember");
    sprite_path = "somebody's.png";
    assert(colour_distance(highlight_colour(), hawk_colour()) >= 100);
    sprite_path = NULL;
    reset_test_config();
}

/* The lit bird is in the catalogue and every renderer draws it: the same
 * silhouette as the bird it was, in the light and not in the ramp, over the rest
 * of the flock, in the sprites, in the cells and in a picture. */
static void test_a_bird_in_a_wave_is_lit_in_every_renderer(void) {
    static png_image_t frames[ROTATION_FRAMES * MAX_SPRITE_SETS];
    kitty_graphics_t graphics;
    bird_t birds[2];
    char want[64];

    reset_test_config();
    legend_enabled = 0;
    config.palette = palette_named("ice");
    config.birds = 2;
    config.hawks = 0;
    apply_screen_size(60, 20, 480, 320);
    assert(alarm_set(0) == trail_set(TRAIL_LENGTH)); /* After everything that was already there. */
    assert(sprite_set_count() == alarm_set(WING_PHASES));
    assert(sprite_set_count() <= MAX_SPRITE_SETS);

    /* Every set is built, so every one is uploaded; the lit ones in the light. */
    assert(rasterise_sprites(frames) == PNG_OK);
    const uint8_t *light = highlight_colour();
    for (int wing = 0; wing < WING_PHASES; wing++) {
        for (int frame = 0; frame < ROTATION_FRAMES; frame++) {
            const png_image_t *lit = &frames[alarm_set(wing) * ROTATION_FRAMES + frame];
            const png_image_t *unlit = &frames[flock_set(0, wing, 0) * ROTATION_FRAMES + frame];
            assert(lit->pixels != NULL);
            assert(lit->width == unlit->width && lit->height == unlit->height);
            for (int i = 0; i < lit->width * lit->height; i++) {
                assert(lit->pixels[i * 4 + 3] == unlit->pixels[i * 4 + 3]);
                if (lit->pixels[i * 4 + 3] != 0) assert(memcmp(lit->pixels + i * 4, light, 3) == 0);
            }
        }
    }

    /* Kitty: a placement of an image in the set, and the others as they were, each
     * placed once, and the lit bird after the rest of the flock, which is over it. */
    birds[0] = (bird_t){.x = 100, .y = 100, .direction = 0.5, .frame = 5, .alarmed = 1};
    birds[1] = (bird_t){.x = 200, .y = 200, .direction = 0.5, .frame = 5};
    assert(sprite_image_id(&birds[0]) == set_image_id(alarm_set(WING_SEQUENCE[0]), 5));
    assert(sprite_image_id(&birds[1]) == set_image_id(flock_set(0, WING_SEQUENCE[0], 0), 5));
    render_mode = RENDER_KITTY;
    assert(kitty_graphics_init(&graphics, STDOUT_FILENO) == KITTY_GRAPHICS_OK);
    assert(queue_render_frame(&graphics, birds) == KITTY_GRAPHICS_OK);
    snprintf(want, sizeof(want), "a=p,I=%u,", (unsigned)sprite_image_id(&birds[0]));
    const char *lit_at = strstr(graphics.buffer, want);
    snprintf(want, sizeof(want), "a=p,I=%u,", (unsigned)sprite_image_id(&birds[1]));
    const char *plain_at = strstr(graphics.buffer, want);
    assert(lit_at != NULL && plain_at != NULL && plain_at < lit_at);
    int placements = 0;
    for (const char *at = graphics.buffer; (at = strstr(at, "a=p,")) != NULL; at++) placements++;
    assert(placements == 2);
    kitty_graphics_destroy(&graphics);

    /* A picture, and a GIF and a snapshot of one: a lit bird over an unlit one in
     * the same place and the same shape leaves nothing of the one it covers. */
    birds[0].x = birds[1].x = 150;
    birds[0].y = birds[1].y = 150;
    static png_image_t canvas;
    png_image_free(&canvas);
    assert(png_image_alloc(&canvas, screen.width, screen.height) == PNG_OK);
    compose_onto(&canvas, frames, birds, 1);
    int in_light = 0, in_ramp = 0;
    for (size_t px = 0; px < (size_t)canvas.width * (size_t)canvas.height; px++) {
        if (memcmp(&canvas.pixels[px * 4], light, 3) == 0) in_light++;
        if (memcmp(&canvas.pixels[px * 4], palette()->tints[0], 3) == 0) in_ramp++;
    }
    assert(in_light > 0 && in_ramp == 0);
    png_image_free(&canvas);

    /* And cells: the canvas they are read from keeps the first thing that is in a
     * pixel, so the lit bird goes down first, or the colour that fills most of a
     * cell the two share would be the unlit one. */
    render_mode = RENDER_BRAILLE;
    assert(prepare_text_renderer());
    assert(text_renderer_fits_the_screen());
    compose_onto(&text_canvas, text_sprites, birds, 0);
    in_light = in_ramp = 0;
    for (size_t px = 0; px < (size_t)text_canvas.width * (size_t)text_canvas.height; px++) {
        const uint8_t *pixel = &text_canvas.pixels[px * 4];
        if (pixel[3] == 0) continue;
        if (memcmp(pixel, light, 3) == 0) in_light++;
        if (memcmp(pixel, palette()->tints[0], 3) == 0) in_ramp++;
    }
    assert(in_light > 0 && in_ramp == 0);
    cells_destroy(&text_cells);
    png_image_free(&text_canvas);
    free_sprites(text_sprites);
    free_sprites(frames);
    render_mode = RENDER_KITTY;
    reset_test_config();
}

/* On a night nothing is hunted and the pointer is a lantern and not a whip: held
 * at a speed that sets a wave off in a flock, it sets none off among fireflies,
 * and none of them is lit as a bird in a wave is. The same whip over the same
 * birds in a flock does start one, so what is measured is the night. */
static void test_a_night_tells_no_alarm(void) {
    night_t run;
    begin_the_night(&run, 96, 26, 30, 3);
    reset_the_waves();
    mouse.present = 1;
    mouse.x = screen.width / 2.0;
    mouse.y = screen.height / 2.0;
    mouse.velocity_x = 2 * POINTER_STARTLE_CELLS * screen.cell_width;
    mouse.velocity_y = 0;
    for (int frame = 0; frame < 90; frame++) {
        mouse.moved_at = clock_state.seconds;
        assert(pointer_startles());
        one_night_frame(&run);
    }
    assert(!waves_in_flight && wave_task_count == 0);
    for (int i = 0; i < config.birds; i++) assert(!run.birds[i].alarmed && !wave_busy(&waves[i]));

    /* The lantern has cleared a hole round itself by now: put it on a firefly. */
    fireflies_mode = 0;
    assert(spatial_grid_build(&run.grid, config.birds, read_bird_position, run.birds) ==
           SPATIAL_GRID_OK);
    mouse.x = run.birds[0].x;
    mouse.y = run.birds[0].y;
    mouse.moved_at = clock_state.seconds;
    spread_the_alarm(run.birds, &run.grid);
    int lit = 0;
    for (int i = 0; i < config.birds; i++) lit += wave_busy(&waves[i]);
    assert(lit > 0);
    reset_the_waves();
    end_the_night(&run);
}

/* --- Text as the flock ---------------------------------------------------------- */

/* A world of letters, built the way main builds one: the text goes through a file
 * and take_the_text, so that the whole path from bytes to birds is the one run. */
typedef struct {
    bird_t *birds, *snapshot;
    spatial_grid_t grid;
    int cols, rows;
} world_t;

static void world_write(const char *path, const char *text) {
    FILE *file = fopen(path, "wb");
    assert(file != NULL);
    assert(fwrite(text, 1, strlen(text), file) == strlen(text));
    assert(fclose(file) == 0);
}

static void world_open(world_t *world, const char *text, int cols, int rows) {
    char path[600];
    scratch_file(path, sizeof(path), "letters.txt");
    world_write(path, text);
    reset_test_config();
    legend_enabled = 0;
    config.palette = palette_named("ember");
    config.pace_notch = DEFAULT_PACE_NOTCH;
    apply_notches();
    apply_screen_size(cols, rows, cols * 8, rows * 16);
    text_path = path;
    assert(take_the_text(cols, rows, 0) == 1);
    text_path = NULL;
    remove(path);
    world->cols = cols;
    world->rows = rows;
    apply_screen_size(cols, rows, 0, 0); /* A cell is eight by sixteen in letters mode. */
    seed_random(7);
    set_frame_seconds(1.0 / FRAME_RATE);
    assert(spatial_grid_init(&world->grid, SPATIAL_CELL_SIZE) == SPATIAL_GRID_OK);
    assert(spatial_grid_prepare(&world->grid, screen.width, screen.height, config.birds) ==
           SPATIAL_GRID_OK);
    world->birds = calloc((size_t)config.birds, sizeof(*world->birds));
    world->snapshot = calloc((size_t)config.birds, sizeof(*world->snapshot));
    assert(world->birds && world->snapshot);
    initialize_birds(world->birds);
    place_hawks();
    assert(cells_init(&text_cells, 1) == CELLS_OK);
    assert(text_renderer_fits_the_screen());
}

static void world_close(world_t *world) {
    free(world->birds);
    free(world->snapshot);
    spatial_grid_destroy(&world->grid);
    cells_destroy(&text_cells);
    letters_destroy(&the_letters);
    forget_the_text();
    letters_mode = 0;
    legend_enabled = 1;
    config.hawks = 0;
    mouse.present = 0;
    paused = 0;
    render_mode = RENDER_KITTY;
    reset_test_config();
}

static void world_step(world_t *world) {
    memcpy(world->snapshot, world->birds, sizeof(*world->birds) * (size_t)config.birds);
    assert(spatial_grid_build(&world->grid, config.birds, read_bird_position, world->snapshot) ==
           SPATIAL_GRID_OK);
    fly(world->birds, world->snapshot, &world->grid);
}

static cell_t *world_picture(world_t *world, int *count) {
    paint_the_letters(world->birds);
    *count = text_cells.cols * text_cells.rows;
    cell_t *copy = malloc((size_t)*count * sizeof(*copy));
    assert(copy != NULL);
    memcpy(copy, text_cells.now, (size_t)*count * sizeof(*copy));
    return copy;
}

static int world_all_home(const world_t *world) {
    for (int i = 0; i < config.birds; i++) {
        const letter_t *letter = &the_letters.letter[i];
        if (letter->state != LETTER_PERCHED || !world->birds[i].perched ||
            world->birds[i].x != letter->home_x || world->birds[i].y != letter->home_y)
            return 0;
    }
    return 1;
}

static const char NEOFETCH_LIKE[] =
    "\033[?25l\033[?7l"
    "\033[1;31m   .-/+oo+/-.   \033[0m\n"
    "\033[1;31m .+sssssssssss+. \033[0m\n"
    "\033[1;31m/sssssdMMMsssss/ \033[0m\n"
    "\033[1;31m+ssssNMMMNssssss+\033[0m\n"
    "\033[1;31m/sssssdMMMsssss/ \033[0m\n"
    "\033[1;31m .+sssssssssss+. \033[0m\n"
    "\033[1;31m   .-/+oo+/-.   \033[0m\n"
    "\033[7A\033[9999999D"
    "\033[20C\033[1;31muser\033[0m@\033[1;31mhost\033[0m\n"
    "\033[20C-----------\n"
    "\033[20C\033[1;31mOS\033[0m: Linux x86_64\n"
    "\033[20C\033[1;31mShell\033[0m: sh\n"
    "\033[20C\033[38;2;255;128;0mCPU\033[0m: a fast one\n"
    "\033[20C\033[4munderlined\033[0m \033[7mreversed\033[0m\n"
    "\033[20C\033[41m   \033[42m   \033[43m   \033[0m\xE4\xB8\xAD\xE6\x96\x87\n"
    "\033[?25h\033[?7h";

static void test_text_piped_in_becomes_the_flock(void) {
    world_t world;
    world_open(&world, NEOFETCH_LIKE, 60, 12);
    assert(letters_mode);
    assert(config.birds == the_letters.count && config.birds > 60);
    /* One flock on one plane, drawn as text whatever else was asked. */
    assert(config.flocks == 1 && !deep_look && !config.trails && !the_rain_is_falling);
    assert(live_render_mode() == RENDER_BRAILLE);
    /* Every letter is at home, perched, and nothing has been drawn yet. */
    assert(world_all_home(&world));
    assert(the_letters.phase == LETTERS_AT_REST);
    assert(formation.count == 0 && !formation.writing);
    begin_the_intro();
    assert(!formation.writing); /* The text is the intro. */
    world_close(&world);
}

static void test_a_flock_of_letters_is_slower_than_a_flock_of_birds_unless_asked(void) {
    char path[600];
    scratch_file(path, sizeof(path), "pace.txt");
    world_write(path, "some text\nmore text");
    reset_test_config();
    config.pace_notch = DEFAULT_PACE_NOTCH;
    apply_notches();
    text_path = path;
    assert(take_the_text(40, 10, 0) == 1);
    assert(config.pace_notch == LETTERS_PACE_NOTCH &&
           config.pace == PACE_STEP * (LETTERS_PACE_NOTCH + 1));
    apply_preset_defaults();
    assert(config.pace_notch == LETTERS_PACE_NOTCH); /* 0 puts it back to its own default. */
    letters_destroy(&the_letters);
    letters_mode = 0;
    /* A speed that was asked for stays. */
    reset_test_config();
    config.pace_notch = 7;
    apply_notches();
    assert(take_the_text(40, 10, 0) == 1);
    assert(config.pace_notch == 7);
    letters_destroy(&the_letters);
    letters_mode = 0;
    text_path = NULL;
    remove(path);
    reset_test_config();
}

static void test_nothing_to_see_leaves_the_flock_as_it_was(void) {
    char path[600];
    scratch_file(path, sizeof(path), "blank.txt");
    reset_test_config();
    int birds = config.birds;
    const char *empty[] = {"", "\n\n   \n", "\033[41m    \033[0m\n"};
    int saved = dup(STDERR_FILENO);
    int quiet = open("/dev/null", O_WRONLY);
    assert(saved >= 0 && quiet >= 0 && dup2(quiet, STDERR_FILENO) == STDERR_FILENO);
    close(quiet);
    for (size_t i = 0; i < sizeof(empty) / sizeof(*empty); i++) {
        world_write(path, empty[i]);
        text_path = path;
        assert(take_the_text(40, 10, 0) == 0);
        assert(!letters_mode && config.birds == birds);
    }
    text_path = NULL;
    dup2(saved, STDERR_FILENO);
    close(saved);
    remove(path);
    reset_test_config();
}

static void test_the_text_at_rest_is_the_text(void) {
    world_t world;
    world_open(&world, NEOFETCH_LIKE, 60, 12);
    int count;
    cell_t *picture = world_picture(&world, &count);
    assert(count == 60 * 12);
    /* The first row: the logo in bold red, as the command gave it, and blanks. */
    assert(picture[3].glyph == '.' && picture[3].has_fg &&
           picture[3].fg_kind == CELLS_COLOUR_ANSI && picture[3].fg[0] == 1 &&
           (picture[3].attributes & CELLS_BOLD));
    assert(picture[0].glyph == 0 && !picture[0].has_fg);
    /* The information beside it, in its own places, and the colour blocks kept as
     * scenery: a blank with a background. */
    assert(picture[20].glyph == 'u' && picture[20].fg[0] == 1);
    assert(picture[24].glyph == '@' && !picture[24].has_fg);
    cell_t *cpu = &picture[4 * 60 + 20];
    assert(cpu->glyph == 'C' && cpu->fg_kind == CELLS_COLOUR_EXACT && cpu->fg[0] == 255);
    cell_t *block = &picture[6 * 60 + 20];
    assert(block->glyph == 0 && block->has_bg && block->bg[0] == 1 &&
           block->bg_kind == CELLS_COLOUR_ANSI);
    cell_t *wide = &picture[6 * 60 + 29];
    assert(wide->glyph == 0x4E2D && wide->wide == CELLS_WIDE_HEAD);
    assert(wide[1].wide == CELLS_WIDE_TAIL);
    /* And what a terminal is sent for it is each of those, once. */
    kitty_graphics_t graphics;
    assert(kitty_graphics_init(&graphics, STDOUT_FILENO) == KITTY_GRAPHICS_OK);
    assert(queue_text_frame(&graphics, world.birds) == KITTY_GRAPHICS_OK);
    assert(strstr(graphics.buffer, "\033[1m\033[31m") != NULL ||
           strstr(graphics.buffer, "\033[1;31m") != NULL ||
           strstr(graphics.buffer, "\033[1m") != NULL);
    assert(strstr(graphics.buffer, "\033[31m") != NULL);
    assert(strstr(graphics.buffer, "\033[38;2;255;128;0m") != NULL);
    assert(strstr(graphics.buffer, "\xE4\xB8\xAD\xE6\x96\x87") != NULL);
    assert(strstr(graphics.buffer, "\033[4m") != NULL &&
           strstr(graphics.buffer, "\033[7m") != NULL);
    /* The same again, nothing: it is at rest. */
    size_t first = graphics.length;
    graphics.length = 0;
    assert(queue_text_frame(&graphics, world.birds) == KITTY_GRAPHICS_OK);
    assert(graphics.length < first / 8);
    kitty_graphics_destroy(&graphics);
    free(picture);
    world_close(&world);
}

/* Runs a world until a condition, a frame at a time, and says how many frames it
 * took, or -1. */
static int world_run_until(world_t *world, letters_phase_t phase, int frames) {
    for (int frame = 0; frame < frames; frame++) {
        if (the_letters.phase == phase) return frame;
        world_step(world);
    }
    return the_letters.phase == phase ? frames : -1;
}

static void test_a_whole_cycle_puts_every_letter_back_on_its_own_cell(void) {
    static const int NOTCHES[] = {0, 1, 4, 12};
    for (size_t n = 0; n < sizeof(NOTCHES) / sizeof(*NOTCHES); n++) {
        world_t world;
        world_open(&world, NEOFETCH_LIKE, 60, 12);
        config.pace_notch = NOTCHES[n];
        apply_notches();
        int count_before, count_after;
        cell_t *before = world_picture(&world, &count_before);

        /* The rest, the wave, and then they are all up. */
        assert(world_run_until(&world, LETTERS_TAKING_OFF, 60 * 6) > 0);
        assert(world_run_until(&world, LETTERS_IN_FLIGHT, 60 * 6) > 0);
        assert(!world_all_home(&world));
        /* Then the flight, and the call home, and landing. The brief's guarantee is
         * that all of it is done within about four seconds of the call. */
        assert(world_run_until(&world, LETTERS_COMING_HOME, 60 * 30) > 0);
        int homing = 0;
        while (the_letters.phase == LETTERS_COMING_HOME && homing < 60 * 8) {
            world_step(&world);
            homing++;
        }
        assert(the_letters.phase == LETTERS_AT_REST);
        double seconds = homing / (double)FRAME_RATE;
        if (getenv("CBIRDS_TEST_NUMBERS"))
            fprintf(stderr, "homing at pace notch %2d: %.2f s\n", NOTCHES[n], seconds);
        assert(seconds <= LETTERS_HOMING_DEADLINE + 0.1);
        assert(world_all_home(&world));

        /* The grid on the screen is exactly the input's grid again. */
        cell_t *after = world_picture(&world, &count_after);
        assert(count_before == count_after);
        assert(memcmp(before, after, (size_t)count_before * sizeof(cell_t)) == 0);
        free(before);
        free(after);

        /* And it goes round again. */
        assert(world_run_until(&world, LETTERS_TAKING_OFF, 60 * 10) > 0);
        assert(the_letters.cycles == 2);
        world_close(&world);
    }
}

static void test_enter_brings_them_home_in_four_seconds_whatever_they_were_doing(void) {
    static const double MOMENTS[] = {0.0, 0.3, 0.7, 1.5, 4.0, 9.0};
    for (size_t m = 0; m < sizeof(MOMENTS) / sizeof(*MOMENTS); m++) {
        world_t world;
        world_open(&world, NEOFETCH_LIKE, 60, 12);
        config.pace_notch = m % 2 ? 0 : 6;
        apply_notches();
        assert(feed_input("\r") == 1); /* Enter at rest: the wave, now. */
        assert(the_letters.phase == LETTERS_TAKING_OFF);
        for (int frame = 0; frame < (int)(MOMENTS[m] * FRAME_RATE); frame++) world_step(&world);
        assert(feed_input("\n") == 1); /* And Enter again: home, at once. */
        assert(the_letters.phase == LETTERS_COMING_HOME);
        int frames = 0;
        while (the_letters.phase != LETTERS_AT_REST && frames < 60 * 8) {
            world_step(&world);
            frames++;
        }
        assert(the_letters.phase == LETTERS_AT_REST);
        assert(frames / (double)FRAME_RATE <= LETTERS_HOMING_DEADLINE + 0.1);
        assert(world_all_home(&world));
        world_close(&world);
    }
}

static void test_a_straggler_is_home_by_the_deadline_however_far_it_is(void) {
    world_t world;
    world_open(&world, "ab", 20, 2);
    config.pace_notch = 0;
    apply_notches();
    feed_input("\r");
    world_run_until(&world, LETTERS_IN_FLIGHT, 60 * 5);
    feed_input("\r");
    /* One of them a very long way off, and facing away, at the slowest pace. */
    world.birds[1].x = 4000;
    world.birds[1].y = -3000;
    world.birds[1].direction = 1.0;
    int frames = 0;
    while (the_letters.phase != LETTERS_AT_REST && frames < 60 * 8) {
        world_step(&world);
        frames++;
    }
    assert(the_letters.phase == LETTERS_AT_REST);
    assert(frames / (double)FRAME_RATE <= LETTERS_HOMING_DEADLINE + 0.1);
    assert(world_all_home(&world));
    world_close(&world);
}

static void test_a_letter_lands_exactly_and_does_not_circle_its_cell(void) {
    world_t world;
    world_open(&world, "x", 20, 2);
    config.pace_notch = 12;
    apply_notches();
    config.turning_notch = 0; /* The laziest turn there is: a bird that banks cannot land. */
    feed_input("\r");
    world_run_until(&world, LETTERS_IN_FLIGHT, 60 * 5);
    feed_input("\r");
    int frames = 0;
    double furthest_after_near = 0;
    int was_near = 0;
    while (the_letters.phase != LETTERS_AT_REST && frames < 60 * 8) {
        world_step(&world);
        frames++;
        double dx = world.birds[0].x - the_letters.letter[0].home_x;
        double dy = world.birds[0].y - the_letters.letter[0].home_y;
        double distance = sqrt(dx * dx + dy * dy);
        if (distance < 20) was_near = 1;
        /* Once it is within a couple of cells it only gets nearer. */
        if (was_near && distance > furthest_after_near && distance > 20)
            furthest_after_near = distance;
    }
    assert(the_letters.phase == LETTERS_AT_REST);
    assert(world.birds[0].x == the_letters.letter[0].home_x &&
           world.birds[0].y == the_letters.letter[0].home_y);
    assert(furthest_after_near == 0);
    world_close(&world);
}

static void test_the_pointer_scatters_what_it_touches_and_they_find_their_way_back(void) {
    world_t world;
    world_open(&world, NEOFETCH_LIKE, 60, 12);
    int count_before;
    cell_t *before = world_picture(&world, &count_before);
    mouse.present = 1;
    mouse.x = 30.5 * 8;
    mouse.y = 3.5 * 16;
    world_step(&world);
    int up = 0;
    for (int i = 0; i < config.birds; i++)
        if (!world.birds[i].perched) up++;
    assert(up > 3 && up < config.birds / 3);
    assert(the_letters.phase == LETTERS_AT_REST);
    /* They fly, and none strays far from where it started in a moment, but they
     * are gone from home: the cells they left are blank. */
    for (int frame = 0; frame < 30; frame++) world_step(&world);
    int count_now;
    cell_t *now = world_picture(&world, &count_now);
    assert(memcmp(before, now, (size_t)count_before * sizeof(cell_t)) != 0);
    free(now);
    /* The pointer goes away, off the text; they come down, each on its own cell. */
    mouse.x = -500;
    mouse.y = -500;
    int frames = 0;
    while (!world_all_home(&world) && the_letters.phase == LETTERS_AT_REST && frames < 60 * 6) {
        world_step(&world);
        frames++;
    }
    assert(world_all_home(&world));
    int count_after;
    cell_t *after = world_picture(&world, &count_after);
    assert(memcmp(before, after, (size_t)count_before * sizeof(cell_t)) == 0);
    free(before);
    free(after);
    world_close(&world);
}

static void test_a_hawk_over_the_text_is_an_arrow_and_scatters_it(void) {
    world_t world;
    world_open(&world, NEOFETCH_LIKE, 60, 12);
    config.hawks = 1;
    place_hawks();
    hawks[0].x = 30 * 8;
    hawks[0].y = 3.5 * 16;
    hawks[0].direction = 0;
    for (int frame = 0; frame < 20; frame++) world_step(&world);
    int up = 0;
    for (int i = 0; i < config.birds; i++)
        if (!world.birds[i].perched) up++;
    assert(up > 0);
    int count;
    cell_t *picture = world_picture(&world, &count);
    int arrows = 0;
    for (int i = 0; i < count; i++)
        if (picture[i].glyph >= 0x2190 && picture[i].glyph <= 0x2199 && picture[i].has_fg) arrows++;
    assert(arrows == 1);
    free(picture);
    world_close(&world);
}

static void test_the_panel_lies_over_the_text_and_the_letters_under_it_still_land(void) {
    world_t world;
    world_open(&world, NEOFETCH_LIKE, 80, 24);
    legend_enabled = 1;
    measure_legend();
    update_turn_distances();
    assert(screen.legend_width > 0);
    assert(!legend_turn_zone(10, 10)); /* No wall: it is a window laid on the text. */
    int count_before;
    cell_t *before = world_picture(&world, &count_before);
    feed_input("\r");
    assert(world_run_until(&world, LETTERS_IN_FLIGHT, 60 * 6) > 0);
    feed_input("\r");
    int frames = 0;
    while (the_letters.phase != LETTERS_AT_REST && frames < 60 * 8) {
        world_step(&world);
        frames++;
    }
    assert(the_letters.phase == LETTERS_AT_REST && world_all_home(&world));
    int count_after;
    cell_t *after = world_picture(&world, &count_after);
    assert(memcmp(before, after, (size_t)count_before * sizeof(cell_t)) == 0);
    free(before);
    free(after);
    world_close(&world);
}

static void test_pausing_stops_the_cycle_and_the_text_is_not_drawn_twice(void) {
    world_t world;
    world_open(&world, NEOFETCH_LIKE, 60, 12);
    kitty_graphics_t graphics;
    assert(kitty_graphics_init(&graphics, STDOUT_FILENO) == KITTY_GRAPHICS_OK);
    paused = 1;
    double clock_before = the_letters.clock;
    for (int frame = 0; frame < 60 * 10; frame++) {
        graphics.length = 0;
        assert(render_frame(&graphics, world.birds, world.snapshot, &world.grid) ==
               KITTY_GRAPHICS_OK);
    }
    /* Ten seconds paused is no time at all to the text: still the first rest. */
    assert(the_letters.clock == clock_before && the_letters.phase == LETTERS_AT_REST);
    paused = 0;
    step_once = 0;
    kitty_graphics_destroy(&graphics);
    world_close(&world);
}

static void test_the_keys_that_change_the_population_do_nothing_to_text(void) {
    world_t world;
    world_open(&world, NEOFETCH_LIKE, 60, 12);
    int birds = config.birds;
    assert(feed_input("+") == 1 && feed_input("-") == 1 && feed_input("e") == 1);
    assert(config.birds == birds && !population_changed && !config.trails);
    assert(feed_input("q") == 0); /* q quits, as it always did. */
    world_close(&world);
}

static void test_quitting_flies_the_text_off_the_top(void) {
    world_t world;
    world_open(&world, NEOFETCH_LIKE, 60, 12);
    for (int frame = 0; frame < OUTRO_FRAMES_AT_SIXTY; frame++) {
        fly_away(world.birds);
        for (int i = 0; i < config.birds; i++) assert(!world.birds[i].perched);
    }
    int still_here = 0;
    for (int i = 0; i < config.birds; i++)
        if (world.birds[i].y >= 0) still_here++;
    assert(still_here == 0);
    /* And what is drawn on the way is sound: nothing off the screen is written. */
    int count;
    cell_t *picture = world_picture(&world, &count);
    int glyphs = 0;
    for (int i = 0; i < count; i++)
        if (picture[i].glyph != 0) glyphs++;
    assert(glyphs == 0);
    free(picture);
    world_close(&world);
}

static void test_a_letter_at_home_is_neither_moved_nor_seen_by_the_flock(void) {
    world_t world;
    world_open(&world, "abc\ndef", 20, 4);
    /* Launch one letter by hand and put it among the perched. Alone it has nothing
     * to flock with: the same as a bird by itself. */
    letters_poke(&the_letters);
    world_step(&world);
    int flying = -1;
    for (int i = 0; i < config.birds; i++)
        if (!world.birds[i].perched) flying = i;
    assert(flying >= 0);
    bird_t alone = world.birds[flying];
    alone.perched = 0;
    memcpy(world.snapshot, world.birds, sizeof(*world.birds) * (size_t)config.birds);
    assert(spatial_grid_build(&world.grid, config.birds, read_bird_position, world.snapshot) ==
           SPATIAL_GRID_OK);
    double with_company = flock_direction(world.snapshot, &world.grid, flying);
    /* Without any other letters in the grid at all. */
    bird_t lonely[1] = {alone};
    spatial_grid_t one;
    assert(spatial_grid_init(&one, SPATIAL_CELL_SIZE) == SPATIAL_GRID_OK);
    assert(spatial_grid_prepare(&one, screen.width, screen.height, 1) == SPATIAL_GRID_OK);
    assert(spatial_grid_build(&one, 1, read_bird_position, lonely) == SPATIAL_GRID_OK);
    int saved = config.birds;
    config.birds = 1;
    double without = flock_direction(lonely, &one, 0);
    config.birds = saved;
    assert(with_company == without);
    spatial_grid_destroy(&one);
    /* And a perched one stays exactly where it is while the others fly. */
    bird_t before[6];
    memcpy(before, world.birds, sizeof(before));
    for (int frame = 0; frame < 20; frame++) world_step(&world);
    for (int i = 0; i < config.birds; i++)
        if (before[i].perched && world.birds[i].perched) {
            assert(world.birds[i].x == before[i].x && world.birds[i].y == before[i].y);
        }
    world_close(&world);
}

static void test_how_the_flock_of_letters_looks_in_the_air(void) {
    world_t world;
    world_open(&world, NEOFETCH_LIKE, 80, 24);
    config.pace_notch = LETTERS_PACE_NOTCH;
    apply_notches();
    feed_input("\r");
    assert(world_run_until(&world, LETTERS_IN_FLIGHT, 60 * 6) > 0);
    double visible_total = 0, speed_total = 0;
    int samples = 0;
    for (int frame = 0; frame < 60 * 12; frame++) {
        bird_t before[1024];
        int tracked = config.birds < 1024 ? config.birds : 1024;
        memcpy(before, world.birds, sizeof(*before) * (size_t)tracked);
        world_step(&world);
        if (frame % 30 != 0) continue;
        int count;
        cell_t *picture = world_picture(&world, &count);
        int glyphs = 0;
        for (int i = 0; i < count; i++)
            if (picture[i].glyph != 0 && picture[i].has_fg) glyphs++;
        free(picture);
        visible_total += (double)glyphs / config.birds;
        double moved = 0;
        for (int i = 0; i < tracked; i++)
            moved += hypot(world.birds[i].x - before[i].x, world.birds[i].y - before[i].y);
        speed_total += moved / tracked / 8.0; /* Cells a frame. */
        samples++;
    }
    if (getenv("CBIRDS_TEST_NUMBERS"))
        fprintf(stderr, "letters in the air at 0.2x: %.0f%% visible, %.2f cells a frame\n",
                100.0 * visible_total / samples, speed_total / samples);
    assert(visible_total / samples > 0.3);
    world_close(&world);
}

static void test_a_text_recording_is_the_text_flying(void) {
    char path[600], text_file[600];
    scratch_file(path, sizeof(path), "letters.cast");
    scratch_file(text_file, sizeof(text_file), "rec.txt");
    world_write(text_file, NEOFETCH_LIKE);
    reset_test_config();
    config.palette = palette_named("ember");
    text_path = text_file;
    record_path = path;
    record_fps = 20;
    record_seconds = 8;
    record_columns = 60;
    record_rows = 12;
    assert(take_the_text(record_columns, record_rows, 1) == 1);
    fflush(stdout);
    int saved = dup(STDOUT_FILENO);
    assert(freopen("/dev/null", "w", stdout) != NULL);
    int status = run_recording();
    fflush(stdout);
    dup2(saved, STDOUT_FILENO);
    close(saved);
    clearerr(stdout);
    assert(status == EXIT_SUCCESS);
    letters_mode = 0;
    text_path = NULL;
    remove(text_file);

    FILE *file = fopen(path, "r");
    assert(file != NULL);
    static char line[1 << 16];
    assert(fgets(line, sizeof(line), file) != NULL);
    assert(strstr(line, "\"width\": 60, \"height\": 12") != NULL);
    assert(fgets(line, sizeof(line), file) != NULL); /* The opening. */
    assert(fgets(line, sizeof(line), file) != NULL); /* The first frame: the text. */
    assert(strstr(line, "user") != NULL || strstr(line, "u") != NULL);
    assert(strstr(line, "\\u001b[1m\\u001b[31m") != NULL || strstr(line, "\\u001b[1;31m") != NULL ||
           strstr(line, "\\u001b[31m") != NULL);
    assert(strstr(line, "\xE4\xB8\xAD") != NULL);
    assert(strstr(line, "\\u001b[38;2;255;128;0m") != NULL);
    int events = 1;
    while (fgets(line, sizeof(line), file) != NULL) events++;
    fclose(file);
    remove(path);
    assert(events == 20 * 8 + 1);
    record_path = NULL;
    record_fps = 25;
    record_seconds = 6;
    record_columns = 96;
    record_rows = 26;
    render_mode = RENDER_KITTY;
    reset_test_config();
}

/* The terminal itself, as far as it matters here: the bytes cbirds sends, fed to the
 * emulator, are what a terminal would be showing. The test of the emitter is not
 * that it wrote the cells it was given but that a terminal which read all of it
 * shows them, with every wide glyph whole and nothing left behind by a letter that
 * moved. */
static void feed_the_frame(world_t *world, kitty_graphics_t *graphics, vt_t *terminal) {
    graphics->length = 0;
    assert(queue_text_frame(graphics, world->birds) == KITTY_GRAPHICS_OK);
    vt_feed(terminal, graphics->buffer, graphics->length);
}

static void assert_the_terminal_shows_the_cells(const vt_t *terminal) {
    for (int row = 0; row < text_cells.rows; row++)
        for (int col = 0; col < text_cells.cols; col++) {
            const cell_t *drawn =
                &text_cells.before[(size_t)row * (size_t)text_cells.cols + (size_t)col];
            const vt_cell_t *shown = vt_cell(terminal, col, row);
            assert(shown != NULL);
            uint32_t want = drawn->glyph == 0 ? ' ' : drawn->glyph;
            uint32_t have = shown->glyph == 0 ? ' ' : shown->glyph;
            if (drawn->wide == CELLS_WIDE_TAIL) {
                assert(shown->width == 0);
                continue;
            }
            assert(have == want);
            assert(shown->width == (drawn->wide == CELLS_WIDE_HEAD ? 2 : 1));
            if (want != ' ') {
                /* A letter's own colour is what the terminal shows, as it was given. */
                assert(drawn->has_fg == (shown->style.fg.kind != VT_COLOUR_DEFAULT));
                if (drawn->has_fg && drawn->fg_kind == CELLS_COLOUR_ANSI)
                    assert(shown->style.fg.kind == VT_COLOUR_ANSI &&
                           shown->style.fg.value[0] == drawn->fg[0]);
                if (drawn->has_fg &&
                    (drawn->fg_kind == CELLS_COLOUR_EXACT || drawn->fg_kind == CELLS_COLOUR_RGB))
                    assert(shown->style.fg.kind == VT_COLOUR_RGB &&
                           memcmp(shown->style.fg.value, drawn->fg, 3) == 0);
                assert((shown->style.attributes & VT_BOLD) ==
                       ((drawn->attributes & CELLS_BOLD) ? VT_BOLD : 0));
                assert((shown->style.attributes & VT_UNDERLINE) ==
                       ((drawn->attributes & CELLS_UNDERLINE) ? VT_UNDERLINE : 0));
                assert((shown->style.attributes & VT_REVERSE) ==
                       ((drawn->attributes & CELLS_REVERSE) ? VT_REVERSE : 0));
            }
            assert(drawn->has_bg == (shown->style.bg.kind != VT_COLOUR_DEFAULT));
            if (drawn->has_bg && drawn->bg_kind == CELLS_COLOUR_ANSI)
                assert(shown->style.bg.kind == VT_COLOUR_ANSI &&
                       shown->style.bg.value[0] == drawn->bg[0]);
        }
}

static void test_a_terminal_that_is_sent_everything_shows_the_text_at_every_frame_and_after_a_cycle(
    void) {
    world_t world;
    world_open(&world, NEOFETCH_LIKE, 60, 12);
    config.pace_notch = 6;
    apply_notches();
    kitty_graphics_t graphics;
    assert(kitty_graphics_init(&graphics, STDOUT_FILENO) == KITTY_GRAPHICS_OK);
    vt_t terminal;
    assert(vt_init(&terminal, 60, 12) == 0);
    vt_t original;
    assert(vt_init(&original, 60, 12) == 0);
    vt_feed(&original, NEOFETCH_LIKE, strlen(NEOFETCH_LIKE));
    vt_finish(&original);

    feed_the_frame(&world, &graphics, &terminal);
    assert_the_terminal_shows_the_cells(&terminal);
    /* The very first frame is the command's own output, glyph for glyph and colour
     * for colour, as the emulator reads it. */
    for (int row = 0; row < 12; row++)
        for (int col = 0; col < 60; col++) {
            const vt_cell_t *a = vt_cell(&original, col, row), *b = vt_cell(&terminal, col, row);
            assert((a->glyph == 0 ? ' ' : a->glyph) == (b->glyph == 0 ? ' ' : b->glyph));
            assert(a->width == b->width);
            if (!vt_cell_is_blank(a)) assert(memcmp(&a->style, &b->style, sizeof(vt_style_t)) == 0);
            assert(memcmp(&a->style.bg, &b->style.bg, sizeof(vt_colour_t)) == 0);
        }

    feed_input("\r");
    int frames = 0;
    while (frames < 60 * 12 && the_letters.phase != LETTERS_IN_FLIGHT) {
        world_step(&world);
        feed_the_frame(&world, &graphics, &terminal);
        assert_the_terminal_shows_the_cells(&terminal);
        frames++;
    }
    for (int frame = 0; frame < 120; frame++) {
        world_step(&world);
        feed_the_frame(&world, &graphics, &terminal);
        if (frame % 7 == 0) assert_the_terminal_shows_the_cells(&terminal);
    }
    feed_input("\r");
    while (frames < 60 * 40 && the_letters.phase != LETTERS_AT_REST) {
        world_step(&world);
        feed_the_frame(&world, &graphics, &terminal);
        assert_the_terminal_shows_the_cells(&terminal);
        frames++;
    }
    assert(the_letters.phase == LETTERS_AT_REST && world_all_home(&world));
    feed_the_frame(&world, &graphics, &terminal);
    /* After a whole cycle the terminal shows the command's output again. */
    for (int row = 0; row < 12; row++)
        for (int col = 0; col < 60; col++) {
            const vt_cell_t *a = vt_cell(&original, col, row), *b = vt_cell(&terminal, col, row);
            assert((a->glyph == 0 ? ' ' : a->glyph) == (b->glyph == 0 ? ' ' : b->glyph));
            assert(a->width == b->width);
            if (!vt_cell_is_blank(a)) assert(memcmp(&a->style, &b->style, sizeof(vt_style_t)) == 0);
            assert(memcmp(&a->style.bg, &b->style.bg, sizeof(vt_colour_t)) == 0);
        }
    vt_destroy(&original);
    vt_destroy(&terminal);
    kitty_graphics_destroy(&graphics);
    world_close(&world);
}

static void test_a_text_recording_as_a_gif_is_painted_with_the_font(void) {
    char path[600], text_file[600];
    scratch_file(path, sizeof(path), "letters.gif");
    scratch_file(text_file, sizeof(text_file), "rec.txt");
    world_write(text_file, NEOFETCH_LIKE);
    reset_test_config();
    config.palette = palette_named("ember");
    text_path = text_file;
    record_path = path;
    record_fps = 10;
    record_seconds = 2;
    record_columns = 60;
    record_rows = 12;
    assert(take_the_text(record_columns, record_rows, 1) == 1);
    fflush(stdout);
    int saved = dup(STDOUT_FILENO);
    assert(freopen("/dev/null", "w", stdout) != NULL);
    int status = run_recording();
    fflush(stdout);
    dup2(saved, STDOUT_FILENO);
    close(saved);
    clearerr(stdout);
    assert(status == EXIT_SUCCESS);
    letters_mode = 0;
    text_path = NULL;
    remove(text_file);

    FILE *file = fopen(path, "rb");
    assert(file != NULL);
    uint8_t header[10];
    assert(fread(header, 1, sizeof(header), file) == sizeof(header));
    assert(memcmp(header, "GIF89a", 6) == 0);
    assert((header[6] | header[7] << 8) == 60 * LETTER_PICTURE_WIDTH);
    assert((header[8] | header[9] << 8) == 12 * LETTER_PICTURE_HEIGHT);
    int descriptors = 0, c;
    while ((c = fgetc(file)) != EOF)
        if (c == 0x2C) descriptors++;
    fclose(file);
    remove(path);
    assert(descriptors >= 20);
    record_path = NULL;
    record_fps = 25;
    record_seconds = 6;
    record_columns = 96;
    record_rows = 26;
    render_mode = RENDER_KITTY;
    reset_test_config();
}

static void test_a_window_that_changes_size_lays_the_text_out_again(void) {
    static const char LONG_LINE[] =
        "0123456789012345678901234567890123456789012345678901234567890123456789\n"
        "\033[31mred\033[0m end";
    world_t world;
    world_open(&world, LONG_LINE, 60, 6);
    /* On sixty columns the first line wraps after sixty. */
    assert(the_letters.letter[60].row == 1 && the_letters.letter[60].col == 0);
    int before = config.birds;
    assert(the_text != NULL && the_text_length == strlen(LONG_LINE));

    /* The same size: nothing to do, however long it takes. */
    for (int frame = 0; frame < 60; frame++)
        assert(reflow_the_letters(&world.birds, &world.snapshot) == 0);

    /* A new size is laid out once it has held still for a moment, not before. */
    apply_screen_size(80, 8, 0, 0);
    int frames = 0, laid_out = 0;
    while (!laid_out && frames < 120) {
        laid_out = reflow_the_letters(&world.birds, &world.snapshot);
        frames++;
    }
    assert(laid_out && frames / (double)FRAME_RATE >= REFLOW_SETTLE_SECONDS - 0.02);
    assert(the_letters.cols == 80 && the_letters.rows == 8);
    assert(config.birds == before); /* The same letters, wrapped otherwise: */
    assert(the_letters.letter[60].row == 0 && the_letters.letter[60].col == 60);
    assert(the_letters.letter[70].glyph == 'r' && the_letters.letter[70].row == 1);
    assert(the_letters.phase == LETTERS_AT_REST && world_all_home(&world));
    assert(reflow_the_letters(&world.birds, &world.snapshot) == 0);

    /* And the new letters fly: a whole cycle on the new screen, to be sure the
     * arrays are the right size, which a sanitizer checks. */
    assert(spatial_grid_prepare(&world.grid, screen.width, screen.height, config.birds) ==
           SPATIAL_GRID_OK);
    assert(text_renderer_fits_the_screen());
    int count_before;
    cell_t *picture = world_picture(&world, &count_before);
    assert(count_before == 80 * 8);
    feed_input("\r");
    assert(world_run_until(&world, LETTERS_IN_FLIGHT, 60 * 6) > 0);
    feed_input("\r");
    int home = 0;
    while (the_letters.phase != LETTERS_AT_REST && home < 60 * 8) {
        world_step(&world);
        home++;
    }
    int count_after;
    cell_t *after = world_picture(&world, &count_after);
    assert(memcmp(picture, after, (size_t)count_before * sizeof(cell_t)) == 0);
    free(picture);
    free(after);

    /* A window with no room for any of it keeps what it has. */
    apply_screen_size(80, 8, 0, 0);
    forget_the_text();
    assert(reflow_the_letters(&world.birds, &world.snapshot) == 0);
    world_close(&world);
}

static void test_text_that_does_not_fit_the_screen_it_is_laid_out_on_scrolls(void) {
    /* Forty lines on a screen of twelve: the last twelve are what is there, as they
     * would be in a terminal that had printed them, less the line feed at the end. */
    static char lines[4096];
    size_t at = 0;
    for (int i = 0; i < 40; i++)
        at += (size_t)snprintf(lines + at, sizeof(lines) - at, "line %02d\n", i);
    world_t world;
    world_open(&world, lines, 20, 12);
    assert(the_letters.letter[0].glyph == 'l' && the_letters.letter[2].glyph == 'n');
    int found_last = 0, found_first = 0;
    for (int i = 0; i < config.birds; i++) {
        if (the_letters.letter[i].row == 11 && the_letters.letter[i].glyph == '9') found_last = 1;
        if (the_letters.letter[i].glyph == '0' && the_letters.letter[i + 1].glyph == '1' && i < 8)
            found_first = 1;
    }
    assert(found_last && !found_first);
    /* Line 39 is on the last row: the line feed that ends it is not a line. */
    assert(the_letters.letter[config.birds - 1].row == 11);
    world_close(&world);
}

static void test_a_benchmark_of_text_flies_it_from_the_first_frame(void) {
    char text_file[600];
    scratch_file(text_file, sizeof(text_file), "bench.txt");
    world_write(text_file, NEOFETCH_LIKE);
    reset_test_config();
    text_path = text_file;
    bench_frames = 30;
    assert(take_the_text(200, 50, 0) == 1);
    fflush(stdout);
    int saved = dup(STDOUT_FILENO);
    int capture[2];
    assert(pipe(capture) == 0);
    assert(dup2(capture[1], STDOUT_FILENO) == STDOUT_FILENO);
    close(capture[1]);
    int status = run_benchmark();
    fflush(stdout);
    dup2(saved, STDOUT_FILENO);
    close(saved);
    assert(status == EXIT_SUCCESS);
    static char out[4096];
    ssize_t got = read(capture[0], out, sizeof(out) - 1);
    close(capture[0]);
    assert(got > 0);
    out[got] = '\0';
    assert(strstr(out, "letters ") != NULL);
    assert(strstr(out, "render       braille") != NULL);
    letters_mode = 0;
    text_path = NULL;
    bench_frames = 0;
    remove(text_file);
    render_mode = RENDER_KITTY;
    reset_test_config();
}

/* --- The keys come from the terminal, whatever standard input is ------------------ */

static void test_standard_input_that_is_not_a_terminal_leaves_the_keys_to_the_tty(void) {
    int master = posix_openpt(O_RDWR | O_NOCTTY);
    assert(master >= 0 && grantpt(master) == 0 && unlockpt(master) == 0);
    const char *name = ptsname(master);
    assert(name != NULL);
    int pipes[2];
    assert(pipe(pipes) == 0);
    fflush(NULL);

    /* With a controlling terminal: the keys are read from it, not from the pipe. */
    pid_t child = fork();
    assert(child >= 0);
    if (child == 0) {
        alarm(20);
        close(pipes[1]);
        if (setsid() < 0) _exit(90);
        int terminal = open(name, O_RDWR);
        if (terminal < 0 || ioctl(terminal, TIOCSCTTY, 0) < 0) _exit(91);
        if (dup2(pipes[0], STDIN_FILENO) < 0) _exit(92);
        input_fd = STDIN_FILENO;
        open_the_keys();
        _exit(input_fd != STDIN_FILENO && isatty(input_fd) ? 0 : 1);
    }
    int status = 0;
    assert(waitpid(child, &status, 0) == child);
    assert(WIFEXITED(status) && WEXITSTATUS(status) == 0);

    /* Without one it says so and stops, in words. */
    int errors[2];
    assert(pipe(errors) == 0);
    child = fork();
    assert(child >= 0);
    if (child == 0) {
        alarm(20);
        close(pipes[1]);
        close(errors[0]);
        if (setsid() < 0) _exit(90);
        if (dup2(pipes[0], STDIN_FILENO) < 0 || dup2(errors[1], STDERR_FILENO) < 0) _exit(92);
        input_fd = STDIN_FILENO;
        open_the_keys();
        _exit(0);
    }
    close(errors[1]);
    static char message[512];
    ssize_t got = read(errors[0], message, sizeof(message) - 1);
    assert(got > 0);
    message[got] = '\0';
    assert(waitpid(child, &status, 0) == child);
    assert(WIFEXITED(status) && WEXITSTATUS(status) == EXIT_FAILURE);
    assert(strstr(message, "needs a terminal") != NULL && strstr(message, "/dev/tty") != NULL);
    close(errors[0]);
    close(pipes[0]);
    close(pipes[1]);
    close(master);
}

static void test_keys_and_colour_questions_use_the_descriptor_that_was_chosen(void) {
    int keys[2];
    assert(pipe(keys) == 0);
    int saved = input_fd;
    input_fd = keys[0];
    const char *report = "\033[<35;10;5M";
    assert(write(keys[1], report, strlen(report)) == (ssize_t)strlen(report));
    reset_test_config();
    apply_screen_size(80, 24, 80 * 8, 24 * 16);
    mouse.present = 0;
    assert(handle_input() == 1); /* Read from it, not from standard input. */
    assert(mouse.present && mouse.x == 9.5 * screen.cell_width);
    mouse.present = 0;
    /* A colour question is answered through it as well: the reply arrives there. */
    const char *answer = "\033]11;rgb:0000/0000/0000\033\\";
    assert(write(keys[1], answer, strlen(answer)) == (ssize_t)strlen(answer));
    uint8_t rgb[3] = {1, 1, 1};
    char reply[64];
    size_t length = terminal_query("", 0, reply, sizeof(reply), 200);
    assert(length > 0 && parse_osc_colour(reply, rgb) && rgb[0] == 0);
    input_fd = saved;
    close(keys[0]);
    close(keys[1]);
}

/* ---- Text beside the other modes ----------------------------------------------- */

/* A screen of letters, more than a flock can be: the wave state is the size of a
 * flock, and a text may be four times that. */
static char *screen_of_letters(int cols, int rows) {
    size_t size = (size_t)(cols + 1) * (size_t)rows + 1;
    char *text = malloc(size);
    assert(text != NULL);
    size_t at = 0;
    for (int row = 0; row < rows; row++) {
        for (int col = 0; col < cols; col++) text[at++] = 'a' + (char)((row + col) % 26);
        text[at++] = '\n';
    }
    text[at] = '\0';
    return text;
}

/* No escape wave runs over text. The letters have a take-off wave of their own, a
 * hawk or the pointer scatters them, and nothing else may start one: not a hawk's
 * dive, not a pointer whipped through. The wave state is a flock's size, so a text
 * of more letters than that must not be read from it or written to it at all,
 * which the sanitizers see if it is. */
static void test_a_text_tells_no_alarm_and_never_reads_the_wave_state(void) {
    world_t world;
    char *text = screen_of_letters(120, 50);
    world_open(&world, text, 120, 50);
    free(text);
    assert(config.birds == 6000 && config.birds > MAX_BIRDS);
    config.hawks = 2;
    place_hawks();
    letters_poke(&the_letters);

    int whipped = 0, scattered = 0;
    for (int frame = 0; frame < 400; frame++) {
        /* A pointer going across the screen faster than a whip has to. */
        mouse.present = 1;
        mouse.x = 40.0 + 8.0 * frame;
        mouse.y = 200.0;
        mouse.velocity_x = 100.0 * screen.cell_width;
        mouse.velocity_y = 0;
        mouse.moved_at = clock_state.seconds;
        clock_state.seconds += frame_seconds;
        if (pointer_startles()) whipped++;
        world_step(&world);
        assert(wave_task_count == 0 && !waves_in_flight);
        for (int i = 0; i < config.birds; i += 97) assert(!world.birds[i].alarmed);
        for (int i = 0; i < config.birds; i++)
            if (!world.birds[i].perched) {
                scattered++;
                break;
            }
    }
    assert(whipped > 300); /* The pointer was a whip, and the text did not care. */
    assert(scattered > 0);
    for (int i = 0; i < MAX_BIRDS; i++) {
        assert(waves[i].wait == 0 && waves[i].left == 0 && waves[i].rest == 0);
        assert(waves[i].swerve == 0 && waves[i].heading == 0);
    }
    for (int i = 0; i < config.birds; i++) assert(!world.birds[i].alarmed);
    world_close(&world);
}

/* A recording of text with hawks has the colours the letters fly in and the hawk's
 * in its table, and not the light of a wave: there is none. */
static int text_recording_has(const uint8_t colour[3], int hawks) {
    char path[600], text_file[600];
    scratch_file(path, sizeof(path), "text_light.gif");
    scratch_file(text_file, sizeof(text_file), "text_light.txt");
    world_write(text_file, NEOFETCH_LIKE);
    reset_test_config();
    config.palette = palette_named("ember");
    config.hawks = hawks;
    text_path = text_file;
    record_path = path;
    record_fps = 10;
    record_seconds = 5; /* Long enough for hawks to be about and for a wave, were there any. */
    record_columns = 60;
    record_rows = 12;
    assert(take_the_text(record_columns, record_rows, 1) == 1);
    fflush(stdout);
    int saved = dup(STDOUT_FILENO);
    assert(freopen("/dev/null", "w", stdout) != NULL);
    int status = run_recording();
    fflush(stdout);
    dup2(saved, STDOUT_FILENO);
    close(saved);
    clearerr(stdout);
    assert(status == EXIT_SUCCESS);
    letters_mode = 0;
    text_path = NULL;
    remove(text_file);
    FILE *file = fopen(path, "rb");
    assert(file != NULL);
    uint8_t header[13 + 768];
    assert(fread(header, 1, sizeof(header), file) == sizeof(header));
    fclose(file);
    remove(path);
    record_path = NULL;
    record_fps = 25;
    record_seconds = 6;
    record_columns = 96;
    record_rows = 26;
    render_mode = RENDER_KITTY;
    int found = 0;
    for (int entry = 0; entry < 256; entry++)
        if (memcmp(header + 13 + entry * 3, colour, 3) == 0) found = 1;
    reset_test_config();
    return found;
}

static void test_a_text_with_hawks_records_the_flight_and_not_the_light(void) {
    reset_test_config();
    config.palette = palette_named("ember");
    uint8_t light[3], hawk[3], first[3], last[3];
    memcpy(light, highlight_colour(), 3);
    memcpy(hawk, hawk_colour(), 3);
    memcpy(first, palette()->tints[0], 3);
    memcpy(last, palette()->tints[palette()->shades - 1], 3);
    assert(text_recording_has(first, 0) && text_recording_has(last, 0)); /* The ramp it flies in. */
    assert(text_recording_has(hawk, 2) && !text_recording_has(hawk, 0));
    assert(!text_recording_has(light, 2)); /* No wave, so no light. */
    /* A flock with hawks still has it, in the palette that helper records in. */
    config.palette = palette_named("ice");
    memcpy(light, highlight_colour(), 3);
    assert(palette_of_a_recording_has(light, 2, 5));
    reset_test_config();
}

/* Zero is "not asked for", and six seconds for a flock and a whole cycle for text
 * are what it settles to: once, before any reader sees it. The readers are the two
 * recorders, and the light of a wave, which is decided by how long a clip is. */
static void test_an_unasked_for_recording_length_is_settled_before_anything_reads_it(void) {
    reset_test_config();
    record_seconds = 0;
    letters_mode = 0;
    settle_the_recording_length();
    assert(record_seconds == FLOCK_RECORD_SECONDS);
    settle_the_recording_length();
    assert(record_seconds == FLOCK_RECORD_SECONDS);
    record_seconds = 0;
    letters_mode = 1;
    settle_the_recording_length();
    assert(record_seconds == TEXT_RECORD_SECONDS);
    record_seconds = 12; /* Asked for: left alone, text or not. */
    settle_the_recording_length();
    assert(record_seconds == 12);
    letters_mode = 0;

    /* The cast recorder reads it, and a night and a flock with hawks are flocks. */
    char path[600];
    scratch_file(path, sizeof(path), "unasked.cast");
    for (int night_run = 0; night_run < 2; night_run++) {
        reset_test_config();
        if (night_run) {
            fireflies_mode = 1;
            config.birds = FIREFLY_COUNT;
            config.palette = palette_named("firefly");
            config.shape = shape_named("dot");
            config.bird_size = 0;
        } else {
            config.birds = 50;
            config.hawks = 1;
        }
        record_path = path;
        record_fps = 5;
        record_seconds = 0;
        record_columns = 60;
        record_rows = 20;
        requested_seed = 7;
        fflush(stdout);
        int saved = dup(STDOUT_FILENO);
        assert(freopen("/dev/null", "w", stdout) != NULL);
        int status = run_recording();
        fflush(stdout);
        dup2(saved, STDOUT_FILENO);
        close(saved);
        clearerr(stdout);
        assert(status == EXIT_SUCCESS);
        assert(record_seconds == FLOCK_RECORD_SECONDS);
        FILE *file = fopen(path, "r");
        assert(file != NULL);
        static char line[1 << 16];
        int events = 0;
        assert(fgets(line, sizeof(line), file) != NULL); /* The header. */
        while (fgets(line, sizeof(line), file) != NULL) events++;
        fclose(file);
        remove(path);
        /* The clear, a frame for every one of the seconds, and the end. */
        assert(events == record_fps * FLOCK_RECORD_SECONDS + 2);
    }
    record_path = NULL;
    record_fps = 25;
    record_seconds = 6;
    record_columns = 96;
    record_rows = 26;
    requested_seed = -1;
    reset_test_config();

    /* The light of a wave is decided by the length of a clip: unasked for, it is
     * the length it settles to and not zero, so a flock with hawks has it. */
    reset_test_config();
    config.palette = palette_named("ice");
    uint8_t light[3];
    memcpy(light, highlight_colour(), 3);
    assert(palette_of_a_recording_has(light, 2, 0));
    reset_test_config();
}

/* A text and a night are two flocks, and the text is the flock: said in one line,
 * before anything is read, with the exit status of every other usage error. Text on
 * a pipe is said when it is found there, and a pipe with nothing in it is no text. */
static int status_of_a_run_that_may_exit(void (*run)(void *), void *context, char *said,
                                         size_t size) {
    int errors[2];
    assert(pipe(errors) == 0);
    fflush(NULL);
    pid_t child = fork();
    assert(child >= 0);
    if (child == 0) {
        alarm(20);
        close(errors[0]);
        if (dup2(errors[1], STDERR_FILENO) < 0) _exit(99);
        run(context);
        _exit(0);
    }
    close(errors[1]);
    /* To its end, so that a child that says more than a line is not killed for it. */
    size_t length = 0;
    ssize_t got;
    while ((got = read(errors[0], said + length, size - 1 - length)) > 0) length += (size_t)got;
    said[length] = '\0';
    close(errors[0]);
    int status = 0;
    assert(waitpid(child, &status, 0) == child);
    assert(WIFEXITED(status));
    return WEXITSTATUS(status);
}

static void parse_a_night_with_text(void *context) {
    char *argv[] = {"cbirds", "--fireflies", "--text", (char *)context, NULL};
    read_options(4, argv);
}

static void parse_text_with_a_night(void *context) {
    char *argv[] = {"cbirds", "--text", (char *)context, "--fireflies", NULL};
    read_options(4, argv);
}

static void take_the_text_piped_in_on_a_night(void *context) {
    int fd = open((const char *)context, O_RDONLY);
    if (fd < 0 || dup2(fd, STDIN_FILENO) < 0) _exit(98);
    fireflies_mode = 1;
    int taken = take_the_text(60, 12, 1);
    _exit(taken ? 0 : 3); /* A night goes on when there was nothing to see. */
}

static void test_the_text_is_the_flock_and_a_night_is_refused_with_it(void) {
    char file[600], empty[600], said[512];
    scratch_file(file, sizeof(file), "refuse.txt");
    scratch_file(empty, sizeof(empty), "refuse_empty.txt");
    world_write(file, "hello, world\n");
    world_write(empty, "");

    /* Either way round on the line, a usage error and one line of it. */
    assert(status_of_a_run_that_may_exit(parse_a_night_with_text, file, said, sizeof(said)) ==
           EXIT_USAGE);
    assert(strstr(said, "--fireflies does not go with --text") != NULL);
    assert(strchr(said, '\n') == said + strlen(said) - 1);
    assert(status_of_a_run_that_may_exit(parse_text_with_a_night, file, said, sizeof(said)) ==
           EXIT_USAGE);
    assert(strstr(said, "--fireflies does not go with --text") != NULL);

    /* On a pipe it is the text that is found, not the pipe. */
    assert(status_of_a_run_that_may_exit(take_the_text_piped_in_on_a_night, file, said,
                                         sizeof(said)) == EXIT_USAGE);
    assert(strstr(said, "does not go with text on standard input") != NULL);
    assert(strchr(said, '\n') == said + strlen(said) - 1);
    /* An empty one is nothing to see, and the night goes on. */
    assert(status_of_a_run_that_may_exit(take_the_text_piped_in_on_a_night, empty, said,
                                         sizeof(said)) == 3);
    assert(said[0] == '\0');

    remove(file);
    remove(empty);
    reset_test_config();
}

/* A whip is read from the descriptor the keys come from, when standard input is a
 * pipe and the keys come from the terminal: two reports a moment apart are a
 * velocity, and a quick one is a whip. */
static void test_a_whip_is_read_from_the_descriptor_that_was_chosen(void) {
    int keys[2];
    assert(pipe(keys) == 0);
    int saved = input_fd;
    input_fd = keys[0];
    reset_test_config();
    apply_screen_size(80, 24, 80 * 8, 24 * 16);
    mouse.present = 0;
    clock_state.seconds = 10.0;
    const char *first = "\033[<35;5;5M";
    assert(write(keys[1], first, strlen(first)) == (ssize_t)strlen(first));
    assert(handle_input() == 1);
    clock_state.seconds = 10.05;
    const char *second = "\033[<35;45;5M"; /* Forty columns in fifty milliseconds. */
    assert(write(keys[1], second, strlen(second)) == (ssize_t)strlen(second));
    assert(handle_input() == 1);
    assert(mouse.velocity_x > 0 && pointer_startles());
    mouse.present = 0;
    input_fd = saved;
    close(keys[0]);
    close(keys[1]);
    reset_test_config();
}

/* ---- Signs beside the other modes ----------------------------------------------- */

/* What a line of options can leave behind in the program's own variables, put back
 * as a fresh process has them. */
static void forget_the_options(void) {
    reset_sign_state();
    text_path = NULL;
    forget_the_text();
    letters_mode = 0;
    fireflies_mode = 0;
    sky_mode = 0;
    deep_look = 0;
    matrix_mode = 0;
    the_rain_is_falling = 0;
    requested_preset = -1;
    config.shape = 0;
    config.bird_size = 0;
    config.hawks = 0;
}

/* A line, split at its spaces, read as the program reads its own. */
static void read_the_line(const char *line) {
    static char copy[512];
    char *words[32] = {"cbirds"};
    int count = 1;
    forget_the_options();
    assert(strlen(line) < sizeof(copy));
    strcpy(copy, line);
    for (char *word = strtok(copy, " "); word != NULL && count < 31; word = strtok(NULL, " "))
        words[count++] = word;
    words[count] = NULL;
    read_options(count, words);
}

/* The three things a night, a picture and the shipped flock disagree about are
 * left open while the line is read and settled together once it has been, so the
 * order of the words on the line cannot matter and no stand-in is left behind. */
static void test_what_was_not_asked_for_is_settled_together(void) {
    /* The pair, by hand. */
    forget_the_options();
    config.birds = 800;
    config.shape = 0;
    config.palette = 0;
    leave_the_defaults_open();
    assert(config.birds == 0 && config.shape == -1 && config.palette == -1);
    assert(shipped.birds == 800 && shipped.shape == 0 && shipped.palette == 0);
    settle_the_defaults();
    assert(config.birds == 800 && config.shape == 0 && config.palette == 0);
    assert(!palette_was_asked_for);

    /* A night takes its own for whatever was left open. */
    config.palette = palette_named("ice");
    leave_the_defaults_open();
    fireflies_mode = 1;
    settle_the_defaults();
    assert(config.birds == FIREFLY_COUNT && config.shape == shape_named("dot"));
    assert(config.palette == palette_named("firefly") && !palette_was_asked_for);

    /* And not for what was asked for, which is a ramp given even when it is the
     * default's own name. */
    forget_the_options();
    config.birds = 800;
    config.palette = 0;
    leave_the_defaults_open();
    config.birds = 50;
    config.palette = 0;
    fireflies_mode = 1;
    settle_the_defaults();
    assert(config.birds == 50 && config.shape == shape_named("dot") && config.palette == 0);
    assert(palette_was_asked_for);

    /* The same through the whole option reader, whichever way round the line is. */
    char picture[512];
    write_a_picture("settled.png", 1, 255);
    scratch_file(picture, sizeof(picture), "settled.png");
    char line[600];

    read_the_line("");
    assert(config.birds == 800 && config.shape == 0 && config.palette == 0);
    assert(!fireflies_mode && !palette_was_asked_for);

    read_the_line("--fireflies");
    assert(config.birds == FIREFLY_COUNT && config.shape == shape_named("dot"));
    assert(config.palette == palette_named("firefly"));

    /* A space takes its own birds and ramp for whatever was left open, a bird as
     * the flock has it, and what was asked for wins whichever side of --3d it is
     * on, even when it is the very thing the flock ships with. */
    read_the_line("--3d");
    assert(sky_mode && config.birds == SKY_BIRDS && config.shape == 0);
    assert(config.palette == palette_named("ink") && !palette_was_asked_for);
    const char *space_both_ways[][2] = {
        {"-n 800 --3d", "--3d -n 800"},           {"--color ash --3d", "--3d --color ash"},
        {"--color ink --3d", "--3d --color ink"}, {"--color theme --3d", "--3d --color theme"},
        {"--shape dot --3d", "--3d --shape dot"},
    };
    const int space_birds[] = {800, SKY_BIRDS, SKY_BIRDS, SKY_BIRDS, SKY_BIRDS};
    const char *space_ramp[] = {"ink", "ash", "ink", "theme", "ink"};
    for (size_t pair = 0; pair < sizeof(space_both_ways) / sizeof(*space_both_ways); pair++)
        for (int way = 0; way < 2; way++) {
            read_the_line(space_both_ways[pair][way]);
            assert(sky_mode && config.birds == space_birds[pair]);
            assert(config.palette == palette_named(space_ramp[pair]));
            assert(palette_was_asked_for == (pair == 1 || pair == 2 || pair == 3));
            assert(config.shape == (pair == 4 ? shape_named("dot") : 0));
        }
    /* The flock's own birds are 800 and its ramp theme, as ever, with no --3d. */
    read_the_line("");
    assert(!sky_mode && config.birds == 800 && config.palette == 0);

    const char *both_ways[][2] = {
        {"-n 50 --fireflies", "--fireflies -n 50"},
        {"--shape bird --fireflies", "--fireflies --shape bird"},
        {"--color theme --fireflies", "--fireflies --color theme"},
        {"--color ice --fireflies", "--fireflies --color ice"},
    };
    for (size_t pair = 0; pair < sizeof(both_ways) / sizeof(*both_ways); pair++) {
        int birds[2], shape[2], palette[2], asked[2];
        for (int way = 0; way < 2; way++) {
            read_the_line(both_ways[pair][way]);
            birds[way] = config.birds;
            shape[way] = config.shape;
            palette[way] = config.palette;
            asked[way] = palette_was_asked_for;
            /* No stand-in survives the reader. */
            assert(config.birds > 0 && config.shape >= 0 && config.shape < SHAPE_COUNT);
            assert(config.palette >= 0 && config.palette < PALETTE_COUNT);
        }
        assert(birds[0] == birds[1] && shape[0] == shape[1] && palette[0] == palette[1]);
        assert(asked[0] == asked[1]);
    }
    read_the_line("-n 50 --fireflies");
    assert(config.birds == 50 && config.palette == palette_named("firefly"));
    read_the_line("--shape bird --fireflies");
    assert(config.shape == shape_named("bird") && config.birds == FIREFLY_COUNT);
    read_the_line("--color theme --fireflies");
    assert(config.palette == 0 && palette_was_asked_for);

    /* A picture is in its own colours unless a ramp was named, first or last. */
    snprintf(line, sizeof(line), "--picture %s", picture);
    read_the_line(line);
    assert(picture_colours_in_use && !palette_was_asked_for && config.palette == 0);
    snprintf(line, sizeof(line), "--picture %s --color ice", picture);
    read_the_line(line);
    assert(!picture_colours_in_use && palette_was_asked_for);
    assert(config.palette == palette_named("ice"));
    snprintf(line, sizeof(line), "--color ice --picture %s", picture);
    read_the_line(line);
    assert(!picture_colours_in_use && config.palette == palette_named("ice"));
    snprintf(line, sizeof(line), "--picture %s --color theme", picture);
    read_the_line(line);
    assert(!picture_colours_in_use && palette_was_asked_for && config.palette == 0);

    /* --matrix names the green ramp, so a picture beside it is drawn in that; and
     * with no picture, it is what it was. */
    snprintf(line, sizeof(line), "--picture %s --matrix", picture);
    read_the_line(line);
    assert(!picture_colours_in_use && palette_was_asked_for);
    assert(config.palette == palette_named("matrix") && the_rain_is_falling);
    read_the_line("--matrix");
    assert(config.palette == palette_named("matrix") && the_rain_is_falling);

    assert(unlink(picture) == 0);
    forget_the_options();
}

/* A line read in a child, for the lines that end the run. */
static void read_the_line_in_a_child(void *line) {
    read_the_line((const char *)line);
}

/* The same line, and then the text found on a pipe. */
typedef struct {
    const char *line, *text_file;
} piped_line_t;

static void take_the_text_piped_in_on_a_line(void *context) {
    const piped_line_t *piped = context;
    read_the_line(piped->line);
    int fd = open(piped->text_file, O_RDONLY);
    if (fd < 0 || dup2(fd, STDIN_FILENO) < 0) _exit(98);
    int taken = take_the_text(60, 12, 1);
    _exit(taken ? 0 : 3); /* A flock goes on when there was nothing to see. */
}

/* A night, a text and a sign each want the flock: a night has no letters to write
 * with and a text leaves nobody to write a sign. Said in one line, naming the two,
 * with the status of every other usage error, in either order on the line, and
 * before the picture is read. */
static void test_a_sign_is_refused_with_a_night_and_with_text(void) {
    char text[600], empty[600], picture[600], said[512], line[1200];
    scratch_file(text, sizeof(text), "signs_refuse.txt");
    scratch_file(empty, sizeof(empty), "signs_refuse_empty.txt");
    world_write(text, "hello, world\n");
    world_write(empty, "");
    write_a_picture("refuse.png", 1, 255);
    scratch_file(picture, sizeof(picture), "refuse.png");

    const char *signs[][2] = {
        {"--say hi", "--say"},
        {"--clock", "--clock"},
        {"--clock-at 10:00", "--clock-at"},
        {"--picture /nowhere/at/all.png", "--picture"}, /* Never opened. */
    };
    for (size_t s = 0; s < sizeof(signs) / sizeof(*signs); s++) {
        const char *asked = signs[s][0], *name = signs[s][1];
        char expected[128];

        /* A night, either way round. */
        snprintf(line, sizeof(line), "--fireflies %s", asked);
        assert(status_of_a_run_that_may_exit(read_the_line_in_a_child, line, said, sizeof(said)) ==
               EXIT_USAGE);
        snprintf(expected, sizeof(expected), "--fireflies does not go with %s:", name);
        assert(strstr(said, expected) != NULL);
        assert(strchr(said, '\n') == said + strlen(said) - 1);
        snprintf(line, sizeof(line), "%s --fireflies", asked);
        assert(status_of_a_run_that_may_exit(read_the_line_in_a_child, line, said, sizeof(said)) ==
               EXIT_USAGE);
        assert(strstr(said, expected) != NULL && strchr(said, '\n') == said + strlen(said) - 1);

        /* A text file, either way round, before anything is opened. */
        snprintf(line, sizeof(line), "%s --text %s", asked, text);
        assert(status_of_a_run_that_may_exit(read_the_line_in_a_child, line, said, sizeof(said)) ==
               EXIT_USAGE);
        snprintf(expected, sizeof(expected), "%s does not go with --text:", name);
        assert(strstr(said, expected) != NULL);
        assert(strchr(said, '\n') == said + strlen(said) - 1);
        snprintf(line, sizeof(line), "--text %s %s", text, asked);
        assert(status_of_a_run_that_may_exit(read_the_line_in_a_child, line, said, sizeof(said)) ==
               EXIT_USAGE);
        assert(strstr(said, expected) != NULL);
        /* And the text of standard input, named, which is the same text. */
        snprintf(line, sizeof(line), "--text - %s", asked);
        assert(status_of_a_run_that_may_exit(read_the_line_in_a_child, line, said, sizeof(said)) ==
               EXIT_USAGE);

        /* Text on a pipe is said when it is found, and the status is the same. */
        if (strstr(asked, "--picture") != NULL)
            snprintf(line, sizeof(line), "--picture %s", picture);
        else
            snprintf(line, sizeof(line), "%s", asked);
        piped_line_t piped = {line, text};
        assert(status_of_a_run_that_may_exit(take_the_text_piped_in_on_a_line, &piped, said,
                                             sizeof(said)) == EXIT_USAGE);
        snprintf(expected, sizeof(expected), "%s does not go with text on standard input:", name);
        assert(strstr(said, expected) != NULL);
        assert(strchr(said, '\n') == said + strlen(said) - 1);
        /* A pipe with nothing in it is no text, and the sign goes on. */
        piped.text_file = empty;
        assert(status_of_a_run_that_may_exit(take_the_text_piped_in_on_a_line, &piped, said,
                                             sizeof(said)) == 3);
        assert(said[0] == '\0');
    }

    /* Asked for is asked for, whatever came of it: a text the font cannot draw is
     * no sign, and a text beside it is still a mistake. */
    snprintf(line, sizeof(line), "--say \x01 --text %s", text);
    assert(status_of_a_run_that_may_exit(read_the_line_in_a_child, line, said, sizeof(said)) ==
           EXIT_USAGE);
    assert(strstr(said, "--say does not go with --text") != NULL);

    /* What does go with them goes: the lock screen goes with everything. */
    const char *fine[] = {"--screensaver --fireflies", "--screensaver --say hi",
                          "--screensaver --clock", "--screensaver --clock-at 10:00"};
    for (size_t f = 0; f < sizeof(fine) / sizeof(*fine); f++) {
        assert(status_of_a_run_that_may_exit(read_the_line_in_a_child, (void *)fine[f], said,
                                             sizeof(said)) == 0);
        assert(said[0] == '\0');
    }
    snprintf(line, sizeof(line), "--screensaver --text %s", text);
    assert(status_of_a_run_that_may_exit(read_the_line_in_a_child, line, said, sizeof(said)) == 0);
    assert(said[0] == '\0');
    snprintf(line, sizeof(line), "--screensaver --picture %s", picture);
    assert(status_of_a_run_that_may_exit(read_the_line_in_a_child, line, said, sizeof(said)) == 0);
    assert(said[0] == '\0');

    assert(unlink(text) == 0 && unlink(empty) == 0 && unlink(picture) == 0);
    forget_the_options();
}

/* A space is the flock in another sky, and a night, a text and a sign are all made
 * on the flat one, with its screen to lay them out on: said in one line, naming the
 * two, with the status of every other usage error, in either order on the line,
 * before anything is opened. A text on a pipe is said when it is found. What does
 * go with it goes: the lock screen, hawks, and the notes about what it replaces. */
static void test_a_space_is_refused_with_a_night_a_text_and_a_sign(void) {
    char text[600], empty[600], picture[600], said[512], line[1200];
    scratch_file(text, sizeof(text), "space_refuse.txt");
    scratch_file(empty, sizeof(empty), "space_refuse_empty.txt");
    world_write(text, "hello, world\n");
    world_write(empty, "");
    write_a_picture("space_refuse.png", 1, 255);
    scratch_file(picture, sizeof(picture), "space_refuse.png");

    const char *asked[][2] = {
        {"--fireflies", "--fireflies"},
        {"--say hi", "--say"},
        {"--clock", "--clock"},
        {"--clock-at 10:00", "--clock-at"},
        {"--picture /nowhere/at/all.png", "--picture"}, /* Never opened. */
    };
    for (size_t a = 0; a < sizeof(asked) / sizeof(*asked); a++) {
        char expected[128];
        snprintf(expected, sizeof(expected), "--3d does not go with %s:", asked[a][1]);
        snprintf(line, sizeof(line), "--3d %s", asked[a][0]);
        assert(status_of_a_run_that_may_exit(read_the_line_in_a_child, line, said, sizeof(said)) ==
               EXIT_USAGE);
        assert(strstr(said, expected) != NULL);
        assert(strchr(said, '\n') == said + strlen(said) - 1);
        snprintf(line, sizeof(line), "%s --3d", asked[a][0]);
        assert(status_of_a_run_that_may_exit(read_the_line_in_a_child, line, said, sizeof(said)) ==
               EXIT_USAGE);
        assert(strstr(said, expected) != NULL && strchr(said, '\n') == said + strlen(said) - 1);
    }

    /* A text file, either way round, and the text of standard input, named. */
    snprintf(line, sizeof(line), "--3d --text %s", text);
    assert(status_of_a_run_that_may_exit(read_the_line_in_a_child, line, said, sizeof(said)) ==
           EXIT_USAGE);
    assert(strstr(said, "--3d does not go with --text:") != NULL);
    assert(strchr(said, '\n') == said + strlen(said) - 1);
    snprintf(line, sizeof(line), "--text %s --3d", text);
    assert(status_of_a_run_that_may_exit(read_the_line_in_a_child, line, said, sizeof(said)) ==
           EXIT_USAGE);
    assert(strstr(said, "--3d does not go with --text:") != NULL);
    assert(status_of_a_run_that_may_exit(read_the_line_in_a_child, "--3d --text -", said,
                                         sizeof(said)) == EXIT_USAGE);
    assert(strstr(said, "--3d does not go with --text:") != NULL);

    /* Text on a pipe is said when it is found, and a pipe with nothing in it is no
     * text, so the space goes on. */
    piped_line_t piped = {"--3d", text};
    assert(status_of_a_run_that_may_exit(take_the_text_piped_in_on_a_line, &piped, said,
                                         sizeof(said)) == EXIT_USAGE);
    assert(strstr(said, "--3d does not go with text on standard input:") != NULL);
    assert(strchr(said, '\n') == said + strlen(said) - 1);
    piped.text_file = empty;
    assert(status_of_a_run_that_may_exit(take_the_text_piped_in_on_a_line, &piped, said,
                                         sizeof(said)) == 3);
    assert(said[0] == '\0');

    /* The refusals the space had before stand, and say the same. */
    assert(status_of_a_run_that_may_exit(read_the_line_in_a_child, "--3d --flocks 2", said,
                                         sizeof(said)) == EXIT_USAGE);
    assert(strstr(said, "--3d is one flock over one roost; --flocks is for the flat sky") != NULL);
    assert(status_of_a_run_that_may_exit(read_the_line_in_a_child, "--3d --matrix", said,
                                         sizeof(said)) == EXIT_USAGE);
    assert(strstr(said, "--matrix is for the flat sky") != NULL);

    /* What does go with it goes, and what it replaces is said once. */
    const char *fine[] = {"--3d --screensaver", "--3d --hawks 2", "--3d --hawks 1 --color ember",
                          "--3d -n 800 --render braille"};
    for (size_t f = 0; f < sizeof(fine) / sizeof(*fine); f++) {
        assert(status_of_a_run_that_may_exit(read_the_line_in_a_child, (void *)fine[f], said,
                                             sizeof(said)) == 0);
        assert(said[0] == '\0');
    }
    assert(status_of_a_run_that_may_exit(read_the_line_in_a_child, "--3d --depth", said,
                                         sizeof(said)) == 0);
    assert(strstr(said, "--3d replaces --depth") != NULL);

    assert(unlink(text) == 0 && unlink(empty) == 0 && unlink(picture) == 0);
    forget_the_options();
}

/* A writer has somewhere to be: in a sign as in the intro it is never caught by an
 * alarm, never lit and never turns. A writer that a hawk or the pointer has
 * scattered is a bird of the sky until it is home, and is caught like one. */
static void test_a_writer_is_never_alarmed_and_a_scattered_one_is(void) {
    sign_sky_t world;
    lay_out_a_sign_on(&world, 160, 45, 25, "HI", 400, 5);
    int writer = -1, free_bird = -1;
    for (int i = 0; i < config.birds; i++) {
        if (formation.slot[i] >= 0 && writer < 0) writer = i;
        if (formation.slot[i] < 0 && world.birds[i].layer == 0 && free_bird < 0) free_bird = i;
    }
    assert(writer >= 0 && free_bird >= 0);
    reset_the_waves();

    /* Asked of the function that decides it. */
    assert(!bird_can_be_alarmed(&world.birds[writer], writer));
    assert(bird_can_be_alarmed(&world.birds[free_bird], free_bird));
    world.birds[writer].scattered = 1.0;
    assert(bird_can_be_alarmed(&world.birds[writer], writer));
    world.birds[writer].scattered = 0;
    assert(!bird_can_be_alarmed(&world.birds[writer], writer));

    /* The intro's letters are as they were: never. */
    formation_clear();
    begin_the_sign();
    formation_clear();
    formation.sign = 0;
    assert(formation_layout("BOIDS") > 0);
    assert(!bird_can_be_alarmed(&world.birds[0], 0));
    formation_clear();
    assert(bird_can_be_alarmed(&world.birds[0], 0));
    close_the_world(&world);
    reset_sign_state();
}

/* A bird that was told to swerve, and then was sent to write before it turned,
 * forgets what it was told: it does not turn, is not lit, and tells nobody. The
 * control is a free bird told the same, which does all three. */
static void test_a_bird_sent_to_write_forgets_the_alarm_it_was_given(void) {
    sign_sky_t world;
    lay_out_a_sign_on(&world, 160, 45, 25, "HI", 400, 5);
    int writer = -1, free_bird = -1;
    for (int i = 0; i < config.birds; i++) {
        if (formation.slot[i] >= 0 && writer < 0) writer = i;
        if (formation.slot[i] < 0 && world.birds[i].layer == 0 && free_bird < 0) free_bird = i;
    }
    assert(writer >= 0 && free_bird >= 0);
    reset_the_waves();
    waves[writer] = (wave_t){.wait = 0.001, .swerve = 0.9};
    waves[free_bird] = (wave_t){.wait = 0.001, .swerve = 0.9};
    waves_in_flight = 1; /* Something is going on, which is what makes it look. */
    step_the_world(&world, 0.04);
    assert(waves[free_bird].left > 0 || waves[free_bird].rest > 0);
    assert(world.birds[free_bird].alarmed);
    assert(waves[writer].wait == 0 && waves[writer].left == 0 && waves[writer].rest == 0);
    assert(!world.birds[writer].alarmed);
    close_the_world(&world);
    reset_sign_state();
}

/* Hawks over a sign for twenty seconds: the free birds have their waves, and no
 * writer is ever lit or caught. */
static void test_hawks_over_a_sign_light_the_free_birds_and_not_the_writers(void) {
    sign_sky_t world;
    lay_out_a_sign_on(&world, 160, 45, 50, "HELLO WORLD", 800, 5);
    config.hawks = 2;
    place_hawks();
    reset_the_waves();
    int lit_free = 0, lit_scattered = 0, writing_frames = 0;
    for (double at = 0.02; at < 20; at += 0.02) {
        step_the_world(&world, at);
        for (int i = 0; i < config.birds; i++) {
            double x, y;
            /* As the step saw it: a bird that comes home in this step is a writer only
             * in the next, which forgets what it was told. */
            int writing = formation_target_for(&world.snapshot[i], i, &x, &y);
            if (writing) {
                writing_frames++;
                assert(!world.birds[i].alarmed);
                assert(waves[i].wait == 0);
            } else if (world.birds[i].alarmed) {
                lit_free++;
                if (formation.slot[i] >= 0) lit_scattered++;
            }
        }
    }
    assert(writing_frames > 0);
    assert(lit_free > 0);      /* The flock round the sign has its waves... */
    assert(lit_scattered > 0); /* ...and so do the letters a hawk has scattered. */
    assert(the_sign.up);
    config.hawks = 0;
    close_the_world(&world);
    reset_the_waves();
    reset_sign_state();
}

/* A lock screen reads its keys from where the keys come from: the descriptor that
 * was chosen, which is the terminal itself when standard input is a pipe, and not
 * standard input. Here standard input is at its end and the keys are on the other. */
static void test_a_screensaver_reads_the_descriptor_that_was_chosen(void) {
    int keys[2], silent[2];
    assert(pipe(keys) == 0 && pipe(silent) == 0);
    /* As the terminal is in raw mode: a read with nothing there comes back. */
    assert(fcntl(keys[0], F_SETFL, O_NONBLOCK) == 0);
    close(silent[1]); /* Standard input is at its end: nobody is there. */
    int saved_stdin = dup(STDIN_FILENO);
    assert(saved_stdin >= 0 && dup2(silent[0], STDIN_FILENO) == STDIN_FILENO);
    close(silent[0]);
    int saved = input_fd;
    input_fd = keys[0];

    reset_sign_state();
    apply_screen_size(80, 24, 640, 384);
    screensaver_mode = 1;
    clock_state.seconds = SCREENSAVER_GRACE + 1;
    assert(handle_input() == 1); /* Nothing on either: carry on. */
    assert(write(keys[1], "x", 1) == 1);
    assert(handle_input() == 0); /* A key where the keys come from. */
    /* The first half second is whatever started it, there as well. */
    clock_state.seconds = SCREENSAVER_GRACE - 0.1;
    assert(write(keys[1], "\033[<35;10;5M", 10) == 10);
    assert(handle_input() == 1);

    input_fd = saved;
    assert(dup2(saved_stdin, STDIN_FILENO) == STDIN_FILENO);
    close(saved_stdin);
    close(keys[0]);
    close(keys[1]);
    reset_sign_state();
}

/* The whole program, on a terminal of its own, as a flock, a night, a clock, a
 * sign, a text from a file and a text on a pipe, each of them a lock screen: it
 * runs until a key is typed after the grace, and then goes at once with the
 * status of a run that went well, having given the terminal back. */
static void test_a_screensaver_is_a_lock_screen_for_every_mode(void) {
    char text[600];
    scratch_file(text, sizeof(text), "saver.txt");
    world_write(text, "hello, world\nsecond line\n");
    const char *runs[][4] = {
        {NULL, NULL, NULL, NULL},
        {"--fireflies", NULL, NULL, NULL},
        {"--clock", NULL, NULL, NULL},
        {"--say", "hi", NULL, NULL},
        {"--text", text, NULL, NULL},
        {"--hawks", "2", NULL, NULL},
        {NULL, NULL, NULL, NULL} /* The text of this one is on a pipe. */,
    };
    int count = (int)(sizeof(runs) / sizeof(*runs));
    for (int which = 0; which < count; which++) {
        int piped = which == count - 1;
        int master = posix_openpt(O_RDWR | O_NOCTTY);
        assert(master >= 0 && grantpt(master) == 0 && unlockpt(master) == 0);
        const char *name = ptsname(master);
        assert(name != NULL);
        int text_pipe[2];
        assert(pipe(text_pipe) == 0);
        const char *words = "hello, world\nsecond line\n";
        assert(write(text_pipe[1], words, strlen(words)) == (ssize_t)strlen(words));
        close(text_pipe[1]);
        fflush(NULL);
        pid_t child = fork();
        assert(child >= 0);
        if (child == 0) {
            alarm(60);
            close(master);
            /* The terminal is the controlling one, so that /dev/tty is it. */
            if (setsid() < 0) _exit(90);
            int terminal = open(name, O_RDWR);
            if (terminal < 0 || ioctl(terminal, TIOCSCTTY, 0) < 0) _exit(91);
            int quiet = open("/dev/null", O_WRONLY);
            if (quiet < 0 || dup2(piped ? text_pipe[0] : terminal, STDIN_FILENO) < 0 ||
                dup2(terminal, STDOUT_FILENO) < 0 || dup2(quiet, STDERR_FILENO) < 0)
                _exit(99);
            terminal_is_raw = terminal_restored = alt_screen_is_on = sprites_uploaded = 0;
            input_fd = STDIN_FILENO;
            char *argv[12] = {"cbirds", "--screensaver", "--color", "ember", "--seed", "3"};
            int argc = 6;
            for (int word = 0; word < 4 && runs[which][word] != NULL; word++)
                argv[argc++] = (char *)runs[which][word];
            argv[argc] = NULL;
            /* Not _exit: the terminal is given back by the exit handler. */
            exit(cbirds_application_main(argc, argv));
        }
        close(text_pipe[0]);
        /* Once the program has taken the screen, and the grace is over, a key. */
        int status = 0, typed = 0;
        size_t seen = 0;
        char drain[4096], screen_taken[] = ALT_SCREEN_ON, tail[256] = "";
        struct timespec taken_at = {0, 0}, now;
        for (;;) {
            if (readable_within(master, 50) > 0) {
                ssize_t got = read(master, drain, sizeof(drain));
                if (got <= 0) break;
                for (ssize_t i = 0; i < got; i++) {
                    if (!typed) {
                        seen = drain[i] == screen_taken[seen] ? seen + 1
                                                              : (drain[i] == '\033' ? 1 : 0);
                        if (seen == sizeof(screen_taken) - 1) {
                            clock_gettime(CLOCK_MONOTONIC, &taken_at);
                            typed = -1;
                        }
                    }
                    memmove(tail, tail + 1, sizeof(tail) - 2);
                    tail[sizeof(tail) - 2] = drain[i];
                }
            }
            if (typed == -1) {
                clock_gettime(CLOCK_MONOTONIC, &now);
                if (elapsed_seconds(&taken_at, &now) > SCREENSAVER_GRACE + 0.5) {
                    assert(write(master, "x", 1) == 1);
                    typed = 1;
                }
            }
            if (waitpid(child, &status, WNOHANG) == child) {
                child = -1;
                break;
            }
        }
        /* What it had still to say when it went. */
        while (readable_within(master, 100) > 0) {
            ssize_t got = read(master, drain, sizeof(drain));
            if (got <= 0) break;
            for (ssize_t i = 0; i < got; i++) {
                memmove(tail, tail + 1, sizeof(tail) - 2);
                tail[sizeof(tail) - 2] = drain[i];
            }
        }
        assert(typed == 1);
        if (child > 0) assert(waitpid(child, &status, 0) == child);
        assert(WIFEXITED(status) && WEXITSTATUS(status) == EXIT_SUCCESS);
        assert(strstr(tail, ALT_SCREEN_OFF) != NULL); /* The terminal is given back. */
        close(master);
    }
    assert(unlink(text) == 0);
    reset_sign_state();
}

/* Every track gave a bird a field of its own: the letters one that is home and not
 * flying, the waves one that is lit, the signs one that is scattered. A flock that
 * grows or shrinks keeps each bird exactly as it was, those fields included, and
 * every bird it adds starts from nothing, in every one of them. */
static void test_resizing_the_flock_keeps_every_bird_and_starts_every_new_one_clean(void) {
    reset_sign_state();
    apply_screen_size(120, 34, 120 * 8, 34 * 16);
    config.bird_size = DEFAULT_BIRD_SIZE;
    seed_random(11);
    enum { FROM = 100, MORE = 160, LESS = 60 };
    bird_t *birds = malloc(sizeof(*birds) * FROM), *snapshot = malloc(sizeof(*snapshot) * FROM);
    assert(birds != NULL && snapshot != NULL);
    bird_t kept[MORE];
    for (int i = 0; i < FROM; i++) {
        /* Every byte of it set: nothing is left to look like a zero by luck. */
        memset(&birds[i], 0x5a + i % 7, sizeof(birds[i]));
        birds[i].perched = 1;
        birds[i].alarmed = 1;
        birds[i].scattered = 0.75;
        kept[i] = birds[i];
        waves[i] = (wave_t){0.5, 0.25, 0.125, 0.0625, 3.0};
    }
    wave_t wave_kept[MORE];
    memcpy(wave_kept, waves, sizeof(*waves) * FROM);
    /* What a bigger flock left behind in the places the new birds will have. */
    for (int i = FROM; i < MORE; i++) waves[i] = (wave_t){1, 1, 1, 1, 1};

    /* Grown. The old birds are untouched, bit for bit, and so is the wave state they
     * carry; the new ones have nothing lit, nothing scattered, nothing perched and no
     * tail, nothing told, and a place in the sky. */
    assert(resize_the_flock(&birds, &snapshot, FROM, MORE) == 1);
    for (int i = 0; i < FROM; i++) {
        assert(memcmp(&birds[i], &kept[i], sizeof(birds[i])) == 0);
        assert(memcmp(&waves[i], &wave_kept[i], sizeof(waves[i])) == 0);
    }
    config.birds = MORE;
    for (int i = FROM; i < MORE; i++) {
        assert(birds[i].perched == 0 && birds[i].alarmed == 0 && birds[i].scattered == 0);
        assert(birds[i].gliding == 0 && birds[i].trail_at == 0 && birds[i].trail_held == 0);
        assert(birds[i].layer == 0 && birds[i].flock == 0);
        assert(birds[i].direction > 0 && birds[i].direction < 2 * M_PI); /* Placed. */
        assert(wave_busy(&waves[i]) == 0);
        assert(birds[i].x >= 0 && birds[i].x <= screen.width);
        assert(birds[i].y >= 0 && birds[i].y <= screen.height);
        for (int t = 0; t < TRAIL_LENGTH; t++)
            assert(birds[i].trail_x[t] == 0 && birds[i].trail_y[t] == 0);
    }

    /* Shrunk. The ones that stay are untouched. */
    assert(resize_the_flock(&birds, &snapshot, MORE, LESS) == 1);
    for (int i = 0; i < LESS; i++)
        assert(memcmp(&birds[i], i < FROM ? &kept[i] : &birds[i], sizeof(birds[i])) == 0);

    free(birds);
    free(snapshot);

    /* A sign that was up when the flock changed is laid out again for what there is,
     * before any bird of it is read: every writer is a bird that is there, aimed at
     * a place that is there, and the birds that came or went change nothing else. */
    sign_sky_t world;
    lay_out_a_sign_on(&world, 120, 34, 25, "HELLO", 600, 5);
    for (int change = 0; change < 6; change++) {
        int from = config.birds;
        int to = change % 2 == 0 ? from - from / 5 - 1 : from + from / 4 + 1;
        bird_t *grown = malloc(sizeof(*grown) * (size_t)to);
        bird_t *grown_snapshot = malloc(sizeof(*grown_snapshot) * (size_t)to);
        assert(grown != NULL && grown_snapshot != NULL);
        memcpy(grown, world.birds, sizeof(*grown) * (size_t)(from < to ? from : to));
        for (int i = from; i < to; i++) place_one_bird(&grown[i], i);
        free(world.birds);
        free(world.snapshot);
        world.birds = grown;
        world.snapshot = grown_snapshot;
        config.birds = to;
        assert(spatial_grid_prepare(&world.grid, screen.width, screen.height, to) ==
               SPATIAL_GRID_OK);
        step_the_world(&world, (change + 1) * 0.04);
        assert(the_sign.up && sign_layout_is_current());
        int writers = 0;
        for (int i = 0; i < config.birds; i++) {
            if (formation.slot[i] < 0) continue;
            assert(formation.slot[i] < formation.count);
            writers++;
        }
        assert(writers == the_sign.per_cell * formation.count);
    }
    close_the_world(&world);
    reset_the_waves();
    reset_sign_state();
}

/* The pointer whipped through a sign: the letters it reaches scatter, and the ones
 * that have scattered are birds of the flock, so the wave the whip starts lights
 * them. The writers it has not reached are not lit. */
static void test_a_whipped_pointer_scatters_a_sign_and_lights_the_letters_it_scattered(void) {
    sign_sky_t world;
    lay_out_a_sign_on(&world, 160, 45, 50, "HELLO WORLD", 800, 5);
    reset_the_waves();
    int lit_scattered = 0, scattered = 0;
    double across = formation.x[formation.count - 1] - formation.x[0];
    for (int frame = 1; frame <= 60; frame++) {
        double at = frame * 0.02;
        /* Across the middle of the text, two hundred cells a second. */
        mouse.present = 1;
        mouse.x = formation.x[0] + across * frame / 60.0;
        mouse.y = (formation.y[0] + formation.y[formation.count - 1]) / 2;
        mouse.velocity_x = 200.0 * screen.cell_width;
        mouse.velocity_y = 0;
        mouse.moved_at = at;
        clock_state.seconds = at;
        assert(pointer_startles());
        step_the_world(&world, at);
        for (int i = 0; i < config.birds; i++) {
            if (formation.slot[i] < 0) continue;
            double x, y;
            int writing = formation_target_for(&world.snapshot[i], i, &x, &y);
            if (writing) {
                assert(!world.birds[i].alarmed); /* A writer that is home is never lit. */
            } else {
                scattered++;
                lit_scattered += world.birds[i].alarmed;
            }
        }
    }
    assert(scattered > 0);     /* The whip reached the letters... */
    assert(lit_scattered > 0); /* ...and the ones it scattered lit with the wave. */
    mouse.present = 0;
    mouse.velocity_x = mouse.velocity_y = 0;
    close_the_world(&world);
    reset_the_waves();
    reset_sign_state();
}

/* A GIF's palette is made from its first frame, and the light of a wave is on none.
 * The intro's letters are never lit, so a short clip with hawks does without it; a
 * sign is a flock with letters in it, and the birds round it have their waves from
 * the first frame, however short the clip. */
static int palette_of_a_sign_recording_has(const uint8_t colour[3], int hawks, int seconds) {
    char path[600];
    scratch_file(path, sizeof(path), "sign_light.gif");
    reset_sign_state();
    config.palette = palette_named("ice");
    config.birds = 200;
    config.hawks = hawks;
    ask_for_a_sign("HI");
    record_path = path;
    record_fps = 25;
    record_seconds = seconds;
    record_columns = 96;
    record_rows = 32;
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
    FILE *file = fopen(path, "rb");
    assert(file != NULL);
    uint8_t header[13 + 768];
    assert(fread(header, 1, sizeof(header), file) == sizeof(header));
    fclose(file);
    remove(path);
    record_path = NULL;
    record_seconds = 6;
    int found = 0;
    for (int entry = 0; entry < 256; entry++)
        if (memcmp(header + 13 + entry * 3, colour, 3) == 0) found = 1;
    reset_sign_state();
    return found;
}

static void test_a_sign_recording_with_hawks_has_the_light_in_its_palette(void) {
    reset_sign_state();
    config.palette = palette_named("ice");
    uint8_t light[3];
    memcpy(light, highlight_colour(), 3);
    assert(palette_of_a_sign_recording_has(light, 2, 2)); /* A wave can happen at once. */
    assert(palette_of_a_sign_recording_has(light, 1, 5));
    assert(!palette_of_a_sign_recording_has(light, 0, 5)); /* No hawks, so none can. */
    reset_sign_state();
}

int main(void) {
    make_scratch();
    trig_lookup_init();
    /* First, while every global is as a fresh process has it. */
    test_a_closed_pipe_leaves_the_terminal_as_it_was();
    test_the_trig_lookup_covers_the_circle();
    test_the_frame_rate_can_be_unlocked();
    test_engine_matches_brute_force();
    test_boundary_bands_follow_the_viewport();
    test_the_edge_pushes_harder_the_further_out_a_bird_is();
    test_bottom_band_scales_on_a_short_viewport();
    test_birds_start_spread_inside_the_free_region();
    test_a_grown_flock_starts_its_new_birds_clean();
    test_the_recording_rate_is_one_a_gif_has();
    test_birds_bank_rather_than_snap();
    test_the_konami_code();
    test_only_the_rain_has_a_wind();
    test_autopilot_wanders_and_yields();
    test_hawks_hunt_and_the_flock_flees();
    test_motion_follows_elapsed_time();
    test_more_flocks_are_more_colours();
    test_the_flock_can_be_laid_out_as_text();
    test_the_intro_is_untouched_by_signs();
    test_a_key_ends_the_intro_and_leaves_a_sign_up();
    test_a_text_is_laid_out_as_a_sign_in_lines();
    test_a_sign_that_does_not_fit_says_so_and_the_flock_flies();
    test_a_hovering_bird_stays_within_its_loop_and_does_not_stand_still();
    test_a_sign_draws_no_random_numbers_and_the_flock_keeps_its_own();
    test_the_sign_comes_back_after_its_flight();
    test_a_pause_holds_a_sign_too();
    test_the_clock_tells_the_time_and_changes_it_a_letter_at_a_time();
    test_a_new_minute_lets_go_of_the_letters_that_changed_and_of_nothing_else();
    test_the_hour_lets_go_of_the_whole_clock();
    test_a_twelve_hour_clock_changes_a_letter_at_a_time_too();
    test_every_lit_cell_of_a_clock_keeps_its_writers_through_an_hour();
    test_a_sign_is_coloured_along_its_text();
    test_the_letters_of_a_clock_are_a_shade_each_and_keep_it();
    test_the_clock_tells_local_time();
    test_the_colon_lifts_with_the_seconds();
    test_the_pointer_scatters_a_sign_and_it_comes_back();
    test_a_hawk_over_a_sign_scatters_the_places_it_is_over();
    test_the_intro_birds_are_never_scattered();
    test_a_screensaver_quits_at_the_first_sign_of_anybody();
    test_the_options_that_make_a_sign();
    test_a_picture_gives_every_bird_a_place_and_a_colour();
    test_a_picture_wears_a_ramp_somebody_chose();
    test_a_picture_in_black_is_not_a_picture_of_nothing();
    test_a_large_picture_is_kept_small();
    test_a_picture_that_cannot_be_drawn_says_so();
    test_a_colour_given_is_told_from_the_default();
    test_a_small_screen_leaves_the_flock_sky_and_a_roomy_one_is_as_it_was();
    test_a_sign_that_cannot_be_laid_out_says_so_when_the_run_is_over();
    test_a_text_too_big_for_the_terminal_is_said_after_the_terminal_is_given_back();
    test_a_sign_has_a_bird_as_wide_as_its_cells_unless_it_is_told();
    test_a_sign_records_in_a_gif_and_a_cast();
    test_a_sign_survives_the_flock_growing_under_it();
    test_presets_set_every_notch();
    test_a_notch_survives_the_round_trip();
    test_the_pointer_moves_the_flock();
    test_the_shade_follows_the_heading();
    test_the_hawk_is_never_the_colour_of_the_flock();
    test_the_help_names_every_ramp();
    test_no_ramp_fades_into_a_black_terminal();
    test_the_theme_ramp_never_reaches_the_background();
    test_theme_colours_are_parsed();
    test_each_flock_flies_at_its_own_pace();
    test_the_matrix_is_the_only_thing_that_rains();
    test_the_sprite_catalogue_has_a_place_for_everything();
    test_the_far_layer_is_another_sky();
    test_a_seed_draws_the_same_numbers_everywhere();
    test_wings_beat_and_sometimes_glide();
    test_braille_unless_asked();
    test_a_text_terminal_gets_the_flock_in_braille();
    test_a_text_renderer_records_its_cells();
    test_a_cast_is_the_flock_as_text();
    test_recording_gives_the_whole_frame_to_the_flock();
    test_flocks_keep_to_their_own_side_of_the_sky();
    test_flocks_do_not_align_with_each_other();
    test_a_key_ends_the_intro();
    test_mouse_reports_are_parsed();
    test_vision_controls();
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
    test_the_speed_slider_flies_the_same_path_faster();
    test_the_default_size_follows_the_renderer();
    test_the_speed_is_a_flag();
    test_a_hawk_holds_a_chase_for_a_distance();
    test_a_fast_flock_is_flown_in_steps();
    test_the_avoidance_slider_needs_two_flocks();
    test_the_avoidance_is_a_flag();
    test_flocks_avoid_each_other_as_much_as_asked();
    test_the_flat_flock_is_what_it_was();
    test_three_d_has_defaults_of_its_own();
    test_three_d_says_what_it_replaces();
    test_ink_runs_from_the_foreground_towards_the_background();
    test_ink_does_not_fade_into_a_black_terminal();
    test_ink_is_drawn_as_it_was_built();
    test_a_flock_of_ink_is_seen_on_a_white_ground();
    test_the_light_of_a_wave_is_seen_on_the_ground_ink_knows();
    test_ink_is_asked_of_the_terminal();
    test_ink_without_a_terminal_is_ash();
    test_the_birds_are_rebuilt_when_the_window_settles();
    test_a_space_has_a_picture_for_every_size_and_shape();
    test_a_birds_shape_follows_what_the_camera_sees();
    test_a_space_is_drawn_by_the_renderers_of_the_flat_flock();
    test_the_panel_says_what_its_rows_do_in_a_space();
    test_the_sliders_steer_the_flight_in_a_space();
    test_a_space_grows_and_shrinks_with_the_keys();
    test_a_space_flies_at_the_pace_it_is_told();
    test_hawks_hunt_in_a_space_and_are_drawn_over_it();
    test_the_pointer_pokes_the_flock_in_a_space();
    test_a_space_records_headless();
    test_fireflies_change_the_defaults_and_nothing_else();
    test_a_night_leaves_the_flocks_switches_with_nothing_to_do();
    test_the_swarm_falls_into_step_with_nothing_in_charge();
    test_the_swarm_keeps_time_in_seconds_not_frames();
    test_the_lantern_scatters_the_phases_and_the_swarm_heals();
    test_the_fireflies_drift_slowly_and_keep_to_the_meadow();
    test_a_dark_firefly_is_a_faint_body();
    test_the_panel_says_what_a_night_does();
    test_the_swarm_is_the_same_swarm_on_any_screen();
    test_a_night_records_with_the_whole_ramp_in_its_colour_table();
    test_a_night_benchmarks_and_reports_its_sync();
    test_a_far_swarm_flashes_to_itself();
    test_a_night_is_left_alone_and_flies_off_still_flashing();
    test_the_sprites_of_a_night_are_placed_lit_or_as_bodies();
    test_every_slider_does_what_it_says_to_a_night();
    test_a_wave_crosses_a_line_of_birds_faster_than_they_fly();
    test_the_wave_covers_the_same_ground_at_thirty_and_sixty_frames_a_second();
    test_a_wave_crosses_a_flock_at_the_same_moments_at_any_frame_rate();
    test_a_bird_alarmed_by_a_neighbour_copies_its_swerve();
    test_the_refractory_time_holds();
    test_a_dive_alarms_the_birds_in_its_way();
    test_nothing_is_alarmed_without_hawks();
    test_a_strike_sends_a_wave_across_the_flock();
    test_a_swerve_is_sharper_than_the_banking();
    test_far_birds_are_never_alarmed();
    test_the_letters_are_never_alarmed();
    test_a_wave_stays_in_its_flock_unless_the_flocks_are_kin();
    test_a_pointer_whipped_through_the_flock_starts_a_wave();
    test_a_recording_with_hawks_has_the_light_in_its_palette();
    test_a_recording_of_a_space_asks_for_its_sprites_and_not_for_a_wave();
    test_the_light_of_a_wave_stands_clear_of_everything();
    test_a_bird_in_a_wave_is_lit_in_every_renderer();
    test_a_night_tells_no_alarm();
    test_text_piped_in_becomes_the_flock();
    test_a_flock_of_letters_is_slower_than_a_flock_of_birds_unless_asked();
    test_nothing_to_see_leaves_the_flock_as_it_was();
    test_the_text_at_rest_is_the_text();
    test_a_whole_cycle_puts_every_letter_back_on_its_own_cell();
    test_enter_brings_them_home_in_four_seconds_whatever_they_were_doing();
    test_a_straggler_is_home_by_the_deadline_however_far_it_is();
    test_a_letter_lands_exactly_and_does_not_circle_its_cell();
    test_the_pointer_scatters_what_it_touches_and_they_find_their_way_back();
    test_a_hawk_over_the_text_is_an_arrow_and_scatters_it();
    test_the_panel_lies_over_the_text_and_the_letters_under_it_still_land();
    test_pausing_stops_the_cycle_and_the_text_is_not_drawn_twice();
    test_the_keys_that_change_the_population_do_nothing_to_text();
    test_quitting_flies_the_text_off_the_top();
    test_a_letter_at_home_is_neither_moved_nor_seen_by_the_flock();
    test_how_the_flock_of_letters_looks_in_the_air();
    test_a_terminal_that_is_sent_everything_shows_the_text_at_every_frame_and_after_a_cycle();
    test_a_text_recording_is_the_text_flying();
    test_a_text_recording_as_a_gif_is_painted_with_the_font();
    test_a_window_that_changes_size_lays_the_text_out_again();
    test_text_that_does_not_fit_the_screen_it_is_laid_out_on_scrolls();
    test_a_benchmark_of_text_flies_it_from_the_first_frame();
    test_standard_input_that_is_not_a_terminal_leaves_the_keys_to_the_tty();
    test_keys_and_colour_questions_use_the_descriptor_that_was_chosen();
    test_a_text_tells_no_alarm_and_never_reads_the_wave_state();
    test_a_text_with_hawks_records_the_flight_and_not_the_light();
    test_an_unasked_for_recording_length_is_settled_before_anything_reads_it();
    test_the_text_is_the_flock_and_a_night_is_refused_with_it();
    test_a_whip_is_read_from_the_descriptor_that_was_chosen();
    test_what_was_not_asked_for_is_settled_together();
    test_a_sign_is_refused_with_a_night_and_with_text();
    test_a_space_is_refused_with_a_night_a_text_and_a_sign();
    test_a_writer_is_never_alarmed_and_a_scattered_one_is();
    test_a_bird_sent_to_write_forgets_the_alarm_it_was_given();
    test_hawks_over_a_sign_light_the_free_birds_and_not_the_writers();
    test_a_screensaver_reads_the_descriptor_that_was_chosen();
    test_a_screensaver_is_a_lock_screen_for_every_mode();
    test_resizing_the_flock_keeps_every_bird_and_starts_every_new_one_clean();
    test_a_whipped_pointer_scatters_a_sign_and_lights_the_letters_it_scattered();
    test_a_sign_recording_with_hawks_has_the_light_in_its_palette();
    /* Every test removes what it wrote, so this fails if one did not. */
    assert(rmdir(scratch) == 0);
    return 0;
}
