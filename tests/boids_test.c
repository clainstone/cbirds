#define main cbirds_application_main
#include "../boids.c"
#undef main

#include <assert.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <termios.h>

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
    clock_seconds = 0;
    sign_font_rows = 0;
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
    input_state = INPUT_NORMAL;
    input_escape_at.tv_sec = -1;
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
    assert(most_x - least_x <= screen.width * SIGN_WIDTH_MOST);
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

/* A clock with the seconds, at 200 by 50 cells with a flock of 800. */
static void open_a_seconds_clock_at(sign_sky_t *world, int hour, int minute, int second) {
    open_a_clock_at(world, hour, minute, second, 800);
    the_sign.seconds = 1;
}

/* The writers of a letter, and whether every one of them is in its loop. */
static int the_glyph_of(int bird) {
    return formation.slot[bird] < 0 ? -1 : formation.glyph[formation.slot[bird]];
}

static int writers_of_glyph_home(const sign_sky_t *world, int glyph) {
    for (int i = 0; i < config.birds; i++)
        if (the_glyph_of(i) == glyph && home_distance(world, i) > formation.hover + 1e-6) return 0;
    return 1;
}

/* With the seconds the clock changes every second, and every second it is a digit
 * that changes and not the clock: the birds of a digit of the seconds move over to
 * the next one and are home within half a second, the minutes are written by other
 * birds as they always were, and the rest never moves. A new minute is not the
 * hour, though the seconds end in two zeros then. */
static void test_a_clock_with_seconds_ticks_a_digit_at_a_time(void) {
    sign_sky_t world;
    open_a_seconds_clock_at(&world, 10, 9, 55);
    fly_the_world(&world, 0, 4.9, 60);
    assert(the_sign.up && strcmp(the_sign.written, "10:09:59") == 0);
    assert(formation.count == font_text_cells("10:09:59"));
    assert_every_cell_has_its_writers(&world);
    int per_cell = the_sign.per_cell;
    double cell = formation.cell;

    /* "10:09:59": the hour and its colon are 0 to 2, the minutes 3 and 4, the
     * second colon 5, and the seconds 6 and 7. */
    static int before[MAX_BIRDS];
    static double place_x[MAX_BIRDS], place_y[MAX_BIRDS];
    int written[8] = {0};
    for (int i = 0; i < config.birds; i++) {
        before[i] = the_glyph_of(i);
        if (before[i] < 0) continue;
        written[before[i]]++;
        place_x[i] = formation.x[formation.slot[i]];
        place_y[i] = formation.y[formation.slot[i]];
    }

    /* 10:10:00: the minutes and both seconds change, and nothing else. */
    step_the_world(&world, 5.0);
    assert(the_sign.up && formation.writing && strcmp(the_sign.written, "10:10:00") == 0);
    assert(the_sign.per_cell == per_cell && formation.cell == cell);
    int now_cells[8] = {0}, from_own[8] = {0};
    for (int t = 0; t < formation.count; t++) now_cells[formation.glyph[t]]++;
    for (int i = 0; i < config.birds; i++) {
        int g = the_glyph_of(i);
        if (before[i] == 0 || before[i] == 1 || before[i] == 2 || before[i] == 5) {
            /* Kept: the same letter, the same place. */
            assert(g == before[i]);
            assert(formation.x[formation.slot[i]] == place_x[i] &&
                   formation.y[formation.slot[i]] == place_y[i]);
        }
        if (before[i] == 3 || before[i] == 4) assert(g < 0); /* The minutes: let go. */
        if (g == 3 || g == 4) assert(before[i] < 0);         /* Written by others. */
        if (g >= 6 && before[i] == g) from_own[g]++;
    }
    for (int g = 6; g <= 7; g++) {
        int wanted = now_cells[g] * per_cell;
        assert(from_own[g] == (written[g] < wanted ? written[g] : wanted));
    }
    /* Half a second on, the seconds are written whole, by birds in their loops. */
    fly_the_world(&world, 5.0 + 1.0 / 60, 5.5, 60);
    assert(writers_of_glyph_home(&world, 6) && writers_of_glyph_home(&world, 7));

    /* 10:10:01: the last digit, and all of its birds were the zero's. */
    fly_the_world(&world, 5.5, 5.99, 60);
    for (int i = 0; i < config.birds; i++) before[i] = the_glyph_of(i);
    step_the_world(&world, 6.0);
    assert(strcmp(the_sign.written, "10:10:01") == 0);
    for (int i = 0; i < config.birds; i++) {
        int g = the_glyph_of(i);
        if (before[i] >= 0 && before[i] < 7) assert(g == before[i]);
        if (g == 7) assert(before[i] == 7);
    }
    fly_the_world(&world, 6.0 + 1.0 / 60, 6.5, 60);
    assert_every_cell_has_its_writers(&world);
    assert(the_sign.per_cell == per_cell && formation.cell == cell);
    close_the_world(&world);
    reset_sign_state();
}

/* The hour lets go of the whole clock with the seconds as without them, and the
 * clock comes back on the second it has got to. */
static void test_the_hour_lets_go_of_a_clock_with_seconds(void) {
    sign_sky_t world;
    open_a_seconds_clock_at(&world, 10, 59, 57);
    fly_the_world(&world, 0, 2.9, 60);
    assert(the_sign.up && strcmp(the_sign.written, "10:59:59") == 0);
    step_the_world(&world, 3.0);
    assert(!the_sign.up && !formation.writing);
    fly_the_world(&world, 3.0 + 1.0 / 60, 3.0 + SIGN_CLOCK_FLIGHT - 0.1, 60);
    assert(!the_sign.up);
    fly_the_world(&world, 3.0 + SIGN_CLOCK_FLIGHT - 0.1, 3.0 + SIGN_CLOCK_FLIGHT + 1.5, 60);
    assert(the_sign.up && strncmp(the_sign.written, "11:00:0", 7) == 0);
    close_the_world(&world);
    reset_sign_state();
}

/* HH:MM:SS is wider than HH:MM, and fits the screens a clock does, in cells that
 * are letters: from 96 by 26 up it takes the most a sign may take to keep eight
 * pixels a cell, and at 80 by 24 it has all of that, which is seven and a half.
 * And a twelve hour clock with one digit of hour has the cell of one with two. */
static void test_a_clock_with_seconds_fits_the_screens_a_clock_does(void) {
    int sizes[][2] = {{80, 24}, {96, 26}, {120, 34}, {200, 50}};
    for (int s = 0; s < 4; s++) {
        double cells[2];
        for (int twelve = 0; twelve < 2; twelve++) {
            sign_sky_t world;
            reset_sign_state();
            config.pace_notch = DEFAULT_PACE_NOTCH;
            apply_notches();
            apply_screen_size(sizes[s][0], sizes[s][1], sizes[s][0] * 8, sizes[s][1] * 16);
            config.bird_size = 0;
            settle_the_bird_size();
            the_sign.kind = SIGN_CLOCK;
            the_sign.virtual_clock = 1;
            the_sign.seconds = 1;
            the_sign.twelve_hours = twelve;
            the_sign.origin = local_time(13, 9, 5);
            begin_the_intro();
            open_the_world(&world, 800, 6);
            sign_advance(world.birds);
            assert(the_sign.up);
            assert(strcmp(the_sign.written, twelve ? "1:09:05" : "13:09:05") == 0);
            /* Eight pixels a cell, or what the most a sign may take gives if that is
             * less, which it is only on the smallest of these. */
            double pad = config.bird_size * 2.0, most_cell;
            sign_lines_t lines;
            assert(sign_fit("00:00:00", 0, (screen.width - 2 * pad) * SIGN_WIDTH_MOST,
                            (screen.height - 2 * pad) * SIGN_HEIGHT_MOST,
                            SIGN_LARGEST_CELL * config.bird_size, &lines, &most_cell) == 1);
            double floor_cell = most_cell < SIGN_SMALLEST_CELL ? most_cell : SIGN_SMALLEST_CELL;
            assert(formation.cell >= floor_cell - 1e-9);
            if (sizes[s][0] >= 96) assert(formation.cell >= SIGN_SMALLEST_CELL - 1e-9);
            for (int t = 0; t < formation.count; t++)
                assert(formation.x[t] > 0 && formation.x[t] < screen.width && formation.y[t] > 0 &&
                       formation.y[t] < screen.height);
            cells[twelve] = formation.cell;
            close_the_world(&world);
        }
        assert(fabs(cells[0] - cells[1]) < 1e-9);
    }
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
    /* As large as letters get, so that the pointer's reach is a part of the word. */
    sign_font_rows = SIGN_FONT_ROWS_MAX;
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
    /* But the first half second of the program, not of its first frame: a start that
     * took longer than that has no key in it that started anything, and the one
     * typed meanwhile is somebody waking the screen. A quick start is as it was. */
    clock_state.seconds = 0;
    launch_lag = SCREENSAVER_GRACE / 2;
    assert(feed_input("x") == 1);
    launch_lag = SCREENSAVER_GRACE + 1.5;
    assert(feed_input("x") == 0);
    launch_lag = 0;

    /* After it, a key, a click, a pointer that moves, an arrow: any of them. */
    clock_state.seconds = SCREENSAVER_GRACE + 0.01;
    assert(feed_input("x") == 0);
    assert(feed_input("\r") == 0);
    assert(feed_input("\033[<0;10;5M") == 0);  /* A click. */
    assert(feed_input("\033[<35;11;5M") == 0); /* Just moving. */
    assert(feed_input("\033[A") == 0);
    assert(feed_input("\033OA") == 0); /* An arrow, as an application mode sends it. */
    assert(feed_input("\033x") == 0);  /* Alt with a key. */
    /* The Escape key is an escape and nothing after it: it goes when nothing has
     * come after it for a moment. */
    assert(feed_input("\033") == 1);
    assert(feed_input("") == 1);
    input_escape_at.tv_sec -= 5;
    assert(feed_input("") == 0);
    clock_state.seconds = 3600;
    assert(feed_input("z") == 0);
    /* Nothing at all is not a reason to leave. */
    assert(feed_input("") == 1);
    screensaver_mode = 0;
    reset_sign_state();
}

/* A lock screen that asks its terminal for colours is answered with strings, and
 * a late answer is read after the grace. It is the terminal and not somebody: the
 * strings are swallowed whole, and what is left is the key that wakes it. */
static void test_a_screensaver_does_not_quit_for_a_late_reply_and_does_for_a_key(void) {
    reset_sign_state();
    apply_screen_size(80, 24, 640, 384);
    screensaver_mode = 1;
    clock_state.seconds = SCREENSAVER_GRACE + 2;
    static const char *const REPLIES[] = {
        "\033]11;rgb:bbbb/bbbb/bbbb\033\\", /* ST. */
        "\033]10;rgb:eeee/aaaa/0000\007",   /* A bell. */
        "\033]4;3;rgb:cccc/0000/0000\033\\",
        "\033P>|terminal 1.2\007still\033\\", /* A DCS: only ST ends it. */
    };
    for (size_t i = 0; i < sizeof(REPLIES) / sizeof(*REPLIES); i++) {
        assert(feed_input(REPLIES[i]) == 1 && input_state == INPUT_NORMAL);
        /* The reply is no key, and the key after it, or before it, is. */
        char with_a_key[80];
        snprintf(with_a_key, sizeof(with_a_key), "%sx", REPLIES[i]);
        assert(feed_input(with_a_key) == 0);
        snprintf(with_a_key, sizeof(with_a_key), "x%s", REPLIES[i]);
        assert(feed_input(with_a_key) == 0);
        /* Two of them, the way a terminal answers two questions. */
        snprintf(with_a_key, sizeof(with_a_key), "%s%s", REPLIES[i], REPLIES[i]);
        assert(feed_input(with_a_key) == 1);
    }
    /* In pieces, cut anywhere: between the escape and what follows it, in the
     * body, between the escape that ends it and the backslash. Not one of the
     * pieces is somebody, and the key after the last of them is. */
    static const char REPLY[] = "\033]11;rgb:bbbb/bbbb/bbbb\033\\";
    for (size_t cut = 1; cut < sizeof(REPLY) - 1; cut++) {
        char head[40], tail[40];
        memcpy(head, REPLY, cut);
        head[cut] = '\0';
        strcpy(tail, REPLY + cut);
        assert(feed_input(head) == 1);
        /* The escape that is waited on is waited on for a moment and not for ever. */
        assert(feed_input(tail) == 1);
        assert(input_state == INPUT_NORMAL);
        assert(feed_input("x") == 0);
    }
    /* None of it reaches the flock: b moves the boundary slider, and the reply is
     * full of them. */
    reset_test_config();
    config.boundary_notch = 4;
    apply_notches();
    assert(feed_input("\033]11;rgb:bbbb/bbbb/bbbb\033\\") == 1);
    assert(config.boundary_notch == 4);
    /* A pointer report is somebody all the same, whatever its body says. */
    assert(feed_input("\033[<35;10;5M") == 0);
    assert(config.boundary_notch == 4 && !mouse.present);

    /* A reply that begins in the grace and goes on past it is swallowed whole. */
    clock_state.seconds = SCREENSAVER_GRACE - 0.2;
    assert(feed_input("\033]11;rgb:bb") == 1);
    clock_state.seconds = SCREENSAVER_GRACE + 0.2;
    assert(feed_input("bb/bbbb/bbbb\033\\") == 1 && input_state == INPUT_NORMAL);
    assert(feed_input("x") == 0);

    /* An escape that a reply follows, in time, is the start of it. Nobody waits
     * for ever for the rest of one, as nobody waits for ever for a key. */
    assert(feed_input("\033") == 1);
    assert(feed_input("]11;rgb:bbbb/bbbb/bbbb\007") == 1);
    assert(input_state == INPUT_NORMAL);
    assert(feed_input("\033") == 1);
    input_escape_at.tv_sec -= 5;
    assert(feed_input("]11;rgb:bbbb/bbbb/bbbb\007") == 0);

    /* And a string that never ends is not a lock screen that never wakes. */
    input_state = INPUT_NORMAL;
    assert(feed_input("\033]") == 1 && input_state == INPUT_STRING);
    input_string_at.tv_sec -= 5;
    assert(feed_input("x") == 0);
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

/* --seconds is a clock on its own, as --clock-at is, and goes with either of them;
 * beside another sign it is refused as --clock is. */
static void test_seconds_is_a_clock(void) {
    reset_sign_state();
    char *alone[] = {"cbirds", "--seconds", NULL};
    read_options(2, alone);
    assert(the_sign.kind == SIGN_CLOCK && the_sign.seconds);
    reset_sign_state();
    char *both[] = {"cbirds", "--clock", "--seconds", NULL};
    read_options(3, both);
    assert(the_sign.kind == SIGN_CLOCK && the_sign.seconds);
    reset_sign_state();
    char *from[] = {"cbirds", "--clock-at", "10:09:58", "--seconds", NULL};
    read_options(4, from);
    assert(the_sign.kind == SIGN_CLOCK && the_sign.seconds && the_sign.virtual_clock);
    {
        char text[SIGN_TEXT_MAX];
        clock_state.seconds = 0;
        sign_text_now(text, sizeof(text));
        assert(strcmp(text, "10:09:58") == 0);
        clock_state.seconds = 2.5;
        sign_text_now(text, sizeof(text));
        assert(strcmp(text, "10:10:00") == 0);
    }
    reset_sign_state();
    char *plain[] = {"cbirds", "--clock", NULL};
    read_options(2, plain);
    assert(the_sign.kind == SIGN_CLOCK && !the_sign.seconds);
    reset_sign_state();
    char *said[] = {"cbirds", "--say", "hi", "--seconds", NULL};
    assert(exit_status_of(4, said) == EXIT_USAGE);
    reset_sign_state();
}

/* --font-size is rows, four to ten, and --fontsize is the same option; outside
 * that it is a usage error. */
static void test_the_font_size_is_rows_from_four_to_ten(void) {
    reset_sign_state();
    char *six[] = {"cbirds", "--clock", "--font-size", "6", NULL};
    read_options(4, six);
    assert(sign_font_rows == 6 && the_sign.kind == SIGN_CLOCK);
    reset_sign_state();
    char *joined[] = {"cbirds", "--say", "hi", "--fontsize", "9", NULL};
    read_options(5, joined);
    assert(sign_font_rows == 9 && the_sign.kind == SIGN_SAY);
    reset_sign_state();
    char *edges[][5] = {{"cbirds", "--clock", "--font-size", "4", NULL},
                        {"cbirds", "--clock", "--font-size", "10", NULL}};
    for (int e = 0; e < 2; e++) {
        read_options(4, edges[e]);
        assert(sign_font_rows == (e == 0 ? SIGN_FONT_ROWS_MIN : SIGN_FONT_ROWS_MAX));
        reset_sign_state();
    }
    char *small[] = {"cbirds", "--clock", "--font-size", "3", NULL};
    assert(exit_status_of(4, small) == EXIT_USAGE);
    char *large[] = {"cbirds", "--clock", "--font-size", "11", NULL};
    assert(exit_status_of(4, large) == EXIT_USAGE);
    char *not_a_number[] = {"cbirds", "--clock", "--font-size", "big", NULL};
    assert(exit_status_of(4, not_a_number) == EXIT_USAGE);
    reset_sign_state();
}

/* Not given, the letters are a seventh of the window's rows, four at the least
 * and ten at the most; given, they are what was given whatever the window. */
static void test_the_letters_are_a_seventh_of_the_rows_unless_told(void) {
    int rows[] = {18, 24, 26, 34, 45, 50, 60, 80};
    int wanted[] = {4, 4, 4, 5, 6, 7, 9, 10};
    for (int r = 0; r < 8; r++) {
        reset_sign_state();
        apply_screen_size(rows[r] * 3, rows[r], rows[r] * 3 * 8, rows[r] * 16);
        assert(sign_font_rows_now() == wanted[r]);
        assert(fabs(sign_font_cell() - wanted[r] * 16.0 / FONT_HEIGHT) < 1e-9);
        sign_font_rows = 8;
        assert(sign_font_rows_now() == 8);
    }
    /* In rows, not pixels: a screen of twice the pixels has cells twice as large. */
    reset_sign_state();
    apply_screen_size(200, 50, 3200, 1600);
    assert(fabs(sign_font_cell() - 7 * 32.0 / FONT_HEIGHT) < 1e-9);
    reset_sign_state();
}

/* A sign of the size asked for, if it fits, in the bird that size asks for: a
 * clock at five rows of a 200 by 50 screen is cells of five sevenths of sixteen
 * pixels, and its birds as wide; at ten, larger, and still the asked size. Too
 * large for the room is as large as fits, and never larger than asked. A long text
 * takes more lines at a larger size. */
static void test_a_sign_is_as_tall_as_its_font_where_it_fits(void) {
    for (int font = SIGN_FONT_ROWS_MIN; font <= SIGN_FONT_ROWS_MAX; font += 3) {
        sign_sky_t world;
        reset_sign_state();
        apply_screen_size(200, 50, 1600, 800);
        sign_font_rows = font;
        config.bird_size = 0;
        the_sign.kind = SIGN_CLOCK;
        the_sign.virtual_clock = 1;
        the_sign.origin = local_time(10, 9, 0);
        settle_the_bird_size();
        begin_the_intro();
        open_the_world(&world, 800, 6);
        sign_advance(world.birds);
        assert(the_sign.up && strcmp(the_sign.written, "10:09") == 0);
        double cell = font * 16.0 / FONT_HEIGHT;
        assert(fabs(formation.cell - cell) < 1e-9);
        assert(config.bird_size == (int)(cell + 0.5));
        /* The rows the letters take, from the top of the highest cell to the bottom
         * of the lowest: as many as were asked for. */
        double top = 1e9, bottom = 0;
        for (int t = 0; t < formation.count; t++) {
            if (formation.lift[t] > 0) continue; /* The colon rises with its breath. */
            if (formation.y[t] < top) top = formation.y[t];
            if (formation.y[t] > bottom) bottom = formation.y[t];
        }
        assert(fabs((bottom - top + cell) / 16.0 - font) < 1e-6);
        close_the_world(&world);
    }
    /* HH:MM:SS at ten rows is wider than an 80 column screen leaves it. */
    {
        sign_sky_t world;
        reset_sign_state();
        apply_screen_size(80, 24, 640, 384);
        sign_font_rows = SIGN_FONT_ROWS_MAX;
        config.bird_size = 0;
        the_sign.kind = SIGN_CLOCK;
        the_sign.seconds = 1;
        the_sign.virtual_clock = 1;
        the_sign.origin = local_time(10, 9, 0);
        settle_the_bird_size();
        begin_the_intro();
        open_the_world(&world, 800, 6);
        sign_advance(world.birds);
        assert(the_sign.up && formation.cell < SIGN_FONT_ROWS_MAX * 16.0 / FONT_HEIGHT);
        double most_right = 0, most_left = 1e9;
        for (int t = 0; t < formation.count; t++) {
            if (formation.x[t] > most_right) most_right = formation.x[t];
            if (formation.x[t] < most_left) most_left = formation.x[t];
        }
        assert(most_left > 0 && most_right < 640);
        close_the_world(&world);
    }
    /* A long text: on more lines at ten rows than at four. */
    int lines_at[2];
    for (int f = 0; f < 2; f++) {
        sign_sky_t world;
        reset_sign_state();
        apply_screen_size(200, 50, 1600, 800);
        sign_font_rows = f == 0 ? SIGN_FONT_ROWS_MIN : SIGN_FONT_ROWS_MAX;
        ask_for_a_sign("back in five minutes");
        config.bird_size = 0;
        settle_the_bird_size();
        begin_the_intro();
        open_the_world(&world, 800, 6);
        sign_advance(world.birds);
        assert(the_sign.up);
        double top = 1e9, bottom = 0;
        for (int t = 0; t < formation.count; t++) {
            if (formation.y[t] < top) top = formation.y[t];
            if (formation.y[t] > bottom) bottom = formation.y[t];
        }
        int rows = (int)((bottom - top) / formation.cell + 0.5) + 1;
        lines_at[f] = rows == sign_rows(1) ? 1 : rows == sign_rows(2) ? 2 : 3;
        close_the_world(&world);
    }
    assert(lines_at[0] < lines_at[1]);
    reset_sign_state();
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
    /* --matrix names the green ramp, so a picture beside it is drawn in that; and
     * with no picture, it is what it was. */
    char *rain[] = {"cbirds", "--picture", path, "--matrix", NULL};
    read_options(4, rain);
    assert(palette_was_asked_for && !picture_colours_in_use);
    assert(config.palette == palette_named("matrix") && the_rain_is_falling);
    reset_sign_state();
    the_rain_is_falling = 0;
    char *only_rain[] = {"cbirds", "--matrix", NULL};
    read_options(2, only_rain);
    assert(config.palette == palette_named("matrix") && the_rain_is_falling);
    reset_sign_state();
    the_rain_is_falling = 0;
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

/* A hawk is over the places it flew over in the last step and not only the ones it
 * ended it on: it flies two or three times its reach in a frame. */
static void test_a_hawk_scatters_the_places_along_the_step_it_flew(void) {
    reset_sign_state();
    apply_screen_size(200, 50, 1600, 800);
    config.hawks = 1;
    double reach = HAWK_SCATTER * hawk_reach();
    hawks[0] = (hawk_t){.x = 500, .y = 300, .direction = 0, .stride = 100};
    assert(a_hawk_is_over(500, 300));                /* Where it is. */
    assert(a_hawk_is_over(420, 300 + 0.9 * reach));  /* Behind it, on the step... */
    assert(!a_hawk_is_over(420, 300 + 1.1 * reach)); /* ...and not off to one side. */
    assert(a_hawk_is_over(400, 300));                /* Where the step began. */
    assert(!a_hawk_is_over(400 - 1.1 * reach, 300)); /* Not before that. */
    assert(a_hawk_is_over(500 + 0.9 * reach, 300));  /* Its own reach beyond it... */
    assert(!a_hawk_is_over(500 + 1.1 * reach, 300)); /* ...and not where it has yet to go. */
    /* Whichever way it heads: down the screen, the step is above it. */
    hawks[0].direction = M_PI / 2;
    assert(a_hawk_is_over(500, 230) && !a_hawk_is_over(500, 370));
    assert(a_hawk_is_over(500 + 0.9 * reach, 230) && !a_hawk_is_over(500 + 1.1 * reach, 230));
    /* A hawk that has flown nowhere is a point, as it was. */
    hawks[0].stride = 0;
    assert(!a_hawk_is_over(500, 230) && a_hawk_is_over(500, 300 + 0.9 * reach));
    /* And the hunt is where the stride comes from: the step it just took. */
    config.hawks = 0;
    reset_sign_state();
    apply_screen_size(200, 50, 1600, 800);
    sign_sky_t world;
    open_the_world(&world, 200, 3);
    config.hawks = 1;
    place_hawks();
    assert(hawks[0].stride == 0);
    fly_the_world(&world, 0, 1, 60);
    assert(fabs(hawks[0].stride - config.speed * HAWK_SPEED) < 1e-9 ||
           fabs(hawks[0].stride - config.speed * HAWK_DIVE_SPEED) < 1e-9);
    config.hawks = 0;
    close_the_world(&world);
    reset_sign_state();
}

/* The share of a sign's writers that are scattered, over the seconds it is held,
 * with this many hawks hunting the flock round it. */
static double share_of_a_sign_scattered(const char *text, int hawks_up, int fps, int seed) {
    sign_sky_t world;
    lay_out_a_sign_on(&world, 96, 26, fps, text, 800, seed);
    config.hawks = hawks_up;
    place_hawks();
    double sum = 0;
    int frames = 0;
    for (double at = 1.0 / fps; at < 22; at += 1.0 / fps) {
        step_the_world(&world, at);
        if (at < 8 || !the_sign.up) continue; /* Written by then, and held. */
        int writers = 0, away = 0;
        for (int i = 0; i < config.birds; i++) {
            if (formation.slot[i] < 0) continue;
            writers++;
            away += world.birds[i].scattered > 0;
        }
        sum += (double)away / writers;
        frames++;
    }
    assert(frames > 0 && the_sign.up);
    config.hawks = 0;
    close_the_world(&world);
    reset_sign_state();
    return sum / frames;
}

/* A hawk hunts the free birds, which are all round a sign, so it is over the sign
 * much of the time. On 800 birds at 96 by 26 cells it left 43% of the writers
 * scattered with one hawk and 67% with two, and a clock could not be read; then 5 to
 * 6% and 10 to 11%; and turned from the text as the flock is, 3 to 5% and 6 to 8%. */
static void test_one_hawk_leaves_a_sign_readable(void) {
    for (int seed = 5; seed <= 6; seed++) {
        double one = share_of_a_sign_scattered("HELLO WORLD", 1, 25, seed);
        double two = share_of_a_sign_scattered("HELLO WORLD", 2, 25, seed);
        assert(one > 0.01); /* A hawk over the sign does scatter it... */
        assert(one < 0.10); /* ...and a sign with one hawk up is whole nearly always. */
        /* Two scatter it too, and not by much more: a hawk hunts round the text now,
         * and whether two cross it more often than one in a flight of fourteen
         * seconds is a matter of the chase, which came out either way. */
        assert(two > 0.01 && two < 0.15);
    }
    /* A clock is the same story, and at the rate a person sees it. */
    reset_sign_state();
    apply_screen_size(96, 26, 96 * 8, 26 * 16);
    config.pace_notch = DEFAULT_PACE_NOTCH;
    apply_notches();
    frame_seconds = 1.0 / 60;
    update_speed();
    the_sign.kind = SIGN_CLOCK;
    the_sign.virtual_clock = 1;
    the_sign.origin = local_time(10, 9, 0);
    config.bird_size = 0;
    settle_the_bird_size();
    begin_the_intro();
    sign_sky_t world;
    open_the_world(&world, 800, 5);
    sign_advance(world.birds);
    assert(the_sign.up);
    config.hawks = 1;
    place_hawks();
    double sum = 0;
    int frames = 0;
    for (double at = 1.0 / 60; at < 30; at += 1.0 / 60) {
        step_the_world(&world, at);
        if (at < 8 || !the_sign.up) continue;
        int writers = 0, away = 0;
        for (int i = 0; i < config.birds; i++) {
            if (formation.slot[i] < 0) continue;
            writers++;
            away += world.birds[i].scattered > 0;
        }
        sum += (double)away / writers;
        frames++;
    }
    assert(frames > 0 && sum / frames < 0.10);
    config.hawks = 0;
    close_the_world(&world);
    reset_sign_state();
}

/* The other side of it: a hawk that dives straight through a line of letters takes
 * a good part of them from their places, a quarter of the line at 96 by 26 cells,
 * and as many places in a recording at 25 frames a second as live at 60, though it
 * flies two and a half times as far in a frame there. */
static double share_of_a_line_a_dive_takes(int fps, int seed, int *places, int *count) {
    sign_sky_t world;
    lay_out_a_sign_on(&world, 96, 26, fps, "HELLO", 800, seed);
    fly_the_world(&world, 1.0 / fps, 8, fps);
    config.hawks = 1;
    place_hawks();
    double middle_x = 0, middle_y = 0;
    for (int t = 0; t < formation.count; t++) {
        middle_x += formation.x[t] / formation.count;
        middle_y += formation.y[t] / formation.count;
    }
    int writers = 0;
    for (int i = 0; i < config.birds; i++) writers += formation.slot[i] >= 0;
    char *scattered = calloc((size_t)config.birds, 1);
    char *gone = calloc((size_t)config.birds, 1);
    char *passed = calloc(FORMATION_MAX_TARGETS, 1);
    assert(scattered != NULL && gone != NULL && passed != NULL);
    /* Along the line, at the pace of a dive, from well before the first letter. */
    double step = config.speed * HAWK_DIVE_SPEED;
    double x = middle_x - 4 * formation.cell * FONT_ADVANCE;
    double end = middle_x + 4 * formation.cell * FONT_ADVANCE;
    for (double at = 8 + 1.0 / fps; x < end; at += 1.0 / fps, x += step) {
        hawks[0].x = x;
        hawks[0].y = middle_y;
        hawks[0].direction = 0;
        hawks[0].stride = step;
        hawks[0].prey = -1;
        /* The places under the step it flew, before the hunt turns it. */
        for (int t = 0; t < formation.count; t++)
            if (a_hawk_is_over(formation.x[t], formation.y[t])) passed[t] = 1;
        step_the_world(&world, at);
        hawks[0].x = x; /* Held to its line: the hunt moved it a little. */
        hawks[0].y = middle_y;
        for (int i = 0; i < config.birds; i++) {
            if (formation.slot[i] < 0) continue;
            if (world.birds[i].scattered > 0) scattered[i] = 1;
            if (home_distance(&world, i) > 3 * formation.cell) gone[i] = 1;
        }
    }
    int hit = 0, away = 0;
    for (int i = 0; i < config.birds; i++) hit += scattered[i], away += gone[i];
    *places = 0;
    for (int t = 0; t < formation.count; t++) *places += passed[t];
    *count = formation.count;
    double taken = (double)hit / writers;
    assert(away * 10 >= hit * 9); /* Visibly: nearly every one it takes flies off. */
    free(scattered);
    free(gone);
    free(passed);
    config.hawks = 0;
    close_the_world(&world);
    reset_sign_state();
    return taken;
}

/* The places are the same at either rate, to a place or two at the ends of the
 * line: that is the step being a segment, and it is exact. Which birds come off
 * them is not: the hunt, and the text it is turned from, turn the hawk within the
 * frame, more at 25 frames than at 60, and a recording's dive, held to its line
 * only between frames, took 16 to 25% of the line from flock to flock where live
 * took 27% every time. */
static void test_a_hawk_diving_through_the_letters_scatters_them_at_any_frame_rate(void) {
    int places[2], count[2];
    double live = share_of_a_line_a_dive_takes(60, 5, &places[1], &count[1]);
    double recorded = share_of_a_line_a_dive_takes(25, 5, &places[0], &count[0]);
    assert(count[0] == count[1]);
    assert(places[1] > count[1] * 15 / 100 && places[1] < count[1] / 2); /* A stripe, not all. */
    assert(abs(places[0] - places[1]) <= 2 + places[1] / 20);
    assert(live > 0.15 && live < 0.5);
    assert(recorded > 0.05 && recorded < 0.5);
}

/* The pull round a sign, as geometry: along the ellipse, one way while the sign is
 * held and the other way the next time, back towards it from far off, nothing to a
 * bird of the far sky, and nothing without a sign. */
static void test_the_free_flock_is_turned_round_a_sign(void) {
    reset_sign_state();
    apply_screen_size(200, 50, 1600, 800);
    config.bird_size = 12;
    formation_clear();
    formation.sign = 1;
    formation.writing = 1;
    formation.keep_out = 1;
    formation.box = (sign_box_t){600, 300, 1000, 500};
    formation.band = SIGN_KEEP_OUT_BAND * config.bird_size;
    bird_t bird;
    memset(&bird, 0, sizeof(bird));
    for (unsigned cycle = 0; cycle < 4; cycle++) {
        the_sign.cycle = cycle;
        double way = cycle % 2 ? -1 : 1;
        /* Right of the text: down the screen, then up; above it: right, then left;
         * left of it: up; below it: left. */
        double at[4][2] = {{1150, 400}, {800, 200}, {450, 400}, {800, 600}};
        double expect[4][2] = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};
        for (int k = 0; k < 4; k++) {
            bird.x = at[k][0];
            bird.y = at[k][1];
            vector_t flow = sign_orbit_vector(&bird);
            double along = flow.x * expect[k][0] + flow.y * expect[k][1];
            assert(way * along > 0.5 * SIGN_ORBIT_ALONG);
        }
    }
    /* From a corner of the screen it is pulled in as well as round. */
    the_sign.cycle = 0;
    bird.x = 30;
    bird.y = 30;
    vector_t flow = sign_orbit_vector(&bird);
    assert(flow.x * (800 - 30) + flow.y * (400 - 30) > 0);
    /* Not for a bird of the far sky, nor with no sign up. */
    bird.layer = 1;
    flow = sign_orbit_vector(&bird);
    assert(flow.x == 0 && flow.y == 0);
    bird.layer = 0;
    formation.writing = 0;
    flow = sign_orbit_vector(&bird);
    assert(flow.x == 0 && flow.y == 0);
    formation_clear();
    reset_sign_state();
}

/* And as flight: the free birds round a sign wheel round it one way, most of them
 * at any moment, and the other way on the next hold. Measured on 800 birds saying
 * HELLO WORLD at 96 by 26 cells: without the pull, about half of them went each
 * way. */
static double share_wheeling_down_the_right(int columns, int rows, unsigned cycle, int seed) {
    sign_sky_t world;
    lay_out_a_sign_on(&world, columns, rows, 25, "HELLO WORLD", 800, seed);
    the_sign.cycle = cycle;
    double cx = (formation.box.left + formation.box.right) / 2;
    double cy = (formation.box.top + formation.box.bottom) / 2;
    double sum = 0;
    int samples = 0;
    for (double at = 0.04; at < 12; at += 0.04) {
        step_the_world(&world, at);
        if (at < 6 || ((int)(at * 25 + 0.5)) % 5 != 0) continue;
        int free_birds = 0, wheeling = 0;
        for (int i = 0; i < config.birds; i++) {
            if (formation.slot[i] >= 0 || world.birds[i].layer > 0) continue;
            free_birds++;
            double dx = world.birds[i].x - cx, dy = world.birds[i].y - cy;
            wheeling += dx * sin(world.birds[i].direction) - dy * cos(world.birds[i].direction) > 0;
        }
        sum += (double)wheeling / free_birds;
        samples++;
    }
    assert(the_sign.up && samples > 0);
    close_the_world(&world);
    reset_sign_state();
    return sum / samples;
}

static void test_the_free_flock_wheels_round_a_sign_one_way(void) {
    int sizes[][2] = {{96, 26}, {200, 50}};
    for (int s = 0; s < 2; s++)
        for (int seed = 1; seed <= 2; seed++) {
            assert(share_wheeling_down_the_right(sizes[s][0], sizes[s][1], 0, seed) > 0.75);
            assert(share_wheeling_down_the_right(sizes[s][0], sizes[s][1], 1, seed) < 0.25);
        }
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

    /* A roomy screen gives the sign the roomy shares of the room, in letters a
     * seventh of its rows tall, the writers a sixth of a flock short of three
     * fifths, the band four steps of flight. */
    lay_out_a_sign_on(&world, 200, 50, 25, "HELLO WORLD", 800, 5);
    {
        double pad = config.bird_size * 2.0;
        sign_lines_t lines;
        double cell;
        assert(sign_font_rows_now() == 7 && fabs(sign_font_cell() - 7 * 16.0 / FONT_HEIGHT) < 1e-9);
        assert(sign_fit("HELLO WORLD", 0, (1600 - 2 * pad) * SIGN_WIDTH_SHARE,
                        (800 - 2 * pad) * SIGN_HEIGHT_SHARE,
                        fmin(SIGN_LARGEST_CELL * config.bird_size, sign_font_cell()), &lines,
                        &cell) == 2);
        assert(fabs(formation.cell - cell) < 1e-9 && fabs(cell - sign_font_cell()) < 1e-9);
        assert(the_sign.per_cell == (int)(800 * SIGN_WRITER_SHARE) / formation.count);
        assert(fabs(sign_band() - SIGN_KEEP_OUT_STEPS * config.speed) < 1e-9);
    }
    close_the_world(&world);

    /* A small one gives the sign the small shares, half and half, in cells that are
     * still letters and smaller than the most a sign may take would make them, and
     * the band is a fifth of the screen at the most, at the pace of a recording and
     * as it was at the pace of a terminal. */
    lay_out_a_sign_on(&world, 96, 26, 25, "HELLO WORLD", 800, 5);
    {
        double pad = config.bird_size * 2.0;
        sign_lines_t lines;
        double small_cell, most_cell;
        assert(sign_fit("HELLO WORLD", 0, (768 - 2 * pad) * SIGN_WIDTH_SHARE_SMALL,
                        (416 - 2 * pad) * SIGN_HEIGHT_SHARE_SMALL,
                        fmin(SIGN_LARGEST_CELL * config.bird_size, sign_font_cell()), &lines,
                        &small_cell) > 0);
        assert(sign_fit("HELLO WORLD", 0, (768 - 2 * pad) * SIGN_WIDTH_MOST,
                        (416 - 2 * pad) * SIGN_HEIGHT_MOST, SIGN_LARGEST_CELL * config.bird_size,
                        &lines, &most_cell) > 0);
        assert(fabs(formation.cell - small_cell) < 1e-9);
        assert(formation.cell < most_cell && formation.cell >= SIGN_SMALLEST_CELL);
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
     * 80 by 24 its cell is the floor, or what the most a sign may take gives if that
     * is less. */
    reset_sign_state();
    apply_screen_size(80, 24, 640, 384);
    config.bird_size = 12;
    {
        sign_lines_t lines, roomy;
        double cell, roomy_cell, width, height;
        double room_width = 640 - 2 * 24.0, room_height = 384 - 2 * 24.0;
        assert(sign_fit_in("BACK IN FIVE MINUTES", 0, room_width, room_height, 1e9, &lines, &cell,
                           &width, &height) == 3);
        assert(sign_fit("BACK IN FIVE MINUTES", 0, room_width * SIGN_WIDTH_MOST,
                        room_height * SIGN_HEIGHT_MOST, 1e9, &roomy, &roomy_cell) == 3);
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
        struct pollfd wait = {.fd = master, .events = POLLIN};
        if (poll(&wait, 1, 100) > 0) {
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
    for (struct pollfd wait = {.fd = master, .events = POLLIN}; poll(&wait, 1, 100) > 0;) {
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

/* A snapshot is taken at the end of a live run, and what is said about it, that it
 * was written or that it could not be, is said where it can be read: when the
 * alternate screen has been given back, and not on it, where it is gone with it. */
static void test_a_snapshot_is_reported_after_the_terminal_is_given_back(void) {
    static char output[1 << 20];
    char good[600], bad[700];
    scratch_file(good, sizeof(good), "taken.png");
    snprintf(bad, sizeof(bad), "%s/missing/taken.png", scratch);
    for (int written = 0; written < 2; written++) {
        reset_sign_state();
        int master = posix_openpt(O_RDWR | O_NOCTTY);
        assert(master >= 0 && grantpt(master) == 0 && unlockpt(master) == 0);
        const char *name = ptsname(master);
        assert(name != NULL);
        int terminal = open(name, O_RDWR | O_NOCTTY);
        assert(terminal >= 0);
        struct winsize size = {.ws_row = 24, .ws_col = 80, .ws_xpixel = 640, .ws_ypixel = 384};
        assert(ioctl(terminal, TIOCSWINSZ, &size) == 0);
        fflush(NULL);
        pid_t child = fork();
        assert(child >= 0);
        if (child == 0) {
            alarm(60);
            if (dup2(terminal, STDIN_FILENO) < 0 || dup2(terminal, STDOUT_FILENO) < 0 ||
                dup2(terminal, STDERR_FILENO) < 0)
                _exit(99);
            terminal_is_raw = terminal_restored = alt_screen_is_on = sprites_uploaded = 0;
            char *argv[] = {
                "cbirds", "--unlock-fps", "--frames",           "30", "-n", "200", "--seed",
                "3",      "--snapshot",   written ? good : bad, NULL};
            exit(cbirds_application_main(10, argv)); /* exit: the terminal is put back. */
        }
        close(terminal);
        size_t length = 0;
        int status = 0;
        for (;;) {
            struct pollfd wait = {.fd = master, .events = POLLIN};
            if (poll(&wait, 1, 100) > 0) {
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
        for (struct pollfd wait = {.fd = master, .events = POLLIN}; poll(&wait, 1, 100) > 0;) {
            ssize_t got = read(master, output + length, sizeof(output) - 1 - length);
            if (got <= 0) break;
            length += (size_t)got;
        }
        output[length] = '\0';
        close(master);
        assert(WIFEXITED(status) && WEXITSTATUS(status) == (written ? EXIT_SUCCESS : EXIT_FAILURE));
        char message[800];
        snprintf(message, sizeof(message),
                 written ? "cbirds: wrote %s" : "cbirds: could not write %s", written ? good : bad);
        const char *told = strstr(output, message);
        const char *taken = strstr(output, ALT_SCREEN_ON);
        const char *given_back = NULL;
        for (const char *at = strstr(output, ALT_SCREEN_OFF); at != NULL;
             at = strstr(at + 1, ALT_SCREEN_OFF))
            given_back = at;
        assert(taken != NULL && given_back != NULL && told != NULL);
        assert(told > given_back && given_back > taken);
        assert(strstr(told + 1, message) == NULL);
        if (written) {
            FILE *file = fopen(good, "rb");
            assert(file != NULL);
            unsigned char signature[4];
            assert(fread(signature, 1, 4, file) == 4 && memcmp(signature, "\x89PNG", 4) == 0);
            fclose(file);
            remove(good);
        }
    }
    reset_sign_state();
}

/* The keys are read from standard input, and a question to the terminal is
 * written to standard output: these tests put a pipe on the one and nothing on
 * the other, so that the terminal running them is not asked anything, and give
 * both back after. */
static int saved_keys = -1, saved_output = -1;

static void keys_from(int fd) {
    fflush(stdout);
    saved_keys = dup(STDIN_FILENO);
    saved_output = dup(STDOUT_FILENO);
    int quiet = open("/dev/null", O_WRONLY);
    assert(saved_keys >= 0 && saved_output >= 0 && quiet >= 0);
    assert(dup2(fd, STDIN_FILENO) == STDIN_FILENO && dup2(quiet, STDOUT_FILENO) == STDOUT_FILENO);
    close(quiet);
}

static void keys_back(void) {
    fflush(stdout);
    assert(dup2(saved_keys, STDIN_FILENO) == STDIN_FILENO);
    assert(dup2(saved_output, STDOUT_FILENO) == STDOUT_FILENO);
    close(saved_keys);
    close(saved_output);
    saved_keys = saved_output = -1;
}

/* A terminal that answers in pieces, with a wait before each: a child that writes
 * them to the descriptor the questions are read from while the test is asking. The
 * waits are from the moment the child is running, which the test waits for before
 * it asks: a question is given sixty milliseconds, and on a Mac with the address
 * sanitizer the fork alone took longer than that. */
static pid_t answer_in_pieces(int fd, const char *const pieces[], const int waits[], int count) {
    int started[2];
    assert(pipe(started) == 0);
    fflush(NULL);
    pid_t child = fork();
    assert(child >= 0);
    if (child == 0) {
        close(started[0]);
        if (write(started[1], "", 1) != 1) _exit(1);
        close(started[1]);
        for (int i = 0; i < count; i++) {
            usleep((useconds_t)waits[i] * 1000);
            if (write(fd, pieces[i], strlen(pieces[i])) < 0) _exit(1);
        }
        _exit(0);
    }
    close(started[1]);
    char running;
    assert(read(started[0], &running, 1) == 1);
    close(started[0]);
    return child;
}

static void wait_for_the_answerer(pid_t child) {
    int status = 0;
    assert(waitpid(child, &status, 0) == child);
    assert(WIFEXITED(status) && WEXITSTATUS(status) == 0);
}

static void test_a_colour_reply_is_whole_at_its_terminator_and_not_at_a_letter_in_it(void) {
    int keys[2];
    assert(pipe(keys) == 0);
    keys_from(keys[0]);
    input_state = INPUT_NORMAL;
    uint8_t rgb[3];

    /* The red of a terminal whose first colour is cc0000, in one piece: the 'c' in
     * the digits used to end the read after "rgb:c". */
    const char *whole = "\033]4;1;rgb:cccc/0000/0000\033\\";
    assert(write(keys[1], whole, strlen(whole)) == (ssize_t)strlen(whole));
    assert(ask_colour("\033]4;1;?\033\\", rgb) && rgb[0] == 0xcc && rgb[1] == 0 && rgb[2] == 0);

    /* In pieces, with the cut where the digits are, where the ST is, and a bell for
     * the end of it. */
    static const char *const CUT_IN_THE_DIGITS[] = {"\033]4;1;rgb:cc", "cc/0000/0000\033\\"};
    static const char *const CUT_IN_THE_ST[] = {"\033]4;1;rgb:cccc/0000/0000\033", "\\"};
    static const char *const WITH_A_BELL[] = {"\033]4;1;rgb:cc", "cc/0000/0000\007"};
    static const char *const *const SPLIT[] = {CUT_IN_THE_DIGITS, CUT_IN_THE_ST, WITH_A_BELL};
    /* Apart enough that the first piece is read alone, and soon enough after the
     * question that a machine whose sleeps run long still answers in time. */
    static const int WAITS[] = {2, 10};
    for (size_t i = 0; i < sizeof(SPLIT) / sizeof(*SPLIT); i++) {
        rgb[0] = rgb[1] = rgb[2] = 7;
        pid_t terminal = answer_in_pieces(keys[1], SPLIT[i], WAITS, 2);
        /* Asked as ask_colour asks, with more than its sixty milliseconds to wait:
         * the joining of the pieces is what is tested, and a Mac running the
         * address sanitizer took longer than that between them. */
        char reply[128];
        const char *question = "\033]4;1;?\033\\";
        assert(terminal_query(question, strlen(question), reply, sizeof(reply), 2000) > 0);
        assert(parse_osc_colour(reply, rgb));
        wait_for_the_answerer(terminal);
        assert(rgb[0] == 0xcc && rgb[1] == 0 && rgb[2] == 0);
    }

    /* The letter that ends a device attributes reply is the end of a CSI reply, and
     * the bytes after it are not part of it. */
    char reply[64];
    const char *attributes = "\033[?64;1;2cXY";
    assert(write(keys[1], attributes, strlen(attributes)) == (ssize_t)strlen(attributes));
    assert(terminal_query("\033[c", 3, reply, sizeof(reply), 200) == strlen("\033[?64;1;2c"));
    assert(strcmp(reply, "\033[?64;1;2c") == 0);
    keys_back();
    close(keys[0]);
    close(keys[1]);
}

static void test_a_late_reply_to_an_earlier_question_is_not_taken_for_this_one(void) {
    int keys[2];
    assert(pipe(keys) == 0);
    keys_from(keys[0]);
    input_state = INPUT_NORMAL;
    uint8_t rgb[3];
    /* The background's answer, late, and then the third colour's own. */
    const char *late = "\033]11;rgb:bbbb/bbbb/bbbb\033\\";
    const char *own = "\033]4;3;rgb:1111/2222/3333\033\\";
    assert(write(keys[1], late, strlen(late)) == (ssize_t)strlen(late));
    assert(write(keys[1], own, strlen(own)) == (ssize_t)strlen(own));
    assert(ask_colour("\033]4;3;?\033\\", rgb));
    assert(rgb[0] == 0x11 && rgb[1] == 0x22 && rgb[2] == 0x33);
    /* Only the late one in the pipe: nothing answers, and it is not the answer. */
    assert(write(keys[1], late, strlen(late)) == (ssize_t)strlen(late));
    assert(!ask_colour("\033]4;3;?\033\\", rgb));
    keys_back();
    close(keys[0]);
    close(keys[1]);
    assert(input_state == INPUT_NORMAL);
}

/* The slider that b moves, as a key reader finds it. */
static int boundary_after_reading(const char *bytes) {
    config.boundary_notch = 4;
    apply_notches();
    feed_input(bytes);
    return config.boundary_notch;
}

static void test_a_reply_that_comes_late_is_not_read_as_keys(void) {
    reset_test_config();
    input_state = INPUT_NORMAL;
    /* An OSC reply with b in every channel, which moved the boundary slider to its
     * end when it was read as keys, and the same ended by a bell. */
    assert(boundary_after_reading("\033]11;rgb:bbbb/bbbb/bbbb\033\\") == 4);
    assert(boundary_after_reading("\033]11;rgb:bbbb/bbbb/bbbb\007") == 4);
    assert(boundary_after_reading("\033]4;3;rgb:eeee/aaaa/0000\033\\") == 4);
    /* A DCS reply, which only ST ends: the bell in it is part of it. */
    assert(boundary_after_reading("\033P>|bbb\007bb\033\\") == 4);
    assert(input_state == INPUT_NORMAL);
    /* And the key after it is a key. */
    assert(boundary_after_reading("\033]11;rgb:bbbb/bbbb/bbbb\033\\b") == 3);
    assert(boundary_after_reading("b\033]11;rgb:aaaa/aaaa/aaaa\033\\b") == 2);

    /* In pieces, the cut anywhere: after the introducer, in the body, between the
     * escape and the backslash. */
    static const char REPLY[] = "\033]11;rgb:bbbb/bbbb/bbbb\033\\";
    for (size_t cut = 1; cut < sizeof(REPLY) - 1; cut++) {
        char head[40], tail[40];
        memcpy(head, REPLY, cut);
        head[cut] = '\0';
        strcpy(tail, REPLY + cut);
        config.boundary_notch = 4;
        apply_notches();
        feed_input(head);
        feed_input(tail);
        assert(config.boundary_notch == 4 && input_state == INPUT_NORMAL);
    }
}

static void test_a_string_the_key_reader_cannot_end_is_given_up(void) {
    reset_test_config();
    input_state = INPUT_NORMAL;
    /* Alt with ] begins one too, and what is typed after it is typed. Nothing more
     * of it comes for a moment: it is over. */
    feed_input("\033]");
    assert(input_state == INPUT_STRING);
    input_string_at.tv_sec -= 5;
    assert(boundary_after_reading("b") == 3 && input_state == INPUT_NORMAL);
    /* A string that goes on for longer than any reply is not one. The reader takes a
     * hundred bytes at a time. */
    char chunk[91];
    memset(chunk, 'x', sizeof(chunk) - 1);
    chunk[sizeof(chunk) - 1] = '\0';
    feed_input("\033]");
    for (int i = 0; i < 2; i++) {
        feed_input(chunk);
        assert(input_state == INPUT_STRING);
    }
    feed_input(chunk);
    assert(input_state == INPUT_NORMAL);
    assert(boundary_after_reading("b") == 3);
    /* An escape in a string ends it and begins what it begins: here a mouse report
     * is read, and a key after it. */
    mouse.present = 0;
    apply_screen_size(80, 24, 80 * 8, 24 * 16);
    assert(boundary_after_reading("\033]11;rgb:bb\033[<35;10;5Mb") == 3);
    assert(mouse.present && input_state == INPUT_NORMAL);
    mouse.present = 0;
}

static void test_the_rest_of_a_reply_that_was_cut_off_by_the_deadline_is_thrown_away(void) {
    reset_test_config();
    int keys[2];
    assert(pipe(keys) == 0);
    keys_from(keys[0]);
    input_state = INPUT_NORMAL;
    uint8_t rgb[3];
    /* The terminal is slow: the head of its answer is in time and the tail is not.
     * It is no answer, and the tail, when it comes, is not keys either. */
    const char *head = "\033]11;rgb:bb";
    assert(write(keys[1], head, strlen(head)) == (ssize_t)strlen(head));
    assert(!ask_colour("\033]11;?\033\\", rgb));
    assert(input_state == INPUT_STRING);
    keys_back();
    assert(boundary_after_reading("bb/bbbb/bbbb\033\\") == 4 && input_state == INPUT_NORMAL);
    close(keys[0]);
    close(keys[1]);
}

static void test_a_sign_has_a_bird_as_wide_as_its_cells_unless_it_is_told(void) {
    reset_sign_state();
    /* As wide as the cells of its letters, where a bird of thirty pixels is a smudge
     * on a letter of twelve: a letter a seventh of the rows of 200 by 50 cells is
     * seven rows of sixteen pixels, which is a cell of sixteen. Never more than the
     * usual thirty, which a screen of twice the pixels reaches. */
    apply_screen_size(200, 50, 1600, 800);
    ask_for_a_sign("HI");
    config.bird_size = 0;
    settle_the_bird_size();
    assert(config.bird_size == 16);
    reset_sign_state();
    apply_screen_size(200, 50, 3200, 1600);
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
            struct pollfd wait = {.fd = master, .events = POLLIN};
            if (poll(&wait, 1, 100) > 0) {
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

/* A report is a cell of the screen. Billions of columns is not a place. */
static void test_a_pointer_reported_beyond_the_screen_is_at_its_edge(void) {
    reset_test_config();
    apply_screen_size(80, 24, 80 * 8, 24 * 16);
    mouse.present = 0;
    read_mouse_report("<35;2147483647;5");
    assert(mouse.present && mouse.x == 79.5 * screen.cell_width &&
           mouse.y == 4.5 * screen.cell_height);
    read_mouse_report("<35;5;2147483647");
    assert(mouse.x == 4.5 * screen.cell_width && mouse.y == 23.5 * screen.cell_height);
    read_mouse_report("<35;81;25");
    assert(mouse.x == 79.5 * screen.cell_width && mouse.y == 23.5 * screen.cell_height);
    /* On the screen it is where it says, and nothing at all is not a place. */
    read_mouse_report("<35;80;24");
    assert(mouse.x == 79.5 * screen.cell_width && mouse.y == 23.5 * screen.cell_height);
    read_mouse_report("<35;3;2");
    assert(mouse.x == 2.5 * screen.cell_width && mouse.y == 1.5 * screen.cell_height);
    read_mouse_report("<35;0;2147483647");
    assert(mouse.x == 2.5 * screen.cell_width && mouse.y == 1.5 * screen.cell_height);
    mouse.present = 0;
}

/* -h is one screen: with the lines that wrap on an 80 column terminal counted as
 * two, it fits the 24 rows of an 80 by 24 terminal. */
static void test_the_short_help_is_one_screen_at_eighty_columns(void) {
    char path[512], text[8192];
    scratch_file(path, sizeof(path), "short-help.txt");
    FILE *help = fopen(path, "w+");
    assert(help != NULL);
    usage(help, "cbirds", 0);
    rewind(help);
    size_t length = fread(text, 1, sizeof(text) - 1, help);
    text[length] = '\0';
    fclose(help);
    assert(unlink(path) == 0);

    int rows = 0, columns = 0;
    for (const char *c = text; *c != '\0'; c++) {
        if ((*c & 0xC0) == 0x80) continue; /* Not a column: the rest of a character. */
        if (*c == '\n') {
            rows += columns <= 80 ? 1 : (columns + 79) / 80;
            columns = 0;
            continue;
        }
        columns++;
    }
    assert(rows <= 24);
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
        if (palette_follows_the_theme()) continue;
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
    assert(strstr(line, "user") != NULL);
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

/* Whether a descriptor has something to read within this many milliseconds:
 * select, which macOS answers for a terminal as Linux does. */
static int readable_within(int fd, int milliseconds) {
    fd_set readable;
    FD_ZERO(&readable);
    FD_SET(fd, &readable);
    struct timeval wait = {milliseconds / 1000, (milliseconds % 1000) * 1000};
    return select(fd + 1, &readable, NULL, NULL, &wait);
}

/* The text is on the first row of twenty and the lines after it are empty, so a
 * screen of ten has scrolled it away and a larger one has no letter to show. */
static const char TEXT_AT_THE_TOP[] = "hello world\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n";

static void test_a_window_with_none_of_the_text_left_keeps_the_letters_and_their_size(void) {
    world_t world;
    world_open(&world, TEXT_AT_THE_TOP, 40, 20);
    int before = config.birds;
    assert(before == 10 && the_letters.cols == 40 && the_letters.rows == 20);

    /* A bigger screen, with nothing of the text on it once it is laid out there. The
     * letters are still the ones laid out on forty by twenty, in arrays of that size,
     * and the size they say is the size they are: a sanitizer sees the arrays. */
    apply_screen_size(100, 10, 0, 0);
    assert(text_renderer_fits_the_screen());
    for (int frame = 0; frame < 120; frame++)
        assert(reflow_the_letters(&world.birds, &world.snapshot) == 0);
    assert(the_letters.cols == 40 && the_letters.rows == 20 && config.birds == before);
    assert(the_letters.grid != NULL && the_letters.at != NULL && the_letters.claim != NULL);
    /* It was looked at once and found empty, and is not looked at again. */
    assert(reflow_wait == 0);

    /* The text is still painted, in the corner of a screen that has more room. */
    int cells;
    cell_t *picture = world_picture(&world, &cells);
    assert(cells == 100 * 10);
    assert(picture[0].glyph == 'h' && picture[4].glyph == 'o' && picture[6].glyph == 'w');
    free(picture);

    /* And it flies and comes home on the screen it does not fit. */
    assert(spatial_grid_prepare(&world.grid, screen.width, screen.height, config.birds) ==
           SPATIAL_GRID_OK);
    feed_input("\r");
    assert(world_run_until(&world, LETTERS_IN_FLIGHT, 60 * 6) > 0);
    for (int frame = 0; frame < 120; frame++) {
        world_step(&world);
        picture = world_picture(&world, &cells);
        free(picture);
    }
    feed_input("\r");
    assert(world_run_until(&world, LETTERS_AT_REST, 60 * 40) > 0 && world_all_home(&world));

    /* A smaller screen is no different: what there is stays. */
    apply_screen_size(20, 5, 0, 0);
    assert(text_renderer_fits_the_screen());
    for (int frame = 0; frame < 120; frame++)
        assert(reflow_the_letters(&world.birds, &world.snapshot) == 0);
    assert(the_letters.cols == 40 && the_letters.rows == 20);
    picture = world_picture(&world, &cells);
    assert(cells == 20 * 5 && picture[0].glyph == 'h');
    free(picture);

    /* A size where it does show is laid out, whatever the sizes before were. */
    apply_screen_size(60, 20, 0, 0);
    assert(text_renderer_fits_the_screen());
    int laid_out = 0;
    for (int frame = 0; frame < 60 && !laid_out; frame++)
        laid_out = reflow_the_letters(&world.birds, &world.snapshot);
    assert(laid_out && the_letters.cols == 60 && the_letters.rows == 20 && config.birds == before);
    /* The size that had nothing is still known to have nothing. */
    apply_screen_size(100, 10, 0, 0);
    for (int frame = 0; frame < 120; frame++)
        assert(reflow_the_letters(&world.birds, &world.snapshot) == 0);
    assert(the_letters.cols == 60 && the_letters.rows == 20);
    world_close(&world);
}

/* A letter that is in the air is a bird that is flying, and the other way round: the
 * cycle waits for every letter to land, and a bird that is told nothing never leaves. */
static void assert_the_letters_in_the_air_are_the_birds_in_the_air(const world_t *world) {
    for (int i = 0; i < config.birds; i++)
        assert(letter_is_airborne(&the_letters.letter[i]) == !world->birds[i].perched);
}

/* How many times the cycle began over `frames` frames of a world with hawks, and
 * whether a bird and its letter ever disagreed about being in the air. */
static int cycles_with_hawks(int hawk_count, int seed, int frames) {
    world_t world;
    world_open(&world, NEOFETCH_LIKE, 60, 12);
    config.hawks = hawk_count;
    seed_random((unsigned)seed);
    place_hawks();
    for (int frame = 0; frame < frames; frame++) {
        world_step(&world);
        assert_the_letters_in_the_air_are_the_birds_in_the_air(&world);
    }
    int cycles = the_letters.cycles;
    world_close(&world);
    return cycles;
}

static void test_a_hawk_over_the_text_does_not_stop_the_cycle(void) {
    /* A hawk that touches letters in the very step the rest ends used to leave them
     * in the air for the cycle and on their cells for the flock, and the text then
     * never rested or flew again. Two minutes is four cycles of a text left alone. */
    for (int hawks = 1; hawks <= 3; hawks++)
        for (int seed = 1; seed <= 4; seed++) assert(cycles_with_hawks(hawks, seed, 60 * 120) >= 3);
}

/* The same through the keys, over text, for a run under the float-cast sanitizer to
 * see: the pointer far past the last letter touches none and breaks nothing. */
static void test_a_pointer_at_the_limits_does_not_disturb_the_text(void) {
    world_t world;
    world_open(&world, NEOFETCH_LIKE, 60, 12);
    static const char *const REPORTS[] = {"\033[<35;5;2147483647M", "\033[<35;2147483647;5M",
                                          "\033[<35;2147483647;2147483647M"};
    for (size_t i = 0; i < sizeof(REPORTS) / sizeof(*REPORTS); i++) {
        mouse.present = 0;
        assert(feed_input(REPORTS[i]) == 1);
        assert(mouse.present && mouse.x < screen.width && mouse.y < screen.height);
        for (int frame = 0; frame < 30; frame++) world_step(&world);
    }
    mouse.present = 0;
    world_close(&world);
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
    /* But the first half second of the program, not of its first frame: a start that
     * took longer than that has no key in it that started anything, and the one
     * typed meanwhile is somebody waking the screen. A quick start is as it was. */
    clock_state.seconds = 0;
    launch_lag = SCREENSAVER_GRACE / 2;
    assert(write(keys[1], "x", 1) == 1);
    assert(handle_input() == 1);
    launch_lag = SCREENSAVER_GRACE + 1.5;
    assert(write(keys[1], "x", 1) == 1);
    assert(handle_input() == 0);
    launch_lag = 0;

    input_fd = saved;
    assert(dup2(saved_stdin, STDIN_FILENO) == STDIN_FILENO);
    close(saved_stdin);
    close(keys[0]);
    close(keys[1]);
    reset_sign_state();
}

/* The whole program, on a terminal of its own, as a flock, a clock, a sign, a
 * text from a file and a text on a pipe, each of them a lock screen: it
 * runs until a key is typed after the grace, and then goes at once with the
 * status of a run that went well, having given the terminal back. */
static void test_a_screensaver_is_a_lock_screen_for_every_mode(void) {
    char text[600];
    scratch_file(text, sizeof(text), "saver.txt");
    world_write(text, "hello, world\nsecond line\n");
    const char *runs[][4] = {
        {NULL, NULL, NULL, NULL},
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
                    /* The terminal answers a colour question late: that is not
                     * somebody, and the lock screen is still there a moment later. */
                    const char *late = "\033]11;rgb:bbbb/bbbb/bbbb\033\\";
                    assert(write(master, late, strlen(late)) == (ssize_t)strlen(late));
                    taken_at = now;
                    typed = -2;
                }
            } else if (typed == -2) {
                clock_gettime(CLOCK_MONOTONIC, &now);
                if (elapsed_seconds(&taken_at, &now) > 0.3) {
                    assert(waitpid(child, &status, WNOHANG) == 0);
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
        /* The tail is filled from its end, so a run that said little has NULs in front
         * of it, and a run that was told to go before its first frame said little. */
        const char *said = tail;
        while (said < tail + sizeof(tail) - 1 && *said == '\0') said++;
        assert(strstr(said, ALT_SCREEN_OFF) != NULL); /* The terminal is given back. */
        close(master);
    }
    assert(unlink(text) == 0);
    reset_sign_state();
}

/* The whole program with text on a pipe, on a terminal of its own that is its
 * controlling one, as `fastfetch | cbirds` has it: the text is read from the pipe,
 * the keys from /dev/tty, the size from the terminal, and the terminal is given
 * back as it was found. The terminal is made a controlling one the portable way,
 * setsid and then TIOCSCTTY, which macOS needs and Linux takes, by a session
 * leader that starts the program and outlives it: macOS revokes a controlling
 * terminal when its session leader exits, and a look at its modes after that is
 * no look at all. The leader compares the modes from before and after and says so
 * in its status. The released 1.5 said "Can't enable raw mode: Inappropriate
 * ioctl for device" here, from tcgetattr on the pipe. */
/* Whether a row of the screen has these words on it. */
static int vt_shows(const vt_t *shown, const char *words) {
    for (int row = 0; row < shown->rows; row++) {
        char line[512];
        int cols = shown->cols < 511 ? shown->cols : 511;
        for (int col = 0; col < cols; col++) {
            uint32_t glyph = vt_cell(shown, col, row)->glyph;
            line[col] = glyph >= 32 && glyph < 127 ? (char)glyph : ' ';
        }
        line[cols] = '\0';
        if (strstr(line, words) != NULL) return 1;
    }
    return 0;
}

enum { LEADER_RAN_WELL = 0, LEADER_MODES_CHANGED = 50, LEADER_NO_TERMINAL = 90 };

static void test_text_on_a_pipe_flies_on_a_terminal_and_gives_it_back(void) {
    for (int resize = 0; resize < 2; resize++) {
        int master = posix_openpt(O_RDWR | O_NOCTTY);
        assert(master >= 0 && grantpt(master) == 0 && unlockpt(master) == 0);
        const char *name = ptsname(master);
        assert(name != NULL);
        int sizing = open(name, O_RDWR | O_NOCTTY);
        assert(sizing >= 0);
        struct winsize size = {.ws_row = 24, .ws_col = 80, .ws_xpixel = 0, .ws_ypixel = 0};
        assert(ioctl(sizing, TIOCSWINSZ, &size) == 0);
        /* Held open, so that the terminal has a side to it until the leader opens
         * its own: a pseudo terminal with none says EIO to a read. */
        int text[2];
        assert(pipe(text) == 0);
        fflush(NULL);
        pid_t leader = fork();
        assert(leader >= 0);
        if (leader == 0) {
            alarm(60);
            close(text[1]);
            close(master);
            close(sizing);
            if (setsid() < 0) _exit(LEADER_NO_TERMINAL);
            int terminal = open(name, O_RDWR);
            if (terminal < 0 || ioctl(terminal, TIOCSCTTY, 0) < 0) _exit(LEADER_NO_TERMINAL + 1);
            struct termios before, after;
            if (tcgetattr(terminal, &before) < 0) _exit(LEADER_NO_TERMINAL + 2);
            pid_t run = fork();
            if (run < 0) _exit(LEADER_NO_TERMINAL + 3);
            if (run == 0) {
                if (dup2(text[0], STDIN_FILENO) < 0 || dup2(terminal, STDOUT_FILENO) < 0 ||
                    dup2(terminal, STDERR_FILENO) < 0)
                    _exit(92);
                close(text[0]);
                close(terminal);
                /* A run as from the shell: nothing the tests before this one left. */
                reset_sign_state();
                render_mode = RENDER_UNSET;
                letters_mode = 0;
                text_path = NULL;
                forget_the_text();
                input_fd = STDIN_FILENO;
                terminal_is_raw = terminal_restored = alt_screen_is_on = sprites_uploaded = 0;
                char *argv[] = {"cbirds", "--seed", "3", NULL};
                exit(cbirds_application_main(3, argv)); /* exit: the terminal is put back. */
            }
            close(text[0]);
            int status = 0;
            if (waitpid(run, &status, 0) != run) _exit(LEADER_NO_TERMINAL + 4);
            if (!WIFEXITED(status) || WEXITSTATUS(status) != EXIT_SUCCESS)
                _exit(WIFEXITED(status) ? 10 + WEXITSTATUS(status) : 40);
            /* Echo and lines, the same characters: the terminal as it was found. */
            if (tcgetattr(terminal, &after) < 0) _exit(LEADER_NO_TERMINAL + 5);
            int same = after.c_lflag == before.c_lflag && after.c_iflag == before.c_iflag &&
                       after.c_oflag == before.c_oflag &&
                       memcmp(after.c_cc, before.c_cc, sizeof(before.c_cc)) == 0;
            _exit(same ? LEADER_RAN_WELL : LEADER_MODES_CHANGED);
        }
        close(text[0]);
        const char *words = "hello from a pipe\nand a second line\n";
        assert(write(text[1], words, strlen(words)) == (ssize_t)strlen(words));
        close(text[1]);

        static char output[1 << 21];
        size_t length = 0, drawn = 0, fed = 0;
        int status = 0, typed = 0, resized = 0;
        double drawn_at = 0;
        const char *taken = NULL;
        /* What a terminal shows, fed everything from the moment the screen is taken:
         * the words, at home, before any key. A slow machine takes longer to get
         * there, so it is watched for rather than looked at once at a set time. */
        vt_t shown;
        assert(vt_init(&shown, 80, 24) == 0);
        struct timespec began, now;
        clock_gettime(CLOCK_MONOTONIC, &began);
        for (;;) {
            if (readable_within(master, 50) > 0) {
                ssize_t got = read(master, output + length, sizeof(output) - 1 - length);
                if (got <= 0) break;
                length += (size_t)got;
                output[length] = '\0';
            }
            clock_gettime(CLOCK_MONOTONIC, &now);
            double since = elapsed_seconds(&began, &now);
            if (taken == NULL) taken = strstr(output, ALT_SCREEN_ON);
            if (!drawn && taken != NULL) {
                if (fed < (size_t)(taken - output)) fed = (size_t)(taken - output);
                vt_feed(&shown, output + fed, length - fed);
                fed = length;
                if (vt_shows(&shown, "hello from a pipe")) {
                    drawn = length;
                    drawn_at = since;
                }
            }
            /* Drawn, and then a bigger window, which it lays the text out again for. */
            if (resize && !resized && drawn) {
                struct winsize bigger = {
                    .ws_row = 30, .ws_col = 100, .ws_xpixel = 0, .ws_ypixel = 0};
                assert(ioctl(master, TIOCSWINSZ, &bigger) == 0);
                resized = 1;
            }
            if (!typed && drawn && since > drawn_at + (resize ? 1.0 : 0.2)) {
                assert(write(master, "q", 1) == 1);
                typed = 1;
            }
            if (waitpid(leader, &status, WNOHANG) == leader) {
                leader = -1;
                break;
            }
            assert(since < 30); /* It ends, and does not hang. */
        }
        vt_destroy(&shown);
        while (readable_within(master, 100) > 0) {
            ssize_t got = read(master, output + length, sizeof(output) - 1 - length);
            if (got <= 0) break;
            length += (size_t)got;
            output[length] = '\0';
        }
        if (leader > 0) assert(waitpid(leader, &status, 0) == leader);
        /* The words were on the screen, and only then was q typed. */
        assert(drawn && typed);
        /* The program went well, and the terminal is as it was. */
        assert(WIFEXITED(status) && WEXITSTATUS(status) == LEADER_RAN_WELL);
        /* It took the screen, drew the words, and gave the screen back, and nothing
         * about the terminal went wrong on the way. */
        assert(taken != NULL && drawn > (size_t)(taken - output));
        const char *given_back = NULL;
        for (const char *at = strstr(output, ALT_SCREEN_OFF); at != NULL;
             at = strstr(at + 1, ALT_SCREEN_OFF))
            given_back = at;
        assert(given_back != NULL && given_back > taken);
        assert(strstr(output, "ioctl") == NULL && strstr(output, "raw mode") == NULL);
        close(sizing);
        close(master);
    }
}

/* Text on a pipe with no terminal anywhere, as from a service or an editor's run
 * button: said in words, with a failed status, and not as a failed ioctl. */
static void test_text_on_a_pipe_with_no_terminal_says_so(void) {
    int text[2], said[2];
    assert(pipe(text) == 0 && pipe(said) == 0);
    fflush(NULL);
    pid_t child = fork();
    assert(child >= 0);
    if (child == 0) {
        alarm(20);
        close(text[1]);
        close(said[0]);
        if (setsid() < 0) _exit(90); /* No controlling terminal. */
        if (dup2(text[0], STDIN_FILENO) < 0 || dup2(said[1], STDOUT_FILENO) < 0 ||
            dup2(said[1], STDERR_FILENO) < 0)
            _exit(92);
        reset_sign_state();
        render_mode = RENDER_UNSET;
        letters_mode = 0;
        text_path = NULL;
        forget_the_text();
        input_fd = STDIN_FILENO;
        terminal_is_raw = terminal_restored = alt_screen_is_on = sprites_uploaded = 0;
        char *argv[] = {"cbirds", NULL};
        exit(cbirds_application_main(1, argv));
    }
    close(text[0]);
    close(said[1]);
    assert(write(text[1], "hello\n", 6) == 6);
    close(text[1]);
    char message[1024];
    size_t length = 0;
    ssize_t got;
    while ((got = read(said[0], message + length, sizeof(message) - 1 - length)) > 0)
        length += (size_t)got;
    message[length] = '\0';
    close(said[0]);
    int status = 0;
    assert(waitpid(child, &status, 0) == child);
    assert(WIFEXITED(status) && WEXITSTATUS(status) == EXIT_FAILURE);
    assert(strstr(message, "needs a terminal") != NULL && strstr(message, "/dev/tty") != NULL);
    assert(strstr(message, "ioctl") == NULL);
}

/* A run that reads text from standard input and also asks for a sign. */
static int status_of_text_piped_in_with(int argc, char **argv, const char *file) {
    fflush(NULL);
    pid_t child = fork();
    assert(child >= 0);
    if (child == 0) {
        alarm(20);
        int quiet = open("/dev/null", O_WRONLY), fd = open(file, O_RDONLY);
        if (quiet < 0 || fd < 0 || dup2(quiet, STDERR_FILENO) < 0 || dup2(fd, STDIN_FILENO) < 0)
            _exit(99);
        read_options(argc, argv);
        _exit(take_the_text(60, 12, 1) ? 0 : 3); /* A flock goes on when there was no text. */
    }
    int status = 0;
    assert(waitpid(child, &status, 0) == child);
    return WIFEXITED(status) ? WEXITSTATUS(status) : -1;
}

/* Text is the flock, and a sign is what the flock writes: a text leaves nobody to
 * write a sign. Refused as every usage error is, in either order on the line, and
 * for text on a pipe when it is found; a pipe with nothing in it is no text, and
 * the sign goes on. */
static void test_a_sign_is_refused_with_text(void) {
    char text[600], empty[600], picture[600];
    scratch_file(text, sizeof(text), "signs_refuse.txt");
    scratch_file(empty, sizeof(empty), "signs_refuse_empty.txt");
    world_write(text, "hello, world\n");
    world_write(empty, "");
    write_a_picture("refuse.png", 1, 255);
    scratch_file(picture, sizeof(picture), "refuse.png");
    const char *signs[][2] = {{"--say", "hi"},
                              {"--clock", NULL},
                              {"--clock-at", "10:00"},
                              {"--seconds", NULL},
                              {"--picture", NULL}};
    for (size_t k = 0; k < sizeof(signs) / sizeof(*signs); k++) {
        const char *value = signs[k][1];
        if (strcmp(signs[k][0], "--picture") == 0) value = picture;
        char *first[7] = {"cbirds", (char *)signs[k][0]},
             *last[7] = {"cbirds", "--text", text, (char *)signs[k][0]};
        int n = 2, m = 4;
        if (value != NULL) first[n++] = (char *)value, last[m++] = (char *)value;
        first[n++] = "--text";
        first[n++] = text;
        first[n] = last[m] = NULL;
        assert(exit_status_of(n, first) == EXIT_USAGE);
        assert(exit_status_of(m, last) == EXIT_USAGE);
        char *alone[4] = {"cbirds", (char *)signs[k][0], (char *)value, NULL};
        int count = value != NULL ? 3 : 2;
        assert(status_of_text_piped_in_with(count, alone, text) == EXIT_USAGE);
        assert(status_of_text_piped_in_with(count, alone, empty) == 3);
    }
    assert(unlink(text) == 0 && unlink(empty) == 0 && unlink(picture) == 0);
    reset_sign_state();
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

/* --- --text, and the runs that take text without a terminal ---------------------- */

/* What a child wrote to the descriptor it was given, to its end. */
static void read_to_the_end(int fd, char *out, size_t size) {
    size_t length = 0;
    ssize_t got;
    while (length + 1 < size && (got = read(fd, out + length, size - 1 - length)) > 0)
        length += (size_t)got;
    out[length] = '\0';
}

/* A child about to run as a shell starts one: nothing the tests before it left. */
static void as_a_fresh_run(void) {
    reset_sign_state();
    render_mode = RENDER_UNSET;
    letters_mode = 0;
    text_path = NULL;
    record_path = NULL;
    record_seconds = 0;
    bench_frames = 0;
    forget_the_text();
    input_fd = STDIN_FILENO;
    terminal_is_raw = terminal_restored = alt_screen_is_on = sprites_uploaded = 0;
}

/* --text - names standard input: what is there is the text even where a pipe on
 * its own would not count, as in a benchmark. A terminal there has no text in it,
 * and is a usage error. */
static void test_text_dash_is_standard_input_by_name(void) {
    reset_sign_state();
    const char *words = "hello from standard input\n";
    int saved = dup(STDIN_FILENO), text[2];
    assert(saved >= 0 && pipe(text) == 0);
    assert(write(text[1], words, strlen(words)) == (ssize_t)strlen(words));
    close(text[1]);
    assert(dup2(text[0], STDIN_FILENO) == STDIN_FILENO);
    close(text[0]);
    text_path = "-";
    int taken = take_the_text(60, 12, 0);
    assert(dup2(saved, STDIN_FILENO) == STDIN_FILENO);
    close(saved);
    assert(taken == 1 && letters_mode);
    assert(the_letters.count == 22); /* Every letter of it, and no blank. */
    assert(the_text_length == strlen(words) && memcmp(the_text, words, strlen(words)) == 0);
    letters_destroy(&the_letters);
    letters_mode = 0;
    text_path = NULL;
    forget_the_text();
    reset_test_config();

    int master = posix_openpt(O_RDWR | O_NOCTTY);
    assert(master >= 0 && grantpt(master) == 0 && unlockpt(master) == 0);
    const char *name = ptsname(master);
    assert(name != NULL);
    int said[2];
    assert(pipe(said) == 0);
    fflush(NULL);
    pid_t child = fork();
    assert(child >= 0);
    if (child == 0) {
        alarm(20);
        close(said[0]);
        int terminal = open(name, O_RDWR | O_NOCTTY);
        if (terminal < 0 || dup2(terminal, STDIN_FILENO) < 0 || dup2(said[1], STDERR_FILENO) < 0)
            _exit(92);
        char *argv[] = {"cbirds", "--text", "-", NULL};
        read_options(3, argv);
        take_the_text(60, 12, 1);
        _exit(0);
    }
    close(said[1]);
    char message[512];
    read_to_the_end(said[0], message, sizeof(message));
    close(said[0]);
    int status = 0;
    assert(waitpid(child, &status, 0) == child);
    assert(WIFEXITED(status) && WEXITSTATUS(status) == EXIT_USAGE);
    assert(strstr(message, "--text -") != NULL && strstr(message, "terminal") != NULL);
    close(master);
}

/* A file --text cannot open is the mistake, and is said before anything else is
 * looked for, the terminal included; one that opens and cannot be read, as a
 * directory, is said as well. Neither is a flock of birds instead. */
static void test_a_text_file_that_cannot_be_had_is_said_and_is_not_birds(void) {
    char missing[600], folder[600];
    scratch_file(missing, sizeof(missing), "not_there.txt");
    scratch_file(folder, sizeof(folder), "a_folder");
    assert(mkdir(folder, 0700) == 0);
    for (int k = 0; k < 2; k++) {
        const char *path = k == 0 ? missing : folder;
        int said[2];
        assert(pipe(said) == 0);
        fflush(NULL);
        pid_t child = fork();
        assert(child >= 0);
        if (child == 0) {
            alarm(20);
            close(said[0]);
            int none = open("/dev/null", O_RDWR);
            if (none < 0 || dup2(none, STDIN_FILENO) < 0 || dup2(none, STDOUT_FILENO) < 0 ||
                dup2(said[1], STDERR_FILENO) < 0)
                _exit(92);
            as_a_fresh_run();
            char *argv[] = {"cbirds", "--text", (char *)path, NULL};
            if (k == 0) {
                /* With no terminal at all: the file is still what is said. */
                if (setsid() < 0) _exit(90);
                exit(cbirds_application_main(3, argv));
            }
            read_options(3, argv);
            exit(take_the_text(60, 12, 1) ? 0 : 3);
        }
        close(said[1]);
        char message[1024];
        read_to_the_end(said[0], message, sizeof(message));
        close(said[0]);
        int status = 0;
        assert(waitpid(child, &status, 0) == child);
        assert(WIFEXITED(status) && WEXITSTATUS(status) == EXIT_FAILURE);
        assert(strstr(message, k == 0 ? "cannot open" : "cannot read") != NULL);
        assert(strstr(message, path) != NULL && strstr(message, "terminal") == NULL);
    }
    assert(rmdir(folder) == 0);
}

/* A benchmark measures the flock it is told to, and a pipe it happens to be in is
 * not asked for: what is in it is left there, for nobody, and the run is of birds.
 * Through main, as a script runs `cbirds --bench`. */
static void test_a_benchmark_never_reads_a_pipe_it_is_in(void) {
    int text[2], said[2];
    assert(pipe(text) == 0 && pipe(said) == 0);
    const char *words = "hello\n";
    assert(write(text[1], words, strlen(words)) == (ssize_t)strlen(words));
    fflush(NULL);
    pid_t child = fork();
    assert(child >= 0);
    if (child == 0) {
        alarm(20);
        close(text[1]);
        close(said[0]);
        int quiet = open("/dev/null", O_WRONLY);
        if (quiet < 0 || dup2(text[0], STDIN_FILENO) < 0 || dup2(said[1], STDOUT_FILENO) < 0 ||
            dup2(quiet, STDERR_FILENO) < 0)
            _exit(92);
        as_a_fresh_run();
        char *argv[] = {"cbirds", "--bench", "10", NULL};
        exit(cbirds_application_main(3, argv));
    }
    close(said[1]);
    static char out[4096];
    read_to_the_end(said[0], out, sizeof(out));
    close(said[0]);
    int status = 0;
    assert(waitpid(child, &status, 0) == child);
    assert(WIFEXITED(status) && WEXITSTATUS(status) == EXIT_SUCCESS);
    assert(strstr(out, "birds ") != NULL && strstr(out, "letters ") == NULL);
    char left[16];
    assert(fcntl(text[0], F_SETFL, O_NONBLOCK) == 0);
    assert(read(text[0], left, sizeof(left)) == (ssize_t)strlen(words)); /* Still there. */
    close(text[0]);
    close(text[1]);
}

/* The bytes of a JSON string as a cast writes one, `from` at its opening quote, out
 * of their quotes and escapes. Returns how many. */
static size_t json_string_bytes(const char *from, char *out, size_t size) {
    size_t length = 0;
    assert(*from == '"');
    for (const char *at = from + 1; *at != '"' && *at != '\0'; at++) {
        char c = *at;
        if (c == '\\') {
            at++;
            if (*at == 'u') {
                unsigned value = 0;
                assert(sscanf(at + 1, "%4x", &value) == 1 && value < 0x80);
                c = (char)value;
                at += 4;
            } else {
                c = *at; /* \" and \\, the only others a cast of ours has. */
            }
        }
        assert(length + 1 < size);
        out[length++] = c;
    }
    return length;
}

/* Whether what a terminal shows is the text as the command printed it: every glyph,
 * its width, its colours and attributes, and every background. */
static int the_screen_is_the_text(const vt_t *shown, const char *text) {
    vt_t original;
    assert(vt_init(&original, shown->cols, shown->rows) == 0);
    vt_feed(&original, text, strlen(text));
    vt_finish(&original);
    int same = 1;
    for (int row = 0; row < shown->rows; row++)
        for (int col = 0; col < shown->cols; col++) {
            const vt_cell_t *a = vt_cell(&original, col, row), *b = vt_cell(shown, col, row);
            if ((a->glyph == 0 ? ' ' : a->glyph) != (b->glyph == 0 ? ' ' : b->glyph) ||
                a->width != b->width ||
                (!vt_cell_is_blank(a) && memcmp(&a->style, &b->style, sizeof(vt_style_t)) != 0) ||
                memcmp(&a->style.bg, &b->style.bg, sizeof(vt_colour_t)) != 0)
                same = 0;
        }
    vt_destroy(&original);
    return same;
}

/* Runs main as the shell would for `TEXT | cbirds --seed SEED --record PATH`, at ten
 * frames a second, which is enough to see where every letter is. */
static void record_text_from_a_pipe(const char *text, const char *seed, const char *path) {
    int pipe_in[2];
    assert(pipe(pipe_in) == 0);
    fflush(NULL);
    pid_t child = fork();
    assert(child >= 0);
    if (child == 0) {
        alarm(120);
        close(pipe_in[1]);
        int quiet = open("/dev/null", O_WRONLY);
        if (quiet < 0 || dup2(pipe_in[0], STDIN_FILENO) < 0 || dup2(quiet, STDOUT_FILENO) < 0 ||
            dup2(quiet, STDERR_FILENO) < 0)
            _exit(92);
        as_a_fresh_run();
        char *argv[] = {"cbirds",        "--seed", (char *)seed,   "--record", (char *)path,
                        "--record-size", "60x14",  "--record-fps", "10",       NULL};
        exit(cbirds_application_main(9, argv));
    }
    close(pipe_in[0]);
    assert(write(pipe_in[1], text, strlen(text)) == (ssize_t)strlen(text));
    close(pipe_in[1]);
    int status = 0;
    assert(waitpid(child, &status, 0) == child);
    assert(WIFEXITED(status) && WEXITSTATUS(status) == EXIT_SUCCESS);
}

/* A recording of the flock is six seconds unless told, and one of text a whole
 * cycle, the longest there can be: the first rest, a wave cut short at its
 * deadline, the longest flight and the slowest homing. A shorter cycle lands
 * sooner, and a recording does not start a wave it has no time to finish, so it
 * opens and it ends on the text as the command printed it, and a GIF of it loops
 * without a jump. Through main, with the text on a pipe, as `fastfetch | cbirds
 * --record fetch.cast` runs, for texts and seeds that land early and late. */
static void test_a_recording_of_text_is_a_whole_cycle_unless_told(void) {
    assert(TEXT_RECORD_SECONDS >= LETTERS_FIRST_REST + LETTERS_WAVE_DEADLINE + LETTERS_FLIGHT_MOST +
                                      LETTERS_HOMING_DEADLINE);
    int was = record_seconds;
    record_seconds = 0;
    letters_mode = 0;
    settle_the_recording_length();
    assert(record_seconds == FLOCK_RECORD_SECONDS);
    record_seconds = 0;
    letters_mode = 1;
    settle_the_recording_length();
    assert(record_seconds == TEXT_RECORD_SECONDS);
    record_seconds = 3;
    settle_the_recording_length();
    assert(record_seconds == 3); /* Told. */
    letters_mode = 0;
    record_seconds = was;

    static const char *const TEXTS[] = {NEOFETCH_LIKE, "hello world\nsecond line\n"};
    static const char *const SEEDS[] = {"1", "2", "3", "4"};
    char path[600];
    scratch_file(path, sizeof(path), "whole_cycle.cast");
    static char line[1 << 18], bytes[1 << 18];
    for (size_t t = 0; t < sizeof(TEXTS) / sizeof(*TEXTS); t++)
        for (size_t s = 0; s < sizeof(SEEDS) / sizeof(*SEEDS); s++) {
            record_text_from_a_pipe(TEXTS[t], SEEDS[s], path);
            FILE *file = fopen(path, "r");
            assert(file != NULL);
            assert(fgets(line, sizeof(line), file) != NULL);
            assert(strstr(line, "\"width\": 60, \"height\": 14") != NULL);
            vt_t shown;
            assert(vt_init(&shown, 60, 14) == 0);
            double last = 0;
            int events = 0, flew = 0;
            while (fgets(line, sizeof(line), file) != NULL) {
                assert(strchr(line, '\n') != NULL); /* Each event whole. */
                last = strtod(line + 1, NULL);
                const char *output = strstr(line, "\"o\", ");
                assert(output != NULL);
                vt_feed(&shown, bytes, json_string_bytes(output + 5, bytes, sizeof(bytes)));
                /* The opening, and then the first frame: the text at rest. */
                if (++events == 2) assert(the_screen_is_the_text(&shown, TEXTS[t]));
                /* Ten seconds in, it is flying: the first rest is never held. */
                if (last > 9.95 && last < 10.05) flew = !the_screen_is_the_text(&shown, TEXTS[t]);
            }
            fclose(file);
            assert(remove(path) == 0);
            assert(flew);
            assert(events == 10 * TEXT_RECORD_SECONDS + 2);
            assert(last >= TEXT_RECORD_SECONDS - 0.05);
            assert(the_screen_is_the_text(&shown, TEXTS[t]));
            vt_destroy(&shown);
        }
}

/* What the README says of a pipe: three seconds for something to come, a second and
 * a half of quiet, eight seconds in all, a megabyte. letters_test tries the reading
 * with shorter numbers; these are the ones a run uses. */
static void test_a_pipe_is_read_for_as_long_as_the_readme_says(void) {
    assert(PIPE_READING.first_byte_wait == 3.0);
    assert(PIPE_READING.quiet == 1.5);
    assert(PIPE_READING.patience == 8.0);
    assert(PIPE_READING.limit == 1 << 20);
}

/* --text with a named pipe, as from a command's output given a name: it is opened
 * once, by main, and read from that descriptor. A second open would find the
 * writer gone, what it wrote lost with it, and wait for ever for another. On a
 * terminal of its own, with a writer that writes once and closes at once, the run
 * takes the text and ends, every time. */
static void test_text_from_a_named_pipe_is_opened_once(void) {
    char fifo[600];
    scratch_file(fifo, sizeof(fifo), "text.fifo");
    assert(mkfifo(fifo, 0600) == 0);
    for (int run_number = 0; run_number < 3; run_number++) {
        int master = posix_openpt(O_RDWR | O_NOCTTY);
        assert(master >= 0 && grantpt(master) == 0 && unlockpt(master) == 0);
        const char *name = ptsname(master);
        assert(name != NULL);
        int sizing = open(name, O_RDWR | O_NOCTTY);
        assert(sizing >= 0);
        struct winsize size = {.ws_row = 24, .ws_col = 80, .ws_xpixel = 0, .ws_ypixel = 0};
        assert(ioctl(sizing, TIOCSWINSZ, &size) == 0);
        fflush(NULL);
        pid_t run = fork();
        assert(run >= 0);
        if (run == 0) {
            alarm(10);
            close(master);
            close(sizing);
            if (setsid() < 0) _exit(90);
            int terminal = open(name, O_RDWR);
            if (terminal < 0 || ioctl(terminal, TIOCSCTTY, 0) < 0) _exit(91);
            if (dup2(terminal, STDIN_FILENO) < 0 || dup2(terminal, STDOUT_FILENO) < 0 ||
                dup2(terminal, STDERR_FILENO) < 0)
                _exit(92);
            close(terminal);
            as_a_fresh_run();
            char *argv[] = {"cbirds", "--text", fifo, "--frames", "2", NULL};
            exit(cbirds_application_main(5, argv));
        }
        pid_t writer = fork();
        assert(writer >= 0);
        if (writer == 0) {
            alarm(10);
            signal(SIGPIPE, SIG_DFL);
            int fd = open(fifo, O_WRONLY);
            if (fd < 0) _exit(93);
            const char *words = "hello from a named pipe\n";
            int wrote = write(fd, words, strlen(words)) == (ssize_t)strlen(words);
            close(fd);
            _exit(wrote ? 0 : 94);
        }
        static char output[1 << 20];
        size_t length = 0;
        int status = 0;
        for (;;) {
            if (readable_within(master, 50) > 0) {
                ssize_t got = read(master, output + length, sizeof(output) - 1 - length);
                if (got > 0) length += (size_t)got;
            }
            if (waitpid(run, &status, WNOHANG) == run) break;
        }
        output[length] = '\0';
        int wrote = 0;
        assert(waitpid(writer, &wrote, 0) == writer);
        /* The writer was read, and the run ended, and not by the alarm a second open
         * waiting for ever would meet. */
        assert(WIFEXITED(wrote) && WEXITSTATUS(wrote) == 0);
        assert(WIFEXITED(status) && WEXITSTATUS(status) == EXIT_SUCCESS);
        /* And it was the text that flew, not the birds that nothing to read gives. */
        const char *taken = strstr(output, ALT_SCREEN_ON);
        assert(taken != NULL);
        vt_t shown;
        assert(vt_init(&shown, 80, 24) == 0);
        vt_feed(&shown, taken, length - (size_t)(taken - output));
        assert(vt_shows(&shown, "hello from a named pipe"));
        vt_destroy(&shown);
        close(sizing);
        close(master);
    }
    assert(unlink(fifo) == 0);

    /* The reading takes the descriptor main opened and not the name again: with the
     * name gone in between, the text is read all the same. */
    char gone[600];
    scratch_file(gone, sizeof(gone), "gone.txt");
    world_write(gone, "still here\n");
    reset_sign_state();
    text_path = gone;
    text_fd = open_the_text_file();
    assert(unlink(gone) == 0);
    assert(take_the_text(40, 10, 0) == 1 && text_fd == -1);
    assert(the_letters.count == 9);
    letters_destroy(&the_letters);
    letters_mode = 0;
    text_path = NULL;
    forget_the_text();
    reset_test_config();
}

/* What is for birds is not for text: the rain of --matrix, with its green ramp and
 * its alignment; a sprite of somebody's own; more flocks, a second sky and tails;
 * kitty's sprites. The letters fly in the ramp that was asked for, as text. */
static void test_what_is_for_birds_does_not_apply_to_text(void) {
    char path[600], sprite[600];
    scratch_file(path, sizeof(path), "for_birds.txt");
    world_write(path, "hello\n");
    write_a_picture("for_birds.png", 1, 255);
    scratch_file(sprite, sizeof(sprite), "for_birds.png");
    reset_sign_state();
    int alignment = config.alignment_notch;
    char *argv[] = {"cbirds", "--color",  "ice",   "--matrix", "--sprite", sprite, "--depth",
                    "-e",     "--render", "kitty", "--text",   path,       NULL};
    read_options(12, argv);
    config.flocks = 3;
    assert(config.palette == palette_named("matrix") && the_rain_is_falling);
    assert(take_the_text(40, 10, 0) == 1);
    assert(config.palette == palette_named("ice") && config.alignment_notch == alignment);
    assert(!the_rain_is_falling && !config.trails && config.flocks == 1 && !deep_look);
    assert(sprite_path == NULL && palette_shades() == palette()->shades);
    assert(live_render_mode() == RENDER_BRAILLE);
    letters_destroy(&the_letters);
    letters_mode = 0;
    text_path = NULL;
    forget_the_text();
    render_mode = RENDER_KITTY;
    assert(remove(path) == 0 && remove(sprite) == 0);
    reset_sign_state();
}

/* More letters than there can be birds: a screenful of a large terminal is more
 * than the flock's own limit, and the letters' limit is the one that holds. With
 * hawks and the panel over them, they go and every one comes home. */
static void test_more_letters_than_birds_go_and_come_home(void) {
    enum { COLS = 100, ROWS = 50 };
    static char text[(COLS + 1) * ROWS + 1];
    size_t at = 0;
    for (int row = 0; row < ROWS; row++) {
        for (int col = 0; col < COLS; col++) text[at++] = (char)('a' + (row + col) % 26);
        if (row + 1 < ROWS) text[at++] = '\n';
    }
    text[at] = '\0';
    world_t world;
    world_open(&world, text, COLS, ROWS);
    assert(config.birds == COLS * ROWS && config.birds > MAX_BIRDS);
    config.hawks = 2;
    place_hawks();
    legend_enabled = 1;
    measure_legend();
    update_turn_distances();
    feed_input("\r");
    assert(world_run_until(&world, LETTERS_IN_FLIGHT, 60 * 6) > 0);
    for (int frame = 0; frame < 30; frame++) world_step(&world);
    feed_input("\r");
    int frames = 0;
    while (the_letters.phase != LETTERS_AT_REST && frames < 60 * 8) {
        world_step(&world);
        frames++;
    }
    assert(the_letters.phase == LETTERS_AT_REST);
    config.hawks = 0; /* A hawk over a letter at home lifts it: counted without them. */
    assert(world_all_home(&world));
    int count;
    free(world_picture(&world, &count));
    world_close(&world);
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
    test_a_clock_with_seconds_ticks_a_digit_at_a_time();
    test_the_hour_lets_go_of_a_clock_with_seconds();
    test_seconds_is_a_clock();
    test_the_font_size_is_rows_from_four_to_ten();
    test_the_letters_are_a_seventh_of_the_rows_unless_told();
    test_a_sign_is_as_tall_as_its_font_where_it_fits();
    test_a_clock_with_seconds_fits_the_screens_a_clock_does();
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
    test_a_screensaver_does_not_quit_for_a_late_reply_and_does_for_a_key();
    test_the_options_that_make_a_sign();
    test_a_picture_gives_every_bird_a_place_and_a_colour();
    test_a_picture_wears_a_ramp_somebody_chose();
    test_a_picture_in_black_is_not_a_picture_of_nothing();
    test_a_large_picture_is_kept_small();
    test_a_picture_that_cannot_be_drawn_says_so();
    test_a_colour_given_is_told_from_the_default();
    test_a_hawk_scatters_the_places_along_the_step_it_flew();
    test_one_hawk_leaves_a_sign_readable();
    test_a_hawk_diving_through_the_letters_scatters_them_at_any_frame_rate();
    test_a_small_screen_leaves_the_flock_sky_and_a_roomy_one_is_as_it_was();
    test_the_free_flock_is_turned_round_a_sign();
    test_the_free_flock_wheels_round_a_sign_one_way();
    test_a_sign_that_cannot_be_laid_out_says_so_when_the_run_is_over();
    test_a_text_too_big_for_the_terminal_is_said_after_the_terminal_is_given_back();
    test_a_snapshot_is_reported_after_the_terminal_is_given_back();
    test_a_colour_reply_is_whole_at_its_terminator_and_not_at_a_letter_in_it();
    test_a_late_reply_to_an_earlier_question_is_not_taken_for_this_one();
    test_a_reply_that_comes_late_is_not_read_as_keys();
    test_a_string_the_key_reader_cannot_end_is_given_up();
    test_the_rest_of_a_reply_that_was_cut_off_by_the_deadline_is_thrown_away();
    test_a_sign_has_a_bird_as_wide_as_its_cells_unless_it_is_told();
    test_a_sign_records_in_a_gif_and_a_cast();
    test_a_sign_survives_the_flock_growing_under_it();
    test_presets_set_every_notch();
    test_a_notch_survives_the_round_trip();
    test_the_pointer_moves_the_flock();
    test_the_short_help_is_one_screen_at_eighty_columns();
    test_a_pointer_reported_beyond_the_screen_is_at_its_edge();
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
    test_a_window_with_none_of_the_text_left_keeps_the_letters_and_their_size();
    test_a_hawk_over_the_text_does_not_stop_the_cycle();
    test_a_pointer_at_the_limits_does_not_disturb_the_text();
    test_a_screensaver_reads_the_descriptor_that_was_chosen();
    test_a_screensaver_is_a_lock_screen_for_every_mode();
    test_text_on_a_pipe_flies_on_a_terminal_and_gives_it_back();
    test_text_on_a_pipe_with_no_terminal_says_so();
    test_a_sign_is_refused_with_text();
    test_keys_and_colour_questions_use_the_descriptor_that_was_chosen();
    test_text_dash_is_standard_input_by_name();
    test_a_text_file_that_cannot_be_had_is_said_and_is_not_birds();
    test_a_benchmark_never_reads_a_pipe_it_is_in();
    test_a_recording_of_text_is_a_whole_cycle_unless_told();
    test_a_pipe_is_read_for_as_long_as_the_readme_says();
    test_text_from_a_named_pipe_is_opened_once();
    test_what_is_for_birds_does_not_apply_to_text();
    test_more_letters_than_birds_go_and_come_home();
    /* Every test removes what it wrote, so this fails if one did not. */
    assert(rmdir(scratch) == 0);
    return 0;
}
