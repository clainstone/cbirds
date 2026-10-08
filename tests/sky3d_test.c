#include "../sky3d.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* A small deterministic generator of the test's own, so that what is built here
 * does not depend on the one in the module, whose draws are what is under test. */
static uint64_t scrap = 88172645463325252ull;

static double scrap_unit(void) {
    scrap ^= scrap << 13;
    scrap ^= scrap >> 7;
    scrap ^= scrap << 17;
    return (double)(scrap >> 11) / 9007199254740992.0;
}

static double wrap(double angle) {
    angle = fmod(angle + M_PI, 2 * M_PI);
    if (angle < 0) angle += 2 * M_PI;
    return angle - M_PI;
}

static void fill_cloud(sky_t *sky, int count, double width, double depth) {
    for (int i = 0; i < count; i++) {
        sky_bird_t *bird = &sky->birds[i];
        memset(bird, 0, sizeof(*bird));
        bird->x = (scrap_unit() - 0.5) * width;
        bird->y = (scrap_unit() - 0.5) * width;
        bird->z = (scrap_unit() - 0.5) * depth;
        bird->hx = 1;
        bird->speed = 10;
    }
}

/* What the index is meant to be: every bird looked at, sorted, the first k within
 * reach. The squared distances are computed the same way the module computes them,
 * so a tie is a tie in both and the order of ties is the order of indices. */
static int brute_force(const sky_t *sky, int count, int self, int k, double reach, int *index,
                       double *squared) {
    typedef struct {
        double squared;
        int index;
    } candidate_t;
    candidate_t *all = malloc(sizeof(*all) * (size_t)count);
    int found = 0;
    assert(all != NULL);
    for (int i = 0; i < count; i++) {
        if (i == self) continue;
        double dx = sky->birds[i].x - sky->birds[self].x;
        double dy = sky->birds[i].y - sky->birds[self].y;
        double dz = sky->birds[i].z - sky->birds[self].z;
        double d = dx * dx + dy * dy + dz * dz;
        if (d > reach * reach) continue;
        all[found++] = (candidate_t){d, i};
    }
    for (int a = 1; a < found; a++) {
        candidate_t held = all[a];
        int b = a;
        while (b > 0 && (all[b - 1].squared > held.squared ||
                         (all[b - 1].squared == held.squared && all[b - 1].index > held.index))) {
            all[b] = all[b - 1];
            b--;
        }
        all[b] = held;
    }
    if (found > k) found = k;
    for (int n = 0; n < found; n++) {
        index[n] = all[n].index;
        squared[n] = all[n].squared;
    }
    free(all);
    return found;
}

static void agree_with_brute_force(sky_t *sky, int count, int k, double reach) {
    int near[SKY_MAX_NEIGHBOURS], expected[SKY_MAX_NEIGHBOURS];
    double squared[SKY_MAX_NEIGHBOURS], expected_squared[SKY_MAX_NEIGHBOURS];
    assert(sky_index(sky, count) == SKY_OK);
    for (int i = 0; i < count; i++) {
        int got = sky_neighbours(sky, i, k, reach, near, squared);
        int wanted = brute_force(sky, count, i, k, reach, expected, expected_squared);
        assert(got == wanted);
        for (int n = 0; n < got; n++) {
            assert(near[n] == expected[n]);
            assert(squared[n] == expected_squared[n]);
        }
    }
}

/* The seven nearest are the seven nearest, wherever the birds are and however the
 * index is cut: a cloud, a crowd, a sheet, a lattice where every distance is a tie,
 * a pile of birds on one point, a bird alone, and cells from far smaller than a
 * bird's neighbours to far larger than the flock. A search that was only usually
 * right would be a flock that was only usually one. */
static void test_the_nearest_birds_are_the_ones_a_search_of_everything_finds(void) {
    static const int KS[] = {1, 3, 7, 13, SKY_MAX_NEIGHBOURS};
    static const double REACHES[] = {1.5, 6, 1e9};
    static const double CELLS[] = {0.3, 1.1, 4, 60};
    sky_t sky;

    assert(sky_init(&sky, 500, 1) == SKY_OK);
    for (int shape = 0; shape < 5; shape++) {
        int count = 400;
        if (shape == 0) fill_cloud(&sky, count, 30, 30);
        if (shape == 1) fill_cloud(&sky, count, 4, 4);
        if (shape == 2) fill_cloud(&sky, count, 40, 0.5);
        if (shape == 3) {
            /* Every bird on a lattice a metre apart, in a block seven by seven by
             * eight: many distances equal, which only the index order can settle. */
            for (int i = 0; i < count; i++) {
                memset(&sky.birds[i], 0, sizeof(sky.birds[i]));
                sky.birds[i].x = i % 7;
                sky.birds[i].y = (i / 7) % 7;
                sky.birds[i].z = i / 49;
            }
        }
        if (shape == 4) {
            /* Eight piles of fifty birds, each exactly on one point. */
            for (int i = 0; i < count; i++) {
                memset(&sky.birds[i], 0, sizeof(sky.birds[i]));
                sky.birds[i].x = (i % 8) * 3.0;
                sky.birds[i].y = (i % 8) * -2.0;
            }
        }
        for (size_t c = 0; c < sizeof(CELLS) / sizeof(*CELLS); c++) {
            sky.cell = CELLS[c];
            for (size_t k = 0; k < sizeof(KS) / sizeof(*KS); k++)
                for (size_t r = 0; r < sizeof(REACHES) / sizeof(*REACHES); r++)
                    agree_with_brute_force(&sky, count, KS[k], REACHES[r]);
        }
    }
    /* One bird has no neighbours, and two have each other. */
    fill_cloud(&sky, 2, 5, 5);
    sky.cell = 2;
    agree_with_brute_force(&sky, 1, 7, 1e9);
    agree_with_brute_force(&sky, 2, 7, 1e9);
    sky_destroy(&sky);
}

/* A cloud so wide that a cell as fine as the flock wants would be more cells than
 * the index may have is cut coarser rather than refused, and the answer is the
 * same. */
static void test_an_index_too_fine_for_the_sky_is_cut_coarser(void) {
    sky_t sky;

    assert(sky_init(&sky, 300, 2) == SKY_OK);
    fill_cloud(&sky, 300, 100000, 100000);
    sky.cell = 0.4;
    agree_with_brute_force(&sky, 300, 7, 1e9);
    assert(sky.grid.cells <= (1 << 18));
    assert(sky.grid.cell > 0.4);
    sky_destroy(&sky);
}

static double polarisation(const sky_t *sky, int count) {
    double x = 0, y = 0, z = 0;
    for (int i = 0; i < count; i++) {
        x += sky->birds[i].hx;
        y += sky->birds[i].hy;
        z += sky->birds[i].hz;
    }
    return sqrt(x * x + y * y + z * z) / count;
}

static void test_a_flock_is_born_flying_together(void) {
    enum { COUNT = 300 };
    sky_t sky;

    assert(sky_init(&sky, 2 * COUNT, 3) == SKY_OK);
    sky_populate(&sky, 0, COUNT, 0);
    assert(polarisation(&sky, COUNT) > 0.85);
    double centre[3], radius;
    sky_measure(&sky, COUNT, centre, &radius);
    assert(fabs(centre[0]) < 2 && fabs(centre[1]) < 2 && fabs(centre[2]) < 1);
    assert(radius > 3 && radius < 20);
    for (int i = 0; i < COUNT; i++) {
        double length = sqrt(sky.birds[i].hx * sky.birds[i].hx + sky.birds[i].hy * sky.birds[i].hy +
                             sky.birds[i].hz * sky.birds[i].hz);
        assert(fabs(length - 1) < 1e-12);
        assert(sky.birds[i].speed > 8 && sky.birds[i].speed < 12);
    }
    /* A bird added later joins one that is flying, a few metres from it and going
     * the same way, and the birds already flying are not touched. */
    sky_bird_t before = sky.birds[17];
    sky_populate(&sky, COUNT, 20, 1);
    assert(memcmp(&before, &sky.birds[17], sizeof(before)) == 0);
    double agreement = 0;
    for (int i = COUNT; i < COUNT + 20; i++) {
        double nearest = 1e9;
        double heading = 0;
        for (int j = 0; j < COUNT; j++) {
            double dx = sky.birds[i].x - sky.birds[j].x, dy = sky.birds[i].y - sky.birds[j].y,
                   dz = sky.birds[i].z - sky.birds[j].z;
            double d = sqrt(dx * dx + dy * dy + dz * dz);
            if (d < nearest) {
                nearest = d;
                heading = sky.birds[i].hx * sky.birds[j].hx + sky.birds[i].hy * sky.birds[j].hy +
                          sky.birds[i].hz * sky.birds[j].hz;
            }
        }
        assert(nearest < 4);
        agreement += heading / 20;
    }
    /* Not exactly a bird's own heading, which is a flock's to the nearest few
     * degrees, but near it. */
    assert(agreement > 0.8);
    sky_destroy(&sky);
}

/* A roost holds a flock: nothing carries it off, whatever the air does and
 * whichever way it happened to be born flying. Four hundred birds for a minute
 * and a half, at three seeds: the flock's middle never strays far, the birds are
 * hardly ever far from the roost, and none goes up or down far from its height. */
static void test_the_flock_stays_within_reach_of_its_roost(void) {
    enum { COUNT = 400, STEPS = 1800 };
    sky_rules_t rules = sky_default_rules();

    for (uint32_t seed = 1; seed <= 3; seed++) {
        sky_t sky;
        long outside = 0, sampled = 0;
        double worst_middle = 0, worst_height = 0, farthest = 0;
        assert(sky_init(&sky, COUNT, seed) == SKY_OK);
        sky_populate(&sky, 0, COUNT, 0);
        for (int step = 0; step < STEPS; step++) {
            sky_step(&sky, COUNT, &rules, NULL, 0.05);
            if (step % 20 != 0 || step < 100) continue;
            double centre[3], radius;
            sky_measure(&sky, COUNT, centre, &radius);
            double middle = sqrt(centre[0] * centre[0] + centre[1] * centre[1]);
            if (middle > worst_middle) worst_middle = middle;
            for (int i = 0; i < COUNT; i++) {
                double sideways =
                    sqrt(sky.birds[i].x * sky.birds[i].x + sky.birds[i].y * sky.birds[i].y);
                if (sideways > 30) outside++;
                if (sideways > farthest) farthest = sideways;
                if (fabs(sky.birds[i].z) > worst_height) worst_height = fabs(sky.birds[i].z);
                sampled++;
            }
        }
        /* It overshoots the roost, as a flock that has to turn round does, by a
         * turning circle and a flock's width: its middle by up to twice the roost's
         * radius, and the birds by three times, and never past it by more. */
        assert(worst_middle < 30);
        assert((double)outside / (double)sampled < 0.02);
        assert(farthest < 70);
        assert(worst_height < 10);
        sky_destroy(&sky);
    }
}

/* The flock holds together as well as it holds to its roost: most of the birds
 * are in one piece, a bird's neighbours are about as near as they were meant to
 * be, and it is flatter than it is long. */
static void test_the_flock_is_a_sheet_and_not_a_ball(void) {
    enum { COUNT = 500, STEPS = 1200 };
    sky_rules_t rules = sky_default_rules();
    sky_t sky;
    double thin = 0, long_side = 0;
    int samples = 0;

    assert(sky_init(&sky, COUNT, 5) == SKY_OK);
    sky_populate(&sky, 0, COUNT, 0);
    for (int step = 0; step < STEPS; step++) {
        sky_step(&sky, COUNT, &rules, NULL, 0.05);
        if (step < 400 || step % 40 != 0) continue;
        /* The spread of the heights against the spread along the flock's length,
         * both about the middle. */
        double centre[3], radius;
        sky_measure(&sky, COUNT, centre, &radius);
        double sz = 0, sxx = 0, sxy = 0, syy = 0;
        for (int i = 0; i < COUNT; i++) {
            double dx = sky.birds[i].x - centre[0], dy = sky.birds[i].y - centre[1],
                   dz = sky.birds[i].z - centre[2];
            sz += dz * dz;
            sxx += dx * dx;
            sxy += dx * dy;
            syy += dy * dy;
        }
        double trace = (sxx + syy) / COUNT;
        double determinant = (sxx * syy - sxy * sxy) / ((double)COUNT * COUNT);
        double biggest = trace / 2 + sqrt(fmax(trace * trace / 4 - determinant, 0));
        thin += sqrt(sz / COUNT);
        long_side += sqrt(biggest);
        samples++;
    }
    assert(samples > 10);
    assert(thin / samples < 0.5 * long_side / samples);

    /* One piece: follow the links between a bird and its nearest four that are
     * shorter than three metres. */
    int *parent = malloc(sizeof(int) * COUNT), *size = calloc(COUNT, sizeof(int));
    assert(parent != NULL && size != NULL);
    for (int i = 0; i < COUNT; i++) parent[i] = i;
    assert(sky_index(&sky, COUNT) == SKY_OK);
    for (int i = 0; i < COUNT; i++) {
        int near[4];
        double squared[4];
        int found = sky_neighbours(&sky, i, 4, 3.0, near, squared);
        for (int n = 0; n < found; n++) {
            int a = i, b = near[n];
            while (parent[a] != a) a = parent[a] = parent[parent[a]];
            while (parent[b] != b) b = parent[b] = parent[parent[b]];
            parent[b] = a;
        }
    }
    int biggest = 0;
    for (int i = 0; i < COUNT; i++) {
        int root = i;
        while (parent[root] != root) root = parent[root];
        if (++size[root] > biggest) biggest = size[root];
    }
    assert(biggest > COUNT * 6 / 10);
    free(parent);
    free(size);
    sky_destroy(&sky);
}

/* Turning is a rate: the same bird flies the same curve whether the steps that
 * make it are thirtieths of a second or sixtieths. The bird is alone, so that
 * nothing it does depends on how often it looks, and its restlessness is off. */
static void test_turning_is_per_second_and_not_per_step(void) {
    sky_rules_t rules = sky_default_rules();
    rules.wander = 0;
    sky_t slow, fast;

    assert(sky_init(&slow, 4, 1) == SKY_OK && sky_init(&fast, 4, 1) == SKY_OK);
    for (int world = 0; world < 2; world++) {
        sky_t *sky = world ? &fast : &slow;
        memset(sky->birds, 0, sizeof(*sky->birds));
        /* Well out of the roost and pointed away from it: it has a half turn to
         * make, which at this rate takes two and a half seconds. */
        sky->birds[0].x = 22;
        sky->birds[0].speed = 10;
        sky->birds[0].hx = 1;
    }
    for (int step = 0; step < 90; step++) sky_step(&slow, 1, &rules, NULL, 1.0 / 30);
    for (int step = 0; step < 180; step++) sky_step(&fast, 1, &rules, NULL, 1.0 / 60);
    double dx = slow.birds[0].x - fast.birds[0].x, dy = slow.birds[0].y - fast.birds[0].y;
    assert(sqrt(dx * dx + dy * dy) < 1.0);
    assert(fabs(wrap(slow.birds[0].yaw - fast.birds[0].yaw)) < 0.05);
    /* And it did turn: it is on its way back. */
    assert(slow.birds[0].x < 22 && fast.birds[0].x < 22);

    /* However it is asked, a bird turns no faster than the rate. */
    sky_rules_t quick = sky_default_rules();
    sky_t flock;
    double yaw[200];
    assert(sky_init(&flock, 200, 4) == SKY_OK);
    sky_populate(&flock, 0, 200, 0);
    for (int i = 0; i < 200; i++) yaw[i] = flock.birds[i].yaw;
    for (int step = 0; step < 100; step++) {
        sky_step(&flock, 200, &quick, NULL, 0.04);
        for (int i = 0; i < 200; i++) {
            double turned = fabs(wrap(flock.birds[i].yaw - yaw[i]));
            assert(turned <= quick.turn_rate * 0.04 + 1e-9);
            yaw[i] = flock.birds[i].yaw;
        }
    }
    sky_destroy(&slow);
    sky_destroy(&fast);
    sky_destroy(&flock);
}

/* A bird that is turning leans into it, the way it leans, and no further than a
 * bird can. */
static void test_a_bird_leans_into_its_turn(void) {
    sky_rules_t rules = sky_default_rules();
    rules.wander = 0;
    sky_t sky;

    assert(sky_init(&sky, 2, 1) == SKY_OK);
    for (int side = -1; side <= 1; side += 2) {
        memset(sky.birds, 0, sizeof(*sky.birds));
        sky.birds[0].x = 0;
        sky.birds[0].y = 25.0 * side; /* Out of the roost, so called home across its path. */
        sky.birds[0].speed = 10;
        sky.birds[0].hx = 1;
        for (int step = 0; step < 30; step++) sky_step(&sky, 1, &rules, NULL, 1.0 / 30);
        assert(fabs(sky.birds[0].roll) > 0.3);
        assert(fabs(sky.birds[0].roll) <= 1.0);
        /* Called to the other side of where it is going: the sign of the lean is the
         * sign of the turn. */
        assert((sky.birds[0].roll > 0) == (side < 0));
    }
    sky_destroy(&sky);
}

/* The camera orbits the roost, and what it looks at is in the middle of the
 * picture, at the camera's distance, wherever on the circle it has got to and
 * however big the picture is. */
static void test_the_roost_is_the_middle_of_the_picture(void) {
    static const int SIZES[][2] = {{768, 512}, {640, 384}, {1600, 800}, {320, 960}};

    for (size_t s = 0; s < sizeof(SIZES) / sizeof(*SIZES); s++) {
        for (double seconds = 0; seconds < 300; seconds += 17.3) {
            sky_camera_t camera;
            double sx, sy, depth;
            sky_camera_orbit(&camera, seconds, SIZES[s][0], SIZES[s][1]);
            assert(sky_camera_project(&camera, 0, 0, 0, &sx, &sy, &depth) == 1);
            assert(fabs(sx - SIZES[s][0] / 2.0) < 1e-6);
            assert(fabs(sy - SIZES[s][1] / 2.0) < 1e-6);
            assert(fabs(depth - camera.distance) < 1e-6);
            /* The vertical stays vertical: a point straight above the roost is above
             * the middle of the picture, whatever the camera is doing. */
            assert(sky_camera_project(&camera, 0, 0, 2, &sx, &sy, &depth) == 1);
            assert(fabs(sx - SIZES[s][0] / 2.0) < 1e-6);
            assert(sy < SIZES[s][1] / 2.0);
            /* The axes are a frame: unit, at right angles. */
            double dot = camera.forward[0] * camera.right[0] + camera.forward[1] * camera.right[1] +
                         camera.forward[2] * camera.right[2];
            assert(fabs(dot) < 1e-12);
            double length = sqrt(camera.up[0] * camera.up[0] + camera.up[1] * camera.up[1] +
                                 camera.up[2] * camera.up[2]);
            assert(fabs(length - 1) < 1e-12);
        }
    }
}

/* Nearer is bigger: the same metre across, twice as far, is half as many pixels. */
static void test_what_is_twice_as_far_is_half_as_big(void) {
    sky_camera_t camera;
    double near_x, near_y, near_depth, far_x, far_y, far_depth;

    sky_camera_orbit(&camera, 31, 768, 512);
    double distance = camera.distance;
    for (int axis = 0; axis < 2; axis++) {
        const double *across = axis ? camera.up : camera.right;
        /* A metre to the side, at the camera's distance, and the same metre twice
         * the distance away along the line of sight. */
        double at[3], beyond[3];
        for (int c = 0; c < 3; c++) {
            at[c] = camera.target[c] + across[c];
            beyond[c] = camera.target[c] + camera.forward[c] * distance + across[c];
        }
        assert(sky_camera_project(&camera, at[0], at[1], at[2], &near_x, &near_y, &near_depth));
        assert(sky_camera_project(&camera, beyond[0], beyond[1], beyond[2], &far_x, &far_y,
                                  &far_depth));
        assert(fabs(far_depth - 2 * near_depth) < 1e-6);
        double near_offset = hypot(near_x - camera.centre_x, near_y - camera.centre_y);
        double far_offset = hypot(far_x - camera.centre_x, far_y - camera.centre_y);
        assert(near_offset > 1);
        assert(fabs(near_offset - 2 * far_offset) < 1e-6);
    }
    /* Behind the camera is nowhere. */
    double sx, sy, depth;
    assert(sky_camera_project(&camera, camera.eye[0] - camera.forward[0] * 5,
                              camera.eye[1] - camera.forward[1] * 5,
                              camera.eye[2] - camera.forward[2] * 5, &sx, &sy, &depth) == 0);
    assert(depth < 0);
}

/* A ray through a pixel passes through whatever is drawn at that pixel. */
static void test_the_ray_through_a_pixel_finds_what_is_drawn_there(void) {
    sky_camera_t camera;
    sky_camera_orbit(&camera, 77, 768, 512);
    for (double x = 0; x < 3; x++)
        for (double y = -2; y < 3; y++) {
            double sx, sy, depth, origin[3], direction[3];
            assert(sky_camera_project(&camera, x, y * 1.5, 0.5 * x, &sx, &sy, &depth));
            sky_camera_ray(&camera, sx, sy, origin, direction);
            /* How far the point is from the ray. */
            double rx = x - origin[0], ry = y * 1.5 - origin[1], rz = 0.5 * x - origin[2];
            double along = rx * direction[0] + ry * direction[1] + rz * direction[2];
            double ox = rx - along * direction[0], oy = ry - along * direction[1],
                   oz = rz - along * direction[2];
            assert(sqrt(ox * ox + oy * oy + oz * oz) < 1e-9);
            assert(along > 0);
        }
    /* The middle of the picture is the way the camera looks. */
    double origin[3], direction[3];
    sky_camera_ray(&camera, camera.centre_x, camera.centre_y, origin, direction);
    for (int c = 0; c < 3; c++) assert(fabs(direction[c] - camera.forward[c]) < 1e-12);
}

/* It takes two minutes to go once round the roost, at a distance that does not
 * change, a little above the flock. */
static void test_the_camera_goes_round_in_two_minutes(void) {
    sky_camera_t start, later, half;

    sky_camera_orbit(&start, 10, 768, 512);
    sky_camera_orbit(&later, 130, 768, 512);
    sky_camera_orbit(&half, 70, 768, 512);
    double a = atan2(start.eye[1], start.eye[0]), b = atan2(later.eye[1], later.eye[0]);
    assert(fabs(wrap(a - b)) < 1e-9);
    /* Half way round, on the other side. */
    double c = atan2(half.eye[1], half.eye[0]);
    assert(fabs(fabs(wrap(a - c)) - M_PI) < 1e-9);
    assert(start.distance == later.distance);
    for (double seconds = 0; seconds < 240; seconds += 7) {
        sky_camera_t camera;
        sky_camera_orbit(&camera, seconds, 768, 512);
        assert(camera.eye[2] > 0.5); /* Above the flock, never under it. */
        assert(camera.elevation > 0.05 && camera.elevation < 0.7);
    }
}

/* A bird's size says how far off it is: the bins are an even geometric ladder,
 * and a bird at the middle of the flock's depth is in the middle of it. */
static void test_the_sizes_say_how_far_off_a_bird_is(void) {
    double previous = 0;
    for (int bin = 0; bin < SKY_BINS; bin++) {
        double scale = sky_bin_scale(bin);
        assert(scale > previous);
        if (bin > 0) assert(fabs(scale / previous - sky_bin_scale(1) / sky_bin_scale(0)) < 1e-9);
        assert(sky_bin_for(scale) == bin);
        previous = scale;
    }
    assert(sky_bin_for(1.0) == SKY_BINS / 2);
    assert(sky_bin_for(0.0) == 0 && sky_bin_for(-3) == 0);
    assert(sky_bin_for(1e9) == SKY_BINS - 1);

    sky_t sky;
    sky_camera_t camera;
    sky_view_t views[3];
    assert(sky_init(&sky, 3, 1) == SKY_OK);
    sky_camera_orbit(&camera, 5, 768, 512);
    memset(sky.birds, 0, 3 * sizeof(*sky.birds));
    /* One a third of the distance beyond the target, one at it, one a third of the
     * distance towards the camera. */
    for (int i = 0; i < 3; i++) {
        double along = (1 - i) * camera.distance / 3.0; /* Negative is towards the camera. */
        sky.birds[i].x = camera.target[0] + camera.forward[0] * along;
        sky.birds[i].y = camera.target[1] + camera.forward[1] * along;
        sky.birds[i].z = camera.target[2] + camera.forward[2] * along;
        sky.birds[i].hx = 1;
    }
    sky_view(&sky, 3, &camera, views);
    assert(views[1].visible && views[1].bin == sky_bin_for(1.0));
    assert(fabs(views[1].scale - 1.0) < 1e-6);
    assert(views[2].scale > views[1].scale && views[1].scale > views[0].scale);
    assert(views[2].bin >= views[1].bin && views[1].bin >= views[0].bin);
    assert(views[2].bin > views[0].bin);
    for (int i = 0; i < 3; i++)
        assert(fabs(views[i].x - 384) < 1e-3 && fabs(views[i].y - 256) < 1e-3);
    sky_destroy(&sky);
}

/* A bird seen head on is short, and the same bird seen from the side is long; its
 * wings are the other way round. The angle on the screen is the angle of its
 * projected heading. */
static void test_a_bird_flying_at_the_camera_is_foreshortened(void) {
    sky_t sky;
    sky_camera_t camera;
    sky_view_t views[2];

    assert(sky_init(&sky, 2, 1) == SKY_OK);
    sky_camera_orbit(&camera, 5, 768, 512);
    memset(sky.birds, 0, 2 * sizeof(*sky.birds));
    /* Both level, at a distance from the middle; one flying at the camera and one
     * across its view. */
    double flat = hypot(camera.forward[0], camera.forward[1]);
    sky.birds[0].hx = -camera.forward[0] / flat;
    sky.birds[0].hy = -camera.forward[1] / flat;
    sky.birds[1].hx = -camera.forward[1] / flat;
    sky.birds[1].hy = camera.forward[0] / flat;
    sky_view(&sky, 2, &camera, views);
    assert(views[0].along < 0.5 * views[1].along);
    assert(views[1].along > 0.8);
    assert(views[0].across > views[1].across);
    /* Across the view, to the right or the left: the angle is along the screen's
     * horizontal one way or the other. */
    assert(fabs(sin(views[1].angle)) < 0.5);
    sky_destroy(&sky);
}

/* The pointer is a stick poked into the sky: birds near its line are pushed off
 * it, and birds far from it are not. The birds are far enough apart not to see
 * each other, so that the stick is the only thing that differs. */
static void test_the_pointer_is_a_stick_in_the_sky(void) {
    sky_rules_t rules = sky_default_rules();
    rules.wander = 0;
    rules.current = 0;
    sky_poke_t poke = {.active = 1, .reach = 5};
    sky_t pushed, left;

    assert(sky_init(&pushed, 3, 1) == SKY_OK && sky_init(&left, 3, 1) == SKY_OK);
    /* The line runs along y from far off, through x = 0 and z = 0. */
    poke.origin[1] = -40;
    poke.direction[1] = 1;
    for (int world = 0; world < 2; world++) {
        sky_t *sky = world ? &left : &pushed;
        memset(sky->birds, 0, 3 * sizeof(*sky->birds));
        sky->birds[0] = (sky_bird_t){.x = 1.0, .y = 0, .z = 0, .hy = 1, .speed = 10};
        sky->birds[1] = (sky_bird_t){.x = 0, .y = 100, .z = 1.0, .hy = 1, .speed = 10};
        sky->birds[2] = (sky_bird_t){.x = 9.0, .y = 200, .z = 0, .hy = 1, .speed = 10};
        for (int i = 0; i < 3; i++) sky->birds[i].yaw = M_PI / 2;
    }
    for (int step = 0; step < 6; step++) {
        sky_step(&pushed, 3, &rules, &poke, 1.0 / 30);
        sky_step(&left, 3, &rules, NULL, 1.0 / 30);
    }
    /* The one a metre to the side is sent out to that side, the one a metre above
     * is sent up, and the one nine metres away, past the stick's reach, is left
     * alone. */
    assert(pushed.birds[0].x > left.birds[0].x + 0.05);
    assert(pushed.birds[1].z > left.birds[1].z + 0.001);
    assert(memcmp(&pushed.birds[2], &left.birds[2], sizeof(pushed.birds[2])) == 0);
    /* And a stick that is not there does nothing. */
    sky_t quiet;
    assert(sky_init(&quiet, 3, 1) == SKY_OK);
    memset(quiet.birds, 0, 3 * sizeof(*quiet.birds));
    quiet.birds[0] = (sky_bird_t){.x = 1.0, .y = 0, .z = 0, .hy = 1, .speed = 10, .yaw = M_PI / 2};
    quiet.birds[1] =
        (sky_bird_t){.x = 0, .y = 100, .z = 1.0, .hy = 1, .speed = 10, .yaw = M_PI / 2};
    quiet.birds[2] =
        (sky_bird_t){.x = 9.0, .y = 200, .z = 0, .hy = 1, .speed = 10, .yaw = M_PI / 2};
    poke.active = 0;
    for (int step = 0; step < 6; step++) sky_step(&quiet, 3, &rules, &poke, 1.0 / 30);
    assert(memcmp(quiet.birds, left.birds, 3 * sizeof(*left.birds)) == 0);
    sky_destroy(&quiet);
    sky_destroy(&pushed);
    sky_destroy(&left);
}

/* The same seed is the same flock, step for step; another is not. */
static void test_a_seed_is_a_flock(void) {
    sky_rules_t rules = sky_default_rules();
    sky_t a, b, c;

    assert(sky_init(&a, 100, 7) == SKY_OK && sky_init(&b, 100, 7) == SKY_OK &&
           sky_init(&c, 100, 8) == SKY_OK);
    sky_populate(&a, 0, 100, 0);
    sky_populate(&b, 0, 100, 0);
    sky_populate(&c, 0, 100, 0);
    for (int step = 0; step < 100; step++) {
        sky_step(&a, 100, &rules, NULL, 1.0 / 30);
        sky_step(&b, 100, &rules, NULL, 1.0 / 30);
        sky_step(&c, 100, &rules, NULL, 1.0 / 30);
    }
    assert(memcmp(a.birds, b.birds, 100 * sizeof(*a.birds)) == 0);
    assert(memcmp(a.birds, c.birds, 100 * sizeof(*a.birds)) != 0);
    sky_destroy(&a);
    sky_destroy(&b);
    sky_destroy(&c);
}

/* A bird that has gone to nothing is put back at the roost, not left to poison
 * its neighbours' sums, and a step that is no time at all, or no birds, does
 * nothing. */
static void test_a_bad_bird_is_put_back_and_a_bad_step_is_nothing(void) {
    sky_rules_t rules = sky_default_rules();
    sky_t sky;

    assert(sky_init(&sky, 50, 1) == SKY_OK);
    sky_populate(&sky, 0, 50, 0);
    sky.birds[3].x = NAN;
    sky.birds[4].y = INFINITY;
    sky_step(&sky, 50, &rules, NULL, 1.0 / 30);
    for (int i = 0; i < 50; i++) assert(isfinite(sky.birds[i].x + sky.birds[i].y + sky.birds[i].z));
    sky_bird_t before = sky.birds[9];
    sky_step(&sky, 50, &rules, NULL, 0);
    sky_step(&sky, 0, &rules, NULL, 1.0 / 30);
    sky_step(&sky, 51, &rules, NULL, 1.0 / 30);
    assert(memcmp(&before, &sky.birds[9], sizeof(before)) == 0);
    /* And a long step is cut to a short one rather than flown whole. */
    double clock = sky.clock;
    sky_step(&sky, 50, &rules, NULL, 5.0);
    assert(sky.clock - clock <= 0.1 + 1e-12);
    sky_destroy(&sky);
}

/* The index is sized by the flock: whatever the cell it started at, a few steps
 * bring it to about a bird's seventh neighbour's distance, and it stays inside
 * the range it is allowed. */
static void test_the_index_is_sized_by_the_flock(void) {
    sky_rules_t rules = sky_default_rules();
    sky_t sky;

    for (double start = 0.1; start < 100; start *= 8) {
        assert(sky_init(&sky, 400, 1) == SKY_OK);
        sky_populate(&sky, 0, 400, 0);
        sky.cell = start;
        for (int step = 0; step < 60; step++) sky_step(&sky, 400, &rules, NULL, 1.0 / 30);
        assert(sky.cell >= 0.4 && sky.cell <= 6.0);
        /* Close to what two runs from very different starts agree on. */
        assert(sky.cell > 0.7 && sky.cell < 2.5);
        sky_destroy(&sky);
    }
}

/* The middle and the size of a flock, and the camera that frames them: it looks
 * most of the way to the middle and stands off in proportion to the size. */
static void test_the_camera_frames_the_flock(void) {
    sky_t sky;
    sky_camera_t camera;
    double centre[3], radius;

    assert(sky_init(&sky, 6, 1) == SKY_OK);
    memset(sky.birds, 0, 6 * sizeof(*sky.birds));
    /* Six birds at the corners of an octahedron round (4, -2, 1), five metres out. */
    double middle[3] = {4, -2, 1};
    for (int i = 0; i < 6; i++) {
        double offset[3] = {0, 0, 0};
        offset[i / 2] = i % 2 ? 5 : -5;
        sky.birds[i].x = middle[0] + offset[0];
        sky.birds[i].y = middle[1] + offset[1];
        sky.birds[i].z = middle[2] + offset[2];
    }
    sky_measure(&sky, 6, centre, &radius);
    for (int c = 0; c < 3; c++) assert(fabs(centre[c] - middle[c]) < 1e-12);
    assert(fabs(radius - 5) < 1e-9);

    sky_camera_orbit(&camera, 0, 768, 512);
    double roost_distance = camera.distance;
    /* Until the flock has been measured there is nothing to frame. */
    sky_camera_frame(&camera, &sky);
    assert(camera.distance == roost_distance);
    sky.framed = 1;
    sky.flock_radius = 5;
    memcpy(sky.flock_centre, middle, sizeof(middle));
    sky_camera_frame(&camera, &sky);
    double near_distance = camera.distance;
    for (int c = 0; c < 3; c++) assert(fabs(camera.target[c] - 0.9 * middle[c]) < 1e-12);
    sky.flock_radius = 10;
    sky_camera_frame(&camera, &sky);
    assert(fabs(camera.distance - 2 * near_distance) < 1e-9);
    sky_destroy(&sky);
}

/* Hawks come in from the edge of the roost, as many as are asked for and no
 * more than there is room for, and the ones already hunting are left alone when
 * another is called. */
static void test_hawks_come_and_go_without_disturbing_each_other(void) {
    sky_t sky;

    assert(sky_init(&sky, 50, 1) == SKY_OK);
    sky_populate(&sky, 0, 50, 0);
    assert(sky.hawk_count == 0);
    sky_set_hawks(&sky, 1);
    assert(sky.hawk_count == 1 && sky.hawks[0].prey == -1);
    double radius = hypot(sky.hawks[0].x, sky.hawks[0].y);
    assert(radius > 15 && radius < 40);
    /* Heading for the roost. */
    assert(sky.hawks[0].hx * -sky.hawks[0].x + sky.hawks[0].hy * -sky.hawks[0].y > 0);
    sky_hawk_t first = sky.hawks[0];
    sky_set_hawks(&sky, 3);
    assert(sky.hawk_count == 3 && memcmp(&first, &sky.hawks[0], sizeof(first)) == 0);
    for (int i = 1; i < 3; i++) assert(hypot(sky.hawks[i].x, sky.hawks[i].y) > 15);
    sky_set_hawks(&sky, 99);
    assert(sky.hawk_count == SKY_MAX_HAWKS);
    sky_set_hawks(&sky, -4);
    assert(sky.hawk_count == 0);
    sky_set_hawks(&sky, 2);
    assert(sky.hawk_count == 2);
    sky_destroy(&sky);
}

/* The flock flees a hawk it is near, round it and not straight away, and takes
 * no notice of one it cannot see. One bird, one step, with and without. */
static void test_a_bird_flees_a_hawk_in_reach(void) {
    sky_rules_t rules = sky_default_rules();
    rules.wander = 0;
    rules.current = 0;
    sky_t with, without, far;

    assert(sky_init(&with, 1, 1) == SKY_OK && sky_init(&without, 1, 1) == SKY_OK &&
           sky_init(&far, 1, 1) == SKY_OK);
    for (int world = 0; world < 3; world++) {
        sky_t *sky = world == 0 ? &with : world == 1 ? &without : &far;
        memset(sky->birds, 0, sizeof(*sky->birds));
        sky->birds[0] = (sky_bird_t){.speed = 10, .hx = 1};
    }
    sky_set_hawks(&with, 1);
    sky_set_hawks(&far, 1);
    /* Five metres off the bird's left, level with it, heading across its path. */
    with.hawks[0] = (sky_hawk_t){.x = 0,
                                 .y = 5,
                                 .z = 0,
                                 .hy = -1,
                                 .speed = 14,
                                 .prey = 0,
                                 .commitment = 5,
                                 .yaw = -M_PI / 2};
    far.hawks[0] = (sky_hawk_t){.x = 0,
                                .y = 40,
                                .z = 0,
                                .hy = -1,
                                .speed = 14,
                                .prey = 0,
                                .commitment = 5,
                                .yaw = -M_PI / 2};
    for (int step = 0; step < 4; step++) {
        sky_step(&with, 1, &rules, NULL, 1.0 / 30);
        sky_step(&without, 1, &rules, NULL, 1.0 / 30);
        sky_step(&far, 1, &rules, NULL, 1.0 / 30);
    }
    /* Sent to the right, away from a hawk on its left, and by more than a bird that
     * has no hawk, which does not turn at all. */
    assert(with.birds[0].y < -0.01);
    assert(fabs(without.birds[0].y) < 1e-9);
    /* One that is forty metres off is not feared. */
    assert(fabs(far.birds[0].y) < 1e-9);
    sky_destroy(&with);
    sky_destroy(&without);
    sky_destroy(&far);
}

/* A hawk dives at its bird, goes through, and then runs on straight for a moment
 * before it picks another. */
static void test_a_hawk_goes_through_its_bird_and_runs_on(void) {
    sky_rules_t rules = sky_default_rules();
    rules.wander = 0;
    rules.current = 0;
    rules.hawk_weight = 0;
    sky_t sky;

    assert(sky_init(&sky, 1, 1) == SKY_OK);
    memset(sky.birds, 0, sizeof(*sky.birds));
    sky.birds[0] = (sky_bird_t){.x = 12, .speed = 0, .hx = 1};
    sky_set_hawks(&sky, 1);
    sky.hawks[0] =
        (sky_hawk_t){.x = 0, .y = 0, .z = 0, .hx = 1, .speed = 14, .prey = 0, .commitment = 5};
    int struck_at = -1;
    double run_on_start = 0;
    for (int step = 0; step < 90; step++) {
        sky_step(&sky, 1, &rules, NULL, 1.0 / 30);
        if (struck_at < 0 && sky.hawks[0].passing > 0) {
            struck_at = step;
            run_on_start = sky.hawks[0].yaw;
            assert(sky.hawks[0].prey == -1 && sky.hawks[0].commitment == 0);
            /* Where the bird was, give or take a step. */
            assert(fabs(sky.hawks[0].x - 12) < 6);
        }
        assert(isfinite(sky.hawks[0].x + sky.hawks[0].y + sky.hawks[0].z));
    }
    assert(struck_at >= 0 && struck_at < 40); /* Twelve metres at fourteen a second. */
    (void)run_on_start;
    /* It runs on, in the dive, for the pass and no longer: passing counted down. */
    assert(sky.hawks[0].passing == 0);
    sky_destroy(&sky);
}

/* Over a long hunt a hawk strikes again and again, keeps near the roost, and flies
 * between a bird's cruising speed and its dive. */
static void test_a_hawk_hunts_for_as_long_as_it_is_left(void) {
    enum { COUNT = 300, STEPS = 1200 };
    sky_rules_t rules = sky_default_rules();
    sky_t sky;
    int strikes = 0, dives = 0;
    double farthest = 0, slowest = 1e9, fastest = 0;

    assert(sky_init(&sky, COUNT, 6) == SKY_OK);
    sky_populate(&sky, 0, COUNT, 0);
    for (int step = 0; step < 200; step++) sky_step(&sky, COUNT, &rules, NULL, 0.05);
    sky_set_hawks(&sky, 2);
    for (int step = 0; step < STEPS; step++) {
        double before[2] = {sky.hawks[0].passing, sky.hawks[1].passing};
        sky_step(&sky, COUNT, &rules, NULL, 0.05);
        for (int h = 0; h < 2; h++) {
            const sky_hawk_t *hawk = &sky.hawks[h];
            if (before[h] <= 0 && hawk->passing > 0) strikes++;
            dives += hawk->diving;
            double out = hypot(hawk->x, hawk->y);
            if (out > farthest) farthest = out;
            if (hawk->speed < slowest) slowest = hawk->speed;
            if (hawk->speed > fastest) fastest = hawk->speed;
            assert(isfinite(hawk->x + hawk->y + hawk->z));
        }
    }
    assert(strikes >= 6); /* Three a minute at the least, each, in a minute. */
    assert(dives > STEPS / 10);
    assert(farthest < 70);
    assert(slowest > 8.9 && fastest < 14.6);
    for (int i = 0; i < COUNT; i++)
        assert(isfinite(sky.birds[i].x + sky.birds[i].y + sky.birds[i].z));
    sky_destroy(&sky);
}

/* The camera gives a flock that spreads room at once and takes it back slowly: a
 * flock cut off at the edge is worse than one a little small, and a camera that
 * pumps in and out is worse than either. */
static void test_the_camera_makes_room_quickly_and_takes_it_back_slowly(void) {
    enum { COUNT = 200 };
    sky_rules_t rules = sky_default_rules();
    sky_t wide, narrow;
    double settled;

    assert(sky_init(&wide, COUNT, 3) == SKY_OK && sky_init(&narrow, COUNT, 3) == SKY_OK);
    sky_populate(&wide, 0, COUNT, 0);
    for (int step = 0; step < 600; step++) sky_step(&wide, COUNT, &rules, NULL, 0.05);
    memcpy(narrow.birds, wide.birds, COUNT * sizeof(*wide.birds));
    /* Both cameras have settled on the flock as it is. */
    double centre[3], radius;
    sky_measure(&wide, COUNT, centre, &radius);
    wide.flock_radius = narrow.flock_radius = settled = radius;
    narrow.framed = 1;
    assert(settled > 1);

    /* One flock is suddenly twice as big, the other half the size, about the same
     * middle; a second of flight is all they get to show it. */
    for (int i = 0; i < COUNT; i++) {
        wide.birds[i].x = centre[0] + (wide.birds[i].x - centre[0]) * 2;
        wide.birds[i].y = centre[1] + (wide.birds[i].y - centre[1]) * 2;
        wide.birds[i].z = centre[2] + (wide.birds[i].z - centre[2]) * 2;
        narrow.birds[i].x = centre[0] + (narrow.birds[i].x - centre[0]) * 0.5;
        narrow.birds[i].y = centre[1] + (narrow.birds[i].y - centre[1]) * 0.5;
        narrow.birds[i].z = centre[2] + (narrow.birds[i].z - centre[2]) * 0.5;
    }
    sky_rules_t still = rules;
    still.cruise = 0;
    for (int step = 0; step < 20; step++) {
        sky_step(&wide, COUNT, &still, NULL, 0.05);
        sky_step(&narrow, COUNT, &still, NULL, 0.05);
    }
    double grown = (wide.flock_radius - settled) / settled;
    double shrunk = (settled - narrow.flock_radius) / settled;
    /* Of the whole way to the new size, which is +100% and -50%, how much each has
     * covered after a second: the growing one has most of it, the shrinking one a
     * fraction. */
    assert(grown / 1.0 > 0.5);
    assert(shrunk / 0.5 < 0.4);
    assert(grown / 1.0 > 2 * (shrunk / 0.5));
    sky_destroy(&wide);
    sky_destroy(&narrow);
}

int main(void) {
    test_the_nearest_birds_are_the_ones_a_search_of_everything_finds();
    test_an_index_too_fine_for_the_sky_is_cut_coarser();
    test_a_flock_is_born_flying_together();
    test_the_flock_stays_within_reach_of_its_roost();
    test_the_flock_is_a_sheet_and_not_a_ball();
    test_turning_is_per_second_and_not_per_step();
    test_a_bird_leans_into_its_turn();
    test_the_roost_is_the_middle_of_the_picture();
    test_what_is_twice_as_far_is_half_as_big();
    test_the_ray_through_a_pixel_finds_what_is_drawn_there();
    test_the_camera_goes_round_in_two_minutes();
    test_the_sizes_say_how_far_off_a_bird_is();
    test_a_bird_flying_at_the_camera_is_foreshortened();
    test_the_pointer_is_a_stick_in_the_sky();
    test_a_seed_is_a_flock();
    test_a_bad_bird_is_put_back_and_a_bad_step_is_nothing();
    test_the_index_is_sized_by_the_flock();
    test_the_camera_frames_the_flock();
    test_the_camera_makes_room_quickly_and_takes_it_back_slowly();
    test_hawks_come_and_go_without_disturbing_each_other();
    test_a_bird_flees_a_hawk_in_reach();
    test_a_hawk_goes_through_its_bird_and_runs_on();
    test_a_hawk_hunts_for_as_long_as_it_is_left();
    return 0;
}
