#include "../waves.h"

#include <assert.h>
#include <math.h>
#include <string.h>

/* Not M_PI, which a strict C99 build does not have. */
static const double TURN = 6.283185307179586;

static int near(double a, double b) {
    return fabs(a - b) < 1e-9;
}

/* A bird is told once, and then it is not catchable until it has swerved and
 * rested: swerving and resting each say no, and waiting says yes only to what is
 * sooner, which is then what it copies. */
static void test_a_bird_is_caught_once_until_it_has_rested(void) {
    wave_t bird;
    memset(&bird, 0, sizeof(bird));
    assert(wave_catchable(&bird));
    assert(!wave_busy(&bird));

    assert(wave_catch(&bird, 0.9, 0.01));
    assert(wave_busy(&bird));
    assert(!wave_catchable(&bird));
    assert(wave_waiting(&bird));
    /* Waiting, it takes what is sooner and nothing else, and copies it. */
    assert(!wave_catch(&bird, -0.9, 0.02));
    assert(!wave_catch(&bird, -0.9, 0.01));
    assert(bird.swerve == 0.9 && near(bird.wait, 0.01));
    assert(wave_catch(&bird, -0.5, 0.005));
    assert(bird.swerve == -0.5 && near(bird.wait, 0.005));

    wave_begin(&bird, 0, 0.01, 0.03, 1.0);
    assert(!wave_waiting(&bird));
    assert(!wave_catchable(&bird));
    assert(!wave_catch(&bird, -0.9, 0.01)); /* Swerving. */

    wave_advance(&bird, 0.1);
    assert(bird.left == 0 && bird.rest > 0); /* The swerve is over and the rest is not. */
    assert(!wave_catchable(&bird));
    assert(wave_busy(&bird));
    assert(!wave_catch(&bird, -0.9, 0.01)); /* Resting. */

    wave_advance(&bird, 1.0);
    assert(bird.rest == 0);
    assert(wave_catchable(&bird));
    assert(!wave_busy(&bird));
    assert(wave_catch(&bird, -0.9, 0.01)); /* And it can be caught again. */
    assert(bird.swerve == -0.9);
}

/* The refractory time is counted from when the swerve began, not from when the
 * step that noticed it ended. */
static void test_the_rest_is_counted_from_the_start_of_the_swerve(void) {
    wave_t bird;
    memset(&bird, 0, sizeof(bird));
    wave_catch(&bird, 0.5, 0.004);
    wave_begin(&bird, 1.0, 0.006, 0.03, 1.5);
    assert(near(bird.left, 0.024)); /* Six thousandths of it were in the step already. */
    assert(near(bird.rest, 1.494));
    wave_advance(&bird, 1.494 - 1e-6);
    assert(!wave_catchable(&bird));
    wave_advance(&bird, 2e-6);
    assert(wave_catchable(&bird));
}

/* However the time is cut into steps, a wait ends at the same moment: the steps
 * it is carried through before the one it ends in, and how far into that one,
 * add up to the wait. */
static void test_a_wait_ends_at_the_same_moment_at_any_frame_rate(void) {
    static const double rates[] = {25, 30, 33, 50, 60, 120};
    for (size_t r = 0; r < sizeof(rates) / sizeof(*rates); r++) {
        double step = 1.0 / rates[r];
        wave_t bird;
        memset(&bird, 0, sizeof(bird));
        wave_catch(&bird, 0.5, 0.1);
        double elapsed = 0;
        while (bird.wait > step) {
            wave_carry(&bird, step);
            elapsed += step;
        }
        assert(bird.wait > 0 && bird.wait <= step);
        assert(near(elapsed + bird.wait, 0.1));
    }
}

/* And a swerve runs down by the same total whether a second is thirty steps or
 * sixty. */
static void test_the_swerve_and_the_rest_run_down_at_any_frame_rate(void) {
    double left[2], rest[2];
    static const int rates[] = {30, 60};
    for (int r = 0; r < 2; r++) {
        wave_t bird;
        memset(&bird, 0, sizeof(bird));
        wave_catch(&bird, 0.5, 0.001);
        wave_begin(&bird, 0, 0, 0.5, 3.0);
        for (int step = 0; step < rates[r]; step++) wave_advance(&bird, 0.5 / rates[r]);
        left[r] = bird.left;
        rest[r] = bird.rest;
    }
    assert(near(left[0], left[1]) && near(rest[0], rest[1]));
    assert(near(left[0], 0) && near(rest[0], 2.5));
}

/* The heading is fixed when the swerve begins, from the heading the bird has
 * then, and stays on the circle. */
static void test_the_heading_is_the_direction_turned_by_the_swerve(void) {
    wave_t bird;
    memset(&bird, 0, sizeof(bird));
    wave_catch(&bird, 1.0, 0.01);
    wave_begin(&bird, 6.0, 0, 0.03, 1.0);
    assert(near(bird.heading, 7.0 - TURN));
    assert(bird.heading >= 0 && bird.heading < TURN);

    memset(&bird, 0, sizeof(bird));
    wave_catch(&bird, -1.0, 0.01);
    wave_begin(&bird, 0.25, 0, 0.03, 1.0);
    assert(near(bird.heading, TURN - 0.75));
}

/* A swerve that has to be shorter than the part of the step it began in is
 * still a swerve, and a wait of nothing is still a wait: neither state may
 * collapse into "not alarmed" while somebody is relying on it. */
static void test_a_swerve_and_a_wait_never_vanish_in_the_step_they_begin(void) {
    wave_t bird;
    memset(&bird, 0, sizeof(bird));
    assert(wave_catch(&bird, 0.5, 0));
    assert(bird.wait > 0 && !wave_catchable(&bird));
    wave_begin(&bird, 0, 0.5, 0.03, 1.0); /* Begun half a second ago in a step that long. */
    assert(bird.left > 0);
    assert(bird.rest >= bird.left);
    assert(wave_busy(&bird));
}

int main(void) {
    test_a_bird_is_caught_once_until_it_has_rested();
    test_the_rest_is_counted_from_the_start_of_the_swerve();
    test_a_wait_ends_at_the_same_moment_at_any_frame_rate();
    test_the_swerve_and_the_rest_run_down_at_any_frame_rate();
    test_the_heading_is_the_direction_turned_by_the_swerve();
    test_a_swerve_and_a_wait_never_vanish_in_the_step_they_begin();
    return 0;
}
