#include "../fireflies.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/* The test's own random numbers: xorshift, so a failure is the same failure on
 * every system. */
static uint64_t random_word;

static void seed(uint64_t value) {
    random_word = value * 0x9E3779B97F4A7C15ULL + 12345;
    for (int i = 0; i < 20; i++) {
        random_word ^= random_word << 13;
        random_word ^= random_word >> 7;
        random_word ^= random_word << 17;
    }
}

static double roll(void) {
    random_word ^= random_word << 13;
    random_word ^= random_word >> 7;
    random_word ^= random_word << 17;
    return (double)(random_word >> 11) / 9007199254740992.0;
}

/* What the program ships, in pixels, for 400 fireflies on a 1600 by 800 screen
 * (the sight is three and a bit spacings of 57 pixels). The push is larger than
 * the shipped one, because these fireflies hold still: a swarm that drifts mixes,
 * and falls into step in a quarter of the time one that does not. */
static const fireflies_law_t LAW = {.sight = 180, .push = 0.02, .bend = 3, .refractory = 0.15};
enum { WIDTH = 1600, HEIGHT = 800 };

static void scatter_over_the_screen(fireflies_t *swarm) {
    for (int i = 0; i < swarm->count; i++) {
        swarm->fly[i].x = roll() * WIDTH;
        swarm->fly[i].y = roll() * HEIGHT;
    }
}

/* The clock bends: a flash seen early in a cycle moves a firefly a little and one
 * seen late moves it a lot, which is what makes absorption possible. */
static void test_the_clock_is_concave(void) {
    for (double bend = 0.5; bend <= 6; bend += 0.5) {
        assert(fireflies_state(0, bend) == 0);
        assert(fabs(fireflies_state(1, bend) - 1) < 1e-12);
        double before = 0, jump_early = 0, jump_late = 0;
        for (int i = 1; i <= 100; i++) {
            double phase = i / 100.0;
            double state = fireflies_state(phase, bend);
            assert(state > before);             /* Rising. */
            if (i < 100) assert(state > phase); /* Above the straight line: concave. */
            before = state;
            assert(fabs(fireflies_phase(state, bend) - phase) < 1e-9);
            /* The same push in state is a bigger step in phase later on. */
            double step = fireflies_phase(fireflies_state(phase, bend) + 0.01, bend) - phase;
            if (i == 10) jump_early = step;
            if (i == 90) jump_late = step;
        }
        assert(jump_late > jump_early);
        if (bend >= 3) assert(jump_late > 2 * jump_early); /* Where it ships, by a lot. */
    }
    /* No bend is the straight line, and nothing divides by it. */
    assert(fireflies_state(0.3, 0) == 0.3 && fireflies_phase(0.3, 0) == 0.3);
}

static void test_the_order_of_a_swarm_is_what_the_phases_say(void) {
    fireflies_t swarm;
    fireflies_init(&swarm);
    assert(fireflies_order(&swarm) == 0);
    assert(fireflies_grow(&swarm, 360, 1.0, 0.04, roll) == FIREFLIES_OK);
    for (int i = 0; i < 360; i++) swarm.fly[i].phase = 0.37;
    assert(fabs(fireflies_order(&swarm) - 1) < 1e-9);
    /* Evenly round the clock, nothing is in step. */
    for (int i = 0; i < 360; i++) swarm.fly[i].phase = i / 360.0;
    assert(fireflies_order(&swarm) < 1e-9);
    /* Two halves half a cycle apart cancel. */
    for (int i = 0; i < 360; i++) swarm.fly[i].phase = i % 2 ? 0.1 : 0.6;
    assert(fireflies_order(&swarm) < 1e-9);
    /* And a random start is about one over the root of the count, which is where
     * "from about a tenth" comes from. */
    seed(3);
    fireflies_destroy(&swarm);
    assert(fireflies_grow(&swarm, 400, 1.0, 0.04, roll) == FIREFLIES_OK);
    assert(fireflies_order(&swarm) < 0.15);
    fireflies_destroy(&swarm);
}

static void test_a_swarm_grows_and_keeps_what_it_has(void) {
    fireflies_t swarm;
    seed(4);
    fireflies_init(&swarm);
    assert(fireflies_grow(&swarm, 50, 1.0, 0.04, roll) == FIREFLIES_OK);
    assert(swarm.count == 50);
    double phase[50];
    for (int i = 0; i < 50; i++) {
        firefly_t *f = &swarm.fly[i];
        phase[i] = f->phase;
        /* A period of its own, a few percent off, and a phase in the cycle. */
        assert(f->period >= 0.96 - 1e-9 && f->period <= 1.04 + 1e-9);
        assert(f->phase >= 0 && f->phase < 1);
        /* It flashed that long ago: so the first picture has glow in it. */
        assert(fabs(f->age - f->phase * f->period) < 1e-12);
    }
    int distinct = 0;
    for (int i = 1; i < 50; i++) distinct += swarm.fly[i].period != swarm.fly[0].period;
    assert(distinct == 49);

    assert(fireflies_grow(&swarm, 120, 1.0, 0.04, roll) == FIREFLIES_OK);
    assert(swarm.count == 120);
    for (int i = 0; i < 50; i++) assert(swarm.fly[i].phase == phase[i]);
    assert(fireflies_grow(&swarm, 10, 1.0, 0.04, roll) == FIREFLIES_OK);
    assert(swarm.count == 10);
    assert(fireflies_grow(&swarm, -1, 1.0, 0.04, roll) == FIREFLIES_ERR_ARGUMENT);
    assert(fireflies_grow(&swarm, 10, 0, 0.04, roll) == FIREFLIES_ERR_ARGUMENT);
    fireflies_destroy(&swarm);
    fireflies_destroy(&swarm); /* Twice is nothing. */
}

/* The clocks run on seconds and not on steps: one firefly with nobody to see has
 * the same phase after the same time at any frame rate, and flashes as often. */
static void test_the_phases_advance_per_second(void) {
    static const int RATES[] = {15, 25, 30, 60, 120, 240};
    double phases[6];
    int flashes[6];
    for (int r = 0; r < 6; r++) {
        fireflies_t swarm;
        seed(5);
        fireflies_init(&swarm);
        assert(fireflies_grow(&swarm, 1, 1.0, 0.0, roll) == FIREFLIES_OK);
        swarm.fly[0].phase = 0.3;
        swarm.fly[0].period = 0.8;
        flashes[r] = 0;
        int steps = RATES[r] * 3; /* Three seconds of 0.8: three cycles and three quarters. */
        for (int i = 0; i < steps; i++)
            flashes[r] += fireflies_step(&swarm, 1.0 / RATES[r], WIDTH, HEIGHT, &LAW);
        phases[r] = swarm.fly[0].phase;
        fireflies_destroy(&swarm);
    }
    for (int r = 0; r < 6; r++) {
        /* 0.3 + 3 / 0.8 = 4.05 cycles: four flashes and 0.05 into the next, at every
         * rate, because what a step overshoots is carried into the next. */
        assert(flashes[r] == 4);
        assert(fabs(phases[r] - 0.05) < 1e-9);
    }
    /* A step longer than a whole cycle flashes once and does not run away. */
    fireflies_t swarm;
    fireflies_init(&swarm);
    assert(fireflies_grow(&swarm, 1, 1.0, 0.0, roll) == FIREFLIES_OK);
    assert(fireflies_step(&swarm, 3.5, WIDTH, HEIGHT, &LAW) == 1);
    assert(swarm.fly[0].phase >= 0 && swarm.fly[0].phase < 1);
    fireflies_destroy(&swarm);
}

/* Three fireflies in a row, one about to flash. */
static void line_up(fireflies_t *swarm, double flasher, double near_phase, double far_phase) {
    seed(6);
    fireflies_init(swarm);
    assert(fireflies_grow(swarm, 3, 1.0, 0.0, roll) == FIREFLIES_OK);
    firefly_t *f = swarm->fly;
    f[0].x = 800, f[0].y = 400, f[0].phase = flasher;
    f[1].x = 850, f[1].y = 400, f[1].phase = near_phase;                 /* Fifty pixels off. */
    f[2].x = 800 + LAW.sight + 20, f[2].y = 400, f[2].phase = far_phase; /* Beyond sight. */
    for (int i = 0; i < 3; i++) f[i].age = 10;
}

static void test_a_flash_is_seen_within_sight_and_louder_nearby(void) {
    fireflies_t swarm;
    line_up(&swarm, 0.9999, 0.5, 0.5);
    assert(fireflies_step(&swarm, 0.001, WIDTH, HEIGHT, &LAW) == 1);
    double near_push = swarm.fly[1].phase - (0.5 + 0.001);
    assert(near_push > 1e-4);                                 /* Seen. */
    assert(fabs(swarm.fly[2].phase - (0.5 + 0.001)) < 1e-12); /* Not, out of sight. */
    fireflies_destroy(&swarm);

    /* The same flash, further off, is quieter. */
    fireflies_t other;
    line_up(&other, 0.9999, 0.5, 0.5);
    other.fly[1].x = 800 + 150;
    assert(fireflies_step(&other, 0.001, WIDTH, HEIGHT, &LAW) == 1);
    double far_push = other.fly[1].phase - (0.5 + 0.001);
    assert(far_push > 0 && far_push < near_push / 2);
    fireflies_destroy(&other);

    /* A firefly in another sky does not see it at all. */
    fireflies_t sky;
    line_up(&sky, 0.9999, 0.5, 0.5);
    sky.fly[1].sky = 1;
    assert(fireflies_step(&sky, 0.001, WIDTH, HEIGHT, &LAW) == 1);
    assert(fabs(sky.fly[1].phase - (0.5 + 0.001)) < 1e-12);
    fireflies_destroy(&sky);
}

/* Pushed past the end, a firefly flashes at once and starts again with the one it
 * saw: that is what welds groups together. And the flash it makes is seen in its
 * turn, in the same step, by those round it. */
static void test_a_firefly_pushed_past_the_end_flashes_at_once(void) {
    fireflies_t swarm;
    line_up(&swarm, 0.9999, 0.995, 0.5);
    assert(fireflies_step(&swarm, 0.001, WIDTH, HEIGHT, &LAW) == 2);
    assert(swarm.fly[1].phase < 0.01 && swarm.fly[1].age < 0.01);
    assert(swarm.fly[0].phase < 0.01);
    fireflies_destroy(&swarm);

    /* A chain, each only just in reach of the next: a cascade that runs all the way
     * down it in the one step, and stops at the one that was not close enough. */
    fireflies_t chain;
    seed(7);
    fireflies_init(&chain);
    assert(fireflies_grow(&chain, 7, 1.0, 0.0, roll) == FIREFLIES_OK);
    for (int i = 0; i < 7; i++) {
        chain.fly[i].x = 100 + 60 * i; /* A third of the sight apart. */
        chain.fly[i].y = 400;
        chain.fly[i].phase = i == 0 ? 0.9999 : 0.995;
        chain.fly[i].age = 10;
    }
    chain.fly[6].phase = 0.3; /* The last is nowhere near the end. */
    assert(fireflies_step(&chain, 0.001, WIDTH, HEIGHT, &LAW) == 6);
    for (int i = 0; i < 6; i++) assert(chain.fly[i].age < 0.01);
    assert(fabs(chain.fly[6].age - 10.001) < 1e-9);
    fireflies_destroy(&chain);
}

/* Not a firefly that has only just flashed: it is looking at its own light. */
static void test_a_flash_is_not_seen_while_dazzled(void) {
    fireflies_t swarm;
    line_up(&swarm, 0.9999, 0.05, 0.5);
    assert(fireflies_step(&swarm, 0.001, WIDTH, HEIGHT, &LAW) == 1);
    assert(fabs(swarm.fly[1].phase - (0.05 + 0.001)) < 1e-12);
    fireflies_destroy(&swarm);
}

/* Coupling only ever pushes a clock forward: a firefly that sees flashes ends the
 * step further round its cycle than the clock alone would have taken it, or it has
 * flashed. Over a swarm in disorder, a step at a time, from several seeds. */
static void test_coupling_only_pushes_forward(void) {
    for (uint64_t s = 1; s <= 4; s++) {
        fireflies_t swarm;
        seed(10 + s);
        fireflies_init(&swarm);
        assert(fireflies_grow(&swarm, 300, 1.0, 0.04, roll) == FIREFLIES_OK);
        scatter_over_the_screen(&swarm);
        double dt = 1.0 / 60;
        long pushed = 0, flashed = 0;
        for (int step = 0; step < 60 * 20; step++) {
            double alone[300];
            for (int i = 0; i < 300; i++) alone[i] = swarm.fly[i].phase + dt / swarm.fly[i].period;
            fireflies_step(&swarm, dt, WIDTH, HEIGHT, &LAW);
            for (int i = 0; i < 300; i++) {
                const firefly_t *f = &swarm.fly[i];
                if (alone[i] >= 1 || f->age < dt) { /* It flashed, on its own or seen. */
                    flashed++;
                    assert(f->phase <= dt / f->period + 1e-9); /* Starting again. */
                    continue;
                }
                assert(f->phase >= alone[i] - 1e-12);
                if (f->phase > alone[i] + 1e-12) pushed++;
            }
        }
        assert(pushed > 100 && flashed > 1000); /* The test really saw some. */
        fireflies_destroy(&swarm);
    }
}

/* The point of it: from a random start the order climbs from about a tenth to
 * above 0.95, nothing in charge. Measured here with the fireflies holding still, at
 * the push this test uses; the program's own version, with the swarm drifting
 * and the shipped push, is in boids_test.c. */
static double time_to_unison(uint64_t which, int rate, double *start) {
    fireflies_t swarm;
    seed(which);
    fireflies_init(&swarm);
    assert(fireflies_grow(&swarm, 400, 1.0, 0.04, roll) == FIREFLIES_OK);
    scatter_over_the_screen(&swarm);
    *start = fireflies_order(&swarm);
    double when = -1;
    for (int step = 0; step < rate * 90 && when < 0; step++) {
        fireflies_step(&swarm, 1.0 / rate, WIDTH, HEIGHT, &LAW);
        if (fireflies_order(&swarm) > 0.95) when = (step + 1.0) / rate;
    }
    fireflies_destroy(&swarm);
    return when;
}

static void test_a_swarm_with_nobody_in_charge_falls_into_step(void) {
    for (uint64_t which = 1; which <= 6; which++) {
        double start;
        double when = time_to_unison(which, 30, &start);
        assert(start < 0.15);
        assert(when > 0 && when < 60);
    }
}

/* And as soon at 30 frames a second as at 60, and at 120: the same swarm, the
 * same time to unison, to the scatter of a different step each. One swarm falls
 * into step in anything from 6 to 50 seconds, depending on how its clocks start,
 * so it is the mean of sixteen that is compared: 15.3, 17.9 and 18.4 seconds. */
static void test_the_time_to_unison_does_not_depend_on_the_frame_rate(void) {
    double mean[3] = {0, 0, 0};
    static const int RATES[3] = {30, 60, 120};
    enum { SEEDS = 16 };
    for (int r = 0; r < 3; r++) {
        for (uint64_t which = 1; which <= SEEDS; which++) {
            double start;
            double when = time_to_unison(which, RATES[r], &start);
            assert(when > 0);
            mean[r] += when / SEEDS;
        }
    }
    for (int r = 1; r < 3; r++) assert(mean[r] > mean[0] * 0.7 && mean[r] < mean[0] * 1.43);
}

/* Once together they stay together, with every period still a few percent off:
 * the coupling has to be stronger than the spread, or unison would be a moment. */
static void test_unison_holds(void) {
    fireflies_t swarm;
    seed(21);
    fireflies_init(&swarm);
    assert(fireflies_grow(&swarm, 400, 1.0, 0.04, roll) == FIREFLIES_OK);
    scatter_over_the_screen(&swarm);
    int step = 0;
    while (fireflies_order(&swarm) < 0.98 && step < 60 * 120) {
        fireflies_step(&swarm, 1.0 / 60, WIDTH, HEIGHT, &LAW);
        step++;
    }
    assert(fireflies_order(&swarm) >= 0.98);
    double least = 1;
    for (int i = 0; i < 60 * 30; i++) {
        fireflies_step(&swarm, 1.0 / 60, WIDTH, HEIGHT, &LAW);
        double order = fireflies_order(&swarm);
        if (order < least) least = order;
    }
    assert(least > 0.95);
    fireflies_destroy(&swarm);
}

static void test_the_glow_fades_in_steps_and_goes_dark(void) {
    fireflies_t swarm;
    seed(8);
    fireflies_init(&swarm);
    assert(fireflies_grow(&swarm, 1, 1.0, 0.0, roll) == FIREFLIES_OK);
    int last = 0;
    for (double age = 0; age < 0.5; age += 0.01) {
        swarm.fly[0].age = age;
        int level = fireflies_level(&swarm, 0, 5);
        assert(level >= last && level < 5); /* Only ever dimmer, and on the ramp. */
        last = level;
    }
    assert(last == 4);
    swarm.fly[0].age = 0;
    assert(fireflies_level(&swarm, 0, 5) == 0); /* The flash itself is the brightest. */
    swarm.fly[0].age = 0.5;
    assert(fireflies_level(&swarm, 0, 5) == -1); /* Dark. */
    swarm.fly[0].age = 3;
    assert(fireflies_level(&swarm, 0, 5) == -1);
    assert(fireflies_level(&swarm, 0, 1) == -1);
    swarm.fly[0].age = 0.1;
    assert(fireflies_level(&swarm, 0, 1) == 0); /* A ramp of one is lit or dark. */
    fireflies_destroy(&swarm);
}

static void test_a_startled_firefly_is_thrown_to_a_new_phase(void) {
    fireflies_t swarm;
    seed(9);
    fireflies_init(&swarm);
    assert(fireflies_grow(&swarm, 2, 1.0, 0.0, roll) == FIREFLIES_OK);
    swarm.fly[0].phase = 0.8;
    swarm.fly[0].age = 0.2;
    fireflies_scatter(&swarm, 0, 0.1);
    assert(swarm.fly[0].phase == 0.1); /* Back, as well as forward. */
    assert(swarm.fly[0].age == 0.2);   /* The glow it has is its own. */
    fireflies_scatter(&swarm, 0, 1.0);
    assert(swarm.fly[0].phase == 0);
    fireflies_destroy(&swarm);
}

int main(void) {
    test_the_clock_is_concave();
    test_the_order_of_a_swarm_is_what_the_phases_say();
    test_a_swarm_grows_and_keeps_what_it_has();
    test_the_phases_advance_per_second();
    test_a_flash_is_seen_within_sight_and_louder_nearby();
    test_a_firefly_pushed_past_the_end_flashes_at_once();
    test_a_flash_is_not_seen_while_dazzled();
    test_coupling_only_pushes_forward();
    test_a_swarm_with_nobody_in_charge_falls_into_step();
    test_the_time_to_unison_does_not_depend_on_the_frame_rate();
    test_unison_holds();
    test_the_glow_fades_in_steps_and_goes_dark();
    test_a_startled_firefly_is_thrown_to_a_new_phase();
    return 0;
}
