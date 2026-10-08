#include "fireflies.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

/* How long a flash takes to die. A real one is over in a few tenths of a second;
 * on a screen half a second is what lets the eye follow a wave across it, because
 * a wave a tenth of a second wide is a wave that is gone before it is seen. */
static const double GLOW_SECONDS = 0.5;
/* A whole turn, spelled out: M_PI is not in C99, and this is the only place a
 * module with no other use for a circle would want it. */
static const double TURN = 6.283185307179586476925287;

double fireflies_state(double phase, double bend) {
    if (bend < 1e-9) return phase;
    return log1p(expm1(bend) * phase) / bend;
}

double fireflies_phase(double state, double bend) {
    if (bend < 1e-9) return state;
    return expm1(bend * state) / expm1(bend);
}

void fireflies_init(fireflies_t *swarm) {
    memset(swarm, 0, sizeof(*swarm));
}

void fireflies_destroy(fireflies_t *swarm) {
    if (swarm == NULL) return;
    free(swarm->fly);
    free(swarm->queue);
    if (swarm->grid_ready) spatial_grid_destroy(&swarm->grid);
    memset(swarm, 0, sizeof(*swarm));
}

fireflies_status_t fireflies_grow(fireflies_t *swarm, int count, double period, double spread,
                                  fireflies_roll_t roll) {
    if (swarm == NULL || count < 0 || period <= 0 || roll == NULL) return FIREFLIES_ERR_ARGUMENT;
    if (count > swarm->capacity) {
        firefly_t *fly = realloc(swarm->fly, sizeof(*fly) * (size_t)count);
        if (fly == NULL) return FIREFLIES_ERR_MEMORY;
        swarm->fly = fly;
        int *queue = realloc(swarm->queue, sizeof(*queue) * (size_t)count);
        if (queue == NULL) return FIREFLIES_ERR_MEMORY;
        swarm->queue = queue;
        swarm->capacity = count;
    }
    for (int i = swarm->count; i < count; i++) {
        firefly_t *f = &swarm->fly[i];
        memset(f, 0, sizeof(*f));
        f->phase = roll();
        f->period = period * (1 + spread * (2 * roll() - 1));
        /* It flashed a fraction of a cycle ago, and is still glowing if that was
         * lately: so the first picture already has every step of the glow in it. */
        f->age = f->phase * f->period;
    }
    swarm->count = count;
    return FIREFLIES_OK;
}

/* The firefly flashes now, `late` seconds ago: that is how far past the end of its
 * cycle the step carried it. */
static void flash(firefly_t *f, double late, unsigned step) {
    f->phase = late / f->period;
    if (f->phase >= 1) f->phase = 0; /* A step longer than a cycle skips the cycles. */
    f->age = late;
    f->stamp = step;
}

static void read_firefly(const void *context, int index, double *x, double *y) {
    const fireflies_t *swarm = context;
    *x = swarm->fly[index].x;
    *y = swarm->fly[index].y;
}

/* The grid is cut to the sight, so that a flash is looked for in a handful of
 * cells and not in a few hundred: half the sight a cell, and the search is the
 * five by five round the flash. */
static int prepare_grid(fireflies_t *swarm, int width, int height, double sight) {
    int cell = (int)(sight / 2);
    if (cell < 12) cell = 12;
    if (swarm->grid_ready && swarm->grid.cell_size != cell) {
        spatial_grid_destroy(&swarm->grid);
        swarm->grid_ready = 0;
    }
    if (!swarm->grid_ready) {
        if (spatial_grid_init(&swarm->grid, cell) != SPATIAL_GRID_OK) return 0;
        swarm->grid_ready = 1;
    }
    if (spatial_grid_prepare(&swarm->grid, width, height, swarm->capacity) != SPATIAL_GRID_OK)
        return 0;
    return spatial_grid_build(&swarm->grid, swarm->count, read_firefly, swarm) == SPATIAL_GRID_OK;
}

/* Every firefly in sight of flasher j sees it. A near flash is louder than a far
 * one, and none is louder than the push itself.
 *
 * The flash happened `late` seconds before the end of the step, and the ones that
 * see it were that much earlier in their cycles: so the push is applied to the
 * phase they had then, and they run on from there. Treated as if every flash of
 * a step were at the end of it, a coarse step pushed everybody at a later phase,
 * where the clock is steeper, and the swarm fell into step at the pace of the
 * frame rate and not of the fireflies. */
static int be_seen(fireflies_t *swarm, int j, const fireflies_law_t *law, int queued) {
    const firefly_t *source = &swarm->fly[j];
    const spatial_grid_t *grid = &swarm->grid;
    double sight_squared = law->sight * law->sight;
    double late = source->age;
    int center_x, center_y;
    spatial_grid_cell_for_position(grid, source->x, source->y, &center_x, &center_y);
    int reach = (int)ceil(law->sight / grid->cell_size);
    int min_x = center_x - reach, max_x = center_x + reach;
    int min_y = center_y - reach, max_y = center_y + reach;
    if (min_x < 0) min_x = 0;
    if (min_y < 0) min_y = 0;
    if (max_x >= grid->columns) max_x = grid->columns - 1;
    if (max_y >= grid->rows) max_y = grid->rows - 1;

    for (int cell_y = min_y; cell_y <= max_y; cell_y++) {
        for (int cell_x = min_x; cell_x <= max_x; cell_x++) {
            int cell = cell_y * grid->columns + cell_x;
            for (int slot = grid->offsets[cell]; slot < grid->offsets[cell + 1]; slot++) {
                int k = grid->indices[slot];
                firefly_t *seer = &swarm->fly[k];
                if (k == j || seer->sky != source->sky) continue;
                /* Not the ones that have flashed this step. */
                if (seer->stamp == swarm->step) continue;
                double since = late / seer->period;
                double then = seer->phase - since;
                if (then < law->refractory) continue; /* Still dazzled by its own. */
                double dx = seer->x - source->x, dy = seer->y - source->y;
                double squared = dx * dx + dy * dy;
                if (squared >= sight_squared) continue;
                double loudness = 1 - sqrt(squared) / law->sight;
                double state = fireflies_state(then, law->bend) + law->push * loudness;
                if (state >= 1) {
                    /* Pushed past the end of its cycle: absorbed. It flashes with
                     * the one it saw, and from now on they are one. */
                    flash(seer, late, swarm->step);
                    swarm->queue[queued++] = k;
                    continue;
                }
                double phase = fireflies_phase(state, law->bend) + since;
                if (phase < seer->phase) phase = seer->phase;
                if (phase >= 1) { /* Carried to the end by running on, not by the push. */
                    flash(seer, (phase - 1) * seer->period, swarm->step);
                    swarm->queue[queued++] = k;
                } else {
                    seer->phase = phase;
                }
            }
        }
    }
    return queued;
}

int fireflies_step(fireflies_t *swarm, double seconds, int width, int height,
                   const fireflies_law_t *law) {
    if (swarm == NULL || law == NULL || swarm->count == 0 || seconds < 0) return 0;
    swarm->step++;
    int queued = 0;
    for (int i = 0; i < swarm->count; i++) {
        firefly_t *f = &swarm->fly[i];
        f->age += seconds;
        f->phase += seconds / f->period;
        if (f->phase >= 1) {
            flash(f, (f->phase - 1) * f->period, swarm->step);
            swarm->queue[queued++] = i;
        }
    }
    int flashed = queued;
    if (queued == 0 || law->push <= 0 || law->sight <= 0) return flashed;
    if (!prepare_grid(swarm, width, height, law->sight)) return flashed;

    /* The queue grows as it is read: a firefly absorbed by one flash is itself a
     * flash, seen by those round it in the same step, and the cascade runs until
     * nobody is left within reach of the end of their cycle. */
    for (int head = 0; head < queued; head++) queued = be_seen(swarm, swarm->queue[head], law, queued);
    return queued;
}

double fireflies_order(const fireflies_t *swarm) {
    if (swarm == NULL || swarm->count == 0) return 0;
    double cosine = 0, sine = 0;
    for (int i = 0; i < swarm->count; i++) {
        double angle = TURN * swarm->fly[i].phase;
        cosine += cos(angle);
        sine += sin(angle);
    }
    return hypot(cosine, sine) / swarm->count;
}

int fireflies_level(const fireflies_t *swarm, int index, int levels) {
    double age = swarm->fly[index].age;
    if (levels < 1 || age >= GLOW_SECONDS) return -1;
    int level = (int)(age / GLOW_SECONDS * levels);
    return level >= levels ? levels - 1 : level;
}

void fireflies_scatter(fireflies_t *swarm, int index, double roll) {
    firefly_t *f = &swarm->fly[index];
    f->phase = roll >= 1 ? 0 : (roll < 0 ? 0 : roll);
}
