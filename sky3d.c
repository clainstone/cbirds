#include "sky3d.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*
 * The numbers below were found by running a flock and looking at it, and by
 * measuring it over a hundred seconds at four seeds: how much of the time it is
 * in one piece, how well its birds agree on a heading, how thick it is. Where a
 * value has a story it is told beside it.
 */

/* The index. A cell as long as the seventh neighbour is far was most queries
 * needing a second ring of cells; a third longer, the first ring holds the answer
 * almost always, with three or four birds to a cell and about a hundred and
 * twenty birds read a query. The cell is re-measured every step, because a flock
 * that tightens to half a metre between neighbours is a fifth of the cell it was
 * when it was spread: left at three metres, two thousand birds were forty to a
 * cell and a step cost two and three quarter milliseconds, against one and a third
 * now. The ceiling is for a flock that has blown itself across the sky. */
enum { GRID_MAX_CELLS = 1 << 18 };
static const double GRID_CELL = 2.0;
static const double GRID_CELL_LEAST = 0.4;
static const double GRID_CELL_MOST = 6.0;
static const double GRID_CELL_PER_NEIGHBOUR = 1.3;

/* The roost: a bird this far from it sideways, or this far above or below the
 * height it likes, is called home, harder the further it has gone. Sixteen metres
 * let the flock wander off the screen's middle for half a minute at a time; at ten
 * it is outside once in a hundred samples. The height is why the flock is a sheet
 * and not a ball: with four metres of slack it was a cloud, a metre thick it was
 * a plate, and two is a sheet with a thickness that shows when it is seen from
 * above. */
static const double ROOST_RADIUS = 10.0;
static const double ROOST_HEIGHT = 2.0;
/* How far a bird may point up or down: a starling turns in the horizontal far more
 * freely than it climbs, and a murmuration is made of the sheets and ribbons that
 * leaves. A flock allowed 0.62 of a radian was a ball. */
static const double PITCH_LIMIT = 0.3;
static const double PITCH_SHARE = 0.55;

/* What the neighbours say. Cohesion has nothing to say to a bird in the thick of
 * them: only the part of the way to their middle beyond this many metres counts,
 * so it is the edge of a flock that is drawn in and the inside is left to
 * separate. Birds nearer than the separation radius push each other apart, hardest
 * when they touch; the nearest neighbour settles at about half a metre. */
static const double COHESION_SLACK = 0.5;
static const double COHESION_RANGE = 2.2;
static const double SEPARATION_RADIUS = 1.1;

/* The air. A flock that is only pushed by its neighbours and its roost settles
 * into a single polarised plate that flies a loop over the roost for ever, and one
 * that is pushed by a point it is drawn to turns into a mill round it. This is a
 * current that wanders over the roost, slowly and at a scale larger than a flock,
 * which every bird leans into a little. Nothing in the three rules knows it is
 * there. What it does is bend the flock: the front feels a different current from
 * the back, so the flock is stretched, folded, thinned and thickened as it flies,
 * and the places where streams of birds draw together are the dark knots.
 *
 * The wavelength matters more than the strength. At twenty five metres, shorter
 * than the flock is long, the current tore it into streams (a third to a half of
 * the time in more than one piece, and the birds agreeing on a heading at 0.3); at
 * eighty it bends it (one piece four fifths of the time, agreeing at 0.6). */
static const double CURRENT_K = 0.08;
typedef struct {
    int component;
    double direction[3];
    double rate, phase, weight;
} wave_t;
static const wave_t CURRENT_WAVES[] = {
    {0, {0.0, 0.8, 1.6}, 0.37, 1.0, 1.0},    {0, {0.55, 1.3, 0.0}, -0.23, 4.0, 0.6},
    {1, {0.75, 0.0, -1.3}, -0.29, 2.0, 1.0}, {1, {0.45, 1.1, 0.0}, 0.31, 5.0, 0.6},
    {2, {0.9, 0.7, 0.0}, -0.19, 0.5, 0.15}, /* A little lift: the air is mostly level. */
};

/* A bird's own restlessness is a small push that forgets itself in a second or so. */
static const double WANDER_MEMORY = 1.4;
/* Climbing costs speed and a dive gains it, and a bird leaning into a turn loses a
 * little: the way speed, and so density, ripples through a flock when it turns. */
static const double CLIMB_COST = 0.30;
static const double BANK_COST = 0.10;
static const double SPEED_MEMORY = 0.7;
static const double ROLL_MEMORY = 0.25;
static const double ROLL_LIMIT = 0.95;

/* The camera. The frame follows the flock's middle nearly as it is, because a
 * flock flies at ten metres a second and a camera that follows it through a
 * smoothing of a second trails it by ten metres and loses it off the side. Its size
 * is eased over seconds: the flock breathes, and the picture should not. */
static const double ORBIT_SECONDS = 120.0;
static const double FRAME_MEMORY = 0.12;
static const double SIZE_ATTACK = 0.8;
static const double SIZE_RELEASE = 4.0;
static const double FOLLOW = 0.9;
/* How many times the flock's root mean square radius the camera stands off, and
 * the focal length as a share of the picture: a wide view, nearly sixty degrees,
 * because the near edge of a flock should be a good deal nearer than the far one
 * for the sizes to say anything. */
static const double FRAME_DISTANCE = 2.6;
static const double FRAME_FOCAL = 0.72;
/* A little above the flock, and rocking slowly, so that the orbit is never quite
 * a circle and a still frame is never quite the last one. */
static const double ELEVATION = 0.34;
static const double ELEVATION_SWING = 0.20;
static const double ELEVATION_SECONDS = 96.0;

/* The sizes a bird is drawn at, as a factor on the sprite size, each the same step
 * bigger than the one before: far enough apart to tell, near enough that a bird
 * crossing from one to the next does not jump. */
static const double BIN_FIRST = 0.62;
static const double BIN_STEP = 1.27;

/* Hawks. The hunt is the flat one's, in a space: it picks a bird from outside the
 * flock's alarm, holds on to it, dives when it is near, goes through, and climbs
 * back for another. It is faster than the flock in the dive and a little slower at
 * rest, so that it has to commit to catch anything, and it turns more sharply
 * than a bird can. The birds flee it as they flee a pointer, and not straight
 * away: part of it is sideways, round the hawk, on whichever side the bird is
 * already turning. Straight away and the flock bursts like a firework and is gone;
 * with a curl to it the flock opens, streams past, and closes behind, which is the
 * shape worth recording. */
static const double HAWK_REACH = 14.0;
static const double HAWK_SWIRL = 0.9;
static const double HAWK_CRUISE = 0.9;
static const double HAWK_DIVE_SPEED = 1.45;
static const double HAWK_DIVE = 14.0;
static const double HAWK_STALK = 16.0;
static const double HAWK_GIVE_UP = 34.0;
static const double HAWK_COMMITMENT = 0.8;
static const double HAWK_PASS = 0.7;
static const double HAWK_STRIKE = 2.0;
static const double HAWK_TURN = 2.6;
static const double HAWK_PITCH_SHARE = 0.7;
static const double HAWK_PITCH_LIMIT = 0.7;
static const double HAWK_APART = 14.0;
/* How far off the roost a hawk goes before it is called home, and how high. */
static const double HAWK_HOME = 22.0;
static const double HAWK_HEIGHT = 7.0;

static uint64_t next_bits(sky_t *sky) {
    uint64_t x = sky->random;
    x ^= x >> 12;
    x ^= x << 25;
    x ^= x >> 27;
    sky->random = x;
    return x * 2685821657736338717ull;
}

static double unit_random(sky_t *sky) {
    return (double)(next_bits(sky) >> 11) / 9007199254740992.0;
}

/* Roughly normal, from the sum of four uniforms: a flock is born as a cloud
 * thickest in the middle, and nothing else about the generator matters. */
static double bell_random(sky_t *sky) {
    double sum = unit_random(sky) + unit_random(sky) + unit_random(sky) + unit_random(sky);
    return (sum - 2.0) * 1.7320508075688772;
}

static double clamp(double value, double low, double high) {
    return value < low ? low : value > high ? high : value;
}

static double wrap_half_turn(double angle) {
    angle = fmod(angle + M_PI, 2 * M_PI);
    if (angle < 0) angle += 2 * M_PI;
    return angle - M_PI;
}

sky_status_t sky_init(sky_t *sky, int capacity, uint32_t seed) {
    if (sky == NULL || capacity <= 0) return SKY_ERR_ARGUMENT;
    memset(sky, 0, sizeof(*sky));
    sky->random = 0x9E3779B97F4A7C15ull ^ ((uint64_t)seed * 0xD1B54A32D192ED03ull);
    if (sky->random == 0) sky->random = 1;
    for (int i = 0; i < 8; i++) next_bits(sky);
    sky->cell = GRID_CELL;
    return sky_reserve(sky, capacity);
}

void sky_destroy(sky_t *sky) {
    if (sky == NULL) return;
    free(sky->birds);
    free(sky->next);
    free(sky->grid.start);
    free(sky->grid.item);
    free(sky->grid.cell_of);
    free(sky->grid.at_x); /* The three coordinates are one block. */
    memset(sky, 0, sizeof(*sky));
}

sky_status_t sky_reserve(sky_t *sky, int capacity) {
    if (sky == NULL || capacity <= 0) return SKY_ERR_ARGUMENT;
    if (capacity <= sky->capacity) return SKY_OK;
    sky_bird_t *birds = malloc(sizeof(*birds) * (size_t)capacity);
    sky_bird_t *next = malloc(sizeof(*next) * (size_t)capacity);
    int *item = malloc(sizeof(int) * (size_t)capacity);
    int *cell_of = malloc(sizeof(int) * (size_t)capacity);
    double *at = malloc(sizeof(double) * 3 * (size_t)capacity);
    if (birds == NULL || next == NULL || item == NULL || cell_of == NULL || at == NULL) {
        free(birds);
        free(next);
        free(item);
        free(cell_of);
        free(at);
        return SKY_ERR_MEMORY;
    }
    if (sky->birds != NULL) memcpy(birds, sky->birds, sizeof(*birds) * (size_t)sky->capacity);
    free(sky->birds);
    free(sky->next);
    free(sky->grid.item);
    free(sky->grid.cell_of);
    free(sky->grid.at_x);
    sky->birds = birds;
    sky->next = next;
    sky->grid.item = item;
    sky->grid.cell_of = cell_of;
    sky->grid.at_x = at;
    sky->grid.at_y = at + capacity;
    sky->grid.at_z = at + 2 * (size_t)capacity;
    sky->grid.capacity = capacity;
    sky->capacity = capacity;
    return SKY_OK;
}

/* Weights found together: they are the balance at which the flock holds as one
 * piece and still folds. Alignment is the heaviest, because it is what makes a
 * flock one thing; the roost is heavier than the air, so the air bends the flock
 * and never carries it off. */
sky_rules_t sky_default_rules(void) {
    sky_rules_t rules = {
        .neighbours = 7,
        .reach = 12.0,
        .separation = 1.0,
        .alignment = 5.0,
        .cohesion = 1.5,
        .roost = 3.0,
        .current = 1.0,
        .wander = 0.25,
        .turn_rate = 1.2,
        .cruise = 10.0,
        .poke_weight = 4.0,
        .hawk_weight = 8.0,
    };
    return rules;
}

static void point_along(sky_bird_t *bird, double yaw, double pitch) {
    pitch = clamp(pitch, -PITCH_LIMIT, PITCH_LIMIT);
    bird->yaw = yaw;
    bird->pitch = pitch;
    double flat = cos(pitch);
    bird->hx = flat * cos(yaw);
    bird->hy = flat * sin(yaw);
    bird->hz = sin(pitch);
}

static void point_hawk(sky_hawk_t *hawk, double yaw, double pitch) {
    pitch = clamp(pitch, -HAWK_PITCH_LIMIT, HAWK_PITCH_LIMIT);
    hawk->yaw = yaw;
    hawk->pitch = pitch;
    double flat = cos(pitch);
    hawk->hx = flat * cos(yaw);
    hawk->hy = flat * sin(yaw);
    hawk->hz = sin(pitch);
}

void sky_populate(sky_t *sky, int first, int count, int already_flying) {
    if (sky == NULL || first < 0 || count <= 0 || first + count > sky->capacity) return;
    /* One way for the flock to be flying to begin with, near enough that it is a
     * flock and not a cloud of birds that all have to turn round. */
    double yaw0 = 2 * M_PI * unit_random(sky);
    for (int i = first; i < first + count; i++) {
        sky_bird_t *bird = &sky->birds[i];
        memset(bird, 0, sizeof(*bird));
        if (already_flying && first > 0) {
            /* Beside a bird that is flying, going where it goes. */
            const sky_bird_t *host = &sky->birds[(int)(unit_random(sky) * first) % first];
            bird->x = host->x + 1.5 * bell_random(sky);
            bird->y = host->y + 1.5 * bell_random(sky);
            bird->z = host->z + 1.0 * bell_random(sky);
            point_along(bird, host->yaw + 0.1 * bell_random(sky), host->pitch);
            bird->speed = host->speed;
            continue;
        }
        bird->x = 6.5 * bell_random(sky);
        bird->y = 6.5 * bell_random(sky);
        bird->z = 2.5 * bell_random(sky);
        point_along(bird, yaw0 + 0.35 * bell_random(sky), 0.06 * bell_random(sky));
        bird->speed = 10.0 * (0.9 + 0.2 * unit_random(sky));
    }
}

/* --- The index ---------------------------------------------------------- */

static int clamp_cell(double coordinate, double origin, double cell, int dimension) {
    double at = floor((coordinate - origin) / cell);
    if (!(at >= 0)) return 0;
    if (at >= dimension) return dimension - 1;
    return (int)at;
}

sky_status_t sky_index(sky_t *sky, int count) {
    sky_grid_t *grid = &sky->grid;
    if (count < 0 || count > sky->capacity) return SKY_ERR_ARGUMENT;
    double low[3] = {0, 0, 0}, high[3] = {0, 0, 0};
    for (int i = 0; i < count; i++) {
        const sky_bird_t *bird = &sky->birds[i];
        double at[3] = {bird->x, bird->y, bird->z};
        for (int axis = 0; axis < 3; axis++) {
            if (!isfinite(at[axis])) at[axis] = 0;
            if (i == 0 || at[axis] < low[axis]) low[axis] = at[axis];
            if (i == 0 || at[axis] > high[axis]) high[axis] = at[axis];
        }
    }
    double cell = sky->cell > 0 ? sky->cell : GRID_CELL;
    int dims[3];
    for (;;) {
        double cells = 1;
        for (int axis = 0; axis < 3; axis++) {
            dims[axis] = (int)floor((high[axis] - low[axis]) / cell) + 1;
            cells *= dims[axis];
        }
        if (cells <= GRID_MAX_CELLS) break;
        cell *= 1.25;
    }
    size_t cells = (size_t)dims[0] * (size_t)dims[1] * (size_t)dims[2];
    if (cells + 1 > grid->cells_allocated) {
        int *start = realloc(grid->start, (cells + 1) * sizeof(int));
        if (start == NULL) return SKY_ERR_MEMORY;
        grid->start = start;
        grid->cells_allocated = cells + 1;
    }
    grid->cell = cell;
    for (int axis = 0; axis < 3; axis++) {
        grid->origin[axis] = low[axis];
        grid->dims[axis] = dims[axis];
    }
    grid->cells = (int)cells;

    memset(grid->start, 0, (cells + 1) * sizeof(int));
    for (int i = 0; i < count; i++) {
        const sky_bird_t *bird = &sky->birds[i];
        int cx = clamp_cell(bird->x, low[0], cell, dims[0]);
        int cy = clamp_cell(bird->y, low[1], cell, dims[1]);
        int cz = clamp_cell(bird->z, low[2], cell, dims[2]);
        int at = (cz * dims[1] + cy) * dims[0] + cx;
        grid->cell_of[i] = at;
        grid->start[at + 1]++;
    }
    for (size_t c = 0; c < cells; c++) grid->start[c + 1] += grid->start[c];
    /* Each bird goes in at its cell's cursor, which leaves every cursor at the end
     * of its cell, that is at the start of the next; shifted up by one they are the
     * starts again, and the birds in a cell are still in index order. */
    for (int i = 0; i < count; i++) grid->item[grid->start[grid->cell_of[i]]++] = i;
    for (size_t c = cells; c > 0; c--) grid->start[c] = grid->start[c - 1];
    grid->start[0] = 0;
    for (int slot = 0; slot < count; slot++) {
        const sky_bird_t *bird = &sky->birds[grid->item[slot]];
        grid->at_x[slot] = bird->x;
        grid->at_y[slot] = bird->y;
        grid->at_z[slot] = bird->z;
    }
    return SKY_OK;
}

/* --- Neighbours --------------------------------------------------------- */

/* The k best so far, nearest first. Ties go to the lower index, so the answer is
 * one list and not whichever order the cells happened to be read in. */
typedef struct {
    int count, wanted;
    int index[SKY_MAX_NEIGHBOURS];
    double squared[SKY_MAX_NEIGHBOURS];
} nearest_t;

static void offer(nearest_t *best, int index, double squared) {
    int at = best->count;
    if (at == best->wanted) {
        double worst = best->squared[at - 1];
        if (squared > worst || (squared == worst && index > best->index[at - 1])) return;
        at--;
    } else {
        best->count++;
    }
    while (at > 0 && (best->squared[at - 1] > squared ||
                      (best->squared[at - 1] == squared && best->index[at - 1] > index))) {
        best->squared[at] = best->squared[at - 1];
        best->index[at] = best->index[at - 1];
        at--;
    }
    best->squared[at] = squared;
    best->index[at] = index;
}

/* The birds in slots [first, last), which are whole cells that follow each other. */
static void read_slots(const sky_grid_t *grid, int first, int last, int self, const double at[3],
                       double reach_squared, nearest_t *best) {
    for (int slot = first; slot < last; slot++) {
        double dx = grid->at_x[slot] - at[0], dy = grid->at_y[slot] - at[1],
               dz = grid->at_z[slot] - at[2];
        double squared = dx * dx + dy * dy + dz * dz;
        if (squared > reach_squared) continue;
        if (best->count == best->wanted && squared > best->squared[best->count - 1]) continue;
        int i = grid->item[slot];
        if (i != self) offer(best, i, squared);
    }
}

int sky_neighbours(const sky_t *sky, int self, int k, double reach, int *index, double *squared) {
    const sky_grid_t *grid = &sky->grid;
    const sky_bird_t *me = &sky->birds[self];
    nearest_t best = {0, k < SKY_MAX_NEIGHBOURS ? k : SKY_MAX_NEIGHBOURS, {0}, {0}};
    double at[3] = {me->x, me->y, me->z};
    int home[3];
    for (int axis = 0; axis < 3; axis++)
        home[axis] = clamp_cell(at[axis], grid->origin[axis], grid->cell, grid->dims[axis]);
    double reach_squared = reach * reach;

    /* Ring by ring out from the bird's own cell. After a ring, anything not yet
     * read is further off than the nearest face of the block read so far, so the
     * search ends when the k-th best is nearer than that, or when the block is
     * the whole index, or when the faces are past the reach. The cells along x are
     * numbered in a row, so a row of them is one run of slots. */
    for (int ring = 0;; ring++) {
        int low[3], high[3];
        for (int axis = 0; axis < 3; axis++) {
            low[axis] = home[axis] - ring;
            high[axis] = home[axis] + ring;
        }
        int first_x = low[0] < 0 ? 0 : low[0];
        int last_x = high[0] >= grid->dims[0] ? grid->dims[0] - 1 : high[0];
        for (int z = low[2] < 0 ? 0 : low[2]; z <= high[2] && z < grid->dims[2]; z++) {
            for (int y = low[1] < 0 ? 0 : low[1]; y <= high[1] && y < grid->dims[1]; y++) {
                int row = (z * grid->dims[1] + y) * grid->dims[0];
                if (z == low[2] || z == high[2] || y == low[1] || y == high[1]) {
                    read_slots(grid, grid->start[row + first_x], grid->start[row + last_x + 1],
                               self, at, reach_squared, &best);
                } else {
                    if (low[0] >= 0)
                        read_slots(grid, grid->start[row + low[0]], grid->start[row + low[0] + 1],
                                   self, at, reach_squared, &best);
                    if (high[0] < grid->dims[0] && high[0] != low[0])
                        read_slots(grid, grid->start[row + high[0]], grid->start[row + high[0] + 1],
                                   self, at, reach_squared, &best);
                }
            }
        }
        double nearest_face = INFINITY;
        for (int axis = 0; axis < 3; axis++) {
            if (low[axis] > 0) {
                double face = at[axis] - (grid->origin[axis] + low[axis] * grid->cell);
                if (face < nearest_face) nearest_face = face;
            }
            if (high[axis] < grid->dims[axis] - 1) {
                double face = grid->origin[axis] + (high[axis] + 1) * grid->cell - at[axis];
                if (face < nearest_face) nearest_face = face;
            }
        }
        /* Strictly: a bird exactly on the face is exactly as far as the face is, and
         * if that is as far as the reach or as far as the k-th best, it still counts,
         * or goes in ahead of it when its index is the lower. */
        if (nearest_face == INFINITY || nearest_face > reach) break;
        if (best.count == best.wanted && best.squared[best.count - 1] < nearest_face * nearest_face)
            break;
    }
    for (int n = 0; n < best.count; n++) {
        index[n] = best.index[n];
        squared[n] = best.squared[n];
    }
    return best.count;
}

/* --- Flight ------------------------------------------------------------- */

static void current_at(double clock, double x, double y, double z, double out[3]) {
    out[0] = out[1] = out[2] = 0;
    for (size_t i = 0; i < sizeof(CURRENT_WAVES) / sizeof(*CURRENT_WAVES); i++) {
        const wave_t *wave = &CURRENT_WAVES[i];
        double along = wave->direction[0] * x + wave->direction[1] * y + wave->direction[2] * z;
        out[wave->component] +=
            wave->weight * sin(CURRENT_K * along - wave->rate * clock + wave->phase);
    }
}

/* One bird, for one step: what it wants to do from what it sees, and how far it
 * can do it in the time. `farthest` is how far off its last neighbour was, which
 * is what the index is sized by. */
static void fly_one(const sky_t *sky, sky_bird_t *out, int self, const sky_rules_t *rules,
                    const sky_poke_t *poke, double seconds, double *farthest) {
    const sky_bird_t *me = &sky->birds[self];
    int near[SKY_MAX_NEIGHBOURS];
    double squared[SKY_MAX_NEIGHBOURS];
    int count = sky_neighbours(sky, self, rules->neighbours, rules->reach, near, squared);
    *farthest = count > 0 ? sqrt(squared[count - 1]) : rules->reach;

    /* Where it wants to go is the way it is going, and each of the pulls added to
     * that: the direction of the sum is what matters, how long it is only says how
     * the pulls weigh against its own heading. */
    double want[3] = {me->hx, me->hy, me->hz};

    if (count > 0) {
        double heading[3] = {0, 0, 0}, middle[3] = {0, 0, 0}, apart[3] = {0, 0, 0};
        for (int n = 0; n < count; n++) {
            const sky_bird_t *other = &sky->birds[near[n]];
            double dx = other->x - me->x, dy = other->y - me->y, dz = other->z - me->z;
            heading[0] += other->hx;
            heading[1] += other->hy;
            heading[2] += other->hz;
            middle[0] += dx;
            middle[1] += dy;
            middle[2] += dz;
            double distance = sqrt(squared[n]);
            if (distance < SEPARATION_RADIUS && distance > 1e-9) {
                double push = (SEPARATION_RADIUS - distance) / (SEPARATION_RADIUS * distance);
                apart[0] -= dx * push;
                apart[1] -= dy * push;
                apart[2] -= dz * push;
            }
        }
        double share = 1.0 / count;
        double away =
            sqrt(middle[0] * middle[0] + middle[1] * middle[1] + middle[2] * middle[2]) * share;
        double pull = 0;
        if (away > 1e-9)
            pull = clamp((away - COHESION_SLACK) / COHESION_RANGE, 0, 1) * share / away;
        for (int axis = 0; axis < 3; axis++)
            want[axis] += rules->alignment * heading[axis] * share +
                          rules->cohesion * middle[axis] * pull + rules->separation * apart[axis];
    }

    /* The roost: sideways towards it beyond its radius, and up or down towards the
     * preferred height beyond its slack. The pull grows with the distance, so a
     * bird that has gone far comes back with purpose, and there is none at all
     * inside. */
    double sideways = sqrt(me->x * me->x + me->y * me->y);
    if (sideways > ROOST_RADIUS) {
        double pull = clamp((sideways - ROOST_RADIUS) / ROOST_RADIUS, 0, 3) / sideways;
        want[0] -= rules->roost * me->x * pull;
        want[1] -= rules->roost * me->y * pull;
    }
    double height = fabs(me->z);
    if (height > ROOST_HEIGHT)
        want[2] -= rules->roost * (me->z > 0 ? 1 : -1) *
                   clamp((height - ROOST_HEIGHT) / ROOST_HEIGHT, 0, 3);

    double flow[3];
    current_at(sky->clock, me->x, me->y, me->z, flow);
    for (int axis = 0; axis < 3; axis++)
        want[axis] += rules->current * flow[axis] + rules->wander * out->wander[axis];

    if (poke != NULL && poke->active) {
        double rx = me->x - poke->origin[0], ry = me->y - poke->origin[1],
               rz = me->z - poke->origin[2];
        double along = rx * poke->direction[0] + ry * poke->direction[1] + rz * poke->direction[2];
        if (along > 0) {
            double off[3] = {rx - along * poke->direction[0], ry - along * poke->direction[1],
                             rz - along * poke->direction[2]};
            double distance = sqrt(off[0] * off[0] + off[1] * off[1] + off[2] * off[2]);
            if (distance < poke->reach && distance > 1e-9) {
                double strength = (poke->reach - distance) / poke->reach;
                for (int axis = 0; axis < 3; axis++)
                    want[axis] += rules->poke_weight * strength * off[axis] / distance;
            }
        }
    }

    for (int h = 0; h < sky->hawk_count; h++) {
        const sky_hawk_t *hawk = &sky->hawks[h];
        double away[3] = {me->x - hawk->x, me->y - hawk->y, me->z - hawk->z};
        double distance = sqrt(away[0] * away[0] + away[1] * away[1] + away[2] * away[2]);
        if (distance >= HAWK_REACH || distance < 1e-9) continue;
        for (int axis = 0; axis < 3; axis++) away[axis] /= distance;
        /* The way round is the way it is already going: its heading with the part
         * that points away from the hawk taken out. */
        double along = me->hx * away[0] + me->hy * away[1] + me->hz * away[2];
        double side[3] = {me->hx - along * away[0], me->hy - along * away[1],
                          me->hz - along * away[2]};
        double length = sqrt(side[0] * side[0] + side[1] * side[1] + side[2] * side[2]);
        if (length > 1e-9)
            for (int axis = 0; axis < 3; axis++) side[axis] /= length;
        else
            side[0] = side[1] = side[2] = 0;
        double strength = (HAWK_REACH - distance) / HAWK_REACH;
        for (int axis = 0; axis < 3; axis++)
            want[axis] += rules->hawk_weight * strength * (away[axis] + HAWK_SWIRL * side[axis]);
    }

    /* Turned towards, at a limited rate: the yaw by the bird's own turn rate, the
     * pitch by a part of it. Nothing in the pull can flip a bird round in a step. */
    double flat = sqrt(want[0] * want[0] + want[1] * want[1]);
    double yaw = me->yaw, pitch = me->pitch;
    double yaw_step = rules->turn_rate * seconds;
    double pitch_step = rules->turn_rate * PITCH_SHARE * seconds;
    double turned = 0;
    if (flat > 1e-9) {
        /* A want that points nearly straight up or down says little about which way
         * round to go: the yaw is moved less the more vertical it is. */
        double sure = flat / sqrt(flat * flat + want[2] * want[2]);
        double delta = wrap_half_turn(atan2(want[1], want[0]) - yaw);
        turned = clamp(delta, -yaw_step, yaw_step) * clamp(sure * 3, 0, 1);
        yaw += turned;
    }
    double aim = atan2(want[2], flat > 1e-9 ? flat : 1e-9);
    pitch += clamp(aim - pitch, -pitch_step, pitch_step);
    point_along(out, yaw, pitch);

    /* It leans into the turn, as far as the turn is sharp. */
    double rate = seconds > 0 ? turned / seconds : 0;
    double roll_goal = clamp(rate / rules->turn_rate, -1, 1) * ROLL_LIMIT;
    out->roll = me->roll + (roll_goal - me->roll) * clamp(seconds / ROLL_MEMORY, 0, 1);

    double speed_goal =
        rules->cruise * (1 - CLIMB_COST * out->hz - BANK_COST * fabs(out->roll) / ROLL_LIMIT);
    out->speed = me->speed + (speed_goal - me->speed) * clamp(seconds / SPEED_MEMORY, 0, 1);

    out->x = me->x + out->hx * out->speed * seconds;
    out->y = me->y + out->hy * out->speed * seconds;
    out->z = me->z + out->hz * out->speed * seconds;
}

/* --- Hawks -------------------------------------------------------------- */

void sky_set_hawks(sky_t *sky, int count) {
    if (sky == NULL) return;
    if (count < 0) count = 0;
    if (count > SKY_MAX_HAWKS) count = SKY_MAX_HAWKS;
    for (int i = sky->hawk_count; i < count; i++) {
        /* From the edge of the roost, each from its own side, heading for the middle
         * of it, at a height of its own. */
        sky_hawk_t *hawk = &sky->hawks[i];
        double angle = 2 * M_PI * (i + unit_random(sky) * 0.5) / SKY_MAX_HAWKS;
        memset(hawk, 0, sizeof(*hawk));
        hawk->x = 24 * cos(angle);
        hawk->y = 24 * sin(angle);
        hawk->z = (unit_random(sky) - 0.5) * 6;
        hawk->prey = -1;
        hawk->speed = 10 * HAWK_CRUISE;
        point_hawk(hawk, angle + M_PI, 0);
    }
    sky->hawk_count = count;
}

static int nearest_bird(const sky_t *sky, int count, const sky_hawk_t *hawk, int self,
                        int unclaimed, double no_nearer_than) {
    int best = -1;
    double best_distance = 0, floor_squared = no_nearer_than * no_nearer_than;
    for (int b = 0; b < count; b++) {
        if (unclaimed) {
            int taken = 0;
            for (int h = 0; h < sky->hawk_count && !taken; h++)
                if (h != self && sky->hawks[h].prey == b) taken = 1;
            if (taken) continue;
        }
        double dx = sky->birds[b].x - hawk->x, dy = sky->birds[b].y - hawk->y,
               dz = sky->birds[b].z - hawk->z;
        double squared = dx * dx + dy * dy + dz * dz;
        if (squared < floor_squared) continue;
        if (best < 0 || squared < best_distance) {
            best_distance = squared;
            best = b;
        }
    }
    return best;
}

static double distance_to_bird(const sky_t *sky, const sky_hawk_t *hawk, int bird) {
    double dx = sky->birds[bird].x - hawk->x, dy = sky->birds[bird].y - hawk->y,
           dz = sky->birds[bird].z - hawk->z;
    return sqrt(dx * dx + dy * dy + dz * dz);
}

/* How near the hawk passes its bird over the whole of this step's travel, and not
 * merely where it stands now: at fourteen metres a second a hawk can go clean
 * through a bird between one step and the next. */
static double reach_along_the_step(const sky_t *sky, const sky_hawk_t *hawk, int bird,
                                   double step) {
    double dx = sky->birds[bird].x - hawk->x, dy = sky->birds[bird].y - hawk->y,
           dz = sky->birds[bird].z - hawk->z;
    double along = clamp(dx * hawk->hx + dy * hawk->hy + dz * hawk->hz, 0, step);
    dx -= along * hawk->hx;
    dy -= along * hawk->hy;
    dz -= along * hawk->hz;
    return sqrt(dx * dx + dy * dy + dz * dz);
}

/* Pick, or keep. A hawk holds on to its bird for a time whatever else flies past,
 * and only when that runs out will it trade up, and then for a bird a clear
 * quarter nearer. It picks from outside the flock's alarm, so that there is a chase
 * to watch: the nearest bird to a hawk that has just flown through the middle of a
 * flock is one already beside it. */
static void choose_prey(sky_t *sky, int count, int self) {
    sky_hawk_t *hawk = &sky->hawks[self];
    if (hawk->prey >= count) hawk->prey = -1;
    if (hawk->prey >= 0 && hawk->commitment > 0) return;
    if (hawk->prey >= 0 && distance_to_bird(sky, hawk, hawk->prey) > HAWK_GIVE_UP) hawk->prey = -1;
    int candidate = nearest_bird(sky, count, hawk, self, 1, HAWK_STALK);
    if (candidate < 0) candidate = nearest_bird(sky, count, hawk, self, 1, 0);
    if (candidate < 0) candidate = nearest_bird(sky, count, hawk, self, 0, 0);
    if (candidate < 0) return;
    if (hawk->prey >= 0 &&
        distance_to_bird(sky, hawk, candidate) > 0.75 * distance_to_bird(sky, hawk, hawk->prey))
        return;
    hawk->prey = candidate;
    hawk->commitment = HAWK_COMMITMENT;
}

static void hunt(sky_t *sky, int count, const sky_rules_t *rules, double seconds) {
    for (int i = 0; i < sky->hawk_count; i++) {
        sky_hawk_t *hawk = &sky->hawks[i];
        double cruise = rules->cruise;
        if (hawk->commitment > 0) hawk->commitment = fmax(hawk->commitment - seconds, 0);
        if (hawk->passing > 0) {
            hawk->passing = fmax(hawk->passing - seconds, 0);
            hawk->prey = -1;
        } else {
            int struck = hawk->prey >= 0 && hawk->prey < count &&
                         reach_along_the_step(sky, hawk, hawk->prey,
                                              cruise * HAWK_DIVE_SPEED * seconds) < HAWK_STRIKE;
            if (struck) {
                hawk->prey = -1;
                hawk->commitment = 0;
                hawk->passing = HAWK_PASS;
            } else {
                choose_prey(sky, count, i);
            }
        }

        double pace = HAWK_CRUISE;
        double want[3] = {0, 0, 0};
        /* Room for the other hawks, and the way home when it has gone too far. */
        for (int h = 0; h < sky->hawk_count; h++) {
            if (h == i) continue;
            double dx = hawk->x - sky->hawks[h].x, dy = hawk->y - sky->hawks[h].y,
                   dz = hawk->z - sky->hawks[h].z;
            double distance = sqrt(dx * dx + dy * dy + dz * dz);
            if (distance >= HAWK_APART || distance < 1e-9) continue;
            double strength = HAWK_APART / distance - 1;
            want[0] += strength * dx / distance;
            want[1] += strength * dy / distance;
            want[2] += strength * dz / distance;
        }
        double sideways = sqrt(hawk->x * hawk->x + hawk->y * hawk->y);
        if (sideways > HAWK_HOME) {
            double pull = 2.5 * clamp((sideways - HAWK_HOME) / HAWK_HOME, 0, 2) / sideways;
            want[0] -= hawk->x * pull;
            want[1] -= hawk->y * pull;
        }
        if (fabs(hawk->z) > HAWK_HEIGHT)
            want[2] -= 2.5 * (hawk->z > 0 ? 1 : -1) *
                       clamp((fabs(hawk->z) - HAWK_HEIGHT) / HAWK_HEIGHT, 0, 2);

        if (hawk->prey >= 0) {
            const sky_bird_t *prey = &sky->birds[hawk->prey];
            double gap = distance_to_bird(sky, hawk, hawk->prey);
            if (gap < HAWK_DIVE) pace = HAWK_DIVE_SPEED;
            /* Aimed where the bird will be when the hawk gets there, and not a fixed
             * distance ahead: close in that is almost no lead at all, and a fixed
             * one had it cutting across in front of the bird. */
            double lead = fmin(gap / (pace * cruise), 0.8);
            double to[3] = {prey->x + prey->hx * prey->speed * lead - hawk->x,
                            prey->y + prey->hy * prey->speed * lead - hawk->y,
                            prey->z + prey->hz * prey->speed * lead - hawk->z};
            double reach = sqrt(to[0] * to[0] + to[1] * to[1] + to[2] * to[2]);
            if (reach > 1e-9)
                for (int axis = 0; axis < 3; axis++) want[axis] += to[axis] / reach;
        }

        double flat = sqrt(want[0] * want[0] + want[1] * want[1]);
        if (flat > 1e-9 || fabs(want[2]) > 1e-9) {
            double yaw = hawk->yaw, pitch = hawk->pitch;
            if (flat > 1e-9)
                yaw += clamp(wrap_half_turn(atan2(want[1], want[0]) - yaw), -HAWK_TURN * seconds,
                             HAWK_TURN * seconds);
            pitch += clamp(atan2(want[2], flat > 1e-9 ? flat : 1e-9) - pitch,
                           -HAWK_TURN * HAWK_PITCH_SHARE * seconds,
                           HAWK_TURN * HAWK_PITCH_SHARE * seconds);
            point_hawk(hawk, yaw, pitch);
        }
        hawk->diving = pace > HAWK_CRUISE || hawk->passing > 0;
        hawk->speed = cruise * pace;
        hawk->x += hawk->hx * hawk->speed * seconds;
        hawk->y += hawk->hy * hawk->speed * seconds;
        hawk->z += hawk->hz * hawk->speed * seconds;
    }
}

void sky_measure(const sky_t *sky, int count, double centre[3], double *radius) {
    double sum[3] = {0, 0, 0}, squares = 0;
    for (int i = 0; i < count; i++) {
        const sky_bird_t *bird = &sky->birds[i];
        sum[0] += bird->x;
        sum[1] += bird->y;
        sum[2] += bird->z;
        squares += bird->x * bird->x + bird->y * bird->y + bird->z * bird->z;
    }
    double n = count > 0 ? count : 1;
    for (int axis = 0; axis < 3; axis++) centre[axis] = sum[axis] / n;
    double spread =
        squares / n - (centre[0] * centre[0] + centre[1] * centre[1] + centre[2] * centre[2]);
    *radius = spread > 0 ? sqrt(spread) : 0;
}

void sky_step(sky_t *sky, int count, const sky_rules_t *rules, const sky_poke_t *poke,
              double seconds) {
    if (sky == NULL || count <= 0 || count > sky->capacity || !(seconds > 0)) return;
    /* A long step is a bird that cannot turn where it was going to. */
    if (seconds > 0.1) seconds = 0.1;
    if (sky_index(sky, count) != SKY_OK) return;
    sky->clock += seconds;
    hunt(sky, count, rules, seconds);

    /* The wander is uniform with a spread of one, and kept at half in the vertical:
     * a sheet is flat because nothing pushes it up and down much. */
    double keep = exp(-seconds / WANDER_MEMORY);
    double kick = sqrt(1 - keep * keep) * 3.4641016151377544;
    double spread = 0;
    for (int i = 0; i < count; i++) {
        sky_bird_t *out = &sky->next[i];
        const sky_bird_t *me = &sky->birds[i];
        for (int axis = 0; axis < 3; axis++)
            out->wander[axis] =
                me->wander[axis] * keep + kick * (unit_random(sky) - 0.5) * (axis == 2 ? 0.5 : 1.0);
        double farthest;
        fly_one(sky, out, i, rules, poke, seconds, &farthest);
        /* Capped, so that a straggler a long way from anyone does not stretch
         * every cell for the sake of one. */
        spread += farthest < sky->cell * 3 ? farthest : sky->cell * 3;
        if (!isfinite(out->x + out->y + out->z + out->yaw + out->pitch + out->speed)) {
            /* Back to the roost, rather than carrying a NaN into every neighbour. */
            memset(out, 0, sizeof(*out));
            point_along(out, 2 * M_PI * unit_random(sky), 0);
            out->speed = rules->cruise;
        }
    }
    sky_bird_t *flown = sky->next;
    sky->next = sky->birds;
    sky->birds = flown;
    /* The cell the next step's index is built with, from how far apart the birds
     * were found: eased, because it is a measurement of a moving thing. */
    double wanted =
        clamp(GRID_CELL_PER_NEIGHBOUR * spread / count, GRID_CELL_LEAST, GRID_CELL_MOST);
    sky->cell += (wanted - sky->cell) * 0.5;

    double centre[3], radius;
    sky_measure(sky, count, centre, &radius);
    if (!sky->framed) {
        memcpy(sky->flock_centre, centre, sizeof(centre));
        sky->flock_radius = radius;
        sky->framed = 1;
    } else {
        double ease = 1 - exp(-seconds / FRAME_MEMORY);
        /* Quick to give the flock room when it spreads, slow to take it back when it
         * draws in: a flock that is cut off at the edge is worse than one that is a
         * little small, and a camera that pumps in and out is worse than either. */
        double ease_size =
            1 - exp(-seconds / (radius > sky->flock_radius ? SIZE_ATTACK : SIZE_RELEASE));
        for (int axis = 0; axis < 3; axis++)
            sky->flock_centre[axis] += (centre[axis] - sky->flock_centre[axis]) * ease;
        sky->flock_radius += (radius - sky->flock_radius) * ease_size;
    }
}

/* --- The camera --------------------------------------------------------- */

static void cross(const double a[3], const double b[3], double out[3]) {
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

static void normalise(double v[3]) {
    double length = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (length < 1e-12) return;
    v[0] /= length;
    v[1] /= length;
    v[2] /= length;
}

void sky_camera_aim(sky_camera_t *camera) {
    double flat = cos(camera->elevation);
    camera->eye[0] = camera->target[0] + camera->distance * flat * cos(camera->azimuth);
    camera->eye[1] = camera->target[1] + camera->distance * flat * sin(camera->azimuth);
    camera->eye[2] = camera->target[2] + camera->distance * sin(camera->elevation);
    for (int axis = 0; axis < 3; axis++)
        camera->forward[axis] = camera->target[axis] - camera->eye[axis];
    normalise(camera->forward);
    static const double world_up[3] = {0, 0, 1};
    cross(camera->forward, world_up, camera->right);
    normalise(camera->right);
    cross(camera->right, camera->forward, camera->up);
    normalise(camera->up);
}

/* Framed by the shorter of the two, in the proportion of a picture three by two:
 * a tall narrow window would otherwise be framed by its height and lose the sides
 * of the flock. */
double sky_picture(int width, int height) {
    return height < width / 1.5 ? height : width / 1.5;
}

void sky_camera_orbit(sky_camera_t *camera, double seconds, int width, int height) {
    double picture = sky_picture(width, height);
    memset(camera, 0, sizeof(*camera));
    camera->azimuth = 0.6 + 2 * M_PI * seconds / ORBIT_SECONDS;
    camera->elevation = ELEVATION + ELEVATION_SWING * sin(2 * M_PI * seconds / ELEVATION_SECONDS);
    camera->distance = FRAME_DISTANCE * 6.0;
    camera->focal = FRAME_FOCAL * picture;
    camera->centre_x = width / 2.0;
    camera->centre_y = height / 2.0;
    sky_camera_aim(camera);
}

void sky_camera_frame(sky_camera_t *camera, const sky_t *sky) {
    if (!sky->framed) return;
    double radius = sky->flock_radius < 3 ? 3 : sky->flock_radius;
    camera->distance = FRAME_DISTANCE * radius;
    for (int axis = 0; axis < 3; axis++) camera->target[axis] = FOLLOW * sky->flock_centre[axis];
    sky_camera_aim(camera);
}

int sky_camera_project(const sky_camera_t *camera, double x, double y, double z, double *sx,
                       double *sy, double *depth) {
    double v[3] = {x - camera->eye[0], y - camera->eye[1], z - camera->eye[2]};
    double d = v[0] * camera->forward[0] + v[1] * camera->forward[1] + v[2] * camera->forward[2];
    if (depth != NULL) *depth = d;
    if (d < camera->distance * 0.1) return 0;
    double right = v[0] * camera->right[0] + v[1] * camera->right[1] + v[2] * camera->right[2];
    double up = v[0] * camera->up[0] + v[1] * camera->up[1] + v[2] * camera->up[2];
    *sx = camera->centre_x + camera->focal * right / d;
    *sy = camera->centre_y - camera->focal * up / d;
    return 1;
}

void sky_camera_ray(const sky_camera_t *camera, double sx, double sy, double origin[3],
                    double direction[3]) {
    double right = (sx - camera->centre_x) / camera->focal;
    double up = -(sy - camera->centre_y) / camera->focal;
    for (int axis = 0; axis < 3; axis++) {
        origin[axis] = camera->eye[axis];
        direction[axis] =
            camera->forward[axis] + right * camera->right[axis] + up * camera->up[axis];
    }
    normalise(direction);
}

/* --- What the camera sees ---------------------------------------------- */

double sky_bin_scale(int bin) {
    return BIN_FIRST * pow(BIN_STEP, bin < 0 ? 0 : bin >= SKY_BINS ? SKY_BINS - 1 : bin);
}

int sky_bin_for(double scale) {
    if (!(scale > 0)) return 0;
    double at = log(scale / BIN_FIRST) / log(BIN_STEP);
    int bin = (int)floor(at + 0.5);
    return bin < 0 ? 0 : bin >= SKY_BINS ? SKY_BINS - 1 : bin;
}

/* One thing seen: where it is, which way it points, how far off, and how much of
 * its length and of its span the camera sees. */
static void view_one(const sky_camera_t *camera, double x, double y, double z,
                     const double heading[3], double roll, sky_view_t *view) {
    double sx, sy, depth, hx, hy, hd;
    memset(view, 0, sizeof(*view));
    if (!sky_camera_project(camera, x, y, z, &sx, &sy, &depth)) return;
    view->visible = 1;
    view->x = (float)sx;
    view->y = (float)sy;
    view->scale = (float)(camera->distance / depth);
    view->bin = sky_bin_for(view->scale);
    /* A metre ahead, to see which way the body points on the screen and how much of
     * its length that is. */
    double one_metre = camera->focal / depth;
    if (sky_camera_project(camera, x + heading[0], y + heading[1], z + heading[2], &hx, &hy, &hd)) {
        double dx = hx - sx, dy = hy - sy;
        view->angle = (float)atan2(dy, dx);
        view->along = (float)clamp(sqrt(dx * dx + dy * dy) / one_metre, 0, 1);
    }
    /* The wings: the span lies across the heading, level when it flies level and
     * tipped by the bank. */
    double up[3] = {-heading[2] * heading[0], -heading[2] * heading[1],
                    1 - heading[2] * heading[2]};
    normalise(up);
    double span[3], wing[3];
    cross(heading, up, span);
    double cr = cos(roll), sr = sin(roll);
    for (int axis = 0; axis < 3; axis++) wing[axis] = span[axis] * cr + up[axis] * sr;
    double ax, ay, bx, by, ad, bd;
    view->across = 1;
    if (sky_camera_project(camera, x + wing[0] * 0.5, y + wing[1] * 0.5, z + wing[2] * 0.5, &ax,
                           &ay, &ad) &&
        sky_camera_project(camera, x - wing[0] * 0.5, y - wing[1] * 0.5, z - wing[2] * 0.5, &bx,
                           &by, &bd)) {
        double dx = ax - bx, dy = ay - by;
        view->across = (float)clamp(sqrt(dx * dx + dy * dy) / one_metre, 0, 1);
    }
}

void sky_view(const sky_t *sky, int count, const sky_camera_t *camera, sky_view_t *views) {
    for (int i = 0; i < count; i++) {
        const sky_bird_t *bird = &sky->birds[i];
        double heading[3] = {bird->hx, bird->hy, bird->hz};
        view_one(camera, bird->x, bird->y, bird->z, heading, bird->roll, &views[i]);
    }
}

void sky_hawk_view(const sky_t *sky, int hawk, const sky_camera_t *camera, sky_view_t *view) {
    const sky_hawk_t *h = &sky->hawks[hawk];
    double heading[3] = {h->hx, h->hy, h->hz};
    view_one(camera, h->x, h->y, h->z, heading, 0, view);
}

const char *sky_status_string(sky_status_t status) {
    switch (status) {
        case SKY_OK:
            return "ok";
        case SKY_ERR_ARGUMENT:
            return "invalid argument";
        case SKY_ERR_MEMORY:
            return "out of memory";
    }
    return "unknown error";
}
