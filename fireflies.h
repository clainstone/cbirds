/*
 * Fireflies that fall into step.
 *
 * Every firefly has a clock: a phase that runs from 0 to 1 over a period of its
 * own, and at 1 it flashes and starts again. When it sees a neighbour flash it
 * nudges its clock forward, and that is the whole of the model. Nothing is in
 * charge, nothing counts the swarm, and left alone for long enough the flashes
 * line up anyway: Photinus carolinus in the Great Smoky Mountains and Pteroptyx
 * in South-East Asia do exactly this, and it is the other great example of
 * emergence beside a murmuration.
 *
 * The model is Mirollo and Strogatz, "Synchronization of pulse-coupled
 * biological oscillators", SIAM Journal on Applied Mathematics, 1990. Their
 * clock is not linear: it is a concave function of the phase, so a flash seen
 * early in a cycle moves a firefly a little and one seen late moves it a lot,
 * and a firefly pushed past the end flashes at once. That is absorption, and it
 * is what welds two groups into one.
 *
 * Nothing is delayed on its way. A firefly answers a flash in the step it sees it,
 * and a real one does not: it takes a few tenths of a second. That was tried, as
 * a hold between being pushed over and flashing, and it does not work here. With
 * 400 fireflies and a push that falls into step in 20 seconds, a hold of 20
 * milliseconds got the swarm to a sync of 0.9 and not to 0.95 in 60 seconds, and
 * one of 50 milliseconds left it at 0.3 for ever: the one that answers late is
 * late again next cycle, and it is the same lag every time.
 *
 * This file knows nothing of birds, screens or colours. It knows positions
 * (handed over by the caller each step), clocks, and how far a flash is seen.
 */

#ifndef FIREFLIES_H
#define FIREFLIES_H

#include "spatial_grid.h"

typedef enum { FIREFLIES_OK = 0, FIREFLIES_ERR_ARGUMENT, FIREFLIES_ERR_MEMORY } fireflies_status_t;

typedef struct {
    double x, y;   /* Where it is, as the caller last said. */
    double phase;  /* Zero to one through its cycle. */
    double period; /* Seconds for a whole cycle: its own, a little off the others'. */
    double age;    /* Seconds since it last flashed. */
    unsigned stamp; /* The step in which it last flashed. */
    int sky;       /* Which plane: only fireflies in the same one see each other. */
} firefly_t;

typedef struct {
    double sight;      /* Pixels at which a flash is still seen. */
    double push;       /* How far a flash seen close by moves the clock, in state (see below). */
    double bend;       /* How concave the clock is: zero is a straight line. */
    double refractory; /* Phase after a flash during which flashes are not seen. */
} fireflies_law_t;

typedef struct {
    int count, capacity;
    firefly_t *fly;
    int *queue; /* Flashes waiting to be seen, one step's worth. */
    spatial_grid_t grid;
    int grid_ready;
    unsigned step;
} fireflies_t;

/* A roll is a number from zero to one; the caller's own random generator, so that
 * a seed gives the same swarm everywhere. */
typedef double (*fireflies_roll_t)(void);

void fireflies_init(fireflies_t *swarm);
void fireflies_destroy(fireflies_t *swarm);

/* Makes the swarm count fireflies. The ones already there are kept; each new one
 * starts at a phase of its own, with a period of its own, spread by `spread`
 * either side of `period`. */
fireflies_status_t fireflies_grow(fireflies_t *swarm, int count, double period, double spread,
                                  fireflies_roll_t roll);

/* One step of `seconds`. The fireflies' x, y and sky are read as they stand. Every
 * clock runs on, and every flash is seen by every firefly in sight of it, nearer
 * ones louder, which may bring more fireflies to the end of their cycle in the
 * same step. Returns how many flashed. */
int fireflies_step(fireflies_t *swarm, double seconds, int width, int height,
                   const fireflies_law_t *law);

/* r = |mean(exp(2 pi i phase))|: about 1/sqrt(n) for a swarm in disorder, and 1
 * for one that flashes together. */
double fireflies_order(const fireflies_t *swarm);

/* How bright it is, as a step of a ramp `levels` long: 0 is the flash itself, the
 * last is the glow dying, and -1 is dark. */
int fireflies_level(const fireflies_t *swarm, int index, int levels);

/* Startled: the clock is thrown to a phase of the caller's choosing, forward or
 * back. The glow it is wearing is its own business and is left to fade. */
void fireflies_scatter(fireflies_t *swarm, int index, double roll);

/* Mirollo and Strogatz's state function and its inverse, exposed for the tests:
 * x = ln(1 + (e^b - 1) phase) / b. */
double fireflies_state(double phase, double bend);
double fireflies_phase(double state, double bend);

#endif
