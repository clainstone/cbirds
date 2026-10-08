/*
 * What a bird remembers of an alarm, for the escape waves.
 *
 * A bird that sees a hawk dive, or sees a neighbour swerve, swerves a moment
 * later in the same way, and that is all a wave is: the turn passes from bird to
 * bird. This is one bird's part of it, as a small state machine with no idea of
 * where anything is. Caught, it waits out its reaction time; swerving, it holds
 * a turn for a short while; resting, it cannot be caught again, which is what
 * lets a wave pass through the flock and not slosh back and forth in it.
 *
 * Every time here is in seconds of flight and the caller says how many have
 * passed, so the clocks run the same at any frame rate. The flock, the hawks and
 * the neighbour search are the program's, not this file's.
 */

#ifndef WAVES_H
#define WAVES_H

typedef struct {
    double wait;    /* Seconds until the swerve begins, or zero. */
    double left;    /* Seconds of swerve left, or zero. */
    double rest;    /* Seconds until it may be caught again, from the start of a swerve. */
    double swerve;  /* The turn it makes, signed, in radians: what it passes on. */
    double heading; /* Where it swerves to, fixed when the swerve begins. */
} wave_t;

/* Not waiting, not swerving and not resting. */
int wave_catchable(const wave_t *wave);

/* Anything at all going on, which is what lets a flock with no alarm skip the lot. */
int wave_busy(const wave_t *wave);

/* Catches a bird that can be caught: it will begin to swerve through `swerve`
 * radians after `wait` seconds, which must be above zero. Returns whether it was
 * caught; a bird that is already waiting, swerving or resting is left alone. */
int wave_catch(wave_t *wave, double swerve, double wait);

/* Runs the wait down by `seconds`. If it ran out inside them, returns when, in
 * seconds from the start of them, and the wait is over; otherwise a negative
 * number, with the wait that much shorter. */
double wave_waited(wave_t *wave, double seconds);

/* The swerve begins, `late` seconds ago, at a bird flying at `direction`: the
 * heading it swerves to is fixed here, and its swerve and its rest are counted
 * from when it began and not from when this was noticed. */
void wave_begin(wave_t *wave, double direction, double late, double duration, double refractory);

/* Runs the swerve and the rest down by `seconds`. */
void wave_advance(wave_t *wave, double seconds);

#endif
