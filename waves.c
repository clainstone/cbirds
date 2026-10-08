#include "waves.h"

#include <math.h>

/* Not M_PI, which a strict C99 build does not have. */
static const double TURN = 6.283185307179586;

int wave_catchable(const wave_t *wave) {
    return wave->wait <= 0 && wave->left <= 0 && wave->rest <= 0;
}

int wave_busy(const wave_t *wave) {
    return wave->wait > 0 || wave->left > 0 || wave->rest > 0;
}

int wave_catch(wave_t *wave, double swerve, double wait) {
    if (!wave_catchable(wave)) return 0;
    /* A wait of nothing would be a bird that is caught and not waiting, which is
     * not a state this has: the least it can be is a hair above it. */
    wave->wait = wait > 1e-9 ? wait : 1e-9;
    wave->swerve = swerve;
    return 1;
}

double wave_waited(wave_t *wave, double seconds) {
    if (wave->wait <= 0) return -1;
    if (wave->wait > seconds) {
        wave->wait -= seconds;
        return -1;
    }
    double when = wave->wait;
    wave->wait = 0;
    return when;
}

void wave_begin(wave_t *wave, double direction, double late, double duration, double refractory) {
    double turned = direction + wave->swerve;
    /* Kept on the circle like every other heading in the program. */
    wave->heading = fmod(fmod(turned, TURN) + TURN, TURN);
    wave->wait = 0;
    /* Begun in the middle of a step, so a part of the swerve is already over. A
     * swerve shorter than that is still a swerve, for the instant it takes to
     * show it. */
    wave->left = duration - late > 1e-9 ? duration - late : 1e-9;
    wave->rest = refractory - late > wave->left ? refractory - late : wave->left;
}

void wave_advance(wave_t *wave, double seconds) {
    wave->left = wave->left > seconds ? wave->left - seconds : 0;
    wave->rest = wave->rest > seconds ? wave->rest - seconds : 0;
}
