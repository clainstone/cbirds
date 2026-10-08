#include "waves.h"

#include <math.h>

/* Not M_PI, which a strict C99 build does not have. */
static const double TURN = 6.283185307179586;

int wave_catchable(const wave_t *wave) {
    return wave->wait <= 0 && wave->left <= 0 && wave->rest <= 0;
}

int wave_waiting(const wave_t *wave) {
    return wave->wait > 0;
}

int wave_busy(const wave_t *wave) {
    return wave->wait > 0 || wave->left > 0 || wave->rest > 0;
}

int wave_catch(wave_t *wave, double swerve, double wait) {
    /* A wait of nothing would be a bird that is caught and not waiting, which is
     * not a state this has: the least it can be is a hair above it. */
    if (wait < 1e-9) wait = 1e-9;
    if (wave_waiting(wave) ? wait >= wave->wait : !wave_catchable(wave)) return 0;
    wave->wait = wait;
    wave->swerve = swerve;
    return 1;
}

void wave_carry(wave_t *wave, double seconds) {
    if (wave->wait > seconds) wave->wait -= seconds;
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
