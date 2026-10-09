/* Feature test macros must precede every include. */
#define _XOPEN_SOURCE 700
#define _DEFAULT_SOURCE
#define _DARWIN_C_SOURCE

#include <errno.h>
#include <fcntl.h>
#include <langinfo.h>
#include <limits.h>
#include <locale.h>
#include <math.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/time.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include "cells.h"
#include "fireflies.h"
#include "font.h"
#include "gif.h"
#include "kitty_graphics.h"
#include "letters.h"
#include "options.h"
#include "picture.h"
#include "png.h"
#include "sign.h"
#include "sky3d.h"
#include "spatial_grid.h"
#include "sprite_png.h"
#include "vt.h"
#include "waves.h"

enum {
    /* Sixty rotations, six degrees apart. It was ninety, and at four degrees a
     * fifteen pixel bird moves less than a pixel between frames; with wing beats
     * and a second layer there are four times the images to build, and six
     * degrees is still smoother than any sprite sheet a game ever shipped. */
    ROTATION_FRAMES = 60,
    /* A power of two so a rounded angle wraps with one mask. At 4096 entries the
     * samples are 0.088 degrees apart, and sine plus cosine as floats occupy one
     * typical 32 KB L1 data cache. */
    TRIG_LOOKUP_SIZE = 4096,
    TRIG_LOOKUP_MASK = TRIG_LOOKUP_SIZE - 1,
    /* Wings out, wings half, wings folded: three pictures, beaten in a cycle of
     * four so the flap goes out and back. */
    WING_PHASES = 3,
    WING_CYCLE = 4,
    /* Near and far. A far bird is smaller, dimmer and slower, flocks only with
     * other far birds, and is drawn underneath: two planes, and the parallax
     * between them is what makes a flat screen read as a sky with depth in it. */
    LAYERS = 2,
    MAX_PALETTE_SHADES = 8,
    /* Three is as many as a five step ramp can tell apart: at four the two nearest
     * shades are closer to each other than two birds are wide. */
    MAX_FLOCKS = 3,
    MOUSE_REACH = 120, /* Pixels from the pointer within which a bird feels it. */
    DEFAULT_VISION_NOTCH = 6,
    DEFAULT_TURNING_NOTCH = 8,
    /* Only every sixteenth bird leaves one, because a tail behind all of them is
     * three times the bandwidth for a picture that reads as mud. Fifty comets in
     * a flock of eight hundred is what says "moving" in a still frame. */
    TRAIL_EVERY = 4,
    TRAIL_LENGTH = 3,
    INTRO_SECONDS = 3,
    /* Four is as many as the sky holds: past that they overlap each other, crowd
     * the flock into the edges and stop reading as separate animals. */
    MAX_HAWKS = 4,
    HAWK_REACH = 150,
    HAWK_COMMITMENT_FRAMES = 40, /* Sixty-hertz frames; converted to seconds below. */
    HAWK_GIVE_UP = 200,          /* Pixels past which a reconsidered chase is dropped. */
    HAWK_STALK = 340,            /* And how far off it looks for the next bird. */
    HAWK_PASS_FRAMES = 14,       /* Sixty-hertz frames; converted to seconds below. */
    HAWK_SPACING = 150,          /* Pixels two hawks try to keep between them. */
    /* A GIF's delay is in hundredths of a second, so the rates it can express are
     * 100/1, 100/2, 100/3 and so on. Viewers also clamp anything under two
     * hundredths up to a tenth of a second, which puts the real ceiling at fifty:
     * sixty is simply not a rate a GIF has. */
    MAX_RECORD_FPS = 50,
    AUTOPILOT_PERIOD = 4, /* Seconds between one slider moving and the next. */
    OUTRO_FRAMES_AT_SIXTY = 40,
    FRAME_ANGLE = 360 / ROTATION_FRAMES,
    SPRITE_SUPERSAMPLE = 6,
    SPRITE_WORK_MAX = 256,
    MIN_BIRD_SIZE = 4,
    MAX_BIRD_SIZE = 64,
    MAX_BIRDS = 4096,
    INPUT_BUFFER_SIZE = 100,
    DEFAULT_COLS = 80,
    DEFAULT_ROWS = 24,
    DEFAULT_CELL_WIDTH = 8,
    DEFAULT_CELL_HEIGHT = 16,
    /* Edge bands the flock turns away from, as a fraction of the viewport: a
     * third on the sides and the top, half of that at the bottom. */
    TURN_BAND_DIVISOR = 3,
    BOTTOM_BAND_DIVISOR = 6,
    /* Sixty is the rate, not a setting: every speed and turn in the program is a
     * per second quantity divided by it, and a picture a frame terminal is the
     * one thing that lowers it. A cast may be recorded at up to twice that. */
    FRAME_RATE = 60,
    MAX_CAST_FPS = 120,
    DEFAULT_SPEED = 40,
    DEFAULT_BIRD_SIZE = 30,
    /* In three dimensions there are more birds, and they are further off. What
     * --size means there is the bird at the middle of the flock's depth; the
     * nearest size is three fifths bigger than that and the farthest a little over
     * half of it. */
    SKY_BIRDS = 2000,
    SKY_BIRD_SIZE = 8,
    /* Dots are coarser than sprites: a bird that a sprite renders in eight pixels
     * lights as many dots as one of five, and the flock's depth shows in how thick
     * the dots lie, which a smaller bird leaves room for. */
    SKY_TEXT_BIRD_SIZE = 5,
    /* How a bird is drawn at its distance: its body foreshortened or not, and its
     * wings spread, half spread or edge on. A bird seen from the side of its
     * flight is a long thin thing, and head on a short wide one. */
    SKY_ALONG_LEVELS = 2,
    SKY_ACROSS_LEVELS = 3,
    SKY_SHAPES = SKY_ALONG_LEVELS * SKY_ACROSS_LEVELS,
    SPATIAL_CELL_SIZE = 12,
    /* Perception is tuned as a radius in pixels rather than in whole grid cells,
     * which is what lets it share the twelve notch travel: the cell scan derives
     * from it, and the distance test was always the exact radius anyway. Sixty
     * pixels is the five cell ceiling it had before. */
    MIN_VISION_RADIUS = 12,
    MAX_VISION_RADIUS = 60,
    DEFAULT_VISION_RADIUS = 36,
    MAX_VISION_CELLS = MAX_VISION_RADIUS / SPATIAL_CELL_SIZE,
    /* The parameter panel, anchored to the top left corner. Its size in cells is
     * fixed: it follows the longest parameter name and the bar, never the
     * terminal. Below the minimum viewport it is dropped and the flock keeps
     * everything; the minimums leave a corridor to the right of the panel and
     * one underneath it. */
    LEGEND_COLUMNS = 38,
    /* Ten rows with one flock; with more there is a slider for how much they
     * avoid each other, and it only exists when there is somebody to avoid. */
    LEGEND_ROWS = 10,
    LEGEND_MAX_ROWS = LEGEND_ROWS + 1,
    /* One notch a keypress, so this is also the number of steps every parameter
     * travels through, from its floor to its ceiling. */
    LEGEND_BAR_CELLS = 12,
    LEGEND_NAME_WIDTH = 10,
    LEGEND_VALUE_WIDTH = 5,
    /* The panel is 38 by 11 cells at most and the flock may not enter it. At the
     * smallest terminal it used to appear in it covered half the screen, and half
     * the flock was squeezed off the edges of what was left: it needs to be a
     * quarter of the room at most, not a half, so it waits for a window it fits
     * inside. */
    LEGEND_MIN_COLS = 76,
    LEGEND_MIN_ROWS = 22,
    LEGEND_LINE_MAX = 128,
    SPAWN_ATTEMPTS = 32
};

/* The same scale as the original bottom edge turn: large enough that it settles
 * the direction on its own, whatever the flocking terms are doing. */
static const double LEGEND_PUSH = 100000.0;
/* How hard an edge pushes once a bird reaches the screen's own edge, against a
 * flocking sum of one to four. Twelve turns them well inside the band and still
 * leaves them the whole screen to use; much more and the flock plays in a box. */
static const double EDGE_FIRM = 12.0;
/* And how much harder each further band-width of straying costs. */
static const double ESCAPE_PENALTY = 8.0;
/* Heavy enough to bend a flock that is busy flocking, light enough that it bends
 * rather than shatters. */
static const double MOUSE_WEIGHT = 4.0;
/* A hawk frightens a bird; it does not get to throw it off the screen. At six
 * the flee beat the edge even at the screen's own edge, and four birds in fifty
 * were outside the frame at any moment with four hawks up. */
static const double HAWK_WEIGHT = 3.0;
/* A shade slower than the flock when it is only cruising, so it has to dive to
 * catch anything: a hawk that outruns the birds at rest never has to commit, and
 * the moment it commits is the moment worth watching. */
static const double HAWK_SPEED = 0.90;
/* Sharper than a bird's bank, because a raptor is more agile, but a limit all the
 * same: without one it turned forty degrees a frame and read as a glitch. A fifth
 * of a radian looked calm and never caught anything: the turning circle was wider
 * than the flock, so every miss became a long trip to a wall and back. Half a
 * radian still only landed one chase in eight; at a whole one it lands half of
 * them, and the wall is reached a third as often. */
static const double HAWK_TURN = 1.0;
/* That is per step at sixty a second. It follows elapsed flight time so the hawk
 * remains the same animal when frames arrive faster or slower, and when the speed
 * slider flies everything faster or slower. */
#define HAWK_TURN_PER_FRAME() (HAWK_TURN * FRAME_RATE * flight_seconds())
/* At most the distance a bird covers in three sixty-hertz steps. */
static const double HAWK_LEAD_DISTANCE = DEFAULT_SPEED * 3.0;
/* Inside the dive it accelerates and stops leading: a bird that flees is only a
 * tenth slower than a cruising hawk, so without this the chase never closes and
 * there is no moment to watch. */
static const double HAWK_DIVE = 90.0;
/* Wing beats a second, for the birds; a starling manages about eight, and six
 * reads as effort without reading as panic. A bird glides now and then — wings
 * out and still for half a second or so — because a flock in which every wing is
 * always beating looks like a machine. */
static const double WING_HZ = 6.0;
static const double GLIDE_CHANCE = 0.06; /* Per beat completed. */
static const double GLIDE_SECONDS_MIN = 0.4, GLIDE_SECONDS_MAX = 1.2;
/* How the far layer differs from the near: smaller, slower — the parallax — and
 * dimmed toward the ground, which is what distance does to colour. */
static const double FAR_SHARE = 0.35;
static const double FAR_SIZE = 0.62;
static const double FAR_PACE = 0.72;
static const double FAR_DIM = 0.40; /* How far toward the ground its tint goes. */
/* What a wing at each phase does to the span: seen from above, a beat is the
 * wings foreshortening, not a new drawing. */
static const double WING_SPAN[WING_PHASES] = {1.0, 0.72, 0.45};
static const int WING_SEQUENCE[WING_CYCLE] = {0, 1, 2, 1};
/* Tails: three ghosts behind every fourth bird, each fainter and a touch smaller
 * than the last, so a tail is a fade and not a queue. */
static const double TRAIL_ALPHA[TRAIL_LENGTH] = {0.40, 0.25, 0.12};
static const double TRAIL_SIZE = 0.85;
static const double HAWK_DIVE_SPEED = 1.45;
/* How much of the flee is sideways rather than straight away. Purely radial and
 * the flock bursts like a firework and is gone; with a curl to it the birds peel
 * around the hawk and close up behind, which is the shape people watch for. */
static const double HAWK_SWIRL = 0.9;
/* What one hawk's company is worth to another, against a chase of one. */
static const double HAWK_APART = 1.2;
/* And what a wall is worth: more than the chase, or it follows a bird into the
 * edge and bounces off it. */
static const double HAWK_WALL = 2.5;
/* Flocking is local — a bird sees sixty pixels at most — so nothing in the three
 * rules keeps a flock together as a body across a whole screen. The leash is the
 * missing long range term: nothing at all within a flock's own width of its
 * centre, and a pull that grows outside it. The width grows with the flock, as
 * the square root of its birds, twelve pixels to the root, which is about the
 * width a flock that size takes up with no leash at all. A fixed 150 was
 * narrower than a flock of five hundred, so the leash pulled on its whole rim
 * and it wound itself into a mill: 1000 birds in two flocks turned 37 times a
 * minute, 4096 in three 45 times. With the width grown they turn two to four. */
static const int FLOCK_LEASH = 150; /* The narrowest it gets, for a small flock. */
static const double LEASH_PER_ROOT_BIRD = 12.0;
static const double LEASH_WEIGHT = 2.5;
/* A breeze the whole flock leans into. Enough to shape it, not enough to carry
 * it off: at the top notch it is about a third of the alignment weight. */
static const double WIND_WEIGHT = 0.5;

/*
 * Fireflies (--fireflies).
 *
 * A swarm does not flock; it drifts and flashes, and the interest is in the
 * clocks (see fireflies.h). Everything here that is a distance is measured in
 * spacings, the distance between neighbours if the swarm were laid out evenly
 * on the screen, so that the same swarm on a small recording and on a large
 * display has the same number of fireflies in sight of each other and the same
 * story to tell. In pixels it did not: a sight of 180, which fell 400 fireflies
 * holding still into step in about half a minute on 1600 by 800, did it in a
 * second and a half to 300 of them on 768 by 416, where each sees a third of the
 * swarm, and not in ninety seconds on 3000 by 1900, where each sees seven.
 *
 * The times below are the order parameter passing 0.95, over sixteen seeds of
 * 400 fireflies at 96 by 26 cells and 30 frames a second, as the mean and then
 * the worst. The tests in boids_test.c measure the same thing.
 */
/* Four hundred, and fourteen pixels. Eight hundred read as a flock and these as a
 * meadow; at 1600 by 800 they are a spacing of 57 pixels apart, which leaves the
 * dark between them dark. The count does not move the story, because the sight
 * follows it: 100, 200, 400, 800 and 1600 fireflies fall into step in 21, 25, 23,
 * 28 and 29 seconds on average (eight seeds at 120 by 34). */
enum { FIREFLY_COUNT = 400, FIREFLY_SIZE = 14 };
/* A second a cycle and four percent either way: a few percent, so that unison
 * has to be earned. At two percent it is 21.8 seconds (worst 32.9) and at eight
 * it is 40.1 (61.5): the spread is what the coupling has to beat. */
static const double FIREFLY_PERIOD = 1.0;
static const double FIREFLY_SPREAD = 0.04;
/* How far a flash seen close by moves a clock, in the state of the curve and not
 * of the phase. This is the number that sets the time. At 0.004 it is 51.6
 * seconds (worst 73), at 0.005 35.8 (63.7), here 22.2 (37.7), at 0.0085 15.7 and
 * at 0.011 11.4: all sixteen seeds got there at every one of them, but below this
 * the worst is more than a minute, and the viewer who has waited a minute for the
 * unison has stopped waiting. The coupling slider multiplies it, from a fifteenth
 * at the bottom of the bar, where nothing happens, to nearly three at the top. */
static const double FIREFLY_PUSH = 0.0065;
/* The concavity of the clock, which is what makes absorption work: the more bent
 * it is the more a flash seen late in a cycle moves a firefly and a flash seen
 * early does not. At 0.5, nearly straight, none of four swarms got past a sync of
 * 0.45 in a minute. At 1.5 it was 57 seconds, and one in sixteen had not got
 * there in a hundred; at 6 it was 10 seconds, which is a snap and not a climb. */
static const double FIREFLY_BEND = 3.0;
/* After its own flash a firefly does not see the others for this much of its
 * cycle. It is the worst case it cuts: with none it is 27.1 seconds on average
 * and 67.1 at worst, at 0.15 22.2 and 37.7, at 0.3 18.0 and 30.9. A firefly that
 * has just flashed and is pushed again is being pushed by the very flashes it has
 * just joined. */
static const double FIREFLY_REFRACTORY = 0.15;
/* Spacings a flash is seen, at the default perception. That is thirty or so of
 * the others, whatever the screen. At 2.0 sixteen of sixteen never fell into step
 * in 100 seconds, at 2.6 it was 55.1 (worst 99), at 4.0 10.8: below a sight of
 * two spacings the swarm is not one swarm, and the slider's bottom notches are the
 * way to watch that. */
static const double FIREFLY_SIGHT = 3.2;
/* A spacing a second at the shipped pace: slow, and a screen in about half a
 * minute. It matters less than it looks, because the swarm mixes either way: at
 * 0.4 it is 27.1 seconds and at 2.5 it is 20.0. */
static const double FIREFLY_DRIFT = 1.0;
/* Personal space, and how hard the others push. Pushing as hard as the heading
 * itself, a firefly among five neighbours turned wherever their sum said and
 * traced a tangle of loops the size of itself; at three tenths it bends round
 * them and carries on. The wander is a heading that takes a random turn a step,
 * in radians a root second: at 1.0 the paths still double back on themselves, at
 * 0.6 they hover, and 0.7 is the meander between. */
static const double FIREFLY_ROOM = 0.8;
static const double FIREFLY_PUSH_APART = 0.3;
static const double FIREFLY_PULL_TOGETHER = 0.01;
static const double FIREFLY_WANDER = 0.7;
/* The edges, and the meadow. A tenth of the screen at each side pushes back, and
 * the top third leans the swarm down, softly: with the flock's own bands, a third
 * of the screen at each side, the swarm sat in a box a third of the screen's area
 * and fell into step in eight seconds. */
static const double FIREFLY_EDGE = 0.10;
static const double FIREFLY_MEADOW = 0.35;
/* Between flashes a firefly is not quite dark: a small dot, the ramp's last shade
 * most of the way back to the ground, so that the swarm can be felt in the dark
 * and the dark is still dark. With nothing drawn at all, the quiet between two
 * synchronised flashes was an empty screen, and a swarm that had lost its
 * fireflies; with the last shade of the ramp itself it was a lawn, and the flash
 * was a brightening of it and not a light in the night. At four fifths of the way
 * to the ground it was gone on the GIF; at three fifths it is there when looked
 * for. */
static const double FIREFLY_BODY_SIZE = 0.55; /* Of the firefly's size. */
static const double FIREFLY_BODY_FADE = 0.60; /* How far from the last shade to the ground. */
/* The pointer is a lantern four spacings wide. At three, the fireflies it was
 * held over fled it faster than it could scatter them, and the whole swarm's
 * sync only fell to 0.94. A firefly in it is thrown to a new phase four times a
 * second on average. */
static const double FIREFLY_LANTERN = 4.0;
static const double FIREFLY_STARTLE = 4.0;

/* Needed in the config initializer, so macros rather than constants. */
#define DEFAULT_SEPARATION_W 0.005
#define DEFAULT_ALIGNMENT_W 1.5
/* Not a slider any more. Dragging it from end to end moved the flock's own
 * measure of itself — neighbours within sixty pixels — by four and a half, where
 * simply removing the term moves it by eleven: the force does something, the
 * knob did not, and it was costing a row of a six row panel and two of the
 * fourteen keys. */
#define COHESION_W 0.01
#define DEFAULT_BOUNDARY_W 0.2

static const double BOUNDARY_MIN = 0.01;
static const double SEPARATION_MIN = 0.001;
static const double ALIGNMENT_MIN = 0.1;

/* Each ceiling is placed so that the default lands exactly on the fourth of
 * twelve notches, a third along the bar. There are no step constants any more:
 * a step is one notch, which is a twelfth of the travel by construction, and
 * that is what makes a keypress worth exactly one cell of bar. */
#define DEFAULT_NOTCH 4
#define NOTCH_CEILING(minimum, default_value) ((minimum) + 3 * ((default_value) - (minimum)))
static const double BOUNDARY_MAX = NOTCH_CEILING(0.01, DEFAULT_BOUNDARY_W);
static const double SEPARATION_MAX = NOTCH_CEILING(0.001, DEFAULT_SEPARATION_W);
static const double ALIGNMENT_MAX = NOTCH_CEILING(0.1, DEFAULT_ALIGNMENT_W);

/* Flight speed, as a factor on the pace everything else was tuned at. It scales
 * how far a bird flies in a second and, with it, how far it may turn in that
 * second, so the path a bird traces is the one the edges, the panel and the
 * hawks were tuned on, flown faster or slower: without the turn a fast flock
 * swung wide into the edges and a slow one spun on the spot. A fifth of the pace
 * is slow motion; at the top a frame is flown in three steps (see fly), which
 * keeps eight hundred birds near a millisecond a frame. A fifth a notch, from a
 * fifth at notch zero to thirteen fifths at the top, so the panel prints each
 * one as it is. The default is the second notch, two fifths: the flock starts
 * slow enough to follow one bird with the eye, and v/V is there for more. */
#define DEFAULT_PACE_NOTCH 1
#define DEFAULT_PACE 0.4
static const double PACE_STEP = 0.2;

/* How much one flock avoids another, with two or more of them.
 *
 * On the fourth notch, the default, a flock keeps to its own kind and to nothing
 * else: it flies where it likes, and meets and crosses the others. It used to be
 * sent a room away from them as well, to a home of its own, and a flock sent
 * home flies round it: two flocks turned twelve times a minute where one flock
 * alone turns twice. Measured over six runs of a minute and a half.
 *
 * Below it the others become kin by halves, a half a notch, until at the bottom
 * a bird aligns with and closes on every bird it sees: one flock in two or three
 * colours, each bird's nearest neighbour a stranger as often as chance says. The
 * leash that keeps a flock together and the pace that tells flocks apart let go
 * with it, or the colours sort themselves out again.
 *
 * Above it each flock is sent a room away from the others, up to twice the room
 * it used to keep, and a bird steers away from the strangers it can see, which
 * is what makes two flocks that meet part around each other rather than pass
 * through. At the top a stranger in sight weighs what the flock's own heading
 * does by default. The panel shows the whole bar as a factor, a quarter a notch. */
static const double AVOID_WEIGHT_MAX = 1.5;

#define ALT_SCREEN_ON "\033[?1049h"
#define ALT_SCREEN_OFF "\033[?1049l"
#define CURSOR_HIDE "\033[?25l"
#define CURSOR_SHOW "\033[?25h"
#define SYNC_UPDATE_END "\033[?2026l"
/* Any event tracking plus SGR coordinates: 1003 reports plain motion as well as
 * clicks, and 1006 lifts the 223 column ceiling of the original encoding. */
#define MOUSE_REPORT_ON "\033[?1003h\033[?1006h"
#define MOUSE_REPORT_OFF "\033[?1006l\033[?1003l"
#define KITTY_FREE_IMAGES "\033_Ga=d,d=A,q=2\033\\"

typedef struct {
    double x, y;
} vector_t;

typedef struct {
    float cosine, sine;
} trig_entry_t;

static trig_entry_t trig_lookup_table[TRIG_LOOKUP_SIZE];

/* Built once before any flock is run. The simulation keeps its continuous
 * double direction; only the unit vector read hundreds of thousands of times by
 * the neighbour loop is rounded to the nearest table entry. */
static void trig_lookup_init(void) {
    for (int i = 0; i < TRIG_LOOKUP_SIZE; i++) {
        double angle = (double)i * 2 * M_PI / TRIG_LOOKUP_SIZE;
        trig_lookup_table[i].cosine = (float)cos(angle);
        trig_lookup_table[i].sine = (float)sin(angle);
    }
}

static trig_entry_t trig_lookup(double angle) {
    double scaled = angle * (TRIG_LOOKUP_SIZE / (2 * M_PI));
    int nearest = (int)(scaled + (scaled >= 0 ? 0.5 : -0.5));
    return trig_lookup_table[(unsigned)nearest & TRIG_LOOKUP_MASK];
}

typedef struct {
    double x, y, direction;
    int frame;
    int shade; /* Index into the palette, and half of the image id. */
    int flock; /* Which flock it reads: separation ignores this, the rest does not. */
    int layer; /* Near or far; the two never see each other. */
    int wing;  /* Where in the beat it is: an index into WING_SEQUENCE. */
    int shape; /* In three dimensions, which of the shapes of its size it is drawn as. */
    double wing_clock;
    double gliding; /* Seconds of wings held out and still. */
    int alarmed;    /* Swerving in an escape wave, which is what makes it light. */
    double trail_x[TRAIL_LENGTH], trail_y[TRAIL_LENGTH];
    int trail_at, trail_held;
    /* A letter that is at home and not flying: it neither moves nor is seen by the
     * birds that do. Always zero for a bird. */
    int perched;
    double scattered; /* Seconds a bird of a sign stays away from its place. */
} bird_t;

typedef struct {
    uint8_t *data;
    size_t length;
} image_frame_t;

typedef struct {
    int width, height, cols, rows;
    int cell_width, cell_height, turn_x, turn_y, turn_bottom;
    int legend_width, legend_height; /* The panel in pixels, zero when there is none. */
} screen_t;

typedef struct {
    int birds, bird_size, palette, flocks;
    int trails, hawks, shape;
    int turning_notch;
    double speed;      /* Pixels a bird covers this frame, at the chosen pace. */
    double base_speed; /* The same at pace one, which is what the way out flies at. */
    double pace;       /* The speed slider's factor; one is the shipped flock. */
    int vision_cells, vision_radius, vision_radius_squared;
    double separation, alignment, boundary;
    /* Notch positions, zero to LEGEND_BAR_CELLS. These are the state the keys
     * move; every value above is derived from them, which is what makes one
     * keypress exactly one notch of bar rather than nearly one. */
    int boundary_notch, separation_notch, alignment_notch;
    int vision_notch, pace_notch;
    int avoid_notch;
    double avoid_kinship; /* How much of kin a stranger is: one at the bottom, then halves. */
    double avoid_room;    /* The room between flock homes, as a share of 2 * FLOCK_LEASH. */
    double avoid_weight;  /* And the weight of a bird's wariness of strangers. */
} config_t;

static config_t config = {
    .birds = 800,
    .speed = DEFAULT_SPEED,
    .base_speed = DEFAULT_SPEED,
    .pace = DEFAULT_PACE,
    .bird_size = 0, /* not given: settle_the_bird_size makes it 30 */
    .palette = 0,
    .flocks = 1,
    .turning_notch = DEFAULT_TURNING_NOTCH,
    .vision_cells = DEFAULT_VISION_RADIUS / SPATIAL_CELL_SIZE,
    .vision_radius = DEFAULT_VISION_RADIUS,
    .vision_radius_squared = DEFAULT_VISION_RADIUS * DEFAULT_VISION_RADIUS,
    .separation = DEFAULT_SEPARATION_W,
    .alignment = DEFAULT_ALIGNMENT_W,
    .boundary = DEFAULT_BOUNDARY_W,
    .boundary_notch = DEFAULT_NOTCH,
    .separation_notch = DEFAULT_NOTCH,
    .alignment_notch = DEFAULT_NOTCH,
    /* Twelve to sixty pixels in steps of four: thirty six is the sixth notch. */
    .vision_notch = DEFAULT_VISION_NOTCH,
    .pace_notch = DEFAULT_PACE_NOTCH,
    .avoid_notch = DEFAULT_NOTCH,
};
static screen_t screen;
static int legend_enabled; /* Hidden until --panel or h asks for it. */
/*
 * How the frame reaches the screen.
 *
 * Braille, in every terminal, unless asked otherwise: the frame is rendered to
 * pixels exactly as it is for a recording, and the pixels are read back as
 * braille — eight dots a cell, the finest thing text can do — or as sextants or
 * half blocks, each cell in the colour of the bird in it. Kitty's graphics
 * protocol draws real sprites, and only when --render kitty asks for it: Kitty
 * and Ghostty place them right, and elsewhere what it does is undefined. Other
 * terminals answer for the protocol and then place nothing, too few birds or
 * the wrong ones, so no terminal is guessed at.
 */
typedef enum {
    RENDER_UNSET = -1, /* Not asked for: braille live, sprites in a recording. */
    RENDER_KITTY,
    RENDER_BRAILLE,
    RENDER_SEXTANTS, /* Solid two by three blocks: bolder than dots, needs a 2020 font. */
    RENDER_BLOCKS,
} render_mode_t;
static const char *const RENDER_NAMES[] = {"kitty", "braille", "sextants", "blocks", NULL};
static int render_mode = RENDER_UNSET;
/* --depth: a second plane of birds further off. The default is the one. */
static int deep_look;
/* --fireflies: a summer night. The birds become fireflies that drift and flash
 * instead of a flock, and the program says so in the places that differ. */
static int fireflies_mode;
static fireflies_t night;
/* Text as the flock: --text, or text piped in. The letters are the birds, one for
 * each, and letters.c holds what is about text. */
static const char *text_path;
static int letters_mode;
static letters_t the_letters;
/* The text as it was read, so that it can be laid out again when the window changes
 * size, and how long the window has been a size the letters are not laid out for. */
static uint8_t *the_text;
static size_t the_text_length;
static double reflow_wait;
/* Where the keys come from, and the terminal's modes are set and queried: the
 * standard input, unless that is a pipe with text in it, in which case the
 * controlling terminal. */
static int input_fd = STDIN_FILENO;
/* --3d: the flock flies in a space and is seen from a camera that orbits its
 * roost. The birds below are then only what the camera sees of it, one frame at a
 * time: a place on the screen, which way it points there, and a size. */
static int sky_mode;

/* What a picture is painted in: the flat sky has two planes, far then near, and
 * the space has a bin for every size, farthest first. */
static int layer_count(void) {
    return sky_mode ? SKY_BINS : LAYERS;
}

static int layer_in_pass(int pass) {
    return sky_mode ? pass : LAYERS - 1 - pass;
}

/* How big a bin's sprite is. The bird at the middle of the flock's depth is the
 * size that was asked for on a picture 512 pixels high, and the sprites grow with
 * the picture: the flock is framed to fill it, so on a bigger one the same bird
 * would be a speck. The height they were built for is kept, because where they
 * are placed must agree with what they were drawn at, and they are rebuilt only
 * once the window has settled at another size (fit_the_sprites_to_the_window). */
static int sky_picture_size;

static int sky_bin_size(int bin) {
    double picture =
        sky_picture_size > 0 ? sky_picture_size : sky_picture(screen.width, screen.height);
    int size = (int)(config.bird_size * picture / 512.0 * sky_bin_scale(bin) + 0.5);
    return size < MIN_BIRD_SIZE ? MIN_BIRD_SIZE : size;
}

static int drawing_with_text(void) {
    return render_mode == RENDER_BRAILLE || render_mode == RENDER_SEXTANTS ||
           render_mode == RENDER_BLOCKS;
}

static cells_style_t text_style(void) {
    if (render_mode == RENDER_SEXTANTS) return CELLS_SEXTANTS;
    if (render_mode == RENDER_BLOCKS) return CELLS_BLOCKS;
    return CELLS_BRAILLE;
}

/* Where the pointer is, in pixels, and whether it has ever been seen. The
 * terminal reports cells, so the position is the middle of the cell it names:
 * that is as precise as the protocol gets. */
static struct {
    int present;
    double x, y;
    /* How fast it is going, in pixels a second on the clock, read over a short
     * stretch of reports because one is a cell and a cell is a lot of pixels. */
    double velocity_x, velocity_y;
    double anchor_x, anchor_y, anchor_at; /* Where, and when, the last stretch began. */
    double moved_at;                      /* The last report. */
} mouse;

/* A monotonic clock for everything that animates on its own: the frame counter
 * for anything that wants to act every so many frames, the seconds for anything
 * that has to look the same whatever the frame rate. */
static struct {
    long frame;
    double seconds;
} clock_state;

/* How much real or recorded time the next simulation step represents. A normal
 * live frame is about 1/60 s, a picture renderer about 1/30 s, and an unlocked
 * frame whatever elapsed since the previous one. */
static double frame_seconds = 1.0 / FRAME_RATE;

/* How much flying the next step represents: the frame's time at the pace the
 * speed slider asks for. What a bird or a hawk covers, what it may turn through
 * and how long a hawk holds a chase run on this. The wing beats, the clocks and
 * the way out run on the frame's own time: six beats a second reads as effort
 * at any speed, and at two and a half times it would strobe. */
static double flight_seconds(void) {
    return frame_seconds * config.pace;
}

/* How far a bird flies in a second of flight, at the screen it is on: the same
 * as config.base_speed over the frame, kept as a figure of its own because the
 * frame can be of no length at all. */
static double flight_pixels_per_second = DEFAULT_SPEED * FRAME_RATE;

/* Paused holds the simulation still but keeps drawing and reading keys, so the
 * panel still answers and a single step is possible. Stepping is one frame of
 * motion granted while paused. */
static int paused;
static int step_once;
static int population_changed;
/* Whether the panel is currently on screen. The panel is anchored at the origin
 * and constant in cells, so it never leaves text behind by moving: the only row
 * ever needing an erase is one it occupied before being switched off. Clearing
 * the whole screen would take the uploaded sprites with it. */
static int legend_drawn;
static struct termios saved_termios;
static volatile sig_atomic_t terminal_is_raw;
static volatile sig_atomic_t terminal_restored;
static volatile sig_atomic_t alt_screen_is_on;
static volatile sig_atomic_t sprites_uploaded;

static void write_all(const void *data, size_t length) {
    const char *bytes = data;
    while (length > 0) {
        ssize_t written = write(STDOUT_FILENO, bytes, length);
        if (written < 0) {
            if (errno == EINTR) continue;
            return;
        }
        if (written == 0) return;
        bytes += written;
        length -= (size_t)written;
    }
}

static void restore_terminal(void) {
    if (terminal_restored) return;
    terminal_restored = 1;
    if (terminal_is_raw) {
        tcsetattr(input_fd, TCSAFLUSH, &saved_termios);
        terminal_is_raw = 0;
    }
    if (!alt_screen_is_on) return; /* The probe failed before we took the screen. */
    /* The sprites were uploaded once and outlive the frames that placed them:
     * the lowercase delete every frame sends clears placements only. Uppercase
     * frees every image left without one, so the terminal is not holding a few
     * megabytes of birds after they have flown. */
    if (sprites_uploaded) write_all(KITTY_FREE_IMAGES, sizeof(KITTY_FREE_IMAGES) - 1);
    write_all(MOUSE_REPORT_OFF, sizeof(MOUSE_REPORT_OFF) - 1);
    write_all(SYNC_UPDATE_END, sizeof(SYNC_UPDATE_END) - 1);
    write_all(CURSOR_SHOW, sizeof(CURSOR_SHOW) - 1);
    write_all(ALT_SCREEN_OFF, sizeof(ALT_SCREEN_OFF) - 1);
}

static void signal_handler(int signal_number) {
    restore_terminal();
    _exit(128 + signal_number);
}

static void install_signal_handlers(void) {
    static const int signals[] = {SIGINT,  SIGTERM, SIGHUP, SIGQUIT,
                                  SIGSEGV, SIGFPE,  SIGBUS, SIGABRT};
    struct sigaction action;
    memset(&action, 0, sizeof(action));
    action.sa_handler = signal_handler;
    action.sa_flags = (int)SA_RESETHAND;
    sigemptyset(&action.sa_mask);
    for (size_t i = 0; i < sizeof(signals) / sizeof(*signals); i++)
        sigaction(signals[i], &action, NULL);
    /* A reader that goes away, as head does, is an error to report rather than a
     * death: SIGPIPE's default kills the process before the terminal is put back,
     * and leaves the shell without echo. Ignored, the write fails with EPIPE and
     * the program leaves through exit, which restores it. */
    action.sa_handler = SIG_IGN;
    action.sa_flags = 0;
    sigaction(SIGPIPE, &action, NULL);
}

/* Sends a request and collects whatever comes back until a terminator or the
 * deadline, whichever is first. The colour queries ask the terminal a question
 * it may simply not answer, and may not hang the startup path waiting for a
 * reply that is never coming. Raw mode has to be on already, or the reply would
 * be echoed and held until a newline. */
static size_t terminal_query(const char *request, size_t request_length, char *reply,
                             size_t reply_size, int milliseconds) {
    struct timespec start, now;
    size_t length = 0;

    if (reply_size == 0) return 0;
    reply[0] = '\0';
    /* The question goes out the way the answer comes back: through the controlling
     * terminal when that is where the keys are read, since the standard output may
     * be anywhere. */
    if (input_fd == STDIN_FILENO) {
        write_all(request, request_length);
    } else {
        const char *bytes = request;
        size_t left = request_length;
        while (left > 0) {
            ssize_t written = write(input_fd, bytes, left);
            if (written < 0 && errno == EINTR) continue;
            if (written <= 0) break;
            bytes += written;
            left -= (size_t)written;
        }
    }
    clock_gettime(CLOCK_MONOTONIC, &start);

    for (;;) {
        clock_gettime(CLOCK_MONOTONIC, &now);
        long spent = (now.tv_sec - start.tv_sec) * 1000L + (now.tv_nsec - start.tv_nsec) / 1000000L;
        if (spent >= milliseconds) break;

        /* select and not poll: macOS's poll does not answer for /dev/tty, which is
         * where the keys come from when the standard input is a pipe. */
        fd_set readable;
        FD_ZERO(&readable);
        FD_SET(input_fd, &readable);
        long left = milliseconds - spent;
        struct timeval timeout = {left / 1000, (left % 1000) * 1000};
        int ready = select(input_fd + 1, &readable, NULL, NULL, &timeout);
        if (ready < 0) {
            if (errno == EINTR) continue;
            break;
        }
        if (ready == 0) break;

        ssize_t got = read(input_fd, reply + length, reply_size - 1 - length);
        if (got <= 0) break;
        length += (size_t)got;
        reply[length] = '\0';
        /* Every reply this program asks for ends one of these three ways. */
        if (memchr(reply, '\a', length) != NULL || strstr(reply, "\033\\") != NULL ||
            memchr(reply, 'c', length) != NULL)
            break;
        if (length + 1 >= reply_size) break;
    }
    return length;
}

/* What a live run draws with: braille unless something else was asked for. */
static int live_render_mode(void) {
    /* Letters are text: --render kitty draws sprites, and a letter has none. */
    if (letters_mode) return RENDER_BRAILLE;
    return render_mode == RENDER_UNSET ? RENDER_BRAILLE : render_mode;
}

static void enter_alt_screen(void) {
    write_all(ALT_SCREEN_ON, sizeof(ALT_SCREEN_ON) - 1);
    write_all(CURSOR_HIDE, sizeof(CURSOR_HIDE) - 1);
    write_all(MOUSE_REPORT_ON, sizeof(MOUSE_REPORT_ON) - 1);
    alt_screen_is_on = 1;
}

static int enter_terminal(void) {
    struct termios raw;
    if (tcgetattr(input_fd, &raw) < 0) return -1;
    saved_termios = raw;
    raw.c_iflag &= (tcflag_t) ~(tcflag_t)(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
    raw.c_oflag &= (tcflag_t) ~(tcflag_t)OPOST;
    raw.c_cflag |= CS8;
    raw.c_lflag &= (tcflag_t) ~(tcflag_t)(ECHO | ICANON | IEXTEN);
    raw.c_cc[VSUSP] = _POSIX_VDISABLE;
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    if (tcsetattr(input_fd, TCSAFLUSH, &raw) < 0) return -1;
    terminal_is_raw = 1;
    return 0;
}

/* Kept proportional to the viewport: a fixed pixel distance covers a short
 * terminal entirely and pins the whole flock against one edge. */
static void update_turn_distances(void) {
    screen.turn_x = screen.width / TURN_BAND_DIVISOR;
    screen.turn_y = screen.height / TURN_BAND_DIVISOR;
    screen.turn_bottom = screen.height / BOTTOM_BAND_DIVISOR;
    if (screen.turn_x < 1) screen.turn_x = 1;
    if (screen.turn_y < 1) screen.turn_y = 1;
    if (screen.turn_bottom < 1) screen.turn_bottom = 1;
}

/* The panel takes a corner rather than a row, so the flyable area stays an L and
 * no other derivation has to shrink: the flock keeps the full width below the
 * panel and the full height beside it, and is kept out of the corner by a force
 * instead of by a bound. */
static int legend_rows(void) {
    /* A night has no flocks to avoid each other, and the row is for how much of the
     * swarm is flashing together instead. */
    return config.flocks > 1 || fireflies_mode ? LEGEND_MAX_ROWS : LEGEND_ROWS;
}

static void measure_legend(void) {
    screen.legend_width = screen.legend_height = 0;
    if (!legend_enabled) return;
    if (screen.cols < LEGEND_MIN_COLS || screen.rows < LEGEND_MIN_ROWS) return;
    screen.legend_width = LEGEND_COLUMNS * screen.cell_width;
    screen.legend_height = legend_rows() * screen.cell_height;
}

/* Split out of the ioctl query so the tests drive the real derivation. */
/* Defined with the other notch arithmetic; needed here because the step a bird
 * takes is capped by the size of the screen it is taking it on. */
static void update_speed(void);

static void apply_screen_size(int cols, int rows, int pixel_width, int pixel_height) {
    screen.cols = cols > 0 ? cols : DEFAULT_COLS;
    screen.rows = rows > 0 ? rows : DEFAULT_ROWS;
    screen.width = pixel_width;
    screen.height = pixel_height;
    if (screen.width <= 0 || screen.height <= 0) {
        screen.width = screen.cols * DEFAULT_CELL_WIDTH;
        screen.height = screen.rows * DEFAULT_CELL_HEIGHT;
    }
    screen.cell_width = screen.width / screen.cols;
    screen.cell_height = screen.height / screen.rows;
    if (screen.cell_width < 1) screen.cell_width = 1;
    if (screen.cell_height < 1) screen.cell_height = 1;
    /* Before the distances, because the panel's turn zone is the panel grown by
     * one frame of travel and the frame of travel depends on the screen. */
    update_speed();
    measure_legend();
    update_turn_distances();
}

static void update_screen_dimensions(void) {
    struct winsize size;
    memset(&size, 0, sizeof(size));
    if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &size) < 0) memset(&size, 0, sizeof(size));
    /* Text is measured in cells, and a cell is eight by sixteen pixels here whatever
     * it is on this screen: the same letters fly the same way in a window and on a
     * high density display, in a recording and live. */
    if (letters_mode) size.ws_xpixel = size.ws_ypixel = 0;
    apply_screen_size(size.ws_col, size.ws_row, size.ws_xpixel, size.ws_ypixel);
}

/*
 * The terminal's own colours, asked for rather than guessed.
 *
 * OSC 4 names a palette entry, OSC 10 and 11 the foreground and background. The
 * answer comes back as rgb:RRRR/GGGG/BBBB, four hex digits a channel, and a
 * terminal that does not implement the query simply says nothing. A ramp is then
 * built between the most saturated answer and the background, which is what
 * makes a screenshot match the poster's own setup: the single thing that decides
 * whether a terminal toy looks native or looks imported.
 */
static uint8_t theme_tints[5][3];
static int theme_is_known;
/* And the terminal's background, which is what a light has to stand clear of
 * when the terminal is a light one. Until it is asked, the picture's ground. */
static uint8_t theme_ground[3] = {18, 18, 24};

static int parse_osc_colour(const char *reply, uint8_t rgb[3]) {
    const char *at = strstr(reply, "rgb:");
    unsigned r, g, b;
    if (at == NULL) return 0;
    if (sscanf(at + 4, "%4x/%4x/%4x", &r, &g, &b) != 3) return 0;
    /* Four hex digits a channel is the usual answer, but some terminals send
     * two; scale whichever came back down to a byte. */
    const char *slash = strchr(at + 4, '/');
    int digits = slash ? (int)(slash - (at + 4)) : 4;
    int shift = digits >= 4 ? 8 : 0;
    rgb[0] = (uint8_t)(r >> shift);
    rgb[1] = (uint8_t)(g >> shift);
    rgb[2] = (uint8_t)(b >> shift);
    return 1;
}

static int ask_colour(const char *request, uint8_t rgb[3]) {
    char reply[128];
    if (terminal_query(request, strlen(request), reply, sizeof(reply), 60) == 0) return 0;
    return parse_osc_colour(reply, rgb);
}

static int saturation_of(const uint8_t rgb[3]) {
    int high = rgb[0] > rgb[1] ? rgb[0] : rgb[1];
    int low = rgb[0] < rgb[1] ? rgb[0] : rgb[1];
    if (rgb[2] > high) high = rgb[2];
    if (rgb[2] < low) low = rgb[2];
    return high - low;
}

/* The relative luminance the WCAG contrast ratio is built on. Used to keep the
 * flock off the terminal's own background, which is the one colour a bird must
 * never be. */
static double luminance_of(const uint8_t rgb[3]) {
    double channel[3];
    for (int c = 0; c < 3; c++) {
        double v = rgb[c] / 255.0;
        channel[c] = v <= 0.04045 ? v / 12.92 : pow((v + 0.055) / 1.055, 2.4);
    }
    return 0.2126 * channel[0] + 0.7152 * channel[1] + 0.0722 * channel[2];
}

static double contrast_between(const uint8_t a[3], const uint8_t b[3]) {
    double high = luminance_of(a), low = luminance_of(b);
    if (high < low) {
        double swap = high;
        high = low;
        low = swap;
    }
    return (high + 0.05) / (low + 0.05);
}

/* Five steps from the accent towards the background, stopping well short of it.
 * Over four steps the last shade *was* the background, byte for byte: a fifth of
 * the flock was painted in the colour of the sky and simply did not exist, and
 * with three flocks up a whole flock went missing. Over seven, the far end still
 * reads as distance and the worst case across the common colour schemes keeps a
 * contrast of 1.8 against the ground instead of 1.0. */
static void ramp_between(const uint8_t from[3], const uint8_t to[3]) {
    for (int i = 0; i < 5; i++)
        for (int c = 0; c < 3; c++)
            theme_tints[i][c] = (uint8_t)(from[c] + (to[c] - from[c]) * i / 7);
}

static int learn_the_theme(void) {
    uint8_t accent[3] = {0, 0, 0}, background[3] = {0, 0, 0};
    double best = -1;

    /* The background first, because the accent is chosen against it. */
    if (!ask_colour("\033]11;?\033\\", background)) {
        /* No background: fade towards black, which is the common case. */
        background[0] = background[1] = background[2] = 0;
    }
    memcpy(theme_ground, background, sizeof(theme_ground));
    /* Entries one to six are the terminal's own reds through cyans, which is
     * where a colour scheme keeps its character. The most saturated of them is
     * not always the one to take: on stock xterm that is pure blue on black,
     * which is both the dimmest colour on the screen and the ugliest. Saturation
     * times contrast picks the colour with character that can also be seen. */
    for (int entry = 1; entry <= 6; entry++) {
        char request[32];
        uint8_t rgb[3];
        snprintf(request, sizeof(request), "\033]4;%d;?\033\\", entry);
        if (!ask_colour(request, rgb)) continue;
        double score = saturation_of(rgb) * contrast_between(rgb, background);
        if (score > best) {
            best = score;
            memcpy(accent, rgb, sizeof(accent));
        }
    }
    if (best < 0) return 0;
    ramp_between(accent, background);
    theme_is_known = 1;
    return 1;
}

/*
 * Ink: the terminal's own text colour, fading into its own ground.
 *
 * A murmuration is a dark shape on a pale sky or a pale one on a dark sky, and
 * the terminal already says which it is: the foreground is what its text is
 * written in, the background what it is written on. Five shades from one
 * towards the other make the flock the terminal's own ink, grey on a black
 * terminal and near black on a white one, where a fixed ramp is right on only
 * one of them. It is built at startup, like theme, from OSC 10 and 11, and
 * kept apart from theme's because the two are different questions: theme wants
 * the terminal's most colourful colour and ink its plainest.
 */
static uint8_t ink_tints[5][3];
static uint8_t ink_ground[3];
static int ink_is_known;

/* A tint pulled a share of the way to the ground it is seen against: all that
 * distance does to a colour, and the one place the arithmetic is written. */
static void pulled_towards(const uint8_t tint[3], const uint8_t ground[3], double share,
                           uint8_t out[3]) {
    for (int c = 0; c < 3; c++) out[c] = (uint8_t)(tint[c] + (ground[c] - tint[c]) * share + 0.5);
}

/* How far towards the ground the farthest shade goes when the terminal has the
 * contrast to spare, and the contrast the farthest bird is kept at when it does
 * not. Ash, which was tuned by eye on a dark ground and ends 0.69 of the way
 * from white to it, is where the reach comes from; the contrast is the two to
 * one that the far birds in three dimensions were found to need, because at 1.4
 * a bird on a terminal is not there at all. */
static const double INK_REACH = 0.7;
static const double INK_FAR_CONTRAST = 2.0;

/* `pull` is how far the renderer will dim the farthest bird again on its way to
 * the ground, and the ramp is cut short until that bird, as it is drawn, still
 * keeps its contrast: the dimming is why a fixed fraction would not do, and why
 * a pair with little contrast to begin with gets a short ramp rather than one
 * that runs into the ground. The loop settles to the nearest hundredth, and
 * measures the bird as the bytes it is drawn in, so rounding cannot take the
 * floor away. A terminal whose text has no contrast to spare gets its text colour
 * five times over, which is a flock with no depth and not one in the background. */
static void build_the_ink(const uint8_t foreground[3], const uint8_t background[3], double pull) {
    double reach = INK_REACH;
    for (;;) {
        for (int shade = 0; shade < 5; shade++)
            for (int c = 0; c < 3; c++)
                ink_tints[shade][c] =
                    (uint8_t)(foreground[c] + (background[c] - foreground[c]) * reach * shade / 4 +
                              0.5);
        uint8_t drawn[3];
        pulled_towards(ink_tints[4], background, pull, drawn);
        if (reach <= 0 || contrast_between(drawn, background) >= INK_FAR_CONTRAST) break;
        reach = reach > 0.01 ? reach - 0.01 : 0;
    }
    memcpy(ink_ground, background, sizeof(ink_ground));
    ink_is_known = 1;
}

/* Both colours or neither: a ramp from a foreground that was guessed would be
 * invisible on exactly the terminals that need it. The background is asked first
 * and a silent terminal costs one wait, not two. */
static int learn_the_ink(double pull) {
    uint8_t foreground[3], background[3];
    if (!ask_colour("\033]11;?\033\\", background)) return 0;
    if (!ask_colour("\033]10;?\033\\", foreground)) return 0;
    build_the_ink(foreground, background, pull);
    return 1;
}

/*
 * A palette is a list of tints applied to the one embedded sprite. The first
 * entry of every palette is the sprite untouched, so a bird with no shade of
 * its own looks exactly as it always did.
 */
typedef struct {
    const char *name;
    const char *help;
    int shades;
    const uint8_t (*tints)[3];
    png_tint_mode_t mode;
} palette_t;

static const uint8_t EMBER_TINTS[][3] = {
    {255, 214, 138}, {255, 176, 66}, {247, 122, 41}, {224, 74, 39}, {173, 44, 51},
};
static const uint8_t ICE_TINTS[][3] = {
    {226, 246, 255}, {160, 220, 250}, {96, 176, 236}, {58, 122, 206}, {44, 74, 158},
};
static const uint8_t ACID_TINTS[][3] = {
    {238, 255, 176}, {186, 244, 96}, {118, 214, 74}, {54, 176, 108}, {26, 122, 106},
};
static const uint8_t MATRIX_TINTS[][3] = {
    {198, 255, 198}, {120, 246, 120}, {54, 210, 70}, {26, 150, 48}, {12, 92, 30},
};
static const uint8_t AURORA_TINTS[][3] = {
    {206, 255, 222}, {110, 240, 170}, {44, 204, 170}, {60, 140, 210}, {110, 84, 200},
};
/* A ramp is read in order, not by brightness: one flock is coloured along it by
 * heading, a shade at a time as a bird turns, and several flocks from its ends
 * inwards. So prism goes round the rainbow instead of from light to dark, and
 * potion is two hues that meet with nothing between them. */
static const uint8_t PRISM_TINTS[][3] = {
    {255, 92, 92}, {255, 196, 64}, {96, 220, 110}, {80, 160, 255}, {176, 110, 255},
};
static const uint8_t POTION_TINTS[][3] = {
    {170, 255, 110}, {72, 214, 104}, {206, 160, 255}, {160, 104, 240}, {118, 64, 206},
};
static const uint8_t DUSK_TINTS[][3] = {
    {255, 214, 170}, {255, 148, 120}, {232, 86, 136}, {160, 70, 170}, {92, 64, 168},
};
static const uint8_t ASH_TINTS[][3] = {
    {244, 244, 246}, {206, 208, 214}, {164, 168, 178}, {124, 128, 140}, {88, 92, 104},
};
/* Read as a flash dying: the pale yellow of the instant it lights, through the
 * yellow green of a firefly's own light, to the dark green it fades into. Shade
 * zero is the brightest, which is the opposite way round from most ramps, because
 * here the shade is not a colour a bird wears but how long ago it flashed. */
static const uint8_t FIREFLY_TINTS[][3] = {
    {255, 250, 190}, {236, 238, 84}, {176, 216, 56}, {98, 170, 52}, {48, 106, 50},
};
/* What the embedded sprite is actually painted, for the palettes that leave it
 * alone: a hawk still has to stand off that. */
static const uint8_t SPRITE_OWN_COLOUR[3] = {237, 28, 36};

static const palette_t PALETTES[] = {
    {"theme", "the terminal's own colours, asked for at startup", 5,
     (const uint8_t (*)[3])theme_tints, PNG_TINT_REPLACE},
    {"ember", "embers, pale gold to deep red", 5, EMBER_TINTS, PNG_TINT_REPLACE},
    {"ice", "ice, white through to deep blue", 5, ICE_TINTS, PNG_TINT_REPLACE},
    {"acid", "acid, lime through to teal", 5, ACID_TINTS, PNG_TINT_REPLACE},
    {"matrix", "the green of the film it is named after", 5, MATRIX_TINTS, PNG_TINT_REPLACE},
    {"aurora", "the northern lights, mint through to violet", 5, AURORA_TINTS, PNG_TINT_REPLACE},
    {"prism", "light through a prism, red to violet", 5, PRISM_TINTS, PNG_TINT_REPLACE},
    {"potion", "two potions that will not mix, green and violet", 5, POTION_TINTS,
     PNG_TINT_REPLACE},
    {"dusk", "the sky at dusk, peach through to indigo", 5, DUSK_TINTS, PNG_TINT_REPLACE},
    {"ash", "ash, white through to slate grey", 5, ASH_TINTS, PNG_TINT_REPLACE},
    {"firefly", "a flash dying, pale yellow through to dark green", 5, FIREFLY_TINTS,
     PNG_TINT_REPLACE},
    {"ink", "the terminal's text colour, fading into its background", 5,
     (const uint8_t (*)[3])ink_tints, PNG_TINT_REPLACE},
};
enum { PALETTE_COUNT = sizeof(PALETTES) / sizeof(*PALETTES) };

static const char *PALETTE_NAMES[PALETTE_COUNT + 1];

/* A PNG of somebody's own, if they gave one: it keeps its own colours, which is
 * the whole reason for drawing one. Every palette replaces the colour it is
 * given, so the flock used to come out flat ember whatever the artwork was. */
static const char *sprite_path;

/* --picture: a PNG the flock draws. Its colours, cut down to a ramp of the
 * program's own kind, are the palette of the run unless --color was given, and
 * then the picture's light and dark pick shades of that. See the sign, below. */
static const char *picture_path;
static png_image_t picture_image;
static int picture_colours_in_use;
static int palette_was_asked_for;
static uint8_t picture_tints[MAX_PALETTE_SHADES][3];
static palette_t picture_palette = {"picture", "the colours of the picture", 0, NULL,
                                    PNG_TINT_REPLACE};

/* Named rather than numbered: the table's order is a presentation choice and
 * should not be load bearing. */
static int palette_named(const char *name) {
    for (int i = 0; i < PALETTE_COUNT; i++)
        if (strcmp(PALETTES[i].name, name) == 0) return i;
    return 0;
}

static int palette_follows_the_theme(void) {
    return !picture_colours_in_use && strcmp(PALETTES[config.palette].name, "theme") == 0;
}

static int palette_is_ink(void) {
    return strcmp(PALETTES[config.palette].name, "ink") == 0;
}

/* Whether the colour of the terminal's ground is known: only ink asks for it. */
static int the_ground_is_known(void) {
    return palette_is_ink() && ink_is_known;
}

#define FALLBACK_PALETTE palette_named("ember")
/* Where ink goes when there is no terminal to ask, or it does not answer: the
 * ramp that was the three dimensional default before ink, for a dark ground. */
#define FALLBACK_INK palette_named("ash")

static void name_the_palettes(void) {
    for (int i = 0; i < PALETTE_COUNT; i++) PALETTE_NAMES[i] = PALETTES[i].name;
    PALETTE_NAMES[PALETTE_COUNT] = NULL;
}

static const palette_t *palette(void) {
    return picture_colours_in_use ? &picture_palette : &PALETTES[config.palette];
}

static int palette_shades(void) {
    if (sprite_path != NULL) return 1; /* One set, untinted, as it was drawn. */
    return palette()->shades;
}

/* A hawk is not one of the flock's shades. What has to read instantly is that
 * this one is different.
 *
 * It used to be a dark silhouette, which is what a hawk looks like against the
 * sky and what nothing looks like against a black terminal: barely a twentieth of
 * a stop above the background, invisible in every recording. Scarlet fixed that
 * everywhere except the warm ramps, where scarlet is just another ember — against
 * ember the contrast was 1.16, which is no contrast at all. So there are three
 * colours and the palette picks: whichever of them stands furthest from the
 * nearest thing the flock is wearing. */
static const uint8_t HAWK_COLOURS[][3] = {
    {255, 60, 72},   /* Scarlet, for every cold or grey ramp. */
    {255, 246, 210}, /* A hot near-white, against a dark ramp, or one with red and blue in it. */
    {96, 226, 255},  /* And an electric cyan, for the ramps that are already fire. */
};
enum { HAWK_COLOUR_COUNT = sizeof(HAWK_COLOURS) / sizeof(*HAWK_COLOURS) };
static const double HAWK_GROUND_CONTRAST = 3.0;

/* How far apart two colours look, which is not how far apart their brightnesses
 * are: scarlet and pale ice blue are a stone's throw apart by luminance and could
 * not be more different to look at. The usual weighted RGB distance, good enough
 * for choosing between two candidates. */
static double colour_distance(const uint8_t a[3], const uint8_t b[3]) {
    double mean_red = (a[0] + b[0]) / 2.0;
    double dr = a[0] - b[0], dg = a[1] - b[1], db = a[2] - b[2];
    return sqrt((2 + mean_red / 256) * dr * dr + 4 * dg * dg +
                (2 + (255 - mean_red) / 256) * db * db);
}

static const uint8_t *hawk_colour(void) {
    const palette_t *chosen = palette();
    /* Against artwork nobody has looked at, scarlet and no cleverness. */
    if (sprite_path != NULL) return HAWK_COLOURS[0];
    int best = 0;
    double best_gap = -1;
    for (int candidate = 0; candidate < HAWK_COLOUR_COUNT; candidate++) {
        /* A hawk has to be seen against the ground as well as the flock. Only a
         * ground that was asked for is known, and a white one rules out the near
         * white and the cyan, which are far from black ink and not from paper. */
        if (the_ground_is_known() &&
            contrast_between(HAWK_COLOURS[candidate], ink_ground) < HAWK_GROUND_CONTRAST)
            continue;
        double gap = 1e9;
        for (int shade = 0; shade < chosen->shades; shade++) {
            const uint8_t *tint = chosen->tints != NULL ? chosen->tints[shade] : SPRITE_OWN_COLOUR;
            double against = colour_distance(HAWK_COLOURS[candidate], tint);
            if (against < gap) gap = against;
        }
        if (gap > best_gap) {
            best_gap = gap;
            best = candidate;
        }
    }
    return HAWK_COLOURS[best];
}

static void hawk_tint(png_image_t *image) {
    const uint8_t *colour = hawk_colour();
    png_tint(image, colour[0], colour[1], colour[2], PNG_TINT_REPLACE);
}

/* A bird in an escape wave is lit, and what it is lit in has to stand clear of
 * everything else on the screen: the ramp, the hawk, and the ground. White is
 * the first choice, because it is light, and is taken wherever it stands clear.
 * On ice and ash it does not, since their palest shade is already nearly white
 * and a fifth of the flock wears it, and a lit bird that looks like an unlit one
 * is no wave; there the next candidate that stands clear is taken, and where none
 * does, the one that stands furthest. Prism has a cream hawk, which white is
 * too near, and pale ice is the one that is not.
 *
 * The ground has to show it too, which is a matter of how bright it is against
 * the ground and not of how different a colour it is: on the terminal's own
 * colours the ground can be a light one, and then the light of a wave is dark. */
static const uint8_t HIGHLIGHT_COLOURS[][3] = {
    {255, 255, 255}, /* White. */
    {255, 244, 200}, /* Cream. */
    {255, 226, 120}, /* Gold: against the cold ramps, which are pale at one end. */
    {206, 240, 255}, /* Pale ice, for a hawk that is already cream. */
    {200, 255, 215}, /* Mint. */
    {255, 214, 230}, /* Blush. */
    {226, 212, 255}, /* Lilac. */
    {28, 28, 36},    /* Ink, for a ground that is light. */
};
enum { HIGHLIGHT_COLOUR_COUNT = sizeof(HIGHLIGHT_COLOURS) / sizeof(*HIGHLIGHT_COLOURS) };
static const double HIGHLIGHT_CLEARANCE = 140.0;
static const double HIGHLIGHT_CONTRAST = 3.0; /* Against the ground, as a reader would ask. */

/* Shade zero of a tinted palette is still a tint: the list is the whole ramp. */
static void palette_tint(png_image_t *image, int shade) {
    const palette_t *chosen = palette();
    if (sprite_path != NULL) return; /* Somebody's own artwork, left alone. */
    if (chosen->tints == NULL) return;
    if (shade < 0) shade = 0;
    if (shade >= chosen->shades) shade = chosen->shades - 1;
    png_tint(image, chosen->tints[shade][0], chosen->tints[shade][1], chosen->tints[shade][2],
             chosen->mode);
}

static const uint8_t *highlight_colour(void) {
    const palette_t *chosen = palette();
    const uint8_t *hawk = hawk_colour();
    /* The terminal's own ground, when theme or ink has asked it: ink on a white
     * terminal is a flock of near black, and its light has to be dark to be seen. */
    const uint8_t *ground = the_ground_is_known() ? ink_ground : theme_ground;
    int best = 0;
    double best_gap = -1;
    for (int candidate = 0; candidate < HIGHLIGHT_COLOUR_COUNT; candidate++) {
        const uint8_t *colour = HIGHLIGHT_COLOURS[candidate];
        int shows = contrast_between(colour, ground) >= HIGHLIGHT_CONTRAST;
        double gap = colour_distance(colour, hawk);
        /* Somebody's own artwork has no ramp to stand clear of, only the hawk. */
        for (int shade = 0; sprite_path == NULL && shade < chosen->shades; shade++) {
            const uint8_t *tint = chosen->tints != NULL ? chosen->tints[shade] : SPRITE_OWN_COLOUR;
            double against = colour_distance(colour, tint);
            if (against < gap) gap = against;
        }
        if (!shows) gap -= 1000; /* Whatever else it is, it is not seen. */
        if (gap >= HIGHLIGHT_CLEARANCE) return colour;
        if (gap > best_gap) {
            best_gap = gap;
            best = candidate;
        }
    }
    return HIGHLIGHT_COLOURS[best];
}

static void highlight_tint(png_image_t *image) {
    const uint8_t *colour = highlight_colour();
    png_tint(image, colour[0], colour[1], colour[2], PNG_TINT_REPLACE);
}

/* Twice a bird: a hawk has to read as the bigger thing at a glance, at the
 * largest bird too, which is why its ceiling is twice the bird's. */
static int hawk_sprite_size(void) {
    int size = config.bird_size * 2;
    return size > 2 * MAX_BIRD_SIZE ? 2 * MAX_BIRD_SIZE : size;
}

/* A bird is drawn from its top left corner; a hawk from its middle, so that the
 * point the flock flees and the silhouette on the screen are the same place. One
 * function, because the live frame and the recorder both have to agree. */
static int hawk_draw_offset(void) {
    return hawk_sprite_size() / 2;
}

/* In a space a hawk is three times the bird of its size, whichever it is: it has to
 * be the thing the eye goes to among two thousand, and the far ones are small. */
static int hawk_size_in_layer(int layer) {
    int size = sky_bin_size(layer) * 3;
    return size > 2 * MAX_BIRD_SIZE ? 2 * MAX_BIRD_SIZE : size;
}

/*
 * The bird, or something else.
 *
 * A shape is a handful of triangles in a unit square, pointing along +x because
 * that is what frame zero means, rasterised with four samples a pixel so the
 * edges survive the rotation and the shrink. Drawing them rather than embedding
 * them means five more sprites for no more bytes, and the only external file the
 * program will ever read is one the user chose.
 */
typedef struct {
    double x[3], y[3];
} triangle_t;

typedef struct {
    const char *name;
    int count;
    const triangle_t *triangles;
    double roundness; /* Radius of a filled circle to union in, zero for none. */
} shape_t;

static const triangle_t ARROW_TRIANGLES[] = {
    {{0.05, 0.95, 0.05}, {0.15, 0.50, 0.85}},
    {{0.05, 0.45, 0.05}, {0.35, 0.50, 0.65}},
};
static const triangle_t PLANE_TRIANGLES[] = {
    {{0.10, 0.95, 0.10}, {0.44, 0.50, 0.56}}, /* Fuselage. */
    {{0.30, 0.55, 0.20}, {0.48, 0.50, 0.08}}, /* Upper wing. */
    {{0.30, 0.55, 0.20}, {0.52, 0.50, 0.92}}, /* Lower wing. */
    {{0.08, 0.22, 0.08}, {0.30, 0.50, 0.70}}, /* Tail. */
};

static const shape_t SHAPES[] = {
    {"bird", 0, NULL, 0.0}, /* The embedded drawing, not a shape at all. */
    {"arrow", 2, ARROW_TRIANGLES, 0.0},
    {"plane", 4, PLANE_TRIANGLES, 0.0},
    {"dot", 0, NULL, 0.40},
};
enum { SHAPE_COUNT = sizeof(SHAPES) / sizeof(*SHAPES) };
static const char *SHAPE_NAMES[SHAPE_COUNT + 1];

static void name_the_shapes(void) {
    for (int i = 0; i < SHAPE_COUNT; i++) SHAPE_NAMES[i] = SHAPES[i].name;
    SHAPE_NAMES[SHAPE_COUNT] = NULL;
}

static int shape_named(const char *name) {
    for (int i = 0; i < SHAPE_COUNT; i++)
        if (strcmp(SHAPES[i].name, name) == 0) return i;
    return 0;
}

static int inside_triangle(const triangle_t *t, double x, double y) {
    double d1 = (x - t->x[1]) * (t->y[0] - t->y[1]) - (t->x[0] - t->x[1]) * (y - t->y[1]);
    double d2 = (x - t->x[2]) * (t->y[1] - t->y[2]) - (t->x[1] - t->x[2]) * (y - t->y[2]);
    double d3 = (x - t->x[0]) * (t->y[2] - t->y[0]) - (t->x[2] - t->x[0]) * (y - t->y[0]);
    return (d1 >= 0 && d2 >= 0 && d3 >= 0) || (d1 <= 0 && d2 <= 0 && d3 <= 0);
}

static int inside_shape(const shape_t *shape, double x, double y) {
    if (shape->roundness > 0) {
        double dx = x - 0.5, dy = y - 0.5;
        if (dx * dx + dy * dy <= shape->roundness * shape->roundness) return 1;
    }
    for (int i = 0; i < shape->count; i++)
        if (inside_triangle(&shape->triangles[i], x, y)) return 1;
    return 0;
}

static png_status_t draw_shape(int which, int size, png_image_t *out) {
    const shape_t *shape = &SHAPES[which];
    png_status_t status = png_image_alloc(out, size, size);
    if (status != PNG_OK) return status;

    for (int py = 0; py < size; py++) {
        for (int px = 0; px < size; px++) {
            int hits = 0;
            /* Four samples a pixel: enough of an edge to survive a rotation and a
             * shrink, and cheap enough to do once at startup. */
            for (int sy = 0; sy < 2; sy++)
                for (int sx = 0; sx < 2; sx++)
                    hits += inside_shape(shape, (px + 0.25 + sx * 0.5) / size,
                                         (py + 0.25 + sy * 0.5) / size);
            uint8_t *pixel = out->pixels + ((size_t)py * (size_t)size + (size_t)px) * 4;
            pixel[0] = pixel[1] = pixel[2] = 255;
            pixel[3] = (uint8_t)(hits * 255 / 4);
        }
    }
    return PNG_OK;
}

static const char *program_name = "cbirds"; /* As invoked, for every message. */

/* A PNG of somebody's own, for a sprite or a picture: up to four megabytes of
 * it, and every way it can go wrong said as it is. */
static png_status_t load_png_file(const char *path, const char *what, png_image_t *out) {
    FILE *file = fopen(path, "rb");
    if (file == NULL) {
        fprintf(stderr, "%s: cannot open %s\n", program_name, path);
        exit(EXIT_FAILURE);
    }
    static uint8_t buffer[1 << 22]; /* Four megabytes of PNG is a generous bird. */
    size_t length = fread(buffer, 1, sizeof(buffer), file);
    /* Said as it is, rather than decoding the first four megabytes and
     * reporting a truncated file the user knows is whole. */
    int too_large = length == sizeof(buffer) && fgetc(file) != EOF;
    int unreadable = ferror(file);
    fclose(file);
    if (too_large || unreadable) {
        if (too_large)
            fprintf(stderr, "%s: %s is over 4 MB, too large for %s\n", program_name, path, what);
        else
            fprintf(stderr, "%s: cannot read %s\n", program_name, path);
        exit(EXIT_FAILURE);
    }
    return png_decode(buffer, length, out);
}

/* Whichever the user asked for: a PNG of their own, one of the drawn shapes, or
 * the drawing compiled into the binary. */
static png_status_t load_sprite(png_image_t *out) {
    if (sprite_path != NULL) return load_png_file(sprite_path, "a sprite", out);
    if (config.shape != 0) return draw_shape(config.shape, SPRITE_WORK_MAX, out);
    return png_decode(sprite_png, sprite_png_len, out);
}

static int sprite_set_count(void);

/* Kitty draws only what has been uploaded, so every set is encoded to PNG and
 * sent before the first frame. The images are the very ones the other renderers
 * read pixels from; nothing is built twice. */
static kitty_graphics_status_t upload_sprite_sets(kitty_graphics_t *graphics,
                                                  const png_image_t *frames) {
    int images = sprite_set_count() * ROTATION_FRAMES;
    for (int i = 0; i < images; i++) {
        if (frames[i].pixels == NULL) continue;
        uint8_t *encoded = NULL;
        size_t length = 0;
        if (png_encode(&frames[i], &encoded, &length) != PNG_OK) return KITTY_GRAPHICS_ERR_MEMORY;
        kitty_graphics_status_t status =
            kitty_graphics_upload_png(graphics, (uint32_t)i + 1, encoded, length);
        free(encoded);
        if (status != KITTY_GRAPHICS_OK) return status;
    }
    kitty_graphics_status_t status = kitty_graphics_delete_all_placements(graphics);
    return status == KITTY_GRAPHICS_OK ? kitty_graphics_flush(graphics) : status;
}

/*
 * The flock's own random numbers.
 *
 * --seed promises the same flock for the same seed, and rand() cannot keep that:
 * its sequence belongs to the C library, and glibc, macOS and the BSDs each draw
 * a different one from the same seed. This is glibc's generator written out, an
 * additive lagged Fibonacci over 31 words seeded by a Park and Miller LCG and
 * run 310 steps before its first number. A seed now gives the same flock
 * everywhere, and every clip recorded on Linux before it existed still comes
 * out byte for byte.
 */
enum { RANDOM_WORDS = 31, RANDOM_LAG = 3, RANDOM_WARMUP = 310, RANDOM_MAX = 2147483647 };
static struct {
    uint32_t word[RANDOM_WORDS];
    int front, rear;
} random_state;

static uint32_t next_random(void) {
    uint32_t sum = random_state.word[random_state.front] += random_state.word[random_state.rear];
    random_state.front = (random_state.front + 1) % RANDOM_WORDS;
    random_state.rear = (random_state.rear + 1) % RANDOM_WORDS;
    return sum >> 1; /* The lowest bit is the least random one. */
}

static void seed_random(unsigned seed) {
    if (seed == 0) seed = 1;
    /* glibc holds the seed as a 32 bit signed word; the wrap is spelled out so it
     * does not rest on how this compiler converts an unsigned that does not fit. */
    int64_t word = seed > INT32_MAX ? (int64_t)seed - 4294967296 : (int64_t)seed;
    random_state.word[0] = (uint32_t)seed;
    for (int i = 1; i < RANDOM_WORDS; i++) {
        /* 16807 * word % 2147483647 by Schrage's method, as glibc computes it. */
        int64_t high = word / 127773, low = word % 127773;
        word = 16807 * low - 2836 * high;
        if (word < 0) word += 2147483647;
        random_state.word[i] = (uint32_t)word;
    }
    random_state.front = RANDOM_LAG;
    random_state.rear = 0;
    for (int i = 0; i < RANDOM_WARMUP; i++) next_random();
}

static double random_unit(void) {
    return (double)next_random() / RANDOM_MAX;
}

static int direction_frame(double radians) {
    int degrees = (int)(radians * 180.0 / M_PI);
    return ((degrees % 360 + 360) % 360) / FRAME_ANGLE;
}

/* Where the force acts: the panel grown by one frame of travel. A bird just
 * outside this rectangle lands at worst one epsilon inside the panel edge, which
 * is still outside the panel itself, and by then the push is on. That is what
 * makes the panel unreachable rather than merely unwelcoming, and the margin
 * follows the distance covered by this simulation step. */
static int legend_turn_zone(double x, double y) {
    /* Over text the panel is a window laid on top of it, not a wall: the letters
     * under it have homes there, and one that can never land is not a letter. */
    return !letters_mode && screen.legend_width > 0 && x < screen.legend_width + config.speed &&
           y < screen.legend_height + config.speed;
}

/* Out through the nearer of the two open sides. The panel sits in a corner, so
 * the only ways out are right and down, and the push never aims at a screen
 * edge. The magnitude settles the direction by itself. */
static int legend_repels(const bird_t *bird, vector_t *boundary) {
    if (!legend_turn_zone(bird->x, bird->y)) return 0;
    double escape_x = screen.legend_width + config.speed - bird->x;
    double escape_y = screen.legend_height + config.speed - bird->y;
    if (escape_x <= escape_y)
        boundary->x = LEGEND_PUSH;
    else
        boundary->y = LEGEND_PUSH;
    return 1;
}

/* Spread over the region no turn band covers, so no bird starts by fleeing an
 * edge and the flock does not begin stacked on a single point. */
/*
 * The flock writes.
 *
 * A target per lit cell of the text, laid out in the rectangle the panel leaves
 * free, and a bird per target round robin so a cell with several birds on it
 * reads as a thick stroke. While it is writing, a bird steers at its target and
 * moves the smaller of its speed and the distance left, which is what lets it
 * land exactly instead of orbiting: the letters come out crisp and then breathe,
 * because the flocking terms are still there underneath.
 *
 * Targets never fall inside the panel's turn zone, because the force that keeps
 * birds off the panel is unanswerable and a target in there could never be
 * reached.
 */
/* A target for every bird there can be: a picture sends them all. */
enum { FORMATION_MAX_TARGETS = MAX_BIRDS };

static struct {
    int count;
    double x[FORMATION_MAX_TARGETS];
    double y[FORMATION_MAX_TARGETS];
    double until; /* Seconds on the clock at which to let go; negative is never. */
    int writing;
    /* What only a sign has (see below). The intro leaves all of it alone, and a
     * bird of the intro is told by nothing but its number. */
    int sign;
    double cell;                                   /* The side of one lit cell of a letter. */
    double hover;                                  /* How far a bird loops round its place. */
    int slot[MAX_BIRDS];                           /* A bird's target, or -1 if it flocks on. */
    double hover_x[MAX_BIRDS], hover_y[MAX_BIRDS]; /* Where in its loop, this frame. */
    double lift[FORMATION_MAX_TARGETS];    /* One for a target that rises with the breath. */
    double shift_y[FORMATION_MAX_TARGETS]; /* And how far it has risen, this frame. */
    int glyph[FORMATION_MAX_TARGETS];      /* Which letter of the text a target is a cell of. */
    double across[FORMATION_MAX_TARGETS];  /* How far along the text a target is, 0 to 1. */
    int keep_out;                          /* The rest of the flock stays out of box. */
    sign_box_t box;
    double band;
} formation;

static void formation_clear(void) {
    formation.count = 0;
    formation.writing = 0;
    formation.keep_out = 0;
}

/* Puts lines of text on the grid of cells, centred in the rectangle, each cell
 * `cell` pixels on a side; formation.count says how many targets that made. Each
 * line is centred on the widest. A lone line is laid out exactly as it always
 * was, to the last bit, because the intro is a lone line and every recording
 * opens with it. */
static void formation_place(const char *const *lines, int line_count, double left, double top,
                            double right, double bottom, double cell, int lift_the_colon) {
    int columns = 0;
    for (int l = 0; l < line_count; l++)
        if (font_text_width(lines[l]) > columns) columns = font_text_width(lines[l]);

    double width = columns * cell, height = sign_rows(line_count) * cell;
    double origin_x = left + (right - left - width) / 2;
    double origin_y = top + (bottom - top - height) / 2;

    int letter = 0; /* Counted through all the lines: a clock changes a letter at a time. */
    for (int l = 0; l < line_count; l++) {
        double indent = (columns - font_text_width(lines[l])) / 2.0;
        int column = 0;
        for (const char *c = lines[l]; *c != '\0'; c++) {
            const char *glyph = font_glyph(*c);
            if (glyph == NULL) continue;
            letter++;
            for (int row = 0; row < FONT_HEIGHT; row++) {
                for (int x = 0; x < FONT_WIDTH; x++) {
                    if (glyph[row * FONT_WIDTH + x] != '#') continue;
                    if (formation.count >= FORMATION_MAX_TARGETS) break;
                    double px = origin_x + (indent + column + x + 0.5) * cell;
                    double py = origin_y + (l * (FONT_HEIGHT + SIGN_LINE_GAP) + row + 0.5) * cell;
                    if (legend_turn_zone(px, py)) continue;
                    formation.x[formation.count] = px;
                    formation.y[formation.count] = py;
                    formation.lift[formation.count] = lift_the_colon && *c == ':' ? 1.0 : 0.0;
                    formation.shift_y[formation.count] = 0;
                    formation.glyph[formation.count] = letter - 1;
                    formation.across[formation.count] = (indent + column + x + 0.5) / columns;
                    formation.count++;
                }
            }
            column += FONT_ADVANCE;
        }
    }
    formation.writing = formation.count > 0;
}

/* Lays the text out and returns how many targets it made, zero if it will not
 * fit or the text has nothing to draw. */
static int formation_layout(const char *text) {
    double pad = config.bird_size * 2.0;
    double left = screen.legend_width > 0 ? screen.legend_width + config.speed + pad : pad;
    double top = pad, right = screen.width - pad, bottom = screen.height - pad;
    int columns = font_text_width(text);

    formation_clear();
    formation.sign = 0;
    if (columns <= 0 || right - left < columns || bottom - top < FONT_HEIGHT) return 0;

    /* One cell is as large as both dimensions allow, so the text fills the space
     * it has without being stretched out of shape. */
    double cell = (right - left) / columns;
    double by_height = (bottom - top) / FONT_HEIGHT;
    if (by_height < cell) cell = by_height;

    formation_place(&text, 1, left, top, right, bottom, cell, 0);
    return formation.count;
}

static int formation_target_of(int index, double *x, double *y) {
    if (!formation.writing || formation.count == 0) return 0;
    if (!formation.sign) {
        *x = formation.x[index % formation.count];
        *y = formation.y[index % formation.count];
        return 1;
    }
    /* A sign picks its writers, and they hover: a few pixels of loop of their own
     * round the place, and the colon's birds lift with the breath. */
    int target = index >= 0 && index < MAX_BIRDS ? formation.slot[index] : -1;
    if (target < 0) return 0;
    *x = formation.x[target] + formation.hover_x[index];
    *y = formation.y[target] + formation.hover_y[index] + formation.shift_y[target];
    return 1;
}

/* A bird of a sign that has been scattered is no writer until it has come back:
 * it flocks, and flees what scattered it, and then it goes home. */
static int formation_target_for(const bird_t *bird, int index, double *x, double *y) {
    if (bird->scattered > 0) return 0;
    return formation_target_of(index, x, y);
}

/*
 * The flock as a sign.
 *
 * The intro is three seconds of letters. A sign is the same idea held for as
 * long as it takes to read: --say writes what it is given, --clock the time, and
 * in both the flock stays at it, lets go now and then for a murmuration, and
 * writes it again.
 *
 * What three seconds can do without and a minute cannot is stand still. A bird
 * that lands and stops is dead on the screen after the first few seconds, so a
 * bird of a sign hovers: it flies a small loop of its own about its place, with
 * its own phase and pace, and the strokes shimmer and stay sharp. Not every bird
 * writes. Round robin over eight hundred birds would put a dozen on every cell
 * of a short word and leave nobody to fly; a sign gives a lit cell a few birds,
 * and the rest keep flocking round it, kept out of the box the text is in.
 */
typedef enum { SIGN_NONE, SIGN_SAY, SIGN_CLOCK, SIGN_PICTURE } sign_kind_t;
typedef enum { SIGN_FITS, SIGN_NO_ROOM, SIGN_TOO_FEW_BIRDS } sign_failure_t;

static const char *say_text;           /* --say, as it was typed. */
static char sign_words[SIGN_TEXT_MAX]; /* And as the font can draw it. */
static int clock_mode;
static const char *clock_start; /* --clock-at: a time to start from. */
static int screensaver_mode;

/* Two thirds of the free width, centred, which is sky on either side for the
 * flock; and three fifths of the height, so that two or three lines of a long
 * text are still lines of a sign and not the whole screen. That is for a roomy
 * screen, 160 by 45 cells and up, where it leaves half the screen outside the
 * band the rest of the flock is kept out of. On a small one it left a thin ring:
 * the band is as wide in pixels there as anywhere, and the free flock streamed
 * round the sign along the edges. So the sign takes less of a small screen, down
 * to half the width and half the height, and the writers more of the flock:
 * measured on 800 birds saying HELLO WORLD at 96 by 26 cells and 25 frames a
 * second, the sky outside the band went from 7% of the screen to 42%, the free
 * birds within a bird of the sign or an edge from 23% to 6%, and the share of the
 * sky with a bird in it from 1% to 26%. Half and half were the smallest that
 * kept the letters as large as 11 pixels a cell; a width share alone left the
 * height, which is what binds two lines, as it was. */
static const double SIGN_WIDTH_SHARE = 2.0 / 3.0, SIGN_WIDTH_SHARE_SMALL = 0.5;
static const double SIGN_HEIGHT_SHARE = 0.6, SIGN_HEIGHT_SHARE_SMALL = 0.5;
/* The largest a cell grows, in bird sizes: past two and a half the cells of a
 * short word are further apart than a bird is wide, and the letters fall apart
 * into dots. */
static const double SIGN_LARGEST_CELL = 2.5;
/* At most this share of the flock writes, whatever the text, so that there is
 * always a flock for it to be a sign in front of; and at most this many to a lit
 * cell, because past four a cell is a smudge. A small screen lets more of the
 * flock write, which is the other way round from what it sounds: the sky there
 * is small, and every bird that is not writing is one more in it. With half the
 * flock writing, a text of three lines was one bird to a cell and dim, and the
 * free birds were a cloud round it; at seven tenths it is two to a cell, the
 * letters are strokes, and there are fewer birds in the sky. HELLO WORLD is three
 * to a cell at either. */
static const double SIGN_WRITER_SHARE = 0.6, SIGN_WRITER_SHARE_SMALL = 0.7;
enum { SIGN_PER_CELL_MAX = 4, PICTURE_KEPT_MAX = 1024 };
/* A screen is small by its shorter side, in pixels: at or under SMALL_SIDE, which
 * is the 96 by 26 of a recording, it is as small as it gets, and at or over
 * ROOMY_SIDE, which is 160 by 45, it is as roomy as it needs to be, and between
 * the two the shares above go from one end to the other in a straight line. */
static const double SIGN_SMALL_SIDE = 416, SIGN_ROOMY_SIDE = 720;
/* A sign does not give up size for sky below this cell, or what the roomy shares
 * give it if that is less: at 64 by 18 with a long text the letters are already
 * nine pixels, and at seven they stopped being letters. */
static const double SIGN_SMALLEST_CELL = 8;
/* The loop is a seventh of a bird across, and never more than a fifth of the
 * distance between two cells, or the strokes blur into each other; a pixel and a
 * half at least, or there is nothing to see, and six at most. */
static const double SIGN_HOVER_SIZE = 0.14, SIGN_HOVER_CELL = 0.22;
static const double SIGN_HOVER_MIN = 1.5, SIGN_HOVER_MAX = 6.0;
/* The rest of the flock is turned away from the text about as firmly as from
 * the pointer, from three birds off at the least. */
static const double SIGN_KEEP_OUT_WEIGHT = 5.0;
static const double SIGN_KEEP_OUT_BAND = 3.0;
static const double SIGN_KEEP_OUT_STEPS = 4.0;
/* The band is never wider than this share of the screen's shorter side. At the
 * pace of a 25 frame recording four steps of flight are 150 pixels whatever the
 * screen, which on 96 by 26 cells is all the sky there is: a fifth of the shorter
 * side is 83. Measured at 25 frames, 800 birds, the free birds inside the text
 * were 0.1%, as against none, and at 60 frames the band is four steps and
 * under a fifth, as it was. On 200 by 50 cells it is as it was at either. */
static const double SIGN_KEEP_OUT_MOST = 0.2;
/* The room kept clear past the panel's own edge, when there is one: a frame of
 * flight at the shipped pace, which with the pad is more than a frame of flight at
 * any pace short of the top two or three notches, and so a letter does not land in
 * the panel's turn zone. At three frames, which is what the top notch asks, the
 * room under the panel on an eighty column terminal was a hand's breadth. */
static const double SIGN_PANEL_GAP = DEFAULT_SPEED;
/* A clock lets go as a whole for this long, once an hour: long enough to be a
 * murmuration, which is the show. The rest of the hour it only ever lets go of the
 * letters that change, and the time can be read at any moment. It used to let go
 * of every letter at each minute, and was unreadable for four seconds in sixty. */
static const double SIGN_CLOCK_FLIGHT = 3.0;
/* The colon rises by this many cells at the top of its breath. */
static const double SIGN_BREATH_LIFT = 0.5;
/* A pointer that has moved this lately is moving. A bird whose place it
 * reaches is scattered for this long after the last time it did, and a little
 * longer, bird by bird, so that they do not all come home as one. */
static const double POINTER_MOVING_SECONDS = 0.6;
static const double SCATTER_SECONDS = 1.2;
static const double SCATTER_STAGGER = 1.0;
/* A hawk scatters the places it is within this share of the way to where the
 * flock starts to flee it, for a shorter time than the pointer does. */
static const double HAWK_SCATTER = 0.35;
static const double HAWK_SCATTER_SECONDS = 0.4, HAWK_SCATTER_STAGGER = 0.6;
/* A lock command is started by the key that is about to be pressed: input in the
 * first half second is whatever started it. That is the first half second of the
 * program and not of its first frame: a space builds its flock, and a Kitty
 * terminal is sent its sprites, before there is a first frame, and a key typed in
 * all that time is somebody waking the screen, long after what started it, and
 * would be thrown away. So the grace is over when the start took longer than it. */
static const double SCREENSAVER_GRACE = 0.5;
static double launch_lag; /* Seconds from the program's first line to its first frame. */

/* What a picture brings: the share of it that is ink, which says how far apart
 * the birds that draw it are, the light and the dark of its colours, which say
 * where on a ramp of somebody else's choosing each of them goes, and the shade
 * every bird wears from the moment the picture is laid out. */
static double picture_ink, picture_dark, picture_light;
static int picture_shade[MAX_BIRDS];

static struct {
    sign_kind_t kind;
    int up;         /* Being held, as far as the flock is concerned. */
    unsigned cycle; /* Times held and let go, which varies the rhythm. */
    double until;   /* On the run's clock: when the hold or the flight is over. */
    double last_tick;
    char written[SIGN_TEXT_MAX]; /* What is up now. */
    int per_cell;                /* Birds to a lit cell, the same for every cell of it. */
    /* Why the last layout failed, if one did, and what has come of it: said once at
     * the end of the run, when the terminal is back, because inside it nobody
     * could read it. */
    sign_failure_t why;
    int needs, has; /* Birds a text takes and the flock has, when that is what failed. */
    int failures, was_up;
    int failed_columns, failed_rows;
    int twelve_hours;
    unsigned seed;     /* --seed, for what a picture picks without the flock's numbers. */
    int virtual_clock; /* The time is the start time and the run's clock, not the wall's. */
    time_t origin;
    /* The screen it was laid out for: a sign that outlives a window resize or the
     * panel being opened is laid out again, and its birds fly to the new places. */
    int key_width, key_height, key_legend_width, key_legend_height, key_birds, key_size;
} the_sign;

static int a_sign_is_asked_for(void) {
    return the_sign.kind != SIGN_NONE;
}

/* The clock's second, to the nanosecond when it is the wall's, so the colon
 * breathes on the tick; and a recording's own, from where it was told to start. */
static time_t sign_wall_time(double *into_the_second) {
    if (the_sign.virtual_clock) {
        double whole = floor(clock_state.seconds);
        *into_the_second = clock_state.seconds - whole;
        return the_sign.origin + (time_t)whole;
    }
    struct timespec now;
    clock_gettime(CLOCK_REALTIME, &now);
    *into_the_second = (double)now.tv_nsec / 1e9;
    return now.tv_sec;
}

static void sign_local_now(struct tm *local) {
    double unused;
    time_t now = sign_wall_time(&unused);
    if (localtime_r(&now, local) == NULL) memset(local, 0, sizeof(*local));
}

static void sign_text_now(char *out, size_t size) {
    if (the_sign.kind == SIGN_CLOCK) {
        struct tm local;
        sign_local_now(&local);
        sign_clock_text(&local, the_sign.twelve_hours, out, size);
    } else {
        snprintf(out, size, "%s", sign_words);
    }
}

static int sign_layout_is_current(void) {
    return the_sign.key_width == screen.width && the_sign.key_height == screen.height &&
           the_sign.key_legend_width == screen.legend_width &&
           the_sign.key_legend_height == screen.legend_height &&
           the_sign.key_birds == config.birds && the_sign.key_size == config.bird_size;
}

/* A layout starts from nothing, and remembers the screen it is for. */
static void sign_begin_layout(void) {
    formation_clear();
    formation.sign = 1;
    the_sign.key_width = screen.width;
    the_sign.key_height = screen.height;
    the_sign.key_legend_width = screen.legend_width;
    the_sign.key_legend_height = screen.legend_height;
    the_sign.key_birds = config.birds;
    the_sign.key_size = config.bird_size;
}

/* 1 on a small screen and 0 on a roomy one, and between them in a straight line. */
static double sign_smallness(void) {
    double shorter = screen.width < screen.height ? screen.width : screen.height;
    double small = (SIGN_ROOMY_SIDE - shorter) / (SIGN_ROOMY_SIDE - SIGN_SMALL_SIDE);
    return small < 0 ? 0 : (small > 1 ? 1 : small);
}

static double sign_share(double roomy, double small) {
    return roomy + sign_smallness() * (small - roomy);
}

/* How far from the text the rest of the flock is turned away: felt some steps of
 * flight before the box and not at a fixed distance, but never more than a share
 * of the screen, and never less than three birds. */
static double sign_band(void) {
    double shorter = screen.width < screen.height ? screen.width : screen.height;
    double band = SIGN_KEEP_OUT_STEPS * config.speed;
    if (band > SIGN_KEEP_OUT_MOST * shorter) band = SIGN_KEEP_OUT_MOST * shorter;
    return band < formation.band ? formation.band : band;
}

/* The sign that fits in a room: the cleaned text wrapped onto the lines that make
 * its cell largest, in the share of the room the screen allows, which is less of
 * it the smaller the screen is. The cell is not taken below SMALLEST_CELL, or
 * below what the roomy shares give if that is less, so that a long text on a small
 * screen is as large as it was and not smaller still. Returns the lines (0 if it
 * does not fit), and the width and height of the share that was used. */
static int sign_fit_in(const char *clean, int reference_columns, double room_width,
                       double room_height, double largest, sign_lines_t *lines, double *cell,
                       double *width, double *height) {
    double roomy_width = room_width * SIGN_WIDTH_SHARE,
           roomy_height = room_height * SIGN_HEIGHT_SHARE;
    *width = room_width * sign_share(SIGN_WIDTH_SHARE, SIGN_WIDTH_SHARE_SMALL);
    *height = room_height * sign_share(SIGN_HEIGHT_SHARE, SIGN_HEIGHT_SHARE_SMALL);
    int count = sign_fit(clean, reference_columns, *width, *height, largest, lines, cell);
    if (count > 0 && *cell >= SIGN_SMALLEST_CELL) return count;
    sign_lines_t roomy;
    double roomy_cell;
    int roomy_count =
        sign_fit(clean, reference_columns, roomy_width, roomy_height, largest, &roomy, &roomy_cell);
    if (roomy_count == 0) return count;
    double wanted = roomy_cell < SIGN_SMALLEST_CELL ? roomy_cell : SIGN_SMALLEST_CELL;
    if (count > 0 && *cell >= wanted) return count;
    *lines = roomy;
    *cell = wanted;
    *width = roomy_width;
    *height = roomy_height;
    return roomy_count;
}

/* How far a bird loops, for targets this far apart. */
static double sign_hover_for(double cell) {
    double hover = SIGN_HOVER_SIZE * config.bird_size;
    if (SIGN_HOVER_CELL * cell < hover) hover = SIGN_HOVER_CELL * cell;
    if (hover < SIGN_HOVER_MIN) hover = SIGN_HOVER_MIN;
    if (hover > SIGN_HOVER_MAX) hover = SIGN_HOVER_MAX;
    return hover;
}

/* The room a sign may have: the screen less a pad all round, as the intro has it.
 * With the panel up there are two, the panel's corner being left out of either: the
 * one beside it, which is what the intro takes, and the one under it, which on an
 * eighty column terminal is the wider by far. The sign takes whichever makes its
 * letters larger. */
typedef struct {
    double left, top, right, bottom;
} sign_room_t;

static int sign_rooms(sign_room_t rooms[2]) {
    double pad = config.bird_size * 2.0;
    rooms[0] = (sign_room_t){pad, pad, screen.width - pad, screen.height - pad};
    if (screen.legend_width <= 0) return 1;
    rooms[1] = rooms[0];
    rooms[0].left = screen.legend_width + SIGN_PANEL_GAP + pad;
    rooms[1].top = screen.legend_height + SIGN_PANEL_GAP + pad;
    return 2;
}

/* Lays the cleaned text out as a sign and picks its writers from the birds.
 * Returns whether it fits: the screen may be too small, or the flock too small
 * for the text, and then the flock simply flocks. */
static int sign_place(const char *clean, int reference_columns, int lift_the_colon,
                      const bird_t *birds) {
    sign_room_t rooms[2];
    int room_count = sign_rooms(rooms);
    sign_lines_t lines;
    double cell = 0;
    int line_count = 0;
    double left = 0, top = 0, free_width = 0, free_height = 0, width = 0, height = 0;
    for (int r = 0; r < room_count; r++) {
        sign_lines_t tried;
        double tried_cell, tried_width, tried_height;
        double room_width = rooms[r].right - rooms[r].left;
        double room_height = rooms[r].bottom - rooms[r].top;
        int tried_count = sign_fit_in(clean, reference_columns, room_width, room_height,
                                      SIGN_LARGEST_CELL * config.bird_size, &tried, &tried_cell,
                                      &tried_width, &tried_height);
        if (tried_count == 0 || tried_cell <= cell) continue;
        lines = tried;
        cell = tried_cell;
        line_count = tried_count;
        left = rooms[r].left;
        top = rooms[r].top;
        free_width = room_width;
        free_height = room_height;
        width = tried_width;
        height = tried_height;
    }
    sign_begin_layout();
    the_sign.why = SIGN_NO_ROOM;
    if (line_count == 0) return 0;

    int lit = 0;
    const char *text[SIGN_MAX_LINES];
    for (int l = 0; l < line_count; l++) {
        text[l] = lines.line[l];
        lit += font_text_cells(lines.line[l]);
    }
    int near_birds = 0;
    for (int i = 0; i < config.birds; i++) near_birds += birds[i].layer == 0;
    if (lit > FORMATION_MAX_TARGETS || lit > near_birds) {
        the_sign.why = SIGN_TOO_FEW_BIRDS;
        the_sign.needs = lit;
        the_sign.has = near_birds < FORMATION_MAX_TARGETS ? near_birds : FORMATION_MAX_TARGETS;
        return 0;
    }

    /* The text is centred by where its birds are drawn, which is a bird's top left
     * corner: half a bird off to the right and down of the place it is flown to. */
    double shift = config.bird_size / 2.0;
    left += (free_width - width) / 2 - shift;
    top += (free_height - height) / 2 - shift;
    formation_place(text, line_count, left, top, left + width, top + height, cell, lift_the_colon);
    if (formation.count == 0) return 0; /* All of it in the panel's corner. */

    formation.cell = cell;
    formation.hover = sign_hover_for(cell);

    /* A few birds to a lit cell, as many as the flock can spare, and the same
     * number to every cell for as long as the text is up. A clock is budgeted for
     * the busiest minute of its hour and not for the minute it is writing: it
     * changes a letter at a time, and a cell that had four birds at ten past ten
     * and three at ten to eleven would be a stroke that thickens and thins. */
    int budget = formation.count;
    if (the_sign.kind == SIGN_CLOCK) {
        struct tm when;
        sign_local_now(&when);
        for (int minute = 0; minute < 60; minute++) {
            char time_text[SIGN_TEXT_MAX];
            when.tm_min = minute;
            sign_clock_text(&when, the_sign.twelve_hours, time_text, sizeof(time_text));
            if (font_text_cells(time_text) > budget) budget = font_text_cells(time_text);
        }
    }
    int per_cell =
        (int)(near_birds * sign_share(SIGN_WRITER_SHARE, SIGN_WRITER_SHARE_SMALL)) / budget;
    if (per_cell > SIGN_PER_CELL_MAX) per_cell = SIGN_PER_CELL_MAX;
    if (per_cell < 1) per_cell = 1;
    the_sign.per_cell = per_cell;

    /* The box the rest of the flock is turned away from: the text, and a bird's
     * width round it. */
    double x_min = left + width, x_max = left, y_min = top + height, y_max = top;
    for (int t = 0; t < formation.count; t++) {
        if (formation.x[t] < x_min) x_min = formation.x[t];
        if (formation.x[t] > x_max) x_max = formation.x[t];
        if (formation.y[t] < y_min) y_min = formation.y[t];
        if (formation.y[t] > y_max) y_max = formation.y[t];
    }
    formation.box = (sign_box_t){x_min - config.bird_size, y_min - config.bird_size,
                                 x_max + config.bird_size, y_max + config.bird_size};
    formation.band = SIGN_KEEP_OUT_BAND * config.bird_size;
    formation.keep_out = 1;
    the_sign.why = SIGN_FITS;
    return 1;
}

/* Every lit cell gets its birds, the first of the near flock in order, so a cell
 * is told only by its number. */
static void sign_send_the_writers(const bird_t *birds) {
    int near_birds = 0;
    for (int i = 0; i < config.birds; i++) near_birds += birds[i].layer == 0;
    int writers = the_sign.per_cell * formation.count;
    if (writers > near_birds) writers = near_birds;
    for (int i = 0; i < MAX_BIRDS; i++) formation.slot[i] = -1;
    int next = 0;
    for (int i = 0; i < config.birds && next < writers; i++) {
        if (birds[i].layer > 0) continue; /* The far sky is another sky. */
        formation.slot[i] = next++ % formation.count;
    }
}

/* Which shade of the run's ramp a colour of the picture is: the nearest of the
 * picture's own colours when they are the ramp, and otherwise its light and dark
 * stretched along the ramp somebody chose, light at the light end of it. */
static int picture_shade_of(const uint8_t rgb[3]) {
    int shades = palette_shades();
    if (shades <= 1) return 0;
    if (picture_colours_in_use) {
        int nearest =
            picture_nearest((const uint8_t(*)[3])picture_tints, picture_palette.shades, rgb);
        return nearest < shades ? nearest : shades - 1;
    }
    double span = picture_light - picture_dark;
    double lightness = span > 1e-6 ? (picture_luminance(rgb) - picture_dark) / span : 0.5;
    if (lightness < 0) lightness = 0;
    if (lightness > 1) lightness = 1;
    int shade = (int)((1.0 - lightness) * shades);
    return shade >= shades ? shades - 1 : shade;
}

/* The picture, fitted into the free rectangle as the intro's letters are, and a
 * place in it for every bird there is: they are spread evenly over the ink, and
 * each of them wears the colour of the picture where it goes. */
static int sign_place_picture(void) {
    static picture_point_t points[MAX_BIRDS];
    sign_room_t rooms[2];
    int room_count = sign_rooms(rooms);

    sign_begin_layout();
    the_sign.why = SIGN_NO_ROOM;
    /* Centred by where a bird is drawn, as a sign is: from its top left corner. */
    double shift = config.bird_size / 2.0;
    picture_fit_t fit = {0, 0, 0, 0, 0};
    for (int r = 0; r < room_count; r++) {
        picture_fit_t tried =
            picture_fit(&picture_image, rooms[r].left - shift, rooms[r].top - shift,
                        rooms[r].right - rooms[r].left, rooms[r].bottom - rooms[r].top);
        if (tried.scale > fit.scale) fit = tried;
    }
    if (fit.scale <= 0) return 0;
    int made = picture_sample(&picture_image, &fit, config.birds, the_sign.seed, points);
    if (made == 0) return 0;
    for (int i = 0; i < made; i++) {
        formation.x[i] = points[i].x;
        formation.y[i] = points[i].y;
        formation.lift[i] = 0;
        formation.shift_y[i] = 0;
        formation.slot[i] = i;
        picture_shade[i] = picture_shade_of(points[i].rgb);
    }
    for (int i = made; i < MAX_BIRDS; i++) formation.slot[i] = -1;
    formation.count = made;
    formation.writing = 1;
    /* The distance between two birds that are neighbours in the picture. */
    formation.cell = sqrt(fit.width * fit.height * picture_ink / made);
    formation.hover = sign_hover_for(formation.cell);
    the_sign.why = SIGN_FITS;
    return 1;
}

/* Where every writer is in its loop, and how far the colon has risen, this frame. */
static void sign_move_the_letters(void) {
    for (int i = 0; i < config.birds; i++)
        if (formation.slot[i] >= 0)
            sign_hover((unsigned)i, clock_state.seconds, formation.hover, &formation.hover_x[i],
                       &formation.hover_y[i]);
    double fraction = 0;
    if (the_sign.kind == SIGN_CLOCK) (void)sign_wall_time(&fraction);
    double breath = sign_breath(fraction);
    for (int t = 0; t < formation.count; t++)
        formation.shift_y[t] = -formation.lift[t] * SIGN_BREATH_LIFT * formation.cell * breath;
}

/* The targets of a letter are a run of them, and the first and how many of the
 * run are all there is to know of where a letter is. */
static int sign_find_the_letters(int first[], int cells[], int most) {
    int letters = 0;
    for (int t = 0; t < formation.count; t++) {
        int letter = formation.glyph[t];
        if (letter >= most) break;
        while (letters <= letter) {
            first[letters] = t;
            cells[letters] = 0;
            letters++;
        }
        cells[letter]++;
    }
    return letters;
}

typedef struct {
    double distance;
    int bird;
} sign_candidate_t;

static int sign_nearer(const void *a, const void *b) {
    double d = ((const sign_candidate_t *)a)->distance - ((const sign_candidate_t *)b)->distance;
    return (d > 0) - (d < 0);
}

/* The birds that write a letter that was not there: the ones nearest to it that
 * are flocking. Those that wrote what it replaces have only now been let go, and
 * are used only if there is nobody else, so that a letter is a letter made of new
 * birds and not the same ones turning into another. They are matched to the
 * letter's cells nearest first, each cell taking as many as every other, so that
 * the letter is as thick as the ones that held. */
static void sign_send_the_birds_to_a_letter(const bird_t *birds, const char *let_go, int first,
                                            int cells) {
    static sign_candidate_t candidates[MAX_BIRDS];
    double centre_x = 0, centre_y = 0;
    for (int t = first; t < first + cells; t++) {
        centre_x += formation.x[t] / cells;
        centre_y += formation.y[t] / cells;
    }
    int count = 0;
    for (int i = 0; i < config.birds; i++) {
        if (formation.slot[i] >= 0 || birds[i].layer > 0 || birds[i].scattered > 0) continue;
        double dx = birds[i].x - centre_x, dy = birds[i].y - centre_y;
        /* Behind everybody who was already flying free, however far they are. */
        candidates[count].distance = dx * dx + dy * dy + (let_go[i] ? 1e12 : 0);
        candidates[count].bird = i;
        count++;
    }
    qsort(candidates, (size_t)count, sizeof(*candidates), sign_nearer);

    static int taken[FORMATION_MAX_TARGETS];
    for (int t = first; t < first + cells; t++) taken[t] = 0;
    int wanted = the_sign.per_cell * cells;
    if (wanted > count) wanted = count;
    for (int c = 0; c < wanted; c++) {
        int bird = candidates[c].bird, best = -1;
        double nearest = 0;
        for (int t = first; t < first + cells; t++) {
            if (taken[t] >= the_sign.per_cell) continue;
            double dx = birds[bird].x - formation.x[t], dy = birds[bird].y - formation.y[t];
            double d = dx * dx + dy * dy;
            if (best < 0 || d < nearest) {
                best = t;
                nearest = d;
            }
        }
        taken[best]++;
        formation.slot[bird] = best;
    }
}

/* A new minute on a clock that is not a new hour: the letters that changed are
 * let go of and written by other birds, and the rest are not touched, so the time
 * can be read throughout. Returns 0, with nothing done, when it is the whole sign
 * that has to go: the hour, a text of another length or another hour, or a
 * layout that is no longer for this screen. */
static int sign_change_the_letters(const bird_t *birds, const char *shown) {
    char was[SIGN_TEXT_MAX], now[SIGN_TEXT_MAX];
    sign_clean(the_sign.written, was, sizeof(was));
    int letters = sign_clean(shown, now, sizeof(now));
    size_t length = strlen(now);
    const char *colon = strchr(now, ':');
    if (letters == 0 || strlen(was) != length || colon == NULL || length < 2) return 0;
    if (strncmp(was, now, (size_t)(colon - now) + 1) != 0) return 0; /* The hour changed. */
    if (now[length - 2] == '0' && now[length - 1] == '0') return 0;  /* The hour is here. */
    if (!sign_layout_is_current()) return 0;

    static int was_slot[MAX_BIRDS];
    static char let_go[MAX_BIRDS];
    int was_first[SIGN_TEXT_MAX], was_cells[SIGN_TEXT_MAX];
    int now_first[SIGN_TEXT_MAX], now_cells[SIGN_TEXT_MAX];
    int was_letters = sign_find_the_letters(was_first, was_cells, SIGN_TEXT_MAX);
    memcpy(was_slot, formation.slot, sizeof(*was_slot) * (size_t)config.birds);
    sign_box_t box = formation.box;
    double band = formation.band;

    /* Laid out again for the new text, which puts every letter that did not change
     * exactly where it was: the same cell, the same size. Only the numbers of its
     * targets move, with the letters before it. */
    if (!sign_place(now, sign_columns("00:00"), 1, birds)) return 0;
    formation.box = box;
    formation.band = band;
    int now_letters = sign_find_the_letters(now_first, now_cells, SIGN_TEXT_MAX);
    int changed[SIGN_TEXT_MAX];
    for (int g = 0; g < now_letters; g++)
        changed[g] = g >= was_letters || was[g] != now[g] || was_cells[g] != now_cells[g];

    for (int i = 0; i < config.birds; i++) {
        let_go[i] = 0;
        formation.slot[i] = -1;
        int old = was_slot[i];
        if (old < 0) continue;
        for (int g = 0; g < was_letters; g++) {
            if (old < was_first[g] || old >= was_first[g] + was_cells[g]) continue;
            if (g < now_letters && !changed[g])
                formation.slot[i] = now_first[g] + (old - was_first[g]);
            else
                let_go[i] = 1;
        }
    }
    for (int g = 0; g < now_letters; g++)
        if (changed[g]) sign_send_the_birds_to_a_letter(birds, let_go, now_first[g], now_cells[g]);

    snprintf(the_sign.written, sizeof(the_sign.written), "%s", shown);
    sign_move_the_letters();
    return 1;
}

static int sign_write(const bird_t *birds) {
    char text[SIGN_TEXT_MAX] = "";
    char clean[SIGN_TEXT_MAX];
    int fits;
    if (the_sign.kind == SIGN_PICTURE) {
        fits = sign_place_picture();
    } else {
        sign_text_now(text, sizeof(text));
        sign_clean(text, clean, sizeof(clean));
        fits = sign_place(clean, the_sign.kind == SIGN_CLOCK ? sign_columns("00:00") : 0,
                          the_sign.kind == SIGN_CLOCK, birds);
        if (fits) sign_send_the_writers(birds);
    }
    if (!fits) {
        formation_clear();
        the_sign.up = 0;
        the_sign.failures++;
        the_sign.failed_columns = screen.cols;
        the_sign.failed_rows = screen.rows;
        return 0;
    }
    snprintf(the_sign.written, sizeof(the_sign.written), "%s", text);
    formation.until = -1; /* A sign lets go when it is time, not when the intro is. */
    the_sign.up = 1;
    the_sign.was_up = 1;
    sign_move_the_letters();
    return 1;
}

static void sign_let_go(void) {
    formation_clear();
    the_sign.up = 0;
    the_sign.until =
        clock_state.seconds +
        (the_sign.kind == SIGN_CLOCK ? SIGN_CLOCK_FLIGHT : sign_flight_seconds(the_sign.cycle));
    the_sign.cycle++; /* One held and flown: the next is a little different. */
}

/* Said when the run is over and the terminal is back, and not before: a sign that
 * cannot be laid out once the run has started, because the window is smaller than
 * it was told of or the flock has shrunk, leaves the flock flying as usual, and
 * inside the alternate screen nobody can read why. Once, for the last time it
 * failed. */
static void sign_report_failure(void) {
    if (the_sign.failures == 0) return;
    const char *option = the_sign.kind == SIGN_CLOCK     ? "--clock"
                         : the_sign.kind == SIGN_PICTURE ? "--picture"
                                                         : "--say";
    const char *what = the_sign.kind == SIGN_CLOCK     ? "the time"
                       : the_sign.kind == SIGN_PICTURE ? "the picture"
                                                       : "the text";
    char reason[160];
    if (the_sign.why == SIGN_TOO_FEW_BIRDS)
        snprintf(reason, sizeof(reason),
                 "it takes %d birds to write and the flock has %d to write with", the_sign.needs,
                 the_sign.has);
    else if (the_sign.kind == SIGN_PICTURE)
        snprintf(reason, sizeof(reason), "there is no room for it");
    else
        snprintf(reason, sizeof(reason), "%s is too big for it", what);
    fprintf(stderr, "%s: %s could not be laid out%s on a screen of %d by %d cells: %s%s\n",
            program_name, option, the_sign.was_up ? " for a time" : "", the_sign.failed_columns,
            the_sign.failed_rows, reason, the_sign.was_up ? "" : ", so the flock flew as usual");
    the_sign.failures = 0; /* Said. */
}

static void begin_the_sign(void) {
    the_sign.up = 0;
    the_sign.cycle = 0;
    the_sign.until = clock_state.seconds;
    the_sign.last_tick = clock_state.seconds;
    formation_clear();
    formation.sign = 1;
}

/* Once a frame, before the flock flies: the sign is written, held, let go of
 * and written again. A pause holds it too, so that a sign does not let go the
 * moment a pause is over. */
static void sign_advance(const bird_t *birds) {
    if (the_sign.kind == SIGN_NONE) return;
    double now = clock_state.seconds;
    double waited = now - the_sign.last_tick;
    the_sign.last_tick = now;
    if (paused && !step_once) {
        if (the_sign.until >= 0) the_sign.until += waited;
        return;
    }

    if (!the_sign.up) {
        if (now < the_sign.until) return;
        if (sign_write(birds)) {
            the_sign.until =
                the_sign.kind == SIGN_CLOCK ? -1 : now + sign_hold_seconds(the_sign.cycle);
        } else {
            the_sign.until = now + 1; /* The screen may be bigger by then. */
        }
        return;
    }
    int new_minute = 0;
    if (the_sign.kind == SIGN_CLOCK) {
        char shown[SIGN_TEXT_MAX];
        sign_text_now(shown, sizeof(shown));
        new_minute = strcmp(shown, the_sign.written) != 0;
        /* A minute lets go of the letters that changed and nothing else; the hour
         * lets go of all of them, which is the show. */
        if (new_minute && sign_change_the_letters(birds, shown)) return;
    }
    if (new_minute || (the_sign.until >= 0 && now >= the_sign.until)) {
        sign_let_go();
        return;
    }
    if (!sign_layout_is_current() && !sign_write(birds)) {
        the_sign.until = now + 1;
        return;
    }
    sign_move_the_letters();
}

/* The rest of the flock is turned away from the text while it is up. Zero for a
 * bird of the far sky, and with no sign. */
static vector_t sign_keep_out_vector(const bird_t *bird) {
    vector_t push = {0, 0};
    if (!formation.writing || !formation.keep_out || bird->layer > 0) return push;
    /* Felt some steps of flight before the box, and not at a fixed distance: a
     * band of three birds is crossed in a frame or two at the pace of a 25 frame
     * recording. Measured on 800 birds saying HELLO WORLD for forty seconds, with
     * the band alone, a hundredth of the free flock was inside the text at any
     * moment at 25 frames a second on 96 by 26 cells, and a five hundredth at 60
     * on 200 by 50; with the band at four steps of flight, none and a three
     * thousandth. */
    sign_box_push(&formation.box, sign_band(), bird->x, bird->y, &push.x, &push.y);
    return push;
}

/* A writer is coloured by where its place is along the text, from one end of the
 * ramp to the other, and a wrapped text runs the whole ramp on every line. Tried
 * against a hash of the bird, which speckled every letter across the ramp, and
 * against a colour a line (flat, and sloping down the lines): the gradient reads
 * best, a word coming out as a few letters of one shade each. By line, the last
 * line of ice or ember is the dark end of the ramp from end to end, and is the
 * dimmest line of the sign; with the ramp on every line, every line is as easy to
 * read as the first. The shade is the place's and not the bird's, so a letter that
 * is written again by other birds is the colour it was. */
static int sign_shade_for(int index) {
    int shades = palette_shades();
    int shade = (int)(formation.across[formation.slot[index]] * shades);
    return shade >= shades ? shades - 1 : shade;
}

/* It opens by writing its name, in the middle, for a moment; then the flock
 * takes over from wherever the letters left it, which is the nicest part to
 * watch. A keypress ends it early, and a screen too small for the word simply
 * starts flocking. */
static void begin_the_intro(void) {
    if (fireflies_mode) return; /* A night does not open by writing its name. */
    if (letters_mode) return;   /* The text is the intro. */
    if (sky_mode) return;       /* The letters are laid out on a screen, and this is a space. */
    /* A sign is what the flock writes instead, if one was asked for. */
    if (a_sign_is_asked_for()) {
        begin_the_sign();
        return;
    }
    if (formation_layout("BOIDS")) formation.until = INTRO_SECONDS;
}

static double normalized_angle(double y, double x) {
    double angle = atan2(y, x);
    return angle < 0 ? angle + 2 * M_PI : angle;
}

/*
 * Wind.
 *
 * A slow wandering breeze, which is what keeps a flock off centre and stops a
 * long run looking like it has settled. The direction takes a small random step
 * each frame, so it drifts rather than jumping, and the strength is a notch like
 * everything else.
 */
/* The rain falls, and nothing else has a wind. As a slider it did not do the one
 * thing it was for — it was meant to keep a flock off centre, and at full strength
 * the flock sat 34% off centre against 33% with no wind at all — while tripling
 * the share of birds outside the frame. What is left of it is the downdraught
 * that makes --matrix rain. */
static int the_rain_is_falling;

static vector_t wind_vector(void) {
    vector_t force = {0, 0};
    if (!the_rain_is_falling) return force;
    force.y = 1; /* Straight down, and it stays there. */
    return force;
}

/*
 * Turning inertia.
 *
 * A bird that can turn any amount in one frame moves like a particle: the flock
 * comes out as a blob that changes shape instantly. Cap the turn and it banks
 * instead, which gives the flock curved fronts, a leading edge, and the look of
 * something with mass. This is the single change that makes it read as birds
 * rather than as points, which is why the default is eight of twelve rather than
 * the twelve it used to effectively be.
 *
 * Writing is exempt. It already overrules the flocking rules, and a bird that
 * cannot turn sharply cannot land on a letter: it would circle one instead, and
 * the crispness of the letters is the whole point of them.
 */
static double turn_towards(double from, double to, double most) {
    double delta = atan2(sin(to - from), cos(to - from));
    if (delta > most) delta = most;
    if (delta < -most) delta = -most;
    double turned = from + delta;
    if (turned < 0) turned += 2 * M_PI;
    if (turned >= 2 * M_PI) turned -= 2 * M_PI;
    return turned;
}

/* Twelve is instant, zero is as near a straight line as a bird gets — near, and
 * not exactly, because a bird that cannot turn at all cannot be turned back by
 * the edges either: at a true zero the whole flock flew out of the frame within a
 * second and the screen stayed black until something else was pressed.
 *
 * Scaled by elapsed time, like the speed is, so that both are really per-second
 * quantities and a bird's turning circle is the same number of pixels whatever
 * rate the renderer achieves. Left per frame, an unlocked run would move and turn
 * the flock faster simply because its loop ran more often. */
/* What the notch itself means, before elapsed time has its say: this is what
 * the panel shows, because the panel is about notches. */
static double turning_notch_radians(void) {
    if (config.turning_notch >= LEGEND_BAR_CELLS) return 2 * M_PI;
    /* The whole bar is a setting somebody might want: from a sixth of a turn a
     * frame, which is a long lazy bank that swings wide, up to a quarter.
     * Measured from zero instead, the bottom third of the travel put between a
     * tenth and all of the flock outside the screen — at the very bottom no bird
     * could turn at all, so the edges could not turn them back either and the
     * whole flock left inside a second — and the bar was half poison, with the
     * README recommending a value from the poisoned half. */
    return M_PI / 6 + (M_PI / 2 - M_PI / 6) * config.turning_notch / LEGEND_BAR_CELLS;
}

static double turn_limit(void) {
    if (config.turning_notch >= LEGEND_BAR_CELLS) return 2 * M_PI;
    double scaled = turning_notch_radians() * FRAME_RATE * flight_seconds();
    return scaled > 2 * M_PI ? 2 * M_PI : scaled;
}

/*
 * Hawks.
 *
 * A predator is what gives a clip a story: the flock splits, streams around it
 * and closes again behind, which is the part people loop. A hawk is not a boid.
 * It has no neighbours, obeys none of the three rules, and is kept out of the
 * grid entirely, so the flocking maths is untouched by its existence: it simply
 * chases the nearest bird and every bird flees it.
 */
typedef struct {
    double x, y, direction;
    int frame;
    int prey;          /* Index of the bird it is chasing, negative for none. */
    double commitment; /* Seconds before it may change its mind. */
    double passing;    /* Seconds left of a straight run out of the flock. */
    int wing;          /* A hawk soars, wings out, and beats them only in the dive. */
    double wing_clock;
    int diving; /* In the dive or the pass out of it: what sets a wave off. */
    int layer;  /* In three dimensions, the size it is drawn at, as a bird's is. */
} hawk_t;

static hawk_t hawks[MAX_HAWKS];
/* The hawks' images always go up, so k can summon one at any time. */
static int hawk_sets_built;

/*
 * The catalogue of sprite sets. One set is ROTATION_FRAMES images of one thing:
 * a near bird of one shade at one wing phase; a far bird of one shade, wings out
 * (it is too small to be seen flapping); a hawk at one wing phase; a ghost of a
 * tail at one step of fading. Every renderer indexes the same catalogue, and a
 * Kitty image id is the set's position times ROTATION_FRAMES plus the frame,
 * plus one, because zero is not an id.
 */
enum {
    /* The flat sky's, as it has always been: every shade of the near bird at every
     * wing phase, the far bird, the hawk, the tails, and last the near bird in the
     * light of an escape wave. */
    MAX_SPRITE_SETS_FLAT =
        MAX_PALETTE_SHADES * (WING_PHASES + 1) + WING_PHASES + TRAIL_LENGTH + WING_PHASES,
    /* A space has a hawk for every size as well as a bird, the tails as before (it
     * builds none of them, nor the light of a wave: there are no waves in a space,
     * and a set that is not built is not uploaded or drawn). */
    MAX_SPRITE_SETS_SKY = SKY_BINS * (SKY_SHAPES + WING_PHASES) + TRAIL_LENGTH + WING_PHASES,
    MAX_SPRITE_SETS =
        MAX_SPRITE_SETS_FLAT > MAX_SPRITE_SETS_SKY ? MAX_SPRITE_SETS_FLAT : MAX_SPRITE_SETS_SKY
};

/* In three dimensions there is no far layer and no shade of the bird's own: what
 * a bird looks like is how far off it is, one size and one tint for each of the
 * camera's bins, and the wing beat on top. Every set after the flock's follows it
 * wherever it ends. */
static int flock_set_count(void) {
    if (sky_mode) return SKY_BINS * SKY_SHAPES;
    return palette_shades() * (WING_PHASES + 1);
}

static int flock_set(int shade, int wing, int layer) {
    if (sky_mode) return layer * SKY_SHAPES + wing; /* The shape stands where the beat does. */
    if (layer > 0) return palette_shades() * WING_PHASES + shade;
    return shade * WING_PHASES + wing;
}

/* And a hawk is a set a size and a wing phase, the sizes in order. */
static int hawk_set_count(void) {
    return sky_mode ? SKY_BINS * WING_PHASES : WING_PHASES;
}

static int hawk_set(int wing) {
    return flock_set_count() + wing;
}

static int trail_set(int step) {
    return flock_set_count() + hawk_set_count() + step;
}

/* A bird in an escape wave, at one wing phase: after everything else, so no set
 * that was there before has moved. */
static int alarm_set(int wing) {
    return trail_set(TRAIL_LENGTH) + wing;
}

static int sprite_set_count(void) {
    return alarm_set(WING_PHASES);
}

static uint32_t set_image_id(int set, int frame) {
    return (uint32_t)(set * ROTATION_FRAMES + frame) + 1;
}

/* What picks a bird's picture within its size: where it is in its beat, in the
 * flat flock, and in a space its shape, which has the beat in it. */
static int bird_wing(const bird_t *bird) {
    return sky_mode ? bird->shape : WING_SEQUENCE[bird->wing % WING_CYCLE];
}

static uint32_t sprite_image_id(const bird_t *bird) {
    /* A bird in an escape wave is drawn from its own sets, and there are none in a
     * space, where a layer is a size and zero is the farthest. */
    if (bird->alarmed && !sky_mode && bird->layer == 0)
        return set_image_id(alarm_set(WING_SEQUENCE[bird->wing % WING_CYCLE]), bird->frame);
    return set_image_id(flock_set(bird->shade, bird_wing(bird), bird->layer), bird->frame);
}

/* A tail's ghost, or on a night a firefly's body: smaller than the bird, and drawn
 * from its top left like it, so it is set in by half the difference. */
static int trail_sprite_size(void) {
    int size = (int)(config.bird_size * (fireflies_mode ? FIREFLY_BODY_SIZE : TRAIL_SIZE) + 0.5);
    return size < MIN_BIRD_SIZE ? MIN_BIRD_SIZE : size;
}

static int firefly_body_inset(void) {
    return (config.bird_size - trail_sprite_size()) / 2;
}

/* Which of its sets a hawk is drawn from: the phase of its wings, and in a space
 * the size that goes with how far off it is. */
static int hawk_wing(const hawk_t *hawk) {
    return (sky_mode ? hawk->layer * WING_PHASES : 0) + WING_SEQUENCE[hawk->wing % WING_CYCLE];
}

static int hawk_offset_of(const hawk_t *hawk) {
    return sky_mode ? hawk_size_in_layer(hawk->layer) / 2 : hawk_draw_offset();
}

static uint32_t hawk_image_id(const hawk_t *hawk) {
    return set_image_id(hawk_set(hawk_wing(hawk)), hawk->frame);
}

/* One hawk, so that summoning one with k leaves the hawks already hunting exactly
 * where they are instead of teleporting the lot of them. */
static void place_one_hawk(int i) {
    hawks[i].x = screen.width * (i + 1.0) / (config.hawks + 1.0);
    hawks[i].y = screen.height * (i % 2 ? 0.75 : 0.25);
    hawks[i].direction = 2 * M_PI * random_unit();
    hawks[i].frame = direction_frame(hawks[i].direction);
    hawks[i].prey = -1;
    hawks[i].commitment = 0;
    hawks[i].passing = 0;
    hawks[i].wing = 0;
    hawks[i].wing_clock = 0;
    hawks[i].diving = 0;
    hawks[i].layer = 0;
}

static void place_hawks(void) {
    for (int i = 0; i < config.hawks; i++) place_one_hawk(i);
}

/* The nearest bird, by brute force: eight hawks against four thousand birds is a
 * few tens of thousands of comparisons, well under a percent of a frame, and it
 * needs no structure of its own. A hawk another hawk has already chosen is passed
 * over when `unclaimed` is asked for: two silhouettes converging on one bird read
 * as one hawk with a rendering fault, and they arrive on top of each other. */
static int nearest_bird(const bird_t *birds, double x, double y, int self, int unclaimed,
                        double no_nearer_than) {
    int best = -1;
    double best_distance = 0;
    double floor_squared = no_nearer_than * no_nearer_than;
    for (int b = 0; b < config.birds; b++) {
        if (birds[b].layer > 0 || birds[b].perched) continue; /* Its own sky, and in the air. */
        if (unclaimed) {
            int taken = 0;
            for (int h = 0; h < config.hawks && !taken; h++)
                if (h != self && hawks[h].prey == b) taken = 1;
            if (taken) continue;
        }
        double dx = birds[b].x - x, dy = birds[b].y - y;
        double distance = dx * dx + dy * dy;
        if (distance < floor_squared) continue;
        if (best < 0 || distance < best_distance) {
            best_distance = distance;
            best = b;
        }
    }
    return best;
}

static double distance_to_bird(const bird_t *birds, const hawk_t *hawk, int bird) {
    double dx = birds[bird].x - hawk->x, dy = birds[bird].y - hawk->y;
    return sqrt(dx * dx + dy * dy);
}

/* How near the hawk will pass its bird over the whole of this frame's travel, not
 * merely where it stands now. At forty pixels a frame a hawk can pass clean
 * through a bird between one frame and the next; the nearest point of the segment
 * it is about to fly is what it actually reaches. */
static double reach_along_the_step(const bird_t *birds, const hawk_t *hawk, int bird) {
    double step = config.speed * HAWK_DIVE_SPEED;
    double dx = birds[bird].x - hawk->x, dy = birds[bird].y - hawk->y;
    double along = dx * cos(hawk->direction) + dy * sin(hawk->direction);
    if (along < 0) along = 0;
    if (along > step) along = step;
    double near_x = hawk->x + along * cos(hawk->direction);
    double near_y = hawk->y + along * sin(hawk->direction);
    dx = birds[bird].x - near_x;
    dy = birds[bird].y - near_y;
    return sqrt(dx * dx + dy * dy);
}

/* Pick, or keep. A hawk holds on to its bird for a fixed time whatever
 * else wanders past, and only when that runs out will it trade up, and then only
 * for a bird a clear quarter closer than the one it is on. Without the hold it
 * re-picked the nearest bird every frame, which in a dense flock means a different
 * bird most frames: it turned on the spot and looked broken rather than hungry. */
static void choose_prey(hawk_t *hawk, const bird_t *birds, int self) {
    if (hawk->prey >= config.birds) hawk->prey = -1;
    if (hawk->prey >= 0 && hawk->commitment > 0) return; /* Committed: hold on. */
    /* And with the commitment spent, a chase that has not closed by now is over:
     * the bird outran it, and there are five hundred others. */
    if (hawk->prey >= 0 && distance_to_bird(birds, hawk, hawk->prey) > HAWK_GIVE_UP)
        hawk->prey = -1;

    /* And it picks its bird from outside the flock's own alarm radius, so there is
     * a chase to watch: the nearest bird to a hawk that has just flown through the
     * middle of a flock is one already beside it, and a strike with no approach is
     * over before anybody sees it start. */
    int candidate = nearest_bird(birds, hawk->x, hawk->y, self, 1, HAWK_STALK);
    if (candidate < 0) candidate = nearest_bird(birds, hawk->x, hawk->y, self, 1, 0);
    if (candidate < 0) candidate = nearest_bird(birds, hawk->x, hawk->y, self, 0, 0);
    if (candidate < 0) return;
    if (hawk->prey >= 0 &&
        distance_to_bird(birds, hawk, candidate) > 0.75 * distance_to_bird(birds, hawk, hawk->prey))
        return; /* Not enough of an improvement to be worth changing its mind. */
    hawk->prey = candidate;
    hawk->commitment = (double)HAWK_COMMITMENT_FRAMES / FRAME_RATE;
}

static double hawk_turn_limit(void) {
    double limit = HAWK_TURN_PER_FRAME();
    /* And never so lazy that it cannot turn inside the room it has: on a small
     * terminal a turning circle wider than a third of the screen means the hawk
     * is committed to the glass before it can see it, and it spends the run
     * bouncing from wall to wall. */
    double shorter = screen.width < screen.height ? screen.width : screen.height;
    if (shorter > 0) {
        double needed = config.speed * HAWK_DIVE_SPEED / (shorter / 3.0);
        if (needed > limit) limit = needed;
    }
    return limit > M_PI ? M_PI : limit;
}

/* How tight a circle it can fly. */
static double hawk_turning_radius(void) {
    double limit = hawk_turn_limit();
    return limit > 0 ? config.speed * HAWK_DIVE_SPEED / limit : 0;
}

/* And how far off it has to see a wall: a whole diameter, not a radius. A radius
 * is the bare minimum to turn ninety degrees, and since the push starts at
 * nothing at the band's edge the hawk was always committed before the push was
 * worth anything — it reflected off the glass about once a second. Three gives it
 * room to lean out of the turn instead of hauling on it. */
static double hawk_wall_band(void) {
    return hawk_turning_radius() * 3;
}

/* A wall, seen a turning circle ahead. Without this the hawk flew into the edge
 * and reflected: a hundred and eighty degrees in one frame, three times a second,
 * which is exactly what "buggy" looks like. Now it banks away like the flock does,
 * and the reflection below is a safety net that almost never fires. */
static vector_t hawk_wall_vector(const hawk_t *hawk) {
    vector_t wall = {0, 0};
    double band = hawk_wall_band();
    /* Never more than a third of the room there is, or on a small terminal the two
     * sides of the same axis would overlap and the nearer one would win
     * everywhere: the hawk would be pushed one way from every position on the
     * screen and end up pinned against the far edge. */
    double band_x = band < screen.width / 3.0 ? band : screen.width / 3.0;
    double band_y = band < screen.height / 3.0 ? band : screen.height / 3.0;
    if (band_x >= 1) {
        if (hawk->x < band_x)
            wall.x = (band_x - hawk->x) / band_x;
        else if (hawk->x > screen.width - band_x)
            wall.x = -(hawk->x - (screen.width - band_x)) / band_x;
    }
    if (band_y >= 1) {
        if (hawk->y < band_y)
            wall.y = (band_y - hawk->y) / band_y;
        else if (hawk->y > screen.height - band_y)
            wall.y = -(hawk->y - (screen.height - band_y)) / band_y;
    }
    /* The panel is a wall as well. Every bird is forbidden from it and a hawk was
     * not, so it flew over the sliders in a sixth of all frames: the one thing on
     * the screen that is not sky, with something flying through it. It is steered
     * around rather than forbidden, because a hawk that stopped dead at an edge
     * nobody can see would look stranger than one that crosses it. */
    if (screen.legend_width > 0 && hawk->x < screen.legend_width + band_x &&
        hawk->y < screen.legend_height + band_y) {
        double out_right = screen.legend_width + band_x - hawk->x;
        double out_below = screen.legend_height + band_y - hawk->y;
        if (out_right / band_x < out_below / band_y)
            wall.x += out_right / band_x;
        else
            wall.y += out_below / band_y;
    }
    return wall;
}

/* Room for the other hawks: a nudge away from any hawk closer than HAWK_SPACING,
 * so eight of them share a flock instead of flying as one thick smear. */
static vector_t hawk_spacing(int self) {
    vector_t apart = {0, 0};
    for (int i = 0; i < config.hawks; i++) {
        if (i == self) continue;
        double dx = hawks[self].x - hawks[i].x, dy = hawks[self].y - hawks[i].y;
        double squared = dx * dx + dy * dy;
        if (squared >= (double)HAWK_SPACING * HAWK_SPACING || squared < 1e-9) continue;
        double distance = sqrt(squared);
        /* Unanswerable at contact rather than merely firm: a push that fades to
         * nothing as they touch is a push that lets two hawks overlap, which is
         * the one thing this rule exists to prevent. */
        double strength = HAWK_SPACING / distance - 1;
        apart.x += strength * dx / distance;
        apart.y += strength * dy / distance;
    }
    return apart;
}

/*
 * The chase.
 *
 * A hawk aims where its bird is going rather than where it is, which is what makes
 * the pursuit look intelligent instead of trailing. It banks: a limit on the turn
 * per step, looser than a bird's because a raptor is more agile, but a limit all
 * the same. Inside HAWK_DIVE it stops leading, goes straight at the bird and
 * accelerates, because a fleeing bird is only a tenth slower than a cruising hawk
 * and without that last push the gap never closes and there is nothing to watch.
 *
 * Then it is through them, and flies straight for a short time before turning
 * back: that exit is the half of a stoop that makes the flock close up behind.
 */
static void hunt(const bird_t *birds) {
    if (sky_mode) return; /* The hunt is the space's own, in the flight. */
    for (int i = 0; i < config.hawks; i++) {
        hawk_t *hawk = &hawks[i];
        if (hawk->commitment > 0) {
            hawk->commitment -= flight_seconds();
            if (hawk->commitment < 0) hawk->commitment = 0;
        }

        if (hawk->passing > 0) {
            hawk->passing -= flight_seconds();
            if (hawk->passing < 0) hawk->passing = 0;
            hawk->prey = -1;
        } else {
            /* A pass is being among them, not catching the one it set out after:
             * the bird it chose is fleeing, so what it actually flies through is
             * whichever birds are there when it arrives. */
            /* How near it has to get: two silhouettes, and no wider. Widening it
             * to a frame's travel made the strike distance depend on the frame
             * rate — six times as many strikes at --fps 30 as at 60 — so instead
             * the whole of this frame's travel is tested below, which is the same
             * thing without the arithmetic depending on how fast the clock runs. */
            double arrived = config.bird_size * 2;
            /* Its own bird, and only its own bird. Counting whatever else it
             * happened to pass close to ended nine chases in ten before they were
             * chases: the strike fired the frame the dive began, sixty pixels from
             * the bird it had chosen, and the commitment, the lead and the dive
             * were all dead letters. */
            int struck = hawk->prey >= 0 && hawk->prey < config.birds &&
                         reach_along_the_step(birds, hawk, hawk->prey) < arrived;
            if (struck) {
                hawk->prey = -1;
                hawk->commitment = 0;
                hawk->passing = (double)HAWK_PASS_FRAMES / FRAME_RATE;
            } else {
                choose_prey(hawk, birds, i);
            }
        }

        double pace = HAWK_SPEED;
        hawk->diving = hawk->passing > 0;
        vector_t apart = hawk_spacing(i);
        vector_t wall = hawk_wall_vector(hawk);
        double want_x = apart.x * HAWK_APART + wall.x * HAWK_WALL;
        double want_y = apart.y * HAWK_APART + wall.y * HAWK_WALL;
        if (hawk->prey >= 0) {
            const bird_t *prey = &birds[hawk->prey];
            double gap = distance_to_bird(birds, hawk, hawk->prey);
            if (gap < HAWK_DIVE) {
                pace = HAWK_DIVE_SPEED;
                hawk->diving = 1;
            }
            /* It soars until the dive, and then it beats. */
            if (gap < HAWK_DIVE || hawk->passing > 0) {
                hawk->wing_clock += WING_HZ * WING_CYCLE * frame_seconds;
                while (hawk->wing_clock >= 1.0) {
                    hawk->wing_clock -= 1.0;
                    hawk->wing = (hawk->wing + 1) % WING_CYCLE;
                }
            } else {
                hawk->wing = 0;
            }
            /* Aim where the bird will be when the hawk gets there, not a fixed
             * distance ahead: close in, that is almost no lead at all, and a fixed
             * one had it cutting across in front of the bird and out the far side,
             * round and round. Never further ahead than three baseline steps,
             * because past that the guess is worth less than the chase. */
            double lead = gap / pace;
            if (lead > HAWK_LEAD_DISTANCE) lead = HAWK_LEAD_DISTANCE;
            double to_x = prey->x + cos(prey->direction) * lead - hawk->x;
            double to_y = prey->y + sin(prey->direction) * lead - hawk->y;
            double reach = sqrt(to_x * to_x + to_y * to_y);
            if (reach > 1e-9) {
                want_x += to_x / reach;
                want_y += to_y / reach;
            }
        }
        if (want_x * want_x + want_y * want_y > 1e-9)
            hawk->direction =
                turn_towards(hawk->direction, normalized_angle(want_y, want_x), hawk_turn_limit());

        hawk->x += config.speed * pace * cos(hawk->direction);
        hawk->y += config.speed * pace * sin(hawk->direction);

        /* Turned back at the walls rather than pinned against them: a clamp left
         * it sliding along an edge for a quarter of every run. The margin is half
         * its own silhouette, so it turns while it is still wholly on the screen —
         * clamping to the screen edge instead put the sprite's far half outside it,
         * and a placement that does not fit is a placement the terminal drops, so
         * every wall cost a one frame blink. */
        double margin = hawk_draw_offset();
        double last_x = screen.width - 1 - margin, last_y = screen.height - 1 - margin;
        if (last_x < margin) last_x = margin;
        if (last_y < margin) last_y = margin;
        if (hawk->x < margin || hawk->x > last_x) {
            hawk->x = hawk->x < margin ? margin : last_x;
            hawk->direction = normalized_angle(sin(hawk->direction), -cos(hawk->direction));
            hawk->prey = -1; /* Whatever it was after, it is not that way now. */
            hawk->commitment = 0;
            hawk->passing = 0;
        }
        if (hawk->y < margin || hawk->y > last_y) {
            hawk->y = hawk->y < margin ? margin : last_y;
            hawk->direction = normalized_angle(-sin(hawk->direction), cos(hawk->direction));
            hawk->prey = -1;
            hawk->commitment = 0;
            hawk->passing = 0;
        }
        hawk->frame = direction_frame(hawk->direction);
    }
}

/* How far a hawk's alarm carries. Never more than a third of the shorter side of
 * the screen: eight hawks with a hundred and fifty pixel reach on a terminal three
 * hundred pixels wide leave the flock nowhere at all to be, and it spends its time
 * pressed against the edges. */
static double hawk_reach(void) {
    double shorter = screen.width < screen.height ? screen.width : screen.height;
    double fits = shorter / 3.0;
    return HAWK_REACH < fits ? HAWK_REACH : fits;
}

/* Every bird flees every hawk in reach, hardest when it is closest, and not
 * straight away from it: part of the flee is sideways, around the hawk, on
 * whichever side the bird is already heading. Straight away and the flock bursts
 * open and is simply gone; around, and it opens, streams past and closes behind,
 * which is the shape worth recording. */
static vector_t hawk_vector(const bird_t *bird) {
    vector_t force = {0, 0};
    if (bird->layer > 0) return force; /* Another sky; nothing to fear. */
    for (int i = 0; i < config.hawks; i++) {
        double reach = hawk_reach();
        double dx = bird->x - hawks[i].x, dy = bird->y - hawks[i].y;
        double squared = dx * dx + dy * dy;
        if (squared >= reach * reach || squared < 1e-9) continue;
        double distance = sqrt(squared);
        double strength = (reach - distance) / reach;
        double away_x = dx / distance, away_y = dy / distance;
        /* One of the two ways round; the one the bird is already turning. */
        double side_x = -away_y, side_y = away_x;
        if (cos(bird->direction) * side_x + sin(bird->direction) * side_y < 0) {
            side_x = -side_x;
            side_y = -side_y;
        }
        force.x += strength * (away_x + HAWK_SWIRL * side_x);
        force.y += strength * (away_y + HAWK_SWIRL * side_y);
    }
    return force;
}

/*
 * Escape waves.
 *
 * A hawk coming down on a flock does not frighten the whole of it at once. The
 * birds in its way see it and swerve; the birds beside them see them swerve and
 * swerve too, a moment later and in the same way; and the turn runs through the
 * flock as a band, faster than any one bird flies. That band is the most
 * striking thing a murmuration does, so it is drawn: a bird in the middle of a
 * swerve is lit.
 *
 * The scale is the program's, not a starling's. A bird here covers two thousand
 * four hundred pixels in a second of flight, which is its whole perception
 * radius in fifteen milliseconds, so a reaction time of a tenth of a second is
 * two hundred and forty pixels of flight: a wave that waits that long for every
 * hop of a perception radius moves at 360 pixels a second, a seventh of the
 * birds' own speed, and is left behind by the flock it is meant to cross. The reaction time is
 * therefore not a figure but a consequence: a wave runs WAVE_PACE times as fast as a bird flies,
 * whatever the screen, the pace or the frame rate, and the time a bird takes to follow what it saw
 * is the time that takes over the distance. Measured on a flock of three hundred, settled and then
 * held still with one bird alarmed, the front crosses it at 3.2 to 3.5 times the birds' speed,
 * and 3.1 to 3.3 along a line of birds, where the farthest bird in sight is a little short of the
 * edge of it.
 *
 * It is told as it happens, inside the step, rather than a hop a frame: with the
 * reaction time shorter than a frame a hop a frame would make the wave as fast
 * as the frame rate, and the same flock would be crossed in half the time at
 * thirty frames a second as at sixty. Done in order of time, which is what the
 * heap below is for, it crosses the same pixels a second at any rate: a flock of
 * two hundred, held still, is crossed at the same moment bird for bird at twenty
 * five frames a second, thirty and sixty. Told in the order the birds were found
 * instead, the worst of them began three milliseconds of flight out between
 * thirty and sixty.
 *
 * A bird sees a swerve WAVE_SIGHT times as far as it feels its neighbours.
 * At the perception radius alone a wave stopped at the first gap, and a flock of
 * three hundred in a small terminal is several patches with gaps between them: a
 * wave reached 56% of the flock, on average over thirty seeds, where at one and a
 * half times it reached 78%, at twice 87%, at two and a half 95% and at three no
 * more than that. A swerve is a big movement, and a bird watches the sky more
 * than it watches its neighbours.
 *
 * What it does is turn, through SWERVE_ANGLE and for SWERVE_SECONDS, harder than
 * its banking would let it: three times as hard. The turn is the one the bird it
 * saw made, so a wave carries a direction as well as a bird, and that is what
 * makes it a wave and not a burst: every bird that sees a hawk and runs from it
 * runs its own way, and every bird that sees a bird swerve swerves with it. The
 * birds the hawk alarms itself turn away from its line, to whichever side they
 * are on. The flock is otherwise as it was: at the hero's size, over twenty
 * seeds, 1.05% of the birds were off the screen with hawks and no waves and
 * 1.13% with them, and a hawk struck as often: 6.1 times in eight seconds
 * against 6.4.
 *
 * A bird that has swerved is deaf to the next for WAVE_REFRACTORY, or the wave
 * would come back through the birds it had just crossed. It has to be longer than
 * it looks: a wave does not die at the far side of a flock, it finds a bird whose
 * rest has run out and goes round. At half a second, in the hero's flock, every
 * bird was alarmed five times over and the wave never stopped; at a second one
 * bird in seventeen was alarmed twice; at a second and a half, none. It
 * is three here, on the clock rather than in flight,
 * so that it is the same at any pace and a fast flock does not flash faster.
 * With a hawk in the sky a bird comes out of its rest into a fresh dive almost at
 * once, and at a second and a half the hero's flock was lit four times in eight
 * seconds against twice at three: a light that flickers, where a wave every few
 * strikes is something happening.
 *
 * The letters of the intro are never alarmed: a bird writing has somewhere to be,
 * and the writing is the one thing the hawks cannot spoil. The writers of a sign
 * are the same while they write, and are lit by nothing and told nothing; a hawk
 * or the pointer can scatter the places it is over, and a writer so scattered is a
 * bird of the sky, wave and all, until it has come home. A bird in the far sky
 * is never alarmed, for the same reason it is never hunted, and does not alarm
 * those in the near one. Birds of other flocks are watched as far as they are
 * kin, which is what the avoidance slider says: at the default none, so a wave
 * stays in its own flock and the colours still tell three flocks apart, where
 * letting it cross lit all three at once, and at the bottom, where they are one
 * flock in three colours, all of them.
 */
static const double ALARM_SHARE = 0.6; /* Of hawk_reach(): 90 pixels, the dive's own. */
static const double ALARM_CONE = 0.5;  /* Cosine of the angle ahead of a hawk that is not diving. */
static const double WAVE_PACE = 3.5;
static const double WAVE_SIGHT = 2.5;
static const double SWERVE_SECONDS = 0.03;
static const double SWERVE_ANGLE = 55.0 * M_PI / 180.0;
static const double SWERVE_TURN = 3.0;
static const double WAVE_REFRACTORY = 3.0; /* Seconds on the clock. */
/* Against the sum of the flocking terms, which is a handful at most: enough to
 * settle a bird on the heading it was told to take, as the edges at the very
 * edge of the screen are not enough to take it off. */
static const double SWERVE_WEIGHT = 8.0;

static wave_t waves[MAX_BIRDS];
/* Whether any bird has anything going on, so that a flock that was never
 * alarmed pays for none of it. */
static int waves_in_flight;

/* Who has begun to swerve in this step, and when in it: what a test can look at,
 * and what a recording could be asked. A bird is on the list at most once a step,
 * because it can only begin once. */
typedef struct {
    int bird;
    double at;
} wave_task_t;
static wave_task_t wave_tasks[MAX_BIRDS];
static int wave_task_count;

/*
 * The birds that will begin to swerve before this step is over, soonest first.
 *
 * A bird begins when it is its turn, and what it tells the birds it can see
 * reaches them later than it began, never sooner, so taking them in order of when
 * they begin gives every bird the first thing that could have reached it, whatever
 * the step is: a wave told in the order it was found would be told something
 * different at thirty frames a second from at sixty, because a bird found first
 * is not always a bird that began first. A heap on the wait each bird has, with
 * each bird's place in it, so that a bird told something sooner than it was is
 * moved up and not put in twice.
 */
static int wave_heap[MAX_BIRDS];
static int wave_heap_place[MAX_BIRDS]; /* Where a bird is in it, or minus one. */
static int wave_heap_count;

static void wave_heap_swap(int i, int j) {
    int bird = wave_heap[i];
    wave_heap[i] = wave_heap[j];
    wave_heap[j] = bird;
    wave_heap_place[wave_heap[i]] = i;
    wave_heap_place[wave_heap[j]] = j;
}

static void wave_heap_up(int at) {
    while (at > 0 && waves[wave_heap[at]].wait < waves[wave_heap[(at - 1) / 2]].wait) {
        wave_heap_swap(at, (at - 1) / 2);
        at = (at - 1) / 2;
    }
}

static void wave_heap_down(int at) {
    for (;;) {
        int soonest = at, left = 2 * at + 1, right = left + 1;
        if (left < wave_heap_count && waves[wave_heap[left]].wait < waves[wave_heap[soonest]].wait)
            soonest = left;
        if (right < wave_heap_count &&
            waves[wave_heap[right]].wait < waves[wave_heap[soonest]].wait)
            soonest = right;
        if (soonest == at) return;
        wave_heap_swap(at, soonest);
        at = soonest;
    }
}

/* A bird that is not in it goes in; one that is has been told something sooner. */
static void wave_heap_add(int bird) {
    if (wave_heap_place[bird] < 0) {
        wave_heap_place[bird] = wave_heap_count;
        wave_heap[wave_heap_count++] = bird;
    }
    wave_heap_up(wave_heap_place[bird]);
}

static int wave_heap_take(void) {
    int bird = wave_heap[0];
    wave_heap_swap(0, --wave_heap_count);
    wave_heap_place[bird] = -1;
    wave_heap_down(0);
    return bird;
}

/* A bird is told `swerve` at `at` seconds into the step of `seconds`: it will
 * begin then, if that is inside the step and nothing has told it sooner. */
static void tell_the_bird(int index, double swerve, double at, double seconds) {
    if (wave_catch(&waves[index], swerve, at) && waves[index].wait <= seconds) wave_heap_add(index);
}

static int bird_can_be_alarmed(const bird_t *bird, int index) {
    double unused_x, unused_y;
    if (bird->layer > 0) return 0;
    /* A writer has somewhere to be, in the intro and in a sign alike. A writer of a
     * sign that a hawk or the pointer has scattered is flocking until it is called
     * home, and is as alarmed as any bird in the sky. */
    if (formation_target_for(bird, index, &unused_x, &unused_y)) return 0;
    return wave_catchable(&waves[index]) || wave_waiting(&waves[index]);
}

/* How long a bird takes to follow what it has seen at `distance`, out of the
 * most it can see: half of the reaction time for what is under its wing and the
 * whole of it for what is at the edge of its sight, and that whole is the time a
 * wave takes to cover its sight at WAVE_PACE times the pace a bird flies at.
 * Without the difference every bird a step away follows on the same step, and
 * the front of a wave moves in jerks of that many pixels instead of running. */
static double reaction_time(double distance, double sight) {
    return (sight + distance) / (2.0 * WAVE_PACE * flight_pixels_per_second);
}

/* The turn a bird makes to get out of the way of something coming along
 * `direction`: towards the side of that line it is on. */
static double swerve_away_from(const bird_t *bird, double x, double y, double direction) {
    double side = cos(direction) * (bird->y - y) - sin(direction) * (bird->x - x);
    return side < 0 ? -SWERVE_ANGLE : SWERVE_ANGLE;
}

/* A hawk, or anything else that comes down on the flock: the birds within
 * `radius` of it are told if it is coming at them, and all of them if it is
 * diving. Closer is sooner. */
static void alarm_the_birds_near(const bird_t *birds, double x, double y, double direction,
                                 int diving, double radius) {
    for (int i = 0; i < config.birds; i++) {
        double dx = birds[i].x - x, dy = birds[i].y - y;
        double squared = dx * dx + dy * dy;
        if (squared >= radius * radius || !bird_can_be_alarmed(&birds[i], i)) continue;
        double distance = sqrt(squared);
        if (!diving && dx * cos(direction) + dy * sin(direction) <= ALARM_CONE * distance) continue;
        wave_catch(&waves[i], swerve_away_from(&birds[i], x, y, direction),
                   reaction_time(distance, radius));
    }
}

/* The pointer, whipped through the flock, is a hawk to the birds it is going
 * through, and sets a wave off the same way: a swipe across the screen and not a
 * drift. Fast is a number of cells a second, because the terminal reports cells,
 * and moving is the last report being recent, because it reports nothing when
 * the pointer is still. */
static const double POINTER_STARTLE_CELLS = 80.0;
static const double POINTER_SPAN = 0.03;
static const double POINTER_RECENT = 0.1;

static int pointer_startles(void) {
    if (!mouse.present || clock_state.seconds - mouse.moved_at > POINTER_RECENT) return 0;
    double speed = sqrt(mouse.velocity_x * mouse.velocity_x + mouse.velocity_y * mouse.velocity_y);
    return speed >= POINTER_STARTLE_CELLS * screen.cell_width;
}

/* A bird that has begun to swerve tells everyone it can see. */
static void tell_the_neighbours(const bird_t *birds, const spatial_grid_t *grid, int source,
                                double at, double seconds) {
    const bird_t *from = &birds[source];
    double sight = WAVE_SIGHT * config.vision_radius;
    int cells = (int)ceil(sight / SPATIAL_CELL_SIZE);
    int center_x, center_y;
    spatial_grid_cell_for_position(grid, from->x, from->y, &center_x, &center_y);
    int min_x = center_x - cells, max_x = center_x + cells;
    int min_y = center_y - cells, max_y = center_y + cells;
    if (min_x < 0) min_x = 0;
    if (min_y < 0) min_y = 0;
    if (max_x >= grid->columns) max_x = grid->columns - 1;
    if (max_y >= grid->rows) max_y = grid->rows - 1;
    for (int cell_y = min_y; cell_y <= max_y; cell_y++) {
        for (int cell_x = min_x; cell_x <= max_x; cell_x++) {
            int cell = cell_y * grid->columns + cell_x;
            for (int slot = grid->offsets[cell]; slot < grid->offsets[cell + 1]; slot++) {
                int i = grid->indices[slot];
                if (i == source || birds[i].layer != from->layer) continue;
                if (birds[i].flock != from->flock && config.avoid_kinship <= 0) continue;
                double dx = from->x - birds[i].x, dy = from->y - birds[i].y;
                double squared = dx * dx + dy * dy;
                if (squared >= sight * sight || !bird_can_be_alarmed(&birds[i], i)) continue;
                tell_the_bird(i, waves[source].swerve, at + reaction_time(sqrt(squared), sight),
                              seconds);
            }
        }
    }
}

/* One step of alarm: the clocks, the hawks and the pointer that set a wave off,
 * and the wave itself as far as it gets in the time the step covers. Every wait
 * is counted from the start of the step while it is run, and what has not run
 * out by the end is carried into the next. */
static void spread_the_alarm(const bird_t *birds, const spatial_grid_t *grid) {
    double seconds = flight_seconds();
    int startled = pointer_startles();
    if (config.hawks == 0 && !waves_in_flight && !startled) return;

    wave_task_count = 0;
    for (int i = 0; i < config.birds; i++) wave_advance(&waves[i], seconds);
    for (int h = 0; h < config.hawks; h++)
        alarm_the_birds_near(birds, hawks[h].x, hawks[h].y, hawks[h].direction, hawks[h].diving,
                             ALARM_SHARE * hawk_reach());
    if (startled)
        alarm_the_birds_near(birds, mouse.x, mouse.y, atan2(mouse.velocity_y, mouse.velocity_x), 1,
                             ALARM_SHARE * MOUSE_REACH);
    wave_heap_count = 0;
    for (int i = 0; i < config.birds; i++) {
        wave_heap_place[i] = -1;
        /* Told, and then sent to write before it turned, as a sign does with the
         * birds it picks: it forgets what it was told. */
        double unused_x, unused_y;
        if (waves[i].wait > 0 && formation_target_for(&birds[i], i, &unused_x, &unused_y))
            waves[i].wait = 0;
        if (waves[i].wait > 0 && waves[i].wait <= seconds) wave_heap_add(i);
    }
    while (wave_heap_count > 0) {
        int bird = wave_heap_take();
        double at = waves[bird].wait;
        wave_begin(&waves[bird], birds[bird].direction, seconds - at, SWERVE_SECONDS,
                   WAVE_REFRACTORY * config.pace);
        wave_tasks[wave_task_count++] = (wave_task_t){bird, at};
        tell_the_neighbours(birds, grid, bird, at, seconds);
    }

    waves_in_flight = 0;
    for (int i = 0; i < config.birds; i++) {
        wave_carry(&waves[i], seconds);
        if (wave_busy(&waves[i])) waves_in_flight = 1;
    }
}

/* Whether a bird is in the middle of a swerve. A letter never is: a text has its own
 * take-off wave (see letters.c) and a hawk or a pointer scatters its letters instead,
 * and the wave state is the size of a flock, not of a text. */
static int is_swerving(int index) {
    return !letters_mode && waves[index].left > 0;
}

/* The swerve as a pull, towards the heading the bird was told to take. */
static vector_t swerve_vector(int index) {
    vector_t force = {0, 0};
    if (!is_swerving(index)) return force;
    force.x = cos(waves[index].heading);
    force.y = sin(waves[index].heading);
    return force;
}

/* Each flock flies at its own pace, the first at full speed and the last at
 * seven eighths of it. Two flocks at identical speeds pass through each other and
 * come out symmetrical; a little apart and they shear, overtake and tangle, which
 * is the part that looks alive. Small enough that nobody reads it as a bug. */
static double flock_pace(int flock) {
    if (config.flocks <= 1) return 1.0;
    return 1.0 - (1 - config.avoid_kinship) * 0.125 * flock / (config.flocks - 1);
}

/* Where each flock is, measured once a frame off the same snapshot every bird
 * reads, so every bird in a flock agrees about where its flock is. */
static double flock_center_x[MAX_FLOCKS], flock_center_y[MAX_FLOCKS];
/* Where each flock is told to be: its own centre, shoved clear of the others
 * when the avoidance is above the default. */
static double flock_home_x[MAX_FLOCKS], flock_home_y[MAX_FLOCKS];
/* How far a bird may stray from home before the leash pulls: see FLOCK_LEASH. */
static double flock_leash[MAX_FLOCKS];

/* A flock's shade: the ramp's ends first, then the space between them, so two
 * flocks come out as far apart as the palette allows instead of as shade zero and
 * shade one, which on any five step ramp are nearly the same colour. */
static int shade_for_flock(int flock) {
    int shades = palette_shades();
    if (shades <= 1 || config.flocks <= 1) return 0;
    int spread = flock * (shades - 1) / (config.flocks - 1);
    return spread >= shades ? shades - 1 : spread;
}

/* The distance between neighbours if the swarm were laid out evenly: the unit
 * every distance of the fireflies is counted in. */
static double firefly_spacing(void) {
    double area = (double)screen.width * (double)screen.height;
    return config.birds > 0 && area > 0 ? sqrt(area / config.birds) : 1.0;
}

/* One firefly, anywhere in the meadow. They drift slowly, so unlike a flock they
 * cannot be left to find their own way out of the middle of the screen: they
 * start spread over all of it, a margin in from the edges. Dark until their
 * clocks say otherwise. */
static void place_one_firefly(bird_t *bird) {
    double pad = firefly_spacing() / 2;
    double min_x = pad, max_x = screen.width - pad;
    double min_y = pad, max_y = screen.height - pad;
    if (max_x <= min_x) min_x = max_x = screen.width / 2.0;
    if (max_y <= min_y) min_y = max_y = screen.height / 2.0;
    for (int attempt = 0;; attempt++) {
        bird->x = min_x + (max_x - min_x) * random_unit();
        bird->y = min_y + (max_y - min_y) * random_unit();
        if (!legend_turn_zone(bird->x, bird->y)) break;
        if (attempt + 1 >= SPAWN_ATTEMPTS) {
            bird->y = screen.legend_height + config.speed + 1;
            if (bird->y > max_y) bird->x = screen.legend_width + config.speed + 1;
            break;
        }
    }
    bird->direction = 2 * M_PI * random_unit();
    bird->frame = direction_frame(bird->direction);
    bird->layer = deep_look && random_unit() < FAR_SHARE ? 1 : 0;
    bird->shade = -1;
}

/* One bird, so that growing the flock at runtime places only the new ones. */
static void place_one_bird(bird_t *bird, int index) {
    /* A bird starts from nothing: the memory a + grows the flock into is
     * whatever the allocator left there, and the tail index is read as an array
     * index the moment trails are on. */
    *bird = (bird_t){0};
    if (fireflies_mode) {
        place_one_firefly(bird);
        return;
    }
    double min_x = screen.turn_x, max_x = screen.width - screen.turn_x;
    double min_y = screen.turn_y, max_y = screen.height - screen.turn_bottom;
    if (max_x <= min_x) min_x = max_x = screen.width / 2.0;
    if (max_y <= min_y) min_y = max_y = screen.height / 2.0;

    /* Flocks are handed out round robin so they come out even. Each one starts in
     * its own column of the screen, because scattered evenly they begin as one
     * speckled cloud and take half a minute to sort themselves out; started apart
     * they read as separate flocks from the first frame and then go and meet. */
    bird->flock = index % config.flocks;
    if (config.flocks > 1) {
        double band = (max_x - min_x) / config.flocks;
        min_x += band * bird->flock;
        max_x = min_x + band;
        /* Unless that flock is already flying somewhere, in which case a bird
         * added with + joins it there rather than appearing in the column it
         * started in a minute ago and being dragged across the screen. */
        if (flock_home_x[bird->flock] != 0 || flock_home_y[bird->flock] != 0) {
            min_x = flock_home_x[bird->flock] - FLOCK_LEASH / 2.0;
            max_x = min_x + FLOCK_LEASH;
            min_y = flock_home_y[bird->flock] - FLOCK_LEASH / 2.0;
            max_y = min_y + FLOCK_LEASH;
        }
    }

    /* Rejected rather than clamped, so the whole free region stays in play
     * instead of the flock piling up along one edge of the panel. Bounded, with a
     * deterministic fallback below the panel, because on a small viewport the
     * free region can be almost entirely covered. */
    for (int attempt = 0;; attempt++) {
        bird->x = min_x + (max_x - min_x) * random_unit();
        bird->y = min_y + (max_y - min_y) * random_unit();
        if (!legend_turn_zone(bird->x, bird->y)) break;
        if (attempt + 1 >= SPAWN_ATTEMPTS) {
            bird->y = screen.legend_height + config.speed + 1;
            if (bird->y > max_y) bird->x = screen.legend_width + config.speed + 1;
            break;
        }
    }
    bird->direction = 2 * M_PI * random_unit();
    bird->frame = direction_frame(bird->direction);
    /* A plane of its own, and a place in the beat of its own, or every wing in
     * the sky would go up and down together. */
    bird->layer = deep_look && random_unit() < FAR_SHARE ? 1 : 0;
    bird->wing = (int)(random_unit() * WING_CYCLE) % WING_CYCLE;
    bird->wing_clock = random_unit();
    bird->gliding = 0;
    /* When there is more than one flock the palette follows them: a colour per
     * flock is what makes two of them legible as two. */
    bird->shade = config.flocks > 1 ? shade_for_flock(bird->flock)
                                    : (int)(random_unit() * palette_shades()) % palette_shades();
}

static void place_the_letters(bird_t *birds);
static void begin_the_sky(bird_t *birds);
static int grow_the_sky(int from, int to);

static void initialize_birds(bird_t *birds) {
    if (letters_mode) {
        place_the_letters(birds);
        return;
    }
    if (sky_mode) {
        begin_the_sky(birds);
        return;
    }
    for (int i = 0; i < config.birds; i++) place_one_bird(&birds[i], i);
}

/* Grown or shrunk by a keypress: the birds already flying carry on and only the
 * new ones are placed. Both arrays change or neither does. A realloc each could
 * leave one resized and the other not, and one count cannot describe two
 * lengths, so both are built new and swapped in only once both exist. */
static int resize_the_flock(bird_t **birds, bird_t **snapshot, int from, int to) {
    bird_t *grown = malloc(sizeof(**birds) * (size_t)to);
    bird_t *grown_snapshot = malloc(sizeof(**snapshot) * (size_t)to);
    if (grown == NULL || grown_snapshot == NULL) {
        free(grown);
        free(grown_snapshot);
        return 0;
    }
    if (sky_mode && !grow_the_sky(from, to)) {
        free(grown);
        free(grown_snapshot);
        return 0;
    }
    memcpy(grown, *birds, sizeof(**birds) * (size_t)(from < to ? from : to));
    for (int i = from; i < to; i++) {
        place_one_bird(&grown[i], i);
        waves[i] = (wave_t){0};
    }
    free(*birds);
    free(*snapshot);
    *birds = grown;
    *snapshot = grown_snapshot;
    return 1;
}

/*
 * The pointer, as a thing in the world.
 *
 * flee is the default because it is the one that explains itself: the flock
 * parts around the cursor within a second of the viewer moving it, and nobody
 * has to be told what happened. follow is the opposite and makes the pointer a
 * feeder.
 *
 * The force falls off with distance rather than being a hard wall like the
 * panel's, because the flock has to bend around the pointer and close again
 * behind it, not bounce off it.
 */
static vector_t pointer_push(const bird_t *bird, double reach) {
    vector_t force = {0, 0};
    if (!mouse.present) return force;

    double dx = bird->x - mouse.x, dy = bird->y - mouse.y;
    double squared = dx * dx + dy * dy;
    if (squared >= reach * reach || squared < 1e-9) return force;

    double distance = sqrt(squared);
    /* One at the pointer, nothing at the edge of its reach. */
    double strength = (reach - distance) / reach;

    force.x = strength * dx / distance;
    force.y = strength * dy / distance;
    return force;
}

static vector_t pointer_vector(const bird_t *bird) {
    return pointer_push(bird, MOUSE_REACH);
}

/*
 * How hard an edge pushes, given how far into its band a bird has gone.
 *
 * This used to be a flat 1 anywhere in the band, which is why most of the flock
 * ended up off screen: a constant push loses to a strong enough alignment, and
 * once it has lost there is nothing stronger further out to win it back. It now
 * grows, gently through the band and then steeply past the edge of the screen,
 * so leaving is always possible and staying away never is.
 */
static double edge_push(double past, double band) {
    if (past <= 0) return 0;
    double depth = past / band; /* Zero at the band's inner edge, one at the screen's. */
    if (depth <= 1) return EDGE_FIRM * depth * depth;
    /* Past the screen itself the weight stops having a vote: the softest boundary
     * is still a boundary. Dividing it out here means the push a bird feels once
     * it is out of sight is the same at notch 0 as at notch 12, so a low boundary
     * makes them turn late rather than leave. Carrying the weight in the first
     * term keeps the whole thing continuous at the edge: without it the push
     * jumped by a factor of a hundred at notch 0 and a bird was slapped rather
     * than turned. */
    return EDGE_FIRM * (config.boundary + (depth - 1) * ESCAPE_PENALTY) / config.boundary;
}

static vector_t boundary_vector(const bird_t *bird) {
    vector_t boundary = {0, 0};
    if (legend_repels(bird, &boundary)) return boundary;
    if (the_rain_is_falling) return boundary; /* A door, so no wall. */
    if (bird->x < screen.turn_x)
        boundary.x = edge_push(screen.turn_x - bird->x, screen.turn_x);
    else if (bird->x > screen.width - screen.turn_x)
        boundary.x = -edge_push(bird->x - (screen.width - screen.turn_x), screen.turn_x);
    if (bird->y < screen.turn_y)
        boundary.y = edge_push(screen.turn_y - bird->y, screen.turn_y);
    else if (bird->y > screen.height - screen.turn_bottom)
        boundary.y = -edge_push(bird->y - (screen.height - screen.turn_bottom), screen.turn_bottom);
    return boundary;
}

/* How much room one flock asks of another: two leashes, or nearly the whole of
 * the shorter side of the screen when that is less, so that on a small viewport
 * the flocks are not all shoved into the corners. It is an ask, not a promise:
 * five flocks cannot all be three hundred pixels apart on a screen four hundred
 * pixels tall, and what they settle at is as far apart as there is room for. */
static double flock_room(void) {
    double room = 2.0 * FLOCK_LEASH * config.avoid_room;
    double shorter = screen.width < screen.height ? screen.width : screen.height;
    double fits = 0.9 * shorter;
    return room < fits ? room : fits;
}

static void measure_flocks(const bird_t *birds) {
    int counted[MAX_FLOCKS] = {0};
    for (int f = 0; f < MAX_FLOCKS; f++) {
        flock_center_x[f] = flock_center_y[f] = 0;
        flock_home_x[f] = flock_home_y[f] = 0;
    }
    if (config.flocks <= 1) return;
    for (int i = 0; i < config.birds; i++) {
        int flock = birds[i].flock;
        if (flock < 0 || flock >= MAX_FLOCKS) continue;
        flock_center_x[flock] += birds[i].x;
        flock_center_y[flock] += birds[i].y;
        counted[flock]++;
    }
    for (int f = 0; f < MAX_FLOCKS; f++) {
        if (counted[f] > 0) {
            flock_center_x[f] /= counted[f];
            flock_center_y[f] /= counted[f];
        }
        double width = LEASH_PER_ROOT_BIRD * sqrt((double)counted[f]);
        flock_leash[f] = width > FLOCK_LEASH ? width : FLOCK_LEASH;
    }

    /* Left to themselves the three centres of gravity drift to the middle of the
     * screen and sit on top of each other, and three flocks become one cloud in
     * three colours. So each flock is sent home to its own centre moved clear of
     * every other flock's: they shoulder each other apart, find their own corners
     * of the sky and keep them, without any of it being on rails. */
    double room = flock_room();
    for (int f = 0; f < config.flocks && f < MAX_FLOCKS; f++) {
        flock_home_x[f] = flock_center_x[f];
        flock_home_y[f] = flock_center_y[f];
        if (counted[f] == 0) continue;
        double shove_x = 0, shove_y = 0;
        int crowding = 0;
        for (int g = 0; g < config.flocks && g < MAX_FLOCKS; g++) {
            if (g == f || counted[g] == 0) continue;
            double dx = flock_center_x[f] - flock_center_x[g];
            double dy = flock_center_y[f] - flock_center_y[g];
            double distance = sqrt(dx * dx + dy * dy);
            if (distance >= room) continue;
            if (distance < 1e-9) {
                /* Exactly on top of one another: pick a direction rather than
                 * dividing by nothing, one for each flock, and let them unfold. */
                double angle = 2 * M_PI * f / config.flocks;
                dx = cos(angle);
                dy = sin(angle);
                distance = 1;
            }
            shove_x += (room - distance) * dx / distance;
            shove_y += (room - distance) * dy / distance;
            crowding++;
        }
        /* The sum of the shoves, but never further than one room's worth of it:
         * four flocks all pushing the middle one used to throw its home two
         * hundred pixels clear off the screen, where the leash drags the flock
         * into an edge that is pushing back just as hard. Averaging instead was
         * worse — in a row of three the two outer shoves on the middle flock
         * oppose each other, and an average of opposites is nothing at all. */
        if (crowding > 0) {
            double shove = sqrt(shove_x * shove_x + shove_y * shove_y);
            if (shove > room) {
                shove_x = shove_x * room / shove;
                shove_y = shove_y * room / shove;
            }
            flock_home_x[f] += shove_x;
            flock_home_y[f] += shove_y;
        }
        /* And home is somewhere a flock can actually be. */
        if (flock_home_x[f] < 0) flock_home_x[f] = 0;
        if (flock_home_y[f] < 0) flock_home_y[f] = 0;
        if (flock_home_x[f] > screen.width) flock_home_x[f] = screen.width;
        if (flock_home_y[f] > screen.height) flock_home_y[f] = screen.height;
    }
}

/* The leash: nothing within the flock's own width of home, and outside that a
 * pull that grows with the distance. It lets go as strangers become kin, cubed so
 * that it is all but gone by the time they are half kin: a flock tied to its own
 * centre while it mixes with another is two knots in one cloud. */
static vector_t leash_vector(const bird_t *bird) {
    vector_t pull = {0, 0};
    if (config.flocks <= 1 || bird->flock < 0 || bird->flock >= MAX_FLOCKS) return pull;
    double dx = flock_home_x[bird->flock] - bird->x;
    double dy = flock_home_y[bird->flock] - bird->y;
    double distance = sqrt(dx * dx + dy * dy);
    double width = flock_leash[bird->flock] > 0 ? flock_leash[bird->flock] : FLOCK_LEASH;
    if (distance <= width || distance < 1e-9) return pull;
    double strength = (distance - width) / width;
    if (strength > 1) strength = 1;
    double apart = 1 - config.avoid_kinship;
    strength *= apart * apart * apart;
    pull.x = strength * dx / distance;
    pull.y = strength * dy / distance;
    return pull;
}

static void read_bird_position(const void *context, int index, double *x, double *y) {
    const bird_t *birds = context;
    *x = birds[index].x;
    *y = birds[index].y;
}

static double flock_direction(const bird_t *birds, const spatial_grid_t *grid, int target_index) {
    const bird_t *target = &birds[target_index];
    double want_x, want_y;
    /* Writing overrules flocking while it lasts: a bird with a target steers at
     * it and nothing else, which is what makes a letter a letter. */
    if (formation_target_for(target, target_index, &want_x, &want_y)) {
        double to_x = want_x - target->x, to_y = want_y - target->y;
        if (to_x * to_x + to_y * to_y > 1e-9) return normalized_angle(to_y, to_x);
        return target->direction;
    }
    vector_t separation = {0, 0}, alignment = {0, 0}, cohesion = {0, 0}, wary = {0, 0};
    vector_t boundary = boundary_vector(target);
    vector_t leash = leash_vector(target);
    vector_t pointer = pointer_vector(target);
    vector_t hawk = hawk_vector(target);
    vector_t swerve = swerve_vector(target_index);
    vector_t wind = wind_vector();
    vector_t keep_out = sign_keep_out_vector(target);
    int neighbors = 0, strangers = 0;
    double kin = 0; /* Counted in kinship: a whole bird for its own flock. */
    int center_x, center_y;
    spatial_grid_cell_for_position(grid, target->x, target->y, &center_x, &center_y);
    int min_x = center_x - config.vision_cells;
    int max_x = center_x + config.vision_cells;
    int min_y = center_y - config.vision_cells;
    int max_y = center_y + config.vision_cells;
    if (min_x < 0) min_x = 0;
    if (min_y < 0) min_y = 0;
    if (max_x >= grid->columns) max_x = grid->columns - 1;
    if (max_y >= grid->rows) max_y = grid->rows - 1;

    for (int cell_y = min_y; cell_y <= max_y; cell_y++) {
        for (int cell_x = min_x; cell_x <= max_x; cell_x++) {
            int cell = cell_y * grid->columns + cell_x;
            for (int slot = grid->offsets[cell]; slot < grid->offsets[cell + 1]; slot++) {
                int i = grid->indices[slot];
                if (i == target_index) continue;
                const bird_t *other = &birds[i];
                double dx = target->x - other->x, dy = target->y - other->y;
                if (dx * dx + dy * dy >= config.vision_radius_squared) continue;
                /* The far layer is another sky: a bird sees nothing in the other. A
                 * letter at home is not in the air at all. */
                if (other->layer != target->layer || other->perched) continue;
                /* Separation is physical and applies to every bird in reach.
                 * Alignment and cohesion are social, and a bird only reads its
                 * own flock: two flocks pass through each other, swirl, and
                 * refuse to merge. With one flock every neighbour is kin and the
                 * arithmetic is exactly what it was. */
                separation.x += dx;
                separation.y += dy;
                neighbors++;
                if (other->flock != target->flock) {
                    /* Kin in part, below the default: its heading and place count
                     * for that share of a bird of the flock's own. */
                    if (config.avoid_kinship > 0) {
                        trig_entry_t heading = trig_lookup(other->direction);
                        alignment.x += config.avoid_kinship * heading.cosine;
                        alignment.y += config.avoid_kinship * heading.sine;
                        cohesion.x += config.avoid_kinship * other->x;
                        cohesion.y += config.avoid_kinship * other->y;
                        kin += config.avoid_kinship;
                    }
                    /* Away from a stranger, hardest when it is nearest, as the
                     * pointer is felt; averaged below, so a front of them weighs
                     * what one does and the weight alone says how much. */
                    if (config.avoid_weight > 0) {
                        double distance = sqrt(dx * dx + dy * dy);
                        if (distance > 1e-9) {
                            double strength = 1 - distance / config.vision_radius;
                            wary.x += strength * dx / distance;
                            wary.y += strength * dy / distance;
                            strangers++;
                        }
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
        }
    }
    if (neighbors) {
        if (kin != 0) {
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
                   pointer.x * MOUSE_WEIGHT + hawk.x * HAWK_WEIGHT + wind.x * WIND_WEIGHT +
                   wary.x * config.avoid_weight + swerve.x * SWERVE_WEIGHT +
                   keep_out.x * SIGN_KEEP_OUT_WEIGHT;
        double y = separation.y * config.separation + alignment.y * config.alignment +
                   cohesion.y * COHESION_W + boundary.y * config.boundary + leash.y * LEASH_WEIGHT +
                   pointer.y * MOUSE_WEIGHT + hawk.y * HAWK_WEIGHT + wind.y * WIND_WEIGHT +
                   wary.y * config.avoid_weight + swerve.y * SWERVE_WEIGHT +
                   keep_out.y * SIGN_KEEP_OUT_WEIGHT;
        return x == 0 && y == 0 ? target->direction : normalized_angle(y, x);
    }
    boundary.x = boundary.x * config.boundary + leash.x * LEASH_WEIGHT + pointer.x * MOUSE_WEIGHT +
                 hawk.x * HAWK_WEIGHT + wind.x * WIND_WEIGHT + swerve.x * SWERVE_WEIGHT +
                 keep_out.x * SIGN_KEEP_OUT_WEIGHT;
    boundary.y = boundary.y * config.boundary + leash.y * LEASH_WEIGHT + pointer.y * MOUSE_WEIGHT +
                 hawk.y * HAWK_WEIGHT + wind.y * WIND_WEIGHT + swerve.y * SWERVE_WEIGHT +
                 keep_out.y * SIGN_KEEP_OUT_WEIGHT;
    if (boundary.x != 0 || boundary.y != 0) {
        double x = cos(target->direction) + boundary.x;
        double y = sin(target->direction) + boundary.y;
        if (x != 0 || y != 0) return normalized_angle(y, x);
    }
    return target->direction;
}

/* The edges, for something that drifts. A bird is turned from a band a third of
 * the screen wide and has the speed to use the rest of it; a firefly at a
 * fiftieth of the speed does not, and with the bands a bird has the whole swarm
 * ended up in a box in the middle of the screen, a third as wide as the screen
 * and twice as dense as it should have been, where it synchronised in eight
 * seconds. Here the bands are a tenth, and the meadow is a lean instead of a
 * wall: the top third pushes down, softly, and that is the whole of the
 * preference for the lower two thirds of the sky. */
static vector_t firefly_edge_vector(const bird_t *bird) {
    vector_t push = {0, 0};
    if (legend_repels(bird, &push)) return push;
    double band_x = screen.width * FIREFLY_EDGE, band_y = screen.height * FIREFLY_EDGE;
    if (band_x < 1) band_x = 1;
    if (band_y < 1) band_y = 1;
    if (bird->x < band_x)
        push.x = edge_push(band_x - bird->x, band_x);
    else if (bird->x > screen.width - band_x)
        push.x = -edge_push(bird->x - (screen.width - band_x), band_x);
    if (bird->y < band_y)
        push.y = edge_push(band_y - bird->y, band_y);
    else if (bird->y > screen.height - band_y)
        push.y = -edge_push(bird->y - (screen.height - band_y), band_y);
    return push;
}

/*
 * A firefly's way: it drifts.
 *
 * No alignment, no leash, no hawks. It keeps a little room round itself, leans a
 * little towards the ones near it, is turned from the edges and the lantern as
 * any bird is, and between those it wanders: the heading takes a small random
 * turn every step, sized so that the path is the same whatever the frame rate
 * (a random walk grows with the square root of the time it has had, so the turn
 * is scaled the same way) and the same shape however fast the speed slider flies
 * it. The turning slider sets how sharp the wander is as well as how sharp the
 * sharpest turn can be, because for a drifter those are the one thing.
 */
static double firefly_direction(const bird_t *birds, const spatial_grid_t *grid, int index) {
    const bird_t *target = &birds[index];
    double room = FIREFLY_ROOM * firefly_spacing();
    int reach = (int)ceil(room / SPATIAL_CELL_SIZE);
    vector_t apart = {0, 0}, together = {0, 0};
    int neighbours = 0;
    int center_x, center_y;
    spatial_grid_cell_for_position(grid, target->x, target->y, &center_x, &center_y);
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
                int i = grid->indices[slot];
                const bird_t *other = &birds[i];
                if (i == index || other->layer != target->layer) continue;
                double dx = target->x - other->x, dy = target->y - other->y;
                double squared = dx * dx + dy * dy;
                if (squared >= room * room || squared < 1e-9) continue;
                double distance = sqrt(squared);
                double strength = 1 - distance / room;
                apart.x += strength * dx / distance;
                apart.y += strength * dy / distance;
                together.x -= dx;
                together.y -= dy;
                neighbours++;
            }
        }
    }
    if (neighbours) {
        together.x /= neighbours * room;
        together.y /= neighbours * room;
    }

    double sharpness = turning_notch_radians() / (M_PI * 70 / 180);
    if (sharpness > 2.5) sharpness = 2.5;
    double wander = FIREFLY_WANDER * sharpness * sqrt(flight_seconds());
    double heading = target->direction + wander * sqrt(3.0) * (2 * random_unit() - 1);

    vector_t boundary = firefly_edge_vector(target);
    vector_t pointer = pointer_push(target, FIREFLY_LANTERN * firefly_spacing());
    double keep_apart = FIREFLY_PUSH_APART * config.separation / DEFAULT_SEPARATION_W;
    double lean = target->y < screen.height / 3.0 ? 1 - target->y / (screen.height / 3.0) : 0;
    double x = cos(heading) + apart.x * keep_apart + together.x * FIREFLY_PULL_TOGETHER +
               boundary.x * config.boundary + pointer.x * MOUSE_WEIGHT;
    double y = sin(heading) + apart.y * keep_apart + together.y * FIREFLY_PULL_TOGETHER +
               boundary.y * config.boundary + pointer.y * MOUSE_WEIGHT +
               lean * lean * FIREFLY_MEADOW;
    return x == 0 && y == 0 ? target->direction : normalized_angle(y, x);
}

/*
 * What decides a bird's shade within the ramp, and it is not a setting: one flock
 * is coloured by heading, and more than one by flock, because those are the two
 * answers anybody wanted.
 *
 * There used to be a --color-by with four modes and a key to cycle them. Two of
 * them were the same thing — colouring by flock is what a flock is given at birth
 * anyway — one painted two thirds of a default flock in the single darkest shade
 * of the ramp and got worse the more birds there were, and the fourth is this.
 *
 * Heading is the striking one: a turn runs a ripple of colour through the whole
 * flock, because neighbours that agree on a heading agree on a colour.
 */
static int shade_for(const bird_t *bird) {
    int shades = palette_shades();
    if (shades <= 1) return 0;
    if (config.flocks > 1) return shade_for_flock(bird->flock);

    double turns = normalized_angle(sin(bird->direction), cos(bird->direction)) / (2 * M_PI);
    /* Folded at the half turn, because a heading is a circle and a ramp is a line:
     * laid straight on to it, two birds a degree apart either side of due east got
     * the two ends of the palette, and the flock came out salted with dark speckle
     * that no turn of it explained. Folded, the two ways round meet in the middle
     * and the colour runs smoothly with the heading; opposite headings share a
     * shade, which nobody can see, and the seam, which everybody could, is gone. */
    double folded = turns < 0.5 ? turns * 2 : (1 - turns) * 2;
    int shade = (int)(folded * shades);
    return shade >= shades ? shades - 1 : shade;
}

/* Off one edge and back on the other. The edges stop pushing back when this is
 * on, because a wall and a door in the same place is neither. */
static void wrap_position(bird_t *bird) {
    double width = screen.width, height = screen.height;
    if (bird->x < 0) bird->x += width;
    if (bird->x >= width) bird->x -= width;
    if (bird->y < 0) bird->y += height;
    if (bird->y >= height) bird->y -= height;
}

/* One frame of wing: the clock runs at WING_HZ beats a second whatever the frame
 * rate, and a bird that has just finished a beat sometimes stops to glide. */
static void beat_wings(bird_t *bird) {
    if (bird->gliding > 0) {
        bird->gliding -= frame_seconds;
        if (bird->gliding < 0) bird->gliding = 0;
        bird->wing = 0;
        return;
    }
    bird->wing_clock += WING_HZ * WING_CYCLE * frame_seconds;
    while (bird->wing_clock >= 1.0) {
        bird->wing_clock -= 1.0;
        bird->wing = (bird->wing + 1) % WING_CYCLE;
        if (bird->wing == 0 && random_unit() < GLIDE_CHANCE) {
            double seconds =
                GLIDE_SECONDS_MIN + (GLIDE_SECONDS_MAX - GLIDE_SECONDS_MIN) * random_unit();
            bird->gliding = seconds;
        }
    }
}

/*
 * Text as the flock.
 *
 * Every letter is a bird whose home is its cell. What is about text, who is where
 * and when each one leaves, lives in letters.c; what is here is what is about
 * flying. A letter at home is perched and the birds neither see it nor move it. A
 * letter that has left is a bird like the rest, flocking by the same rules, and
 * the one thing it adds is that it can be called home: it then steers at its cell
 * and lands on it exactly, as the intro's formation does, with a guarantee on top.
 */
enum { MAX_LETTERS = 16384, LETTERS_PACE_NOTCH = 0 };
/* The pointer scatters what it is on: five cells across and three down, or so. A
 * letter it has scattered comes down when its home is a third of a screen's
 * thickness clear of it, wider than the touch so that it does not land to be
 * scattered again. Hawks are bigger. */
static const double POINTER_TOUCH_X = 20.0, POINTER_TOUCH_Y = 26.0, POINTER_CLEAR = 64.0;
static const double HAWK_TOUCH = 44.0, HAWK_CLEAR = 100.0;
/* A letter leaves with a hop, not at full speed: it climbs for this long, from a
 * fifth of its pace, so that a wave is seen to lift the text before it flies. */
static const double LETTER_LIFT_SECONDS = 0.4, LETTER_LIFT_FLOOR = 0.2;
/* Banking is for flocking. A letter coming home turns freely once it is within this
 * many turning circles of its cell, since to bank there is to orbit. */
static const double LETTER_FREE_TURNS = 3.0;

static void read_letter_pose(const void *context, int index, letters_pose_t *pose) {
    const bird_t *birds = context;
    pose->x = birds[index].x;
    pose->y = birds[index].y;
    pose->shade = birds[index].shade;
}

static void place_the_letters(bird_t *birds) {
    for (int i = 0; i < config.birds; i++) {
        birds[i] = (bird_t){0};
        birds[i].x = the_letters.letter[i].home_x;
        birds[i].y = the_letters.letter[i].home_y;
        birds[i].perched = 1;
    }
}

/* One frame of the cycle, before the flight: who is scattered, who leaves. The
 * letters that left are put in the air at their homes, facing the way they go. The
 * snapshot is told as well, because it was taken before this and the flight reads
 * it. */
static void tick_the_letters(bird_t *birds, bird_t *snapshot) {
    letters_disturbance_t disturbances[1 + MAX_HAWKS];
    int count = 0;
    if (mouse.present)
        disturbances[count++] = (letters_disturbance_t){
            mouse.x, mouse.y, POINTER_TOUCH_X, POINTER_TOUCH_Y, POINTER_CLEAR, POINTER_CLEAR};
    for (int h = 0; h < config.hawks; h++)
        disturbances[count++] = (letters_disturbance_t){hawks[h].x, hawks[h].y, HAWK_TOUCH,
                                                        HAWK_TOUCH, HAWK_CLEAR, HAWK_CLEAR};
    letters_advance(&the_letters, frame_seconds, disturbances, count);
    for (int k = 0; k < the_letters.launched_count; k++) {
        int i = the_letters.launched[k];
        const letter_t *letter = &the_letters.letter[i];
        birds[i].x = letter->home_x;
        birds[i].y = letter->home_y;
        birds[i].direction = letter->launch_direction;
        birds[i].frame = direction_frame(birds[i].direction);
        birds[i].perched = 0;
        snapshot[i] = birds[i];
    }
}

/* How much of its pace a letter has in the first moments in the air. */
static double letter_lift(int index) {
    double t = the_letters.letter[index].airborne / LETTER_LIFT_SECONDS;
    if (t >= 1) return 1;
    return LETTER_LIFT_FLOOR + (1 - LETTER_LIFT_FLOOR) * t * t * (3 - 2 * t);
}

/* A letter that has been called home steers at its cell and goes no further than
 * it: the last step is the distance left, and it is put on its home exactly, which
 * is what makes the text whole again. Up to a couple of seconds after the call it
 * banks like a bird, except near home where banking would only make it orbit; after
 * that nothing holds it back, and the pace rises with the time left so that the last
 * of them are in by the deadline whatever they were doing. Returns whether it was
 * one of these. */
static int steer_a_letter_home(bird_t *birds, const bird_t *snapshot, int i) {
    letter_t *letter = &the_letters.letter[i];
    if (letter->state != LETTER_HOMING) return 0;
    double dx = letter->home_x - snapshot[i].x, dy = letter->home_y - snapshot[i].y;
    double remaining = sqrt(dx * dx + dy * dy);
    double step = config.speed;
    int relaxed = letter->homing_for >= LETTERS_HOMING_RELAX;
    if (relaxed) {
        double left = LETTERS_HOMING_DEADLINE - letter->homing_for;
        double needed = left > frame_seconds ? remaining * frame_seconds / left : remaining;
        if (needed > step) step = needed;
    }
    if (remaining <= step) {
        birds[i].x = letter->home_x;
        birds[i].y = letter->home_y;
        birds[i].perched = 1;
        letters_land(&the_letters, i);
        return 1;
    }
    double heading = normalized_angle(dy, dx);
    double limit = turn_limit();
    double turning_circle = limit > 1e-9 ? step / limit : 1e9;
    if (!relaxed && remaining > LETTER_FREE_TURNS * turning_circle + step)
        heading = turn_towards(snapshot[i].direction, heading, limit);
    birds[i].direction = heading;
    birds[i].x = snapshot[i].x + step * cos(heading);
    birds[i].y = snapshot[i].y + step * sin(heading);
    birds[i].shade = shade_for(&birds[i]);
    return 1;
}

static int pointer_is_moving(void) {
    return mouse.present && clock_state.seconds - mouse.moved_at < POINTER_MOVING_SECONDS;
}

/* Whether the place is under a hawk: nearer than most of the way to where the
 * flock starts to flee it. */
static int a_hawk_is_over(double x, double y) {
    double reach = HAWK_SCATTER * hawk_reach();
    for (int i = 0; i < config.hawks; i++) {
        double dx = hawks[i].x - x, dy = hawks[i].y - y;
        if (dx * dx + dy * dy < reach * reach) return 1;
    }
    return 0;
}

/* How long a bird of a sign has still to stay away: the pointer, while it moves,
 * and a hawk, while it is over, scatter every bird whose place they reach, and
 * the birds come home when they have gone. A hawk that flew through a sign and
 * moved nothing would look like a fault. */
static double sign_scatter_left(const bird_t *was, int index) {
    if (!formation.writing) return 0;
    double left = was->scattered - frame_seconds;
    if (left < 0) left = 0;
    int target = index >= 0 && index < MAX_BIRDS ? formation.slot[index] : -1;
    if (target < 0) return left;
    double staying = 0;
    if (pointer_is_moving()) {
        double dx = formation.x[target] - mouse.x, dy = formation.y[target] - mouse.y;
        if (dx * dx + dy * dy < (double)MOUSE_REACH * MOUSE_REACH)
            staying = SCATTER_SECONDS + SCATTER_STAGGER * sign_unit((unsigned)index);
    }
    /* A hawk is over in a moment and a long way off, and the places it passes are
     * back in a moment too: a hawk that left the sign in pieces for as long as the
     * pointer does left it in pieces for most of a minute with two hawks up. */
    if (staying == 0 && a_hawk_is_over(formation.x[target], formation.y[target]))
        staying = HAWK_SCATTER_SECONDS + HAWK_SCATTER_STAGGER * sign_unit((unsigned)index);
    return staying > left ? staying : left;
}

/*
 * The flock in three dimensions.
 *
 * The flight itself is sky3d.c's: birds in a space, around a roost. What is here
 * is the seam to everything that draws. Once a frame the camera takes its place on
 * its orbit, every bird is projected, and what comes out is written into the same
 * bird_t the flat flock is drawn from: the pixel its sprite's corner goes to, the
 * way it points on the screen, and its bin, which is a size and a tint. Past that
 * point the renderers, the recorder and the snapshot do not know the difference.
 */
static struct {
    sky_t world;
    sky_view_t *views;
    int views_capacity;
    int flying;
} sky;

/* The flight is tuned in metres and seconds, and at the pace the program ships at
 * a second of the show should be a second of it: the speed slider is the factor
 * on that. */
static const double SKY_TIME = 1.0 / DEFAULT_PACE;
/* No step longer than this, in seconds of flight: a bird turns at a limited rate
 * per step, and a long one is a bird that cannot turn where it was going to. */
static const double SKY_LONGEST_STEP = 1.0 / 30;
enum { SKY_WARMUP_STEPS = 8 * 30 };
/* How far from the line the pointer makes through the sky a bird feels it, in
 * metres: a stick wider than a bird and narrower than the flock. */
static const double SKY_POKE_REACH = 5.0;

/* The sliders, as the weights the flight reads. Each is a factor on what the
 * flight was tuned at, which is the factor its notch is on the default, so every
 * slider means to the space what it means to the plane: more or less of the same. */
static sky_rules_t sky_rules(void) {
    sky_rules_t rules = sky_default_rules();
    rules.separation *= config.separation / DEFAULT_SEPARATION_W;
    rules.alignment *= config.alignment / DEFAULT_ALIGNMENT_W;
    rules.roost *= config.boundary / DEFAULT_BOUNDARY_W;
    rules.turn_rate = turning_notch_radians();
    /* Perception is how far a bird sees in the plane. Here it is how many birds it
     * heeds, one more a notch, and seven, the number starlings were measured
     * heeding, at the default. */
    rules.neighbours = 1 + config.vision_notch;
    return rules;
}

static void sky_camera(sky_camera_t *camera) {
    sky_camera_orbit(camera, sky.world.clock, screen.width, screen.height);
    sky_camera_frame(camera, &sky.world);
}

/* Which of its shapes a bird is drawn as: how much of its length the camera sees,
 * and how much of its span, which its wing beat takes a part of as well. */
static int sky_shape(const sky_view_t *view, double beat) {
    double span = view->across * beat;
    int across = span >= 0.78 ? 0 : span >= 0.48 ? 1 : 2;
    int along = view->along >= 0.6 ? 0 : 1;
    return along * SKY_ACROSS_LEVELS + across;
}

/* Where the birds are on the screen, as the renderers want them. */
static void project_the_sky(bird_t *birds) {
    sky_camera_t camera;
    sky_camera(&camera);
    if (sky.views_capacity < config.birds) {
        sky_view_t *views = realloc(sky.views, sizeof(*views) * (size_t)config.birds);
        if (views == NULL) return;
        sky.views = views;
        sky.views_capacity = config.birds;
    }
    sky_view(&sky.world, config.birds, &camera, sky.views);
    for (int i = 0; i < config.birds; i++) {
        const sky_view_t *view = &sky.views[i];
        bird_t *bird = &birds[i];
        if (!view->visible) {
            bird->x = bird->y = -1e6; /* Nowhere on any screen. */
            continue;
        }
        double half = sky_bin_size(view->bin) / 2.0;
        bird->x = view->x - half;
        bird->y = view->y - half;
        bird->layer = view->bin;
        bird->direction = view->angle;
        bird->frame = direction_frame(view->angle);
        bird->shape = sky_shape(view, WING_SPAN[WING_SEQUENCE[bird->wing % WING_CYCLE]]);
        beat_wings(bird);
    }
    for (int i = 0; i < config.hawks && i < sky.world.hawk_count; i++) {
        sky_view_t view;
        hawk_t *hawk = &hawks[i];
        sky_hawk_view(&sky.world, i, &camera, &view);
        if (!view.visible) {
            hawk->x = hawk->y = -1e6;
            continue;
        }
        hawk->x = view.x;
        hawk->y = view.y;
        hawk->layer = view.bin;
        hawk->direction = view.angle;
        hawk->frame = direction_frame(view.angle);
        /* It soars, wings out, until the dive, and then it beats. */
        if (sky.world.hawks[i].diving) {
            hawk->wing_clock += WING_HZ * WING_CYCLE * frame_seconds;
            while (hawk->wing_clock >= 1.0) {
                hawk->wing_clock -= 1.0;
                hawk->wing = (hawk->wing + 1) % WING_CYCLE;
            }
        } else {
            hawk->wing = 0;
        }
    }
}

static void begin_the_sky(bird_t *birds) {
    sky_destroy(&sky.world);
    /* The only draw from the flock's own numbers, so a seed is still a flock. */
    uint32_t seed = next_random();
    if (sky_init(&sky.world, config.birds, seed) != SKY_OK) exit(EXIT_FAILURE);
    sky_populate(&sky.world, 0, config.birds, 0);
    sky.flying = 1;
    /* Let it settle before anybody looks: a flock thrown into the air as a cloud
     * takes a few seconds to become one. */
    sky_rules_t rules = sky_rules();
    for (int step = 0; step < SKY_WARMUP_STEPS; step++)
        sky_step(&sky.world, config.birds, &rules, NULL, SKY_LONGEST_STEP);
    sky.world.clock = 0;
    sky_set_hawks(&sky.world, config.hawks);
    for (int i = 0; i < config.birds; i++) {
        birds[i] = (bird_t){0};
        birds[i].wing = (int)(random_unit() * WING_CYCLE) % WING_CYCLE;
        birds[i].wing_clock = random_unit();
    }
    project_the_sky(birds);
}

static int grow_the_sky(int from, int to) {
    if (!sky.flying) return 1;
    if (sky_reserve(&sky.world, to) != SKY_OK) return 0;
    if (to > from) sky_populate(&sky.world, from, to - from, 1);
    return 1;
}

static void end_the_sky(void) {
    sky_destroy(&sky.world);
    free(sky.views);
    memset(&sky, 0, sizeof(sky));
}

static void fly_the_sky(bird_t *birds) {
    sky_rules_t rules = sky_rules();
    sky_poke_t poke = {0};
    if (mouse.present) {
        sky_camera_t camera;
        sky_camera(&camera);
        poke.active = 1;
        poke.reach = SKY_POKE_REACH;
        sky_camera_ray(&camera, mouse.x, mouse.y, poke.origin, poke.direction);
    }
    sky_set_hawks(&sky.world, config.hawks);
    double seconds = flight_seconds() * SKY_TIME;
    int steps = (int)ceil(seconds / SKY_LONGEST_STEP - 1e-9);
    if (steps < 1) steps = 1;
    for (int step = 0; step < steps; step++)
        sky_step(&sky.world, config.birds, &rules, &poke, seconds / steps);
    project_the_sky(birds);
}

static void update_birds(bird_t *birds, const bird_t *snapshot, const spatial_grid_t *grid) {
    measure_flocks(snapshot);
    for (int i = 0; i < config.birds; i++) {
        if (snapshot[i].perched) continue; /* A letter at home stays where it is. */
        if (letters_mode && steer_a_letter_home(birds, snapshot, i)) continue;
        double direction = flock_direction(snapshot, grid, i);
        /* Banking is for flocking. Two things are not flocking and are exempt: a
         * bird writing a letter, which has to be able to land on it, and a bird
         * inside the panel's turn zone, whose push is a constraint rather than a
         * force. Limiting that one would break the panel's unreachability, which
         * is proved on the assumption that a bird can turn away at once. */
        double want_x, want_y;
        int writing = formation_target_for(&snapshot[i], i, &want_x, &want_y);
        if (!writing && !legend_turn_zone(snapshot[i].x, snapshot[i].y)) {
            /* A swerve is a turn the banking would not allow. */
            double limit = turn_limit();
            if (is_swerving(i))
                limit = limit * SWERVE_TURN < 2 * M_PI ? limit * SWERVE_TURN : 2 * M_PI;
            direction = turn_towards(snapshot[i].direction, direction, limit);
        }
        birds[i].direction = direction;
        birds[i].alarmed = !writing && is_swerving(i);
        /* Never past the target: the last step is the distance left, which is
         * what makes a letter crisp instead of a cloud orbiting one. */
        double step = config.speed * flock_pace(snapshot[i].flock);
        if (snapshot[i].layer > 0) step *= FAR_PACE; /* Further away moves slower: parallax. */
        if (letters_mode) step *= letter_lift(i);
        if (writing) {
            double dx = want_x - birds[i].x, dy = want_y - birds[i].y;
            double remaining = sqrt(dx * dx + dy * dy);
            if (remaining < step) step = remaining;
        }
        birds[i].x += step * cos(direction);
        birds[i].y += step * sin(direction);
        if (the_rain_is_falling) wrap_position(&birds[i]);
        if (formation.sign) birds[i].scattered = sign_scatter_left(&snapshot[i], i);
        birds[i].shade = shade_for(&birds[i]);
        /* A writer of a sign is the colour of its place, and keeps it from the
         * moment it is sent: its heading goes round the loop of its hover once or
         * twice a second, and a colour that went round the ramp with it would make
         * the letters flicker. */
        if (the_sign.kind == SIGN_PICTURE)
            birds[i].shade = picture_shade[i]; /* The picture's colour, flying or home. */
        else if (writing && formation.sign && config.flocks == 1 && palette_shades() > 1)
            birds[i].shade = sign_shade_for(i);
        if (!letters_mode) beat_wings(&birds[i]); /* Letters do not have wings. */
        if (config.trails && i % TRAIL_EVERY == 0) {
            /* Where it was, not where it is: a tail behind, never under. */
            birds[i].trail_x[birds[i].trail_at] = snapshot[i].x;
            birds[i].trail_y[birds[i].trail_at] = snapshot[i].y;
            birds[i].trail_at = (birds[i].trail_at + 1) % TRAIL_LENGTH;
            if (birds[i].trail_held < TRAIL_LENGTH) birds[i].trail_held++;
        }
    }
}

/* One step of drifting. How far is counted in spacings a second at the pace the
 * speed slider asks for, so a swarm crosses the screen in about the same time on
 * any screen: about half a minute at the shipped pace. */
static void drift_the_fireflies(bird_t *birds, const bird_t *snapshot, const spatial_grid_t *grid) {
    double step = FIREFLY_DRIFT * firefly_spacing() * (config.pace / DEFAULT_PACE) * frame_seconds;
    for (int i = 0; i < config.birds; i++) {
        double direction = firefly_direction(snapshot, grid, i);
        /* The panel's push is a constraint and not a steer, as it is for a bird. */
        if (!legend_turn_zone(snapshot[i].x, snapshot[i].y))
            direction = turn_towards(snapshot[i].direction, direction, turn_limit());
        birds[i].direction = direction;
        birds[i].x += step * cos(direction);
        birds[i].y += step * sin(direction);
    }
}

/* What the coupling slider and the sight slider mean here. The push is the
 * shipped one times the alignment slider's ratio to its default, so the notch
 * that was a weight of one and a half is a push of one, and the bottom of the
 * bar is no coupling worth the name. The sight is the perception slider's ratio
 * to its default times the shipped sight. */
static fireflies_law_t firefly_law(void) {
    fireflies_law_t law;
    law.sight = FIREFLY_SIGHT * firefly_spacing() * config.vision_radius / DEFAULT_VISION_RADIUS;
    law.push = FIREFLY_PUSH * config.alignment / DEFAULT_ALIGNMENT_W;
    law.bend = FIREFLY_BEND;
    law.refractory = FIREFLY_REFRACTORY;
    return law;
}

/* The clocks, once a frame: the lantern throws the phases of the ones near it,
 * every clock runs on, every flash is seen, and each firefly takes the shade its
 * glow has reached, or none. */
static void light_the_night(bird_t *birds) {
    if (fireflies_grow(&night, config.birds, FIREFLY_PERIOD, FIREFLY_SPREAD, random_unit) !=
        FIREFLIES_OK) {
        perror("Out of memory");
        exit(EXIT_FAILURE);
    }
    double startled = 1 - exp(-FIREFLY_STARTLE * frame_seconds);
    double reach = FIREFLY_LANTERN * firefly_spacing();
    for (int i = 0; i < config.birds; i++) {
        night.fly[i].x = birds[i].x;
        night.fly[i].y = birds[i].y;
        night.fly[i].sky = birds[i].layer;
        if (!mouse.present) continue;
        double dx = birds[i].x - mouse.x, dy = birds[i].y - mouse.y;
        if (dx * dx + dy * dy < reach * reach && random_unit() < startled)
            fireflies_scatter(&night, i, random_unit());
    }
    fireflies_law_t law = firefly_law();
    fireflies_step(&night, frame_seconds, screen.width, screen.height, &law);
    int shades = palette_shades();
    for (int i = 0; i < config.birds; i++) birds[i].shade = fireflies_level(&night, i, shades);
}

static int bird_placement(const bird_t *bird, kitty_graphics_placement_t *placement) {
    if (bird->x < 0 || bird->y < 0) return 0;

    /* The flat flock is kept out of the panel by a force. A space is projected
     * wherever it lands, so what falls behind the panel is not drawn. */
    if (sky_mode && bird->x < screen.legend_width && bird->y < screen.legend_height) return 0;

    int pixel_x = (int)bird->x;
    int pixel_y = (int)bird->y;
    int column = pixel_x / screen.cell_width;
    int row = pixel_y / screen.cell_height;
    if (column >= screen.cols || row >= screen.rows) return 0;

    /* The far layer underneath; in a space each size over the one before it. */
    int z_index = bird->layer > 0 ? -1 : 0;
    if (sky_mode) z_index = bird->layer;
    *placement = (kitty_graphics_placement_t){
        .image_id = sprite_image_id(bird),
        .placement_id = 0,
        .row = row,
        .column = column,
        .x_offset = pixel_x % screen.cell_width,
        .y_offset = pixel_y % screen.cell_height,
        .z_index = z_index,
    };
    return 1;
}

static int frame_limit;
static int bench_frames;
static const char *snapshot_path;
static const char *record_path;
static int record_fps = 25;
/* Zero until asked for: six seconds for a flock, and for text the whole cycle of it
 * (see settle_the_recording_length). */
static int record_seconds;
/* The size of the recording, in cells, as one thing: two flags for one idea, and
 * nobody was ever going to reach for four hundred by a hundred and twenty. */
static int record_columns = 96;
static int record_rows = 26;
static const char *requested_record_size;

/* What the panel's stats row reports, averaged over the last second so the
 * numbers are readable rather than flickering. */
static struct {
    double frame_ms;
    double bytes;
    double rate;
    long counted;
    double window_started;
    double window_ms;
    double window_bytes;
} stats;

/* One slider row: the name, the bar, and the two keys that move it, the lowering
 * one first because that is the end of the bar it works from. The filled length
 * is the notch itself, not a value scaled into cells, so a keypress moves the bar
 * by exactly one cell and nothing rounds. No number: the bar is the readout. */
static void legend_slider(char *line, size_t size, const char *name, int notch, const char *value,
                          char lower, char raise) {
    char keys[8] = "   ";
    if (lower != 0) snprintf(keys, sizeof(keys), "%c/%c", lower, raise); /* A reading has none. */
    char bar[LEGEND_BAR_CELLS * 3 + 1];
    size_t at = 0;

    for (int cell = 0; cell < LEGEND_BAR_CELLS; cell++) {
        const char *glyph = cell < notch ? "\u2593" : "\u2591";
        memcpy(bar + at, glyph, 3);
        at += 3;
    }
    bar[at] = '\0';
    /* The value is right aligned in a column of its own, so the numbers line up
     * under each other and the keys stay where the eye already looks for them.
     * Padded by cells rather than by bytes: a degree sign is two bytes and one
     * cell, and printf counts the bytes, so that row came out a cell short and
     * the panel had a bite out of its right hand side. */
    size_t bytes = strlen(value), glyphs = 0;
    for (const unsigned char *c = (const unsigned char *)value; *c != '\0'; c++)
        if ((*c & 0xc0) != 0x80) glyphs++;
    int column = LEGEND_VALUE_WIDTH + (int)(bytes - glyphs);
    snprintf(line, size, "\u2502 %-*s %s %*s  %s \u2502", LEGEND_NAME_WIDTH, name, bar, column,
             value, keys);
}

/* Enough decimals to tell one notch from the next, and no more: separation and
 * cohesion step in thousandths, the other two in hundredths. */
static void legend_number(char *out, size_t size, double value, int decimals) {
    snprintf(out, size, "%.*f", decimals, value);
}

/* The panel, ten rows of it, anchored to the top left corner. */
static void build_legend(char lines[LEGEND_MAX_ROWS][LEGEND_LINE_MAX]) {
    int rows = legend_rows();
    int inner = LEGEND_COLUMNS - 2;
    size_t at = 0;

    memcpy(lines[0], "\u256d", 3);
    at = 3;
    for (int i = 0; i < inner; i++, at += 3) memcpy(lines[0] + at, "\u2500", 3);
    memcpy(lines[0] + at, "\u256e", 4);

    char value[LEGEND_VALUE_WIDTH + 8];
    /* In three dimensions a row says what it does there, as a factor on the flock's
     * own: the edges are a roost that pulls, and the numbers of the plane are not
     * the space's. */
    legend_number(value, sizeof(value), config.boundary, 2);
    if (sky_mode)
        snprintf(value, sizeof(value), "%.2f\u00d7", config.boundary / DEFAULT_BOUNDARY_W);
    legend_slider(lines[1], LEGEND_LINE_MAX, sky_mode ? "roost" : "boundary", config.boundary_notch,
                  value, 'b', 'B');
    legend_number(value, sizeof(value), config.separation, 3);
    if (sky_mode)
        snprintf(value, sizeof(value), "%.2f\u00d7", config.separation / DEFAULT_SEPARATION_W);
    legend_slider(lines[2], LEGEND_LINE_MAX, "separation", config.separation_notch, value, 's',
                  'S');
    legend_number(value, sizeof(value), config.alignment, 2);
    /* Matching neighbours' clocks and not their headings; as a factor on the push
     * the swarm ships with, like the speed is on its pace. */
    if (fireflies_mode || sky_mode)
        snprintf(value, sizeof(value), "%.*f\u00d7", fireflies_mode ? 1 : 2,
                 config.alignment / DEFAULT_ALIGNMENT_W);
    legend_slider(lines[3], LEGEND_LINE_MAX, fireflies_mode ? "coupling" : "alignment",
                  config.alignment_notch, value, 'a', 'A');
    /* On the panel because it has keys: t and T used to change the banking with
     * nothing on the screen to say what they had done, which is how somebody could
     * turn it to nothing and be left looking at an empty sky. */
    snprintf(value, sizeof(value), "%.0f\u00b0", turning_notch_radians() * 180 / M_PI);
    legend_slider(lines[4], LEGEND_LINE_MAX, "turning", config.turning_notch, value, 't', 'T');
    /* Pixels are whole numbers, so they print as such. */
    snprintf(value, sizeof(value), "%dpx", config.vision_radius);
    /* How far a flash is seen, which is a screen's worth of pixels and not a
     * bird's: in thousands once it will not fit the column. */
    if (fireflies_mode) {
        double sight = firefly_law().sight;
        snprintf(value, sizeof(value), sight < 1000 ? "%.0fpx" : "%.1fk",
                 sight < 1000 ? sight : sight / 1000);
    }
    if (sky_mode) snprintf(value, sizeof(value), "%d", 1 + config.vision_notch);
    legend_slider(lines[5], LEGEND_LINE_MAX,
                  fireflies_mode ? "sight" : (sky_mode ? "neighbours" : "perception"),
                  config.vision_notch, value, 'p', 'P');
    /* A factor on the shipped pace, so 1.0 is the flock as it comes. Last, so the
     * rows above keep the places they have always had. */
    snprintf(value, sizeof(value), "%.1f\u00d7", config.pace);
    legend_slider(lines[6], LEGEND_LINE_MAX, "speed", config.pace_notch, value, 'v', 'V');
    /* Only with somebody to avoid: with one flock the row would be a slider that
     * moves nothing. A quarter a notch, one on the fourth, like the speed. */
    if (config.flocks > 1) {
        snprintf(value, sizeof(value), "%.2f\u00d7", config.avoid_notch / 4.0);
        legend_slider(lines[7], LEGEND_LINE_MAX, "avoidance", config.avoid_notch, value, 'g', 'G');
    }
    /* What the swarm is doing, which no key sets: the order parameter, as a bar. */
    if (fireflies_mode) {
        double order = fireflies_order(&night);
        snprintf(value, sizeof(value), "%.2f", order);
        legend_slider(lines[7], LEGEND_LINE_MAX, "sync", (int)(order * LEGEND_BAR_CELLS + 0.5),
                      value, 0, 0);
    }

    /* What the frame costs, always, rather than behind a flag: it was a switch
     * that did nothing at all unless the panel was up, and the row it wrote into
     * was otherwise blank. Built as text and padded to the same inner width as
     * every other row — written straight into the row it came out four cells short
     * and the panel had a notch in its right hand side. The units are on the
     * numbers, because 2.1 30 60 is three numbers and not a sentence. */
    char measured[LEGEND_LINE_MAX / 2];
    snprintf(measured, sizeof(measured), "%-*s %5.1fms %5.0fKB %3.0ffps", LEGEND_NAME_WIDTH,
             "frame", stats.frame_ms, stats.bytes / 1024.0, stats.rate);
    snprintf(lines[rows - 3], LEGEND_LINE_MAX, "\u2502 %-*s \u2502", inner - 2, measured);
    snprintf(lines[rows - 2], LEGEND_LINE_MAX, "\u2502 %-*s q%*s \u2502", LEGEND_NAME_WIDTH, "quit",
             inner - LEGEND_NAME_WIDTH - 4, "");

    memcpy(lines[rows - 1], "\u2570", 3);
    at = 3;
    for (int i = 0; i < inner; i++, at += 3) memcpy(lines[rows - 1] + at, "\u2500", 3);
    memcpy(lines[rows - 1] + at, "\u256f", 4);
}

static kitty_graphics_status_t queue_legend(kitty_graphics_t *graphics) {
    char lines[LEGEND_MAX_ROWS][LEGEND_LINE_MAX];

    if (screen.legend_width == 0) {
        /* Switched off by a viewport that shrank under it. The panel never moves
         * and never changes size, so these are the only rows that can ever hold
         * stale text, and they are erased one line at a time: an erase of the
         * whole screen would delete the uploaded sprites and leave every later
         * placement pointing at nothing. */
        if (!legend_drawn) return KITTY_GRAPHICS_OK;
        for (int row = 0; row < legend_rows(); row++) {
            kitty_graphics_status_t status = kitty_graphics_write_text(graphics, row, 0, "\033[K");
            if (status != KITTY_GRAPHICS_OK) return status;
        }
        legend_drawn = 0;
        return KITTY_GRAPHICS_OK;
    }

    build_legend(lines);
    for (int row = 0; row < legend_rows(); row++) {
        kitty_graphics_status_t status = kitty_graphics_write_text(graphics, row, 0, lines[row]);
        if (status != KITTY_GRAPHICS_OK) return status;
    }
    legend_drawn = 1;
    return KITTY_GRAPHICS_OK;
}

static kitty_graphics_status_t queue_text_frame(kitty_graphics_t *graphics, const bird_t *birds);

static kitty_graphics_status_t queue_render_frame(kitty_graphics_t *graphics, const bird_t *birds) {
    if (drawing_with_text()) return queue_text_frame(graphics, birds);
    kitty_graphics_status_t status = kitty_graphics_begin_synchronized_update(graphics);
    if (status == KITTY_GRAPHICS_OK) status = kitty_graphics_delete_all_placements(graphics);
    /* Far birds first and underneath, then the tails, the near birds, the hawks. */
    for (int pass = 0; pass < layer_count() && status == KITTY_GRAPHICS_OK; pass++) {
        int layer = layer_in_pass(pass);
        /* The bodies of the fireflies that are dark just now, under the ones that
         * are not. */
        if (layer == 0 && fireflies_mode) {
            for (int i = 0; status == KITTY_GRAPHICS_OK && i < config.birds; i++) {
                if (birds[i].layer != 0 || birds[i].shade >= 0) continue;
                bird_t body = birds[i];
                body.x += firefly_body_inset();
                body.y += firefly_body_inset();
                kitty_graphics_placement_t placement;
                if (bird_placement(&body, &placement)) {
                    placement.image_id = set_image_id(trail_set(0), body.frame);
                    status = kitty_graphics_place(graphics, &placement);
                }
            }
        }
        if (layer == 0 && config.trails && !sky_mode) {
            for (int i = 0; status == KITTY_GRAPHICS_OK && i < config.birds; i += TRAIL_EVERY) {
                if (birds[i].layer != 0) continue;
                for (int step = 0; step < birds[i].trail_held && status == KITTY_GRAPHICS_OK;
                     step++) {
                    /* The newest ghost is the strongest: steps count back from the
                     * write position, oldest first in the ring. */
                    int age = (birds[i].trail_at - 1 - step + TRAIL_LENGTH) % TRAIL_LENGTH;
                    bird_t ghost = birds[i];
                    ghost.x = birds[i].trail_x[age];
                    ghost.y = birds[i].trail_y[age];
                    kitty_graphics_placement_t placement;
                    if (bird_placement(&ghost, &placement)) {
                        placement.image_id = set_image_id(trail_set(step), ghost.frame);
                        status = kitty_graphics_place(graphics, &placement);
                    }
                }
            }
        }
        for (int i = 0; status == KITTY_GRAPHICS_OK && i < config.birds; i++) {
            if (birds[i].layer != layer || birds[i].shade < 0 || birds[i].alarmed) continue;
            kitty_graphics_placement_t placement;
            if (bird_placement(&birds[i], &placement))
                status = kitty_graphics_place(graphics, &placement);
        }
    }
    /* The birds in a wave over the rest of the flock, and under the hawks. */
    for (int i = 0; status == KITTY_GRAPHICS_OK && i < config.birds; i++) {
        kitty_graphics_placement_t placement;
        if (birds[i].alarmed && bird_placement(&birds[i], &placement))
            status = kitty_graphics_place(graphics, &placement);
    }
    for (int i = 0; status == KITTY_GRAPHICS_OK && i < config.hawks; i++) {
        /* Over every bird, in a space too: it is what the picture is of. In the flat
         * sky that is the near plane, where a layer above it would be the far one. */
        bird_t as_bird = {.x = hawks[i].x - hawk_offset_of(&hawks[i]),
                          .y = hawks[i].y - hawk_offset_of(&hawks[i]),
                          .frame = hawks[i].frame,
                          .layer = sky_mode ? SKY_BINS : 0};
        kitty_graphics_placement_t placement;
        if (bird_placement(&as_bird, &placement)) {
            placement.image_id = hawk_image_id(&hawks[i]);
            status = kitty_graphics_place(graphics, &placement);
        }
    }
    if (status == KITTY_GRAPHICS_OK) status = queue_legend(graphics);
    if (status == KITTY_GRAPHICS_OK) status = kitty_graphics_end_synchronized_update(graphics);
    return status;
}

/*
 * The text renderer's own state: the sprites as pixels, a canvas the size of
 * the screen, and the grid of cells that is diffed against the last frame.
 */
static png_status_t rasterise_sprites(png_image_t *frames);
static void compose_onto(png_image_t *canvas, const png_image_t *frames, const bird_t *birds,
                         int with_ground);

static png_image_t text_sprites[ROTATION_FRAMES * MAX_SPRITE_SETS];
static png_image_t text_canvas;
static cells_t text_cells;
static int text_legend_was_drawn;

/* Whether the terminal can take a 24 bit colour. COLORTERM is the convention,
 * and nearly every terminal that can sets it; one that cannot gets the nearest
 * of the 256 colour cube, which is coarser and still a flock. */
static int terminal_has_truecolor(void) {
    const char *colorterm = getenv("COLORTERM");
    if (colorterm == NULL) return 0;
    return strcmp(colorterm, "truecolor") == 0 || strcmp(colorterm, "24bit") == 0;
}

static int prepare_text_renderer(void) {
    /* Letters are drawn as themselves: there is nothing to rasterise. */
    if (!letters_mode && rasterise_sprites(text_sprites) != PNG_OK) return 0;
    if (cells_init(&text_cells, terminal_has_truecolor()) != CELLS_OK) return 0;
    return 1;
}

/* The ground a composed picture is painted on: a recording's frames, a snapshot
 * under a text renderer, and the colour a far bird's tint is pulled towards. The
 * live screen is never painted, so this is the one place a background exists. */
static const uint8_t PICTURE_GROUND[3] = {18, 18, 24};
/* A cell of a picture of text: the 5 by 7 font at twice its size with a pixel of air
 * a side, which is as small as it reads in a GIF on a page. */
enum { LETTER_PICTURE_WIDTH = 12, LETTER_PICTURE_HEIGHT = 20 };

/* The ground a picture is painted on, and the sky a far bird fades into. A dark
 * one, unless ink has asked the terminal and been told otherwise: a bird as dark
 * as ink pulled towards a dark ground gets darker, and on a white terminal that
 * is the wrong way. */
static const uint8_t *picture_ground(void) {
    return the_ground_is_known() ? ink_ground : PICTURE_GROUND;
}

/* Sized to the screen, and resized with it. */
static int text_renderer_fits_the_screen(void) {
    if (!letters_mode &&
        (text_canvas.width != screen.width || text_canvas.height != screen.height)) {
        png_image_free(&text_canvas);
        if (png_image_alloc(&text_canvas, screen.width, screen.height) != PNG_OK) return 0;
    }
    return cells_resize(&text_cells, screen.cols, screen.rows) == CELLS_OK;
}

/* A hawk over text is an arrow in the hawk's colour, pointing the way it flies:
 * east, then clockwise round the compass on a screen whose y runs down. */
static const uint32_t HAWK_ARROWS[8] = {0x2192, 0x2198, 0x2193, 0x2199,
                                        0x2190, 0x2196, 0x2191, 0x2197};

/* The text as it is this frame: the letters at home as printed, the ones in the air
 * where they are, in the flock's colours if they have none of their own, and the
 * hawks on top. */
static void paint_the_letters(const bird_t *birds) {
    letters_look_t look = {NULL, 0};
    if (sprite_path == NULL && palette()->tints != NULL) {
        look.ramp = palette()->tints;
        look.ramp_shades = palette()->shades;
    }
    letters_paint(&the_letters, &text_cells, read_letter_pose, birds, &look);
    for (int h = 0; h < config.hawks; h++) {
        int col = (int)floor(hawks[h].x / screen.cell_width);
        int row = (int)floor(hawks[h].y / screen.cell_height);
        cell_t *cell = cells_at(&text_cells, col, row);
        if (cell == NULL || cell->wide != CELLS_NARROW) continue;
        cell->glyph = HAWK_ARROWS[(int)floor(hawks[h].direction / (M_PI / 4) + 0.5) & 7];
        memcpy(cell->fg, hawk_colour(), 3);
        cell->fg_kind = CELLS_COLOUR_RGB;
        cell->has_fg = 1;
        cell->attributes = CELLS_BOLD;
    }
}

static kitty_graphics_status_t queue_text_frame(kitty_graphics_t *graphics, const bird_t *birds) {
    if (!text_renderer_fits_the_screen()) return KITTY_GRAPHICS_ERR_MEMORY;
    kitty_graphics_status_t status = kitty_graphics_begin_synchronized_update(graphics);
    /* The panel first, so that when it is switched off the rows it erases are
     * redrawn by the cells that follow in the same frame, and when it is on the
     * cells leave its corner alone rather than drawing over its text. */
    if (status == KITTY_GRAPHICS_OK) status = queue_legend(graphics);
    if (legend_drawn != text_legend_was_drawn) {
        cells_invalidate(&text_cells);
        text_legend_was_drawn = legend_drawn;
    }
    cells_keep_out_of(&text_cells, legend_drawn ? LEGEND_COLUMNS : 0,
                      legend_drawn ? legend_rows() : 0);

    if (letters_mode) {
        paint_the_letters(birds);
    } else {
        compose_onto(&text_canvas, text_sprites, birds, 0);
        cells_read(&text_cells, text_style(), &text_canvas, screen.cell_width, screen.cell_height);
    }
    if (cells_emit(&text_cells) != CELLS_OK) return KITTY_GRAPHICS_ERR_MEMORY;
    if (status == KITTY_GRAPHICS_OK)
        status = kitty_graphics_write_raw(graphics, text_cells.text, text_cells.length);
    if (status == KITTY_GRAPHICS_OK) status = kitty_graphics_end_synchronized_update(graphics);
    return status;
}

static void set_frame_seconds(double seconds);

/*
 * One frame of flight, for the hawks and the flock, from the snapshot and grid the
 * caller has just built of the birds.
 *
 * Above pace one the frame is flown in as many equal steps as the pace needs to
 * keep each no longer than a step at pace one. Flown in a single step, a bird at
 * two and a half times the pace crossed a small terminal's edge band in a frame,
 * decided once where the flock was, and one bird in ten was off the screen at any
 * moment against one in forty at pace one. Each step is the same decision the
 * flock always made, so the flock flies faster and not worse; the price is the
 * simulation run that many times, which --bench shows.
 */
static void fly(bird_t *birds, bird_t *snapshot, spatial_grid_t *grid) {
    if (sky_mode) {
        fly_the_sky(birds);
        return;
    }
    if (letters_mode) tick_the_letters(birds, snapshot);
    int steps = (int)ceil(config.pace - 1e-9);
    if (steps < 1) steps = 1;
    double whole = frame_seconds;
    if (steps > 1) set_frame_seconds(whole / steps);
    for (int step = 0; step < steps; step++) {
        if (step > 0) {
            memcpy(snapshot, birds, sizeof(*birds) * (size_t)config.birds);
            /* Prepared for this many birds by the caller, so it cannot fail here. */
            spatial_grid_build(grid, config.birds, read_bird_position, snapshot);
        }
        hunt(snapshot);
        if (fireflies_mode) {
            /* No wave on a night: nothing there is hunted, and the pointer is a
             * lantern and not a whip. */
            drift_the_fireflies(birds, snapshot, grid);
        } else {
            /* No escape wave over text either: the letters have their own take-off
             * wave, and what a hawk or the pointer does to them is to scatter them. */
            if (!letters_mode) spread_the_alarm(snapshot, grid);
            update_birds(birds, snapshot, grid);
        }
    }
    if (steps > 1) set_frame_seconds(whole);
    for (int i = 0; i < config.birds; i++) birds[i].frame = direction_frame(birds[i].direction);
    if (fireflies_mode) light_the_night(birds);
}

/* The way out after q for letters, which are mostly at home when it comes: each
 * goes up at a speed of its own after a short wait of its own, so that the text
 * does not rise as one sheet. The wait is kept in the field a letter never uses for
 * gliding, and the speed in the one it never uses for its wings. */
static void fly_the_letters_away(bird_t *birds) {
    for (int i = 0; i < config.birds; i++) {
        if (birds[i].perched) {
            birds[i].perched = 0;
            letters_release(&the_letters, i);
            birds[i].gliding = 0.15 * random_unit();
            birds[i].wing_clock = 0.6 + 0.8 * random_unit();
        }
        birds[i].direction = 3 * M_PI / 2;
        if (birds[i].gliding > 0) {
            birds[i].gliding -= frame_seconds;
            continue;
        }
        birds[i].y -= config.base_speed * birds[i].wing_clock;
        birds[i].frame = direction_frame(birds[i].direction);
    }
}

/* The way out after q: straight up, every one of them, and nothing else steering.
 * At the shipped pace whatever the slider says, so that leaving takes the same
 * moment every time: at a fifth of the pace the flock was still on the screen
 * when the program closed it. */
static void fly_away(bird_t *birds) {
    if (letters_mode) {
        fly_the_letters_away(birds);
        return;
    }
    for (int i = 0; i < config.birds; i++) {
        birds[i].direction = 3 * M_PI / 2;
        birds[i].y -= config.base_speed;
        birds[i].frame = direction_frame(birds[i].direction);
    }
    /* They go up still flashing, and not with whatever glow they had on. */
    if (fireflies_mode) light_the_night(birds);
}

/* Move first, then draw, and move both the flock and the hawks before drawing
 * either. Drawing first meant the frame on the screen held the birds from before
 * the step and the hawks from after it — a hawk a whole step, forty pixels, ahead
 * of the hole it had just made, which is not what the recorder drew and not what
 * the physics said. Paused still draws and still reads keys, so the panel answers
 * and the sliders can be explored on a still frame; a step grants one frame of
 * motion and then stands still again. */
static kitty_graphics_status_t render_frame(kitty_graphics_t *graphics, bird_t *birds,
                                            bird_t *snapshot, spatial_grid_t *grid) {
    if (!paused || step_once) {
        step_once = 0;
        fly(birds, snapshot, grid);
    }
    return queue_render_frame(graphics, birds);
}

static void update_speed(void) {
    double pixels_per_second = DEFAULT_SPEED * FRAME_RATE;
    /* On a small screen, choose a safe velocity once at the scheduled rate: no
     * more than a tenth of the shorter side in one frame at sixty. Do not cap the
     * final step, because doing that every time a real frame is late silently
     * lowers the velocity and recreates the FPS-dependent bug. Above about four
     * hundred and fifty pixels tall this changes nothing. */
    double shorter = screen.width < screen.height ? screen.width : screen.height;
    double safe_per_second = shorter / 10.0 * FRAME_RATE;
    if (shorter > 0 && pixels_per_second > safe_per_second) pixels_per_second = safe_per_second;
    config.base_speed = pixels_per_second * frame_seconds;
    flight_pixels_per_second = pixels_per_second;
    /* The pace multiplies what the screen allows rather than being capped by it,
     * so on a small screen every notch of the slider still does something. */
    config.speed = config.base_speed * config.pace;
}

static void set_frame_seconds(double seconds) {
    /* CLOCK_MONOTONIC may still return the same timestamp for an extremely short
     * frame on a coarse clock. Zero time means zero movement, never a fallback to
     * a full sixty-hertz step. */
    frame_seconds = seconds > 0 ? seconds : 0;
    update_speed();
}

static double notch_value(int notch, double minimum, double maximum) {
    return minimum + (maximum - minimum) * notch / LEGEND_BAR_CELLS;
}

/* The inverse: which notch a real value belongs to. */
static int notch_for_integer(int value, int minimum, int maximum) {
    return ((value - minimum) * LEGEND_BAR_CELLS + (maximum - minimum) / 2) / (maximum - minimum);
}

static int notch_integer(int notch, int minimum, int maximum) {
    return minimum + ((maximum - minimum) * notch + LEGEND_BAR_CELLS / 2) / LEGEND_BAR_CELLS;
}

/* Derives every tunable from its notch. Called after any key that moves one, so
 * the values and the bars can never disagree. */
static void apply_notches(void) {
    config.boundary = notch_value(config.boundary_notch, BOUNDARY_MIN, BOUNDARY_MAX);
    config.separation = notch_value(config.separation_notch, SEPARATION_MIN, SEPARATION_MAX);
    config.alignment = notch_value(config.alignment_notch, ALIGNMENT_MIN, ALIGNMENT_MAX);

    config.vision_radius = notch_integer(config.vision_notch, MIN_VISION_RADIUS, MAX_VISION_RADIUS);
    config.vision_radius_squared = config.vision_radius * config.vision_radius;
    /* The block of cells the search sweeps has to cover the radius, so it rounds
     * up: a radius that is not a whole number of cells still needs the cell it
     * reaches into. */
    config.vision_cells = (config.vision_radius + SPATIAL_CELL_SIZE - 1) / SPATIAL_CELL_SIZE;

    /* Counted in whole fifths rather than through notch_value, whose arithmetic
     * lands a hair off one at the default: one times anything is that thing, so
     * at the fourth notch the flock is bit for bit the one that shipped. */
    config.pace = PACE_STEP * (config.pace_notch + 1);

    /* Kin by halves below the fourth notch, room and wariness above it, and
     * exactly none of any of it on it. */
    config.avoid_kinship = 0;
    config.avoid_room = 0;
    config.avoid_weight = 0;
    if (config.avoid_notch < DEFAULT_NOTCH) {
        config.avoid_kinship = ldexp(1.0, -config.avoid_notch);
    } else {
        double above =
            (config.avoid_notch - DEFAULT_NOTCH) / (double)(LEGEND_BAR_CELLS - DEFAULT_NOTCH);
        config.avoid_room = 2 * above;
        config.avoid_weight = AVOID_WEIGHT_MAX * above;
    }
    update_speed();
}

/* Left alone for this long, the sliders start wandering by themselves. */
enum { IDLE_SECONDS = 60 };
static int matrix_mode;
static int unlock_fps;
static int requested_perception = DEFAULT_VISION_RADIUS;
static int requested_seed = -1;
static int requested_preset = -1;

/*
 * A preset is the six notches together, because the interesting settings are
 * combinations rather than single values, and naming one is how a look gets
 * shared. The order is boundary, separation, alignment, perception, rate, which
 * is the order the panel shows them in.
 */
typedef struct {
    const char *name;
    const char *help;
    int notch[4];
} preset_t;

static const preset_t PRESETS[] = {
    /* No preset takes the boundary below 7, whatever else it does. Under that a
     * preset is not a look, it is a flock that spends its time outside the frame:
     * school had one bird in nine off the screen. */
    {"murmuration", "one great restless body, the starling look", {7, 3, 9, 8}},
    {"swarm", "tight, fast and nervous, like insects", {7, 8, 3, 3}},
    {"storm", "loose and violent, thrown about", {9, 10, 2, 6}},
};
enum { PRESET_COUNT = sizeof(PRESETS) / sizeof(*PRESETS) };
static const char *PRESET_NAMES[PRESET_COUNT + 1];

static void name_the_presets(void) {
    for (int i = 0; i < PRESET_COUNT; i++) PRESET_NAMES[i] = PRESETS[i].name;
    PRESET_NAMES[PRESET_COUNT] = NULL;
}

static void apply_preset(int which) {
    const preset_t *preset = &PRESETS[which];
    config.boundary_notch = preset->notch[0];
    config.separation_notch = preset->notch[1];
    config.alignment_notch = preset->notch[2];
    config.vision_notch = preset->notch[3];
    apply_notches();
}

/* The option table: the parser and the help text both come off this, so adding a
 * switch is one row and never a second place to keep in step. */
static const option_t OPTIONS[] = {
    /* short, long, alias, kind, target, min, max, names, metavar, help, group, on -h */
    {'n', "birds", NULL, OPTION_INT, &config.birds, 1, MAX_BIRDS, NULL, "COUNT",
     "how many birds (default 800)", "Flock", 1},
    {'s', "size", NULL, OPTION_INT, &config.bird_size, MIN_BIRD_SIZE, MAX_BIRD_SIZE, NULL, "PIXELS",
     "sprite size in pixels (default 30)", "Flock", 1},
    /* k is the hawk key in the panel, so k is the hawk flag on the line: -k 3 used
     * to mean three flocks, which is a trap laid by the program's own help. */
    {'g', "flocks", "groups", OPTION_INT, &config.flocks, 1, MAX_FLOCKS, NULL, "COUNT",
     "flocks that keep to their own kind (default 1)", "Flock", 1},
    {'k', "hawks", NULL, OPTION_INT, &config.hawks, 0, MAX_HAWKS, NULL, "COUNT",
     "predators hunting the flock (default 0)", "Flock", 1},
    {0, "preset", NULL, OPTION_ENUM, &requested_preset, 0, 0, PRESET_NAMES, "NAME",
     "murmuration, swarm, storm", "Flock", 1},
    {0, "seed", NULL, OPTION_INT, &requested_seed, 0, 2147483647, NULL, "N",
     "the same seed gives the same flock", "Flock", 0},

    {0, "boundary", NULL, OPTION_INT, &config.boundary_notch, 0, LEGEND_BAR_CELLS, NULL, "NOTCH",
     "how hard the edges push back (default 4)", "Sliders   0 to 12, as the panel shows them", 0},
    {0, "separation", NULL, OPTION_INT, &config.separation_notch, 0, LEGEND_BAR_CELLS, NULL,
     "NOTCH", "how much a bird keeps its distance (default 4)",
     "Sliders   0 to 12, as the panel shows them", 0},
    {0, "alignment", NULL, OPTION_INT, &config.alignment_notch, 0, LEGEND_BAR_CELLS, NULL, "NOTCH",
     "how much a bird matches its neighbours (default 4)",
     "Sliders   0 to 12, as the panel shows them", 0},
    {0, "turning", NULL, OPTION_INT, &config.turning_notch, 0, LEGEND_BAR_CELLS, NULL, "NOTCH",
     "sharpest turn a frame, 12 is instant (default 8)",
     "Sliders   0 to 12, as the panel shows them", 0},
    {0, "perception", NULL, OPTION_INT, &requested_perception, MIN_VISION_RADIUS, MAX_VISION_RADIUS,
     NULL, "PIXELS", "how far a bird sees, 12 to 60 (default 36)",
     "Sliders   0 to 12, as the panel shows them", 0},
    {0, "speed", NULL, OPTION_INT, &config.pace_notch, 0, LEGEND_BAR_CELLS, NULL, "NOTCH",
     "how fast the flock flies, 0.2x to 2.6x (default 1, 0.4x)",
     "Sliders   0 to 12, as the panel shows them", 0},
    {0, "avoidance", NULL, OPTION_INT, &config.avoid_notch, 0, LEGEND_BAR_CELLS, NULL, "NOTCH",
     "how much flocks keep out of each other's way (default 4)",
     "Sliders   0 to 12, as the panel shows them", 0},

    {'c', "color", "palette", OPTION_ENUM, &config.palette, 0, 0, PALETTE_NAMES, "RAMP",
     "theme, ember, ice, acid, matrix, aurora, prism, potion, dusk, ash, firefly, ink", "Look", 1},
    {0, "shape", NULL, OPTION_ENUM, &config.shape, 0, 0, SHAPE_NAMES, "NAME",
     "bird, arrow, plane, dot", "Look", 1},
    {0, "sprite", NULL, OPTION_STRING, &sprite_path, 0, 0, NULL, "FILE",
     "a PNG you supply, kept in its own colours", "Look", 0},
    {'e', "trails", NULL, OPTION_FLAG, &config.trails, 0, 0, NULL, NULL,
     "faint tails behind the flock", "Look", 0},
    {0, "depth", NULL, OPTION_FLAG, &deep_look, 0, 0, NULL, NULL,
     "a second sky further off: smaller, slower, dimmer birds", "Look", 1},
    {0, "3d", NULL, OPTION_FLAG, &sky_mode, 0, 0, NULL, NULL,
     "a murmuration in three dimensions, seen from a slow orbit (2000 birds, in ink)", "Look", 1},
    {'l', "panel", NULL, OPTION_FLAG, &legend_enabled, 0, 0, NULL, NULL,
     "the sliders in the corner from the start; h toggles them", "Look", 1},
    {0, "render", NULL, OPTION_ENUM, &render_mode, 0, 0, RENDER_NAMES, "HOW",
     "braille by default; sextants, blocks, or kitty in Kitty and Ghostty", "Look", 1},

    {0, "say", NULL, OPTION_STRING, &say_text, 0, 0, NULL, "TEXT",
     "the flock writes TEXT and holds it as a sign", "Sign", 0},
    {0, "clock", NULL, OPTION_FLAG, &clock_mode, 0, 0, NULL, NULL,
     "the flock tells the time, HH:MM, in local time", "Sign", 0},
    {0, "clock-at", NULL, OPTION_STRING, &clock_start, 0, 0, NULL, "TIME",
     "start the clock at HH:MM or HH:MM:SS, not now", "Sign", 0},
    {0, "picture", NULL, OPTION_STRING, &picture_path, 0, 0, NULL, "FILE",
     "the flock draws a PNG, in its colours unless --color is given", "Sign", 0},

    {0, "matrix", NULL, OPTION_FLAG, &matrix_mode, 0, 0, NULL, NULL, "it is raining birds",
     "Oddities", 0},
    {0, "fireflies", NULL, OPTION_FLAG, &fireflies_mode, 0, 0, NULL, NULL,
     "a summer night; they fall into step", "Oddities", 0},
    {0, "text", NULL, OPTION_STRING, &text_path, 0, 0, NULL, "FILE",
     "a file whose letters take flight; text piped in does the same", "Oddities", 0},

    {0, "bench", NULL, OPTION_INT, &bench_frames, 0, 1000000, NULL, "N",
     "run N frames with no terminal, print the numbers, quit", "Output", 0},
    {0, "frames", NULL, OPTION_INT, &frame_limit, 0, 1000000, NULL, "N",
     "quit after N frames, for recording", "Output", 0},
    {0, "snapshot", NULL, OPTION_STRING, &snapshot_path, 0, 0, NULL, "FILE",
     "write the last frame as a PNG", "Output", 0},
    {0, "record", NULL, OPTION_STRING, &record_path, 0, 0, NULL, "FILE",
     "record a GIF, or a .cast for asciinema, with no terminal, and quit", "Output", 0},
    {0, "record-fps", NULL, OPTION_INT, &record_fps, 2, MAX_CAST_FPS, NULL, "RATE",
     "frames a second; a GIF can carry up to 50 (default 25)", "Output", 0},
    {0, "record-seconds", NULL, OPTION_INT, &record_seconds, 1, 120, NULL, "SECONDS",
     "how long the recording runs (default 6, 34 for text)", "Output", 0},
    {0, "record-size", NULL, OPTION_STRING, &requested_record_size, 0, 0, NULL, "COLSxROWS",
     "the size to record at, in cells (default 96x26)", "Output", 0},

    {0, "unlock-fps", NULL, OPTION_FLAG, &unlock_fps, 0, 0, NULL, NULL,
     "render as fast as the terminal allows", "General", 0},
    {0, "screensaver", NULL, OPTION_FLAG, &screensaver_mode, 0, 0, NULL, NULL,
     "quit at once on any key, click or movement, for tmux's lock-command", "General", 0},
};
enum { OPTION_COUNT = sizeof(OPTIONS) / sizeof(*OPTIONS) };

/* The panel teaches the slider keys, so this only has to list the rest. */
#define KEYS_HELP                                                                  \
    "\nKeys   b/B s/S a/A t/T p/P v/V   one notch down / up\n"                     \
    "       space pause   . step   0 reset   +/- birds   Tab preset\n"             \
    "       h panel   e trails   k/K hawks   g/G flocks avoid, with two or more\n" \
    "       enter letters off, or home (with text)   q quit\n"

enum { EXIT_USAGE = 2 }; /* A mistyped command is not a run that went wrong. */

static const option_example_t EXAMPLES[] = {
    {"cbirds", "a flock in braille, and nothing to read"},
    {"cbirds --preset murmuration", "the starling look"},
    {"cbirds --hawks 2 --color ice", "something to watch"},
    {"cbirds --flocks 3 --color ember", "three of them, keeping to their own"},
    {"cbirds --depth --trails", "a second sky behind the first"},
    {"cbirds --fireflies", "a summer night, and they fall into step"},
    {"cbirds --3d --hawks 1", "a murmuration, and a hawk through it"},
    {"cbirds --render kitty", "sprites, in Kitty or Ghostty"},
    {"fastfetch | cbirds", "its letters take flight, and come home"},
    {"cbirds --say \"back in five\"", "the flock writes it and holds it"},
    {"cbirds --screensaver --clock", "a lock screen that tells the time"},
    {"cbirds --record flock.gif", "a GIF, with no terminal in the way"},
    {NULL, NULL},
};

/* Back to the shipped look, which is what someone reaches for after pressing
 * every key to see what it does. */
static void apply_preset_defaults(void) {
    config.boundary_notch = DEFAULT_NOTCH;
    config.separation_notch = DEFAULT_NOTCH;
    config.alignment_notch = DEFAULT_NOTCH;
    config.vision_notch = DEFAULT_VISION_NOTCH;
    /* The banking as well: it was once off the panel, so somebody who had turned
     * it down with t could not see what they did and had nothing else to undo it
     * with. And the speed, which no preset touches, so this is its only way home. */
    config.turning_notch = DEFAULT_TURNING_NOTCH;
    config.pace_notch = letters_mode ? LETTERS_PACE_NOTCH : DEFAULT_PACE_NOTCH;
    config.avoid_notch = DEFAULT_NOTCH;
    apply_notches();
}

/* Reads the decimal digits at *text into *value and moves past them. sscanf's %d
 * is undefined on a number too large for an int, and what the digits are is up
 * to a terminal's reply or a command line, so the overflow is checked here. */
static int read_decimal(const char **text, int *value) {
    const char *at = *text;
    int result = 0;
    if (*at < '0' || *at > '9') return 0;
    for (; *at >= '0' && *at <= '9'; at++) {
        int digit = *at - '0';
        if (result > (INT_MAX - digit) / 10) return 0;
        result = result * 10 + digit;
    }
    *text = at;
    *value = result;
    return 1;
}

/* CSI < button ; column ; row M or m, one based, as mode 1006 sends it. */
static void read_mouse_report(const char *sequence) {
    int button, column, row;
    const char *at = sequence + 1;
    if (sequence[0] != '<') return;
    if (!read_decimal(&at, &button) || *at++ != ';' || !read_decimal(&at, &column) ||
        *at++ != ';' || !read_decimal(&at, &row))
        return;
    if (column < 1 || row < 1) return;
    double x = (column - 0.5) * screen.cell_width, y = (row - 0.5) * screen.cell_height;
    double now = clock_state.seconds;
    if (!mouse.present) {
        mouse.anchor_x = x;
        mouse.anchor_y = y;
        mouse.anchor_at = now;
        mouse.velocity_x = mouse.velocity_y = 0;
    } else if (now - mouse.anchor_at >= POINTER_SPAN) {
        mouse.velocity_x = (x - mouse.anchor_x) / (now - mouse.anchor_at);
        mouse.velocity_y = (y - mouse.anchor_y) / (now - mouse.anchor_at);
        mouse.anchor_x = x;
        mouse.anchor_y = y;
        mouse.anchor_at = now;
    }
    mouse.moved_at = now;
    mouse.x = x;
    mouse.y = y;
    mouse.present = 1;
}

/*
 * Autopilot.
 *
 * A terminal left open on a second monitor becomes the demo for whoever walks
 * past the chair, which is how a thing like this actually spreads. One notch
 * every few seconds, of one slider at a time, so the change is always legible as
 * a change rather than a new program. A keypress ends it, because fighting the
 * user for a slider is worse than not moving it.
 */
static double last_key_at;
static double last_drift_at;

static void drift_a_slider(void) {
    int *notches[] = {&config.boundary_notch, &config.separation_notch, &config.alignment_notch,
                      &config.vision_notch};
    enum { NOTCH_COUNT = sizeof(notches) / sizeof(*notches) };
    int which = (int)(random_unit() * NOTCH_COUNT) % NOTCH_COUNT;
    int *notch = notches[which];
    int step = random_unit() < 0.5 ? -1 : 1;

    /* Turned back at the ends rather than stuck against them. */
    if (*notch + step < 0 || *notch + step > LEGEND_BAR_CELLS) step = -step;
    *notch += step;
    apply_notches();
}

/* It waits a minute to be sure nobody is watching. */
static int flying_itself(void) {
    return clock_state.seconds - last_key_at >= IDLE_SECONDS;
}

static void maybe_drift(void) {
    /* A swarm left alone is the demonstration; the sliders wandering off by
     * themselves would only take its coupling away. */
    if (fireflies_mode || !flying_itself()) return;
    if (clock_state.seconds - last_drift_at < AUTOPILOT_PERIOD) return;
    last_drift_at = clock_state.seconds;
    drift_a_slider();
}

/*
 * Up up down down left right left right b a.
 *
 * Listed in --help rather than hidden, because an easter egg nobody finds is
 * wasted and the line itself is a screenshot. The arrows arrive as CSI final
 * bytes, which the sequence reader already has in hand; b and a are the same
 * keys that move the boundary and alignment sliders, so those move too, and
 * nothing is lost by that.
 */
static const char KONAMI[] = "AABBDCDCba";
enum { KONAMI_LENGTH = sizeof(KONAMI) - 1 };
static char konami_seen[KONAMI_LENGTH];
static int konami_at;

static void konami_note(char key) {
    /* The last ten keys, compared as a whole. A ring rather than a running match
     * because people mash arrows, and a stutter should not throw the sequence
     * away: AAABBDCDCba has the code in it and ought to count. */
    konami_seen[konami_at % KONAMI_LENGTH] = key;
    konami_at++;
    if (konami_at < KONAMI_LENGTH) return;
    for (int i = 0; i < KONAMI_LENGTH; i++)
        if (konami_seen[(konami_at + i) % KONAMI_LENGTH] != KONAMI[i]) return;

    konami_at = 0;
    memset(konami_seen, 0, sizeof(konami_seen));
    if (fireflies_mode) return; /* Hawks do nothing to fireflies. */
    config.hawks = MAX_HAWKS;
    place_hawks();
}

static int handle_input(void) {
    enum { INPUT_NORMAL, INPUT_ESCAPE, INPUT_SEQUENCE };
    static int input_state = INPUT_NORMAL;
    static char sequence[32];
    static size_t sequence_length;
    char input[INPUT_BUFFER_SIZE];
    ssize_t length = read(input_fd, input, sizeof(input));
    if (length > 0) last_key_at = clock_state.seconds;
    /* A screensaver goes at the first sign of anybody: a key, a click, a pointer
     * that moves, all of which arrive here as bytes. Whatever arrives in its first
     * moments is what started it, or the terminal answering something, and is read
     * and thrown away. */
    if (screensaver_mode && length > 0)
        return clock_state.seconds < SCREENSAVER_GRACE && launch_lag <= SCREENSAVER_GRACE;
    for (ssize_t i = 0; i < length; i++) {
        unsigned char key = (unsigned char)input[i];
        int *notch = NULL, step = 0;
        if (input_state == INPUT_ESCAPE) {
            if (key == '[' || key == 'O') {
                input_state = INPUT_SEQUENCE;
                sequence_length = 0;
            } else if (key != '\033') {
                input_state = INPUT_NORMAL;
            }
            continue;
        }
        if (input_state == INPUT_SEQUENCE) {
            /* A final byte ends the sequence; everything before it is its body.
             * Anything longer than the buffer is not a report we know. */
            if (key >= 0x40 && key <= 0x7e) {
                input_state = INPUT_NORMAL;
                sequence[sequence_length] = '\0';
                if (key == 'M' || key == 'm')
                    read_mouse_report(sequence);
                else if (sequence_length == 0)
                    konami_note((char)key); /* A bare arrow, not a modified one. */
                continue;
            }
            if (sequence_length + 1 < sizeof(sequence)) sequence[sequence_length++] = (char)key;
            continue;
        }
        if (key == '\033') {
            input_state = INPUT_ESCAPE;
            continue;
        }

        if (key == 'b' || key == 'a') konami_note((char)key);
        /* Any key ends the intro; the pointer does not. A sign is for reading, and
         * the keys are for the flock: they leave it up. */
        if (!formation.sign) formation_clear();
        switch (key) {
            case 'q':
                return 0;
            case ' ':
                paused = !paused;
                continue;
            case '.':
                step_once = 1; /* One frame of motion, then still again. */
                continue;
            case '0':
                apply_preset_defaults();
                continue;
            case '\r': /* Enter: the letters off, or home. */
            case '\n':
                if (letters_mode) letters_poke(&the_letters);
                continue;
            case '+':
            case '=':
                /* The text decides how many letters there are. */
                if (!letters_mode && config.birds < MAX_BIRDS) {
                    config.birds += config.birds / 4 + 1;
                    if (config.birds > MAX_BIRDS) config.birds = MAX_BIRDS;
                    population_changed = 1;
                }
                continue;
            case '-':
                if (!letters_mode && config.birds > 1) {
                    config.birds -= config.birds / 5 + 1;
                    if (config.birds < 1) config.birds = 1;
                    population_changed = 1;
                }
                continue;
            case 'h':
                legend_enabled = !legend_enabled;
                measure_legend();
                update_turn_distances();
                continue;
            case 'k':
                if (hawk_sets_built && !fireflies_mode && config.hawks < MAX_HAWKS) {
                    config.hawks++;
                    place_one_hawk(config.hawks - 1);
                }
                continue;
            case 'K':
                if (config.hawks > 0) config.hawks--;
                continue;
            case 'T':
                if (config.turning_notch < LEGEND_BAR_CELLS) config.turning_notch++;
                continue;
            case 't':
                if (config.turning_notch > 0) config.turning_notch--;
                continue;
            case 'e':
                /* A tail behind a letter is a smear of the text it is made of, and a
                 * space draws none. */
                if (!fireflies_mode && !letters_mode && !sky_mode) config.trails = !config.trails;
                continue;
            case '\t':
                if (fireflies_mode) continue;
                requested_preset = (requested_preset + 1) % PRESET_COUNT;
                apply_preset(requested_preset);
                continue;
            case 'B':
                notch = &config.boundary_notch;
                step = 1;
                break;
            case 'b':
                notch = &config.boundary_notch;
                step = -1;
                break;
            case 'S':
                notch = &config.separation_notch;
                step = 1;
                break;
            case 's':
                notch = &config.separation_notch;
                step = -1;
                break;
            case 'A':
                notch = &config.alignment_notch;
                step = 1;
                break;
            case 'a':
                notch = &config.alignment_notch;
                step = -1;
                break;
            case 'P':
                notch = &config.vision_notch;
                step = 1;
                break;
            case 'p':
                notch = &config.vision_notch;
                step = -1;
                break;
            case 'V':
                notch = &config.pace_notch;
                step = 1;
                break;
            case 'v':
                notch = &config.pace_notch;
                step = -1;
                break;
            case 'G':
            case 'g':
                /* Nothing to avoid with one flock, and no row to show it on. */
                if (config.flocks < 2) continue;
                notch = &config.avoid_notch;
                step = key == 'G' ? 1 : -1;
                break;
            default:
                continue;
        }
        *notch += step;
        if (*notch < 0) *notch = 0;
        if (*notch > LEGEND_BAR_CELLS) *notch = LEGEND_BAR_CELLS;
        apply_notches();
    }
    return 1;
}

static int wait_for_terminal_io(void) {
    int result;
    do {
        /* The sets are rebuilt each time round: select leaves in them only what was
         * ready. */
        fd_set readable, writable;
        FD_ZERO(&readable);
        FD_ZERO(&writable);
        FD_SET(input_fd, &readable);
        FD_SET(STDOUT_FILENO, &writable);
        result = select((input_fd > STDOUT_FILENO ? input_fd : STDOUT_FILENO) + 1, &readable,
                        &writable, NULL, NULL);
    } while (result < 0 && errno == EINTR);
    return result < 0 ? -1 : 0;
}

#define CBIRDS_VERSION "1.4.0"

/*
 * The program writes its own PNG, with its own encoder.
 *
 * Compositing is the one thing the renderer never has to do, because Kitty does
 * it: so a snapshot rebuilds the rotated sprites as pixels, alpha blends every
 * bird into one canvas, and hands it to png_encode. Fifty milliseconds and a few
 * megabytes for a still, which is a fair price for a picture that the README can
 * honestly say the program drew of itself.
 */
/* `mix` blends the sprite's edges with what is under them, which is what a
 * picture wants. Without it the more opaque pixel simply takes the place: what a
 * text terminal wants, because a cell's colour is read back from its pixels and
 * a blend of two tints where two birds overlap is a colour that is neither —
 * ten thousand of them a recording, each a colour sequence of its own. */
static void blend_sprite(png_image_t *canvas, const png_image_t *sprite, int at_x, int at_y,
                         int mix) {
    for (int y = 0; y < sprite->height; y++) {
        int cy = at_y + y;
        if (cy < 0 || cy >= canvas->height) continue;
        for (int x = 0; x < sprite->width; x++) {
            int cx = at_x + x;
            if (cx < 0 || cx >= canvas->width) continue;
            const uint8_t *src =
                sprite->pixels + ((size_t)y * (size_t)sprite->width + (size_t)x) * 4;
            uint8_t *dst = canvas->pixels + ((size_t)cy * (size_t)canvas->width + (size_t)cx) * 4;
            unsigned alpha = src[3];
            if (alpha == 0) continue;
            if (!mix) {
                if (alpha > dst[3]) memcpy(dst, src, 4);
                continue;
            }
            /* Straight alpha over, done properly: the colour underneath counts
             * only for as much of it as is there. Blending against a transparent
             * pixel as if it were opaque black darkened every anti-aliased edge,
             * which on a text terminal — where the cell's colour is the mean of
             * its pixels — made every bird a slightly different shade and every
             * cell a fresh colour sequence. */
            unsigned under = dst[3] * (255 - alpha) / 255;
            unsigned out_alpha = alpha + under;
            for (int c = 0; c < 3; c++)
                dst[c] = (uint8_t)((src[c] * alpha + dst[c] * under) / out_alpha);
            dst[3] = (uint8_t)out_alpha;
        }
    }
}

/* The rotated sprites as pixels, which is what compositing needs and the
 * renderer never does, because Kitty does it. */
/*
 * Every shade, and the hawks' bigger silhouette after them, laid out exactly as
 * the image ids are: shade major, then the hawk set. A snapshot that rasterised
 * one shade and used it for every bird was a picture of a flock that does not
 * exist, which is what the README's stills and the demo were until now.
 */
/* A wing beat, seen from above, is the span foreshortening: the sprite is
 * squashed across the axis of flight and set back in the middle of its square. */
static png_status_t squash_wings(const png_image_t *square, double span, png_image_t *out) {
    int height = (int)(square->height * span + 0.5);
    if (height < 1) height = 1;
    png_image_t narrow = {0, 0, NULL};
    png_status_t status = png_resize(square, square->width, height, &narrow);
    if (status == PNG_OK) status = png_image_alloc(out, square->width, square->height);
    if (status == PNG_OK) {
        int top = (square->height - height) / 2;
        for (int y = 0; y < height; y++)
            memcpy(out->pixels + ((size_t)(top + y) * (size_t)out->width) * 4,
                   narrow.pixels + (size_t)y * (size_t)narrow.width * 4, (size_t)narrow.width * 4);
    }
    png_image_free(&narrow);
    return status;
}

static void fade_alpha(png_image_t *image, double factor) {
    size_t count = (size_t)image->width * (size_t)image->height;
    for (size_t i = 0; i < count; i++)
        image->pixels[i * 4 + 3] = (uint8_t)(image->pixels[i * 4 + 3] * factor + 0.5);
}

/* A far bird's tint: the shade's own, pulled toward the ground. */
static void far_tint(png_image_t *image, int shade) {
    const palette_t *chosen = palette();
    if (sprite_path != NULL || chosen->tints == NULL) {
        /* Artwork of somebody's own keeps its colours and is only dimmed. */
        png_tint(image, 150, 150, 150, PNG_TINT_MULTIPLY);
        return;
    }
    if (shade >= chosen->shades) shade = chosen->shades - 1;
    uint8_t rgb[3];
    pulled_towards(chosen->tints[shade], picture_ground(), FAR_DIM, rgb);
    png_tint(image, rgb[0], rgb[1], rgb[2], chosen->mode);
}

/* What distance does to a bird in three dimensions: smaller, which the bin's size
 * says, and dimmer, which is this. The nearest bin is the ramp's first shade as it
 * is and the farthest its last, pulled a quarter of the way to the ground as well.
 * A farther bird has to be fainter than the ramp's darkest shade says, or the
 * flock has no depth to it; and at half way, which is what looked best on a
 * picture, the farthest bin was a contrast of 1.4 against the ground, which in a
 * terminal is not there at all. At a quarter it is two to one, the contrast the
 * ramps themselves are kept to, and the nearest is seventeen. */
static const double SKY_DIM = 0.25;

static void tint_sky(png_image_t *image, int bin) {
    const palette_t *chosen = palette();
    double far = (double)(SKY_BINS - 1 - bin) / (SKY_BINS - 1);
    double dim = SKY_DIM * far;
    if (sprite_path != NULL || chosen->tints == NULL) {
        uint8_t level = (uint8_t)(255 * (1 - dim) + 0.5);
        png_tint(image, level, level, level, PNG_TINT_MULTIPLY);
        return;
    }
    int shade = (int)(far * (chosen->shades - 1) + 0.5);
    uint8_t rgb[3];
    pulled_towards(chosen->tints[shade], picture_ground(), dim, rgb);
    png_tint(image, rgb[0], rgb[1], rgb[2], chosen->mode);
}

/* One geometry — a size and a wing span — rotated once per frame, and every set
 * that shares it copied and tinted from that. Rotation is the expensive half and
 * does not depend on the colour. */
typedef void (*tint_fn)(png_image_t *image, int argument);

/* Not a shade of the ramp: the light a bird in a wave is drawn in. */
enum { LIT_SHADE = -1 };

static void tint_flock(png_image_t *image, int shade) {
    if (shade == LIT_SHADE)
        highlight_tint(image);
    else
        palette_tint(image, shade);
}
static void tint_hawk(png_image_t *image, int unused) {
    (void)unused;
    hawk_tint(image);
}
static void tint_trail(png_image_t *image, int step) {
    palette_tint(image, palette_shades() / 2);
    fade_alpha(image, TRAIL_ALPHA[step]);
}
/* The tails are not drawn on a night, so their sets are the firefly's body. */
static void tint_body(png_image_t *image, int unused) {
    (void)unused;
    const palette_t *chosen = palette();
    if (sprite_path != NULL || chosen->tints == NULL) {
        png_tint(image, 70, 70, 70, PNG_TINT_MULTIPLY);
        return;
    }
    const uint8_t *last = chosen->tints[chosen->shades - 1];
    uint8_t rgb[3];
    pulled_towards(last, picture_ground(), FIREFLY_BODY_FADE, rgb);
    png_tint(image, rgb[0], rgb[1], rgb[2], chosen->mode);
}

static png_status_t rasterise_geometry(const png_image_t *source, png_image_t *frames, int size,
                                       double span, const int *sets, const int *arguments,
                                       int count, tint_fn tint) {
    png_image_t square = {0, 0, NULL}, squashed = {0, 0, NULL};
    int work = size * SPRITE_SUPERSAMPLE;
    if (work > SPRITE_WORK_MAX) work = SPRITE_WORK_MAX;
    if (work > source->width) work = source->width;
    png_status_t status = png_resize(source, work, work, &square);
    if (status == PNG_OK && span < 1.0) {
        status = squash_wings(&square, span, &squashed);
        png_image_free(&square);
        square = squashed;
    }
    for (int i = 0; i < ROTATION_FRAMES && status == PNG_OK; i++) {
        png_image_t base = {0, 0, NULL};
        status = png_rotate_resize(&square, i * FRAME_ANGLE * M_PI / 180.0, size, size, &base);
        for (int k = 0; k < count && status == PNG_OK; k++) {
            png_image_t *frame = &frames[sets[k] * ROTATION_FRAMES + i];
            status = png_image_alloc(frame, base.width, base.height);
            if (status != PNG_OK) break;
            memcpy(frame->pixels, base.pixels, (size_t)base.width * (size_t)base.height * 4);
            tint(frame, arguments[k]);
        }
        png_image_free(&base);
    }
    png_image_free(&square);
    return status;
}

/* A sign is read at the size of its letters, and a bird of thirty pixels on a
 * letter whose cells are twelve apart is a smudge: on an eighty column terminal
 * it was not a word any more. So when --size was not given a sign's bird is as
 * wide as the distance between two cells of its letters, and never more than the
 * usual thirty. This is the same fit the sign is laid out by, on the screen as it
 * is, with a margin of its own since the bird's size is what is being chosen. */
static int sign_bird_size(void) {
    if (screen.width <= 0 || screen.height <= 0) return DEFAULT_BIRD_SIZE;
    char clean[SIGN_TEXT_MAX];
    sign_lines_t lines;
    double cell;
    const double margin = 40;
    if (the_sign.kind == SIGN_PICTURE) {
        /* The distance between neighbours, as the picture will have them. */
        picture_fit_t fit = picture_fit(&picture_image, margin, margin, screen.width - 2 * margin,
                                        screen.height - 2 * margin);
        int size = (int)(sqrt(fit.width * fit.height * picture_ink / config.birds) + 0.5);
        return size < MIN_BIRD_SIZE ? MIN_BIRD_SIZE
                                    : (size > DEFAULT_BIRD_SIZE ? DEFAULT_BIRD_SIZE : size);
    }
    if (the_sign.kind == SIGN_CLOCK)
        sign_clean("00:00", clean, sizeof(clean));
    else
        snprintf(clean, sizeof(clean), "%s", sign_words);
    double used_width, used_height;
    if (sign_fit_in(clean, 0, screen.width - 2 * margin, screen.height - 2 * margin, 1e9, &lines,
                    &cell, &used_width, &used_height) == 0)
        return DEFAULT_BIRD_SIZE;
    int size = (int)(cell + 0.5);
    return size < MIN_BIRD_SIZE ? MIN_BIRD_SIZE
                                : (size > DEFAULT_BIRD_SIZE ? DEFAULT_BIRD_SIZE : size);
}

/* How much of its length and of its span a bird shows in each of its shapes. A
 * bird seen from the side is long and, from above and level, spread; seen head
 * on it is short, and the wings that were out are edge on. The spans are the beat
 * of the flat flock's, so that a bird flapping face on to the camera runs through
 * them, and one seen along its wings stays at the last. */
static const double SKY_LENGTH[SKY_ALONG_LEVELS] = {1.0, 0.5};
static const double SKY_SPAN[SKY_ACROSS_LEVELS] = {1.0, 0.62, 0.34};

/* Squashed along the axis of flight as well as across it, and set back in the
 * middle of its square. */
static png_status_t squash_axes(const png_image_t *square, double length, double span,
                                png_image_t *out) {
    int width = (int)(square->width * length + 0.5), height = (int)(square->height * span + 0.5);
    if (width < 1) width = 1;
    if (height < 1) height = 1;
    png_image_t narrow = {0, 0, NULL};
    png_status_t status = png_resize(square, width, height, &narrow);
    if (status == PNG_OK) status = png_image_alloc(out, square->width, square->height);
    if (status == PNG_OK) {
        int left = (square->width - width) / 2, top = (square->height - height) / 2;
        for (int y = 0; y < height; y++)
            memcpy(out->pixels + ((size_t)(top + y) * (size_t)out->width + (size_t)left) * 4,
                   narrow.pixels + (size_t)y * (size_t)narrow.width * 4, (size_t)narrow.width * 4);
    }
    png_image_free(&narrow);
    return status;
}

/* One shape of one bin, at every heading. */
static png_status_t rasterise_the_sky_shape(const png_image_t *source, png_image_t *frames, int bin,
                                            int shape) {
    png_image_t square = {0, 0, NULL}, squashed = {0, 0, NULL};
    int size = sky_bin_size(bin), set = flock_set(0, shape, bin);
    int work = size * SPRITE_SUPERSAMPLE;
    if (work > SPRITE_WORK_MAX) work = SPRITE_WORK_MAX;
    if (work > source->width) work = source->width;
    png_status_t status = png_resize(source, work, work, &square);
    double length = SKY_LENGTH[shape / SKY_ACROSS_LEVELS],
           span = SKY_SPAN[shape % SKY_ACROSS_LEVELS];
    if (status == PNG_OK && (length < 1.0 || span < 1.0)) {
        status = squash_axes(&square, length, span, &squashed);
        png_image_free(&square);
        square = squashed;
    }
    for (int i = 0; i < ROTATION_FRAMES && status == PNG_OK; i++) {
        png_image_t *frame = &frames[set * ROTATION_FRAMES + i];
        status = png_rotate_resize(&square, i * FRAME_ANGLE * M_PI / 180.0, size, size, frame);
        if (status == PNG_OK) tint_sky(frame, bin);
    }
    png_image_free(&square);
    return status;
}

/* Before anything is sized from the bird: thirty pixels under every renderer
 * when --size was not given, or what fits the letters when it is a sign. */
static void settle_the_bird_size(void) {
    if (config.bird_size != 0) return;
    if (fireflies_mode)
        config.bird_size = FIREFLY_SIZE;
    else if (sky_mode)
        config.bird_size = drawing_with_text() ? SKY_TEXT_BIRD_SIZE : SKY_BIRD_SIZE;
    else
        config.bird_size = a_sign_is_asked_for() ? sign_bird_size() : DEFAULT_BIRD_SIZE;
}

static png_status_t rasterise_sprites(png_image_t *frames) {
    png_image_t source = {0, 0, NULL};
    settle_the_bird_size();
    png_status_t status = load_sprite(&source);
    if (status != PNG_OK) return status;
    int shades = palette_shades();
    int sets[MAX_PALETTE_SHADES + 1], arguments[MAX_PALETTE_SHADES + 1];

    /* In three dimensions a bin is a size and a tint, and every shape of each. */
    if (sky_mode) sky_picture_size = (int)sky_picture(screen.width, screen.height);
    for (int bin = 0; sky_mode && bin < SKY_BINS && status == PNG_OK; bin++)
        for (int shape = 0; shape < SKY_SHAPES && status == PNG_OK; shape++)
            status = rasterise_the_sky_shape(&source, frames, bin, shape);
    /* Near birds: one geometry a wing phase, every shade off each, and the same
     * bird in the light of an escape wave, which costs no rotation of its own. */
    for (int wing = 0; !sky_mode && wing < WING_PHASES && status == PNG_OK; wing++) {
        for (int shade = 0; shade < shades; shade++) {
            sets[shade] = flock_set(shade, wing, 0);
            arguments[shade] = shade;
        }
        sets[shades] = alarm_set(wing);
        arguments[shades] = LIT_SHADE;
        status = rasterise_geometry(&source, frames, config.bird_size, WING_SPAN[wing], sets,
                                    arguments, shades + 1, tint_flock);
    }
    /* Far birds: smaller, wings out, dimmed. */
    if (status == PNG_OK && !sky_mode) {
        int far_size = (int)(config.bird_size * FAR_SIZE + 0.5);
        if (far_size < MIN_BIRD_SIZE) far_size = MIN_BIRD_SIZE;
        for (int shade = 0; shade < shades; shade++) {
            sets[shade] = flock_set(shade, 0, 1);
            arguments[shade] = shade;
        }
        status =
            rasterise_geometry(&source, frames, far_size, 1.0, sets, arguments, shades, far_tint);
    }
    /* Hawks, at twice the size, at each wing phase, and in a space at each size. */
    for (int bin = 0; bin < (sky_mode ? SKY_BINS : 1); bin++) {
        for (int wing = 0; wing < WING_PHASES && status == PNG_OK; wing++) {
            int set = hawk_set(bin * WING_PHASES + wing), argument = 0;
            status = rasterise_geometry(&source, frames,
                                        sky_mode ? hawk_size_in_layer(bin) : hawk_sprite_size(),
                                        WING_SPAN[wing], &set, &argument, 1, tint_hawk);
        }
    }
    /* Tails: one geometry, a set a step of fading. Not in three dimensions, where a
     * tail would have to be a set for every size. */
    if (status == PNG_OK && !sky_mode) {
        int trail_sets[TRAIL_LENGTH], steps[TRAIL_LENGTH];
        for (int step = 0; step < TRAIL_LENGTH; step++) {
            trail_sets[step] = trail_set(step);
            steps[step] = step;
        }
        status = rasterise_geometry(&source, frames, trail_sprite_size(), 1.0, trail_sets, steps,
                                    TRAIL_LENGTH, fireflies_mode ? tint_body : tint_trail);
    }
    png_image_free(&source);
    return status;
}

static void free_sprites(png_image_t *frames) {
    for (int i = 0; i < ROTATION_FRAMES * MAX_SPRITE_SETS; i++) png_image_free(&frames[i]);
}

/*
 * A window of another size wants birds of another size, in a space.
 *
 * The flat flock's birds are as big as --size says in whatever window they fly,
 * but a space is framed to fill the picture and its birds are a share of the
 * picture, so a terminal made twice as big showed the same flock at half the
 * size, every bird a speck. The sprites are rebuilt, then, when the picture is
 * a different size; and not before it has stayed that size for a third of a
 * second, because dragging a corner is a new size every frame and a rebuild is a
 * pause of a few hundredths of a second. A change of under a twentieth is not
 * worth the pause, and the birds are a pixel off at most.
 */
enum { RESIZE_SETTLE_FRAMES = 20 };
static const double RESIZE_WORTH = 0.05;
static int picture_held_for, picture_last_seen;

static int the_sprites_are_for_another_picture(void) {
    if (!sky_mode || sky_picture_size <= 0) return 0;
    double now = sky_picture(screen.width, screen.height);
    return fabs(now - sky_picture_size) >= RESIZE_WORTH * sky_picture_size;
}

typedef enum { SPRITES_FIT, SPRITES_REBUILT, SPRITES_FAILED } sprite_fit_t;

/* Called every frame, after the window has been measured. A failure is the
 * caller's to report, as it is at startup. A rebuild is the caller's to tell the
 * clock about: it takes between a third of a second and two, as it does at
 * startup, and the flock should wait for it, not fly on through it. */
static sprite_fit_t fit_the_sprites_to_the_window(kitty_graphics_t *graphics) {
    if (!sky_mode) return SPRITES_FIT;
    int picture = (int)sky_picture(screen.width, screen.height);
    picture_held_for = picture == picture_last_seen ? picture_held_for + 1 : 0;
    picture_last_seen = picture;
    if (picture_held_for < RESIZE_SETTLE_FRAMES || !the_sprites_are_for_another_picture())
        return SPRITES_FIT;

    free_sprites(text_sprites);
    if (rasterise_sprites(text_sprites) != PNG_OK) return SPRITES_FAILED;
    if (render_mode != RENDER_KITTY) return SPRITES_REBUILT;
    /* Kitty keeps what it was sent, under the same ids, so the new images replace
     * the old; and the placements are all made again every frame. */
    kitty_graphics_status_t status = upload_sprite_sets(graphics, text_sprites);
    free_sprites(text_sprites);
    return status == KITTY_GRAPHICS_OK ? SPRITES_REBUILT : SPRITES_FAILED;
}

/* The ground, opaque, so a picture looks like the terminal it was taken in
 * rather than like a cut out. */
static void fill_ground(png_image_t *canvas) {
    const uint8_t *ground = picture_ground();
    for (size_t i = 0; i < (size_t)canvas->width * (size_t)canvas->height; i++) {
        canvas->pixels[i * 4 + 0] = ground[0];
        canvas->pixels[i * 4 + 1] = ground[1];
        canvas->pixels[i * 4 + 2] = ground[2];
        canvas->pixels[i * 4 + 3] = 255;
    }
}

/* The same order the live renderer places in: tails, then the flock, then the
 * hawks over the top. On a ground for a picture; on nothing at all for a text
 * terminal, whose own background is the sky and whose cells must not paint it. */
static void compose_the_wave(png_image_t *canvas, const png_image_t *frames, const bird_t *birds,
                             int with_ground) {
    for (int i = 0; i < config.birds; i++) {
        if (!birds[i].alarmed || birds[i].layer != 0) continue;
        int set = alarm_set(WING_SEQUENCE[birds[i].wing % WING_CYCLE]);
        const png_image_t *sprite =
            &frames[set * ROTATION_FRAMES + birds[i].frame % ROTATION_FRAMES];
        if (sprite->pixels == NULL) continue;
        blend_sprite(canvas, sprite, (int)birds[i].x, (int)birds[i].y, with_ground);
    }
}

static void compose_onto(png_image_t *canvas, const png_image_t *frames, const bird_t *birds,
                         int with_ground) {
    int shades = palette_shades();
    if (with_ground)
        fill_ground(canvas);
    else
        memset(canvas->pixels, 0, (size_t)canvas->width * (size_t)canvas->height * 4);
    /* A picture lays the lit birds over the flock, as a later bird covers an
     * earlier one. A text terminal keeps the first thing in a cell, so there
     * they go down first: the cell is read back as the colour that fills most of
     * it, and a lit bird that loses to the one beside it is not lit. */
    if (!with_ground) compose_the_wave(canvas, frames, birds, with_ground);

    for (int pass = 0; pass < layer_count(); pass++) {
        int layer = layer_in_pass(pass);
        if (layer == 0 && fireflies_mode) {
            for (int i = 0; i < config.birds; i++) {
                if (birds[i].layer != 0 || birds[i].shade >= 0) continue;
                const png_image_t *body =
                    &frames[trail_set(0) * ROTATION_FRAMES + birds[i].frame % ROTATION_FRAMES];
                if (body->pixels != NULL)
                    blend_sprite(canvas, body, (int)birds[i].x + firefly_body_inset(),
                                 (int)birds[i].y + firefly_body_inset(), with_ground);
            }
        }
        if (layer == 0 && config.trails && !sky_mode) {
            for (int i = 0; i < config.birds; i += TRAIL_EVERY) {
                if (birds[i].layer != 0) continue;
                for (int step = 0; step < birds[i].trail_held; step++) {
                    int age = (birds[i].trail_at - 1 - step + TRAIL_LENGTH) % TRAIL_LENGTH;
                    const png_image_t *sprite = &frames[trail_set(step) * ROTATION_FRAMES +
                                                        birds[i].frame % ROTATION_FRAMES];
                    if (sprite->pixels != NULL)
                        blend_sprite(canvas, sprite, (int)birds[i].trail_x[age],
                                     (int)birds[i].trail_y[age], with_ground);
                }
            }
        }
        for (int i = 0; i < config.birds; i++) {
            /* Dark: a body, above. Lit by a wave: drawn with the wave. */
            if (birds[i].layer != layer || birds[i].shade < 0 || birds[i].alarmed) continue;
            int set = flock_set(birds[i].shade % shades, bird_wing(&birds[i]), birds[i].layer);
            const png_image_t *sprite =
                &frames[set * ROTATION_FRAMES + birds[i].frame % ROTATION_FRAMES];
            if (sprite->pixels == NULL) continue;
            blend_sprite(canvas, sprite, (int)birds[i].x, (int)birds[i].y, with_ground);
        }
    }
    if (with_ground) compose_the_wave(canvas, frames, birds, with_ground);
    for (int i = 0; i < config.hawks; i++) {
        int set = hawk_set(hawk_wing(&hawks[i]));
        const png_image_t *sprite =
            &frames[set * ROTATION_FRAMES + hawks[i].frame % ROTATION_FRAMES];
        if (sprite->pixels == NULL) continue;
        blend_sprite(canvas, sprite, (int)hawks[i].x - hawk_offset_of(&hawks[i]),
                     (int)hawks[i].y - hawk_offset_of(&hawks[i]), with_ground);
    }
}

/* A picture of what is on the screen. Under Kitty that is the sprites on a
 * ground; on a text terminal it is the dots or the blocks, painted the way the
 * terminal shows them, because a snapshot of the pixels the cells were read from
 * would be a picture of something nobody saw. */
static int write_snapshot(const char *path, const bird_t *birds) {
    png_image_t canvas = {0, 0, NULL};
    static png_image_t frames[ROTATION_FRAMES * MAX_SPRITE_SETS];
    uint8_t *encoded = NULL;
    size_t encoded_length = 0;
    int written = 0;
    const uint8_t *ground = picture_ground();

    png_status_t status = PNG_OK;
    if (drawing_with_text()) {
        if (cells_paint(&text_cells, letters_mode ? CELLS_TEXT : text_style(), &canvas,
                        letters_mode ? LETTER_PICTURE_WIDTH : screen.cell_width,
                        letters_mode ? LETTER_PICTURE_HEIGHT : screen.cell_height,
                        ground) != CELLS_OK)
            status = PNG_ERR_MEMORY;
    } else {
        status = rasterise_sprites(frames);
        if (status == PNG_OK) status = png_image_alloc(&canvas, screen.width, screen.height);
        if (status == PNG_OK) compose_onto(&canvas, frames, birds, 1);
    }
    if (status == PNG_OK) {
        status = png_encode(&canvas, &encoded, &encoded_length);
    }
    free_sprites(frames);
    png_image_free(&canvas);

    if (status == PNG_OK) {
        FILE *out = fopen(path, "wb");
        if (out != NULL) {
            written = fwrite(encoded, 1, encoded_length, out) == encoded_length;
            /* A full disk may only say so when the buffer is flushed on close. */
            if (fclose(out) != 0) written = 0;
        }
    }
    free(encoded);
    return written;
}

static void usage(FILE *out, const char *program, int everything) {
    options_usage(out, program, "cbirds \u2014 a flock of birds in your terminal.", EXAMPLES,
                  OPTIONS, OPTION_COUNT, everything);
    if (everything) fputs(KEYS_HELP, out);
}

/* A night leaves some of the flock's switches with nothing to do. Each is said
 * once, on stderr, and then does nothing, instead of quietly meaning something
 * else: hawks leave fireflies alone, there is one swarm, a preset is a flock's
 * look, a drifter leaves no tail worth drawing, and the rain is another night. */
static void settle_the_night(void) {
    if (config.hawks > 0) {
        fprintf(stderr, "%s: --hawks does nothing with --fireflies\n", program_name);
        config.hawks = 0;
    }
    if (config.flocks > 1) {
        fprintf(stderr, "%s: --flocks does nothing with --fireflies, which is one swarm\n",
                program_name);
        config.flocks = 1;
    }
    if (requested_preset >= 0) {
        fprintf(stderr, "%s: --preset does nothing with --fireflies\n", program_name);
        requested_preset = -1;
    }
    if (config.trails) {
        fprintf(stderr, "%s: --trails does nothing with --fireflies\n", program_name);
        config.trails = 0;
    }
    if (matrix_mode) {
        fprintf(stderr, "%s: --matrix does nothing with --fireflies\n", program_name);
        matrix_mode = 0;
    }
}

/* HH:MM or HH:MM:SS, as many digits as the person felt like typing. */
static int read_clock_start(const char *text, int *hour, int *minute, int *second) {
    const char *at = text;
    *second = 0;
    if (!read_decimal(&at, hour) || *at++ != ':' || !read_decimal(&at, minute)) return 0;
    if (*at == ':' && (at++, !read_decimal(&at, second))) return 0;
    return *at == '\0' && *hour < 24 && *minute < 60 && *second < 60;
}

/* The picture, read once and for the whole run, and its colours worked out: cut
 * down to a ramp by median cut, and either they are the palette, or, when --color
 * was given, they are only how light and how dark the picture goes. A file that
 * is not a PNG, or too large, is an error like a bad --sprite, said the same way;
 * a picture with no ink in it is only a picture the flock has nothing to draw. */
static void settle_the_picture(void) {
    png_status_t status = load_png_file(picture_path, "a picture", &picture_image);
    if (status != PNG_OK) {
        fprintf(stderr, "%s: %s: %s\n", program_name, picture_path, png_status_string(status));
        exit(EXIT_FAILURE);
    }
    /* Kept no larger than a thousand pixels across: the picture is laid out again
     * whenever the window changes size, and the birds draw it at a fraction of
     * that. A photograph of forty megapixels would cost a quarter of a second a
     * time to cut up; this costs fifteen milliseconds, whatever came in. */
    int longest =
        picture_image.width > picture_image.height ? picture_image.width : picture_image.height;
    if (longest > PICTURE_KEPT_MAX) {
        png_image_t smaller = {0, 0, NULL};
        int width = (int)((double)picture_image.width * PICTURE_KEPT_MAX / longest + 0.5);
        int height = (int)((double)picture_image.height * PICTURE_KEPT_MAX / longest + 0.5);
        if (png_resize(&picture_image, width > 0 ? width : 1, height > 0 ? height : 1, &smaller) ==
            PNG_OK) {
            png_image_free(&picture_image);
            picture_image = smaller;
        }
    }
    uint8_t colours[MAX_PALETTE_SHADES][3];
    int made = picture_quantise(&picture_image, MAX_PALETTE_SHADES, colours);
    if (made == 0) {
        fprintf(stderr, "%s: %s has nothing opaque in it, so the flock flies as usual\n",
                program_name, picture_path);
        png_image_free(&picture_image);
        return;
    }
    /* A bird of the picture's black is a bird nobody can see: the picture's dark is
     * kept for ranking shades, and the colours the birds wear are lifted, towards
     * white, until they stand as clear of the ground as the ramps' own do. */
    picture_dark = picture_luminance(colours[made - 1]);
    picture_light = picture_luminance(colours[0]); /* Lightest first. */
    for (int c = 0; c < made; c++)
        for (int step = 0; step < 32 && contrast_between(colours[c], PICTURE_GROUND) < 1.8; step++)
            for (int channel = 0; channel < 3; channel++) {
                int lifted = colours[c][channel] + (255 - colours[c][channel]) / 12 + 1;
                colours[c][channel] = (uint8_t)(lifted > 255 ? 255 : lifted);
            }
    size_t pixels = (size_t)picture_image.width * (size_t)picture_image.height, ink = 0;
    for (size_t i = 0; i < pixels; i++) ink += picture_image.pixels[i * 4 + 3] > 127;
    picture_ink = (double)ink / (double)pixels;
    memcpy(picture_tints, colours, sizeof(picture_tints));
    picture_palette.shades = made;
    picture_palette.tints = (const uint8_t(*)[3])picture_tints;
    picture_colours_in_use = !palette_was_asked_for;
    if (sprite_path != NULL)
        fprintf(stderr, "%s: --sprite keeps its own colours, so the picture is drawn in them\n",
                program_name);
    the_sign.kind = SIGN_PICTURE;
}

/* The options that make the flock a sign, settled together: which one it is,
 * whether the font can draw it, and what the clock is going by. Said out loud
 * and then ignored where the flock can simply fly as usual, and refused where
 * the command is a mistake. */
static void settle_the_sign(void) {
    int hour = 0, minute = 0, second = 0;
    if (clock_start != NULL) {
        if (!read_clock_start(clock_start, &hour, &minute, &second)) {
            fprintf(stderr, "%s: --clock-at wants HH:MM or HH:MM:SS, not '%s'\n", program_name,
                    clock_start);
            exit(EXIT_USAGE);
        }
        clock_mode = 1;
    }
    if ((say_text != NULL) + clock_mode + (picture_path != NULL) > 1) {
        fprintf(stderr, "%s: --say, --clock and --picture each take the whole sign, so only one\n",
                program_name);
        exit(EXIT_USAGE);
    }
    the_sign.seed = requested_seed >= 0 ? (unsigned)requested_seed : 0u;
    if (picture_path != NULL) settle_the_picture();
    if (say_text != NULL) {
        int kept = sign_clean(say_text, sign_words, sizeof(sign_words));
        int needs = font_text_cells(sign_words);
        if (kept == 0) {
            fprintf(stderr,
                    "%s: --say has nothing the font can draw, so the flock flies as usual\n",
                    program_name);
        } else if (needs > FORMATION_MAX_TARGETS) {
            fprintf(stderr, "%s: --say is too long to write, so the flock flies as usual\n",
                    program_name);
        } else if (needs > config.birds) {
            fprintf(stderr,
                    "%s: that text takes %d birds to write and the flock has %d, so the flock "
                    "flies as usual\n",
                    program_name, needs, config.birds);
        } else {
            the_sign.kind = SIGN_SAY;
        }
    }
    if (clock_mode) {
        the_sign.kind = SIGN_CLOCK;
        /* The locale's own idea of the hour, and nothing else of it: only the
         * names of things in time are asked of it. */
        setlocale(LC_TIME, "");
        the_sign.twelve_hours = sign_wants_twelve_hours(nl_langinfo(T_FMT));
        /* A recording runs on its own clock and not the wall's, so the time it tells
         * is the time it started at, moved on by its own frames. */
        the_sign.virtual_clock = record_path != NULL || bench_frames > 0 || clock_start != NULL;
        the_sign.origin = time(NULL);
        if (clock_start != NULL) {
            struct tm today;
            if (localtime_r(&the_sign.origin, &today) != NULL) {
                today.tm_hour = hour;
                today.tm_min = minute;
                today.tm_sec = second;
                today.tm_isdst = -1;
                the_sign.origin = mktime(&today);
            }
        }
    }
}

/* The option that asks for a sign, or NULL if none does. It goes by what was
 * typed and not by what came of it: --say with nothing the font can draw is no sign
 * and the flock flies as usual, but it was still asked for, and a command that asks
 * for a sign and for text is a mistake whichever of them turns out to be nothing. */
static const char *the_sign_option(void) {
    if (say_text != NULL) return "--say";
    if (clock_start != NULL) return "--clock-at";
    if (clock_mode) return "--clock";
    if (picture_path != NULL) return "--picture";
    return NULL;
}

/* Things that each want to be the whole flock. A night is the flock, the text is
 * the flock, and a sign is what the flock writes: a night has no letters to write
 * with, and text that is the flock leaves nobody to write a sign. A space is the
 * flock in another sky altogether: a night, a text and a sign are all made on the
 * flat one, with its screen to lay them out on, and a space has no screen, only a
 * camera. The one or the other, said in one line and with the status of the other
 * usage errors, before anything is read or opened. Text on a pipe is said when it
 * is found. */
static void refuse_what_does_not_go_together(void) {
    const char *sign = the_sign_option();
    if (sky_mode && fireflies_mode) {
        fprintf(stderr, "%s: --3d does not go with --fireflies: a night is on the flat sky\n",
                program_name);
        exit(EXIT_USAGE);
    }
    if (sky_mode && text_path != NULL) {
        fprintf(stderr, "%s: --3d does not go with --text: a text is laid out on the flat sky\n",
                program_name);
        exit(EXIT_USAGE);
    }
    if (sky_mode && sign != NULL) {
        fprintf(stderr, "%s: --3d does not go with %s: a sign is drawn on the flat sky\n",
                program_name, sign);
        exit(EXIT_USAGE);
    }
    if (sky_mode && (config.flocks > 1 || matrix_mode)) {
        fprintf(stderr, "%s: --3d is one flock over one roost; %s is for the flat sky\n",
                program_name, matrix_mode ? "--matrix" : "--flocks");
        exit(EXIT_USAGE);
    }
    if (fireflies_mode && text_path != NULL) {
        fprintf(stderr, "%s: --fireflies does not go with --text: the text is the flock\n",
                program_name);
        exit(EXIT_USAGE);
    }
    if (fireflies_mode && sign != NULL) {
        fprintf(stderr, "%s: --fireflies does not go with %s: a night cannot write\n", program_name,
                sign);
        exit(EXIT_USAGE);
    }
    if (sign != NULL && text_path != NULL) {
        fprintf(stderr, "%s: %s does not go with --text: the text is the flock\n", program_name,
                sign);
        exit(EXIT_USAGE);
    }
}

/*
 * What was not asked for.
 *
 * Three settings have a default that depends on a switch which may come after them
 * on the line. A night wants more birds, a dot and a ramp of its own, and a space
 * more birds and its own ramp; a picture wants to know whether there was a ramp
 * asked for at all, since it draws in its own colours unless there was. So while the line is read a
 * value no option can give stands for "not asked for" (no birds, no shape, no ramp), the shipped
 * ones are kept to one side, and once the line has been read the three are settled together, here,
 * with the mode that is known by then. Nothing between the two calls may read birds, shape or
 * palette: it would find the stand-ins.
 */
static struct {
    int birds, shape, palette;
} shipped;

static void leave_the_defaults_open(void) {
    shipped.birds = config.birds;
    shipped.shape = config.shape;
    shipped.palette = config.palette;
    config.birds = 0;
    config.shape = config.palette = -1;
}

/* A night's, or the shipped ones, for whatever was left open. What was asked for
 * wins, even when it is the very thing the flock ships with: the name of the
 * default ramp, given, is a ramp given, and a picture then wears it. --matrix names
 * a ramp too, the green one, and a picture beside it is drawn in that: the rain is
 * green, and a picture in its own colours would be the one thing in it that is not. */
static void settle_the_defaults(void) {
    palette_was_asked_for = config.palette >= 0 || matrix_mode;
    if (config.birds == 0)
        config.birds = fireflies_mode ? FIREFLY_COUNT : (sky_mode ? SKY_BIRDS : shipped.birds);
    if (config.shape < 0) config.shape = fireflies_mode ? shape_named("dot") : shipped.shape;
    if (config.palette < 0)
        config.palette = fireflies_mode ? palette_named("firefly")
                                        : (sky_mode ? palette_named("ink") : shipped.palette);
}

/* What does not fly in a space, said once and plainly rather than ignored. */
static void settle_the_space(void) {
    if (deep_look) {
        fprintf(stderr, "%s: --3d replaces --depth: every bird has its own distance\n",
                program_name);
        deep_look = 0;
    }
    if (config.trails) {
        fprintf(stderr, "%s: --3d draws no tails\n", program_name);
        config.trails = 0;
    }
}

static void read_options(int argc, char **argv) {
    char error[160];
    if (argc > 0 && argv[0] != NULL) program_name = argv[0];
    name_the_palettes();
    name_the_presets();
    name_the_shapes();
    leave_the_defaults_open();
    options_status_t status =
        options_parse(OPTIONS, OPTION_COUNT, argc, argv, error, sizeof(error));

    if (status == OPTIONS_HELP || status == OPTIONS_HELP_FULL) {
        usage(stdout, program_name, status == OPTIONS_HELP_FULL);
        exit(EXIT_SUCCESS);
    }
    if (status == OPTIONS_COMPLETION) {
        if (!options_completion(stdout, error, "cbirds", OPTIONS, OPTION_COUNT)) {
            fprintf(stderr, "%s: --completion wants bash, zsh or fish\n", program_name);
            exit(EXIT_USAGE);
        }
        exit(EXIT_SUCCESS);
    }
    if (status == OPTIONS_VERSION) {
        printf("cbirds %s\n", CBIRDS_VERSION);
        exit(EXIT_SUCCESS);
    }
    if (status != OPTIONS_OK) {
        /* Told what was wrong, and where to look, and exiting two rather than one
         * so a script can tell a mistyped command from a run that went wrong. */
        fprintf(stderr, "%s: %s\n", program_name, error);
        fprintf(stderr, "Try '%s --help'.\n", program_name);
        exit(EXIT_USAGE);
    }
    refuse_what_does_not_go_together();
    if (fireflies_mode) settle_the_night();
    if (sky_mode) settle_the_space();
    settle_the_defaults();
    /* A preset is expanded first so that a slider given after it still wins: the
     * table cannot express that order, so the parser's left to right reading is
     * honoured by putting the broad stroke before the fine ones. */
    if (requested_preset >= 0) {
        int boundary = config.boundary_notch, separation = config.separation_notch;
        int alignment = config.alignment_notch;
        apply_preset(requested_preset);
        if (boundary != DEFAULT_NOTCH) config.boundary_notch = boundary;
        if (separation != DEFAULT_NOTCH) config.separation_notch = separation;
        if (alignment != DEFAULT_NOTCH) config.alignment_notch = alignment;
        if (requested_perception != DEFAULT_VISION_RADIUS)
            config.vision_notch =
                notch_for_integer(requested_perception, MIN_VISION_RADIUS, MAX_VISION_RADIUS);
    } else {
        /* The notch is the state the keys move, so a value off that grid could
         * not be one: snap what was asked for to the nearest. */
        config.vision_notch =
            notch_for_integer(requested_perception, MIN_VISION_RADIUS, MAX_VISION_RADIUS);
    }
    apply_notches();
    /* Checked here rather than where it is used, so every mode reports a bad
     * sprite the same way and none of them gets halfway into a run first. */
    if (sprite_path != NULL) {
        png_image_t probe = {0, 0, NULL};
        png_status_t sprite_status = load_sprite(&probe);
        if (sprite_status != PNG_OK) {
            fprintf(stderr, "%s: %s: %s\n", program_name, sprite_path,
                    png_status_string(sprite_status));
            exit(EXIT_FAILURE);
        }
        png_image_free(&probe);
    }
    settle_the_sign();
    if (requested_record_size != NULL) {
        int columns = 0, rows = 0;
        const char *at = requested_record_size;
        if (!read_decimal(&at, &columns) || *at++ != 'x' || !read_decimal(&at, &rows) ||
            *at != '\0' || columns < 40 || columns > 400 || rows < 14 || rows > 120) {
            fprintf(stderr, "%s: --record-size wants COLUMNSxROWS, 40x14 to 400x120, not '%s'\n",
                    program_name, requested_record_size);
            exit(EXIT_USAGE);
        }
        record_columns = columns;
        record_rows = rows;
    }
    /* A palette with one colour in it cannot tell the flocks apart, which is worth
     * saying out loud rather than letting somebody wonder where their three flocks
     * went. */
    if (config.flocks == 1 && config.avoid_notch != DEFAULT_NOTCH)
        fprintf(stderr, "%s: --avoidance is how flocks avoid each other, and there is one\n",
                program_name);
    if (config.flocks > 1 && palette_shades() <= 1)
        fprintf(stderr, "%s: %s has one colour, so the %d flocks will look like one\n",
                program_name, palette()->name, config.flocks);
    /* It is raining birds: green, falling, wrapping, with tails. Every part of it
     * is a switch that already existed, which is the whole joke. */
    if (matrix_mode) {
        config.palette = palette_named("matrix");
        config.trails = 1;
        config.alignment_notch = LEGEND_BAR_CELLS;
        the_rain_is_falling = 1;
        apply_notches();
    }
}

static void forget_the_text(void) {
    free(the_text);
    the_text = NULL;
    the_text_length = 0;
}

/*
 * Where the keys come from, and where the text does.
 *
 * Text piped in takes the standard input, so the keys have to come from the
 * terminal itself: its controlling one, /dev/tty. That is not new ground, the
 * program only ever asked its standard input for two things, keys and answers to
 * its colour questions, and both go through one descriptor now.
 */
static void open_the_keys(void) {
    if (isatty(STDIN_FILENO)) return;
    int fd = open("/dev/tty", O_RDWR);
    if (fd < 0) {
        fprintf(stderr,
                "%s: needs a terminal, and standard input is not one and /dev/tty cannot be "
                "opened: %s\n",
                program_name, strerror(errno));
        exit(EXIT_FAILURE);
    }
    input_fd = fd;
}

/* The file --text names, opened, or the reason it cannot be and an end to the run. */
static int open_the_text_file(void) {
    int fd = open(text_path, O_RDONLY);
    if (fd < 0) {
        fprintf(stderr, "%s: cannot open %s: %s\n", program_name, text_path, strerror(errno));
        exit(EXIT_FAILURE);
    }
    return fd;
}

/* How much of a pipe is read, and for how long. A command that prints and ends is
 * read to its end; one that never ends (tail -f) is read until it has been quiet
 * for a moment, or has gone on long enough, or has sent a megabyte, and the text is
 * what it had sent by then. One that sends nothing at all is given a few seconds,
 * and then the flock flies as it always did. */
static const letters_reading_t PIPE_READING = {3.0, 1.5, 8.0, 1 << 20};

/*
 * Takes the text from --text or from a pipe, lays it out on a screen of this many
 * cells, and makes the letters the flock. Returns whether it did: no text, empty
 * text, and text with nothing to see in it leave the flock as it is. `pipes` says
 * whether a standard input that is not a terminal counts, which a benchmark has no
 * business assuming.
 */
static int take_the_text(int cols, int rows, int pipes) {
    int fd = -1, opened = 0;
    if (text_path != NULL && strcmp(text_path, "-") != 0) {
        fd = open_the_text_file();
        opened = 1;
    } else if (text_path != NULL) {
        if (isatty(STDIN_FILENO)) {
            fprintf(stderr, "%s: --text - reads standard input, and that is a terminal\n",
                    program_name);
            exit(EXIT_USAGE);
        }
        fd = STDIN_FILENO;
    } else if (pipes && !isatty(STDIN_FILENO)) {
        fd = STDIN_FILENO;
    }
    if (fd < 0) return 0;

    vt_t vt;
    if (vt_init(&vt, cols, rows) != 0) {
        fprintf(stderr, "%s: out of memory\n", program_name);
        exit(EXIT_FAILURE);
    }
    size_t bytes = 0;
    free(the_text);
    the_text = NULL;
    letters_read_end_t end = letters_read(&vt, fd, &PIPE_READING, &bytes, &the_text);
    the_text_length = bytes;
    if (opened) close(fd);
    if (end == LETTERS_READ_ERROR && text_path != NULL) {
        fprintf(stderr, "%s: cannot read %s\n", program_name, text_path);
        exit(EXIT_FAILURE);
    }
    int count = end == LETTERS_READ_ERROR
                    ? 0
                    : letters_build(&the_letters, &vt, DEFAULT_CELL_WIDTH, DEFAULT_CELL_HEIGHT,
                                    MAX_LETTERS, random_unit);
    vt_destroy(&vt);
    if (count < 0) {
        fprintf(stderr, "%s: out of memory\n", program_name);
        exit(EXIT_FAILURE);
    }
    if (count == 0) {
        forget_the_text();
        if (text_path != NULL)
            fprintf(stderr, "%s: nothing to see in %s, so here are birds\n", program_name,
                    text_path);
        return 0;
    }

    if (fireflies_mode) {
        fprintf(stderr,
                "%s: --fireflies does not go with text on standard input: the text is the flock\n",
                program_name);
        exit(EXIT_USAGE);
    }
    if (sky_mode) {
        fprintf(stderr,
                "%s: --3d does not go with text on standard input: a text is laid out on the flat "
                "sky\n",
                program_name);
        exit(EXIT_USAGE);
    }
    if (the_sign_option() != NULL) {
        fprintf(stderr, "%s: %s does not go with text on standard input: the text is the flock\n",
                program_name, the_sign_option());
        exit(EXIT_USAGE);
    }
    letters_mode = 1;
    config.birds = count;
    /* A cell is eight pixels across, a quarter of the sprite the pace was tuned for:
     * at the default a letter crosses two cells a frame and the eye sees it skip,
     * and at the slowest it is a cell a frame, which is a flight a letter can be
     * followed in. A speed that was asked for is left alone. */
    if (config.pace_notch == DEFAULT_PACE_NOTCH) config.pace_notch = LETTERS_PACE_NOTCH;
    apply_notches();
    /* One flock, on one plane, with no tails and no rain: the text is what is
     * flying, and the things that would draw over it are for birds. */
    config.flocks = 1;
    deep_look = 0;
    config.trails = 0;
    the_rain_is_falling = 0;
    return 1;
}

static const double REFLOW_SETTLE_SECONDS = 0.3;

/* The window has a size the letters were not laid out for. The text is laid out
 * again on this one, as it would have been had the command run here, and the cycle
 * begins again from the text at rest: a letter in the air has nowhere to land on a
 * grid that is gone. Waits until the size has held still for a moment, so that
 * dragging a corner is not a layout a frame. Returns whether it laid out afresh. */
static int reflow_the_letters(bird_t **birds, bird_t **snapshot) {
    if (!letters_mode || the_text == NULL) return 0;
    if (screen.cols == the_letters.cols && screen.rows == the_letters.rows) {
        reflow_wait = 0;
        return 0;
    }
    reflow_wait += frame_seconds;
    if (reflow_wait < REFLOW_SETTLE_SECONDS) return 0;
    reflow_wait = 0;

    vt_t vt;
    if (vt_init(&vt, screen.cols, screen.rows) != 0) return 0;
    vt_feed(&vt, the_text, letters_text_end(the_text, the_text_length));
    vt_finish(&vt);
    letters_t fresh;
    int count = letters_build(&fresh, &vt, DEFAULT_CELL_WIDTH, DEFAULT_CELL_HEIGHT, MAX_LETTERS,
                              random_unit);
    vt_destroy(&vt);
    bird_t *grown = count > 0 ? malloc(sizeof(**birds) * (size_t)count) : NULL;
    bird_t *grown_snapshot = count > 0 ? malloc(sizeof(**snapshot) * (size_t)count) : NULL;
    if (count <= 0 || grown == NULL || grown_snapshot == NULL) {
        /* Nothing of it fits on the new screen, or no memory: what there is stays, and
         * the size it was laid out for is the size to compare with from now on. */
        free(grown);
        free(grown_snapshot);
        letters_destroy(&fresh);
        the_letters.cols = screen.cols;
        the_letters.rows = screen.rows;
        return 0;
    }
    free(*birds);
    free(*snapshot);
    *birds = grown;
    *snapshot = grown_snapshot;
    letters_destroy(&the_letters);
    the_letters = fresh;
    config.birds = count;
    place_the_letters(*birds);
    memcpy(*snapshot, *birds, sizeof(**birds) * (size_t)count);
    return 1;
}

static long elapsed_microseconds(const struct timespec *start, const struct timespec *end) {
    return (end->tv_sec - start->tv_sec) * 1000000L + (end->tv_nsec - start->tv_nsec) / 1000L;
}

static double elapsed_seconds(const struct timespec *start, const struct timespec *end) {
    return (double)(end->tv_sec - start->tv_sec) + (double)(end->tv_nsec - start->tv_nsec) / 1e9;
}

/* The terminal can still impose backpressure; unlocked means this program adds
 * no wait of its own. Kept as arithmetic outside the loop so both halves of the
 * switch are testable without making a test spend time asleep. */
static long frame_delay_after(long elapsed) {
    if (unlock_fps) return 0;
    long remaining = 1000000L / FRAME_RATE - elapsed;
    return remaining > 0 ? remaining : 0;
}

/*
 * The numbers, with no terminal in the way.
 *
 * What the README claims about this program should be measurable by anyone who
 * clones it, so the measurement is a flag rather than a paragraph. No terminal is
 * opened and nothing is drawn: the frame is built into the graphics buffer and
 * its length counted, which is exactly what a real frame would put on the wire.
 */
/*
 * Hundredths of a second a frame, for a rate asked for in frames a second.
 *
 * A GIF carries its delay as whole hundredths, so the only rates it has are
 * 100/1, 100/2, 100/3 and so on, and viewers clamp anything under two hundredths
 * up to a tenth of a second. Fifty is therefore the ceiling and sixty is not a
 * rate a GIF has at all: whatever is asked for is rounded to one it does.
 */
static int record_delay_for(int fps) {
    int best = 100 / MAX_RECORD_FPS;
    double error = -1;
    /* Chosen by the error in the rate, not by rounding the delay: the two are not
     * the same, because the rate is a hundred over the delay. Rounding gives 50
     * for 41 frames a second where 33 is nearer to it. */
    for (int delay = 100 / MAX_RECORD_FPS; delay <= 100; delay++) {
        double mistake = 100.0 / delay - fps;
        if (mistake < 0) mistake = -mistake;
        if (error < 0 || mistake < error) {
            error = mistake;
            best = delay;
        }
    }
    return best;
}

/*
 * Recording.
 *
 * Headless, like the benchmark, and for the same reason: the demo in the README
 * has to be regenerable by anyone who clones this, with one command, and nothing
 * about it should depend on a terminal being attached or on how fast one happens
 * to be. Every frame is simulated, every --record-every'th is composited and
 * handed to the GIF writer.
 */
/* Headless there is no terminal to ask what colours it uses, so the palette that
 * follows the terminal has nothing in it: black birds on a black ground, which is
 * what `cbirds --record flock.gif` — the example in the help — recorded. Both the
 * modes that never open a terminal fall back to the shipped ramp. */
static void settle_the_palette_without_a_terminal(void) {
    if (palette_follows_the_theme()) config.palette = FALLBACK_PALETTE;
    if (palette_is_ink()) config.palette = FALLBACK_INK;
}

/* Six seconds is a flock's clip. A text's is a cycle: four seconds at rest, a wave
 * of one, a flight of up to twenty-two, and four to come home, and the text whole
 * again for the last two. */
enum { FLOCK_RECORD_SECONDS = 6, TEXT_RECORD_SECONDS = 34 };

static void settle_the_recording_length(void) {
    if (record_seconds == 0)
        record_seconds = letters_mode ? TEXT_RECORD_SECONDS : FLOCK_RECORD_SECONDS;
}

/* JSON needs its control characters spelled out, and an escape sequence is
 * nothing but control characters and text. Everything else, braille included,
 * goes through as the UTF-8 it already is. */
static void write_json_string(FILE *out, const char *text, size_t length) {
    fputc('"', out);
    for (size_t i = 0; i < length; i++) {
        unsigned char c = (unsigned char)text[i];
        if (c == '"' || c == '\\')
            fprintf(out, "\\%c", c);
        else if (c < 0x20)
            fprintf(out, "\\u%04x", c);
        else
            fputc(c, out);
    }
    fputc('"', out);
}

/*
 * The recording as text: an asciinema cast, which is a JSON header and then
 * one line per frame of what the braille renderer would have sent a terminal.
 * It plays back in any terminal with `asciinema play`, embeds anywhere the
 * player does, and a five second flock is a few hundred kilobytes where the
 * GIF of it is five megabytes — because only the cells that changed are in it.
 */
static int run_cast_recording(void) {
    spatial_grid_t grid;
    settle_the_recording_length();
    int total = record_fps * record_seconds;
    settle_the_palette_without_a_terminal();
    set_frame_seconds(1.0 / record_fps);
    legend_enabled = 0;
    render_mode = RENDER_BRAILLE;
    apply_screen_size(record_columns, record_rows, record_columns * DEFAULT_CELL_WIDTH,
                      record_rows * DEFAULT_CELL_HEIGHT);
    settle_the_bird_size();
    if (spatial_grid_init(&grid, SPATIAL_CELL_SIZE) != SPATIAL_GRID_OK) return EXIT_FAILURE;
    if (spatial_grid_prepare(&grid, screen.width, screen.height, config.birds) != SPATIAL_GRID_OK)
        return EXIT_FAILURE;
    if (!prepare_text_renderer() || !text_renderer_fits_the_screen()) {
        fprintf(stderr, "%s: cannot build the sprites to record with\n", program_name);
        return EXIT_FAILURE;
    }
    /* A cast is played in somebody else's terminal, whose colours are unknown:
     * 24 bit colour is what every player of casts understands. */
    text_cells.truecolor = 1;

    bird_t *birds = calloc((size_t)config.birds, sizeof(*birds));
    bird_t *snapshot = malloc(sizeof(*snapshot) * (size_t)config.birds);
    FILE *out = birds != NULL && snapshot != NULL ? fopen(record_path, "w") : NULL;
    if (out == NULL) {
        if (birds != NULL && snapshot != NULL)
            fprintf(stderr, "%s: %s: %s\n", program_name, record_path, strerror(errno));
        free(birds);
        free(snapshot);
        return EXIT_FAILURE;
    }
    fprintf(out,
            "{\"version\": 2, \"width\": %d, \"height\": %d, \"timestamp\": %ld, "
            "\"title\": \"cbirds\", \"env\": {\"TERM\": \"xterm-256color\", \"SHELL\": "
            "\"/bin/sh\"}}\n",
            screen.cols, screen.rows, (long)time(NULL));
    seed_random(requested_seed >= 0 ? (unsigned)requested_seed : 1u);
    initialize_birds(birds);
    place_hawks();
    begin_the_intro();

    /* Hidden cursor and a clean slate first; the pen put back at the end. */
    static const char opening[] = "\033[?25l\033[2J";
    fprintf(out, "[0, \"o\", ");
    write_json_string(out, opening, sizeof(opening) - 1);
    fprintf(out, "]\n");

    long bytes = 0;
    for (int frame = 0; frame < total; frame++) {
        clock_state.frame = frame;
        clock_state.seconds = (double)frame / record_fps;
        if (formation.writing && formation.until >= 0 && clock_state.seconds >= formation.until)
            formation_clear();
        sign_advance(birds);
        maybe_drift();

        memcpy(snapshot, birds, sizeof(*birds) * (size_t)config.birds);
        spatial_grid_build(&grid, config.birds, read_bird_position, snapshot);
        fly(birds, snapshot, &grid);

        if (letters_mode) {
            paint_the_letters(birds);
        } else {
            compose_onto(&text_canvas, text_sprites, birds, 0);
            cells_read(&text_cells, CELLS_BRAILLE, &text_canvas, screen.cell_width,
                       screen.cell_height);
        }
        if (cells_emit(&text_cells) != CELLS_OK) break;
        /* Inside a synchronized update, for the players that honour it. */
        fprintf(out, "[%.4f, \"o\", ", clock_state.seconds);
        fputs("\"\\u001b[?2026h", out);
        for (size_t i = 0; i < text_cells.length; i++) {
            unsigned char c = (unsigned char)text_cells.text[i];
            if (c == '"' || c == '\\')
                fprintf(out, "\\%c", c);
            else if (c < 0x20)
                fprintf(out, "\\u%04x", c);
            else
                fputc(c, out);
        }
        fputs("\\u001b[?2026l\"]\n", out);
        bytes += (long)text_cells.length;
    }
    static const char closing[] = "\033[0m\033[?25h";
    fprintf(out, "[%.4f, \"o\", ", (double)total / record_fps);
    write_json_string(out, closing, sizeof(closing) - 1);
    fprintf(out, "]\n");
    int closed = fclose(out) == 0;
    formation_clear(); /* A clip shorter than the intro would leave it writing. */

    cells_destroy(&text_cells);
    png_image_free(&text_canvas);
    free_sprites(text_sprites);
    spatial_grid_destroy(&grid);
    fireflies_destroy(&night);
    letters_destroy(&the_letters);
    forget_the_text();
    end_the_sky();
    free(snapshot);
    free(birds);
    if (!closed) {
        fprintf(stderr, "%s: %s: %s\n", program_name, record_path, strerror(errno));
        return EXIT_FAILURE;
    }
    printf("%s: %d frames, %dx%d cells, %d fps, %.1fs, %.1f KB of %s\n", record_path, total,
           screen.cols, screen.rows, record_fps, (double)total / record_fps, (double)bytes / 1024.0,
           letters_mode ? "text" : "braille");
    sign_report_failure();
    return EXIT_SUCCESS;
}

/* The light of an escape wave is on no first frame, and a GIF's palette is made
 * from the first frame: so it is asked for, with its edges, which are the light
 * and the ground in quarters. Only for a clip in which a wave can happen, which
 * is one with hawks that is longer than the intro, because the letters of the
 * intro are never alarmed, and not one of text, in which nothing is: any other
 * clip keeps exactly the palette it always had. A sign is not the intro: it has
 * no time at which it lets go, and the birds round it have their waves from the
 * first moment, so a clip of one with hawks has the light however short it is. A
 * space has no waves, and keeps the colours its own caller asks for below. */
static void reserve_the_light(gif_writer_t *gif, double seconds) {
    if (config.hawks == 0 || letters_mode || sky_mode ||
        seconds <= (formation.writing ? formation.until : 0))
        return;
    const uint8_t *light = highlight_colour();
    uint8_t colours[4][3];
    for (int quarter = 0; quarter < 4; quarter++)
        for (int c = 0; c < 3; c++)
            colours[quarter][c] =
                (uint8_t)(light[c] + (picture_ground()[c] - light[c]) * quarter / 4.0 + 0.5);
    gif_reserve_colours(gif, (const uint8_t(*)[3])colours, 4);
}

/* Ascending by the bucket gif.c counts a colour in, five bits a channel. */
static int by_gif_bucket(const void *a, const void *b) {
    const uint8_t *x = a, *y = b;
    for (int c = 0; c < 3; c++)
        if ((x[c] >> 3) != (y[c] >> 3)) return (x[c] >> 3) - (y[c] >> 3);
    return 0;
}

/* The first frame of a text is the text at rest, in the colours of the command, and
 * the colours the letters fly in are on no frame until they lift: the ramp they wear
 * by heading, and the hawk's. They are asked for in the order of their buckets, so
 * that a table is the same whichever way round a ramp is written. */
static void reserve_the_flight(gif_writer_t *gif) {
    uint8_t flying[GIF_RESERVED_MAX][3];
    int colours = 0;
    for (int shade = 0; shade < palette()->shades && colours < GIF_RESERVED_MAX - 1;
         shade++, colours++)
        memcpy(flying[colours], palette()->tints[shade], 3);
    if (config.hawks > 0) memcpy(flying[colours++], hawk_colour(), 3);
    qsort(flying, (size_t)colours, sizeof(*flying), by_gif_bucket);
    gif_reserve_colours(gif, (const uint8_t(*)[3])flying, colours);
}

/* A recording's table of colours is made from its first frame, and in a space the
 * first frame does not show everything: a size of bird nobody has flown into view
 * yet, or a hawk that is still off the screen, would be drawn in the nearest colour
 * the table has. The colours that matter are the flat tints of the sprites, each
 * a handful, and they are asked for by name, in the order of their buckets like
 * the text's, so that the table does not depend on how the sets are numbered. The
 * flat flock shows everything it has from the first frame, and is left as it was. */
static void reserve_the_colours_of_the_sprites(gif_writer_t *gif, const png_image_t *frames) {
    uint8_t colours[GIF_RESERVED_MAX][3];
    int count = 0;
    for (int set = 0; set < sprite_set_count() && count < GIF_RESERVED_MAX; set++) {
        const png_image_t *image = &frames[set * ROTATION_FRAMES];
        if (image->pixels == NULL) continue;
        for (int i = 0; i < image->width * image->height; i++) {
            const uint8_t *pixel = image->pixels + (size_t)i * 4;
            if (pixel[3] != 255) continue;
            int known = 0;
            for (int c = 0; c < count; c++) known |= memcmp(colours[c], pixel, 3) == 0;
            if (!known) memcpy(colours[count++], pixel, 3);
            break;
        }
    }
    qsort(colours, (size_t)count, sizeof(*colours), by_gif_bucket);
    gif_reserve_colours(gif, (const uint8_t(*)[3])colours, count);
}

static int run_recording(void) {
    static png_image_t frames[ROTATION_FRAMES * MAX_SPRITE_SETS];
    png_image_t canvas = {0, 0, NULL};
    spatial_grid_t grid;
    gif_writer_t *gif = NULL;
    size_t bytes = 0;
    int written = 0;
    /* The name says which: a .cast is text, anything else is a GIF. */
    size_t name_length = strlen(record_path);
    if (name_length > 5 && strcmp(record_path + name_length - 5, ".cast") == 0)
        return run_cast_recording();
    /* The delay is whole hundredths, so the rate asked for is rounded to one the
     * format can carry and the rate actually achieved is reported rather than
     * claimed. Every simulated frame is recorded, and the simulation steps at the
     * recording rate, so the motion in the GIF runs at life speed. */
    settle_the_palette_without_a_terminal();
    settle_the_recording_length();
    int delay = record_delay_for(record_fps);
    /* The rate a hundredth-of-a-second delay really gives, which is not always a
     * whole number: a delay of 17 plays at 5.88 a second, not 5. */
    double actual_fps = 100.0 / delay;
    /* Counted on that rate, so --record-seconds means the seconds it lasts rather
     * than the seconds it was meant to. */
    int total = (int)(actual_fps * record_seconds + 0.5);

    /* A recording advances on its encoded clock rather than wall time, so it is
     * reproducible no matter how long a frame takes to produce. */
    set_frame_seconds(1.0 / actual_fps);

    /* A GIF has no panel in it: the panel is terminal text, and compose() draws
     * birds. Left enabled it would still reserve its corner and keep the flock
     * out of it, and every recording would have an unexplained empty rectangle in
     * the top left. */
    legend_enabled = 0;
    apply_screen_size(record_columns, record_rows, record_columns * DEFAULT_CELL_WIDTH,
                      record_rows * DEFAULT_CELL_HEIGHT);
    if (spatial_grid_init(&grid, SPATIAL_CELL_SIZE) != SPATIAL_GRID_OK) return EXIT_FAILURE;
    if (spatial_grid_prepare(&grid, screen.width, screen.height, config.birds) != SPATIAL_GRID_OK)
        return EXIT_FAILURE;
    if (!letters_mode && rasterise_sprites(frames) != PNG_OK) {
        fprintf(stderr, "%s: cannot build the sprites to record with\n", program_name);
        return EXIT_FAILURE;
    }
    if (png_image_alloc(&canvas, screen.width, screen.height) != PNG_OK) return EXIT_FAILURE;
    /* Under --render braille, sextants or blocks the GIF is of the cells, painted
     * the way a text terminal shows them, because a GIF of the pixels the cells
     * were read from would be a picture of something nobody saw. Text is always
     * that: it is drawn with the font, in cells of a size the font reads at. */
    int as_text = letters_mode || drawing_with_text();
    png_image_t painted = {0, 0, NULL};
    int picture_cell_width = letters_mode ? LETTER_PICTURE_WIDTH : screen.cell_width;
    int picture_cell_height = letters_mode ? LETTER_PICTURE_HEIGHT : screen.cell_height;
    if (as_text && (cells_init(&text_cells, 1) != CELLS_OK ||
                    cells_resize(&text_cells, screen.cols, screen.rows) != CELLS_OK))
        return EXIT_FAILURE;

    gif_status_t gif_status =
        gif_open(&gif, record_path, as_text ? screen.cols * picture_cell_width : screen.width,
                 as_text ? screen.rows * picture_cell_height : screen.height, delay);
    if (gif_status != GIF_OK) {
        fprintf(stderr, "%s: %s: %s\n", program_name, record_path, gif_status_string(gif_status));
        return EXIT_FAILURE;
    }
    if (sky_mode) reserve_the_colours_of_the_sprites(gif, frames);

    if (letters_mode) reserve_the_flight(gif);

    bird_t *birds = calloc((size_t)config.birds, sizeof(*birds));
    bird_t *snapshot = malloc(sizeof(*snapshot) * (size_t)config.birds);
    if (birds == NULL || snapshot == NULL) return EXIT_FAILURE;
    seed_random(requested_seed >= 0 ? (unsigned)requested_seed : 1u);
    initialize_birds(birds);
    place_hawks();
    begin_the_intro();
    reserve_the_light(gif, (double)total / actual_fps);

    for (int frame = 0; frame < total && gif_status == GIF_OK; frame++) {
        /* The clock the features read has to advance, or nothing that animates
         * on its own terms would animate at all. */
        clock_state.frame = frame;
        clock_state.seconds = (double)frame / actual_fps;
        if (formation.writing && formation.until >= 0 && clock_state.seconds >= formation.until)
            formation_clear();
        sign_advance(birds);
        maybe_drift();

        memcpy(snapshot, birds, sizeof(*birds) * (size_t)config.birds);
        spatial_grid_build(&grid, config.birds, read_bird_position, snapshot);
        fly(birds, snapshot, &grid);

        if (as_text) {
            if (letters_mode) {
                paint_the_letters(birds);
            } else {
                compose_onto(&canvas, frames, birds, 0);
                cells_read(&text_cells, text_style(), &canvas, screen.cell_width,
                           screen.cell_height);
            }
            png_image_free(&painted);
            if (cells_emit(&text_cells) != CELLS_OK ||
                cells_paint(&text_cells, letters_mode ? CELLS_TEXT : text_style(), &painted,
                            picture_cell_width, picture_cell_height,
                            picture_ground()) != CELLS_OK) {
                gif_status = GIF_ERR_MEMORY;
                break;
            }
            gif_status = gif_add_frame(gif, &painted);
        } else {
            compose_onto(&canvas, frames, birds, 1);
            gif_status = gif_add_frame(gif, &canvas);
        }
    }
    formation_clear();

    gif_status_t closed = gif_close(gif, &bytes, &written);
    if (gif_status == GIF_OK) gif_status = closed;
    free_sprites(frames);
    png_image_free(&canvas);
    png_image_free(&painted);
    if (as_text) cells_destroy(&text_cells);
    spatial_grid_destroy(&grid);
    fireflies_destroy(&night);
    letters_destroy(&the_letters);
    forget_the_text();
    end_the_sky();
    free(snapshot);
    free(birds);

    if (gif_status != GIF_OK) {
        fprintf(stderr, "%s: %s: %s\n", program_name, record_path, gif_status_string(gif_status));
        return EXIT_FAILURE;
    }
    printf("%s: %d frames, %dx%d, %.4g fps, %.1fs, %.1f KB\n", record_path, written,
           as_text ? screen.cols * picture_cell_width : screen.width,
           as_text ? screen.rows * picture_cell_height : screen.height, actual_fps,
           written / actual_fps, (double)bytes / 1024.0);
    if (delay != record_delay_for(record_fps) || (int)(actual_fps + 0.5) != record_fps)
        fprintf(stderr,
                "%s: asked for %d fps, recorded at %.4g. A GIF's delay between frames is\n"
                "whole hundredths of a second, so the only rates it has are 100/1, 100/2,\n"
                "100/3 and so on, and viewers clamp anything under two hundredths up to a\n"
                "tenth. %.4g is the nearest rate this format can actually carry.\n",
                program_name, record_fps, actual_fps, actual_fps);
    sign_report_failure();
    return EXIT_SUCCESS;
}

static int run_benchmark(void) {
    kitty_graphics_t graphics;
    spatial_grid_t grid;
    struct timespec start, finish;

    settle_the_palette_without_a_terminal();
    apply_screen_size(200, 50, 1600, 800);
    /* There is no terminal, so unasked means the sprites, as in a recording; any
     * other renderer is measured as asked for, sprites built the way it builds
     * them. */
    if (render_mode == RENDER_UNSET) render_mode = RENDER_KITTY;
    if (letters_mode) render_mode = RENDER_BRAILLE; /* Letters are text. */
    settle_the_bird_size();
    if (drawing_with_text() && !prepare_text_renderer()) return EXIT_FAILURE;
    set_frame_seconds(1.0 / FRAME_RATE);
    if (spatial_grid_init(&grid, SPATIAL_CELL_SIZE) != SPATIAL_GRID_OK) return EXIT_FAILURE;
    if (spatial_grid_prepare(&grid, screen.width, screen.height, config.birds) != SPATIAL_GRID_OK)
        return EXIT_FAILURE;
    if (kitty_graphics_init(&graphics, STDOUT_FILENO) != KITTY_GRAPHICS_OK) return EXIT_FAILURE;

    bird_t *birds = calloc((size_t)config.birds, sizeof(*birds));
    bird_t *snapshot = malloc(sizeof(*snapshot) * (size_t)config.birds);
    if (birds == NULL || snapshot == NULL) return EXIT_FAILURE;
    seed_random(requested_seed >= 0 ? (unsigned)requested_seed : 1u);
    initialize_birds(birds);
    place_hawks();
    /* Text at rest costs nothing, which is not what anybody wants to know: the wave
     * is started at once, and the frames are those of a flock in the air. */
    if (letters_mode) letters_poke(&the_letters);
    /* The default bench has never written anything, and still does not; a sign
     * is measured with the clock it would have at sixty frames a second. */
    if (a_sign_is_asked_for()) begin_the_intro();

    double bytes = 0;
    clock_gettime(CLOCK_MONOTONIC, &start);
    for (int frame = 0; frame < bench_frames; frame++) {
        if (a_sign_is_asked_for()) {
            clock_state.seconds = (double)frame / FRAME_RATE;
            sign_advance(birds);
        }
        memcpy(snapshot, birds, sizeof(*birds) * (size_t)config.birds);
        spatial_grid_build(&grid, config.birds, read_bird_position, snapshot);
        hunt(snapshot);
        graphics.length = 0;
        render_frame(&graphics, birds, snapshot, &grid);
        bytes += (double)graphics.length;
    }
    clock_gettime(CLOCK_MONOTONIC, &finish);

    double seconds =
        (double)(finish.tv_sec - start.tv_sec) + (double)(finish.tv_nsec - start.tv_nsec) / 1e9;
    double per_frame = seconds / bench_frames;
    printf("birds        %d\n", config.birds);
    if (letters_mode)
        printf("letters      %d of %d glyphs\n", the_letters.count, the_letters.cells_with_glyphs);
    printf("flocks       %d\n", config.flocks);
    printf("hawks        %d\n", config.hawks);
    printf("viewport     %dx%d px\n", screen.width, screen.height);
    printf("render       %s\n", RENDER_NAMES[render_mode]);
    printf("frames       %d\n", bench_frames);
    printf("frame time   %.3f ms\n", per_frame * 1000.0);
    printf("ceiling      %.0f fps\n", 1.0 / per_frame);
    printf("bytes/frame  %.0f (%.1f KB)\n", bytes / bench_frames, bytes / bench_frames / 1024.0);
    printf("at %d fps    %.1f MB/s\n", FRAME_RATE, bytes / bench_frames * FRAME_RATE / 1e6);
    if (fireflies_mode) printf("sync         %.2f\n", fireflies_order(&night));
    sign_report_failure();

    kitty_graphics_destroy(&graphics);
    spatial_grid_destroy(&grid);
    if (drawing_with_text()) {
        cells_destroy(&text_cells);
        png_image_free(&text_canvas);
        free_sprites(text_sprites);
    }
    fireflies_destroy(&night);
    letters_destroy(&the_letters);
    forget_the_text();
    end_the_sky();
    free(snapshot);
    free(birds);
    return EXIT_SUCCESS;
}

int main(int argc, char **argv) {
    kitty_graphics_t graphics;
    spatial_grid_t grid;
    struct timespec frame_start, frame_end, launched;
    clock_gettime(CLOCK_MONOTONIC, &launched);
    read_options(argc, argv);
    trig_lookup_init();
    if (bench_frames > 0) {
        /* A benchmark reads a file it is named, never a pipe it happens to be in. */
        take_the_text(200, 50, 0);
        return run_benchmark();
    }
    if (record_path != NULL) {
        take_the_text(record_columns, record_rows, 1);
        return run_recording();
    }
    install_signal_handlers();
    atexit(restore_terminal);

    /* The keys, and then the text, before the terminal is taken: the text is laid
     * out on a screen of the size this one is, and reading it may take a moment. */
    /* A file that is not there is said first: it is the mistake, and the keys are not. */
    if (text_path != NULL && strcmp(text_path, "-") != 0) close(open_the_text_file());
    open_the_keys();
    update_screen_dimensions();
    take_the_text(screen.cols, screen.rows, 1);

    /* The terminal is asked its questions before anything is built for it: can
     * you draw this at all, and what colours do you use? The sprites are then
     * built once, in the answers. */
    if (enter_terminal() < 0) {
        perror("Can't enable raw mode");
        exit(EXIT_FAILURE);
    }
    /* Braille in every terminal, and the sprites when --render kitty asks: there
     * is no terminal this refuses to run in, and none it has to guess about. */
    render_mode = live_render_mode();
    if (palette_follows_the_theme() && !learn_the_theme()) config.palette = FALLBACK_PALETTE;
    if (palette_is_ink() && !learn_the_ink(sky_mode ? SKY_DIM : FAR_DIM))
        config.palette = FALLBACK_INK;

    spatial_grid_status_t grid_status = spatial_grid_init(&grid, SPATIAL_CELL_SIZE);
    if (grid_status != SPATIAL_GRID_OK) {
        fprintf(stderr, "Cannot initialize spatial grid: %s\n",
                spatial_grid_status_string(grid_status));
        exit(EXIT_FAILURE);
    }
    /* A named seed makes a run repeatable, which is what lets a look be shared
     * and a bug report be reproduced. */
    seed_random(requested_seed >= 0 ? (unsigned)requested_seed : (unsigned)time(NULL));
    update_screen_dimensions();
    settle_the_bird_size(); /* From the screen, if it is a sign that is being sized. */
    grid_status = spatial_grid_prepare(&grid, screen.width, screen.height, config.birds);
    if (grid_status != SPATIAL_GRID_OK) {
        fprintf(stderr, "Cannot prepare spatial grid: %s\n",
                spatial_grid_status_string(grid_status));
        exit(EXIT_FAILURE);
    }
    /* The sprites, once, as pixels: the text renderers read them back as cells
     * every frame, and Kitty is sent them encoded and then places them by id. */
    if (drawing_with_text()) {
        if (!prepare_text_renderer()) {
            fprintf(stderr, "%s: cannot build the sprites to draw with\n", program_name);
            exit(EXIT_FAILURE);
        }
    } else if (rasterise_sprites(text_sprites) != PNG_OK) {
        fprintf(stderr, "%s: cannot build the sprites to draw with\n", program_name);
        exit(EXIT_FAILURE);
    }
    set_frame_seconds(1.0 / FRAME_RATE);
    hawk_sets_built = 1;

    bird_t *birds = calloc((size_t)config.birds, sizeof(*birds));
    bird_t *snapshot = malloc(sizeof(*snapshot) * (size_t)config.birds);
    if (!birds || !snapshot) {
        perror("Out of memory");
        exit(EXIT_FAILURE);
    }
    kitty_graphics_status_t graphics_status = kitty_graphics_init(&graphics, STDOUT_FILENO);
    if (graphics_status != KITTY_GRAPHICS_OK) {
        fprintf(stderr, "Cannot initialize Kitty graphics: %s\n",
                kitty_graphics_status_string(graphics_status));
        exit(EXIT_FAILURE);
    }

    enter_alt_screen();
    write_all("\x1b[J", sizeof("\x1b[J") - 1);
    update_screen_dimensions();
    initialize_birds(birds);
    place_hawks();
    begin_the_intro();
    if (render_mode == RENDER_KITTY) {
        sprites_uploaded = 1; /* Even a failed upload may have left some behind. */
        graphics_status = upload_sprite_sets(&graphics, text_sprites);
        free_sprites(text_sprites);
        if (graphics_status != KITTY_GRAPHICS_OK) {
            fprintf(stderr, "Cannot upload Kitty graphics: %s\n",
                    kitty_graphics_status_string(graphics_status));
            exit(EXIT_FAILURE);
        }
    }

    struct timespec started;
    clock_gettime(CLOCK_MONOTONIC, &started);
    launch_lag = elapsed_seconds(&launched, &started);
    struct timespec previous_frame = started;
    int live_birds = config.birds;
    int running = 1;
    double leaving = 0; /* Seconds left of the flight out. */
    while (running) {
        if (!handle_input() && leaving <= 0) {
            /* A screensaver is gone the moment it is asked, with no flight out: the
             * person at the keyboard is waiting for the shell. */
            if (screensaver_mode) break;
            /* Asked to quit: fly off the top first, so the last thing seen is
             * the flock leaving rather than the screen blinking out. */
            leaving = (double)OUTRO_FRAMES_AT_SIXTY / FRAME_RATE;
            formation_clear();
        }

        clock_gettime(CLOCK_MONOTONIC, &frame_start);
        set_frame_seconds(elapsed_seconds(&previous_frame, &frame_start));
        previous_frame = frame_start;
        if (leaving > 0) {
            leaving -= frame_seconds;
            if (leaving <= 0) break;
        }
        clock_state.frame++;
        clock_state.seconds = (double)(frame_start.tv_sec - started.tv_sec) +
                              (double)(frame_start.tv_nsec - started.tv_nsec) / 1e9;
        /* Writing lets go when its hold is up, and the flock takes over again
         * from wherever the letters left it, which is the nicest part to watch. */
        if (formation.writing && formation.until >= 0 && clock_state.seconds >= formation.until)
            formation_clear();
        maybe_drift();
        update_screen_dimensions();
        if (reflow_the_letters(&birds, &snapshot)) live_birds = config.birds;
        sprite_fit_t fit = fit_the_sprites_to_the_window(&graphics);
        if (fit == SPRITES_FAILED) {
            fprintf(stderr, "%s: cannot build the sprites to draw with\n", program_name);
            exit(EXIT_FAILURE);
        }
        /* The pause was building, not flying: the next frame is one frame long. */
        if (fit == SPRITES_REBUILT) clock_gettime(CLOCK_MONOTONIC, &previous_frame);
        grid_status = spatial_grid_prepare(&grid, screen.width, screen.height, config.birds);
        if (grid_status != SPATIAL_GRID_OK) {
            fprintf(stderr, "Cannot resize spatial grid: %s\n",
                    spatial_grid_status_string(grid_status));
            exit(EXIT_FAILURE);
        }
        if (population_changed) {
            /* Grown or shrunk by a keypress, and the grid needs room for them. */
            population_changed = 0;
            if (resize_the_flock(&birds, &snapshot, live_birds, config.birds))
                live_birds = config.birds;
            else
                config.birds = live_birds; /* Keep what we have rather than lose it. */
            grid_status = spatial_grid_prepare(&grid, screen.width, screen.height, config.birds);
            if (grid_status != SPATIAL_GRID_OK) {
                fprintf(stderr, "Cannot resize spatial grid: %s\n",
                        spatial_grid_status_string(grid_status));
                exit(EXIT_FAILURE);
            }
        }
        /* After the flock has the size it is to have, and after the screen is
         * measured: a sign is laid out for both, and reads every bird of it. */
        if (leaving <= 0) sign_advance(birds);
        memcpy(snapshot, birds, sizeof(*birds) * (size_t)config.birds);
        grid_status = spatial_grid_build(&grid, config.birds, read_bird_position, snapshot);
        if (grid_status != SPATIAL_GRID_OK) {
            fprintf(stderr, "Cannot build spatial grid: %s\n",
                    spatial_grid_status_string(grid_status));
            exit(EXIT_FAILURE);
        }
        if (leaving > 0) fly_away(birds);
        graphics_status = leaving > 0 ? queue_render_frame(&graphics, birds)
                                      : render_frame(&graphics, birds, snapshot, &grid);
        if (graphics_status != KITTY_GRAPHICS_OK) {
            fprintf(stderr, "Cannot render Kitty graphics: %s\n",
                    kitty_graphics_status_string(graphics_status));
            exit(EXIT_FAILURE);
        }
        size_t frame_bytes = graphics.length;
        while (running && graphics.length > 0) {
            graphics_status = kitty_graphics_flush_nonblocking(&graphics);
            if (graphics_status == KITTY_GRAPHICS_AGAIN) {
                if (wait_for_terminal_io() < 0) {
                    perror("Cannot wait for terminal output");
                    exit(EXIT_FAILURE);
                }
                running = handle_input();
                continue;
            }
            if (graphics_status != KITTY_GRAPHICS_OK) {
                /* Whatever the renderer: the text ones go through this buffer
                 * too, and a reader that went away is the usual reason. */
                if (graphics_status == KITTY_GRAPHICS_ERR_IO)
                    fprintf(stderr, "%s: cannot write to the terminal: %s\n", program_name,
                            strerror(errno));
                else
                    fprintf(stderr, "%s: cannot write to the terminal: %s\n", program_name,
                            kitty_graphics_status_string(graphics_status));
                exit(EXIT_FAILURE);
            }
        }
        if (!running) break;
        if (frame_limit > 0 && clock_state.frame >= frame_limit) break;
        clock_gettime(CLOCK_MONOTONIC, &frame_end);
        /* Averaged over a second, because a number that changes sixty times a
         * second is decoration rather than information. */
        stats.window_ms += (double)elapsed_microseconds(&frame_start, &frame_end) / 1000.0;
        stats.window_bytes += (double)frame_bytes;
        stats.counted++;
        if (clock_state.seconds - stats.window_started >= 1.0) {
            double span = clock_state.seconds - stats.window_started;
            stats.frame_ms = stats.window_ms / (double)stats.counted;
            stats.bytes = stats.window_bytes / (double)stats.counted;
            stats.rate = (double)stats.counted / span;
            stats.window_started = clock_state.seconds;
            stats.window_ms = stats.window_bytes = 0;
            stats.counted = 0;
        }
        long remaining = frame_delay_after(elapsed_microseconds(&frame_start, &frame_end));
        if (remaining > 0) {
            struct timespec delay = {remaining / 1000000L, (remaining % 1000000L) * 1000L};
            nanosleep(&delay, NULL);
        }
    }
    /* The terminal is back before anything is said to the person at it. */
    if (the_sign.failures > 0) {
        restore_terminal();
        sign_report_failure();
    }
    /* A snapshot asked for and not written is a failed run, so a script that
     * takes one can tell. */
    int outcome = EXIT_SUCCESS;
    if (snapshot_path != NULL) {
        if (write_snapshot(snapshot_path, birds)) {
            fprintf(stderr, "%s: wrote %s\n", program_name, snapshot_path);
        } else {
            fprintf(stderr, "%s: could not write %s\n", program_name, snapshot_path);
            outcome = EXIT_FAILURE;
        }
    }
    spatial_grid_destroy(&grid);
    kitty_graphics_destroy(&graphics);
    fireflies_destroy(&night);
    letters_destroy(&the_letters);
    forget_the_text();
    end_the_sky();
    free(snapshot);
    free(birds);
    return outcome;
}
