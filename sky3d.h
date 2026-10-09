/*
 * A murmuration in three dimensions.
 *
 * The flat flock steers on x, y and a heading. This one flies in a space: a
 * position and a heading in three dimensions, the same three rules, and a bird
 * heeds its seven or so nearest neighbours however far off they are, rather than
 * everything within a radius, because that is what starlings were measured doing
 * (Ballerini et al., PNAS 2008) and it is what lets a flock stay one body while
 * its density rises and falls. Turning is limited per second, so a bird banks.
 * There are no walls: a roost pulls a bird home when it strays, sideways towards
 * the roost and up or down towards a preferred height, as Hildenbrandt, Carere
 * and Hemelrijk modelled it (Behavioral Ecology 2010), and the air over it moves.
 *
 * Nothing here knows about pixels, sprites or the terminal. It knows birds in a
 * world measured in metres, a camera that orbits the roost, and how to say where
 * on a screen a bird lands and which way it points there. boids.c does the rest.
 */

#ifndef SKY3D_H
#define SKY3D_H

#include <stddef.h>
#include <stdint.h>

enum {
    SKY_MAX_NEIGHBOURS = 16,
    SKY_MAX_HAWKS = 4,
    /* How many sizes a bird is drawn at. Five is as many as the depth of a flock
     * can be told apart by, and each is a set of sprites to build and upload. */
    SKY_BINS = 5
};

typedef enum { SKY_OK = 0, SKY_ERR_ARGUMENT, SKY_ERR_MEMORY } sky_status_t;

typedef struct {
    double x, y, z; /* Metres; z is up and the roost is the origin. */
    double yaw, pitch;
    double hx, hy, hz; /* The same heading as a unit vector, kept beside the angles. */
    double speed;      /* Metres a second. */
    double roll;       /* Banking, radians: how far it leans into its turn. */
    double wander[3];
} sky_bird_t;

/* A hawk is no bird of the flock: it has no neighbours, follows none of the three
 * rules, and the flock only ever feels it as something to get away from. */
typedef struct {
    double x, y, z;
    double yaw, pitch;
    double hx, hy, hz;
    double speed;
    int prey;          /* The bird it is after, negative for none. */
    double commitment; /* Seconds before it may change its mind. */
    double passing;    /* Seconds left of a straight run out of the flock. */
    int diving;        /* Beating its wings, in the stoop or the run out of it. */
} sky_hawk_t;

/* A ray from the camera through the pointer, and how far from it a bird feels
 * it: a stick poked into the sky. */
typedef struct {
    int active;
    double origin[3], direction[3];
    double reach;
} sky_poke_t;

/* Everything the flight depends on, in metres and seconds, so that the caller's
 * sliders are weights here and nothing else. */
typedef struct {
    int neighbours; /* Nearest birds heeded, however far. */
    double reach;   /* And the farthest any of them may be. */
    double separation, alignment, cohesion;
    double roost;     /* The pull home. */
    double current;   /* How hard the air pushes. */
    double wander;    /* Each bird's own restlessness. */
    double turn_rate; /* Radians a second at the yaw; the pitch gets a part of it. */
    double cruise;
    double poke_weight;
    double hawk_weight; /* How hard a bird flees a hawk. */
} sky_rules_t;

typedef struct {
    double origin[3];
    double cell;
    int dims[3];
    int cells;
    int capacity; /* Birds the index can hold. */
    int *start;   /* cells + 1 offsets into item. */
    int *item;    /* Bird indices, grouped by cell. */
    int *cell_of;
    /* The positions again, in the order of item, so that a search reads a run of
     * neighbouring cells as one run of memory instead of one jump a bird. */
    double *at_x, *at_y, *at_z;
    size_t cells_allocated;
} sky_grid_t;

typedef struct {
    sky_bird_t *birds, *next;
    int capacity;
    sky_grid_t grid;
    uint64_t random;
    double clock; /* Seconds flown, for whatever drifts. */
    /* Where each wave of the air is in its cycle at the start, which is what a seed
     * changes about the flock's shape: the same air for every seed folded every flock
     * the same way. */
    double air_phase[5];
    /* Where the flock is and how big, which is what the camera frames: its middle
     * as it is, and its size eased over seconds, because a flock that breathes
     * should not make the picture lurch. */
    double flock_centre[3], flock_radius;
    int framed;
    /* The edge of an index cell: about as long as a bird's seventh neighbour is
     * far, which the step re-measures, so the cell a bird is in holds a few birds
     * and the search reads a few dozen. */
    double cell;
    sky_hawk_t hawks[SKY_MAX_HAWKS];
    int hawk_count;
} sky_t;

sky_status_t sky_init(sky_t *sky, int capacity, uint32_t seed);
void sky_destroy(sky_t *sky);
/* Grows the room for birds, keeping those already flying. */
sky_status_t sky_reserve(sky_t *sky, int capacity);

sky_rules_t sky_default_rules(void);

/* Births for birds[first .. first + count): the first ones as a flock over the
 * roost, flying about together; later ones, on a + key, among birds already in
 * the air, where they join. */
void sky_populate(sky_t *sky, int first, int count, int already_flying);

/* The index every neighbour search reads, rebuilt from the birds' positions. It
 * is a uniform grid in three dimensions over the flock's own bounding box, so a
 * flock that drifts costs nothing and one that explodes cannot ask for more cells
 * than the cap. */
sky_status_t sky_index(sky_t *sky, int count);

/* The nearest birds to birds[self], nearest first, ties broken by index, none
 * further than reach. Exact: the same list a search through every bird makes.
 * Returns how many it found, at most k. Needs sky_index to have run. */
int sky_neighbours(const sky_t *sky, int self, int k, double reach, int *index, double *squared);

/* How many hawks hunt the flock. New ones come in from the edge of the roost, and
 * the ones already hunting are left where they are. */
void sky_set_hawks(sky_t *sky, int count);

/* One step of flight for the first count birds, and for the hawks after them. */
void sky_step(sky_t *sky, int count, const sky_rules_t *rules, const sky_poke_t *poke,
              double seconds);

/* The flock's middle and the root mean square of its distances from it. */
void sky_measure(const sky_t *sky, int count, double centre[3], double *radius);

/* The orbit, and the perspective. */
typedef struct {
    double azimuth, elevation, distance;
    double target[3];
    double focal; /* Pixels from the eye to the picture plane. */
    double centre_x, centre_y;
    double eye[3], right[3], up[3], forward[3];
} sky_camera_t;

/* How big the picture is for the purposes of framing, in pixels: the flock is
 * framed to fill it, and so the sprites are sized by it. */
double sky_picture(int width, int height);

/* Where the camera is `seconds` into the show, for a picture of this size: on its
 * circle round the roost, looking at it. */
void sky_camera_orbit(sky_camera_t *camera, double seconds, int width, int height);
/* The same camera, turned to look partway towards the flock and moved in or out
 * to frame its size: only where it looks and how far off it is change. */
void sky_camera_frame(sky_camera_t *camera, const sky_t *sky);
/* Recomputes the eye and the axes from the angles. */
void sky_camera_aim(sky_camera_t *camera);
/* Returns 0 for a point behind the camera. */
int sky_camera_project(const sky_camera_t *camera, double x, double y, double z, double *sx,
                       double *sy, double *depth);
/* The ray through a pixel. */
void sky_camera_ray(const sky_camera_t *camera, double sx, double sy, double origin[3],
                    double direction[3]);

/* What a bird looks like from the camera. */
typedef struct {
    float x, y;   /* The pixel its middle lands on. */
    float scale;  /* One at the middle of the flock's depth, more nearer. */
    float angle;  /* Radians on the screen, y down, the way its body points. */
    float along;  /* Of its length, how much the screen shows, 0 to 1. */
    float across; /* Of its wings' span, the same. */
    int bin;      /* 0 is the farthest size, SKY_BINS - 1 the nearest. */
    int visible;
} sky_view_t;

/* The scale a bin stands for, which is what its sprites are drawn at. */
double sky_bin_scale(int bin);
int sky_bin_for(double scale);
void sky_view(const sky_t *sky, int count, const sky_camera_t *camera, sky_view_t *views);
/* The same for a hawk: where it is, which way it points, and how far off. */
void sky_hawk_view(const sky_t *sky, int hawk, const sky_camera_t *camera, sky_view_t *view);

const char *sky_status_string(sky_status_t status);

#endif
