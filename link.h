/*
 * One sky, shared by several terminals.
 *
 * Each `cbirds --link` binds a Unix datagram socket in a directory of the
 * user's own and finds the other windows by listing it. Windows lie in a row,
 * left to right, in the order they joined: a window's neighbours are the one
 * before it and the one after it. A bird that flies out of a window through an
 * edge that has a neighbour behind it is posted to that neighbour as a
 * traveller, and flies in through the facing edge.
 *
 * Nothing here knows what a bird is. A traveller is a handful of numbers that
 * the program fills in and reads back, and every call returns at once: the
 * sockets are non blocking, and a datagram the other side did not take is a bird
 * the sender keeps.
 */

#ifndef LINK_H
#define LINK_H

#include <stddef.h>
#include <stdint.h>
#include <sys/stat.h>
#include <sys/types.h>

enum {
    LINK_LEFT = 0,
    LINK_RIGHT = 1,
    /* Travellers posted in one datagram. Linux queues ten datagrams a socket by
     * default, whatever their size, so a flock crossing at forty birds a frame
     * needs to travel in batches: one bird a datagram was refused at the tenth. */
    LINK_BATCH = 8,
    /* Always this long, whatever it carries: a header and LINK_BATCH slots. */
    LINK_MESSAGE_SIZE = 24 + 48 * LINK_BATCH,
    LINK_PATH_SIZE = 128,
    LINK_NAME_SIZE = 32,
    /* Entries that were sent to and refused for good, and are let be until the
     * directory changes. Past this the oldest is forgotten, and tried again. */
    LINK_REFUSED_MAX = 16
};

typedef enum { LINK_BIRD = 1, LINK_HAWK = 2 } link_kind_t;

/* A bird or a hawk in the post. The numbers are the sender's own, as they were
 * when it left; the receiver decides what they mean in its window. */
typedef struct {
    int kind;         /* LINK_BIRD or LINK_HAWK. */
    int enters;       /* The receiver's edge it comes in by: LINK_LEFT or LINK_RIGHT. */
    double height;    /* Where it left, as a share of the window's height, 0 to 1. */
    double reach;     /* How far past the edge its middle had gone, in pixels. */
    double direction; /* Radians. */
    int flock, shade, layer, wing; /* Small numbers, 0 to 15, whose meaning is the program's. */
    double wing_clock;             /* 0 to 1. */
    double holding;                /* Seconds left of whatever it was in the middle of, 0 to 60. */
} link_traveller_t;

/* Who sent a message: the order in which they joined, and the process. */
typedef struct {
    uint64_t joined;
    uint32_t pid;
} link_sender_t;

/* One datagram, decoded. A status says whether its sender has room for another
 * bird, and for another hawk; anything else carries up to LINK_BATCH travellers. */
typedef struct {
    int status;
    link_sender_t from;
    int birds_ok, hawks_ok;
    int count;
    link_traveller_t travellers[LINK_BATCH];
} link_message_t;

typedef struct {
    int present;
    link_sender_t who;
    char name[LINK_NAME_SIZE];
    int birds_ok, hawks_ok; /* As it last said. */
    double heard;           /* When it last said so; negative before it has. */
} link_neighbour_t;

typedef struct {
    int opened;
    int fd;
    char directory[LINK_PATH_SIZE];
    char name[LINK_NAME_SIZE];
    char path[LINK_PATH_SIZE];
    link_sender_t me;
    link_neighbour_t next[2]; /* Indexed by LINK_LEFT and LINK_RIGHT. */
    int birds_ok, hawks_ok;   /* What this window tells its neighbours. */
    double now, scanned;
    int scan_wanted;
    /* Names of entries that cannot be sent to, and how the directory stood when the
     * last of them was refused: while it stands so, they are not neighbours. */
    char refused[LINK_REFUSED_MAX][LINK_NAME_SIZE];
    int refused_count;
    long long refused_seconds;
    long refused_nanoseconds;
    link_traveller_t waiting[LINK_BATCH]; /* The rest of a batch being handed out. */
    int waiting_count, waiting_at;
} link_t;

typedef enum {
    LINK_OK = 0,
    LINK_ERR_ARGUMENT,
    LINK_ERR_PATH_TOO_LONG,
    LINK_ERR_DIRECTORY, /* mkdir or lstat failed; errno says how. */
    LINK_ERR_NOT_A_DIRECTORY,
    LINK_ERR_NOT_YOURS,
    LINK_ERR_NOT_PRIVATE,
    LINK_ERR_SOCKET /* socket or bind failed; errno says how. */
} link_status_t;

/*
 * Where the sky lives: $XDG_RUNTIME_DIR/cbirds when that is set to an absolute
 * path, else $TMPDIR/cbirds-UID, else /tmp/cbirds-UID. The environment is passed
 * in, so that the choice can be tested. Returns 0 when it does not fit.
 */
int link_directory_for(char *out, size_t size, const char *runtime, const char *temporary,
                       uid_t user);

/* A directory is fit to hold the sky when it is a directory (a link to one is
 * not), belongs to the user, and nobody else can write in it. */
link_status_t link_directory_check(const struct stat *info, uid_t user);

/*
 * Joins the sky kept in `directory`, making it if need be. The socket is named
 * for the moment of joining and the process, so that listing the directory gives
 * the order; sockets of processes that are gone are removed on the way. `now` is
 * the caller's clock in seconds, which every later call is given again. On
 * failure nothing is left behind.
 */
link_status_t link_open(link_t *link, const char *directory, double now);

/* Leaves: removes the socket and closes it. Safe to call twice, and from a
 * signal handler, because it does nothing but unlink and close. */
void link_close(link_t *link);

/* Once a frame. Lists the directory and tells the neighbours how this window
 * stands about once a second, and sooner when something has changed. */
void link_update(link_t *link, double now);

/* What this window can take, told to the neighbours when it changes. */
void link_set_room(link_t *link, int birds_ok, int hawks_ok);

/* Whether there is a neighbour on that side. */
int link_has_neighbour(const link_t *link, int side);

/* Whether a bird (or a hawk) sent that way would be taken: there is a neighbour
 * that has been heard from lately and has said it has room. */
int link_edge_open(const link_t *link, int side, int kind);

/*
 * Posts travellers to the neighbour on `side`, as many as it will take, and
 * returns how many went. The rest are the caller's still: a neighbour that is
 * full, or gone, or has no room is not an error, and nothing is lost.
 */
int link_send(link_t *link, int side, const link_traveller_t *travellers, int count);

/* The next traveller that has arrived, or 0 when there is none. Statuses are
 * taken in on the way; malformed and foreign datagrams are thrown away. */
int link_receive(link_t *link, link_traveller_t *traveller);

/* The wire format, exposed for the tests. */
void link_encode_travellers(uint8_t *out, const link_sender_t *from,
                            const link_traveller_t *travellers, int count);
void link_encode_status(uint8_t *out, const link_sender_t *from, int birds_ok, int hawks_ok);
/* 1 when the bytes are one of ours and every field is in range. */
int link_decode(const uint8_t *bytes, size_t length, link_message_t *message);

const char *link_status_string(link_status_t status);

#endif
