/* Feature test macros must precede every include. */
#define _XOPEN_SOURCE 700
#define _DEFAULT_SOURCE
#define _DARWIN_C_SOURCE

#include "link.h"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>

/* The doubles travel as their own eight bytes, which is only a format when a
 * double is that. Every system this builds on has it so; this says so out loud
 * rather than finding out on a machine that has not. */
typedef char double_is_eight_bytes[sizeof(double) == 8 ? 1 : -1];

enum {
    VERSION = 1,
    TYPE_TRAVELLERS = 1,
    TYPE_STATUS = 2,
    HEADER_SIZE = 24,
    SLOT_SIZE = 48,
    /* A small number is a flock, a shade, a layer or a wing: the program knows
     * what they mean and none of them is anywhere near this. */
    SMALL_MAX = 15,
    /* More datagrams than this that carry no bird, in one call, and the rest waits
     * for the next frame: whoever is flooding the socket, with garbage or with
     * statuses, does not get to stall the flock. */
    NO_BIRD_LIMIT = 4096
};
static const uint8_t MAGIC[4] = {'C', 'B', 'L', 'K'};
static const double TWO_PI = 6.283185307179586;
static const double REACH_MAX = 10000.0;
static const double HOLDING_MAX = 60.0;

/* How often a window lists the directory and says how it stands. Once a second
 * is plenty for a window to appear or go, and lets a lost status be made good by
 * the next without anybody noticing. */
static const double SCAN_SECONDS = 1.0;
/* A neighbour not heard from in this long is not trusted with a bird: it may be
 * stopped, or stalled on its terminal, and a bird posted to it would wait in a
 * queue nobody is reading. Three heartbeats, so that one lost status is not
 * enough to close a door. */
static const double STALE_SECONDS = 3.0;
/* Somebody new is heard from, and the directory is read at once; but not more
 * often than this, so a flood of foreign datagrams cannot become a flood of
 * directory reads. */
static const double HURRY_SECONDS = 0.05;

/* ------------------------------------------------------------------------ */
/* The wire format. All integers big endian, whatever the machine, so that the  */
/* bytes mean one thing; the doubles are IEEE 754 as they come.                 */
/*                                                                              */
/*   0  4  "CBLK"                                                               */
/*   4  1  version                                                              */
/*   5  1  type: 1 travellers, 2 status                                         */
/*   6  1  count of travellers, zero for a status                               */
/*   7  1  flags: the edge the travellers come in by, or the status bits        */
/*   8  8  joined, nanoseconds                                                  */
/*  16  4  pid                                                                  */
/*  20  4  zero                                                                 */
/*  24  .  LINK_BATCH slots of 48 bytes; those beyond count are zero            */
/*                                                                              */
/* A slot: kind, layer, flock, shade, wing (a byte each), three zero bytes, and */
/* five doubles: height, reach, direction, wing_clock, holding.                 */
/* ------------------------------------------------------------------------ */

static void put32(uint8_t *at, uint32_t value) {
    at[0] = (uint8_t)(value >> 24);
    at[1] = (uint8_t)(value >> 16);
    at[2] = (uint8_t)(value >> 8);
    at[3] = (uint8_t)value;
}

static void put64(uint8_t *at, uint64_t value) {
    put32(at, (uint32_t)(value >> 32));
    put32(at + 4, (uint32_t)value);
}

static uint32_t get32(const uint8_t *at) {
    return (uint32_t)at[0] << 24 | (uint32_t)at[1] << 16 | (uint32_t)at[2] << 8 | at[3];
}

static uint64_t get64(const uint8_t *at) {
    return (uint64_t)get32(at) << 32 | get32(at + 4);
}

static void put_double(uint8_t *at, double value) {
    uint64_t bits;
    memcpy(&bits, &value, sizeof(bits));
    put64(at, bits);
}

static double get_double(const uint8_t *at) {
    uint64_t bits = get64(at);
    double value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

static void put_header(uint8_t *out, const link_sender_t *from, int type, int count, int flags) {
    memset(out, 0, LINK_MESSAGE_SIZE);
    memcpy(out, MAGIC, sizeof(MAGIC));
    out[4] = VERSION;
    out[5] = (uint8_t)type;
    out[6] = (uint8_t)count;
    out[7] = (uint8_t)flags;
    put64(out + 8, from->joined);
    put32(out + 16, from->pid);
}

void link_encode_travellers(uint8_t *out, const link_sender_t *from,
                            const link_traveller_t *travellers, int count) {
    if (count < 0) count = 0;
    if (count > LINK_BATCH) count = LINK_BATCH;
    put_header(out, from, TYPE_TRAVELLERS, count, count > 0 ? travellers[0].enters : 0);
    for (int i = 0; i < count; i++) {
        uint8_t *slot = out + HEADER_SIZE + SLOT_SIZE * i;
        const link_traveller_t *t = &travellers[i];
        slot[0] = (uint8_t)t->kind;
        slot[1] = (uint8_t)t->layer;
        slot[2] = (uint8_t)t->flock;
        slot[3] = (uint8_t)t->shade;
        slot[4] = (uint8_t)t->wing;
        put_double(slot + 8, t->height);
        put_double(slot + 16, t->reach);
        put_double(slot + 24, t->direction);
        put_double(slot + 32, t->wing_clock);
        put_double(slot + 40, t->holding);
    }
}

void link_encode_status(uint8_t *out, const link_sender_t *from, int birds_ok, int hawks_ok) {
    put_header(out, from, TYPE_STATUS, 0, (birds_ok ? 1 : 0) | (hawks_ok ? 2 : 0));
}

static int all_zero(const uint8_t *bytes, size_t length) {
    for (size_t i = 0; i < length; i++)
        if (bytes[i] != 0) return 0;
    return 1;
}

static int within(double value, double low, double high) {
    return isfinite(value) && value >= low && value <= high;
}

static int decode_slot(const uint8_t *slot, int enters, link_traveller_t *t) {
    if (slot[0] != LINK_BIRD && slot[0] != LINK_HAWK) return 0;
    if (slot[1] > SMALL_MAX || slot[2] > SMALL_MAX || slot[3] > SMALL_MAX || slot[4] > SMALL_MAX)
        return 0;
    if (!all_zero(slot + 5, 3)) return 0;
    t->kind = slot[0];
    t->enters = enters;
    t->layer = slot[1];
    t->flock = slot[2];
    t->shade = slot[3];
    t->wing = slot[4];
    t->height = get_double(slot + 8);
    t->reach = get_double(slot + 16);
    t->direction = get_double(slot + 24);
    t->wing_clock = get_double(slot + 32);
    t->holding = get_double(slot + 40);
    return within(t->height, 0, 1) && within(t->reach, 0, REACH_MAX) &&
           within(t->direction, 0, TWO_PI) && within(t->wing_clock, 0, 1) &&
           within(t->holding, 0, HOLDING_MAX);
}

int link_decode(const uint8_t *bytes, size_t length, link_message_t *message) {
    if (bytes == NULL || message == NULL || length != LINK_MESSAGE_SIZE) return 0;
    if (memcmp(bytes, MAGIC, sizeof(MAGIC)) != 0 || bytes[4] != VERSION) return 0;
    if (!all_zero(bytes + 20, 4)) return 0;

    memset(message, 0, sizeof(*message));
    message->from.joined = get64(bytes + 8);
    message->from.pid = get32(bytes + 16);
    if (message->from.pid == 0) return 0;
    int count = bytes[6], flags = bytes[7];
    if (count > LINK_BATCH) return 0;
    /* Whatever is not used is zero, so a datagram that is only partly ours, or
     * is somebody else's with a few bytes that happen to match, does not pass. */
    if (!all_zero(bytes + HEADER_SIZE + SLOT_SIZE * count,
                  (size_t)(LINK_MESSAGE_SIZE - HEADER_SIZE - SLOT_SIZE * count)))
        return 0;

    if (bytes[5] == TYPE_STATUS) {
        if (count != 0 || flags > 3) return 0;
        message->status = 1;
        message->birds_ok = flags & 1;
        message->hawks_ok = (flags & 2) != 0;
        return 1;
    }
    if (bytes[5] != TYPE_TRAVELLERS || count < 1 || count > LINK_BATCH || flags > 1) return 0;
    for (int i = 0; i < count; i++)
        if (!decode_slot(bytes + HEADER_SIZE + SLOT_SIZE * i, flags, &message->travellers[i]))
            return 0;
    message->count = count;
    return 1;
}

/* ------------------------------------------------------------------------ */
/* Where the sky lives                                                         */
/* ------------------------------------------------------------------------ */

/* Without the trailing slashes, which would make "//" of the join. */
static size_t trimmed_length(const char *path) {
    size_t length = strlen(path);
    while (length > 0 && path[length - 1] == '/') length--;
    return length;
}

int link_directory_for(char *out, size_t size, const char *runtime, const char *temporary,
                       uid_t user) {
    int written;
    if (out == NULL || size == 0) return 0;
    /* The runtime directory is the right place, and private by construction, but
     * only when it is given as a path that means the same to every process. */
    if (runtime != NULL && runtime[0] == '/') {
        written = snprintf(out, size, "%.*s/cbirds", (int)trimmed_length(runtime), runtime);
    } else {
        if (temporary == NULL || temporary[0] != '/') temporary = "/tmp";
        written = snprintf(out, size, "%.*s/cbirds-%lu", (int)trimmed_length(temporary), temporary,
                           (unsigned long)user);
    }
    return written > 0 && (size_t)written < size;
}

link_status_t link_directory_check(const struct stat *info, uid_t user) {
    if (!S_ISDIR(info->st_mode)) return LINK_ERR_NOT_A_DIRECTORY;
    if (info->st_uid != user) return LINK_ERR_NOT_YOURS;
    if ((info->st_mode & (S_IWGRP | S_IWOTH)) != 0) return LINK_ERR_NOT_PRIVATE;
    return LINK_OK;
}

/* ------------------------------------------------------------------------ */
/* Names, and who is there                                                     */
/* ------------------------------------------------------------------------ */

static void format_name(char *out, size_t size, const link_sender_t *who) {
    snprintf(out, size, "%016llx-%lu", (unsigned long long)who->joined, (unsigned long)who->pid);
}

/* Sixteen lower case hex digits, a dash and a pid: and only exactly what
 * format_name writes, so that nothing else in the directory is ever taken for a
 * window, let alone removed as one. */
static int parse_name(const char *name, link_sender_t *who) {
    uint64_t joined = 0;
    for (int i = 0; i < 16; i++) {
        char c = name[i];
        int digit = c >= '0' && c <= '9' ? c - '0' : c >= 'a' && c <= 'f' ? c - 'a' + 10 : -1;
        if (digit < 0) return 0;
        joined = joined << 4 | (uint64_t)digit;
    }
    if (name[16] != '-') return 0;
    unsigned long pid = 0;
    int digits = 0;
    for (const char *p = name + 17; *p != '\0'; p++) {
        if (*p < '0' || *p > '9' || ++digits > 10) return 0;
        pid = pid * 10 + (unsigned long)(*p - '0');
    }
    if (digits == 0 || pid == 0 || pid > 2147483647UL) return 0;
    who->joined = joined;
    who->pid = (uint32_t)pid;
    char again[LINK_NAME_SIZE];
    format_name(again, sizeof(again), who);
    return strcmp(again, name) == 0;
}

typedef struct {
    char name[LINK_NAME_SIZE];
    link_sender_t who;
} window_t;

static int path_for(const link_t *link, const char *name, char *out, size_t size) {
    int written = snprintf(out, size, "%s/%s", link->directory, name);
    return written > 0 && (size_t)written < size;
}

/* How the directory stands: when anything last came into it or went out of it. */
static int stamp_of_the_directory(const link_t *link, long long *seconds, long *nanoseconds) {
    struct stat info;
    if (lstat(link->directory, &info) < 0) return 0;
#if defined(__APPLE__)
    *seconds = info.st_mtimespec.tv_sec;
    *nanoseconds = info.st_mtimespec.tv_nsec;
#else
    *seconds = info.st_mtim.tv_sec;
    *nanoseconds = info.st_mtim.tv_nsec;
#endif
    return 1;
}

static int is_refused(const link_t *link, const char *name) {
    for (int i = 0; i < link->refused_count; i++)
        if (strcmp(link->refused[i], name) == 0) return 1;
    return 0;
}

/* The refusals are good for the directory as it was. Something has come or gone
 * since: an entry may be another one under the same name, and is looked at again. */
static void forget_refusals_if_the_directory_changed(link_t *link) {
    long long seconds;
    long nanoseconds;
    if (link->refused_count == 0) return;
    if (!stamp_of_the_directory(link, &seconds, &nanoseconds) || seconds != link->refused_seconds ||
        nanoseconds != link->refused_nanoseconds)
        link->refused_count = 0;
}

/* An entry that was sent to and refused for a reason that will not mend (it is no
 * socket of ours, it is gone) is let be, and not made a neighbour again at the next
 * look and the one after, announcing to it and giving it up every time. */
static void refuse(link_t *link, const char *name) {
    long long seconds;
    long nanoseconds;
    if (!stamp_of_the_directory(link, &seconds, &nanoseconds)) return;
    forget_refusals_if_the_directory_changed(link);
    if (is_refused(link, name)) return;
    if (link->refused_count == LINK_REFUSED_MAX) {
        memmove(link->refused[0], link->refused[1],
                sizeof(link->refused[0]) * (LINK_REFUSED_MAX - 1));
        link->refused_count--;
    }
    strcpy(link->refused[link->refused_count++], name);
    link->refused_seconds = seconds;
    link->refused_nanoseconds = nanoseconds;
}

/* What a window needs of the directory, and no more: the windows beside it, and
 * when the last one joined. There is no list to cut short, so there is nothing a
 * crowd of entries can push a window off. */
typedef struct {
    window_t left, right;
    int has_left, has_right;
    uint64_t latest; /* When the latest of them joined; zero if there are none. */
} sky_t;

/* The windows that are there, in the order they joined. A socket whose process
 * is gone is a leftover of one that was killed, and is removed; one whose pid is
 * alive is taken at its word, and found out by the first datagram it refuses.
 * Only a socket can be a window, so a link, a directory or a file by a window's
 * name is not one; and an entry that was refused is not, until the directory has
 * changed. `mine` is this window's name, or NULL before it has one. */
static void read_the_sky(link_t *link, const char *mine, sky_t *sky) {
    DIR *directory = opendir(link->directory);
    memset(sky, 0, sizeof(*sky));
    if (directory == NULL) return;
    forget_refusals_if_the_directory_changed(link);
    for (struct dirent *entry; (entry = readdir(directory)) != NULL;) {
        window_t window;
        char path[LINK_PATH_SIZE];
        struct stat info;
        if (strlen(entry->d_name) >= sizeof(window.name)) continue;
        if (!parse_name(entry->d_name, &window.who)) continue;
        strcpy(window.name, entry->d_name);
        if (!path_for(link, window.name, path, sizeof(path))) continue;
        /* A socket first, and then whether its process is there: a file, a link or a
         * pipe that happens to have a window's name is nobody's leftover, and is let
         * be whatever its name says. */
        if (lstat(path, &info) < 0 || !S_ISSOCK(info.st_mode)) continue;
        if (kill((pid_t)window.who.pid, 0) < 0 && errno == ESRCH) {
            unlink(path);
            continue;
        }
        if (window.who.joined > sky->latest) sky->latest = window.who.joined;
        if (mine == NULL || strcmp(window.name, mine) == 0 || is_refused(link, window.name))
            continue;
        if (strcmp(window.name, mine) < 0) {
            if (!sky->has_left || strcmp(window.name, sky->left.name) > 0) sky->left = window;
            sky->has_left = 1;
        } else {
            if (!sky->has_right || strcmp(window.name, sky->right.name) < 0) sky->right = window;
            sky->has_right = 1;
        }
    }
    closedir(directory);
}

static int same_sender(const link_sender_t *a, const link_sender_t *b) {
    return a->joined == b->joined && a->pid == b->pid;
}

static void forget(link_neighbour_t *neighbour) {
    memset(neighbour, 0, sizeof(*neighbour));
    neighbour->heard = -1;
}

static void adopt(link_neighbour_t *neighbour, const window_t *window) {
    if (window == NULL) {
        if (neighbour->present) forget(neighbour);
        return;
    }
    /* The same window as before keeps what it has said; a different one has said
     * nothing yet, and gets no bird until it does. */
    if (neighbour->present && same_sender(&neighbour->who, &window->who)) return;
    forget(neighbour);
    neighbour->present = 1;
    neighbour->who = window->who;
    strcpy(neighbour->name, window->name);
}

/* ------------------------------------------------------------------------ */
/* The socket                                                                  */
/* ------------------------------------------------------------------------ */

static int address_of(const char *path, struct sockaddr_un *address, socklen_t *length) {
    size_t size = strlen(path);
    if (size >= sizeof(address->sun_path)) return 0;
    memset(address, 0, sizeof(*address));
    address->sun_family = AF_UNIX;
    memcpy(address->sun_path, path, size + 1);
    *length = (socklen_t)(offsetof(struct sockaddr_un, sun_path) + size + 1);
#if defined(__APPLE__) || defined(__FreeBSD__) || defined(__NetBSD__) || defined(__OpenBSD__)
    address->sun_len = (unsigned char)*length;
#endif
    return 1;
}

static int configure(int fd) {
    int flags = fcntl(fd, F_GETFL);
    if (flags < 0 || fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) return 0;
    flags = fcntl(fd, F_GETFD);
    if (flags < 0 || fcntl(fd, F_SETFD, flags | FD_CLOEXEC) < 0) return 0;
#ifdef SO_NOSIGPIPE
    int on = 1;
    setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &on, sizeof(on));
#endif
    /* A larger queue where the system lets us have one, by way of the bytes. It
     * does not lift the count of datagrams Linux allows, which is why they are
     * posted in batches, but macOS counts bytes and starts at four thousand, ten
     * datagrams. Eighty, a few frames of a busy door from both sides, and no more:
     * a window that has stopped is posted to until it falls silent, and what is
     * in its queue when it is killed is lost with it. */
    int bytes = 80 * LINK_MESSAGE_SIZE;
    setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &bytes, sizeof(bytes));
    return 1;
}

static link_status_t bind_the_socket(link_t *link) {
    struct sockaddr_un address;
    socklen_t length;
    if (!address_of(link->path, &address, &length)) return LINK_ERR_PATH_TOO_LONG;
    int fd = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (fd < 0) return LINK_ERR_SOCKET;
    if (!configure(fd)) {
        int saved = errno;
        close(fd);
        errno = saved;
        return LINK_ERR_SOCKET;
    }
    /* Owner only, whatever the directory around it allows. */
    mode_t before = umask(0177);
    int bound = bind(fd, (struct sockaddr *)&address, length);
    if (bound < 0 && errno == EADDRINUSE) {
        /* A leftover with exactly our name: the same pid and the same
         * nanosecond, which is a file nobody is using. */
        unlink(link->path);
        bound = bind(fd, (struct sockaddr *)&address, length);
    }
    int saved = errno;
    umask(before);
    if (bound < 0) {
        close(fd);
        errno = saved;
        return LINK_ERR_SOCKET;
    }
    link->fd = fd;
    return LINK_OK;
}

/* One datagram to a neighbour. Zero when it went, else why not. */
static int post(link_t *link, int side, const uint8_t *bytes) {
    char path[LINK_PATH_SIZE];
    struct sockaddr_un address;
    socklen_t length;
    int flags = 0;
#ifdef MSG_NOSIGNAL
    flags = MSG_NOSIGNAL;
#endif
    if (!path_for(link, link->next[side].name, path, sizeof(path)) ||
        !address_of(path, &address, &length))
        return ENAMETOOLONG;
    for (;;) {
        ssize_t sent =
            sendto(link->fd, bytes, LINK_MESSAGE_SIZE, flags, (struct sockaddr *)&address, length);
        if (sent == LINK_MESSAGE_SIZE) return 0;
        if (sent < 0 && errno == EINTR) continue;
        return sent < 0 ? errno : EMSGSIZE;
    }
}

/* Not a failure of the neighbour: it is there and busy, and the bird waits. */
static int is_busy(int error) {
    return error == EAGAIN || error == EWOULDBLOCK || error == ENOBUFS || error == EINTR;
}

/* A neighbour that refused, or is not there. A socket file with nobody behind it
 * is what a killed window leaves, and is swept away; one that has gone cleanly
 * has swept its own. Either way the arrangement is read again at once. */
static void neighbour_is_gone(link_t *link, int side, int error) {
    char path[LINK_PATH_SIZE];
    if (error == ECONNREFUSED && path_for(link, link->next[side].name, path, sizeof(path)))
        unlink(path);
    refuse(link, link->next[side].name);
    forget(&link->next[side]);
    link->scan_wanted = 1;
}

static void announce(link_t *link) {
    uint8_t bytes[LINK_MESSAGE_SIZE];
    link_encode_status(bytes, &link->me, link->birds_ok, link->hawks_ok);
    for (int side = 0; side < 2; side++) {
        if (!link->next[side].present) continue;
        int error = post(link, side, bytes);
        if (error != 0 && !is_busy(error)) neighbour_is_gone(link, side, error);
    }
}

/* The directory, made if it is not there, and fit to hold the sky: a directory that
 * is ours and that nobody else can write in. Made with the owner's rights whatever
 * the umask, which may take away the owner's own: a directory the window cannot
 * write in would be left there for every later run to fail on. */
static link_status_t sky_status(const char *directory) {
    if (mkdir(directory, 0700) == 0) {
        if (chmod(directory, 0700) < 0) return LINK_ERR_DIRECTORY;
    } else if (errno != EEXIST) {
        return LINK_ERR_DIRECTORY;
    }
    struct stat info;
    if (lstat(directory, &info) < 0) return LINK_ERR_DIRECTORY;
    return link_directory_check(&info, geteuid());
}

static int make_the_sky(const char *directory) {
    return sky_status(directory) == LINK_OK;
}

/* Reads the directory and works out who is on either side. If this window's own
 * socket has been taken out from under it (a tidy up of the temporary
 * directory, a careless rm) it is made again, under the same name, so that it
 * keeps its place. It looks for its own file by name, and not for its name in what
 * was listed: a listing is no proof that it is not there. */
static void scan(link_t *link) {
    sky_t sky;
    struct stat info;
    /* The directory as it is now, and not as it was when the window joined: taken
     * away, it is made again and the window joins it again; made into something
     * that will not do, a link or another's, the window is alone, and neither lists
     * nor binds nor sweeps anything through it. */
    link->scanned = link->now;
    link->scan_wanted = 0;
    int fit = make_the_sky(link->directory);
    /* Its own socket, made again if it has been taken away, and then the windows
     * beside it, in the same look: it is back in its place at once. */
    if (fit && (lstat(link->path, &info) < 0 || !S_ISSOCK(info.st_mode))) {
        int old = link->fd;
        fit = bind_the_socket(link) == LINK_OK;
        if (fit && old >= 0) close(old);
    }
    if (!fit) {
        adopt(&link->next[LINK_LEFT], NULL);
        adopt(&link->next[LINK_RIGHT], NULL);
        return;
    }
    read_the_sky(link, link->name, &sky);
    adopt(&link->next[LINK_LEFT], sky.has_left ? &sky.left : NULL);
    adopt(&link->next[LINK_RIGHT], sky.has_right ? &sky.right : NULL);
}

static uint64_t nanoseconds_now(void) {
    struct timespec now;
    clock_gettime(CLOCK_REALTIME, &now);
    return (uint64_t)now.tv_sec * 1000000000ULL + (uint64_t)now.tv_nsec;
}

link_status_t link_open(link_t *link, const char *directory, double now) {
    if (link == NULL || directory == NULL || directory[0] == '\0') return LINK_ERR_ARGUMENT;
    memset(link, 0, sizeof(*link));
    link->fd = -1;
    forget(&link->next[LINK_LEFT]);
    forget(&link->next[LINK_RIGHT]);
    link->me.pid = (uint32_t)getpid();
    link->now = link->scanned = now;

    /* Before anything is made: a path that will not fit a socket is refused
     * having touched nothing. The pid's own digits are counted, not the longest
     * a pid could have. */
    char digits[16];
    int pid_length = snprintf(digits, sizeof(digits), "%lu", (unsigned long)link->me.pid);
    struct sockaddr_un probe;
    if (strlen(directory) + 1 + 16 + 1 + (size_t)pid_length >= sizeof(probe.sun_path) ||
        strlen(directory) >= sizeof(link->directory))
        return LINK_ERR_PATH_TOO_LONG;
    strcpy(link->directory, directory);

    link_status_t status = sky_status(directory);
    if (status != LINK_OK) return status;

    /* After the last to join, even if the clock has been set back since. */
    sky_t others;
    read_the_sky(link, NULL, &others);
    uint64_t joined = nanoseconds_now();
    if (others.latest >= joined) joined = others.latest + 1;
    link->me.joined = joined;
    format_name(link->name, sizeof(link->name), &link->me);
    if (!path_for(link, link->name, link->path, sizeof(link->path))) return LINK_ERR_PATH_TOO_LONG;

    status = bind_the_socket(link);
    if (status != LINK_OK) return status;
    link->opened = 1;
    scan(link);
    return LINK_OK;
}

void link_close(link_t *link) {
    if (link == NULL || !link->opened) return;
    /* The file first and the flag last, so that a signal arriving between the
     * two still finds the window open and finishes the job. */
    unlink(link->path);
    if (link->fd >= 0) close(link->fd);
    link->fd = -1;
    link->opened = 0;
}

void link_update(link_t *link, double now) {
    if (!link->opened) return;
    link->now = now;
    double since = now - link->scanned;
    /* A scan that is wanted, because somebody new has written or a neighbour has
     * refused, is hurried, but no more often than HURRY_SECONDS whatever asks for
     * it: a flood of strangers is not a listing of the directory a frame. */
    if (since < SCAN_SECONDS && !(link->scan_wanted && since >= HURRY_SECONDS)) return;
    scan(link);
    announce(link);
}

void link_set_room(link_t *link, int birds_ok, int hawks_ok) {
    birds_ok = birds_ok != 0;
    hawks_ok = hawks_ok != 0;
    if (!link->opened) return;
    if (birds_ok == link->birds_ok && hawks_ok == link->hawks_ok) return;
    link->birds_ok = birds_ok;
    link->hawks_ok = hawks_ok;
    announce(link);
}

int link_has_neighbour(const link_t *link, int side) {
    return link->opened && (side == LINK_LEFT || side == LINK_RIGHT) && link->next[side].present;
}

int link_edge_open(const link_t *link, int side, int kind) {
    if (!link_has_neighbour(link, side)) return 0;
    const link_neighbour_t *neighbour = &link->next[side];
    if (neighbour->heard < 0 || link->now - neighbour->heard > STALE_SECONDS) return 0;
    return kind == LINK_HAWK ? neighbour->hawks_ok : neighbour->birds_ok;
}

int link_send(link_t *link, int side, const link_traveller_t *travellers, int count) {
    int sent = 0;
    if (!link->opened || (side != LINK_LEFT && side != LINK_RIGHT)) return 0;
    while (sent < count) {
        link_traveller_t batch[LINK_BATCH];
        uint8_t bytes[LINK_MESSAGE_SIZE];
        int size = 0;
        /* A traveller comes in by the edge facing the one it left by. */
        while (size < LINK_BATCH && sent + size < count &&
               link_edge_open(link, side, travellers[sent + size].kind)) {
            batch[size] = travellers[sent + size];
            batch[size].enters = side == LINK_RIGHT ? LINK_LEFT : LINK_RIGHT;
            size++;
        }
        if (size == 0) break;
        link_encode_travellers(bytes, &link->me, batch, size);
        int error = post(link, side, bytes);
        if (error != 0) {
            if (!is_busy(error)) neighbour_is_gone(link, side, error);
            break;
        }
        sent += size;
    }
    return sent;
}

/* Somebody we do not know has written: a window has joined, or one has gone and
 * another taken its place. Look again now, not at the next tick. */
static void notice(link_t *link, const link_sender_t *from) {
    for (int side = 0; side < 2; side++)
        if (link->next[side].present && same_sender(&link->next[side].who, from)) return;
    if (link->now - link->scanned >= HURRY_SECONDS) {
        scan(link);
        announce(link);
    } else {
        link->scan_wanted = 1;
    }
}

static void take_status(link_t *link, const link_message_t *message) {
    for (int side = 0; side < 2; side++) {
        link_neighbour_t *neighbour = &link->next[side];
        if (!neighbour->present || !same_sender(&neighbour->who, &message->from)) continue;
        neighbour->birds_ok = message->birds_ok;
        neighbour->hawks_ok = message->hawks_ok;
        neighbour->heard = link->now;
    }
}

int link_receive(link_t *link, link_traveller_t *traveller) {
    int without_a_bird = 0;
    if (!link->opened) return 0;
    for (;;) {
        if (link->waiting_at < link->waiting_count) {
            *traveller = link->waiting[link->waiting_at++];
            return 1;
        }
        uint8_t bytes[LINK_MESSAGE_SIZE + 1];
        ssize_t length = recv(link->fd, bytes, sizeof(bytes), 0);
        if (length < 0) {
            if (errno == EINTR) continue;
            return 0;
        }
        link_message_t message;
        if (!link_decode(bytes, (size_t)length, &message) ||
            same_sender(&message.from, &link->me)) {
            if (++without_a_bird >= NO_BIRD_LIMIT) return 0;
            continue;
        }
        notice(link, &message.from);
        if (message.status) {
            take_status(link, &message);
            if (++without_a_bird >= NO_BIRD_LIMIT) return 0;
            continue;
        }
        memcpy(link->waiting, message.travellers,
               sizeof(message.travellers[0]) * (size_t)message.count);
        link->waiting_count = message.count;
        link->waiting_at = 0;
    }
}

const char *link_status_string(link_status_t status) {
    switch (status) {
        case LINK_OK:
            return "ok";
        case LINK_ERR_ARGUMENT:
            return "invalid argument";
        case LINK_ERR_PATH_TOO_LONG:
            return "is too long for a socket path";
        case LINK_ERR_DIRECTORY:
            return "cannot be made or read";
        case LINK_ERR_NOT_A_DIRECTORY:
            return "is not a directory";
        case LINK_ERR_NOT_YOURS:
            return "does not belong to you";
        case LINK_ERR_NOT_PRIVATE:
            return "can be written to by others";
        case LINK_ERR_SOCKET:
            return "cannot hold a socket";
    }
    return "unknown error";
}
