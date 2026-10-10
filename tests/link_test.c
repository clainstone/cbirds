/* Feature test macros must precede every include. */
#define _XOPEN_SOURCE 700
#define _DEFAULT_SOURCE
#define _DARWIN_C_SOURCE

#include "../link.h"

#include <assert.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <unistd.h>

/* A directory of the run's own, and the sky inside it: the test never goes near
 * the real one, and a fixed name in /tmp would collide with a second run. It is
 * under /tmp whatever $TMPDIR says, because a socket's path may be 104 bytes at
 * most on macOS, and $TMPDIR there is fifty of them before the test has named
 * anything. */
static char scratch[64];
static char sky[100];

static void make_scratch(void) {
    snprintf(scratch, sizeof(scratch), "/tmp/cbl.XXXXXX");
    assert(mkdtemp(scratch) != NULL);
    snprintf(sky, sizeof(sky), "%s/sky", scratch);
}

static int count_entries(const char *directory) {
    DIR *dir = opendir(directory);
    int count = 0;
    if (dir == NULL) return -1;
    for (struct dirent *entry; (entry = readdir(dir)) != NULL;)
        if (strcmp(entry->d_name, ".") != 0 && strcmp(entry->d_name, "..") != 0) count++;
    closedir(dir);
    return count;
}

/* Everything in the sky and then the sky itself, so that the next test starts
 * with none; the files that are not sockets go too, which only a test makes. */
static void clear_the_sky(void) {
    DIR *dir = opendir(sky);
    if (dir != NULL) {
        for (struct dirent *entry; (entry = readdir(dir)) != NULL;) {
            char path[512];
            if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) continue;
            snprintf(path, sizeof(path), "%.200s/%.250s", sky, entry->d_name);
            unlink(path);
        }
        closedir(dir);
    }
    rmdir(sky);
}

static link_traveller_t example(int kind, int salt) {
    link_traveller_t t = {
        .kind = kind,
        .enters = salt % 2,
        .height = 0.1 + 0.01 * (salt % 50),
        .reach = 3.5 + salt,
        .direction = 0.7 + 0.1 * (salt % 50),
        .flock = salt % 3,
        .shade = (salt + 1) % 5,
        .layer = salt % 2,
        .wing = (salt + 2) % 4,
        .wing_clock = 0.25 + 0.01 * (salt % 50),
        .holding = 0.4 + 0.1 * (salt % 50),
    };
    return t;
}

static void assert_same(const link_traveller_t *a, const link_traveller_t *b) {
    assert(a->kind == b->kind && a->enters == b->enters);
    assert(a->flock == b->flock && a->shade == b->shade);
    assert(a->layer == b->layer && a->wing == b->wing);
    /* Exactly, not nearly: the doubles travel as their own bits. */
    assert(a->height == b->height && a->reach == b->reach);
    assert(a->direction == b->direction && a->wing_clock == b->wing_clock);
    assert(a->holding == b->holding);
}

/* Reads whatever is waiting, travellers included, and says how many there were. */
static int drain(link_t *window) {
    link_traveller_t t;
    int count = 0;
    while (link_receive(window, &t)) count++;
    return count;
}

/* The time every test's windows share, moved on by a little over a heartbeat
 * each time so that every window lists the directory and speaks. */
static double the_time;

static void settle(link_t **windows, int count) {
    the_time += 1.1;
    for (int pass = 0; pass < 3; pass++) {
        for (int i = 0; i < count; i++) {
            link_update(windows[i], the_time);
            link_set_room(windows[i], 1, 1);
        }
        for (int i = 0; i < count; i++) drain(windows[i]);
    }
}

static void test_the_wire_format_round_trips(void) {
    uint8_t bytes[LINK_MESSAGE_SIZE];
    link_message_t message;
    link_sender_t from = {0x0123456789abcdefULL, 4242};
    link_traveller_t sent[LINK_BATCH];

    for (int count = 1; count <= LINK_BATCH; count++) {
        for (int i = 0; i < count; i++) sent[i] = example(i % 2 ? LINK_HAWK : LINK_BIRD, i + count);
        for (int i = 0; i < count; i++) sent[i].enters = count % 2;
        link_encode_travellers(bytes, &from, sent, count);
        assert(link_decode(bytes, sizeof(bytes), &message));
        assert(!message.status && message.count == count);
        assert(message.from.joined == from.joined && message.from.pid == from.pid);
        for (int i = 0; i < count; i++) assert_same(&sent[i], &message.travellers[i]);
    }

    for (int bits = 0; bits < 4; bits++) {
        link_encode_status(bytes, &from, bits & 1, (bits & 2) != 0);
        assert(link_decode(bytes, sizeof(bytes), &message));
        assert(message.status && message.count == 0);
        assert(message.birds_ok == (bits & 1) && message.hawks_ok == ((bits & 2) != 0));
        assert(message.from.pid == 4242);
    }

    /* The edges of every range are still inside it. */
    link_traveller_t edge = example(LINK_BIRD, 0);
    edge.height = 1;
    edge.reach = 0;
    edge.direction = 0;
    edge.wing_clock = 1;
    edge.holding = 0;
    edge.flock = edge.shade = edge.layer = edge.wing = 15;
    link_encode_travellers(bytes, &from, &edge, 1);
    assert(link_decode(bytes, sizeof(bytes), &message));
    assert_same(&edge, &message.travellers[0]);
}

/* One valid datagram, spoiled in one place at a time. */
static void test_malformed_datagrams_are_not_ours(void) {
    uint8_t good[LINK_MESSAGE_SIZE], bad[LINK_MESSAGE_SIZE + 1];
    link_message_t message;
    link_sender_t from = {77, 4242};
    link_traveller_t t = example(LINK_BIRD, 3);

    link_encode_travellers(good, &from, &t, 1);
    assert(link_decode(good, sizeof(good), &message));

    /* The wrong size, in both directions. */
    assert(!link_decode(good, 0, &message));
    assert(!link_decode(good, 1, &message));
    assert(!link_decode(good, sizeof(good) - 1, &message));
    memcpy(bad, good, sizeof(good));
    bad[sizeof(good)] = 0;
    assert(!link_decode(bad, sizeof(good) + 1, &message));
    assert(!link_decode(NULL, sizeof(good), &message));

    /* The magic, the version, the type, the count and the flags, spoiled one at
     * a time. */
    for (int at = 0; at < 8; at++) {
        memcpy(bad, good, sizeof(good));
        bad[at] ^= 0x5a;
        assert(!link_decode(bad, sizeof(good), &message));
    }
    memcpy(bad, good, sizeof(good));
    bad[4] = 2; /* A version that does not exist yet. */
    assert(!link_decode(bad, sizeof(good), &message));
    memcpy(bad, good, sizeof(good));
    bad[5] = 9; /* A type that does not exist. */
    assert(!link_decode(bad, sizeof(good), &message));
    memcpy(bad, good, sizeof(good));
    bad[6] = 0; /* No travellers: nothing was sent. */
    assert(!link_decode(bad, sizeof(good), &message));
    for (int count = LINK_BATCH + 1; count < 256; count++) {
        memcpy(bad, good, sizeof(good));
        bad[6] = (uint8_t)count;
        assert(!link_decode(bad, sizeof(good), &message));
    }
    memcpy(bad, good, sizeof(good));
    bad[6] = 2; /* A second traveller that is all zeros is no traveller. */
    assert(!link_decode(bad, sizeof(good), &message));
    memcpy(bad, good, sizeof(good));
    bad[8 + 8] = bad[8 + 9] = bad[8 + 10] = bad[8 + 11] = 0; /* The pid. */
    assert(!link_decode(bad, sizeof(good), &message));
    for (int at = 20; at < 24; at++) { /* Reserved. */
        memcpy(bad, good, sizeof(good));
        bad[at] = 1;
        assert(!link_decode(bad, sizeof(good), &message));
    }
    /* Anything past what is used must be nothing. */
    for (int at = 24 + 48; at < LINK_MESSAGE_SIZE; at += 7) {
        memcpy(bad, good, sizeof(good));
        bad[at] = 1;
        assert(!link_decode(bad, sizeof(good), &message));
    }
    /* The slot's own small numbers and its padding. */
    for (int at = 24; at < 24 + 8; at++) {
        memcpy(bad, good, sizeof(good));
        bad[at] = at == 24 ? 3 : 16; /* A kind that is neither; numbers past 15. */
        if (at >= 29) bad[at] = 1;   /* Padding. */
        assert(!link_decode(bad, sizeof(good), &message));
    }
    memcpy(bad, good, sizeof(good));
    bad[24] = 0;
    assert(!link_decode(bad, sizeof(good), &message));
    /* A status that carries travellers, or flags that mean nothing. */
    link_encode_status(bad, &from, 1, 1);
    bad[6] = 1;
    assert(!link_decode(bad, sizeof(good), &message));
    link_encode_status(bad, &from, 1, 1);
    bad[7] = 4;
    assert(!link_decode(bad, sizeof(good), &message));

    /* Each double: not a number, infinite, and just out of its range. */
    struct {
        int at;
        double outside[4];
    } fields[] = {
        {24 + 8, {NAN, INFINITY, -0.001, 1.001}}, /* height */
        {24 + 16, {NAN, -INFINITY, -1, 10000.5}}, /* reach */
        {24 + 24, {NAN, INFINITY, -0.1, 6.2832}}, /* direction */
        {24 + 32, {NAN, INFINITY, -0.1, 1.5}},    /* wing clock */
        {24 + 40, {NAN, -INFINITY, -0.5, 60.5}},  /* holding */
    };
    for (size_t f = 0; f < sizeof(fields) / sizeof(*fields); f++) {
        for (int v = 0; v < 4; v++) {
            uint64_t bits;
            memcpy(&bits, &fields[f].outside[v], sizeof(bits));
            memcpy(bad, good, sizeof(good));
            for (int b = 0; b < 8; b++) bad[fields[f].at + b] = (uint8_t)(bits >> (56 - 8 * b));
            assert(!link_decode(bad, sizeof(good), &message));
        }
    }

    /* And noise, a great deal of it, is never one of ours. */
    unsigned long long state = 88172645463325252ULL;
    for (int round = 0; round < 20000; round++) {
        for (size_t i = 0; i < sizeof(good); i++) {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            bad[i] = (uint8_t)(state >> 24);
        }
        assert(!link_decode(bad, sizeof(good), &message));
    }
}

static void test_the_sky_is_kept_where_the_environment_says(void) {
    char path[64];

    assert(link_directory_for(path, sizeof(path), "/run/user/1000", "/var/tmp", 1000));
    assert(strcmp(path, "/run/user/1000/cbirds") == 0);
    /* A trailing slash does not make a double one. */
    assert(link_directory_for(path, sizeof(path), "/run/user/1000///", NULL, 1000));
    assert(strcmp(path, "/run/user/1000/cbirds") == 0);
    /* No runtime directory: the temporary one, with the user in the name. */
    assert(link_directory_for(path, sizeof(path), NULL, "/var/folders/xx/T/", 501));
    assert(strcmp(path, "/var/folders/xx/T/cbirds-501") == 0);
    assert(link_directory_for(path, sizeof(path), "", "/var/tmp", 7));
    assert(strcmp(path, "/var/tmp/cbirds-7") == 0);
    assert(link_directory_for(path, sizeof(path), NULL, NULL, 1000));
    assert(strcmp(path, "/tmp/cbirds-1000") == 0);
    assert(link_directory_for(path, sizeof(path), NULL, "", 1000));
    assert(strcmp(path, "/tmp/cbirds-1000") == 0);
    /* A relative path means something different to every process. */
    assert(link_directory_for(path, sizeof(path), "run", "tmp", 1000));
    assert(strcmp(path, "/tmp/cbirds-1000") == 0);
    /* It must fit, with its end. */
    assert(!link_directory_for(path, 10, NULL, NULL, 1000));
    assert(!link_directory_for(path, strlen("/tmp/cbirds-1000"), NULL, NULL, 1000));
    assert(link_directory_for(path, strlen("/tmp/cbirds-1000") + 1, NULL, NULL, 1000));
}

static void test_a_directory_must_be_private_and_ours(void) {
    struct stat info;
    memset(&info, 0, sizeof(info));
    info.st_uid = 1000;

    info.st_mode = S_IFDIR | 0700;
    assert(link_directory_check(&info, 1000) == LINK_OK);
    /* Others may look; they may not write. */
    info.st_mode = S_IFDIR | 0755;
    assert(link_directory_check(&info, 1000) == LINK_OK);
    info.st_mode = S_IFDIR | 0770;
    assert(link_directory_check(&info, 1000) == LINK_ERR_NOT_PRIVATE);
    info.st_mode = S_IFDIR | 0707;
    assert(link_directory_check(&info, 1000) == LINK_ERR_NOT_PRIVATE);
    info.st_mode = S_IFDIR | 01777; /* /tmp itself. */
    assert(link_directory_check(&info, 1000) == LINK_ERR_NOT_PRIVATE);
    info.st_mode = S_IFDIR | 0700;
    assert(link_directory_check(&info, 1001) == LINK_ERR_NOT_YOURS);
    assert(link_directory_check(&info, 0) == LINK_ERR_NOT_YOURS);
    /* A file, a link to a directory, a socket. */
    info.st_mode = S_IFREG | 0600;
    assert(link_directory_check(&info, 1000) == LINK_ERR_NOT_A_DIRECTORY);
    info.st_mode = S_IFLNK | 0777;
    assert(link_directory_check(&info, 1000) == LINK_ERR_NOT_A_DIRECTORY);
    info.st_mode = S_IFSOCK | 0600;
    assert(link_directory_check(&info, 1000) == LINK_ERR_NOT_A_DIRECTORY);
}

static void test_the_sky_is_made_private_and_refused_otherwise(void) {
    link_t window;
    struct stat info;
    char other[400];

    /* Made with nobody else able to get in. */
    assert(link_open(&window, sky, 0) == LINK_OK);
    assert(stat(sky, &info) == 0);
    assert(S_ISDIR(info.st_mode) && (info.st_mode & 0777) == 0700);
    assert(info.st_uid == geteuid());
    link_close(&window);
    assert(count_entries(sky) == 0);

    /* Open to others to write in: refused, and left exactly as it was. */
    assert(chmod(sky, 0777) == 0);
    assert(link_open(&window, sky, 0) == LINK_ERR_NOT_PRIVATE);
    assert(!window.opened && count_entries(sky) == 0);
    assert(chmod(sky, 0770) == 0);
    assert(link_open(&window, sky, 0) == LINK_ERR_NOT_PRIVATE);
    assert(stat(sky, &info) == 0 && (info.st_mode & 0777) == 0770);
    assert(chmod(sky, 0700) == 0);
    assert(link_open(&window, sky, 0) == LINK_OK);
    link_close(&window);
    clear_the_sky();

    /* A file where the sky should be. */
    int fd = open(sky, O_CREAT | O_WRONLY, 0600);
    assert(fd >= 0);
    close(fd);
    assert(link_open(&window, sky, 0) == LINK_ERR_NOT_A_DIRECTORY);
    assert(unlink(sky) == 0);

    /* A link to a good directory is not one either: whoever made it may
     * repoint it. */
    snprintf(other, sizeof(other), "%s/elsewhere", scratch);
    assert(mkdir(other, 0700) == 0);
    assert(symlink(other, sky) == 0);
    assert(link_open(&window, sky, 0) == LINK_ERR_NOT_A_DIRECTORY);
    assert(count_entries(other) == 0);
    assert(unlink(sky) == 0);
    assert(rmdir(other) == 0);

    /* A parent that is not there cannot be made for it. */
    snprintf(other, sizeof(other), "%s/missing/sky", scratch);
    assert(link_open(&window, other, 0) == LINK_ERR_DIRECTORY);
    assert(errno == ENOENT);
}

static void test_a_path_too_long_for_a_socket_is_refused(void) {
    link_t window;
    struct sockaddr_un probe;
    char longer[400];
    size_t limit = sizeof(probe.sun_path);
    char digits[16];
    size_t room = strlen(scratch) + 1;
    int pid_digits = snprintf(digits, sizeof(digits), "%ld", (long)getpid());

    /* Far too long: refused before a thing is made. */
    memset(longer, 'x', sizeof(longer));
    memcpy(longer, scratch, strlen(scratch));
    longer[strlen(scratch)] = '/';
    longer[300] = '\0';
    assert(link_open(&window, longer, 0) == LINK_ERR_PATH_TOO_LONG);
    assert(access(longer, F_OK) != 0);

    /* The last length that fits, and the one after it, to the byte: the path is
     * the directory, a slash, sixteen digits, a dash, the pid, and the NUL. */
    size_t fits = limit - 1 - (1 + 16 + 1 + (size_t)pid_digits);
    assert(fits > room + 1);
    snprintf(longer, sizeof(longer), "%s/", scratch);
    size_t at = strlen(longer);
    memset(longer + at, 'd', fits - at);
    longer[fits] = '\0';
    assert(strlen(longer) == fits);
    assert(link_open(&window, longer, 0) == LINK_OK);
    assert(strlen(window.path) == limit - 1);
    link_close(&window);
    assert(count_entries(longer) == 0);
    assert(rmdir(longer) == 0);

    longer[fits] = 'd';
    longer[fits + 1] = '\0';
    assert(link_open(&window, longer, 0) == LINK_ERR_PATH_TOO_LONG);
    assert(access(longer, F_OK) != 0);
    assert(strcmp(link_status_string(LINK_ERR_PATH_TOO_LONG), "") != 0);
}

static void test_windows_lie_in_the_order_they_joined(void) {
    link_t a, b, c;
    link_t *all[] = {&a, &b, &c};

    assert(link_open(&a, sky, 0) == LINK_OK);
    assert(!link_has_neighbour(&a, LINK_LEFT) && !link_has_neighbour(&a, LINK_RIGHT));
    assert(link_open(&b, sky, 0) == LINK_OK);
    assert(link_open(&c, sky, 0) == LINK_OK);

    /* The names sort in the order of joining, and say so. */
    assert(strcmp(a.name, b.name) < 0 && strcmp(b.name, c.name) < 0);
    assert(a.me.joined < b.me.joined && b.me.joined < c.me.joined);

    settle(all, 3);
    assert(!link_has_neighbour(&a, LINK_LEFT) && link_has_neighbour(&a, LINK_RIGHT));
    assert(link_has_neighbour(&b, LINK_LEFT) && link_has_neighbour(&b, LINK_RIGHT));
    assert(link_has_neighbour(&c, LINK_LEFT) && !link_has_neighbour(&c, LINK_RIGHT));
    assert(strcmp(a.next[LINK_RIGHT].name, b.name) == 0);
    assert(strcmp(b.next[LINK_LEFT].name, a.name) == 0);
    assert(strcmp(b.next[LINK_RIGHT].name, c.name) == 0);
    assert(strcmp(c.next[LINK_LEFT].name, b.name) == 0);
    assert(count_entries(sky) == 3);

    /* The middle one leaves: the other two are neighbours as soon as they look. */
    link_close(&b);
    link_t *outer[] = {&a, &c};
    settle(outer, 2);
    assert(strcmp(a.next[LINK_RIGHT].name, c.name) == 0);
    assert(strcmp(c.next[LINK_LEFT].name, a.name) == 0);
    assert(count_entries(sky) == 2);

    /* A newcomer joins on the right, whatever the clock said. */
    link_t d;
    assert(link_open(&d, sky, 0) == LINK_OK);
    assert(d.me.joined > c.me.joined);
    link_t *four[] = {&a, &c, &d};
    settle(four, 3);
    assert(strcmp(d.next[LINK_LEFT].name, c.name) == 0);
    assert(strcmp(c.next[LINK_RIGHT].name, d.name) == 0);

    link_close(&a);
    link_close(&c);
    link_close(&d);
    assert(count_entries(sky) == 0);
}

static void test_a_late_clock_does_not_put_a_newcomer_first(void) {
    link_t a, b;
    uint64_t far_ahead;

    assert(link_open(&a, sky, 0) == LINK_OK);
    /* A window that joined, by the system clock, a year from now, as if the
     * clock had since been set back. */
    far_ahead = a.me.joined + 365ULL * 24 * 3600 * 1000000000ULL;
    char name[LINK_NAME_SIZE], path[400];
    snprintf(name, sizeof(name), "%016llx-%lu", (unsigned long long)far_ahead,
             (unsigned long)getpid());
    snprintf(path, sizeof(path), "%s/%s", sky, name);
    struct sockaddr_un address;
    memset(&address, 0, sizeof(address));
    address.sun_family = AF_UNIX;
    strcpy(address.sun_path, path);
    int fd = socket(AF_UNIX, SOCK_DGRAM, 0);
    assert(fd >= 0 && bind(fd, (struct sockaddr *)&address, sizeof(address)) == 0);

    assert(link_open(&b, sky, 0) == LINK_OK);
    assert(b.me.joined > far_ahead);
    link_close(&b);
    link_close(&a);
    close(fd);
    unlink(path);
}

static void test_a_traveller_arrives_with_every_field_intact(void) {
    link_t a, b;
    link_t *both[] = {&a, &b};
    link_traveller_t sent = example(LINK_BIRD, 5), got;

    assert(link_open(&a, sky, 0) == LINK_OK);
    assert(link_open(&b, sky, 0) == LINK_OK);
    /* Nothing goes anywhere before the other side has said it is there. */
    assert(!link_edge_open(&a, LINK_RIGHT, LINK_BIRD));
    assert(link_send(&a, LINK_RIGHT, &sent, 1) == 0);
    settle(both, 2);
    assert(link_edge_open(&a, LINK_RIGHT, LINK_BIRD));
    assert(link_edge_open(&a, LINK_RIGHT, LINK_HAWK));
    assert(link_edge_open(&b, LINK_LEFT, LINK_BIRD));
    assert(!link_edge_open(&a, LINK_LEFT, LINK_BIRD)); /* Nobody there. */
    assert(!link_edge_open(&b, LINK_RIGHT, LINK_BIRD));

    /* Out of a's right edge and in by b's left. */
    assert(link_send(&a, LINK_RIGHT, &sent, 1) == 1);
    assert(link_receive(&b, &got) == 1);
    assert(got.enters == LINK_LEFT);
    sent.enters = LINK_LEFT;
    assert_same(&sent, &got);
    assert(link_receive(&b, &got) == 0);

    /* The other way, and a hawk. */
    sent = example(LINK_HAWK, 2);
    assert(link_send(&b, LINK_LEFT, &sent, 1) == 1);
    assert(link_receive(&a, &got) == 1);
    assert(got.enters == LINK_RIGHT);
    sent.enters = LINK_RIGHT;
    assert_same(&sent, &got);
    assert(link_receive(&a, &got) == 0);

    /* Nothing is sent to a side that has nobody on it. */
    assert(link_send(&a, LINK_LEFT, &sent, 1) == 0);
    assert(link_send(&b, LINK_RIGHT, &sent, 1) == 0);

    /* A great many at once come out in order, in batches. */
    link_traveller_t many[50], back;
    for (int i = 0; i < 50; i++) many[i] = example(i % 7 == 0 ? LINK_HAWK : LINK_BIRD, i);
    assert(link_send(&a, LINK_RIGHT, many, 50) == 50);
    for (int i = 0; i < 50; i++) {
        assert(link_receive(&b, &back) == 1);
        many[i].enters = LINK_LEFT;
        assert_same(&many[i], &back);
    }
    assert(link_receive(&b, &back) == 0);

    link_close(&a);
    link_close(&b);
}

static void test_a_full_neighbour_is_a_wall(void) {
    link_t a, b;
    link_t *both[] = {&a, &b};
    link_traveller_t bird = example(LINK_BIRD, 1), hawk = example(LINK_HAWK, 1);

    assert(link_open(&a, sky, 0) == LINK_OK);
    assert(link_open(&b, sky, 0) == LINK_OK);
    settle(both, 2);
    assert(link_edge_open(&a, LINK_RIGHT, LINK_BIRD));

    /* No room for birds: the door is shut to them, and to nothing else. */
    link_set_room(&b, 0, 1);
    drain(&a);
    assert(!link_edge_open(&a, LINK_RIGHT, LINK_BIRD));
    assert(link_edge_open(&a, LINK_RIGHT, LINK_HAWK));
    assert(link_send(&a, LINK_RIGHT, &bird, 1) == 0);
    assert(link_send(&a, LINK_RIGHT, &hawk, 1) == 1);
    assert(drain(&b) == 1);

    /* None for hawks either. */
    link_set_room(&b, 0, 0);
    drain(&a);
    assert(!link_edge_open(&a, LINK_RIGHT, LINK_HAWK));
    assert(link_send(&a, LINK_RIGHT, &hawk, 1) == 0);

    /* A batch with a hawk in it stops where the door shuts. */
    link_set_room(&b, 1, 0);
    drain(&a);
    link_traveller_t mixed[3] = {bird, bird, hawk};
    assert(link_send(&a, LINK_RIGHT, mixed, 3) == 2);
    assert(drain(&b) == 2);

    /* Room again, and it is open again at once. */
    link_set_room(&b, 1, 1);
    drain(&a);
    assert(link_edge_open(&a, LINK_RIGHT, LINK_BIRD));
    assert(link_send(&a, LINK_RIGHT, &bird, 1) == 1);
    assert(drain(&b) == 1);

    link_close(&a);
    link_close(&b);
}

static void test_a_neighbour_that_falls_silent_is_not_trusted(void) {
    link_t a, b;
    link_t *both[] = {&a, &b};
    link_traveller_t bird = example(LINK_BIRD, 1);

    assert(link_open(&a, sky, 0) == LINK_OK);
    assert(link_open(&b, sky, 0) == LINK_OK);
    settle(both, 2);
    assert(link_edge_open(&a, LINK_RIGHT, LINK_BIRD));

    /* a keeps going; b says nothing for three seconds and a bit. */
    the_time += 2.0;
    link_update(&a, the_time);
    assert(link_edge_open(&a, LINK_RIGHT, LINK_BIRD));
    the_time += 2.0;
    link_update(&a, the_time);
    assert(!link_edge_open(&a, LINK_RIGHT, LINK_BIRD));
    assert(link_send(&a, LINK_RIGHT, &bird, 1) == 0);
    assert(link_has_neighbour(&a, LINK_RIGHT)); /* Still there; just not heard. */

    /* b speaks, and the door is open. */
    link_update(&b, the_time);
    drain(&a);
    assert(link_edge_open(&a, LINK_RIGHT, LINK_BIRD));

    link_close(&a);
    link_close(&b);
}

static void test_a_stranger_makes_a_window_look_again(void) {
    link_t a, b;

    assert(link_open(&a, sky, 0) == LINK_OK);
    link_set_room(&a, 1, 1);
    the_time += 0.5; /* Not yet time to look. */
    link_update(&a, the_time);
    assert(!link_has_neighbour(&a, LINK_RIGHT));

    /* b arrives and says hello: a hears of it now and not in a second. */
    assert(link_open(&b, sky, the_time) == LINK_OK);
    link_set_room(&b, 1, 1);
    the_time += 0.1;
    link_update(&a, the_time);
    assert(drain(&a) == 0);
    assert(link_has_neighbour(&a, LINK_RIGHT));
    assert(link_edge_open(&a, LINK_RIGHT, LINK_BIRD));
    /* And it greeted b in return, so b can send at once. */
    assert(drain(&b) == 0);
    assert(link_edge_open(&b, LINK_LEFT, LINK_BIRD));

    link_close(&a);
    link_close(&b);
}

/* A process that joined and was killed: its socket is there and its pid is not. */
static void leave_a_corpse(const char *directory) {
    pid_t child = fork();
    assert(child >= 0);
    if (child == 0) {
        link_t ghost;
        if (link_open(&ghost, directory, 0) != LINK_OK) _exit(1);
        _exit(0); /* Not link_close: that is the point. */
    }
    int status = 0;
    assert(waitpid(child, &status, 0) == child);
    assert(WIFEXITED(status) && WEXITSTATUS(status) == 0);
}

static void test_a_killed_window_is_dropped_and_swept_away(void) {
    link_t a, b;
    link_t *both[] = {&a, &b};

    assert(link_open(&a, sky, 0) == LINK_OK);
    leave_a_corpse(sky);
    assert(count_entries(sky) == 2);

    /* The corpse sits to a's right, and a finds it gone when it looks. */
    the_time += 1.1;
    link_update(&a, the_time);
    assert(!link_has_neighbour(&a, LINK_RIGHT));
    assert(count_entries(sky) == 1);

    /* One that is left behind while another is joining does not break the
     * joining: the newcomer is a's neighbour, not the corpse. */
    leave_a_corpse(sky);
    assert(count_entries(sky) == 2);
    assert(link_open(&b, sky, the_time) == LINK_OK);
    assert(count_entries(sky) == 2);
    settle(both, 2);
    assert(strcmp(a.next[LINK_RIGHT].name, b.name) == 0);
    assert(link_edge_open(&a, LINK_RIGHT, LINK_BIRD));

    link_close(&a);
    link_close(&b);
    assert(count_entries(sky) == 0);
}

/* A socket file with nobody behind it, under the name of a process that is
 * alive: our own. Taking a pid as proof of life is the weak spot of looking at
 * names, and a datagram that is refused is the proof that settles it. */
static void test_a_socket_nobody_holds_is_swept_away_by_the_first_word(void) {
    link_t a;
    char name[LINK_NAME_SIZE], path[400];
    struct sockaddr_un address;

    assert(link_open(&a, sky, 0) == LINK_OK);
    snprintf(name, sizeof(name), "%016llx-%lu", (unsigned long long)(a.me.joined + 1000),
             (unsigned long)getpid());
    snprintf(path, sizeof(path), "%s/%s", sky, name);
    memset(&address, 0, sizeof(address));
    address.sun_family = AF_UNIX;
    strcpy(address.sun_path, path);
    int fd = socket(AF_UNIX, SOCK_DGRAM, 0);
    assert(fd >= 0 && bind(fd, (struct sockaddr *)&address, sizeof(address)) == 0);
    close(fd); /* Gone, and the file stays. */
    assert(access(path, F_OK) == 0);

    /* Its pid is alive, so it is taken for a neighbour; the heartbeat to it is
     * refused, and it is a neighbour no longer. */
    the_time += 1.1;
    link_update(&a, the_time);
    assert(!link_has_neighbour(&a, LINK_RIGHT));
    assert(access(path, F_OK) != 0);
    assert(count_entries(sky) == 1);

    link_close(&a);
}

static void test_a_neighbour_that_dies_between_two_words_is_found_by_a_refusal(void) {
    link_t a, b;
    link_t *both[] = {&a, &b};
    link_traveller_t bird = example(LINK_BIRD, 1);

    assert(link_open(&a, sky, 0) == LINK_OK);
    assert(link_open(&b, sky, 0) == LINK_OK);
    settle(both, 2);
    assert(link_edge_open(&a, LINK_RIGHT, LINK_BIRD));

    /* b is killed: the socket is closed under it and its file is left behind,
     * under a pid that is still alive, because it is this process's. */
    close(b.fd);
    b.fd = -1;
    assert(access(b.path, F_OK) == 0);
    assert(link_send(&a, LINK_RIGHT, &bird, 1) == 0); /* Refused: the bird stays. */
    assert(!link_has_neighbour(&a, LINK_RIGHT));
    assert(access(b.path, F_OK) != 0);

    link_close(&b); /* Nothing left of it to remove, and no complaint. */
    link_close(&a);
    assert(count_entries(sky) == 0);
}

static void test_a_full_queue_holds_the_birds_and_loses_none(void) {
    link_t a, b;
    link_t *both[] = {&a, &b};
    link_traveller_t bird = example(LINK_BIRD, 1), got;
    int posted = 0, received = 0, refused = 0;

    assert(link_open(&a, sky, 0) == LINK_OK);
    assert(link_open(&b, sky, 0) == LINK_OK);
    settle(both, 2);

    /* b reads nothing while a posts as fast as it can, until the system says
     * enough. Whatever it said was taken is in b's queue, and nothing else. */
    for (int round = 0; round < 100000 && !refused; round++) {
        int sent = link_send(&a, LINK_RIGHT, &bird, 1);
        if (sent == 0) refused = 1;
        posted += sent;
    }
    assert(refused && posted > 0);
    /* Busy is not gone: it is still a neighbour, and it will take more once it
     * has read what it has. */
    assert(link_has_neighbour(&a, LINK_RIGHT));
    while (link_receive(&b, &got)) received++;
    assert(received == posted);
    assert(link_send(&a, LINK_RIGHT, &bird, 1) == 1);
    assert(link_receive(&b, &got) == 1);

    link_close(&a);
    link_close(&b);
}

static void test_malformed_and_foreign_datagrams_are_ignored(void) {
    link_t a;
    link_traveller_t sent = example(LINK_BIRD, 4), got;
    uint8_t bytes[LINK_MESSAGE_SIZE];
    link_sender_t stranger = {5, 31337};
    struct sockaddr_un to;

    assert(link_open(&a, sky, 0) == LINK_OK);
    memset(&to, 0, sizeof(to));
    to.sun_family = AF_UNIX;
    strcpy(to.sun_path, a.path);
    int fd = socket(AF_UNIX, SOCK_DGRAM, 0);
    assert(fd >= 0);

    const char *hello = "hello, is this the sky?";
    assert(sendto(fd, hello, strlen(hello), 0, (struct sockaddr *)&to, sizeof(to)) > 0);
    assert(sendto(fd, hello, 0, 0, (struct sockaddr *)&to, sizeof(to)) == 0);
    memset(bytes, 0, sizeof(bytes));
    assert(sendto(fd, bytes, sizeof(bytes), 0, (struct sockaddr *)&to, sizeof(to)) > 0);
    link_encode_travellers(bytes, &stranger, &sent, 1);
    bytes[4] = 9; /* A later version. */
    assert(sendto(fd, bytes, sizeof(bytes), 0, (struct sockaddr *)&to, sizeof(to)) > 0);
    link_encode_travellers(bytes, &stranger, &sent, 1);
    memset(bytes + 24 + 8, 0xff, 8); /* A height that is not a number. */
    assert(sendto(fd, bytes, sizeof(bytes), 0, (struct sockaddr *)&to, sizeof(to)) > 0);
    link_encode_travellers(bytes, &stranger, &sent, 1);
    assert(sendto(fd, bytes, sizeof(bytes) - 1, 0, (struct sockaddr *)&to, sizeof(to)) > 0);
    /* And one that is right. */
    link_encode_travellers(bytes, &stranger, &sent, 1);
    assert(sendto(fd, bytes, sizeof(bytes), 0, (struct sockaddr *)&to, sizeof(to)) > 0);

    the_time += 0.01;
    assert(link_receive(&a, &got) == 1);
    sent.enters = 0;
    assert_same(&sent, &got);
    assert(link_receive(&a, &got) == 0);

    close(fd);
    link_close(&a);
}

/* A flood of rubbish is read and thrown away, and the call comes back. */
static void test_a_flood_of_rubbish_is_thrown_away(void) {
    link_t a;
    link_traveller_t got;
    struct sockaddr_un to;
    char junk[16] = "not ours";

    assert(link_open(&a, sky, 0) == LINK_OK);
    memset(&to, 0, sizeof(to));
    to.sun_family = AF_UNIX;
    strcpy(to.sun_path, a.path);
    int fd = socket(AF_UNIX, SOCK_DGRAM, 0);
    assert(fd >= 0 && fcntl(fd, F_SETFL, O_NONBLOCK) == 0);
    for (int i = 0; i < 100000; i++)
        if (sendto(fd, junk, sizeof(junk), 0, (struct sockaddr *)&to, sizeof(to)) < 0) break;
    assert(link_receive(&a, &got) == 0);
    close(fd);
    link_close(&a);
}

static void test_the_socket_is_removed_on_leaving(void) {
    link_t a;

    assert(link_open(&a, sky, 0) == LINK_OK);
    assert(access(a.path, F_OK) == 0);
    int fd = a.fd;
    link_close(&a);
    assert(access(sky, F_OK) == 0 && count_entries(sky) == 0);
    assert(fcntl(fd, F_GETFD) < 0 && errno == EBADF);
    link_close(&a); /* Twice is nothing. */
    link_close(NULL);
    /* A window that never opened closes as nothing. */
    link_t never;
    memset(&never, 0, sizeof(never));
    link_close(&never);
    assert(link_open(&a, sky, 0) == LINK_OK);
    link_close(&a);
}

static void test_a_socket_that_was_taken_away_is_made_again(void) {
    link_t a, b;
    link_t *both[] = {&a, &b};

    assert(link_open(&a, sky, 0) == LINK_OK);
    assert(link_open(&b, sky, 0) == LINK_OK);
    settle(both, 2);
    assert(unlink(b.path) == 0); /* A tidy up of the temporary directory. */
    settle(both, 2);             /* b makes its socket again... */
    assert(access(b.path, F_OK) == 0);
    settle(both, 2); /* ...and a finds it where it was. */
    assert(link_edge_open(&a, LINK_RIGHT, LINK_BIRD));
    link_traveller_t bird = example(LINK_BIRD, 2), got;
    assert(link_send(&a, LINK_RIGHT, &bird, 1) == 1);
    assert(link_receive(&b, &got) == 1);
    link_close(&a);
    link_close(&b);
    assert(count_entries(sky) == 0);
}

static void test_files_that_are_not_windows_are_left_alone(void) {
    link_t a, b;
    link_t *both[] = {&a, &b};
    const char *names[] = {"README",
                           "0123456789abcdef",
                           "0123456789abcdef-",
                           "0123456789abcdef-0",
                           "0123456789ABCDEF-12",
                           "0123456789abcdef-12x",
                           "0123456789abcdef-012",
                           "0123456789abcdef-99999999999",
                           ".hidden",
                           "zzzz"};

    assert(link_open(&a, sky, 0) == LINK_OK);
    for (size_t i = 0; i < sizeof(names) / sizeof(*names); i++) {
        char path[400];
        snprintf(path, sizeof(path), "%s/%s", sky, names[i]);
        int fd = open(path, O_CREAT | O_WRONLY, 0600);
        assert(fd >= 0);
        close(fd);
    }
    assert(link_open(&b, sky, 0) == LINK_OK);
    settle(both, 2);
    assert(strcmp(a.next[LINK_RIGHT].name, b.name) == 0);
    assert(strcmp(b.next[LINK_LEFT].name, a.name) == 0);
    for (size_t i = 0; i < sizeof(names) / sizeof(*names); i++) {
        char path[400];
        snprintf(path, sizeof(path), "%s/%s", sky, names[i]);
        assert(access(path, F_OK) == 0);
    }
    link_close(&a);
    link_close(&b);
    assert(count_entries(sky) == (int)(sizeof(names) / sizeof(*names)));
}

/* An entry in the sky that looks like a window and is not one: its name is
 * exactly what a window's is, and its pid is this process's own, which is alive,
 * so that nothing in the name gives it away. */
static void fake_path(char *path, size_t size, uint64_t joined) {
    snprintf(path, size, "%s/%016llx-%lu", sky, (unsigned long long)joined,
             (unsigned long)getpid());
}

static ino_t inode_of(const char *path) {
    struct stat info;
    assert(lstat(path, &info) == 0);
    return info.st_ino;
}

/* A window finds itself by its own name and not by a place in a list that is cut
 * off: with hundreds of entries about it, its socket is the same file at every
 * look, and not made again, which would throw away what was queued in it. */
static void test_a_window_keeps_its_own_socket_in_a_sky_of_hundreds(void) {
    link_t a, b;
    link_t *both[] = {&a, &b};
    enum { FAKES = 700 };

    assert(link_open(&a, sky, 0) == LINK_OK);
    for (int i = 0; i < FAKES; i++) {
        char path[400];
        fake_path(path, sizeof(path), a.me.joined + (uint64_t)(i - FAKES / 2) * 1000 + 500);
        assert(symlink("/nonexistent/nowhere", path) == 0);
    }
    assert(link_open(&b, sky, 0) == LINK_OK);
    ino_t a_was = inode_of(a.path), b_was = inode_of(b.path);
    /* Where in the listing they come is up to the file system, so each is named
     * before the hundreds and after them. */
    for (int round = 0; round < 12; round++) {
        settle(both, 2);
        assert(inode_of(a.path) == a_was && inode_of(b.path) == b_was);
    }
    /* None of the entries is a window: the two are each other's neighbours. */
    assert(strcmp(a.next[LINK_RIGHT].name, b.name) == 0);
    assert(strcmp(b.next[LINK_LEFT].name, a.name) == 0);
    assert(link_edge_open(&a, LINK_RIGHT, LINK_BIRD));
    link_close(&a);
    link_close(&b);
    assert(count_entries(sky) == FAKES);
}

/* An entry that nothing can be sent to is not tried again at every look: a link to
 * a file that is not there, a directory, a plain file, and a socket of the wrong
 * kind, which only a refusal shows. The last is left alone until the directory has
 * changed, and then it is tried, and found to be a window after all. */
static void test_an_entry_that_can_never_be_sent_to_is_let_be(void) {
    link_t a;
    char path[400];

    assert(link_open(&a, sky, 0) == LINK_OK);
    fake_path(path, sizeof(path), a.me.joined + 1000);
    assert(symlink("/nonexistent/nowhere", path) == 0);
    fake_path(path, sizeof(path), a.me.joined + 2000);
    assert(mkdir(path, 0700) == 0);
    fake_path(path, sizeof(path), a.me.joined + 3000);
    int plain = open(path, O_CREAT | O_WRONLY, 0600);
    assert(plain >= 0);
    close(plain);
    char stream_path[400];
    fake_path(stream_path, sizeof(stream_path), a.me.joined + 4000);
    struct sockaddr_un address;
    memset(&address, 0, sizeof(address));
    address.sun_family = AF_UNIX;
    strcpy(address.sun_path, stream_path);
    int stream = socket(AF_UNIX, SOCK_STREAM, 0);
    assert(stream >= 0 && bind(stream, (struct sockaddr *)&address, sizeof(address)) == 0);
    assert(listen(stream, 1) == 0);

    /* Three seconds of frames. Looking again at once, because a neighbour has been
     * lost, is for a neighbour that was there; these are never one. */
    int scans = 0;
    for (int frame = 0; frame < 180; frame++) {
        double before = a.scanned;
        the_time += 1.0 / 60;
        link_update(&a, the_time);
        if (a.scanned != before) scans++;
        assert(!link_has_neighbour(&a, LINK_RIGHT));
    }
    assert(scans <= 8);
    assert(!a.scan_wanted);

    /* The directory changes: the socket of the wrong kind is replaced by a window's
     * kind, under the same name. The file system's clock is coarse, so it waits. */
    close(stream);
    unlink(stream_path); /* Some systems have swept it already. */
    usleep(60000);
    int dgram = socket(AF_UNIX, SOCK_DGRAM, 0);
    assert(dgram >= 0 && bind(dgram, (struct sockaddr *)&address, sizeof(address)) == 0);
    for (int frame = 0; frame < 180 && !link_has_neighbour(&a, LINK_RIGHT); frame++) {
        the_time += 1.0 / 60;
        link_update(&a, the_time);
    }
    assert(link_has_neighbour(&a, LINK_RIGHT));
    uint8_t heard[LINK_MESSAGE_SIZE + 1];
    assert(recv(dgram, heard, sizeof(heard), MSG_DONTWAIT) == LINK_MESSAGE_SIZE);
    close(dgram);
    link_close(&a);
    fake_path(path, sizeof(path), a.me.joined + 2000);
    assert(rmdir(path) == 0); /* The sky is cleared of what can be unlinked. */
}

int main(void) {
    make_scratch();
    test_the_wire_format_round_trips();
    test_malformed_datagrams_are_not_ours();
    test_the_sky_is_kept_where_the_environment_says();
    test_a_directory_must_be_private_and_ours();

    /* The rest each start with no sky and leave none. */
    void (*const with_a_sky[])(void) = {
        test_the_sky_is_made_private_and_refused_otherwise,
        test_a_path_too_long_for_a_socket_is_refused,
        test_windows_lie_in_the_order_they_joined,
        test_a_late_clock_does_not_put_a_newcomer_first,
        test_a_traveller_arrives_with_every_field_intact,
        test_a_full_neighbour_is_a_wall,
        test_a_neighbour_that_falls_silent_is_not_trusted,
        test_a_stranger_makes_a_window_look_again,
        test_a_killed_window_is_dropped_and_swept_away,
        test_a_socket_nobody_holds_is_swept_away_by_the_first_word,
        test_a_neighbour_that_dies_between_two_words_is_found_by_a_refusal,
        test_a_full_queue_holds_the_birds_and_loses_none,
        test_malformed_and_foreign_datagrams_are_ignored,
        test_a_flood_of_rubbish_is_thrown_away,
        test_the_socket_is_removed_on_leaving,
        test_a_socket_that_was_taken_away_is_made_again,
        test_files_that_are_not_windows_are_left_alone,
        test_a_window_keeps_its_own_socket_in_a_sky_of_hundreds,
        test_an_entry_that_can_never_be_sent_to_is_let_be,
    };
    for (size_t i = 0; i < sizeof(with_a_sky) / sizeof(*with_a_sky); i++) {
        clear_the_sky();
        the_time = 0;
        with_a_sky[i]();
        clear_the_sky();
    }
    /* Every test removes what it wrote, so this fails if one did not. */
    assert(rmdir(scratch) == 0);
    return 0;
}
