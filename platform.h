/*
 * Everything cbirds asks of the operating system's terminal, in one place.
 *
 * On POSIX these are the termios, ioctl, poll, sigaction and clock calls the
 * program always made, moved here unchanged. On Windows they are the console
 * API: the same escape sequences go out and the same bytes come in, so nothing
 * above this file knows which it is running on. Only the live run uses the
 * terminal half; the clock and the byte conversions are used everywhere.
 */
#ifndef PLATFORM_H
#define PLATFORM_H

#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

#ifdef _WIN32
#ifndef STDIN_FILENO
#define STDIN_FILENO 0
#define STDOUT_FILENO 1
#define STDERR_FILENO 2
#endif
/* ssize_t comes with <sys/types.h> in MinGW-w64; this is for a header that
 * does not have it. */
#ifndef _SSIZE_T_DEFINED
#define _SSIZE_T_DEFINED
typedef long long ssize_t;
#endif
/* The names POSIX gives the clock's fields, so the arithmetic above this file
 * reads the same. `struct timespec` is left alone: with -std=c99 MinGW's headers
 * may or may not declare it, and a type of our own cannot clash. */
typedef struct {
    long long tv_sec;
    long tv_nsec;
} platform_time_t;
#else
#include <time.h>
#include <unistd.h>
typedef struct timespec platform_time_t;
#endif

/* Once, first thing in main. Windows: stdout in binary mode, so a \n is a \n. */
void platform_init(void);

/* A monotonic clock, and a sleep that is as good as the clock: Windows rounds
 * Sleep to the 15.6 ms tick, which at sixty frames a second is not a pace. */
void platform_now(platform_time_t *now);
void platform_sleep_microseconds(long microseconds);

/* write(2) on POSIX; WriteFile on Windows, setting errno the way write would. */
ssize_t platform_write(int fd, const void *data, size_t length);

/* The terminal as the program wants it: no line editing, no echo, keys as they
 * come, and on Windows escape sequences understood in both directions. The
 * input half is undone first and the output half last, because on Windows the
 * sequences that put the screen back need the output mode still on. Both are
 * safe to call from a signal handler or a console control thread, and twice. */
int platform_enter_raw(void);
void platform_leave_raw(void);
void platform_leave_output(void);
/* Says why there is no terminal to run in, on stderr. */
void platform_report_no_terminal(void);

/* Columns, rows and, where it is known, pixels of the visible window; zeroes
 * for what is not. */
void platform_window_size(int *columns, int *rows, int *pixel_width, int *pixel_height);

/* Waits up to the given time for input and says whether there is some: 1, 0 for
 * none, -1 for an error (with errno; POSIX may report EINTR). Then a read that
 * never blocks: the bytes that are there, 0 for none, -1 for an error. */
int platform_wait_input(int milliseconds);
ssize_t platform_read_input(void *buffer, size_t size);
/* Waits, without limit, until the terminal can take output or give input. */
int platform_wait_terminal_io(void);

/* What happens when the program is told to stop, from outside or by a crash:
 * `restore` puts the terminal back, then the process exits with 128 plus the
 * signal's number, as a shell reports a death by signal. Windows: Ctrl-C and
 * Ctrl-Break (130 and 149), the window being closed (129), logoff and shutdown
 * (143). The lock makes `restore` and a control event on its own thread take
 * turns; it is recursive and does nothing on POSIX, where a handler may not
 * take one. */
void platform_install_exit_handlers(void (*restore)(void));
void platform_lock_restore(void);
void platform_unlock_restore(void);

/*
 * Byte conversions the Windows console input needs. They are plain functions of
 * plain integers and are built everywhere, so that the tests run them on Linux.
 */

/* One UTF-16 unit from a key event to UTF-8, in out[4]; returns the number of
 * bytes, 0 when none yet (a high surrogate waits in *high for its pair) or none
 * at all (NUL, or a surrogate with no partner, which is dropped). */
size_t platform_utf8_from_utf16(uint16_t *high, unsigned unit, char out[4]);

/* One MOUSE_EVENT_RECORD as the SGR mouse reports the program already reads
 * (ESC [ < button ; column ; row M or m), appended to out. `held` is the button
 * state of the last record and is updated. state, flags and wheel are the
 * record's dwButtonState (low word), dwEventFlags and the signed high word of
 * dwButtonState; column and row are zero based within the visible window.
 * Returns the length written, which is 0 for a record that changed nothing. */
size_t platform_sgr_mouse(unsigned *held, unsigned state, unsigned flags, int wheel, int column,
                          int row, char *out, size_t size);

#endif
