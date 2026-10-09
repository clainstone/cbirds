/* Feature test macros must precede every include. */
#ifdef _WIN32
/* The pseudo console and the high resolution timer are Windows 10. */
#ifndef _WIN32_WINNT
#define _WIN32_WINNT 0x0A00
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#else
#define _XOPEN_SOURCE 700
#define _DEFAULT_SOURCE
#define _DARWIN_C_SOURCE
#endif

#include "platform.h"

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#include <windows.h>
#else
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#endif

/* ------------------------------------------------------------------------- */
/* Byte conversions: built everywhere, used by the Windows input.            */
/* ------------------------------------------------------------------------- */

size_t platform_utf8_from_utf16(uint16_t *high, unsigned unit, char out[4]) {
    unsigned long code;
    if (unit >= 0xD800 && unit <= 0xDBFF) {
        *high = (uint16_t)unit; /* A second one in a row replaces the first. */
        return 0;
    }
    if (unit >= 0xDC00 && unit <= 0xDFFF) {
        if (*high == 0) return 0; /* Nothing to pair with. */
        code = 0x10000UL + (((unsigned long)*high - 0xD800UL) << 10) + (unit - 0xDC00UL);
        *high = 0;
    } else {
        *high = 0; /* A high surrogate that was never answered is dropped. */
        code = unit;
    }
    if (code == 0) return 0;
    if (code < 0x80) {
        out[0] = (char)code;
        return 1;
    }
    if (code < 0x800) {
        out[0] = (char)(0xC0 | (code >> 6));
        out[1] = (char)(0x80 | (code & 0x3F));
        return 2;
    }
    if (code < 0x10000) {
        out[0] = (char)(0xE0 | (code >> 12));
        out[1] = (char)(0x80 | ((code >> 6) & 0x3F));
        out[2] = (char)(0x80 | (code & 0x3F));
        return 3;
    }
    out[0] = (char)(0xF0 | (code >> 18));
    out[1] = (char)(0x80 | ((code >> 12) & 0x3F));
    out[2] = (char)(0x80 | ((code >> 6) & 0x3F));
    out[3] = (char)(0x80 | (code & 0x3F));
    return 4;
}

size_t platform_sgr_mouse(unsigned *held, unsigned state, unsigned flags, int wheel, int column,
                          int row, char *out, size_t size) {
    /* The console numbers its buttons left, right, middle; SGR left, middle,
     * right. */
    static const struct {
        unsigned mask;
        int code;
    } buttons[] = {{0x1, 0}, {0x4, 1}, {0x2, 2}};
    enum { MOVED = 0x1, WHEELED = 0x4, HWHEELED = 0x8, ALL_BUTTONS = 0x7 };
    size_t length = 0;
    int changed = 0;
    int x = column < 0 ? 1 : column + 1, y = row < 0 ? 1 : row + 1;

    if (size == 0) return 0;
    out[0] = '\0';
    if (flags & (WHEELED | HWHEELED)) {
        /* Forward is up and to the right in the console's own sign. */
        int code = (flags & WHEELED) ? (wheel > 0 ? 64 : 65) : (wheel > 0 ? 67 : 66);
        int n = snprintf(out, size, "\033[<%d;%d;%dM", code, x, y);
        return n > 0 && (size_t)n < size ? (size_t)n : 0;
    }
    for (size_t i = 0; i < sizeof(buttons) / sizeof(*buttons); i++) {
        unsigned now = state & buttons[i].mask, before = *held & buttons[i].mask;
        if (now == before) continue;
        int n = snprintf(out + length, size - length, "\033[<%d;%d;%d%c", buttons[i].code, x, y,
                         now ? 'M' : 'm');
        if (n < 0 || (size_t)n >= size - length) break;
        length += (size_t)n;
        changed = 1;
    }
    *held = state & ALL_BUTTONS;
    if (!changed && (flags & MOVED)) {
        /* Motion carries the button that is down, or 3 for none, plus 32. */
        int code = 3;
        for (size_t i = 0; i < sizeof(buttons) / sizeof(*buttons); i++)
            if (state & buttons[i].mask) {
                code = buttons[i].code;
                break;
            }
        int n = snprintf(out + length, size - length, "\033[<%d;%d;%dM", 32 + code, x, y);
        if (n > 0 && (size_t)n < size - length) length += (size_t)n;
    }
    return length;
}

#ifndef _WIN32
/* ------------------------------------------------------------------------- */
/* POSIX: what boids.c always did, moved here unchanged.                     */
/* ------------------------------------------------------------------------- */

static struct termios saved_termios;
static volatile sig_atomic_t termios_saved;
static void (*restore_hook)(void);

void platform_init(void) {}

void platform_now(platform_time_t *now) {
    clock_gettime(CLOCK_MONOTONIC, now);
}

void platform_sleep_microseconds(long microseconds) {
    struct timespec delay = {microseconds / 1000000L, (microseconds % 1000000L) * 1000L};
    nanosleep(&delay, NULL);
}

ssize_t platform_write(int fd, const void *data, size_t length) {
    return write(fd, data, length);
}

int platform_enter_raw(void) {
    struct termios raw;
    if (tcgetattr(STDIN_FILENO, &raw) < 0) return -1;
    saved_termios = raw;
    raw.c_iflag &= (tcflag_t) ~(tcflag_t)(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
    raw.c_oflag &= (tcflag_t) ~(tcflag_t)OPOST;
    raw.c_cflag |= CS8;
    raw.c_lflag &= (tcflag_t) ~(tcflag_t)(ECHO | ICANON | IEXTEN);
    raw.c_cc[VSUSP] = _POSIX_VDISABLE;
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw) < 0) return -1;
    termios_saved = 1;
    return 0;
}

void platform_leave_raw(void) {
    if (!termios_saved) return;
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &saved_termios);
    termios_saved = 0;
}

void platform_leave_output(void) {}

void platform_report_no_terminal(void) {
    perror("Can't enable raw mode");
}

void platform_window_size(int *columns, int *rows, int *pixel_width, int *pixel_height) {
    struct winsize size;
    memset(&size, 0, sizeof(size));
    if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &size) < 0) memset(&size, 0, sizeof(size));
    *columns = size.ws_col;
    *rows = size.ws_row;
    *pixel_width = size.ws_xpixel;
    *pixel_height = size.ws_ypixel;
}

int platform_wait_input(int milliseconds) {
    struct pollfd wait = {.fd = STDIN_FILENO, .events = POLLIN};
    return poll(&wait, 1, milliseconds);
}

ssize_t platform_read_input(void *buffer, size_t size) {
    return read(STDIN_FILENO, buffer, size);
}

int platform_wait_terminal_io(void) {
    struct pollfd descriptors[] = {
        {.fd = STDIN_FILENO, .events = POLLIN},
        {.fd = STDOUT_FILENO, .events = POLLOUT},
    };
    int result;
    do {
        result = poll(descriptors, sizeof(descriptors) / sizeof(*descriptors), -1);
    } while (result < 0 && errno == EINTR);
    return result < 0 ? -1 : 0;
}

static void signal_handler(int signal_number) {
    if (restore_hook != NULL) restore_hook();
    _exit(128 + signal_number);
}

void platform_install_exit_handlers(void (*restore)(void)) {
    static const int signals[] = {SIGINT,  SIGTERM, SIGHUP, SIGQUIT,
                                  SIGSEGV, SIGFPE,  SIGBUS, SIGABRT};
    struct sigaction action;
    restore_hook = restore;
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

void platform_lock_restore(void) {}
void platform_unlock_restore(void) {}

#else
/* ------------------------------------------------------------------------- */
/* Windows 10 and 11: the console API.                                       */
/* ------------------------------------------------------------------------- */

#ifndef ENABLE_VIRTUAL_TERMINAL_PROCESSING
#define ENABLE_VIRTUAL_TERMINAL_PROCESSING 0x0004
#endif
#ifndef DISABLE_NEWLINE_AUTO_RETURN
#define DISABLE_NEWLINE_AUTO_RETURN 0x0008
#endif
#ifndef ENABLE_VIRTUAL_TERMINAL_INPUT
#define ENABLE_VIRTUAL_TERMINAL_INPUT 0x0200
#endif
#ifndef CREATE_WAITABLE_TIMER_HIGH_RESOLUTION
#define CREATE_WAITABLE_TIMER_HIGH_RESOLUTION 0x00000002
#endif

/* SIGBREAK is 21 in the C runtime; the name is not in the strict headers. */
enum { BREAK_SIGNAL = 21, INTERRUPT_SIGNAL = 2, HANGUP_SIGNAL = 1, TERMINATE_SIGNAL = 15 };

static void (*restore_hook)(void);
static CRITICAL_SECTION restore_lock;
static volatile LONG restore_lock_ready;

static HANDLE input_handle = INVALID_HANDLE_VALUE;
static HANDLE output_handle = INVALID_HANDLE_VALUE;
static DWORD saved_input_mode, saved_output_mode;
static UINT saved_output_code_page;
static volatile LONG input_mode_changed, output_mode_changed, code_page_changed;
/* Set by a console control event before it puts the screen back, so a frame the
 * main thread is still writing cannot land on the shell's screen afterwards. */
static volatile LONG halting;
static volatile DWORD halting_thread;

static int is_console(HANDLE handle, DWORD *mode) {
    return handle != NULL && handle != INVALID_HANDLE_VALUE && GetConsoleMode(handle, mode);
}

static void ensure_restore_lock(void) {
    if (restore_lock_ready) return;
    /* Only ever first reached from main, before any control event can be taken. */
    InitializeCriticalSection(&restore_lock);
    restore_lock_ready = 1;
}

void platform_lock_restore(void) {
    if (restore_lock_ready) EnterCriticalSection(&restore_lock);
}

void platform_unlock_restore(void) {
    if (restore_lock_ready) LeaveCriticalSection(&restore_lock);
}

/* The console decodes what it is given with its output code page, which is a
 * legacy one (437 or 850) until told otherwise, so the UTF-8 the program writes,
 * the em dash in --help included, would come out as three odd characters. Put
 * to UTF-8 for the life of the process and put back at the end, but only on a
 * console: a pipe or a file has no code page and is left alone. */
static void use_utf8_output(void) {
    DWORD mode;
    HANDLE out = GetStdHandle(STD_OUTPUT_HANDLE);
    if (code_page_changed || !is_console(out, &mode)) return;
    saved_output_code_page = GetConsoleOutputCP();
    if (SetConsoleOutputCP(CP_UTF8)) code_page_changed = 1;
}

void platform_init(void) {
    /* Without this a \n written to a file or a pipe becomes \r\n: a recording
     * is not the same bytes, and a cast has a \r on every line. */
    _setmode(_fileno(stdout), _O_BINARY);
    ensure_restore_lock();
    use_utf8_output();
    atexit(platform_leave_output);
}

/* ---- the clock and the pace ---- */

void platform_now(platform_time_t *now) {
    static LARGE_INTEGER frequency;
    LARGE_INTEGER counter;
    if (frequency.QuadPart == 0) QueryPerformanceFrequency(&frequency);
    QueryPerformanceCounter(&counter);
    now->tv_sec = counter.QuadPart / frequency.QuadPart;
    now->tv_nsec = (long)((counter.QuadPart % frequency.QuadPart) * 1000000000LL / frequency.QuadPart);
}

typedef BOOL(WINAPI *time_begin_period_function)(UINT);

/* A waitable timer made with the high resolution flag wakes within a fraction of
 * a millisecond (Windows 10 1803 and later). Without it, or where the flag is
 * refused, Sleep is as coarse as the system tick, so the tick is raised to one
 * millisecond for the life of the process with timeBeginPeriod, which lives in
 * winmm.dll and is looked up rather than linked so that nothing extra is needed
 * at build time. */
void platform_sleep_microseconds(long microseconds) {
    static HANDLE timer;
    static int tried;
    if (microseconds <= 0) return;
    if (!tried) {
        tried = 1;
        timer = CreateWaitableTimerExW(NULL, NULL, CREATE_WAITABLE_TIMER_HIGH_RESOLUTION,
                                       TIMER_ALL_ACCESS);
        if (timer == NULL) {
            HMODULE winmm = LoadLibraryW(L"winmm.dll");
            if (winmm != NULL) {
                time_begin_period_function begin =
                    (time_begin_period_function)(void (*)(void))GetProcAddress(winmm,
                                                                               "timeBeginPeriod");
                if (begin != NULL) begin(1);
            }
        }
    }
    if (timer != NULL) {
        LARGE_INTEGER due;
        due.QuadPart = -(LONGLONG)microseconds * 10; /* Negative: relative, in 100 ns. */
        if (SetWaitableTimer(timer, &due, 0, NULL, NULL, FALSE)) {
            WaitForSingleObject(timer, INFINITE);
            return;
        }
    }
    Sleep((DWORD)((microseconds + 999) / 1000));
}

/* ---- output ---- */

ssize_t platform_write(int fd, const void *data, size_t length) {
    /* WriteFile on the handle behind the descriptor: it is what a console, a
     * file and a pipe all take, and it is not the runtime's text mode. A console
     * is given modest pieces, and never cut inside a UTF-8 sequence. */
    enum { PIECE = 32768 };
    const unsigned char *bytes = data;
    HANDLE handle = (HANDLE)_get_osfhandle(fd);
    DWORD wrote = 0;
    size_t piece = length > PIECE ? PIECE : length;
    if (handle == INVALID_HANDLE_VALUE) {
        errno = EBADF;
        return -1;
    }
    if (halting && GetCurrentThreadId() != halting_thread) Sleep(INFINITE);
    while (piece < length && piece > 1 && (bytes[piece] & 0xC0) == 0x80) piece--;
    if (!WriteFile(handle, bytes, (DWORD)piece, &wrote, NULL)) {
        DWORD error = GetLastError();
        errno = (error == ERROR_BROKEN_PIPE || error == ERROR_NO_DATA) ? EPIPE : EIO;
        return -1;
    }
    return (ssize_t)wrote;
}

/* ---- the console's modes ---- */

int platform_enter_raw(void) {
    DWORD in_mode, out_mode, wanted;
    HANDLE in = GetStdHandle(STD_INPUT_HANDLE), out = GetStdHandle(STD_OUTPUT_HANDLE);
    if (!is_console(in, &in_mode) || !is_console(out, &out_mode)) {
        errno = ENOTTY;
        return -1;
    }
    input_handle = in;
    output_handle = out;
    saved_input_mode = in_mode;
    saved_output_mode = out_mode;

    /* Sequences out, and a line feed that only goes down a line, as on a Unix
     * terminal. The second flag is newer than the first and is allowed to fail. */
    wanted = out_mode | ENABLE_PROCESSED_OUTPUT | ENABLE_VIRTUAL_TERMINAL_PROCESSING;
    if (!SetConsoleMode(out, wanted | DISABLE_NEWLINE_AUTO_RETURN) &&
        !SetConsoleMode(out, wanted)) {
        errno = ENOTTY;
        return -1;
    }
    output_mode_changed = 1;
    use_utf8_output();

    /* No line input and no echo. Processed input stays on so that Ctrl-C is a
     * control event we can answer and not a character. Quick edit is off (and
     * extended flags on, or the old setting is kept) so that a click does not
     * freeze the program in a selection; the window and the mouse report as
     * records, and keys and the pointer as sequences where the console can. */
    wanted = ENABLE_PROCESSED_INPUT | ENABLE_WINDOW_INPUT | ENABLE_MOUSE_INPUT |
             ENABLE_EXTENDED_FLAGS;
    if (!SetConsoleMode(in, wanted | ENABLE_VIRTUAL_TERMINAL_INPUT) &&
        !SetConsoleMode(in, wanted)) {
        platform_leave_output();
        errno = ENOTTY;
        return -1;
    }
    input_mode_changed = 1;
    return 0;
}

void platform_leave_raw(void) {
    if (input_mode_changed) {
        input_mode_changed = 0;
        SetConsoleMode(input_handle, saved_input_mode);
        FlushConsoleInputBuffer(input_handle);
    }
}

void platform_leave_output(void) {
    if (code_page_changed) {
        code_page_changed = 0;
        SetConsoleOutputCP(saved_output_code_page);
    }
    if (output_mode_changed) {
        output_mode_changed = 0;
        SetConsoleMode(output_handle, saved_output_mode);
    }
}

void platform_report_no_terminal(void) {
    fputs("cbirds: needs a console to run in: Windows Terminal or the classic console, "
          "not a pipe, a file, or mintty (use winpty there). --record and --snapshot "
          "write a file from anywhere.\n",
          stderr);
}

void platform_window_size(int *columns, int *rows, int *pixel_width, int *pixel_height) {
    /* The visible window, not the buffer, which is thousands of lines tall in the
     * classic console. */
    CONSOLE_SCREEN_BUFFER_INFO info;
    HANDLE handle = output_handle != INVALID_HANDLE_VALUE ? output_handle
                                                          : GetStdHandle(STD_OUTPUT_HANDLE);
    *columns = *rows = *pixel_width = *pixel_height = 0;
    if (!GetConsoleScreenBufferInfo(handle, &info)) return;
    *columns = info.srWindow.Right - info.srWindow.Left + 1;
    *rows = info.srWindow.Bottom - info.srWindow.Top + 1;
}

/* ---- input ---- */

/* What the records have been turned into and not yet read. The reader asks for
 * a hundred bytes at a time; a paste can be more, and what does not fit stays
 * in the console's own queue until there is room. */
static unsigned char pending[1024];
static size_t pending_length, pending_at;
static uint16_t high_surrogate;
static unsigned held_buttons;
/* Whether the console has ever reported the pointer as a sequence. If it has it
 * will not also send records for the same movement, but the classic console
 * without that support sends only records; the first sequence settles it. */
static int pointer_is_sequences;
static unsigned char sequence_tail[3];

static void pending_add(const char *bytes, size_t count) {
    for (size_t i = 0; i < count && pending_length < sizeof(pending); i++)
        pending[pending_length++] = (unsigned char)bytes[i];
}

static void note_sequence_byte(unsigned char byte) {
    sequence_tail[0] = sequence_tail[1];
    sequence_tail[1] = sequence_tail[2];
    sequence_tail[2] = byte;
    if (sequence_tail[0] == 0x1b && sequence_tail[1] == '[' && sequence_tail[2] == '<')
        pointer_is_sequences = 1;
}

static void take_record(const INPUT_RECORD *record, const SMALL_RECT *window) {
    char bytes[128];
    switch (record->EventType) {
        case KEY_EVENT: {
            const KEY_EVENT_RECORD *key = &record->Event.KeyEvent;
            /* A key up carries nothing the key down did not; the sequences the
             * console writes for arrows and the like are all key downs. */
            if (!key->bKeyDown) break;
            unsigned repeat = key->wRepeatCount > 0 ? key->wRepeatCount : 1;
            if (repeat > 32) repeat = 32;
            for (unsigned r = 0; r < repeat; r++) {
                size_t count = platform_utf8_from_utf16(&high_surrogate,
                                                        (unsigned)key->uChar.UnicodeChar, bytes);
                pending_add(bytes, count);
                for (size_t i = 0; i < count; i++) note_sequence_byte((unsigned char)bytes[i]);
            }
            break;
        }
        case MOUSE_EVENT: {
            const MOUSE_EVENT_RECORD *mouse = &record->Event.MouseEvent;
            if (pointer_is_sequences) break;
            size_t count = platform_sgr_mouse(
                &held_buttons, (unsigned)(mouse->dwButtonState & 0xFFFF), mouse->dwEventFlags,
                (int)(short)(mouse->dwButtonState >> 16), mouse->dwMousePosition.X - window->Left,
                mouse->dwMousePosition.Y - window->Top, bytes, sizeof(bytes));
            pending_add(bytes, count);
            break;
        }
        default:
            /* A resize, a focus change, a menu: the size is read every frame, and
             * none of them is a key. Taken off the queue so the wait ends. */
            break;
    }
}

/* Reads every record that is waiting, without ever blocking. */
static void pump_input(void) {
    INPUT_RECORD records[32];
    SMALL_RECT window = {0, 0, 0, 0};
    int have_window = 0;
    if (pending_at > 0) { /* Keep the unread bytes at the front. */
        memmove(pending, pending + pending_at, pending_length - pending_at);
        pending_length -= pending_at;
        pending_at = 0;
    }
    for (;;) {
        DWORD available = 0, got = 0;
        /* A record makes up to four bytes and a repeat up to 32 of them; stop
         * with room to spare rather than lose a key. */
        if (sizeof(pending) - pending_length < 32 * 4 + 128) return;
        if (!GetNumberOfConsoleInputEvents(input_handle, &available) || available == 0) return;
        if (!ReadConsoleInputW(input_handle, records, 32, &got) || got == 0) return;
        for (DWORD i = 0; i < got; i++) {
            if (records[i].EventType == MOUSE_EVENT && !have_window) {
                CONSOLE_SCREEN_BUFFER_INFO info;
                if (GetConsoleScreenBufferInfo(output_handle, &info)) window = info.srWindow;
                have_window = 1;
            }
            take_record(&records[i], &window);
        }
    }
}

ssize_t platform_read_input(void *buffer, size_t size) {
    size_t count;
    if (input_handle == INVALID_HANDLE_VALUE) return 0;
    pump_input();
    count = pending_length - pending_at;
    if (count > size) count = size;
    memcpy(buffer, pending + pending_at, count);
    pending_at += count;
    return (ssize_t)count;
}

int platform_wait_input(int milliseconds) {
    ULONGLONG deadline = GetTickCount64() + (ULONGLONG)(milliseconds > 0 ? milliseconds : 0);
    if (input_handle == INVALID_HANDLE_VALUE) return 0;
    for (;;) {
        ULONGLONG now = GetTickCount64();
        DWORD wait_result;
        if (pending_length > pending_at) return 1;
        wait_result = WaitForSingleObject(input_handle,
                                          now >= deadline ? 0 : (DWORD)(deadline - now));
        if (wait_result == WAIT_OBJECT_0) {
            /* Woken for anything: a resize and a focus change are not keys, and
             * are drained here so that a reply that is not coming is not mistaken
             * for one that is. */
            pump_input();
            if (pending_length > pending_at) return 1;
            if (GetTickCount64() >= deadline) return 0;
            continue;
        }
        if (wait_result == WAIT_TIMEOUT) return 0;
        errno = EIO;
        return -1;
    }
}

int platform_wait_terminal_io(void) {
    /* A console write blocks until it is taken; there is no state to wait for. */
    return 0;
}

/* ---- being told to stop ---- */

static BOOL WINAPI console_event(DWORD type) {
    int signal_number;
    switch (type) {
        case CTRL_C_EVENT:
            signal_number = INTERRUPT_SIGNAL;
            break;
        case CTRL_BREAK_EVENT:
            signal_number = BREAK_SIGNAL;
            break;
        case CTRL_CLOSE_EVENT:
            signal_number = HANGUP_SIGNAL;
            break;
        case CTRL_LOGOFF_EVENT:
        case CTRL_SHUTDOWN_EVENT:
            signal_number = TERMINATE_SIGNAL;
            break;
        default:
            return FALSE;
    }
    /* This is a thread of its own, and the main thread may be in the middle of a
     * frame. Anything it writes from here on waits, the screen is put back, and
     * the process leaves with the status a shell would report for the signal. */
    halting_thread = GetCurrentThreadId();
    halting = 1;
    if (restore_hook != NULL) restore_hook();
    ExitProcess((UINT)(128 + signal_number));
    return TRUE;
}

static void crash_handler(int signal_number) {
    if (restore_hook != NULL) restore_hook();
    _Exit(128 + signal_number); /* C99's, so no header has to be found for it. */
}

void platform_install_exit_handlers(void (*restore)(void)) {
    static const int signals[] = {SIGSEGV, SIGFPE, SIGILL, SIGABRT, SIGTERM};
    restore_hook = restore;
    ensure_restore_lock();
    for (size_t i = 0; i < sizeof(signals) / sizeof(*signals); i++) signal(signals[i], crash_handler);
    SetConsoleCtrlHandler(console_event, TRUE);
}

#endif
