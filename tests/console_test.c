/*
 * The real cbirds.exe, run in a Windows pseudo console (ConPTY) and driven the
 * way a terminal drives it: its output is read, keys and a resize are sent.
 *
 * Built everywhere; on anything but Windows main says it is skipped and returns
 * 0. The pseudo console is what Windows Terminal itself uses, so what passes
 * here is what a user in Windows Terminal gets; the classic console (conhost
 * with no terminal in front of it) takes the same API calls in the program and
 * is checked by hand. Every wait has a timeout and a failure ends the run with
 * the output so far printed, so a broken build fails instead of hanging.
 */
#ifndef _WIN32

#include <stdio.h>

int main(void) {
    puts("console_test: skipped, it drives cbirds.exe through a Windows pseudo console");
    return 0;
}

#else

#ifndef _WIN32_WINNT
#define _WIN32_WINNT 0x0A00
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <windows.h>

#ifndef PROC_THREAD_ATTRIBUTE_PSEUDOCONSOLE
#define PROC_THREAD_ATTRIBUTE_PSEUDOCONSOLE 0x00020016
#endif

/* The pseudo console calls are looked up in kernel32 rather than taken from the
 * headers, which differ in whether they have them and what they call the type:
 * a pseudo console is a handle. */
typedef HANDLE pseudo_console_t;
typedef HRESULT(WINAPI *create_pseudo_console_function)(COORD, HANDLE, HANDLE, DWORD,
                                                        pseudo_console_t *);
typedef HRESULT(WINAPI *resize_pseudo_console_function)(pseudo_console_t, COORD);
typedef void(WINAPI *close_pseudo_console_function)(pseudo_console_t);

static create_pseudo_console_function create_pseudo_console;
static resize_pseudo_console_function resize_pseudo_console;
static close_pseudo_console_function close_pseudo_console;

enum { SHORT_WAIT = 15000, LONG_WAIT = 30000 };

typedef struct {
    pseudo_console_t console;
    HANDLE keys;   /* Written to: what the terminal sends the program. */
    HANDLE screen; /* Read from: what the program draws. */
    HANDLE process;
    HANDLE reader;
    CRITICAL_SECTION lock;
    unsigned char *output;
    size_t length, capacity;
} session_t;

/* Without spaces in it: it is also put on a command line for cmd.exe, which
 * mishandles a quoted program name followed by redirections. */
static const char *executable(void) {
    const char *given = getenv("CBIRDS_EXE");
    return given != NULL && *given != '\0' ? given : ".\\cbirds.exe";
}

static void escaped(const unsigned char *bytes, size_t length) {
    for (size_t i = 0; i < length; i++) {
        if (bytes[i] == 0x1b)
            fputs("\\e", stderr);
        else if (bytes[i] >= 0x20 && bytes[i] < 0x7f)
            fputc(bytes[i], stderr);
        else
            fprintf(stderr, "\\x%02x", bytes[i]);
    }
    fputc('\n', stderr);
}

static void fail(session_t *session, const char *what, int line) {
    fprintf(stderr, "console_test: line %d: %s\n", line, what);
    if (session != NULL) {
        size_t from = session->length > 600 ? session->length - 600 : 0;
        fprintf(stderr, "the last of %lu bytes of output:\n", (unsigned long)session->length);
        escaped(session->output + from, session->length - from);
        if (session->process != NULL) TerminateProcess(session->process, 1);
    }
    fflush(stderr);
    ExitProcess(1);
}

#define EXPECT(session, condition)                               \
    do {                                                         \
        if (!(condition)) fail((session), #condition, __LINE__); \
    } while (0)

static void load_pseudo_console_api(void) {
    HMODULE kernel = GetModuleHandleW(L"kernel32.dll");
    FARPROC create = kernel ? GetProcAddress(kernel, "CreatePseudoConsole") : NULL;
    FARPROC resize = kernel ? GetProcAddress(kernel, "ResizePseudoConsole") : NULL;
    FARPROC closer = kernel ? GetProcAddress(kernel, "ClosePseudoConsole") : NULL;
    if (create == NULL || resize == NULL || closer == NULL) {
        fprintf(stderr, "console_test: this Windows has no pseudo console (needs 10 1809)\n");
        ExitProcess(1);
    }
    create_pseudo_console = (create_pseudo_console_function)(void (*)(void))create;
    resize_pseudo_console = (resize_pseudo_console_function)(void (*)(void))resize;
    close_pseudo_console = (close_pseudo_console_function)(void (*)(void))closer;
}

/* Where the program's output goes, for as long as there is any. It is read on a
 * thread of its own, because a full pipe stops the program and a test that is
 * waiting for the program would wait for ever. The console also asks where the
 * cursor is when it starts (ESC [ 6 n) and waits for the answer, as every
 * terminal gives it. */
static DWORD WINAPI read_screen(LPVOID argument) {
    session_t *session = argument;
    unsigned char chunk[4096];
    DWORD got;
    while (ReadFile(session->screen, chunk, sizeof(chunk), &got, NULL) && got > 0) {
        EnterCriticalSection(&session->lock);
        if (session->length + got + 1 > session->capacity) {
            size_t wanted = (session->length + got + 1) * 2;
            unsigned char *grown = realloc(session->output, wanted);
            if (grown == NULL) {
                LeaveCriticalSection(&session->lock);
                break;
            }
            session->output = grown;
            session->capacity = wanted;
        }
        memcpy(session->output + session->length, chunk, got);
        session->length += got;
        session->output[session->length] = 0;
        /* Looked for within a chunk: a request split across two is answered when
         * the console asks again. */
        for (DWORD i = 0; i + 3 < got; i++) {
            if (chunk[i] == 0x1b && chunk[i + 1] == '[' && chunk[i + 2] == '6' &&
                chunk[i + 3] == 'n') {
                DWORD wrote;
                WriteFile(session->keys, "\033[1;1R", 6, &wrote, NULL);
            }
        }
        LeaveCriticalSection(&session->lock);
    }
    return 0;
}

static void start(session_t *session, const char *arguments, int columns, int rows) {
    HANDLE keys_read = NULL, screen_write = NULL;
    STARTUPINFOEXW startup;
    PROCESS_INFORMATION process;
    SIZE_T attribute_size = 0;
    char narrow[1024];
    wchar_t wide[1024];
    COORD size;

    memset(session, 0, sizeof(*session));
    InitializeCriticalSection(&session->lock);
    EXPECT(NULL, CreatePipe(&keys_read, &session->keys, NULL, 0));
    EXPECT(NULL, CreatePipe(&session->screen, &screen_write, NULL, 0));
    size.X = (SHORT)columns;
    size.Y = (SHORT)rows;
    EXPECT(NULL,
           create_pseudo_console(size, keys_read, screen_write, 0, &session->console) == S_OK);
    CloseHandle(keys_read);
    CloseHandle(screen_write);

    memset(&startup, 0, sizeof(startup));
    startup.StartupInfo.cb = sizeof(startup);
    /* The program must take the pseudo console's handles and not the test's own
     * (which a CI runner has redirected to pipes): asking for standard handles,
     * and giving none, is how that is said. And nothing is inherited. */
    startup.StartupInfo.dwFlags = STARTF_USESTDHANDLES;
    startup.StartupInfo.hStdInput = NULL;
    startup.StartupInfo.hStdOutput = NULL;
    startup.StartupInfo.hStdError = NULL;
    InitializeProcThreadAttributeList(NULL, 1, 0, &attribute_size);
    startup.lpAttributeList = HeapAlloc(GetProcessHeap(), 0, attribute_size);
    EXPECT(NULL, startup.lpAttributeList != NULL);
    EXPECT(NULL,
           InitializeProcThreadAttributeList(startup.lpAttributeList, 1, 0, &attribute_size));
    EXPECT(NULL, UpdateProcThreadAttribute(startup.lpAttributeList, 0,
                                           PROC_THREAD_ATTRIBUTE_PSEUDOCONSOLE, session->console,
                                           sizeof(session->console), NULL, NULL));
    snprintf(narrow, sizeof(narrow), "\"%s\" %s", executable(), arguments);
    EXPECT(NULL, MultiByteToWideChar(CP_UTF8, 0, narrow, -1, wide, 1024) > 0);

    session->reader = CreateThread(NULL, 0, read_screen, session, 0, NULL);
    EXPECT(NULL, session->reader != NULL);
    memset(&process, 0, sizeof(process));
    if (!CreateProcessW(NULL, wide, NULL, NULL, FALSE, EXTENDED_STARTUPINFO_PRESENT, NULL, NULL,
                        &startup.StartupInfo, &process)) {
        fprintf(stderr, "console_test: cannot start %s (error %lu)\n", narrow,
                (unsigned long)GetLastError());
        ExitProcess(1);
    }
    CloseHandle(process.hThread);
    session->process = process.hProcess;
}

static size_t output_length(session_t *session) {
    size_t length;
    EnterCriticalSection(&session->lock);
    length = session->length;
    LeaveCriticalSection(&session->lock);
    return length;
}

static int contains(const unsigned char *haystack, size_t length, const void *needle,
                    size_t needle_length) {
    for (size_t i = 0; i + needle_length <= length; i++)
        if (memcmp(haystack + i, needle, needle_length) == 0) return 1;
    return 0;
}

/* Whether a byte string appears in what was drawn from `from` on. */
static int output_has(session_t *session, size_t from, const void *needle, size_t needle_length) {
    int found;
    EnterCriticalSection(&session->lock);
    found = from <= session->length &&
            contains(session->output + from, session->length - from, needle, needle_length);
    LeaveCriticalSection(&session->lock);
    return found;
}

/* A braille cell is U+2800 to U+28FF: E2 A0 80 to E2 A3 BF. */
static int output_has_braille(session_t *session, size_t from) {
    int found = 0;
    EnterCriticalSection(&session->lock);
    for (size_t i = from; i + 2 < session->length && !found; i++)
        found = session->output[i] == 0xE2 && session->output[i + 1] >= 0xA0 &&
                session->output[i + 1] <= 0xA3 && (session->output[i + 2] & 0xC0) == 0x80;
    LeaveCriticalSection(&session->lock);
    return found;
}

static int output_has_any_of(session_t *session, size_t from, const char *const *needles) {
    for (; *needles != NULL; needles++)
        if (output_has(session, from, *needles, strlen(*needles))) return 1;
    return 0;
}

static void wait_for_braille(session_t *session, size_t from, DWORD timeout) {
    DWORD waited = 0;
    while (!output_has_braille(session, from)) {
        Sleep(20);
        waited += 20;
        if (waited > timeout) fail(session, "no braille was drawn in time", __LINE__);
    }
}

static void wait_for_text(session_t *session, size_t from, const char *text, DWORD timeout) {
    DWORD waited = 0;
    while (!output_has(session, from, text, strlen(text))) {
        Sleep(20);
        waited += 20;
        if (waited > timeout) fail(session, text, __LINE__);
    }
}

static void wait_for_any(session_t *session, size_t from, const char *const *needles,
                         DWORD timeout) {
    DWORD waited = 0;
    while (!output_has_any_of(session, from, needles)) {
        Sleep(20);
        waited += 20;
        if (waited > timeout) fail(session, "the cells of that render were not drawn", __LINE__);
    }
}

static void send_keys(session_t *session, const char *keys) {
    DWORD wrote;
    EXPECT(session, WriteFile(session->keys, keys, (DWORD)strlen(keys), &wrote, NULL));
}

static DWORD wait_for_exit(session_t *session, DWORD timeout) {
    DWORD status = 0;
    if (WaitForSingleObject(session->process, timeout) != WAIT_OBJECT_0)
        fail(session, "the program did not end in time", __LINE__);
    GetExitCodeProcess(session->process, &status);
    return status;
}

static void release(session_t *session) {
    WaitForSingleObject(session->reader, 5000);
    CloseHandle(session->reader);
    CloseHandle(session->keys);
    CloseHandle(session->screen);
    CloseHandle(session->process);
    free(session->output);
    DeleteCriticalSection(&session->lock);
}

static void finish(session_t *session) {
    /* The console is closed while the reader is still draining it: closing it
     * waits for the last of the output to be taken. */
    close_pseudo_console(session->console);
    release(session);
}

/* The screen is put back: the alternate screen is left and the cursor shown. */
static void expect_the_screen_was_put_back(session_t *session) {
    wait_for_text(session, 0, "\033[?1049l", 5000);
    EXPECT(session, output_has(session, 0, "\033[?25h", 6));
}

/* ---- the tests ---- */

static void test_an_80_by_24_run_draws_braille_and_q_ends_it(void) {
    session_t session;
    start(&session, "--seed 1", 80, 24);
    wait_for_braille(&session, 0, SHORT_WAIT);
    /* It took the alternate screen to draw on. */
    EXPECT(&session, output_has(&session, 0, "\033[?1049h", 8));
    Sleep(300);
    send_keys(&session, "q");
    EXPECT(&session, wait_for_exit(&session, SHORT_WAIT) == 0);
    expect_the_screen_was_put_back(&session);
    finish(&session);
    puts("ok: an 80x24 run draws braille and q ends it with status 0");
}

static void test_ctrl_c_ends_it_with_130_and_puts_the_screen_back(void) {
    session_t session;
    start(&session, "--seed 2", 80, 24);
    wait_for_braille(&session, 0, SHORT_WAIT);
    /* 128 plus SIGINT, which is the status the POSIX handler leaves with. */
    send_keys(&session, "\003");
    EXPECT(&session, wait_for_exit(&session, SHORT_WAIT) == 130);
    expect_the_screen_was_put_back(&session);
    finish(&session);
    puts("ok: Ctrl-C ends it with status 130 and the screen is put back");
}

static void test_closing_the_console_ends_it(void) {
    session_t session;
    DWORD status;
    start(&session, "--seed 3", 80, 24);
    wait_for_braille(&session, 0, SHORT_WAIT);
    /* What closing a terminal window does to the program in it. The reader keeps
     * draining while the close waits for the last of the output. */
    close_pseudo_console(session.console);
    status = wait_for_exit(&session, SHORT_WAIT);
    release(&session);
    printf("ok: closing the console ended it (status %lu; 129 when it saw the close)\n",
           (unsigned long)status);
}

static void test_frames_ends_by_itself_with_status_0(void) {
    session_t session;
    start(&session, "--frames 40 --seed 4", 80, 24);
    EXPECT(&session, wait_for_exit(&session, SHORT_WAIT) == 0);
    EXPECT(&session, output_has_braille(&session, 0));
    expect_the_screen_was_put_back(&session);
    finish(&session);
    puts("ok: --frames 40 ends by itself with status 0");
}

/* The pace holds in a console. Three hundred frames are five seconds at sixty a
 * second, and a sleep rounded up to Windows' 15.6 ms tick would make them ten.
 * Start up (the colour questions the console never answers take a few tenths of
 * a second) is in the figure, so it reads a little under sixty. The number is
 * printed for the log; the bounds are wide because a CI machine is shared. */
static void test_the_frame_rate_holds_in_a_console(void) {
    session_t session;
    ULONGLONG began = GetTickCount64(), took;
    double rate;
    start(&session, "--frames 300 --seed 7", 80, 24);
    EXPECT(&session, wait_for_exit(&session, LONG_WAIT) == 0);
    took = GetTickCount64() - began;
    finish(&session);
    rate = 300.0 * 1000.0 / (double)took;
    printf("ok: 300 frames took %lu ms in a console, %.1f frames a second\n", (unsigned long)took,
           rate);
    EXPECT(NULL, rate > 40.0 && rate < 70.0);
}

static void test_sextants_and_blocks_are_drawn(void) {
    /* The sextants U+1FB00 to U+1FB3B are F0 9F AC 80 to F0 9F AC BB, and the
     * half blocks U+2580 and U+2584 are E2 96 80 and E2 96 84. */
    static const char *const SEXTANTS[] = {"\xF0\x9F\xAC", NULL};
    static const char *const BLOCKS[] = {"\xE2\x96\x80", "\xE2\x96\x84", NULL};
    session_t session;

    start(&session, "--render sextants --seed 5", 80, 24);
    wait_for_any(&session, 0, SEXTANTS, SHORT_WAIT);
    send_keys(&session, "q");
    EXPECT(&session, wait_for_exit(&session, SHORT_WAIT) == 0);
    finish(&session);

    start(&session, "--render blocks --seed 5", 80, 24);
    wait_for_any(&session, 0, BLOCKS, SHORT_WAIT);
    send_keys(&session, "q");
    EXPECT(&session, wait_for_exit(&session, SHORT_WAIT) == 0);
    finish(&session);
    puts("ok: --render sextants and --render blocks draw their cells");
}

static int read_file(const char *path, unsigned char **bytes, size_t *length) {
    FILE *file = fopen(path, "rb");
    long size;
    if (file == NULL) return 0;
    fseek(file, 0, SEEK_END);
    size = ftell(file);
    fseek(file, 0, SEEK_SET);
    *bytes = malloc((size_t)size + 1);
    *length = *bytes != NULL ? fread(*bytes, 1, (size_t)size, file) : 0;
    fclose(file);
    return *bytes != NULL && *length == (size_t)size;
}

static unsigned long be32(const unsigned char *at) {
    return (unsigned long)at[0] << 24 | (unsigned long)at[1] << 16 | (unsigned long)at[2] << 8 |
           at[3];
}

/* Whether, from `mark` on, the cursor was sent past column 80: ESC [ row ; col H
 * or ESC [ col G or ESC [ n C. What a console writes to draw a wider window. */
static int drew_past_column_80(session_t *session, size_t mark) {
    int past = 0;
    EnterCriticalSection(&session->lock);
    for (size_t i = mark; i + 3 < session->length && !past; i++) {
        int first = 0, second = 0, in_second = 0;
        size_t j = i + 2;
        if (session->output[i] != 0x1b || session->output[i + 1] != '[') continue;
        for (; j < session->length && j < i + 16; j++) {
            unsigned char c = session->output[j];
            if (c >= '0' && c <= '9') {
                if (in_second)
                    second = second * 10 + (c - '0');
                else
                    first = first * 10 + (c - '0');
            } else if (c == ';') {
                in_second = 1;
            } else {
                break;
            }
        }
        if (j >= session->length) break;
        if ((session->output[j] == 'H' || session->output[j] == 'f') && second > 80) past = 1;
        if ((session->output[j] == 'G' || session->output[j] == 'C') && first > 80) past = 1;
    }
    LeaveCriticalSection(&session->lock);
    return past;
}

/* A resize to a wider and taller window is seen: the snapshot a run writes at
 * its end is the size of the screen it last had, so its width and height are the
 * new window's in cells times the cell size the program assumes (8 by 16). */
static void test_a_resize_is_followed(void) {
    session_t session;
    unsigned char *png;
    size_t length, mark, at = 8;
    int ended = 0;
    COORD size;

    size.X = 120;
    size.Y = 30;
    remove("console_test_resize.png");
    start(&session, "--frames 420 --seed 6 --snapshot console_test_resize.png", 80, 24);
    wait_for_braille(&session, 0, SHORT_WAIT);
    Sleep(500);
    mark = output_length(&session);
    EXPECT(&session, resize_pseudo_console(session.console, size) == S_OK);
    Sleep(800);
    EXPECT(&session, drew_past_column_80(&session, mark));
    EXPECT(&session, wait_for_exit(&session, LONG_WAIT) == 0);
    finish(&session);

    EXPECT(NULL, read_file("console_test_resize.png", &png, &length));
    EXPECT(NULL, length > 33 && memcmp(png, "\x89PNG\r\n\x1a\n", 8) == 0);
    EXPECT(NULL, be32(png + 16) == 120UL * 8);
    EXPECT(NULL, be32(png + 20) == 30UL * 16);
    /* Chunks end exactly at IEND, with nothing after it. */
    while (at + 12 <= length && !ended) {
        unsigned long chunk = be32(png + at);
        ended = memcmp(png + at + 4, "IEND", 4) == 0;
        at += 12 + chunk;
    }
    EXPECT(NULL, ended && at == length);
    free(png);
    remove("console_test_resize.png");
    puts("ok: a resize to 120x30 is followed, and the snapshot is a whole PNG of that size");
}

/* ---- files that are written ---- */

static void run(const char *command) {
    if (system(command) != 0) {
        fprintf(stderr, "console_test: %s failed\n", command);
        ExitProcess(1);
    }
}

/* A GIF as its blocks: the header, the screen, the colour table, then
 * extensions and images until the trailer, which is the last byte of the file. */
static int gif_is_whole(const unsigned char *gif, size_t length) {
    size_t at = 13;
    int images = 0;
    if (length < 14 || memcmp(gif, "GIF89a", 6) != 0) return 0;
    if (gif[10] & 0x80) at += (size_t)3 << ((gif[10] & 7) + 1);
    while (at < length) {
        if (gif[at] == 0x3B) return at + 1 == length && images > 0;
        if (gif[at] == 0x21) {
            at += 2;
        } else if (gif[at] == 0x2C) {
            if (at + 11 > length) return 0;
            images++;
            at += 10; /* The descriptor; no local table. */
            if (gif[at - 1] & 0x80) return 0;
            at += 1; /* The minimum code size. */
        } else {
            return 0;
        }
        while (at < length && gif[at] != 0) at += 1u + gif[at]; /* Sub-blocks. */
        at += 1;
    }
    return 0;
}

static void test_recordings_are_exact(void) {
    unsigned char *first, *second, *cast, *again;
    size_t first_length, second_length, cast_length, again_length;
    char command[512];

    snprintf(command, sizeof(command),
             "%s --record console_test_a.gif --record-seconds 2 --seed 5 --hawks 2 > NUL",
             executable());
    run(command);
    snprintf(command, sizeof(command),
             "%s --record console_test_b.gif --record-seconds 2 --seed 5 --hawks 2 > NUL",
             executable());
    run(command);
    EXPECT(NULL, read_file("console_test_a.gif", &first, &first_length));
    EXPECT(NULL, read_file("console_test_b.gif", &second, &second_length));
    /* The same seed, the same bytes. */
    EXPECT(NULL, first_length == second_length && memcmp(first, second, first_length) == 0);
    EXPECT(NULL, gif_is_whole(first, first_length));
    free(first);
    free(second);
    remove("console_test_a.gif");
    remove("console_test_b.gif");

    snprintf(command, sizeof(command),
             "%s --record console_test_a.cast --record-seconds 2 --seed 5 > NUL", executable());
    run(command);
    snprintf(command, sizeof(command),
             "%s --record console_test_b.cast --record-seconds 2 --seed 5 > NUL", executable());
    run(command);
    EXPECT(NULL, read_file("console_test_a.cast", &cast, &cast_length));
    EXPECT(NULL, read_file("console_test_b.cast", &again, &again_length));
    /* No carriage return anywhere: a cast is text with \n line ends on every
     * system. The timestamp in the first line is the only thing that may differ,
     * so the comparison starts at the second. */
    EXPECT(NULL, memchr(cast, '\r', cast_length) == NULL);
    EXPECT(NULL, cast_length > 2 && cast[cast_length - 1] == '\n');
    {
        const unsigned char *a = memchr(cast, '\n', cast_length);
        const unsigned char *b = memchr(again, '\n', again_length);
        EXPECT(NULL, a != NULL && b != NULL);
        EXPECT(NULL, (cast + cast_length) - a == (again + again_length) - b &&
                         memcmp(a, b, (size_t)((cast + cast_length) - a)) == 0);
    }
    free(cast);
    free(again);
    remove("console_test_a.cast");
    remove("console_test_b.cast");
    puts("ok: recordings are exact: no \\r in a cast, a whole GIF, the same seed the same bytes");
}

static void test_headless_modes_print_what_they_should(void) {
    char command[512];
    unsigned char *text;
    size_t length;

    snprintf(command, sizeof(command), "%s --version > console_test_version.txt", executable());
    run(command);
    EXPECT(NULL, read_file("console_test_version.txt", &text, &length));
    EXPECT(NULL, length >= 7 && memcmp(text, "cbirds ", 7) == 0);
    /* One \n and no \r: stdout is binary. */
    EXPECT(NULL, memchr(text, '\r', length) == NULL && text[length - 1] == '\n');
    free(text);
    remove("console_test_version.txt");

    snprintf(command, sizeof(command), "%s --bench 30 --render braille > NUL", executable());
    run(command);
    puts("ok: --version prints cbirds and a bare newline, --bench runs");
}

int main(void) {
    load_pseudo_console_api();
    test_recordings_are_exact();
    test_headless_modes_print_what_they_should();
    test_an_80_by_24_run_draws_braille_and_q_ends_it();
    test_frames_ends_by_itself_with_status_0();
    test_the_frame_rate_holds_in_a_console();
    test_sextants_and_blocks_are_drawn();
    test_ctrl_c_ends_it_with_130_and_puts_the_screen_back();
    test_a_resize_is_followed();
    test_closing_the_console_ends_it();
    puts("console_test: all passed");
    return 0;
}

#endif
