#include <stdio.h>
#include <sys/ioctl.h>

int main(int argc, char **argv) {
    struct winsize sz;
    ioctl(0, TIOCGWINSZ, &sz);
    printf(
        "number of rows: %i, number of columns: %i, screen width: %i, screen height: %i\n",
        sz.ws_row, sz.ws_col, sz.ws_xpixel, sz.ws_ypixel);
    return 0;
}
