#ifdef _WIN32
#error "placeholder"
#else
#include <stdio.h>
int main(void) {
    puts("console_test: skipped, it drives cbirds.exe through a Windows pseudo console");
    return 0;
}
#endif
