/* firstframe IN.gif OUT.png: the first frame of a GIF as a PNG, as it is. */
#define main gifpeek_main
#include "gifpeek.c"
#undef main

int main(int argc, char **argv) {
    if (argc < 3) {
        fprintf(stderr, "firstframe IN.gif OUT.png\n");
        return 2;
    }
    gif_t g;
    memset(&g, 0, sizeof(g));
    load_file(argv[1], &g);
    const char *fatal = scan_gif(&g);
    if (fatal != NULL) die(1, "%s", fatal);
    player_t p;
    player_init(&p, &g);
    if (!player_step(&p)) return 1;
    png_image_t frame = {g.width, g.height, p.canvas};
    uint8_t *data = NULL;
    size_t length = 0;
    if (png_encode(&frame, &data, &length) != PNG_OK) return 1;
    FILE *f = fopen(argv[2], "wb");
    if (f == NULL || fwrite(data, 1, length, f) != length || fclose(f) != 0) return 1;
    free(data);
    printf("%s: %dx%d, %.1f KB\n", argv[2], g.width, g.height, length / 1024.0);
    return 0;
}
