/* One frame of the spinning donut, in colour: a torus lit from above, drawn in
 * the characters .,-~:;=!*#$@ from dark to bright, each in a warm tint of its
 * brightness. After Andy Sloane's donut.c (2006).
 *
 *     cc donut.c -lm -o donut && ./donut | cbirds
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

enum { W = 80, H = 22 };

int main(void) {
    const char *shades = ".,-~:;=!*#$@";
    float depth[W * H];
    int light[W * H];
    double a = 0.6, b = 0.8; /* The two turns the frame is caught at. */
    memset(depth, 0, sizeof(depth));
    for (int i = 0; i < W * H; i++) light[i] = -1;

    for (double j = 0; j < 6.283; j += 0.05) {     /* Round the tube. */
        for (double i = 0; i < 6.283; i += 0.015) { /* Round the ring. */
            double sini = sin(i), cosj = cos(j), sina = sin(a), sinj = sin(j);
            double cosa = cos(a), h = cosj + 2, d = 1 / (sini * h * sina + sinj * cosa + 5);
            double cosi = cos(i), cosb = cos(b), sinb = sin(b);
            double t = sini * h * cosa - sinj * sina;
            int x = (int)(40 + 28 * d * (cosi * h * cosb - t * sinb));
            int y = (int)(12 + 14 * d * (cosi * h * sinb + t * cosb));
            int n = (int)(8 * ((sinj * sina - sini * cosj * cosa) * cosb - sini * cosj * sina -
                               sinj * cosa - cosi * cosj * sinb));
            int at = x + W * y;
            if (y >= 0 && y < H && x >= 0 && x < W && d > depth[at]) {
                depth[at] = (float)d;
                light[at] = n > 0 ? n : 0;
            }
        }
    }
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            int n = light[x + W * y];
            if (n < 0) {
                putchar(' ');
                continue;
            }
            /* From a deep red-brown in the shade to a pale gold in the light. */
            double s = n / 11.0;
            int r = (int)(150 + 105 * s), g = (int)(50 + 170 * s * s), bl = (int)(40 + 120 * s * s * s);
            printf("\033[38;2;%d;%d;%dm%c", r, g, bl, shades[n]);
        }
        printf("\033[0m\n");
    }
    return 0;
}
