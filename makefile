CC        = gcc
CFLAGS    = -Wall -Wextra -O3 -g
LDLIBS    = -lm
TARGET    = cbirds
SRC       = boids.c kitty_graphics.c png.c spatial_grid.c
HDR       = kitty_graphics.h png.h spatial_grid.h sprite_png.h
ASSET     = sprite_png.h
ASSET_SRC = matrix.png
MKASSET   = mkasset
TESTS     = kitty_graphics_test spatial_grid_test boids_test

.PHONY: all clean run asset test

all: $(TARGET)

$(TARGET): $(SRC) $(HDR)
	$(CC) $(CFLAGS) $(SRC) -o $(TARGET) $(LDLIBS)

run: $(TARGET)
	./$(TARGET)

# Regenerates the embedded sprite from the original artwork
asset: $(MKASSET)
	./$(MKASSET) $(ASSET_SRC) $(ASSET) sprite_png

$(MKASSET): mkasset.c png.c png.h
	$(CC) $(CFLAGS) mkasset.c png.c -o $(MKASSET) $(LDLIBS)

test: $(TESTS)
	./kitty_graphics_test
	./spatial_grid_test
	./boids_test

kitty_graphics_test: kitty_graphics_test.c kitty_graphics.c kitty_graphics.h
	$(CC) $(CFLAGS) kitty_graphics_test.c kitty_graphics.c -o kitty_graphics_test

spatial_grid_test: spatial_grid_test.c spatial_grid.c spatial_grid.h
	$(CC) $(CFLAGS) spatial_grid_test.c spatial_grid.c -o spatial_grid_test $(LDLIBS)

boids_test: boids_test.c boids.c kitty_graphics.c kitty_graphics.h png.c png.h spatial_grid.c spatial_grid.h sprite_png.h
	$(CC) $(CFLAGS) boids_test.c kitty_graphics.c png.c spatial_grid.c -o boids_test $(LDLIBS)

clean:
	rm -f $(TARGET) $(MKASSET) $(TESTS) *.o *~
