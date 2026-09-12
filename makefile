CC        = gcc
CFLAGS    = -Wall -Wextra -O3 -g
LDLIBS    = -lm
TARGET    = cbirds
SRC       = boids.c font.c kitty_graphics.c options.c png.c spatial_grid.c
HDR       = font.h kitty_graphics.h options.h png.h spatial_grid.h sprite_png.h
ASSET     = sprite_png.h
ASSET_SRC = matrix.png
MKASSET   = mkasset
TESTDIR   = tests
TESTS     = $(TESTDIR)/kitty_graphics_test $(TESTDIR)/options_test $(TESTDIR)/png_test \
            $(TESTDIR)/spatial_grid_test $(TESTDIR)/boids_test

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

# Each test includes what it needs with a ../ path, so it builds from anywhere
# without a search path of its own.
test: $(TESTS)
	@for t in $(TESTS); do echo "$$t"; ./$$t || exit 1; done

$(TESTDIR)/kitty_graphics_test: $(TESTDIR)/kitty_graphics_test.c kitty_graphics.c kitty_graphics.h
	$(CC) $(CFLAGS) $< kitty_graphics.c -o $@

$(TESTDIR)/options_test: $(TESTDIR)/options_test.c options.c options.h
	$(CC) $(CFLAGS) $< options.c -o $@

$(TESTDIR)/png_test: $(TESTDIR)/png_test.c png.c png.h
	$(CC) $(CFLAGS) $< png.c -o $@ $(LDLIBS)

$(TESTDIR)/spatial_grid_test: $(TESTDIR)/spatial_grid_test.c spatial_grid.c spatial_grid.h
	$(CC) $(CFLAGS) $< spatial_grid.c -o $@ $(LDLIBS)

$(TESTDIR)/boids_test: $(TESTDIR)/boids_test.c $(SRC) $(HDR)
	$(CC) $(CFLAGS) $< font.c kitty_graphics.c options.c png.c spatial_grid.c -o $@ $(LDLIBS)

clean:
	rm -f $(TARGET) $(MKASSET) $(TESTS) *.o *~
