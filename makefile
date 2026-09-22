# The system's compiler and whatever flags a packager hands in. What the code
# needs in order to build at all is kept apart from them, so a CFLAGS of one's
# own replaces the optimisation and never the language standard.
CC       ?= cc
CFLAGS   ?= -O3 -g
BUILD     = -std=c99 -Wall -Wextra $(CPPFLAGS) $(CFLAGS)
LDLIBS    = -lm
TARGET    = cbirds
SRC       = boids.c cells.c font.c gif.c kitty_graphics.c options.c png.c spatial_grid.c
HDR       = cells.h font.h gif.h kitty_graphics.h options.h png.h spatial_grid.h sprite_png.h
ASSET     = sprite_png.h
ASSET_SRC = matrix.png
MKASSET   = mkasset
TESTDIR   = tests
TESTS     = $(TESTDIR)/kitty_graphics_test $(TESTDIR)/options_test $(TESTDIR)/png_test \
            $(TESTDIR)/gif_test $(TESTDIR)/cells_test $(TESTDIR)/spatial_grid_test \
            $(TESTDIR)/boids_test

PREFIX   ?= /usr/local
BINDIR    = $(DESTDIR)$(PREFIX)/bin

.PHONY: all clean run asset test install uninstall

all: $(TARGET)

install: $(TARGET)
	mkdir -p $(BINDIR)
	install -m 755 $(TARGET) $(BINDIR)/$(TARGET)

uninstall:
	rm -f $(BINDIR)/$(TARGET)

$(TARGET): $(SRC) $(HDR)
	$(CC) $(BUILD) $(SRC) -o $(TARGET) $(LDFLAGS) $(LDLIBS)

run: $(TARGET)
	./$(TARGET)

# Regenerates the embedded sprite from the original artwork
asset: $(MKASSET)
	./$(MKASSET) $(ASSET_SRC) $(ASSET) sprite_png

$(MKASSET): mkasset.c png.c png.h
	$(CC) $(BUILD) mkasset.c png.c -o $(MKASSET) $(LDFLAGS) $(LDLIBS)

# Each test includes what it needs with a ../ path, so it builds from anywhere
# without a search path of its own. Its source is named $@.c rather than $<,
# which BSD make leaves empty outside suffix rules.
test: $(TESTS)
	@for t in $(TESTS); do echo "$$t"; ./$$t || exit 1; done

$(TESTDIR)/kitty_graphics_test: $(TESTDIR)/kitty_graphics_test.c kitty_graphics.c kitty_graphics.h
	$(CC) $(BUILD) $@.c kitty_graphics.c -o $@ $(LDFLAGS)

$(TESTDIR)/options_test: $(TESTDIR)/options_test.c options.c options.h
	$(CC) $(BUILD) $@.c options.c -o $@ $(LDFLAGS)

$(TESTDIR)/png_test: $(TESTDIR)/png_test.c png.c png.h
	$(CC) $(BUILD) $@.c png.c -o $@ $(LDFLAGS) $(LDLIBS)

$(TESTDIR)/gif_test: $(TESTDIR)/gif_test.c gif.c gif.h png.c png.h
	$(CC) $(BUILD) $@.c gif.c png.c -o $@ $(LDFLAGS) $(LDLIBS)

$(TESTDIR)/cells_test: $(TESTDIR)/cells_test.c cells.c cells.h png.c png.h
	$(CC) $(BUILD) $@.c cells.c png.c -o $@ $(LDFLAGS) $(LDLIBS)

$(TESTDIR)/spatial_grid_test: $(TESTDIR)/spatial_grid_test.c spatial_grid.c spatial_grid.h
	$(CC) $(BUILD) $@.c spatial_grid.c -o $@ $(LDFLAGS) $(LDLIBS)

$(TESTDIR)/boids_test: $(TESTDIR)/boids_test.c $(SRC) $(HDR)
	$(CC) $(BUILD) $@.c cells.c font.c gif.c kitty_graphics.c options.c png.c spatial_grid.c -o $@ $(LDFLAGS) $(LDLIBS)

clean:
	rm -f $(TARGET) $(MKASSET) $(TESTS) *.o *~
