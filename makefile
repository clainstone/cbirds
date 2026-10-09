# The system's compiler and whatever flags a packager hands in. What the code
# needs in order to build at all is kept apart from them, so a CFLAGS of one's
# own replaces the optimisation and never the language standard.
CC       ?= cc
CFLAGS   ?= -O3 -g
BUILD     = -std=c99 -Wall -Wextra $(CPPFLAGS) $(CFLAGS)
LDLIBS    = -lm
TARGET    = cbirds
SRC       = boids.c cells.c fireflies.c font.c gif.c kitty_graphics.c letters.c options.c \
            picture.c png.c sign.c sky3d.c spatial_grid.c vt.c waves.c
HDR       = cells.h fireflies.h font.h gif.h kitty_graphics.h letters.h options.h picture.h \
            png.h sign.h sky3d.h spatial_grid.h sprite_png.h vt.h waves.h
ASSET     = sprite_png.h
ASSET_SRC = matrix.png
MKASSET   = mkasset
TESTDIR   = tests
TESTS     = $(TESTDIR)/kitty_graphics_test $(TESTDIR)/options_test $(TESTDIR)/png_test \
            $(TESTDIR)/gif_test $(TESTDIR)/cells_test $(TESTDIR)/spatial_grid_test \
            $(TESTDIR)/fireflies_test $(TESTDIR)/waves_test $(TESTDIR)/vt_test \
            $(TESTDIR)/letters_test $(TESTDIR)/sign_test $(TESTDIR)/picture_test \
            $(TESTDIR)/sky3d_test $(TESTDIR)/boids_test

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

$(TESTDIR)/cells_test: $(TESTDIR)/cells_test.c cells.c cells.h font.c font.h png.c png.h
	$(CC) $(BUILD) $@.c cells.c font.c png.c -o $@ $(LDFLAGS) $(LDLIBS)

$(TESTDIR)/spatial_grid_test: $(TESTDIR)/spatial_grid_test.c spatial_grid.c spatial_grid.h
	$(CC) $(BUILD) $@.c spatial_grid.c -o $@ $(LDFLAGS) $(LDLIBS)

$(TESTDIR)/fireflies_test: $(TESTDIR)/fireflies_test.c fireflies.c fireflies.h spatial_grid.c \
                            spatial_grid.h
	$(CC) $(BUILD) $@.c fireflies.c spatial_grid.c -o $@ $(LDFLAGS) $(LDLIBS)

$(TESTDIR)/waves_test: $(TESTDIR)/waves_test.c waves.c waves.h
	$(CC) $(BUILD) $@.c waves.c -o $@ $(LDFLAGS) $(LDLIBS)

$(TESTDIR)/vt_test: $(TESTDIR)/vt_test.c vt.c vt.h
	$(CC) $(BUILD) $@.c vt.c -o $@ $(LDFLAGS)

$(TESTDIR)/letters_test: $(TESTDIR)/letters_test.c letters.c letters.h cells.c cells.h font.c font.h vt.c vt.h png.c png.h
	$(CC) $(BUILD) $@.c letters.c cells.c font.c vt.c png.c -o $@ $(LDFLAGS) $(LDLIBS)

$(TESTDIR)/sign_test: $(TESTDIR)/sign_test.c sign.c sign.h font.c font.h
	$(CC) $(BUILD) $@.c sign.c font.c -o $@ $(LDFLAGS) $(LDLIBS)

$(TESTDIR)/picture_test: $(TESTDIR)/picture_test.c picture.c picture.h png.c png.h
	$(CC) $(BUILD) $@.c picture.c png.c -o $@ $(LDFLAGS) $(LDLIBS)

$(TESTDIR)/sky3d_test: $(TESTDIR)/sky3d_test.c sky3d.c sky3d.h
	$(CC) $(BUILD) $@.c sky3d.c -o $@ $(LDFLAGS) $(LDLIBS)

$(TESTDIR)/boids_test: $(TESTDIR)/boids_test.c $(SRC) $(HDR)
	$(CC) $(BUILD) $@.c cells.c fireflies.c font.c gif.c kitty_graphics.c letters.c options.c picture.c png.c sign.c sky3d.c spatial_grid.c vt.c waves.c -o $@ $(LDFLAGS) $(LDLIBS)

clean:
	rm -f $(TARGET) $(MKASSET) $(TESTS) *.o *~
