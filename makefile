CC        = gcc
CFLAGS    = -Wall -Wextra -O3 -g
LDLIBS    = -lm
TARGET    = cbirds
SRC       = boids.c kitty_graphics.c png.c
HDR       = kitty_graphics.h png.h sprite_png.h
ASSET     = sprite_png.h
ASSET_SRC = matrix.png
MKASSET   = mkasset
TEST      = kitty_graphics_test

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

test: $(TEST)
	./$(TEST)

$(TEST): kitty_graphics_test.c kitty_graphics.c kitty_graphics.h
	$(CC) $(CFLAGS) kitty_graphics_test.c kitty_graphics.c -o $(TEST)

clean:
	rm -f $(TARGET) $(MKASSET) $(TEST) *.o *~
