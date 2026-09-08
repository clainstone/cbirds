CC        = gcc
CFLAGS    = -Wall -Wextra -O3 -g
LDLIBS    = -lm
TARGET    = cbirds
SRC       = main.c png.c
HDR       = png.h sprite_png.h
ASSET     = sprite_png.h
ASSET_SRC = matrix.png
MKASSET   = mkasset

.PHONY: all clean run asset

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

clean:
	rm -f $(TARGET) $(MKASSET) *.o *~
