CC      = gcc
CFLAGS  = -Wall -Wextra -O3 -g
LDLIBS  = -lm
TARGET  = cbirds
SRC     = main.c

.PHONY: all clean run

all: $(TARGET)

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) $(SRC) -o $(TARGET) $(LDLIBS)

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET) *.o *~
