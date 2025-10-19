CC=gcc
CFLAGS=-Wall -O3 -g -Wextra
DEPS =	
OBJ = main.o

clean:
	rm -f *.o main
	rm -f *~

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

cboids : $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS)
