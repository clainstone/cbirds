CC=gcc
CFLAGS=-Wall -O3 -g -Wextra -lm
DEPS =	
OBJ = main.o

clean:
	rm -f *.o main
	rm -f *~

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

main : $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS)
