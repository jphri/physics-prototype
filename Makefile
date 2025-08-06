.PHONY: all clean

all: a.out

clean:
	rm -f a.out

a.out: main.c util.c
	$(CC) -O3 $^ -lSDL2 -lm -o $@

