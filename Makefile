SOURCES = z-sim.c flight-computer.c physics.c

all: z-sim

z-sim: $(SOURCES) fc.h interface.h physics.h vec.h
	$(CC) -Wall -Werror -O3 -ffast-math -fwhole-program -combine $(SOURCES) -lm -o $@

clean:
	rm -f z-sim
