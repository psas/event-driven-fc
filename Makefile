SOURCES = z-sim.c flight-computer.c

all: z-sim

z-sim: $(SOURCES) fc.h interface.h
	$(CC) -Wall -Werror -O3 -ffast-math -fwhole-program -combine $(SOURCES) -lm -o $@

clean:
	rm -f z-sim
