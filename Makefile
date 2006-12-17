SOURCES = z-sim.c flight-computer.c

all: z-sim

z-sim: $(SOURCES) fc.h interface.h
	gcc -Wall -Werror -O3 -ffast-math -fwhole-program -combine $(SOURCES) -o $@

clean:
	rm -f z-sim
