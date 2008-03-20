SOURCES = z-sim.c flight-computer.c physics.c pressure_sensor.c

all: z-sim

z-sim: $(SOURCES) fc.h interface.h physics.h pressure_sensor.h vec.h
	$(CC) -Wall -Werror -O3 -ffast-math -fwhole-program -combine $(SOURCES) -lm -o $@

clean:
	rm -f z-sim
