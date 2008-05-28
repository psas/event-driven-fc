ZIGGURAT_SOURCES = ziggurat/isaac.c ziggurat/random.c ziggurat/normal.c ziggurat/normal_tab.c ziggurat/polynomial.c ziggurat/polynomial_tab.c
SOURCES = z-sim.c flight-computer.c physics.c pressure_sensor.c resample-optimal.c $(ZIGGURAT_SOURCES)

all: z-sim

z-sim: $(SOURCES) fc.h interface.h particle.h physics.h pressure_sensor.h resample.h vec.h gprob.h ziggurat/random.h
	$(CC) -Wall -Werror -O3 -ffast-math -fwhole-program -combine $(SOURCES) -lm -o $@

ziggurat/normal_tab.c:
	make -C ziggurat normal_tab.c

ziggurat/polynomial_tab.c:
	make -C ziggurat polynomial_tab.c

clean:
	make -C ziggurat clean
	rm -f z-sim
