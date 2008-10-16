ZIGGURAT_SOURCES = ziggurat/isaac.c ziggurat/random.c ziggurat/normal.c ziggurat/normal_tab.c ziggurat/polynomial.c ziggurat/polynomial_tab.c
SOURCES = z-sim.c flight-computer.c physics.c pressure_sensor.c resample-optimal.c mat.c $(ZIGGURAT_SOURCES)
ZIGGURAT_HEADERS = ziggurat/random.h
HEADERS = fc.h interface.h particle.h physics.h pressure_sensor.h resample.h mat.h vec.h gprob.h $(ZIGGURAT_HEADERS)
CFLAGS = -fwhole-program

all: z-sim

z-sim: $(SOURCES) $(HEADERS)
	$(CC) -Wall -Werror -O3 -ffast-math $(CFLAGS) -combine $(SOURCES) -lm -o $@

ziggurat/normal_tab.c:
	make -C ziggurat normal_tab.c

ziggurat/polynomial_tab.c:
	make -C ziggurat polynomial_tab.c

clean:
	make -C ziggurat clean
	rm -f z-sim
