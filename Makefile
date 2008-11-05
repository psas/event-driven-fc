ZIGGURAT_SOURCES = ziggurat/isaac.c ziggurat/random.c ziggurat/normal.c ziggurat/normal_tab.c ziggurat/polynomial.c ziggurat/polynomial_tab.c
SOURCES = z-sim.c flight-computer.c physics.c pressure_sensor.c resample-optimal.c mat.c vec.c $(ZIGGURAT_SOURCES)
ZIGGURAT_HEADERS = ziggurat/random.h
HEADERS = fc.h interface.h particle.h physics.h pressure_sensor.h resample.h mat.h vec.h gprob.h compiler.h $(ZIGGURAT_HEADERS)

# These flags are not supported by the OS X version of gcc (4.0.1).
#  Invoke with "make [targets] CFLAGS=" on the Mac.
CFLAGS = -fwhole-program -combine

TARGETS = z-sim coordtest

all: $(TARGETS)

z-sim: $(SOURCES) $(HEADERS)
	$(CC) -Wall -Werror -O3 -ffast-math $(CFLAGS) $(SOURCES) -lm -o $@

ziggurat/normal_tab.c:
	make -C ziggurat normal_tab.c

ziggurat/polynomial_tab.c:
	make -C ziggurat polynomial_tab.c

COORDTEST_SOURCES = coord.c coordtest.c mat.c vec.c
COORDTEST_HEADERS = vec.h mat.h coord.h compiler.h

coordtest: $(COORDTEST_SOURCES) $(COORDTEST_HEADERS)
	$(CC) -Wall -Werror -O3 -ffast-math $(CFLAGS) $(COORDTEST_SOURCES) -lm -o $@

test: coordtest
	./coordtest

clean:
	make -C ziggurat clean
	rm -f $(TARGETS)
