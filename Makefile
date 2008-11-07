ZIGGURAT_SOURCES = ziggurat/isaac.c ziggurat/random.c ziggurat/normal.c ziggurat/normal_tab.c ziggurat/polynomial.c ziggurat/polynomial_tab.c
SOURCES = z-sim.c flight-computer.c physics.c pressure_sensor.c sensors.c resample-optimal.c coord.c mat.c vec.c $(ZIGGURAT_SOURCES)
ZIGGURAT_HEADERS = ziggurat/random.h
HEADERS = fc.h interface.h particle.h physics.h pressure_sensor.h sensors.h resample.h coord.h mat.h vec.h gprob.h compiler.h $(ZIGGURAT_HEADERS)

# These flags are not supported by the OS X version of gcc (4.0.1).
#  Invoke with "make [targets] CFLAGS=" on the Mac.
CFLAGS = -fwhole-program -combine

OPTS = -O3 -ffast-math
WARNINGS = -Werror -Wall -Wextra -Wmissing-prototypes

TARGETS = z-sim coordtest

all: $(TARGETS)

z-sim: $(SOURCES) $(HEADERS)
	$(CC) $(WARNINGS) $(OPTS) $(CFLAGS) $(SOURCES) -lm -o $@

ziggurat/normal_tab.c:
	make -C ziggurat normal_tab.c

ziggurat/polynomial_tab.c:
	make -C ziggurat polynomial_tab.c

COORDTEST_SOURCES = coord.c coordtest.c mat.c vec.c
COORDTEST_HEADERS = vec.h mat.h coord.h compiler.h

coordtest: $(COORDTEST_SOURCES) $(COORDTEST_HEADERS)
	$(CC) $(WARNINGS) $(OPTS) $(CFLAGS) $(COORDTEST_SOURCES) -lm -o $@

test: coordtest
	./coordtest

clean:
	make -C ziggurat clean
	rm -f $(TARGETS)
