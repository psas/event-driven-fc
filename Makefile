# Compiler flag detection based on similar logic in Kbuild
try-run = $(shell if ($(1)) >/dev/null 2>&1; then echo '$(2)'; else echo '$(3)'; fi)
cc-option = $(call try-run,$(CC) $(1) -S -xc /dev/null -o /dev/null,$(1),$(2))

OPTS := -O3 -ffast-math $(call cc-option,-fwhole-program -combine)
WARNINGS := -Werror -Wall -Wextra -Wmissing-prototypes -Wwrite-strings
CFLAGS := -MD $(OPTS) $(WARNINGS)

TARGETS = z-sim coordtest

all: $(TARGETS)

ZIGGURAT_SOURCES = ziggurat/isaac.c ziggurat/random.c ziggurat/normal.c ziggurat/normal_tab.c ziggurat/polynomial.c ziggurat/polynomial_tab.c
SOURCES = z-sim.c flight-computer.c physics.c pressure_sensor.c sensors.c resample-optimal.c coord.c mat.c vec.c $(ZIGGURAT_SOURCES)

z-sim: $(SOURCES)
	$(CC) $(CFLAGS) $(SOURCES) -lm -o $@

ziggurat/normal_tab.c:
	make -C ziggurat normal_tab.c

ziggurat/polynomial_tab.c:
	make -C ziggurat polynomial_tab.c

COORDTEST_SOURCES = coord.c coordtest.c mat.c vec.c

coordtest: $(COORDTEST_SOURCES)
	$(CC) $(CFLAGS) $(COORDTEST_SOURCES) -lm -o $@

-include *.d

test: coordtest
	./coordtest

clean:
	make -C ziggurat clean
	rm -f $(TARGETS) *.d
