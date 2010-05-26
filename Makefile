# Compiler flag detection based on similar logic in Kbuild
try-run = $(shell if ($(1)) >/dev/null 2>&1; then echo '$(2)'; else echo '$(3)'; fi)
cc-option = $(call try-run,$(CC) $(1) -S -xc /dev/null -o /dev/null,$(1),$(2))

OPTS := -O3 -ffast-math 
#$(call cc-option,-fwhole-program -combine)
WARNINGS := -Werror -Wall -Wextra -Wmissing-prototypes -Wwrite-strings
CFLAGS := -g -MD -std=gnu99 $(OPTS) $(WARNINGS) -fno-strict-aliasing

TARGETS = sim lv2log coordtest gpstest

all: $(TARGETS)

ZIGGURAT_SOURCES = ziggurat/isaac.c ziggurat/random.c ziggurat/normal.c ziggurat/normal_tab.c ziggurat/polynomial.c ziggurat/polynomial_tab.c
FC_SOURCES = flight-computer.c physics.c pressure_sensor.c sensors.c resample-optimal.c coord.c mat.c vec.c spherical_harmonics.c $(ZIGGURAT_SOURCES)
ZSIM_SOURCES = sim.c sim-common.c $(FC_SOURCES)

sim: $(ZSIM_SOURCES) Makefile data_WMM.h
	$(CC) $(CFLAGS) $(ZSIM_SOURCES) -lm -o $@

ziggurat/normal_tab.c:
	make -C ziggurat normal_tab.c

ziggurat/polynomial_tab.c:
	make -C ziggurat polynomial_tab.c

LV2LOG_SOURCES = lv2log.c gps.c sim-common.c $(FC_SOURCES)

lv2log: $(LV2LOG_SOURCES)
	$(CC) $(CFLAGS) $(LV2LOG_SOURCES) -lm -o $@

COORDTEST_SOURCES = coord.c coordtest.c mat.c vec.c

coordtest: $(COORDTEST_SOURCES)
	$(CC) $(CFLAGS) $(COORDTEST_SOURCES) -lm -o $@

GPSTEST_SOURCES = gpstest.c gps.c

gpstest: $(GPSTEST_SOURCES)
	$(CC) $(CFLAGS) $(GPSTEST_SOURCES) -lm -o $@

-include *.d

test: coordtest
	./coordtest

data_WMM.h: mag_data env_data/WMM2010.COF
	./mag_data < env_data/WMM2010.COF > $@

data_EMM.h: mag_data env_data/EMM2010.COF
	./EMM_data < env_data/EMM2010.COF > $@

data_EGM84.h: grav_data env_data/egm180.nor
	./grav_data < env_data/egm180.nor > $@

clean:
	make -C ziggurat clean
	rm -f $(TARGETS) *.d
