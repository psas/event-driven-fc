OBJECTS := control.o filter.o flightsim.o main.o query.o rocket.o sensor/pressure_sensor.o ziggurat/librandom.a

sim: ${OBJECTS}
	gcc -Wall -Werror --std=gnu99 -Os -o $@ $+ -lm

%.o: %.c
	gcc -Wall -Werror --std=gnu99 -Os -c -o $@ $<

ziggurat/%:
	make -C ziggurat $*

clean:
	git clean

.PHONY: clean
