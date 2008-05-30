OBJECTS := control.o filter.o flightsim.o main.o query.o rocket.o sensor/pressure_sensor.o ziggurat/librandom.a

sim: ${OBJECTS}
	gcc -Wall -Werror --std=gnu99 -O3 -o $@ $+ -lm

%.o: %.c
	gcc -Wall -Werror --std=gnu99 -O3 -c -o $@ $<

ziggurat/%:
	make -C ziggurat $*

clean:
	git clean
	make -C ziggurat clean

.PHONY: clean
