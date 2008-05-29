%.o: %.c
	gcc -Wall -Werror --std=gnu99 -Os -c -o $@ $<

clean:
	git clean

.PHONY: clean
