default: all

CFLAGS := -I./include -g --std=gnu99
CC := gcc

BINARIES := hubo-ach-multi-chan
all : $(BINARIES)

LIBS := -lach 

hubo-ach-multi-chan: src/hubo-ach-multi-chan.o
	gcc -o $@ $< $(LIBS)

%.o: %.c
	$(CC) $(CFLAGS) -o $@ -c $<

clean:
	rm -f $(BINARIES) src/*.o
