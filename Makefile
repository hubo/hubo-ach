default: all

CFLAGS := -I./include -g --std=gnu99
CC := gcc

BINARIES := hubo-simple-demo
all : $(BINARIES)

LIBS := -lach 

hubo-simple-demo: src/hubo-simple-demo.o
	gcc -o $@ $< $(LIBS)

%.o: %.c
	$(CC) $(CFLAGS) -o $@ -c $<

clean:
	rm -f $(BINARIES) src/*.o
