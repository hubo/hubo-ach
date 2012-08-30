default: hubo-main hubo-default


CFLAGS := -g

hubo_main_objs := hubo-main.o hubo-socketcan.o

hubo-main: $(hubo_main_objs)
	gcc -o $@  $(hubo_main_objs) -lach -lrt

hubo-default: hubo-default.c
	gcc -o $@ $< -lach -lrt

%.o: %.c
	gcc $(CFLAGS) -o $@ -c $<

clean:
	rm -f hubo-main hubo-default *.o
