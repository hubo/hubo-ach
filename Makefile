default: all

all: hubo-main hubo-default

CFLAGS := -I./include -g --std=gnu99


# SOCKETCAN #
CAN_LIBS :=
CAN_OBJS := src/hubo-socketcan.o
CAN_DEFS :=

# esd CAN #
# CAN_LIBS := -lntcan
# CAN_OBJS := src/hubo-esdcan.o
# CAN_DEFS := -DHUBO_CONFIG_ESD


LIBS := -lach -lrt $(CAN_LIBS)

hubo_main_objs := src/hubo-main.o $(CAN_OBJS)

hubo-main: $(hubo_main_objs)
	gcc -o $@  $(hubo_main_objs) $(LIBS)

hubo-default: src/hubo-default.c
	gcc $(CFLAGS) -o $@ $< -lach -lrt

%.o: %.c
	gcc $(CFLAGS) -o $@ -c $<

clean:
	rm -f hubo-main hubo-default src/*.o
