default: all

all: hubo-main hubo-default

CFLAGS := -g --std=gnu99


# SOCKETCAN #
CAN_LIBS :=
CAN_OBJS := hubo-socketcan.o
CAN_DEFS :=

# esd CAN #
# CAN_LIBS := -lntcan
# CAN_OBJS := hubo-esdcan.o
# CAN_DEFS := -DHUBO_CONFIG_ESD


LIBS := -lach -lrt $(CAN_LIBS)

hubo_main_objs := hubo-main.o $(CAN_OBJS)

hubo-main: $(hubo_main_objs)
	gcc -o $@  $(hubo_main_objs) $(LIBS)

hubo-default: hubo-default.c
	gcc -o $@ $< -lach -lrt

%.o: %.c
	gcc $(CFLAGS) -o $@ -c $<

clean:
	rm -f hubo-main hubo-default *.o
