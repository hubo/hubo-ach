default: all



CFLAGS := -I./include -g --std=gnu99
CXXFLAGS := -I./include -g

CC := gcc
CXX := g++

# SOCKETCAN #
CAN_LIBS :=
CAN_OBJS := src/hubo-socketcan.o
CAN_DEFS :=

# esd CAN #
# CAN_LIBS := -lntcan
# CAN_OBJS := src/hubo-esdcan.o
# CAN_DEFS := -DHUBO_CONFIG_ESD

BINARIES := hubo-main hubo-console hubo-loop hubo-read
all : $(BINARIES)

LIBS := -lach -lrt $(CAN_LIBS)

hubo_main_objs := src/hubo-main.o src/hubo-jointparams.o $(CAN_OBJS)

hubo-main: $(hubo_main_objs)
	$(CC) -o $@  $(hubo_main_objs) $(LIBS) -lm -lc

hubo-read: src/hubo-read.c
	$(CC) $(CFLAGS) -o $@ $< -lach

hubo_console_objs := src/hubo-jointparams.o src/hubo-console.o 

hubo-console: $(hubo_console_objs)
	$(CXX) $(CFLAGS) -o $@ $(hubo_console_objs) -lach -lreadline -lm -lc

hubo_loop_objs := src/hubo-jointparams.o src/hubo-loop.o

hubo-loop: $(hubo_loop_objs)
	$(CC) $(CFLAGS) -o $@ $(hubo_loop_objs) -lach -lrt -lm -lc

%.o: %.c
	$(CC) $(CFLAGS) -o $@ -c $<

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -o $@ -c $<


clean:
	rm -f $(BINARIES) src/*.o

