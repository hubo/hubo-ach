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

BINARIES := hubo-daemon hubo-console hubo-loop hubo-read hubo-ref-filter
all : $(BINARIES)

LIBS := -lach -lrt $(CAN_LIBS)

hubo_daemon_objs := src/hubo-daemonizer.o src/hubo-daemon.o src/hubo-jointparams.o $(CAN_OBJS)
hubo_main_objs := src/hubo-main.o src/hubo-jointparams.o $(CAN_OBJS)

hubo-daemon: $(hubo_daemon_objs)
	$(CC) -o $@  $(hubo_daemon_objs) $(LIBS) -lm -lc

hubo_read_objs := src/hubo-jointparams.o
hubo-read: src/hubo-read.c
	$(CC) $(CFLAGS) -o $@ $(hubo_read_objs) $< -lach

hubo_ref_filter_objs := src/hubo-jointparams.o
hubo-ref-filter: src/hubo-ref-filter.c
	$(CC) $(CFLAGS) -o $@ $(hubo_ref_filter_objs) $< -lach

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

