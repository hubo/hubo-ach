default: all



CFLAGS := -I./include -g --std=gnu99
CXXFLAGS := -I./include -g -I/usr/local/include/eigen3

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

#BINARIES := hubo-daemon hubo-console hubo-loop hubo-read control-daemon 
BINARIES := hubo-daemon hubo-console hubo-read control-daemon test-plus home-test
all : $(BINARIES)

LIBS := -lach -lrt $(CAN_LIBS) -lsomatic

hubo_daemon_objs := src/hubo-daemonizer.o src/hubo-daemon.o src/hubo-jointparams.o $(CAN_OBJS)

hubo-daemon: $(hubo_daemon_objs)
	$(CC) -o $@  $(hubo_daemon_objs) $(LIBS) -lm -lc

hubo-read: src/hubo-read.c
	$(CC) $(CFLAGS) -o $@ $< -lach

control_daemon_objs := src/control-daemon.o src/hubo-jointparams.o src/daemonizer.o

control-daemon: $(control_daemon_objs)
	$(CC) $(CFLAGS) -o $@ $(control_daemon_objs) -lach -lm -lc

control_test_objs := src/daemonizer.o src/test-plus.o src/hubo_plus.o src/hubo-jointparams.o

test-plus: $(control_test_objs)
	$(CXX) $(CXXFLAGS) -o $@ $(control_test_objs) -lach -lm -lc

home_test_objs := src/daemonizer.o src/home-test.o src/hubo_plus.o src/hubo-jointparams.o

home-test: $(home_test_objs)
	$(CXX) $(CXXFLAGS) -o $@ $(home_test_objs) -lach -lm -lc

hubo_console_objs := src/hubo-jointparams.o src/hubo-console.o 

hubo-console: $(hubo_console_objs)
	$(CXX) $(CFLAGS) -o $@ $(hubo_console_objs) -lach -lreadline -lm -lc

hubo_loop_objs := src/hubo-jointparams.o src/hubo-loop.o

#hubo-loop: $(hubo_loop_objs)
#	$(CC) $(CFLAGS) -o $@ $(hubo_loop_objs) $(LIBS) -lprotobuf-c -lach -lamino

%.o: %.c
	$(CC) $(CFLAGS) -o $@ -c $<

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -o $@ -c $<


clean:
	rm -f $(BINARIES) src/*.o

