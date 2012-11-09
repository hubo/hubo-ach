#!/bin/bash

# mxGrey 10/25/2012




HUBO_BITRATE=0x0014 # 1 Mbit/s
#HUBO_BITRATE=0x001C #  500 kbit/s
#HUBO_BITRATE=0x011C #  250 kbit/s
#HUBO_BITRATE=0x031C #  125 kbit/s
#HUBO_BITRATE=0x432F #  100 kbit/s
#HUBO_BITRATE=0x472F #  50 kbit/s
#HUBO_BITRATE=0x532F #  20 kbit/s
#HUBO_BITRATE=0x672F #  10 kbit/s
#HUBO_BITRATE=0x7F7F #  5 kbit/s

HUBO_REF_CHAN='hubo-ref'
HUBO_STATE_CHAN='hubo-state'
HUBO_INIT_CMD='hubo-init-cmd'
HUBO_PARAM='hubo-param'

sudo echo "i 0x0014 e" > /dev/pcan0
sudo echo "i 0x0014 e" > /dev/pcan1




StopHubo()
{
	sudo killall hubo-main
	
	sudo ach -U hubo-ref
	sudo ach -U hubo-state
	sudo ach -U hubo-init-cmd
	sudo ach -U hubo-param
	
	sudo ifconfig can0 down
	sudo ifconfig can1 down
	sudo ifconfig can2 down
	sudo ifconfig can3 down
}

StartHubo()
{
	# I AM NOT CONVINCED THESE DO ANYTHING:
	#sudo ip link set can0 type can tq 125 prop-seg 1 phase-seg1 2 phase-seg2 4 sjw 1
	#sudo ip link set can1 type can tq 125 prop-seg 1 phase-seg1 2 phase-seg2 4 sjw 1
	#sudo ip link set can2 type can tq 125 prop-seg 1 phase-seg1 2 phase-seg2 4 sjw 1
	#sudo ip link set can3 type can tq 125 prop-seg 1 phase-seg1 2 phase-seg2 4 sjw 1
	sudo ifconfig can0 up
	sudo ifconfig can1 up
	sudo ifconfig can2 up
	sudo ifconfig can3 up
	
	sudo ach -1 -C hubo-ref -m 10 -n 3000
	sudo ach -1 -C hubo-state -m 10 -n 3000
	sudo ach -1 -C hubo-init-cmd -m 10 -n 3000
	
	sudo ./hubo-main & 
	sudo ./hubo-console
}

VirtualHubo()
{
	
	sudo ifconfig can0 up
	sudo ifconfig can1 up
	sudo ifconfig can2 up
	sudo ifconfig can3 up
	
	sudo ach -1 -C hubo-ref -m 10 -n 3000
	sudo ach -1 -C hubo-state -m 10 -n 3000
	sudo ach -1 -C hubo-init-cmd -m 10 -n 3000
	
	sudo ./hubo-main -v &
	sudo ./hubo-console

}


PrintHuboStatus()
{
	echo TODO: Print out how Hubo is doing...
}

ShowUsage()
{
	echo
	echo start : Start all channels and processes
	echo stop : Close all channels and processes
	echo restart : Restart all channels and processes
	echo status : Print out Hubo\'s current status
	echo
}





case "$1" in
# Start all channels and processes
	'start' )
		StartHubo
	;;

# Close all channels and processes
	'stop' )
		StopHubo
	;;

# Close and then reopen all channels and processes
	'restart' )
		StopHubo
		StartHubo
	;;

# Run the main daemon in virtual mode (does not require actual CAN communication)
	'virtual' )
		VirtualHubo
	;;

# Check the status of Hubo
		'status' )
		PrintHuboStatus
	;;

	*)
		ShowUsage
		exit 1
	;;
esac

exit 0
