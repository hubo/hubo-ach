#!/bin/bash

# M.X. Grey ( mxgrey@gatech.edu )
# Last updated: 11/11/2012

export LD_LIBRARY_PATH=/usr/local/lib:


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

DAEMON_DIR='/etc/hubo-daemon'

sudo echo "i $HUBO_BITRATE e" > /dev/pcan0
sudo echo "i $HUBO_BITRATE e" > /dev/pcan1

MakeAch()
{

	ach -1 -C hubo-ref -m 10 -n 3000
	ach -1 -C hubo-ref-filter -m 10 -n 3000
	ach -1 -C hubo-state -m 10 -n 3000
	ach -1 -C hubo-init-cmd -m 10 -n 3000
}

Kill()
{

	sudo kill -9 $(pidof hubo*)
}

KillAll()
{
	KillHubo
	Kill
	sudo rm /dev/shm/achshm-hubo*
}

StopHubo()
{
	case "$1" in
	'help')
		echo
		echo
		echo 'This will shut down the'
		echo 'hubo-daemon gracefully,'
		echo 'close all ach channels, and'
		echo 'put down the CAN devices.'
		echo
		echo 'This is the recommended way'
		echo 'to close down hubo-daemon.'
		echo
		echo 'If this fails to turn off'
		echo 'or unlock the daemon, try:'
		echo '   sudo ./run.sh kill'
		echo
		echo
	;;

	*)
		sudo killall -e hubo-daemon
	
		sudo ach -U hubo-ref
		sudo ach -U hubo-state
		sudo ach -U hubo-init-cmd
	
		sudo ifconfig can0 down
		sudo ifconfig can1 down
		sudo ifconfig can2 down
		sudo ifconfig can3 down

	;;
	esac
}

StartHubo()
{
	case "$2" in
	'help')
		echo
		echo
		echo 'This will switch all the CAN'
		echo 'devices to up mode, open all'
		echo 'ach channels, boot up the'
		echo 'hubo-daemon, and start up'
		echo 'the hubo-console.'
		echo
		echo 'You can pass in arguments to the'
		echo 'hubo-daemon by appending them to'
		echo 'your run.sh start command. Example:'
		echo "   sudo ./run.sh start -v -d"
		echo 'will run the daemon while passing'
		echo 'in the arguments -v and -d'
		echo
		echo
	;;

	*)
		sudo ifconfig can0 up
		sudo ifconfig can1 up
		sudo ifconfig can2 up
		sudo ifconfig can3 up
		
#		sudo ach -1 -C hubo-ref -m 10 -n 3000
#		sudo ach -1 -C hubo-ref-filter -m 10 -n 3000
#		sudo ach -1 -C hubo-state -m 10 -n 3000
#		sudo ach -1 -C hubo-init-cmd -m 10 -n 3000
		
		MakeAch
#		ach -1 -C hubo-ref -m 10 -n 3000
#		ach -1 -C hubo-ref-filter -m 10 -n 3000
#		ach -1 -C hubo-state -m 10 -n 3000
#		ach -1 -C hubo-init-cmd -m 10 -n 3000

		DAEMON_ARGS=''
		for IN_ARG in $@
		do
			if [ $IN_ARG != 'start'  ]
			then
				DAEMON_ARGS+=" $IN_ARG"
			fi
		done	
		
		sudo ./hubo-daemon $DAEMON_ARGS
		sudo ./hubo-console
	;;
	esac
}

VirtualHubo()
{
	case "$1" in
	'help')
		echo
		echo
		echo 'This is the same as running'
		echo '   sudo ./run.sh start -v'
		echo 'which runs the daemon in'
		echo 'virtual mode.'
		echo
		echo 'Virtual mode allows you to'
		echo 'run the daemon without sending'
		echo 'actual CAN commands. This way'
		echo 'the processes can be run on a'
		echo 'computer without a CAN card.'
		echo
		echo 'While running virtual mode,'
		echo 'the computer will not'
		echo 'communicate in any way with'
		echo "Hubo's hardware."
		echo
		echo	
	;;
	
	*)
		sudo rmmod vcan
		sudo modprobe vcan

		sudo ip link add type vcan
		sudo ifconfig vcan0 up
		sudo ip link add type vcan
		sudo ifconfig vcan1 up
		sudo ip link add type vcan
		sudo ifconfig vcan2 up
		sudo ip link add type vcan
		sudo ifconfig vcan3 up	

		MakeAch		
#		ach -1 -C hubo-ref -m 10 -n 3000
#		ach -1 -C hubo-ref-filter -m 10 -n 3000
#		ach -1 -C hubo-state -m 10 -n 3000
#		ach -1 -C hubo-init-cmd -m 10 -n 3000
	
		sudo ./hubo-daemon -v
		sudo ./hubo-console
	;;
	esac
}

DebugHubo()
{
	case "$1" in
	'help')
		echo
		echo
		echo 'This is the same as running'
		echo '   sudo ./run.sh start -d'
		echo 'which runs the daemon in'
		echo 'debug mode.'
		echo
		echo 'This mode produces a lot of'
		echo 'output which can flood memory'
		echo 'space. You should only run this'
		echo 'mode when investigating problems'
		echo 'and do not leave it running for'
		echo 'more than a short time.'
		echo
		echo
	;;

	*)

		sudo ifconfig can0 up
		sudo ifconfig can1 up
		sudo ifconfig can2 up
		sudo ifconfig can3 up
	

		MakeAch

#		sudo ach -1 -C hubo-ref -m 10 -n 3000
#		sudo ach -1 -C hubo-state -m 10 -n 3000
#		sudo ach -1 -C hubo-init-cmd -m 10 -n 3000
	
		sudo ./hubo-daemon -d $1
		sleep 1
		sudo ./hubo-console
	;;
	esac
}

KillHubo()
{
	case "$1" in	
	'help')
		echo
		echo
		echo 'This forces the daemon to turn off'
		echo 'with no regard for safety or grace.'
		echo 'This should only be used if'
		echo '   sudo ./run.sh stop'
		echo 'is not working for some reason.'
		echo
		echo 'You can also use this to unlock'
		echo 'hubo-daemon if it is considered to'
		echo 'be locked when it should not be.'
		echo '(i.e. if a new instance of'
		echo ' hubo-daemon refuses to turn on'
		echo ' even though it should)'
		echo
		echo

	;;

	*)
		# This forcibly interrupts all hubo-daemon processes
		sudo killall -e -s SIGKILL hubo-daemon

		# This forcibly unlocks hubo-daemon
		sudo rm /var/lock/hubo-daemon
	;;
	esac
}

LogHubo()
{


	case "$1" in
	'help')
		echo
		echo
		echo 'This prints out all recent Hubo logs.'
		echo
		echo 'The logs are split into three parts:'
		echo '1) The system log'
		echo '   This deals with the daemonization'
		echo '   and other process management'
		echo '   procedures.'
		echo '2) The standard output'
		echo '   These are operational messages.'
		echo '   Anything in hubo-daemon which is'
		echo '   sent to stdout is redirected here.'
		echo '3) The error output'
		echo '   These are error-related messages.'
		echo '   Anything in hubo-daemon which is'
		echo '   sent to stderr is redirected here.'
		echo
		echo
	;;

	*)
		echo 
		echo "System Log:"
		tail /var/log/syslog | grep hubo-daemon
		echo "_______________________________"
		echo
		echo "Hubo Output:"
		cat $DAEMON_DIR/daemon-output
		echo "_______________________________"
		echo
		echo "Hubo Errors:"
		cat $DAEMON_DIR/daemon-error
		echo
	;;
	esac
}

SaveLog()
{
	case "$1" in
	'help')
		echo
		echo
		echo 'Type the following into the terminal:'
		echo '   sudo ./run.sh savelog filename.log'
		echo 'in order to save the latest logs into'
		echo 'the file filename.log'
		echo
		echo 'Note that this will overwrite anything'
		echo 'currently contained in filename.log'
		echo
		echo 'If you do not specify a filename, the'
		echo 'logs will automatically go to saved.log'
		echo
		echo
	;;
	
	'')
		echo "System Log:" > saved.log
		tail /var/log/syslog | grep hubo-daemon >> saved.log
		echo "_______________________________" >> saved.log
		echo >> saved.log
		echo "Hubo Output:" >> saved.log
		cat $DAEMON_DIR/daemon-output >> saved.log
		echo "_______________________________" >> saved.log
		echo >> saved.log
		echo "Hubo Errors:" >> saved.log
		cat $DAEMON_DIR/daemon-error >> saved.log
	;;

	*)
		echo "System Log:" > $1
		tail /var/log/syslog | grep hubo-daemon >> $1
		echo "_______________________________" >> $1
		echo >> $1
		echo "Hubo Output:" >> $1
		cat $DAEMON_DIR/daemon-output >> $1
		echo "_______________________________" >> $1
		echo >> $1
		echo "Hubo Errors:" >> $1
		cat $DAEMON_DIR/daemon-error >> $1
	;;
	esac
}

ConfigHubo()
{
	case "$1" in
	'help')
		echo
		echo
		echo 'This is a first-time setup script'
		echo 'which creates some directories and'
		echo 'default files which are needed for'
		echo 'the hubo-daemon and other tools.'
		echo
		echo 'This only needs to be run once'
		echo 'ever on your computer.'
		echo
		echo
	;;

	*)
		sudo mkdir $DAEMON_DIR
		sudo cp joint.table $DAEMON_DIR/joint.table
		sudo cp joint.table $DAEMON_DIR/default-joint.table
		sudo chmod a-w $DAEMON_DIR/default-joint.table
	;;
	esac
}

ParamHubo()
{
	case "$1" in
	'help')
		echo
		echo
		echo 'Type the following into the terminal:'
		echo '   sudo ./run.sh filename.table'
		echo 'to use the values given in filename.table'
		echo "as Hubo's joint motor controller"
		echo 'parameters.'
		echo
		echo 'Type the following:'
		echo '   sudo ./run.sh default'
		echo 'to have the parameters return to their'
		echo 'default values.'
		echo
		echo
	;;

	'')
		echo
		echo
		echo 'You need to specify a file name!'
		echo
		echo
	;;
	*)
		cp -f $1 $DAEMON_DIR/joint.table
	;;
	esac

}

DefaultHubo()
{
	case "$1" in
	'help')
		echo
		echo
		echo 'Use this to overwrite the current'
		echo 'joint motor controller parameters'
		echo 'with their default values.'
		echo
		echo 'This can be used to restore functionality'
		echo 'if you have modified the JMC parameter'
		echo 'file in a bad way.'
		echo
		echo
	;;
	
	*)
		cp -f $DAEMON_DIR/default-joint.table $DAEMON_DIR/joint.table
	;;
	esac
}


PrintHuboStatus()
{
	echo TODO: Print out how Hubo is doing...
}

RestartHelp()
{
	echo
	echo
	echo 'This is exactly the same as running'
	echo '   sudo ./run stop'
	echo 'immediately followed by'
	echo '   sudo ./run start'
	echo
	echo


}

ShowUsage()
{
	echo
	echo 'start         : Start all channels and processes'
	echo 'stop          : Close all channels and processes'
	echo 'restart       : Restart all channels and processes'
	echo 'kill          : Emergency kill the daemon process'
	echo 'killprocess   : Kill all processes starting with hubo'
	echo 'killall       : Killprocess and rm all achshm-hubo ACH channels'
	echo "log           : Print out Hubo's latest log"
	echo 'savelog       : Save latest log to specified file'
	echo "param         : Sets Hubo's params from given file"
	echo "default       : Sets Hubo's params to default values"
	echo 'debug         : Run the daemon in debug mode'
	echo 'virtual       : Run the daemon without using CAN'
	echo 'config        : Do a first-time setup'
	echo
	echo 'Type help after any of the above commands for'
	echo '   a more detailed description'
	echo
}





case "$1" in
# Start all channels and processes
	'start' )
		StartHubo $@
	;;

# Close all channels and processes
	'stop' )
		StopHubo $2
	;;

# Kill all hubo processies
	'killprocess' )
		Kill
	;;

# Kill all hubo processies and remove ach
	'killall' )
		KillAll
	;;

# Close and then reopen all channels and processes
	'restart' )
		case "$2" in
		'help')
			RestartHelp
		;;
		*)
			StopHubo
			StartHubo
		;;
		esac
	;;

# Run the main daemon in virtual mode (does not require actual CAN communication)
	'virtual' )
		VirtualHubo $2
	;;

# Emergency kill
	'kill' )
		KillHubo $2
	;;

# Set joint motor parameters
	'param' )
		ParamHubo $2
	;;

# Set joint motor parameters to default values
	'default' )
		DefaultHubo $2
	;;	

# Print out Hubo's logs
	'log' )
		LogHubo $2
	;;

# Save the latest log
	'savelog' )
		SaveLog $2
	;;

# Debug
	'debug' )
		DebugHubo $2
	;;

# Check the status of Hubo
	'status' )
		PrintHuboStatus $2
	;;

# Do a first-time setup for the daemon and scripts
	'config' )
		ConfigHubo $2
	;;

	*)
		ShowUsage
		exit 1
	;;
esac

exit 0
