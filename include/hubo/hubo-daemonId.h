/* -*-	indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */

/*
	These are enumerated flags for interacting with the hubo daemon


	Author: M.X. Grey ( mxgrey@gatech.edu )
	Last updated: 11/16/12


*/

#ifndef HUBO_DAEMONID_H
#define HUBO_DAEMONID_H


/* unless otherwise noted cmd[0] = command, cmd[1] = motor# */
/* D_ = Daemon command */
typedef enum {
	D_BLANK			= 0,	///> Null value

	D_JMC_INITIALIZE	,	///> Initilize jmc
	D_JMC_INITIALIZE_ALL	,	///> Initilize all JMC boards
	
	D_FET_SWITCH		,	///> turn fet on or off cmd[2] = 1 (on), 0 (off)
	D_CTRL_SWITCH		,	///> turn control on or off cmd[2] = 1 (on), 0 (off)
	D_ZERO_ENCODER		,	///> zero encoder for given motor
	D_JMC_ALARM		,	///> Use a JMC alarm
	D_JMC_BEEP		,	///> make beep val[0] = beep time in sec
	
	D_GET_CURRENT		,	///> Request current (amp) reading from JMC
	D_RESET_ZERO		,	///> Set current encoder position to zero
	D_SET_POS_GAIN_0	,	///> Set the 0th position gain
	D_SET_POS_GAIN_1	,	///> Set the 1st position gain
	D_SET_CUR_GAIN_0	,	///> Set the 0th current gain
	D_SET_CUR_GAIN_1	,	///> Set the 1st current gain

	D_MOTOR_DRIVER_ON	,	///> Enable motor driver for a JMC
	D_MOTOR_DRIVER_OFF	,	///> Disable motor driver for a JMC	

	D_OPENLOOP_PWM		,	///> Utilize open-loop pulse width modulation

	D_CTRL_ON		,	///> Turn motor control on
	D_CTRL_OFF		,	///> Turn motor control off
	D_CTRL_MODE		,	///> Set motor control mode
	D_GOTO_HOME		,	///> Go home position
	D_GOTO_HOME_ALL		,	///> Home all joints
	
	D_SET_DEAD_ZONE		,	///> Set dead zone 0 ~ 255
	
	D_GET_BOARD_PARAMS	,	///> Get the JMC's board parameters
	D_SET_HOME_PARAMS	,	///> Set the home search parameters
	D_SET_ENC_RESOLUTION	,	///> Set the encoder resolution
	D_SET_MAX_ACC_VEL	,	///> Set maximum acceleration and velocity
	D_SET_LOW_POS_LIM	,	///> Set a lower bound for the position limit
	D_SET_UPP_POS_LIM	,	///> Set an upper bound for the position limit
	D_SET_HOME_VEL_ACC	,	///> Set maximum acceleration/velocity while homing
	D_SET_GAIN_SCALE	,	///> Scale down the position control gains (?)
	D_SET_BOARD_NUM		,	///> Change a JMC's board number
	D_SET_SAT_LIMIT		,	///> Set jam & PWM saturation limits
	D_SET_ERR_BOUND		,	///> Set input max difference error, max error, and max temp warning
	





	/* Some other miscellaneous parameters */
	D_ALARM_SOUND1		,	///> Alarm sound #1 : Used along with D_JMC_ALARM
	D_ALARM_SOUND2		,	///> Alarm sound #2 : Used along with D_JMC_ALARM
	D_ALARM_SOUND3		,	///> Alarm sound #3 : Used along with D_JMC_ALARM
	D_ALARM_SOUND4		,	///> Alarm sound #4 : Used along with D_JMC_ALARM
	D_ALARM_OFF		,	///> Turn alarm off : Used along with D_JMC_ALARM

	D_CURRENT		,	///> Parameter for indicating (electrical) current
	D_POSITION		,	///> Parameter for indicating position

	D_PARAM_MOTOR		,	///> Motor-related parameters : Used along with D_GET_BOARD_PARAMS (Return A)
	D_PARAM_HOME		,	///> Home-related parameters  : Used along with D_GET_BOARD_PARAMS (Return B&C)
	D_PARAM_LIMITS		,	///> Limit-related parameters : Used along with D_GET_BOARD_PARAMS (Return D&H)
	D_PARAM_CURRENT		,	///> Current-related parameters: Used along with D_GET_BOARD_PARAMS(Return E)
	D_PARAM_F		,	///> I have no idea what this is supposed to be  D_GET_BOARD_PARAMS(Return F)
	D_PARAM_CAN		,	///> CAN-related parameters   : Used along with D_GET_BOARD_PARAMS (Return G)
	D_PARAM_ERROR		,	///> Error-related parameters : Used along with D_GET_BOARD_PARAMS (Return I)

	D_ENABLE		,	///> Parameter to request enabling
	D_DISABLE		,	///> Parameter to request disabling
	D_UPDATE		,	///> Parameter to request updating

} hubo_daemon_t;



































#endif




