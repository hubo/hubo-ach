/* -*-	indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */

/*
	These are enumerated flags for interacting with the hubo daemon


	Author: M.X. Grey ( mxgrey@gatech.edu )
	Last updated: 11/16/12


*/

#ifndef HUBO_DAEMONID_H
#define HUBO_DAEMONID_H


#ifdef __cplusplus
extern "C" {
#endif


/* D_ = Daemon-related parameter */
// hubo_d_cmd_t are used to specify board commands being sent to the hubo daemon
//	They should be placed hubo_board_cmd_t.
typedef enum {  

	// type			||	Description
	D_BLANK			= 0,	///> Null value


        D_GET_STATUS            ,///> Updates the status for given motor driver
			/*		joint: Any joint number on the board */
	D_SENSOR_STARTUP	,///> Starts up and nulls all sensor values
			/*		joint: Any joint number on the board
					param: N/A
					iValues: N/A
					dValues: N/A	*/
	D_JMC_INITIALIZE	,///> Initilize jmc
			/*		joint: Any joint number on the board
					param: N/A
					iValues: N/A
					dValues: N/A	*/
	D_NULL_SENSOR		,///> Send a null request to any sensor
			/*		joint: N/A
					param: Sensor board
						D_R_FOOT_FT, D_L_FOOT_FT,
						D_R_HAND_FT, D_L_HAND_FT,
						D_R_FOOT_ACC, D_L_FOOT_ACC,
						D_IMU_SENSOR_0, D_IMU_SENSOR_1,
						D_IMU_SENSOR_2
					iValues: N/A
					dValues: N/A	*/
	D_NULL_SENSORS_ALL	,///> Send a null request to all sensors
			/*		joint: N/A
					param: N/A
					iValues: N/A
					dValues: N/A	*/
	D_NULL_FT_SENSOR	,///> Send a null request to force-torque sensor
			/*		joint: N/A
					param: Force-torque board
						D_R_FOOT_FT, D_L_FOOT_FT,
						D_R_HAND_FT, D_L_HAND_FT
					iValues: N/A
					dValues: N/A	*/
	D_NULL_ACC_SENSOR	,///> Send null request to accelerometer sensor
			/*		joint: N/A
					param: Accelerometer board
						D_R_FOOT_ACC or D_L_FOOT_ACC
					iValues: N/A
					dValues: N/A	*/
	D_NULL_ACC_SENSOR_ALL	,///> Send null requests to all accelerometer sensors
			/*		joint: N/A
					param: N/A
					iValues: N/A
					dValues: N/A	*/
	D_NULL_FT_SENSOR_ALL	,///> Send null requests to all force-torque sensors
			/*		joint: N/A
					param: N/A
					iValues: N/A
					dValues: N/A	*/
	D_NULL_FT_ACC_SENSOR_ALL,///> Send null requests to all ft and accel sensors
			/*		joint: N/A
					param: N/A
					iValues: N/A
					dValues: N/A	*/
	D_NULL_IMU_SENSOR	,///> Send null request to specific IMU sensor
			/*		joint: N/A
					param: IMU sensor board
						D_IMU_SENSOR_0, D_IMU_SENSOR_1
						D_IMU_SENSOR_2
					iValues: N/A
					dValues: N/A	*/
	D_NULL_IMU_SENSOR_ALL	,///> Send null requests to all IMU sensors
			/*		joint: N/A
					param: N/A
					iValues: N/A
					dValues: N/A	*/
	D_INIT_FT_ACC_SENSOR	,///> Send init request to FT/Acc sensor
			/*		joint: N/A
					param: FT/Accelerometer board
						D_R_FOOT_FT, D_L_FOOT_FT,
	//These two options are redundant	D_R_FOOT_ACC, D_L_FOOT_ACC,
						D_R_HAND_FT, D_L_HAND_FT
	//Note: Acc and FT must		iValues: N/A
	// always both be initialized	dValues: N/A	*/
	D_INIT_FT_ACC_SENSOR_ALL,///> Send null requests to all IMU sensors
			/*		joint: N/A
					param: N/A
					iValues: N/A
					dValues: N/A	*/
	D_JMC_INITIALIZE_ALL	,///> Initilize all JMC boards
			/*		joint: N/A
					param: N/A
					iValues: N/A
					dValues: N/A	*/	
	D_FET_SWITCH		,///> Turn fet on or off
			/*	 	joint: Any joint number on the board
					param: [0] = D_ENABLE or D_DISABLE
					iValues: N/A
					dValues: N/A	*/	
	D_CTRL_SWITCH		,///> Turn control on or off
			/*		joint: Any joint number on the board
					param: [0] = D_ENABLE or D_DISABLE
					iValues: N/A
					dValues: N/A	*/	
	D_ZERO_ENCODER		,///> zero encoder for given motor
			/*		joint: The target joint number
					param: N/A
					iValues: N/A
					dValues: N/A	*/	
	D_JMC_ALARM		,///> Use a JMC alarm
			/*		joint: Any joint number on the board
					param: [0] =	D_ALARM_SOUND1, D_ALARM_SOUND2,
							D_ALARM_SOUND3, D_ALARM_SOUND4,
							D_ALARM_OFF
					iValues: N/A
					dValues: N/A	*/	
	D_JMC_BEEP		,///> Emit a beep
			/*		joint: Any joint number on the board
					param: N/A
					iValues: N/A
					dValues: [0] = Length of Beep Time	*/	
	
	D_SET_POS_GAIN		,///> Set the position ctrl gains for a joint
						// NOTE: For 3 or 5 channel boards,
						//	 all channels have the same gains
				/*	joint: Target joint
					param: N/A
					iValues:	[0] = Proportional Gain
							[1] = Integral Gain
							[2] = Derivative Gain
					dValues: N/A	*/
	D_SET_CUR_GAIN		,///> Set the current gain
						// NOTE: For 3 or 5 channel boards,
						//	 all channels have the same gains
			/*		joint: Target joint number
					param: N/A
					iValues:	[0] = Proportional Gain
							[1] = Integral Gain
							[2] = Derivative Gain
					dValues: N/A	*/	
	
	D_OPENLOOP_PWM		,///> Utilize open-loop pulse width modulation
			/*		joint: Target joint number
					param:		[0] Direction for channel 0
								D_CLOCKWISE or D_COUNTERCLOCKWISE
							[1] Direction for channel 1
							[2] Direction for channel 2
		Only for 3+ channel boards  >		[3] Direction for channel 3
		Only for 5  channel boards  >		[4] Direction for channel 4
		Only for 5  channel boards  >		[5] Direction for channel 5

					iValues:	[0] = Percent PWM Duty for channel 0
								[-100, 100]
							[1] = Percent PWM Duty for channel 1
		Only for 3+ channel boards  >		[2] = Percent PWM Duty for channel 2
		Only for 5  channel boards  >		[3] = Percent PWM Duty for channel 3
		Only for 5  channel boards  >		[4] = Percent PWM Duty for channel 4
					dValues: N/A	*/	
        D_CTRL_ON_OFF           ,///>turn on and off control to motor XXX.  If Y = 0 then Off, if Y = 1 then On 
                                /* Note: must home joint first*/
        D_CTRL_ON_OFF_ALL       ,///>turn on and off control to all motors. If Y = 0 then Off, if Y = 1 then On 
                                /*Note: must home joint first.*/
	D_CTRL_ON		,///> Turn motor control on
			/*		joint: Any joint number on the board
	// Note: This is redundant	param: N/A
	// with D_CTRL_SWITCH		iValues: N/A
					dValues: N/A	*/	
	D_CTRL_OFF		,///> Turn motor control off
			/*		joint: Any joint number on the board
	// Note: This is redundant	param: N/A
	// with D_CTRL_SWITCH		iValues: N/A
					dValues: N/A	*/	
	D_CTRL_MODE		,///> Set motor control mode
			/*		joint: Any joint number on the board
					param:		[0] = D_POSITION or D_CURRENT
					iValues: N/A
					dValues: N/A	*/	
	D_GOTO_HOME		,///> Go home position
			/*		joint: Target joint number
					param: N/A
					iValues: N/A
					dValues: N/A	*/	
	D_GOTO_HOME_ALL		,///> Home all joints
			/*		joint: N/A
					param: N/A
					iValues: N/A
					dValues: N/A	*/	
	
	D_SET_DEAD_ZONE		,///> Set dead zone
			/*		joint: Target joint number
					param: N/A
					iValues: 	[0] = Deadzone Value [0,255]
					dValues: N/A	*/	
	
	D_SET_HOME_PARAMS	,///> Set the home search parameters
			/*		joint: Target joint number
					param: 		[0] = Search direction
								D_CLOCKWISE or D_COUNTERCLOCKWISE
					iValues:	[0] = Search limit
							      (maximum number of spins)
							[1] = Offset from index position
					dValues: N/A	*/	
	D_SET_ENC_RESOLUTION	,///> Set the encoder resolution
			/*		joint: Target joint number
					param: 		[0] = Motor direction
								D_CLOCKWISE or D_COUNTERCLOCKWISE
							[1] = Auto-Scale
								D_ENABLE or D_DISABLE
					iValues:	[0] = Encoder Resolution
								Max: 16383
					dValues: N/A	*/	
	D_SET_MAX_ACC_VEL	,///> Set maximum acceleration and velocity
			/*		joint: Target joint number
					param: N/A
					iValues: 	[0] Max Acceleration (up to 65535)
							[1] Max Velocity (up to 65535)
					dValues: N/A	*/	
	D_SET_LOW_POS_LIM	,///> Set a lower bound for the position limit
			/*		joint: Target joint number
					param:		[0] = Should the new limit value be used
								or ignored until rebooting?
									D_UPDATE or D_IGNORE
							[1] = Should the lower limit be enabled?
								D_ENABLE or D_DISABLE
					iValues:	[0] = Lower position limit value
					dValues: N/A	*/	
	D_SET_UPP_POS_LIM	,///> Set an upper bound for the position limit
			/*		joint: Target joint number
					param:		[0] = Should the new limit value be used
								or ignored until rebooting?
									D_UPDATE or D_IGNORE
							[1] = Should the upper limit be enabled?
								D_ENABLE or D_DISABLE
					iValues:	[0] = Upper position limit value
					dValues: N/A	*/	
	D_SET_HOME_VEL_ACC	,///> Set maximum acceleration/velocity while homing
			/*		joint: Target joint number
					param:		[0] = Home Search Mode
								D_SWITCH_AND_INDEX or D_SWITCH
								or D_JAM_LIMIT
					iValues:	[0] = Max velocity to reach Limit Switch
							[1] = Max velocity to reach Offset Position
							[2] = PWM Duty% for jam limit detection
					dValues:	[0] = Max acceleration	*/	
	D_SET_GAIN_SCALE	,///> Scale down the position control gains (?)// TODO: Look into this. It seems fishy.
			/*		joint: Target board number
					param: N/A
	//Note: 3+ channel boards	iValues:	[0] = % Override value for channel 0
	//will all use the [0] value			[1] = % Override value for channel 1
					dValues:	[0] = Time duration of override in seconds //TODO: Verify this
							*/		
	D_SET_BOARD_NUM		,///> Change a JMC's board number or baud rate
			/*		joint: Any joint number on the board
					param:	N/A
					iValues:	[0] = New board number
							[1] = New CAN Rate (in msec)
					dValues: N/A	*/	
	D_SET_JAM_SAT_LIMIT	,///> Set jam & PWM saturation limits
			/*		joint: Any joint number on the board
					param: N/A
					iValues:	[0] = Jam limit in % duty
							[1] = PWM duty % for limit detection.
								Used for limit search mode 2.
					dValues:	[0] = Jam detection time (seconds)
							[1] = PWM saturation detection time (seconds)	*/	
	D_SET_ERR_BOUND		,///> Set input max difference error, max error, and max temp warning
			/*		joint: Any joint number on the board
					param: N/A
					iValues:	[0] = Maximum input difference error
							[1] = Maximum error
							[2] = Maximum temperature warning temperature
					dValues: N/A	*/
	D_GET_BOARD_ERRORS	,///> Request board status and error flags
			/* INCOMPLETE	joint: Any joint number on the board
					param: N/A
					iValues: N/A
					dValues: N/A	
			Return type: */
	D_SET_AND_GET_BOARD_INFO,///> Set board CAN rate and get board info
			/* INCOMPLETE	joint: Any joint number on the board
					param: 		[0] Whether or not to ignore iValues[0]
						D_UPDATE or D_IGNORE
					iValues:	[0] = CAN rate (in msec)
								Default: 5ms
					dValues: N/A	*/
			
	D_GET_BOARD_PARAMS	///> Get the JMC's board parameters
			/* INCOMPLETE	joint: Target joint number
					param: 		[0] = Type of parameters
						D_PARAM_MOTOR, D_PARAM_HOME, D_PARAM_LIMITS
						D_PARAM_CURRENT, D_PARAM_F, D_PARAM_CAN
						D_PARAM_ERROR
					iValues: N/A
					dValues: N/A	*/	

	
} hubo_d_cmd_t;


typedef enum {

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
	D_IGNORE		,	///> Parameter to request ignoring

	D_CLOCKWISE		,	///> Parameter to indicate clockwise turning
	D_COUNTERCLOCKWISE	,	///> Parameter to indicate counter-clockwise turning

	// Homing modes
	D_SWITCH_AND_INDEX	,	///> Sets homing mode to use the switch plus the offset position
	D_SWITCH		,	///> Homes only using the limit switch
	D_JAM_LIMIT		,	///> Homes without the limit switch. "LIMD is used to detect jam"

	D_R_FOOT_FT		,	///> Right foot force-torque sensor
	D_L_FOOT_FT		,	///> Left foot force-torque sensor
	D_R_HAND_FT		,	///> Right hand force-torque sensor
	D_L_HAND_FT		,	///> Left hand force-torque sensor

	D_R_FOOT_ACC		,	///> Right foot accelerometer
	D_L_FOOT_ACC		,	///> Left foot accelerometer

	D_IMU_SENSOR_0		,	///> Parameter to indicate IMU sensor 0
	D_IMU_SENSOR_1		,	///> Parameter to indicate IMU sensor 1
	D_IMU_SENSOR_2			///> Parameter to indicate IMU sensor 2

} hubo_d_param_t;


typedef enum {

	D_ERROR			,///> Indicates an error message
			/* INCOMPLETE	param: N/A
					values: N/A	*/


} hubo_d_msg_t;
















#ifdef __cplusplus
}
#endif










#endif




