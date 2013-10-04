/* -*-	indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
/*
Copyright (c) 2012, Daniel M. Lofaro <dan@danlofaro.com>
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the author nor the names of its contributors may
      be used to endorse or promote products derived from this software
      without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef HUBO_PRIMARY_H
#define HUBO_PRIMARY_H


#ifdef __cplusplus
extern "C" {
#endif


// for Hubo
#include "hubo-daemonID.h"

//888888888888888888888888888888888888888888
//---------[Prerequisites for ACH]----------
#include <stdint.h>
#include <time.h>
#include <string.h>
#include <pthread.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <ach.h>
//888888888888888888888888888888888888888888






//888888888888888888888888888888888888888888
//-----[Static Definitions and Offsets]-----
//888888888888888888888888888888888888888888

/* Joint Numbers/Index values */
#define		RHY		26		//	Right Hip Yaw
#define		RHR		27		//	Right Hip Roll
#define		RHP		28		//	Right Hip Pitch
#define		RKN		29		//	Right Knee Pitch
#define		RAP		30		//	Right Ankle Pitch
#define		RAR		31		//	Right Ankle Roll

#define		LHY		19		//	Left Hip Yaw
#define		LHR		20		//	Left Hip Roll
#define		LHP		21		//	Left Hip Pitch
#define		LKN		22		//	Left Knee Pitch
#define		LAP		23		//	Left Ankle Pitch
#define		LAR		24		//	Left Ankle Roll

#define		RSP		11		//	Right Shoulder Pitch
#define		RSR		12		//	Right Shoulder Roll
#define		RSY		13		//	Right Shoulder Yaw
#define		REB		14		//	Right Elbow Pitch
#define		RWY		15		// right wrist yaw
#define		RWR		16		// right wrist roll
#define		RWP		17		// right wrist Pitch

#define		LSP		4		//	Left Shoulder Pitch
#define		LSR		5		//	Left Shoulder Yaw
#define		LSY		6		//	Left Shoulder Roll
#define		LEB		7		//	Left Elbow Pitch
#define		LWY		8		// left wrist yaw
#define		LWR		9		// left wrist roll
#define		LWP		10		// left wrist pitch

#define		NKY		1		// neck yaw
#define		NK1		2		// neck 1
#define		NK2		3		// neck 2

#define		WST		0		//	Trunk Yaw

#define		RF1		32		//	Right Finger
#define		RF2		33		//	Right Finger
#define		RF3		34		//	Right Finger
#define		RF4		35		//	Right Finger
#define		RF5		36		//	Right Finger
#define		LF1		37		//	Left Finger
#define		LF2		38		//	Left Finger
#define		LF3		39		//	Left Finger
#define		LF4		40		//	Left Finger
#define		LF5		41		//	Left Finger

#define 	HUBO_CAN_CHAN_NUM	4	///> Number of CAN channels avaliable
#define         HUBO_JOINT_COUNT        42              ///> The max number of joints
#define         HUBO_JMC_COUNT          0x36            ///> The max number of JMCs
#define         BNO_SENSOR_BASE         0x2F
#define         HUBO_SENSOR_COUNT       0x36-BNO_SENSOR_BASE    ///> The max number of sensor units

#define		HUBO_CHAN_REF_NAME         "hubo-ref"                    ///> hubo ach channel
#define		HUBO_CHAN_REF_NECK_NAME    "hubo-ref-neck"               ///> hubo ach channel ref for neck
#define		HUBO_CHAN_BOARD_CMD_NAME   "hubo-board-cmd"              ///> hubo console channel for ach
#define		HUBO_CHAN_STATE_NAME       "hubo-state"                  ///> hubo state ach channel
#define         HUBO_CHAN_PWM_GAINS_NAME   "hubo-pwm-gains"              ///> PWM Gain control channel
#define		HUBO_CHAN_BOARD_PARAM_NAME "hubo-board-param"                  ///> hubo param ach channel
#define 	HUBO_CHAN_REF_FILTER_NAME  "hubo-ref-filter"            ///> hubo reference with filter ach channel
#define 	HUBO_CHAN_VIRTUAL_TO_SIM_NAME "hubo-virtual-to-sim"    ///> virtual channel trigger to simulator
#define 	HUBO_CHAN_VIRTUAL_FROM_SIM_NAME "hubo-virtual-from-sim"  ///> virtual channel trigger from simulator
//#define		HUBO_CAN_TIMEOUT_DEFAULT 0.0005		///> Default time for CAN to time out
//#define		HUBO_CAN_TIMEOUT_DEFAULT 0.0002		///> Default time for CAN to time out
#define		HUBO_CAN_TIMEOUT_DEFAULT 0.00018		///> Default time for CAN to time out
#define         HUBO_REF_FILTER_LENGTH   40
#define         HUBO_LOOP_PERIOD         0.005  ///> period for main loopin sec (0.005 = 200hz)
//#define         HUBO_LOOP_PERIOD         0.010  ///> period for main loopin sec (0.010 = 100hz)
#define         HUBO_FINGER_CURRENT_CTRL_MODE 0x01
#define         HUBO_STARTUP_SEND_REF_DELAY 0.8   ///> setup delay in secons
#define         HUBO_FINGER_SAT_VALUE 10          ///> value in 0.01A units


#define         HUBO_COMP_RIGID_TRANS_MULTIPLIER 10  ///> multiplication factor for the filter
                                                     ///> which transitions from compliant to rigid
#define         HUBO_COMP_RIGID_TRANS_THRESHOLD 0.0075  ///> threshold for finishing the transition

#define MAX_SAFE_STACK (1024*1024) /* The maximum stack size which is
				   guaranteed safe to access without
				   faulting */

#define OFF 0 // off static
#define ON  1 // on static

// array of joint name strings (total of 42)
static const char *jointNames[HUBO_JOINT_COUNT] =
	{"WST", "NKY", "NK1", "NK2", // 0 1 2 3
	 "LSP", "LSR", "LSY", "LEB", "LWY", "LWR", "LWP", // 4 5 6 7 8 9 10
	 "RSP", "RSR", "RSY", "REB", "RWY", "RWR", "RWP", "N/A", // 11 12 13 14 15 16 17 18
	 "LHY", "LHR", "LHP", "LKN", "LAP", "LAR", "N/A", // 19 20 21 22 23 24 25
	 "RHY", "RHR", "RHP", "RKN", "RAP", "RAR", // 26 27 28 29 30 31
	 "RF1", "RF2", "RF3", "RF4", "RF5", // 32 33 34 35 36
	 "LF1", "LF2", "LF3", "LF4", "LF5"}; // 37 38 39 40 41

typedef enum {
    HUBO_VIRTUAL_MODE_NONE        = 0, ///< not virtual mode
    HUBO_VIRTUAL_MODE_VIRTUAL     = 1, ///< virtual mode just uses vcan
    HUBO_VIRTUAL_MODE_OPENHUBO    = 2  ///< changes timing for use with openhubo
}__attribute__((packed)) hubo_virtual_mode_index_t;

typedef enum {
    HUBO_ROBOT_TYPE_HUBO_PLUS   = 0, ///> HUBO+ MODEL
    HUBO_ROBOT_TYPE_DRC_HUBO    = 1 ///> DRC HUBO MODEL
}__attribute__((packed)) hubo_robot_type_t;

typedef enum {
	HUBO_FT_R_HAND    = 0, ///< Index of right hand FT
	HUBO_FT_L_HAND    = 1, ///< Index of left hand FT
	HUBO_FT_R_FOOT    = 2, ///< Index of right foot FT
	HUBO_FT_L_FOOT    = 3, ///< Index of left foot FT
	HUBO_IMU0	  = 4, ///< Index of IMU0
	HUBO_IMU1	  = 5, ///< Index of IMU1
	HUBO_IMU2	  = 6,  ///< Index of IMU2
    SENSOR_INDEX_COUNT
}__attribute__((packed)) hubo_sensor_index_t;

typedef enum{
    HUBO_COMPLIANT_MODE_RIGID            = 0,  ///> 0: Rigid mode
    HUBO_COMPLIANT_MODE_COMPLIANT        = 1,  ///< 1: Compliant mode
    HUBO_COMPLIANT_MODE_COMPLIANT2RIGID  = 2,  ///< 2: Transitioning back to rigid
    HUBO_COMPLIANT_MODE_TURNING_MOTOR_ON = 3,  ///< 3: Turning motor control back on (should never be seen by the user) 
  COMPLIANT_INDEX_COUNT
}__attribute__((packed)) hubo_compliant_mode_index_t;

#define HUBO_IMU_COUNT 3
typedef enum {
    TILT_R  = 0,
    TILT_L  = 1,
    IMU     = 2
}__attribute__((packed)) hubo_imu_index_t;

typedef enum {
   HUBO_HOME_OK       = 6,
   HUBO_HOME_OK_WRIST = 2
}__attribute__((packed)) hubo_status_return_t;

typedef enum {
	HUBO_REF_MODE_REF_FILTER    = 0, ///< Reference to reference filter
	HUBO_REF_MODE_REF           = 1, ///< Direct reference control
	HUBO_REF_MODE_COMPLIANT     = 2, ///< Compliant mode, sets ref to current encoder position. 
	HUBO_REF_MODE_ENC_FILTER    = 3, ///< Reference filter
}__attribute__((packed)) hubo_mode_type_t;

#define RIGHT 0
#define LEFT 1


typedef struct hubo_sensor_param {
	uint16_t sensNo;	///< Sensor number
	uint16_t can;		///< Can channel
	uint16_t boardNo;	///< Sensor Board Nuber
	uint8_t active;		///< Active sensor
	char name[5];		///< Name of sensor
        int16_t xsign;
        int16_t ysign;
        int16_t zsign;
}__attribute__((packed)) hubo_sensor_param_t;

typedef struct hubo_joint_param {
	uint32_t refEnc; 	///< encoder reference
	uint16_t motNo;		///< Onboard channel number
	uint16_t jntNo;		///< what overall number joint is it i.e. what RSP=23
	uint16_t drive;		///< size of drive wheel
	uint16_t driven;	///< size of driven wheel
	uint16_t harmonic;	///< gear ratio of harmonic drive
	uint16_t enc;		///< encoder size
	uint16_t jmc;		///< motor controller number
	int16_t dir;		    ///< direction
	uint8_t can;		///< can channel
	uint8_t numMot;		///< number of motors
	char name[4];		///< name
}__attribute__((packed)) hubo_joint_param_t;

typedef struct hubo_jmc_param {
	uint8_t joints[5]; // other motors on the same drive
}__attribute__((packed)) hubo_jmc_param_t;

typedef struct hubo_board_joint_param {
    
    int confidence;

    uint16_t deadZone;

    int32_t homeOffsetRaw;
    double homeOffset;
    uint8_t searchDirection;
    uint8_t searchMode;
    uint16_t searchLimit;
//    uint16_t searchLimitRaw;
//    double searchLimit;

    uint16_t maxHomeAccelRaw;
    double maxHomeAccel;
    uint16_t maxHomeLimitVelRaw; ///< Maximum Velocity for Home limit search
    double maxHomeLimitVel;
    uint16_t maxHomeOffsetVelRaw; ///< Maximum Velocity to Offset position
    double maxHomeOffsetVel;

    int32_t lowerLimitRaw;
    double lowerLimit;
    int32_t upperLimitRaw;
    double upperLimit;
    uint16_t maxAccelRaw;
    double maxAccel;

    uint16_t maxVelRaw;
    double maxVel;
    uint16_t maxPWM;
    uint16_t maxCurrent;
    
    uint16_t Kp;
    uint16_t Ki;
    uint16_t Kd;
    uint16_t Kpt; ///< Motor position gain ...t?
    uint16_t Kdt; ///< Motor derivative gain ...t?
    uint16_t Kft; ///< Motor current gain ...t?

    uint16_t encoderResolution;
    uint8_t motorDirection;
    uint8_t autoScale;
    
    uint16_t canRate;
    uint8_t boardType;

    uint16_t jamTimeRaw;
    double jamTime;
    uint16_t pwmSaturationTimeRaw;
    double pwmSaturationTime;
    uint8_t pwmDutyLimit;
    uint8_t pwmDutyJam;

    uint16_t maxInputDifference;
    uint16_t maxError;
    uint16_t maxEncError; ///< Maximum error for encoder failure

}__attribute__((packed)) hubo_board_joint_param_t;

typedef struct hubo_board_param {
    
    hubo_board_joint_param_t joint[HUBO_JOINT_COUNT];
    
}__attribute__((packed)) hubo_board_param_t;

typedef struct hubo_param {
	hubo_joint_param_t joint[HUBO_JOINT_COUNT];     ///< Joint param
	hubo_jmc_param_t driver[HUBO_JMC_COUNT];		///< Motor driver param
	hubo_sensor_param_t sensor[HUBO_SENSOR_COUNT];	///< Sensor param
}__attribute__((packed)) hubo_param_t;

typedef struct hubo_imu {
	double a_x;     ///< angular position around x (rad)
	double a_y;     ///< angular position around y (rad)
        double a_z;     ///< angular position around z
	double w_x;     ///< rotational velocity in x (rad/s)
	double w_y;     ///< rotational velocity in y (rad/s)
	double w_z;     ///< rotational velocity in z (rad/s)
}__attribute__((packed)) hubo_imu_t;


typedef struct hubo_virtual {
    double time; ///< trigger channel time in (sec)
}__attribute__((packed)) hubo_virtual_t;

typedef struct hubo_ft {
	double m_x;	///< Moment in X (Mx)
	double m_y;	///< Moment in Y (My)
	double f_z;	///< Force in Z (Fz)
}__attribute__((packed)) hubo_ft_t;

typedef struct hubo_joint_state {
        double ref;         ///< Last reference value sent
	uint8_t comply;		///< Are we in compliance mode?
                        ///< 0: Rigid mode
                        ///< 1: Compliant mode
                        ///< 2: Transitioning back to rigid
                        ///< 3: Turning motor control back on (should never be seen by the user) 
	double pos;     	///< actual position (rad)
	double cur;     	///< actual current (amps)
	double vel;     	///< actual velocity (rad/sec)
        double duty;            ///< PWM duty cycle
	double heat;		///< Heat generated by motor in Joules >> NOTE: THIS IS NOT TEMPERATURE
        double tmp;
	uint8_t active; 	///< checks if the joint is active or not
	uint8_t zeroed;		///< checks to see if the motor is zeroed
}__attribute__((packed)) hubo_joint_state_t;

typedef struct hubo_joint_status {
    // STATx0
    uint8_t driverOn;
    uint8_t ctrlOn;
    uint8_t mode;
    uint8_t limitSwitch;
    uint8_t homeFlag;

    // STATx1
    uint8_t jam;
    uint8_t pwmSaturated;
    uint8_t bigError;
    uint8_t encError;
    uint8_t driverFault;
    uint8_t motorFail0;
    uint8_t motorFail1;
    
    // STATx2
    uint8_t posMinError;
    uint8_t posMaxError;
    uint8_t velError;
    uint8_t accError;
    uint8_t tempError;
}__attribute__((packed)) hubo_joint_status_t;

typedef struct hubo_jmc_state {
	double temp;	///< temperature (dec C)
	// TODO: Add more things, such as whether an alarm is on
	//	 or whether motor control / FETs are on
}__attribute__((packed)) hubo_jmc_state_t;

typedef struct hubo_power {
	double voltage;
	double current;
	double power;
}__attribute__((packed)) hubo_power_t;

typedef struct hubo_state {
	hubo_imu_t imu[HUBO_IMU_COUNT];	///< IMU
	hubo_ft_t ft[4];   ///< ft sensors
	struct hubo_joint_state joint[HUBO_JOINT_COUNT]; ///> Joint pos, velos, and current
        hubo_joint_status_t status[HUBO_JOINT_COUNT];
	struct hubo_jmc_state driver[HUBO_JMC_COUNT];
	hubo_power_t power; // back power board
        double time;
	int16_t refWait;
}__attribute__((packed)) hubo_state_t;

typedef struct hubo_ref {
	double ref[HUBO_JOINT_COUNT];	///< joint reference
	int16_t mode[HUBO_JOINT_COUNT]; 	///< mode 0 = filter mode, 1 = direct reference mode
	int8_t comply[HUBO_JOINT_COUNT];
}__attribute__((packed)) hubo_ref_t;


typedef struct hubo_joint_pwm_gains {
    double pwmCommand;
	double Kp;
	double Kd;
    int8_t maxPWM;
}__attribute__((packed)) hubo_joint_pwm_gains_t;

typedef struct hubo_pwm_gains {
    hubo_joint_pwm_gains_t joint[HUBO_JOINT_COUNT];
}__attribute__((packed)) hubo_pwm_gains_t;

typedef struct jmcDriver{
	uint8_t jmc[5]; // other motors on the same drive
}__attribute__((packed)) jmcDriver_t;

// Structure for sending board commands to the daemon
typedef struct hubo_board_cmd {

	hubo_d_cmd_t type;		// Type of command. This value is REQUIRED. 
					// Enumerated in hubo-daemonID.h

	int16_t joint;			// Target joint. If the message is meant for an entire board,
					// then fill this value with any joint number belonging to that
					// board. This value is REQUIRED (with a few exceptions).

	hubo_d_param_t param[8];	// Parameters for the command. Enumerated in hubo-daemonID.h
					// Note: This might or might not be used depending on the 
					// type of message. TODO: Figure out if 8 is sufficient (or excessive)

	int32_t iValues[8];			// Integer values for the message. This may or may not be used
					// depending on the type of message. TODO: Figure out of 10 is sufficient
	
	double dValues[8];		// Double values for the message. This may or may not be used
					// depending on the type of message. TODO: Figure out of 8 is sufficient
}__attribute__((packed)) hubo_board_cmd_t;


extern int hubo_debug;


#ifdef __cplusplus
}
#endif



#endif //HUBO_PRIMARY_H

