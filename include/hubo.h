/* -*-	indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
#include "hubo/canID.h"
#include "hubo-daemonID.h"
#include <stdint.h>

//#define true 1;
//#define false 0;
//888888888888888888888888888888888888888888
//-----[Static Definitions and Offsets]-----
//888888888888888888888888888888888888888888


/* Tx Message ID */

// Motor Command Message ID
#define txMoCmdId		0x01

// Sensor Command Message ID
#define txSeCmdId		0x02

// Reference Message ID 	(txReMsID + BNO)
#define txReMesId		0x10



/* Rx Message ID */

// FT sensor data Message ID 		(rxFtMsgId + SBNO)
#define rxFtMsgId		0x40

// IMU and Tilt Sensor data Message ID 	(rxImuMsgId + SBNO)
#define rxImuMsgId		0x50

// Encoder Value Message ID 		(rxEncValId + BNO)
#define rxEncValId 		0x60

// Status Message ID 			(rxStaMsgId + BNO)
#define rxStaMsgId		0x150

// Board Information Message ID 	(rxBrdInfoId + BNO + BOFF)
#define rxBrdInfoId		0x190

// Board para and Current Message ID 	(rxBrdCurMsgId + BNO)
#define rxBrdCurMsgId 		0x1C0

// BNO = Board number
// SBNO = sensor board number SBNO=BNO-0x2F
// BOFF=0 	for BNO < 0x30
// BOFF=0x80 	for BNO >= 0x30


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


#define		HUBO_CHAN_REF_NAME       "hubo-ref"        ///> hubo ach channel
#define		HUBO_CHAN_BOARD_CMD_NAME "hubo-board-cmd"   ///> hubo console channel for ach
#define		HUBO_CHAN_STATE_NAME     "hubo-state"      ///> hubo state ach channel
#define		HUBO_CHAN_PARAM_NAME     "hubo-param"      ///> hubo param ach channel
#define         HUBO_CHAN_CTRL_NAME      "hubo-control"     ///> controller ach channel
#define		HUBO_CAN_TIMEOUT_DEFAULT 0.0005		///> Defautl time for CAN to time out


#define MAX_SAFE_STACK (1024*1024) /* The maximum stack size which is
				   guaranteed safe to access without
				   faulting */



typedef enum {
	HUBO_FT_R_FOOT    = 0, ///< Index of right foot FT
	HUBO_FT_L_FOOT    = 1, ///< Index of left foot FT
	HUBO_FT_R_HAND    = 2, ///< Index of right hand FT
	HUBO_FT_L_HAND    = 3  ///< Index of left hand FT
} hubo_ft_index_t;

typedef enum {
    CTRL_OFF    = 0,
    CTRL_POS,
    CTRL_VEL
} hubo_ctrl_mode_t;

#define RIGHT 0
#define LEFT 1



struct hubo_joint_param {
	uint16_t motNo;		///< Onboard channel number
	uint16_t jntNo;		///< what overall number joint is it i.e. what RSP=23
	uint32_t refEnc; 	///< encoder reference
	uint16_t drive;		///< size of drive wheel
	uint16_t driven;	///< size of driven wheel
	uint16_t harmonic;	///< gear ratio of harmonic drive
	uint16_t enc;		///< encoder size
	uint8_t dir;		///< direction
	char name[4];		///< name
	uint16_t jmc;		///< motor controller number
	uint8_t can;		///< can channel
	uint8_t numMot;		///< number of motors
};

struct hubo_jmc_param {
	uint8_t joints[5]; // other motors on the same drive
};

struct hubo_param {
	struct hubo_joint_param joint[HUBO_JOINT_COUNT];
	struct hubo_jmc_param driver[HUBO_JMC_COUNT];	// motor driver conneciton info
};

struct hubo_imu {
	
	// LEFT and RIGHT are enumerated above
	double a_foot_x[2];	//< Linear accelerations for the feet along the x-axis (m/s/s)
	double a_foot_y[2];	//< Linear accelerations for the feet along the y-axis (m/s/s)
	double a_foot_z[2];	//< Linear accelerations for the feet along the z-axis (m/s/s)

	double angle_x;		//< IMU angle about the x-axis (degrees)
	double angle_y;		//< IMU angle about the y-axis (degrees)

	double w_x;		//< IMU rotational velocity about the x-axis (degrees/sec)
	double w_y;		//< IMU rotational velocity about the y-axis (degrees/sec)

};

struct hubo_ft {
	double m_x;	///< Moment in X (Mx)
	double m_y;	///< Moment in Y (My)
	double f_z;	///< Force in Z (Fz)
};

struct hubo_joint_state {
	double pos;     	///< actual position (rad)
	double cur;     	///< actual current (amps)
	double vel;     	///< actual velocity (rad/sec)
	double heat;		///< Heat generated by motor in Joules >> NOTE: THIS IS NOT TEMPERATURE
				///	Temperature is stored in the "hubo_jmc_state driver[]"
	uint8_t active; 	///< checks if the joint is active or not
	uint8_t zeroed;		///< checks to see if the motor is zeroed
};

struct hubo_board_msg {
	hubo_d_msg_t type;		// Type of message. Enumerated in hubo-daemonID.h
	int board;			// Board number which the message originates from
	hubo_d_param_t param[8];	// Parameters for the command. Enumerated in hubo-daemonID.h
	int values[8];			// Content of the message TODO: Figure out if 8 is sufficient
};

struct hubo_jmc_state {
	double temp;	///< temperature (dec C)
//	hubo_d_param_t ctrlMode;
	// TODO: Add more things, such as whether an alarm is on
	//	 or whether motor control / FETs are on
};

struct hubo_state {
	struct hubo_imu imu;	///< IMU
	struct hubo_ft ft[4];   ///< ft sensors
	struct hubo_joint_state joint[HUBO_JOINT_COUNT]; ///> Joint pos, velos, and current
	struct hubo_jmc_state driver[HUBO_JMC_COUNT];
	struct hubo_board_msg msg;
        double time;
};

struct hubo_ref {
	double ref[HUBO_JOINT_COUNT];	///< joint reference
	struct timespec time;           ///< time message sent
};

// Structure for sending board commands to the daemon
struct hubo_board_cmd {

	hubo_d_cmd_t type;		// Type of command. This value is REQUIRED. 
					// Enumerated in hubo-daemonID.h

	int joint;			// Target joint. If the message is meant for an entire board,
					// then fill this value with any joint number belonging to that
					// board. This value is REQUIRED (with a few exceptions).

	hubo_d_param_t param[8];	// Parameters for the command. Enumerated in hubo-daemonID.h
					// Note: This might or might not be used depending on the 
					// type of message. TODO: Figure out if 8 is sufficient (or excessive)

	int iValues[8];			// Integer values for the message. This may or may not be used
					// depending on the type of message. TODO: Figure out of 10 is sufficient
	
	double dValues[8];		// Double values for the message. This may or may not be used
					// depending on the type of message. TODO: Figure out of 8 is sufficient
	
};

struct hubo_joint_control {
    double position;
    double velocity;
    double acceleration;

    double speed_limit;
    double accel_limit;

    double pos_min;
    double pos_max;

    hubo_ctrl_mode_t mode;
};

struct hubo_control {
    struct hubo_joint_control joint[HUBO_JOINT_COUNT];
    int active;
};

//extern int hubo_debug;
//extern int verbose;
