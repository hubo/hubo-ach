/* -*-	indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
#include "hubo/canId.h"
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
#define		RSR		12		//	Right Shoulder Pitch
#define		RSY		13		//	Right Shoulder Roll
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

#define		HUBO_JOINT_COUNT	50	///< The size of the array
						///< For the joints
#define 	HUBO_JMC_COUNT		0X40	///< Numbher of jmc
#define 	HUBO_SENSOR_COUNT	10	///< Hubo Sensor Count
//#define		numOfCmd	3		//  	number of commiands
//#define 	numOfJmc	0x40		//	number of JMCs

#define		HUBO_CHAN_REF_NAME       "hubo-ref"        ///> hubo ach channel
#define		HUBO_CHAN_INIT_CMD_NAME	 "hubo-init-cmd"   ///> hubo console channel for ach
#define		HUBO_CHAN_STATE_NAME     "hubo-state"      ///> hubo state ach channel
#define		HUBO_CHAN_PARAM_NAME     "hubo-param"      ///> hubo param ach channel
#define 	HUBO_CHAN_REF_FILTER_NAME "hubo-ref-filter" ///> hubo reference with filter ach channel
#define		HUBO_CAN_TIMEOUT_DEFAULT 0.0005		///> Defautl time for CAN to time out


#define MAX_SAFE_STACK (1024*1024) /* The maximum stack size which is
				   guaranteed safe to access without
				   faulting */


/* def for console do flags */
/* unless otherwise noted cmd[0] = command, cmd[1] = motor# */
typedef enum {
	HUBO_JMC_INI 		= 1,	///> Initilize jmc
	HUBO_FET_ON_OFF 	= 2,	///> turn fet on or off cmd[2] = 1 (on), 0 (off)
	HUBO_CTRL_ON_OFF 	= 3,	///> turn control on or off cmd[2] = 1 (on), 0 (off)
	HUBO_ZERO_ENC		= 4,	///> zero encoder for given motor
	HUBO_GOTO_REF		= 5,	///> go to ref val[0] = ref (rad)
	HUBO_JMC_BEEP		= 6,	///> make beep val[0] = beep time in sec
	HUBO_GOTO_HOME		= 7,	///> go home position
	HUBO_GOTO_HOME_ALL	= 8,	///> home all joints
	HUBO_JMC_INI_ALL	= 9,	///> Initilize all JMC boards
	HUBO_ZERO_SENSOR    	= 10,   ///> Zero out the given FT sensor
	HUBO_ZERO_ACC    	= 11    ///> Zero out the given Accelerometer
} hubo_console_t;

typedef enum {
	HUBO_FT_R_HAND    = 0, ///< Index of right hand FT
	HUBO_FT_L_HAND    = 1, ///< Index of left hand FT
	HUBO_FT_R_FOOT    = 2, ///< Index of right foot FT
	HUBO_FT_L_FOOT    = 3, ///< Index of left foot FT
	HUBO_IMU0	  = 4, ///< Index of IMU0
	HUBO_IMU1	  = 5, ///< Index of IMU1
	HUBO_IMU2	  = 6  ///< Index of IMU2
} hubo_sensor_index_t;


struct hubo_sensor_param {
	char name[5];		///< Name of sensor
	uint16_t sensNo;	///< Sensor number
	uint16_t can;		///< Can channel
	uint8_t active;		///< Active sensor
	uint16_t canID;		///< Can I.D. of the sensor
	uint16_t boardNo;	///< Sensor Board Nuber
};

struct hubo_joint_param {
	uint16_t motNo;		///< joint number (on board i.e. 0, 1, 2)
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
//}__attribute__((packed));

struct hubo_joint_state {
	double pos;     ///< actual position (rad)
	double cur;     ///< actual current (amps)
	double vel;     ///< actual velocity (rad/sec)
	double tmp;	///< temperature (dec C)
	uint8_t active; 	///< checks if the joint is active or not
	uint8_t zeroed;		///< checks to see if the motor is zeroed
}hubo_joint_state_t;

struct hubo_ft {
	double m_x;	///< Moment in X (Mx) in Nm
	double m_y;       ///< Moment in Y (My) in Nm
	double f_z;       ///< Force in Z (Fz) in N
};

struct hubo_imu {
	double w_x;    ///< IMU: rotational velocity in x (rad/s)
	double w_y;    ///< IMU: rotational velocity in y (rad/s)
//	double w_z;    ///< rotational velocity in z (rad/s)
	double a_x;    ///< IMU/ACC: Angle in X (rad); 
	double a_y;    ///< IMU/ACC: Angle in Y (rad); 
	double a_z;    ///< ACC: Gforce in z (G)
};

struct hubo_ref {
	double ref[HUBO_JOINT_COUNT];	///< joint reference
	struct timespec time;           ///< time message sent
}hubo_ref_t;

struct hubo_state {
	struct hubo_imu imu[3];	///< IMU
	struct hubo_ft ft[4];   ///< ft sensors
	struct hubo_joint_state joint[HUBO_JOINT_COUNT]; ///> Joint pos, velos, and current
}hubo_state_t;

struct hubo_init_cmd {
	/* values for console commands */
	double val[8];
	uint16_t cmd[8];
};

struct jmcDriver{
	uint8_t jmc[5]; // other motors on the same drive
};

struct hubo_param {
	struct hubo_joint_param joint[HUBO_JOINT_COUNT];	///< Joint param
	struct jmcDriver driver[HUBO_JMC_COUNT];		///< Motor driver param
	struct hubo_sensor_param sensor[HUBO_SENSOR_COUNT];	///< Sensor param
};

extern int hubo_debug;
