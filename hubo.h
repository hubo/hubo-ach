#include "canId.h"
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

#define		LSP		3		//	Left Shoulder Pitch
#define		LSR		4		//	Left Shoulder Yaw
#define		LSY		5		//	Left Shoulder Roll
#define		LEB		6		//	Left Elbow Pitch
#define		LWY		7		// left wrist yaw
#define		LWR		8		// left wrist roll
#define		LWP		9		// left wrist pitch

#define		NKY		1		// neck yaw
#define		NKP		2		// neck pitch
#define		NK1		2		// neck 1
#define		NK2		2		// neck 2

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


#define		numOfJoints	50		// 	the size of the array
						//	for the joints
#define 	pi		3.141596



struct jmcDriver{
	uint8_t jmc[5]; // other motors on the same drive
}__attribute__((packed));


struct jnt {
	uint16_t motNo;	// joint number (on board i.e. 0, 1, 2)
	uint16_t jntNo;	// what overall number joint is it i.e. what RSP=23
	double ref;	// reference (rad)
	uint32_t refEnc; // encoder reference
	uint16_t drive;	// size of drive wheel
	uint16_t driven;	// size of driven wheel
	uint16_t harmonic;// gear ratio of harmonic drive
	uint16_t enc;	// encoder size
	uint8_t dir;	// direction
	char *name;	// name
	uint16_t jmc;	// motor controller number
	uint8_t can;	// can channel
	uint8_t active; 	// checks if the joint is active or not
	uint8_t numMot;		// number of motors 
}__attribute__((packed));

struct sensFt {
	uint8_t bno;
	double x;
	double y;
	double z;
}__attribute__((packed));

struct sensImu {
	uint8_t bno;
	double wx;
	double wy;
	double wz;
	double ax;
	double ay;
	double az;
}__attribute__((packed));

struct hubo {
// packed struct gcc
	struct 	jnt joint[50];	// joints
	struct 	sensImu imu;	// imu
	struct 	sensFt ft[4];	// ft
	uint8_t	socket[4];	// can channel
	struct 	jmcDriver driver[0x40];	// motor driver conneciton info
}__attribute__((packed));



