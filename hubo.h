#include "canId.h"
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
#define		RHY		0		//	Right Hip Yaw
#define		RHR		1		//	Right Hip Roll
#define		RHP		2		//	Right Hip Pitch
#define		RKN		3		//	Right Knee Pitch
#define		RAP		4		//	Right Ankle Pitch
#define		RAR		5		//	Right Ankle Roll
#define		LHY		6		//	Left Hip Yaw
#define		LHR		7		//	Left Hip Roll
#define		LHP		8		//	Left Hip Pitch

#define		LKN		9		//	Left Knee Pitch
#define		LAP		10		//	Left Ankle Pitch
#define		LAR		11		//	Left Ankle Roll

#define		RSP		12		//	Right Shoulder Pitch
#define		RSR		13		//	Right Shoulder Pitch
#define		RSY		14		//	Right Shoulder Roll
#define		REB		15		//	Right Elbow Pitch
#define		RWY		16		// right wrist yaw
#define		RWP		17		// right wrist Pitch

#define		LSP		18		//	Left Shoulder Pitch
#define		LSR		19		//	Left Shoulder Yaw
#define		LSY		20		//	Left Shoulder Roll
#define		LEB		21		//	Left Elbow Pitch
#define		LWY		22		// left wrist yaw
#define		LWP		23		// left wrist pitch

#define		NKY		24		// neck yaw
#define		NK1		25		// neck 1
#define		NK2		26		// neck 2

#define		WST		27		//	Trunk Yaw

#define		RF1		28		//	Right Finger
#define		RF2		29		//	Right Finger
#define		RF3		30		//	Right Finger
#define		RF4		31		//	Right Finger
#define		RF5		32		//	Right Finger
#define		LF1		33		//	Left Finger
#define		LF2		34		//	Left Finger
#define		LF3		35		//	Left Finger
#define		LF4		36		//	Left Finger
#define		LF5		37		//	Left Finger

#define 	pi		3.141596




struct jnt {
	int motNo;	// joint number (on board i.e. 0, 1, 2)
	int jntNo;	// what overall number joint is it i.e. what RSP=23
	double ref;	// reference (rad)
	int drive;	// size of drive wheel
	int driven;	// size of driven wheel
	int harmonic;// gear ratio of harmonic drive
	int enc;	// encoder size
	int dir;	// direction
	char *name;	// name
	int jmc;	// motor controller number
	int can;	// can channel
};

struct sensFt {
	int bno;
	double x;
	double y;
	double z;
};

struct sensImu {
	int bno;
	double wx;
	double wy;
	double wz;
	double ax;
	double ay;
	double az;
};

struct hubo {
	struct 	jnt joint[50];	// joints
	struct 	sensImu imu;	// imu
	struct 	sensFt ft[4];	// ft
	int	socket[4];	// can channel
};




