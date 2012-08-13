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




