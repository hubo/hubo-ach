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


struct jnt {
	int bno;	// board number
	int jntNo;	// joint number
	double ref;	// reference (rad)
	double ratio;	// gear ratio
	int dir;	// direction
	char *name;	// name
	int jmc;	// motor controller number
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
	struct jnt joint[50];	// joints
	struct sensImu imu;	// imu
	struct sensFt ft;	// ft
};




