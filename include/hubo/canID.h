/* -*-	indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
#ifndef CAN_ID_H
#define CAN_ID_H

#ifdef __cplusplus
extern "C" {
#endif

#define		LOWER_CAN		0
#define		UPPER_CAN		1

#define		SBNO_RIGHT_FOOT_FT	1
#define		SBNO_LEFT_FOOT_FT	SBNO_RIGHT_FOOT_FT+1
#define		SBNO_IMU_0		SBNO_RIGHT_FOOT_FT+2
#define		SBNO_IMU_1		SBNO_RIGHT_FOOT_FT+3
#define		SBNO_IMU_2		SBNO_RIGHT_FOOT_FT+4
#define		SBNO_RIGHT_HAND_FT	SBNO_RIGHT_FOOT_FT+5
#define		SBNO_LEFT_HAND_FT	SBNO_RIGHT_FOOT_FT+6

#define     BNO_SENSOR_BASE     0x2F

#define		BNO_RIGHT_FOOT_FT	0x30
#define		BNO_LEFT_FOOT_FT	BNO_RIGHT_FOOT_FT+1
#define		BNO_IMU_0		BNO_RIGHT_FOOT_FT+2
#define		BNO_IMU_1		BNO_RIGHT_FOOT_FT+3
#define		BNO_IMU_2		BNO_RIGHT_FOOT_FT+4
#define		BNO_RIGHT_HAND_FT	BNO_RIGHT_FOOT_FT+5
#define		BNO_LEFT_HAND_FT	BNO_RIGHT_FOOT_FT+6


#define		HUBO_JOINT_COUNT	42		///> The max number of joints
#define 	HUBO_JMC_COUNT		0x26		///> The max number of JMCs
#define		HUBO_SENSOR_COUNT	0x36-BNO_SENSOR_BASE	///> The max number of sensor units

#define H_BLANK		0x00


// This file contains 'CAN-ID' for HUBO2

#define CMD_TXDF			0x01	// Base hex for sending motor/board commands
#define REQ_SENSOR_TXDF			0x02	// Base hex for requesting sensor commands

#define REF_BASE_TXDF			0x10	// Base hex for receiving messages

#define H_SENSOR_FT_BASE_RXDF		0x40	// Hex for receiving force-torque values
#define H_SENSOR_FT_MAX_RXDF		H_SENSOR_FT_BASE_RXDF+HUBO_SENSOR_COUNT

#define H_SENSOR_IMU_BASE_RXDF		0x50	// Hex for receiving IMU values
#define H_SENSOR_IMU_MAX_RXDF		H_SENSOR_IMU_BASE_RXDF+HUBO_SENSOR_COUNT

#define H_ENC_BASE_RXDF			0x60	// Hex for receiving encoder values
#define H_ENC_MAX_RXDF			H_ENC_BASE_RXDF+HUBO_JMC_COUNT

#define H_INFO_BASE_RXDF		0x190 	// Hex for recieving board information
#define H_INFO_MAX_RXDF			H_INFO_BASE_RXDF+HUBO_SENSOR_COUNT
#define H_INFO_SENSOR_BASE_RXDF		H_INFO_BASE_RXDF+0x80 // Hex for board info related to sensor boards
#define H_INFO_SENSOR_MAX_RXDF		H_INFO_SENSOR_BASE_RXDF+HUBO_SENSOR_COUNT

#define H_CURRENT_BASE_RXDF		0x1C0	// Hex for receiving current (ampage) values
#define H_CURRENT_MAX_RXDF		H_CURRENT_BASE_RXDF+HUBO_JMC_COUNT

#define DAOFFSET_BASE_RXDF		0x310
#define ADOFFSET_BASE_RXDF		0x320
#define OFFSET_BASE_RXDF		0x330

#define H_STAT_BASE_RXDF		0x150
#define H_STAT_MAX_RXDF         H_STAT_BASE_RXDF+HUBO_JMC_COUNT

// For JMC0 sub-controller
// Right Hip Yaw-Roll
#define JMC0					0x00
#define REF0_TXDF				REF_BASE_TXDF+JMC0
#define ENC0_RXDF				H_ENC_BASE_RXDF+JMC0
#define CUR0_RXDF				H_CUR_BASE_RXDF+JMC0
#define PM0_RXDF				PM_BASE_RXDF+JMC0
#define STAT0_RXDF				STAT_BASE_RXDF+JMC0
#define NAME0_RXDF				NAME_BASE_RXDF+JMC0
#define SETTING0_RXDF			H_CURRENT_BASE_RXDF+JMC0


// For JMC1 sub-controller
// Right Hip Pitch
#define JMC1					0x01
#define REF1_TXDF				REF_BASE_TXDF+JMC1
#define ENC1_RXDF				H_ENC_BASE_RXDF+JMC1
#define CUR1_RXDF				H_CUR_BASE_RXDF+JMC1
#define PM1_RXDF				PM_BASE_RXDF+JMC1
#define STAT1_RXDF				STAT_BASE_RXDF+JMC1
#define NAME1_RXDF				NAME_BASE_RXDF+JMC1
#define SETTING1_RXDF			H_CURRENT_BASE_RXDF+JMC1


// For JMC2 sub-controller
// Right Knee Pitch
#define JMC2					0x02
#define REF2_TXDF				REF_BASE_TXDF+JMC2
#define ENC2_RXDF				H_ENC_BASE_RXDF+JMC2
#define CUR2_RXDF				H_CUR_BASE_RXDF+JMC2
#define PM2_RXDF				PM_BASE_RXDF+JMC2
#define STAT2_RXDF				STAT_BASE_RXDF+JMC2
#define NAME2_RXDF				NAME_BASE_RXDF+JMC2
#define SETTING2_RXDF			H_CURRENT_BASE_RXDF+JMC2


// For JMC3 sub-controller
// Right Ankle Pitch-Roll
#define JMC3					0x03
#define REF3_TXDF				REF_BASE_TXDF+JMC3
#define ENC3_RXDF				H_ENC_BASE_RXDF+JMC3
#define CUR3_RXDF				H_CUR_BASE_RXDF+JMC3
#define PM3_RXDF				PM_BASE_RXDF+JMC3
#define STAT3_RXDF				STAT_BASE_RXDF+JMC3
#define NAME3_RXDF				NAME_BASE_RXDF+JMC3
#define SETTING3_RXDF			H_CURRENT_BASE_RXDF+JMC3


// For JMC4 sub-controller
// Left Hip Yaw-Roll
#define JMC4					0x04
#define REF4_TXDF				REF_BASE_TXDF+JMC4
#define ENC4_RXDF				H_ENC_BASE_RXDF+JMC4
#define CUR4_RXDF				H_CUR_BASE_RXDF+JMC4
#define PM4_RXDF				PM_BASE_RXDF+JMC4
#define STAT4_RXDF				STAT_BASE_RXDF+JMC4
#define NAME4_RXDF				NAME_BASE_RXDF+JMC4
#define SETTING4_RXDF			H_CURRENT_BASE_RXDF+JMC4


// For JMC5 sub-controller
// Left Hip Pitch
#define JMC5					0x05
#define REF5_TXDF				REF_BASE_TXDF+JMC5
#define ENC5_RXDF				H_ENC_BASE_RXDF+JMC5
#define CUR5_RXDF				H_CUR_BASE_RXDF+JMC5
#define PM5_RXDF				PM_BASE_RXDF+JMC5
#define STAT5_RXDF				STAT_BASE_RXDF+JMC5
#define NAME5_RXDF				NAME_BASE_RXDF+JMC5
#define SETTING5_RXDF			H_CURRENT_BASE_RXDF+JMC5


// For JMC6 sub-controller
// Left Knee Pitch
#define JMC6					0x06
#define REF6_TXDF				REF_BASE_TXDF+JMC6
#define ENC6_RXDF				H_ENC_BASE_RXDF+JMC6
#define CUR6_RXDF				H_CUR_BASE_RXDF+JMC6
#define PM6_RXDF				PM_BASE_RXDF+JMC6
#define STAT6_RXDF				STAT_BASE_RXDF+JMC6
#define NAME6_RXDF				NAME_BASE_RXDF+JMC6
#define SETTING6_RXDF			H_CURRENT_BASE_RXDF+JMC6


// For JMC7 sub-controller
// Left Ankle Pitch-Roll
#define JMC7					0x07
#define REF7_TXDF				REF_BASE_TXDF+JMC7
#define ENC7_RXDF				H_ENC_BASE_RXDF+JMC7
#define CUR7_RXDF				H_CUR_BASE_RXDF+JMC7
#define PM7_RXDF				PM_BASE_RXDF+JMC7
#define STAT7_RXDF				STAT_BASE_RXDF+JMC7
#define NAME7_RXDF				NAME_BASE_RXDF+JMC7
#define SETTING7_RXDF			H_CURRENT_BASE_RXDF+JMC7


// For JMC8 sub-controller
// Right Shoulder Pitch & Roll
#define JMC8					0x08
#define REF8_TXDF				REF_BASE_TXDF+JMC8
#define ENC8_RXDF				H_ENC_BASE_RXDF+JMC8
#define CUR8_RXDF				H_CUR_BASE_RXDF+JMC8
#define PM8_RXDF				PM_BASE_RXDF+JMC8
#define STAT8_RXDF				STAT_BASE_RXDF+JMC8
#define NAME8_RXDF				NAME_BASE_RXDF+JMC8
#define SETTING8_RXDF			H_CURRENT_BASE_RXDF+JMC8


// For JMC9 sub-controller
// Right Shoulder Yaw-Elbow
#define JMC9					0x09
#define REF9_TXDF				REF_BASE_TXDF+JMC9
#define ENC9_RXDF				H_ENC_BASE_RXDF+JMC9
#define CUR9_RXDF				H_CUR_BASE_RXDF+JMC9
#define PM9_RXDF				PM_BASE_RXDF+JMC9
#define STAT9_RXDF				STAT_BASE_RXDF+JMC9
#define NAME9_RXDF				NAME_BASE_RXDF+JMC9
#define SETTING9_RXDF			H_CURRENT_BASE_RXDF+JMC9


// For JMC10 sub-controller
// Left Shoulder Pitch & Roll
#define JMC10					0x0A
#define REF10_TXDF				REF_BASE_TXDF+JMC10
#define ENC10_RXDF				H_ENC_BASE_RXDF+JMC10
#define CUR10_RXDF				H_CUR_BASE_RXDF+JMC10
#define PM10_RXDF				PM_BASE_RXDF+JMC10
#define STAT10_RXDF				STAT_BASE_RXDF+JMC10
#define NAME10_RXDF				NAME_BASE_RXDF+JMC10
#define SETTING10_RXDF			H_CURRENT_BASE_RXDF+JMC10


// For JMC11 sub-controller
// Left Shoulder Yaw-Elbow
#define JMC11					0x0B
#define REF11_TXDF				REF_BASE_TXDF+JMC11
#define ENC11_RXDF				H_ENC_BASE_RXDF+JMC11
#define CUR11_RXDF				H_CUR_BASE_RXDF+JMC11
#define PM11_RXDF				PM_BASE_RXDF+JMC11
#define STAT11_RXDF				STAT_BASE_RXDF+JMC11
#define NAME11_RXDF				NAME_BASE_RXDF+JMC11
#define SETTING11_RXDF			H_CURRENT_BASE_RXDF+JMC11


// For EJMC0 sub-controller
// Right Wrist
#define EJMC0					0x20
#define REF_E0_TXDF				REF_BASE_TXDF+EJMC0
#define ENC_E0_RXDF				H_ENC_BASE_RXDF+EJMC0
#define CUR_E0_RXDF				H_CUR_BASE_RXDF+EJMC0
#define PM_E0_RXDF				PM_BASE_RXDF+EJMC0
#define STAT_E0_RXDF			STAT_BASE_RXDF+EJMC0
#define NAME_E0_RXDF			NAME_BASE_RXDF+EJMC0
#define SETTING_E0_RXDF			H_CURRENT_BASE_RXDF+EJMC0


// For EJMC1 sub-controller
// Left Wrist
#define EJMC1					0x21
#define REF_E1_TXDF				REF_BASE_TXDF+EJMC1
#define ENC_E1_RXDF				H_ENC_BASE_RXDF+EJMC1
#define CUR_E1_RXDF				H_CUR_BASE_RXDF+EJMC1
#define PM_E1_RXDF				PM_BASE_RXDF+EJMC1
#define STAT_E1_RXDF			STAT_BASE_RXDF+EJMC1
#define NAME_E1_RXDF			NAME_BASE_RXDF+EJMC1
#define SETTING_E1_RXDF			H_CURRENT_BASE_RXDF+EJMC1


// For EJMC2 sub-controller
// Neck
#define EJMC2					0x22
#define REF_E2_TXDF				REF_BASE_TXDF+EJMC2
#define ENC_E2_RXDF				H_ENC_BASE_RXDF+EJMC2
#define CUR_E2_RXDF				H_CUR_BASE_RXDF+EJMC2
#define PM_E2_RXDF				PM_BASE_RXDF+EJMC2
#define STAT_E2_RXDF			STAT_BASE_RXDF+EJMC2
#define NAME_E2_RXDF			NAME_BASE_RXDF+EJMC2
#define SETTING_E2_RXDF			H_CURRENT_BASE_RXDF+EJMC2


// For EJMC3 sub-controller
// Waist
#define EJMC3					0x23
#define REF_E3_TXDF				REF_BASE_TXDF+EJMC3
#define ENC_E3_RXDF				H_ENC_BASE_RXDF+EJMC3
#define CUR_E3_RXDF				H_CUR_BASE_RXDF+EJMC3
#define PM_E3_RXDF				PM_BASE_RXDF+EJMC3
#define STAT_E3_RXDF			STAT_BASE_RXDF+EJMC3
#define NAME_E3_RXDF			NAME_BASE_RXDF+EJMC3
#define SETTING_E3_RXDF			H_CURRENT_BASE_RXDF+EJMC3


// For EJMC4 sub-controller
// Right Finger
#define EJMC4					0x24
#define REF_E4_TXDF				REF_BASE_TXDF+EJMC4
#define ENC_E4_RXDF				H_ENC_BASE_RXDF+EJMC4
#define CUR_E4_RXDF				H_CUR_BASE_RXDF+EJMC4
#define PM_E4_RXDF				PM_BASE_RXDF+EJMC4
#define STAT_E4_RXDF			STAT_BASE_RXDF+EJMC4
#define NAME_E4_RXDF			NAME_BASE_RXDF+EJMC4
#define SETTING_E4_RXDF			H_CURRENT_BASE_RXDF+EJMC4


// For EJMC5 sub-controller
// Right Finger
#define EJMC5					0x25
#define REF_E5_TXDF				REF_BASE_TXDF+EJMC5
#define ENC_E5_RXDF				H_ENC_BASE_RXDF+EJMC5
#define CUR_E5_RXDF				H_CUR_BASE_RXDF+EJMC5
#define PM_E5_RXDF				PM_BASE_RXDF+EJMC5
#define STAT_E5_RXDF			STAT_BASE_RXDF+EJMC5
#define NAME_E5_RXDF			NAME_BASE_RXDF+EJMC5
#define SETTING_E5_RXDF			H_CURRENT_BASE_RXDF+EJMC5



// Sensors
// For FT0 sub-controller Right
#define FT0						0x30		// Board Number
#define SENSOR_RECEIVE0			0x01
#define SENSOR_FT0_RXDF			SENSOR_FT_BASE_RXDF+SENSOR_RECEIVE0
#define SENSOR_AD0_RXDF			SENSOR_AD_BASE_RXDF+SENSOR_RECEIVE0
#define DAOFFSET0_RXDF			DAOFFSET_BASE_RXDF+SENSOR_RECEIVE0
#define ADOFFSET0_RXDF			ADOFFSET_BASE_RXDF+SENSOR_RECEIVE0
#define OFFSET0_RXDF			OFFSET_BASE_RXDF+SENSOR_RECEIVE0
#define STAT_FT0_RXDF			STAT_BASE_RXDF+FT0
#define NAME_FT0_RXDF			NAME_BASE_RXDF+FT0+0x80


// For FT1 sub-controller Left
#define FT1						0x31		// Board Number
#define SENSOR_RECEIVE1			0x02
#define SENSOR_FT1_RXDF			SENSOR_FT_BASE_RXDF+SENSOR_RECEIVE1
#define SENSOR_AD1_RXDF			SENSOR_AD_BASE_RXDF+SENSOR_RECEIVE1
#define DAOFFSET1_RXDF			DAOFFSET_BASE_RXDF+SENSOR_RECEIVE1
#define ADOFFSET1_RXDF			ADOFFSET_BASE_RXDF+SENSOR_RECEIVE1
#define OFFSET1_RXDF			OFFSET_BASE_RXDF+SENSOR_RECEIVE1
#define STAT_FT1_RXDF			STAT_BASE_RXDF+FT1
#define NAME_FT1_RXDF			NAME_BASE_RXDF+FT1+0x80


// For IMU0 sub-controller
#define IMU0					0x32
#define SENSOR_RECEIVE2			0x03
#define SENSOR_IMU0_RXDF		SENSOR_AD_BASE_RXDF+SENSOR_RECEIVE2
#define STAT_IMU0_RXDF			STAT_BASE_RXDF+IMU0
#define NAME_IMU0_RXDF			NAME_BASE_RXDF+IMU0+0x80

// For IMU1 sub-controller
#define IMU1					0x33
#define SENSOR_RECEIVE3			0x04
#define SENSOR_IMU1_RXDF		SENSOR_AD_BASE_RXDF+SENSOR_RECEIVE3
#define STAT_IMU1_RXDF			STAT_BASE_RXDF+IMU1
#define NAME_IMU1_RXDF			NAME_BASE_RXDF+IMU1+0x80

// For IMU2 sub-controller
#define IMU2					0x34
#define SENSOR_RECEIVE4			0x05
#define SENSOR_IMU2_RXDF		SENSOR_AD_BASE_RXDF+SENSOR_RECEIVE4
#define STAT_IMU2_RXDF			STAT_BASE_RXDF+IMU2
#define NAME_IMU2_RXDF			NAME_BASE_RXDF+IMU2+0x80


// For FT2 sub-controller Left for right wrist
#define FT2						0x35
#define SENSOR_RECEIVE6			0x06
#define SENSOR_FT2_RXDF			SENSOR_FT_BASE_RXDF+SENSOR_RECEIVE6
#define SENSOR_AD2_RXDF			SENSOR_AD_BASE_RXDF+SENSOR_RECEIVE6
#define DAOFFSET2_RXDF			DAOFFSET_BASE_RXDF+SENSOR_RECEIVE6
#define ADOFFSET2_RXDF			ADOFFSET_BASE_RXDF+SENSOR_RECEIVE6
#define OFFSET2_RXDF			OFFSET_BASE_RXDF+SENSOR_RECEIVE6
#define STAT_FT2_RXDF			STAT_BASE_RXDF+FT2
#define NAME_FT2_RXDF			NAME_BASE_RXDF+FT2+0x80

// For FT3 sub-controller Left for right wrist
#define FT3						0x36
#define SENSOR_RECEIVE7			0x07
#define SENSOR_FT3_RXDF			SENSOR_FT_BASE_RXDF+SENSOR_RECEIVE7
#define SENSOR_AD3_RXDF			SENSOR_AD_BASE_RXDF+SENSOR_RECEIVE7
#define DAOFFSET3_RXDF			DAOFFSET_BASE_RXDF+SENSOR_RECEIVE7
#define ADOFFSET3_RXDF			ADOFFSET_BASE_RXDF+SENSOR_RECEIVE7
#define OFFSET3_RXDF			OFFSET_BASE_RXDF+SENSOR_RECEIVE7
#define STAT_FT3_RXDF			STAT_BASE_RXDF+FT3
#define NAME_FT3_RXDF			NAME_BASE_RXDF+FT3+0x80


// Command(Not CAN ID)
#define AllController				0x20

// Motor Board Information Requests ( H_ = Hex )
#define H_GET_BOARD_INFO			0x01
#define H_GET_STATUS				0x02
#define H_GET_ENCODER				0x03 // Request an encoder position value
#define H_GET_CURRENT				0x04 // Request a current (amp) value
//#define SendPM					0x05	// I don't know where this comes from
#define H_SET_ENC_ZERO				0x06
#define H_SET_POS_GAIN_0			0x07
#define H_SET_POS_GAIN_1			0x08
#define H_SET_CUR_GAIN_0			0x09
#define H_SET_CUR_GAIN_1			0x0A
#define H_SWITCH_DRIVER				0x0B
//#define GoHome					0x0C	// I don't know where this comes from
#define H_OPENLOOP_PWM				0x0D
#define H_MOTOR_CTRL_ON				0x0E
#define H_MOTOR_CTRL_OFF			0x0F
#define H_SET_CTRL_MODE				0x10
#define H_HOME					0x11
#define H_SET_DEADZONE				0x20 // "Set the value of Dead zone to remove FET's PWM null"
#define H_GET_PARAMETERS			0x24 // Get the board parameters
#define H_SET_HOME_PARAM			0x30
#define H_SET_ENC_RES				0x38 // Set encoder resolution
#define H_SET_MAX_ACC_VEL			0x40 // Set maximum acceleration and velocity
#define H_SET_LOW_POS_LIM			0x50 // Set lower position limit
#define H_SET_UPP_POS_LIM			0x56 // Set upper position limit
#define H_SET_HOME_VEL_ACC			0x60
#define H_GAIN_OVERRIDE				0x6F
#define H_SET_BOARD_NUM				0xF0 // Change the number and CAN rate of a board
#define H_SET_JAM_SAT_LIM			0xF2
#define H_SET_ERR_BOUND				0xF3
#define H_INIT_BOARD				0xFA




#define H_REQ_NULL				0x81
#define H_NULL_FT				0x00
#define H_NULL_ACC				0x04
#define H_INIT_DEFAULT_2		0xAA


#define SetPeriod				0x82
#define SetSample				0x83
#define ADRead					0x84
#define DAOut					0x85
#define DataTypeCMD				0x86
#define DAOffsetCMD				0x87
#define ADOffsetCMD				0x88
#define OffsetCMD				0x89



// Auxiliary parameters

#define H_ALARM		0x82
#define H_ALARM_OFF	0x00
#define H_ALARM_S1	0x01	// Sound 1
#define H_ALARM_S2	0x02	// Sound 2
#define H_ALARM_S3	0x03	// Sound 3
#define H_ALARM_S4	0x04	// Sound 4

#define H_BEEP		0x83

#define H_VCREAD	0xE0	// Request voltage and current readings




// Sensor requests
#define H_GET_FT_ACC_DIGIT	0x00
#define H_GET_FT_ACC_SCALED	0x02
#define H_GET_FT_S_ACC_D	0x03	// Get scaled force-torque and digital acceleration
#define H_GET_FT_D_ACC_S	0x04	// Get digital force-torque and scaled acceleration
#define H_GET_FT_DIGIT		0x11
#define H_GET_FT_SCALED		0x12
#define H_GET_ACC_DIGIT		0x21
#define H_GET_ACC_SCALED	0x22
#define H_GET_GYRO_TEMP		0x13


// Parameter requests
#define H_REQ_PARAMS		0x24
#define H_GET_PARAM_A		1
#define H_GET_PARAM_B		2
#define H_GET_PARAM_C		3
#define H_GET_PARAM_D		4
#define H_GET_PARAM_E		5
#define H_GET_PARAM_F		6
#define H_GET_PARAM_G		20
#define H_GET_PARAM_H		21
#define H_GET_PARAM_I		22


// Error status stuff
#define H_HOME_SUCCESS      0x06

#ifdef __cplusplus
}
#endif

#endif
