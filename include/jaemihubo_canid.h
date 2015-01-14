#define jaemi_CMD_TXDF				0x01
#define jaemi_SEND_SENSOR_TXDF		0x02

#define jaemi_REF_BASE_TXDF			0x10
#define jaemi_ENC_BASE_RXDF			0x60
#define jaemi_CUR_BASE_RXDF			0x90
#define jaemi_PM_BASE_RXDF			0x120
#define jaemi_SENSOR_FT_BASE_RXDF		0x40
#define jaemi_SENSOR_AD_BASE_RXDF		0x50
#define jaemi_DAOFFSET_BASE_RXDF		0x310
#define jaemi_ADOFFSET_BASE_RXDF		0x320
#define jaemi_OFFSET_BASE_RXDF		0x330
#define jaemi_STAT_BASE_RXDF			0x150
#define jaemi_NAME_BASE_RXDF			0x190

// REF_BASE_TXDF + 0x03 = REF3_TXDT øÕ ∞∞¿Ã ªÁøÎµ»¥Ÿ.

// For JMC0 sub-controller
// Right Hip Yaw-Roll
#define jaemi_JMC0					0x00
#define jaemi_REF0_TXDF				REF_BASE_TXDF+JMC0
#define jaemi_ENC0_RXDF				ENC_BASE_RXDF+JMC0
#define jaemi_CUR0_RXDF				CUR_BASE_RXDF+JMC0
#define jaemi_PM0_RXDF				PM_BASE_RXDF+JMC0
#define jaemi_STAT0_RXDF				STAT_BASE_RXDF+JMC0
#define jaemi_NAME0_RXDF				NAME_BASE_RXDF+JMC0
#define jaemi_KP0						0x10
#define jaemi_KI0						0x10
#define jaemi_KD0						0x10

// For JMC1 sub-controller
// Right Hip Pitch
#define jaemi_JMC1					0x01
#define jaemi_REF1_TXDF				REF_BASE_TXDF+JMC1
#define jaemi_ENC1_RXDF				ENC_BASE_RXDF+JMC1
#define jaemi_CUR1_RXDF				CUR_BASE_RXDF+JMC1
#define jaemi_PM1_RXDF				PM_BASE_RXDF+JMC1
#define jaemi_STAT1_RXDF				STAT_BASE_RXDF+JMC1
#define jaemi_NAME1_RXDF				NAME_BASE_RXDF+JMC1
#define jaemi_KP1						0x10
#define jaemi_KI1						0x10
#define jaemi_KD1						0x10

// For JMC2 sub-controller
// Right Knee Pitch
#define jaemi_JMC2					0x02
#define jaemi_REF2_TXDF				REF_BASE_TXDF+JMC2
#define jaemi_ENC2_RXDF				ENC_BASE_RXDF+JMC2
#define jaemi_CUR2_RXDF				CUR_BASE_RXDF+JMC2
#define jaemi_PM2_RXDF				PM_BASE_RXDF+JMC2
#define jaemi_STAT2_RXDF				STAT_BASE_RXDF+JMC2
#define jaemi_NAME2_RXDF				NAME_BASE_RXDF+JMC2
#define jaemi_KP2						0x10
#define jaemi_KI2						0x10
#define jaemi_KD2						0x10

// For JMC3 sub-controller
// Right Ankle Pitch-Roll
#define jaemi_JMC3					0x03
#define jaemi_REF3_TXDF				REF_BASE_TXDF+JMC3
#define jaemi_ENC3_RXDF				ENC_BASE_RXDF+JMC3
#define jaemi_CUR3_RXDF				CUR_BASE_RXDF+JMC3
#define jaemi_PM3_RXDF				PM_BASE_RXDF+JMC3
#define jaemi_STAT3_RXDF				STAT_BASE_RXDF+JMC3
#define jaemi_NAME3_RXDF				NAME_BASE_RXDF+JMC3
#define jaemi_KP3						0x10
#define jaemi_KI3						0x10
#define jaemi_KD3						0x10

// For JMC4 sub-controller
// Left Hip Yaw-Roll
#define jaemi_JMC4					0x04
#define jaemi_REF4_TXDF				REF_BASE_TXDF+JMC4
#define jaemi_ENC4_RXDF				ENC_BASE_RXDF+JMC4
#define jaemi_CUR4_RXDF				CUR_BASE_RXDF+JMC4
#define jaemi_PM4_RXDF				PM_BASE_RXDF+JMC4
#define jaemi_STAT4_RXDF				STAT_BASE_RXDF+JMC4
#define jaemi_NAME4_RXDF				NAME_BASE_RXDF+JMC4
#define jaemi_KP4						0x10
#define jaemi_KI4						0x10
#define jaemi_KD4						0x10

// For JMC5 sub-controller
// Left Hip Pitch
#define jaemi_JMC5					0x05
#define jaemi_REF5_TXDF				REF_BASE_TXDF+JMC5
#define jaemi_ENC5_RXDF				ENC_BASE_RXDF+JMC5
#define jaemi_CUR5_RXDF				CUR_BASE_RXDF+JMC5
#define jaemi_PM5_RXDF				PM_BASE_RXDF+JMC5
#define jaemi_STAT5_RXDF				STAT_BASE_RXDF+JMC5
#define jaemi_NAME5_RXDF				NAME_BASE_RXDF+JMC5
#define jaemi_KP5						0x10
#define jaemi_KI5						0x10
#define jaemi_KD5						0x10

// For JMC6 sub-controller
// Left Knee Pitch
#define jaemi_JMC6					0x06
#define jaemi_REF6_TXDF				REF_BASE_TXDF+JMC6
#define jaemi_ENC6_RXDF				ENC_BASE_RXDF+JMC6
#define jaemi_CUR6_RXDF				CUR_BASE_RXDF+JMC6
#define jaemi_PM6_RXDF				PM_BASE_RXDF+JMC6
#define jaemi_STAT6_RXDF				STAT_BASE_RXDF+JMC6
#define jaemi_NAME6_RXDF				NAME_BASE_RXDF+JMC6
#define jaemi_KP6						0x10
#define jaemi_KI6						0x10
#define jaemi_KD6						0x10

// For JMC7 sub-controller
// Left Ankle Pitch-Roll
#define jaemi_JMC7					0x07
#define jaemi_REF7_TXDF				REF_BASE_TXDF+JMC7
#define jaemi_ENC7_RXDF				ENC_BASE_RXDF+JMC7
#define jaemi_CUR7_RXDF				CUR_BASE_RXDF+JMC7
#define jaemi_PM7_RXDF				PM_BASE_RXDF+JMC7
#define jaemi_STAT7_RXDF				STAT_BASE_RXDF+JMC7
#define jaemi_NAME7_RXDF				NAME_BASE_RXDF+JMC7
#define jaemi_KP7						0x10
#define jaemi_KI7						0x10
#define jaemi_KD7						0x10

// For JMC8 sub-controller
// Right Shoulder Pitch & Roll
#define jaemi_JMC8					0x08
#define jaemi_REF8_TXDF				REF_BASE_TXDF+JMC8
#define jaemi_ENC8_RXDF				ENC_BASE_RXDF+JMC8
#define jaemi_CUR8_RXDF				CUR_BASE_RXDF+JMC8
#define jaemi_PM8_RXDF				PM_BASE_RXDF+JMC8
#define jaemi_STAT8_RXDF				STAT_BASE_RXDF+JMC8
#define jaemi_NAME8_RXDF				NAME_BASE_RXDF+JMC8
#define jaemi_KP8						0x10
#define jaemi_KI8						0x10
#define jaemi_KD8						0x10

// For JMC9 sub-controller
// Right Shoulder Yaw-Elbow
#define jaemi_JMC9					0x09
#define jaemi_REF9_TXDF				REF_BASE_TXDF+JMC9
#define jaemi_ENC9_RXDF				ENC_BASE_RXDF+JMC9
#define jaemi_CUR9_RXDF				CUR_BASE_RXDF+JMC9
#define jaemi_PM9_RXDF				PM_BASE_RXDF+JMC9
#define jaemi_STAT9_RXDF				STAT_BASE_RXDF+JMC9
#define jaemi_NAME9_RXDF				NAME_BASE_RXDF+JMC9
#define jaemi_KP9						0x10
#define jaemi_KI9						0x10
#define jaemi_KD9						0x10

// For JMC10 sub-controller
// Left Shoulder Pitch & Roll
#define jaemi_JMC10					0x0A
#define jaemi_REF10_TXDF				REF_BASE_TXDF+JMC10
#define jaemi_ENC10_RXDF				ENC_BASE_RXDF+JMC10
#define jaemi_CUR10_RXDF				CUR_BASE_RXDF+JMC10
#define jaemi_PM10_RXDF				PM_BASE_RXDF+JMC10
#define jaemi_STAT10_RXDF				STAT_BASE_RXDF+JMC10
#define jaemi_NAME10_RXDF				NAME_BASE_RXDF+JMC10
#define jaemi_KP10					0x10
#define jaemi_KI10					0x10
#define jaemi_KD10					0x10

// For JMC11 sub-controller
// Left Shoulder Yaw-Elbow
#define jaemi_JMC11					0x0B
#define jaemi_REF11_TXDF				REF_BASE_TXDF+JMC11
#define jaemi_ENC11_RXDF				ENC_BASE_RXDF+JMC11
#define jaemi_CUR11_RXDF				CUR_BASE_RXDF+JMC11
#define jaemi_PM11_RXDF				PM_BASE_RXDF+JMC11
#define jaemi_STAT11_RXDF				STAT_BASE_RXDF+JMC11
#define jaemi_NAME11_RXDF				NAME_BASE_RXDF+JMC11
#define jaemi_KP11					0x10
#define jaemi_KI11					0x10
#define jaemi_KD11					0x10


// For EJMC0 sub-controller
// Right Wrist
#define jaemi_EJMC0					0x20
#define jaemi_REF32_TXDF				REF_BASE_TXDF+EJMC0
#define jaemi_ENC32_RXDF				ENC_BASE_RXDF+EJMC0
#define jaemi_CUR32_RXDF				CUR_BASE_RXDF+EJMC0
#define jaemi_PM32_RXDF				PM_BASE_RXDF+EJMC0
#define jaemi_STAT32_RXDF				STAT_BASE_RXDF+EJMC0
#define jaemi_NAME32_RXDF				NAME_BASE_RXDF+EJMC0
#define jaemi_KP32					0x10
#define jaemi_KI32					0x10
#define jaemi_KD32					0x10

// For EJMC1 sub-controller
// Left Wrist
#define jaemi_EJMC1					0x21
#define jaemi_REF33_TXDF				REF_BASE_TXDF+EJMC1
#define jaemi_ENC33_RXDF				ENC_BASE_RXDF+EJMC1
#define jaemi_CUR33_RXDF				CUR_BASE_RXDF+EJMC1
#define jaemi_PM33_RXDF				PM_BASE_RXDF+EJMC1
#define jaemi_STAT33_RXDF				STAT_BASE_RXDF+EJMC1
#define jaemi_NAME33_RXDF				NAME_BASE_RXDF+EJMC1
#define jaemi_KP33					0x10
#define jaemi_KI33					0x10
#define jaemi_KD33					0x10

// For EJMC2 sub-controller
// Neck
#define jaemi_EJMC2					0x22
#define jaemi_REF34_TXDF				REF_BASE_TXDF+EJMC2
#define jaemi_ENC34_RXDF				ENC_BASE_RXDF+EJMC2
#define jaemi_CUR34_RXDF				CUR_BASE_RXDF+EJMC2
#define jaemi_PM34_RXDF				PM_BASE_RXDF+EJMC2
#define jaemi_STAT34_RXDF				STAT_BASE_RXDF+EJMC2
#define jaemi_NAME34_RXDF				NAME_BASE_RXDF+EJMC2
#define jaemi_KP34					0x10
#define jaemi_KI34					0x10
#define jaemi_KD34					0x10


// For EJMC3 sub-controller
// Waist
#define jaemi_EJMC3					0x23	//0x23
#define jaemi_REF35_TXDF				REF_BASE_TXDF+EJMC3
#define jaemi_ENC35_RXDF				ENC_BASE_RXDF+EJMC3
#define jaemi_CUR35_RXDF				CUR_BASE_RXDF+EJMC3
#define jaemi_PM35_RXDF				PM_BASE_RXDF+EJMC3
#define jaemi_STAT35_RXDF				STAT_BASE_RXDF+EJMC3
#define jaemi_NAME35_RXDF				NAME_BASE_RXDF+EJMC3
#define jaemi_KP35					0x10
#define jaemi_KI35					0x10
#define jaemi_KD35					0x10


// For EJMC4 sub-controller
// Right Finger
#define jaemi_EJMC4					0x24
#define jaemi_REF36_TXDF				REF_BASE_TXDF+EJMC4
#define jaemi_ENC36_RXDF				ENC_BASE_RXDF+EJMC4
#define jaemi_CUR36_RXDF				CUR_BASE_RXDF+EJMC4
#define jaemi_PM36_RXDF				PM_BASE_RXDF+EJMC4
#define jaemi_STAT36_RXDF				STAT_BASE_RXDF+EJMC4
#define jaemi_NAME36_RXDF				NAME_BASE_RXDF+EJMC4
#define jaemi_KP36					0x10
#define jaemi_KI36					0x10
#define jaemi_KD36					0x10

// For EJMC5 sub-controller
// Right Finger
#define jaemi_EJMC5					0x25
#define jaemi_REF37_TXDF				REF_BASE_TXDF+EJMC5
#define jaemi_ENC37_RXDF				ENC_BASE_RXDF+EJMC5
#define jaemi_CUR37_RXDF				CUR_BASE_RXDF+EJMC5
#define jaemi_PM37_RXDF				PM_BASE_RXDF+EJMC5
#define jaemi_STAT37_RXDF				STAT_BASE_RXDF+EJMC5
#define jaemi_NAME37_RXDF				NAME_BASE_RXDF+EJMC5
#define jaemi_KP37					0x10
#define jaemi_KI37					0x10
#define jaemi_KD37					0x10


// Sensors
// For FT1 sub-controller Right
#define jaemi_FT1						0x30		// Board Number
#define jaemi_SENSOR_RECEIVE1			0x01
#define jaemi_SENSOR_FT1_RXDF			SENSOR_FT_BASE_RXDF+SENSOR_RECEIVE1
#define jaemi_SENSOR_AD1_RXDF			SENSOR_AD_BASE_RXDF+SENSOR_RECEIVE1
#define jaemi_DAOFFSET1_RXDF			DAOFFSET_BASE_RXDF+SENSOR_RECEIVE1
#define jaemi_ADOFFSET1_RXDF			ADOFFSET_BASE_RXDF+SENSOR_RECEIVE1
#define jaemi_OFFSET1_RXDF			OFFSET_BASE_RXDF+SENSOR_RECEIVE1
#define jaemi_STAT_FT1_RXDF			STAT_BASE_RXDF+FT1
#define jaemi_NAME_FT1_RXDF			NAME_BASE_RXDF+FT1

// For FT2 sub-controller Left
#define jaemi_FT2						0x31
#define jaemi_SENSOR_RECEIVE2			0x02
#define jaemi_SENSOR_FT2_RXDF			SENSOR_FT_BASE_RXDF+SENSOR_RECEIVE2
#define jaemi_SENSOR_AD2_RXDF			SENSOR_AD_BASE_RXDF+SENSOR_RECEIVE2
#define jaemi_DAOFFSET2_RXDF			DAOFFSET_BASE_RXDF+SENSOR_RECEIVE2
#define jaemi_ADOFFSET2_RXDF			ADOFFSET_BASE_RXDF+SENSOR_RECEIVE2
#define jaemi_OFFSET2_RXDF			OFFSET_BASE_RXDF+SENSOR_RECEIVE2
#define jaemi_STAT_FT2_RXDF			STAT_BASE_RXDF+FT2
#define jaemi_NAME_FT2_RXDF			NAME_BASE_RXDF+FT2

// For IMU1 sub-controller
#define jaemi_IMU1					0x32
#define jaemi_SENSOR_RECEIVE3			0x03
#define jaemi_SENSOR_IMU1_RXDF		SENSOR_AD_BASE_RXDF+SENSOR_RECEIVE3
#define jaemi_STAT_IMU1_RXDF			STAT_BASE_RXDF+IMU1
#define jaemi_NAME_IMU1_RXDF			NAME_BASE_RXDF+IMU1

// For IMU2 sub-controller
#define jaemi_IMU2					0x33
#define jaemi_SENSOR_RECEIVE4			0x04
#define jaemi_SENSOR_IMU2_RXDF		SENSOR_AD_BASE_RXDF+SENSOR_RECEIVE4
#define jaemi_STAT_IMU2_RXDF			STAT_BASE_RXDF+IMU2
#define jaemi_NAME_IMU2_RXDF			NAME_BASE_RXDF+IMU2

// For IMU3 sub-controller
#define jaemi_IMU3					0x34
#define jaemi_SENSOR_RECEIVE5			0x05
#define jaemi_SENSOR_IMU3_RXDF		SENSOR_AD_BASE_RXDF+SENSOR_RECEIVE5
#define jaemi_STAT_IMU3_RXDF			STAT_BASE_RXDF+IMU3
#define jaemi_NAME_IMU3_RXDF			NAME_BASE_RXDF+IMU3

// For FT3 sub-controller Left for right wrist
#define jaemi_FT3						0x35
#define jaemi_SENSOR_RECEIVE6			0x06
#define jaemi_SENSOR_FT3_RXDF			SENSOR_FT_BASE_RXDF+SENSOR_RECEIVE6
#define jaemi_SENSOR_AD3_RXDF			SENSOR_AD_BASE_RXDF+SENSOR_RECEIVE6
#define jaemi_DAOFFSET3_RXDF			DAOFFSET_BASE_RXDF+SENSOR_RECEIVE6
#define jaemi_ADOFFSET3_RXDF			ADOFFSET_BASE_RXDF+SENSOR_RECEIVE6
#define jaemi_OFFSET3_RXDF			OFFSET_BASE_RXDF+SENSOR_RECEIVE6
#define jaemi_STAT_FT3_RXDF			STAT_BASE_RXDF+FT3
#define jaemi_NAME_FT3_RXDF			NAME_BASE_RXDF+FT3

// For FT4 sub-controller Left for right wrist
#define jaemi_FT4						0x36
#define jaemi_SENSOR_RECEIVE7			0x07
#define jaemi_SENSOR_FT4_RXDF			SENSOR_FT_BASE_RXDF+SENSOR_RECEIVE7
#define jaemi_SENSOR_AD4_RXDF			SENSOR_AD_BASE_RXDF+SENSOR_RECEIVE7
#define jaemi_DAOFFSET4_RXDF			DAOFFSET_BASE_RXDF+SENSOR_RECEIVE7
#define jaemi_ADOFFSET4_RXDF			ADOFFSET_BASE_RXDF+SENSOR_RECEIVE7
#define jaemi_OFFSET4_RXDF			OFFSET_BASE_RXDF+SENSOR_RECEIVE7
#define jaemi_STAT_FT4_RXDF			STAT_BASE_RXDF+FT4
#define jaemi_NAME_FT4_RXDF			NAME_BASE_RXDF+FT4

// Command(Not CAN ID)
#define jaemi_AllController			0x20		
#define jaemi_NameInfo				0x01		
#define jaemi_BoardStatus				0x02		
#define jaemi_SendEncoder				0x03		
#define jaemi_SendCurrent				0x04		
#define jaemi_SendPM					0x05		
#define jaemi_EncZero					0x06		
#define jaemi_SetPosGainA				0x07		
#define jaemi_SetPosGainB				0x08		
#define jaemi_SetTorqueGainA			0x09		
#define jaemi_SetTorqueGainB			0x0A		
#define jaemi_HipEnable				0x0B		
#define jaemi_GoHome					0x0C		
#define jaemi_PwmCMD					0x0D		
#define jaemi_RunCMD					0x0E		
#define jaemi_StopCMD					0x0F		
#define jaemi_ControlMode				0x10		
#define jaemi_GoLimitPos				0x11		
#define jaemi_TorqueLimit				0x12		

#define jaemi_NullCMD					0x81		
#define jaemi_SetPeriod				0x82		 
#define jaemi_SetSample				0x83		
#define jaemi_ADRead					0x84		
#define jaemi_DAOut					0x85		
#define jaemi_DataTypeCMD				0x86		
#define jaemi_DAOffsetCMD				0x87		
#define jaemi_ADOffsetCMD				0x88		
#define jaemi_OffsetCMD				0x89		


					
