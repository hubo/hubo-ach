#ifndef CONTROLDAEMON_H
#define CONTROLDAEMON_H

// For Hubo
#include "hubo.h"
#include "hubo-jointparams.h"

// For control
#include <math.h>
#include "daemonizer.h"

#define ARM_JOINT_COUNT 6
#define LEG_JOINT_COUNT 6
#define FIN_JOINT_COUNT 5
#define AUX_JOINT_COUNT 4

#define     HUBO_CHAN_RL_CTRL_NAME      "hubo-RL-control" // Right Leg control channel
#define     HUBO_CHAN_LL_CTRL_NAME      "hubo-LL-control" // Left Leg control channel
#define     HUBO_CHAN_RA_CTRL_NAME      "hubo-RA-control" // Right Arm control channel
#define     HUBO_CHAN_LA_CTRL_NAME      "hubo-LA-control" // Left Arm control channel
#define     HUBO_CHAN_RF_CTRL_NAME      "hubo-RF-control" // Right Finger control channel
#define     HUBO_CHAN_LF_CTRL_NAME      "hubo-LF-control" // Left Finger control channel
#define     HUBO_CHAN_AUX_CTRL_NAME     "hubo-AUX-control"// Neck and Waist control channel


// TODO: Save these as parameters defined in a table instead:
const int leftarmjoints[ARM_JOINT_COUNT]  = { LSP, LSR, LSY, LEB, LWY, LWP };
const int rightarmjoints[ARM_JOINT_COUNT] = { RSP, RSR, RSY, REB, RWY, RWP };
const int leftlegjoints[LEG_JOINT_COUNT]  = { LHP, LHR, LHY, LKN, LAP, LAR };
const int rightlegjoints[LEG_JOINT_COUNT] = { RHP, RHR, RHY, RKN, RAP, RAR };
const int leftfinjoints[FIN_JOINT_COUNT]  = { LF1, LF2, LF3, LF4, LF5 };
const int rightfinjoints[FIN_JOINT_COUNT]  = { RF1, RF2, RF3, RF4, RF5 };
const int auxjoints[AUX_JOINT_COUNT] = { WST, NKY, NK1, NK2 }; 

typedef enum {
    CTRL_OFF    = 0,
    CTRL_POS,
    CTRL_VEL,
    CTRL_HOME,
    CTRL_RESET
} hubo_ctrl_mode_t;


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

struct hubo_arm_control {
    struct hubo_joint_control joint[ARM_JOINT_COUNT];
    int active;
};

struct hubo_leg_control {
    struct hubo_joint_control joint[LEG_JOINT_COUNT];
    int active;
};

struct hubo_fin_control {
    struct hubo_joint_control joint[FIN_JOINT_COUNT];
    int active;
};

struct hubo_aux_control {
    struct hubo_joint_control joint[AUX_JOINT_COUNT];
    int active;
};



#endif // CONTROLDAEMON_H
