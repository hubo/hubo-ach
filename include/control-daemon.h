#ifndef CONTROLDAEMON_H
#define CONTROLDAEMON_H

// For Hubo
#include "hubo.h"
#include "hubo-jointparams.h"

// For Ach
#include <ach.h>

// For control
#include <math.h>
#include "daemonizer.h"


ach_channel_t chan_hubo_ref;
ach_channel_t chan_hubo_board_cmd;
ach_channel_t chan_hubo_state;
ach_channel_t chan_hubo_ctrl;

void controlLoop();













#endif // CONTROLDAEMON_H
