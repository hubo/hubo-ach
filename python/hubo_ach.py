#!/usr/bin/env python
from ctypes import *

HUBO_JOINT_COUNT         = 42
HUBO_CHAN_REF_NAME       = 'hubo-ref'        
HUBO_CHAN_BOARD_CMD_NAME = 'hubo-board-cmd'
HUBO_CHAN_STATE_NAME     = 'hubo-state'     


class HUBO_REF(Structure):
    _fields_ = [("ref",  c_double*HUBO_JOINT_COUNT),
                ("mode", c_int*HUBO_JOINT_COUNT)]
