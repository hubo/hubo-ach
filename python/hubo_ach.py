#!/usr/bin/env python
# /* -*-  indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
# /*
# Copyright (c) 2013, Daniel M. Lofaro
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the author nor the names of its contributors may
#       be used to endorse or promote products derived from this software
#       without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# */

# from ctypes import *
from ctypes import Structure,c_uint16,c_double,c_ubyte,c_uint32,c_int16
# import ach
# import sys


HUBO_FT_R_HAND    = 0 # Index of right hand FT
HUBO_FT_L_HAND    = 1 # Index of left hand FT
HUBO_FT_R_FOOT    = 2 # Index of right foot FT
HUBO_FT_L_FOOT    = 3 # Index of left foot FT
HUBO_IMU0	  = 4 # Index of IMU0
HUBO_IMU1	  = 5 # Index of IMU1
HUBO_IMU2	  = 6 # Index of IMU2

HUBO_JOINT_COUNT                  = 42
HUBO_JMC_COUNT                    = 0x26
HUBO_IMU_COUNT                    = 3
HUBO_CHAN_REF_NAME                = 'hubo-ref'
HUBO_CHAN_BOARD_CMD_NAME          = 'hubo-board-cmd'
HUBO_CHAN_STATE_NAME              = 'hubo-state'
HUBO_CHAN_VIRTUAL_TO_SIM_NAME     = 'hubo-virtual-to-sim'
HUBO_CHAN_VIRTUAL_FROM_SIM_NAME   = 'hubo-virtual-from-sim'
HUBO_LOOP_PERIOD                  = 0.005

RHY = 26 # Right Hip Yaw
RHR = 27 # Right Hip Roll
RHP = 28 # Right Hip Pitch
RKN = 29 # Right Knee Pitch
RAP = 30 # Right Ankle Pitch
RAR = 31 # Right Ankle Roll

LHY = 19 # Left Hip Yaw
LHR = 20 # Left Hip Roll
LHP = 21 # Left Hip Pitch
LKN = 22 # Left Knee Pitch
LAP = 23 # Left Ankle Pitch
LAR = 24 # Left Ankle Roll

RSP = 11 # Right Shoulder Pitch
RSR = 12 # Right Shoulder Roll
RSY = 13 # Right Shoulder Yaw
REB = 14 # Right Elbow Pitch
RWY = 15 # Right Wrist Yaw
RWR = 16 # Right Wrist Roll
RWP = 17 # Right Wrist Pitch

LSP = 4 # Left Shoulder Pitch
LSR = 5 # Left Shoulder Yaw
LSY = 6 # Left Shoulder Roll
LEB = 7 # Left Elbow Pitch
LWY = 8 # Left Wrist yaw
LWR = 9 # Left Wrist Roll
LWP = 10# Left Wrist Pitch

NKY = 1 # Neck Yaw
NK1 = 2 # Neck 1
NK2 = 3 # Neck 2

WST = 0 # Trunk Yaw

RF1 = 32 # Right Finger
RF2 = 33 # Right Finger
RF3 = 34 # Right Finger
RF4 = 35 # Right Finger
RF5 = 36 # Right Finger
LF1 = 37 # Left Finger
LF2 = 38 # Left Finger
LF3 = 39 # Left Finger
LF4 = 40 # Left Finger
LF5 = 41 # Left Finger


class HUBO_VIRTUAL(Structure):
    _pack_ = 1
    _fields_ = [("time"  , c_double)]

class HUBO_SENSOR_PARAM(Structure):
    _pack_ = 1
    _fields_ = [("sensNo"  , c_uint16),
                ("can"     , c_uint16),
                ("boardNo" , c_uint16),
                ("active"  , c_ubyte),
                ("name"    , c_ubyte*5)]

class HUBO_JOINT_PARAM(Structure):
    _pack_ = 1
    _fields_ = [("refEnc"   , c_uint32),
                ("motNo"    , c_uint16),
                ("jntNo"    , c_uint16),
                ("drive"    , c_uint16),
                ("driven"   , c_uint16),
                ("harmonic" , c_uint16),
                ("enc"      , c_uint16),
                ("jmc"      , c_uint16),
                ("dir"      , c_ubyte),
                ("can"      , c_ubyte),
                ("numMot"   , c_ubyte),
                ("name"     , c_ubyte*4)]

class HUBO_JMC_PARAM(Structure):
    _pack_ = 1
    _fields_ = [("joints" , c_ubyte*5)]


class HUBO_PARAM(Structure):
    _pack_ = 1
    _fields_ = [("joint"  , HUBO_JOINT_PARAM*HUBO_JOINT_COUNT),
                ("driver" , HUBO_JMC_PARAM*HUBO_JOINT_COUNT),
                ("sensor" , HUBO_SENSOR_PARAM*HUBO_JOINT_COUNT)]


class HUBO_IMU(Structure):
    _pack_ = 1
    _fields_ = [("a_x", c_double),
                ("a_y", c_double),
                ("a_z", c_double),
                ("w_x", c_double),
                ("w_y", c_double),
                ("w_z", c_double)]

class HUBO_FT(Structure):
    _pack_ = 1
    _fields_ = [("m_x", c_double),
                ("m_y", c_double),
                ("f_z", c_double)]

class HUBO_JOINT_STATE(Structure):
    _pack_ = 1
    _fields_ = [("ref"   , c_double),
                ("pos"   , c_double),
                ("cur"   , c_double),
                ("vel"   , c_double),
                ("duty"   , c_double),
                ("heat"  , c_double),
                ("tmp"   , c_double),
                ("active", c_ubyte),
                ("zeroed", c_ubyte)]

class HUBO_JOINT_STATUS(Structure):
    _pack_ = 1
    _fields_ = [("driveOn"      , c_ubyte),
                ("ctrlOn"       , c_ubyte),
                ("mode"         , c_ubyte),
                ("limitSwitch"  , c_ubyte),
                ("homeFlag"     , c_ubyte),
                ("jam"          , c_ubyte),
                ("pwmSaturated" , c_ubyte),
                ("bigError"     , c_ubyte),
                ("encError"     , c_ubyte),
                ("driverFault"  , c_ubyte),
                ("motorFail0"   , c_ubyte),
                ("motorFail1"   , c_ubyte),
                ("posMinError"  , c_ubyte),
                ("posMaxError"  , c_ubyte),
                ("velError"     , c_ubyte),
                ("accError"     , c_ubyte),
                ("tempError"    , c_ubyte)]

class HUBO_JMC_STATE(Structure):
    _pack_ = 1
    _fields_ = [("temp" , c_double)]

class HUBO_STATE(Structure):
    _pack_ = 1
    _fields_ = [("imu"    , HUBO_IMU*HUBO_IMU_COUNT),
                ("ft"     , HUBO_FT*4),
                ("joint"  , HUBO_JOINT_STATE*HUBO_JOINT_COUNT),
                ("status" , HUBO_JOINT_STATUS*HUBO_JOINT_COUNT),
                ("driver" , HUBO_JMC_STATE*HUBO_JMC_COUNT),
                ("time"   , c_double),
                ("refWait", c_int16)]




class HUBO_REF(Structure):
    _pack_ = 1
    _fields_ = [("ref",  c_double*HUBO_JOINT_COUNT),
                ("mode", c_int16*HUBO_JOINT_COUNT)]

