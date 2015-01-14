/* -*-	indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
/*
Copyright (c) 2015, Daniel M. Lofaro <dan@danlofaro.com>
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the author nor the names of its contributors may
      be used to endorse or promote products derived from this software
      without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifdef __cplusplus
extern "C" {
#endif


// for Hubo
#include "hubo.h"


/* Jaemi Hubo Joint Mapping */
#define jRHY  39
#define jRHR  40
#define jRHP  41
#define jRKN  42
#define jRAP  43
#define jRAR  44
#define jLHY  45
#define jLHR  46
#define jLHP  47
#define jLKN  48
#define jLAP  49
#define jLAR  50
#define jRSP  20
#define jRSR  21
#define jRSY  22
#define jREB  23
#define jLSP  24
#define jLSR  25
#define jLSY  26
#define jLEB  27
#define jRWY  0
#define jRW1  1
#define jRW2  2
#define jLWY  3
#define jLW1  4
#define jLW2  5
#define jNKY  6
#define jNK1  7
#define jNK2  8
#define jWST  38
#define jRF1  9
#define jRF2  10
#define jRF3  11
#define jRF4  12
#define jRF5  13
#define jLF1  14
#define jLF2  15
#define jLF3  16
#define jLF4  17
#define jLF5  18

#define JAEMI_HUBO_JOINT_COUNT 51
#define JAMEI_HUBO_JMC_COUNT 0x50

typedef struct hubo_jaemi_jmc_param {
  int joint[5];  // joints in the jmc
  int jointNum;  // number of joints in jmc
}__attribute__((packed)) hubo_jaemi_jmc_param_t;


typedef struct hubo_jaemi_param {
  int joint[JAEMI_HUBO_JOINT_COUNT];     ///< jaemi joint to hubo2+ joint
  hubo_jaemi_jmc_param_t jmc[JAEMI_HUBO_JOINT_COUNT];
}__attribute__((packed)) hubo_jaemi_param_t;

#ifdef __cplusplus
}
#endif

