#ifndef JOINT_PARAM_PARSE_H
#define JOINT_PARAM_PARSE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
// sets all the joint parameters for Hubo in a 
// hubo_param struct which is define  hubo.h. See ./include/hubo.h
int setJointParams(hubo_param_t *H_param, struct hubo_state *H_state);

int setSensorDefaults(hubo_param_t* H);
// sets all the hubo_ref.ref[HUBO_JOINT_COUNT]
// values to zero. See ./include/hubo.h
void setPosZeros();

// sets all the hubo_board_cmd struct member values to zero.
// See ./include/hubo.h 
void setConsoleFlags();

// Loads the homing paramaters from a file and commands the hubo-daemon
// to apply them.
int loadHomingParams( const char *file_name );

// Saves the current homing parameters to a file.
// type=0 is for raw encoder units and type=1 is for radians
int saveHomingParams( const char *file_name, int type );
int printHomingParams( FILE* ptr_file, int type );

#ifdef __cplusplus
}
#endif

#endif //ifndef JOINT_PARAM_PARSE_H
