#ifndef JOINT_PARAM_PARSE_H
#define JOINT_PARAM_PARSE_H

// sets all the joint parameters for Hubo in a 
// hubo_param struct which is define  hubo.h. See ./include/hubo.h
int setJointParams(struct hubo_param *H_param, struct hubo_state *H_state);

int setSensorDefaults(struct hubo_param* H);
// sets all the hubo_ref.ref[HUBO_JOINT_COUNT]
// values to zero. See ./include/hubo.h
void setPosZeros();

// sets all the hubo_board_cmd struct member values to zero.
// See ./include/hubo.h 
void setConsoleFlags();

#endif //ifndef JOINT_PARAM_PARSE_H
