// sets all the joint parameters for Hubo in a 
// hubo_param struct which is define  hubo.h. See ./include/hubo.h
int setJointParams(struct hubo_param *H, struct hubo_state *H_state);

// sets all the hubo_ref.ref[HUBO_JOINT_COUNT]
// values to zero. See ./include/hubo.h
void setPosZeros();

// sets all the hubo_init_cmd struct member values to zero.
// See ./include/hubo.h 
void setConsoleFlags();
