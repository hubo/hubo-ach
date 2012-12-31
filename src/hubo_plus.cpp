#include "hubo_plus.h"

hubo_plus::hubo_plus()
{
    memset( &H_Ref,   0, sizeof(H_Ref)   );
    memset( &H_Cmd,   0, sizeof(H_Cmd)   );
    memset( &H_State, 0, sizeof(H_State) );
    memset( &H_Ctrl,  0, sizeof(H_Ctrl)  );
    memset( &H_Param, 0, sizeof(H_Param) );

    setJointParams( &H_Param, &H_State );

    int r = ach_open( &chan_hubo_ref, HUBO_CHAN_REF_NAME, NULL );
    assert( ACH_OK == r );

    r = ach_open( &chan_hubo_state, HUBO_CHAN_STATE_NAME, NULL );
    assert( ACH_OK == r );

    r = ach_open( &chan_hubo_ctrl, HUBO_CHAN_CTRL_NAME, NULL );
    assert( ACH_OK == r );

    r = ach_open( &chan_hubo_board_cmd, HUBO_CHAN_BOARD_CMD_NAME, NULL );
    assert( ACH_OK == r );

    size_t fs;

    ach_get( &chan_hubo_ref, &H_Ref, sizeof(H_Ref), &fs, NULL, ACH_O_LAST );
    ach_get( &chan_hubo_state, &H_State, sizeof(H_State), &fs, NULL, ACH_O_LAST );
    ach_get( &chan_hubo_ctrl, &H_Ctrl, sizeof(H_Ctrl), &fs, NULL, ACH_O_LAST );

    time = 0;
}

hubo_plus::hubo_plus(const char *daemon_name)
{
    hubo_plus();

    daemonize(daemon_name);
}




