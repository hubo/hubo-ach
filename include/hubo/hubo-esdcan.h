/* -*-	indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
#ifndef HUBO_ESDCAN_H
#define HUBO_ESDCAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ntcan.h>

typedef NTCAN_HANDLE hubo_can_t ;

extern hubo_can_t hubo_socket[4];

int sendCan(hubo_can_t, struct can_frame *f);
void openAllCAN(int vCan);
int readCan(hubo_can_t skt, struct can_frame *f, double timeoD);

#ifdef __cplusplus
}
#endif

#endif //HUBO_ESDCAN_H
