/* -*-	indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
#ifndef HUBO_SOCKETCAN_H
#define HUBO_SOCKETCAN_H

#ifdef __cplusplus
extern "C" {
#endif

typedef int hubo_can_t ;

extern hubo_can_t hubo_socket[4];

int sendCan(hubo_can_t, struct can_frame *f);
void openAllCAN(int vCan);
int readCan(hubo_can_t skt, struct can_frame *f, double timeoD);

#ifdef __cplusplus
}
#endif

#endif //HUBO_SOCKETCAN_H
