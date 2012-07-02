#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <string.h>
#include <stdio.h>

/* At time of writing, these constants are not defined in the headers */
#ifndef PF_CAN
#define PF_CAN 29
#endif

#ifndef AF_CAN
#define AF_CAN PF_CAN
#endif

/* ... */

/* Somewhere in your app */

int main(int argc, char **argv) {
   /* Create the socket */
   int skt = socket( PF_CAN, SOCK_RAW, CAN_RAW );
   //int skt = socket( PF_CAN, SOCK_RAW, 0 );

   /* Locate the interface you wish to use */
   struct ifreq ifr;
   //strcpy(ifr.ifr_name, "vcan0");
   strcpy(ifr.ifr_name, "can0");
   ioctl(skt, SIOCGIFINDEX, &ifr); /* ifr.ifr_ifindex gets filled
                                  * with that device's index */

   /* Select that CAN interface, and bind the socket to it. */
   struct sockaddr_can addr;
   addr.can_family = AF_CAN;
   addr.can_ifindex = ifr.ifr_ifindex;
   bind( skt, (struct sockaddr*)&addr, sizeof(addr) );

//	printf("CAN_CALC_BITTIMING = %i",CAN_CALC_BITTIMING);

   /* Send a message to the CAN bus */
   struct can_frame frame;

   int counter = 0;


   sprintf( frame.data, "hello" );

   frame.can_dlc = strlen( frame.data );

   while(1) {
       frame.can_id = counter++;
       int bytes_sent = write( skt, &frame, sizeof(frame) );
       if( bytes_sent < 0 ) {
           perror("bad write");
       } else {
           printf("%d bytes sent\n", bytes_sent);
       }

       sleep(1);

/*
	printf("Press enter to continue\n");
	char enter = 0;
	while (enter != '\r' && enter != '\n') { enter = getchar(); }
	printf("Thank you for pressing enter\n");
*/

   }
}
