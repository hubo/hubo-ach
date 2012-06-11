/* can_select
 *
 * advcan -- LINUX CAN device driver source
 *

 *
 */
#include "defs.h"

__LDDK_SELECT_TYPE can_select( __LDDK_SELECT_PARAM )
{
   unsigned int minor = __LDDK_MINOR;
   msg_fifo_t *RxFifo = &Rx_Buf[minor];
   msg_fifo_t *TxFifo = &Tx_Buf[minor];
   unsigned int mask = 0;
#ifdef DEBUG
   /* CAN_ShowStat(minor); */
#endif
   /* every event queue that could wake up the process
    * and change the status of the poll operation
    * can be added to the poll_table structure by
    * calling the function poll_wait:  
    */
   /*     _select para, wait queue, _select para */
   poll_wait(file, &CanWait[minor] , wait);
   poll_wait(file, &CanOutWait[minor] , wait);

    /* DBGprint(DBG_BRANCH,("POLL: wait returned \n")); */
   if( RxFifo->head != RxFifo->tail ) 
   {
     /* fifo has some telegrams */
	  /* Return a bit mask
	   * describing operations that could be immediately performed
	   * without blocking.

	   * POLLIN This bit must be set
	   * if the device can be read without blocking. 
	   * POLLRDNORM This bit must be set
	   * if "normal" data is available for reading.
	   * A readable device returns (POLLIN | POLLRDNORM)
	 */
      mask |= POLLIN | POLLRDNORM;	/* readable */
   }
   //WangaMao mask the line off and add a new one @07/01/05
   //if( TxFifo->head == TxFifo->tail ) 
   if( TxFifo->free[TxFifo->head] == BUF_EMPTY ) 
   {
      mask |= POLLOUT | POLLWRNORM;	/* writeable */
   }
   /* DBGout(); */
   return mask;
}
