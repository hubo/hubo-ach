/*
 * can_read - advcan CAN driver module
 *
 * advcan -- LINUX CAN device driver source
 *
 */

/* project headers */
#include "defs.h"


__LDDK_READ_TYPE can_read( __LDDK_READ_PARAM )
{
   size_t written = 0;
   DBGin("can_read");
   {
      unsigned int minor = __LDDK_MINOR;
      msg_fifo_t *RxFifo = &Rx_Buf[minor];
      canmsg_t *addr; 
      int blocking;
      int ret;
      int datacount = 0;

      addr = (canmsg_t *)buffer;
      blocking = !(file->f_flags & O_NONBLOCK);
      if ( RxFifo->status == BUF_OVERRUN )
      {
         return -EINVAL;
      }
      if ( !access_ok( VERIFY_WRITE, buffer, count * sizeof(canmsg_t) )) 
      {
	     DBGout();
	     return -EINVAL;
      }
	   /* while( written < count && RxFifo->status == BUF_OK )  */
      while( written < count) 
      {
         /* Look if there are currently messages in the rx queue */
         if ( RxFifo->tail == RxFifo->head )
         {  
            //printk("buf is empty\n"); 
	    RxFifo->status = BUF_EMPTY;
            if (blocking) 
            {
               //if (RxFifo->tail == RxFifo->head)
               //   printk("blocking %d ,%d,%d",minor,RxFifo->tail,RxFifo->head);
               /* printk("empty and blocking, %d = %d\n", */
			      /* RxFifo->tail , RxFifo->head ); */
               if (wait_event_interruptible(CanWait[minor], \
			         RxFifo->tail != RxFifo->head ))
               {
                //printk("empty and blocking, %d = %d\n", 
			       //RxFifo->tail , RxFifo->head ); 
			         return -ERESTARTSYS;
               }
            } 
            else 
               break;
         }	
	 datacount =RxFifo->head-RxFifo->tail;
	if (datacount<0)
	{
		datacount+= MAX_BUFSIZE;
	}
	if(datacount > count )
	{
		datacount =count;
	}
         /* copy one message to the user process */
         ret = __lddk_copy_to_user( (canmsg_t *) &(addr[written]), 
                  (canmsg_t *) &(RxFifo->data[RxFifo->tail]),
                  sizeof(canmsg_t)*datacount);
        //printk("In read,Rxfifo tail:%d data: %d\n",RxFifo->tail,*(int*)RxFifo->data[RxFifo->tail].data);
	     //written++;
             //RxFifo->free[RxFifo->tail] = BUF_EMPTY;
	    written+=datacount;     
	    RxFifo->tail = (datacount+RxFifo->tail) % MAX_BUFSIZE;
      }
   } 
   DBGout();
   return(written);
}
