 /*
 * advcan -- LINUX CAN device driver source
 *
 */

#include "defs.h"
#include "sja1000.h"
#include "linux/delay.h"
/* \fn size_t can_write( __LDDK_WRITE_PARAM) */
/***************************************************************************/
/**

\brief size_t write(int fd, const char *buf, size_t count);
write CAN messages to the network
\param fd The descriptor to write to.
\param buf The data buffer to write (array of CAN canmsg_t).
\param count The number of bytes to write.

write  writes  up to \a count CAN messages to the CAN controller
referenced by the file descriptor fd from the buffer
starting at buf.



\par Errors

the following errors can occur

\li \c EBADF  fd is not a valid file descriptor or is not open
              for writing.

\li \c EINVAL fd is attached to an object which is unsuitable for
              writing.

\li \c EFAULT buf is outside your accessible address space.

\li \c EINTR  The call was interrupted by a signal before any
              data was written.



\returns
On success, the number of CAN messages written are returned
(zero indicates nothing was written).
On error, -1 is returned, and errno is set appropriately.

\internal
*/

#define R_OFF 1
/* WangMao add this funciton @07/01/05
   It copies one message to hardware buffer:TXB from TxFifo of driver
   it is so important to stimulate the transmit interrupt that 
   keeps the tramsit going on when the TXB is actually empty */
extern spinlock_t sendfifo_lock;
void sendfifo(int minor)
{	
	int ext;
	int i;
	unsigned int id;
	msg_fifo_t *TxFifo = &Tx_Buf[minor];
        
	int status = 0;
	int flag;
       uint8 tx2reg;
        
        /* spin_lock() could be broken by bottom half:tasklet,so use
	   spin_lock_bh(),it disables bottom half in lock section */
	spin_lock_bh(&sendfifo_lock);
        /*if( TxFifo->free[TxFifo->tail] == BUF_EMPTY )
		return;*/
        for ( i = 0; TxFifo->free[TxFifo->tail] == BUF_EMPTY && i < MAX_BUFSIZE;++i )
        {
		TxFifo->tail = (++TxFifo->tail ) % MAX_BUFSIZE;
	}
        if ( i >= MAX_BUFSIZE )
        {
		TxFifo->tail = TxFifo->head;
		spin_unlock_bh(&sendfifo_lock);
		return;   
	}
    	status = CANin(minor, canstat);
        flag = status & CAN_TRANSMIT_BUFFER_ACCESS;
	if(flag==0)
	{
		spin_unlock_bh(&sendfifo_lock);
		return;
	}
        if(selfreception[minor]) 
	{
		memcpy((void *)&last_Tx_object[minor],
			(void *)&TxFifo->data[TxFifo->tail],
			sizeof(canmsg_t));
	}
	
    	status = CANin(minor, canstat);
        flag = status & CAN_TRANSMIT_BUFFER_ACCESS;
	//printk("sendfifo: Tail: %d, Write to TXB: %d\n",TxFifo->tail, *(int*)TxFifo->data[TxFifo->tail].data);
	 tx2reg = (TxFifo->data[TxFifo->tail]).length;
	if( (TxFifo->data[TxFifo->tail]).flags & MSG_RTR) 
	{
		tx2reg |= CAN_RTR;
	}
	ext = (TxFifo->data[TxFifo->tail]).flags & MSG_EXT;
	id = (TxFifo->data[TxFifo->tail]).id;
	if(ext) 
	{
		//DBGprint(DBG_DATA, ("---> send ext message \n"));
		CANout(minor, frameinfo, CAN_EFF + tx2reg);
		CANout(minor, frame.extframe.canid1, (uint8)(id >> 21));
		CANout(minor, frame.extframe.canid2, (uint8)(id >> 13));
		CANout(minor, frame.extframe.canid3, (uint8)(id >> 5));
		CANout(minor, frame.extframe.canid4, (uint8)(id << 3) & 0xff);
	} 
	else 
	{
		//DBGprint(DBG_DATA, ("---> send std message \n"));
		CANout(minor, frameinfo, CAN_SFF + tx2reg);
		CANout(minor, frame.stdframe.canid1, (uint8)((id) >> 3) );
		CANout(minor, frame.stdframe.canid2, (uint8)(id << 5 ) & 0xe0);
	}

	tx2reg &= 0x0f;		//restore length only
	//printk("send data is %d\n",*(int*)(TxFifo->data[TxFifo->tail]).data);
	if(ext) 
	{
		for( i=0; i < tx2reg ; i++) 
		{
			CANout(minor, frame.extframe.canxdata[R_OFF * i],
				(TxFifo->data[TxFifo->tail]).data[i]);
		}
	} 
	else 
	{
		for( i=0; i < tx2reg ; i++) 
		{  
			CANout(minor, frame.stdframe.candata[R_OFF * i],
				(TxFifo->data[TxFifo->tail]).data[i]);
		}
	}
	CANout(minor, cancmd, CAN_TRANSMISSION_REQUEST );

	TxFifo->free[TxFifo->tail] = BUF_EMPTY; // now this entry is EMPTY
	TxFifo->tail = (++(TxFifo->tail)) % MAX_BUFSIZE;
	//printk("out of sendfifo,tail is %d\n",TxFifo->tail);
	spin_unlock_bh(&sendfifo_lock);
}

int status;
__LDDK_WRITE_TYPE can_write( __LDDK_WRITE_PARAM )
{
	unsigned int minor = __LDDK_MINOR;
	msg_fifo_t *TxFifo = &Tx_Buf[minor];
	canmsg_t *addr;
        //James Dai mask 
	int written        = 0;
	int ret;
	int blocking;
        int flag;
	DBGin("can_write");
#ifdef DEBUG_COUNTER
	Cnt1[minor] = Cnt1[minor] + 1;
#endif /* DEBUG_COUNTER */


	/* detect write mode */
	blocking = !(file->f_flags & O_NONBLOCK);
	/* DEBUG_TTY(1, "write: %d", count); */
	DBGprint(DBG_DATA,(" -- write %d msg\n", count));
	addr = (canmsg_t *)buffer;
	if(!access_ok(VERIFY_READ, (canmsg_t *)addr, count * sizeof(canmsg_t))) 
	{
		DBGout();
                return -EINVAL;
	}	
       while(written < count)
	{
		/* enter critical section */

		/* local_irq_save(flags); */

      /* Do we really need to protect something here ????
      * e.g. in this case the TxFifo->free[TxFifo->head] value
      *
      * If YES, we have to use spinlocks for synchronization
      */

      /* - new Blocking code -- */

	if(blocking) 
	{    	
		if(wait_event_interruptible(CanOutWait[minor], \
		    		TxFifo->free[TxFifo->head] != BUF_FULL))
				/* TxFifo->tail != TxFifo->head )) */
		{
			return -ERESTARTSYS;
		}
	} 
	else 
	{
	    	/* there are data to write to the network */
		if(TxFifo->free[TxFifo->head] == BUF_FULL) 
		{
			   /* there is already one message at this place */;
			   /* local_irq_restore(flags); */
			DBGout();
   			return written;
		}
	}
	//WangMao add the lines below
	//first,copy data from user mode to driver fifo:TxFifo
	ret = __lddk_copy_from_user((canmsg_t *) &(TxFifo->data[TxFifo->head]),
		(canmsg_t *) &addr[written],  sizeof(canmsg_t) );
	if ( ret < 0)
	{
		return -EINVAL;
	}
	//local_irq_save(flags);
	TxFifo->free[TxFifo->head] = BUF_FULL; /* now this entry is FULL */
	//printk("Head: %d, Tail: %d, Write to TxFifo: %d\n",TxFifo->head,TxFifo->tail, *(int*)TxFifo->data[TxFifo->head].data);
	TxFifo->head = ++(TxFifo->head) % MAX_BUFSIZE;
	written++;
	//local_irq_restore(flags);
	
	/* copy one data from TxFifo to TXB(hardware fifo) */
	status = CANin(minor,canstat);
	flag = status & CAN_TRANSMIT_BUFFER_ACCESS;
	if( flag == 0)
		/* the TXB is locked,it means there are data exist in TXB,
		   so no worry about transmmit interrupt lost */
		continue;
	/* TXB is empty,so copy data from TxFifo to TXB,here may be interrupted by transmmit
	   interrupt handler and tasklet will be called(it does the same thing),so the TXB 
	   may become locked again,therefor check to see if TXB is locked is necessary in 
	   sendfifo*/
	sendfifo( minor);  /* Send, no wait */
	//WangMao add end
   } //end of while
   DBGout();
   return written;
}

