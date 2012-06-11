/*
 * can_close - advcan CAN driver module
 * file close.c
 */
#include <linux/pci.h>
//WangMao add the two lines below to include kernel timeout function
#include <linux/wait.h>
#include <linux/sched.h>

#include "defs.h"

extern int Can_isopen[];   		/* device minor already opened */

#ifndef DOXYGEN_SHOULD_SKIP_THIS
#endif /* DOXYGEN_SHOULD_SKIP_THIS */

/***************************************************************************/

__LDDK_CLOSE_TYPE can_close ( __LDDK_CLOSE_PARAM )
{
	unsigned int minor = iminor(inode);
	/* WangMao add the lines below @07/02/27
	   it ensures all data transmmited could be received,it is a little confusing */
	int status = CANin(minor,canstat);
	wait_queue_head_t wait;
	init_waitqueue_head(&wait);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
	//while((TxFifo->head!=TxFifo->tail||TxFifo->free[TxFifo->tail] != BUF_EMPTY)
	interruptible_sleep_on_timeout(&wait,HZ);
#else
	//while(TxFifo->head!=TxFifo->tail||TxFifo->free[TxFifo->tail] != BUF_EMPTY)
	wait_event_interruptible_timeout(wait,0,HZ);
	/* 1. if the first two condition are not TRUE: head = tail = BUF_EMPTY,it represents
	   that TxFifo is empty,so break 
	   2. the first two confition could never be TRUE(if there are no connector between 2
	   ports),so the third condition time out will take action,after 1s,break */
	/* schedule() is much better than while or for loop,it also ensures the last frame 
	   transmmited to RxFifo */
	//{	
		//printk("in first while\n");
		//schedule();
	//}
#endif
	/* what the following while does? it ensures there are no data in RxFifo before release
	   io ports and irq */
	while((status&CAN_RECEIVE_BUFFER_STATUS_BIT)!=0)
	{
		status = CANin(minor,canstat);
                schedule();
	}
     
	//WangMao add end
	DBGin("can_close");  
	CAN_StopChip(minor);
	Can_FreeIrq(minor, IRQ[minor]);

	if (IOModel[minor] == 'm'){
   		/* release I/O memory mapping -> release virtual memory */
   		iounmap((void*)can_base[minor]);
   		/* Release the memory region */
   		release_mem_region(Base[minor], can_range[minor]);
	}

#ifdef CAN_USE_FILTER
   Can_FilterCleanup(minor);
#endif

   /* printk("CAN module %d has been closed\n",minor); */

   if(Can_isopen[minor] > 0) 
   {
      --Can_isopen[minor];
		/* MOD_DEC_USE_COUNT; */
      DBGout();
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
      MOD_DEC_USE_COUNT;
#endif
		return 0;
   }

   DBGout();
   return -EBADF;
}
