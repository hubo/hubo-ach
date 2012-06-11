/*
 * advcan -- LINUX CAN device driver source
 *
 */


#include <linux/sched.h> 
#include <linux/proc_fs.h>
#include <linux/pci.h>
#include "defs.h"

/*
 * Refuse to compile under versions older than 1.99.4
 */

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,4,0)
#  error "This module needs Linux 2.4 or newer"
#endif

/* each CAN channel has one wait_queue for read and one for write */
wait_queue_head_t CanWait[MAX_CHANNELS];
wait_queue_head_t CanOutWait[MAX_CHANNELS];

/* for each CAN channel allocate a TX and RX FIFO */
msg_fifo_t   Tx_Buf[MAX_CHANNELS] = {{0}};
msg_fifo_t   Rx_Buf[MAX_CHANNELS] = {{0}};

#ifdef CAN_USE_FILTER
    msg_filter_t Rx_Filter[MAX_CHANNELS] = {{0}}; 
#endif
/* used to store always the last frame sent on this channel */
canmsg_t     last_Tx_object[MAX_CHANNELS];

void __iomem *can_base[MAX_CHANNELS] = {0};	/* ioremapped adresses */
unsigned int can_range[MAX_CHANNELS] = {0};	/* ioremapped adresses */

int selfreception[MAX_CHANNELS] = {0};	/* flag indicating that selfreception of frames is allowed */
int timestamp[MAX_CHANNELS] = {1};	/* flag indicating that timestamp value should assigned to rx messages */
int wakeup[MAX_CHANNELS] = {1};		/* flag indicating that leeping processes are waken up in case of events */




int Can_WaitInit(int minor)
{
   DBGin("Can_WaitInit");
	/* reset the wait_queue pointer */
	init_waitqueue_head(&CanWait[minor]);
	init_waitqueue_head(&CanOutWait[minor]);
   DBGout();
   return 0;
}

/*
initialize RX and TX Fifo's
*/
int Can_FifoInit(int minor)
{
   int i;

   DBGin("Can_FifoInit");
   Tx_Buf[minor].head   = Rx_Buf[minor].head = 0;
   Tx_Buf[minor].tail   = Rx_Buf[minor].tail = 0;
   Tx_Buf[minor].status = Rx_Buf[minor].status = 0;
   Tx_Buf[minor].active = Rx_Buf[minor].active = 0;
   for(i = 0; i < MAX_BUFSIZE; i++) 
   {
	   Tx_Buf[minor].free[i]  = BUF_EMPTY;
   }
   DBGout();
   return 0;
}

#ifdef CAN_USE_FILTER
int Can_FilterInit(int minor)
{
   int i;

   DBGin("Can_FilterInit");
   Rx_Filter[minor].use      = 0;
   Rx_Filter[minor].signo[0] = 0;
   Rx_Filter[minor].signo[1] = 0;
   Rx_Filter[minor].signo[2] = 0;

   for (i=0;i<MAX_ID_NUMBER;i++)	
	   Rx_Filter[minor].filter[i].rtr_response = NULL;

   DBGout();
   return 0;
}

int Can_FilterCleanup(int minor)
{
   int i;

   DBGin("Can_FilterCleanup");
   for(i=0;i<MAX_ID_NUMBER;i++) 
   {
	   if ( Rx_Filter[minor].filter[i].rtr_response != NULL )	
	      kfree( Rx_Filter[minor].filter[i].rtr_response);
	   Rx_Filter[minor].filter[i].rtr_response = NULL;
   }
   DBGout();
   return 0;
}


int Can_FilterOnOff(int minor, unsigned on) 
{
    DBGin("Can_FilterOnOff");
    Rx_Filter[minor].use = (on!=0);
    DBGout();
    return 0;
}

int Can_FilterMessage(int minor, unsigned message, unsigned enable) 
{
    DBGin("Can_FilterMessage");
       Rx_Filter[minor].filter[message].enable = (enable!=0);
    DBGout();
    return 0;
}

int Can_FilterTimestamp(int minor, unsigned message, unsigned stamp) 
{
    DBGin("Can_FilterTimestamp");
    Rx_Filter[minor].filter[message].timestamp = (stamp!=0);
    DBGout();
    return 0;
}

int Can_FilterSignal(int minor, unsigned id, unsigned signal) 
{
    DBGin("Can_FilterSignal");
    if( signal <= 3 )
       Rx_Filter[minor].filter[id].signal = signal;
    DBGout();
    return 0;
}

int Can_FilterSigNo(int minor, unsigned signo, unsigned signal )
{
    DBGin("Can_FilterSigNo");
    if( signal < 3 )
       Rx_Filter[minor].signo[signal] = signo;
    DBGout();
    return 0;
}
#endif

#ifdef CAN_RTR_CONFIG
int Can_ConfigRTR( int minor, unsigned message, canmsg_t *Tx )
{
   canmsg_t *tmp;

   DBGin("Can_ConfigRTR");
   if( (tmp = kmalloc ( sizeof(canmsg_t), GFP_ATOMIC )) == NULL )
   {
      DBGprint(DBG_BRANCH,("memory problem"));
	   DBGout(); return -1;
   }
   Rx_Filter[minor].filter[message].rtr_response = tmp;
   memcpy( Rx_Filter[minor].filter[message].rtr_response , Tx, sizeof(canmsg_t));	
   DBGout(); return 1;
   return 0;
}

int Can_UnConfigRTR( int minor, unsigned message )
{
   canmsg_t *tmp;

   DBGin("Can_UnConfigRTR");
   if( Rx_Filter[minor].filter[message].rtr_response != NULL ) 
   {
      kfree(Rx_Filter[minor].filter[message].rtr_response);
	   Rx_Filter[minor].filter[message].rtr_response = NULL;
   }	
   DBGout(); 
   return 1;
   return 0;
}
#endif


#ifdef DEBUG

/* dump_CAN or CAN_dump() which is better ?? */

#include <asm/io.h>

#if 1
/* simply dump a memory area bytewise for n*16 addresses */
/*
 * adress - start address 
 * n      - number of 16 byte rows, 
 * offset - print every n-th byte
 */
void dump_CAN(unsigned long adress, int n, int offset)
{
   int i, j;
   printk("     CAN at Adress 0x%lx\n", adress);
   for(i = 0; i < n; i++) 
   {
	   printk("     ");
	   for(j = 0; j < 16; j++) 
      {
         /* printk("%02x ", *ptr++); */
         printk("%02x ", readb((void __iomem *)adress));
         adress += offset;
      }
      printk("\n");
   }
}
#endif

#ifdef CPC_PCI 
# define REG_OFFSET 4
#else
# define REG_OFFSET 1
#endif
/**
*   Dump the CAN controllers register contents,
*   identified by the device minr number to stdout
*
*   Base[minor] should contain the virtual adress
*/
void can_dump(int minor)
{
   int i, j;
   int index = 0;

	for(i = 0; i < 2; i++) 
   {
	   printk("0x%04lx: ", Base[minor] + (i * 16));
	   for(j = 0; j < 16; j++) 
      {
         printk("%02x ",
#ifdef  CAN_PORT_IO
         inb((int) (Base[minor] + index)) );
#else
         readb((void __iomem *) (can_base[minor] + index)) );
#endif
         /* slow_down_io(); */
         index += REG_OFFSET;
      }
      printk("\n");
	}
}
#endif

void canout(int bd,void * adr1,void * adr2,uint8 v)	
{  
   (*Canout[bd])(adr1,adr2,v);
} 

unsigned canin(int bd,void * adr1,void * adr2)	
{
	return (*Canin[bd])(adr1,adr2);
}

unsigned cantest(int bd,void * adr1,void * adr2, unsigned m)	
{
	return (*Cantest[bd])(adr1,adr2,m);
}

void canreset(int bd,void * adr1,void * adr2,int m)
{                       
	(*Canreset[bd])(adr1,adr2,m);
}

void canset(int bd,void * adr1,void * adr2,int m)	
{                       
   (*Canset[bd])(adr1,adr2,m);
}


/////////////////////
void canout_isa(void * adr1,void * adr2,uint8 v)	
{            
	writeb(v, (void __iomem *)adr2 );
} 

unsigned canin_isa(void * adr1,void * adr2)	
{
	return (readb((void __iomem *)adr2 ));
}

unsigned cantest_isa(void * adr1,void * adr2, unsigned m)	
{
	return (readb ((void __iomem *)adr2 ) & (m) );
}

void canreset_isa(void * adr1,void * adr2,int m)
{                       
	writeb((readb((void __iomem *)adr2)) & ~(m), (void __iomem *)adr2 );
}

void canset_isa(void * adr1,void * adr2,int m)	
{                       
	writeb((readb((void __iomem *)adr2)) | (m) , (void __iomem *)adr2 ); 
}

//////////////////////
void canout_pci(void * adr1,void * adr2,uint8 v)	
{            
	outb(v, (int)adr1 );
} 

unsigned canin_pci(void * adr1,void * adr2)	
{
	return (inb ((int)adr1)) ;
}

unsigned cantest_pci(void * adr1,void * adr2, unsigned m)	
{
	return (inb((int)adr1  ) & m ); 
}

void canreset_pci(void * adr1,void * adr2,int m)
{                       
	outb((inb((int)adr1)) & ~m,(int)adr1 );
}

void canset_pci(void * adr1,void * adr2,int m)	
{                       
	outb((inb((int)adr1)) | m ,(int)adr1 );
}

void (*Canout[MAX_CHANNELS])(void * adr1,void * adr2,uint8 v);
unsigned (*Canin[ MAX_CHANNELS])(void * adr1,void * adr2);
void (*Canset[ MAX_CHANNELS])(void * adr1,void * adr2,int m);
void (*Canreset[ MAX_CHANNELS])(void * adr1,void * adr2,int m);
unsigned (*Cantest[ MAX_CHANNELS])(void * adr1,void * adr2, unsigned m);
