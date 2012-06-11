/* can_sysctl
 *
 * advcan -- LINUX CAN device driver source
 *
 */
#include "defs.h"
#include <linux/mm.h>
#include <linux/sysctl.h>
#include <linux/ctype.h>


#define SYSCTL_Can 1

/* ----- Prototypes */

/* ----- global variables accessible through /proc/sys/Can */

char version[] = VERSION;
char IOModel[MAX_CHANNELS+1] = { 0 };
char Chipset[] ="SJA1000";

int IRQ[MAX_CHANNELS]              = { 0x0 };
/* dont assume a standard address, always configure,
 * address                         = = 0 means no board available */
unsigned long Base[MAX_CHANNELS]    = { 0x0 };
//WangMao mask the line off and add a new one to support 64bit arch @V2.6
//int Baud[MAX_CHANNELS]             = { 0x0 };
unsigned long Baud[MAX_CHANNELS]     = { 0x0 };
unsigned int AccCode[MAX_CHANNELS] = { 0x0 };
unsigned int AccMask[MAX_CHANNELS] = { 0x0 };
int Timeout[MAX_CHANNELS] 	   = { 0x0 };
/* predefined value of the output control register,
* depends of TARGET set by Makefile */
int Outc[MAX_CHANNELS]	  = { 0x0 };
int TxErr[MAX_CHANNELS]   = { 0x0 };
int RxErr[MAX_CHANNELS]   = { 0x0 };
int Overrun[MAX_CHANNELS] = { 0x0 };

#ifdef DEBUG_COUNTER
int Cnt1[MAX_CHANNELS]    = { 0x0 };
int Cnt2[MAX_CHANNELS]    = { 0x0 };
#endif /* DEBUG_COUNTER */

/* ----- the sysctl table */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 24)
ctl_table Can_sysctl_table[]={
	{
		.ctl_name	= SYSCTL_VERSION,
		.procname	= "version",
		.data		= &version,
		.maxlen		= PROC_VER_LENGTH,
		.mode		= 0444,
		.proc_handler	= proc_dostring,
		.strategy	= sysctl_string,
	},
	{
		.ctl_name	= SYSCTL_CHIPSET,
		.procname	= "Chipset",
		.data		= &Chipset,
		.maxlen		= PROC_CHIPSET_LENGTH,
		.mode		= 0444,
		.proc_handler	= proc_dostring,
		.strategy	= sysctl_string,
	},
	{
		.ctl_name	= SYSCTL_IOMODEL,
		.procname	= "IOModel",
		.data		= &IOModel,
		.maxlen		= MAX_CHANNELS + 1,
		.mode		= 0444,
		.proc_handler	= proc_dostring,
		.strategy	= sysctl_string,
	},
	{
		.ctl_name	= SYSCTL_IRQ,
		.procname	= "IRQ",
		.data		= (void *)IRQ,
		.maxlen		= MAX_CHANNELS*sizeof(int),
		.mode		= 0644,
		.proc_handler	= proc_dointvec,
	},
	{
		.ctl_name	= SYSCTL_BASE,
		.procname	= "Base",
		.data		= (void *)Base,
		.maxlen		= MAX_CHANNELS*sizeof(int),
		.mode		= 0644,
		.proc_handler	= proc_dointvec,
	},
	{
		.ctl_name	= SYSCTL_BAUD,
		.procname	= "Baud",
		.data		= (void *)Baud,
		.maxlen		= MAX_CHANNELS*sizeof(int),
		.mode		= 0666,
		.proc_handler	= proc_dointvec,
	},
	{
		.ctl_name	= SYSCTL_ACCCODE,
		.procname	= "AccCode",
		.data		= (void *)AccCode,
		.maxlen		= MAX_CHANNELS*sizeof(unsigned int),
		.mode		= 0644,
		.proc_handler	= proc_dointvec,
	},
	{
		.ctl_name	= SYSCTL_ACCMASK,
		.procname	= "AccMask",
		.data		= (void *)AccMask,
		.maxlen		= MAX_CHANNELS*sizeof(unsigned int),
		.mode		= 0644,
		.proc_handler	= proc_dointvec,
	},
 	{
		.ctl_name	= SYSCTL_TIMEOUT,
		.procname	= "Timeout",
		.data		= (void *)Timeout,
		.maxlen		= MAX_CHANNELS*sizeof(int),
		.mode		= 0644,
		.proc_handler	= proc_dointvec,
	}, 
 	{
		.ctl_name	= SYSCTL_OUTC,
		.procname	= "Outc",
		.data		= (void *)Outc,
		.maxlen		= MAX_CHANNELS*sizeof(int),
		.mode		= 0644,
		.proc_handler	= proc_dointvec,
	},	
 	{
		.ctl_name	= SYSCTL_TXERR,
		.procname	= "TxErr",
		.data		= (void *)TxErr,
		.maxlen		= MAX_CHANNELS*sizeof(int),
		.mode		= 0444,
		.proc_handler	= proc_dointvec,
	},	
 	{
		.ctl_name	= SYSCTL_RXERR,
		.procname	= "RxErr",
		.data		= (void *)RxErr,
		.maxlen		= MAX_CHANNELS*sizeof(int),
		.mode		= 0444,
		.proc_handler	= proc_dointvec,
	}, 
	{
		.ctl_name	= SYSCTL_OVERRUN,
		.procname	= "Overrun",
		.data		= (void *)Overrun,
		.maxlen		= MAX_CHANNELS*sizeof(int),
		.mode		= 0444,
		.proc_handler	= proc_dointvec,
	},
	{
		.ctl_name	= SYSCTL_DBGMASK,
		.procname	= "dbgMask",
		.data		= (void *)&dbgMask,
		.maxlen		= 1*sizeof(int),
		.mode		= 0644,
		.proc_handler	= proc_dointvec,
	},
 
#ifdef DEBUG_COUNTER
	{
		.ctl_name	= SYSCTL_CNT1,
		.procname	= "cnt1",
		.data		= (void *)Cnt1,
		.maxlen		= MAX_CHANNELS*sizeof(int),
		.mode		= 0444,
		.proc_handler	= proc_dointvec,
	},
	{
		.ctl_name	= SYSCTL_CNT2,
		.procname	= "cnt2",
		.data		= (void *)Cnt2,
		.maxlen		= MAX_CHANNELS*sizeof(int),
		.mode		= 0444,
		.proc_handler	= proc_dointvec,
	},
#endif 	
	{ .ctl_name = 0 },
};
ctl_table Can_sys_table[] = {
	{
		.ctl_name	= SYSCTL_Can,
		.procname	= "Can",
		.mode		= 0555,
		.child		= Can_sysctl_table,
	},
	{ .ctl_name = 0 },
};
#else //(>=2.6.25)
ctl_table Can_sysctl_table[]={
	{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 32)
		.ctl_name	= CTL_UNNUMBERED,
#endif
		.procname	= "version",
		.data		= &version,
		.maxlen		= PROC_VER_LENGTH,
		.mode		= 0444,
		.proc_handler	= proc_dostring,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 32)
		.strategy	= sysctl_string,
#endif
	},
	{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 32)
		.ctl_name	= CTL_UNNUMBERED,
#endif
		.procname	= "Chipset",
		.data		= &Chipset,
		.maxlen		= PROC_CHIPSET_LENGTH,
		.mode		= 0444,
		.proc_handler	= proc_dostring,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 32)
		.strategy	= sysctl_string,
#endif
	},
	{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 32)
		.ctl_name	= CTL_UNNUMBERED,
#endif
		.procname	= "IOModel",
		.data		= &IOModel,
		.maxlen		= MAX_CHANNELS + 1,
		.mode		= 0444,
		.proc_handler	= proc_dostring,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 32)
		.strategy	= sysctl_string,
#endif
	},
	{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 32)
		.ctl_name	= CTL_UNNUMBERED,
#endif
		.procname	= "IRQ",
		.data		= (void *)IRQ,
		.maxlen		= MAX_CHANNELS*sizeof(int),
		.mode		= 0644,
		.proc_handler	= proc_dointvec,
	},
	{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 32)
		.ctl_name	= CTL_UNNUMBERED,
#endif
		.procname	= "Base",
		.data		= (void *)Base,
		.maxlen		= MAX_CHANNELS*sizeof(int),
		.mode		= 0644,
		.proc_handler	= proc_dointvec,
	},
	{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 32)
		.ctl_name	= CTL_UNNUMBERED,
#endif
		.procname	= "Baud",
		.data		= (void *)Baud,
		.maxlen		= MAX_CHANNELS*sizeof(int),
		.mode		= 0666,
		.proc_handler	= proc_dointvec,
	},
	{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 32)
		.ctl_name	= CTL_UNNUMBERED,
#endif
		.procname	= "AccCode",
		.data		= (void *)AccCode,
		.maxlen		= MAX_CHANNELS*sizeof(unsigned int),
		.mode		= 0644,
		.proc_handler	= proc_dointvec,
	},
	{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 32)
		.ctl_name	= CTL_UNNUMBERED,
#endif
		.procname	= "AccMask",
		.data		= (void *)AccMask,
		.maxlen		= MAX_CHANNELS*sizeof(unsigned int),
		.mode		= 0644,
		.proc_handler	= proc_dointvec,
	},
 	{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 32)
		.ctl_name	= CTL_UNNUMBERED,
#endif
		.procname	= "Timeout",
		.data		= (void *)Timeout,
		.maxlen		= MAX_CHANNELS*sizeof(int),
		.mode		= 0644,
		.proc_handler	= proc_dointvec,
	}, 
 	{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 32)
		.ctl_name	= CTL_UNNUMBERED,
#endif
		.procname	= "Outc",
		.data		= (void *)Outc,
		.maxlen		= MAX_CHANNELS*sizeof(int),
		.mode		= 0644,
		.proc_handler	= proc_dointvec,
	},	
 	{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 32)
		.ctl_name	= CTL_UNNUMBERED,
#endif
		.procname	= "TxErr",
		.data		= (void *)TxErr,
		.maxlen		= MAX_CHANNELS*sizeof(int),
		.mode		= 0444,
		.proc_handler	= proc_dointvec,
	},	
 	{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 32)
		.ctl_name	= CTL_UNNUMBERED,
#endif
		.procname	= "RxErr",
		.data		= (void *)RxErr,
		.maxlen		= MAX_CHANNELS*sizeof(int),
		.mode		= 0444,
		.proc_handler	= proc_dointvec,
	}, 
	{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 32)
		.ctl_name	= CTL_UNNUMBERED,
#endif
		.procname	= "Overrun",
		.data		= (void *)Overrun,
		.maxlen		= MAX_CHANNELS*sizeof(int),
		.mode		= 0444,
		.proc_handler	= proc_dointvec,
	},
	{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 32)
		.ctl_name	= CTL_UNNUMBERED,
#endif
		.procname	= "dbgMask",
		.data		= (void *)&dbgMask,
		.maxlen		= 1*sizeof(int),
		.mode		= 0644,
		.proc_handler	= proc_dointvec,
	},
 
#ifdef DEBUG_COUNTER
	{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 32)
		.ctl_name	= CTL_UNNUMBERED,
#endif
		.procname	= "cnt1",
		.data		= (void *)Cnt1,
		.maxlen		= MAX_CHANNELS*sizeof(int),
		.mode		= 0444,
		.proc_handler	= proc_dointvec,
	},
	{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 32)
		.ctl_name	= CTL_UNNUMBERED,
#endif
		.procname	= "cnt2",
		.data		= (void *)Cnt2,
		.maxlen		= MAX_CHANNELS*sizeof(int),
		.mode		= 0444,
		.proc_handler	= proc_dointvec,
	},
#endif 	
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 32)
	{ .ctl_name = 0 },
#else
	{  0 },
#endif
};
ctl_table Can_sys_table[] = {
	{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 32)
		.ctl_name	= CTL_UNNUMBERED,
#endif
		.procname	= "Can",
		.mode		= 0555,
		.child		= Can_sysctl_table,
	},
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 32)
	{ .ctl_name = 0 },
#else
	{  0 },
#endif
};
#endif

/* ----- register and unregister entrys */

struct ctl_table_header *Can_systable = NULL;

void register_systables(void)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 21)
   Can_systable = register_sysctl_table( Can_sys_table, 0 );
#else
   Can_systable = register_sysctl_table( Can_sys_table );
#endif
}

void unregister_systables(void)
{
   unregister_sysctl_table(Can_systable);
}

