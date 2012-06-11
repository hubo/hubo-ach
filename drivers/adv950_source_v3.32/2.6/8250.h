/*
 *  linux/drivers/char/8250.h
 *
 *  Driver for 8250/16550-type serial ports
 *
 *  Based on drivers/char/serial.c, by Linus Torvalds, Theodore Ts'o.
 *
 *  Copyright (C) 2001 Russell King.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 *  $Id: 8250.h,v 1.8 2002/07/21 21:32:30 rmk Exp $
 */

//lipeng add at 2006/06/08
#include <linux/version.h>

#ifndef KERNEL_VERSION
#define KERNEL_VERSION(a,b,c)	((a)*65536+(b)*256+(c))
#endif
//lipeng add end

//lipeng modify at 2007/09/07
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 18)
#include <linux/config.h>
#endif
//lipeng modify end

//jinxin added to support DMA
#include <linux/pci.h>
#define TX_BUF_TOT_LEN 128
#define RX_BUF_TOT_LEN 128

#ifndef MIN
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif

//DMA registers
#define DMAADL (0x100 + 0x00)		//DMA Address Low
#define DMAADH (0x100 + 0x04)		//DMA Address High
#define DMATL  (0x100 + 0x08)		//DMA Transfer Length
#define DMASTA (0x100 + 0x0c)  		//DMA Status

//DMA Status
#define DMAREAD	(1UL <<  31)
#define DMAACT	(1UL << 0)
#define DMAERR	(1UL << 1)
#define DMADONE	(1UL << 2)
//jinxin added to support dtrdsr flow control
#ifndef CDTRDSR
#define CDTRDSR	  004000000000  /* DTR/DSR flow control */
#endif

//jinxin added end
void adv_serial8250_get_irq_map(unsigned int *map);
void adv_serial8250_suspend_port(int line);
void adv_serial8250_resume_port(int line);

struct uart_8250_port {
	struct uart_port	port;
	struct timer_list	timer;		/* "no irq" timer */
	struct list_head	list;		/* ports on this IRQ */
	unsigned int		capabilities;	/* port capabilities */
	unsigned short		rev;
	unsigned char		acr;
	unsigned char		ier;
	unsigned char		lcr;
	unsigned char		mcr;
	unsigned char		mcr_mask;	/* mask of user bits */
	unsigned char		mcr_force;	/* mask of forced bits */
	unsigned char		lsr_break_flag;
	//jinxin added to support DMA
	dma_addr_t		rx_ring_dma;
	dma_addr_t 		tx_ring_dma;
	unsigned char 		*rx_ring;
	unsigned char 		*tx_ring;


	/*
	 * We provide a per-port pm hook.
	 */
	void			(*pm)(struct uart_port *port,
				      unsigned int state, unsigned int old);
};

struct old_serial_port {
	unsigned int uart;
	unsigned int baud_base;
	unsigned int port;
	unsigned int irq;
	unsigned int flags;
	unsigned char hub6;
	unsigned char io_type;
	unsigned char *iomem_base;
	unsigned short iomem_reg_shift;
};

#undef SERIAL_DEBUG_PCI

#if defined(__i386__) && (defined(CONFIG_M386) || defined(CONFIG_M486))
#define SERIAL_INLINE
#endif
  
#ifdef SERIAL_INLINE
#define _INLINE_ inline
#else
#define _INLINE_
#endif

#define PROBE_RSA	(1 << 0)
#define PROBE_ANY	(~0)

#define HIGH_BITS_OFFSET ((sizeof(long)-sizeof(int))*8)

#ifdef CONFIG_SERIAL_8250_SHARE_IRQ
#define SERIAL8250_SHARE_IRQS 1
#else
#define SERIAL8250_SHARE_IRQS 0
#endif
