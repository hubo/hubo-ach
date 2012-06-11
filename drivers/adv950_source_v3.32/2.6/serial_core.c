//******************************************************************************
//
// Copyright (c) 2008 Advantech Industrial Automation Group.
//
// Oxford PCI-954/952/16C950 with Advantech RS232/422/485 capacities
// 
// This program is free software; you can redistribute it and/or modify it 
// under the terms of the GNU General Public License as published by the Free 
// Software Foundation; either version 2 of the License, or (at your option) 
// any later version.
// 
// This program is distributed in the hope that it will be useful, but WITHOUT 
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for 
// more details.
// 
// You should have received a copy of the GNU General Public License along with
// this program; if not, write to the Free Software Foundation, Inc., 59 
// Temple Place - Suite 330, Boston, MA  02111-1307, USA.
// 
//
//
//******************************************************************************

//***********************************************************************
// File:        serial_core.c
// 
// PLEASE SEE 8250.c FOR DETAIL
//***********************************************************************
//lipeng modify at 2007/09/07
#include <linux/version.h> // Po-Cheng Chen add, 02/15/2006

#ifndef KERNEL_VERSION
#define KERNEL_VERSION(a,b,c)	((a)*65536+(b)*256+(c))
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 18)
#include <linux/config.h>
#endif
//lipeng modify end
#include <linux/module.h>
#include <linux/tty.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/console.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 31)
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#endif
#include <linux/serial_core.h>
#include <linux/smp_lock.h>
#include <linux/device.h>
#include <linux/serial.h> /* for serial_state and serial_icounter_struct */

#include <asm/irq.h>
#include <asm/uaccess.h>
#include "8250.h"
#include <linux/serial_reg.h>
#include <asm/io.h>


#undef	DEBUG
#ifdef DEBUG
#define DPRINTK(x...)	printk(x)
#else
#define DPRINTK(x...)	do { } while (0)
#endif


/*
 * This is used to lock changes in serial line configuration.
 */

//lipeng modify at 06/08/2006
//James.dai add UBUNTU_2_6_15 @V3.20 to support ubuntu6
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 15) || UBUNTU_2_6_15
	static DECLARE_MUTEX(port_sem);
#else
	static DEFINE_MUTEX(port_mutex);
#endif
//lipeng modify end
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
static struct lock_class_key port_lock_key;
#endif

#define HIGH_BITS_OFFSET	((sizeof(long)-sizeof(int))*8)

#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 28)
#define uart_users(state)	((state)->count + (state)->info.port.blocked_open)
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)
#define uart_users(state)	((state)->count + ((state)->info ? (state)->info->port.blocked_open : 0))
#else
#define uart_users(state)	((state)->count + ((state)->info ? (state)->info->blocked_open : 0))
#endif

#ifdef CONFIG_SERIAL_CORE_CONSOLE
#define uart_console(port)	((port)->cons && (port)->cons->index == (port)->line)
//lipeng modify at 2007/09/07
//James.dai add @ V3.20 to support RedHat Enterprise Server 4.3 
//satisify RedHat Enterprise Server 4.3 smp
//WangMao add RHE4SMP to satisify RedHat Enterprise Server 4.3 none-smp

#ifdef RHE4SMP
#if RHE4SMP
//#ifdef SUPPORT_SYSRQ
int sercons_escape_char = -1;
//#endif
#else
int sercons_escape_char = -1;
#endif
#endif
int sercons_escape_char = -1;

//James.dai add end
//lipeng modify end
#else
#define uart_console(port)	(0)
#endif


extern struct uart_8250_port serial8250_ports[64];
#define serial_outp(up, offset, value)	serial_out(up, offset, value)////
static _INLINE_ void serial_out(struct uart_8250_port *up, int offset, int value)
{
	offset <<= up->port.regshift;
	writeb(value, up->port.membase + offset);
}
static _INLINE_ unsigned int serial_in(struct uart_8250_port *up, int offset)
{
	offset <<= up->port.regshift;
	return readb(up->port.membase + offset);
}

/* WangMao add this ioctl code and it's handler @V3.20 to 
   test XON/XOFF flow control,this would be useful when 
   debug driver */
 #define SERIALMAGIC   'd'  // use 'd' as magic number
#define SENDXON _IOW(SERIALMAGIC,0,int)
#define SENDXOFF _IOW(SERIALMAGIC,1,int)
static void uart_send_xchar(struct tty_struct *tty, char ch);
void sendxon(struct tty_struct *tty)
{
	if (I_IXOFF(tty))
	{
		printk("sendxon: XON is %x\n",START_CHAR(tty));
		uart_send_xchar(tty, START_CHAR(tty));	
	}
}
void sendxoff(struct tty_struct *tty)
{
	if(I_IXOFF(tty))
	{
		printk("sendxoff: XOFF is %x\n",STOP_CHAR(tty));
		uart_send_xchar(tty, STOP_CHAR(tty));
	}
}
//WangMao add end
/*WangMao add the line below to add quick response to v3_20
#define ENABLE_QUICK_RESPONSE _IOW(SERIALMAGIC,2,int) */
int quick_response = 0;
extern struct serial_uart_config uart_config[PORT_MAX_8250+2];
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 23)
static void uart_change_speed(struct uart_state *state, struct termios *old_termios);
#else
static void uart_change_speed(struct uart_state *state, struct ktermios *old_termios);
#endif
static void uart_wait_until_sent(struct tty_struct *tty, int timeout);
static void uart_change_pm(struct uart_state *state, int pm_state);
/*
 * This routine is used by the interrupt handler to schedule processing in
 * the software interrupt portion of the driver.
 */
void adv_uart_write_wakeup(struct uart_port *port)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	struct uart_state *state = port->state;
	tasklet_schedule(&state->tlet);
#else
	struct uart_info *info = port->info;
	tasklet_schedule(&info->tlet);
#endif
}

static void uart_stop(struct tty_struct *tty)
{
	struct uart_state *state = tty->driver_data;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	struct uart_port *port = state->uart_port;
#else
	struct uart_port *port = state->port;
#endif
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);
//lipeng modify at 06/08/2006
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 15)
	port->ops->stop_tx(port, 1);
#else
	port->ops->stop_tx(port);
#endif
//lipeng modify end
	spin_unlock_irqrestore(&port->lock, flags);
}

static void __uart_start(struct tty_struct *tty)
{
	struct uart_state *state = tty->driver_data;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	struct uart_port *port = state->uart_port;
#else
	struct uart_port *port = state->port;
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	if (!uart_circ_empty(&state->xmit) && state->xmit.buf &&
	    !tty->stopped && !tty->hw_stopped)
#elif LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 28)
	if (!uart_circ_empty(&state->info.xmit) && state->info.xmit.buf &&
	    !tty->stopped && !tty->hw_stopped)
#else
	if (!uart_circ_empty(&state->info->xmit) && state->info->xmit.buf &&
	    !tty->stopped && !tty->hw_stopped)
#endif
//lipeng modify at 06/08/2006
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 15)
	port->ops->start_tx(port, 1);
#else
	port->ops->start_tx(port);
#endif
//lipeng modify end
}

static void uart_start(struct tty_struct *tty)
{
	struct uart_state *state = tty->driver_data;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	struct uart_port *port = state->uart_port;
#else
	struct uart_port *port = state->port;
#endif
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);
	__uart_start(tty);
	spin_unlock_irqrestore(&port->lock, flags);
}

static void uart_tasklet_action(unsigned long data)
{
	struct uart_state *state = (struct uart_state *)data;
#if LINUX_VERSION_CODE <  KERNEL_VERSION(2, 6, 28)
	struct tty_struct *tty;
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	tty_wakeup(state->port.tty);
#elif LINUX_VERSION_CODE >  KERNEL_VERSION(2, 6, 28)
	tty_wakeup(state->info.port.tty);
#elif LINUX_VERSION_CODE ==  KERNEL_VERSION(2, 6, 28)
	tty_wakeup(state->info->port.tty);	
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)
	tty = state->info->port.tty;
	if (tty) {
		if ((tty->flags & (1 << TTY_DO_WRITE_WAKEUP)) &&
		    tty->ldisc.ops->write_wakeup)
			tty->ldisc.ops->write_wakeup(tty);
		wake_up_interruptible(&tty->write_wait);
	}
#else
	tty = state->info->tty;
	if (tty) {
		if ((tty->flags & (1 << TTY_DO_WRITE_WAKEUP)) &&
		    tty->ldisc.write_wakeup)
			tty->ldisc.write_wakeup(tty);
		wake_up_interruptible(&tty->write_wait);
	}
#endif	
}

static inline void
uart_update_mctrl(struct uart_port *port, unsigned int set, unsigned int clear)
{
	unsigned long flags;
	unsigned int old;

	spin_lock_irqsave(&port->lock, flags);
	old = port->mctrl;
	port->mctrl = (old & ~clear) | set;
	if (old != port->mctrl)
		port->ops->set_mctrl(port, port->mctrl);
	spin_unlock_irqrestore(&port->lock, flags);
}

#define uart_set_mctrl(port,set)	uart_update_mctrl(port,set,0)
#define uart_clear_mctrl(port,clear)	uart_update_mctrl(port,0,clear)

/*
 * Startup the port.  This will be called once per open.  All calls
 * will be serialised by the per-port semaphore.
 */
static int uart_startup(struct uart_state *state, int init_hw)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	struct uart_port *uport = state->uart_port;
	struct tty_port *port = &state->port;
#else
 #if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 28)
	struct uart_info *info = &state->info;
 #else
	struct uart_info *info = state->info;
 #endif
	struct uart_port *port = state->port;
#endif
	unsigned long page;
	int retval = 0;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	if (port->flags & ASYNC_INITIALIZED)
		return 0;
#else
	if (info->flags & UIF_INITIALIZED)
		return 0;
#endif
	/*
	 * Set the TTY IO error marker - we will only clear this
	 * once we have successfully opened the port.  Also set
	 * up the tty->alt_speed kludge
	 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
        set_bit(TTY_IO_ERROR, &port->tty->flags);
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)
	if (info->port.tty)
		set_bit(TTY_IO_ERROR, &info->port.tty->flags);
#else
	if (info->tty)
		set_bit(TTY_IO_ERROR, &info->tty->flags);
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
        if (uport->type == PORT_UNKNOWN)
		return 0;
#else
	if (port->type == PORT_UNKNOWN)
		return 0;
#endif

	/*
	 * Initialise and allocate the transmit and temporary
	 * buffer.
	 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	if (!state->xmit.buf) {
		page = get_zeroed_page(GFP_KERNEL);
		if (!page)
			return -ENOMEM;
		state->xmit.buf = (unsigned char *) page;
		uart_circ_clear(&state->xmit);
	}
#else
	if (!info->xmit.buf) {
		//printk("#### info xmit buf not exists before\n");
		page = get_zeroed_page(GFP_KERNEL);
		if (!page)
			return -ENOMEM;

		info->xmit.buf = (unsigned char *) page;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 10) // Po-Cheng Chen add, 02/15/2006
		info->tmpbuf = info->xmit.buf + UART_XMIT_SIZE;
		init_MUTEX(&info->tmpbuf_sem);
#endif // Po-Cheng Chen add, 02/15/2006
		uart_circ_clear(&info->xmit);
	}
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
       retval = uport->ops->startup(uport);
#else
	retval = port->ops->startup(port);
#endif
	if (retval == 0) {
		if (init_hw) {
			/*
			 * Initialise the hardware port settings.
			 */
			uart_change_speed(state, NULL);

			/*
			 * Setup the RTS and DTR signals once the
			 * port is open and ready to respond.
			 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
			if (port->tty->termios->c_cflag & CBAUD)
				uart_set_mctrl(uport, TIOCM_RTS | TIOCM_DTR);
#else
 #if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)
			if (info->port.tty->termios->c_cflag & CBAUD)
 #else
			if (info->tty->termios->c_cflag & CBAUD)
 #endif
				uart_set_mctrl(port, TIOCM_RTS | TIOCM_DTR);
#endif
		}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
		if (port->flags & ASYNC_CTS_FLOW) {
			spin_lock_irq(&uport->lock);
			if (!(uport->ops->get_mctrl(uport) & TIOCM_CTS))
				port->tty->hw_stopped = 1;
			spin_unlock_irq(&uport->lock);
		}
		set_bit(ASYNCB_INITIALIZED, &port->flags);
#else
		info->flags |= UIF_INITIALIZED;
#endif


#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
		clear_bit(TTY_IO_ERROR, &port->tty->flags);
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)
		clear_bit(TTY_IO_ERROR, &info->port.tty->flags);
#else
		clear_bit(TTY_IO_ERROR, &info->tty->flags);
#endif
	}

	if (retval && capable(CAP_SYS_ADMIN))
		retval = 0;

	return retval;
}

/*
 * This routine will shutdown a serial port; interrupts are disabled, and
 * DTR is dropped if the hangup on close termio flag is on.  Calls to
 * uart_shutdown are serialised by the per-port semaphore.
 */
static void uart_shutdown(struct uart_state *state)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
        struct uart_port *uport = state->uart_port;
        struct tty_port *port = &state->port;
        struct tty_struct *tty = port->tty;
#else
 #if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 28)
	struct uart_info *info = &state->info;
 #else
	struct uart_info *info = state->info;
 #endif
	struct uart_port *port = state->port;
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
  /*
         * Set the TTY IO error marker
         */
        if (tty)
                set_bit(TTY_IO_ERROR, &tty->flags);

        if (test_and_clear_bit(ASYNCB_INITIALIZED, &port->flags)) {
                /*
                 * Turn off DTR and RTS early.
                 */
                if (!tty || (tty->termios->c_cflag & HUPCL))
                        uart_clear_mctrl(uport, TIOCM_DTR | TIOCM_RTS);

                /*
                 * clear delta_msr_wait queue to avoid mem leaks: we may free
                 * the irq here so the queue might never be woken up.  Note
                 * that we won't end up waiting on delta_msr_wait again since
                 * any outstanding file descriptors should be pointing at
                 * hung_up_tty_fops now.
                 */
                wake_up_interruptible(&port->delta_msr_wait);

                /*
                 * Free the IRQ and disable the port.
                 */
                uport->ops->shutdown(uport);

                /*
                 * Ensure that the IRQ handler isn't running on another CPU.
                 */
                synchronize_irq(uport->irq);
        }

        /*
         * kill off our tasklet
         */
        tasklet_kill(&state->tlet);

        /*
         * Free the transmit buffer page.
         */
        if (state->xmit.buf) {
                free_page((unsigned long)state->xmit.buf);
                state->xmit.buf = NULL;
        }
#else                                                //kernel version <2.6.32
	if (!(info->flags & UIF_INITIALIZED))
		return;

	/*
	 * Turn off DTR and RTS early.
	 */
 #if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)
	if (!info->port.tty || (info->port.tty->termios->c_cflag & HUPCL))
 #else
	if (!info->tty || (info->tty->termios->c_cflag & HUPCL))
 #endif
		uart_clear_mctrl(port, TIOCM_DTR | TIOCM_RTS);

	/*
	 * clear delta_msr_wait queue to avoid mem leaks: we may free
	 * the irq here so the queue might never be woken up.  Note
	 * that we won't end up waiting on delta_msr_wait again since
	 * any outstanding file descriptors should be pointing at
	 * hung_up_tty_fops now.
	 */
	wake_up_interruptible(&info->delta_msr_wait);

	/*
	 * Free the IRQ and disable the port.
	 */
	port->ops->shutdown(port);

	/*
	 * Ensure that the IRQ handler isn't running on another CPU.
	 */
	synchronize_irq(port->irq);

	/*
	 * Free the transmit buffer page.
	 */
	if (info->xmit.buf) {
		free_page((unsigned long)info->xmit.buf);
		info->xmit.buf = NULL;
 #if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 10) // Po-Cheng Chen add, 02/15/2006
		info->tmpbuf = NULL;
 #endif // Po-Cheng Chen add, 02/15/2006
	}

	/*
	 * kill off our tasklet
	 */
	tasklet_kill(&info->tlet);
 #if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)
	if (info->port.tty)
		set_bit(TTY_IO_ERROR, &info->port.tty->flags);
 #else
	if (info->tty)
		set_bit(TTY_IO_ERROR, &info->tty->flags);
 #endif

	info->flags &= ~UIF_INITIALIZED;
#endif                                               //kernel version <2.6.32
}

/**
 *	uart_update_timeout - update per-port FIFO timeout.
 *	@port:  uart_port structure describing the port
 *	@cflag: termios cflag value
 *	@baud:  speed of the port
 *
 *	Set the port FIFO timeout value.  The @cflag value should
 *	reflect the actual hardware settings.
 */
//lipeng modify at 06/08/2006
//James.dai add UBUNTU_2_6_15 @V3.20 to support ubuntu6
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 15) || UBUNTU_2_6_15
void
uart_update_timeout(struct uart_port *port, unsigned int cflag,
		    unsigned int baud)
{
	unsigned int bits;

	/* byte size and parity */
	switch (cflag & CSIZE) {
	case CS5:
		bits = 7;
		break;
	case CS6:
		bits = 8;
		break;
	case CS7:
		bits = 9;
		break;
	default:
		bits = 10;
		break; // CS8
	}

	if (cflag & CSTOPB)
		bits++;
	if (cflag & PARENB)
		bits++;

	/*
	 * The total number of bits to be transmitted in the fifo.
	 */
	bits = bits * port->fifosize;

	/*
	 * Figure the timeout to send the above number of bits.
	 * Add .02 seconds of slop
	 */
	port->timeout = (HZ * bits) / baud + HZ/50;
}

EXPORT_SYMBOL(uart_update_timeout);
#else
void
adv_uart_update_timeout(struct uart_port *port, unsigned int cflag,
		    unsigned int baud)
{
	unsigned int bits;

	/* byte size and parity */
	switch (cflag & CSIZE) {
	case CS5:
		bits = 7;
		break;
	case CS6:
		bits = 8;
		break;
	case CS7:
		bits = 9;
		break;
	default:
		bits = 10;
		break; // CS8
	}

	if (cflag & CSTOPB)
		bits++;
	if (cflag & PARENB)
		bits++;

	/*
	 * The total number of bits to be transmitted in the fifo.
	 */
	bits = bits * port->fifosize;

	/*
	 * Figure the timeout to send the above number of bits.
	 * Add .02 seconds of slop
	 */
	port->timeout = (HZ * bits) / baud + HZ/50;
}

EXPORT_SYMBOL(adv_uart_update_timeout);
#endif
//lipeng modify end
/**
 *	uart_get_baud_rate - return baud rate for a particular port
 *	@port: uart_port structure describing the port in question.
 *	@termios: desired termios settings.
 *	@old: old termios (or NULL)
 *	@min: minimum acceptable baud rate
 *	@max: maximum acceptable baud rate
 *
 *	Decode the termios structure into a numeric baud rate,
 *	taking account of the magic 38400 baud rate (with spd_*
 *	flags), and mapping the %B0 rate to 9600 baud.
 *
 *	If the new baud rate is invalid, try the old termios setting.
 *	If it's still invalid, we try 9600 baud.
 *
 *	Update the @termios structure to reflect the baud rate
 *	we're actually going to be using.
 */
//lipeng modify at 06/08/2006
//James.dai add UBUNTU_2_6_15 @V3.20 to support ubuntu6
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 15) || UBUNTU_2_6_15
unsigned int
uart_get_baud_rate(struct uart_port *port, struct termios *termios,
		   struct termios *old, unsigned int min, unsigned int max)
{
	unsigned int try, baud, altbaud = 38400;
	unsigned int flags = port->flags & UPF_SPD_MASK;

	if (flags == UPF_SPD_HI)
		altbaud = 57600;
	if (flags == UPF_SPD_VHI)
		altbaud = 115200;
	if (flags == UPF_SPD_SHI)
		altbaud = 230400;
	if (flags == UPF_SPD_WARP)
		altbaud = 460800;

	for (try = 0; try < 2; try++) {
		baud = tty_termios_baud_rate(termios);

		/*
		 * The spd_hi, spd_vhi, spd_shi, spd_warp kludge...
		 * Die! Die! Die!
		 */
		if (baud == 38400)
			baud = altbaud;

		/*
		 * Special case: B0 rate.
		 */
		if (baud == 0)
			baud = 9600;

		if (baud >= min && baud <= max)
			return baud;

		/*
		 * Oops, the quotient was zero.  Try again with
		 * the old baud rate if possible.
		 */
		termios->c_cflag &= ~CBAUD;
		if (old) {
			termios->c_cflag |= old->c_cflag & CBAUD;
			old = NULL;
			continue;
		}

		/*
		 * As a last resort, if the quotient is zero,
		 * default to 9600 bps
		 */
		termios->c_cflag |= B9600;
	}

	return 0;
}

EXPORT_SYMBOL(uart_get_baud_rate);
#else
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 23)
unsigned int
adv_uart_get_baud_rate(struct uart_port *port, struct termios *termios,
		   struct termios *old, unsigned int min, unsigned int max)
#else
unsigned int
adv_uart_get_baud_rate(struct uart_port *port, struct ktermios *termios,
		   struct ktermios *old, unsigned int min, unsigned int max)
#endif
{
	unsigned int try, baud, altbaud = 38400;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	int hung_up = 0;
#endif
	unsigned int flags = port->flags & UPF_SPD_MASK;

	if (flags == UPF_SPD_HI)
		altbaud = 57600;
	if (flags == UPF_SPD_VHI)
		altbaud = 115200;
	if (flags == UPF_SPD_SHI)
		altbaud = 230400;
	if (flags == UPF_SPD_WARP)
		altbaud = 460800;

	for (try = 0; try < 2; try++) {
		baud = tty_termios_baud_rate(termios);

		/*
		 * The spd_hi, spd_vhi, spd_shi, spd_warp kludge...
		 * Die! Die! Die!
		 */
		if (baud == 38400)
			baud = altbaud;

		/*
		 * Special case: B0 rate.
		 */
		if (baud == 0)
		{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
			hung_up = 1;
#endif
			baud = 9600;

		}

		if (baud >= min && baud <= max)
			return baud;

		/*
		 * Oops, the quotient was zero.  Try again with
		 * the old baud rate if possible.
		 */
		termios->c_cflag &= ~CBAUD;
		if (old) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
                        baud = tty_termios_baud_rate(old);
                        if (!hung_up)
                                tty_termios_encode_baud_rate(termios,
                                                                baud, baud);
#else

			termios->c_cflag |= old->c_cflag & CBAUD;
#endif
			old = NULL;
			continue;
		}

		/*
		 * As a last resort, if the quotient is zero,
		 * default to 9600 bps
		 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
              if (!hung_up)
			tty_termios_encode_baud_rate(termios, 9600, 9600);
#else
		termios->c_cflag |= B9600;
#endif
	}

	return 0;
}

EXPORT_SYMBOL(adv_uart_get_baud_rate);
#endif
/**
 *	uart_get_divisor - return uart clock divisor
 *	@port: uart_port structure describing the port.
 *	@baud: desired baud rate
 *
 *	Calculate the uart clock divisor for the port.
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 15)
unsigned int
uart_get_divisor(struct uart_port *port, unsigned int baud)
{
	unsigned int quot;

	/*
	 * Old custom speed handling.
	 */
	if (baud == 38400 && (port->flags & UPF_SPD_MASK) == UPF_SPD_CUST)
	{
		quot = port->custom_divisor;
	}
	else
	{
		quot = port->uartclk / (16 * baud);
	}

	return quot;
}

EXPORT_SYMBOL(uart_get_divisor);
#else
unsigned int
adv_uart_get_divisor(struct uart_port *port, unsigned int baud)
{
	unsigned int quot;

	/*
	 * Old custom speed handling.
	 */
	if (baud == 38400 && (port->flags & UPF_SPD_MASK) == UPF_SPD_CUST)
		quot = port->custom_divisor;
	else
	{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
		quot = (port->uartclk + (8 * baud)) / (16 * baud);
#else
		quot = port->uartclk / (16 * baud);
#endif
	}

	return quot;
}

EXPORT_SYMBOL(adv_uart_get_divisor);
#endif
//lipeng modify end
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 23)
static void
uart_change_speed(struct uart_state *state, struct termios *old_termios)
#else
static void
uart_change_speed(struct uart_state *state, struct ktermios *old_termios)
#endif
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
       struct tty_port *port = &state->port;
       struct tty_struct *tty = port->tty;
#elif LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 28)
	struct tty_struct *tty = state->info.port.tty;
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)
	struct tty_struct *tty = state->info->port.tty;
#else
	struct tty_struct *tty = state->info->tty;
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
       struct uart_port *uport = state->uart_port;
#else
	struct uart_port *port = state->port;
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 23)
	struct termios *termios;
#else
	struct ktermios *termios;
#endif


	/*
	 * If we have no tty, termios, or the port does not exist,
	 * then we can't set the parameters for this port.
	 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
       if (!tty || !tty->termios || uport->type == PORT_UNKNOWN)
#else
	if (!tty || !tty->termios || port->type == PORT_UNKNOWN)
#endif
               return;

	termios = tty->termios;

	/*
	 * Set flags based on termios cflag
	 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	if (termios->c_cflag & CRTSCTS)
		set_bit(ASYNCB_CTS_FLOW, &port->flags);
	else
		clear_bit(ASYNCB_CTS_FLOW, &port->flags);

	if (termios->c_cflag & CLOCAL)
		clear_bit(ASYNCB_CHECK_CD, &port->flags);
	else
		set_bit(ASYNCB_CHECK_CD, &port->flags);
#elif LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 28)
	if (termios->c_cflag & CRTSCTS)
		state->info.flags |= UIF_CTS_FLOW;
	else
		state->info.flags &= ~UIF_CTS_FLOW;

	if (termios->c_cflag & CLOCAL)
		state->info.flags &= ~UIF_CHECK_CD;
	else
		state->info.flags |= UIF_CHECK_CD;
#else
	if (termios->c_cflag & CRTSCTS)
		state->info->flags |= UIF_CTS_FLOW;
	else
		state->info->flags &= ~UIF_CTS_FLOW;

	if (termios->c_cflag & CLOCAL)
		state->info->flags &= ~UIF_CHECK_CD;
	else
		state->info->flags |= UIF_CHECK_CD;
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
       uport->ops->set_termios(uport, termios, old_termios);
#else
	if(port->type == 1) printk("****** 8250 uart change speed\n");
	port->ops->set_termios(port, termios, old_termios);
#endif
}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
static inline int
#else
static inline void
#endif
__uart_put_char(struct uart_port *port, struct circ_buf *circ, unsigned char c)
{
	unsigned long flags;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
       int ret = 0;
#endif
	if (!circ->buf)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
		return 0;
#else
		return;
#endif

	spin_lock_irqsave(&port->lock, flags);
	if (uart_circ_chars_free(circ) != 0) {
		circ->buf[circ->head] = c;
		circ->head = (circ->head + 1) & (UART_XMIT_SIZE - 1);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
		ret = 1;
#endif
	}
	spin_unlock_irqrestore(&port->lock, flags);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	return ret;
#endif
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 10) // Po-Cheng Chen add, 02/15/2006
static inline int
__uart_user_write(struct uart_port *port, struct circ_buf *circ,
		  const unsigned char *buf, int count)
{
	unsigned long flags;
	int c, ret = 0;

	if (down_interruptible(&port->info->tmpbuf_sem))
		return -EINTR;

	while (1) {
		int c1;
		c = CIRC_SPACE_TO_END(circ->head, circ->tail, UART_XMIT_SIZE);
		if (count < c)
			c = count;
		if (c <= 0)
			break;

		c -= copy_from_user(port->info->tmpbuf, buf, c);
		if (!c) {
			if (!ret)
				ret = -EFAULT;
			break;
		}
		spin_lock_irqsave(&port->lock, flags);
		c1 = CIRC_SPACE_TO_END(circ->head, circ->tail, UART_XMIT_SIZE);
		if (c1 < c)
			c = c1;
		memcpy(circ->buf + circ->head, port->info->tmpbuf, c);
		circ->head = (circ->head + c) & (UART_XMIT_SIZE - 1);
		spin_unlock_irqrestore(&port->lock, flags);
		buf += c;
		count -= c;
		ret += c;
	}
	up(&port->info->tmpbuf_sem);

	return ret;
}
#endif // Po-Cheng Chen add, 02/15/2006

static inline int
__uart_kern_write(struct uart_port *port, struct circ_buf *circ,
		  const unsigned char *buf, int count)
{
	unsigned long flags;
	int c, ret = 0;

	spin_lock_irqsave(&port->lock, flags);
	while (1) {
		c = CIRC_SPACE_TO_END(circ->head, circ->tail, UART_XMIT_SIZE);
		if (count < c)
			c = count;
		if (c <= 0)
			break;
		memcpy(circ->buf + circ->head, buf, c);
		circ->head = (circ->head + c) & (UART_XMIT_SIZE - 1);
		buf += c;
		count -= c;
		ret += c;
	}
	spin_unlock_irqrestore(&port->lock, flags);

	return ret;
}
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 26) 
static void uart_put_char(struct tty_struct *tty, unsigned char ch)
#else
static int uart_put_char(struct tty_struct *tty, unsigned char ch)
#endif
{
	struct uart_state *state = tty->driver_data;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
        return __uart_put_char(state->uart_port, &state->xmit, ch);
//#endif

#elif LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 28)
 	__uart_put_char(state->port, &state->info.xmit, ch);
	return 0;
#else
	__uart_put_char(state->port, &state->info->xmit, ch);
 #if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 26)
	return 0;
 #endif
#endif

}

static void uart_flush_chars(struct tty_struct *tty)
{
	uart_start(tty);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 10) // Po-Cheng Chen add, 02/15/2006
static int
uart_write(struct tty_struct *tty, int from_user, const unsigned char * buf,
	   int count)
{
	struct uart_state *state = tty->driver_data;
	int ret;

	if (!state->info->xmit.buf)
		return 0;

	if (from_user)
		ret = __uart_user_write(state->port, &state->info->xmit, buf, count);
	else
		ret = __uart_kern_write(state->port, &state->info->xmit, buf, count);

	uart_start(tty);
	return ret;
}
#else
static int
uart_write(struct tty_struct *tty, const unsigned char * buf,
	   int count)
{
	struct uart_state *state = tty->driver_data;
	int ret=0;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
       struct uart_port *port;
       struct circ_buf *circ;
       unsigned long flags;
       int c;
       /*
        * This means you called this function _after_ the port was
        * closed.  No cookie for you.
        */
       if (!state) {
               WARN_ON(1);
               return -EL3HLT;
        }
       port = state->uart_port;
       circ = &state->xmit;
       if (!circ->buf)
               return 0;
       spin_lock_irqsave(&port->lock, flags);
       while (1) {
               c = CIRC_SPACE_TO_END(circ->head, circ->tail, UART_XMIT_SIZE);
               if (count < c)
                       c = count;
               if (c <= 0)
                       break;
               memcpy(circ->buf + circ->head, buf, c);
               circ->head = (circ->head + c) & (UART_XMIT_SIZE - 1);
               buf += c;
               count -= c;
               ret += c;
       }
      spin_unlock_irqrestore(&port->lock, flags);
#elif LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 28)
	if (!state->info.xmit.buf)
		return 0;

	ret = __uart_kern_write(state->port, &state->info.xmit, buf, count);
#else
	if (!state->info->xmit.buf)
		return 0;

	ret = __uart_kern_write(state->port, &state->info->xmit, buf, count);
#endif
	uart_start(tty);
	return ret;
}
#endif // Po-Cheng Chen add, 02/15/2006

static int uart_write_room(struct tty_struct *tty)
{
	struct uart_state *state = tty->driver_data;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
       unsigned long flags;
       int ret;
       spin_lock_irqsave(&state->uart_port->lock, flags);
       ret = uart_circ_chars_free(&state->xmit);
       spin_unlock_irqrestore(&state->uart_port->lock, flags);
       return ret;
#elif LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 28)
	return uart_circ_chars_free(&state->info.xmit);
#else
	return uart_circ_chars_free(&state->info->xmit);
#endif
}

static int uart_chars_in_buffer(struct tty_struct *tty)
{
	struct uart_state *state = tty->driver_data;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
       unsigned long flags;
       int ret;
       spin_lock_irqsave(&state->uart_port->lock, flags);
       ret = uart_circ_chars_pending(&state->xmit);
       spin_unlock_irqrestore(&state->uart_port->lock, flags);
       return ret;
#elif LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 28)
	return uart_circ_chars_pending(&state->info.xmit);
#else
	return uart_circ_chars_pending(&state->info->xmit);
#endif
}

static void uart_flush_buffer(struct tty_struct *tty)
{
	struct uart_state *state = tty->driver_data;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
       struct uart_port *port = state->uart_port;
#else
	struct uart_port *port = state->port;
#endif
	unsigned long flags;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	pr_debug("uart_flush_buffer(%d) called\n", tty->index);
#else
	DPRINTK("uart_flush_buffer(%d) called\n", tty->index);
#endif
	spin_lock_irqsave(&port->lock, flags);
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 28)
 #if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
       uart_circ_clear(&state->xmit);
 #else
	uart_circ_clear(&state->info.xmit);
 #endif
	if (port->ops->flush_buffer)
		port->ops->flush_buffer(port);
	spin_unlock_irqrestore(&port->lock, flags);
	tty_wakeup(tty);
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)
	uart_circ_clear(&state->info->xmit);
	if (port->ops->flush_buffer)
		port->ops->flush_buffer(port);
	spin_unlock_irqrestore(&port->lock, flags);
	tty_wakeup(tty);
#else
	uart_circ_clear(&state->info->xmit);
	spin_unlock_irqrestore(&port->lock, flags);
	wake_up_interruptible(&tty->write_wait);
	if ((tty->flags & (1 << TTY_DO_WRITE_WAKEUP)) &&
	    tty->ldisc.write_wakeup)
		(tty->ldisc.write_wakeup)(tty);
#endif
}

/*
 * This function is used to send a high-priority XON/XOFF character to
 * the device
 */
static void uart_send_xchar(struct tty_struct *tty, char ch)
{
	struct uart_state *state = tty->driver_data;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
       struct uart_port *port = state->uart_port;
#else
	struct uart_port *port = state->port;
#endif
	unsigned long flags;

	if (port->ops->send_xchar)
		port->ops->send_xchar(port, ch);
	else {
		port->x_char = ch;
		if (ch) {
			spin_lock_irqsave(&port->lock, flags);
//lipeng modify at 06/08/2006
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 15)
	        	port->ops->start_tx(port, 0);
#else 
			port->ops->start_tx(port);
#endif
//lipeng modify end
			spin_unlock_irqrestore(&port->lock, flags);
		}
	}
}

static void uart_throttle(struct tty_struct *tty)
{
	//printk(KERN_INFO "## uart_throttle ##\n");
	struct uart_state *state;
	state = tty->driver_data;
	int status;
	//lipeng add at 01/04/2008
	int port_num; 
	struct uart_8250_port *port8250; 
	
/*
	//jinxin added begin
	if(haveDMA){
		port8250->ier &= ~(UART_IER_RDI);	
		serial_outp(port8250, UART_IER, port8250->ier);
	}
	//jinxin added end
*/
	if (I_IXOFF(tty))
	{
		//printk("uart_throttle:send XOFF char\n");
		uart_send_xchar(tty, STOP_CHAR(tty));
	}
	//now RTS in MCR is high(inactive), so set RTS to low(active)
	if (tty->termios->c_cflag & CRTSCTS)
	{
		//printk("uart_throttle:set MCR[1]:RTS to inactive(high)\n");
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
		uart_clear_mctrl(state->uart_port, TIOCM_RTS);
#else
		uart_clear_mctrl(state->port, TIOCM_RTS);
#endif
	}
	//jinxin added begin
	if (tty->termios->c_cflag & CDTRDSR)
	{
		//printk("uart_throttle:set MCR[1]:RTS to inactive(high)\n");
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
		uart_clear_mctrl(state->uart_port, TIOCM_DTR);
#else
		uart_clear_mctrl(state->port, TIOCM_DTR);
#endif
	}
/*
	if(haveDMA){
		status = serial_in(port8250, UART_LSR);
		if (status & UART_LSR_DR)
			receive_chars_unthrottle(port8250, &status, 0);
	}	
*/
	//jinxin added end
}

static void uart_unthrottle(struct tty_struct *tty)
{
	//printk(KERN_INFO "## uart_unthrottle ##\n");
	struct uart_state *state = tty->driver_data;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	struct uart_port *port = state->uart_port;
#else
	struct uart_port *port = state->port;
#endif
/*
	//jinxin added begin
	if(haveDMA){
		port8250->ier |= (UART_IER_RDI);	
		serial_outp(port8250, UART_IER, port8250->ier);
	}
*/
	//lipeng add at 01/04/2008
	if (I_IXOFF(tty)) {
		if (port->x_char)
			port->x_char = 0;
		else
			uart_send_xchar(tty, START_CHAR(tty));
	}

	if (tty->termios->c_cflag & CRTSCTS)
		uart_set_mctrl(port, TIOCM_RTS);
	//jinxin added begin
	if (tty->termios->c_cflag & CDTRDSR)
		uart_set_mctrl(port, TIOCM_DTR);
	//jinxin added end
}

static int uart_get_info(struct uart_state *state, struct serial_struct *retinfo)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
       struct uart_port *uport = state->uart_port;
       struct tty_port *port = &state->port;
#else
	struct uart_port *port = state->port;
#endif
	struct serial_struct tmp;

	memset(&tmp, 0, sizeof(tmp));
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
        /* Ensure the state we copy is consistent and no hardware changes
          occur as we go */
       mutex_lock(&port->mutex);
       tmp.type            = uport->type;
       tmp.line            = uport->line;
       tmp.port            = uport->iobase;
       if (HIGH_BITS_OFFSET)
               tmp.port_high = (long) uport->iobase >> HIGH_BITS_OFFSET;
       tmp.irq             = uport->irq;
       tmp.flags           = uport->flags;
       tmp.xmit_fifo_size  = uport->fifosize;
       tmp.baud_base       = uport->uartclk / 16;
       tmp.close_delay     = port->close_delay / 10;
       tmp.closing_wait    = port->closing_wait == ASYNC_CLOSING_WAIT_NONE ?
                               ASYNC_CLOSING_WAIT_NONE :
                               port->closing_wait / 10;
       tmp.custom_divisor  = uport->custom_divisor;
       tmp.hub6            = uport->hub6;
       tmp.io_type         = uport->iotype;
       tmp.iomem_reg_shift = uport->regshift;
       tmp.iomem_base      = (void *)(unsigned long)uport->mapbase;
	/*
	* Report serial port type
	*/
       tmp.reserved_char[0] = uport->unused[0];
       mutex_unlock(&port->mutex);
#else
        mutex_lock(&state->mutex);
	tmp.type	    = port->type;
	tmp.line	    = port->line;
	tmp.port	    = port->iobase;
	if (HIGH_BITS_OFFSET)
		tmp.port_high = (long) port->iobase >> HIGH_BITS_OFFSET;
	tmp.irq		    = port->irq;
	tmp.flags	    = port->flags;
	tmp.xmit_fifo_size  = port->fifosize;
	tmp.baud_base	    = port->uartclk / 16;
	tmp.close_delay	    = state->close_delay;
	tmp.closing_wait    = state->closing_wait;
	tmp.custom_divisor  = port->custom_divisor;
	tmp.hub6	    = port->hub6;
	tmp.io_type         = port->iotype;
	tmp.iomem_reg_shift = port->regshift;
	tmp.iomem_base      = (void *)(unsigned long)port->mapbase;
	/*
	* Report serial port type
	*/
	tmp.reserved_char[0] = port->unused[0];
        mutex_unlock(&state->mutex);
#endif
	if (copy_to_user(retinfo, &tmp, sizeof(*retinfo)))
		return -EFAULT;
	return 0;
}

static int
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
uart_set_info(struct uart_state *state, struct serial_struct __user *newinfo)
#else
uart_set_info(struct uart_state *state, struct serial_struct *newinfo)
#endif
{      
	struct serial_struct new_serial;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
       struct uart_port *uport = state->uart_port;
       struct tty_port *port = &state->port;
       unsigned int change_irq, change_port, closing_wait;        
	unsigned int old_custom_divisor, close_delay;
       upf_t old_flags, new_flags;
#else
	struct uart_port *port = state->port;
	unsigned int change_irq, change_port, old_flags;
	unsigned int old_custom_divisor;
#endif
	unsigned long new_port;
	int retval = 0;

	if (copy_from_user(&new_serial, newinfo, sizeof(new_serial)))
		return -EFAULT;

	new_port = new_serial.port;
	if (HIGH_BITS_OFFSET)
		new_port += (unsigned long) new_serial.port_high << HIGH_BITS_OFFSET;

	new_serial.irq = irq_canonicalize(new_serial.irq);

	/*
	 * This semaphore protects state->count.  It is also
	 * very useful to prevent opens.  Also, take the
	 * port configuration semaphore to make sure that a
	 * module insertion/removal doesn't change anything
	 * under us.
	 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
       close_delay = new_serial.close_delay * 10;
       closing_wait = new_serial.closing_wait == ASYNC_CLOSING_WAIT_NONE ?
                      ASYNC_CLOSING_WAIT_NONE : new_serial.closing_wait * 10;
#endif
//lipeng modify at 06/08/2006
//James.dai add  UBUNTU_2_6_15 @V3.20 to support ubuntu6
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
       mutex_lock(&port->mutex);
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 15) || UBUNTU_2_6_15
	down(&state->sem);
#else
	mutex_lock(&state->mutex);
#endif
//lipeng modify end
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
        change_irq  = !(uport->flags & UPF_FIXED_PORT)
			&& new_serial.irq != uport->irq;
#else
	change_irq  = new_serial.irq != port->irq;
#endif

	/*
	 * Since changing the 'type' of the port changes its resource
	 * allocations, we should treat type changes the same as
	 * IO port changes.
	 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
       change_port = !(uport->flags & UPF_FIXED_PORT)
               && (new_port != uport->iobase ||
                   (unsigned long)new_serial.iomem_base != uport->mapbase ||
                   new_serial.hub6 != uport->hub6 ||
                   new_serial.io_type != uport->iotype ||
                   new_serial.iomem_reg_shift != uport->regshift ||
                   new_serial.type != uport->type);
       old_flags = uport->flags;
       new_flags = new_serial.flags;
       old_custom_divisor = uport->custom_divisor;

       if (!capable(CAP_SYS_ADMIN)) {
               retval = -EPERM;
               if (change_irq || change_port ||
                   (new_serial.baud_base != uport->uartclk / 16) ||
                   (close_delay != port->close_delay) ||
                   (closing_wait != port->closing_wait) ||
                   (new_serial.xmit_fifo_size &&
                    new_serial.xmit_fifo_size != uport->fifosize) ||
                   (((new_flags ^ old_flags) & ~UPF_USR_MASK) != 0))
                       goto exit;
               uport->flags = ((uport->flags & ~UPF_USR_MASK) |
                              (new_flags & UPF_USR_MASK));
               uport->custom_divisor = new_serial.custom_divisor;
               goto check_and_exit;
       }
#else
	change_port = new_port != port->iobase ||
		      (unsigned long)new_serial.iomem_base != port->mapbase ||
		      new_serial.hub6 != port->hub6 ||
		      new_serial.io_type != port->iotype ||
		      new_serial.iomem_reg_shift != port->regshift ||
		      new_serial.type != port->type;
	//printk("the new type is %d\n",new_serial.type);
	old_flags = port->flags;
	old_custom_divisor = port->custom_divisor;

	if (!capable(CAP_SYS_ADMIN)) {
		retval = -EPERM;
		if (change_irq || change_port ||
		    (new_serial.baud_base != port->uartclk / 16) ||
		    (new_serial.close_delay != state->close_delay) ||
		    (new_serial.closing_wait != state->closing_wait) ||
		    (new_serial.xmit_fifo_size != port->fifosize) ||
		    (((new_serial.flags ^ old_flags) & ~UPF_USR_MASK) != 0))
			goto exit;
		port->flags = ((port->flags & ~UPF_USR_MASK) |
			       (new_serial.flags & UPF_USR_MASK));
		port->custom_divisor = new_serial.custom_divisor;
		goto check_and_exit;
	}
#endif
	/*
	 * Ask the low level driver to verify the settings.
	 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	if (uport->ops->verify_port)
		retval = uport->ops->verify_port(uport, &new_serial);
#else
	if (port->ops->verify_port)
		retval = port->ops->verify_port(port, &new_serial);
#endif

	if ((new_serial.irq >= NR_IRQS) || (new_serial.irq < 0) ||
	    (new_serial.baud_base < 9600))
		retval = -EINVAL;

	if (retval)
		goto exit;

	if (change_port || change_irq) {
		retval = -EBUSY;

		/*
		 * Make sure that we are the sole user of this port.
		 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
		if (tty_port_users(port) > 1)
#else
		if (uart_users(state) > 1)
#endif
			goto exit;

		/*
		 * We need to shutdown the serial port at the old
		 * port/type/irq combination.
		 */
		uart_shutdown(state);
	}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
        if (change_port) {
              unsigned long old_iobase, old_mapbase;
              unsigned int old_type, old_iotype, old_hub6, old_shift;
              old_iobase = uport->iobase;
              old_mapbase = uport->mapbase;
              old_type = uport->type;
              old_hub6 = uport->hub6;
              old_iotype = uport->iotype;
              old_shift = uport->regshift;
                /*
               * Free and release old regions
                */
              if (old_type != PORT_UNKNOWN)
                      uport->ops->release_port(uport);
              uport->iobase = new_port;
              uport->type = new_serial.type;
              uport->hub6 = new_serial.hub6;
              uport->iotype = new_serial.io_type;
              uport->regshift = new_serial.iomem_reg_shift;
              uport->mapbase = (unsigned long)new_serial.iomem_base;
                /*
               * Claim and map the new regions
               */
              if (uport->type != PORT_UNKNOWN) {
                      retval = uport->ops->request_port(uport);
              } else {
                     /* Always success - Jean II */
                      retval = 0;
                }
                /*
               * If we fail to request resources for the
               * new port, try to restore the old settings.
                */
              if (retval && old_type != PORT_UNKNOWN) {
                      uport->iobase = old_iobase;
                      uport->type = old_type;
                      uport->hub6 = old_hub6;
                      uport->iotype = old_iotype;
                      uport->regshift = old_shift;
                      uport->mapbase = old_mapbase;
                      retval = uport->ops->request_port(uport);
                      /*
                       * If we failed to restore the old settings,
                       * we fail like this.
                       */
                      if (retval)
                              uport->type = PORT_UNKNOWN;
                       /*
                        * We failed anyway.
                       */
                      retval = -EBUSY;
                      /* Added to return the correct error -Ram Gupta */
                       goto exit;
              }
        }

       if (change_irq)
              uport->irq      = new_serial.irq;
       if (!(uport->flags & UPF_FIXED_PORT))
               uport->uartclk  = new_serial.baud_base * 16;
       uport->flags            = (uport->flags & ~UPF_CHANGE_MASK) |
                                (new_flags & UPF_CHANGE_MASK);
       uport->custom_divisor   = new_serial.custom_divisor;
       port->close_delay     = close_delay;
       port->closing_wait    = closing_wait;
       if (new_serial.xmit_fifo_size)
               uport->fifosize = new_serial.xmit_fifo_size;
       if (port->tty)
               port->tty->low_latency =
                       (uport->flags & UPF_LOW_LATENCY) ? 1 : 0;
#else
	if (change_port) {
		unsigned long old_iobase, old_mapbase;
		unsigned int old_type, old_iotype, old_hub6, old_shift;

		old_iobase = port->iobase;
		old_mapbase = port->mapbase;
		old_type = port->type;
		old_hub6 = port->hub6;
		old_iotype = port->iotype;
		old_shift = port->regshift;

		/*
		 * Free and release old regions
		 */
		if (old_type != PORT_UNKNOWN)
			port->ops->release_port(port);
		//printk("the old port is:%d",port->iobase);
		port->iobase = new_port;
		//printk("the new port is:%d",port->iobase);
		port->type = new_serial.type;
		port->hub6 = new_serial.hub6;
		port->iotype = new_serial.io_type;
		port->regshift = new_serial.iomem_reg_shift;
		port->mapbase = (unsigned long)new_serial.iomem_base;

		/*
		 * Claim and map the new regions
		 */
		if (port->type != PORT_UNKNOWN) {
			retval = port->ops->request_port(port);
		} else {
			/* Always success - Jean II */
			retval = 0;
		}

		/*
		 * If we fail to request resources for the
		 * new port, try to restore the old settings.
		 */
		if (retval && old_type != PORT_UNKNOWN) {
			port->iobase = old_iobase;
			port->type = old_type;
			port->hub6 = old_hub6;
			port->iotype = old_iotype;
			port->regshift = old_shift;
			port->mapbase = old_mapbase;
			retval = port->ops->request_port(port);
			/*
			 * If we failed to restore the old settings,
			 * we fail like this.
			 */
			if (retval)
				port->type = PORT_UNKNOWN;

			/*
			 * We failed anyway.
			 */
			retval = -EBUSY;
		}
	}
	port->irq              = new_serial.irq;
	port->uartclk          = new_serial.baud_base * 16;
	port->flags            = (port->flags & ~UPF_CHANGE_MASK) |
				 (new_serial.flags & UPF_CHANGE_MASK);

	port->custom_divisor   = new_serial.custom_divisor;
	state->close_delay     = new_serial.close_delay * HZ / 100;
	state->closing_wait    =  new_serial.closing_wait * HZ / 100;

	/* WangMao revised the lines below @V3.20 to enable quick response function
           accroding using setserial command in user mode */
	if(new_serial.type==1){
		quick_response = 1;
		port->type = new_serial.type = 10;
		//port->fifosize = new_serial.xmit_fifo_size = 1;
	}
	port->fifosize         = new_serial.xmit_fifo_size;
	//port->fifosize = uart_config[new_serial.type].dfl_xmit_fifo_size;
	//printk("the new fifo size is:%d\n",port->fifosize);
	/* WangMao revise end */
 #if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 28)
	if (state->info.port.tty)
		state->info.port.tty->low_latency =
			(port->flags & UPF_LOW_LATENCY) ? 1 : 0;
 #elif LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)
	if (state->info->port.tty)
		state->info->port.tty->low_latency =
			(port->flags & UPF_LOW_LATENCY) ? 1 : 0;
 #else
	if (state->info->tty)
		state->info->tty->low_latency =
			(port->flags & UPF_LOW_LATENCY) ? 1 : 0;
 #endif
	//WangMao set low_latency flag in kernel 2.6 
	//state->info->tty->low_latency = 1;
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
 check_and_exit:
      retval = 0;
      if (uport->type == PORT_UNKNOWN)
              goto exit;
      if (port->flags & ASYNC_INITIALIZED) {
              if (((old_flags ^ uport->flags) & UPF_SPD_MASK) ||
                  old_custom_divisor != uport->custom_divisor) {
                      /*
                       * If they're setting up a custom divisor or speed,
                       * instead of clearing it, then bitch about it. No
                       * need to rate-limit; it's CAP_SYS_ADMIN only.
                       */
                      if (uport->flags & UPF_SPD_MASK) {
                              char buf[64];
                              printk(KERN_NOTICE
                                     "%s sets custom speed on %s. This "
                                     "is deprecated.\n", current->comm,
                                     tty_name(port->tty, buf));
                      }
                      uart_change_speed(state, NULL);
              }
      } else
              retval = uart_startup(state, 1);
exit:
      mutex_unlock(&port->mutex);
      return retval;
#else
 check_and_exit:
	retval = 0;
	if (port->type == PORT_UNKNOWN)
		goto exit;
 #if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 28)
	if (state->info.flags & UIF_INITIALIZED) {
 #else
	if (state->info->flags & UIF_INITIALIZED) {
 #endif
		if (((old_flags ^ port->flags) & UPF_SPD_MASK) ||
		    old_custom_divisor != port->custom_divisor) {
			/*
			 * If they're setting up a custom divisor or speed,
			 * instead of clearing it, then bitch about it. No
			 * need to rate-limit; it's CAP_SYS_ADMIN only.
			 */
			if (port->flags & UPF_SPD_MASK) {
				char buf[64];
 #if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 28)
				printk(KERN_NOTICE
				       "%s sets custom speed on %s. This "
				       "is deprecated.\n", current->comm,
				       tty_name(state->info.port.tty, buf));
 #elif LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)
				printk(KERN_NOTICE
				       "%s sets custom speed on %s. This "
				       "is deprecated.\n", current->comm,
				       tty_name(state->info->port.tty, buf));
 #else
				printk(KERN_NOTICE
				       "%s sets custom speed on %s. This "
				       "is deprecated.\n", current->comm,
				       tty_name(state->info->tty, buf));
 #endif
			}
			uart_change_speed(state, NULL);
		}
	} else
		retval = uart_startup(state, 1);
 exit:
//lipeng modify at 06/08/2006
 #if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 15) || UBUNTU_2_6_15
	up(&state->sem);
 #else
	mutex_unlock(&state->mutex);
 #endif
//lipeng modify end
	return retval;
#endif
}


/*
 * uart_get_lsr_info - get line status register info.
 * Note: uart_ioctl protects us against hangups.
 */
static int uart_get_lsr_info(struct uart_state *state, unsigned int *value)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	struct uart_port *uport = state->uart_port;
	struct tty_port *port = &state->port;
#else
	struct uart_port *port = state->port;
#endif
	unsigned int result;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	result = uport->ops->tx_empty(uport);
#else
	result = port->ops->tx_empty(port);
#endif

	/*
	 * If we're about to load something into the transmit
	 * register, we'll pretend the transmitter isn't empty to
	 * avoid a race condition (depending on when the transmit
	 * interrupt happens).
	 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	if (uport->x_char ||
	    ((uart_circ_chars_pending(&state->xmit) > 0) &&
	     !port->tty->stopped && !port->tty->hw_stopped))
		result &= ~TIOCSER_TEMT;

#elif LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 28)
	if (port->x_char ||
	    ((uart_circ_chars_pending(&state->info.xmit) > 0) &&
	     !state->info.port.tty->stopped && !state->info.port.tty->hw_stopped))
		result &= ~TIOCSER_TEMT;
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)
	if (port->x_char ||
	    ((uart_circ_chars_pending(&state->info->xmit) > 0) &&
	     !state->info->port.tty->stopped && !state->info->port.tty->hw_stopped))
		result &= ~TIOCSER_TEMT;
#else
	if (port->x_char ||
	    ((uart_circ_chars_pending(&state->info->xmit) > 0) &&
	     !state->info->tty->stopped && !state->info->tty->hw_stopped))
		result &= ~TIOCSER_TEMT;
#endif
	
	return put_user(result, value);
}
//lipeng modify at 06/08/2006
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 15) || UBUNTU_2_6_15
static int uart_tiocmget(struct tty_struct *tty, struct file *file)
{
	struct uart_state *state = tty->driver_data;
	struct uart_port *port = state->port;
	int result = -EIO;
	down(&state->sem);

	if ((!file || !tty_hung_up_p(file)) &&
	    !(tty->flags & (1 << TTY_IO_ERROR))) {
		result = port->mctrl;
		result |= port->ops->get_mctrl(port);
	}
	up(&state->sem);
	return result;
}
#else
static int uart_tiocmget(struct tty_struct *tty, struct file *file)
{
	struct uart_state *state = tty->driver_data;
 #if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	struct tty_port *port = &state->port;
	struct uart_port *uport = state->uart_port;
 #else
	struct uart_port *port = state->port;
 #endif
	int result = -EIO;

 #if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	mutex_lock(&port->mutex);
	if ((!file || !tty_hung_up_p(file)) &&
	    !(tty->flags & (1 << TTY_IO_ERROR))) {
		result = uport->mctrl;

		spin_lock_irq(&uport->lock);
		result |= uport->ops->get_mctrl(uport);
		spin_unlock_irq(&uport->lock);
	}
	mutex_unlock(&port->mutex);
 #else
	mutex_lock(&state->mutex);
	if ((!file || !tty_hung_up_p(file)) &&
	    !(tty->flags & (1 << TTY_IO_ERROR))) {
		result = port->mctrl;
		
		spin_lock_irq(&port->lock);
		result |= port->ops->get_mctrl(port);
		spin_unlock_irq(&port->lock);
	}
	mutex_unlock(&state->mutex);
 #endif
	return result;
}
#endif

//lipeng modify end
static int
uart_tiocmset(struct tty_struct *tty, struct file *file,
	      unsigned int set, unsigned int clear)
{
	struct uart_state *state = tty->driver_data;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	struct uart_port *uport = state->uart_port;
	struct tty_port *port = &state->port;
#else
	struct uart_port *port = state->port;
#endif

	int ret = -EIO;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	mutex_lock(&port->mutex);
	if ((!file || !tty_hung_up_p(file)) &&
	    !(tty->flags & (1 << TTY_IO_ERROR))) {
		uart_update_mctrl(uport, set, clear);
		ret = 0;
	}
	mutex_unlock(&port->mutex);
#else
//lipeng modify at 06/08/2006
 #if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 15) || UBUNTU_2_6_15
	down(&state->sem);
 #else
	mutex_lock(&state->mutex);
 #endif
//lipeng modify end
	if ((!file || !tty_hung_up_p(file)) &&
	    !(tty->flags & (1 << TTY_IO_ERROR))) {
		uart_update_mctrl(port, set, clear);
		ret = 0;
	}
//lipeng modify at 06/08/2006
 #if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 15) || UBUNTU_2_6_15
	up(&state->sem);
 #else
	mutex_unlock(&state->mutex);
 #endif
#endif
//lipeng modify end
	return ret;
}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
static int uart_break_ctl(struct tty_struct *tty, int break_state)
#else
static void uart_break_ctl(struct tty_struct *tty, int break_state)
#endif
{
	struct uart_state *state = tty->driver_data;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	struct tty_port *port = &state->port;
	struct uart_port *uport = state->uart_port;
#else
	struct uart_port *port = state->port;
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	mutex_lock(&port->mutex);

	if (uport->type != PORT_UNKNOWN)
		uport->ops->break_ctl(uport, break_state);

	mutex_unlock(&port->mutex);
	return 0;
#else
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 26)
	BUG_ON(!kernel_locked());
#endif
//lipeng modify at 06/08/2006
 #if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 15) || UBUNTU_2_6_15
	down(&state->sem);
 #else
	mutex_lock(&state->mutex);
 #endif
//lipeng modify end
	if (port->type != PORT_UNKNOWN)
		port->ops->break_ctl(port, break_state);
//lipeng modify at 06/08/2006
 #if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 15) || UBUNTU_2_6_15
	up(&state->sem);
 #else
	mutex_unlock(&state->mutex);
 #endif
//lipeng modify end
#endif
}

static int uart_do_autoconfig(struct uart_state *state)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	struct uart_port *uport = state->uart_port;
	struct tty_port *port = &state->port;
#else
	struct uart_port *port = state->port;
#endif
	int flags, ret;

	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;

	/*
	 * Take the per-port semaphore.  This prevents count from
	 * changing, and hence any extra opens of the port while
	 * we're auto-configuring.
	 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	if (mutex_lock_interruptible(&port->mutex))
//lipeng modify at 06/08/2006
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 15) || UBUNTU_2_6_15
	if (down_interruptible(&state->sem))
#else
	if (mutex_lock_interruptible(&state->mutex))
#endif
//lipeng modify end
		return -ERESTARTSYS;

	ret = -EBUSY;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	if (tty_port_users(port) == 1) {
		uart_shutdown(state);

		/*
		 * If we already have a port type configured,
		 * we must release its resources.
		 */
		if (uport->type != PORT_UNKNOWN)
			uport->ops->release_port(uport);

		flags = UART_CONFIG_TYPE;
		if (uport->flags & UPF_AUTO_IRQ)
			flags |= UART_CONFIG_IRQ;

		/*
		 * This will claim the ports resources if
		 * a port is found.
		 */
		uport->ops->config_port(uport, flags);

		ret = uart_startup(state, 1);
	}
#else
	if (uart_users(state) == 1) {
		uart_shutdown(state);

		/*
		 * If we already have a port type configured,
		 * we must release its resources.
		 */
		if (port->type != PORT_UNKNOWN)
			port->ops->release_port(port);

		flags = UART_CONFIG_TYPE;
		if (port->flags & UPF_AUTO_IRQ)
			flags |= UART_CONFIG_IRQ;

		/*
		 * This will claim the ports resources if
		 * a port is found.
		 */
		port->ops->config_port(port, flags);

		ret = uart_startup(state, 1);
	}
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	mutex_unlock(&port->mutex);
//lipeng modify at 06/08/2006
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 15) || UBUNTU_2_6_15
	up(&state->sem);
#else
	mutex_unlock(&state->mutex);
#endif
//lipeng modify end
	return ret;
}

/*
 * Wait for any of the 4 modem inputs (DCD,RI,DSR,CTS) to change
 * - mask passed in arg for lines of interest
 *   (use |'ed TIOCM_RNG/DSR/CD/CTS for masking)
 * Caller should use TIOCGICOUNT to see which one it was
 */
static int
uart_wait_modem_status(struct uart_state *state, unsigned long arg)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	struct uart_port *uport = state->uart_port;
	struct tty_port *port = &state->port;
#else
	struct uart_port *port = state->port;
#endif
	DECLARE_WAITQUEUE(wait, current);
	struct uart_icount cprev, cnow;
	int ret;

	/*
	 * note the counters on entry
	 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	spin_lock_irq(&uport->lock);
	memcpy(&cprev, &uport->icount, sizeof(struct uart_icount));

	/*
	 * Force modem status interrupts on
	 */
	uport->ops->enable_ms(uport);
	spin_unlock_irq(&uport->lock);

#else
	spin_lock_irq(&port->lock);
	memcpy(&cprev, &port->icount, sizeof(struct uart_icount));

	/*
	 * Force modem status interrupts on
	 */
	port->ops->enable_ms(port);
	spin_unlock_irq(&port->lock);
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	add_wait_queue(&port->delta_msr_wait, &wait);
#elif LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 28)
	add_wait_queue(&state->info.delta_msr_wait, &wait);
#else
	add_wait_queue(&state->info->delta_msr_wait, &wait);
#endif
	for (;;) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
		spin_lock_irq(&uport->lock);
		memcpy(&cnow, &uport->icount, sizeof(struct uart_icount));
		spin_unlock_irq(&uport->lock);
#else
		spin_lock_irq(&port->lock);
		memcpy(&cnow, &port->icount, sizeof(struct uart_icount));
		spin_unlock_irq(&port->lock);
#endif
		set_current_state(TASK_INTERRUPTIBLE);

		if (((arg & TIOCM_RNG) && (cnow.rng != cprev.rng)) ||
		    ((arg & TIOCM_DSR) && (cnow.dsr != cprev.dsr)) ||
		    ((arg & TIOCM_CD)  && (cnow.dcd != cprev.dcd)) ||
		    ((arg & TIOCM_CTS) && (cnow.cts != cprev.cts))) {
		    	ret = 0;
		    	break;
		}

		schedule();

		/* see if a signal did it */
		if (signal_pending(current)) {
			ret = -ERESTARTSYS;
			break;
		}

		cprev = cnow;
	}

	current->state = TASK_RUNNING;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	remove_wait_queue(&port->delta_msr_wait, &wait);
#elif LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 28)
	remove_wait_queue(&state->info.delta_msr_wait, &wait);
#else
	remove_wait_queue(&state->info->delta_msr_wait, &wait);
#endif

	return ret;
}

/*
 * Get counter of input serial line interrupts (DCD,RI,DSR,CTS)
 * Return: write counters to the user passed counter struct
 * NB: both 1->0 and 0->1 transitions are counted except for
 *     RI where only 0->1 is counted.
 */
static int
uart_get_count(struct uart_state *state, struct serial_icounter_struct *icnt)
{
	struct serial_icounter_struct icount;
	struct uart_icount cnow;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	struct uart_port *uport = state->uart_port;

	spin_lock_irq(&uport->lock);
	memcpy(&cnow, &uport->icount, sizeof(struct uart_icount));
	spin_unlock_irq(&uport->lock);
#else
	struct uart_port *port = state->port;

	spin_lock_irq(&port->lock);
	memcpy(&cnow, &port->icount, sizeof(struct uart_icount));
	spin_unlock_irq(&port->lock);
#endif
	icount.cts         = cnow.cts;
	icount.dsr         = cnow.dsr;
	icount.rng         = cnow.rng;
	icount.dcd         = cnow.dcd;
	icount.rx          = cnow.rx;
	icount.tx          = cnow.tx;
	icount.frame       = cnow.frame;
	icount.overrun     = cnow.overrun;
	icount.parity      = cnow.parity;
	icount.brk         = cnow.brk;
	icount.buf_overrun = cnow.buf_overrun;

	return copy_to_user(icnt, &icount, sizeof(icount)) ? -EFAULT : 0;
}

/*
 * Called via sys_ioctl under the BKL.  We can use spin_lock_irq() here.
 */
static int
uart_ioctl(struct tty_struct *tty, struct file *filp, unsigned int cmd,
	   unsigned long arg)
{
	struct uart_state *state = tty->driver_data;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	struct tty_port *port = &state->port;
	void __user *uarg = (void __user *)arg;
#endif
	int ret = -ENOIOCTLCMD;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 26)
	BUG_ON(!kernel_locked());
#endif

	/*
	 * These ioctls don't rely on the hardware to be present.
	 */
	switch (cmd) {
	case TIOCGSERIAL:
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
		ret = uart_get_info(state, uarg);
#else
		ret = uart_get_info(state, (struct serial_struct *)arg);
#endif
		break;

	case TIOCSSERIAL:
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
		ret = uart_set_info(state, uarg);
#else
		ret = uart_set_info(state, (struct serial_struct *)arg);
#endif
		break;

	case TIOCSERCONFIG:
		ret = uart_do_autoconfig(state);
		break;

	case TIOCSERGWILD: /* obsolete */
	case TIOCSERSWILD: /* obsolete */
		ret = 0;
		break;
	/* WangMao add the below to test software flow contron(using XON/XOFF),
           this would be useful when debug driver about XON/XOFF flow control */

	case SENDXON:
		printk("IOCTL:will send XON char\n");
		sendxon(tty);
		break;
	case SENDXOFF:
		printk("IOCTL: will send XOFF char\n");
		sendxoff(tty);			
		break;
	/*WangMao add the line below to enable quick response
	case ENABLE_QUICK_RESPONSE:
		copy_from_user(&quick_response,(void*)arg,sizeof(int));
		break;*/

	}

	if (ret != -ENOIOCTLCMD)
		goto out;

	if (tty->flags & (1 << TTY_IO_ERROR)) {
		ret = -EIO;
		goto out;
	}

	/*
	 * The following should only be used when hardware is present.
	 */
	switch (cmd) {
	case TIOCMIWAIT:
		ret = uart_wait_modem_status(state, arg);
		break;

	case TIOCGICOUNT:
		ret = uart_get_count(state, (struct serial_icounter_struct *)arg);
		break;
	}

	if (ret != -ENOIOCTLCMD)
		goto out;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	mutex_lock(&port->mutex);
//lipeng modify at 06/08/2006
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 15) || UBUNTU_2_6_15
	down(&state->sem);
#else
	mutex_lock(&state->mutex);
#endif
//lipeng modify end
	if (tty_hung_up_p(filp)) {
		ret = -EIO;
		goto out_up;
	}

	/*
	 * All these rely on hardware being present and need to be
	 * protected against the tty being hung up.
	 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	switch (cmd) {
	case TIOCSERGETLSR: /* Get line status register */
		ret = uart_get_lsr_info(state, uarg);
		break;

	default: {
		struct uart_port *uport = state->uart_port;
		if (uport->ops->ioctl)
			ret = uport->ops->ioctl(uport, cmd, arg);
		break;
	}
	}
#else
	switch (cmd) {
	case TIOCSERGETLSR: /* Get line status register */
		ret = uart_get_lsr_info(state, (unsigned int *)arg);
		break;

	default: {
		struct uart_port *port = state->port;
		if (port->ops->ioctl)
			ret = port->ops->ioctl(port, cmd, arg);
		break;
	}
	}
#endif

 out_up:
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	mutex_unlock(&port->mutex);
//lipeng modify at 06/08/2006
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 15) || UBUNTU_2_6_15
	up(&state->sem);
#else
	mutex_unlock(&state->mutex);
#endif
//lipeng modify end
 out:
	return ret;
}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
static void uart_set_ldisc(struct tty_struct *tty)
{
	struct uart_state *state = tty->driver_data;
	struct uart_port *uport = state->uart_port;

	if (uport->ops->set_ldisc)
		uport->ops->set_ldisc(uport);
}
#endif
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 23)
static void uart_set_termios(struct tty_struct *tty, struct termios *old_termios)
#else
static void uart_set_termios(struct tty_struct *tty, struct ktermios *old_termios)
#endif
{
	struct uart_state *state = tty->driver_data;
	unsigned long flags;
	unsigned int cflag = tty->termios->c_cflag;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 26)
	BUG_ON(!kernel_locked());
#endif

	/*
	 * These are the bits that are used to setup various
	 * flags in the low level driver.
	 */
#define RELEVANT_IFLAG(iflag)	((iflag) & (IGNBRK|BRKINT|IGNPAR|PARMRK|INPCK))

	if ((cflag ^ old_termios->c_cflag) == 0 &&
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	    tty->termios->c_ospeed == old_termios->c_ospeed &&
	    tty->termios->c_ispeed == old_termios->c_ispeed &&
#endif
	    RELEVANT_IFLAG(tty->termios->c_iflag ^ old_termios->c_iflag) == 0)
		return;

	uart_change_speed(state, old_termios);

	/* Handle transition to B0 status */
	if ((old_termios->c_cflag & CBAUD) && !(cflag & CBAUD))
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
		uart_clear_mctrl(state->uart_port, TIOCM_RTS | TIOCM_DTR);
#else
		uart_clear_mctrl(state->port, TIOCM_RTS | TIOCM_DTR);
#endif
	/* Handle transition away from B0 status */
	if (!(old_termios->c_cflag & CBAUD) && (cflag & CBAUD)) {
		unsigned int mask = TIOCM_DTR;
		if (!(cflag & CRTSCTS) ||
		    !test_bit(TTY_THROTTLED, &tty->flags))
			mask |= TIOCM_RTS;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
		uart_set_mctrl(state->uart_port, mask);
#else
		uart_set_mctrl(state->port, mask);
#endif
	}

	/* Handle turning off CRTSCTS */
	if ((old_termios->c_cflag & CRTSCTS) && !(cflag & CRTSCTS)) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
		spin_lock_irqsave(&state->uart_port->lock, flags);
#else
		spin_lock_irqsave(&state->port->lock, flags);
#endif
		tty->hw_stopped = 0;
		__uart_start(tty);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
		spin_unlock_irqrestore(&state->uart_port->lock, flags);
#else
		spin_unlock_irqrestore(&state->port->lock, flags);
#endif
	}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	/* Handle turning on CRTSCTS */
	if (!(old_termios->c_cflag & CRTSCTS) && (cflag & CRTSCTS)) {
		spin_lock_irqsave(&state->uart_port->lock, flags);
		if (!(state->uart_port->ops->get_mctrl(state->uart_port) & TIOCM_CTS)) {
			tty->hw_stopped = 1;
			state->uart_port->ops->stop_tx(state->uart_port);
		}
		spin_unlock_irqrestore(&state->uart_port->lock, flags);
	}
#endif
#if 0
	/*
	 * No need to wake up processes in open wait, since they
	 * sample the CLOCAL flag once, and don't recheck it.
	 * XXX  It's not clear whether the current behavior is correct
	 * or not.  Hence, this may change.....
	 */
	if (!(old_termios->c_cflag & CLOCAL) &&
	    (tty->termios->c_cflag & CLOCAL))
		wake_up_interruptible(&state->info->open_wait);
#endif
}

/*
 * In 2.4.5, calls to this will be serialized via the BKL in
 *  linux/drivers/char/tty_io.c:tty_release()
 *  linux/drivers/char/tty_io.c:do_tty_handup()
 */
static void uart_close(struct tty_struct *tty, struct file *filp)
{
	struct uart_state *state = tty->driver_data;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	struct tty_port *port;
	struct uart_port *uport;
#else
	struct uart_port *port;
#endif
	BUG_ON(!kernel_locked());

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	if (!state)
#else
	if (!state || !state->port)
#endif
		return;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	uport = state->uart_port;
	port = &state->port;
#else
	port = state->port;
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	pr_debug("uart_close(%d) called\n", uport->line);
#else
	DPRINTK("uart_close(%d) called\n", port->line);
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	mutex_lock(&port->mutex);
//lipeng modify at 06/08/2006
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 15) || UBUNTU_2_6_15
	down(&state->sem);
#else
	mutex_lock(&state->mutex);
#endif
//lipeng modify end
	if (tty_hung_up_p(filp))
		goto done;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	if ((tty->count == 1) && (port->count != 1)) {
		/*
		 * Uh, oh.  tty->count is 1, which means that the tty
		 * structure will be freed.  port->count should always
		 * be one in these conditions.  If it's greater than
		 * one, we've got real problems, since it means the
		 * serial port won't be shutdown.
		 */
		printk(KERN_ERR "uart_close: bad serial port count; tty->count is 1, "
		       "port->count is %d\n", port->count);
		port->count = 1;
	}
	if (--port->count < 0) {
		printk(KERN_ERR "uart_close: bad serial port count for %s: %d\n",
		       tty->name, port->count);
		port->count = 0;
	}
	if (port->count)
		goto done;
#else
	if ((tty->count == 1) && (state->count != 1)) {
		/*
		 * Uh, oh.  tty->count is 1, which means that the tty
		 * structure will be freed.  state->count should always
		 * be one in these conditions.  If it's greater than
		 * one, we've got real problems, since it means the
		 * serial port won't be shutdown.
		 */
		printk("uart_close: bad serial port count; tty->count is 1, "
		       "state->count is %d\n", state->count);
		state->count = 1;
	}
	if (--state->count < 0) {
		printk("rs_close: bad serial port count for %s: %d\n",
		       tty->name, state->count);
		state->count = 0;
	}
	if (state->count)
		goto done;
#endif
	/*
	 * Now we wait for the transmit buffer to clear; and we notify
	 * the line discipline to only process XON/XOFF characters by
	 * setting tty->closing.
	 */
	tty->closing = 1;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	if (port->closing_wait != ASYNC_CLOSING_WAIT_NONE)
		tty_wait_until_sent(tty, msecs_to_jiffies(port->closing_wait));

	/*
	 * At this point, we stop accepting input.  To do this, we
	 * disable the receive line status interrupts.
	 */
	if (port->flags & ASYNC_INITIALIZED) {
		unsigned long flags;
		spin_lock_irqsave(&uport->lock, flags);
		uport->ops->stop_rx(uport);
		spin_unlock_irqrestore(&uport->lock, flags);
		/*
		 * Before we drop DTR, make sure the UART transmitter
		 * has completely drained; this is especially
		 * important if there is a transmit FIFO!
		 */
		uart_wait_until_sent(tty, uport->timeout);
	}
#else
	if (state->closing_wait != USF_CLOSING_WAIT_NONE)
		tty_wait_until_sent(tty, state->closing_wait);

	/*
	 * At this point, we stop accepting input.  To do this, we
	 * disable the receive line status interrupts.
	 */
 #if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 28)
	if (state->info.flags & UIF_INITIALIZED) {
 #else
	if (state->info->flags & UIF_INITIALIZED) {
 #endif
		unsigned long flags;
		spin_lock_irqsave(&port->lock, flags);
		port->ops->stop_rx(port);
		spin_unlock_irqrestore(&port->lock, flags);
		/*
		 * Before we drop DTR, make sure the UART transmitter
		 * has completely drained; this is especially
		 * important if there is a transmit FIFO!
		 */
		uart_wait_until_sent(tty, port->timeout);
	}
#endif
	uart_shutdown(state);
	uart_flush_buffer(tty);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	tty_ldisc_flush(tty);
	tty->closing = 0;
	tty_port_tty_set(port, NULL);

	if (port->blocked_open) {
		if (port->close_delay)
			msleep_interruptible(port->close_delay);
	} else if (!uart_console(uport)) {
		uart_change_pm(state, 3);
	}

	/*
	 * Wake up anyone trying to open this port.
	 */
	clear_bit(ASYNCB_NORMAL_ACTIVE, &port->flags);
	wake_up_interruptible(&port->open_wait);
#elif LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 28)
	tty_ldisc_flush(tty);
	tty->closing = 0;
	state->info.port.tty = NULL;

	if (state->info.port.blocked_open) {
		if (state->close_delay) {
			set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout(state->close_delay);
		}
	} else if (!uart_console(port)) {
		uart_change_pm(state, 3);
	}

	/*
	 * Wake up anyone trying to open this port.
	 */
	state->info.flags &= ~UIF_NORMAL_ACTIVE;
	wake_up_interruptible(&state->info.port.open_wait);
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)
	if (tty->ldisc.ops->flush_buffer)
		tty->ldisc.ops->flush_buffer(tty);
	tty->closing = 0;
	state->info->port.tty = NULL;

	if (state->info->port.blocked_open) {
		if (state->close_delay) {
			set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout(state->close_delay);
		}
	} else if (!uart_console(port)) {
		uart_change_pm(state, 3);
	}

	/*
	 * Wake up anyone trying to open this port.
	 */
	state->info->flags &= ~UIF_NORMAL_ACTIVE;
	wake_up_interruptible(&state->info->port.open_wait);
#else
	if (tty->ldisc.flush_buffer)
		tty->ldisc.flush_buffer(tty);
	tty->closing = 0;
	state->info->tty = NULL;

	if (state->info->blocked_open) {
		if (state->close_delay) {
			set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout(state->close_delay);
		}
	} else if (!uart_console(port)) {
		uart_change_pm(state, 3);
	}

	/*
	 * Wake up anyone trying to open this port.
	 */
	state->info->flags &= ~UIF_NORMAL_ACTIVE;
	wake_up_interruptible(&state->info->open_wait);
#endif

 done:
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	mutex_unlock(&port->mutex);
//lipeng modify at 06/08/2006
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 15) || UBUNTU_2_6_15
	up(&state->sem);
#else
	mutex_unlock(&state->mutex);
#endif
//lipeng modify end
}

static void uart_wait_until_sent(struct tty_struct *tty, int timeout)
{
	struct uart_state *state = tty->driver_data;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	struct uart_port *port = state->uart_port;
#else
	struct uart_port *port = state->port;
#endif
	unsigned long char_time, expire;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 26)
	BUG_ON(!kernel_locked());
#endif
	if (port->type == PORT_UNKNOWN || port->fifosize == 0)
		return;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 26)
	lock_kernel();
#endif
	/*
	 * Set the check interval to be 1/5 of the estimated time to
	 * send a single character, and make it at least 1.  The check
	 * interval should also be less than the timeout.
	 *
	 * Note: we have to use pretty tight timings here to satisfy
	 * the NIST-PCTS.
	 */
	char_time = (port->timeout - HZ/50) / port->fifosize;
	char_time = char_time / 5;
	if (char_time == 0)
		char_time = 1;
	if (timeout && timeout < char_time)
		char_time = timeout;

	/*
	 * If the transmitter hasn't cleared in twice the approximate
	 * amount of time to send the entire FIFO, it probably won't
	 * ever clear.  This assumes the UART isn't doing flow
	 * control, which is currently the case.  Hence, if it ever
	 * takes longer than port->timeout, this is probably due to a
	 * UART bug of some kind.  So, we clamp the timeout parameter at
	 * 2*port->timeout.
	 */
	if (timeout == 0 || timeout > 2 * port->timeout)
		timeout = 2 * port->timeout;

	expire = jiffies + timeout;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	pr_debug("uart_wait_until_sent(%d), jiffies=%lu, expire=%lu...\n",
		port->line, jiffies, expire);
#else
	DPRINTK("uart_wait_until_sent(%d), jiffies=%lu, expire=%lu...\n",
	        port->line, jiffies, expire);
#endif
	/*
	 * Check whether the transmitter is empty every 'char_time'.
	 * 'timeout' / 'expire' give us the maximum amount of time
	 * we wait.
	 */
	while (!port->ops->tx_empty(port)) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
		msleep_interruptible(jiffies_to_msecs(char_time));
#else
		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(char_time);
#endif
		if (signal_pending(current))
			break;
		if (time_after(jiffies, expire))
			break;
	}
	set_current_state(TASK_RUNNING); /* might not be needed */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 26)
	unlock_kernel();
#endif
}

/*
 * This is called with the BKL held in
 *  linux/drivers/char/tty_io.c:do_tty_hangup()
 * We're called from the eventd thread, so we can sleep for
 * a _short_ time only.
 */
static void uart_hangup(struct tty_struct *tty)
{
	struct uart_state *state = tty->driver_data;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	struct tty_port *port = &state->port;
#endif
	BUG_ON(!kernel_locked());
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	pr_debug("uart_hangup(%d)\n", state->uart_port->line);
#else
	DPRINTK("uart_hangup(%d)\n", state->port->line);
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	mutex_lock(&port->mutex);
//lipeng modify at 06/08/2006
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 15) || UBUNTU_2_6_15
	down(&state->sem);
#else
	mutex_lock(&state->mutex);
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	if (port->flags & ASYNC_NORMAL_ACTIVE) {
		uart_flush_buffer(tty);
		uart_shutdown(state);
		port->count = 0;
		clear_bit(ASYNCB_NORMAL_ACTIVE, &port->flags);
		tty_port_tty_set(port, NULL);
		wake_up_interruptible(&port->open_wait);
		wake_up_interruptible(&port->delta_msr_wait);
	}
//lipeng modify end 
#elif LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 28)
	if (state->info.flags & UIF_NORMAL_ACTIVE) {
		uart_flush_buffer(tty);
		uart_shutdown(state);
		state->count = 0;
		state->info.flags &= ~UIF_NORMAL_ACTIVE;
		state->info.port.tty = NULL;
		wake_up_interruptible(&state->info.port.open_wait);
		wake_up_interruptible(&state->info.delta_msr_wait);
	}
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)
	if (state->info->flags & UIF_NORMAL_ACTIVE) {
		uart_flush_buffer(tty);
		uart_shutdown(state);
		state->count = 0;
		state->info->flags &= ~UIF_NORMAL_ACTIVE;
		state->info->port.tty = NULL;
		wake_up_interruptible(&state->info->port.open_wait);
		wake_up_interruptible(&state->info->delta_msr_wait);
	}
#else
	if (state->info && state->info->flags & UIF_NORMAL_ACTIVE) {
		uart_flush_buffer(tty);
		uart_shutdown(state);
		state->count = 0;
		state->info->flags &= ~UIF_NORMAL_ACTIVE;
		state->info->tty = NULL;
		wake_up_interruptible(&state->info->open_wait);
		wake_up_interruptible(&state->info->delta_msr_wait);
	}
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	mutex_unlock(&port->mutex);
//lipeng modify at 06/08/2006
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 15) || UBUNTU_2_6_15
	up(&state->sem);
#else
	mutex_unlock(&state->mutex);
#endif
//lipeng modify end
}

/*
 * Copy across the serial console cflag setting into the termios settings
 * for the initial open of the port.  This allows continuity between the
 * kernel settings, and the settings init adopts when it opens the port
 * for the first time.
 */
static void uart_update_termios(struct uart_state *state)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	struct tty_struct *tty = state->port.tty;
#elif LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 28)
	struct tty_struct *tty = state->info.port.tty;
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)
	struct tty_struct *tty = state->info->port.tty;
#else
	struct tty_struct *tty = state->info->tty;
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	struct uart_port *port = state->uart_port;
#else
	struct uart_port *port = state->port;
#endif
	if (uart_console(port) && port->cons->cflag) {
		tty->termios->c_cflag = port->cons->cflag;
		port->cons->cflag = 0;
	}

	/*
	 * If the device failed to grab its irq resources,
	 * or some other error occurred, don't try to talk
	 * to the port hardware.
	 */
	if (!(tty->flags & (1 << TTY_IO_ERROR))) {
		/*
		 * Make termios settings take effect.
		 */
		uart_change_speed(state, NULL);

		/*
		 * And finally enable the RTS and DTR signals.
		 */
		if (tty->termios->c_cflag & CBAUD)
			uart_set_mctrl(port, TIOCM_DTR | TIOCM_RTS);
	}
}

/*
 * Block the open until the port is ready.  We must be called with
 * the per-port semaphore held.
 */
static int
uart_block_til_ready(struct file *filp, struct uart_state *state)
{
	DECLARE_WAITQUEUE(wait, current);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	struct tty_port *port = &state->port;
	unsigned int mctrl;
#elif LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 28)
	struct uart_info *info = &state->info;
#else
	struct uart_info *info = state->info;
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	struct uart_port *uport = state->uart_port;
#else
	struct uart_port *port = state->port;
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	port->blocked_open++;
	port->count--;
	add_wait_queue(&port->open_wait, &wait);
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)
	info->port.blocked_open++;
	state->count--;
	add_wait_queue(&info->port.open_wait, &wait);
#else
	info->blocked_open++;
	state->count--;
	add_wait_queue(&info->open_wait, &wait);
#endif
	while (1) {
		set_current_state(TASK_INTERRUPTIBLE);

		/*
		 * If we have been hung up, tell userspace/restart open.
		 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
		if (tty_hung_up_p(filp) || port->tty == NULL)
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)
		if (tty_hung_up_p(filp) || info->port.tty == NULL)
#else
		if (tty_hung_up_p(filp) || info->tty == NULL)
#endif
			break;

		/*
		 * If the port has been closed, tell userspace/restart open.
		 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
		if (!(port->flags & ASYNC_INITIALIZED))
#else
		if (!(info->flags & UIF_INITIALIZED))
#endif
			break;

		/*
		 * If non-blocking mode is set, or CLOCAL mode is set,
		 * we don't want to wait for the modem status lines to
		 * indicate that the port is ready.
		 *
		 * Also, if the port is not enabled/configured, we want
		 * to allow the open to succeed here.  Note that we will
		 * have set TTY_IO_ERROR for a non-existant port.
		 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
		if ((filp->f_flags & O_NONBLOCK) ||
		    (port->tty->termios->c_cflag & CLOCAL) ||
		    (port->tty->flags & (1 << TTY_IO_ERROR)))
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)
		if ((filp->f_flags & O_NONBLOCK) ||
		    (info->port.tty->termios->c_cflag & CLOCAL) ||
		    (info->port.tty->flags & (1 << TTY_IO_ERROR)))
#else
		if ((filp->f_flags & O_NONBLOCK) ||
	            (info->tty->termios->c_cflag & CLOCAL) ||
		    (info->tty->flags & (1 << TTY_IO_ERROR))) 
#endif
			break;
		

		/*
		 * Set DTR to allow modem to know we're waiting.  Do
		 * not set RTS here - we want to make sure we catch
		 * the data from the modem.
		 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
		if (port->tty->termios->c_cflag & CBAUD)
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)
		if (info->port.tty->termios->c_cflag & CBAUD)
#else
		if (info->tty->termios->c_cflag & CBAUD)
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
			uart_set_mctrl(uport, TIOCM_DTR);
#else
			uart_set_mctrl(port, TIOCM_DTR);
#endif
		/*
		 * and wait for the carrier to indicate that the
		 * modem is ready for us.
		 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
		spin_lock_irq(&uport->lock);
		uport->ops->enable_ms(uport);
		mctrl = uport->ops->get_mctrl(uport);
		spin_unlock_irq(&uport->lock);
		if (mctrl & TIOCM_CAR)
			break;

		mutex_unlock(&port->mutex);
//lipeng modify at 06/08/2006
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 15) || UBUNTU_2_6_15
	if (port->ops->get_mctrl(port) & TIOCM_CAR)
		break;
	up(&state->sem);
#else
	spin_lock_irq(&port->lock);
	if (port->ops->get_mctrl(port) & TIOCM_CAR)
		break;
	spin_unlock_irq(&port->lock);
	mutex_unlock(&state->mutex);
#endif
//lipeng modify end
		schedule();
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
		mutex_lock(&port->mutex);
//lipeng modify at 06/08/2006
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 15) || UBUNTU_2_6_15
	down(&state->sem);
#else
	mutex_lock(&state->mutex);
#endif
//lipeng modify end
		if (signal_pending(current))
			break;
	}
	set_current_state(TASK_RUNNING);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	remove_wait_queue(&port->open_wait, &wait);

	port->count++;
	port->blocked_open--;

	if (signal_pending(current))
		return -ERESTARTSYS;

	if (!port->tty || tty_hung_up_p(filp))
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)
	remove_wait_queue(&info->port.open_wait, &wait);

	state->count++;
	info->port.blocked_open--;

	if (signal_pending(current))
		return -ERESTARTSYS;

	if (!info->port.tty || tty_hung_up_p(filp))
#else
	remove_wait_queue(&info->open_wait, &wait);

	state->count++;
	info->blocked_open--;

	if (signal_pending(current))
		return -ERESTARTSYS;

	if (!info->tty || tty_hung_up_p(filp))
#endif
		return -EAGAIN;

	return 0;
}

static struct uart_state *uart_get(struct uart_driver *drv, int line)
{
	struct uart_state *state;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	struct tty_port *port;
	int ret = 0;
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	state = drv->state + line;
	port = &state->port;
	if (mutex_lock_interruptible(&port->mutex))
//lipeng modify at 06/08/2006
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 15) || UBUNTU_2_6_15
	down(&port_sem);
	state = drv->state + line;
	if (down_interruptible(&state->sem))
#else
	mutex_lock(&port_mutex);
	state = drv->state + line;	
	if (mutex_lock_interruptible(&state->mutex))
#endif
//lipeng modify end
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	{
		ret = -ERESTARTSYS;
		goto err;
	}
#else
	{
		state = ERR_PTR(-ERESTARTSYS);
		goto out;
	}
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	port->count++;
	if (!state->uart_port || state->uart_port->flags & UPF_DEAD) {
		ret = -ENXIO;
		goto err_unlock;
	}
	return state;
#else
	state->count++;
	if (!state->port) {
		state->count--;
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
 err_unlock:
	port->count--;
	mutex_unlock(&port->mutex);
 err:
	return ERR_PTR(ret);
#else
//lipeng modify at 06/08/2006
 #if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 15) || UBUNTU_2_6_15
	up(&state->sem);
 #else
	mutex_unlock(&state->mutex);
 #endif
//lipeng modify end
		state = ERR_PTR(-ENXIO);
		goto out;
	}
 #if LINUX_VERSION_CODE <= KERNEL_VERSION(2, 6, 28)
	if (!state->info) {
 #if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)
			if (&state->info) {
			state->info = kzalloc(sizeof(struct uart_info), GFP_KERNEL);
			init_waitqueue_head(&state->info->port.open_wait);
			init_waitqueue_head(&state->info->delta_msr_wait);
 #elif LINUX_VERSION_CODE <= KERNEL_VERSION(2, 6, 26)
		state->info = kmalloc(sizeof(struct uart_info), GFP_KERNEL);
		if (state->info) {
			memset(state->info, 0, sizeof(struct uart_info));
			init_waitqueue_head(&state->info->open_wait);
			init_waitqueue_head(&state->info->delta_msr_wait);
 #endif

			/*
			 * Link the info into the other structures.
			 */
			state->port->info = state->info;

			tasklet_init(&state->info->tlet, uart_tasklet_action,
				     (unsigned long)state);
		} else {
			state->count--;
//lipeng modify at 06/08/2006
 #if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 15) || UBUNTU_2_6_15
	up(&state->sem);
 #else
	mutex_unlock(&state->mutex);
 #endif
//lipeng modify end
			state = ERR_PTR(-ENOMEM);
		}
	}
 #endif
//jinxin added end
 out:
 //lipeng modify at 06/08/2006
 #if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 15) || UBUNTU_2_6_15
	up(&port_sem); 
 #else
	mutex_unlock(&port_mutex);
 #endif
//lipeng modify end
	
	return state;
#endif
}

/*
 * In 2.4.5, calls to uart_open are serialised by the BKL in
 *   linux/fs/devices.c:chrdev_open()
 * Note that if this fails, then uart_close() _will_ be called.
 *
 * In time, we want to scrap the "opening nonpresent ports"
 * behaviour and implement an alternative way for setserial
 * to set base addresses/ports/types.  This will allow us to
 * get rid of a certain amount of extra tests.
 */
static inline void
uart_report_port(struct uart_driver *drv, struct uart_port *port);
static int uart_open(struct tty_struct *tty, struct file *filp)
{
	struct uart_driver *drv = (struct uart_driver *)tty->driver->driver_state;
	struct uart_state *state;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	struct tty_port *port;
#endif
	int retval, line = tty->index;

	BUG_ON(!kernel_locked());
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	pr_debug("uart_open(%d) called\n", line);
#else
	DPRINTK("uart_open(%d) called\n", line);
#endif
	/*
	 * tty->driver->num won't change, so we won't fail here with
	 * tty->driver_data set to something non-NULL (and therefore
	 * we won't get caught by uart_close()).
	 */
	retval = -ENODEV;
	if (line >= tty->driver->num)
		goto fail;

	/*
	 * We take the semaphore inside uart_get to guarantee that we won't
	 * be re-entered while allocating the info structure, or while we
	 * request any IRQs that the driver may need.  This also has the nice
	 * side-effect that it delays the action of uart_hangup, so we can
	 * guarantee that info->tty will always contain something reasonable.
	 */
	state = uart_get(drv, line);
	if (IS_ERR(state)) {
		retval = PTR_ERR(state);
		goto fail;
	}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	port = &state->port;
#endif
	/*
	 * Once we set tty->driver_data here, we are guaranteed that
	 * uart_close() will decrement the driver module use count.
	 * Any failures from here onwards should not touch the count.
	 */
	tty->driver_data = state;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	state->uart_port->state = state;
	tty->low_latency = (state->uart_port->flags & UPF_LOW_LATENCY) ? 1 : 0;
#else
	tty->low_latency = (state->port->flags & UPF_LOW_LATENCY) ? 1 : 0;
#endif
	tty->alt_speed = 0;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	tty_port_tty_set(port, tty);
#elif LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 28)
	state->info.port.tty = tty;
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)
	state->info->port.tty = tty;
#else
	state->info->tty = tty;
#endif

	/*
	 * If the port is in the middle of closing, bail out now.
	 */
	if (tty_hung_up_p(filp)) {
		retval = -EAGAIN;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
		port->count--;
#else
		state->count--;
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
		mutex_unlock(&port->mutex);
//lipeng modify at 06/08/2006
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 15) || UBUNTU_2_6_15
		up(&state->sem);
#else
		mutex_unlock(&state->mutex);
#endif
//lipeng modify end
		goto fail;
	}

	/*
	 * Make sure the device is in D0 state.
	 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	if (port->count == 1)
#else
	if (state->count == 1)
#endif
		uart_change_pm(state, 0);

	/*
	 * Start up the serial port.
	 */
	retval = uart_startup(state, 0);

	/*
	 * If we succeeded, wait until the port is ready.
	 */
	if (retval == 0)
		retval = uart_block_til_ready(filp, state);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	mutex_unlock(&port->mutex);
//lipeng modify at 06/08/2006
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 15) || UBUNTU_2_6_15
	up(&state->sem);
#else
	mutex_unlock(&state->mutex);
#endif
//lipeng modify end
	/*
	 * If this is the first open to succeed, adjust things to suit.
	 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	if (retval == 0 && !(port->flags & ASYNC_NORMAL_ACTIVE)) {
		set_bit(ASYNCB_NORMAL_ACTIVE, &port->flags);
#elif LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 28)
	if (retval == 0 && !(state->info.flags & UIF_NORMAL_ACTIVE)) {
		state->info.flags |= UIF_NORMAL_ACTIVE;
#else
	if (retval == 0 && !(state->info->flags & UIF_NORMAL_ACTIVE)) {
		state->info->flags |= UIF_NORMAL_ACTIVE;
#endif

		uart_update_termios(state);
	}

 fail:
	return retval;
}

static const char *uart_type(struct uart_port *port)
{
	const char *str = NULL;

	if (port->ops->type)
		str = port->ops->type(port);

	if (!str)
		str = "unknown";

	return str;
}

#ifdef CONFIG_PROC_FS
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
static void uart_line_info(struct seq_file *m, struct uart_driver *drv, int i)
#else
static int uart_line_info(char *buf, struct uart_driver *drv, int i)
#endif
{
	struct uart_state *state = drv->state + i;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	struct tty_port *port = &state->port;
	int pm_state;
	struct uart_port *uport = state->uart_port;
	int mmio;
#else
	struct uart_port *port = state->port;
	int ret;
#endif
	char stat_buf[32];
	unsigned int status;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	if (!uport)
		return;
	mmio = uport->iotype >= UPIO_MEM;
	seq_printf(m, "%d: uart:%s %s%08llX irq:%d",
			uport->line, uart_type(uport),
			mmio ? "mmio:0x" : "port:",
			mmio ? (unsigned long long)uport->mapbase
			     : (unsigned long long)uport->iobase,
			uport->irq);

	if (uport->type == PORT_UNKNOWN) {
		seq_putc(m, '\n');
		return;
	}
#else
	if (!port)
		return 0;
	ret = sprintf(buf, "%d: uart:%s %s%08lX irq:%d",
			port->line, uart_type(port),
			port->iotype == UPIO_MEM ? "mmio:0x" : "port:",
			port->iotype == UPIO_MEM ? (long unsigned int)port->mapbase :
						(unsigned long) port->iobase,
			port->irq);

	if (port->type == PORT_UNKNOWN) {
		strcat(buf, "\n");
		return ret + 1;
	}
#endif

	if(capable(CAP_SYS_ADMIN))
	{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
		mutex_lock(&port->mutex);
		pm_state = state->pm_state;
		if (pm_state)
			uart_change_pm(state, 0);
		spin_lock_irq(&uport->lock);
		status = uport->ops->get_mctrl(uport);
		spin_unlock_irq(&uport->lock);
		if (pm_state)
			uart_change_pm(state, pm_state);
		mutex_unlock(&port->mutex);

		seq_printf(m, " tx:%d rx:%d",
				uport->icount.tx, uport->icount.rx);
		if (uport->icount.frame)
			seq_printf(m, " fe:%d",
				uport->icount.frame);
		if (uport->icount.parity)
			seq_printf(m, " pe:%d",
				uport->icount.parity);
		if (uport->icount.brk)
			seq_printf(m, " brk:%d",
				uport->icount.brk);
		if (uport->icount.overrun)
			seq_printf(m, " oe:%d",
				uport->icount.overrun);
#else
//lipeng modify at 06/08/2006
 #if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 15) || UBUNTU_2_6_15
		status = port->ops->get_mctrl(port);
 #else
		spin_lock_irq(&port->lock);
		status = port->ops->get_mctrl(port);
		spin_unlock_irq(&port->lock);
 #endif
//lipeng modify end
		ret += sprintf(buf + ret, " tx:%d rx:%d",
				port->icount.tx, port->icount.rx);
		if (port->icount.frame)
			ret += sprintf(buf + ret, " fe:%d",
				port->icount.frame);
		if (port->icount.parity)
			ret += sprintf(buf + ret, " pe:%d",
				port->icount.parity);
		if (port->icount.brk)
			ret += sprintf(buf + ret, " brk:%d",
				port->icount.brk);
		if (port->icount.overrun)
			ret += sprintf(buf + ret, " oe:%d",
				port->icount.overrun);
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
#define INFOBIT(bit, str) \
	if (uport->mctrl & (bit)) \
		strncat(stat_buf, (str), sizeof(stat_buf) - \
			strlen(stat_buf) - 2)
#else
#define INFOBIT(bit,str) \
	if (port->mctrl & (bit)) \
		strncat(stat_buf, (str), sizeof(stat_buf) - \
			strlen(stat_buf) - 2)
#endif
#define STATBIT(bit,str) \
	if (status & (bit)) \
		strncat(stat_buf, (str), sizeof(stat_buf) - \
		       strlen(stat_buf) - 2)

		stat_buf[0] = '\0';
		stat_buf[1] = '\0';
		INFOBIT(TIOCM_RTS, "|RTS");
		STATBIT(TIOCM_CTS, "|CTS");
		INFOBIT(TIOCM_DTR, "|DTR");
		STATBIT(TIOCM_DSR, "|DSR");
		STATBIT(TIOCM_CAR, "|CD");
		STATBIT(TIOCM_RNG, "|RI");
		if (stat_buf[0])
			stat_buf[0] = ' ';
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
		seq_puts(m, stat_buf);
	}
	seq_putc(m, '\n');
#else
		strcat(stat_buf, "\n");
	
		ret += sprintf(buf + ret, stat_buf);
	} else {
		strcat(buf, "\n");
		ret++;
	}
#endif
#undef STATBIT
#undef INFOBIT
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 32)
	return ret;
#endif
}
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 32)
static int uart_read_proc(char *page, char **start, off_t off,
			  int count, int *eof, void *data)
{
	struct tty_driver *ttydrv = data;
	struct uart_driver *drv = ttydrv->driver_state;
	int i, len = 0, l;
	off_t begin = 0;

	len += sprintf(page, "serinfo:1.0 driver%s%s revision:%s\n",
			"", "", "");
	for (i = 0; i < drv->nr && len < PAGE_SIZE - 96; i++) {
		l = uart_line_info(page + len, drv, i);
		len += l;
		if (len + begin > off + count)
			goto done;
		if (len + begin < off) {
			begin += len;
			len = 0;
		}
	}
	*eof = 1;
 done:
	if (off >= len + begin)
		return 0;
	*start = page + (off - begin);
	return (count < begin + len - off) ? count : (begin + len - off);
}
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 31)
static int uart_proc_show(struct seq_file *m, void *v)
{
	struct tty_driver *ttydrv = m->private;
	struct uart_driver *drv = ttydrv->driver_state;
	int i;

	seq_printf(m, "serinfo:1.0 driver%s%s revision:%s\n",
			"", "", "");
	for (i = 0; i < drv->nr; i++)
		uart_line_info(m, drv, i);
	return 0;
}

static int uart_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, uart_proc_show, PDE(inode)->data);
}

static const struct file_operations uart_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= uart_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};
#endif

#endif

#if defined(CONFIG_SERIAL_CORE_CONSOLE) || defined(CONFIG_CONSOLE_POLL)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
void uart_console_write(struct uart_port *port, const char *s,
			unsigned int count,
			void (*putchar)(struct uart_port *, int))
{
	unsigned int i;

	for (i = 0; i < count; i++, s++) {
		if (*s == '\n')
			putchar(port, '\r');
		putchar(port, *s);
	}
}
//EXPORT_SYMBOL_GPL(uart_console_write);
//#else
//#ifdef CONFIG_SERIAL_CORE_CONSOLE
#endif
/*
 *	Check whether an invalid uart number has been specified, and
 *	if so, search for the first available port that does have
 *	console support.
 */
struct uart_port * __init
uart_get_console(struct uart_port *ports, int nr, struct console *co)
{
	int idx = co->index;

	if (idx < 0 || idx >= nr || (ports[idx].iobase == 0 &&
				     ports[idx].membase == NULL))
		for (idx = 0; idx < nr; idx++)
			if (ports[idx].iobase != 0 ||
			    ports[idx].membase != NULL)
				break;

	co->index = idx;

	return ports + idx;
}

/**
 *	uart_parse_options - Parse serial port baud/parity/bits/flow contro.
 *	@options: pointer to option string
 *	@baud: pointer to an 'int' variable for the baud rate.
 *	@parity: pointer to an 'int' variable for the parity.
 *	@bits: pointer to an 'int' variable for the number of data bits.
 *	@flow: pointer to an 'int' variable for the flow control character.
 *
 *	uart_parse_options decodes a string containing the serial console
 *	options.  The format of the string is <baud><parity><bits><flow>,
 *	eg: 115200n8r
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
void
#else
void __init
#endif
uart_parse_options(char *options, int *baud, int *parity, int *bits, int *flow)
{
	char *s = options;

	*baud = simple_strtoul(s, NULL, 10);
	while (*s >= '0' && *s <= '9')
		s++;
	if (*s)
		*parity = *s++;
	if (*s)
		*bits = *s++ - '0';
	if (*s)
		*flow = *s;
}
//#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
//EXPORT_SYMBOL_GPL(uart_parse_options);
//#endif
struct baud_rates {
	unsigned int rate;
	unsigned int cflag;
};
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
static const struct baud_rates baud_rates[] = {
#else
static struct baud_rates baud_rates[] = {
#endif
	{ 921600, B921600 },
	{ 460800, B460800 },
	{ 230400, B230400 },
	{ 115200, B115200 },
	{  57600, B57600  },
	{  38400, B38400  },
	{  19200, B19200  },
	{   9600, B9600   },
	{   4800, B4800   },
	{   2400, B2400   },
	{   1200, B1200   },
	{      0, B38400  }
};

/**
 *	uart_set_options - setup the serial console parameters
 *	@port: pointer to the serial ports uart_port structure
 *	@co: console pointer
 *	@baud: baud rate
 *	@parity: parity character - 'n' (none), 'o' (odd), 'e' (even)
 *	@bits: number of data bits
 *	@flow: flow control character - 'r' (rts)
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
int
#else
int __init
#endif
uart_set_options(struct uart_port *port, struct console *co,
		 int baud, int parity, int bits, int flow)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 23)
	struct termios termios;
#else
	struct ktermios termios;
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	static struct ktermios dummy;
#endif
	int i;

//lipeng modify at 06/08/2006
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 14) && !UBUNTU_2_6_15
	
	/*
	 * Ensure that the serial console lock is initialised
	 * early.
	 */
	spin_lock_init(&port->lock);

#endif	
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	lockdep_set_class(&port->lock, &port_lock_key);

	memset(&termios, 0, sizeof(struct ktermios));
#else
	memset(&termios, 0, sizeof(struct termios));
#endif
	termios.c_cflag = CREAD | HUPCL | CLOCAL;

	/*
	 * Construct a cflag setting.
	 */
	for (i = 0; baud_rates[i].rate; i++)
		if (baud_rates[i].rate <= baud)
			break;

	termios.c_cflag |= baud_rates[i].cflag;

	if (bits == 7)
		termios.c_cflag |= CS7;
	else
		termios.c_cflag |= CS8;

	switch (parity) {
	case 'o': case 'O':
		termios.c_cflag |= PARODD;
		/*fall through*/
	case 'e': case 'E':
		termios.c_cflag |= PARENB;
		break;
	}

	if (flow == 'r')
		termios.c_cflag |= CRTSCTS;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	port->mctrl |= TIOCM_DTR;

	port->ops->set_termios(port, &termios, &dummy);
	/*
	 * Allow the setting of the UART parameters with a NULL console
	 * too:
	 */
	if (co)
		co->cflag = termios.c_cflag;
#else

	port->ops->set_termios(port, &termios, NULL);
	co->cflag = termios.c_cflag;
#endif
	return 0;
}
//#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
//EXPORT_SYMBOL_GPL(uart_set_options);
//#endif
#endif /* CONFIG_SERIAL_CORE_CONSOLE */

static void uart_change_pm(struct uart_state *state, int pm_state)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	struct uart_port *port = state->uart_port;
#else
	struct uart_port *port = state->port;
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	if (state->pm_state != pm_state) {
#endif
	if (port->ops->pm)
		port->ops->pm(port, pm_state, state->pm_state);
	state->pm_state = pm_state;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	}
#endif
}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
struct uart_match {
	struct uart_port *port;
	struct uart_driver *driver;
};

static int serial_match_port(struct device *dev, void *data)
{
	struct uart_match *match = data;
	struct tty_driver *tty_drv = match->driver->tty_driver;
	dev_t devt = MKDEV(tty_drv->major, tty_drv->minor_start) +
		match->port->line;

	return dev->devt == devt; /* Actually, only one tty per port */
}
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
int adv_uart_suspend_port(struct uart_driver *drv, struct uart_port *uport)
#else
int adv_uart_suspend_port(struct uart_driver *drv, struct uart_port *port)
#endif
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	struct uart_state *state = drv->state + uport->line;
	struct tty_port *port = &state->port;
	struct device *tty_dev;
	struct uart_match match = {uport, drv};
#else
	struct uart_state *state = drv->state + port->line;
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	mutex_lock(&port->mutex);
//lipeng modify at 06/08/2006
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 15) || UBUNTU_2_6_15 
	down(&state->sem);
#else
	mutex_lock(&state->mutex);
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	if (!console_suspend_enabled && uart_console(uport)) {
		/* we're going to avoid suspending serial console */
		mutex_unlock(&port->mutex);
		return 0;
	}

	tty_dev = device_find_child(uport->dev, &match, serial_match_port);
	if (device_may_wakeup(tty_dev)) {
		enable_irq_wake(uport->irq);
		put_device(tty_dev);
		mutex_unlock(&port->mutex);
		return 0;
	}
	uport->suspended = 1;
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	if (port->flags & ASYNC_INITIALIZED) {
//lipeng modify end
#elif LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 28)
	if (state->info.flags & UIF_INITIALIZED) {
#else
	if (state->info && state->info->flags & UIF_INITIALIZED) {
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
		const struct uart_ops *ops = uport->ops;
		int tries;

		set_bit(ASYNCB_SUSPENDED, &port->flags);
		clear_bit(ASYNCB_INITIALIZED, &port->flags);
//lipeng modify at 06/08/2006
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 15) || UBUNTU_2_6_15 
 	        struct uart_ops *ops = port->ops;
#else
 	        const	struct uart_ops *ops = port->ops;
#endif
//lipeng modify end
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
		spin_lock_irq(&uport->lock);
#else
		spin_lock_irq(&port->lock);
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
		ops->stop_tx(uport);
//lipeng modify at 06/08/2006
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 15)
		ops->stop_tx(port, 0);
#else
		ops->stop_tx(port);
#endif
//lipeng modify end
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
		ops->set_mctrl(uport, 0);
		ops->stop_rx(uport);
		spin_unlock_irq(&uport->lock);

		/*
		 * Wait for the transmitter to empty.
		 */
		for (tries = 3; !ops->tx_empty(uport) && tries; tries--)
			msleep(10);
		if (!tries)
			printk(KERN_ERR "%s%s%s%d: Unable to drain "
					"transmitter\n",
			       uport->dev ? dev_name(uport->dev) : "",
			       uport->dev ? ": " : "",
			       drv->dev_name,
			       drv->tty_driver->name_base + uport->line);
		ops->shutdown(uport);
#else
		ops->set_mctrl(port, 0);
		ops->stop_rx(port);
		spin_unlock_irq(&port->lock);

		/*
		 * Wait for the transmitter to empty.
		 */
		while (!ops->tx_empty(port)) {
			set_current_state(TASK_UNINTERRUPTIBLE);
			schedule_timeout(10*HZ/1000);
		}
		set_current_state(TASK_RUNNING);
		ops->shutdown(port);
#endif
	}

	/*
	 * Disable the console device before suspending.
	 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	if (uart_console(uport))
		console_stop(uport->cons);
#else
	if (uart_console(port))
		port->cons->flags &= ~CON_ENABLED;
#endif
	uart_change_pm(state, 3);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	mutex_unlock(&port->mutex);
//lipeng modify at 06/08/2006
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 15) || UBUNTU_2_6_15 
	up(&state->sem);
#else
	mutex_unlock(&state->mutex);
#endif
//lipeng modify end
	return 0;
}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
int adv_uart_resume_port(struct uart_driver *drv, struct uart_port *uport)
#else
int adv_uart_resume_port(struct uart_driver *drv, struct uart_port *port)
#endif
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	struct uart_state *state = drv->state + uport->line;
	struct tty_port *port = &state->port;
	struct device *tty_dev;
	struct uart_match match = {uport, drv};
	struct ktermios termios;
#else
	struct uart_state *state = drv->state + port->line;
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	mutex_lock(&port->mutex);
//lipeng modify at 06/08/2006
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 15) || UBUNTU_2_6_15 
	down(&state->sem);
#else
	mutex_lock(&state->mutex);
#endif
//lipeng modify end
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	if (!console_suspend_enabled && uart_console(uport)) {
		/* no need to resume serial console, it wasn't suspended */
		/*
		 * First try to use the console cflag setting.
		 */
		memset(&termios, 0, sizeof(struct ktermios));
		termios.c_cflag = uport->cons->cflag;
		/*
		 * If that's unset, use the tty termios setting.
		 */
		if (termios.c_cflag == 0)
			termios = *state->port.tty->termios;
		else {
			termios.c_ispeed = termios.c_ospeed =
				tty_termios_input_baud_rate(&termios);
			termios.c_ispeed = termios.c_ospeed =
				tty_termios_baud_rate(&termios);
		}
		uport->ops->set_termios(uport, &termios, NULL);
		mutex_unlock(&port->mutex);
		return 0;
	}
	tty_dev = device_find_child(uport->dev, &match, serial_match_port);
	if (!uport->suspended && device_may_wakeup(tty_dev)) {
		disable_irq_wake(uport->irq);
		mutex_unlock(&port->mutex);
		return 0;
	}
	uport->suspended = 0;

	/*
	 * Re-enable the console device after suspending.
	 */
	if (uart_console(uport)) {
		uart_change_pm(state, 0);
		uport->ops->set_termios(uport, &termios, NULL);
		console_start(uport->cons);
	}
#else
	uart_change_pm(state, 0);

	/*
	 * Re-enable the console device after suspending.
	 */
	if (uart_console(port)) {
		uart_change_speed(state, NULL);
		port->cons->flags |= CON_ENABLED;
	}
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	if (port->flags & ASYNC_SUSPENDED) {
		const struct uart_ops *ops = uport->ops;
		int ret;

		uart_change_pm(state, 0);
		spin_lock_irq(&uport->lock);
		ops->set_mctrl(uport, 0);
		spin_unlock_irq(&uport->lock);
		ret = ops->startup(uport);
		if (ret == 0) {
			uart_change_speed(state, NULL);
			spin_lock_irq(&uport->lock);
			ops->set_mctrl(uport, uport->mctrl);
			ops->start_tx(uport);
			spin_unlock_irq(&uport->lock);
			set_bit(ASYNCB_INITIALIZED, &port->flags);
		} else {
			/*
			 * Failed to resume - maybe hardware went away?
			 * Clear the "initialized" flag so we won't try
			 * to call the low level drivers shutdown method.
			 */
			uart_shutdown(state);
		}

		clear_bit(ASYNCB_SUSPENDED, &port->flags);
	}
#else
 #if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 28)
	if (state->info.flags & UIF_INITIALIZED) {
 #else
	if (state->info && state->info->flags & UIF_INITIALIZED) {
 #endif
//lipeng modify at 06/08/2006
 #if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 15) || UBUNTU_2_6_15 
		struct uart_ops *ops = port->ops;
 #else
		const struct uart_ops *ops = port->ops;
 #endif
//lipeng modify end
		ops->set_mctrl(port, 0);
		ops->startup(port);
		uart_change_speed(state, NULL);
		spin_lock_irq(&port->lock);
		ops->set_mctrl(port, port->mctrl);
//lipeng modify at 06/08/2006
 #if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 15)	
	ops->start_tx(port, 0);
 #else
	ops->start_tx(port);
 #endif
//lipeng modify end
		spin_unlock_irq(&port->lock);
	}
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	mutex_unlock(&port->mutex);
//lipeng modify at 06/08/2006
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 15) || UBUNTU_2_6_15 
	up(&state->sem);
#else
	mutex_unlock(&state->mutex);
#endif
//lipeng modify end
	return 0;
}

static inline void
uart_report_port(struct uart_driver *drv, struct uart_port *port)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	char address[64];
#endif
	printk("%s%d", drv->dev_name, port->line);
	printk(" at ");
	switch (port->iotype) {
	case UPIO_PORT:
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
		snprintf(address, sizeof(address), "I/O 0x%lx", port->iobase);
#else
		printk("I/O 0x%x", port->iobase);
#endif
		break;
	case UPIO_HUB6:
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
		snprintf(address, sizeof(address),
			 "I/O 0x%lx offset 0x%x", port->iobase, port->hub6);
#else
		printk("I/O 0x%x offset 0x%x", port->iobase, port->hub6);
#endif
		break;
	case UPIO_MEM:
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	case UPIO_MEM32:
	case UPIO_AU:
	case UPIO_TSI:
	case UPIO_DWAPB:
#endif
		printk("MMIO 0x%lx", (long unsigned int)port->mapbase);
		break;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	default:
		strlcpy(address, "*unknown*", sizeof(address));
		break;
#endif
	}
	printk(" (irq = %d) is a %s\n", port->irq, uart_type(port));
}

static void
uart_configure_port(struct uart_driver *drv, struct uart_state *state,
		    struct uart_port *port)
{
	unsigned int flags;

	/*
	 * If there isn't a port here, don't do anything further.
	 */
	if (!port->iobase && !port->mapbase && !port->membase)
		return;

	/*
	 * Now do the auto configuration stuff.  Note that config_port
	 * is expected to claim the resources and map the port for us.
	 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	flags = 0;
#else
	flags = UART_CONFIG_TYPE;
#endif
	if (port->flags & UPF_AUTO_IRQ)
		flags |= UART_CONFIG_IRQ;
	if (port->flags & UPF_BOOT_AUTOCONF) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
		if (!(port->flags & UPF_FIXED_TYPE)) {
#endif
		port->type = PORT_UNKNOWN;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
		flags |= UART_CONFIG_TYPE;
		}
#endif
		port->ops->config_port(port, flags);
	}

	if (port->type != PORT_UNKNOWN) {
		unsigned long flags;

		uart_report_port(drv, port);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
		/* Power up port for set_mctrl() */
		uart_change_pm(state, 0);
#endif
		/*
		 * Ensure that the modem control lines are de-activated.
		 * We probably don't need a spinlock around this, but
		 */
		spin_lock_irqsave(&port->lock, flags);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
		port->ops->set_mctrl(port, port->mctrl & TIOCM_DTR);
#else
		port->ops->set_mctrl(port, 0);
#endif
		spin_unlock_irqrestore(&port->lock, flags);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
		/*
		 * If this driver supports console, and it hasn't been
		 * successfully registered yet, try to re-register it.
		 * It may be that the port was not available.
		 */
		if (port->cons && !(port->cons->flags & CON_ENABLED))
			register_console(port->cons);
#endif
		/*
		 * Power down all ports by default, except the
		 * console if we have one.
		 */
		if (!uart_console(port))
			uart_change_pm(state, 3);
	}
}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
#ifdef CONFIG_CONSOLE_POLL

static int uart_poll_init(struct tty_driver *driver, int line, char *options)
{
	struct uart_driver *drv = driver->driver_state;
	struct uart_state *state = drv->state + line;
	struct uart_port *port;
	int baud = 9600;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	if (!state || !state->uart_port)
		return -1;

	port = state->uart_port;
	if (!(port->ops->poll_get_char && port->ops->poll_put_char))
		return -1;

	if (options) {
		uart_parse_options(options, &baud, &parity, &bits, &flow);
		return uart_set_options(port, NULL, baud, parity, bits, flow);
	}

	return 0;
}

static int uart_poll_get_char(struct tty_driver *driver, int line)
{
	struct uart_driver *drv = driver->driver_state;
	struct uart_state *state = drv->state + line;
	struct uart_port *port;

	if (!state || !state->uart_port)
		return -1;

	port = state->uart_port;
	return port->ops->poll_get_char(port);
}

static void uart_poll_put_char(struct tty_driver *driver, int line, char ch)
{
	struct uart_driver *drv = driver->driver_state;
	struct uart_state *state = drv->state + line;
	struct uart_port *port;

	if (!state || !state->uart_port)
		return;

	port = state->uart_port;
	port->ops->poll_put_char(port, ch);
}
#endif
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 32)
/*
 * This reverses the effects of uart_configure_port, hanging up the
 * port before removal.
 */
static void
uart_unconfigure_port(struct uart_driver *drv, struct uart_state *state)
{
	struct uart_port *port = state->port;
 #if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 28)
	struct uart_info *info = &state->info;
 #else
	struct uart_info *info = state->info;
 #endif
 #if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)
	if (info && info->port.tty)
		tty_vhangup(info->port.tty);
 #else
	if (info && info->tty)
		tty_vhangup(info->tty);
 #endif
//lipeng modify at 06/08/2006
 #if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 15) || UBUNTU_2_6_15
       down(&state->sem);
 #else
	mutex_lock(&state->mutex);
 #endif
//lipeng modify end
 #if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 27)
	state->info = NULL;
 #endif

	/*
	 * Free the port IO and memory resources, if any.
	 */
	if (port->type != PORT_UNKNOWN)
		port->ops->release_port(port);

	/*
	 * Indicate that there isn't a port here anymore.
	 */
	port->type = PORT_UNKNOWN;

	/*
	 * Kill the tasklet, and free resources.
	 */
	if (info) {
		tasklet_kill(&info->tlet);
		//kfree(info);
	}
//lipeng modify at 06/08/2006
 #if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 15) || UBUNTU_2_6_15
	up(&state->sem);
 #else
	mutex_unlock(&state->mutex);
 #endif
//lipeng modify end
}
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
static const struct tty_operations uart_ops = {
#else
static struct tty_operations uart_ops = {
#endif
	.open		= uart_open,
	.close		= uart_close,
	.write		= uart_write,
	.put_char	= uart_put_char,
	.flush_chars	= uart_flush_chars,
	.write_room	= uart_write_room,
	.chars_in_buffer= uart_chars_in_buffer,
	.flush_buffer	= uart_flush_buffer,
	.ioctl		= uart_ioctl,
	.throttle	= uart_throttle,
	.unthrottle	= uart_unthrottle,
	.send_xchar	= uart_send_xchar,
	.set_termios	= uart_set_termios,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	.set_ldisc	= uart_set_ldisc,
#endif
	.stop		= uart_stop,
	.start		= uart_start,
	.hangup		= uart_hangup,
	.break_ctl	= uart_break_ctl,
	.wait_until_sent= uart_wait_until_sent,
#ifdef CONFIG_PROC_FS
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 31) 
	.read_proc	= uart_read_proc,
#else
	.proc_fops	= &uart_proc_fops,
#endif
#endif
	.tiocmget	= uart_tiocmget,
	.tiocmset	= uart_tiocmset,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
#ifdef CONFIG_CONSOLE_POLL
	.poll_init	= uart_poll_init,
	.poll_get_char	= uart_poll_get_char,
	.poll_put_char	= uart_poll_put_char,
#endif
#endif
};

/**
 *	uart_register_driver - register a driver with the uart core layer
 *	@drv: low level driver structure
 *
 *	Register a uart driver with the core driver.  We in turn register
 *	with the tty layer, and initialise the core driver per-port state.
 *
 *	We have a proc file in /proc/tty/driver which is named after the
 *	normal driver.
 *
 *	drv->port should be NULL, and the per-port structures should be
 *	registered using uart_add_one_port after this call has succeeded.
 */
int adv_uart_register_driver(struct uart_driver *drv)
{
	struct tty_driver *normal = NULL;
	int i, retval;

	BUG_ON(drv->state);

	/*
	 * Maybe we should be using a slab cache for this, especially if
	 * we have a large number of ports to handle.
	 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	drv->state = kzalloc(sizeof(struct uart_state) * drv->nr, GFP_KERNEL);
#else
	drv->state = kmalloc(sizeof(struct uart_state) * drv->nr, GFP_KERNEL);
#endif
	retval = -ENOMEM;
	if (!drv->state)
		goto out;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 32)
	memset(drv->state, 0, sizeof(struct uart_state) * drv->nr);
#endif
	normal  = alloc_tty_driver(drv->nr);
	if (!normal)
		goto out;

	drv->tty_driver = normal;

	normal->owner		= drv->owner;
	normal->driver_name	= drv->driver_name;
	normal->name		= drv->dev_name;
	normal->major		= drv->major;
	normal->minor_start	= drv->minor;
	normal->type		= TTY_DRIVER_TYPE_SERIAL;
	normal->subtype		= SERIAL_TYPE_NORMAL;
	normal->init_termios	= tty_std_termios;
	normal->init_termios.c_cflag = B9600 | CS8 | CREAD | HUPCL | CLOCAL;
	#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	normal->init_termios.c_ispeed = normal->init_termios.c_ospeed = 9600;
	normal->flags		= TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;
	#elif LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 18)
	normal->flags		= TTY_DRIVER_REAL_RAW ;//| TTY_DRIVER_NO_DEVFS;
	#else
	normal->flags		= TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;
	#endif
	normal->driver_state    = drv;
	tty_set_operations(normal, &uart_ops);

	/*
	 * Initialise the UART state(s).
	 */
	for (i = 0; i < drv->nr; i++) {
		struct uart_state *state = drv->state + i;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
		struct tty_port *port = &state->port;

		tty_port_init(port);
		port->close_delay     = 500;	/* .5 seconds */
		port->closing_wait    = 30000;	/* 30 seconds */
		tasklet_init(&state->tlet, uart_tasklet_action,
			     (unsigned long)state);
#else
		state->close_delay     = 5 * HZ / 10;
		state->closing_wait    = 30 * HZ;
//lipeng modify at 06/08/2006
 #if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 15) || UBUNTU_2_6_15
      init_MUTEX(&state->sem); 
 #else
	mutex_init(&state->mutex);
 #endif
//lipeng modify end
//jianfeng added begin
 #if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 28)
		tty_port_init(&state->info.port);
		init_waitqueue_head(&state->info.delta_msr_wait);
		tasklet_init(&state->info.tlet, uart_tasklet_action,
			     (unsigned long)state);
 #endif
#endif	
//jinxin added end
	}

	retval = tty_register_driver(normal);
 out:
	if (retval < 0) {
		put_tty_driver(normal);
		kfree(drv->state);
	}
	return retval;
}

/**
 *	uart_unregister_driver - remove a driver from the uart core layer
 *	@drv: low level driver structure
 *
 *	Remove all references to a driver from the core driver.  The low
 *	level driver must have removed all its ports via the
 *	uart_remove_one_port() if it registered them with uart_add_one_port().
 *	(ie, drv->port == NULL)
 */
void adv_uart_unregister_driver(struct uart_driver *drv)
{
	struct tty_driver *p = drv->tty_driver;
	tty_unregister_driver(p);
	put_tty_driver(p);
	kfree(drv->state);
	drv->tty_driver = NULL;
}

struct tty_driver *uart_console_device(struct console *co, int *index)
{
	struct uart_driver *p = co->data;
	*index = co->index;
	return p->tty_driver;
}

/**
 *	uart_add_one_port - attach a driver-defined port structure
 *	@drv: pointer to the uart low level driver structure for this port
 *	@port: uart port structure to use for this port.
 *
 *	This allows the driver to register its own uart_port structure
 *	with the core driver.  The main purpose is to allow the low
 *	level uart drivers to expand uart_port, rather than having yet
 *	more levels of structures.
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
int adv_uart_add_one_port(struct uart_driver *drv, struct uart_port *uport)
#else
int adv_uart_add_one_port(struct uart_driver *drv, struct uart_port *port)
#endif
{
	struct uart_state *state;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	struct tty_port *port;
	struct device *tty_dev;
#endif
	int ret = 0;

	BUG_ON(in_interrupt());
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	if (uport->line >= drv->nr)
#else
	if (port->line >= drv->nr)
#endif
		return -EINVAL;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	state = drv->state + uport->line;
	port = &state->port;
#else
	state = drv->state + port->line;
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	mutex_lock(&port_mutex);
	mutex_lock(&port->mutex);
//lipeng modify at 06/08/2006
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 15) || UBUNTU_2_6_15
      	down(&port_sem); 
#else
	mutex_lock(&port_mutex);
#endif
//lipeng modify end
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	if (state->uart_port) {
#else
	if (state->port) {
#endif
		ret = -EINVAL;
		goto out;
	}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	state->uart_port = uport;
	state->pm_state = -1;

	uport->cons = drv->cons;
	uport->state = state;

	/*
	 * If this port is a console, then the spinlock is already
	 * initialised.
	 */
	if (!(uart_console(uport) && (uport->cons->flags & CON_ENABLED))) {
		spin_lock_init(&uport->lock);
		lockdep_set_class(&uport->lock, &port_lock_key);
	}

	uart_configure_port(drv, state, uport);

	/*
	 * Register the port whether it's detected or not.  This allows
	 * setserial to be used to alter this ports parameters.
	 */
	tty_dev = tty_register_device(drv->tty_driver, uport->line, uport->dev);
	if (likely(!IS_ERR(tty_dev))) {
		device_init_wakeup(tty_dev, 1);
		device_set_wakeup_enable(tty_dev, 0);
	} else
		printk(KERN_ERR "Cannot register tty device on line %d\n",
		       uport->line);

	/*
	 * Ensure UPF_DEAD is not set.
	 */
	uport->flags &= ~UPF_DEAD;
#else
	state->port = port;

	spin_lock_init(&port->lock);
	port->cons = drv->cons;
 #if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 28)
	port->info = &state->info;
 #else
	port->info = state->info;
 #endif

	uart_configure_port(drv, state, port);

	/*
	 * Register the port whether it's detected or not.  This allows
	 * setserial to be used to alter this ports parameters.
	 */
	tty_register_device(drv->tty_driver, port->line, port->dev);
#endif
 out:
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	mutex_unlock(&port->mutex);
	mutex_unlock(&port_mutex);
//lipeng modify at 06/08/2006
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 15) || UBUNTU_2_6_15
	up(&port_sem); 
#else
	mutex_unlock(&port_mutex);
#endif
//lipeng modify end

	return ret;
}

/**
 *	uart_remove_one_port - detach a driver defined port structure
 *	@drv: pointer to the uart low level driver structure for this port
 *	@port: uart port structure for this port
 *
 *	This unhooks (and hangs up) the specified port structure from the
 *	core driver.  No further calls will be made to the low-level code
 *	for this port.
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
int adv_uart_remove_one_port(struct uart_driver *drv, struct uart_port *uport)
#else
int adv_uart_remove_one_port(struct uart_driver *drv, struct uart_port *port)
#endif
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	struct uart_state *state = drv->state + uport->line;
	struct tty_port *port = &state->port;
#else
	struct uart_state *state = drv->state + port->line;
#endif
	BUG_ON(in_interrupt());
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	if (state->uart_port != uport)
		printk(KERN_ALERT "Removing wrong port: %p != %p\n",
			state->uart_port, uport);
#else
	if (state->port != port)
		printk(KERN_ALERT "Removing wrong port: %p != %p\n",
			state->port, port);
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	mutex_lock(&port_mutex);
//lipeng modify at 06/08/2006
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 15) || UBUNTU_2_6_15
      	down(&port_sem); 
#else
	mutex_lock(&port_mutex);
#endif
//lipeng modify end
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	uport->flags |= UPF_DEAD;
	mutex_unlock(&port->mutex);

	/*
	 * Remove the devices from the tty layer
	 */
	tty_unregister_device(drv->tty_driver, uport->line);
#else
	/*
	 * Remove the devices from devfs
	 */
	tty_unregister_device(drv->tty_driver, port->line);
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	if (port->tty)
		tty_vhangup(port->tty);

	/*
	 * Free the port IO and memory resources, if any.
	 */
	if (uport->type != PORT_UNKNOWN)
		uport->ops->release_port(uport);

	/*
	 * Indicate that there isn't a port here anymore.
	 */
	uport->type = PORT_UNKNOWN;

	/*
	 * Kill the tasklet, and free resources.
	 */
	tasklet_kill(&state->tlet);

	state->uart_port = NULL;
#else
	uart_unconfigure_port(drv, state);
	state->port = NULL;
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	mutex_unlock(&port_mutex);
 //lipeng modify at 06/08/2006
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 15) || UBUNTU_2_6_15
	up(&port_sem);
#else
	mutex_unlock(&port_mutex);
#endif
//lipeng modify end

	return 0;
}

/*
 *	Are the two ports equivalent?
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 10) // Po-Cheng Chen add, 02/15/2006
static int uart_match_port(struct uart_port *port1, struct uart_port *port2)
#else // Po-Cheng Chen add, 02/15/2006
int uart_match_port(struct uart_port *port1, struct uart_port *port2)
#endif
{
	if (port1->iotype != port2->iotype)
		return 0;

	switch (port1->iotype) {
	case UPIO_PORT:
		return (port1->iobase == port2->iobase);
	case UPIO_HUB6:
		return (port1->iobase == port2->iobase) &&
		       (port1->hub6   == port2->hub6);
	case UPIO_MEM:
//#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
//	case UPIO_MEM32:
//	case UPIO_AU:
//	case UPIO_TSI:
//	case UPIO_DWAPB:
//#endif
		return (port1->membase == port2->membase);
	}
	return 0;
}
//#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
//EXPORT_SYMBOL(uart_match_port);
//#endif
/*
 *	Try to find an unused uart_state slot for a port.
 */
//#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 32)
static struct uart_state *
uart_find_match_or_unused(struct uart_driver *drv, struct uart_port *port)
{
	int i;

	/*
	 * First, find a port entry which matches.  Note: if we do
	 * find a matching entry, and it has a non-zero use count,
	 * then we can't register the port.
	 */
	for (i = 0; i < drv->nr; i++)
	{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
		if (uart_match_port(drv->state[i].uart_port, port))
#else
		if (uart_match_port(drv->state[i].port, port))
#endif
			return &drv->state[i];
	}

	/*
	 * We didn't find a matching entry, so look for the first
	 * free entry.  We look for one which hasn't been previously
	 * used (indicated by zero iobase).
	 */
	for (i = 0; i < drv->nr; i++)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
		if (drv->state[i].uart_port->type == PORT_UNKNOWN &&
		    drv->state[i].uart_port->iobase == 0 &&
		    drv->state[i].port.count == 0)
#else
		if (drv->state[i].port->type == PORT_UNKNOWN &&
		    drv->state[i].port->iobase == 0 &&
		    drv->state[i].count == 0)
#endif
			return &drv->state[i];

	/*
	 * That also failed.  Last resort is to find any currently
	 * entry which doesn't have a real port associated with it.
	 */
	for (i = 0; i < drv->nr; i++)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
		if (drv->state[i].uart_port->type == PORT_UNKNOWN &&
		    drv->state[i].port.count == 0)
#else
		if (drv->state[i].port->type == PORT_UNKNOWN &&
		    drv->state[i].count == 0)
#endif
			return &drv->state[i];

	return NULL;
}
//#endif
/**
 *	uart_register_port: register uart settings with a port
 *	@drv: pointer to the uart low level driver structure for this port
 *	@port: uart port structure describing the port
 *
 *	Register UART settings with the specified low level driver.  Detect
 *	the type of the port if UPF_BOOT_AUTOCONF is set, and detect the
 *	IRQ if UPF_AUTO_IRQ is set.
 *
 *	We try to pick the same port for the same IO base address, so that
 *	when a modem is plugged in, unplugged and plugged back in, it gets
 *	allocated the same port.
 *
 *	Returns negative error, or positive line number.
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
int adv_uart_register_port(struct uart_driver *drv, struct uart_port *uport)
#else
int adv_uart_register_port(struct uart_driver *drv, struct uart_port *port)
#endif
{
	struct uart_state *state;
	int ret;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	struct tty_port *port;
	state = uart_find_match_or_unused(drv, uport);
	port = &state->port;
#else
	state = uart_find_match_or_unused(drv, port);
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	mutex_lock(&port_mutex);
	mutex_lock(&port->mutex);
//lipeng modify at 06/08/2006
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 15) || UBUNTU_2_6_15
      	down(&port_sem); 
#else
	mutex_lock(&port_mutex);
#endif
//lipeng modify end
	if (state) {
		/*
		 * Ok, we've found a line that we can use.
		 *
		 * If we find a port that matches this one, and it appears
		 * to be in-use (even if it doesn't have a type) we shouldn't
		 * alter it underneath itself - the port may be open and
		 * trying to do useful work.
		 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
		if (state->port.count + state->port.blocked_open != 0) {
#else
		if (uart_users(state) != 0) {
#endif
			ret = -EBUSY;
			goto out;
		}
		/*
		 * If the port is already initialised, don't touch it.
		 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
		if (state->uart_port->type == PORT_UNKNOWN) {
			state->uart_port->iobase   = uport->iobase;
			state->uart_port->membase  = uport->membase;
			state->uart_port->irq      = uport->irq;
			state->uart_port->uartclk  = uport->uartclk;
			state->uart_port->fifosize = uport->fifosize;
			state->uart_port->regshift = uport->regshift;
			state->uart_port->iotype   = uport->iotype;
			state->uart_port->flags    = uport->flags;
			state->uart_port->line     = state - drv->state;
			state->uart_port->mapbase  = uport->mapbase;
			
			// Default ACR value for auto DTR RS485 or RS232
			state->uart_port->unused[0] = uport->unused[0];
			//for PCIe958 or DMA
			state->uart_port->unused[1] = uport->unused[1];			

			uart_configure_port(drv, state, state->uart_port);
		}

		ret = state->uart_port->line;
#else
		if (state->port->type == PORT_UNKNOWN) {
			state->port->iobase   = port->iobase;
			state->port->membase  = port->membase;
			state->port->irq      = port->irq;
			state->port->uartclk  = port->uartclk;
			state->port->fifosize = port->fifosize;
			state->port->regshift = port->regshift;
			state->port->iotype   = port->iotype;
			state->port->flags    = port->flags;
			state->port->line     = state - drv->state;
			state->port->mapbase  = port->mapbase;
			
			// Default ACR value for auto DTR RS485 or RS232
			state->port->unused[0] = port->unused[0];
			//for PCIe958 or DMA
			state->port->unused[1] = port->unused[1];

			uart_configure_port(drv, state, state->port);
		}

		ret = state->port->line;
#endif
	} else
		ret = -ENOSPC;
 out:
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	mutex_unlock(&port->mutex);
	mutex_unlock(&port_mutex);
 //lipeng modify at 06/08/2006
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 15) || UBUNTU_2_6_15
	up(&port_sem);
#else
	mutex_unlock(&port_mutex);
#endif
//lipeng modify end
	return ret;
}
//#endif
/**
 *	uart_unregister_port - de-allocate a port
 *	@drv: pointer to the uart low level driver structure for this port
 *	@line: line index previously returned from uart_register_port()
 *
 *	Hang up the specified line associated with the low level driver,
 *	and mark the port as unused.
 */
//#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 32)
void adv_uart_unregister_port(struct uart_driver *drv, int line)
{
	struct uart_state *state;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	struct tty_port *port;
#endif
	if (line < 0 || line >= drv->nr) {
		printk(KERN_ERR "Attempt to unregister ");
		printk("%s%d", drv->dev_name, line);
		printk("\n");
		return;
	}

	state = drv->state + line;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	port= &state->port;
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	mutex_lock(&port_mutex);
	mutex_lock(&port->mutex);
//lipeng modify at 06/08/2006
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 15) || UBUNTU_2_6_15
      	down(&port_sem); 
#else
	mutex_lock(&port_mutex);
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	if (port->tty)
		tty_vhangup(port->tty);

	/*
	 * Free the port IO and memory resources, if any.
	 */
	if (state->uart_port->type != PORT_UNKNOWN)
		state->uart_port->ops->release_port(state->uart_port);

	/*
	 * Indicate that there isn't a port here anymore.
	 */
	state->uart_port->type = PORT_UNKNOWN;

	/*
	 * Kill the tasklet, and free resources.
	 */
	tasklet_kill(&state->tlet);
#else
//lipeng modify end
	uart_unconfigure_port(drv, state);
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	mutex_unlock(&port->mutex);
	mutex_unlock(&port_mutex);
 //lipeng modify at 06/08/2006
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 15) || UBUNTU_2_6_15
	up(&port_sem);
#else
	mutex_unlock(&port_mutex);
#endif
//lipeng modify end
}
//#endif
EXPORT_SYMBOL(adv_uart_write_wakeup);
EXPORT_SYMBOL(adv_uart_register_driver);
EXPORT_SYMBOL(adv_uart_unregister_driver);
EXPORT_SYMBOL(adv_uart_suspend_port);
EXPORT_SYMBOL(adv_uart_resume_port);
EXPORT_SYMBOL(adv_uart_register_port);
EXPORT_SYMBOL(adv_uart_unregister_port);
EXPORT_SYMBOL(adv_uart_add_one_port);
EXPORT_SYMBOL(adv_uart_remove_one_port);

MODULE_DESCRIPTION("Advantech serial driver core");
MODULE_LICENSE("GPL");
