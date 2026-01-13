#ifndef UART_DRIVER_H
#define UART_DRIVER_H

#include <linux/kernel.h> // Core kernel functions (pr_info, pr_err)
#include <linux/module.h> // Module macros (module_init, module_exit)
#include <linux/proc_fs.h> // /proc filesystem interface
#include <linux/uaccess.h> // User-space data transfer (copy_from_user)
#include <linux/io.h>  //  Memory-mapped I/O (ioremap, readl, writel)
#include <linux/delay.h> // Timing functions (udelay, cpu_relax)

#define PROC_UART_TX "uart_tx"
#define PROC_UART_RX "uart_rx" //new chnage
// Base addresses for BCM2711 
#define PERIPHERAL_BASE 0xFE000000UL
#define AUX_BASE        (PERIPHERAL_BASE + 0x215000)
#define GPIO_BASE       (PERIPHERAL_BASE + 0x200000)

// GPIO Function Select values 
#define GPIO_FSEL_INPUT  0x0
#define GPIO_FSEL_OUTPUT 0x1
#define GPIO_FSEL_ALT0   0x4
#define GPIO_FSEL_ALT1   0x5
#define GPIO_FSEL_ALT2   0x6
#define GPIO_FSEL_ALT3   0x7
#define GPIO_FSEL_ALT4   0x3
#define GPIO_FSEL_ALT5   0x2

// GPIO Pull-up/down values 
#define GPIO_PUPDN_NONE  0x0
#define GPIO_PUPDN_UP    0x1
#define GPIO_PUPDN_DOWN  0x2

// Mini UART register structure - matching your bare metal layout 
struct uart_regs {
    volatile u32 IRQ;           /* 0x00 */
    volatile u32 ENABLES;       /* 0x04 */
    volatile u32 RESERVED0[14]; /* 0x08-0x3C */
    volatile u32 MU_IO;         /* 0x40 */
    volatile u32 MU_IER;        /* 0x44 */
    volatile u32 MU_IIR;        /* 0x48 */
    volatile u32 MU_LCR;        /* 0x4C */
    volatile u32 MU_MCR;        /* 0x50 */
    volatile u32 MU_LSR;        /* 0x54 */
    volatile u32 MU_MSR;        /* 0x58 */
    volatile u32 MU_SCRATCH;    /* 0x5C */
    volatile u32 MU_CNTL;       /* 0x60 */
    volatile u32 MU_STAT;       /* 0x64 */
    volatile u32 MU_BAUD;       /* 0x68 */
};

// GPIO register offsets 
#define GPFSEL1    0x04  // GPIO Function Select 1 
#define GPPUD      0x94  /* GPIO Pin Pull-up/down Enable */
#define GPPUDCLK0  0x98  /* GPIO Pin Pull-up/down Enable Clock 0 */
#define GPPUPPDN0  0xE4  /* GPIO Pull-up/down for pins 0-15 (BCM2711) */

#endif
