#include "uart.h"


static struct uart_regs __iomem *uart = NULL;
static void __iomem *gpio = NULL;
static struct proc_dir_entry *proc_tx;
static struct proc_dir_entry *proc_rx;  // new 
// Delay function for GPIO setup 
static void delay_cycles(int count)
{
    while (count--) {
        cpu_relax();  //Linux function that tells the CPU "take a tiny break"

    }
}

// 
// static int get_baud_register(u32 baudrate, u16 *reg_value)
// {
//     u32 system_clock = 500000000; // 500MHz for RPi4 
//     u32 denominator;
//     u32 res;
    
//     if (baudrate == 0 || baudrate > (system_clock / 8))
//         return -1;
    
//     denominator = 8 * baudrate;
//     res = (system_clock / denominator) - 1;
    
//     if (res > 0xFFFF)
//         return -1;
    
//     *reg_value = (u16)res;
//     return 0;
// }

// Initialize Mini UART - following your bare metal sequence 
static void uart_init_os(void)
{
    u32 val;
   // u16 baud_reg_value;
    void __iomem *gpfsel1;  // Pointer to GPIO Function Select register
    void __iomem *gppuppdn0;  // Pointer to GPIO Pull-up/down register
    
    // Configure GPIO14 and GPIO15 for Mini UART (ALT5) 
   gpfsel1 = gpio + GPFSEL1;
val = readl(gpfsel1);
val &= ~((7 << 12) | (7 << 15));  // Clear both GPIO14 and GPIO15 bits
val |= (GPIO_FSEL_ALT5 << 12) | (GPIO_FSEL_ALT5 << 15);  // Set both to ALT5
writel(val, gpfsel1);

    
   gppuppdn0 = gpio + GPPUPPDN0;
val = readl(gppuppdn0);
val &= ~((0x3 << 28) | (0x3 << 30));  // Clear both GPIO14 and GPIO15 bits 
val |= (GPIO_PUPDN_NONE << 28) | (GPIO_PUPDN_UP << 30);  // Set both values 
writel(val, gppuppdn0);

    
    //Wait for GPIO configuration to settle 
    delay_cycles(150);
    
    // Enable Mini UART in AUX enables register 
    val = readl(&uart->ENABLES);
    writel(val | 0x1, &uart->ENABLES);
    
    // Disable TX/RX during configuration
    writel(0x0, &uart->MU_CNTL);
    
    //  Disable interrupts 
    writel(0x0, &uart->MU_IER);
    
    // Clear FIFOs
   // Clear RX FIFO
    writel(0x02, &uart->MU_IIR);  // Bit 1 set (01 in bits 2:1)

    // Clear TX FIFO  
    writel(0x04, &uart->MU_IIR);  // Bit 2 set (10 in bits 2:1)

    
    // Set data format to 8-bit mode 
   writel(0x3, &uart->MU_LCR);  // 8-bit mode 
    
    // Disable 
    writel(0x0, &uart->MU_MCR);
    
    // baud rate 115200 
    // if (get_baud_register(115200, &baud_reg_value) == 0) {
    //     writel(baud_reg_value, &uart->MU_BAUD);
    // } 

u16 baud_val = (500000000/(9600*8))-1;  
writel(baud_val, &uart->MU_BAUD);
    


    
    // Enable TX and RX 
    writel(0x3, &uart->MU_CNTL);  // Enable TX and RX 
    
    // Memory barrier to ensure all writes complete 
    wmb();
    
    pr_info("Mini UART initialized successfully\n");
}

// Send a single char blcoking
static void uart_send_char(char c)
{
    // Handle newline 
    if (c == '\n') {
        uart_send_char('\r');
    }
    
    // Wait until TX FIFO has space 
    while (!(readl(&uart->MU_LSR) & (1 << 5))) {
        /* spin wait */
    }
    
    // Write character to TX FIFO (only lower 8 bits) 
    writel((u32)(c & 0xFF), &uart->MU_IO);
}

// Send a string
static void uart_send_string(const char *s)
{
    while (*s) {
        if (*s == '\n') {
            uart_send_char('\r');  //  carriage return before newline 
        }
        uart_send_char(*s++);
    }
}

//new chnage
// Add these functions after uart_send_string()

// Check if data is available to receive
static int uart_data_available(void)
{
    return (readl(&uart->MU_LSR) & (1 << 0));  // Bit 0 = data ready
}

// Receive a single character (non-blocking)
static char uart_receive_char(void)
{
    if (!uart_data_available()) {
        return 0;  // No data available
    }
    
    return (char)(readl(&uart->MU_IO) & 0xFF);
}

// Proc file read handler for receiving data
static ssize_t uart_proc_read(struct file *file, char __user *buf, 
                              size_t count, loff_t *ppos)
{
    char kbuf[256];
    int i = 0;
    char c;
    
    // Read available characters (up to buffer size)
    while (i < (sizeof(kbuf) - 1) && i < count && uart_data_available()) {
        c = uart_receive_char();
        if (c != 0) {
            kbuf[i++] = c;
        }
    }
    
    if (i == 0) {
        return 0;  // No data available
    }
    
    kbuf[i] = '\0';
    
    // Copy to user space
    if (copy_to_user(buf, kbuf, i)) {
        return -EFAULT;
    }
    
    pr_info("UART RX: received %d bytes\n", i);
    return i;
}


// Proc file write handler 
static ssize_t uart_proc_write(struct file *file,
    const char __user *buf, size_t count, loff_t *ppos)
{
    char kbuf[256];
    size_t len;
    
    // Limit the size to prevent buffer overflow 
    len = min(count, sizeof(kbuf) - 1);
    
    if (copy_from_user(kbuf, buf, len)) //copy_from_user() safely copies data from user space to kernel space 

        return -EFAULT;
    
    kbuf[len] = '\0';
    
    // Send the string via UART 
    uart_send_string(kbuf);
    
    pr_info("UART TX: sent %zu bytes\n", len);
    
    return count;
}

// When someone writes to  /proc file, call the uart_proc_write function
static const struct proc_ops uart_proc_ops = {
    .proc_write = uart_proc_write,   
     .proc_read = uart_proc_read,    // new chnage
};

// Module initialization 
static int __init uart_driver_init(void)  //__init : Special Linux keyword meaning this function runs only once when module loads

{
    // Map GPIO registers 
    gpio = ioremap(GPIO_BASE, 0x1000); //ioremap() creates a "virtual window" to access gpio hardware, Linux doesn't let you directly access physical addresses
    //now the gpio points to the base address and the GPFSEL1 is the offset from the base
    if (!gpio) {
        pr_err("Failed to map GPIO registers\n");
        return -ENOMEM;
    }
    
    // Map UART registers 
    uart = ioremap(AUX_BASE, sizeof(struct uart_regs));
    if (!uart) {
        pr_err("Failed to map UART registers\n");
        iounmap(gpio);  //If UART mapping fails, release the GPIO mapping
        return -ENOMEM;
    }
    
    // Initialize the Mini UART 
    uart_init_os();
    
    // Creates /proc/uart_tx file, so users can write to this file to send UART data

    proc_tx = proc_create(PROC_UART_TX, 0666, NULL, &uart_proc_ops); 
    if (!proc_tx) {
        pr_err("Failed to create /proc/%s\n", PROC_UART_TX);
        iounmap(uart);
        iounmap(gpio);
        return -ENOMEM;
    }
        //new chnage
        proc_rx = proc_create(PROC_UART_RX, 0666, NULL, &uart_proc_ops);
        if (!proc_rx) {
            pr_err("Failed to create /proc/%s\n", PROC_UART_RX);
            proc_remove(proc_tx);
            iounmap(uart);
            iounmap(gpio);
            return -ENOMEM;
        }
    
    // Send a test message 
    uart_send_string("Mini UART driver loaded successfully!\r\n");
    
    pr_info("UART TX driver loaded. Write to /proc/%s to send data.\n", PROC_UART_TX);
    return 0;
}

// Module cleanup - runs when we do sudo rmmod uart_driver
static void __exit uart_driver_exit(void)  //Special Linux keyword meaning  function runs only when module is unloaded

{
    
    uart_send_string("Mini UART driver unloading...\r\n");
    
    // Remove proc entry -Remove /proc/uart_tx- Users can't access it anymore

    proc_remove(proc_tx); //Removes /proc/uart_tx file from the filesystem- so that we cannot write to the file anymore

    proc_remove(proc_rx); // new chnage
    // Unmap registers 
    if (uart) //if UART was successfully mapped- then unmap it
        iounmap(uart);
    if (gpio)
        iounmap(gpio);
    
    pr_info("UART TX driver unloaded.\n");
}

module_init(uart_driver_init);  //when someone does insmod uart.ko, call uart_driver_init()

module_exit(uart_driver_exit);  //when someone does rmmod uart, call uart_driver_exit()

MODULE_AUTHOR("Supriya Mishra");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("BCM2711 Mini UART Driver");
