#include "uart.h"


static struct uart_regs __iomem *uart = NULL;
static void __iomem *gpio = NULL;
static struct proc_dir_entry *proc_tx;
static struct proc_dir_entry *proc_rx;

// Delay function for GPIO setup 
static void delay_cycles(int count)
{
    while (count--) {
        cpu_relax();  //Linux function that tells the CPU "take a tiny break"
    }
}

// Initialize Mini UART - following your bare metal sequence 
static void uart_init_os(void)
{
    u32 val;
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
    
    // baud rate 9600 
    u16 baud_val = (500000000/(9600*8))-1;  
    writel(baud_val, &uart->MU_BAUD);
    
    // Enable TX and RX 
    writel(0x3, &uart->MU_CNTL);  // Enable TX and RX 
    
    // Memory barrier to ensure all writes complete 
    wmb();
    
    pr_info("Mini UART initialized successfully\n");
}

// Send a single char blocking
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
    int consecutive_no_data = 0;
    // At 9600 baud, each character takes ~1.04ms to transmit (10 bits total)
    // Wait for 300ms of no data before considering transmission complete
    // This ensures we capture all characters even with typing delays
    const int MAX_CONSECUTIVE_NO_DATA = 300; // 300 iterations * 1ms = 300ms
    
    // Return 0 on second read (standard /proc behavior)
    // This prevents the kernel from repeatedly calling our read function
    if (*ppos > 0) {
        return 0;
    }
    
    // Wait for first character with timeout (up to 1 second)
    int timeout = 1000;
    while (!uart_data_available() && timeout-- > 0) {
        udelay(1000); // Wait 1ms (1000 microseconds)
    }
    
    if (timeout <= 0) {
        return 0; // No data received
    }
    
    // Read all available characters
    // Keep reading until buffer is full or no more data arrives for a while
    while (i < (sizeof(kbuf) - 1) && i < count) {
        // Read characters as fast as they arrive (drain FIFO quickly)
        while (uart_data_available() && i < (sizeof(kbuf) - 1) && i < count) {
            c = uart_receive_char();
            if (c != 0) {
                kbuf[i++] = c;
                consecutive_no_data = 0; // Reset counter when we get data
            }
        }
        
        // If we've read some data but no more is available, wait a bit
        if (i > 0 && !uart_data_available()) {
            udelay(1000); // Wait 1ms
            consecutive_no_data++;
            
            // If no data for MAX_CONSECUTIVE_NO_DATA iterations, 
            // consider transmission complete
            if (consecutive_no_data >= MAX_CONSECUTIVE_NO_DATA) {
                break; // No more data coming
            }
        } else if (i == 0 && !uart_data_available()) {
            // No data at all, wait a bit and check again
            udelay(1000);
            consecutive_no_data++;
            if (consecutive_no_data >= MAX_CONSECUTIVE_NO_DATA) {
                break;
            }
        }
    }
    
    if (i == 0) {
        return 0;
    }
    
    kbuf[i] = '\0';
    
    if (copy_to_user(buf, kbuf, i)) {
        return -EFAULT;
    }
    
    *ppos += i;  // Update file position - critical for preventing repeated reads
    
    pr_info("UART RX: received %d bytes: %s\n", i, kbuf);
    return i;
}

// Proc file write handler for transmitting data
static ssize_t uart_proc_write(struct file *file,
    const char __user *buf, size_t count, loff_t *ppos)
{
    char kbuf[256];
    size_t len;
    
    // Limit the size to prevent buffer overflow 
    len = min(count, sizeof(kbuf) - 1);
    
    if (copy_from_user(kbuf, buf, len)) {
        return -EFAULT;
    }
    
    kbuf[len] = '\0';
    
    // Send the string via UART 
    uart_send_string(kbuf);
    
    pr_info("UART TX: sent %zu bytes\n", len);
    
    return count;
}

// Proc operations for TX (write to send data)
static const struct proc_ops uart_tx_proc_ops = {
    .proc_write = uart_proc_write,   
};

// Proc operations for RX (read to receive data)
static const struct proc_ops uart_rx_proc_ops = {
    .proc_read = uart_proc_read,
};

// Module initialization 
static int __init uart_driver_init(void)
{
    // Map GPIO registers 
    gpio = ioremap(GPIO_BASE, 0x1000);
    if (!gpio) {
        pr_err("Failed to map GPIO registers\n");
        return -ENOMEM;
    }
    
    // Map UART registers 
    uart = ioremap(AUX_BASE, sizeof(struct uart_regs));
    if (!uart) {
        pr_err("Failed to map UART registers\n");
        iounmap(gpio);
        return -ENOMEM;
    }
    
    // Initialize the Mini UART 
    uart_init_os();
    
    // Create /proc/uart_tx file for transmitting data
    proc_tx = proc_create(PROC_UART_TX, 0666, NULL, &uart_tx_proc_ops); 
    if (!proc_tx) {
        pr_err("Failed to create /proc/%s\n", PROC_UART_TX);
        iounmap(uart);
        iounmap(gpio);
        return -ENOMEM;
    }
    
    // Create /proc/uart_rx file for receiving data
    proc_rx = proc_create(PROC_UART_RX, 0666, NULL, &uart_rx_proc_ops);
    if (!proc_rx) {
        pr_err("Failed to create /proc/%s\n", PROC_UART_RX);
        proc_remove(proc_tx);
        iounmap(uart);
        iounmap(gpio);
        return -ENOMEM;
    }
    
    // Send a test message 
    uart_send_string("Mini UART driver loaded successfully!\r\n");
    
    pr_info("UART driver loaded.\n");
    pr_info("Write to /proc/%s to send data\n", PROC_UART_TX);
    pr_info("Read from /proc/%s to receive data\n", PROC_UART_RX);
    return 0;
}

// Module cleanup
static void __exit uart_driver_exit(void)
{
    uart_send_string("Mini UART driver unloading...\r\n");
    
    // Remove proc entries
    proc_remove(proc_tx);
    proc_remove(proc_rx);
    
    // Unmap registers 
    if (uart)
        iounmap(uart);
    if (gpio)
        iounmap(gpio);
    
    pr_info("UART driver unloaded.\n");
}

module_init(uart_driver_init);
module_exit(uart_driver_exit);

MODULE_AUTHOR("Supriya Mishra");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("BCM2711 Mini UART Driver");
