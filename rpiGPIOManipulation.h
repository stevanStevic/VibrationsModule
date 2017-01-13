#ifndef _RPI_INCLUDED
#define _RPI_INCLUDED

#include <asm/io.h>

// NOTE: Check Broadcom BCM8325 datasheet, page 91+

/* GPIO registers base address. */
#define GPIO_BASE_ADDR (0x3F200000)
//--

//000 = GPIO Pin 'x' is an input
//001 = GPIO Pin 'x' is an output 
// By default GPIO pin is being used as in input 
#define GPIO_DIRECTION_IN  (0)
#define GPIO_DIRECTION_OUT (1)
//--

//Handle GPIO: 0-9
/* GPIO Function Select 0. */
#define GPFSEL0_BASE_ADDR (GPIO_BASE_ADDR + 0x00000000)

//Handle GPIO: 10-19
/* GPIO Function Select 1. */
#define GPFSEL1_BASE_ADDR (GPIO_BASE_ADDR + 0x00000004)

//Handle GPIO: 20-29
/* GPIO Function Select 2. */
#define GPFSEL2_BASE_ADDR (GPIO_BASE_ADDR + 0x00000008)

//Handle GPIO: 30-39
/* GPIO Function Select 3. */
#define GPFSEL3_BASE_ADDR (GPIO_BASE_ADDR + 0x0000000C)

//Handle GPIO: 40-49
/* GPIO Function Select 4. */
#define GPFSEL4_BASE_ADDR (GPIO_BASE_ADDR + 0x00000010)

//Handle GPIO: 50-53
/* GPIO Function Select 5. */
#define GPFSEL5_BASE_ADDR (GPIO_BASE_ADDR + 0x00000014)
//--

//GPIO: 0-31
/* GPIO Pin Output Set 0. */
#define GPSET0_BASE_ADDR (GPIO_BASE_ADDR + 0x0000001C)

//GPIO: 32-53
/* GPIO Pin Output Set 1. */
#define GPSET1_BASE_ADDR (GPIO_BASE_ADDR + 0x00000020)

//--
//GPIO: 0-31
/* GPIO Pin Output Clear 0. */
#define GPCLR0_BASE_ADDR (GPIO_BASE_ADDR + 0x00000028)

//GPIO: 32-53
/* GPIO Pin Output Clear 1. */
#define GPCLR1_BASE_ADDR (GPIO_BASE_ADDR + 0x0000002C)

//--
//GPIO: 0-31
/* GPIO Pin Level 0. */
#define GPLEV0_BASE_ADDR (GPIO_BASE_ADDR + 0x00000034)

//GPIO: 32-53
/* GPIO Pin Level 1. */
#define GPLEV1_BASE_ADDR (GPIO_BASE_ADDR + 0x00000038)
//--

//GPIO: 0-53
/* GPIO Pin Pull-up/down Enable. */
#define GPPUD_BASE_ADDR (GPIO_BASE_ADDR + 0x00000094)

//GPIO: 0-31
/* GPIO Pull-up/down Clock Register 0. */
#define GPPUDCLK0_BASE_ADDR (GPIO_BASE_ADDR + 0x00000098)

//GPIO: 32-53
/* GPIO Pull-up/down Clock Register 1. */
#define GPPUDCLK1_BASE_ADDR (GPIO_BASE_ADDR + 0x0000009C)

/* PUD - GPIO Pin Pull-up/down */
#define PULL_NONE (0)
#define PULL_DOWN (1)
#define PULL_UP   (2)

/* Avilable GPIO pins. */
#define GPIO_02 (2)
#define GPIO_03 (3)
#define GPIO_04 (4)
#define GPIO_05 (5)
#define GPIO_06 (6)
#define GPIO_07 (7)
#define GPIO_08 (8)
#define GPIO_09 (9)
#define GPIO_10 (10)
#define GPIO_11 (11)
#define GPIO_12 (12)
#define GPIO_13 (13)
#define GPIO_14 (14)
#define GPIO_15 (15)
#define GPIO_16 (16)
#define GPIO_17 (17)
#define GPIO_18 (18)
#define GPIO_19 (19)
#define GPIO_20 (20)
#define GPIO_21 (21)
#define GPIO_22 (22)
#define GPIO_23 (23)
#define GPIO_24 (24)
#define GPIO_25 (25)
#define GPIO_26 (26)
#define GPIO_27 (27)

/* Manipulation functions */
unsigned int GetGPFSELReg(char pin)
{
    unsigned int addr;
    
    if(pin >= 0 && pin <10)
        addr = GPFSEL0_BASE_ADDR;
    else if(pin >= 10 && pin <20)
        addr = GPFSEL1_BASE_ADDR;
    else if(pin >= 20 && pin <30)
        addr = GPFSEL2_BASE_ADDR;
    else if(pin >= 30 && pin <40)
        addr = GPFSEL3_BASE_ADDR;
    else if(pin >= 40 && pin <50)
        addr = GPFSEL4_BASE_ADDR;
    else /*if(pin >= 50 && pin <53) */
        addr = GPFSEL5_BASE_ADDR;
  
  return addr;
}

char GetGPIOPinOffset(char pin)
{
    if(pin >= 0 && pin <10)
        pin = pin;
    else if(pin >= 10 && pin <20)
        pin -= 10;
    else if(pin >= 20 && pin <30)
        pin -= 20;
    else if(pin >= 30 && pin <40)
        pin -= 30;
    else if(pin >= 40 && pin <50)
        pin -= 40;
    else /*if(pin >= 50 && pin <53) */
        pin -= 50;

    return pin;
}

void SetInternalPullUpDown(char pin, char value)
{
    unsigned int base_addr_gppud; 
    unsigned int base_addr_gppudclk; 
    void *addr = NULL;
    unsigned int tmp;
    unsigned int mask;
    
    /* Get base address of GPIO Pull-up/down Register (GPPUD). */
    base_addr_gppud = GPPUD_BASE_ADDR;
    
    /* Get base address of GPIO Pull-up/down Clock Register (GPPUDCLK). */
    base_addr_gppudclk = (pin < 32) ? GPPUDCLK0_BASE_ADDR : GPPUDCLK1_BASE_ADDR;

    /* Get pin offset in register . */
    pin = (pin < 32) ? pin : pin - 32;
    
    /* Write to GPPUD to set the required control signal (i.e. Pull-up or Pull-Down or neither
       to remove the current Pull-up/down). */
    addr = ioremap(base_addr_gppud, 4);
    iowrite32(value, addr);

    /* Wait 150 cycles  this provides the required set-up time for the control signal */
    
    /* Write to GPPUDCLK0/1 to clock the control signal into the GPIO pads you wish to
       modify  NOTE only the pads which receive a clock will be modified, all others will
       retain their previous state. */
    addr = ioremap(base_addr_gppudclk, 4);
    tmp = ioread32(addr);    
    mask = 0x1 << pin;
    tmp |= mask;        
    iowrite32(tmp, addr);

    /* Wait 150 cycles  this provides the required hold time for the control signal */

    /* Write to GPPUD to remove the control signal. */
    addr = ioremap(base_addr_gppud, 4);
    iowrite32(PULL_NONE, addr);

    /* Write to GPPUDCLK0/1 to remove the clock. */
    addr = ioremap(base_addr_gppudclk, 4);
    tmp = ioread32(addr);    
    mask = 0x1 << pin;
    tmp &= (~mask);        
    iowrite32(tmp, addr);
}


void SetGpioPinDirection(char pin, char direction)
{
    unsigned int base_addr; 
    void *addr = NULL;
    unsigned int tmp;
    unsigned int mask;
    
    /* Get base address of function selection register. */
    base_addr = GetGPFSELReg(pin);

    /* Calculate gpio pin offset. */
    pin = GetGPIOPinOffset(pin);    
    
    /* Set gpio pin direction. */
    addr = ioremap(base_addr, 4);
    tmp = ioread32(addr);
    if(direction)
    { //set as output: set 1
      mask = 0x1 << (pin*3);
      tmp |= mask;
    }
    else
    { //set as input: set 0
      mask = ~(0x1 << (pin*3));
      tmp &= mask;
    }
    iowrite32(tmp, addr);
}

void SetGpioPin(char pin)
{
    void *addr = NULL;
    unsigned int tmp;
    
    /* Get base address of gpio set register. */
    addr = (pin < 32) ? (void *) GPSET0_BASE_ADDR : (void *)GPSET1_BASE_ADDR;
    pin = (pin < 32) ? pin : pin - 32;
    
    /* Set gpio. */
    addr = ioremap((unsigned long)addr, 4);
    tmp = 0x1 << pin;
    iowrite32(tmp, addr);
}

void ClearGpioPin(char pin)
{
    void *addr = NULL;
    unsigned int tmp;
    
    /* Get base address of gpio clear register. */    
    addr = (pin < 32) ? (void *)GPCLR0_BASE_ADDR : (void *)GPCLR1_BASE_ADDR;
    pin = (pin < 32) ? pin : pin - 32;
        
    /* Clear gpio. */
    addr = ioremap((unsigned long)addr, 4);
    tmp = 0x1 << pin;
    iowrite32(tmp, addr);
}

char GetGpioPinValue(char pin)
{
    void *addr = NULL;
    unsigned int tmp;
    unsigned int mask;
    
    /* Get base address of gpio level register. */
    addr = (pin < 32) ? (void *) GPLEV0_BASE_ADDR : (void *)GPLEV1_BASE_ADDR;
    pin = (pin < 32) ? pin : pin - 32;

    /* Read gpio pin level. */
    addr = ioremap((unsigned long)addr, 4);
    tmp = ioread32(addr);
    mask = 0x1 << pin;
    tmp &= mask;    
    
    return (tmp >> pin);
}

#endif
