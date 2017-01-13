#include "rpiGPIOManipulation.h"

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

#endif

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

