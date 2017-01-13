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
unsigned int GetGPFSELReg(char pin);
char GetGPIOPinOffset(char pin);

void SetInternalPullUpDown(char pin, char value);
void SetGpioPinDirection(char pin, char direction);

void SetGpioPin(char pin);
void ClearGpioPin(char pin);

char GetGpioPinValue(char pin);

#endif
