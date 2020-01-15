//*****************************************************************************
//
// blinky.c - Simple example to blink the on-board LED.
//
// .. Leveraged from the TI Blinky example but no external dependencies
//
//*****************************************************************************

#include <stdint.h>
#include "inc/tm4c123gh6pm.h"

//
// This is a poor delay method.. Will delay appx 1us,
// Probably inaccurate
//
void
delayUs(uint32_t us)
{
    while(us--)
    {
        __asm("    nop\n"
              "    nop\n"
              "    nop\n");
    }
}

//
// This is a Very poor delay method.. Will delay appx 1ms.
// Probably inaccurate
//
void
delayMs(uint32_t ms)
{
    while(ms--)
    {
        delayUs(1000);
    }
}

int
main(void)
{
    volatile uint32_t ui32Loop;

    //
    // Enable the GPIO port that is used for the on-board LED.
    // Pause for a few moments afterwards to let the chip settle..
    //
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOF;
    delayUs(10);

    //
    // setup the main loop delay for 500ms on then 500ms off...
    //
    ui32Loop = 500;

    //
    // Enable the GPIO pin for the LED (PF3).
    // Set the direction as output.
    // Enable the GPIO pin for digital function.
    //
    GPIO_PORTF_DIR_R = (1<<3);
    GPIO_PORTF_DEN_R = (1<<3);

    //
    // Loop forever.
    //
    while(1)
    {
        //
        // Turn on the LED.
        //
        GPIO_PORTF_DATA_R |= (1<<3);

        //
        // Delay for a bit.
        //
        delayMs(ui32Loop);

        //
        // Turn off the LED.
        //
        GPIO_PORTF_DATA_R &= ~(1<<3);

        //
        // Delay for a bit.
        //
        delayMs(ui32Loop);
    }
}
