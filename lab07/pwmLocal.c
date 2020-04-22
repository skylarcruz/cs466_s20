/* Standard includes. */
#include <stdbool.h>
#include <stdio.h> 
#include <stdlib.h>
#include <stdint.h>

/* Hardware includes. */
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"


#include "assert.h"


void ConfigurePWM(uint32_t baseFrequency)
{
    //
    // Enable Peripheral Clocks 
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    //
    // Enable pin PB6 for PWM0 M0PWM0
    //
    GPIOPinConfigure(GPIO_PB6_M0PWM0);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);

    //Configure PWM Clock to match system
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

    // Enable the peripherals used by this program.
    unsigned long ulPeriod = SysCtlClockGet() / baseFrequency; //PWM frequency 15KHz
  
    //Configure PWM Options
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);

    //Set the Period (expressed in clock ticks)
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ulPeriod);

    //Set PWM duty-10% (Period /10);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0,ulPeriod/10);

    // Enable the PWM generator
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);

    // Turn on the Output pins
    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);
}

void setPWMDuty(uint32_t percent)
{
    assert(percent <=100);
    unsigned long period = PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0,(period*percent)/100);
}

