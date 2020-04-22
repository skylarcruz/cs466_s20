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
#include "driverlib/timer.h"

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Optional includes for USB serial output */
#ifdef USB_SERIAL_OUTPUT
#include "driverlib/rom.h"
#include "utils/uartstdio.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#endif


#define PHA   (1<<4) 
#define PHB   (1<<5)

#define IN1   (1<<2)
#define IN2   (1<<3)

#include "assert.h"
#include "pwmLocal.h"
#include "motor.h"

#define DEADBAND 0 // What is this for?

static int32_t _encoder = 0;


// 
// This handler uses the prior and current phase data to indicate a 
// state change.  Looking up a value at that resolved 4-bit (16 possibilities) 
// returns the encoderposition change
//
#if 0
static void 
_interruptHandlerPort(void)
{
    uint32_t mask = GPIOIntStatus(GPIO_PORTC_BASE, 1);
    static uint8_t tt=0;
    static int8_t lookup[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};

    tt <<= 2; // Shift history Left

    // set lower two bits of tt to value of A and B
    uint8_t Port = GPIO_PORTC_DATA_R;
    tt |= (Port & PHA) ? 0x02 : 0x00;
    tt |= (Port & PHB) ? 0x01 : 0x00;

    _encoder += lookup[tt & 0x0F];
    GPIOIntClear(GPIO_PORTC_BASE, mask);
}

#endif


// 
// This handler uses the state diagram that I presented in 
// lab using the change in the 2 state variable system 
// to generate the encoder position change
//
#if 1
static void 
_interruptHandlerPort(void)
{
    uint32_t mask = GPIOIntStatus(GPIO_PORTC_BASE, 1);
    uint8_t port = GPIO_PORTC_DATA_R;

    static int8_t old=0;
    uint8_t new;
    new = (port & PHA)?0x02:0x00;
    new |=(port & PHB)?0x01:0x00;
    
    switch(old)
    {
        case 0:
            switch(new)
            {
                case 0: break;
                case 1: _encoder++; break;
                case 2: _encoder--; break;
                case 3: break;
            }
            break;
        case 1:
            switch(new)
            {
                case 0: _encoder--; break;
                case 1: break;
                case 2: break;
                case 3: _encoder++; break;
            }
            break;
        case 2:
            switch(new)
            {
                case 0: _encoder++; break;
                case 1: break;
                case 2: break;
                case 3: _encoder--; break;
            }
            break;
        case 3:
            switch(new)
            {
                case 0: break;
                case 1: _encoder--; break;
                case 2: _encoder++; break;
                case 3: break;
            }
            break;
    }
    old = new;

    GPIOIntClear(GPIO_PORTC_BASE, mask);
}
#endif

void motorDrive(int32_t sp)
{
    assert(sp >= -100);
    assert(sp <= 100);
    int in1, in2;

    if (sp > 0)
    {
        in1=1;
        in2=0;
        GPIO_PORTE_DATA_R |= IN1;
        GPIO_PORTE_DATA_R &= ~IN2;
    }
    else
    {
        in1=0;
        in2=1;
        GPIO_PORTE_DATA_R &= ~IN1;
        GPIO_PORTE_DATA_R |= IN2;
    }
    
    int duty = (abs(sp) * (100 - DEADBAND) / 100);

    setPWMDuty(abs(sp));

#ifdef USB_SERIAL_OUTPUT
    UARTprintf("    !! sp:%d, i1:%d, i2:%d, pwm%d\n", sp, in1, in2, abs(sp));
#endif
}


int motor[] = {10, 15, 20, 25, 30, 35, 40, 50, 20, 0, -10, +10, +10};
int motorSteps = sizeof(motor)/sizeof(int);

static void
_setupTimer(void)
{
    // 
    // Enable the timer0 periphial and remember to 
    // stall for a few cycles after.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    spinDelayUs(10);

    //
    // Configure TimerA as a full-width perodic 
    // A timer, Note: The driver lib says this will 
    // be a 64 bit timer... It's 32 bits.  A little 
    // testing showed that really quick
    //
    TimerClockSourceSet(TIMER0_BASE, TIMER_CLOCK_SYSTEM);
    //
    // Count Down, had wierdness counting up, reload with 
    // 0xffffffff when timer hits 0
    //
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, UINT32_MAX);

    //
    // Unleash the timer...
    //
    TimerEnable(TIMER0_BASE, TIMER_A);
}

#if 0
static uint32_t 
_readTimer(void)
{
    //
    // Sicne the counter is counting down from UINT32_MAX to
    // zero, perform a subtractaction so it appears to be 
    // counting up.
    //
    return (UINT32_MAX - TimerValueGet(TIMER0_BASE, TIMER_A));
}
#endif

static uint32_t _motorSetpoint = 0;

#define min(x,y) (((x) > (y))?(y):(x))
#define max(x,y) (((x) < (y))?(x):(y))

static void 
_pidServo( void *notUsed )
{
    int previous_error = 0;
    double integral = 0;
    int dt=10;

    double Kp = 0.35;  // TODO: Requires setting see PID Wiki for tuning
    double Ki = 0.002;  // TODO: Requires setting see PID Wiki for tuning
    double Kd = 0.05;  // TODO: Requires setting see PID Wiki for tuning

    while(1)
    {
        int error = _motorSetpoint - _encoder;
        integral = integral + error*dt;
        double derivative = (error - previous_error)/dt;
        double output = Kp*error + Ki*integral + Kd*derivative;
        int drive = output;
        if (drive > 99) drive = 99;
        if (drive < -99) drive = -99;
        motorDrive(drive);
        previous_error = error;

#ifdef USB_SERIAL_OUTPUT
        //UARTprintf("  !! sp:%d, mp:%d, d:%d, o:%d\n", _motorSetpoint, _encoder, drive);
        UARTprintf("  !! sp:%d, mp:%d, d:%d\n", _motorSetpoint, _encoder, drive);
#endif

        vTaskDelay(dt);        
    }
}


void motor_init(void)
{
    ConfigurePWM(15000);
    setPWMDuty(50);
    _setupTimer();


    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);


    GPIO_PORTC_CR_R = PHA | PHB;
    GPIO_PORTE_CR_R = IN1 | IN2;
    GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, PHA | PHB);
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, IN1 | IN2);

    //
    // Register the port-level interrupt handler. This handler is the first
    // level interrupt handler for all the pin interrupts.
    //
    GPIOIntRegister(GPIO_PORTC_BASE, _interruptHandlerPort);
    GPIOIntTypeSet(GPIO_PORTC_BASE, (PHA|PHB), GPIO_BOTH_EDGES);
    GPIOIntEnable(GPIO_PORTC_BASE, (PHA|PHB));

    xTaskCreate(_pidServo,
                "bid",
                1024,   
                NULL,
                tskIDLE_PRIORITY+2, 
                NULL );        

}


void motor_move(uint32_t pos_in_tics)
{

#ifdef USB_SERIAL_OUTPUT
    UARTprintf("  !! motor position is %d..\n", _encoder);
#endif

    _motorSetpoint = pos_in_tics;
}

motor_status_t motor_status(void)
{
    // TODO:
    return M_IDLE;
}



