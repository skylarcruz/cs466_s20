/* Standard includes. */
#include <stdbool.h>
#include <stdio.h>


/* Kernel includes. */
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

/* Hardware includes. */
#include "inc/tm4c123gh6pm.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/watchdog.h"
#include "driverlib/timer.h"
#include "driverlib/rom.h"
#include "inc/hw_memmap.h"

/* Optional includes for USB serial output */
#ifdef USB_SERIAL_OUTPUT
#include "driverlib/rom.h"
#include "utils/uartstdio.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#endif

/*local includes*/
#include "assert.h"

#define LED_R (1<<1)
#define LED_B (1<<2)
#define LED_G (1<<3)
#define SW1   (1<<4)
#define SW2   (1<<0)

#define Quad_A (1<<0) // line A quadrature input
#define Quad_B (1<<1) // line B quadrature input

#define SLA_LED_B (1<<0)
#define SLA_LED_R (1<<1)
#define SLA_LED_G (1<<2)

#define PHASE_1 (0x0)
#define PHASE_2 (0x1)
#define PHASE_3 (0x3)
#define PHASE_4 (0x2)

#define LED_ON(x) (GPIO_PORTF_DATA_R |= (x))
#define LED_OFF(x) (GPIO_PORTF_DATA_R &= ~(x))
#define LED(led,on) ((on)?LED_ON(led):LED_OFF(led))

uint8_t LED_REG = 0x00; // 0b0000 0000; addr = 0x01
uint8_t SW_REG = 0x00; // 0b0000 0011; addr = 0x02
uint8_t INT_REG = 0x00; // 0b0000 0000
uint32_t SystemCoreClock;

uint32_t MOTOR_POS = 0;
//uint8_t MOTOR_STATE = 0x00; // A = bit 0, B = bit 1
uint32_t PREV_MOTOR_STATE = PHASE_1; 

#ifdef USB_SERIAL_OUTPUT

#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
    _assert_failed ("__error__", pcFilename, ui32Line);
}
#endif

//*****************************************************************************
//
//: Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
static void
_configureUART(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    UARTStdioConfig(0, 115200, 16000000);
}

#endif


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

void
delayMs(uint32_t ms)
{
    while(ms--)
    {
        delayUs(1000);
    }
}


static void
_interruptHandlerPortD(void)
{
    // uint32_t A_State = GPIO_PORTD_DATA_R & Quad_A;
	// uint32_t B_State = GPIO_PORTD_DATA_R & Quad_B;
    // uint32_t Phase = A_State | B_State;

    uint32_t Phase = (GPIO_PORTD_DATA_R & Quad_A) | (GPIO_PORTD_DATA_R & Quad_B);

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    uint32_t mask = GPIOIntStatus(GPIO_PORTD_BASE, 1);

    if ( mask & (Quad_A | Quad_B) )
    {
        if(Phase == PHASE_1){
            if(PREV_MOTOR_STATE == PHASE_4){
                MOTOR_POS += 1;
            }
            else if(PREV_MOTOR_STATE == PHASE_2){
                MOTOR_POS -= 1;
            }
            else
                UARTprintf("Phase skipped\n");
        }
        else if(Phase == PHASE_2){
            if(PREV_MOTOR_STATE == PHASE_1){
                MOTOR_POS += 1;
            }
            else if(PREV_MOTOR_STATE == PHASE_3){
                MOTOR_POS -= 1;
            }
            else
                UARTprintf("Phase skipped\n");
        }
        else if(Phase == PHASE_3){
            if(PREV_MOTOR_STATE == PHASE_2){
                MOTOR_POS += 1;
            }
            else if(PREV_MOTOR_STATE == PHASE_4){
                MOTOR_POS -= 1;
            }
            else
                UARTprintf("Phase skipped\n");
        }
        else if(Phase == PHASE_4){
            if(PREV_MOTOR_STATE == PHASE_3){
                MOTOR_POS += 1;
            }
            else if(PREV_MOTOR_STATE == PHASE_1){
                MOTOR_POS -= 1;
            }
            else
                UARTprintf("Phase skipped\n");
        }
        PREV_MOTOR_STATE = Phase;
    }

    //UARTprintf("Phase: %x\n", Phase);

    if (xHigherPriorityTaskWoken)
    {
        // should request a schedule update....  
        // ....stay tuned
    }    

    //UARTprintf("Motor Position: %d\n", MOTOR_POS);
    GPIOIntClear(GPIO_PORTD_BASE, mask);
}


static void
_setupHardware(void)
{
    //
    // Enable the GPIO port that is used for the on-board LED.
    // This is a TiveDriver library function
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    GPIO_PORTF_LOCK_R = 0x4C4F434B; // unlock for SW2
    GPIO_PORTF_CR_R = 0xff;

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);


    // Enable the GPIO pin for the LED (PF3).  Set the direction as output, and
    // enable the GPIO pin for digital function.
    // These are TiveDriver library functions
    //

    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, (LED_G|LED_R|LED_B));
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, (SW1 | SW2) );
    GPIOPadConfigSet(GPIO_PORTF_BASE, (SW1 | SW2), GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, (Quad_A | Quad_B) );
    
    GPIOIntRegister(GPIO_PORTD_BASE, _interruptHandlerPortD);
    GPIOIntTypeSet(GPIO_PORTD_BASE, (Quad_A | Quad_B), GPIO_BOTH_EDGES);
    //GPIOIntTypeSet(GPIO_PORTD_BASE, Quad_B, GPIO_BOTH_EDGES);

    IntPrioritySet(INT_GPIOD, 255);  // Required with FreeRTOS 10.1.1, 
    GPIOIntEnable(GPIO_PORTD_BASE, (Quad_A | Quad_B) );
    //GPIOIntEnable(GPIO_PORTD_BASE, Quad_B);

    // Timer Setup
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, ROM_SysCtlClockGet());
    ROM_TimerEnable(TIMER0_BASE, TIMER_A);


    SystemCoreClock = 80000000;  // Required for FreeRTOS.

    SysCtlClockSet( SYSCTL_SYSDIV_2_5 |
                    SYSCTL_USE_PLL |
                    SYSCTL_XTAL_16MHZ |
                    SYSCTL_OSC_MAIN);

    //PREV_MOTOR_POS

}




static void
_heartbeat( void *notUsed )
{
    uint32_t green500ms = 500; // 1 second
    uint32_t ledOn = 0;

    uint32_t prevTicks = 0;
    uint32_t newTicks = 0;
    uint32_t totalTicks;

    uint32_t startTime;
    uint32_t endTime;
    uint32_t totalTime;
	
    while(true)
    {
        ledOn = !ledOn;
        LED(LED_G, ledOn);

        prevTicks = MOTOR_POS;
        startTime = TimerValueGet(TIMER0_BASE, TIMER_A);

        //UARTprintf("Heartbeat led: %u\n",ledOn);
        vTaskDelay(green500ms / portTICK_RATE_MS);

        newTicks = MOTOR_POS;
        totalTicks = newTicks - prevTicks;

        endTime = TimerValueGet(TIMER0_BASE, TIMER_A);

        if(startTime > endTime)
            totalTime = startTime - endTime;
        else
            totalTime = startTime + (ROM_SysCtlClockGet() - endTime);

        //UARTprintf("Total Motor Rotations = %d\n", totalTicks/300);
        //UARTprintf("Time in system ticks: %d\n", totalTime);
        UARTprintf("%d RPM\n", (totalTicks/300) * 120);



		
    }
}

int main( void )
{
	

    _setupHardware();

#ifdef USB_SERIAL_OUTPUT
    void spinDelayMs(uint32_t ms);
    _configureUART();
    spinDelayMs(1000);  // Allow UART to setup
    UARTprintf("System Start\n");
#endif

    xTaskCreate(_heartbeat,
                "green",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 1,  // higher numbers are higher priority..
                NULL );


	UARTprintf("Interrupt Definitions\n");
	UARTprintf("GPIO_FALLING_EDGE = %u\n",GPIO_FALLING_EDGE);
	UARTprintf("GPIO_RISING_EDGE = %u\n",GPIO_RISING_EDGE);


    /* Start the tasks and timer running. */
    vTaskStartScheduler();

    assert(0); // we should never get here..

    return 0;
}
