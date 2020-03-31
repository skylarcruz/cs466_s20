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

#define LED_ON(x) (GPIO_PORTF_DATA_R |= (x))
#define LED_OFF(x) (GPIO_PORTF_DATA_R &= ~(x))
#define LED(led,on) ((on)?LED_ON(led):LED_OFF(led))

uint8_t LED_REG = 0x00; // 0b0000 0000; addr = 0x01
uint8_t SW_REG = 0x00; // 0b0000 0011; addr = 0x02
uint8_t INT_REG = 0x00; // 0b0000 0000; addr = 0x03

uint32_t SystemCoreClock;

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
	uint32_t A_State = 0;
	uint32_t B_State = 0;
    //
    // We have not woken a task at the start of the ISR.
    //
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	
  
	

    uint32_t mask = GPIOIntStatus(GPIO_PORTD_BASE, 1);

    if ( mask & Quad_A)
    {
		if(A_State)
		{
			UARTprintf("Quad_A High\n");
			A_State = !A_State;
		}else if(!A_State)
		{
			UARTprintf("Quad_A Low\n");
			A_State = !A_State;
		}

    }

    if( mask & Quad_B)
    {
		if(B_State)		
		{
			UARTprintf("Quad_B High\n");
			B_State = !B_State;
		}else if(!B_State)
		{
			UARTprintf("Quad_B Low\n");
			B_State = !B_State;
		}
	
	}


    if (xHigherPriorityTaskWoken)
    {
        // should request a schedule update....  
        // ....stay tuned
    }    

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

    SystemCoreClock = 80000000;  // Required for FreeRTOS.

    SysCtlClockSet( SYSCTL_SYSDIV_2_5 |
                    SYSCTL_USE_PLL |
                    SYSCTL_XTAL_16MHZ |
                    SYSCTL_OSC_MAIN);

}




static void
_heartbeat( void *notUsed )
{
    uint32_t green500ms = 500; // 1 second
    uint32_t ledOn = 0;
	
    while(true)
    {
        ledOn = !ledOn;
        LED(LED_G, ledOn);
		
        UARTprintf("Heartbeat led: %u\n",ledOn);
        vTaskDelay(green500ms / portTICK_RATE_MS);
		
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
