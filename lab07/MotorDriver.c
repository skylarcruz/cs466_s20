/* Standard includes. */
#include <stdbool.h>
#include <stdio.h>

#include "motor.h"


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

// #define Quad_A (1<<0) // line A quadrature input
// #define Quad_B (1<<1) // line B quadrature input

// #define SLA_LED_B (1<<0)
// #define SLA_LED_R (1<<1)
// #define SLA_LED_G (1<<2)

// #define PHASE_1 (0x0)
// #define PHASE_2 (0x1)
// #define PHASE_3 (0x3)
// #define PHASE_4 (0x2)

#define LED_ON(x) (GPIO_PORTF_DATA_R |= (x))
#define LED_OFF(x) (GPIO_PORTF_DATA_R &= ~(x))
#define LED(led,on) ((on)?LED_ON(led):LED_OFF(led))

// uint8_t LED_REG = 0x00; // 0b0000 0000; addr = 0x01
// uint8_t SW_REG = 0x00; // 0b0000 0011; addr = 0x02
// uint8_t INT_REG = 0x00; // 0b0000 0000
uint32_t SystemCoreClock;

int Pos[7] = {1000, 2000, 3000, -5000, 10000, -1000, 0};
int currPos = -1;
int LED_R_flag = 0;
int resetCountdown = -1;

// uint32_t MOTOR_POS = 0;
// //uint8_t MOTOR_STATE = 0x00; // A = bit 0, B = bit 1
// uint32_t PREV_MOTOR_STATE = PHASE_1; 

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
_interruptHandlerPortF(void)
{

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    uint32_t mask = GPIOIntStatus(GPIO_PORTF_BASE, 1);

    if (mask)
    {
        //UARTprintf("Interrupt Triggered\n");
        if(GPIO_PORTF_DATA_R & SW1){
            //UARTprintf("SW1 UnPressed\n");
            resetCountdown = -1;
        }
        else{
            //UARTprintf("SW1 Pressed\n");
            if(currPos < 6){
                currPos++; 
                UARTprintf("Moving Motor to position %d\n", Pos[currPos]);
                GPIO_PORTF_DATA_R |= LED_B;
            }
            else{
                currPos = 0; 
                UARTprintf("Moving Motor to position %d\n", Pos[currPos]);
                GPIO_PORTF_DATA_R |= LED_B;
            }
        }
        if(GPIO_PORTF_DATA_R & SW2){
            //UARTprintf("SW2 Not Pressed\n");
            resetCountdown = -1;
        }
        else{
            //UARTprintf("SW2 Pressed\n");
            if(currPos > 0){
                currPos--; 
                UARTprintf("Moving Motor to position %d\n", Pos[currPos]);
            }
            else if(currPos == 0){
                currPos = 6; 
                UARTprintf("Moving Motor to position %d\n", Pos[currPos]);
            }
        }

        // UARTprintf("sw1 test = %x\n", (~GPIO_PORTF_DATA_R & SW1) >> 4);
        // UARTprintf("sw1 test = %x\n", (~GPIO_PORTF_DATA_R & SW1) >> 4);
        
        // UARTprintf("sw2 test = %x\n", (~GPIO_PORTF_DATA_R & SW2));

        if( ((~GPIO_PORTF_DATA_R & SW1) >> 4) & ((~GPIO_PORTF_DATA_R & SW2)) ){
            if(currPos > -1){
                resetCountdown = 5;
            }
        }
    }

    if (xHigherPriorityTaskWoken)
    {
        // should request a schedule update....  
        // ....stay tuned
    }    

    if(currPos > -1){
        GPIO_PORTF_DATA_R |= LED_B;
    }
    else{
        GPIO_PORTF_DATA_R &= ~LED_B;
    }

    GPIOIntClear(GPIO_PORTF_BASE, mask);
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

    // Enable the GPIO pin for the LED (PF3).  Set the direction as output, and
    // enable the GPIO pin for digital function.
    // These are TiveDriver library functions
    //

    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, (LED_G|LED_R|LED_B));
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, (SW1 | SW2) );
    GPIOPadConfigSet(GPIO_PORTF_BASE, (SW1 | SW2), GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    // Timer Setup
    //ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    //ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    //ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, ROM_SysCtlClockGet());
    //ROM_TimerEnable(TIMER0_BASE, TIMER_A);


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
    uint32_t green100ms = 100; // 1/10 second
    uint32_t ledOn = 0;
    uint32_t count = 0;
    //uint32_t motorPos = 1;

    // uint32_t prevTicks = 0;
    // uint32_t newTicks = 0;
    // uint32_t totalTicks;

    // uint32_t startTime;
    // uint32_t endTime;
    // uint32_t totalTime;

    GPIOIntRegister(GPIO_PORTF_BASE, _interruptHandlerPortF);
    GPIOIntTypeSet(GPIO_PORTF_BASE, (SW1 | SW2), GPIO_BOTH_EDGES);
    GPIOIntEnable(GPIO_PORTF_BASE, (SW1 | SW2));
	
    while(true)
    {

        if(count == 0)
            ledOn = 1;
        else
            ledOn = 0;

        LED(LED_G, ledOn);

        if(count < 9)
            count++;
        else{
            count = 0;
            //motor_move(1000);
        }

        vTaskDelay(green100ms / portTICK_RATE_MS);

        if(resetCountdown > 0){
            resetCountdown--;
        }
        else if(resetCountdown == 0){
            UARTprintf("Begin reset!\n");
            currPos = -1;
            resetCountdown = -1;
        }
        		
    }
}

static void
_motorPoll( void *notUsed )
{

    while(true){

        motor_move(Pos[currPos]);
        if(currPos == -1){
            LED_R_flag = -1;
        }
        if(motor_status() == M_IDLE && currPos > -1){
            LED_R_flag = 0;
        }
        else
            LED_R_flag = 1;

        vTaskDelay(1 / portTICK_RATE_MS);

    }

}

static void
_LED_R_Poll( void *notUsed )
{
    uint32_t ledOn = 0;

    while(true){


        if(LED_R_flag == -1){
            GPIO_PORTF_DATA_R |= LED_R;
            vTaskDelay(50 / portTICK_RATE_MS);
        }
        else if(LED_R_flag == 0){
            GPIO_PORTF_DATA_R &= ~LED_R;
        }
        else{
            ledOn = !ledOn;
            LED(LED_R,ledOn);
        }

        vTaskDelay(50 / portTICK_RATE_MS);

    }

}

int main( void )
{
	
    motor_init();
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

    xTaskCreate(_motorPoll,
                "green",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 2,  // higher numbers are higher priority..
                NULL );

    xTaskCreate(_LED_R_Poll,
                "green",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 2,  // higher numbers are higher priority..
                NULL );

    /* Start the tasks and timer running. */
    vTaskStartScheduler();

    assert(0); // we should never get here..

    return 0;
}
