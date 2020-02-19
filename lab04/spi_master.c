/* Standard includes. */
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Tiva Hardware includes. */
#include "inc/tm4c123gh6pm.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
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


// the three tiva LED'a are attached to GPIO
// port F at pind 1, 2, 3
//#define LED_R (1<<1)
//#define LED_B (1<<2)
#define LED_G (1<<3)

//Moved Pins
#define SPI_CS (1<<2) //PF2

// output
#define SPI_RESET (1<<0) //PD0
//#define SPI_CS (1<<1) //PD1
#define SPI_SCK (1<<2) //PD2
#define SPI_SI (1<<3) //PD3

// input
#define SPI_SO (1<<6) //PD6

#define LED_ON(x) (GPIO_PORTF_DATA_R |= (x))
#define LED_OFF(x) (GPIO_PORTF_DATA_R &= ~(x))
#define LED(led,on) ((on)?LED_ON(led):LED_OFF(led))

#define eLED_TOGGLE(x) (expanderWrite(0x14, x))
#define eLED(on) ((on)?eLED_TOGGLE(0x01):eLED_TOGGLE(0x00))

uint32_t SystemCoreClock;

#ifdef USB_SERIAL_OUTPUT

#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
	_assert_failed ("__error__", pcFilename, ui32Line);
}
#endif

void setMOSI(uint8_t val)
{
    if (val - 0x80 == 0x00) // if one bit
        GPIO_PORTD_DATA_R |= SPI_SI;
    else // else if 0 bit
        GPIO_PORTD_DATA_R &= ~SPI_SI;
}

uint8_t getMISO(){
    if (GPIO_PORTD_DATA_R & SPI_SO)
        return 1;
    else
        return 0;
}

uint8_t transfer(uint8_t out)
{
    uint8_t count, in = 0;

    for (count = 0; count < 8; count++)
    {
        in <<= 1;
        setMOSI(out & 0x80);
        GPIO_PORTD_DATA_R |= SPI_SCK;
        in += getMISO();
        GPIO_PORTD_DATA_R &= ~SPI_SCK;
        out <<= 1;
    }
    setMOSI(0);

    return (in);
}

static uint8_t expanderRead(uint8_t address)
{
    uint8_t value, preRead = 0x41;

    GPIO_PORTF_DATA_R &= ~SPI_CS;
    transfer(preRead);
    transfer(address);
    value = transfer(0);
    GPIO_PORTF_DATA_R |= SPI_CS;

    return value;
}

static void expanderWrite(uint8_t address, uint8_t data)
{
    uint8_t value, preRead = 0x40;

    GPIO_PORTF_DATA_R &= ~SPI_CS;
    transfer(preRead);
    transfer(address);
    transfer(data);
    GPIO_PORTF_DATA_R |= SPI_CS;
}

static void expanderInit(){
	uint8_t IODIRA = 0x00;
	expanderWrite(IODIRA, 0xFE); // Sets GPIO A0 to output
}

//*****************************************************************************
//
//: Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
static void
_configureUART(void)
{
    //
    // Enable UART0
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);

}

#endif


static void
_setupHardware(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    // Enable the GPIO pin for the LED (PF3).  Set the direction as output, and
    // enable the GPIO pin for digital function.
    // These are TiveDriver library functions
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, (LED_G| SPI_CS));

    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, (SPI_RESET | SPI_SCK | SPI_SI));

    GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, SPI_SO);

    SystemCoreClock = 80000000;  // Required for FreeRTOS.

    SysCtlClockSet( SYSCTL_SYSDIV_2_5 |
                    SYSCTL_USE_PLL |
                    SYSCTL_XTAL_16MHZ |
                    SYSCTL_OSC_MAIN);

}

static void
_heartbeat( void *notUsed )
{
    uint32_t greenMs = 500 / portTICK_RATE_MS;
    uint32_t ledOn = 0;
    uint8_t address = 0x03;
    uint8_t data = 0x2F;
    uint8_t readVal;

    while(1)
    {
        ledOn = !ledOn;
        LED(LED_G, ledOn);

        vTaskDelay(greenMs);
    }

}

static void
_heartbeatExpand( void *notUsed )
{
    uint32_t greenMs = 500 / portTICK_RATE_MS;
    uint32_t eledOn = 0;
    uint8_t address = 0x03;
    uint8_t data = 0x2F;
    uint8_t readVal;

    while(1)
    {
        eledOn = !eledOn;

        eLED(eledOn);
        
        // if (ledOn)
        // 	expanderWrite(0x14, 0x01);
        // else
        // 	expanderWrite(0x14, 0x00);

        readVal = expanderRead(0x12);
        UARTprintf("readVal: %X\n", readVal);

        vTaskDelay(greenMs);
    }

}

int main( void )
{

    _setupHardware();


#ifdef USB_SERIAL_OUTPUT
	void spinDelayMs(uint32_t ms);
	_configureUART();
	spinDelayMs(1000);  // Allow UART to setup
#endif

    xTaskCreate(_heartbeat,
                "heartbeat",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY,
                NULL );

    xTaskCreate(_heartbeatExpand,
                "heartbeatExpand",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY,
                NULL );

    GPIO_PORTD_DATA_R &= ~SPI_RESET;
    GPIO_PORTD_DATA_R |= SPI_RESET;

    GPIO_PORTF_DATA_R |= SPI_CS;

    expanderInit();

    /* Start the tasks and timer running. */
    vTaskStartScheduler();

    assert(0); // we should never get here..

    return 0;
}
