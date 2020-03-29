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
#define LED_R (1<<1) // PF1
#define LED_B (1<<2) // PF2
#define LED_G (1<<3) // PF3
#define SPI_SWI (1<<4) // PF4

#define SLA_LED_B (1<<0)
#define SLA_LED_R (1<<1)
#define SLA_LED_G (1<<2)

//Moved Pins
//#define SPI_CS (1<<2) //PF2

// output
#define SPI_RESET (1<<0) //PD0
#define SPI_CS (1<<1) //PD1
#define SPI_SCK (1<<2) //PD2
#define SPI_SI (1<<3) //PD3

// input
#define SPI_SO (1<<6) //PD6

// interrupt
#define SPI_INT (1<<0) //PB0

// SW interrupt Tiva Slave
#define SW_INT (1<<4) //PC4

#define LED_ON(x) (GPIO_PORTF_DATA_R |= (x))
#define LED_OFF(x) (GPIO_PORTF_DATA_R &= ~(x))
#define LED(led,on) ((on)?LED_ON(led):LED_OFF(led))

#define rLED_TOGGLE(x) (byteWrite(0x14, x))
#define rLED(on) ((on)?rLED_TOGGLE(0x01):rLED_TOGGLE(0x00))

uint32_t SystemCoreClock;


#ifdef USB_SERIAL_OUTPUT

#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
	_assert_failed ("__error__", pcFilename, ui32Line);
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

void setMOSI(uint8_t val)
{
    if (val - 0x80 == 0x00) // if one bit
        GPIO_PORTD_DATA_R |= SPI_SI;
    else // else if 0 bit
        GPIO_PORTD_DATA_R &= ~SPI_SI;
    //UARTprintf("MOSI Value = %d\n", (val & 0x80));
}

uint8_t getMISO(){
    if (GPIO_PORTD_DATA_R & SPI_SO)
    {
    //UARTprintf("MISO value = 1\n");
    return 1;
    }
    else
    {
	//UARTprintf("MISO value = 0\n");
        return 0;
    }
}

uint8_t transfer(uint8_t out)
{
    uint8_t count, in = 0;
    //int MsCount = 1/portTICK_RATE_MS;
    int MsCount = 25;

    //int intdelay = MsCount/portTICK_RATE_MS/2;

    for (count = 0; count < 8; count++)
    {
        in <<= 1;
        setMOSI(out & 0x80);
	//UARTprintf("MOSI = %d\n",(out & 0x80));

	//vTaskDelay(MsCount);
    delayMs(MsCount);
	
	GPIO_PORTD_DATA_R |= SPI_SCK;
	
        in += getMISO();
	//UARTprintf("MISO = %d\n",(in & 0x01));
	//vTaskDelay(MsCount);
    delayMs(MsCount);
	
	GPIO_PORTD_DATA_R &= ~SPI_SCK;
        out <<= 1;
    }
    //setMOSI(0);

    return (in);
}

static uint8_t byteRead(uint8_t address)
{
    uint8_t firstByte, value, command = 0xA0;
    int MsCount = 25;

    firstByte = command | address;		
    
    GPIO_PORTD_DATA_R &= ~SPI_CS;    //add 10ms delays in read function too?
    //UARTprintf("CS LOW\n");

    //vTaskDelay(MsCount/portTICK_RATE_MS);
    delayMs(MsCount);

    transfer(firstByte);

    //vTaskDelay(10/portTICK_RATE_MS);
    delayMs(MsCount);

    value = transfer(0);

    //vTaskDelay(MsCount/portTICK_RATE_MS);
    GPIO_PORTD_DATA_R |= SPI_CS;
    //UARTprintf("CS HIGH\n");

    return (value);
}

static void byteWrite(uint8_t address, uint8_t data)		//write command is 0xb_
{
    uint8_t firstByte, command = 0xB0;
    int MsCount = 25; 
    uint8_t value;

    firstByte =  command | address;

    //vTaskDelay(MsCount/portTICK_RATE_MS);
    delayMs(MsCount);

    GPIO_PORTD_DATA_R &= ~SPI_CS;
    //UARTprintf("CS LOW\n");
    
    transfer(firstByte);
    
    //vTaskDelay(MsCount/portTICK_RATE_MS);
    delayMs(MsCount);

    transfer(data);
    
    
    //vTaskDelay(MsCount/portTICK_RATE_MS);
    delayMs(MsCount);

    GPIO_PORTD_DATA_R |= SPI_CS;
    //UARTprintf("CS HIGH\n");
    
}

static void
_interruptHandlerPortC(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint8_t currSW_REG;
    uint8_t currINT_REG;

    uint32_t mask = GPIOIntStatus(GPIO_PORTC_BASE, 1);

    UARTprintf("Interrupt from slave received\n");

    GPIO_PORTD_DATA_R |= SPI_CS;
    byteRead(0xFF); // FLUSH COMMAND
    delayMs(5);

    if (mask)
    {
        currSW_REG = byteRead(0x02);
        UARTprintf("currSW_REG: %X\n", currSW_REG);
        //delayMs(5);

        if(currSW_REG & (1<<2)) // SW1 caused interrupt
        {
            currINT_REG = byteRead(0x03); // Get current enabled INTs
            delayMs(10);
            UARTprintf("SW1: Writing to INT_REG: %X\n", currINT_REG | (1<<0));
            byteWrite(0x03, (currINT_REG | (1<<0))); // Acknowledge SW1
            delayMs(5);
        }

        if(currSW_REG & (1<<3)) // SW2 caused interrupt
        {
            currINT_REG = byteRead(0x03); // Get current enabled INTs
            delayMs(10);
            UARTprintf("SW2: Writing to INT_REG: %X\n", currINT_REG | (1<<1));
            byteWrite(0x03, (currINT_REG | (1<<1))); // Acknowledge SW2
            delayMs(5);
        }

    }

    if (xHigherPriorityTaskWoken)
    {
        // should request a schedule update....  
        // ....stay tuned
    }    

    GPIOIntClear(GPIO_PORTC_BASE, mask);
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

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

    // Enable the GPIO pin for the LED (PF3).  Set the direction as output, and
    // enable the GPIO pin for digital function.
    // These are TiveDriver library functions
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, (LED_G | LED_B | LED_R));
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, (SPI_SWI));

    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, (SPI_RESET | SPI_CS | SPI_SCK | SPI_SI));
    GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, SPI_SO);

    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, (SPI_INT));
    GPIOPadConfigSet(GPIO_PORTB_BASE, SPI_INT, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, SW_INT);
    GPIOPadConfigSet(GPIO_PORTC_BASE, SW_INT, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

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
    
    GPIOIntRegister(GPIO_PORTC_BASE, _interruptHandlerPortC);
    GPIOIntTypeSet(GPIO_PORTC_BASE, SW_INT, GPIO_RISING_EDGE);

    IntPrioritySet(INT_GPIOC, 255);  // Required with FreeRTOS 10.1.1, 
    GPIOIntEnable(GPIO_PORTC_BASE, SW_INT);

    while(1)
    {
	    	
    	ledOn = !ledOn;
        LED(LED_G, ledOn);
    	
        vTaskDelay(greenMs);
    }

}


static void
_RedHeartbeat( void *notUsed )
{
    uint32_t redMs = 1 / portTICK_RATE_MS;
    uint32_t rledOn = 0;

    uint8_t slaveRed = 0x00;
    uint8_t SlaveData;
    
    byteWrite(0xFF,0xFF); // Flush prev slave action
    vTaskDelay(redMs);
    byteWrite(0x03,(1<<2 | 1<<3));   //enable SW1 Interrupt
    vTaskDelay(redMs);
 
    while(1)
    {
        rledOn = !rledOn;
	//UARTprintf("rledOn= %d\n",rledOn);
        LED(LED_R,rledOn);

    

	if(rledOn == 1)
	{
		//slaveRed = 0x0F;	//0x0000.0100 for RED led
	 	byteWrite(0x01, (SLA_LED_B | SLA_LED_R));
        vTaskDelay(redMs);
  		SlaveData = byteRead(0x03);

		//UARTprintf("INT_REG = %x\n", SlaveData);
	}
	else 
	{
		//slaveRed = 0x0F;
		byteWrite(0x01,0x00);
		vTaskDelay(redMs);
		SlaveData = byteRead(0x03);
		//UARTprintf("INT_REG = %x\n", SlaveData);
	}
	
	//byteWrite(0x06,slaveRed); 

	//delayMs(1);

    vTaskDelay(redMs);
    }

}

static void
_IntPoll( void *notUsed )
{

}

int main( void )
{

    _setupHardware();


#ifdef USB_SERIAL_OUTPUT
	void spinDelayMs(uint32_t ms);
	_configureUART();
	spinDelayMs(1000);  // Allow UART to setup
	UARTprintf("RESET\n");
#endif

    GPIO_PORTD_DATA_R &= ~SPI_RESET;
    GPIO_PORTD_DATA_R |= SPI_RESET;

    GPIO_PORTD_DATA_R |= SPI_CS;

    xTaskCreate(_heartbeat,
                "heartbeat",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY,
                NULL );

    
     xTaskCreate(_RedHeartbeat,
                 "RedHeartbeat",
                 configMINIMAL_STACK_SIZE,
                 NULL,
                 tskIDLE_PRIORITY+1,
                 NULL );

    /* Start the tasks and timer running. */
    vTaskStartScheduler();

    assert(0); // we should never get here..

    return 0;
}
