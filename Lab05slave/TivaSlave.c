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
#define LED_G (1<<3)
#define LED_B (1<<2)
#define SW1   (1<<4)
#define SW2   (1<<0)

#define SLA_RESET (1<<0) //PD0
#define SLA_CS (1<<1) //PD1
#define SLA_SCK (1<<2) //PD2
#define SLA_SI (1<<3) //PD3

#define SLA_SO (1<<6) //PD6


#define LED_ON(x) (GPIO_PORTF_DATA_R |= (x))
#define LED_OFF(x) (GPIO_PORTF_DATA_R &= ~(x))
#define LED(led,on) ((on)?LED_ON(led):LED_OFF(led))

enum State {IDLE, COMMAND, READ_REG, WRITE_REG, DONE};

static SemaphoreHandle_t _semBtn = NULL;

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
    //
    // We have not woken a task at the start of the ISR.
    //
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;


    uint32_t mask = GPIOIntStatus(GPIO_PORTD_BASE, 1);

    if (mask)
    {
        // code
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

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    // Enable the GPIO pin for the LED (PF3).  Set the direction as output, and
    // enable the GPIO pin for digital function.
    // These are TiveDriver library functions
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, (LED_G|LED_R|LED_B));
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, SW1 );


    GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, (SLA_RESET | SLA_CS | SLA_SCK | SLA_SI));
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, (SLA_SO));
    GPIOPadConfigSet(GPIO_PORTD_BASE, (SLA_CS| SLA_SCK) , GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);



    //
    // Set weak pull-up for switchs
    // This is a TiveDriver library function
    //
    GPIOPadConfigSet(GPIO_PORTF_BASE, SW1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);


    //
    // Set the clocking to run at (SYSDIV_2_5) 80.0 MHz from the PLL.
    //                            (SYSDIV_3) 66.6 MHz
    //                            (SYSDIV_4) 50.0 MHz
    //                            (SYSDIV_5) 40.0 MHz
    //                            (SYSDIV_6) 33.3 MHz
    //                            (SYSDIV_8) 25.0 MHz
    //                            (SYSDIV_10) 20.0 MHz
    //
    SystemCoreClock = 80000000;  // Required for FreeRTOS.

    SysCtlClockSet( SYSCTL_SYSDIV_2_5 |
                    SYSCTL_USE_PLL |
                    SYSCTL_XTAL_16MHZ |
                    SYSCTL_OSC_MAIN);

}

void setMISO(uint8_t val)
{
    if (val - 0x80 == 0x00) // if one bit
        GPIO_PORTD_DATA_R |= SLA_SO;
    else // else if 0 bit
        GPIO_PORTD_DATA_R &= ~SLA_SO;
}

uint8_t getMOSI(){
    if (GPIO_PORTD_DATA_R & SLA_SI)
        return 1;
    else
        return 0;
}

uint8_t transferSlave(uint8_t out)
{
    uint8_t count = 0;
    uint8_t in = 0;

    int state = 0; // 0 = clock low; 1 = clock high
    int jobFlag = 0; // makes sure jobs complete

    while(count < 8){

        // SCK LOW
        if(state == 0){
            if(jobFlag == 0){
                in <<= 1;
                setMISO(out & 0x80);
                jobFlag = 1;
            }
            if (jobFlag && GPIO_PORTD_DATA_R & SLA_SCK){
                state = 1;
                jobFlag = 0;
            }
        }

        // SCK HIGH
        if(state == 1){
            if(jobFlag == 0){
                in += getMOSI();
                out <<= 1;
                jobFlag = 1;
            }
            if (jobFlag && GPIO_PORTD_DATA_R & ~SLA_SCK){
                state = 0;
                jobFlag = 0;
                count++;
            }
        }
    }

    UARTprintf("in: %X\n", in);

    return in;

}

uint8_t transferSCKLow(uint8_t out, uint8_t in)
{
    setMISO(out & 0x80);
    out <<= 1;
    return out;
}

uint8_t transferSCKHigh(uint8_t out, uint8_t in)
{
	in += getMOSI();
	UARTprintf("bit in: %x\n", in & 0x01);
	return in;
}

static void
_slave( void *pvParameters )
{
    enum State slaveState = IDLE;
    uint8_t dataIn1, dataIn2;
    uint8_t dataOut = 0xce;

    GPIOIntRegister(GPIO_PORTD_BASE, _interruptHandlerPortD);
    GPIOIntTypeSet(GPIO_PORTD_BASE, SLA_CS, GPIO_FALLING_EDGE);

    IntPrioritySet(INT_GPIOD, 255);  // Required with FreeRTOS 10.1.1, 
    GPIOIntEnable(GPIO_PORTD_BASE, SLA_CS);

    struct AMessage *pxRxedMessage;
    uint8_t val;

    uint8_t blank = 0x00;
    uint8_t in = 0;
    uint8_t addr;
    uint8_t command;
    uint8_t data;
    int outCounter = 0;

    while(1){

        // CS and SCK Handler
        if(( xQueueReceive( ((QueueHandle_t)pvParameters), &( pxRxedMessage ), ( TickType_t ) 10 ) ) == pdTRUE)
        {
        	val = (int) pxRxedMessage & 0xFF;
        	//UARTprintf("Int Code: %X\n", val);


		    if(val == 0x00){

		    }
		    else if(val == 0x01){
		    	if(slaveState == IDLE){
		    		UARTprintf("Command entered\n");
        			slaveState = COMMAND;
        			blank = transferSCKLow(blank, in);
        		}
		    }

		    // SCK goes high
		    else if(val == 0x03){
		    	if(slaveState == COMMAND){
        			in = transferSCKHigh(blank, in);
        			in <<= 1;
        			outCounter++;
    			
					  // End of 8 bit read/write
	    			  if (outCounter == 8){
	    			  	command = (in & 0xF0) >> 4;
	    			  	addr = in & 0x0F;
	    			  	if(command == 0x0b){
	    			  		slaveState = WRITE_REG;
	    			  		UARTprintf("Write entered\n");
	    			  	}
	    			  	else if (command == 0x0a){
	    			  		slaveState = READ_REG;
	    			  		UARTprintf("Read entered\n");
	    			  	}
	    			  	else{
	    			  		slaveState = IDLE;
	    			  		UARTprintf("error: reset to IDLE\n");
	    			  		UARTprintf("in full: %X\n", in & 0xFF);
	    			  		in = 0;
	    			  	}
	    			  	outCounter = 0;
	    			  }
	    		 }
		    }

		    

		    // SCK goes Low
		    else if(val == 0x04){
		    	if(slaveState == COMMAND)
        			blank = transferSCKLow(blank, in);
		    }
	    }

        //enum State {IDLE, COMMAND, READ_REG, WRITE_REG, DONE};

        // State Machine
        if (slaveState == IDLE){

        }
        else if(slaveState == COMMAND){

        }
        else if(slaveState == READ_REG){
        	
        }
        else if(slaveState == WRITE_REG){
        	
        }
        else if(slaveState == DONE){
        	
        }

    } // end of while
}

static void
_intPoll( void *pvParameters )
{

	int currCS = ~GPIO_PORTD_DATA_R & SLA_CS;
	int currSCK = GPIO_PORTD_DATA_R & SLA_SCK;
	int newCS;
	int newSCK;

	UARTprintf("CurrCS: %X\n", currCS);

	uint8_t m = 0x00;

    {
    	while(1)
    	{
    		newCS = GPIO_PORTD_DATA_R & SLA_CS;
    		newSCK = GPIO_PORTD_DATA_R & SLA_SCK;

    		// if CS is low
    		if(currCS == 0){
    			// and then it goes high
    			if(currCS != newCS){
    				currCS = newCS;
    				m = 0x00;
    				xQueueSend( ((QueueHandle_t)pvParameters), (void *) &m, ( TickType_t ) 10 );
    			}
    		}
    		// if CS is high
    		else if(currCS == 2){
    			// and then it goes low
    			if(currCS != newCS){
    				currCS = newCS;
    				m = 0x01;
    				xQueueSend( ((QueueHandle_t)pvParameters), (void *) &m, ( TickType_t ) 10 );
    			}
    		}

    		// if CLK is low
    		if(currSCK == 0){
    			// and then it goes high
    			if(currSCK != newSCK){
    				currSCK = newSCK;
    				m = 0x03;
    				xQueueSend( ((QueueHandle_t)pvParameters), (void *) &m, ( TickType_t ) 10 );
    			}
    		}
    		// if CS is high
    		else if(currSCK == 4){
    			// and then it goes low
    			if(currSCK != newSCK){
    				currSCK = newSCK;
    				m = 0x04;
    				xQueueSend( ((QueueHandle_t)pvParameters), (void *) &m, ( TickType_t ) 10 );
    			}
    		}

    		//UARTprintf("Test: %X\n", GPIO_PORTD_DATA_R & SLA_CS);
    		vTaskDelay(1 / portTICK_RATE_MS);

        }
    }

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
        
        vTaskDelay(green500ms / portTICK_RATE_MS);
    }
}

int main( void )
{
	QueueHandle_t xQueue;

    _setupHardware();

#ifdef USB_SERIAL_OUTPUT
    void spinDelayMs(uint32_t ms);
    _configureUART();
    spinDelayMs(1000);  // Allow UART to setup
    UARTprintf("Slave_Start\n");
#endif

    xTaskCreate(_heartbeat,
                "green",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY,  // higher numbers are higher priority..
                NULL );

    xQueue = xQueueCreate(20, sizeof( uint8_t ) );

    xTaskCreate(_intPoll,
                "intPoll",
                configMINIMAL_STACK_SIZE,
                (void *) xQueue,
                tskIDLE_PRIORITY + 3,
                NULL );

    xTaskCreate(_slave,
                "slave",
                configMINIMAL_STACK_SIZE,
                (void *) xQueue,
                tskIDLE_PRIORITY + 2,  // higher numbers are higher priority..
                NULL );


    /* Start the tasks and timer running. */
    vTaskStartScheduler();

    assert(0); // we should never get here..

    return 0;
}
