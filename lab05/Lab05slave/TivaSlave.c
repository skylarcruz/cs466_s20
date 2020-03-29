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

#define SLA_RESET (1<<0) //PD0
#define SLA_CS (1<<1) //PD1
#define SLA_SI (1<<3) //PD3

#define SLA_SCK (1<<1) //PB1
#define SLA_SO (1<<0) //PB0

#define SLA_LED_B (1<<0)
#define SLA_LED_R (1<<1)
#define SLA_LED_G (1<<2)

#define SW_INT (1<<4) //PC4



#define LED_ON(x) (GPIO_PORTF_DATA_R |= (x))
#define LED_OFF(x) (GPIO_PORTF_DATA_R &= ~(x))
#define LED(led,on) ((on)?LED_ON(led):LED_OFF(led))

uint8_t LED_REG = 0x00; // 0b0000 0000; addr = 0x01
uint8_t SW_REG = 0x00; // 0b0000 0011; addr = 0x02
uint8_t INT_REG = 0x00; // 0b0000 0000; addr = 0x03

//QueueHandle_t xQueue = xQueueCreate(20, sizeof( uint8_t ) );
QueueHandle_t xQueue;

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
_watchdogHandler(void)
{
	
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	UARTprintf("Watchdog Interrupt Occured\n");

    uint32_t mask = WatchdogIntStatus(WATCHDOG0_BASE, 1);

    if (mask)
    {
        
    }

    if (xHigherPriorityTaskWoken)
    {
        // should request a schedule update....  
        // ....stay tuned
    }    

    WatchdogIntClear(WATCHDOG0_BASE);

	ResetISR();
}

static void
_interruptHandlerPortB(void)
{
	int m;
    //
    // We have not woken a task at the start of the ISR.
    //
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;


    uint32_t mask = GPIOIntStatus(GPIO_PORTB_BASE, 1);

    if (mask)
    {
        if(GPIO_PORTB_DATA_R & SLA_SCK){
        	m = 0x03;
        	xQueueSendFromISR( xQueue, &m, &xHigherPriorityTaskWoken );
        }
        else{
        	m = 0x04;
        	xQueueSendFromISR( xQueue, &m, &xHigherPriorityTaskWoken );
        }
    }

    if (xHigherPriorityTaskWoken)
    {
        // should request a schedule update....  
        // ....stay tuned
    }    

    GPIOIntClear(GPIO_PORTB_BASE, mask);
}

static void
_interruptHandlerPortD(void)
{
	int m;
    //
    // We have not woken a task at the start of the ISR.
    //
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;


    uint32_t mask = GPIOIntStatus(GPIO_PORTD_BASE, 1);

    if (mask)
    {
        if(GPIO_PORTD_DATA_R & SLA_CS){
        	m = 0x00;
        	xQueueSendFromISR( xQueue, &m, &xHigherPriorityTaskWoken );
        }
        else{
        	m = 0x01;
        	xQueueSendFromISR( xQueue, &m, &xHigherPriorityTaskWoken );
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
_interruptHandlerPortF(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    uint32_t mask = GPIOIntStatus(GPIO_PORTF_BASE, 1);

	//GPIO_PORTC_DATA_R |= SW_INT;

    if (mask & SW1)
    {
        
		// Set value of SW1 pressed state
		if(GPIO_PORTF_DATA_R & SW1)
		{
			UARTprintf("SW1 Depressed\n");
			SW_REG |= (1<<0);
		}
		else
		{
			UARTprintf("SW1 Pressed\n");
			SW_REG &= ~(1<<0);
		}

		// If SW1 interrupts enabled
		if(INT_REG & (1<<2))
		{
			SW_REG |= (1<<2);
			GPIO_PORTC_DATA_R |= SW_INT;

		}
		
    }

    if (mask & SW2)
    {
        
		// Set value of SW2 pressed state
	    if(GPIO_PORTF_DATA_R & SW2)
		{
			UARTprintf("SW2 Depressed\n");
			SW_REG |= (1<<1);
		}
		else
		{
			UARTprintf("SW2 Pressed\n");
			SW_REG &= ~(1<<1);
		}

		// If SW2 interrupts enabled
		if(INT_REG & (1<<3))
		{
			SW_REG |= (1<<3);
			GPIO_PORTC_DATA_R |= SW_INT;
		}

    }

    if (xHigherPriorityTaskWoken)
    {
        // should request a schedule update....  
        // ....stay tuned
    }   

	//SW_REG &= ~((1<<3) | (1<<2));
	GPIO_PORTC_DATA_R &= ~SW_INT;
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

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_WDOG0);

    // Enable the GPIO pin for the LED (PF3).  Set the direction as output, and
    // enable the GPIO pin for digital function.
    // These are TiveDriver library functions
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, (LED_G|LED_R|LED_B));
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, (SW1 | SW2) );
	GPIOPadConfigSet(GPIO_PORTF_BASE, (SW1 | SW2), GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, (SLA_RESET | SLA_CS | SLA_SI));
    GPIOPadConfigSet(GPIO_PORTD_BASE, (SLA_CS) , GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, (SLA_SCK));
    GPIOPadConfigSet(GPIO_PORTB_BASE, (SLA_SCK) , GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, (SLA_SO));

	GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, (SW_INT));

	// Watchdog Timer
	if(WatchdogLockState(WATCHDOG0_BASE) == true)
		WatchdogUnlock(WATCHDOG0_BASE);
	WatchdogReloadSet(WATCHDOG0_BASE, 0xFFFFFFF);
	WatchdogResetEnable(WATCHDOG0_BASE);
	WatchdogEnable(WATCHDOG0_BASE);
	WatchdogIntEnable(WATCHDOG0_BASE);

    SystemCoreClock = 80000000;  // Required for FreeRTOS.

    SysCtlClockSet( SYSCTL_SYSDIV_2_5 |
                    SYSCTL_USE_PLL |
                    SYSCTL_XTAL_16MHZ |
                    SYSCTL_OSC_MAIN);

}

void setMISO(uint8_t val)
{
    if (val - 0x80 == 0x00) // if one bit
        GPIO_PORTB_DATA_R |= SLA_SO;
    else // else if 0 bit
        GPIO_PORTB_DATA_R &= ~SLA_SO;
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
            if (jobFlag && GPIO_PORTB_DATA_R & SLA_SCK){
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
            if (jobFlag && GPIO_PORTB_DATA_R & ~SLA_SCK){
                state = 0;
                jobFlag = 0;
                count++;
            }
        }
    }

    //UARTprintf("in: %X\n", in);

    return in;

}

uint8_t transferSCKLow(uint8_t out, uint8_t in)
{
    setMISO(out & 0x80);
    //UARTprintf("bit out: %X\n", out & 0x80);
    out <<= 1;
    return out;
}

uint8_t transferSCKHigh(uint8_t out, uint8_t in)
{
	in += getMOSI();
	//UARTprintf("bit in: %x\n", in & 0x01);
	return in;
}

static void
_slave( void *pvParameters )
{
    enum State slaveState = IDLE;
    uint8_t dataIn1, dataIn2;

    GPIOIntRegister(GPIO_PORTD_BASE, _interruptHandlerPortD);
    GPIOIntTypeSet(GPIO_PORTD_BASE, SLA_CS, GPIO_BOTH_EDGES);

    IntPrioritySet(INT_GPIOD, 255);  // Required with FreeRTOS 10.1.1, 
    GPIOIntEnable(GPIO_PORTD_BASE, SLA_CS);

    GPIOIntRegister(GPIO_PORTB_BASE, _interruptHandlerPortB);
    GPIOIntTypeSet(GPIO_PORTB_BASE, SLA_SCK, GPIO_BOTH_EDGES);

    IntPrioritySet(INT_GPIOB, 255);  // Required with FreeRTOS 10.1.1, 
    GPIOIntEnable(GPIO_PORTB_BASE, SLA_SCK);

	GPIOIntRegister(GPIO_PORTF_BASE, _interruptHandlerPortF);
	GPIOIntTypeSet(GPIO_PORTF_BASE, SW1, GPIO_BOTH_EDGES);
    GPIOIntTypeSet(GPIO_PORTF_BASE, SW2, GPIO_BOTH_EDGES);

    IntPrioritySet(INT_GPIOF, 255);  // Required with FreeRTOS 10.1.1,
	GPIOIntEnable(GPIO_PORTF_BASE, SW1); 
    GPIOIntEnable(GPIO_PORTF_BASE, SW2);

    struct AMessage *pxRxedMessage;
    uint8_t val;

    uint8_t blank = 0x00;
    uint8_t dataOut = 0x00;
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
        	//UARTprintf("Val: %X\n", val);

		    // State Machine
		    // IDLE
		    // =====================
	        if (slaveState == IDLE){
	        	// CS Low
	        	if(val == 0x01){
		    		//UARTprintf("CS LOW\n");
		    		if(slaveState == IDLE){
		    			//UARTprintf("Command entered\n");
        				slaveState = COMMAND;
        				outCounter = 0;
        				in = 0;
        				blank = transferSCKLow(blank, in);
						command = 0x00;
    				}
        		}
	        }

	        // State Machine
		    // COMMAND
		    // ============================
		    else if(slaveState == COMMAND){
		    	// SCK High
        		if(val == 0x03){
        			in = transferSCKHigh(blank, in);
        			outCounter++;
    			
					  // End of 8 bit read/write
	    			  if (outCounter >= 8){
	    			  	command = (in & 0xF0) >> 4;
						//UARTprintf("Command: %X\n", command);
	    			  	addr = in & 0x0F;
	    			  	//UARTprintf("command: %X\n", command);
	    			  	if(command == 0x0b){
	    			  		if(addr == 0x01 || addr == 0x02 || addr == 0x03)
	    			  			slaveState = WRITE_REG;
	    			  		else
	    			  			slaveState = DONE; // Invalid Address
	    			  		//UARTprintf("Write entered\n");
	    			  	}
	    			  	else if (command == 0x0a){
	    			  		slaveState = READ_REG;
	    			  		//UARTprintf("Read entered\n");
	    			  		if (addr == 0x01)
	    			  			dataOut = LED_REG;
	    			  		else if (addr == 0x02)
	    			  			dataOut = SW_REG;
	    			  		else if (addr == 0x03)
	    			  			dataOut = INT_REG;
	    			  		else{
	    			  			slaveState = DONE;
	    			  			//UARTprintf("Error: invalid Address\n");
	    			  			//UARTprintf("Enterted Done State\n");
	    			  		}
	    			  	}
	    			  	else{
	    			  		slaveState = DONE;
	    			  		UARTprintf("error: Invalid Command\n");
	    			  		UARTprintf("Enterted Done State\n");
	    			  	}
	    			  	outCounter = 0;
	    			  	in = 0;
	    			  }
	    			  else
	    			  	in <<= 1;
	    		}
	    		// SCK Low
	    		else if(val == 0x04){
	    			blank = transferSCKLow(blank, in);
	    		}
	        }

	        // State Machine
		    // WRITE_REG
		    // ==============================
	        else if(slaveState == WRITE_REG){
	        	// SCK goes high
		    	if(val == 0x03){
	    			in = transferSCKHigh(blank, in);
	    			outCounter++;

	    			// begin write to register
	    			if (outCounter >= 8){
	    				if(addr == 0x01)
	    					LED_REG = in;
	    				else if (addr == 0x02)
	    					SW_REG = in;
	    				else if (addr == 0x03){
							if(in & (1<<0)){
								UARTprintf("SW1 interrupt Acknowledged\n");
								in = in &= ~(1<<0);
								SW_REG &= ~(1<<2);
							}
							if(in & (1<<1)){
								UARTprintf("SW2 interrupt Acknowledged\n");
								in = in &= ~(1<<1);
								SW_REG &= ~(1<<3);
							}
	    					INT_REG = in;
						}
	    				outCounter = 0;
	    				in = 0;
	    				slaveState = DONE;
	    			}
	    			// Left shift for next bit
	    			else
	    				in <<= 1;
	    		}
	    		// SCK Low
	    		else if(val == 0x04){
	    			blank = transferSCKLow(blank, in);
	    		}
	        }

	        // State Machine
		    // READ_REG
		    // =============================
	        else if(slaveState == READ_REG){
	        	// SCK goes high
		    	if(val == 0x03){
	    			//in = transferSCKHigh(dataOut, in);
	    			outCounter++;

	    			// begin write to register
	    			if (outCounter >= 8){
	    				outCounter = 0;
	    				in = 0;
	    				slaveState = DONE;
	    			}
	    			// Left shift for next bit
	    			else
	    				in <<= 1;
	    		}
	    		// SCK Low
	    		else if(val == 0x04){
	    			dataOut = transferSCKLow(dataOut, in);
	    		}
	    		else if(val == 0x00){
	    			//UARTprintf("Error: CS went High\n");
	    			slaveState = DONE;
	    		}
	        }


	        // State Machine
		    // DONE
		    // ============================
	        else if(slaveState == DONE){
	        	// CS HIGH
	        	if(val == 0x00){
	        		slaveState = IDLE;
	        		//UARTprintf("Entered Idle State\n");
	    	    }
    	    }
	    }

        //enum State {IDLE, COMMAND, READ_REG, WRITE_REG, DONE};

    } // end of while
}

static void
_intPoll( void *pvParameters )
{

	int currCS = ~GPIO_PORTD_DATA_R & SLA_CS;
	int currSCK = GPIO_PORTB_DATA_R & SLA_SCK;
	int newCS;
	int newSCK;

	//UARTprintf("CurrCS: %X\n", currCS);

	uint8_t m = 0x00;

    {
    	while(1)
    	{
    		newCS = GPIO_PORTD_DATA_R & SLA_CS;
    		newSCK = GPIO_PORTB_DATA_R & SLA_SCK;

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
    		vTaskDelay((1 / portTICK_RATE_MS));

        }
    }

}

static void
_setLEDs( void *notUsed ){
	while(1){
		if(LED_REG & (1<<0))
			GPIO_PORTF_DATA_R |= LED_B;
		else
			GPIO_PORTF_DATA_R &= ~LED_B;

		if(LED_REG & (1<<1))
			GPIO_PORTF_DATA_R |= LED_R;
		else
			GPIO_PORTF_DATA_R &= ~LED_R;

		if(LED_REG & (1<<2))
			GPIO_PORTF_DATA_R |= LED_G;
		else
			GPIO_PORTF_DATA_R &= ~LED_G;

        //UARTprintf("LED: %X\n", GPIO_PORTF_DATA_R);

		//vTaskDelay(10);
	}
}

static void
_heartbeat( void *notUsed )
{
    uint32_t green500ms = 500; // 1 second
    uint32_t ledOn = 0;
	
	//WatchdogIntEnable(WATCHDOG0_BASE);
	WatchdogIntRegister(WATCHDOG0_BASE, _watchdogHandler);
	WatchdogIntTypeSet(WATCHDOG0_BASE, WATCHDOG_INT_TYPE_INT);

    while(true)
    {
        ledOn = !ledOn;
        LED(LED_G, ledOn);
		
        
        vTaskDelay(green500ms / portTICK_RATE_MS);
		UARTprintf("Watchdog Val: %x\n", WatchdogValueGet(WATCHDOG0_BASE));
		WatchdogReloadSet(WATCHDOG0_BASE, 0xFFFFFFF);
    }
}

int main( void )
{
	//QueueHandle_t xQueue;

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
                tskIDLE_PRIORITY + 1,  // higher numbers are higher priority..
                NULL );

    xTaskCreate(_setLEDs,
                "LED",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY,  // higher numbers are higher priority..
                NULL );

    xQueue = xQueueCreate(20, sizeof( uint8_t ) );

    // xTaskCreate(_intPoll,
    //             "intPoll",
    //             configMINIMAL_STACK_SIZE,
    //             (void *) xQueue,
    //             tskIDLE_PRIORITY + 2,
    //             NULL );

    xTaskCreate(_slave,
                "slave",
                configMINIMAL_STACK_SIZE,
                (void *) xQueue,
                tskIDLE_PRIORITY + 3,  // higher numbers are higher priority..
                NULL );


    /* Start the tasks and timer running. */
    vTaskStartScheduler();

    assert(0); // we should never get here..

    return 0;
}
