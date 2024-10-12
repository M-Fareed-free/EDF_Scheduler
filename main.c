/*
 * FreeRTOS Kernel V10.2.0
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/*
	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used.
*/


/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the demo application tasks.
 *
 * Main.c also creates a task called "Check".  This only executes every three
 * seconds but has the highest priority so is guaranteed to get processor time.
 * Its main function is to check that all the other tasks are still operational.
 * Each task (other than the "flash" tasks) maintains a unique count that is
 * incremented each time the task successfully completes its function.  Should
 * any error occur within such a task the count is permanently halted.  The
 * check task inspects the count of each task to ensure it has changed since
 * the last time the check task executed.  If all the count variables have
 * changed all the tasks are still executing error free, and the check task
 * toggles the onboard LED.  Should any task contain an error at any time
 * the LED toggle rate will change from 3 seconds to 500ms.
 *
 */

/* Standard includes. */
#include <stdlib.h>
#include <stdio.h>
#include <time.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "lpc21xx.h"
#include "semphr.h"
#include "event_groups.h"

/* Peripheral includes. */
#include "serial.h"
#include "GPIO.h"


/*-----------------------------------------------------------*/

/* Constants to setup I/O and processor. */
#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )

/* Constants for the ComTest demo application tasks. */
#define mainCOM_TEST_BAUD_RATE	( ( unsigned long ) 115200 )


/*
 * Configure the processor for use with the Keil demo board.  This is very
 * minimal as most of the setup is managed by the settings in the project
 * file.
 */
static void prvSetupHardware( void );
void LED_Task( void * pvParameters );
void Button_Task( void * pvParameters );
void Button_1_Monitor(void * pvParameters);
void vApplicationIdleHook( void );

/***** Project (2) *****/
TaskHandle_t task_1_Handler = NULL;
TaskHandle_t task_2_Handler = NULL;
TaskHandle_t task_3_Handler = NULL;
TaskHandle_t task_4_Handler = NULL;
TaskHandle_t task_5_Handler = NULL;
TaskHandle_t task_6_Handler = NULL;

/*Button events defnitiion*/
#define BUTTON_1_BIT_0_RISING   		(1UL << 0UL) // zero shift for bit0
#define BUTTON_1_BIT_1_FALLING   		(1UL << 1UL) // 1 shift for flag  bit 1
#define BUTTON_2_BIT_2_RISING   		(1UL << 2UL) // zero shift for bit0
#define BUTTON_2_BIT_3_FALLING   		(1UL << 3UL) // 1 shift for flag  bit 1
#define BUTTON_1_DELAY							(50)
#define BUTTON_2_DELAY							(50)
#define UART_TX_DELAY								(100)
#define UART_RX_DELAY								(20)
#define LOAD_1_DELAY								(10)
#define LOAD_2_DELAY								(100)

#define UART_MSG_SIZE								(4)
#define UART_MAX_NO_OF_MESSAGES			(2)


struct UartQueueStruct
{
   char ucMessageID;
   char* ucData;
} UartQueue;

QueueHandle_t xStructQueue = NULL;

static int MainTickCounts = 0;
char runTimeStatsBuf[250];
volatile int Button_1_DeadLineMiss;
volatile int Button_2_DeadLineMiss;
volatile int UartTxDeadLineMiss;
volatile int UartRxDeadLineMiss;
volatile int Load_1_DeadlineMiss;
volatile int Load_2_DeadlineMiss;


int Button_1_ExecTimeMsec = 0;
int Button_2_ExecTimeMsec = 0;
int UartTxExecTimeMsec;
int UartRxExecTimeMsec;
int Load_1_ExecTimeMsec;
int Load_2_ExecTimeMsec;
/*-----------------------------------------------------------*/
/* Task to be created. */
void Button_1_Monitor(void * pvParameters)
{

	TickType_t xLastWakeTime = xTaskGetTickCount();
	static pinState_t previous_state = PIN_IS_HIGH;
	static pinState_t button_1_State;
	int startTime = 0;
	int endTime = 0;

	for(;;)
	{
		button_1_State = GPIO_read(PORT_0, PIN0);

		if( (button_1_State == PIN_IS_HIGH) && (previous_state == PIN_IS_LOW) ) /*Rising*/
		{
			previous_state = PIN_IS_HIGH;

			UartQueue.ucMessageID = 1;
			UartQueue.ucData = "Button_1_Rising\n";
			xQueueSend( xStructQueue, ( void * ) &UartQueue,( TickType_t ) 0 );

		}
		else if((button_1_State == PIN_IS_LOW) && (previous_state == PIN_IS_HIGH))/*Falling*/
		{
			previous_state = PIN_IS_LOW;
			UartQueue.ucMessageID = 1;
			UartQueue.ucData = "Button_1_Falling\n";
			xQueueSend( xStructQueue,( void * ) &UartQueue, ( TickType_t ) 0 );
		}else{
			/*UartQueue.ucMessageID = 1;
			UartQueue.ucData = "Button_1_None\n";
			xQueueSend( xStructQueue,( void * ) &UartQueue, ( TickType_t ) 0 );*/
		}
		endTime = xTaskGetTickCount();
		Button_1_ExecTimeMsec  = endTime - startTime;
		if( Button_1_ExecTimeMsec > BUTTON_1_DELAY)
		{
			Button_1_DeadLineMiss++;
		}
		GPIO_write(PORT_0, PIN2, PIN_IS_LOW);
		vTaskDelayUntil( &xLastWakeTime, BUTTON_1_DELAY);
		GPIO_write(PORT_0, PIN2, PIN_IS_HIGH);

		startTime = xTaskGetTickCount();

		GPIO_write(PORT_1,PIN0, PIN_IS_LOW); //IDLE task
	}//loop
}


void Button_2_Monitor(void * pvParameters)
{
	static pinState_t button_2_State;
	static pinState_t previous_state = PIN_IS_HIGH;
	int startTime = 0;
	int endTime = 0;

	TickType_t xLastWakeTime = xTaskGetTickCount();
	for(;;)
	{
		button_2_State = GPIO_read(PORT_0, PIN1);
		if( (button_2_State == PIN_IS_HIGH) && (previous_state == PIN_IS_LOW) ) /*Rising*/
		{
			previous_state = PIN_IS_HIGH;
			UartQueue.ucMessageID = 2;
			UartQueue.ucData = "Button_2_Rising\n";
			xQueueSend( xStructQueue, ( void * ) &UartQueue,( TickType_t ) 0 );
		}
		else if((button_2_State == PIN_IS_LOW) && (previous_state == PIN_IS_HIGH))/*Falling*/
		{
			previous_state = PIN_IS_LOW;
			UartQueue.ucMessageID = 2;
			UartQueue.ucData = "Button_2_Falling\n";
			xQueueSend( xStructQueue,( void * ) &UartQueue, ( TickType_t ) 0 );
		}else{
			//button_2_message = NULL;
		}
		endTime = xTaskGetTickCount();
		Button_2_ExecTimeMsec = endTime - startTime;
		if( Button_2_ExecTimeMsec > BUTTON_2_DELAY)
		{
			Button_2_DeadLineMiss++;
		}
		GPIO_write(PORT_0, PIN3, PIN_IS_LOW);
		vTaskDelayUntil( &xLastWakeTime, BUTTON_2_DELAY);
		GPIO_write(PORT_0, PIN3, PIN_IS_HIGH);
		startTime = xTaskGetTickCount();

		GPIO_write(PORT_1,PIN0, PIN_IS_LOW); //IDLE task
	}//loop
}

void Periodic_Transmitter(void * pvParameters)
{
	int startTime = 0;
	int endTime = 0;
	TickType_t xLastWakeTime = xTaskGetTickCount();

	for(;;)
	{
		UartQueue.ucMessageID = 0;
		UartQueue.ucData = "Periodic Tx\n";
		xQueueSend( xStructQueue, ( void * ) &UartQueue, ( TickType_t ) 0 );

		endTime = xTaskGetTickCount();
		UartTxExecTimeMsec = endTime - startTime;
		if( UartTxExecTimeMsec > UART_TX_DELAY)
		{
			UartTxDeadLineMiss++;
		}

		GPIO_write(PORT_0, PIN4, PIN_IS_LOW);
		vTaskDelayUntil( &xLastWakeTime, UART_TX_DELAY);
		GPIO_write(PORT_0, PIN4, PIN_IS_HIGH);
		startTime = xTaskGetTickCount();

		/*run-time Analysis*/
		/*vTaskGetRunTimeStats(runTimeStatsBuf);
		vSerialPutString(runTimeStatsBuf,250);
		xSerialPutChar('\n');*/

		GPIO_write(PORT_1,PIN0, PIN_IS_LOW); //IDLE task
	}//loop
}


void Uart_Receiver(void * pvParameters)
{
	int i;

	static signed char tx_rx_buffer[25] ={0};
	static signed char Button_1_rx_buffer[25] = {0};
	static signed char Button_2_rx_buffer[25] = {0};
	struct UartQueueStruct xRxedStructure;
	int startTime = 0;
	int endTime = 0;

	TickType_t xLastWakeTime = xTaskGetTickCount();

	for(;;)
	{
   if( xStructQueue != NULL )
   {
      /* Receive a message from the created queue to hold complex struct UartQueueStruct
      structure.  Block for 10 ticks if a message is not immediately available.
      The value is read into a struct UartQueueStruct variable, so after calling
      xQueueReceive() xRxedStructure will hold a copy of UartQueue. */
      if( xQueueReceive( xStructQueue, &( xRxedStructure ),( TickType_t ) 0 ) == pdPASS )
      {
         /* xRxedStructure now contains a copy of UartQueue. */
				if(xRxedStructure.ucMessageID == 0)
				{
					for(i = 0; i < strlen(xRxedStructure.ucData); i++)
					{
						tx_rx_buffer[i] = xRxedStructure.ucData[i];
					}
					vSerialPutString(tx_rx_buffer, strlen(xRxedStructure.ucData));
				}

				if(xRxedStructure.ucMessageID == 1)
				{

					for(i = 0; i < strlen(xRxedStructure.ucData); i++)
					{
						Button_1_rx_buffer[i] = xRxedStructure.ucData[i];
					}

					vSerialPutString(Button_1_rx_buffer, strlen(xRxedStructure.ucData));
				}

				if(xRxedStructure.ucMessageID == 2)
				{

					for(i = 0; i < strlen(xRxedStructure.ucData); i++)
					{
						Button_2_rx_buffer[i] = xRxedStructure.ucData[i];
					}

					vSerialPutString(Button_2_rx_buffer, strlen(xRxedStructure.ucData));
				}
			}
		}
		endTime = xTaskGetTickCount();
		UartRxExecTimeMsec = endTime - startTime;
		if( UartRxExecTimeMsec > UART_RX_DELAY)
		{
			UartRxDeadLineMiss++;
		}
		GPIO_write(PORT_0, PIN5, PIN_IS_LOW);
		vTaskDelayUntil( &xLastWakeTime, UART_RX_DELAY);
		GPIO_write(PORT_0, PIN5, PIN_IS_HIGH);
		startTime = xTaskGetTickCount();

		GPIO_write(PORT_1,PIN0, PIN_IS_LOW); //IDLE task
	}/*Loop*/
}

void Load_1_Simulation(void * pvParameters)
{
	static TickType_t tickCountCurrent = 0;
	int startTime = 0;
	int endTime = 0;

	TickType_t xLastWakeTime = xTaskGetTickCount();

	for(;;)
	{
		/* delay 5 msec */

		tickCountCurrent = xTaskGetTickCount();
		while( (xTaskGetTickCount() - tickCountCurrent) < 5)
		{
			;
		}
		endTime = xTaskGetTickCount();
		Load_1_ExecTimeMsec = endTime - startTime;
		if( Load_1_ExecTimeMsec > LOAD_1_DELAY)
		{
			Load_1_DeadlineMiss++;
		}
		GPIO_write(PORT_0, PIN6, PIN_IS_LOW);
		vTaskDelayUntil( &xLastWakeTime, LOAD_1_DELAY);
		GPIO_write(PORT_0, PIN6, PIN_IS_HIGH);
		startTime = xTaskGetTickCount();

		GPIO_write(PORT_1,PIN0, PIN_IS_LOW); //IDLE task
	}//loop
}


void Load_2_Simulation(void * pvParameters)
{
	static TickType_t tickCountCurrent = 0;
	TickType_t xLastWakeTime = xTaskGetTickCount();
	int startTime = 0;
	int endTime = 0;

	for(;;)
	{
		/* delay 12 msec */
		tickCountCurrent = xTaskGetTickCount();
		while( (xTaskGetTickCount() - tickCountCurrent) < 12)
		{
			;
		}
		endTime = xTaskGetTickCount();
		Load_2_ExecTimeMsec = endTime - startTime;
		if( Load_2_ExecTimeMsec > LOAD_2_DELAY)
		{
			Load_2_DeadlineMiss++;
		}
		GPIO_write(PORT_0, PIN7, PIN_IS_LOW);
		vTaskDelayUntil( &xLastWakeTime, LOAD_2_DELAY);
		GPIO_write(PORT_0, PIN7, PIN_IS_HIGH);
		startTime = xTaskGetTickCount();

		GPIO_write(PORT_1,PIN0, PIN_IS_LOW); //IDLE task
	}//loop
}


/*Implement tick hook*/
void vApplicationTickHook(void)
{
	GPIO_write(PORT_0, PIN8, PIN_IS_HIGH);
	GPIO_write(PORT_0, PIN8, PIN_IS_LOW);
	MainTickCounts++;
}


void vApplicationIdleHook( void )
{
	GPIO_write(PORT_1,PIN0, PIN_IS_HIGH);

}

/*
 * Application entry point:
 * Starts all the other tasks, then starts the scheduler.
 */
int main( void )
{

	/* Setup the hardware for use with the Keil demo board. */
	prvSetupHardware();

	 xStructQueue = xQueueCreate(
                         /* The number of items the queue can hold. */
                         3,
                         /* Size of each item is big enough to hold the
                         whole structure. */
                         sizeof( UartQueue ) );

	/* Create Tasks here */
	xTaskPeriodicCreate (  //xTaskCreate
											Button_1_Monitor,
											"Button_1",
											100,
											( void * ) 0,
											1,				//2
											&task_1_Handler,
											BUTTON_1_DELAY);

	xTaskPeriodicCreate(		// xTaskPeriodicCreate
											Button_2_Monitor,
											"Button_2",
											100,
											( void * ) 0,
											1,					//2
											&task_2_Handler,
											BUTTON_2_DELAY);

	xTaskPeriodicCreate(						// xTaskPeriodicCreate
											Periodic_Transmitter,
											"Transmitter",
											100,
											( void * ) 0,
											1,		//1
											&task_3_Handler,
											UART_TX_DELAY);

	xTaskPeriodicCreate(	//xTaskPeriodicCreate
											Uart_Receiver,
											"UART_Rx",
											200,
											( void * ) 0,
											1,				//3
											&task_4_Handler,
											UART_RX_DELAY);

	xTaskPeriodicCreate(	//xTaskPeriodicCreate
											Load_1_Simulation,
											"Load_1",
											100,
											( void * ) 0,
											1,				//4
											&task_5_Handler,
											LOAD_1_DELAY);

	xTaskPeriodicCreate(	//xTaskPeriodicCreate
											Load_2_Simulation,
											"Load_2",
											100,
											( void * ) 0,
											1,				//1
											&task_6_Handler,
											LOAD_2_DELAY);


/*Create the UART tx message queue*/
  /*UartQueueHandle = xQueueCreate( (UBaseType_t) 1,
                             (UBaseType_t) sizeof(char) );*/


	/*UartQueueHandle = xQueueCreate( 4, sizeof(char *) );*/

													/*printf("size_test");*/
	/* Now all the tasks have been started - start the scheduler.

	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used here. */
	vTaskStartScheduler();
/*vSerialPutString()*/
	/* Should never reach here!  If you do then there was not enough heap
	available for the idle task to be created. */
	for( ;; );
}
/*-----------------------------------------------------------*/

/* Function to reset timer 1 */
void timer1Reset(void)
{
	T1TCR |= 0x2;
	T1TCR &= ~0x2;
}

/* Function to initialize and start timer 1 */
static void configTimer1(void)
{
	T1PR = 1000;
	T1TCR |= 0x1;
}

static void prvSetupHardware( void )
{
	/* Perform the hardware setup required.  This is minimal as most of the
	setup is managed by the settings in the project file. */

	/* Configure UART */
	xSerialPortInitMinimal(mainCOM_TEST_BAUD_RATE);

	/* Configure GPIO */
	GPIO_init();

	/* Config trace timer 1 and read T1TC to get current tick */
	configTimer1();

	/* Setup the peripheral bus to be the same as the PLL output. */
	VPBDIV = mainBUS_CLK_FULL;
}
/*-----------------------------------------------------------*/


