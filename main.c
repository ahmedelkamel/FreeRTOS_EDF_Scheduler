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

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "lpc21xx.h"
#include "event_groups.h"
#include "queue.h"

/* Peripheral includes. */
#include "serial.h"
#include "GPIO.h"


/*-----------------------------------------------------------*/

/* Constants to setup I/O and processor. */
#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )

/* Constants for the ComTest demo application tasks. */
#define mainCOM_TEST_BAUD_RATE	( ( unsigned long ) 115200 )

/* Task Period/Deadline */
#define TASK1_PERIOD  50
#define TASK2_PERIOD  50
#define TASK3_PERIOD 100
#define TASK4_PERIOD  20
#define TASK5_PERIOD  10
#define TASK6_PERIOD 100
/* Task Capacity */
#define CAPACITY_5_MS 14100
#define CAPACITY_12_MS 34120

/* Event Flag Bits */
#define EVENT_FLAG_BTN_1_RISING  (1<<0)
#define EVENT_FLAG_BTN_1_FALLING (1<<1)
#define EVENT_FLAG_BTN_2_RISING  (1<<2)
#define EVENT_FLAG_BTN_2_FALLING (1<<3)

/* Edege EventGroup */
EventGroupHandle_t xEdgeEventGroup;
/* Queue */
QueueHandle_t xEdgeQueue;
/* CPU Load */
int tasks_in_time = 0, util_time = 0, cpu_load = 0;


/* Button Pins */
#define BUTTON_1_PORT PORT_1
#define BUTTON_2_PORT PORT_1

#define BUTTON_1_PIN PIN8
#define BUTTON_2_PIN PIN9

/* Data type */
typedef struct {
	/*BnE: B1R\n B1F\n ..etc */
	char eData[4]; 
} EMessage;
/*
 * Configure the processor for use with the Keil demo board.  This is very
 * minimal as most of the setup is managed by the settings in the project
 * file.
 */
static void prvSetupHardware( void );
/*-----------------------------------------------------------*/

/* Tick Hook */
void vApplicationTickHook(void)
{
	GPIO_write(PORT_1, PIN2, PIN_IS_HIGH);
	GPIO_write(PORT_1, PIN2, PIN_IS_LOW);
}

/* New Task Creation */
void vTask1( void * pvParameters )
{
    /* The parameter value is expected to be 1 as 1 is passed in the
    pvParameters value in the call to xTaskCreate() below. 
    configASSERT( ( ( uint32_t ) pvParameters ) == 1 );
		*/
		TickType_t xLastWakeTime;
		uint8_t xBtn1_PrevState;
	  /* Initialise the xLastWakeTime variable with the current time. */
		xLastWakeTime = xTaskGetTickCount();
	  /* Initialize button state */
		xBtn1_PrevState = GPIO_read(BUTTON_1_PORT, BUTTON_1_PIN);
    for( ;; )
    {
			/* Task code goes here. */
			uint8_t xBtn1_CurState = GPIO_read(BUTTON_1_PORT, BUTTON_1_PIN);
			/* Rising Edge */
			if(xBtn1_PrevState == PIN_IS_LOW && xBtn1_CurState == PIN_IS_HIGH )
			{
				xEventGroupSetBits(xEdgeEventGroup,    			  /* The event group being updated. */
                           EVENT_FLAG_BTN_1_RISING ); /* The bits being set. */
			}
			/* Falling Edge */
			if(xBtn1_PrevState == PIN_IS_HIGH && xBtn1_CurState == PIN_IS_LOW )
			{
				xEventGroupSetBits(xEdgeEventGroup,    			  /* The event group being updated. */
                           EVENT_FLAG_BTN_1_FALLING ); /* The bits being set. */
			}
			xBtn1_PrevState = xBtn1_CurState;
			vTaskDelayUntil(&xLastWakeTime, TASK1_PERIOD);
    }
}

void vTask2( void * pvParameters )
{
		TickType_t xLastWakeTime;
		uint8_t xBtn2_PrevState;
		/* Initialise the xLastWakeTime variable with the current time. */
		xLastWakeTime = xTaskGetTickCount();
	  /* Initialize button state */
		xBtn2_PrevState = GPIO_read(BUTTON_2_PORT, BUTTON_2_PIN);
    for( ;; )
    {
			/* Task code goes here. */
			uint8_t xBtn2_CurState = GPIO_read(BUTTON_2_PORT, BUTTON_2_PIN);
			/* Rising Edge */
			if(xBtn2_PrevState == PIN_IS_LOW && xBtn2_CurState == PIN_IS_HIGH )
			{
				xEventGroupSetBits(xEdgeEventGroup,    			  /* The event group being updated. */
                           EVENT_FLAG_BTN_2_RISING ); /* The bits being set. */
			}
			/* Falling Edge */
			if(xBtn2_PrevState == PIN_IS_HIGH && xBtn2_CurState == PIN_IS_LOW )
			{
				xEventGroupSetBits(xEdgeEventGroup,    			  /* The event group being updated. */
                           EVENT_FLAG_BTN_2_FALLING ); /* The bits being set. */
			}
			xBtn2_PrevState = xBtn2_CurState;
			vTaskDelayUntil(&xLastWakeTime, TASK2_PERIOD);
    }
}

/* Periodic Transmitter */
void vTask3( void * pvParameters )
{
		TickType_t xLastWakeTime;
		/* Edge EventGroup */
		EventBits_t uxBits;
		/* Initialise the xLastWakeTime variable with the current time. */
		xLastWakeTime = xTaskGetTickCount();
    for( ;; )
    {
			/* Task code goes here. */
			/* Wait a maximum of 1ms for an edge Event bit to be set within
			the event group.  Clear the bits before exiting. */
			uxBits = xEventGroupWaitBits(
								xEdgeEventGroup,   /* The event group being tested. */
								EVENT_FLAG_BTN_1_RISING  | 
								EVENT_FLAG_BTN_1_FALLING |
								EVENT_FLAG_BTN_2_RISING  | 
								EVENT_FLAG_BTN_2_FALLING, /* The bits within the event group to wait for. */
								pdTRUE,        		 /* One of the 4 bits should be cleared before returning. */
								pdFALSE,      		 /* Don't wait for all bits, either bit will do. */
								(TickType_t) 0);		 /* No waiting / Wait a maximum of 0ms for either bit to be set. */
			/* Optimize later */
			if( ( uxBits & EVENT_FLAG_BTN_1_RISING ) != 0 )
			{
				/* xEventGroupWaitBits() returned because just BTN_1_RISING bit was set. */
				xQueueSendToBack(xEdgeQueue, (void*) "B1R\n",
									 /* Block time of 0 says don't block if the queue is already full. */
									 (TickType_t) 0);
			}
			if( ( uxBits & EVENT_FLAG_BTN_1_FALLING ) != 0 )
			{
				/* xEventGroupWaitBits() returned because just BTN_1_FALLING bit was set. */
				xQueueSendToBack(xEdgeQueue, (void*) "B1F\n",
										 /* Block time of 0 says don't block if the queue is already full. */
										 (TickType_t) 0);
			}
			if( ( uxBits & EVENT_FLAG_BTN_2_RISING ) != 0 )
			{
				/* xEventGroupWaitBits() returned because just BTN_2_RISING bit was set. */
				xQueueSendToBack(xEdgeQueue, (void*) "B2R\n",
										 /* Block time of 0 says don't block if the queue is already full. */
										 (TickType_t) 0);
			}
			if( ( uxBits & EVENT_FLAG_BTN_2_FALLING ) != 0 )
			{
				/* xEventGroupWaitBits() returned because just BTN_2_FALLING bit was set. */
				xQueueSendToBack(xEdgeQueue, (void*) "B2F\n",
										 /* Block time of 0 says don't block if the queue is already full. */
										 (TickType_t) 0);
			}
			vTaskDelayUntil(&xLastWakeTime, TASK3_PERIOD);
    }
}

/* UART Task */
void vTask4( void * pvParameters )
{
		TickType_t xLastWakeTime;
		/* Messages from Periodic Transmitter task */
		EMessage eMsg;
		char eMsgs[40] = {0};
		uint8_t n_bytes;
		/* Initialise the xLastWakeTime variable with the current time. */
		xLastWakeTime = xTaskGetTickCount();
    for( ;; )
    {
			/* Task code goes here. */
			n_bytes = 0;
			while(xQueueReceive(xEdgeQueue, (void*) &eMsg, (TickType_t) 0))
			{
				/* Do not copy NULL char */
				memcpy(eMsgs+n_bytes, eMsg.eData, sizeof(EMessage));
				n_bytes+=sizeof(EMessage);
			}
			if(n_bytes)
			{
				vSerialPutString((const signed char*)eMsgs, n_bytes);
			}
			vTaskDelayUntil(&xLastWakeTime, TASK4_PERIOD);
    }
}

void vTask5( void * pvParameters )
{
		TickType_t xLastWakeTime;
		/* Initialise the xLastWakeTime variable with the current time. */
		xLastWakeTime = xTaskGetTickCount();
    for( ;; )
    {
			volatile int i;
			/* Simulate load */
			for (i=0; i<CAPACITY_5_MS; i++)
			{
				i=i;
			}
			/* Task code goes here. */
			vTaskDelayUntil(&xLastWakeTime, TASK5_PERIOD);
    }
}

void vTask6( void * pvParameters )
{
		TickType_t xLastWakeTime;
		/* Initialise the xLastWakeTime variable with the current time. */
		xLastWakeTime = xTaskGetTickCount();
    for( ;; )
    {
			volatile int i;
			/* Simulate load */
			for (i=0; i<CAPACITY_12_MS; i++)
			{
				i=i;
			}
			/* Task code goes here. */
			vTaskDelayUntil(&xLastWakeTime, TASK6_PERIOD);
    }
}

void InitTasks(void)
{
	TaskHandle_t xToggleTask_Handle = NULL;
	TaskHandle_t xTask1 = NULL,
							 xTask2 = NULL,
							 xTask3 = NULL,
							 xTask4 = NULL,
	             xTask5 = NULL,
							 xTask6 = NULL;

	/* Create Event Group */
	xEdgeEventGroup = xEventGroupCreate();
	/* Create Edge Queue capable of containing 10 messages. */
	xEdgeQueue = xQueueCreate( 10, sizeof(EMessage) );

	/* Create the task, storing the handle. */
	xTaskCreatePeriodic(
											vTask1,     				/* Function that implements the task. */
											"Button_1_Monitor",	   				/* Text name for the task. */
											100, 				     				/* Stack size in words, not bytes. */
											( void * ) 1,    				/* Parameter passed into the task. */
											2,											/* Priority at which the task is created. */
											&xTask1, 				/* Used to pass out the created task's handle. */
											TASK1_PERIOD);  

	xTaskCreatePeriodic(
											vTask2,     				/* Function that implements the task. */
											"Button_2_Monitor",	   				/* Text name for the task. */
											100, 				     				/* Stack size in words, not bytes. */
											( void * ) 1,    				/* Parameter passed into the task. */
											2,											/* Priority at which the task is created. */
											&xTask2, 				/* Used to pass out the created task's handle. */
											TASK2_PERIOD);  

	xTaskCreatePeriodic(
											vTask3,     				/* Function that implements the task. */
											"Periodic_Transmitter",	   				/* Text name for the task. */
											100, 				     				/* Stack size in words, not bytes. */
											( void * ) 1,    				/* Parameter passed into the task. */
											2,											/* Priority at which the task is created. */
											&xTask3, 				/* Used to pass out the created task's handle. */
											TASK3_PERIOD);  
											
	xTaskCreatePeriodic(
											vTask4,     				/* Function that implements the task. */
											"Uart_Receiver",	   				/* Text name for the task. */
											100, 				     				/* Stack size in words, not bytes. */
											( void * ) 1,    				/* Parameter passed into the task. */
											2,											/* Priority at which the task is created. */
											&xTask4, 				/* Used to pass out the created task's handle. */
											TASK4_PERIOD);  
											
	xTaskCreatePeriodic(
											vTask5,     				/* Function that implements the task. */
											"Task_5_Load_Sim",	   				/* Text name for the task. */
											100, 				     				/* Stack size in words, not bytes. */
											( void * ) 1,    				/* Parameter passed into the task. */
											2,											/* Priority at which the task is created. */
											&xTask5, 				/* Used to pass out the created task's handle. */
											TASK5_PERIOD);  
										
	xTaskCreatePeriodic(
											vTask6,     				/* Function that implements the task. */
											"Task_6_Load_Sim",	   				/* Text name for the task. */
											100, 				     				/* Stack size in words, not bytes. */
											( void * ) 1,    				/* Parameter passed into the task. */
											2,											/* Priority at which the task is created. */
											&xTask6, 				/* Used to pass out the created task's handle. */
											TASK6_PERIOD);  
											
	/* The handle of the task to which a tag value is being assigned.
		 Passing xTask as NULL causes the tag to be assigned to the calling task, 
		 i.e. from within the task itself */
	/* This tag is used by Trace Hooks */
	vTaskSetApplicationTaskTag(xTask1, (void*)1);
	vTaskSetApplicationTaskTag(xTask2, (void*)2);
	vTaskSetApplicationTaskTag(xTask3, (void*)3);
	vTaskSetApplicationTaskTag(xTask4, (void*)4);
	vTaskSetApplicationTaskTag(xTask5, (void*)5);
	vTaskSetApplicationTaskTag(xTask6, (void*)6);
}




					 
/*
 * Application entry point:
 * Starts all the other tasks, then starts the scheduler. 
 */
int main( void )
{
	/* Setup the hardware for use with the Keil demo board. */
	prvSetupHardware();

	
  /* Create Tasks here */
	InitTasks();


	/* Now all the tasks have been started - start the scheduler.

	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used here. */
	vTaskStartScheduler();

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


