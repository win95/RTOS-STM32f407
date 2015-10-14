//******************************************************************************
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "discoveryf4utils.h"
//******************************************************************************
/* USER ...*/
#include "defines.h"
#include "tm_stm32f4_hd44780.h"
#include "tm_stm32f4_gpio.h"
#include "tm_stm32f4_usart.h"
#include "attributes.h"
#include "tm_stm32f4_delay.h"
#include "tm_stm32f4_ethernet.h"
#include "stm32f4xx_rcc.h"
#include <stdio.h>
#include <string.h>
//******************************************************************************
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "croutine.h"
//******************************************************************************

void vLedBlinkBlue(void *pvParameters);
void vLedBlinkRed(void *pvParameters);
void vLedBlinkGreen(void *pvParameters);
void vLedBlinkOrange(void *pvParameters);

/*Khoi LCD*/
	uint8_t customChar[] = {
		0x1F,	/*  xxx 11111 */
		0x11,	/*  xxx 10001 */
		0x11,	/*  xxx 10001 */
		0x11,	/*  xxx 10001 */
		0x11,	/*  xxx 10001 */
		0x11,	/*  xxx 10001 */
		0x11,	/*  xxx 10001 */
		0x1F	/*  xxx 11111 */
	};
unsigned char flag_LCD = 0;
char buffer_lcd[20];

#define STACK_SIZE_MIN	128	/* usStackDepth	- the stack size DEFINED IN WORDS.*/

//******************************************************************************
int main(void)
{
	/*!< At this stage the microcontroller clock setting is already configured,
	   this is done through SystemInit() function which is called from startup
	   file (startup_stm32f4xx.s) before to branch to application main.
	   To reconfigure the default setting of SystemInit() function, refer to
	   system_stm32f4xx.c file
	 */
	
	/*!< Most systems default to the wanted configuration, with the noticeable 
		exception of the STM32 driver library. If you are using an STM32 with 
		the STM32 driver library then ensure all the priority bits are assigned 
		to be preempt priority bits by calling 
		NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 ); before the RTOS is started.
	*/
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
	
		/* Initialize system */
	SystemInit();
	
	STM_EVAL_LEDInit(LED_BLUE);
	STM_EVAL_LEDInit(LED_GREEN);
	STM_EVAL_LEDInit(LED_ORANGE);
	STM_EVAL_LEDInit(LED_RED);
	TM_USART_Init(USART3, TM_USART_PinsPack_3, 9600);
	TM_USART_Init(USART6, TM_USART_PinsPack_1, 9600);
	xTaskCreate( vLedBlinkBlue, (const signed char*)"Led Blink Task Blue", 
		STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, NULL );
	xTaskCreate( vLedBlinkRed, (const signed char*)"Led Blink Task Red", 
		STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, NULL );
	xTaskCreate( vLedBlinkGreen, (const signed char*)"Led Blink Task Green", 
		STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, NULL );
	xTaskCreate( vLedBlinkOrange, (const signed char*)"Led Blink Task Orange", 
		STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, NULL );
	
	vTaskStartScheduler();
}
//******************************************************************************

//******************************************************************************
void vLedBlinkBlue(void *pvParameters)
{
	for(;;)
	{
		STM_EVAL_LEDToggle(LED_BLUE);
		vTaskDelay( 500 / portTICK_RATE_MS );
	}
}

void vLedBlinkRed(void *pvParameters)
{
	for(;;)
	{
		STM_EVAL_LEDToggle(LED_RED);
		vTaskDelay( 750 / portTICK_RATE_MS );
	}
}

void vLedBlinkGreen(void *pvParameters)
{
	for(;;)
	{
		STM_EVAL_LEDToggle(LED_GREEN);
		vTaskDelay( 250 / portTICK_RATE_MS );
	}
}

void vLedBlinkOrange(void *pvParameters)
{
		/* Init LCD*/
		
		TM_GPIO_Init(HD44780_RW_PORT, HD44780_RW_PIN, TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_High);
		TM_GPIO_SetPinLow(HD44780_RW_PORT,HD44780_RW_PIN);
    TM_HD44780_Init(16, 4);
    //Save custom character on location 0 in LCD
    TM_HD44780_CreateChar(0, &customChar[0]);
    //Put string to LCD
    TM_HD44780_Puts(0, 0, "STM32F407VET\n\rCreartbyR&D-TIS\n\rRTOSTEST"); /* 0 dong 1, 1 dong 2*/
	for(;;)
	{

		STM_EVAL_LEDToggle(LED_ORANGE);
		TM_USART_Puts(USART3, "WelcomLCD");
		vTaskDelay(500 / portTICK_RATE_MS );
	}
}
//******************************************************************************
/* Tran duc code*/
void TM_USART3_ReceiveHandler(uint8_t c) {	 // com 2
	
	TM_USART_Puts(USART3, "Ngat nhan uart3");
}
void TM_USART6_ReceiveHandler(uint8_t c) {	 // com 2
	
	TM_USART_Puts(USART6, "Ngat nhan uart6");
}

/* End code*/