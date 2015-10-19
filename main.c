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
#include "tm_stm32f4_watchdog.h"
#include "attributes.h"
#include "tm_stm32f4_delay.h"
#include "tm_stm32f4_ethernet.h"
#include "tm_stm32f4_exti.h"
#include "tm_stm32f4_fatfs.h"
#include "stm32f4xx_rcc.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//******************************************************************************
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "timers.h"
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
char str[50];
void read_sw_add(void);
/* value DIP switch*/
unsigned char value_dip =0;

#define NUM_TIMERS 10
xTimerHandle xTimers[ NUM_TIMERS ];
long lExpireCounters[ NUM_TIMERS ] = { 0 };
void vTimerCallback( xTimerHandle  pxTimer );

/* Create simple typedef for DNS controlling */
typedef struct {
	uint8_t Working;
	uint8_t HaveIP;
	uint8_t ip[4];
} TM_DNS_t;
TM_DNS_t MyDNS;

uint16_t requests_count = 1;

#define STACK_SIZE_MIN	128	/* usStackDepth	- the stack size DEFINED IN WORDS.*/

const signed char timer_xx[] = "Timer";

/* Fatfs object */
FATFS FatFs;
/* File object */
FIL fil;

//******************************************************************************
int main(void)
{	
		long x=0;
			/* Free and total space */
	uint32_t total, free;
      // the scheduler starts.
      for( x = 0; x <= 2; x++ )
      {
          xTimers[ x ] = xTimerCreate(timer_xx,         // Just a text name, not used by the kernel.
                                          (2000    / portTICK_RATE_MS),     // The timer period in ticks.
                                          pdTRUE,         // The timers will auto-reload themselves when they expire.
                                          (void*)x,     // Assign each timer a unique id equal to its array index.
                                          vTimerCallback     // Each timer calls the same callback when it expires.
                                      );
 
          if( xTimers[ x ] == NULL )
          {
              // The timer was not created.
          }
          else
          {
              // Start the timer.  No block time is specified, and even if one was
              // it would be ignored because the scheduler has not yet been
              // started.
              if( xTimerStart( xTimers[ x ], 0 ) != pdPASS )
              {
                  // The timer could not be set into the Active state.
              }
          }
      }
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
	
//		/* Enable watchdog, 4 seconds before timeout */
//	if (TM_WATCHDOG_Init(TM_WATCHDOG_Timeout_4s)) {
//		/* Report to user */
//		//printf("Reset occured because of Watchdog\n");
//	}

//		/* Try to open file */
//		if (f_open(&fil, "2stfile.txt", FA_CREATE_ALWAYS | FA_READ | FA_WRITE) == FR_OK) {
////			if (f_open(&fil, "stfile.txt", FA_CREATE_ALWAYS | FA_READ | FA_WRITE) == FR_OK) {
//			/* File opened, turn off RED and turn on GREEN led */
//			//TM_DISCO_LedOn(LED_GREEN);
//			//TM_DISCO_LedOff(LED_RED);
//			
//			/* If we put more than 0 characters (everything OK) */
//			if (f_puts("First string in my file RTOS\n", &fil) > 0) {
//				if (TM_FATFS_DriveSize(&total, &free) == FR_OK) {
//					/* Data for drive size are valid */
//				}
//				
//				/* Turn on both leds */
//				//TM_DISCO_LedOn(LED_GREEN | LED_RED);
//			}
//			
//			/* Close file, don't forget this! */
//			f_close(&fil);
//		}
//		
//		/* Unmount drive, don't forget this! */
//		f_mount(0, "0:", 1);
//	}
		/*init interrup INPUT*/
		if (TM_EXTI_Attach(W1_D0_PORT, W1_D0_PIN, TM_EXTI_Trigger_Rising) == TM_EXTI_Result_Ok) {
		//TM_USART_Puts(USART3, "khoi tao ngat W1_D0\n");
	}
		if (TM_EXTI_Attach(W1_D1_PORT, W1_D1_PIN, TM_EXTI_Trigger_Rising) == TM_EXTI_Result_Ok) {
		//TM_USART_Puts(USART3, "khoi tao ngat W1_D1\n");
	}
	STM_EVAL_LEDInit(LED_BLUE);
	STM_EVAL_LEDInit(LED_GREEN);
	STM_EVAL_LEDInit(LED_ORANGE);
	STM_EVAL_LEDInit(LED_RED);
	TM_USART_Init(USART3, TM_USART_PinsPack_3, 115200);
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
							/* Mount drive */
//	if (f_mount(&FatFs,"SD:", 1) == FR_OK) {
//		/* Mounted OK, turn on RED LED */
//		//TM_USART_Puts(USART3, "mount sd OK \n");
//	}
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
		//STM_EVAL_LEDToggle(LED_RED);
				/* Try to make a new connection, port 80 */

		vTaskDelay( 4000 / portTICK_RATE_MS );
	}
}

void vLedBlinkGreen(void *pvParameters)
{
	//TM_USART_Puts(USART3,"Program starting..\n");
	if (TM_ETHERNET_Init(NULL, NULL, NULL, NULL) == TM_ETHERNET_Result_Ok) {
	//TM_USART_Puts(USART3, "TM_ETHERNET_Init OK \n");
	//printf("TM_ETHERNET_Init OK \n");
	}

		/* Reset watchdog */
	//TM_WATCHDOG_Reset();
	for(;;)
	{
//		STM_EVAL_LEDToggle(LED_GREEN);
//		vTaskDelay( 250 / portTICK_RATE_MS );
				/* Update ethernet, call this as fast as possible */
		TM_ETHERNET_Update();
		/* Reset watchdog */
		TM_WATCHDOG_Reset();
	}
	
}

void vLedBlinkOrange(void *pvParameters)
{
		/*init SWADD*/
	TM_GPIO_Init(ADD_BIT0_PORT, ADD_BIT0_PIN, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_UP, TM_GPIO_Speed_Medium);	
	TM_GPIO_Init(ADD_BIT1_PORT, ADD_BIT1_PIN, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_UP, TM_GPIO_Speed_Medium);
	TM_GPIO_Init(ADD_BIT2_PORT, ADD_BIT2_PIN, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_UP, TM_GPIO_Speed_Medium);
	TM_GPIO_Init(ADD_BIT3_PORT, ADD_BIT3_PIN, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_UP, TM_GPIO_Speed_Medium);
	TM_GPIO_Init(ADD_BIT4_PORT, ADD_BIT4_PIN, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_UP, TM_GPIO_Speed_Medium);
	TM_GPIO_Init(ADD_BIT5_PORT, ADD_BIT5_PIN, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_UP, TM_GPIO_Speed_Medium);
	TM_GPIO_Init(ADD_BIT6_PORT, ADD_BIT6_PIN, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_UP, TM_GPIO_Speed_Medium);
	TM_GPIO_Init(ADD_BIT7_PORT, ADD_BIT7_PIN, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_UP, TM_GPIO_Speed_Medium);
		/* Init LCD*/
		TM_GPIO_Init(HD44780_RW_PORT, HD44780_RW_PIN, TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_High);
		TM_GPIO_SetPinLow(HD44780_RW_PORT,HD44780_RW_PIN);
    TM_HD44780_Init(16, 4);
    //Save custom character on location 0 in LCD
    TM_HD44780_CreateChar(0, &customChar[0]);
    //Put string to LCD
    TM_HD44780_Puts(0, 0, "STM32F407VET\n\rCreartbyR&D-TIS\n\rTestethernet"); /* 0 dong 1, 1 dong 2*/
	
		read_sw_add();
		sprintf(str,"%d",value_dip);
		TM_USART_Puts(USART3,str);
	for(;;)
	{
		STM_EVAL_LEDToggle(LED_ORANGE);
		//TM_USART_Puts(USART3, "WelcomLCD");
		vTaskDelay(50 / portTICK_RATE_MS );
	}
	
}

/* For printf function */
//int fputc(int ch, FILE *f) {
//	/* Send over usart */
//	TM_USART_Putc(USART3, ch);

//	/* Return character back */
//	return ch;
//}
//******************************************************************************
/* Tran duc code*/
void TM_USART3_ReceiveHandler(uint8_t c) {	 // com 2
		if (TM_ETHERNETCLIENT_Connect("192.168.1.250",192,168,1,250,8081,"./index.txt") != TM_ETHERNET_Result_Ok) { //&requests_count
		/* Print to user */
		sprintf(str,"Can not make a new connection!\n");
		TM_USART_Puts(USART3,str);
	}
	TM_USART_Puts(USART3, "Ngat nhan uart3");
}
void TM_USART6_ReceiveHandler(uint8_t c) {	 // com 2
	
	TM_USART_Puts(USART6, "Ngat nhan uart6");
}


uint16_t TM_ETHERNETCLIENT_CreateHeadersCallback(TM_TCPCLIENT_t* connection, char* buffer, uint16_t buffer_length) {
	/* Create request headers */
	sprintf(buffer, "GET /index.txt?number=%d HTTP/1.1\r\n", *(uint16_t *)connection->user_parameters);
	//strcat(buffer, "Host: stm32f4-discovery.com\r\n");
	strcat(buffer, "Connection: close\r\n");
	strcat(buffer, "\r\n");
	
	/* Return number of bytes in buffer */
	return strlen(buffer);
}

void TM_ETHERNETCLIENT_ReceiveDataCallback(TM_TCPCLIENT_t* connection, uint8_t* buffer, uint16_t buffer_length, uint16_t total_length) {
	uint16_t i = 0;
	
	/* We have available data for connection to receive */
	sprintf(str,"Receiveing %d bytes of data from %s\n", buffer_length, connection->name);
	TM_USART_Puts(USART3,str);
	
	/* Go through entire buffer, remove response headers */
	if (connection->headers_done == 0) {
		for (i = 0; i < buffer_length; i++) {
			if (
				buffer[i] == '\r' &&
				buffer[i + 1] == '\n' &&
				buffer[i + 2] == '\r' &&
				buffer[i + 3] == '\n'
			) {
				/* Headers done */
				connection->headers_done = 1;
				/* Increase i */
				i += 3;
				/* Break */
				break;
			}
		}
	}
	
	/* Print data */
	for (; i < buffer_length; i++) {
		/* Print response */
//		TM_USART_Puts(USART3,"%c", buffer[i]);
		sprintf(str,"%c", buffer[i]);
		TM_USART_Puts(USART3,str);
	}
}

void TM_ETHERNETCLIENT_ConnectedCallback(TM_TCPCLIENT_t* connection) {
	/* We are connected */
	sprintf(str,"We are connected to %s\n", connection->name);
//	TM_USART_Puts(USART6,"We are connected to %s\n", connection->name);
	TM_USART_Puts(USART3,str);
}

void TM_ETHERNETCLIENT_ConnectionClosedCallback(TM_TCPCLIENT_t* connection, uint8_t success) {
	/* We are disconnected, done with connection */
	if (success) {
	sprintf(str,"Connection %s was successfully closed. Number of active connections: %d\n", connection->name, *connection->active_connections_count);
	TM_USART_Puts(USART3,str);
		//TM_USART_Puts(USART6,"Connection %s was successfully closed. Number of active connections: %d\n", connection->name, *connection->active_connections_count);
	} else {
		sprintf(str,"Connection %s was closed because of error. Number of active connections: %d\n", connection->name, *connection->active_connections_count);
	TM_USART_Puts(USART3,str);
		//TM_USART_Puts(USART6,"Connection %s was closed because of error. Number of active connections: %d\n", connection->name, *connection->active_connections_count);
	}
	
	/* Increase number of requests */
	requests_count++;
}

void TM_ETHERNETCLIENT_ErrorCallback(TM_TCPCLIENT_t* connection) {
	/* Print to user */
	sprintf(str,"An error occured on connection %s\n", connection->name);
	TM_USART_Puts(USART3,str);
//	TM_USART_Puts(USART6,"An error occured on connection %s\n", connection->name);
}

void TM_ETHERNETCLIENT_ConnectionStartedCallback(TM_TCPCLIENT_t* connection) {
	/* Print to user */
	sprintf(str,"Connection %s has started\n", connection->name);
	TM_USART_Puts(USART3,str);
//	TM_USART_Puts(USART6,"Connection %s has started\n", connection->name);
}

/* DNS has IP */
void TM_ETHERNETDNS_FoundCallback(char* host_name, uint8_t ip_addr1, uint8_t ip_addr2, uint8_t ip_addr3, uint8_t ip_addr4) {
	/* If host name is stm32f4-discovery.com */
	if (strstr(host_name, "stm32f4-discovery.com")) {
		/* We have IP */
		MyDNS.HaveIP = 1;
		/* Save IP */
		MyDNS.ip[0] = ip_addr1;
		MyDNS.ip[1] = ip_addr2;
		MyDNS.ip[2] = ip_addr3;
		MyDNS.ip[3] = ip_addr4;
		/* We are not working anymore */
		MyDNS.Working = 0;
		
		/* Print to user */
		sprintf(str,"We have IP address for %s: %d.%d.%d.%d\n", host_name, ip_addr1, ip_addr2, ip_addr3, ip_addr4);
		TM_USART_Puts(USART3,str);
//		TM_USART_Puts(USART6,"We have IP address for %s: %d.%d.%d.%d\n", host_name, ip_addr1, ip_addr2, ip_addr3, ip_addr4);
	}
}

/* DNS error callback */
void TM_ETHERNETDNS_ErrorCallback(char* host_name) {
	/* If host name is stm32f4-discovery.com */
	if (strstr(host_name, "stm32f4-discovery.com")) {
		/* We have IP */
		MyDNS.HaveIP = 0;
		/* We are not working anymore */
		MyDNS.Working = 0;
	}
	/* Print to user */
	sprintf(str,"DNS has failed to get IP address for %s\n", host_name);
	TM_USART_Puts(USART3,str);
//	TM_USART_Puts(USART6,"DNS has failed to get IP address for %s\n", host_name);
}

void TM_ETHERNET_IPIsSetCallback(uint8_t ip_addr1, uint8_t ip_addr2, uint8_t ip_addr3, uint8_t ip_addr4, uint8_t dhcp) {
	/* Called when we have valid IP, it might be static or DHCP */
	
	if (dhcp) {
		/* IP set with DHCP */
		sprintf(str,"IP: %d.%d.%d.%d assigned by DHCP server\n", ip_addr1, ip_addr2, ip_addr3, ip_addr4);
		TM_USART_Puts(USART3,str);
//		TM_USART_Puts(USART6,"IP: %d.%d.%d.%d assigned by DHCP server\n", ip_addr1, ip_addr2, ip_addr3, ip_addr4);
	} else {
		/* Static IP */
		sprintf(str,"IP: %d.%d.%d.%d; STATIC IP used\n", ip_addr1, ip_addr2, ip_addr3, ip_addr4);
		TM_USART_Puts(USART3,str);
//		TM_USART_Puts(USART6,"IP: %d.%d.%d.%d; STATIC IP used\n", ip_addr1, ip_addr2, ip_addr3, ip_addr4);
	}
	
	/* Print MAC address to user */
	sprintf(str,"MAC: %02X-%02X-%02X-%02X-%02X-%02X\n",
		TM_ETHERNET_GetMACAddr(0),
		TM_ETHERNET_GetMACAddr(1),
		TM_ETHERNET_GetMACAddr(2),
		TM_ETHERNET_GetMACAddr(3),
		TM_ETHERNET_GetMACAddr(4),
		TM_ETHERNET_GetMACAddr(5)
	);
	sprintf(str,"MAC: %02X-%02X-%02X-%02X-%02X-%02X\n",
		TM_ETHERNET_GetMACAddr(0),
		TM_ETHERNET_GetMACAddr(1),
		TM_ETHERNET_GetMACAddr(2),
		TM_ETHERNET_GetMACAddr(3),
		TM_ETHERNET_GetMACAddr(4),
		TM_ETHERNET_GetMACAddr(5)
	);
	TM_USART_Puts(USART3,str);
//	TM_USART_Puts(USART6,"MAC: %02X-%02X-%02X-%02X-%02X-%02X\n",
//		TM_ETHERNET_GetMACAddr(0),
//		TM_ETHERNET_GetMACAddr(1),
//		TM_ETHERNET_GetMACAddr(2),
//		TM_ETHERNET_GetMACAddr(3),
//		TM_ETHERNET_GetMACAddr(4),
//		TM_ETHERNET_GetMACAddr(5)
//	);
	/* Print netmask to user */
	sprintf(str,"Netmask: %d.%d.%d.%d\n",
		TM_ETHERNET_GetGateway(0),
		TM_ETHERNET_GetGateway(1),
		TM_ETHERNET_GetGateway(2),
		TM_ETHERNET_GetGateway(3)
	);
	TM_USART_Puts(USART3,str);
//	TM_USART_Puts(USART6,"Netmask: %d.%d.%d.%d\n",
//		TM_ETHERNET_GetGateway(0),
//		TM_ETHERNET_GetGateway(1),
//		TM_ETHERNET_GetGateway(2),
//		TM_ETHERNET_GetGateway(3)
//	);
	/* Print gateway to user */
	sprintf(str,"Gateway: %d.%d.%d.%d\n",
		TM_ETHERNET_GetNetmask(0),
		TM_ETHERNET_GetNetmask(1),
		TM_ETHERNET_GetNetmask(2),
		TM_ETHERNET_GetNetmask(3)
	);
	TM_USART_Puts(USART3,str);
//	TM_USART_Puts(USART6,"Gateway: %d.%d.%d.%d\n",
//		TM_ETHERNET_GetNetmask(0),
//		TM_ETHERNET_GetNetmask(1),
//		TM_ETHERNET_GetNetmask(2),
//		TM_ETHERNET_GetNetmask(3)
//	);
	/* Print 100M link status, 1 = 100M, 0 = 10M */
	sprintf(str,"Link 100M: %d\n", TM_ETHERNET.speed_100m);
	TM_USART_Puts(USART3,str);
//	TM_USART_Puts(USART6,"Link 100M: %d\n", TM_ETHERNET.speed_100m);
	/* Print duplex status: 1 = Full, 0 = Half */
	sprintf(str,"Full duplex: %d\n", TM_ETHERNET.full_duplex);
	TM_USART_Puts(USART3,str);
//	TM_USART_Puts(USART6,"Full duplex: %d\n", TM_ETHERNET.full_duplex);
}

void TM_ETHERNET_LinkIsDownCallback(void) {
	/* This function will be called when ethernet cable will not be plugged */
	/* It will also be called on initialization if connection is not detected */
	
	/* Print to user */
	//sprintf(str,"Link is down, do you have connected to your modem/router?");
	//TM_USART_Puts(USART3,str);
	//TM_USART_Puts(USART6,"Link is down, do you have connected to your modem/router?");
}
void TM_ETHERNET_LinkIsUpCallback(void) {
	/* Cable has been plugged in back, link is detected */
	/* I suggest you that you reboot MCU here */
	/* Do important stuff before */
	
	/* Print to user */
		//sprintf(str,"Link is up back\n");
		//TM_USART_Puts(USART3,str);
	//TM_USART_Puts(USART6,"Link is up back\n");
}
/* Delay 1ms handler */
void TM_DELAY_1msHandler(void) {
	/* Time update for ethernet, 1ms */
	/* Add 1ms time for ethernet */
	TM_ETHERNET_TimeUpdate(1);
}
void vTimerCallback( xTimerHandle  pxTimer )
 {
	long lArrayIndex;
	const long xMaxExpiryCountBeforeStopping = 10;
	//TM_USART_Puts(USART3, "timer");
     /* Optionally do something if the pxTimer parameter is NULL. */
     configASSERT( pxTimer );

     /* Which timer expired? */
     lArrayIndex = ( long ) pvTimerGetTimerID( pxTimer );

     /* Increment the number of times that pxTimer has expired. */
     lExpireCounters[ lArrayIndex ] += 1;

     /* If the timer has expired 10 times then stop it from running. */
     if( lExpireCounters[ lArrayIndex ] == xMaxExpiryCountBeforeStopping )
     {
         /* Do not use a block time if calling a timer API function from a
         timer callback function, as doing so could cause a deadlock! */
         xTimerStop( pxTimer, 0 );
     }
 }
  /* This function assumes xTimer has already
 been created. */
 void vAFunction( xTimerHandle xTimer )
 {
     /* or more simply and equivalently
     "if( xTimerIsTimerActive( xTimer ) )" */
     if( xTimerIsTimerActive( xTimer ) != pdFALSE )
     {
         /* xTimer is active, do something. */
			 TM_USART_Puts(USART3, "xTimer is active");
     }
     else
     {
         /* xTimer is not active, do something else. */
			 TM_USART_Puts(USART3, "xTimer is not active");
     }
 }
void TM_EXTI_Handler(uint16_t GPIO_Pin) {
	if (GPIO_Pin == W1_D0_PIN) { /* run W1 D0 - dieu khien RELAY_DK1_PORT , RELAY_DK1_PIN*/	

		 TM_USART_Puts(USART3, "activeW1D0\n");
	}
		if (GPIO_Pin == W1_D1_PIN) { /* run W1 D0 - dieu khien RELAY_DK2_PORT , RELAY_DK1_PIN*/	

		 TM_USART_Puts(USART3, "activeW1D1\n");
	}
}
void read_sw_add(void){
	unsigned int sw_add[8];
	if(GPIO_ReadInputDataBit(ADD_BIT0_PORT,ADD_BIT0_PIN)==0) sw_add[0] = 1;
			else sw_add[0] = 0;
	if(GPIO_ReadInputDataBit(ADD_BIT1_PORT,ADD_BIT1_PIN)==0) sw_add[1] = 1;
			else sw_add[1] = 0;
	if(GPIO_ReadInputDataBit(ADD_BIT2_PORT,ADD_BIT2_PIN)==0) sw_add[2] = 1;
			else sw_add[2] = 0;
	if(GPIO_ReadInputDataBit(ADD_BIT3_PORT,ADD_BIT3_PIN)==0) sw_add[3] = 1;
			else sw_add[3] = 0;
	if(GPIO_ReadInputDataBit(ADD_BIT4_PORT,ADD_BIT4_PIN)==0) sw_add[4] = 1;
			else sw_add[4] = 0;
	if(GPIO_ReadInputDataBit(ADD_BIT5_PORT,ADD_BIT5_PIN)==0) sw_add[5] = 1;
			else sw_add[5] = 0;
	if(GPIO_ReadInputDataBit(ADD_BIT6_PORT,ADD_BIT6_PIN)==0) sw_add[6] = 1;
			else sw_add[6] = 0;
	if(GPIO_ReadInputDataBit(ADD_BIT7_PORT,ADD_BIT7_PIN)==0) sw_add[7] = 1;
			else sw_add[7] = 0;
	
	//value_dip=6;
	value_dip= 1*sw_add[0]+2*sw_add[1]+4*sw_add[2]+8*sw_add[3]+16*sw_add[4]+32*sw_add[5]+64*sw_add[6]+128*sw_add[7];
	if(value_dip == 0) value_dip=7; 
}
/* End code*/