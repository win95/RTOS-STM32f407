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
void vEthernet(void *pvParameters);
void vMain(void *pvParameters);
void printf_d(char *str);
/*Khoi LCD*/
	/* Fatfs object */
	FATFS FatFs;
	/* File object */
	FIL fil;
	FRESULT fres;
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
char data_tcp[20];
unsigned char  Process=0;	
void WaitPC(unsigned int t);
void ProcessAction(void);
	
unsigned char flag_RFID = 0;	/*Send SelectCard */		
unsigned char flag_RFID1=0;
unsigned char flag_RFID2=0;
unsigned char flag_PC=0;
char *vip_id[5] = {
	"80 50 37 54",
	"12 23 45 78",
	"01 12 34 45",
	"01 12 34 45",
	"01 12 34 45"
	};
char SelectCard[4]= {0xBA,0x02,0x01,0xB9};       
char IDCAR1[7]={0x00,0x00,0x00,0x00,0x00,0x00,0x00};
char IDCAR2[7]={0x00,0x00,0x00,0x00,0x00,0x00,0x00};
int LEDStatus=0;
char BufferCom1[50];
char BufferCom2[50];
char BufferCom3[50];
char UID1[50];
char UID2[50];
char buffer_lcd[20];
/* value DIP switch*/
unsigned char value_dip =0;
/*time out*/
unsigned int timeout = 0	;
volatile uint8_t counter = 0;
/* Khoi Ban phim */
unsigned char express;
/*Khoi 485*/
int set_485_send();
int set_485_recv();
unsigned char flag_485 = 0;
unsigned char flag_com1 = 0;
/* Khoi INPUT*/
unsigned char flag_W1D1 = 0;
unsigned char flag_W1D0 = 0;
//unsigned char flag_W2D1 = 0;
//unsigned char flag_W2D0 = 0;

unsigned char flag_R11x =0;
unsigned char flag_R21x =0;
unsigned char flag_R12 =0;
unsigned char flag_R22 =0;
unsigned char flag_R31 =0;

unsigned char flag_R10 =0;
unsigned char flag_R20 =0;
unsigned char flag_R11 =0;
unsigned char flag_R21 =0;
unsigned char flag_test=0;
int check_vip(char * name);
/* Khoi OUTPUT*/

int turn_on_dk1();
int turn_off_dk1();

int	turn_on_dk2();
int turn_off_dk2();

int	turn_on_dk3();
int turn_off_dk3();

int	turn_on_dk4();
int turn_off_dk4();

int timerdk1 =0;
int timerdk2 =0;
int timerdk3 =0;
int timerdk4 =0;

int timer_dk1 =0;
int timer_dk2 =0;
int timer_dk3 =0;
int timer_dk4 =0;	
#define NUM_TIMERS 10
xTimerHandle xTimers[ NUM_TIMERS ];
long lExpireCounters[ NUM_TIMERS ] = { 0 };
void vTimerCallback1( xTimerHandle  pxTimer );
void vTimerCallback2( xTimerHandle  pxTimer );
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


			/* Free and total space */

//******************************************************************************
int main(void)
{	
		long x=0;
          xTimers[1] = xTimerCreate(timer_xx,         // Just a text name, not used by the kernel.
                                          (500/ portTICK_RATE_MS),     // The timer period in ticks.
                                          pdTRUE,         // The timers will auto-reload themselves when they expire.
                                          (void*)1,     // Assign each timer a unique id equal to its array index.
                                          vTimerCallback1     // Each timer calls the same callback when it expires.
                                      );		
          xTimers[2] = xTimerCreate(timer_xx,         // Just a text name, not used by the kernel.
                                          (100/ portTICK_RATE_MS),     // The timer period in ticks.
                                          pdTRUE,         // The timers will auto-reload themselves when they expire.
                                          (void*)2,     // Assign each timer a unique id equal to its array index.
                                          vTimerCallback2     // Each timer calls the same callback when it expires.
                                      );																						
					if( xTimers[1] != NULL )
          {
              // Start the timer.  No block time is specified, and even if one was
              // it would be ignored because the scheduler has not yet been
              // started.
              if( xTimerStart( xTimers[1], 0 ) != pdPASS )
              {
                  // The timer could not be set into the Active state.
              }
						}
					if( xTimers[2] != NULL )
          {
              // Start the timer.  No block time is specified, and even if one was
              // it would be ignored because the scheduler has not yet been
              // started.
              if( xTimerStart( xTimers[2], 0 ) != pdPASS )
              {
                  // The timer could not be set into the Active state.
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
	/* init OUTPUT*/
	TM_GPIO_Init(RELAY_DK1_PORT, RELAY_DK1_PIN, TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_High);
	TM_GPIO_Init(RELAY_DK2_PORT, RELAY_DK2_PIN, TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_High);
	TM_GPIO_Init(RELAY_DK3_PORT, RELAY_DK3_PIN, TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_High);
	TM_GPIO_Init(RELAY_DK4_PORT, RELAY_DK4_PIN, TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_High);
	/* init OUTPUT*/
	STM_EVAL_LEDInit(LED_ORANGE);
/* Initialize USART6 at 115200 baud, TX: PC6, RX: PC7 , COM 1 - RFID1 gan cong tac nguon*/ 
	TM_USART_Init(USART6, TM_USART_PinsPack_1, 115200);
/* Initialize USART3 at 115200 baud, TX: PD8, RX: PD9 ,	COM 2 -PC gan ethernet*/
	TM_USART_Init(USART3, TM_USART_PinsPack_3, 115200);
/* Initialize USART1 at 115200 baud, TX: PA9, RX: PA10, CONG 485 */
	TM_USART_Init(USART1, TM_USART_PinsPack_1, 9600);

	xTaskCreate( vLedBlinkBlue, (const signed char*)"Led Blink Task Blue", 
		STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, NULL );
	xTaskCreate( vLedBlinkRed, (const signed char*)"Led Blink Task Red", 
		STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, NULL );
	xTaskCreate( vEthernet, (const signed char*)"Led Blink Task Green", 
		STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, NULL );
	xTaskCreate( vMain, (const signed char*)"Main run ...", 
		STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, NULL );

	vTaskStartScheduler();
}
//******************************************************************************
//******************************************************************************
void vLedBlinkBlue(void *pvParameters)
{

	uint32_t total, free;
		/* Mount drive */
		if (f_mount(&FatFs, "SD:", 1) == FR_OK) {
		printf_d("Mount OK \n");
		/* Unmount drive, don't forget this! */
			if(f_mount(0,"0:", 1)) printf_d("Unmount OK \n");
	}
	for(;;)
	{
		STM_EVAL_LEDToggle(LED_BLUE);
		//vTaskDelay( 500 / portTICK_RATE_MS );
	}
}

void vLedBlinkRed(void *pvParameters)
{
	for(;;)
	{
		STM_EVAL_LEDToggle(LED_ORANGE);
		vTaskDelay(50 / portTICK_RATE_MS );
	}
}

void vEthernet(void *pvParameters)
{
	TM_USART_Puts(USART3,"Program starting..\n");
	if (TM_ETHERNET_Init(NULL, NULL, NULL, NULL) == TM_ETHERNET_Result_Ok) {
	printf_d("TM_ETHERNET_Init OK \n");
	}

		/* Reset watchdog */
	TM_WATCHDOG_Reset();
	for(;;)
	{
//		vTaskDelay( 250 / portTICK_RATE_MS );
				/* Update ethernet, call this as fast as possible */
		TM_ETHERNET_Update();
		/* Reset watchdog */
		TM_WATCHDOG_Reset();
	}
	
}

void vMain(void *pvParameters)
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
/* int DIR 485 set = send , reset = recvice*/ 
	TM_GPIO_Init(CCU_DIR_PORT, CCU_DIR_PIN, TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_UP, TM_GPIO_Speed_High);
	TM_GPIO_SetPinHigh(CCU_DIR_PORT,CCU_DIR_PIN);
	/* Init LCD*/
	TM_GPIO_Init(HD44780_RW_PORT, HD44780_RW_PIN, TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_High);
	TM_GPIO_SetPinLow(HD44780_RW_PORT,HD44780_RW_PIN);
	read_sw_add();
	timeout = value_dip;
	memset(str,'\0',0);
	//Initialize LCD 20 cols x 4 rows
	TM_HD44780_Init(16, 4);
	//Save custom character on location 0 in LCD
	TM_HD44780_CreateChar(0, &customChar[0]);    
	//Put string to LCD
	TM_HD44780_Puts(0, 0, "FREERTOS-STM32\n\rCreartbyR&D-TIS"); /* 0 dong 1, 1 dong 2*/
	TM_HD44780_Puts(0, 2, "Welcome");
	vTaskDelay(2000);
	TM_HD44780_Clear();
	sprintf(str,"Timer out %d", timeout);
	TM_HD44780_Puts(0, 0,str);
	vTaskDelay(2000);
	TM_HD44780_Clear();
	TM_HD44780_Puts(0, 0,"----TIS8 PRO----");
	flag_RFID2=0;	
	flag_RFID1=0;
// configure role mo rong
	turn_off_dk3();
	turn_off_dk4();
	TM_WATCHDOG_Reset();
	/*end by duc*/
	for(;;)
	{
/*process 485*/
	if(flag_485){
	flag_485=0;
	if(LEDStatus==0) TM_USART_Puts(USART1, "/LED000>\r\n");
	if(LEDStatus==1) TM_USART_Puts(USART1, "/LED001>\r\n");
	if(LEDStatus==2) TM_USART_Puts(USART1, "/LED002>\r\n");
	}	

/* xu li W1D0 - dk1*/
	if(flag_W1D0 && !LEDStatus){
		turn_on_dk1();
		turn_on_dk3();
	}
/* xu li W1D1 - dk2*/
	if(flag_W1D1 && !LEDStatus){
		turn_on_dk2();
		turn_on_dk4();
	}
/*xu li 1 tien trinh hoan chinh*/
if(Process!=1) TM_HD44780_Puts(0, 2,"Wait for Card"); /* 0 dong 1, 1 dong 2*/
/*xu li khi co su kien nhan the*/
if(flag_RFID1==1)
		{	
		
		Process=1;
		IDCAR1[0]=BufferCom1[4];
		IDCAR1[1]=BufferCom1[5];
		IDCAR1[2]=BufferCom1[6];
		IDCAR1[3]=BufferCom1[7];
		IDCAR1[4]=BufferCom1[8];
		IDCAR1[5]=BufferCom1[9];
		IDCAR1[6]=BufferCom1[10];
		
		if(BufferCom1[1]==0x08)	
			{
			sprintf(UID1,"%02x %02x %02x %02x",IDCAR1[0],IDCAR1[1],IDCAR1[2],IDCAR1[3]);
			}
		if(BufferCom1[1]==0x0B) 
			{
			sprintf(UID1,"%02x %02x %02x %02x %02x %02x %02x",IDCAR1[0],IDCAR1[1],IDCAR1[2],IDCAR1[3],IDCAR1[4],IDCAR1[5],IDCAR1[6]);
			}
		TM_HD44780_Puts(0, 2,"Waiting PC..."); /* 0 dong 1, 1 dong 2*/
				if(check_vip(UID1)){
			flag_PC=1;
			flag_R10=1;
			timerdk1 =0;
			Process=0;
		}
		else{
		if(Process)TM_USART_Puts(USART3,UID1);
		}
		WaitPC(200);
		flag_RFID1=0;
		if(flag_PC)
		{
			TM_HD44780_Puts(0, 2,"Door opened.."); /* 0 dong 1, 1 dong 2*/
			flag_PC=0;
			ProcessAction();
			WaitPC(10);
		}
		else Process=0;
		flag_RFID1=0;	
		flag_test=0;
	}
if(flag_RFID2==1)
		{		
		Process=1;
		IDCAR2[0]=BufferCom2[4];
		IDCAR2[1]=BufferCom2[5];
		IDCAR2[2]=BufferCom2[6];
		IDCAR2[3]=BufferCom2[7];
		IDCAR2[4]=BufferCom2[8];
		IDCAR2[5]=BufferCom2[9];
		IDCAR2[6]=BufferCom2[10];
		
		if(BufferCom2[1]==0x08)	
			{
			sprintf(UID2,"%02x %02x %02x %02x",IDCAR2[0],IDCAR2[1],IDCAR2[2],IDCAR2[3]);
			}
		if(BufferCom2[1]==0x0B) 
			{
			sprintf(UID2,"%02x %02x %02x %02x %02x %02x %02x",IDCAR2[0],IDCAR2[1],IDCAR2[2],IDCAR2[3],IDCAR2[4],IDCAR2[5],IDCAR2[6]);
			}
		TM_HD44780_Puts(0, 2,"Waiting PC..."); /* 0 dong 1, 1 dong 2*/
			if(check_vip(UID2)){
			flag_PC=1;
			flag_R10=1;
			timerdk1 =0;
			Process=0;
		}
		else{
		if(Process)TM_USART_Puts(USART3,UID2);}
		WaitPC(200);
		flag_RFID2=0;
		if(flag_PC)
		{
			TM_HD44780_Puts(0, 2,"Door opened.."); /* 0 dong 1, 1 dong 2*/
			flag_PC=0;
			ProcessAction();
			WaitPC(10);
		}
		else Process=0;
		flag_RFID2=0;
		
		flag_test=0;
	}
		
if(flag_test==1){
	flag_test=0;
	ProcessAction();
}
/* Quan li thoi gian cho tung su kien relay actived*/
timer_dk1 = timerdk1/2;	
if (timer_dk1 >= timeout){
			turn_off_dk1();
			flag_R10 =0;
			flag_R11x =0;
			flag_W1D0=0;
			timerdk1=0;
			timer_dk1=0;
			flag_RFID1=0;
			flag_RFID2=0;
			Process=0;
			WaitPC(200);
		}
timer_dk2 = timerdk2/2;
if (timer_dk2 >= timeout){
			turn_off_dk2();
			flag_R20 =0;
			flag_R31 =0;
			flag_W1D1=0;
			timerdk2=0;
			timer_dk2=0;
			Process=0;
			WaitPC(100);
		}
timer_dk3 = timerdk3;
if (timer_dk3 >= 4){
			turn_off_dk3();
			flag_R11 =0;
			flag_R12 =0;
			timerdk3=0;
			timer_dk3=0;
			if(LEDStatus==0) {Process=0;WaitPC(100);}
		}
timer_dk4 = timerdk4;
if (timer_dk4 >= 4){
			turn_off_dk4();
			flag_R21 =0;
			flag_R22 =0;
			timer_dk4=0;
			timerdk4=0;
			if(LEDStatus==0) {Process=0;WaitPC(100);}
		}


		TM_WATCHDOG_Reset();
}
	}
//******************************************************************************
/* Tran duc code*/
void printf_d(char *str){
	TM_USART_Puts(USART3,str);
}
void 	ProcessAction(void){
		if(strncmp(BufferCom3,"R10",3)==0) // mo relay 1
	{
		flag_R10=1;
		timerdk1 =0;
	}
		if(strncmp(BufferCom3,"R20",3)==0) // mo relay 2
	{
		flag_R20=1;
		timerdk2 =0;
	}
		if(strncmp(BufferCom3,"R11",3)==0) // nuot the mo relay 3
	{
		flag_R11=1;
		timerdk3 =0;
	}
		if(strncmp(BufferCom3,"R21",3)==0) // nha the mo relay 4
	{
		flag_R21=1;
		timerdk4 =0;
	}
	
//	if(strncmp(BufferCom3,"R11x",4)==0)	// active relay 1 and 3
//	{
//		flag_R11x=1;
//		flag_R12=1;
//		timerdk1 =0;
//		timerdk3 =0;
//	}
	
//	if(strncmp(BufferCom3,"R21x",4)==0)	// active relay 1 and 4
//	{
//		flag_R11x=1;
//		flag_R22=1;
//		timerdk1 =0;
//		timerdk4 =0;
//	}
	
//	if(strncmp(BufferCom3,"R12",3)==0)	// active relay 3
//	{
//		flag_R12=1;
//		timerdk3 =0;
//	}
	
//	if(strncmp(BufferCom3,"R11",3)==0)	// active relay 4
//	{
//		flag_R22=1;
//		timerdk4 =0;
//	}
//	if(strncmp(BufferCom3,"R31",3)==0)	// active relay 2
//	{
//		flag_R31=1;
//		timerdk2 =0;
//	}

		if(flag_R10){		// active relay 1
		turn_on_dk1();
	}
		if(flag_R20){	// active relay 2
		turn_on_dk2();
	}
		if(flag_R11){	// active relay 3
		turn_on_dk3();
	}
		if(flag_R21){	// active relay 4
		turn_on_dk4();
	}
	/* xu li flag_R11 - dk1 tu com1*/
	if(flag_R11x){
		//turn_on_dk1();
	}
	/* xu li flag_R31 - dk2 tu com1*/
	if(flag_R31){
		turn_on_dk2();
	}
/* xu li flag_R21 - dk1 tu com1*/
	if(flag_R21x){
		//turn_on_dk1();
	}
/* xu li flag_R12 - dk3 tu com1*/
	if(flag_R12){
		turn_on_dk3();
	}
/* xu li flag_R22 - dk4 tu com1*/
	if(flag_R22){
		turn_on_dk4();
	}
	
/*Update time*/

	if(flag_R11x||flag_W1D0||flag_R21x||flag_R10) timer_dk1 = timerdk1/2;

	if(flag_R31||flag_W1D1||flag_R20) timer_dk2 = timerdk2/2;

	if(flag_R12||flag_R11 ||flag_W1D0) timer_dk3 = timerdk3;

	if(flag_R22||flag_R21 ||flag_W1D1) timer_dk4 = timerdk4;

}

void 	WaitPC(unsigned int t){
	unsigned long int n=0;
	while(n<=t){
		vTaskDelay(10);
	if(flag_PC==1)
		{
			break;
		}
	n++;
	}
}
void 	TM_USART3_ReceiveHandler(uint8_t c) {	// PC
	static uint8_t cnt=0;
	if(c=='R')cnt=0;
	BufferCom3[cnt]=c;
	if(cnt==2)flag_test=1;
	if((cnt==2)&&(Process==1))flag_PC=1;
	if(cnt<48)cnt++;
	else cnt =0;
}
void 	TM_USART6_ReceiveHandler(uint8_t c) {	// RFID1
	static uint8_t cnt=0;
	if(c==0xBD)cnt=0;
	BufferCom1[cnt]=c;
	if((BufferCom1[3]==0x00)&&(cnt==BufferCom1[3]+1)&&cnt)
	{
		if(Process==0)flag_RFID1=1;
		memset(BufferCom1,'\0',0);
	}
	if(cnt<48)cnt++;

}


uint16_t TM_ETHERNETCLIENT_CreateHeadersCallback(TM_TCPCLIENT_t* connection, char* buffer, uint16_t buffer_length) {
	/* Create request headers */
	//sprintf(buffer, "GET /index.txt?number=%d HTTP/1.1\r\n", *(uint16_t *)connection->user_parameters);
	//sprintf(buffer, "number=%d\r\n", *(uint16_t *)connection->user_parameters);
	//strcat(buffer, "Host: stm32f4-discovery.com\r\n");
	//strcat(buffer, "Connection: close\r\n");
	//strcat(buffer, "\r\n");
	//sprintf(buffer,"\r\n");
	sprintf(buffer,"%s",data_tcp);
	/* Return number of bytes in buffer */
	return strlen(buffer);
}

void TM_ETHERNETCLIENT_ReceiveDataCallback(TM_TCPCLIENT_t* connection, uint8_t* buffer, uint16_t buffer_length, uint16_t total_length) {
	uint16_t i = 0;
	
	/* We have available data for connection to receive */
	sprintf(str,"Receiveing %d bytes of data from %s\n", buffer_length, connection->name);
	printf_d(str);
	
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
		printf_d(str);
	}
}

void TM_ETHERNETCLIENT_ConnectedCallback(TM_TCPCLIENT_t* connection) {
	/* We are connected */
	sprintf(str,"We are connected to %s\n", connection->name);
	printf_d(str);
}

void TM_ETHERNETCLIENT_ConnectionClosedCallback(TM_TCPCLIENT_t* connection, uint8_t success) {
	/* We are disconnected, done with connection */
	if (success) {
	sprintf(str,"Connection %s was successfully closed. Number of active connections: %d\n", connection->name, *connection->active_connections_count);
	printf_d(str);
		//TM_USART_Puts(USART6,"Connection %s was successfully closed. Number of active connections: %d\n", connection->name, *connection->active_connections_count);
	} else {
		sprintf(str,"Connection %s was closed because of error. Number of active connections: %d\n", connection->name, *connection->active_connections_count);
	printf_d(str);
		//TM_USART_Puts(USART6,"Connection %s was closed because of error. Number of active connections: %d\n", connection->name, *connection->active_connections_count);
	}
	
	/* Increase number of requests */
	requests_count++;
}

void TM_ETHERNETCLIENT_ErrorCallback(TM_TCPCLIENT_t* connection) {
	/* Print to user */
	sprintf(str,"An error occured on connection %s\n", connection->name);
	printf_d(str);
//	TM_USART_Puts(USART6,"An error occured on connection %s\n", connection->name);
}

void TM_ETHERNETCLIENT_ConnectionStartedCallback(TM_TCPCLIENT_t* connection) {
	/* Print to user */
	sprintf(str,"Connection %s has started\n", connection->name);
	printf_d(str);
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
		printf_d(str);
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
	printf_d(str);
//	TM_USART_Puts(USART6,"DNS has failed to get IP address for %s\n", host_name);
}

void TM_ETHERNET_IPIsSetCallback(uint8_t ip_addr1, uint8_t ip_addr2, uint8_t ip_addr3, uint8_t ip_addr4, uint8_t dhcp) {
	/* Called when we have valid IP, it might be static or DHCP */
	
	if (dhcp) {
		/* IP set with DHCP */
		sprintf(str,"IP: %d.%d.%d.%d assigned by DHCP server\n", ip_addr1, ip_addr2, ip_addr3, ip_addr4);
		printf_d(str);
//		TM_USART_Puts(USART6,"IP: %d.%d.%d.%d assigned by DHCP server\n", ip_addr1, ip_addr2, ip_addr3, ip_addr4);
	} else {
		/* Static IP */
		sprintf(str,"IP: %d.%d.%d.%d; STATIC IP used\n", ip_addr1, ip_addr2, ip_addr3, ip_addr4);
		printf_d(str);
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
	printf_d(str);
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
	printf_d(str);
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
	printf_d(str);
//	TM_USART_Puts(USART6,"Gateway: %d.%d.%d.%d\n",
//		TM_ETHERNET_GetNetmask(0),
//		TM_ETHERNET_GetNetmask(1),
//		TM_ETHERNET_GetNetmask(2),
//		TM_ETHERNET_GetNetmask(3)
//	);
	/* Print 100M link status, 1 = 100M, 0 = 10M */
	sprintf(str,"Link 100M: %d\n", TM_ETHERNET.speed_100m);
	printf_d(str);
//	TM_USART_Puts(USART6,"Link 100M: %d\n", TM_ETHERNET.speed_100m);
	/* Print duplex status: 1 = Full, 0 = Half */
	sprintf(str,"Full duplex: %d\n", TM_ETHERNET.full_duplex);
	printf_d(str);
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
void vTimerCallback1( xTimerHandle  pxTimer )
 {
	long lArrayIndex;
     /* Optionally do something if the pxTimer parameter is NULL. */
     configASSERT( pxTimer );

     /* Which timer expired? */
     lArrayIndex = ( long ) pvTimerGetTimerID( pxTimer );

     /* Increment the number of times that pxTimer has expired. */
     lExpireCounters[ lArrayIndex ] += 1;
		flag_485=1;
		if(flag_R11x||flag_R21x||flag_W1D0||flag_R10){
			timerdk1++;
		}
		if(flag_R31||flag_W1D1||flag_R20){ 
			timerdk2++;	
		}
		if(flag_R12||flag_R11||flag_W1D0){ 
			timerdk3++;
		}
		if(flag_R22||flag_R21 ||flag_W1D1){
			timerdk4++;
		}
 }
void vTimerCallback2( xTimerHandle  pxTimer )
 {
	long lArrayIndex;
     /* Optionally do something if the pxTimer parameter is NULL. */
     configASSERT( pxTimer );

     /* Which timer expired? */
     lArrayIndex = ( long ) pvTimerGetTimerID( pxTimer );

     /* Increment the number of times that pxTimer has expired. */
     lExpireCounters[ lArrayIndex ] += 1;
		TM_USART_Puts(USART6,SelectCard);
 }
void 	TM_EXTI_Handler(uint16_t GPIO_Pin) {
	/* Handle external line 0 interrupts */
	if (GPIO_Pin == DTMF_BIT4_PIN) {
		
		if(TM_GPIO_GetInputPinValue(DTMF_BIT0_PORT, DTMF_BIT0_PIN)==0){ // Q1  =0
			if(TM_GPIO_GetInputPinValue(DTMF_BIT1_PORT, DTMF_BIT1_PIN)==0){//  Q2 =0
				if(TM_GPIO_GetInputPinValue(DTMF_BIT2_PORT, DTMF_BIT2_PIN)==0){// Q3 =0
					if(TM_GPIO_GetInputPinValue(DTMF_BIT3_PORT, DTMF_BIT3_PIN)==0)// Q4 =0 Q321=0 
						express = 'D';
					else express = '8';
						}
				else{ // Q1 =0,Q2=0,Q3=1
					if(TM_GPIO_GetInputPinValue(DTMF_BIT3_PORT, DTMF_BIT3_PIN)==0)
								express = '4';
					else express = '#';
						}
			}
			else //Q1=0,Q2=1,
			{
				if(TM_GPIO_GetInputPinValue(DTMF_BIT2_PORT, DTMF_BIT2_PIN)==0){// Q3 =0
					if(TM_GPIO_GetInputPinValue(DTMF_BIT3_PORT, DTMF_BIT3_PIN)==0)// Q4 =0  
						express = '2';
					else express = '0';
						}
				else{ // Q1 =0,Q2=1,Q3=1
					if(TM_GPIO_GetInputPinValue(DTMF_BIT3_PORT, DTMF_BIT3_PIN)==0)
								express = '6';
					else 	express = 'B';
				}
			}
		}
		else{					//Q1=1
				if(TM_GPIO_GetInputPinValue(DTMF_BIT1_PORT, DTMF_BIT1_PIN)==0){//  Q2 =0
					if(TM_GPIO_GetInputPinValue(DTMF_BIT2_PORT, DTMF_BIT2_PIN)==0){// Q3 =0
						if(TM_GPIO_GetInputPinValue(DTMF_BIT3_PORT, DTMF_BIT3_PIN)==0)// Q4 =0 
						express = '1';
						else express = '9';
						}
					else{ // Q1 =1,Q2=0,Q3=1
				if(TM_GPIO_GetInputPinValue(DTMF_BIT3_PORT, DTMF_BIT3_PIN)==0)
								express = '5';
					else express = 'A';
					}
				}
				else //Q1=1,Q2=1,
				{
				if(TM_GPIO_GetInputPinValue(DTMF_BIT2_PORT, DTMF_BIT2_PIN)==0){// Q3 =0
					if(TM_GPIO_GetInputPinValue(DTMF_BIT3_PORT, DTMF_BIT3_PIN)==0)// Q4 =0  
						express = '3';
					else express = '.';
						}
				else{ // Q1 =1,Q2=1,Q3=1
					if(TM_GPIO_GetInputPinValue(DTMF_BIT3_PORT, DTMF_BIT3_PIN)==0)
						express = '7';
					else express = 'C';
				}
			}
		}
		
//		TM_USART_Putc(USART3,express);
		/* Toggle RED led */
//		TM_DISCO_LedToggle(LED_RED);
//		/* Check counter */
		if (++counter >= 10) {
			/* Detach external interrupt for GPIO_Pin_0 no matter on which GPIOx is connected */
			TM_EXTI_Detach(GPIO_Pin_0);
		}
	}
	
//	/* Handle external line 13 interrupts */
	
	if (GPIO_Pin == W1_D0_PIN) { /* run W1 D0 - dieu khien RELAY_DK1_PORT , RELAY_DK1_PIN*/	
		if(!flag_W1D1)
		{ 
		flag_W1D0 =1 ;
		flag_W1D1 =0;
		timeout = value_dip;
		timer_dk1 =0;
		}
		
	}
	if (GPIO_Pin == W1_D1_PIN) {	/* run W1 D1 - dieu khien RELAY_DK2_PORT , RELAY_DK2_PIN*/
		if(!flag_W1D0) {
		flag_W1D1 =1 ;
		flag_W1D0 =0 ;
		timeout = value_dip;
		timer_dk2 =0;
		}
		
	}
	if (GPIO_Pin == W2_D1_PIN) { // ngat cac cong 
		/* run w2 D1 */
	timerdk1=(timeout*2);
	timerdk2=(timeout*2);	
}
}
void 	read_sw_add(void){
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
int 	turn_on_dk1(){
	if(!LEDStatus){
	TM_GPIO_SetPinHigh(RELAY_DK1_PORT,RELAY_DK1_PIN);
	LEDStatus=1;
	return 1;
	}
	else return 0;
}
int 	turn_off_dk1(){
	TM_GPIO_SetPinLow(RELAY_DK1_PORT,RELAY_DK1_PIN);
	LEDStatus=0;
	return 0;
}
int		turn_on_dk2(){
	if(!LEDStatus){
	TM_GPIO_SetPinHigh(RELAY_DK2_PORT,RELAY_DK2_PIN);
	LEDStatus=2;
	return 1;
	}
	else return 0;
}
int 	turn_off_dk2(){
	TM_GPIO_SetPinLow(RELAY_DK2_PORT,RELAY_DK2_PIN);
	LEDStatus=0;
	return 0;
}
int		turn_off_dk3(){
	TM_GPIO_SetPinHigh(RELAY_DK3_PORT,RELAY_DK3_PIN);
	return 0;
}
int 	turn_on_dk3(){
	TM_GPIO_SetPinLow(RELAY_DK3_PORT,RELAY_DK3_PIN);
	return 0;
}
int		turn_off_dk4(){
	TM_GPIO_SetPinHigh(RELAY_DK4_PORT,RELAY_DK4_PIN);
	return 0;
}
int 	turn_on_dk4(){
	TM_GPIO_SetPinLow(RELAY_DK4_PORT,RELAY_DK4_PIN);
	return 0;
}
int 	set_485_send(){
	TM_GPIO_SetPinHigh(CCU_DIR_PORT,CCU_DIR_PIN);
	return 0;
}
int 	set_485_recv(){
	TM_GPIO_SetPinLow(CCU_DIR_PORT,CCU_DIR_PIN);
	return 0;
}
int 	check_vip(char * name){
	int i =0;
	for(i=0;i<2;i++){
	if(strstr(name,vip_id[i]))
		return 1;
	}
	return 0;
}
/* End code*/