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
#include "tm_stm32f4_rtc.h"
#include "stm32f4xx_rcc.h"
#include "tcp_echoclient.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

//******************************************************************************
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "timers.h"
#include "croutine.h"
//******************************************************************************

/* Static IP ADDRESS: IP_ADDR0.IP_ADDR1.IP_ADDR2.IP_ADDR3 */
#ifndef MAC_ADDR0
#define MAC_ADDR0							0x08
#define MAC_ADDR1							0x00
#define MAC_ADDR2							0x69
#define MAC_ADDR3							0x02
#define MAC_ADDR4							0x00
#define MAC_ADDR5							0xF0
#endif

void vConfigure(void *pvParameters);
void vLedBlinkRed(void *pvParameters);
void vEthernet(void *pvParameters);
void vMain(void *pvParameters);
void printf_d(char *str);

/* Fatfs object */
FATFS FatFs;
/* File object */
FIL fil;
FRESULT fres;
DSTATUS disk_initialize (BYTE drv);
unsigned int turnsend=0;
unsigned int turnrecv=0;
char ACM[8];
/*Khoi LCD*/
	uint8_t customChar[] = {
//		0x1F,	/*  xxx 11111 */
//		0x11,	/*  xxx 10001 */
//		0x11,	/*  xxx 10001 */
//		0x11,	/*  xxx 10001 */
//		0x11,	/*  xxx 10001 */
//		0x11,	/*  xxx 10001 */
//		0x11,	/*  xxx 10001 */
//		0x1F	/*  xxx 11111 */
		0x00,	/*  xxx 11111 */
		0x00,	/*  xxx 10001 */
		0x00,	/*  xxx 10001 */
		0x00,	/*  xxx 10001 */
		0x00,	/*  xxx 10001 */
		0x00,	/*  xxx 10001 */
		0x00,	/*  xxx 10001 */
		0x00	/*  xxx 11111 */
	};
unsigned char flag_LCD = 0;
char buffer_lcd[32];
/*Khoi COM*/
char 					str[64];
char 					BufferCom1[64];
char 					BufferCom2[64];
char 					BufferCom3[64];
/*Khoi Rtc*/
TM_RTC_Time_t datatime;
static void 	updatime(uint8_t h,uint8_t m,uint8_t s,uint8_t d,uint8_t mon,uint8_t year);
unsigned char fag_updatime=0;
static void 	Sttup(char * buffer);
static void 	Timeup(char * buffer);
/*Khoi SW*/
void 					read_sw_add(void);
unsigned char value_dip =0; /* value DIP switch*/
/*Khoi Ethernet*/
char 					data_tcp[128];
typedef struct {	/* Create simple typedef for DNS controlling */
	uint8_t Working;
	uint8_t HaveIP;
	uint8_t ip[4];
} TM_DNS_t;

TM_DNS_t MyDNS;
uint16_t 			requests_count = 1;
uint8_t 			macaddress[24][6]	=	{
															{0x00,0xE0,0x69,0xF0,0x00,0x01}, 	//1
															{0x00,0xE0,0x69,0xF0,0x00,0x02},	
															{0x00,0xE0,0x69,0xF0,0x00,0x03},
															{0x00,0xE0,0x69,0xF0,0x00,0x04},
															{0x00,0xE0,0x69,0xF0,0x00,0x05},	//5
															{0x00,0xE0,0x69,0xF0,0x00,0x06},
															{0x00,0xE0,0x69,0xF0,0x00,0x07},
															{0x00,0xE0,0x69,0xF0,0x00,0x08},
															{0x00,0xE0,0x69,0xF0,0x00,0x09},
															{0x00,0xE0,0x69,0xF0,0x00,0x0A},	//10
															{0x00,0xE0,0x69,0xF0,0x00,0x0B},
															{0x00,0xE0,0x69,0xF0,0x00,0x0C},
															{0x00,0xE0,0x69,0xF0,0x00,0x0D},
															{0x00,0xE0,0x69,0xF0,0x00,0x0E},
															{0x00,0xE0,0x69,0xF0,0x00,0x0F},	//15
															{0x00,0xE0,0x69,0xF0,0x00,0x10},
															{0x00,0xE0,0x69,0xF0,0x00,0x11},
															{0x00,0xE0,0x69,0xF0,0x00,0x12},
															{0x00,0xE0,0x69,0xF0,0x00,0x13},
															{0x00,0xE0,0x69,0xF0,0x00,0x14},	//20
															{0x00,0xE0,0x69,0xF0,0x00,0x15},
															{0x00,0xE0,0x69,0xF0,0x00,0x16},
															{0x00,0xE0,0x69,0xF0,0x00,0x17},
															{0x00,0xE0,0x69,0xF0,0x00,0x18},	//24
														};
uint8_t 			ipadress[4]			=	{10,142,4,100};
uint8_t 			getwayadress[4]	=	{10,142,4,1};
uint8_t 			netmaskadress[4]=	{255,255,255,0};
uint8_t				serveradress[4]	=	{10,142,4,11};
uint16_t			portadress 		= 8866;
//char revdata[100];
//uint8_t lenrevdata=0;
unsigned int 		status_sv=0;			//lenh dk tu server la 0 1 2
unsigned char 	flag_status_sv=0;	// cho phep nhan lenh dk tu server.
unsigned char 	malenh_sv=0;
TM_TCPCLIENT_t *tcpclient;
/*Khoi process*/
unsigned char  	Process_1=0;		// xu li day len PC nhung.
unsigned char  	Process_2=0;		// xu li day len server.
/*Khoi RFID*/
unsigned char 	flag_RFID = 0;	/*Send SelectCard */		
unsigned char 	flag_RFID1=0;
unsigned char 	flag_RFID2=0;
unsigned char 	flag_wiegand=0;
char 						SelectCard[4]= 	{0xBA,0x02,0x01,0xB9};								// code hex dau doc the da nang
char 						Carpull[7]	=		{0x7F,0x40,0x01,0x20,0x0D,0xC0,0x00};	// code hex nuot the
char 						Carpush[7]	=		{0x7F,0x40,0x01,0x21,0x8D,0xC5,0x00};	// code hex nha the
char 						IDCAR1[7]	=			{0x00,0x00,0x00,0x00,0x00,0x00,0x00};	// luu gia tri IDCAR tu dau doc da nang
char 						UID1[30];
char 						UID1_save[30];
/*Khoi Action*/
void 						ProcessAction(void);
void 						Serveprocess(void);
/*Khoi Wait*/	
void 						WaitPC(unsigned int t);
unsigned char 	flag_PC=0;		// doi pc phan hoi va xu li xong du lieu nhan veroi moi bat
unsigned char 	flag_SV=0;		// doi server phan hoi va xu li xong du lieu nhan ve roi moi bat
/*Khoi VIP*/
char *vip_id[3] = {		// ma the vip
	"b7 7b 87 52",
	"04 4f 97 c6",
	"bc 71 85 49",
	};
/*time out*/
unsigned int 			timeout = 0;	// thoi gian active relay
volatile uint8_t 	counter = 0;
/* Khoi Ban phim */
unsigned char express;			// gia tri nhan duoc tu ban phim
unsigned char flag_dtmf =0;	
/*Khoi 485*/
int LEDStatus=0;						// the hien trang thai cua led hien thi
unsigned char flag_485 = 0;	// xu li trong ham chinh duoc set theo timer
/*Khoi flag*/
unsigned char flag_R10 =0;	// active relay 1
unsigned char flag_R20 =0;	// active relay 2
unsigned char flag_R11 =0;	// active relay 3
unsigned char flag_R21 =0;	// active relay 4
unsigned char flag_test=0;	// su dung khi gui lenh test tu cong com 
unsigned char flag_serve=0;	// su dung khi nhan lenh tu server
unsigned char flag_pass=0; 	// xac nhan di qua
unsigned char flag_passed=0;// xac nhan di qua roi
unsigned char flag_vip=0;		// su kien vip
static int check_vip(char * name);
/* Khoi OUTPUT*/
static int turn_on_dk1(void);
static int turn_off_dk1(void);

static int	turn_on_dk2(void);
static int turn_off_dk2(void);

static int	turn_on_dk3(void);
static int turn_off_dk3(void);

static int	turn_on_dk4(void);
static int turn_off_dk4(void);
/*Khoi thoi gian RELAY*/
int timerdk1 =0;
int timerdk2 =0;
int timerdk3 =0;
int timerdk4 =0;

int timer_dk1 =0;
int timer_dk2 =0;
int timer_dk3 =0;
int timer_dk4 =0;	
/*Khoi timer*/
#define NUM_TIMERS 10
xTimerHandle xTimers[ NUM_TIMERS ];
long lExpireCounters[ NUM_TIMERS ] = { 0 };
void vTimerCallback1( xTimerHandle  pxTimer );
void vTimerCallback2( xTimerHandle  pxTimer );
void vTimerCallback3( xTimerHandle  pxTimer );
void vTimerCallback4( xTimerHandle  pxTimer );
/*Khoi Wiegand*/
uint8_t 				flag_finish_1 = 1,	flag_finish_2 = 1;
__IO uint64_t 	card_1 = 0, 				card_2 = 0;
__IO uint8_t  	count_1 = 0, 				count_2 = 0;
uint8_t crc_26bit(uint32_t allnum_bit,uint32_t wiegand);
uint8_t crc_34bit(uint32_t allnum_bit,uint64_t wiegand);

#define STACK_SIZE_MIN	128	/* usStackDepth	- the stack size DEFINED IN WORDS.*/
const signed char timer_xx[] = "Timer";


			/* Free and total space */

//******************************************************************************
int main(void)
{	
          xTimers[1] = xTimerCreate(timer_xx,         // Just a text name, not used by the kernel.
                                          (500/ portTICK_RATE_MS),     // The timer period in ticks.
                                          pdTRUE,         // The timers will auto-reload themselves when they expire.
                                          (void*)1,     // Assign each timer a unique id equal to its array index.
                                          vTimerCallback1     // Each timer calls the same callback when it expires.
                                      );		
          xTimers[2] = xTimerCreate(timer_xx,         // Just a text name, not used by the kernel.
                                          (150/ portTICK_RATE_MS),     // The timer period in ticks.
                                          pdTRUE,         // The timers will auto-reload themselves when they expire.
                                          (void*)2,     // Assign each timer a unique id equal to its array index.
                                          vTimerCallback2     // Each timer calls the same callback when it expires.
                                      );
          xTimers[3] = xTimerCreate(timer_xx,         // Just a text name, not used by the kernel.
                                          (50/ portTICK_RATE_MS),     // The timer period in ticks.
                                          pdTRUE,         // The timers will auto-reload themselves when they expire.
                                          (void*)3,     // Assign each timer a unique id equal to its array index.
                                          vTimerCallback3     // Each timer calls the same callback when it expires.
                                      );
          xTimers[4] = xTimerCreate(timer_xx,         // Just a text name, not used by the kernel.
                                          (50/ portTICK_RATE_MS),     // The timer period in ticks.
                                          pdTRUE,         // The timers will auto-reload themselves when they expire.
                                          (void*)4,     // Assign each timer a unique id equal to its array index.
                                          vTimerCallback4     // Each timer calls the same callback when it expires.
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
					if( xTimers[3] != NULL )
          {
              // Start the timer.  No block time is specified, and even if one was
              // it would be ignored because the scheduler has not yet been
              // started.
              if( xTimerStart( xTimers[3], 0 ) != pdPASS )
              {
                  // The timer could not be set into the Active state.
              }
						}
//					if( xTimers[4] != NULL )
//          {
//              // Start the timer.  No block time is specified, and even if one was
//              // it would be ignored because the scheduler has not yet been
//              // started.
//              if( xTimerStart( xTimers[4], 0 ) != pdPASS )
//              {
//                  // The timer could not be set into the Active state.
//              }
//						}
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
		/*init SWADD*/
	TM_GPIO_Init(ADD_BIT0_PORT, ADD_BIT0_PIN, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_UP, TM_GPIO_Speed_Medium);	
	TM_GPIO_Init(ADD_BIT1_PORT, ADD_BIT1_PIN, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_UP, TM_GPIO_Speed_Medium);
	TM_GPIO_Init(ADD_BIT2_PORT, ADD_BIT2_PIN, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_UP, TM_GPIO_Speed_Medium);
	TM_GPIO_Init(ADD_BIT3_PORT, ADD_BIT3_PIN, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_UP, TM_GPIO_Speed_Medium);
	TM_GPIO_Init(ADD_BIT4_PORT, ADD_BIT4_PIN, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_UP, TM_GPIO_Speed_Medium);
	TM_GPIO_Init(ADD_BIT5_PORT, ADD_BIT5_PIN, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_UP, TM_GPIO_Speed_Medium);
	TM_GPIO_Init(ADD_BIT6_PORT, ADD_BIT6_PIN, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_UP, TM_GPIO_Speed_Medium);
	TM_GPIO_Init(ADD_BIT7_PORT, ADD_BIT7_PIN, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_UP, TM_GPIO_Speed_Medium);
/* Initialize USART6 at 115200 baud, TX: PC6, RX: PC7 , COM 1 - RFID1 gan cong tac nguon*/ 
	TM_USART_Init(USART6, TM_USART_PinsPack_1, 115200);
/* Initialize USART3 at 115200 baud, TX: PD8, RX: PD9 ,	COM 2 -PC gan ethernet*/
	TM_USART_Init(USART3, TM_USART_PinsPack_3, 9600);
/* Initialize USART1 at 115200 baud, TX: PA9, RX: PA10, CONG 485 */
	TM_USART_Init(USART1, TM_USART_PinsPack_1, 9600);
if (!TM_RTC_Init(TM_RTC_ClockSource_Internal)) {
		/* RTC was first time initialized */
		/* Do your stuff here */
		/* eg. set default time */
	}
	/* Set wakeup interrupt every 1 second */
	TM_RTC_Interrupts(TM_RTC_Int_60s);
/*Task Create Begin*/
	xTaskCreate( vConfigure, (const signed char*)"Configure using DTMF", 
		STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, NULL );
	xTaskCreate( vLedBlinkRed, (const signed char*)"Led Blink Task Red", 
		STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, NULL );
	xTaskCreate( vEthernet, (const signed char*)"Ethernet Running ..", 
		STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, NULL );
	xTaskCreate( vMain, (const signed char*)"Main run ...", 
		STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, NULL );

	vTaskStartScheduler();
}
//******************************************************************************
//******************************************************************************
void vConfigure(void *pvParameters)
{
unsigned int config=0;
unsigned int config1=0;
unsigned int config2=0;
unsigned int config3=0;
unsigned int config4=0;
unsigned int config5=0;
unsigned int config6=0;
uint8_t key;
char str_config[20];
static uint8_t	cnt=0;
		TM_WATCHDOG_Reset();
	for(;;)
	{

if(flag_dtmf==1){
	key = express;
	if((key=='*')&&(config==0)) config =1;
	if(config==1){
		if((config1+config2+config3+config4+config5+config6)==0){
		if((key=='1')&&(config1==0)) config1=1;
		if((key=='2')&&(config2==0)) config2=1;
		if((key=='3')&&(config3==0)) config3=1;
		if((key=='4')&&(config4==0)) config4=1;
		if((key=='5')&&(config5==0)) config5=1;
		if((key=='6')&&(config6==0)) config6=1;
		cnt =0;
		memset(str_config,0,0);
		}
		if(config1==1){
		str_config[cnt] = key;
		cnt++;
		if(key=='#') { 
		printf_d("ket thuc chuoi lenh1\n");
		config =0;
		config1 =0;
		cnt=0;
		sprintf(str,"%s",str_config);
		printf_d(str);
		}}
		if(config2==1){
		str_config[cnt] = key;
		cnt++;
		if(key=='#') { 
		printf_d("ket thuc chuoi lenh2\n");
		config =0;
		config2 =0;
		cnt=0;
		sprintf(str,"%s",str_config);
		printf_d(str);
		}}
		if(config3==1){
		str_config[cnt] = key;
		cnt++;
		if(key=='#') { 
		printf_d("ket thuc chuoi lenh3\n");
		config =0;
		config3 =0;
		cnt=0;
		sprintf(str,"%s",str_config);
		printf_d(str);
		}}
		if(config4==1){
		str_config[cnt] = key;
		cnt++;
		if(key=='#') { 
		printf_d("ket thuc chuoi lenh4\n");
		config =0;
		config4 =0;
		cnt=0;
		sprintf(str,"%s",str_config);
		printf_d(str);
		}}
		if(config5==1){
		str_config[cnt] = key;
		cnt++;
		if(key=='#') { 
		printf_d("ket thuc chuoi lenh5\n");
		config =0;
		config5 =0;
		cnt=0;
		sprintf(str,"%s",str_config);
		printf_d(str);
		}}
		if(config6==1){
		str_config[cnt] = key;
		cnt++;
		if(key=='#') { 
		printf_d("ket thuc chuoi lenh6\n");
		config =0;
		config6 =0;
		cnt=0;
		sprintf(str,"%s",str_config);
		printf_d(str);
		}}		
		}	
	if(key=='#') {
		config=0;
		memset(str_config,'\0',0);
	}
	flag_dtmf =0;	
}
	TM_WATCHDOG_Reset();
}
}
void vLedBlinkRed(void *pvParameters)
{
//	/* Free and total space */
//	uint32_t total, free;
//	static FRESULT rc;   /* Result code */  
//	static UINT bw, br, numread;
//    static BYTE buff[64];
//    BYTE Message[] = "HELLO" ; // message's content
//    TCHAR *FilePath = "MESSAGE.TXT" ; // file path
//		/* Mount drive */
//	if (f_mount(&FatFs, "0:", 1) == FR_OK) {
//		/* Mounted OK, turn on RED LED */
//		
//		/* Try to open file */
//		if (f_open(&fil,FilePath, FA_CREATE_ALWAYS | FA_READ | FA_WRITE | FA_OPEN_ALWAYS) == FR_OK) {
//			/* File opened, turn off RED and turn on GREEN led */
//			
////			/* If we put more than 0 characters (everything OK) */
////			if (f_puts("First string in my file\n", &fil) > 0) {
////				if (TM_FATFS_DriveSize(&total, &free) == FR_OK) {
////					/* Data for drive size are valid */
////				}
////			}
//    rc = f_write(&fil, Message, sizeof(Message)-1, &bw); // write file

//    if (!rc) {
//      sprintf(str,"write success. %u bytes written.\n\r", bw);
//			printf_d(str);
//    }else  {
//      sprintf(str,"write file Failure...error = %u\n\r",rc);
//			printf_d(str);
//    }			
//   sprintf(str,"\nClose the file writeen.\n\r");
//    printf_d(str);
//		taskENTER_CRITICAL ();
//		rc = f_close(&fil); // close file
//		taskEXIT_CRITICAL ();
//    if (!rc) {
//      sprintf(str,"File writeen closed success.\n\r");
//    printf_d(str);
//		}else  {
//      sprintf(str,"File close writeen was failure...error = %u\n\r",rc);
//    printf_d(str);
//		}
//		}
//		
//		/* Unmount drive, don't forget this! */
//		f_mount(0, "0:", 1);
//	}
		TM_WATCHDOG_Reset();
	for(;;)
	{
		if(tcpclient->state==CLIENT_CONNECTED){
		STM_EVAL_LEDToggle(LED_ORANGE);
		vTaskDelay(100 / portTICK_RATE_MS );
		}else{
		STM_EVAL_LEDToggle(LED_ORANGE);
		vTaskDelay(50 / portTICK_RATE_MS );
		}
	TM_WATCHDOG_Reset();		
	}
}

void vEthernet(void *pvParameters)
{	
	read_sw_add();
	ipadress[3]=ipadress[3]+value_dip;
	printf_d("Program starting..\n");
	if (TM_ETHERNET_Init(macaddress[value_dip],ipadress, getwayadress, netmaskadress) == TM_ETHERNET_Result_Ok) {
	printf_d("TM_ETHERNET_Init OK \n");
	}

		/* Reset watchdog */
	TM_WATCHDOG_Reset();
	for(;;)
	{
		/* Update ethernet, call this as fast as possible */
		TM_ETHERNET_Update();
		/* Reset watchdog */
		TM_WATCHDOG_Reset();
	}
	
}

void vMain(void *pvParameters)
{
	/* init DTMF*/
	TM_GPIO_Init(DTMF_BIT0_PORT, DTMF_BIT0_PIN, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_Low);
	TM_GPIO_Init(DTMF_BIT1_PORT, DTMF_BIT1_PIN, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_Low);
	TM_GPIO_Init(DTMF_BIT2_PORT, DTMF_BIT2_PIN, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_Low);
	TM_GPIO_Init(DTMF_BIT3_PORT, DTMF_BIT3_PIN, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_Low);
	/* DTMF*/
		if (TM_EXTI_Attach(DFMF_BIT4_PORT, DTMF_BIT4_PIN, TM_EXTI_Trigger_Rising) == TM_EXTI_Result_Ok) {
		printf_d("DFMF_BIT4 init\n");
	}	
	/*init interrup INPUT*/
		TM_EXTI_Attach(W2_D0_PORT, W2_D0_PIN, TM_EXTI_Trigger_Falling);
		TM_EXTI_Attach(W2_D1_PORT, W2_D1_PIN, TM_EXTI_Trigger_Falling);
		TM_EXTI_Attach(W1_D1_PORT, W1_D1_PIN, TM_EXTI_Trigger_Rising);
		TM_EXTI_Attach(W1_D0_PORT, W1_D0_PIN, TM_EXTI_Trigger_Rising);
	/*init interrup OPT*/
		TM_EXTI_Attach(OPT_PORT, 	OPT_PIN, TM_EXTI_Trigger_Falling);
/* int DIR 485 set = send , reset = recvice*/ 
	TM_GPIO_Init(CCU_DIR_PORT, CCU_DIR_PIN, TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_UP, TM_GPIO_Speed_High);
	TM_GPIO_SetPinHigh(CCU_DIR_PORT,CCU_DIR_PIN);
	/* Init LCD*/
	TM_GPIO_Init(HD44780_RW_PORT, HD44780_RW_PIN, TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_High);
	TM_GPIO_SetPinLow(HD44780_RW_PORT,HD44780_RW_PIN);
	read_sw_add();
	sprintf(ACM,"ACM%03d",value_dip);
	timeout = 1;
	memset(str,'\0',0);
	//Initialize LCD 20 cols x 4 rows
	TM_HD44780_Init(16,4);
	//Save custom character on location 0 in LCD
	TM_HD44780_CreateChar(0, &customChar[0]);    
	//Put string to LCD
	TM_HD44780_Puts(0, 0, "----TIS8-PRO----CreartebyR&D-TIS"); /* 0 dong 1, 1 dong 2*/
	TM_HD44780_Puts(0, 2, "-->Welcome");
	vTaskDelay(2000);
	TM_HD44780_Clear();
	sprintf(buffer_lcd,"Timerout %d", timeout);
	TM_HD44780_Puts(0, 0,buffer_lcd);
	vTaskDelay(2000);
	TM_HD44780_Clear();
	/* Print MAC address to user */
	sprintf(buffer_lcd,"ACM:%s ->IP\n\r%d.%d.%d.%d\n\r%02X-%02X-%02X\n\r%02X-%02X-%02X",
		ACM,
		TM_ETHERNET_GetLocalIP(0),
		TM_ETHERNET_GetLocalIP(1),
		TM_ETHERNET_GetLocalIP(2),
		TM_ETHERNET_GetLocalIP(3),
		TM_ETHERNET_GetMACAddr(0),
		TM_ETHERNET_GetMACAddr(1),
		TM_ETHERNET_GetMACAddr(2),
		TM_ETHERNET_GetMACAddr(3),
		TM_ETHERNET_GetMACAddr(4),
		TM_ETHERNET_GetMACAddr(5)
	);
	TM_HD44780_Puts(0, 0,buffer_lcd);
	vTaskDelay(3000);
	TM_HD44780_Clear();
	sprintf(buffer_lcd,"GW:%d.%d.%d.%d\n\rNet%d.%d.%d.%d",
		TM_ETHERNET_GetGateway(0),
		TM_ETHERNET_GetGateway(1),
		TM_ETHERNET_GetGateway(2),
		TM_ETHERNET_GetGateway(3),
		TM_ETHERNET_GetNetmask(0),
		TM_ETHERNET_GetNetmask(1),
		TM_ETHERNET_GetNetmask(2),
		TM_ETHERNET_GetNetmask(3)
	);
	TM_HD44780_Puts(0, 0,buffer_lcd);
	vTaskDelay(3000);
	TM_HD44780_Clear();
	flag_wiegand=0;	
	flag_RFID1=0;
// configure role mo rong
	turn_off_dk3();
	turn_off_dk4();
	/*end by duc*/
	TM_WATCHDOG_Reset();
	for(;;)
	{
/*process server control*/
/*process 485*/
	if(flag_485){
	flag_485=0;
	if(flag_pass!=0){
	flag_pass++;
	if(flag_pass>20) flag_pass=0;
	}
	if(LEDStatus==0){
	TM_USART_Puts(USART1, "/LED000>\r\n");
	}
	if(LEDStatus==1){
	TM_USART_Puts(USART1, "/LED001>\r\n");
	}
	if(LEDStatus==2){
	TM_USART_Puts(USART1, "/LED002>\r\n");
	}
	if(flag_pass!=0){
		if(tcpclient->state==1)sprintf(buffer_lcd,"Device:%s\n\rConnected....\n\rGaTE:Opened",ACM);
		else sprintf(buffer_lcd,"Device:%s\n\rNotConnect.\n\rGaTE:Opened",ACM);
		TM_HD44780_Puts(0, 0,buffer_lcd);
	}
	else{
		if(tcpclient->state==1)sprintf(buffer_lcd,"Device:%s\n\rConnected....\n\rGaTE:Close\n\rID:............",ACM);
		else sprintf(buffer_lcd,"Device:%s\n\rNotConnect.\n\rGaTE:Close\n\rID:............",ACM);
		TM_HD44780_Puts(0, 0,buffer_lcd);
	}

	/*try configure server*/
	if(flag_passed==1){
	sprintf(data_tcp,"(2,%s,%s,1,,%d-%d-%d/%d-%d-%d)",ACM,UID1_save,datatime.hours,datatime.minutes,datatime.seconds,datatime.date,datatime.month,datatime.year);
	vTaskDelay(10);
	TM_ETHERNETCLIENT_Connect("server",serveradress[0],serveradress[1],serveradress[2],serveradress[3],portadress,&requests_count);
	}	
	if(malenh_sv==2){
		
		}
	}	

///*xu li 1 tien trinh hoan chinh*/
//	if(Process_1!=1 && Process_2!=1) TM_HD44780_Puts(0,3,"Wait for Card"); /* 0 dong 1, 1 dong 2*/
//	else TM_HD44780_Puts(0,3,"Door actived");
/*xu li khi co su kien nhan the*/
if((flag_RFID1==1)&&(flag_pass==0)){			
		Process_1=1;
		Process_2=1;
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
		vTaskDelay(20);
		if(check_vip(UID1)){
			flag_PC=1;
			flag_SV=1;
			flag_R10=1;
			timerdk1 =0;
			flag_R21=1;
			timerdk4 =0;
			Process_1=0;
			Process_2=0;
			flag_vip=1;
		}
		else{
		if(Process_1){
			if(UID1 != NULL){
				vTaskDelay(10);
				//TM_USART_Puts(USART3,UID1);
				sprintf(UID1_save,"ID:%s",UID1);
				TM_HD44780_Puts(0, 3,UID1_save);
				memset(UID1,NULL,0);
			}
		}
		if(Process_2){
			if(UID1 != NULL){
				sprintf(data_tcp,"(1,%s,%s,1,,%d-%d-%d/%d-%d-%d)",ACM,UID1,datatime.hours,datatime.minutes,datatime.seconds,datatime.date,datatime.month,datatime.year);
				TM_ETHERNETCLIENT_Connect("server",serveradress[0],serveradress[1],serveradress[2],serveradress[3],portadress,&requests_count);
				vTaskDelay(10);
				sprintf(UID1_save,"ID:%s",UID1);
				TM_HD44780_Puts(0, 3,UID1_save);
				memset(UID1,NULL,0);
			}
		}
		}
		WaitPC(300);
		flag_RFID1=0;
		
		if(flag_PC){
			flag_PC=0;
			ProcessAction();
			WaitPC(10);
		}
		else Process_1=0;
		
		if(flag_SV){ // duoc bat len trong luc cho doi.
			flag_SV=0;
			Serveprocess();
			WaitPC(10);
		}
		else Process_2=0;		
		flag_RFID1=0;	
		flag_test=0;
		flag_serve=0;
	}

if((flag_wiegand==1&&(flag_pass==0))){
		Process_1=1;
		Process_2=1;		
		sprintf(UID1,"%02x %02x %02x %02x",IDCAR1[3],IDCAR1[2],IDCAR1[1],IDCAR1[0]);
		vTaskDelay(20);
		if(check_vip(UID1)){
			flag_PC=1;
			flag_SV=1;
			flag_R10=1;
			timerdk1 =0;
			flag_R21=1;
			timerdk4 =0;
			Process_1=0;
			Process_2=0;
			flag_vip=1;
		}
		else{
		if(Process_1){
			if(UID1 != NULL){
				vTaskDelay(10);
				//TM_USART_Puts(USART3,UID1);
				sprintf(UID1_save,"ID:%s",UID1);
				TM_HD44780_Puts(0, 3,UID1_save);
				memset(UID1,NULL,0);
			}
		}
		if(Process_2){
			if(UID1 != NULL){
				sprintf(data_tcp,"(1,%s,%s,1,,%d-%d-%d/%d-%d-%d)",ACM,UID1,datatime.hours,datatime.minutes,datatime.seconds,datatime.date,datatime.month,datatime.year);
				TM_ETHERNETCLIENT_Connect("server",serveradress[0],serveradress[1],serveradress[2],serveradress[3],portadress,&requests_count);
				vTaskDelay(10);
				sprintf(UID1_save,"ID:%s",UID1);
				TM_HD44780_Puts(0, 3,UID1_save);
				memset(UID1,NULL,0);
			}
		}
		}
		WaitPC(300);
		flag_wiegand=0;
		
		if(flag_PC){
			flag_PC=0;
			ProcessAction();
			WaitPC(10);
		}
		else Process_1=0;
		
		if(flag_SV){ // duoc bat len trong luc cho doi.
			flag_SV=0;
			Serveprocess();
			WaitPC(10);
		}
		else Process_2=0;		
		flag_wiegand=0;	
		flag_test=0;
		flag_serve=0;
}
if(flag_serve==1){
	flag_serve=0;
	Serveprocess();
}	
if(flag_test==1) {
	flag_test=0;
	ProcessAction();
}

/* Quan li thoi gian cho tung su kien relay actived*/
timer_dk1 = timerdk1/2;	
if (timer_dk1 >= timeout){
			turn_off_dk1();
			flag_R10 =0;
			timerdk1=0;
			timer_dk1=0;
			flag_RFID1=0;
			flag_wiegand=0;
			Process_1=0;
			Process_2=0;
			WaitPC(100);
		}
timer_dk2 = timerdk2/2;
if (timer_dk2 >= timeout){
			turn_off_dk2();
			flag_R20 =0;
			timerdk2=0;
			timer_dk2=0;
			flag_RFID1=0;
			flag_wiegand=0;
			Process_1=0;
			Process_2=0;
			WaitPC(100);
		}
timer_dk3 = timerdk3;
if (timer_dk3 >= 4){
			turn_off_dk3();
			flag_R11 =0;
			timerdk3=0;
			timer_dk3=0;
			if(LEDStatus==0) {Process_1=0;Process_2=0;WaitPC(100);}
		}
timer_dk4 = timerdk4;
if (timer_dk4 >= 4){
			turn_off_dk4();
			flag_R21 =0;
			timer_dk4=0;
			timerdk4=0;
			if(LEDStatus==0) {Process_1=0;Process_2=0;WaitPC(100);}
		}


		TM_WATCHDOG_Reset();
}
	}
//******************************************************************************
/* Tran duc code*/
void  				printf_d(char *str){
	//TM_USART_Puts(USART3,str);
}
void 					ProcessAction(void){
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
		if(strncmp(BufferCom3,"R00",3)==0) // TEST
	{
	TM_USART_Puts(USART3,"OK");
	}
		if(flag_R10)
	{		// active relay 1
		turn_on_dk1();
	}
		if(flag_R20)
	{	// active relay 2
		turn_on_dk2();
	}
		if(flag_R11)
	{	// active relay 3
		turn_on_dk3();
	}
		if(flag_R21)
	{	// active relay 4
		turn_on_dk4();
	}
	
/*Update time*/

	if(flag_R10) timer_dk1 = timerdk1/2;

	if(flag_R20) timer_dk2 = timerdk2/2;

	if(flag_R11) timer_dk3 = timerdk3;

	if(flag_R21) timer_dk4 = timerdk4;
}

void 					Serveprocess(){
	if(flag_status_sv){
	flag_status_sv=0;
	if(	status_sv==1)				//mo relay 1 va 3
	{
		flag_R10=1;
		timerdk1 =0;
		flag_R11=1;
		timerdk3 =0;
	}
		if(status_sv==2)				//mo relay 1 va 4
	{
		flag_R10=1;
		timerdk1 =0;
		flag_R21=1;
		timerdk4 =0;		
	}
		if(status_sv==3)				// mo relay 3
	{
		flag_R11=1;
		timerdk3 =0;		
	}	
		if(status_sv==4)				// mo relay 4
	{
		flag_R21=1;
		timerdk4 =0;		
	}
}
		if(flag_R10)
	{		// active relay 1
		turn_on_dk1();
	}
		if(flag_R20)
	{	// active relay 2
		turn_on_dk2();
	}
		if(flag_R11)
	{	// active relay 3
		turn_on_dk3();
	}
		if(flag_R21)
	{	// active relay 4
		turn_on_dk4();
	}
	
/*Update time*/

	if(flag_R10) timer_dk1 = timerdk1/2;

	if(flag_R20) timer_dk2 = timerdk2/2;

	if(flag_R11) timer_dk3 = timerdk3;

	if(flag_R21) timer_dk4 = timerdk4;
}
void 					WaitPC(unsigned int t){
	unsigned long int n=0;
	while(n<=t){
	vTaskDelay(10);
	if(flag_PC==1||flag_SV==1)
		{
			break;
		}
	n++;
	}
}
void 					TM_USART3_ReceiveHandler(uint8_t c) {	// PC
	static uint8_t cnt=0;
	if(c=='R')cnt=0;
	BufferCom3[cnt]=c;
	if(cnt==2){flag_test=1;} // recive 3 ki tu.
	if((cnt==2)&&(Process_1==1))flag_PC=1;
	if(cnt<48)cnt++;
	else cnt =0;
}
void 					TM_USART6_ReceiveHandler(uint8_t c) {	// RFID1
	static uint8_t cnt=0;
	if(c==0xBD)cnt=0;
	BufferCom1[cnt]=c;
	if((BufferCom1[3]==0x00)&&(cnt==BufferCom1[3]+1)&&cnt)
	{
		if(Process_1==0 && Process_2==0 && flag_pass ==0)flag_RFID1=1; // cho phep xu li du lieu xi tien trinh da ok
		memset(BufferCom1,'\0',0);
	}
	if(cnt<48)cnt++;
}
uint16_t 			TM_ETHERNETCLIENT_CreateHeadersCallback(TM_TCPCLIENT_t* connection, char* buffer, uint16_t buffer_length) {
	sprintf(buffer,"%s",data_tcp);
	turnsend ++;
	return strlen(buffer);
}

void 					TM_ETHERNETCLIENT_ReceiveDataCallback(TM_TCPCLIENT_t* connection, uint8_t* buffer, uint16_t buffer_length, uint16_t total_length) {
	static int i=0;
	static const char *s2 = "(,)";
	static char *buffer1;
	static char *token;
	memset(buffer1,0,0);
	buffer1=(char*)buffer;
	token = strtok(buffer1,s2);
	/* We have available data for connection to receive */
	sprintf(str,"Receiveing %d bytes of data from %s\n", buffer_length, connection->name);
	printf_d(str);
	turnrecv++;

	/* Print data */
	for (i=0;i<buffer_length;i++) {
		/* Print response */
		sprintf(str,"%c", buffer[i]);
		printf_d(str);
		//revdata[i]=buffer[i];
	}
	sprintf(str,"thu hien tach :\n");
	printf_d(str);
	 /* walk through other tokens */
   for (i=0;i<6;i++) 
   {	
			switch(i){
				case 0: {
					sprintf(str,"%s,",token);
					malenh_sv=atoi(str);
					printf_d(str);
				}break;
				case 1: {
					sprintf(str,"%s,",token); 
					printf_d(str);
				}break;
				case 2: {
					sprintf(str,"%s,",token); 	
					printf_d(str);
				}break;
				case 3: {
					sprintf(str,"%s,",token);//cong wiegand mac dinh 1 
					printf_d(str);
				}break;
				case 4: {										// Ma lenh dieu khien tu server.
					sprintf(str,"%s,",token); 
					printf_d(str);
					Sttup("0-0-0-0-1");
					//Sttup(str);
					vTaskDelay(10);
				}break;	
				case 5: {										// datetime server.
					sprintf(str,"%s",token); 
					printf_d(str);
					Timeup(str);
				}break;
				default:break;
			}
			token = strtok(NULL, s2);	
   }
	memset(buffer1,0,0);
	memset(buffer,0,0);
	if(malenh_sv==1){
	flag_serve=1;				// xu li trong ham chinh.
	flag_SV=1;
	}
	if(malenh_sv==2){
		flag_passed=0;
	}
	connection->headers_done = 1;
	tcpclient=connection;
}

void 					TM_ETHERNETCLIENT_ConnectedCallback(TM_TCPCLIENT_t* connection) {
	/* We are connected */
	sprintf(str,"We are connected to %s\n", connection->name);
	printf_d(str);
	tcpclient=connection;
}

void 					TM_ETHERNETCLIENT_ConnectionClosedCallback(TM_TCPCLIENT_t* connection, uint8_t success) {
	/* We are disconnected, done with connection */
	if (success) {
	sprintf(str,"Connection %s was successfully closed. Number of active connections: %d\n", connection->name, *connection->active_connections_count);
	printf_d(str);
	} 
	else {
	sprintf(str,"Connection %s was closed because of error. Number of active connections: %d\n", connection->name, *connection->active_connections_count);
	printf_d(str);
	}	
	/* Increase number of requests */
	tcpclient=connection;
	requests_count++;
}

void 					TM_ETHERNETCLIENT_ErrorCallback(TM_TCPCLIENT_t* connection) {
	/* Print to user */
	sprintf(str,"An error occured on connection %s\n", connection->name);
	printf_d(str);
	tcpclient=connection;
}

void 					TM_ETHERNETCLIENT_ConnectionStartedCallback(TM_TCPCLIENT_t* connection) {
	/* Print to user */
	sprintf(str,"Connection %s has started\n", connection->name);
	printf_d(str);
	tcpclient=connection;
}

/* DNS has IP */
void TM_ETHERNETDNS_FoundCallback(char* host_name, uint8_t ip_addr1, uint8_t ip_addr2, uint8_t ip_addr3, uint8_t ip_addr4) {
//	/* If host name is stm32f4-discovery.com */
//	if (strstr(host_name, "stm32f4-discovery.com")) {
//		/* We have IP */
//		MyDNS.HaveIP = 1;
//		/* Save IP */
//		MyDNS.ip[0] = ip_addr1;
//		MyDNS.ip[1] = ip_addr2;
//		MyDNS.ip[2] = ip_addr3;
//		MyDNS.ip[3] = ip_addr4;
//		/* We are not working anymore */
//		MyDNS.Working = 0;
		
//		/* Print to user */
//		sprintf(str,"We have IP address for %s: %d.%d.%d.%d\n", host_name, ip_addr1, ip_addr2, ip_addr3, ip_addr4);
//		printf_d(str);
//	}
}

/* DNS error callback */
void 					TM_ETHERNETDNS_ErrorCallback(char* host_name) {
//	/* If host name is stm32f4-discovery.com */
//	if (strstr(host_name, "stm32f4-discovery.com")) {
//		/* We have IP */
//		MyDNS.HaveIP = 0;
//		/* We are not working anymore */
//		MyDNS.Working = 0;
//	}
//	/* Print to user */
//	sprintf(str,"DNS has failed to get IP address for %s\n", host_name);
//	printf_d(str);
}


void 					TM_ETHERNET_IPIsSetCallback(uint8_t ip_addr1, uint8_t ip_addr2, uint8_t ip_addr3, uint8_t ip_addr4, uint8_t dhcp) {
	/* Called when we have valid IP, it might be static or DHCP */
	
	if (dhcp) {
		/* IP set with DHCP */
		sprintf(str,"IP: %d.%d.%d.%d assigned by DHCP server\n", ip_addr1, ip_addr2, ip_addr3, ip_addr4);
		printf_d(str);
	} else {
		/* Static IP */
		sprintf(str,"IP: %d.%d.%d.%d; STATIC IP used\n", ip_addr1, ip_addr2, ip_addr3, ip_addr4);
		printf_d(str);
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




	/* Print 100M link status, 1 = 100M, 0 = 10M */
	sprintf(str,"Link 100M: %d\n", TM_ETHERNET.speed_100m);
	printf_d(str);
//	TM_USART_Puts(USART6,"Link 100M: %d\n", TM_ETHERNET.speed_100m);
	/* Print duplex status: 1 = Full, 0 = Half */
	sprintf(str,"Full duplex: %d\n", TM_ETHERNET.full_duplex);
	printf_d(str);
//	TM_USART_Puts(USART6,"Full duplex: %d\n", TM_ETHERNET.full_duplex);
}


void 					TM_ETHERNET_LinkIsDownCallback(void) {
	/* This function will be called when ethernet cable will not be plugged */
	/* It will also be called on initialization if connection is not detected */
}

void 					TM_ETHERNET_LinkIsUpCallback(void) {
	/* Cable has been plugged in back, link is detected */
	/* I suggest you that you reboot MCU here */
	/* Do important stuff before */
}
void 					TM_ETHERNET_SystemResetCallback(void) {
		TM_ETHERNET_Init(NULL, ipadress, getwayadress, netmaskadress);
}
/* Delay 1ms handler */
void 					TM_DELAY_1msHandler(void) {
	/* Time update for ethernet, 1ms */
	/* Add 1ms time for ethernet */
	TM_ETHERNET_TimeUpdate(1);
}
void 					vTimerCallback1( xTimerHandle  pxTimer ){
	long lArrayIndex;
     /* Optionally do something if the pxTimer parameter is NULL. */
     configASSERT( pxTimer );

     /* Which timer expired? */
     lArrayIndex = ( long ) pvTimerGetTimerID( pxTimer );

     /* Increment the number of times that pxTimer has expired. */
     lExpireCounters[ lArrayIndex ] += 1;
		flag_485=1;
		TM_RTC_GetDateTime(&datatime, TM_RTC_Format_BIN);
		if(flag_R10){
			timerdk1++;
		}
		if(flag_R20){ 
			timerdk2++;	
		}
		if(flag_R11){ 
			timerdk3++;
		}
		if(flag_R21){
			timerdk4++;
		}
 }
void 					vTimerCallback2( xTimerHandle  pxTimer ){	// mail
	long lArrayIndex;
     /* Optionally do something if the pxTimer parameter is NULL. */
     configASSERT( pxTimer );

     /* Which timer expired? */
     lArrayIndex = ( long ) pvTimerGetTimerID( pxTimer );

     /* Increment the number of times that pxTimer has expired. */
     lExpireCounters[ lArrayIndex ] += 1;
		TM_USART_Puts(USART6,SelectCard);
 }
void 					vTimerCallback3( xTimerHandle  pxTimer ){ 	// Read wiegand 1. 10ms
		static uint32_t countTimeOut1 = 0;
		long lArrayIndex;
     /* Optionally do something if the pxTimer parameter is NULL. */
     configASSERT( pxTimer );

     /* Which timer expired? */
     lArrayIndex = ( long ) pvTimerGetTimerID( pxTimer );

     /* Increment the number of times that pxTimer has expired. */
     lExpireCounters[ lArrayIndex ] += 1;
		
		 if(count_1 > 0){
			 countTimeOut1++;
		 }
		 if(crc_34bit(count_1,card_1)){
			 uint32_t card_buf = 0;			 
				card_1 >>= 1;
				card_1 &= 0xFFFFFFFF;
				card_buf = (uint32_t)(card_1&0xFFFFFFFF);
				IDCAR1[0] = (uint8_t)(card_buf);
				IDCAR1[1] = (uint8_t)(card_buf >> 8);
				IDCAR1[2] = (uint8_t)(card_buf >> 16);
				IDCAR1[3] = (uint8_t)(card_buf >> 24);
//				sprintf(str,"%02x %02x %02x %02x",IDCAR1[3],IDCAR1[2],IDCAR1[1],IDCAR1[0]);
//				printf_d(str);
				countTimeOut1 = 0;
				card_1 = 0;
				count_1 = 0;
				flag_finish_1 = 1;
				if(flag_pass==0)flag_wiegand=1; // cho phep xu li du lieu xi tien trinh da ok
		 }	
				if(countTimeOut1 >= 5){ // 50ms ( gioi han thoi gian nhan du lieu 
			 flag_finish_1 = 0;
			 countTimeOut1 = 0;
			 card_1 = 0;
			 count_1 = 0;			 
			 flag_finish_1 = 1;
		 }
 }
void 					vTimerCallback4( xTimerHandle  pxTimer ){	// Read wiegang 2. 10ms
		static uint32_t countTimeOut2 = 0;
		long lArrayIndex;
     /* Optionally do something if the pxTimer parameter is NULL. */
     configASSERT( pxTimer );

     /* Which timer expired? */
     lArrayIndex = ( long ) pvTimerGetTimerID( pxTimer );

     /* Increment the number of times that pxTimer has expired. */
     lExpireCounters[ lArrayIndex ] += 1;	

			if(count_2 > 0){
			 countTimeOut2++;
			}
			if(crc_34bit(count_2,card_2)){
			 uint32_t card_buf = 0;
			 uint8_t buffArrCard[4];
			 
				card_1 >>= 1;
				card_1 &= 0xFFFFFFFF;
				card_buf = (uint32_t)(card_1&0xFFFFFFFF);
				buffArrCard[0] = (uint8_t)(card_buf);
				buffArrCard[1] = (uint8_t)(card_buf >> 8);
				buffArrCard[2] = (uint8_t)(card_buf >> 16);
				buffArrCard[3] = (uint8_t)(card_buf >> 24);

				sprintf(str,"w2,%02X%02X%02X%02X",buffArrCard[3],buffArrCard[2],buffArrCard[1],buffArrCard[0]);
				countTimeOut2 = 0;
				card_2 = 0;
				count_2 = 0;
				flag_finish_2 = 1;					
		 }
 }
void 					TM_EXTI_Handler(uint16_t GPIO_Pin) {			// Interrupt ext
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
					else express = '*';
						}
				else{ // Q1 =1,Q2=1,Q3=1
					if(TM_GPIO_GetInputPinValue(DTMF_BIT3_PORT, DTMF_BIT3_PIN)==0)
						express = '7';
					else express = 'C';
				}
			}
		}
		flag_dtmf=1;
		TM_USART_Putc(USART3,express);
		/* Toggle RED led */
//		TM_DISCO_LedToggle(LED_RED);
//		/* Check counter */
//		if (++counter >= 10) {
//			/* Detach external interrupt for GPIO_Pin_0 no matter on which GPIOx is connected */
//			TM_EXTI_Detach(GPIO_Pin_0);
//		}
	}
	
//	/* Handle external line 13 interrupts */
	
	if (GPIO_Pin == W1_D0_PIN) { // run W1 D0
		if(flag_finish_1&&(LEDStatus==0)){
		card_1 <<= 1;
		count_1++;	
	}
	}
	if (GPIO_Pin == W1_D1_PIN) {	// run W1 D1
	if(flag_finish_1&&(LEDStatus==0)){
		card_1 <<= 1;
		card_1 |= 1;
		count_1++;	
	}
		
	}
	if (GPIO_Pin == W2_D0_PIN) {
		if(flag_finish_1&&(LEDStatus==0)){
		card_1 <<= 1;
		count_1++;	
	}
	}
	if (GPIO_Pin == W2_D1_PIN) {
	if(flag_finish_1&&(LEDStatus==0)){
		card_1 <<= 1;
		card_1 |= 1;
		count_1++;	
	}
	}

	if (GPIO_Pin == OPT_PIN) { // ngat cac cong 
	if((flag_pass!=0)&&(flag_vip==0)){
	sprintf(data_tcp,"(2,%s,%s,1,,%d-%d-%d/%d-%d-%d)",ACM,UID1_save,datatime.hours,datatime.minutes,datatime.seconds,datatime.date,datatime.month,datatime.year);
	vTaskDelay(10);
	TM_ETHERNETCLIENT_Connect("server",serveradress[0],serveradress[1],serveradress[2],serveradress[3],portadress,&requests_count);	
	flag_passed=1;
	}
	timerdk1=(timeout*2);
	timerdk2=(timeout*2);
	flag_vip=0;
	flag_pass=0;
	flag_RFID1=0;
	flag_wiegand=0;
}
}
/* Called on wakeup interrupt RTC*/
void 					TM_RTC_RequestHandler() {									// Interrupt rtc
	datatime.unix--;
	fag_updatime=1;
	turn_off_dk3();
	turn_off_dk4();
}
void 					read_sw_add(void){												// Read switch 8 bit
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

	value_dip = 1*sw_add[7]+2*sw_add[6]+4*sw_add[5]+8*sw_add[4]+16*sw_add[3]+32*sw_add[2]+64*sw_add[1]+128*sw_add[0];
	//if(value_dip == 0) value_dip=60; 
}
static int 		turn_on_dk1(void){
	if(!LEDStatus){
	TM_GPIO_SetPinHigh(RELAY_DK1_PORT,RELAY_DK1_PIN);
	TM_GPIO_SetPinHigh(RELAY_DK2_PORT,RELAY_DK2_PIN);
	LEDStatus=1;
	flag_pass=1;
	return 1;
	}
	else return 0;
}
static int 		turn_off_dk1(void){
	TM_GPIO_SetPinLow(RELAY_DK1_PORT,RELAY_DK1_PIN);
	TM_GPIO_SetPinLow(RELAY_DK2_PORT,RELAY_DK2_PIN);
	LEDStatus=0;
	return 0;
}
static int		turn_on_dk2(void){
	if(!LEDStatus){
	TM_GPIO_SetPinHigh(RELAY_DK2_PORT,RELAY_DK2_PIN);
	LEDStatus=2;
	return 1;
	}
	else return 0;
}
static int 		turn_off_dk2(void){
	TM_GPIO_SetPinLow(RELAY_DK2_PORT,RELAY_DK2_PIN);
	LEDStatus=0;
	return 0;
}
static int		turn_off_dk3(void){
	TM_GPIO_SetPinHigh(RELAY_DK3_PORT,RELAY_DK3_PIN);
	return 0;
}
static int 		turn_on_dk3(void){
	TM_GPIO_SetPinLow(RELAY_DK3_PORT,RELAY_DK3_PIN);
	TM_USART_Puts(USART3,Carpull);
	return 0;
}
static int		turn_off_dk4(void){
	TM_GPIO_SetPinHigh(RELAY_DK4_PORT,RELAY_DK4_PIN);
	return 0;
}
static int 		turn_on_dk4(void){
	TM_GPIO_SetPinLow(RELAY_DK4_PORT,RELAY_DK4_PIN);
	TM_USART_Puts(USART3,Carpush);
	return 0;
}
static int 		check_vip(char * name){							// check IDcar
	int i =0;
	for(i=0;i<3;i++){
	if(strstr(name,vip_id[i]))
		return 1;
	}
	return 0;
}
static void 	updatime(uint8_t h,uint8_t m,uint8_t s,uint8_t d,uint8_t mon,uint8_t year){ // Up datetime 
			datatime.hours = h;
			datatime.minutes = m;
			datatime.seconds = s;
			datatime.year = year;
			datatime.month = mon;
			datatime.date = d;
			/* Set new time */
			TM_RTC_SetDateTime(&datatime, TM_RTC_Format_BIN);
}
static void 	Sttup(char * buffer){		// 		
		static int i	// thong tin tk
							,j	// so du tk
							,k	// gia dich vu
							,h	// so luot khach di qua
							,l=0;	//
		sscanf(buffer,"%d-%d-%d-%d-%d",&i,&j,&k,&h,&l);
		status_sv=i;
		if(malenh_sv==1)	flag_status_sv=1;
}
static void 	Timeup(char * buffer){
		static int h, m, s, d, mon, year;
		sscanf(buffer,"%d-%d-%d/%d-%d-%d",&h,&m,&s,&d,&mon,&year);
		updatime(h,m,s,d,mon,year);
}
uint8_t 			crc_26bit(uint32_t allnum_bit,uint32_t wiegand){
	uint32_t wiegand1_tam = 0;
	uint32_t a = 0, b = 0;
	uint32_t icount = 0;
	
	if((wiegand & 0xFC000000) != 0)
		return 0;
	if(allnum_bit != 26)
		return 0;
	wiegand1_tam = wiegand;
	for(icount = 1;icount <= 12;icount++){
		if((wiegand1_tam >>= 1)& 1)
			b++;
	}
	b = (b%2)?0:1;  // CRC le
	for(icount = 1;icount <= 12;icount++){
		if((wiegand1_tam >>= 1)& 1)
			a++;
	}
	a %= 2;	        // CRC chan
	
	wiegand1_tam = wiegand;
	if((a == wiegand1_tam >> 25) && (b == (wiegand1_tam & 1)))
		return 1;
	else
		return 0;
}
uint8_t 			crc_34bit(uint32_t allnum_bit,uint64_t wiegand){
	uint64_t wiegand1_tam = 0;
	uint32_t a = 0, b = 0;
	uint32_t icount = 0;
	
	if(allnum_bit != 34)
		return 0;
	wiegand1_tam = wiegand;
	for(icount = 1;icount <= 16;icount++){
		if((wiegand1_tam >>= 1)& 1)
			b++;
	}
	b = (b%2)?0:1;  // CRC le
	for(icount = 1;icount <= 16;icount++){
		if((wiegand1_tam >>= 1)& 1)
			a++;
	}
	a %= 2;	        // CRC chan
	
	wiegand1_tam = wiegand;
	if((a == wiegand1_tam >> 33) && (b == (wiegand1_tam & 1)))
		return 1;
	else
		return 0;
}

