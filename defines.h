/**
 *  Defines for your entire project at one place
 * 
 *	@author 	Tilen Majerle
 *	@email		tilen@majerle.eu
 *	@website	http://stm32f4-discovery.com
 *	@version 	v1.0
 *	@ide		Keil uVision 5
 *	@license	GNU GPL v3
 *	
 * |----------------------------------------------------------------------
 * | Copyright (C) Tilen Majerle, 2014
 * | 
 * | This program is free software: you can redistribute it and/or modify
 * | it under the terms of the GNU General Public License as published by
 * | the Free Software Foundation, either version 3 of the License, or
 * | any later version.
 * |  
 * | This program is distributed in the hope that it will be useful,
 * | but WITHOUT ANY WARRANTY; without even the implied warranty of
 * | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * | GNU General Public License for more details.
 * | 
 * | You should have received a copy of the GNU General Public License
 * | along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * |----------------------------------------------------------------------
 */
#ifndef TM_DEFINES_H
#define TM_DEFINES_H

//#define configUSE_TIMERS  1
#define TM_USART1_USE_CUSTOM_IRQ
#define TM_USART3_USE_CUSTOM_IRQ
#define TM_USART6_USE_CUSTOM_IRQ
/* Put your global defines for all libraries here used in your project */
/*
 * 0: DP83848
 * 1: LAN8720A
 * 2: KSZ8081RNA
 */
///* If you want to use LAN8720A PHY, do this in defines.h file: */
//#define ETHERNET_PHY	0
///* MAC ADDRESS: MAC_ADDR0:MAC_ADDR1:MAC_ADDR2:MAC_ADDR3:MAC_ADDR4:MAC_ADDR5 */
///* In case you want to use custom MAC, use parameter in init function */

#define ETHERNET_PHY 0

#define LINK_TIMER_INTERVAL 1000

///* In case you want to use custom IP, use parameter in init function */
//#ifndef IP_ADDR0
//#define IP_ADDR0							192
//#define IP_ADDR1							168
//#define IP_ADDR2							1
//#define IP_ADDR3							120
//#endif
/* NETMASK */
/* In case you want to use custom netmask, use parameter in init function */
//#ifndef NETMASK_ADDR0
//#define NETMASK_ADDR0						255
//#define NETMASK_ADDR1						255
//#define NETMASK_ADDR2						255
//#define NETMASK_ADDR3						0
//#endif

///* Gateway Address */
///* In case you want to use custom gateway, use parameter in init function */
//#ifndef GW_ADDR0
//#define GW_ADDR0							192
//#define GW_ADDR1							168
//#define GW_ADDR2							1
//#define GW_ADDR3							1
//#endif


/* If you need to change settings for your SPI, then uncomment lines you want to change */
/* Replace x with SPI number, 1 - 6 */
#define FATFS_USE_SDIO	1
/* Activate SDIO 1-bit mode */
#define FATFS_SDIO_4BIT 		1
/* Enable Card detect pin */
//#define FATFS_USE_DETECT_PIN 0
/* Use writeprotect pin */
//#define FATFS_USE_WRITEPROTECT_PIN		0
/* define DTMF*/
#ifndef DTMF_BIT0_PORT
#define DTMF_BIT0_PORT				GPIOD
#define DTMF_BIT0_PIN					GPIO_PIN_15
#define DTMF_BIT1_PORT				GPIOD
#define DTMF_BIT1_PIN					GPIO_PIN_5
#define DTMF_BIT2_PORT				GPIOD
#define DTMF_BIT2_PIN					GPIO_PIN_6
#define DTMF_BIT3_PORT				GPIOD
#define DTMF_BIT3_PIN					GPIO_PIN_7

#define DFMF_BIT4_PORT 				GPIOA
#define DTMF_BIT4_PIN					GPIO_PIN_0
#endif
/* define Sw*/
#ifndef ADD_BIT0_PORT
#define ADD_BIT0_PORT					GPIOD
#define ADD_BIT0_PIN					GPIO_PIN_0
#define ADD_BIT1_PORT					GPIOD
#define ADD_BIT1_PIN					GPIO_PIN_1
#define ADD_BIT2_PORT					GPIOD
#define ADD_BIT2_PIN 					GPIO_PIN_4
#define ADD_BIT3_PORT					GPIOB
#define ADD_BIT3_PIN 					GPIO_PIN_5
#define ADD_BIT4_PORT					GPIOB
#define ADD_BIT4_PIN 					GPIO_PIN_6
#define ADD_BIT5_PORT					GPIOB
#define ADD_BIT5_PIN 					GPIO_PIN_7
#define ADD_BIT6_PORT					GPIOB
#define ADD_BIT6_PIN 					GPIO_PIN_8
#define ADD_BIT7_PORT					GPIOA
#define ADD_BIT7_PIN 					GPIO_PIN_12
#endif
/* define RELAY OUTPUT*/
#ifndef RELAY_DK1_PORT
#define RELAY_DK1_PORT				GPIOC
#define RELAY_DK1_PIN					GPIO_PIN_2
#define RELAY_DK2_PORT				GPIOC
#define RELAY_DK2_PIN					GPIO_PIN_0
#define RELAY_DK3_PORT				GPIOD
#define RELAY_DK3_PIN					GPIO_PIN_10
#define RELAY_DK4_PORT				GPIOD
#define RELAY_DK4_PIN					GPIO_PIN_11
///* define Buzzer */
#define BUZZER_PORT						GPIOB
#define BUZZER_PIN						GPIO_PIN_9
#endif
/* define INPUT*/
#ifndef W1_D0_PORT
#define W1_D0_PORT 						GPIOA
#define W1_D0_PIN							GPIO_PIN_3
#define W1_D1_PORT 						GPIOA
#define W1_D1_PIN							GPIO_PIN_4

#define W2_D0_PORT 						GPIOA
#define W2_D0_PIN							GPIO_PIN_5
#define W2_D1_PORT 						GPIOA
#define W2_D1_PIN							GPIO_PIN_6
#endif
/*define INPUT OPT*/
#define OPT_PORT 							GPIOA
#define OPT_PIN								GPIO_PIN_8
/*define 485 DIR*/
#define CCU_DIR_PORT 					GPIOA
#define CCU_DIR_PIN						GPIO_PIN_11
/*define LCD*/
#ifndef HD44780
/* Control pins */
/* RS - Register select pin */
#define HD44780_RS_PORT			GPIOE
#define HD44780_RS_PIN			GPIO_Pin_8
/* E - Enable pin */
#define HD44780_E_PORT			GPIOE
#define HD44780_E_PIN				GPIO_Pin_10
/*RW - Enable pin */
#define HD44780_RW_PORT			GPIOE
#define HD44780_RW_PIN			GPIO_Pin_9

/* D4 - Data 4 pin */
#define HD44780_D4_PORT			GPIOE
#define HD44780_D4_PIN			GPIO_Pin_4
/* D5 - Data 5 pin */
#define HD44780_D5_PORT			GPIOE
#define HD44780_D5_PIN			GPIO_Pin_5
/* D6 - Data 6 pin */
#define HD44780_D6_PORT			GPIOE
#define HD44780_D6_PIN			GPIO_Pin_6
/* D7 - Data 7 pin */
#define HD44780_D7_PORT			GPIOE
#define HD44780_D7_PIN			GPIO_Pin_7
#endif
#endif
