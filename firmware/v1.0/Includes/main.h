/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    UART/UART_Printf/Inc/main.h
  * @author  MCD Application Team
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "stm32c0xx_ll_adc.h"
#include "stm32c0xx_ll_dma.h"
#include "stm32c0xx_ll_pwr.h"


#include "stm32c0xx_ll_rcc.h"
#include "stm32c0xx_ll_crs.h"
#include "stm32c0xx_ll_bus.h"
#include "stm32c0xx_ll_system.h"
#include "stm32c0xx_ll_exti.h"
#include "stm32c0xx_ll_cortex.h"
#include "stm32c0xx_ll_utils.h"
#include "stm32c0xx_ll_pwr.h"
#include "stm32c0xx_ll_dma.h"
#include "stm32c0xx_ll_tim.h"
#include "stm32c0xx_ll_gpio.h"

#include "stm32c0xx_hal.h"

/* Private includes ----------------------------------------------------------*/

#include <stdio.h>
#include "stm32c0xx_nucleo.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/

typedef struct
{
    uint32_t sec;
    uint32_t min;
    uint32_t hr;
    uint32_t day;
    uint32_t mon;
    uint32_t yr;
} datetime_t;

#define USE_TIMEOUT       0

/**
  * @brief Toggle periods for various blinking modes
  */
#define LED_BLINK_FAST       100  /* Toggle period fast (unit: ms) */
#define LED_BLINK_SLOW       500  /* Toggle period slow (unit: ms) */
#define LED_BLINK_ERROR     1000  /* Toggle period very slow for error case (unit: ms) */

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

extern void RFC1_low(void);
extern void RFC1_high(void);
extern void RFC2_low(void);
extern void RFC2_high(void);
extern void RFC3_low(void);
extern void RFC3_high(void);
extern void RFC4_low(void);
extern void RFC4_high(void);


extern void AGC1_low(void);
extern void AGC1_high(void);
extern void AGC2_low(void);
extern void AGC2_high(void);
extern void AGC3_low(void);
extern void AGC3_high(void);
extern void AGC4_low(void);
extern void AGC4_high(void);

extern void unix_us_to_datetime(uint64_t unix_us, datetime_t *dt);
extern uint64_t get_unix_time_us(void);

void Error_Handler(void);

extern int cap;
extern int agc;
extern float snr;
extern int min;
extern int max;
extern int send_array;
extern int send_snr;
extern int search;

#define BV(x)                 (1UL << (x))
#define CLRBIT(reg, bit)      (reg) &= ~BV(bit)
#define SETBIT(reg, bit)      (reg) |=  BV(bit)
#define XORBIT(reg, bit)      (reg) ^=  BV(bit)
#define GETBIT(reg, bit)      (((reg) & BV(bit)) >> bit)


/* IRQ Handler treatment */
void AdcDmaTransferComplete_Callback(void);
void AdcDmaTransferHalf_Callback(void);
void AdcDmaTransferError_Callback(void);
void AdcGrpRegularOverrunError_Callback(void);



/* Private defines -----------------------------------------------------------*/
#define USER_BUTTON_Pin 		LL_GPIO_PIN_13
#define USER_BUTTON_GPIO_Port 	GPIOC
#define USER_BUTTON_EXTI_IRQn 	EXTI4_15_IRQn
#define USER_BUTTON_STATE_ACTIVE   0


#define TIM3_CH1_Pin 			LL_GPIO_PIN_6
#define TIM3_CH1_GPIO_Port 		GPIOA
#define TIM3_CH2_Pin 			LL_GPIO_PIN_7
#define TIM3_CH2_GPIO_Port 		GPIOC


/* Private defines -----------------------------------------------------------*/
#define LED1_Pin 				LL_GPIO_PIN_5
#define LED1_GPIO_Port			 GPIOA
#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
                                                                 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
                                                                0 bit  for subpriority */
#endif


#define RTC_CLOCK_SOURCE_LSI
//#define RTC_CLOCK_SOURCE_LSE

#ifdef RTC_CLOCK_SOURCE_LSI
#define RTC_ASYNCH_PREDIV    0x7F
#define RTC_SYNCH_PREDIV     0xFF
#endif

#ifdef RTC_CLOCK_SOURCE_LSE
#define RTC_ASYNCH_PREDIV  0x7F
#define RTC_SYNCH_PREDIV   0x00FF
#endif


/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
