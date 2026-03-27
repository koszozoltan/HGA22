/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    UART/UART_Printf/Src/main.c
  * @author  MCD Application Team
  * @brief   This example shows how to retarget the C library printf function
  *          to the UART.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "util.h"
#include <string.h>
#include "queue.h"
#include "math.h"
#include "ringbuffer.h"
#include "tcp_server.h"
#include "arm_math.h"
#include "date.h"
#include "adc.h"
#include "iq.h"
#include "goertzel.h"
#include "decoder.h"



/* Private includes ----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/




/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef huart2;

__IO uint32_t uwMeasuredDutyCycle = 0;

/* TIM2 Clock */
static uint32_t TimOutClock = 1;
uint32_t timxPrescaler = 0;
uint32_t timxPeriod = 0;

void Error_Handler2(int);
static void SystemClockHSIUSB48_Config(void);
static inline void board_clock_init(void) ;

int cap = 0;
int agc = 0;
float snr = 0;
int send_array = 0;
int send_snr   = 0;
int search = 0;



int agc_disable = 0;
int min = 65536;
int max = -1;




volatile uint64_t unix_us = 0;// += 1000ULL;


/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void Timer3Init(void);

/* USER CODE BEGIN PFP */

void ADC_Init(void);
void ADC_Activate(void);
void LED_On(void);
void LED_Off(void);
void LED_Toggle(void);


void RFC1_low(void);
void RFC1_high(void);
void RFC2_low(void);
void RFC2_high(void);
void RFC3_low(void);
void RFC3_high(void);
void RFC4_low(void);
void RFC4_high(void);



void unix_us_to_datetime(uint64_t unix_us, datetime_t *dt);
uint64_t datetime_to_unix_us(const datetime_t *dt, uint32_t usec);

extern void udp_server_init(void);
extern void ShellTask(void);

int Get_HW_Agc(void);



#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)












uint16_t signal_buffer[128];
stream_q_t q_signal;


/* Private user code ---------------------------------------------------------*/


size_t board_get_unique_id(uint8_t id[], size_t max_len) {
  (void) max_len;
  volatile uint32_t * stm32_uuid = (volatile uint32_t *) UID_BASE;
  uint32_t* id32 = (uint32_t*) (uintptr_t) id;
  uint8_t const len = 12;

  id32[0] = stm32_uuid[0];
  id32[1] = stm32_uuid[1];
  id32[2] = stm32_uuid[2];

  return len;
}






extern void loop_tinyusb (void);
extern int main_tinyusb(void);

//void init_demods(void);


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  HAL_Init();
  BSP_LED_Init(LED1);
  BSP_LED_Init(LED2);
  //SystemClockHSIUSB48_Config() ;
  board_clock_init();

  HAL_Init();


  sq_init(&q_signal, (int16_t*)signal_buffer, 128);
 // sq_init(&q_byte,   (int16_t*)byte_buffer, 32);
 // sq_init(&q_fsk,    (int16_t*)fsk_buffer, 8);
  #if 0
  sq_init(&q_voice,voice_buffer,8192);
  #endif




  MX_GPIO_Init();
  MX_USART2_UART_Init();
  Timer3Init();


  ADC_Init();





  Uart2_printf("\r\nHGA22 RECEIVER RNDIS V1.0>\r\n");
  Uart2_printf("%s %s\r\n", BUILD_DATE);

  Uart2_rx_printf("-h\r\n");
  Uart2_rx_printf("?\r\n");


  {
	  datetime_t dt;

	   dt.yr  = 2026;
	   dt.mon = 1;
	   dt.day = 1;
	   dt.hr  = 0;
	   dt.min = 0;
	   dt.sec = 0;

	   uint64_t now = datetime_to_unix_us(&dt, 0);

	   /* RTC -> dt mezők feltöltése */
	   NVIC_DisableIRQ(SysTick_IRQn);
	   unix_us = now;
	   NVIC_EnableIRQ(SysTick_IRQn);
  }


  RFC1_low();
  RFC2_low();
  RFC3_low();
  RFC4_low();

  AGC1_high();
  AGC2_high();
  AGC3_low();
  AGC4_low();

	__HAL_RCC_USB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_SYSCFG_CLK_ENABLE();
	__HAL_RCC_PWR_CLK_ENABLE();
#if 1
  main_tinyusb();
  udp_server_init();

#endif

init_demods();


  while(1){

	  ShellTask();

	  loop_tinyusb();


	  if(ubDmaTransferStatus == 1) {
		  //LED_On();




		  static int send_snr_cnt = 0;
		  if (send_snr){
			  if (++send_snr_cnt >= 1000){
				  send_snr_cnt = 0;
				  Uart2_printf("SNR : %d , min %d, max %d\r\n", (int)snr, (int)min, (int) max );
			  }
		  }

		  static int buffer_size = (SAMPLE_RATE / 1000) * 4;

		   if (sq_size(&q_signal) >= buffer_size){

			   int16_t buffer[buffer_size];


			   sq_peek(&q_signal,buffer,buffer_size);
			   sq_discard(&q_signal,(SAMPLE_RATE / 1000));

			   if (search){
                    if (++send_snr_cnt >= 200){
                        send_snr_cnt = 0;
                        search_freq((uint16_t *)buffer,buffer_size);
                    }
			   }

			   loop(buffer,(SAMPLE_RATE / 1000) * 4);
			   //LED_Off();

		   }

		  //LED_Off();
		  ubDmaTransferStatus = 0;
	  }


  }


}

uint32_t board_button_read(void) {
	return USER_BUTTON_STATE_ACTIVE == HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin);
}



static void MX_TIM3_Init(void) {



  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

  TIM_InitStruct.Prescaler = timxPrescaler;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = timxPeriod;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM3, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM3);
  LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH2);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = ((timxPeriod + 1 ) / 2);
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM3, LL_TIM_CHANNEL_CH2);
  LL_TIM_SetTriggerOutput(TIM3, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM3);

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOC);

  GPIO_InitStruct.Pin = TIM3_CH2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(TIM3_CH2_GPIO_Port, &GPIO_InitStruct);

}


static void Timer3Init(void) {

 /* - Set the pre-scaler value to have TIM3 counter clock equal to 10 kHz  */
 /* - Set the auto-reload value to have a counter frequency of 100 Hz        */
 /* TIM3CLK = SystemCoreClock / (APB prescaler & multiplier)               */
 TimOutClock = SystemCoreClock/1;
 timxPrescaler = __LL_TIM_CALC_PSC (SystemCoreClock, 80000000);
 timxPeriod = __LL_TIM_CALC_ARR    (TimOutClock,     timxPrescaler,   TIMER3_FREQ);

 /* Initialize all configured peripherals */
 MX_TIM3_Init();


 /**************************/
 /* TIM3 interrupts set-up */
 /**************************/
 /* Enable the capture/compare interrupt for channel 1 */
 LL_TIM_EnableIT_CC1(TIM3);

 /**********************************/
 /* Start output signal generation */
 /**********************************/
 /* Enable output channel 1 */
 LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH2);

 /* Enable counter */
 LL_TIM_EnableCounter(TIM3);

 /* Force update generation */
 LL_TIM_GenerateEvent_UPDATE(TIM3);
}



static void SystemClockHSIUSB48_Config(void)
{

  //RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_CRSInitTypeDef RCC_CRSInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */



   {


      RCC_OscInitTypeDef RCC_OscInitStruct = {0};
      RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
      RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;

      if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
      {
        Error_Handler2(3);

      }

  }



  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSIUSB48;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler2(5);
  }

  /** Enable the CRS APB clock
  */
  __HAL_RCC_CRS_CLK_ENABLE();

  /** Configures CRS
  */
  RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;
  RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_USB;
  RCC_CRSInitStruct.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  RCC_CRSInitStruct.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000,1000);
  RCC_CRSInitStruct.ErrorLimitValue = 34;
  RCC_CRSInitStruct.HSI48CalibrationValue = 32;

  HAL_RCCEx_CRSConfig(&RCC_CRSInitStruct);


  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  LL_Init1msTick(48000000);
  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(48000000);

}

static inline void board_clock_init(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSE;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  LL_Init1msTick(48000000);
  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(48000000);

}


static void SystemClockHSE_Config(void) {

	board_clock_init();
	return;


  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* -1- Enable HSE  Oscillator */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* -2- Initializes the CPU, AHB and APB buses clocks */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
 /* -3- Optional: Disable HSIUSB48  Oscillator (if the HSIUSB48  is no more needed by the application) */
  RCC_OscInitStruct.OscillatorType                = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State                      = RCC_HSI48_OFF;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  LL_Init1msTick(48000000);
  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(48000000);

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {

  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler2(1);
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler2(2);
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void){

  huart2.Instance 				= USART2;
  huart2.Init.BaudRate 			= 115200;//921600;// 115200;
  huart2.Init.WordLength 		= UART_WORDLENGTH_8B;
  huart2.Init.StopBits 			= UART_STOPBITS_1;
  huart2.Init.Parity 			= UART_PARITY_NONE;
  huart2.Init.Mode 				= UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl 		= UART_HWCONTROL_NONE;
  huart2.Init.OverSampling	 	= UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling 	= UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler 	= UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

  Uart2BufferInit();
}


/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE {
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART2 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
  //Uart2Putchar(ch);
  return ch;
}





/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void){
  while (1)
  {
    /* Toggle LED1 for error */
    BSP_LED_Toggle(LED1);
    HAL_Delay(200);
    //printf("Error");
  }

}

void Error_Handler2(int cnt){
  while (1)
  {
    BSP_LED_Off(LED1);

   for (int i = 0; i < cnt *2; i++){
    BSP_LED_Toggle(LED1);
    HAL_Delay(100);
   }
    HAL_Delay(1000);
  }

}



/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void) {
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

  /**/
  LL_GPIO_ResetOutputPin(LED1_GPIO_Port, LED1_Pin);

  /**/
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);


  // D4 - PB5
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
  GPIO_InitStruct.Pin 			= LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode 			= LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed 		= LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType 	= LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull 			= LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_5);


  //A2;
  GPIO_InitStruct.Pin 			= LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode 			= LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed 		= LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType 	= LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull 			= LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4);

  //const int RFC2Pin = A3;
   LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
   GPIO_InitStruct.Pin 			= LL_GPIO_PIN_0;
   GPIO_InitStruct.Mode 		= LL_GPIO_MODE_OUTPUT;
   GPIO_InitStruct.Speed 		= LL_GPIO_SPEED_FREQ_LOW;
   GPIO_InitStruct.OutputType 	= LL_GPIO_OUTPUT_PUSHPULL;
   GPIO_InitStruct.Pull 		= LL_GPIO_PULL_NO;
   LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
   LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_0);
  //const int RFC3Pin = A4;
     LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOC);
     GPIO_InitStruct.Pin 		= LL_GPIO_PIN_4;
     GPIO_InitStruct.Mode 		= LL_GPIO_MODE_OUTPUT;
     GPIO_InitStruct.Speed 		= LL_GPIO_SPEED_FREQ_LOW;
     GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
     GPIO_InitStruct.Pull 		= LL_GPIO_PULL_NO;
     LL_GPIO_Init(GPIOC, &GPIO_InitStruct);
     LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_4);
/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */

     // Button
     {
       GPIO_InitTypeDef gpio_init = { 0 };
       gpio_init.Pin = USER_BUTTON_Pin;
       gpio_init.Mode = GPIO_MODE_INPUT;
       gpio_init.Pull = USER_BUTTON_STATE_ACTIVE ? GPIO_PULLDOWN : GPIO_PULLUP;
       HAL_GPIO_Init(USER_BUTTON_GPIO_Port, &gpio_init);
     }

}

void AGC1_low(void){
	CLRBIT(agc,0);
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin			= LL_GPIO_PIN_5;
	GPIO_InitStruct.Mode 		= LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed 		= LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType 	= LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull 		= LL_GPIO_PULL_NO;
	LL_GPIO_Init				(GPIOB, &GPIO_InitStruct);
	LL_GPIO_ResetOutputPin		(GPIOB, LL_GPIO_PIN_5);
}

void AGC1_high(void){
	SETBIT(agc,0);
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin			= LL_GPIO_PIN_5;
	GPIO_InitStruct.Mode 		= LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Speed 		= LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Pull 		= LL_GPIO_PULL_NO;
	LL_GPIO_Init				(GPIOB, &GPIO_InitStruct);
}

void AGC2_low(void){
	CLRBIT(agc,1);
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin			= LL_GPIO_PIN_4;
	GPIO_InitStruct.Mode 		= LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed 		= LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType 	= LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull 		= LL_GPIO_PULL_NO;
	LL_GPIO_Init				(GPIOB, &GPIO_InitStruct);
	LL_GPIO_ResetOutputPin		(GPIOB, LL_GPIO_PIN_4);
}

void AGC2_high(void){
	SETBIT(agc,1);
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin			= LL_GPIO_PIN_4;
	GPIO_InitStruct.Mode 		= LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Speed 		= LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Pull 		= LL_GPIO_PULL_NO;
	LL_GPIO_Init				(GPIOB, &GPIO_InitStruct);
}


void AGC3_low(void){
	CLRBIT(agc,2);
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin			= LL_GPIO_PIN_8;
	GPIO_InitStruct.Mode 		= LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed 		= LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType 	= LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull 		= LL_GPIO_PULL_NO;
	LL_GPIO_Init				(GPIOC, &GPIO_InitStruct);
	LL_GPIO_ResetOutputPin		(GPIOC, LL_GPIO_PIN_8);
}

void AGC3_high(void){
	SETBIT(agc,2);
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin			= LL_GPIO_PIN_8;
	GPIO_InitStruct.Mode 		= LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Speed 		= LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Pull 		= LL_GPIO_PULL_NO;
	LL_GPIO_Init				(GPIOC, &GPIO_InitStruct);
}

void AGC4_low(void){
	CLRBIT(agc,3);
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin			= LL_GPIO_PIN_8;
	GPIO_InitStruct.Mode 		= LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed 		= LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType 	= LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull 		= LL_GPIO_PULL_NO;
	LL_GPIO_Init				(GPIOA, &GPIO_InitStruct);
	LL_GPIO_ResetOutputPin		(GPIOA, LL_GPIO_PIN_8);
}

void AGC4_high(void){
	SETBIT(agc,3);
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin			= LL_GPIO_PIN_8;
	GPIO_InitStruct.Mode 		= LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Speed 		= LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Pull 		= LL_GPIO_PULL_NO;
	LL_GPIO_Init				(GPIOA, &GPIO_InitStruct);
}

void RFC1_low(void){
	CLRBIT(cap,0);

	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin			= LL_GPIO_PIN_4;
	GPIO_InitStruct.Mode 		= LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed 		= LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType 	= LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull 		= LL_GPIO_PULL_NO;
	LL_GPIO_Init				(GPIOA, &GPIO_InitStruct);
	LL_GPIO_ResetOutputPin		(GPIOA, LL_GPIO_PIN_4);
}

void RFC1_high(void){
	SETBIT(cap,0);
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin			= LL_GPIO_PIN_4;
	GPIO_InitStruct.Mode 		= LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Speed 		= LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Pull 		= LL_GPIO_PULL_NO;
	LL_GPIO_Init				(GPIOA, &GPIO_InitStruct);
}

void RFC2_low(void){
	CLRBIT(cap,1);
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin			= LL_GPIO_PIN_0;
	GPIO_InitStruct.Mode 		= LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed 		= LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType 	= LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull 		= LL_GPIO_PULL_NO;
	LL_GPIO_Init				(GPIOB, &GPIO_InitStruct);
	LL_GPIO_ResetOutputPin		(GPIOB, LL_GPIO_PIN_0);
}
void RFC2_high(void){
	SETBIT(cap,1);
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin			= LL_GPIO_PIN_0;
	GPIO_InitStruct.Mode 		= LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Speed 		= LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Pull 		= LL_GPIO_PULL_NO;
	LL_GPIO_Init				(GPIOB, &GPIO_InitStruct);
}
void RFC3_low(void){
	CLRBIT(cap,2);
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin			= LL_GPIO_PIN_4;
	GPIO_InitStruct.Mode 		= LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed 		= LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType 	= LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull 		= LL_GPIO_PULL_NO;
	LL_GPIO_Init				(GPIOC, &GPIO_InitStruct);
	LL_GPIO_ResetOutputPin		(GPIOC, LL_GPIO_PIN_4);
}

void RFC3_high(void){
	SETBIT(cap,2);
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin			= LL_GPIO_PIN_4;
	GPIO_InitStruct.Mode 		= LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Speed 		= LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Pull 		= LL_GPIO_PULL_NO;
	LL_GPIO_Init				(GPIOC, &GPIO_InitStruct);
}

void RFC4_low(void){
	CLRBIT(cap,3);
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin			= LL_GPIO_PIN_5;
	GPIO_InitStruct.Mode 		= LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed 		= LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType 	= LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull 		= LL_GPIO_PULL_NO;
	LL_GPIO_Init				(GPIOC, &GPIO_InitStruct);
	LL_GPIO_ResetOutputPin		(GPIOC, LL_GPIO_PIN_5);
}

void RFC4_high(void){
	SETBIT(cap,3);
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin			= LL_GPIO_PIN_5;
	GPIO_InitStruct.Mode 		= LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Speed 		= LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Pull 		= LL_GPIO_PULL_NO;
	LL_GPIO_Init				(GPIOC, &GPIO_InitStruct);
}




/**
  * @brief  Turn-on LED1.
  * @param  None
  * @retval None
  */
void LED_On(void) {
  /* Turn LED on */
  LL_GPIO_SetOutputPin(LED1_GPIO_Port, LED1_Pin);
}

/**
  * @brief  Turn-off LED1.
  * @param  None
  * @retval None
  */
void LED_Off(void) {
  /* Turn LED off */
  LL_GPIO_ResetOutputPin(LED1_GPIO_Port, LED1_Pin);
}

/**
  * @brief  Toggle LED1.
  * @param  None
  * @retval None
  */
void LED_Toggle(void) {
  /* Turn LED off */
  LL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
}

/******************************************************************************/
/*   USER IRQ HANDLER TREATMENT                                               */
/******************************************************************************/

/**
  * @brief  DMA transfer complete callback
  * @note   This function is executed when the transfer complete interrupt
  *         is generated
  * @retval None
  */

//void AdcDmaTransferComplete_Callback() {
//
//
//	sq_push(&q_signal, (int16_t *)uhADCxConvertedData, ADC_CONVERTED_DATA_BUFFER_SIZE);
//
//	ubDmaTransferStatus = 1;
//	ubDmaTransferCnt++;
//}
//
//
///**
//  * @brief  DMA half transfer callback
//  * @note   This function is executed when the half transfer interrupt
//  *         is generated
//  * @retval None
//  */
//void AdcDmaTransferHalf_Callback() {
//
//}
//
//
//
///**
//  * @brief  DMA transfer error callback
//  * @note   This function is executed when the transfer error interrupt
//  *         is generated during DMA transfer
//  * @retval None
//  */
//void AdcDmaTransferError_Callback() {
//  if(ubDmaTransferStatus == 1)
//  {
//    /* Update status variable of DMA transfer */
//    ubDmaTransferStatus = 0;
//  }
//
//  /* Error detected during DMA transfer */
//  Error_Handler();
//}
//
///**
//  * @brief  ADC group regular overrun interruption callback
//  * @note   This function is executed when ADC group regular
//  *         overrun error occurs.
//  * @retval None
//  */
//void AdcGrpRegularOverrunError_Callback(void) {
//  /* Note: Disable ADC interruption that caused this error before entering in
//           infinite loop below. */
//
//  /* In case of error due to overrun: Disable ADC group regular overrun interruption */
//  LL_ADC_DisableIT_OVR(ADC1);
//
//  /* Error reporting */
//  Error_Handler();
//}
//








