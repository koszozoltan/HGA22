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
#include <string.h>
#include "queue.h"
#include "math.h"
#include "ringbuffer.h"
#include "tcp_server.h"
#include "arm_math.h"
#include "date.h"
#include "adc.h"

/* Private includes ----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

///* Timeout values for ADC operations. */
//  /* (calibration, enable settling time, disable settling time, ...)          */
//  /* Values defined to be higher than worst cases: low clock frequency,       */
//  /* maximum prescalers.                                                      */
//  /* Note: ADC channel configuration ready (ADC_CHANNEL_CONF_RDY_TIMEOUT_MS)  */
//  /*       is added in CubeMx code section.                                   */
//  /* Unit: ms                                                                 */
//  #define ADC_CALIBRATION_TIMEOUT_MS       (   1UL)
//  #define ADC_ENABLE_TIMEOUT_MS            (   1UL)
//  #define ADC_DISABLE_TIMEOUT_MS           (   1UL)
//  #define ADC_STOP_CONVERSION_TIMEOUT_MS   (   1UL)
//  #define ADC_CONVERSION_TIMEOUT_MS        (4000UL)
//
//  /* Delay between ADC end of calibration and ADC enable.                     */
//  /* Delay estimation in CPU cycles: Case of ADC enable done                  */
//  /* immediately after ADC calibration, ADC clock setting slow                */
//  /* (LL_ADC_CLOCK_ASYNC_DIV32). Use a higher delay if ratio                  */
//  /* (CPU clock / ADC clock) is above 32.                                     */
//  #define ADC_DELAY_CALIB_ENABLE_CPU_CYCLES  (LL_ADC_DELAY_CALIB_ENABLE_ADC_CYCLES * 32)
//
///* Definitions of environment analog values */
//  /* Value of analog reference voltage (Vref+), connected to analog voltage   */
//  /* supply Vdda (unit: mV).                                                  */
//  #define VDDA_APPLI                       (3300UL)
//
///* Definitions of data related to this example */
//  /* Definition of ADCx conversions data table size */
////  #define ADC_CONVERTED_DATA_BUFFER_SIZE   (  64UL)
//  #define ADC_CONVERTED_DATA_BUFFER_SIZE   (  8UL)
//
//
//  /* Init variable out of expected ADC conversion data range */
//  #define VAR_CONVERTED_DATA_INIT_VALUE    (__LL_ADC_DIGITAL_SCALE(LL_ADC_RESOLUTION_12B) + 1)
//
//
//
//
//  /* Parameters of time base (used as ADC conversion trigger) */
//  /* Time base frequency (unit: Hz). With a timer 16 bits and time base       */
//  /* freq max 32kHz, range is [min=1Hz, max=32kHz].                           */
//  #define TIMER_FREQUENCY_HZ               (8000UL)
//
////  #define TIMER_FREQUENCY_HZ               (8000UL)
//
//  /* Time base range frequency maximum (unit: Hz).*/
//  /* With a timer 16 bits, minimum frequency will be 1/32000 times this value.*/
//  #define TIMER_FREQUENCY_RANGE_MAX_HZ    (32000UL)


/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef huart2;

/* Duty cycle index */
//static uint8_t iDutyCycle = 0;

/* Measured duty cycle */
__IO uint32_t uwMeasuredDutyCycle = 0;

/* TIM2 Clock */
static uint32_t TimOutClock = 1;
uint32_t timxPrescaler = 0;
uint32_t timxPeriod = 0;



int cap = 0;
int agc = 0;
float snr = 0;
int send_array = 0;
int send_snr   = 0;
int search = 0;

int max_snr1 =0;
int max_snr2 =0;
int min_snr1 =0;
int min_snr2 =0;


volatile uint64_t unix_us = 0;// += 1000ULL;


/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void Timer3Init(void);

//static void MX_DMA_Init(void);
//static void MX_ADC1_Init(void);
//static void MX_TIM1_Init(void);
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


//__IO uint16_t uhADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE];
//
//__IO uint8_t ubDmaTransferStatus = 2U; /* Variable set into DMA interruption callback */
//
//
//__IO uint8_t ubDmaTransferCnt= 0; /* Variable set into DMA interruption callback */

#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)


//#define SAMPLE_RATE   8000
//#define DEPTH         32        // 4 ms
//#define OFFSET_BITS   5         // többségi döntés
//#define BITBUF_SIZE   16

#define SAMPLE_RATE TIMER_FREQUENCY_HZ

#define DEPTH         (ADC_CONVERTED_DATA_BUFFER_SIZE*4)        // 4 ms
#define OFFSET_BITS   5
#define BITBUF_SIZE   16


//#define FREQ_ONE      (1750)
//#define FREQ_TWO      (2050)
//#define TIMER3_FREQ   (133600)


#define FREQ_ONE      (1730)
#define FREQ_TWO      (2070)
#define TIMER3_FREQ   (133600)


#define NOISE_FLOOR   5000
#ifndef PI
    # define PI           3.14159265358979323846  /* pi */
#endif

float coef1;
float coef2;

int min = 65536;
int max = -1;





uint8_t 	bit_to_byte(uint8_t bit, uint8_t *out);
uint32_t 	getBits(const uint8_t *data, uint32_t pos, uint32_t len);
void 		parse_frame(uint8_t *frame);
#if 0
float 		Goertzel_coef(float freq, int samp_rate);
float 		Goertzel_mag(uint16_t samples[], int depth, float coef, int adc_midpoint);
#else
int32_t Goertzel_coef(float freq, int samp_rate);
int32_t Goertzel_mag(const uint16_t *samples,
                            int depth,
                            int32_t coef,
                            int32_t adc_midpoint);


int16_t Goertzel_coef_q15(float freq, int samp_rate);
int32_t Goertzel_mag_fast_q15(const uint16_t *samples,
                              int depth,
                              int16_t coef,
                              int32_t adc_mid);



#endif

void	 	loop(int16_t *buffer, int depth);
void	 	loop2(int16_t *buffer, int depth);


uint16_t signal_buffer[128];
stream_q_t q_signal;

uint16_t byte_buffer[32]; //
stream_q_t q_byte;


uint16_t fsk_buffer[8];
stream_q_t q_fsk;
#if 0
uint16_t voice_buffer[8192];
stream_q_t q_voice;
#endif

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

#define FFT_SIZE 32  // A példád alapján 32

// A bemenetnek és a kimenetnek is 2*FFT_SIZE méretűnek kell lennie!


void run_fft_example(uint16_t samples[], int depth) {
#if 0

    static q15_t testInput_q15[FFT_SIZE * 2];
    static q15_t testOutput_q15[FFT_SIZE * 2];
    // Statikus struktúra használata (vagy arm_rfft_init_q15 hívása egyszer az elején)
    extern const arm_rfft_instance_q15 arm_rfft_sR_q15_len32;
    const arm_rfft_instance_q15 *S = &arm_rfft_sR_q15_len32;
    arm_status status = ARM_MATH_SUCCESS;

    if (depth != FFT_SIZE) return;

    // 1. Adatok másolása és konvertálása
    // Fontos: a testInput_q15 többi részét nullázni kell, ha az RFFT belsőleg használja
    for(int i = 0; i < FFT_SIZE; i++) {
        // Ha a samples[] 0-4095 (ADC), érdemes DC eltolást kivonni vagy skálázni
        testInput_q15[i] = (q15_t)(samples[i] >> 1); // Skálázás a túlcsordulás ellen
    }
    // A puffer második felét érdemes nullázni
    for(int i = FFT_SIZE; i < FFT_SIZE * 2; i++) {
        testInput_q15[i] = 0;
    }

    // 2. RFFT végrehajtása
    // Bemenet: testInput_q15 (valós adatok)
    // Kimenet: testOutput_q15 (komplex adatok: R0, I0, R1, I1...)
    arm_rfft_q15(S, testInput_q15, testOutput_q15);

    // 3. Magnitúdó számítása
    // A kimenet komplex, ebből csinálunk valós spektrumot.
    // Az eredmény az első FFT_SIZE elembe kerül.
    arm_cmplx_mag_q15(testOutput_q15, testOutput_q15, FFT_SIZE);

    // 4. Eredmények kiírása
    // Real FFT esetén a spektrum szimmetrikus, csak az első fele (FFT_SIZE/2) hasznos információ!
    for(int i = 0; i < FFT_SIZE / 2; i++){
        Uart2_printf("%d;", testOutput_q15[i]);
    }
    Uart2_printf("\r\n");

#else
#warning FFT is disabled
#endif
}


int search_freq ( uint16_t samples[], int depth){
#if 0
	run_fft_example ( samples, depth);
	return 1;
#endif

#ifndef ITEMNUM
#define ITEMNUM(x)          (sizeof(x) / sizeof((x)[0]))
#endif // ITEMNUM

     float max = -1;
     int max_id = 0;
     float TONE_ONE_MAG;
     static int first = 0;

     int minimum = 0xEFFF;
     int maximum = -1;

     for(int i = 0; i < depth; i++) {
    	  if (minimum >(int) samples[i]) minimum = (int) samples[i];
    	  if (maximum <(int) samples[i]) maximum = (int) samples[i];
     }
     if (first == 0) {
        first = 1;
         Uart2_printf("FREQ:");
         for (int i = 1500 ; i < 2100 ; i=i+20){

            Uart2_printf("%d;");

         }
          Uart2_printf("\r\n");
     }

     for (int i = 1500 ; i < 2100 ; i=i+20){
    	float coef =  Goertzel_coef(i,  SAMPLE_RATE);


        TONE_ONE_MAG = Goertzel_mag(samples,depth,coef,(maximum-minimum)/2);
        if (TONE_ONE_MAG > max) {
                max = TONE_ONE_MAG;
                max_id = i;
        }
        Uart2_printf("%d;", (int)sqrt(TONE_ONE_MAG));

     }
      Uart2_printf("\r\n");

     //Uart2_printf("MAX FEQ %d\r\n",max_id);
    return max_id;
}
extern void loop_tinyusb (void);
extern int main_tinyusb(void);


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{



  SystemClock_Config();
  HAL_Init();


  BSP_LED_Init(LED1);


  sq_init(&q_signal, (int16_t*)signal_buffer, 128);
  sq_init(&q_byte,   (int16_t*)byte_buffer, 32);
  sq_init(&q_fsk,    (int16_t*)fsk_buffer, 8);
  #if 0
  sq_init(&q_voice,voice_buffer,8192);
  #endif




  MX_GPIO_Init();
  MX_USART2_UART_Init();
  Timer3Init();


  ADC_Init();


#if 0
  coef1 = Goertzel_coef(FREQ_ONE,SAMPLE_RATE);
  coef2 = Goertzel_coef(FREQ_TWO,SAMPLE_RATE);
#else
  coef1 =  Goertzel_coef_q15(FREQ_ONE,SAMPLE_RATE);
  coef2 =  Goertzel_coef_q15(FREQ_TWO,SAMPLE_RATE);
#endif
  Uart2_printf("\r\nHGA22 RECEIVER RNDIS V1.0\r\n>\r\n");
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
  AGC2_low();
  AGC3_low();
  AGC4_low();

	__HAL_RCC_USB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_SYSCFG_CLK_ENABLE();
	__HAL_RCC_PWR_CLK_ENABLE();

  main_tinyusb();
 // tcp_server_init();
  udp_server_init();




  while(1){
#if 1
	  ShellTask();
#else
#warning ShellTask
#endif
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
				   search_freq((uint16_t *)buffer,buffer_size);
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
 //timxPeriod = __LL_TIM_CALC_ARR    (TimOutClock,     timxPrescaler,   (133600));
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

	SystemClockHSE_Config();
	return;

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
    Error_Handler();
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
    Error_Handler();
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
    HAL_Delay(100);
    //printf("Error");
  }

}



//void ADC_Init(void) {
//
//	uint32_t tmp_index;
///* Initialize ADC group regular data buffer values */
//  for (tmp_index = 0; tmp_index < ADC_CONVERTED_DATA_BUFFER_SIZE; tmp_index++)
//  {
//    uhADCxConvertedData[tmp_index] = VAR_CONVERTED_DATA_INIT_VALUE;
//  }
//
//  /* USER CODE END 1 */
//
//  /* MCU Configuration--------------------------------------------------------*/
//
//  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
//  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
//  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
//
//  /* SysTick_IRQn interrupt configuration */
//  NVIC_SetPriority(SysTick_IRQn, 3);
//
//
//  /* Initialize all configured peripherals */
//   //MX_GPIO_Init();
//   MX_DMA_Init();
//   MX_ADC1_Init();
//   MX_TIM1_Init();
//   /* USER CODE BEGIN 2 */
//
//   /* Activate ADC */
//   /* Perform ADC activation procedure to make it ready to convert. */
//   ADC_Activate();
//
//   /* Start ADC group regular conversion */
//   /* Note: ADC conversion will effectively start at timer trigger event */
//   /* Note: Hardware constraint (refer to description of the functions         */
//   /*       below):                                                            */
//   /*       On this STM32 series, setting of this feature is conditioned to    */
//   /*       ADC state:                                                         */
//   /*       ADC must be enabled without conversion on going on group regular,  */
//   /*       without ADC disable command on going.                              */
//   /* Note: In this example, all these checks are not necessary but are        */
//   /*       implemented anyway to show the best practice usages                */
//   /*       corresponding to reference manual procedure.                       */
//   /*       Software can be optimized by removing some of these checks, if     */
//   /*       they are not relevant considering previous settings and actions    */
//   /*       in user application.                                               */
//   if ((LL_ADC_IsEnabled(ADC1) == 1)               &&
//       (LL_ADC_IsDisableOngoing(ADC1) == 0)        &&
//       (LL_ADC_REG_IsConversionOngoing(ADC1) == 0)   )
//   {
//     LL_ADC_REG_StartConversion(ADC1);
//   }
//   else
//   {
//     /* Error: ADC conversion start could not be performed */
//     Error_Handler();
//   }
//
//   /* Start time base */
//   LL_TIM_EnableCounter(TIM1);
//
//
//}
//
//
///**
//  * @brief ADC1 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_ADC1_Init(void) {
//
//  /* USER CODE BEGIN ADC1_Init 0 */
//
//  /* USER CODE END ADC1_Init 0 */
//
//  LL_ADC_InitTypeDef ADC_InitStruct = {0};
//  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
//
//  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//  LL_RCC_SetADCClockSource(LL_RCC_ADC_CLKSOURCE_SYSCLK);
//
//  /* Peripheral clock enable */
//  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC);
//
//  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
//  /**ADC1 GPIO Configuration
//  PA4   ------> ADC1_IN4
//  */
//  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//
//  /* ADC1 DMA Init */
//
//  /* ADC1 Init */
//  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_1, LL_DMAMUX_REQ_ADC1);
//
//  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
//
//  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_LOW);
//
//  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_CIRCULAR);
//
//  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);
//
//  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);
//
//  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_HALFWORD);
//
//  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_HALFWORD);
//
//  /* ADC1 interrupt Init */
//  NVIC_SetPriority(ADC1_IRQn, 0);
//  NVIC_EnableIRQ(ADC1_IRQn);
//
//  /* USER CODE BEGIN ADC1_Init 1 */
//  /* Set DMA transfer addresses of source and destination */
//  LL_DMA_ConfigAddresses(DMA1,
//                         LL_DMA_CHANNEL_1,
//                         LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA),
//                         (uint32_t)&uhADCxConvertedData,
//                         LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
//
//  /* Set DMA transfer size */
//  LL_DMA_SetDataLength(DMA1,
//                       LL_DMA_CHANNEL_1,
//                       ADC_CONVERTED_DATA_BUFFER_SIZE);
//
//  /* Enable DMA transfer interruption: transfer complete */
//  LL_DMA_EnableIT_TC(DMA1,
//                     LL_DMA_CHANNEL_1);
//
//  /* Enable DMA transfer interruption: half transfer */
//  LL_DMA_EnableIT_HT(DMA1,
//                     LL_DMA_CHANNEL_1);
//
//  /* Enable DMA transfer interruption: transfer error */
//  LL_DMA_EnableIT_TE(DMA1,
//                     LL_DMA_CHANNEL_1);
//
//  /* Enable the DMA transfer */
//  LL_DMA_EnableChannel(DMA1,
//                       LL_DMA_CHANNEL_1);
//  /* USER CODE END ADC1_Init 1 */
//
//  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
//  */
//
//   #define ADC_CHANNEL_CONF_RDY_TIMEOUT_MS ( 1U)
//   #if (USE_TIMEOUT == 1)
//   uint32_t Timeout ; /* Variable used for Timeout management */
//   #endif /* USE_TIMEOUT */
//
//  ADC_InitStruct.Clock = LL_ADC_CLOCK_SYNC_PCLK_DIV4;
//
//  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
//  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
//  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
//  LL_ADC_Init(ADC1, &ADC_InitStruct);
//  LL_ADC_REG_SetSequencerConfigurable(ADC1, LL_ADC_REG_SEQ_FIXED);
//  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_EXT_TIM1_TRGO2;
//  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
//  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
//  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
//  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_UNLIMITED;
//  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_OVERWRITTEN;
//  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
//  LL_ADC_REG_SetSequencerScanDirection(ADC1, LL_ADC_REG_SEQ_SCAN_DIR_FORWARD);
//#warning !!!
//#if 1
//  LL_ADC_SetOverSamplingScope(ADC1, LL_ADC_OVS_DISABLE);
//#else
//  LL_ADC_SetOverSamplingScope(ADC1, LL_ADC_OVS_GRP_REGULAR_CONTINUED);
//   LL_ADC_ConfigOverSamplingRatioShift(ADC1, LL_ADC_OVS_RATIO_32, LL_ADC_OVS_SHIFT_RIGHT_5);
//   LL_ADC_SetOverSamplingDiscont(ADC1, LL_ADC_OVS_REG_CONT);
//#endif
//
//
//  LL_ADC_SetTriggerFrequencyMode(ADC1, LL_ADC_CLOCK_FREQ_MODE_HIGH);
//  LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_0);
//
//   /* Poll for ADC channel configuration ready */
//   #if (USE_TIMEOUT == 1)
//   Timeout = ADC_CHANNEL_CONF_RDY_TIMEOUT_MS;
//   #endif /* USE_TIMEOUT */
//   while (LL_ADC_IsActiveFlag_CCRDY(ADC1) == 0)
//     {
//   #if (USE_TIMEOUT == 1)
//   /* Check Systick counter flag to decrement the time-out value */
//   if (LL_SYSTICK_IsActiveCounterFlag())
//     {
//   if(Timeout-- == 0)
//         {
//   Error_Handler();
//         }
//     }
//   #endif /* USE_TIMEOUT */
//     }
//   /* Clear flag ADC channel configuration ready */
//   LL_ADC_ClearFlag_CCRDY(ADC1);
//  LL_ADC_REG_SetTriggerEdge(ADC1, LL_ADC_REG_TRIG_EXT_RISING);
//#warning !!!!
//#if 1
//  LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_COMMON_1, LL_ADC_SAMPLINGTIME_79CYCLES_5);
//#else
//  LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_COMMON_1, LL_ADC_SAMPLINGTIME_7CYCLES_5);
//#endif
//
//  LL_ADC_DisableIT_EOC(ADC1);
//  LL_ADC_DisableIT_EOS(ADC1);
//
//   /* Enable ADC internal voltage regulator */
//   LL_ADC_EnableInternalRegulator(ADC1);
//   /* Delay for ADC internal voltage regulator stabilization. */
//   /* Compute number of CPU cycles to wait for, from delay in us. */
//   /* Note: Variable divided by 2 to compensate partially */
//   /* CPU processing cycles (depends on compilation optimization). */
//   /* Note: If system core clock frequency is below 200kHz, wait time */
//   /* is only a few CPU processing cycles. */
//   uint32_t wait_loop_index;
//   wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
//   while(wait_loop_index != 0)
//     {
//   wait_loop_index--;
//     }
//  /* USER CODE BEGIN ADC1_Init 2 */
//
//  /* Configuration of ADC interruptions */
//  /* Enable interruption ADC group regular overrun */
//  LL_ADC_EnableIT_OVR(ADC1);
//
//  /* USER CODE END ADC1_Init 2 */
//
//}
//
///**
//  * @brief TIM1 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_TIM1_Init(void) {
//
//  /* USER CODE BEGIN TIM1_Init 0 */
//  uint32_t tim_prescaler;
//  uint32_t tim_period;
//  /* USER CODE END TIM1_Init 0 */
//
//  LL_TIM_InitTypeDef TIM_InitStruct = {0};
//
//  /* Peripheral clock enable */
//  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);
//
//  /* USER CODE BEGIN TIM1_Init 1 */
//
//  /* Set timer prescaler value (timer frequency) */
//  /* Note: Value TIMER_FREQUENCY_RANGE_MAX_HZ with factor 2 to have a minimum
//           timer resolution */
//  tim_prescaler = __LL_TIM_CALC_PSC(SystemCoreClock, TIMER_FREQUENCY_RANGE_MAX_HZ * 2);
//
//  /* Set timer period value (time base frequency) */
//  tim_period = __LL_TIM_CALC_ARR(SystemCoreClock, tim_prescaler, TIMER_FREQUENCY_HZ);
//
//  /* USER CODE END TIM1_Init 1 */
//  TIM_InitStruct.Prescaler = tim_prescaler;
//  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
//  TIM_InitStruct.Autoreload = tim_period;
//  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
//  TIM_InitStruct.RepetitionCounter = 0;
//  LL_TIM_Init(TIM1, &TIM_InitStruct);
//  LL_TIM_DisableARRPreload(TIM1);
//  LL_TIM_SetClockSource(TIM1, LL_TIM_CLOCKSOURCE_INTERNAL);
//  LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_UPDATE);
//  LL_TIM_SetTriggerOutput2(TIM1, LL_TIM_TRGO2_UPDATE);
//  LL_TIM_DisableMasterSlaveMode(TIM1);
//  /* USER CODE BEGIN TIM1_Init 2 */
//
//  /* USER CODE END TIM1_Init 2 */
//
//}
//
///**
//  * Enable DMA controller clock
//  */
//static void MX_DMA_Init(void) {
//
//  /* Init with LL driver */
//  /* DMA controller clock enable */
//  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
//
//  /* DMA interrupt init */
//  /* DMA1_Channel1_IRQn interrupt configuration */
//  NVIC_SetPriority(DMA1_Channel1_IRQn, 0);
//  NVIC_EnableIRQ(DMA1_Channel1_IRQn);
//
//}

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


/* USER CODE BEGIN 4 */

/**
  * @brief  Perform ADC activation procedure to make it ready to convert
  *         (ADC instance: ADC1).
  * @param  None
  * @retval None
  */
//void ADC_Activate(void) {
//  __IO uint32_t wait_loop_index = 0U;
//  __IO uint32_t backup_setting_adc_dma_transfer = 0U;
//  #if (USE_TIMEOUT == 1)
//  uint32_t Timeout = 0U; /* Variable used for timeout management */
//  #endif /* USE_TIMEOUT */
//
//  /*## Operation on ADC hierarchical scope: ADC instance #####################*/
//
//  /* Note: Hardware constraint (refer to description of the functions         */
//  /*       below):                                                            */
//  /*       On this STM32 series, setting of these features is conditioned to  */
//  /*       ADC state:                                                         */
//  /*       ADC must be disabled.                                              */
//  /* Note: In this example, all these checks are not necessary but are        */
//  /*       implemented anyway to show the best practice usages                */
//  /*       corresponding to reference manual procedure.                       */
//  /*       Software can be optimized by removing some of these checks, if     */
//  /*       they are not relevant considering previous settings and actions    */
//  /*       in user application.                                               */
//  if (LL_ADC_IsEnabled(ADC1) == 0)
//  {
//    /* Enable ADC internal voltage regulator */
//    LL_ADC_EnableInternalRegulator(ADC1);
//
//    /* Delay for ADC internal voltage regulator stabilization.                */
//    /* Compute number of CPU cycles to wait for, from delay in us.            */
//    /* Note: Variable divided by 2 to compensate partially                    */
//    /*       CPU processing cycles (depends on compilation optimization).     */
//    /* Note: If system core clock frequency is below 200kHz, wait time        */
//    /*       is only a few CPU processing cycles.                             */
//    wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
//    while(wait_loop_index != 0)
//    {
//      wait_loop_index--;
//    }
//
//    /* Disable ADC DMA transfer request during calibration */
//    /* Note: Specificity of this STM32 series: Calibration factor is           */
//    /*       available in data register and also transferred by DMA.           */
//    /*       To not insert ADC calibration factor among ADC conversion data   */
//    /*       in DMA destination address, DMA transfer must be disabled during */
//    /*       calibration.                                                     */
//    backup_setting_adc_dma_transfer = LL_ADC_REG_GetDMATransfer(ADC1);
//    LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_NONE);
//
//    /* Run ADC self calibration */
//    LL_ADC_StartCalibration(ADC1);
//
//    /* Poll for ADC effectively calibrated */
//    #if (USE_TIMEOUT == 1)
//    Timeout = ADC_CALIBRATION_TIMEOUT_MS;
//    #endif /* USE_TIMEOUT */
//
//    while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0)
//    {
//    #if (USE_TIMEOUT == 1)
//      /* Check Systick counter flag to decrement the time-out value */
//      if (LL_SYSTICK_IsActiveCounterFlag())
//      {
//        if(Timeout-- == 0)
//        {
//          /* Error: Time-out */
//          Error_Handler();
//        }
//      }
//    #endif /* USE_TIMEOUT */
//    }
//
//    /* Restore ADC DMA transfer request after calibration */
//    LL_ADC_REG_SetDMATransfer(ADC1, backup_setting_adc_dma_transfer);
//
//    /* Delay between ADC end of calibration and ADC enable.                   */
//    /* Note: Variable divided by 2 to compensate partially                    */
//    /*       CPU processing cycles (depends on compilation optimization).     */
//    wait_loop_index = (ADC_DELAY_CALIB_ENABLE_CPU_CYCLES >> 1);
//    while(wait_loop_index != 0)
//    {
//      wait_loop_index--;
//    }
//
//    /* Enable ADC */
//    LL_ADC_Enable(ADC1);
//
//    /* Poll for ADC ready to convert */
//    #if (USE_TIMEOUT == 1)
//    Timeout = ADC_ENABLE_TIMEOUT_MS;
//    #endif /* USE_TIMEOUT */
//
//    while (LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0)
//    {
//    #if (USE_TIMEOUT == 1)
//      /* Check Systick counter flag to decrement the time-out value */
//      if (LL_SYSTICK_IsActiveCounterFlag())
//      {
//        if(Timeout-- == 0)
//        {
//          /* Error: Time-out */
//          Error_Handler();
//        }
//      }
//    #endif /* USE_TIMEOUT */
//    }
//
//    /* Note: ADC flag ADRDY is not cleared here to be able to check ADC       */
//    /*       status afterwards.                                               */
//    /*       This flag should be cleared at ADC Deactivation, before a new    */
//    /*       ADC activation, using function "LL_ADC_ClearFlag_ADRDY()".       */
//  }
//
//  /*## Operation on ADC hierarchical scope: ADC group regular ################*/
//  /* Note: No operation on ADC group regular performed here.                  */
//  /*       ADC group regular conversions to be performed after this function  */
//  /*       using function:                                                    */
//  /*       "LL_ADC_REG_StartConversion();"                                    */
//
//  /*## Operation on ADC hierarchical scope: ADC group injected ###############*/
//  /* Note: Feature not available on this STM32 series */
//
//}

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

void AdcDmaTransferComplete_Callback() {

#if 1
	sq_push(&q_signal, (int16_t *)uhADCxConvertedData, ADC_CONVERTED_DATA_BUFFER_SIZE);
        #if 0
	if (ubDmaTransferStatus == 0){
		sq_push(&q_voice,  uhADCxConvertedData, ADC_CONVERTED_DATA_BUFFER_SIZE);
	}
        #endif
#else

	 LED_On();
	static uint16_t size = 33838;//sizeof(hga_test) /2;
	static int idx = 0;

	for(int i = 0; i < ADC_CONVERTED_DATA_BUFFER_SIZE; i++ ){
		uhADCxConvertedData[i] = hga22[i + idx ];
	}

	sq_push(&q_signal, uhADCxConvertedData, ADC_CONVERTED_DATA_BUFFER_SIZE);

	idx += ADC_CONVERTED_DATA_BUFFER_SIZE;

	if (idx >= size - ADC_CONVERTED_DATA_BUFFER_SIZE) {
		idx = 0;
	}
#endif
	ubDmaTransferStatus = 1;
	ubDmaTransferCnt++;
}


/**
  * @brief  DMA half transfer callback
  * @note   This function is executed when the half transfer interrupt
  *         is generated
  * @retval None
  */
void AdcDmaTransferHalf_Callback() {
//  uint32_t tmp_index;
//
//  /* Data integrity check: Ensure that 1st half of buffer has not yet been
//     overwritten by DMA transfer: end of 2nd half of buffer should equal to
//     init value */
//  if (uhADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE - 1] != VAR_CONVERTED_DATA_INIT_VALUE)
//  {
//
//    LED_Toggle();
//  }
//
//  /* Computation of ADC conversions raw data to physical values
//     using LL ADC driver helper macro. */
//  /* Management of the 1st half of buffer */
//  for (tmp_index = 0; tmp_index < (ADC_CONVERTED_DATA_BUFFER_SIZE/2); tmp_index++)
//  {
//    uhADCxConvertedData_Voltage_mVolt[tmp_index] = __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, uhADCxConvertedData[tmp_index], LL_ADC_RESOLUTION_12B);
//  }
//
//  /* Set half-buffer last data to init value for further data integrity check */
//  uhADCxConvertedData[tmp_index - 1] = VAR_CONVERTED_DATA_INIT_VALUE;
//
//  /* Update status variable of DMA transfer */
//  ubDmaTransferStatus = 0;
}



/**
  * @brief  DMA transfer error callback
  * @note   This function is executed when the transfer error interrupt
  *         is generated during DMA transfer
  * @retval None
  */
void AdcDmaTransferError_Callback() {
  if(ubDmaTransferStatus == 1)
  {
    /* Update status variable of DMA transfer */
    ubDmaTransferStatus = 0;
  }

  /* Error detected during DMA transfer */
  Error_Handler();
}

/**
  * @brief  ADC group regular overrun interruption callback
  * @note   This function is executed when ADC group regular
  *         overrun error occurs.
  * @retval None
  */
void AdcGrpRegularOverrunError_Callback(void) {
  /* Note: Disable ADC interruption that caused this error before entering in
           infinite loop below. */

  /* In case of error due to overrun: Disable ADC group regular overrun interruption */
  LL_ADC_DisableIT_OVR(ADC1);

  /* Error reporting */
  Error_Handler();
}



#if 0
float Goertzel_coef(float freq, int samp_rate) {
  float coef = 2.0 * cos((2.0 * PI * freq) / samp_rate);
  return coef;
}

float Goertzel_mag(uint16_t samples[], int depth, float coef, int adc_midpoint) {
	float Q1 = 0;
	float Q2 = 0;
	for (int n = 0; n < depth; n++) {
		float Q0 = coef * Q1 - Q2 + (float)((float)samples[n] - adc_midpoint);
		Q2 = Q1;
		Q1 = Q0;
	}
	return (Q1 * Q1 + Q2 * Q2 - coef * Q1 * Q2);
	//return sqrt(Q1 * Q1 + Q2 * Q2 - coef * Q1 * Q2);

}
#else

int16_t Goertzel_coef_q15(float freq, int samp_rate)
{
    float w = (2.0f * (float)M_PI * freq) / (float)samp_rate;
    float c = 2.0f * cosf(w);

    return (int16_t)(c * 32768.0f);   // Q15
}

int32_t Goertzel_mag_fast_q15(const uint16_t *samples,
                              int depth,
                              int16_t coef,
                              int32_t adc_mid)
{
    int32_t q0 = 0;
    int32_t q1 = 0;
    int32_t q2 = 0;

    for (int n = 0; n < depth; n++) {

        int32_t x = (int32_t)samples[n] - adc_mid;

        // 32bit * 16bit → 32bit (gyors)
        q0 = x + ((coef * q1) >> 15) - q2;

        q2 = q1;
        q1 = q0;
    }

#if 1
    int32_t mag = abs(q1)+abs(q2) ;
#else
    // magnitude² (csak itt 64bit → OK)
    int64_t mag =
          (int64_t)q1 * q1
        + (int64_t)q2 * q2
        - ((int64_t)coef * q1 >> 15) * q2;
#endif
    if (mag < 0) mag = 0;
    if (mag > INT32_MAX) mag = INT32_MAX;

    return (int32_t)mag;
}


int32_t Goertzel_coef(float freq, int samp_rate)
{
    float w = (2.0f * (float)M_PI * freq) / (float)samp_rate;
    float c = 2.0f * cosf(w);

    return (int32_t)(c * 2147483648.0f); // Q31
}

#define GOERTZEL_MAG_SHIFT 15   // <<< EZT LEHET HANGOLNI

int32_t Goertzel_mag(const uint16_t *samples,
                            int depth,
                            int32_t coef,
                            int32_t adc_midpoint)
{
    int32_t Q0 = 0;
    int32_t Q1 = 0;
    int32_t Q2 = 0;

    for (int n = 0; n < depth; n++) {
        Q0 = (int32_t)(
                ((int64_t)coef * Q1 >> 31)
                - Q2
                + ((int32_t)samples[n] - adc_midpoint)
             );
        Q2 = Q1;
        Q1 = Q0;
    }

    // magnitude^2 (Q62)
    int64_t mag64 =
          (int64_t)Q1 * Q1
        + (int64_t)Q2 * Q2
        - ((int64_t)coef * Q1 >> 31) * Q2;

    // skálázás int32-re
    mag64 >>= GOERTZEL_MAG_SHIFT;

    // szaturáció (biztonság)
    if (mag64 > INT32_MAX) return INT32_MAX;
    if (mag64 < 0)         return 0;

    return (int32_t)mag64;
}
#endif





/* ================== BIT → BYTE ================== */

uint8_t bit_to_byte(uint8_t bit, uint8_t *out) {
  static uint8_t shift = 0;
  static uint8_t cnt = 0;

  shift = (shift >> 1) | (bit ? 0x80 : 0);
  cnt++;

  if (cnt == 8) {
    *out = shift;
    cnt = 0;
    shift = 0;
    return 1;
  }
  return 0;
}

/* ================== FRAME PARSE ================== */

uint32_t getBits(const uint8_t *data, uint32_t pos, uint32_t len) {
  uint32_t v = 0;
  for (uint32_t i = 0; i < len; i++) {
    uint8_t b = (data[(pos+i)/8] >> ((pos+i)%8)) & 1;
    v |= (b << i);
  }
  return v;
}
#ifndef FLT_MAX
#define FLT_MAX 3.402823466e+38F /* max value */
#endif
#ifndef FLT_MIN
#define FLT_MIN 1.175494351e-38F /* min positive value */
#endif
static float signal = FLT_MIN;
static float noise  = FLT_MAX;



void parse_frame(uint8_t *frame) {

  if (frame[0]!=0x68 || frame[1]!=0x0A || frame[2]!=0x0A || frame[3]!=0x68)
    return;

  uint8_t sum = 0;
  for (int i=4;i<14;i++) sum += frame[i];
  if (frame[14]!=sum || frame[15]!=0x16) return;

  uint8_t *d = &frame[7];

  //uint32_t ms  = getBits(d,  0,10);
  uint32_t sec = getBits(d, 10, 6);
  uint32_t min = getBits(d, 16, 6);
  uint32_t hr  = getBits(d, 24, 5);
  uint32_t day = getBits(d, 32, 5);
  uint32_t mon = getBits(d, 40, 4);
  uint32_t yr  = getBits(d, 48, 7);
  uint8_t dst  = getBits(d, 31, 1);

  Uart2_printf("TIME: 20");
  Uart2_printf("%d",yr);
  Uart2_printf("-");
  Uart2_printf("%s",mon<10?"0":""); Uart2_printf("%d",mon);
  Uart2_printf("-");
  Uart2_printf("%s",day<10?"0":""); Uart2_printf("%d",day);
  Uart2_printf(" ");
  Uart2_printf("%s",hr<10?"0":""); Uart2_printf("%d",hr);
  Uart2_printf(":");
  Uart2_printf("%s",min<10?"0":""); Uart2_printf("%d",min);
  Uart2_printf(":");
  Uart2_printf("%s",sec<10?"0":""); Uart2_printf("%d\r\n",sec);

  datetime_t dt;

  dt.yr = yr+2000;
  dt.mon = mon;
  dt.day = day;
  dt.hr  = hr;
  dt.min = min;
  dt.sec = sec;

  uint64_t now = datetime_to_unix_us(&dt, 0);

  /* RTC -> dt mezők feltöltése */
  NVIC_DisableIRQ(SysTick_IRQn);
  unix_us = now;
  NVIC_EnableIRQ(SysTick_IRQn);




//  printf(".");
//  printf("%d ",ms);
//  printf(" DST:");
//  printf("%s \r\n",dst?"igen":"nem");

//  float snr_db = 10.0f * log10f(signal / noise);

  sq_discard(&q_byte,16);



  if (send_array) {
	  send_array = 0;

  #if 0

	  printf("\r\nDATA [%d]:\r\n",sq_size(&q_voice));
	  while(1){
	 	if (sq_size(&q_voice) > 0){
	      uint16_t buffer[32];
	 	  sq_peek(&q_voice,buffer,32);
	 	  sq_discard(&q_voice,32);
		  for (int i = 0;i < 32; i++){
			  printf("%d;",buffer[i]);
			  printf("\r\n");
		  }
		  //printf("\r\n");
	 	} else {
	 		break;
	 	}
	  }

#endif
  }

}


#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

void loop(int16_t* buffer, int depth) {



  static uint8_t state = 0;
  static uint8_t byte = 0;
  static uint8_t parity = 0;
  static uint8_t start = 0;
  static uint8_t bit_cnt = 0;
  static uint8_t send = 0;
  uint8_t fsk_bit = 0;





  min = 4097;
  max = -1;
  signal = FLT_MIN;

  for(int i = 0; i < depth; i++) {
	  if (min >(int) buffer[i]) min = (int) buffer[i];
	  if (max <(int) buffer[i]) max = (int) buffer[i];
  }

 int32_t avg = (max - min) / 2;
#if 0
  float m1 = Goertzel_mag(buffer, depth, coef1,avg);
  float m2 = Goertzel_mag(buffer, depth, coef2, avg);
#else
  int32_t m1 = Goertzel_mag_fast_q15((uint16_t *)buffer, depth,(int16_t) coef1, avg);
  int32_t m2 = Goertzel_mag_fast_q15((uint16_t *)buffer, depth,(int16_t) coef2, avg);
#endif

  max_snr1 = MAX((int)m1,max_snr1);
  max_snr2 = MAX((int)m2,max_snr2);
  min_snr1 = MIN((int)m1,min_snr1);
  min_snr2 = MIN((int)m2,min_snr2);

  uint16_t bit = (m1 > m2) ? 1 : 0;


 // if (bit ==0) {
//	  Uart2_printf("%d   %d/%d \r\n",(int) (m2/m1), (int)m1,(int)m2);
//  }


  {
	  if (signal < MAX(m1,m2)){
		  signal =  MAX(m1,m2);
		  snr = signal;
	  }
	  if (noise >  MIN(m1,m2)) {
		  noise = MIN(m1,m2);
	  }

  }

  {
	  if (start != 1){
		  if (ubDmaTransferCnt % 1000 == 0){
			  // printf("SNR : %d , min %d, max %d\r\n", (int)signal, (int)min, (int) max );

			   signal = FLT_MIN;
			   noise  = 1000;
		  }
	  }

  }


  sq_push(&q_fsk,(int16_t *)&bit,1);

//  {
//  static int send_idx = 0;
//  if (++send_idx %5 == 0){
//	  printf("\r\nbit :");
//  }
//  Uart2_printf("[%d],",bit);
//  }


  if (sq_size(&q_fsk) >= 5){
	  uint16_t bits[5] = {0};
	  sq_peek(&q_fsk,(int16_t *)bits,5);
	  sq_discard(&q_fsk,1);
	  uint8_t sum = 0;
	  for (uint8_t i=0;i<OFFSET_BITS;i++) {
		  sum += bits[i];
	  }
	  fsk_bit = (sum >= 3) ? 1 : 0;


	  if (start == 0){



		  if (sum <2){
			  start = 1;
			  send  = 5;
			  bit_cnt = 0;
			  //printf("\r\n--start----\r\n");

			  signal = MAX(m1, m2);
			  noise  = MIN(m1, m2); // elkerülni a 0-t
              snr = signal;

		  }

	  }



  }



  if (start != 1){
	  state = 0;
	  bit_cnt = 0;
	  return;
  }

  if (++send < 5){
  	  return;
  }
  send = 0;

//  {
////  static int send_idx = 0;
////  if (++send_idx %32 == 0){
////	  printf("\r\n");
////  }
//  printf("[%d],",fsk_bit);
//  }

  switch (state) {

    case 0: // start
      if (fsk_bit == 0) {
        state = 1;
        byte = 0;
        send = 0;
      }
      break;

    case 1: // data
      if (bit_to_byte(fsk_bit, &byte)){
          state = 2;
        }
      break;

    case 2: // parity
      parity = __builtin_parity(byte);
      if (parity == fsk_bit){
          state = 3;
        }
      else {
        state = 0;
      }
      break;

    case 3: // stop

    {


		   //printf("0x%X, -%d-\r\n",byte,fsk_bit );
    }



      if (fsk_bit == 1) {

//    	  float snr_db;
//    	  {
//
//
//    		  float signal = MAX(m1, m2);
//    		  float noise  = MIN(m1, m2) + 1e-9; // elkerülni a 0-t
//
//    		  snr_db = 10.0f * log10f(signal / noise);
//    	  }

//    	  static int send_idx = 0;
//			  if (++send_idx %16 == 0){
//				  printf("\r\n");
//			  }
//		   printf("0x%X [%f]\r\n",byte, snr_db );
//    	  printf("0x%X,",byte);

          uint16_t data = byte;
    	  sq_push(&q_byte,(int16_t *)&data,1);



    	  while(1){
			   if (sq_size(&q_byte) >= 16){
//				   printf("\r\n");
				   int16_t buffer[16];
				   uint8_t buffer_byte[16];

				   sq_peek(&q_byte,buffer,16);
				   sq_discard(&q_byte,1);
				   for (int i = 0; i < 16; i++){
					   buffer_byte[i] = buffer[i];
				   }


				   parse_frame(buffer_byte);


			   } else{
				   break;
			   }
    	  }







      } else {
 		 start 		= 0;
 		 send  		= 0;
 		 bit_cnt 	= 0;

 		 state  	= 0;
 		 byte   	= 0;
 		 parity 	= 0;
      }
      state = 0;
      break;
  }


  if (++bit_cnt >= 11){
		 start = 0;
		 send  = 0;
		 bit_cnt = 0;

		 state      = 0;
		 byte   = 0;
		 parity = 0;
		 //printf("\r\n--end----\r\n");

	 }



}


void loop2(int16_t *buffer, int depth)
{
    /* ================= UART ================= */

    static uint8_t uart_state = 0;
    static uint8_t byte = 0;
    static uint8_t bit_cnt = 0;
    static uint8_t parity = 0;

    /* ================= BIT FILTER ================= */

    static uint8_t bit_hist = 0;

    /* ================= DEMOD ================= */

    static int32_t diff_lp = 0;
    static int32_t agc = 1000;
    static int32_t noise_floor = 100;

    /* ================= FRAME SYNC ================= */

    static uint32_t sync = 0;
    static uint8_t frame_sync = 0;

    static uint8_t frame[32];
    static uint8_t frame_pos = 0;

    /* ================= SIGNAL RANGE ================= */
    /// int32_t
    min = 0xFFFFF;
    max = -1;

    for(int i=0;i<depth;i++)
    {
        int16_t v = buffer[i];

        if(v < min) min = v;
        if(v > max) max = v;
    }

    int32_t avg = (min + max) >> 1;

    /* =================================================
       GOERTZEL
       ================================================= */

    int32_t m1 = Goertzel_mag_fast_q15((uint16_t*)buffer, depth, (int16_t)coef1, avg);
    int32_t m2 = Goertzel_mag_fast_q15((uint16_t*)buffer, depth, (int16_t)coef2, avg);


#if 1
      max_snr1 = MAX((int)m1,max_snr1);
      max_snr2 = MAX((int)m2,max_snr2);
      min_snr1 = MIN((int)m1,min_snr1);
      min_snr2 = MIN((int)m2,min_snr2);

      snr = MAX(m1, m2);

#endif

    int32_t diff = m1 - m2;

    /* ================= AGC ================= */

    int32_t mag = abs(diff);

    agc += (mag - agc) >> 3;

    if(agc < 1)
        agc = 1;

    diff = (diff << 8) / agc;

    /* ================= NOISE FLOOR ================= */

    noise_floor += (mag - noise_floor) >> 6;

    if(mag < (noise_floor << 1))
        return;

    /* ================= LOWPASS ================= */

    diff_lp += (diff - diff_lp) >> 2;

    uint8_t raw_bit = (diff_lp > 0);

    /* ================= MAJORITY FILTER ================= */

    bit_hist = (bit_hist << 1) | raw_bit;

    uint8_t ones = __builtin_popcount(bit_hist & 0x07);

    uint8_t fsk_bit = (ones >= 2);

    /* =================================================
       UART DECODER
       ================================================= */

    switch(uart_state)
    {
        /* ---------- START ---------- */

        case 0:

            if(fsk_bit == 0)
            {
                uart_state = 1;
                byte = 0;
                bit_cnt = 0;
                bit_hist = 0;
            }

        break;

        /* ---------- DATA ---------- */

        case 1:

            byte >>= 1;

            if(fsk_bit)
                byte |= 0x80;

            bit_cnt++;

            if(bit_cnt >= 8)
                uart_state = 2;

        break;

        /* ---------- PARITY ---------- */

        case 2:

            parity = __builtin_parity(byte);

            if(parity == fsk_bit)
                uart_state = 3;
            else
                uart_state = 0;

        break;

        /* ---------- STOP ---------- */

        case 3:

            if(fsk_bit == 1)
            {
                /* ================= SYNC SEARCH ================= */

                if(!frame_sync)
                {
                    sync = (sync << 8) | byte;

                    if(sync == 0x680A0A68)
                    {
                        frame_sync = 1;

                        frame[0] = 0x68;
                        frame[1] = 0x0A;
                        frame[2] = 0x0A;
                        frame[3] = 0x68;

                        frame_pos = 4;
                    }
                }
                else
                {
                    if(frame_pos < sizeof(frame))
                    {
                        frame[frame_pos++] = byte;
                    }

                    if(frame_pos >= 16)
                    {
                        parse_frame(frame);

                        frame_sync = 0;
                        frame_pos = 0;
                    }
                }
            }

            uart_state = 0;

        break;
    }
}
