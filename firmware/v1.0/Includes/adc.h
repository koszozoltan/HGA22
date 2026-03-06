/*
 * adc.h
 *
 *  Created on: Mar 5, 2026
 *      Author: development
 */

#ifndef APPLICATION_USER_ADC_H_
#define APPLICATION_USER_ADC_H_


/* Definitions of data related to this example */
  /* Definition of ADCx conversions data table size */
//  #define ADC_CONVERTED_DATA_BUFFER_SIZE   (  64UL)
#define ADC_CONVERTED_DATA_BUFFER_SIZE   (  8UL)
#define TIMER_FREQUENCY_HZ               (8000UL)

extern __IO uint16_t uhADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE];
extern __IO uint8_t ubDmaTransferStatus;
extern __IO uint8_t ubDmaTransferCnt;

void	ADC_Init			(void);
void	MX_ADC1_Init		(void);
void	MX_TIM1_Init		(void);
void	MX_DMA_Init			(void);
void 	ADC_Activate		(void);

#endif /* APPLICATION_USER_ADC_H_ */
