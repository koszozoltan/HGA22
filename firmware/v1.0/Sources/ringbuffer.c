
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "stdarg.h"



#include "main.h"


#ifndef RX_BUFFER_SIZE
   #define RX_BUFFER_SIZE    16
#endif

#ifndef TX_BUFFER_SIZE
   #define TX_BUFFER_SIZE    16
#endif

#ifndef TX_TIMEOUT
   #define TX_TIMEOUT    1000
#endif


typedef struct {
  volatile uint8_t rx_buffer[RX_BUFFER_SIZE];
  volatile uint8_t rx_char;

  volatile uint8_t tx_buffer[TX_BUFFER_SIZE];
  volatile uint8_t tx_char;
  volatile uint8_t tx_run;
  volatile uint8_t rx_buffer_overflow;

  volatile uint16_t rx_wr_index;
  volatile uint16_t rx_rd_index;
  volatile uint16_t rx_counter;

  volatile uint16_t tx_wr_index;
  volatile uint16_t tx_rd_index;
  volatile uint16_t tx_counter;

  volatile uint32_t timeout;

  UART_HandleTypeDef *huart;
  IRQn_Type          IRQn;

} RINGBUFFER_ST;


RINGBUFFER_ST uart2_buffer;
extern UART_HandleTypeDef huart2;



void RingBuffer_Flush(RINGBUFFER_ST * ringb){

  ringb->tx_run      = 0;
  ringb->rx_wr_index = 0;
  ringb->rx_rd_index = 0;
  ringb->rx_counter  = 0;
  ringb->tx_wr_index = 0;
  ringb->tx_rd_index = 0;
  ringb->tx_counter  = 0;
}


void RingBuffer_RxISR(RINGBUFFER_ST * ringb){

      char data;
      data = ringb->rx_char;
      ringb->rx_buffer[ringb->rx_wr_index]=data;
      if (++ringb->rx_wr_index == RX_BUFFER_SIZE) ringb->rx_wr_index=0;
      if (++ringb->rx_counter  == RX_BUFFER_SIZE) {
         ringb->rx_counter=0;
         ringb->rx_buffer_overflow=1;
      }

      HAL_UART_Receive_IT(ringb->huart, (uint8_t *)&ringb->rx_char, 1);

}

void RingBuffer_RxPut(RINGBUFFER_ST * ringb, char ch){

      char data;
      HAL_NVIC_DisableIRQ(ringb->IRQn);
      data = ch;
      ringb->rx_buffer[ringb->rx_wr_index]=data;
      if (++ringb->rx_wr_index == RX_BUFFER_SIZE) ringb->rx_wr_index=0;
      if (++ringb->rx_counter  == RX_BUFFER_SIZE) {
         ringb->rx_counter=0;
         ringb->rx_buffer_overflow=1;
      }
      HAL_NVIC_EnableIRQ(ringb->IRQn);
}

void RingBuffer_TxISR(RINGBUFFER_ST * ringb){

   if (ringb->tx_counter) {
      --ringb->tx_counter;
      HAL_UART_Transmit_IT(ringb->huart, (uint8_t*)&ringb->tx_buffer[ringb->tx_rd_index], 1);
      if (++ringb->tx_rd_index == TX_BUFFER_SIZE) ringb->tx_rd_index=0;
   } else {
      ringb->tx_run = 0;
   }
}
void RingBuffer_Init(RINGBUFFER_ST * ringb,UART_HandleTypeDef *huart,IRQn_Type IRQn){

  RingBuffer_Flush(ringb);
  ringb->huart       = huart;
  ringb->IRQn        = IRQn;

  HAL_UART_Receive_IT(ringb->huart, (uint8_t *)&ringb->rx_char, 1);
}
void RingBuffer_Tx(RINGBUFFER_ST * ringb, char  tx_char){

   ringb->timeout = 0;
   while (  ringb->tx_counter == TX_BUFFER_SIZE){
      if (++ ringb->timeout >= TX_TIMEOUT) {
            HAL_NVIC_DisableIRQ(ringb->IRQn);
            RingBuffer_Flush(ringb);
            HAL_NVIC_EnableIRQ(ringb->IRQn);
            return;
      }
      //osDelay(1);
   }

   HAL_NVIC_DisableIRQ(ringb->IRQn);
   if (ringb->tx_counter || (ringb->tx_run == 1)) {
      ringb->tx_buffer[ringb->tx_wr_index]=tx_char;
      if (++ringb->tx_wr_index == TX_BUFFER_SIZE) ringb->tx_wr_index=0;
      ++ringb->tx_counter;
   } else {
      ringb->tx_char = (uint8_t) tx_char;
      ringb->tx_run = 1;
      HAL_UART_Transmit_IT(ringb->huart, (uint8_t*)&ringb->tx_char, 1);
   }
   HAL_NVIC_EnableIRQ(ringb->IRQn);

}
int16_t RingBuffer_Rx(RINGBUFFER_ST * ringb){

    char data;
    if (ringb->rx_counter == 0) return -1;
    data = ringb->rx_buffer[ringb->rx_rd_index];

    if (++ringb->rx_rd_index == RX_BUFFER_SIZE) ringb->rx_rd_index=0;

    NVIC_DisableIRQ(ringb->IRQn);
    --ringb->rx_counter;
    NVIC_EnableIRQ(ringb->IRQn);

    return data;
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
   if (huart->Instance == USART2) {
      RingBuffer_TxISR(&uart2_buffer);
   }
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
   if (huart->Instance == USART2) {
      RingBuffer_RxISR(&uart2_buffer);
   }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){

   if (huart->Instance == USART2) {
      RingBuffer_Flush(&uart2_buffer);
   }
}
void Uart2BufferInit(void){
  RingBuffer_Init(&uart2_buffer,&huart2,USART2_IRQn);
}


int16_t Uart2Getchar(void){
  return RingBuffer_Rx(&uart2_buffer);
}
void Uart2Putchar(char c){
  RingBuffer_Tx(&uart2_buffer,c);
}


void Uart2_printf (char * format, ...) {
#if 1
  char buffer[64];
  va_list args;
  va_start (args, format);
  vsnprintf (buffer, sizeof(buffer)-1, format, args);
  va_end (args);

  char *s = buffer;

  while(*s){
		Uart2Putchar(*s++);
	}
#else
#warning Uart2_printf
#endif
}

void Uart2_rx_printf (char * format, ...) {
#if 1
  char buffer[64];
  va_list args;
  va_start (args, format);
  vsnprintf (buffer, sizeof(buffer)-1, format, args);
  va_end (args);

  char *s = buffer;

  while(*s){
		RingBuffer_RxPut(&uart2_buffer,*s++);
  }
#else
#warning Uart2_rx_printf
#endif
}


