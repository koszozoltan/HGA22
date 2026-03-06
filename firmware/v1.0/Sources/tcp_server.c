#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "stdarg.h"


#include "tcp_server.h"
#include "string.h"


#ifndef RX_BUFFER_SIZE
   #define RX_BUFFER_SIZE    16
#endif

#ifndef TX_BUFFER_SIZE
   #define TX_BUFFER_SIZE   256
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


} RINGBUFFER_ST;


RINGBUFFER_ST tcp_buffer;

static struct tcp_server server;


static err_t tcp_server_accept(void *arg, struct tcp_pcb *new_pcb, err_t err);
static err_t tcp_server_receive(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
static void  tcp_server_error(void *arg, err_t err);
err_t        tcp_server_write(const char *data);

static int   tcp_busy = 0;


static void RingBuffer_Flush(RINGBUFFER_ST * ringb);
//static void RingBuffer_RxISR(RINGBUFFER_ST * ringb);
static void RingBuffer_RxPut(RINGBUFFER_ST * ringb, char ch);
static void RingBuffer_TxISR(RINGBUFFER_ST * ringb);
static void RingBuffer_Init(RINGBUFFER_ST * ringb);
static void RingBuffer_Tx(RINGBUFFER_ST * ringb, char  tx_char);
static int16_t RingBuffer_Rx(RINGBUFFER_ST * ringb);
void 		TcpBufferInit(void);
int16_t 	TcpGetchar(void);
void 		TcpPutchar(char c);
void 		Tcp_printf (char * format, ...) ;
void 		tcp_rx_printf (char * format, ...);

extern void Uart2Putchar(char c);


extern void Uart2_printf (char * format, ...);

void tcp_server_init(void)
{
	err_t status = ERR_OK;

	server.tcp_server_pcb = tcp_new();
	server.server_port = 5000;
	server.accept = tcp_server_accept;
	server.receive = tcp_server_receive;
	server.error  = tcp_server_error;
	if(server.tcp_server_pcb != NULL) {
		status = tcp_bind(server.tcp_server_pcb, IP_ADDR_ANY, server.server_port);
		if(status == ERR_OK) {
			server.tcp_server_pcb = tcp_listen(server.tcp_server_pcb);
			tcp_accept(server.tcp_server_pcb, server.accept);
			Uart2_printf("tcp server init ok \r\n");
		}
	}
	TcpBufferInit();
}

static err_t tcp_server_sent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
	//Uart2_printf("tcp_server_sent %d\r\n",len);
    // ACK megérkezett
	RingBuffer_TxISR(&tcp_buffer);

    return ERR_OK;
}

static err_t tcp_server_accept(void *arg, struct tcp_pcb *new_pcb, err_t err)
{
	//Uart2_printf("server accept \r\n");

	server.client_pcb = new_pcb;
	tcp_arg(new_pcb, &server);
	tcp_err(new_pcb, server.error);
	tcp_sent(new_pcb, tcp_server_sent);
	tcp_recv(new_pcb, server.receive);

	return ERR_OK;

}


static err_t tcp_server_receive(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
	if(p != NULL) {
		tcp_recved(tpcb, p->tot_len);

		 char *s =p->payload;

		  for(int i = 0; i< p->len; i++ ){
			  //Uart2Putchar(*s);
			  RingBuffer_RxPut(&tcp_buffer,*s++);
		  }

		pbuf_free(p);   // <-- KÖTELEZŐ

	}
	else if(err == ERR_OK) {
		return tcp_close(tpcb);
	}

	return ERR_OK;

}

static void tcp_server_error(void *arg, err_t err)
{
	 Uart2_printf("tcp_server_error %d\r\n",err);

}

//err_t tcp_server_write(const char *data)
//{
//    if(server.client_pcb == NULL)
//        return ERR_CONN;   // nincs csatlakozott kliens
//
//    Uart2_printf("[%s]\r\n",data);
//    err_t err = tcp_write(server.client_pcb, data, strlen(data), TCP_WRITE_FLAG_COPY);
//    if(err == ERR_OK)
//    {
//        tcp_output(server.client_pcb);  // push
//    }
//
//    return err;
//}

err_t tcp_server_write_len(const char *data, uint16_t len)
{
    if(server.client_pcb == NULL)
        return ERR_CONN;

/*
    Uart2_printf("tcp_server_write_len %d :\r\n", len);
    for (int i = 0; i < len; i++){
    	 Uart2_printf("[%d]-> %c \r\n",i, data[i]);
    }*/

    err_t err = tcp_write(server.client_pcb,
                          data,
                          len,
                          TCP_WRITE_FLAG_COPY);

    //Uart2_printf("\r\nerror %d\r\n",err);

    if(err == ERR_OK){
        tcp_output(server.client_pcb);
    }


    return err;
}







static void RingBuffer_Flush(RINGBUFFER_ST * ringb){

  ringb->tx_run      = 0;
  ringb->rx_wr_index = 0;
  ringb->rx_rd_index = 0;
  ringb->rx_counter  = 0;
  ringb->tx_wr_index = 0;
  ringb->tx_rd_index = 0;
  ringb->tx_counter  = 0;
  tcp_busy           = 0;
}


//static void RingBuffer_RxISR(RINGBUFFER_ST * ringb){
//
//      char data;
//      data = ringb->rx_char;
//      ringb->rx_buffer[ringb->rx_wr_index]=data;
//      if (++ringb->rx_wr_index == RX_BUFFER_SIZE) ringb->rx_wr_index=0;
//      if (++ringb->rx_counter  == RX_BUFFER_SIZE) {
//         ringb->rx_counter=0;
//         ringb->rx_buffer_overflow=1;
//      }
//
//      //HAL_UART_Receive_IT(ringb->huart, (uint8_t *)&ringb->rx_char, 1);
//
//}

static void RingBuffer_RxPut(RINGBUFFER_ST * ringb, char ch){

      char data;
//      HAL_NVIC_DisableIRQ(ringb->IRQn);
      data = ch;
      ringb->rx_buffer[ringb->rx_wr_index]=data;
      if (++ringb->rx_wr_index == RX_BUFFER_SIZE) ringb->rx_wr_index=0;
      if (++ringb->rx_counter  == RX_BUFFER_SIZE) {
         ringb->rx_counter=0;
         ringb->rx_buffer_overflow=1;
      }
//      HAL_NVIC_EnableIRQ(ringb->IRQn);
}

static void RingBuffer_TxISR(RINGBUFFER_ST * ringb){
   static char buffer[TX_BUFFER_SIZE];

  // Uart2_printf("RingBuffer_TxISR %d, run %d :\r\n", ringb->tx_counter,  ringb->tx_run );

   if (ringb->tx_counter) {
	 int len = ringb->tx_counter;
	 memset(buffer,0,TX_BUFFER_SIZE);

	  for (int i = 0; i < len; i++){

		  buffer[i] = ringb->tx_buffer[ringb->tx_rd_index];
		  if (++ringb->tx_rd_index == TX_BUFFER_SIZE) ringb->tx_rd_index=0;
	  }

	  tcp_server_write_len(buffer,(uint16_t)len );
	  ringb->tx_counter = 0;

   } else {
      ringb->tx_run = 0;
   }

}
static void RingBuffer_Init(RINGBUFFER_ST * ringb){

  RingBuffer_Flush(ringb);

}
static void RingBuffer_Tx(RINGBUFFER_ST * ringb, char  tx_char){


   static uint8_t buffer;
   if (ringb->tx_counter || (ringb->tx_run == 1)) {
      ringb->tx_buffer[ringb->tx_wr_index]=tx_char;
      if (++ringb->tx_wr_index == TX_BUFFER_SIZE) ringb->tx_wr_index=0;
      ++ringb->tx_counter;
   } else {
      ringb->tx_char = (uint8_t) tx_char;
      ringb->tx_run = 1;
      buffer = tx_char;

      tcp_server_write_len((char *)&buffer, 1);

   }

}
static int16_t RingBuffer_Rx(RINGBUFFER_ST * ringb){

    char data;
    if (ringb->rx_counter == 0) return -1;
    data = ringb->rx_buffer[ringb->rx_rd_index];

    if (++ringb->rx_rd_index == RX_BUFFER_SIZE) ringb->rx_rd_index=0;

//    NVIC_DisableIRQ(ringb->IRQn);
    --ringb->rx_counter;
//    NVIC_EnableIRQ(ringb->IRQn);

    return data;
}
//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
//   if (huart->Instance == USART2) {
//      RingBuffer_TxISR(&uart2_buffer);
//   }
//}
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//   if (huart->Instance == USART2) {
//      RingBuffer_RxISR(&uart2_buffer);
//   }
//}
//void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
//
//   if (huart->Instance == USART2) {
//      RingBuffer_Flush(&uart2_buffer);
//   }
//}
void TcpBufferInit(void){
  RingBuffer_Init(&tcp_buffer);
}


int16_t TcpGetchar(void){
  return RingBuffer_Rx(&tcp_buffer);
}
void TcpPutchar(char c){
  RingBuffer_Tx(&tcp_buffer,c);
}


void Tcp_printf (char * format, ...) {
#if 1
  char buffer[64];
  va_list args;
  va_start (args, format);
  vsnprintf (buffer, sizeof(buffer)-1, format, args);
  va_end (args);

  char *s = buffer;

  while(*s){
		TcpPutchar(*s++);
	}
#else
#warning Tcp_printf
#endif
}

void tcp_rx_printf (char * format, ...) {
//  char buffer[64];
//  va_list args;
//  va_start (args, format);
//  vsnprintf (buffer, sizeof(buffer)-1, format, args);
//  va_end (args);
//
//  char *s = buffer;
//
//  while(*s){
//		RingBuffer_RxPut(&tcp_buffer,*s++);
//  }

	char buffer[128];
	va_list args;

	va_start(args, format);
	int len = vsnprintf(buffer, sizeof(buffer), format, args);
	va_end(args);

	if(len > 0)
		tcp_server_write_len(buffer, len);
}





