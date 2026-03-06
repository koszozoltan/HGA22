/**
  ******************************************************************************
  * @file    LwIP/LwIP_UDP_Echo_Server/Src/udp_echoserver.c
  * @author  MCD Application Team
  * @brief   UDP echo server
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017 STMicroelectronics.
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
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "lwip/tcp.h"
#include <string.h>
#include <stdio.h>


#include "stdio.h"
#include "stdlib.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define UDP_SERVER_PORT    123   /* define the UDP local connection port */
#define UDP_CLIENT_PORT    123   /* define the UDP remote connection port */
#define NTP_PACKET_SIZE 48
#define NTP_UNIX_EPOCH_DELTA 2208988800UL

extern uint64_t get_unix_time_us(void);
extern void Uart2_printf (char * format, ...) ;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void udp_server_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initialize the server application.
  * @param  None
  * @retval None
  */
void udp_server_init(void)
{
   struct udp_pcb *upcb;
   err_t err;
   
   /* Create a new UDP control block  */
   upcb = udp_new();
   
   if (!upcb) return;

   if (upcb)
   {
     /* Bind the upcb to the UDP_PORT port */
     /* Using IP_ADDR_ANY allow the upcb to be used by any local interface */
      err = udp_bind(upcb, IP_ADDR_ANY, UDP_SERVER_PORT);
      
      if(err == ERR_OK)
      {
        /* Set a receive callback for the upcb */
        udp_recv(upcb, udp_server_receive_callback, NULL);
      }
   }
}
#if 0
static void write_u32(uint8_t *buf, uint32_t v) {
    buf[0] = (v >> 24) & 0xFF;
    buf[1] = (v >> 16) & 0xFF;
    buf[2] = (v >> 8) & 0xFF;
    buf[3] = v & 0xFF;
}
static void ntp_fill_timestamp(uint8_t *buf, uint64_t unix_us) {
    uint64_t sec = unix_us / 1000000ULL;
    uint64_t frac_us = unix_us % 1000000ULL;

    uint32_t ntp_sec = (uint32_t)(sec + NTP_UNIX_EPOCH_DELTA);
    uint32_t ntp_frac = (uint32_t)((frac_us * ((uint64_t)1<<32)) / 1000000ULL);

    write_u32(buf, ntp_sec);
    write_u32(buf + 4, ntp_frac);
}

/**
  * @brief This function is called when an UDP datagrm has been received on the port UDP_PORT.
  * @param arg user supplied argument (udp_pcb.recv_arg)
  * @param pcb the udp_pcb which received data
  * @param p the packet buffer that was received
  * @param addr the remote IP address from which the packet was received
  * @param port the remote port from which the packet was received
  * @retval None
  */
void udp_server_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{


	if (!p) return;

	Uart2_printf("\r\nget udp data [%d]\r\n", p->len);
	if (p->len < NTP_PACKET_SIZE) {
		pbuf_free(p);
		return;
	}

	uint8_t *req = (uint8_t*)p->payload;
	struct pbuf *resp = pbuf_alloc(PBUF_TRANSPORT, NTP_PACKET_SIZE, PBUF_RAM);
	if (!resp)
	{
		pbuf_free(p);
		return;
	}

	uint8_t *buf = (uint8_t*)resp->payload;
	memset(buf, 0, NTP_PACKET_SIZE);
 // LI=0 VN=4 MODE=4 (server)
	buf[0] = (0 << 6) | (4 << 3) | 4;
	buf[1] = 1; // stratum
	buf[2] = 6; // poll
	buf[3] = 0xEC; // precision (~-20)

	// Root delay / dispersion
	write_u32(buf + 4, 1 << 16);
	write_u32(buf + 8, 1 << 16);

	// Reference ID
	buf[12] = 'H';
	buf[13] = 'G';
	buf[14] = 'A';
	buf[15] = '2';
	uint64_t now = get_unix_time_us();

   // Originate timestamp (client transmit -> copy)
	memcpy(buf + 24, req + 40, 8);

	// Receive timestamp
	ntp_fill_timestamp(buf + 32, now);

	// Transmit timestamp
	ntp_fill_timestamp(buf + 40, now);

	udp_sendto(upcb, resp, addr, port);

	pbuf_free(resp);
	pbuf_free(p);
	

}
#endif
/*******************************************************************************************/
static void write_u32(uint8_t *buf, uint32_t v) {
    buf[0] = (v >> 24) & 0xFF;
    buf[1] = (v >> 16) & 0xFF;
    buf[2] = (v >> 8) & 0xFF;
    buf[3] = v & 0xFF;
}

static void ntp_fill_timestamp(uint8_t *buf, uint64_t unix_us) {
    uint64_t sec = unix_us / 1000000ULL;
    uint64_t usec = unix_us % 1000000ULL;

    uint32_t ntp_sec = (uint32_t)(sec + NTP_UNIX_EPOCH_DELTA);

    // Pontos frakció számítás: (usec / 1e6) * 2^32
    uint32_t ntp_frac = (uint32_t)(((usec * 4294967296ULL) + 500000ULL) / 1000000ULL);
    // +500000 az egész számra való kerekítéshez

    write_u32(buf, ntp_sec);
    write_u32(buf + 4, ntp_frac);
}

void udp_server_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
    if (!p) return;
    if (p->len < NTP_PACKET_SIZE) {
        pbuf_free(p);
        return;
    }

    uint8_t *req = (uint8_t*)p->payload;
    struct pbuf *resp = pbuf_alloc(PBUF_TRANSPORT, NTP_PACKET_SIZE, PBUF_RAM);
    if (!resp) {
        pbuf_free(p);
        return;
    }

    uint8_t *buf = (uint8_t*)resp->payload;
    memset(buf, 0, NTP_PACKET_SIZE);

    // LI=0, VN=4, MODE=4 (server)
    buf[0] = (0 << 6) | (4 << 3) | 4;
    buf[1] = 1; // Stratum
    buf[2] = 6; // Poll
    buf[3] = 0xEC; // Precision (~-20)

    // Root delay / dispersion
    write_u32(buf + 4, 1 << 16);
    write_u32(buf + 8, 1 << 16);

    // Reference ID
    buf[12] = 'H';
    buf[13] = 'G';
    buf[14] = 'A';
    buf[15] = '2';

    uint64_t now_us = get_unix_time_us();

    // Originate timestamp: másold a kliens által küldött transmit timestamp-et
    memcpy(buf + 24, req + 40, 8);

    // Receive timestamp
    ntp_fill_timestamp(buf + 32, now_us);

    // Transmit timestamp
    ntp_fill_timestamp(buf + 40, now_us);

    udp_sendto(upcb, resp, addr, port);

    pbuf_free(resp);
    pbuf_free(p);
}




