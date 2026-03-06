#ifndef __TCP_SERVER_H
#define __TCP_SERVER_H


#include "lwip/tcp.h"

struct tcp_server {
	struct tcp_pcb *tcp_server_pcb;
	struct tcp_pcb *client_pcb;
	uint16_t server_port;

	err_t (*accept)(void *arg, struct tcp_pcb *new_pcb, err_t err);
	err_t (*receive)(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
	void (*error)(void *arg, err_t err);
};

extern void tcp_server_init(void);

extern int16_t 	    TcpGetchar(void);
extern void 		TcpPutchar(char c);
extern void 		Tcp_printf (char * format, ...) ;
extern void 		tcp_rx_printf (char * format, ...);


#endif
