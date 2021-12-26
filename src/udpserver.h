#ifndef __UDP_SERVER_H__
#define __UDP_SERVER_H__

#include "lwip/pbuf.h"
#include "lwip/udp.h"

#define UDP_SERVER_PORT    8000   /* define the UDP local connection port */

enum {
    DATA_SPECIFIC_PORT = 0,
    DATA_ALL_PORT,
    DATA_PORT_UPDATE
};

void udp_echoserver_init(ip_addr_t *local_ip, uint16_t local_port);
void udp_echoserver_send(unsigned char * src, int len);

#endif /* __UDP_SERVER_H__ */