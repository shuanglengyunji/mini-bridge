#include "udpserver.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"

#define UDP_SERVER_PORT    8000   /* define the UDP local connection port */

struct ip4_addr dst_ip = IPADDR4_INIT_BYTES(192, 168, 7, 2);
u16_t dst_port = 8000;

struct udp_pcb *udp_server_pcb = NULL;

/**
  * @brief This function is called when an UDP datagrm has been received on the port UDP_PORT.
  * @param arg user supplied argument (udp_pcb.recv_arg)
  * @param pcb the udp_pcb which received data
  * @param p the packet buffer that was received
  * @param addr the remote IP address from which the packet was received
  * @param port the remote port from which the packet was received
  * @retval None
  */
static void udp_echoserver_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const struct ip4_addr *addr, u16_t port)
{
  LWIP_UNUSED_ARG(arg);
//   LWIP_UNUSED_ARG(upcb);

  /*data loopback*/
  udp_sendto(upcb, p, addr, port);

  /* Record remote client */
  dst_ip = *addr;
  dst_port = port;

  /* Free the p buffer */
  pbuf_free(p);
}

/**
  * @brief  Initialize the server application.
  * @param  None
  * @retval None
  */
void udp_echoserver_init(void)
{ 
  /* Create a new UDP control block  */
  udp_server_pcb = udp_new();

  if (udp_server_pcb == NULL) {
    printf("can not create udp pcb\n");
    return;
  }

  /* Bind the upcb to the UDP_PORT port */
  /* Using IP_ADDR_ANY allow the upcb to be used by any local interface */
  if (udp_bind(udp_server_pcb, IP_ADDR_ANY, UDP_SERVER_PORT) != ERR_OK) {
    printf("can not bind udp pcb\n");
    memp_free(MEMP_UDP_PCB, udp_server_pcb);
    return;
  }

  /* Set a receive callback for the upcb */
  udp_recv(udp_server_pcb, udp_echoserver_receive_callback, NULL);
}

void udp_echoserver_send(unsigned char * src, int len) {

  struct pbuf *p;

  /* verify upcb */
  if (!udp_server_pcb) {
    return;
  }

  /* allocate pbuf */
  p = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
  
  if (p != NULL) {
    /* copy data to pbuf */
    pbuf_take(p, (unsigned char *)src, len);
      
    /* send udp data */
    udp_sendto(udp_server_pcb, p, &dst_ip, dst_port);
    
    /* free pbuf */
    pbuf_free(p);
  }
}