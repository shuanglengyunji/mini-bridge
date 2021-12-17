#ifndef __UDP_SERVER_H__
#define __UDP_SERVER_H__

void udp_echoserver_init(void);
void udp_echoserver_send(unsigned char * src, int len);

#endif /* __UDP_SERVER_H__ */