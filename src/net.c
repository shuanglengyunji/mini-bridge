#include "main.h"

// Include LWIP
#include "dhserver.h"
#include "lwip/init.h"
#include "lwip/timeouts.h"
#include "httpd.h"

// network task
#define NET_STACK_SZIE      2048
StackType_t  net_stack[NET_STACK_SZIE];
StaticTask_t net_taskdef;

#define INIT_IP4(a,b,c,d) { PP_HTONL(LWIP_MAKEU32(a,b,c,d)) }

/* lwip context */
static struct netif netif_data;

/* network parameters of this MCU */
static const ip4_addr_t ipaddr  = INIT_IP4(192, 168, 7, 1);
static const ip4_addr_t netmask = INIT_IP4(255, 255, 255, 0);
static const ip4_addr_t gateway = INIT_IP4(0, 0, 0, 0);
static const uint8_t    mac_address[6] = {0x02,0x02,0x84,0x6A,0x96,0x01};

/* database IP addresses that can be offered to the host; this must be in RAM to store assigned MAC addresses */
static dhcp_entry_t entries[] =
{
    /* ip address                    lease time */
    { {0}, INIT_IP4(192, 168, 7, 2), 24 * 60 * 60 },
    { {0}, INIT_IP4(192, 168, 7, 3), 24 * 60 * 60 },
    { {0}, INIT_IP4(192, 168, 7, 4), 24 * 60 * 60 },
};

static const dhcp_config_t dhcp_config =
{
    .router = INIT_IP4(0, 0, 0, 0),            /* router address (if any) */
    .port = 67,                                /* listen port */
    .dns = INIT_IP4(192, 168, 7, 1),           /* dns server (if any) */
    "usb",                                     /* dns suffix */
    TU_ARRAY_SIZE(entries),                    /* num entry */
    entries                                    /* entries */
};

static err_t linkoutput_fn(struct netif *netif, struct pbuf *p)
{
  (void)netif;

  for (;;)
  {
    /* if TinyUSB isn't ready, we must signal back to lwip that there is nothing we can do */
    if (!tud_ready())
      return ERR_USE;

    /* if the network driver can accept another packet, we make it happen */
    if (tud_network_can_xmit(p->tot_len))
    {
      tud_network_xmit(p, 0 /* unused */);
      return ERR_OK;
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

static err_t ip4_output_fn(struct netif *netif, struct pbuf *p, const ip4_addr_t *addr)
{
  return etharp_output(netif, p, addr);
}

static err_t netif_init_cb(struct netif *netif)
{
  LWIP_ASSERT("netif != NULL", (netif != NULL));
  netif->mtu = CFG_TUD_NET_MTU;
  netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_LINK_UP | NETIF_FLAG_UP;
  netif->state = NULL;
  netif->name[0] = 'E';
  netif->name[1] = 'X';
  netif->linkoutput = linkoutput_fn;
  netif->output = ip4_output_fn;
  return ERR_OK;
}

static void init_lwip(void)
{
  struct netif *netif = &netif_data;

  lwip_init();

  /* the lwip virtual MAC address must be different from the host's; to ensure this, we toggle the LSbit */
  netif->hwaddr_len = sizeof(mac_address);
  memcpy(netif->hwaddr, mac_address, sizeof(mac_address));

  netif = netif_add(netif, &ipaddr, &netmask, &gateway, NULL, netif_init_cb, ip_input);
  netif_set_default(netif);
}

/** This basic CGI function can parse param/value pairs and return an url that
 * is sent as a response by httpd.
 */
static const char * cgi_handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[])
{
  (void) iIndex;

  for (int i=0; i<iNumParams; i++)
  {
    printf("%d %s %s\n", i, pcParam[i], pcValue[i]);
  }
  return "/index.shtml";
}

static const tCGI cgi_handlers[] = {
  {
    "/text_cgi",
    cgi_handler
  }
};

size_t print_ip4_addr_t(char* dst, int max_len, ip4_addr_t ip)
{
  return snprintf(dst, max_len, "%ld.%ld.%ld.%ld", ip.addr & 0xFF, (ip.addr>>8) & 0xFF, (ip.addr>>16) & 0xFF, ip.addr>>24);
}

u16_t ssi_handler(const char* ssi_tag_name, char *pcInsert, int iInsertLen)
{
  if (!strcmp(ssi_tag_name, "SERADD"))
  {
    return (u16_t)print_ip4_addr_t(pcInsert, iInsertLen, ipaddr);
  } 
  else if (!strcmp(ssi_tag_name, "SERPORT"))
  {
    return (u16_t)snprintf(pcInsert, iInsertLen, "%d", 8000);
  }
  else if (!strcmp(ssi_tag_name, "DESTADD"))
  {
    return (u16_t)print_ip4_addr_t(pcInsert, iInsertLen, ipaddr);
  }
  else if (!strcmp(ssi_tag_name, "DESTPORT"))
  {
    return (u16_t)snprintf(pcInsert, iInsertLen, "%d", 8000);
  }
  else
  {
    return (u16_t)snprintf(pcInsert, iInsertLen, "Error: Known tag");
  }
}

void net_task(void* params)
{
  (void) params;

#if (LWIP_STATS_DISPLAY != 0)
  uint32_t last_display = 0;
#endif /* LWIP_STATS_DISPLAY */

  init_lwip();
  while (!netif_is_up(&netif_data));
  while (dhserv_init(&dhcp_config) != ERR_OK);

  http_set_cgi_handlers(cgi_handlers, LWIP_ARRAYSIZE(cgi_handlers));
  http_set_ssi_handler(ssi_handler, NULL, 0);
  httpd_init();

  // RTOS forever loop
  while ( 1 )
  {
    struct pbuf *p = pbuf_alloc(PBUF_RAW, CFG_TUD_NET_MTU, PBUF_POOL);  // CFG_TUD_NET_MTU is the maximum package length 
    if (p)
    {
      size_t size = xMessageBufferReceive(usbToLwipMessageBuffer, (void*)p->payload, CFG_TUD_NET_MTU, 0);
      if (size > 0)
      {
        pbuf_realloc(p, size);
        ethernet_input(p, &netif_data);
        tud_network_recv_renew();
      }
      pbuf_free(p);
    }
    sys_check_timeouts();

#if (LWIP_STATS_DISPLAY != 0)
    if (board_millis() > last_display + LWIP_STATS_DISPLAY_PERIOD_MS)
    {
      stats_display();
      last_display = board_millis();
    }
#endif /* LWIP_STATS_DISPLAY */
  }
}

void create_net_task(void)
{
  // Create net task
  (void) xTaskCreateStatic( net_task, "net", NET_STACK_SZIE, NULL, configMAX_PRIORITIES-2, net_stack, &net_taskdef);
}

//--------------------------------------------------------------------+
// LWIP system support (fake) 
//--------------------------------------------------------------------+

/* lwip has provision for using a mutex, when applicable */
sys_prot_t sys_arch_protect(void)
{
  return 0;
}

void sys_arch_unprotect(sys_prot_t pval)
{
  (void)pval;
}

/* lwip needs a millisecond time source, and the TinyUSB board support code has one available */
uint32_t sys_now(void)
{
  return board_millis();
}
