#include "main.h"
#include "net.h"

#include "lwip/pbuf.h"

#if defined FREERTOS_STATS_DISPLAY && (FREERTOS_STATS_DISPLAY == 1)
//--------------------------------------------------------------------+
// FREERTOS STATS TASK
//--------------------------------------------------------------------+

#define STATS_STACK_SIZE             512

// Freertos stats task
void freertos_stats_task(void * param)
{
  (void) param;

  // Display buffer size, 40 bytes per task
  // see https://www.freertos.org/a00021.html for details 
  char buffer[40 * 5];

  while (1)
  {
    vTaskGetRunTimeStats(buffer);
    printf("Name\tCounter\t\tCPU\n");
    printf("%s\n", buffer);

    vTaskList(buffer);
    printf("Name\tState\tPriority\tFree\tId\n");
    printf("%s\n", buffer);

    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}
#endif

//--------------------------------------------------------------------+
// Inter-task buffer 
//--------------------------------------------------------------------+

static uint8_t bufferUsbToLwip[ CFG_TUD_NET_MTU*3 ];
StaticMessageBuffer_t usbToLwipMessageBufferStruct;
MessageBufferHandle_t usbToLwipMessageBuffer;

StreamBufferHandle_t fromUartStreamBuffer;
StreamBufferHandle_t toUartStreamBuffer;

//--------------------------------------------------------------------+
// USB network stack
//--------------------------------------------------------------------+

/* this is used by this code, ./class/net/net_driver.c, and usb_descriptors.c */
/* ideally speaking, this should be generated from the hardware's unique ID (if available) */
/* it is suggested that the first byte is 0x02 to indicate a link-local address */
const uint8_t tud_network_mac_address[6] = {0x02,0x02,0x84,0x6A,0x96,0x00};

// Increase stack size when debug log is enabled
#define USBD_STACK_SIZE    (3*configMINIMAL_STACK_SIZE/2) * (CFG_TUSB_DEBUG ? 2 : 1)
StackType_t  usb_stack[USBD_STACK_SIZE];
StaticTask_t usb_taskdef;

// Invoked when device is mounted
void tud_mount_cb(void)
{
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
  (void) remote_wakeup_en;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
}

void tud_network_init_cb(void)
{
  // Is this operation safe ???
  xMessageBufferReset(usbToLwipMessageBuffer);
  xStreamBufferReset(fromUartStreamBuffer);
  xStreamBufferReset(toUartStreamBuffer);
}

bool tud_network_recv_cb(const uint8_t *src, uint16_t size)
{
  if (size == 0)
  {
    return false;
  }
  
  if( xMessageBufferSend(usbToLwipMessageBuffer, (const void *) src, size, 0) != size)
  {
    return false;
  }

  return true;
}

uint16_t tud_network_xmit_cb(uint8_t *dst, void *ref, uint16_t arg)
{
  struct pbuf *p = (struct pbuf *)ref;

  (void)arg; /* unused for this example */

  return pbuf_copy_partial(p, dst, p->tot_len, 0);
}

// USB Device Driver task
// This top level thread process all usb events and invoke callbacks
void usb_task(void* param)
{
  (void) param;

  // This should be called after scheduler/kernel is started.
  // Otherwise it could cause kernel issue since USB IRQ handler does use RTOS queue API.
  tusb_init();

  // RTOS forever loop
  while (1)
  {
    // tinyusb device task
    tud_task();
  }
}

//--------------------------------------------------------------------+
// Main
//--------------------------------------------------------------------+

int main(void)
{
  board_init();

#if defined FREERTOS_STATS_DISPLAY && (FREERTOS_STATS_DISPLAY == 1)
  // Create a task for freertos stats
  xTaskCreate( freertos_stats_task, "stats", STATS_STACK_SIZE, ( void * ) NULL, configMAX_PRIORITIES-3, NULL );
#endif

  // create message buffer  
  usbToLwipMessageBuffer = xMessageBufferCreateStatic( sizeof( bufferUsbToLwip ), bufferUsbToLwip, &usbToLwipMessageBufferStruct);
  fromUartStreamBuffer = xStreamBufferCreate(1024, 1);
  toUartStreamBuffer = xStreamBufferCreate(1024, 1);

  // Create a task for tinyusb device stack
  (void) xTaskCreateStatic( usb_task, "usbd", USBD_STACK_SIZE, NULL, configMAX_PRIORITIES-1, usb_stack, &usb_taskdef);

  // Create a task for network (lwip) stack
  create_net_task();

  // Start scheduler
  vTaskStartScheduler();

  return 0;
}
