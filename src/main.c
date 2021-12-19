/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include "main.h"
#include "net.h"

#include "lwip/pbuf.h"

#if defined FREERTOS_STATS_DISPLAY && (FREERTOS_STATS_DISPLAY == 1)
// Heap space for freertos task's dynamic memory allocation 
uint8_t ucHeap[ configTOTAL_HEAP_SIZE ];

//--------------------------------------------------------------------+
// FREERTOS STATS TASK
//--------------------------------------------------------------------+

// Display buffer size, 40 bytes per task
// see https://www.freertos.org/a00021.html for details 
#define DISPLAY_BUFFER_SIZE         40 * 5

// Freertos stats task
void freertos_stats_task(void * param)
{
  (void) param;

  char buffer[DISPLAY_BUFFER_SIZE];
  uint32_t last_display = 0;
  
  while (1)
  {
    uint32_t current_ms = board_millis();
    if (current_ms >= last_display + 5000 || current_ms < last_display)
    {
      vTaskGetRunTimeStats(buffer);
      printf("Name\tCounter\t\tCPU\n");
      printf("%s\n", buffer);

      // BUG ??
      // Starting from the second time, the free memory of this task (the 
      // stats task) calculated by vTaskList is 0
      vTaskList(buffer);
      printf("Name\tState\tPriority\tFree\tId\n");
      printf("%s\n", buffer);

      last_display = current_ms;
    } 
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}
#endif

//--------------------------------------------------------------------+
// Inter-task buffer 
//--------------------------------------------------------------------+

static uint8_t bufferUsbToLwip[ CFG_TUD_NET_MTU*3 ];
StaticMessageBuffer_t usbToLwipMessageBufferStruct;
MessageBufferHandle_t usbToLwipMessageBuffer;

//--------------------------------------------------------------------+
// BLINKING TASK (toggle led)
//--------------------------------------------------------------------+

// led timer
StaticTimer_t blinky_tmdef;
TimerHandle_t blinky_tm;

void led_blinky_cb(TimerHandle_t xTimer)
{
  (void) xTimer;
  static bool led_state = false;

  board_led_write(led_state);
  led_state = 1 - led_state; // toggle
}

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
  xTimerChangePeriod(blinky_tm, pdMS_TO_TICKS(BLINK_MOUNTED), 0);
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
  xTimerChangePeriod(blinky_tm, pdMS_TO_TICKS(BLINK_NOT_MOUNTED), 0);
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
  (void) remote_wakeup_en;
  xTimerChangePeriod(blinky_tm, pdMS_TO_TICKS(BLINK_SUSPENDED), 0);
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
  xTimerChangePeriod(blinky_tm, pdMS_TO_TICKS(BLINK_MOUNTED), 0);
}

void tud_network_init_cb(void)
{
  // Is this operation safe ???
  xMessageBufferReset(usbToLwipMessageBuffer);
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
  xTaskCreate( freertos_stats_task, "stats", configMINIMAL_STACK_SIZE, ( void * ) NULL, configMAX_PRIORITIES-3, NULL );
#endif

  // create message buffer  
  usbToLwipMessageBuffer = xMessageBufferCreateStatic( sizeof( bufferUsbToLwip ), bufferUsbToLwip, &usbToLwipMessageBufferStruct);
  
  // Create a soft timer for blinky
  blinky_tm = xTimerCreateStatic(NULL, pdMS_TO_TICKS(BLINK_NOT_MOUNTED), true, NULL, led_blinky_cb, &blinky_tmdef);
  xTimerStart(blinky_tm, 0);

  // Create a task for tinyusb device stack
  (void) xTaskCreateStatic( usb_task, "usbd", USBD_STACK_SIZE, NULL, configMAX_PRIORITIES-1, usb_stack, &usb_taskdef);

  // Create a task for network (lwip) stack
  create_net_task();

  // Start scheduler
  vTaskStartScheduler();

  return 0;
}
