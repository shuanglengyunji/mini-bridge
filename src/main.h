/** 
 * Main.h provides shared headers for all tasks.
 * */

#ifndef __MAIN_H__
#define __MAIN_H__

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

// Include FreeRTOS
#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"
#include "task.h"
#include "timers.h"
#include "message_buffer.h"

// Include TinyUSB
#include "board.h"
#include "tusb.h"

/* Blink pattern
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum  {
  BLINK_NOT_MOUNTED = 250,
  BLINK_MOUNTED = 1000,
  BLINK_SUSPENDED = 2500,
};

// Inter-task buffers
extern MessageBufferHandle_t usbToLwipMessageBuffer;

#endif /* __MAIN_H__ */
