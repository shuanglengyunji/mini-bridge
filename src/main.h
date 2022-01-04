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

// Inter-task buffers
extern MessageBufferHandle_t usbToLwipMessageBuffer;
extern StreamBufferHandle_t fromUartStreamBuffer;
extern StreamBufferHandle_t toUartStreamBuffer;

#endif /* __MAIN_H__ */
