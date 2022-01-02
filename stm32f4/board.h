#ifndef __BOARD_H_
#define __BOARD_H_

#include <stdint.h>
#include <stdbool.h>

#include "tusb.h"

#define CFG_BOARD_UART_BAUDRATE    115200

// Initialize on-board peripherals : led, uart and USB
void board_init(void);

// Uart2 functions
uint32_t board_uart2_read(uint8_t* buf, uint32_t len);
void board_uart2_write(uint8_t * buf, uint32_t len);

static inline uint32_t board_millis(void)
{
  return ( ( ((uint64_t) xTaskGetTickCount()) * 1000) / configTICK_RATE_HZ );
}

#endif /* __BOARD_H_ */
