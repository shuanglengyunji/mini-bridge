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
 * This file is part of the TinyUSB stack.
 */

#include "stm32f4xx_hal.h"
#include "board.h"
#include "main.h"

#define TX_LED_PORT           GPIOB
#define TX_LED_PIN            GPIO_PIN_6
#define TX_LED_STATE_ON       0

#define RX_LED_PORT           GPIOB
#define RX_LED_PIN            GPIO_PIN_7
#define RX_LED_STATE_ON       0

// LED
#define LED_PORT              TX_LED_PORT
#define LED_PIN               TX_LED_PIN
#define LED_STATE_ON          TX_LED_STATE_ON

//--------------------------------------------------------------------+
// RCC Clock
//--------------------------------------------------------------------+
static inline void board_clock_init(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = HSE_VALUE/1000000;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  // Enable clocks for LED, Uart, USB
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_USART1_CLK_ENABLE();
  __HAL_RCC_USART2_CLK_ENABLE();
  __HAL_RCC_USB_OTG_FS_CLK_ENABLE();
}

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM
//--------------------------------------------------------------------+
UART_HandleTypeDef UartHandle;
UART_HandleTypeDef Uart2Handle;
#if defined FREERTOS_STATS_DISPLAY && (FREERTOS_STATS_DISPLAY == 1)
TIM_HandleTypeDef htim2;
#endif

void board_init(void)
{
  board_clock_init();
  //SystemCoreClockUpdate();

  // Explicitly disable systick to prevent its ISR runs before scheduler start
  SysTick->CTRL &= ~1U;

  // Set NVIC to priority group 4 for STM32, which assigns all priority bits to preempt priority bits
  // see https://www.freertos.org/RTOS-Cortex-M3-M4.html for details 
  // the NVIC_PriorityGroupConfig function from the Standard Peripheral Library 
  // is replaced by its HAL successor in our implementation 
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);  

  // In freeRTOS, IRQ priority is limit by max syscall ( smaller is higher )
  // Freertos use CMSIS's NVIC configuration function on its CM4 portings
  NVIC_SetPriority(OTG_FS_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY );

  GPIO_InitTypeDef  GPIO_InitStruct;

  // LED
  GPIO_InitStruct.Pin = LED_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);

  board_led_write(false);

  // UART
  GPIO_InitStruct.Pin       = GPIO_PIN_9 | GPIO_PIN_10;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  UartHandle = (UART_HandleTypeDef){
    .Instance        = USART1,
    .Init.BaudRate   = CFG_BOARD_UART_BAUDRATE,
    .Init.WordLength = UART_WORDLENGTH_8B,
    .Init.StopBits   = UART_STOPBITS_1,
    .Init.Parity     = UART_PARITY_NONE,
    .Init.HwFlowCtl  = UART_HWCONTROL_NONE,
    .Init.Mode       = UART_MODE_TX_RX,
    .Init.OverSampling = UART_OVERSAMPLING_16
  };
  HAL_UART_Init(&UartHandle);

  // UART2
  GPIO_InitStruct.Pin       = GPIO_PIN_2 | GPIO_PIN_3;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  Uart2Handle = (UART_HandleTypeDef){
    .Instance        = USART2,
    .Init.BaudRate   = CFG_BOARD_UART_BAUDRATE,
    .Init.WordLength = UART_WORDLENGTH_8B,
    .Init.StopBits   = UART_STOPBITS_1,
    .Init.Parity     = UART_PARITY_NONE,
    .Init.HwFlowCtl  = UART_HWCONTROL_NONE,
    .Init.Mode       = UART_MODE_TX_RX,
    .Init.OverSampling = UART_OVERSAMPLING_16
  };
  HAL_UART_Init(&Uart2Handle);
  NVIC_SetPriority(USART2_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
  NVIC_EnableIRQ(USART2_IRQn);
  __HAL_UART_ENABLE_IT(&Uart2Handle, UART_IT_RXNE);   // Enable the UART Data Register not empty Interrupt
  // __HAL_UART_ENABLE_IT(&Uart2Handle, UART_IT_TC);    // Enable the UART Transmit complete Interrupt

  // USB
  // Configure USB D+ D- Pins
  GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Blackpill doesn't use VBUS sense (B device) explicitly disable it
  USB_OTG_FS->GCCFG |= USB_OTG_GCCFG_NOVBUSSENS;
  USB_OTG_FS->GCCFG &= ~USB_OTG_GCCFG_VBUSBSEN;
  USB_OTG_FS->GCCFG &= ~USB_OTG_GCCFG_VBUSASEN;

  // Force device mode so the ID Pin can be ignored
  USB_OTG_FS->GUSBCFG &= ~USB_OTG_GUSBCFG_FHMOD;
  USB_OTG_FS->GUSBCFG |= USB_OTG_GUSBCFG_FDMOD;

#if defined FREERTOS_STATS_DISPLAY && (FREERTOS_STATS_DISPLAY == 1)
  // Init TIM2 CLK
  __HAL_RCC_TIM2_CLK_ENABLE();

  // If freeRTOS is used, IRQ priority is limit by max syscall ( smaller is higher )
  NVIC_SetPriority(TIM2_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY+1);

  // Init TIM2
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 840-1;  // CLK=84MHz
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  HAL_TIM_Base_Init(&htim2);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);
#endif
}

//--------------------------------------------------------------------+
// Forward USB interrupt events to TinyUSB IRQ Handler
//--------------------------------------------------------------------+
void OTG_FS_IRQHandler(void)
{
  tud_int_handler(0);
}

//--------------------------------------------------------------------+
// Handle USART2 interrupt transmission
//--------------------------------------------------------------------+

void USART2_IRQHandler(void)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE; // Initialised to pdFALSE

  uint32_t isrflags = READ_REG(Uart2Handle.Instance->SR);
  if ((isrflags & USART_SR_RXNE) != RESET)
  {
    const uint8_t data = (Uart2Handle.Instance->DR & (uint8_t)0x00FF);
    xStreamBufferSendFromISR(fromUartStreamBuffer, &data, 1, &xHigherPriorityTaskWoken);
  }
  if ((isrflags & USART_SR_TXE) != RESET)
  {
    uint8_t data;
    if (xStreamBufferReceiveFromISR(toUartStreamBuffer, &data, 1, &xHigherPriorityTaskWoken))
    {
      Uart2Handle.Instance->DR = data;
    }
    else
    {
      __HAL_UART_DISABLE_IT(&Uart2Handle, UART_IT_TXE);
    }
  }

  // BUG?? Cannot find taskYIELD_FROM_ISR() in tasks.h
  // Use taskYIELD_FROM_ISR() in accord with
  // https://forums.freertos.org/t/taskyield-from-isr-portyield-from-isr-and-portend-switching-isr/12792
  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );   // taskYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+

void board_led_write(bool state)
{
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, state ? LED_STATE_ON : (1-LED_STATE_ON));
}

int board_uart_read(uint8_t* buf, int len)
{
  (void) buf; (void) len;
  return 0;
}

int board_uart_write(void const * buf, int len)
{
  HAL_UART_Transmit(&UartHandle, (uint8_t*)(uintptr_t) buf, len, 0xffff);
  return len;
}

uint32_t board_uart2_read(uint8_t* buf, uint32_t len)
{
  return xStreamBufferReceive(fromUartStreamBuffer, (void *)buf, len, 0);
}

void board_uart2_write(uint8_t * buf, uint32_t len)
{
  if (len <= 0)
  {
    return;
  }
  // TODO: check fifo space before sending
  // TODO: return sent bytes 
  xStreamBufferSend(toUartStreamBuffer, (void *)buf, len, 0);
  __HAL_UART_ENABLE_IT(&Uart2Handle, UART_IT_TXE);
}

#if defined FREERTOS_STATS_DISPLAY && (FREERTOS_STATS_DISPLAY == 1)
volatile uint32_t htim2_ticks = 0;
void TIM2_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim2);
  htim2_ticks++;
}
void board_timer2_start(void)
{
  NVIC_EnableIRQ(TIM2_IRQn);
  HAL_TIM_Base_Start_IT(&htim2);
}
uint32_t board_timer2_ticks(void)
{
  return htim2_ticks;
}
#endif

void HardFault_Handler (void)
{
  asm("bkpt");
}

// Required by __libc_init_array in startup code if we are compiling using
// -nostdlib/-nostartfiles.
void _init(void)
{

}

//--------------------------------------------------------------------+
// newlib read()/write() retarget
//--------------------------------------------------------------------+

#define sys_write   _write
#define sys_read    _read

#if defined(LOGGER_SWO)
// Logging with SWO for ARM Cortex
TU_ATTR_USED int sys_write (int fhdl, const void *buf, size_t count)
{
  (void) fhdl;
  uint8_t const* buf8 = (uint8_t const*) buf;
  for(size_t i=0; i<count; i++)
  {
    ITM_SendChar(buf8[i]);
  }
  return count;
}
TU_ATTR_USED int sys_read (int fhdl, char *buf, size_t count)
{
  (void) fhdl;
  (void) buf;
  (void) count;
  return 0;
}
#else
// Default logging with on-board UART
TU_ATTR_USED int sys_write (int fhdl, const void *buf, size_t count)
{
  (void) fhdl;
  return board_uart_write(buf, (int) count);
}
TU_ATTR_USED int sys_read (int fhdl, char *buf, size_t count)
{
  (void) fhdl;
  return board_uart_read((uint8_t*) buf, (int) count);
}
#endif
