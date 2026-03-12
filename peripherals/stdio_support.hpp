#pragma once

#include "main.h"

#ifdef HAL_UART_MODULE_ENABLED

#ifdef STM32_LIBRARY_STDIO_ENABLE
#define STM32_LIBRARY_STD_PRINTF_ENABLE
#define STM32_LIBRARY_STD_SCANF_ENABLE
#endif // STM32_LIBRARY_STDIO_ENABLE

#include <stdio.h>

namespace abu2023::stm32_peripherals {
[[maybe_unused]] static UART_HandleTypeDef *hstduart;

#ifdef STM32_LIBRARY_STD_PRINTF_ENABLE
// 浮動小数点の表示を使う場合はリンカーフラグに-u_printf_floatを追加
void enable_std_printf(UART_HandleTypeDef *handle) {
  hstduart = handle;
  // HalStartupManager::add_startup_callback( [](){ setbuf(stdout, NULL); } );
#ifdef STM32_LIBRARY_USE_STATIC_INSTANCE
  HalStartupManager::add_startup_callback([this] { setbuf(stdout, NULL); });
#else
  setbuf(stdout, NULL);
#endif
}
#endif // STM32_LIBRARY_STD_PRINTF_ENABLE

#ifdef STM32_LIBRARY_STD_SCANF_ENABLE
// 浮動小数点の入力を使う場合はリンカーフラグに-u_scanf_floatを追加
void enable_std_scanf(UART_HandleTypeDef *handle) {
  hstduart = handle;
#ifdef STM32_LIBRARY_USE_STATIC_INSTANCE
  HALStartupManager::add_startup_callback([this] { setbuf(stdin, NULL); });
#else
  setbuf(stdin, NULL);
#endif
}
#endif // STM32_LIBRARY_STD_SCANF_ENABLE

#if defined(STM32_LIBRARY_STD_PRINTF_ENABLE) && defined(STM32_LIBRARY_STD_SCANF_ENABLE)
void enable_stdio(UART_HandleTypeDef *handle) {
  enable_std_printf(handle);
  enable_std_scanf(handle);
}
#endif

} // namespace abu2023::stm32_peripherals

#ifdef STM32_LIBRARY_STD_PRINTF_ENABLE
extern "C" int __io_putchar(int ch) {
  if (abu2023::stm32_peripherals::hstduart != NULL)
    HAL_UART_Transmit(abu2023::stm32_peripherals::hstduart, (uint8_t *)&ch, 1, 10);
  return 0;
}
#endif // STM32_LIBRARY_STD_PRINTF_ENABLE

#ifdef STM32_LIBRARY_STD_SCANF_ENABLE
extern "C" int __io_getchar(void) {
  if (abu2023::stm32_peripherals::hstduart != NULL) {
    HAL_StatusTypeDef status = HAL_BUSY;
    uint8_t data;

    while (status != HAL_OK)
      status = HAL_UART_Receive(abu2023::stm32_peripherals::hstduart, &data, 1, 10);

    return data;
  }
  return 0;
}

#endif // STM32_LIBRARY_STD_SCANF_ENABLE

#endif // HAL_UART_MODULE_ENABLED
