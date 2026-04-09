#pragma once

#include "main.h"

#ifdef HAL_UART_MODULE_ENABLED

#ifdef STM32_LIBRARY_STDIO_ENABLE
#define STM32_LIBRARY_STD_PRINTF_ENABLE
#define STM32_LIBRARY_STD_SCANF_ENABLE
#endif

#include <stdio.h>

namespace stm32_library::stm32_peripherals {

extern UART_HandleTypeDef *hstduart;

#ifdef STM32_LIBRARY_STD_PRINTF_ENABLE
void enable_std_printf(UART_HandleTypeDef *handle);
#endif

#ifdef STM32_LIBRARY_STD_SCANF_ENABLE
void enable_std_scanf(UART_HandleTypeDef *handle);
#endif

#if defined(STM32_LIBRARY_STD_PRINTF_ENABLE) && defined(STM32_LIBRARY_STD_SCANF_ENABLE)
void enable_stdio(UART_HandleTypeDef *handle);
#endif

} // namespace stm32_library::stm32_peripherals

#ifdef STM32_LIBRARY_STD_PRINTF_ENABLE
extern "C" int __io_putchar(int ch);
#endif

#ifdef STM32_LIBRARY_STD_SCANF_ENABLE
extern "C" int __io_getchar(void);
#endif

#endif // HAL_UART_MODULE_ENABLED
