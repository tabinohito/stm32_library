#include "../Inc/stdio_support.hpp"

#ifdef HAL_UART_MODULE_ENABLED

namespace stm32_library::stm32_peripherals {

UART_HandleTypeDef *hstduart = nullptr;

#ifdef STM32_LIBRARY_STD_PRINTF_ENABLE
void enable_std_printf(UART_HandleTypeDef *handle) {
  hstduart = handle;
#ifdef STM32_LIBRARY_USE_STATIC_INSTANCE
  HalStartupManager::add_startup_callback([] { setbuf(stdout, NULL); });
#else
  setbuf(stdout, NULL);
#endif
}
#endif

#ifdef STM32_LIBRARY_STD_SCANF_ENABLE
void enable_std_scanf(UART_HandleTypeDef *handle) {
  hstduart = handle;
#ifdef STM32_LIBRARY_USE_STATIC_INSTANCE
  HALStartupManager::add_startup_callback([] { setbuf(stdin, NULL); });
#else
  setbuf(stdin, NULL);
#endif
}
#endif

#if defined(STM32_LIBRARY_STD_PRINTF_ENABLE) && defined(STM32_LIBRARY_STD_SCANF_ENABLE)
void enable_stdio(UART_HandleTypeDef *handle) {
  enable_std_printf(handle);
  enable_std_scanf(handle);
}
#endif

} // namespace stm32_library::stm32_peripherals

#ifdef STM32_LIBRARY_STD_PRINTF_ENABLE
extern "C" int __io_putchar(int ch) {
  if (stm32_library::stm32_peripherals::hstduart != nullptr) {
    HAL_UART_Transmit(
      stm32_library::stm32_peripherals::hstduart,
      reinterpret_cast<uint8_t *>(&ch), 1, 10
    );
  }
  return ch;
}
#endif

#ifdef STM32_LIBRARY_STD_SCANF_ENABLE
extern "C" int __io_getchar(void) {
  if (stm32_library::stm32_peripherals::hstduart != nullptr) {
    HAL_StatusTypeDef status = HAL_BUSY;
    uint8_t data = 0;

    while (status != HAL_OK) {
      status = HAL_UART_Receive(
        stm32_library::stm32_peripherals::hstduart, &data, 1, 10
      );
    }
    return data;
  }
  return 0;
}
#endif

#endif // HAL_UART_MODULE_ENABLED
