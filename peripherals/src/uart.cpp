#include "../Inc/uart.hpp"

#ifdef HAL_UART_MODULE_ENABLED

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  using namespace stm32_library::stm32_peripherals;
  callback::callback<Uart::CallbackFnType>(reinterpret_cast<intptr_t>(huart));
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  using namespace stm32_library::stm32_peripherals;
  Uart::tx_complete_callback(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  using namespace stm32_library::stm32_peripherals;
  Uart::error_callback(huart);
}

#endif
