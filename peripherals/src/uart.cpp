#include "../Inc/uart.hpp"

#ifdef HAL_UART_MODULE_ENABLED
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  using namespace stm32_library::stm32_peripherals;
  callback::callback<Uart::CallbackFnType>(reinterpret_cast<intptr_t>(huart));
}
#endif
