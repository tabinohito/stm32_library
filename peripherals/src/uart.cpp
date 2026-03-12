#include "../uart.hpp"

#ifdef HAL_UART_MODULE_ENABLED
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  using namespace abu2023::stm32_peripherals;
  callback::callback<Uart::CallbackFnType>(reinterpret_cast<intptr_t>(huart));
}
#endif
