#include "../Inc/ticker.hpp"

#ifdef HAL_TIM_MODULE_ENABLED
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) { stm32_library::stm32_peripherals::Ticker::tim_it(htim); }

extern "C" uint8_t stm32_library_ticker_irq_handler(TIM_HandleTypeDef *htim) {
  return stm32_library::stm32_peripherals::Ticker::irq_handler(htim) ? 1U : 0U;
}
#endif // HAL_TIM_MODULE_ENABLED
