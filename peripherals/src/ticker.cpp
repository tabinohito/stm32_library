#include "../Inc/ticker.hpp"

#ifdef HAL_TIM_MODULE_ENABLED
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) { stm32_library::stm32_peripherals::Ticker::tim_it(htim); }
#endif // HAL_TIM_MODULE_ENABLED
