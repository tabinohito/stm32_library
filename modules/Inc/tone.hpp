#pragma once

#include <stdio.h>
#include "../../peripherals/stm32_peripherals.hpp"

#ifdef HAL_TIM_MODULE_ENABLED

namespace stm32_library::stm32_modules
{
    class Tone {
    public:
    explicit Tone(stm32_library::stm32_peripherals::PwmOut& pwm) : pwm_(pwm) {}

    void play(uint32_t frequency);
    void stop();

    private:
    stm32_library::stm32_peripherals::PwmOut& pwm_;
    };

}

#endif /* HAL_TIM_MODULE_ENABLED */
