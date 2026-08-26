#pragma once

#include "main.h"

#ifdef HAL_TIM_MODULE_ENABLED

#include "../../peripherals/Inc/pwm_out.hpp"

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
