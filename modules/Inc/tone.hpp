#pragma once

#include <stdio.h>
#include "../../peripherals/stm32_peripherals.hpp"

#ifdef HAL_TIM_MODULE_ENABLED

namespace stm32_library::stm32_modules
{
    class Tone {
    public:
    explicit Tone(stm32_library::stm32_peripherals::PwmOut& pwm) : pwm_(pwm) {}

    void play(uint32_t frequency) {
        if (frequency == 0) {
        stop();
        return;
        }

        TIM_HandleTypeDef* htim = pwm_.get_handle();
        
        // 1. MCUのベースクロックをHAL（実質的にRCCレジスタ）から動的に取得
        uint32_t base_clock = HAL_RCC_GetSysClockFreq();

        // 2. 目標周波数に合わせて Prescaler (PSC) と Auto-Reload (ARR) を計算
        // ARRが16bitの最大値(65535)に収まるようにPSCを逆算する
        // オーバーフロー防止のため uint64_t でキャストして計算
        uint32_t psc = base_clock / (static_cast<uint64_t>(frequency) * 65536);
        uint32_t arr = (base_clock / ((psc + 1) * frequency)) - 1;

        // 3. レジスタへ直接書き込み（CubeMXの設定を上書き）
        htim->Instance->PSC = psc;
        htim->Instance->ARR = arr;

        // 4. Update Generation (UG) ビットを立てて、変更したPSCとARRを即時反映させる
        htim->Instance->EGR = TIM_EGR_UG;

        // 5. Duty比 50% を設定して音を鳴らす
        pwm_.write(0.5f);
    }

    void stop() {
        // Duty比を 0% にして音を止める
        pwm_.write(0.0f);
    }

    private:
    stm32_library::stm32_peripherals::PwmOut& pwm_;
    };

}

#endif /* HAL_TIM_MODULE_ENABLED */
