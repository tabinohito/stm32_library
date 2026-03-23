#pragma once

#include "main.h"

#ifdef HAL_TIM_MODULE_ENABLED

namespace stm32_library::stm32_peripherals {
class PwmOut {
public:
  PwmOut(TIM_HandleTypeDef *handle, uint32_t ch) : handle_(handle), ch_(ch) {
#ifdef STM32_LIBRARY_USE_STATIC_INSTANCE
    HalStartupManager::add_startup_callback([this] { startup(); });
#else
    startup();
#endif
  }

  void write(float value) {
    // C++20: 値を 0.0 ~ 1.0 の範囲にクランプ
    value = std::clamp(value, 0.0f, 1.0f);
    
    // 【修正箇所】 CubeMxの初期値(Init.Period)ではなく、現在のレジスタ(ARR)から周期を直接読み取る
    uint32_t current_arr = handle_->Instance->ARR;
    uint32_t pulse = (uint32_t)((float)current_arr * value + 0.5f);
    __HAL_TIM_SET_COMPARE(handle_, ch_, pulse);
  }

  float read() {
    uint32_t current_arr = handle_->Instance->ARR;
    if (current_arr > 0) {
      float value = (float)(__HAL_TIM_GET_COMPARE(handle_, ch_)) / (float)current_arr;
      return std::clamp(value, 0.0f, 1.0f);
    }
    return 0.0f;
  }

  // クロックとレジスタから現在のPWM周波数(Hz)を読み取るメソッド
  uint32_t get_frequency() const {
    uint32_t base_clock = HAL_RCC_GetSysClockFreq(); 
    uint32_t psc = handle_->Instance->PSC;
    uint32_t arr = handle_->Instance->ARR;
    if (arr == 0) return 0;
    return base_clock / ((psc + 1) * (arr + 1));
  }

  // Toneクラス等からアクセスするためのゲッター
  TIM_HandleTypeDef* get_handle() const { return handle_; }
  uint32_t get_ch() const { return ch_; }

  PwmOut &operator=(float value) {
    write(value);
    return *this;
  }

  PwmOut &operator=(PwmOut &rhs) {
    write(rhs.read());
    return *this;
  }

  operator float() { return read(); }

protected:
  void startup() { HAL_TIM_PWM_Start(handle_, ch_); }

private:
  TIM_HandleTypeDef *handle_;
  uint32_t ch_;
};
} // namespace stm32_library::stm32_peripherals

#endif // HAL_TIM_MODULE_ENABLED
