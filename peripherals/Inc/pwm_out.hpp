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
    if (value < 0)
      value = 0;
    if (value > 1)
      value = 1;
    uint32_t pulse = (uint32_t)((float)handle_->Init.Period * value + 0.5);
    __HAL_TIM_SET_COMPARE(handle_, ch_, pulse);
  }

  float read() {
    float value = 0;
    if (handle_->Init.Period > 0) {
      value = (float)(__HAL_TIM_GET_COMPARE(handle_, ch_)) / (float)(handle_->Init.Period);
    }
    return ((value > (float)1.0) ? (float)(1.0) : (value));
  }

  void period(float seconds) { period_us(1000000 * seconds); } // あとでなおす 16bitだときびしい(はいりきらんかも)
  void period_ms(int ms) { period_us(1000 * ms); }
  void period_us(int us) { __HAL_TIM_SetCompare(handle_, ch_, us); } // あとでなおす(1us / cnt と仮定)
  // int read_period_us();

  // void pulsewidth(float seconds);
  // void pulsewidth_ms(int ms);
  // void pulsewidth_us(int us);
  // int read_pulsewitdth_us();

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
