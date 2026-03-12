#pragma once

#include "main.h"

#ifdef HAL_GPIO_MODULE_ENABLED

#include <forward_list>
#include <functional>
#include <unordered_map>

#include "../misc/callback.hpp"

namespace stm32_library::stm32_peripherals {
class DigitalIn {
private:
  GPIO_TypeDef *port_;
  uint16_t pin_;

public:
  using CallbackFnType = void();

  DigitalIn(GPIO_TypeDef *port, uint16_t pin) : port_(port), pin_(pin) {}

  int read() { return HAL_GPIO_ReadPin(port_, pin_); }
  operator int() { return read(); }

  void attach(std::function<CallbackFnType> &&fn, uint8_t priority = 100) {
    callback::attach(pin_, std::move(fn), priority);
  }
};
} // namespace stm32_library::stm32_peripherals
#endif // HAL_GPIO_MODULE_ENABLED
