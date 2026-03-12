#pragma once

#include "main.h"

#ifdef HAL_ADC_MODULE_ENABLED

#include <cassert>
#include <map>
#include <vector>

namespace abu2023::stm32_peripherals {
class DmaSingleAnalogIn {
public:
  DmaSingleAnalogIn(ADC_HandleTypeDef *handle)
      : // ADC_CHANNEL_1
        handle_(handle) {
    switch (handle_->Init.Resolution) {
    case ADC_RESOLUTION_12B:
      max_val_ = 1UL << 12;
      break;
    case ADC_RESOLUTION_10B:
      max_val_ = 1UL << 10;
      break;
    case ADC_RESOLUTION_8B:
      max_val_ = 1UL << 8;
      break;
    case ADC_RESOLUTION_6B:
      max_val_ = 1UL << 6;
      break;
    }
    HAL_ADC_Start_DMA(handle_, (uint32_t *)raw_data_, 1);
  }

  uint16_t read_raw() { return raw_data_[0]; }

  float read() { return (float)raw_data_[0] / max_val_; }

  operator float() { return read(); }

private:
  ADC_HandleTypeDef *handle_;

  uint16_t raw_data_[1];
  uint16_t max_val_;
};

} // namespace abu2023::stm32_peripherals

#endif // HAL_ADC_MODULE_ENABLED
