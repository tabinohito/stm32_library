#pragma once

#include "main.h"

#ifdef HAL_ADC_MODULE_ENABLED

#include <cassert>
#include <map>
#include <vector>

namespace abu2023::stm32_peripherals {
class DmaAnalogIn {
public:
  static void attach_handle(ADC_HandleTypeDef *handle) {
    assert(handle->Init.NbrOfConversion > 0);

    if (handles_.find(handle) == handles_.end()) {
      handles_.insert(std::make_pair(handle, adc_handle_t{}));

      // resolution check
      uint16_t &max_val = handles_.at(handle).max_val;
      switch (handle->Init.Resolution) {
      case ADC_RESOLUTION_12B:
        max_val = 1UL << 12;
        break;
      case ADC_RESOLUTION_10B:
        max_val = 1UL << 10;
        break;
      case ADC_RESOLUTION_8B:
        max_val = 1UL << 8;
        break;
      case ADC_RESOLUTION_6B:
        max_val = 1UL << 6;
        break;
      }

      // get rank map
      auto &rank_map = handles_.at(handle).rank_map;
      rank_map = get_rank_map(handle, handle->Init.NbrOfConversion);

      // start DMA
      std::vector<uint16_t> &raw_data = handles_.at(handle).raw_data;
      raw_data.resize(handle->Init.NbrOfConversion);
      HAL_ADC_Start_DMA(handle, (uint32_t *)raw_data.data(), raw_data.size());
    }
  }

  DmaAnalogIn(ADC_HandleTypeDef *handle, uint32_t ch)
      : // ADC_CHANNEL_1
        handle_(handle) {
    assert(handles_.find(handle_) != handles_.end());
    auto &rank_map = handles_.at(handle_).rank_map;
    assert(rank_map.find(ch) != rank_map.end());

    index_ = rank_map.at(ch) - 1;
  }

  uint16_t read_raw() { return handles_.at(handle_).raw_data[index_]; }

  float read() { return (float)read_raw() / handles_.at(handle_).max_val; }

  operator float() { return read(); }

private:
  static std::map<size_t, size_t> get_rank_map(ADC_HandleTypeDef *handle, size_t map_size) {
    std::map<size_t, size_t> ret;
    volatile uint32_t *SQR_REG1 = &(handle->Instance->SQR1);

#define EXPAND_GET_RANK(rank)                                                                                          \
  if (rank <= map_size) {                                                                                              \
    size_t reg_num = (LL_ADC_REG_RANK_##rank & 0x700) >> 8;                                                            \
    size_t mask_pos = (LL_ADC_REG_RANK_##rank & 0x1F);                                                                 \
    size_t ch = (*(SQR_REG1 + reg_num) >> mask_pos) & 0x1F;                                                            \
    ret.insert(std::make_pair(ch, static_cast<size_t>(rank)));                                                         \
  }

#ifdef LL_ADC_REG_RANK_1
    EXPAND_GET_RANK(1)
#endif
#ifdef LL_ADC_REG_RANK_2
    EXPAND_GET_RANK(2)
#endif
#ifdef LL_ADC_REG_RANK_3
    EXPAND_GET_RANK(3)
#endif
#ifdef LL_ADC_REG_RANK_4
    EXPAND_GET_RANK(4)
#endif
#ifdef LL_ADC_REG_RANK_5
    EXPAND_GET_RANK(5)
#endif
#ifdef LL_ADC_REG_RANK_6
    EXPAND_GET_RANK(6)
#endif
#ifdef LL_ADC_REG_RANK_7
    EXPAND_GET_RANK(7)
#endif
#ifdef LL_ADC_REG_RANK_8
    EXPAND_GET_RANK(8)
#endif
#ifdef LL_ADC_REG_RANK_9
    EXPAND_GET_RANK(9)
#endif
#ifdef LL_ADC_REG_RANK_10
    EXPAND_GET_RANK(10)
#endif
#ifdef LL_ADC_REG_RANK_11
    EXPAND_GET_RANK(11)
#endif
#ifdef LL_ADC_REG_RANK_12
    EXPAND_GET_RANK(12)
#endif
#ifdef LL_ADC_REG_RANK_13
    EXPAND_GET_RANK(13)
#endif
#ifdef LL_ADC_REG_RANK_14
    EXPAND_GET_RANK(14)
#endif
#ifdef LL_ADC_REG_RANK_15
    EXPAND_GET_RANK(15)
#endif
#ifdef LL_ADC_REG_RANK_16
    EXPAND_GET_RANK(16)
#endif

#undef EXPAND_GET_RANK

    return ret;
  }

private:
  size_t index_;
  ADC_HandleTypeDef *handle_;

  struct adc_handle_t {
    std::vector<uint16_t> raw_data;
    std::map<size_t, size_t> rank_map;
    uint16_t max_val;
  };

  static inline std::map<ADC_HandleTypeDef *, adc_handle_t> handles_;
};

} // namespace abu2023::stm32_peripherals

#endif // HAL_ADC_MODULE_ENABLED
