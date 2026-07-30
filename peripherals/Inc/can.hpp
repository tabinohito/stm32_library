#pragma once

#include "main.h"

#if defined(HAL_CAN_MODULE_ENABLED) || defined(HAL_FDCAN_MODULE_ENABLED)

#include <algorithm>
#include <cstring>
#include <forward_list>
#include <functional>
#include <ranges>
#include <span>
#include <unordered_map>

#include "../misc/callback.hpp"

namespace stm32_library::stm32_peripherals {
static constexpr uint32_t CanSffMask = 0x000007FFU;
static constexpr uint32_t CanEffMask = 0x1FFFFFFFU;
static constexpr uint32_t CanEffFlag = 0x80000000U;

struct CanMessage {
  uint32_t id;
  std::array<uint8_t, 8> data;
  uint32_t size;

  CanMessage() = default;

  CanMessage(uint32_t id, uint32_t size) : id(id), size(size > 8 ? 8 : size) {}

  CanMessage(uint32_t id, std::array<uint8_t, 8> &&data, uint32_t size)
      : id(id), data(std::move(data)), size(size > 8 ? 8 : size) {}

  CanMessage(uint32_t id, uint8_t *data_ptr, uint32_t size) : CanMessage(id, size) {
    std::memcpy(data.data(), data_ptr, size);
  }

  template <class T>
  requires(sizeof(T) <= 8) CanMessage(uint32_t id, const T &data_val) : id(id), size(sizeof(T)) {
    std::memcpy(data.data(), &data_val, sizeof(T));
  }
};

class Can {
private:
#if defined(HAL_CAN_MODULE_ENABLED)
  using CanHandleType = CAN_HandleTypeDef;
#elif defined(HAL_FDCAN_MODULE_ENABLED)
  using CanHandleType = FDCAN_HandleTypeDef;
#endif
  CanHandleType *handle_;

public:
  using CallbackFnType = void(const CanMessage &msg);

  // CAN初期化
  Can(CanHandleType *handle, uint32_t filter_id, uint32_t filter_mask);

  Can(CanHandleType *handle) : Can(handle, 0, 0) {}

#if defined(HAL_CAN_MODULE_ENABLED)
  struct BitTimingConfig {
    uint32_t prescaler = 1;
    uint32_t mode = CAN_MODE_NORMAL;
    uint32_t sync_jump_width = CAN_SJW_1TQ;
    uint32_t time_segment1 = CAN_BS1_1TQ;
    uint32_t time_segment2 = CAN_BS2_1TQ;
    FunctionalState time_triggered_mode = DISABLE;
    FunctionalState auto_bus_off = DISABLE;
    FunctionalState auto_wakeup = DISABLE;
    FunctionalState auto_retransmission = ENABLE;
    FunctionalState receive_fifo_locked = DISABLE;
    FunctionalState transmit_fifo_priority = DISABLE;
    uint32_t filter_id = 0;
    uint32_t filter_mask = 0;
  };

  HAL_StatusTypeDef start(uint32_t filter_id = 0, uint32_t filter_mask = 0);
  HAL_StatusTypeDef stop();
  HAL_StatusTypeDef configure(const BitTimingConfig& config);
  HAL_StatusTypeDef restart(uint32_t mode, uint32_t filter_id = 0, uint32_t filter_mask = 0);
#endif

  // CAN送信
  HAL_StatusTypeDef write(uint32_t id, uint8_t *data, uint32_t size, bool blocking = false);

  HAL_StatusTypeDef write(CanMessage &msg, bool blocking = false) {
    return write(msg.id, msg.data.data(), msg.size, blocking);
  }

  // コールバック関数を登録
  void attach(std::function<CallbackFnType> &&fn, uint8_t priority = 100) {
    callback::attach(reinterpret_cast<intptr_t>(handle_), std::move(fn), priority);
  }
};
} // namespace stm32_library::stm32_peripherals
#endif
