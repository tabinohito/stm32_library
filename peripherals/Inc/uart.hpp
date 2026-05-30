#pragma once

#include "main.h"

#ifdef HAL_UART_MODULE_ENABLED

#include "../misc/callback.hpp"
#include "../misc/format.hpp"

#include <climits>
#include <cstdint>
#include <functional>
#include <queue>
#include <string>
#include <vector>

#if __cplusplus >= 202002L
#include <span>
#endif

namespace stm32_library::stm32_peripherals {

class Uart {
private:
  UART_HandleTypeDef *handle_ = nullptr;

  bool use_dma_transmit_ = false;

  std::string buffer_ = {};

  uint8_t *data_p_ = nullptr;
  int16_t data_size_ = 0;
  int16_t index_read_ = 0;

public:
  using CallbackFnType = void();

  explicit Uart(UART_HandleTypeDef *handle) : handle_(handle) {}

  virtual ~Uart() = default;

  virtual UART_HandleTypeDef *get_handle() {
    return handle_;
  }

  virtual const UART_HandleTypeDef *get_handle() const {
    return handle_;
  }

  virtual void use_dma_transmit(bool use_dma = true) {
    use_dma_transmit_ = use_dma;
  }

  virtual bool is_dma_transmit_enabled() const {
    return use_dma_transmit_;
  }

  // ---------------------------------------------------------------------------
  // TX
  // ---------------------------------------------------------------------------

  // New const-correct virtual root API.
  // HAL APIs are not const-correct, so const_cast is used only at the HAL boundary.
  virtual HAL_StatusTypeDef write(
      const uint8_t *data,
      uint16_t size,
      uint32_t timeout = 10) {
    if (handle_ == nullptr || data == nullptr || size == 0) {
      return HAL_ERROR;
    }

    if (use_dma_transmit_) {
      return HAL_UART_Transmit_DMA(
          handle_,
          const_cast<uint8_t *>(data),
          size);
    }

    return HAL_UART_Transmit(
        handle_,
        const_cast<uint8_t *>(data),
        size,
        timeout);
  }

  // Existing-compatible API.
  virtual HAL_StatusTypeDef write(
      uint8_t *data,
      uint16_t size,
      uint32_t timeout = 10) {
    return write(static_cast<const uint8_t *>(data), size, timeout);
  }

#if __cplusplus >= 202002L
  virtual HAL_StatusTypeDef write(
      std::span<const uint8_t> data,
      uint32_t timeout = 10) {
    if (data.size() > UINT16_MAX) {
      return HAL_ERROR;
    }

    return write(
        data.data(),
        static_cast<uint16_t>(data.size()),
        timeout);
  }
#endif

  virtual HAL_StatusTypeDef write_byte(uint8_t byte, uint32_t timeout = 10) {
    return write(&byte, 1, timeout);
  }

  // Flush string buffer when DMA transmit mode is used.
  virtual HAL_StatusTypeDef write() {
    if (handle_ == nullptr) {
      return HAL_ERROR;
    }

    int buf_size = static_cast<int>(buffer_.length());
    if (buf_size > UINT16_MAX) {
      buf_size = UINT16_MAX;
    }

    if (buf_size <= 0) {
      return HAL_OK;
    }

    if (handle_->gState == HAL_UART_STATE_READY) {
      // NOTE:
      // Keep static string to preserve buffer memory until HAL starts transfer.
      // This follows the original implementation behavior.
      static std::string send_str;
      send_str = buffer_.substr(0, static_cast<size_t>(buf_size));
      buffer_ = buffer_.substr(static_cast<size_t>(buf_size));

      return write(
          reinterpret_cast<const uint8_t *>(send_str.c_str()),
          static_cast<uint16_t>(buf_size));
    }

    return HAL_BUSY;
  }

  // Template functions cannot be virtual.
  // Derived classes should reimplement this if formatted writes must use
  // a different TX backend.
  template <class... Args>
  HAL_StatusTypeDef write(const char *fmt, Args... args) {
    if (use_dma_transmit_) {
      push_buffer(fmt, args...);
      return write();
    }

    std::vector<char> str = utility::format(fmt, args...);
    if (str.empty()) {
      return HAL_ERROR;
    }

    return write(
        reinterpret_cast<const uint8_t *>(str.data()),
        static_cast<uint16_t>(str.size() - 1));
  }

  template <class... Args>
  void push_buffer(const char *fmt, Args... args) {
    std::vector<char> buf = utility::format(fmt, args...);
    if (buf.empty()) {
      return;
    }

    buffer_ += std::string(buf.begin(), buf.end() - 1);
  }

  virtual size_t pending_buffer_size() const {
    return buffer_.size();
  }

  virtual void clear_buffer() {
    buffer_.clear();
  }

  // ---------------------------------------------------------------------------
  // Blocking RX
  // ---------------------------------------------------------------------------

  virtual HAL_StatusTypeDef read(
      void *buffer,
      size_t size,
      uint32_t time_out = 10) {
    if (handle_ == nullptr || buffer == nullptr || size == 0) {
      return HAL_ERROR;
    }

    if (size > UINT16_MAX) {
      return HAL_ERROR;
    }

    HAL_StatusTypeDef ret = HAL_UART_Receive(
        handle_,
        static_cast<uint8_t *>(buffer),
        static_cast<uint16_t>(size),
        time_out);

    if (ret != HAL_OK) {
      HAL_UART_Abort(handle_);
    }

    return ret;
  }

  // ---------------------------------------------------------------------------
  // Callback
  // ---------------------------------------------------------------------------

  virtual void attach(
      std::function<CallbackFnType> &&fn,
      uint8_t priority = 100) {
    callback::attach(
        reinterpret_cast<intptr_t>(handle_),
        std::move(fn),
        priority);
  }

  // ---------------------------------------------------------------------------
  // RX DMA
  // ---------------------------------------------------------------------------

  // New virtual root API.
  virtual HAL_StatusTypeDef start_receive_dma(
      uint8_t *data_p,
      uint16_t data_size) {
    if (handle_ == nullptr || data_p == nullptr || data_size == 0) {
      return HAL_ERROR;
    }

    data_p_ = data_p;
    data_size_ = static_cast<int16_t>(data_size);
    index_read_ = 0;

    return HAL_UART_Receive_DMA(handle_, data_p, data_size);
  }

  // Existing-compatible API.
  virtual void start_receive_dma(
      uint8_t *data_p,
      int data_size,
      bool is_dma_start_test = false) {
    if (data_size <= 0 || data_size > UINT16_MAX) {
      return;
    }

    (void)start_receive_dma(
        data_p,
        static_cast<uint16_t>(data_size));

    if (is_dma_start_test) {
      dma_receive_test(data_p, static_cast<size_t>(data_size));
    }
  }

  virtual HAL_StatusTypeDef stop_receive_dma() {
    if (handle_ == nullptr) {
      return HAL_ERROR;
    }

    return HAL_UART_DMAStop(handle_);
  }

  virtual void dma_receive_test(
      uint8_t *data_p,
      size_t data_size) {
    if (handle_ == nullptr ||
        handle_->hdmarx == nullptr ||
        data_p == nullptr ||
        data_size == 0 ||
        data_size > UINT16_MAX) {
      return;
    }

    size_t start_size = __HAL_DMA_GET_COUNTER(handle_->hdmarx);
    uint32_t start_ms = HAL_GetTick();

    // 1秒で勝手にタイムアウト
    // 1byteでも受信するまでエラーを見張る
    // 電源投入直後からデータを垂れ流すようなデバイス相手でないと使えない
    while (
        start_size == __HAL_DMA_GET_COUNTER(handle_->hdmarx) &&
        (HAL_GetTick() - start_ms <= 1000)) {
      if (__HAL_UART_GET_FLAG(handle_, UART_FLAG_ORE) ||
          __HAL_UART_GET_FLAG(handle_, UART_FLAG_NE) ||
          __HAL_UART_GET_FLAG(handle_, UART_FLAG_FE) ||
          __HAL_UART_GET_FLAG(handle_, UART_FLAG_PE)) {
        HAL_UART_Abort(handle_);
        HAL_UART_Receive_DMA(
            handle_,
            data_p,
            static_cast<uint16_t>(data_size));
      }
    }
  }

  virtual uint16_t dma_receive_data_num() {
    if (handle_ == nullptr ||
        handle_->hdmarx == nullptr ||
        data_p_ == nullptr ||
        data_size_ <= 0) {
      return 0;
    }

    int16_t index = static_cast<int16_t>(
        data_size_ -
        static_cast<int16_t>(__HAL_DMA_GET_COUNTER(handle_->hdmarx)));

    int16_t remain_data = static_cast<int16_t>(index - index_read_);

    return static_cast<uint16_t>(
        (remain_data < 0) ? remain_data + data_size_ : remain_data);
  }

  virtual uint8_t dma_receive_data() {
    uint8_t read_data = 0;

    uint16_t remain_data = dma_receive_data_num();
    if (remain_data > 0) {
      read_data = data_p_[index_read_];

      index_read_++;
      if (index_read_ >= data_size_) {
        index_read_ = 0;
      }
    }

    return read_data;
  }
};

} // namespace stm32_library::stm32_peripherals

#endif // HAL_UART_MODULE_ENABLED
