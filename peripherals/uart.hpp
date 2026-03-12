#pragma once

#include "main.h"

#ifdef HAL_UART_MODULE_ENABLED

#include "misc/callback.hpp"
#include "misc/format.hpp"
#include <queue>
#include <string>

namespace abu2023::stm32_peripherals {
class Uart {
private:
  UART_HandleTypeDef *handle_;
  bool use_dma_transmit_ = false;
  std::string buffer_ = {};
  uint8_t *data_p_;
  int16_t data_size_;
  int16_t index_read_ = 0;

public:
  using CallbackFnType = void();

  Uart(UART_HandleTypeDef *handle) : handle_(handle) {}
  UART_HandleTypeDef* get_handle(){return handle_;}

  void use_dma_transmit(bool use_dma = true) {
    use_dma_transmit_ = use_dma;
  } // dma送信と通常の送信を混ぜると問題を起こす可能性がある

  HAL_StatusTypeDef write(uint8_t *data, uint16_t size, uint32_t timeout = 10) {
    if (use_dma_transmit_)
      return HAL_UART_Transmit_DMA(handle_, data, size);
    else
      return HAL_UART_Transmit(handle_, data, size, timeout);
  }
  HAL_StatusTypeDef write() {
    int buf_size = buffer_.length();
    if (buf_size > UINT16_MAX)
      buf_size = UINT16_MAX;

    if (handle_->gState == HAL_UART_STATE_READY) { // 送信
      static std::string send_str;
      send_str = buffer_.substr(0, buf_size);
      buffer_ = buffer_.substr(buf_size);
      return write((uint8_t *)(send_str.c_str()), buf_size);
    } else {
      return HAL_BUSY;
    }
  }

  template <class... Args> HAL_StatusTypeDef write(const char *fmt, Args... args) {
    if (use_dma_transmit_) {
      push_buffer(fmt, args...);
      return write();
    } else {
      std::vector<char> str = utility::format(fmt, args...);
      return write(reinterpret_cast<uint8_t *>(str.data()), static_cast<uint16_t>(str.size() - 1));
    }
  }

  template <class... Args> void push_buffer(const char *fmt, Args... args) {
    std::vector<char> buf = utility::format(fmt, args...);
    buffer_ += std::string(buf.begin(), buf.end() - 1);
  }

  HAL_StatusTypeDef read(void *buffer, size_t size, uint32_t time_out = 10) {
    HAL_StatusTypeDef ret = HAL_UART_Receive(handle_, (uint8_t *)buffer, size, time_out);
    if (ret != HAL_OK) {
      HAL_UART_Abort(handle_);
    }
    return ret;
  }

  void attach(std::function<CallbackFnType> &&fn, uint8_t priority = 100) {
    callback::attach(reinterpret_cast<intptr_t>(handle_), std::move(fn), priority);
  }

  void start_receive_dma(uint8_t *data_p, int data_size, bool is_dma_start_test = false) {
    data_p_ = data_p;
    data_size_ = data_size;
    HAL_UART_Receive_DMA(handle_, data_p, data_size);
    if (is_dma_start_test) {
      dma_receive_test(data_p, data_size);
    }
  }

  void dma_receive_test(uint8_t *data_p, size_t data_size) { // 1秒で勝手にタイムアウト
    size_t start_size = __HAL_DMA_GET_COUNTER(handle_->hdmarx);
    uint32_t start_ms = HAL_GetTick();
    // 1byteでも受信するまでエラーを見張る
    // 電源入ってからすぐデータ垂れ流してくるようなデバイス相手じゃないと使えない
    while (start_size == __HAL_DMA_GET_COUNTER(handle_->hdmarx) && (HAL_GetTick() - start_ms <= 1000)) {
      if (__HAL_UART_GET_FLAG(handle_, UART_FLAG_ORE) || __HAL_UART_GET_FLAG(handle_, UART_FLAG_NE) ||
          __HAL_UART_GET_FLAG(handle_, UART_FLAG_FE) || __HAL_UART_GET_FLAG(handle_, UART_FLAG_PE)) {
        HAL_UART_Abort(handle_);
        HAL_UART_Receive_DMA(handle_, data_p, data_size);
      }
    }
  }

  uint16_t dma_receive_data_num(){
    int16_t index = data_size_ - __HAL_DMA_GET_COUNTER(handle_->hdmarx);
    int16_t remain_data = index - index_read_;
    return (remain_data < 0) ? remain_data + data_size_ : remain_data;
  }

  uint8_t dma_receive_data(){
    uint8_t read_data = 0;

    uint8_t remain_data = dma_receive_data_num();
    if (remain_data > 0) {
      read_data = data_p_[index_read_];
      index_read_++;
      if (index_read_ >= data_size_) {
        index_read_ = 0;
      }
    }
    return read_data;
  };
};
} // namespace abu2023::stm32_peripherals

#endif // HAL_UART_MODULE_ENABLED
