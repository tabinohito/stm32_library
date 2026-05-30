#pragma once

#include "uart.hpp"

#include <cstddef>
#include <cstdint>
#include <climits>
#include <vector>

#if __cplusplus >= 202002L
#include <span>
#endif

// -----------------------------------------------------------------------------
// LL USART include
// -----------------------------------------------------------------------------
// If your project does not define one of these STM32 family macros, replace this
// block with the concrete LL USART include for your MCU, for example:
//   #include "stm32f7xx_ll_usart.h"
// -----------------------------------------------------------------------------
#if defined(STM32F0)
#include "stm32f0xx_ll_usart.h"
#elif defined(STM32F1)
#include "stm32f1xx_ll_usart.h"
#elif defined(STM32F2)
#include "stm32f2xx_ll_usart.h"
#elif defined(STM32F3)
#include "stm32f3xx_ll_usart.h"
#elif defined(STM32F4)
#include "stm32f4xx_ll_usart.h"
#elif defined(STM32F7)
#include "stm32f7xx_ll_usart.h"
#elif defined(STM32G0)
#include "stm32g0xx_ll_usart.h"
#elif defined(STM32G4)
#include "stm32g4xx_ll_usart.h"
#elif defined(STM32H7)
#include "stm32h7xx_ll_usart.h"
#elif defined(STM32L0)
#include "stm32l0xx_ll_usart.h"
#elif defined(STM32L1)
#include "stm32l1xx_ll_usart.h"
#elif defined(STM32L4)
#include "stm32l4xx_ll_usart.h"
#elif defined(STM32L5)
#include "stm32l5xx_ll_usart.h"
#elif defined(STM32U5)
#include "stm32u5xx_ll_usart.h"
#else
#error "Please include the correct stm32xxxx_ll_usart.h for your MCU before including uart_ll.hpp"
#endif

namespace stm32_library::stm32_peripherals {

namespace uart_ll_detail {

inline bool tx_ready(USART_TypeDef *instance) {
#if defined(STM32_LIBRARY_UART_LL_USE_TXE_TXFNF)
  return LL_USART_IsActiveFlag_TXE_TXFNF(instance);
#else
  return LL_USART_IsActiveFlag_TXE(instance);
#endif
}

inline void enable_tx_irq(USART_TypeDef *instance) {
#if defined(STM32_LIBRARY_UART_LL_USE_TXE_TXFNF)
  LL_USART_EnableIT_TXE_TXFNF(instance);
#else
  LL_USART_EnableIT_TXE(instance);
#endif
}

inline void disable_tx_irq(USART_TypeDef *instance) {
#if defined(STM32_LIBRARY_UART_LL_USE_TXE_TXFNF)
  LL_USART_DisableIT_TXE_TXFNF(instance);
#else
  LL_USART_DisableIT_TXE(instance);
#endif
}

inline bool is_tx_irq_enabled(USART_TypeDef *instance) {
#if defined(STM32_LIBRARY_UART_LL_USE_TXE_TXFNF)
  return LL_USART_IsEnabledIT_TXE_TXFNF(instance);
#else
  return LL_USART_IsEnabledIT_TXE(instance);
#endif
}

inline bool tc_active(USART_TypeDef *instance) {
  return LL_USART_IsActiveFlag_TC(instance);
}

inline bool tc_irq_enabled(USART_TypeDef *instance) {
  return LL_USART_IsEnabledIT_TC(instance);
}

inline void enable_tc_irq(USART_TypeDef *instance) {
  LL_USART_EnableIT_TC(instance);
}

inline void disable_tc_irq(USART_TypeDef *instance) {
  LL_USART_DisableIT_TC(instance);
}

inline void clear_tc(USART_TypeDef *instance) {
  LL_USART_ClearFlag_TC(instance);
}

} // namespace uart_ll_detail

// =============================================================================
// UartLl
// -----------------------------------------------------------------------------
// LL TX version of the existing Uart class.
//
// Design:
//   - Inherits existing Uart to keep HAL RX DMA/read/attach compatibility.
//   - Replaces the common write(data, size) path with LL non-blocking TX.
//   - Does not use HAL_UART_Transmit* for TX.
//   - RX can still use inherited HAL DMA functions.
//
// Important:
//   - If a call site stores this object as Uart* and the current Uart::write is
//     not virtual, calls through Uart* will use Uart's HAL write.
//   - For a "drop-in" replacement, instantiate/use this as UartLl<...> directly.
//   - Do not call Uart::use_dma_transmit(true) expecting it to affect LL TX.
// =============================================================================

template <size_t TxRingSize = 4096>
class UartLl : public Uart {
  static_assert(TxRingSize >= 2, "TxRingSize must be >= 2");
  static_assert((TxRingSize & (TxRingSize - 1)) == 0,
                "TxRingSize must be a power of two");

private:
  static constexpr size_t MASK = TxRingSize - 1;

  UART_HandleTypeDef *handle_ = nullptr;
  USART_TypeDef *instance_ = nullptr;

  bool use_tx_interrupt_ = true;

  uint8_t tx_ring_[TxRingSize] = {};

  volatile size_t tx_head_ = 0;
  volatile size_t tx_tail_ = 0;

  volatile uint32_t tx_drop_count_ = 0;
  volatile uint32_t tx_enqueue_count_ = 0;
  volatile uint32_t tx_sent_count_ = 0;

  volatile bool tx_idle_ = true;

public:
  using Uart::read;
  using Uart::attach;
  using Uart::start_receive_dma;
  using Uart::dma_receive_data_num;
  using Uart::dma_receive_data;

  explicit UartLl(UART_HandleTypeDef *handle, bool use_tx_interrupt = true)
      : Uart(handle),
        handle_(handle),
        instance_(handle ? handle->Instance : nullptr),
        use_tx_interrupt_(use_tx_interrupt) {}

  USART_TypeDef *get_instance() {
    return instance_;
  }

  const USART_TypeDef *get_instance() const {
    return instance_;
  }

  UART_HandleTypeDef *get_handle() {
    return handle_;
  }

  const UART_HandleTypeDef *get_handle() const {
    return handle_;
  }

  void set_use_tx_interrupt(bool enable) {
    use_tx_interrupt_ = enable;

    if (!enable && instance_ != nullptr) {
      uart_ll_detail::disable_tx_irq(instance_);
      uart_ll_detail::disable_tc_irq(instance_);
    }
  }

  bool use_tx_interrupt() const {
    return use_tx_interrupt_;
  }

  HAL_StatusTypeDef write(const uint8_t *data,
                          uint16_t size,
                          uint32_t timeout = 0) {
    (void)timeout;

    if (instance_ == nullptr || data == nullptr || size == 0) {
      return HAL_ERROR;
    }

    const size_t pushed = enqueue(data, size);

    if (use_tx_interrupt_) {
      kick_tx_irq();
    } else {
      poll();
    }

    if (pushed != size) {
      tx_drop_count_ += static_cast<uint32_t>(size - pushed);
      return HAL_BUSY;
    }

    return HAL_OK;
  }

  HAL_StatusTypeDef write(uint8_t *data,
                          uint16_t size,
                          uint32_t timeout = 0) {
    return write(static_cast<const uint8_t *>(data), size, timeout);
  }

#if __cplusplus >= 202002L
  HAL_StatusTypeDef write(std::span<const uint8_t> data,
                          uint32_t timeout = 0) {
    if (data.size() > UINT16_MAX) {
      return HAL_ERROR;
    }

    return write(data.data(), static_cast<uint16_t>(data.size()), timeout);
  }
#endif

  HAL_StatusTypeDef write_byte(uint8_t byte, uint32_t timeout = 0) {
    return write(&byte, 1, timeout);
  }

  template <class... Args>
  HAL_StatusTypeDef write(const char *fmt, Args... args) {
    std::vector<char> str = utility::format(fmt, args...);
    if (str.empty()) {
      return HAL_ERROR;
    }

    const size_t payload_size = str.size() - 1;
    if (payload_size > UINT16_MAX) {
      return HAL_ERROR;
    }

    return write(reinterpret_cast<const uint8_t *>(str.data()),
                 static_cast<uint16_t>(payload_size));
  }

  size_t write_available(const uint8_t *data, size_t size) {
    if (instance_ == nullptr || data == nullptr || size == 0) {
      return 0;
    }

    const size_t pushed = enqueue(data, size);

    if (pushed != size) {
      tx_drop_count_ += static_cast<uint32_t>(size - pushed);
    }

    if (use_tx_interrupt_) {
      kick_tx_irq();
    } else {
      poll();
    }

    return pushed;
  }

  void poll() {
    if (instance_ == nullptr) {
      return;
    }

    drain_to_hardware();

    if (empty()) {
      uart_ll_detail::disable_tx_irq(instance_);
    }
  }

  void irq_handler() {
    if (instance_ == nullptr) {
      return;
    }

    if (uart_ll_detail::is_tx_irq_enabled(instance_) &&
        uart_ll_detail::tx_ready(instance_)) {
      drain_to_hardware();

      if (empty()) {
        uart_ll_detail::disable_tx_irq(instance_);
        uart_ll_detail::enable_tc_irq(instance_);
      }
    }

    if (uart_ll_detail::tc_irq_enabled(instance_) &&
        uart_ll_detail::tc_active(instance_)) {
      uart_ll_detail::clear_tc(instance_);
      uart_ll_detail::disable_tc_irq(instance_);
      tx_idle_ = true;
    }
  }

  void irq_handler_with_hal_rx() {
    irq_handler();

    if (handle_ != nullptr) {
      HAL_UART_IRQHandler(handle_);
    }
  }

  uint32_t tx_drop_count() const {
    return tx_drop_count_;
  }

  uint32_t tx_enqueue_count() const {
    return tx_enqueue_count_;
  }

  uint32_t tx_sent_count() const {
    return tx_sent_count_;
  }

  uint32_t tx_queue_size() const {
    return static_cast<uint32_t>(queue_size_unsafe());
  }

  uint32_t tx_queue_capacity() const {
    return static_cast<uint32_t>(TxRingSize - 1);
  }

  bool tx_idle() const {
    return tx_idle_;
  }

  bool empty() const {
    return tx_head_ == tx_tail_;
  }

  bool full() const {
    return next_index(tx_head_) == tx_tail_;
  }

  void clear_tx_queue() {
    const uint32_t primask = enter_critical();
    tx_head_ = 0;
    tx_tail_ = 0;
    restore_critical(primask);
  }

  void reset_counters() {
    tx_drop_count_ = 0;
    tx_enqueue_count_ = 0;
    tx_sent_count_ = 0;
  }

private:
  static size_t next_index(size_t value) {
    return (value + 1U) & MASK;
  }

  size_t queue_size_unsafe() const {
    return (tx_head_ - tx_tail_) & MASK;
  }

  size_t enqueue(const uint8_t *data, size_t size) {
    size_t pushed = 0;

    const uint32_t primask = enter_critical();

    while (pushed < size) {
      const size_t next = next_index(tx_head_);

      if (next == tx_tail_) {
        break;
      }

      tx_ring_[tx_head_] = data[pushed];
      tx_head_ = next;
      pushed++;
    }

    tx_enqueue_count_ += static_cast<uint32_t>(pushed);

    restore_critical(primask);
    return pushed;
  }

  bool pop(uint8_t &byte) {
    if (tx_tail_ == tx_head_) {
      return false;
    }

    byte = tx_ring_[tx_tail_];
    tx_tail_ = next_index(tx_tail_);
    return true;
  }

  void drain_to_hardware() {
    while (!empty() && uart_ll_detail::tx_ready(instance_)) {
      uint8_t byte = 0;

      if (!pop(byte)) {
        break;
      }

      LL_USART_TransmitData8(instance_, byte);
      tx_sent_count_++;
      tx_idle_ = false;
    }
  }

  void kick_tx_irq() {
    if (instance_ == nullptr) {
      return;
    }

    tx_idle_ = false;
    uart_ll_detail::enable_tx_irq(instance_);
  }

  static uint32_t enter_critical() {
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
  }

  static void restore_critical(uint32_t primask) {
    if (primask == 0U) {
      __enable_irq();
    }
  }
};

} // namespace stm32_library::stm32_peripherals
