#pragma once

#include "uart.hpp"

#include <cstddef>
#include <cstdint>
#include <climits>
#include <vector>

#if __cplusplus >= 202002L
#include <span>
#endif

#ifdef __cplusplus
extern "C" {
#endif

void uart_ll_irq_callback_c(UART_HandleTypeDef *huart);
void uart_ll_irq_callback_with_hal_rx_c(UART_HandleTypeDef *huart);
void uart_ll_dma_tx_complete_callback_c(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

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

inline void add_u32(volatile uint32_t &value, uint32_t delta) {
  value = static_cast<uint32_t>(value + delta);
}

inline void inc_u32(volatile uint32_t &value) {
  value = static_cast<uint32_t>(value + 1U);
}

inline void clear_uart_error_flags(USART_TypeDef *instance) {
#if defined(USART_ISR_ORE) || defined(USART_SR_ORE)
  if (LL_USART_IsActiveFlag_ORE(instance)) {
    LL_USART_ClearFlag_ORE(instance);
  }
#endif

#if defined(USART_ISR_FE) || defined(USART_SR_FE)
  if (LL_USART_IsActiveFlag_FE(instance)) {
    LL_USART_ClearFlag_FE(instance);
  }
#endif

#if defined(USART_ISR_NE) || defined(USART_SR_NE)
  if (LL_USART_IsActiveFlag_NE(instance)) {
    LL_USART_ClearFlag_NE(instance);
  }
#endif

#if defined(USART_ISR_PE) || defined(USART_SR_PE)
  if (LL_USART_IsActiveFlag_PE(instance)) {
    LL_USART_ClearFlag_PE(instance);
  }
#endif
}

class IrqTarget {
public:
  virtual ~IrqTarget() = default;
  virtual USART_TypeDef *irq_instance() = 0;
  virtual void irq_handler() = 0;
  virtual void irq_handler_with_hal_rx() = 0;
  virtual void dma_tx_complete_handler() = 0;
};

inline IrqTarget **irq_targets() {
  static IrqTarget *targets[16] = {};
  return targets;
}

inline constexpr size_t irq_target_capacity() {
  return 16;
}

inline bool register_irq_target(USART_TypeDef *instance, IrqTarget *target) {
  if (instance == nullptr || target == nullptr) {
    return false;
  }

  IrqTarget **targets = irq_targets();

  for (size_t i = 0; i < irq_target_capacity(); i++) {
    if (targets[i] != nullptr && targets[i]->irq_instance() == instance) {
      targets[i] = target;
      return true;
    }
  }

  for (size_t i = 0; i < irq_target_capacity(); i++) {
    if (targets[i] == nullptr) {
      targets[i] = target;
      return true;
    }
  }

  return false;
}

inline void unregister_irq_target(USART_TypeDef *instance, IrqTarget *target) {
  if (instance == nullptr || target == nullptr) {
    return;
  }

  IrqTarget **targets = irq_targets();

  for (size_t i = 0; i < irq_target_capacity(); i++) {
    if (targets[i] == target && targets[i]->irq_instance() == instance) {
      targets[i] = nullptr;
      return;
    }
  }
}

inline bool dispatch_irq(USART_TypeDef *instance) {
  IrqTarget **targets = irq_targets();

  for (size_t i = 0; i < irq_target_capacity(); i++) {
    if (targets[i] != nullptr && targets[i]->irq_instance() == instance) {
      targets[i]->irq_handler();
      return true;
    }
  }

  return false;
}

inline bool dispatch_irq_with_hal_rx(USART_TypeDef *instance) {
  IrqTarget **targets = irq_targets();

  for (size_t i = 0; i < irq_target_capacity(); i++) {
    if (targets[i] != nullptr && targets[i]->irq_instance() == instance) {
      targets[i]->irq_handler_with_hal_rx();
      return true;
    }
  }

  return false;
}

inline bool dispatch_dma_tx_complete(UART_HandleTypeDef *huart) {
  if (huart == nullptr) {
    return false;
  }

  IrqTarget **targets = irq_targets();

  for (size_t i = 0; i < irq_target_capacity(); i++) {
    if (targets[i] != nullptr &&
        targets[i]->irq_instance() == huart->Instance) {
      targets[i]->dma_tx_complete_handler();
      return true;
    }
  }

  return false;
}

inline bool irq_number_from_instance(USART_TypeDef *instance, IRQn_Type &irqn) {
#if defined(USART1) && defined(USART1_IRQn)
  if (instance == USART1) {
    irqn = USART1_IRQn;
    return true;
  }
#endif
#if defined(USART2) && defined(USART2_IRQn)
  if (instance == USART2) {
    irqn = USART2_IRQn;
    return true;
  }
#endif
#if defined(USART3) && defined(USART3_IRQn)
  if (instance == USART3) {
    irqn = USART3_IRQn;
    return true;
  }
#endif
#if defined(UART4) && defined(UART4_IRQn)
  if (instance == UART4) {
    irqn = UART4_IRQn;
    return true;
  }
#endif
#if defined(UART5) && defined(UART5_IRQn)
  if (instance == UART5) {
    irqn = UART5_IRQn;
    return true;
  }
#endif
#if defined(USART6) && defined(USART6_IRQn)
  if (instance == USART6) {
    irqn = USART6_IRQn;
    return true;
  }
#endif
#if defined(UART7) && defined(UART7_IRQn)
  if (instance == UART7) {
    irqn = UART7_IRQn;
    return true;
  }
#endif
#if defined(UART8) && defined(UART8_IRQn)
  if (instance == UART8) {
    irqn = UART8_IRQn;
    return true;
  }
#endif
#if defined(UART9) && defined(UART9_IRQn)
  if (instance == UART9) {
    irqn = UART9_IRQn;
    return true;
  }
#endif
#if defined(USART10) && defined(USART10_IRQn)
  if (instance == USART10) {
    irqn = USART10_IRQn;
    return true;
  }
#endif

  (void)instance;
  return false;
}

} // namespace uart_ll_detail

inline bool uart_ll_irq_handler(USART_TypeDef *instance) {
  return uart_ll_detail::dispatch_irq(instance);
}

inline bool uart_ll_irq_handler_with_hal_rx(USART_TypeDef *instance) {
  return uart_ll_detail::dispatch_irq_with_hal_rx(instance);
}

inline bool uart_ll_dma_tx_complete_handler(UART_HandleTypeDef *huart) {
  return uart_ll_detail::dispatch_dma_tx_complete(huart);
}

template <size_t TxRingSize = 4096>
class UartLl : public Uart, public uart_ll_detail::IrqTarget {
  static_assert(TxRingSize >= 2, "TxRingSize must be >= 2");
  static_assert((TxRingSize & (TxRingSize - 1)) == 0,
                "TxRingSize must be a power of two");

private:
  static constexpr size_t MASK = TxRingSize - 1;

  UART_HandleTypeDef *handle_ = nullptr;
  USART_TypeDef *instance_ = nullptr;

  bool use_tx_interrupt_ = false;
  bool use_tx_dma_ = false;
  bool registered_irq_target_ = false;

  uint8_t tx_ring_[TxRingSize] = {};

  volatile size_t tx_head_ = 0;
  volatile size_t tx_tail_ = 0;

  volatile uint32_t tx_drop_count_ = 0;
  volatile uint32_t tx_enqueue_count_ = 0;
  volatile uint32_t tx_sent_count_ = 0;

  volatile bool tx_idle_ = true;

  volatile uint32_t irq_count_ = 0;
  volatile uint32_t txe_irq_count_ = 0;
  volatile uint32_t tc_irq_count_ = 0;
  volatile uint32_t uart_error_irq_count_ = 0;

  volatile bool dma_tx_active_ = false;
  volatile uint16_t dma_tx_active_size_ = 0;

  volatile uint32_t dma_tx_start_count_ = 0;
  volatile uint32_t dma_tx_complete_count_ = 0;
  volatile uint32_t dma_tx_error_count_ = 0;

public:
  using Uart::read;
  using Uart::attach;
  using Uart::start_receive_dma;
  using Uart::dma_receive_data_num;
  using Uart::dma_receive_data;

  explicit UartLl(UART_HandleTypeDef *handle, bool use_tx_interrupt = false)
      : Uart(handle),
        handle_(handle),
        instance_(handle ? handle->Instance : nullptr),
        use_tx_interrupt_(use_tx_interrupt),
        use_tx_dma_(false) {
    registered_irq_target_ =
        uart_ll_detail::register_irq_target(instance_, this);
  }

  ~UartLl() override {
    if (registered_irq_target_) {
      uart_ll_detail::unregister_irq_target(instance_, this);
    }
  }

  USART_TypeDef *get_instance() {
    return instance_;
  }

  const USART_TypeDef *get_instance() const {
    return instance_;
  }

  UART_HandleTypeDef *get_handle() override {
    return handle_;
  }

  const UART_HandleTypeDef *get_handle() const override {
    return handle_;
  }

  USART_TypeDef *irq_instance() override {
    return instance_;
  }

  void set_use_tx_interrupt(bool enable) {
    use_tx_interrupt_ = enable;

    if (enable) {
      use_tx_dma_ = false;
    }

    if (instance_ != nullptr) {
      uart_ll_detail::disable_tc_irq(instance_);
      if (!enable) {
        uart_ll_detail::disable_tx_irq(instance_);
      }
    }
  }

  bool use_tx_interrupt() const {
    return use_tx_interrupt_;
  }

  void set_use_tx_dma(bool enable) {
    use_tx_dma_ = enable;

    if (enable) {
      use_tx_interrupt_ = false;
    }

    if (instance_ != nullptr) {
      uart_ll_detail::disable_tx_irq(instance_);
      uart_ll_detail::disable_tc_irq(instance_);
    }

    if (enable) {
      kick_tx_dma();
    }
  }

  bool use_tx_dma() const {
    return use_tx_dma_;
  }

  bool is_irq_registered() const {
    return registered_irq_target_;
  }

  bool enable_nvic(uint32_t preempt_priority = 5, uint32_t sub_priority = 0) {
    if (instance_ == nullptr) {
      return false;
    }

    IRQn_Type irqn{};
    if (!uart_ll_detail::irq_number_from_instance(instance_, irqn)) {
      return false;
    }

    HAL_NVIC_SetPriority(irqn, preempt_priority, sub_priority);
    HAL_NVIC_EnableIRQ(irqn);
    return true;
  }

  HAL_StatusTypeDef write(const uint8_t *data,
                          uint16_t size,
                          uint32_t timeout = 0) override {
    (void)timeout;

    if (instance_ == nullptr || data == nullptr || size == 0) {
      return HAL_ERROR;
    }

    const size_t pushed = enqueue(data, size);

    kick_tx();

    if (pushed != size) {
      uart_ll_detail::add_u32(
          tx_drop_count_,
          static_cast<uint32_t>(size - pushed));
      return HAL_BUSY;
    }

    return HAL_OK;
  }

  HAL_StatusTypeDef write(uint8_t *data,
                          uint16_t size,
                          uint32_t timeout = 0) override {
    return write(static_cast<const uint8_t *>(data), size, timeout);
  }

#if __cplusplus >= 202002L
  HAL_StatusTypeDef write(std::span<const uint8_t> data,
                          uint32_t timeout = 0) override {
    if (data.size() > UINT16_MAX) {
      return HAL_ERROR;
    }

    return write(data.data(), static_cast<uint16_t>(data.size()), timeout);
  }
#endif

  HAL_StatusTypeDef write_byte(uint8_t byte, uint32_t timeout = 0) override {
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
      uart_ll_detail::add_u32(
          tx_drop_count_,
          static_cast<uint32_t>(size - pushed));
    }

    kick_tx();

    return pushed;
  }

  void poll() override {
    if (instance_ == nullptr || use_tx_dma_) {
      return;
    }

    drain_to_hardware();

    if (empty() && !use_tx_interrupt_) {
      uart_ll_detail::disable_tx_irq(instance_);
      tx_idle_ = true;
    }
  }

  void irq_handler() override {
    uart_ll_detail::inc_u32(irq_count_);

    if (instance_ == nullptr) {
      return;
    }

    uart_ll_detail::clear_uart_error_flags(instance_);

    if (use_tx_dma_) {
      return;
    }

    if (uart_ll_detail::is_tx_irq_enabled(instance_) &&
        uart_ll_detail::tx_ready(instance_)) {
      uart_ll_detail::inc_u32(txe_irq_count_);

      drain_to_hardware();

      if (empty()) {
        uart_ll_detail::disable_tx_irq(instance_);
        tx_idle_ = true;
      }
    }

    // TC IRQ is intentionally not used.
    // TXE IRQ / poll / DMA are enough for byte enqueueing.
  }

  void irq_handler_with_hal_rx() override {
    irq_handler();

    if (handle_ != nullptr) {
      HAL_UART_IRQHandler(handle_);
    }
  }

  void dma_tx_complete_handler() override {
    uart_ll_detail::inc_u32(dma_tx_complete_count_);

    uint16_t completed_size = 0;

    {
      const uint32_t primask = enter_critical();

      completed_size = dma_tx_active_size_;

      tx_tail_ = (tx_tail_ + completed_size) & MASK;
      dma_tx_active_size_ = 0;
      dma_tx_active_ = false;

      uart_ll_detail::add_u32(tx_sent_count_, completed_size);

      if (empty()) {
        tx_idle_ = true;
      }

      restore_critical(primask);
    }

    kick_tx_dma();
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

  uint32_t irq_count() const {
    return irq_count_;
  }

  uint32_t txe_irq_count() const {
    return txe_irq_count_;
  }

  uint32_t tc_irq_count() const {
    return tc_irq_count_;
  }

  uint32_t uart_error_irq_count() const {
    return uart_error_irq_count_;
  }

  uint32_t dma_tx_start_count() const {
    return dma_tx_start_count_;
  }

  uint32_t dma_tx_complete_count() const {
    return dma_tx_complete_count_;
  }

  uint32_t dma_tx_error_count() const {
    return dma_tx_error_count_;
  }

  bool dma_tx_active() const {
    return dma_tx_active_;
  }

  void clear_tx_queue() {
    if (handle_ != nullptr && dma_tx_active_) {
      HAL_UART_AbortTransmit(handle_);
    }

    const uint32_t primask = enter_critical();

    tx_head_ = 0;
    tx_tail_ = 0;

    dma_tx_active_ = false;
    dma_tx_active_size_ = 0;

    restore_critical(primask);

    tx_idle_ = true;
  }

  void reset_counters() {
    tx_drop_count_ = 0;
    tx_enqueue_count_ = 0;
    tx_sent_count_ = 0;

    irq_count_ = 0;
    txe_irq_count_ = 0;
    tc_irq_count_ = 0;
    uart_error_irq_count_ = 0;

    dma_tx_start_count_ = 0;
    dma_tx_complete_count_ = 0;
    dma_tx_error_count_ = 0;
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

    uart_ll_detail::add_u32(
        tx_enqueue_count_,
        static_cast<uint32_t>(pushed));

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
      uart_ll_detail::inc_u32(tx_sent_count_);
      tx_idle_ = false;
    }
  }

  void kick_tx() {
    if (use_tx_dma_) {
      kick_tx_dma();
    } else if (use_tx_interrupt_) {
      kick_tx_irq();
    } else {
      poll();
    }
  }

  void kick_tx_irq() {
    if (instance_ == nullptr) {
      return;
    }

    tx_idle_ = false;
    uart_ll_detail::disable_tc_irq(instance_);
    uart_ll_detail::enable_tx_irq(instance_);
  }

  void kick_tx_dma() {
    if (handle_ == nullptr || instance_ == nullptr || !use_tx_dma_) {
      return;
    }

    uint8_t *dma_ptr = nullptr;
    uint16_t dma_size = 0;

    {
      const uint32_t primask = enter_critical();

      if (dma_tx_active_ || empty()) {
        restore_critical(primask);
        return;
      }

      const size_t tail = tx_tail_;
      const size_t head = tx_head_;

      size_t contiguous = 0;
      if (head > tail) {
        contiguous = head - tail;
      } else {
        contiguous = TxRingSize - tail;
      }

      if (contiguous > UINT16_MAX) {
        contiguous = UINT16_MAX;
      }

      if (contiguous == 0) {
        restore_critical(primask);
        return;
      }

      dma_ptr = &tx_ring_[tail];
      dma_size = static_cast<uint16_t>(contiguous);

      dma_tx_active_ = true;
      dma_tx_active_size_ = dma_size;
      tx_idle_ = false;

      restore_critical(primask);
    }

    #if (__DCACHE_PRESENT == 1U)
  SCB_CleanDCache_by_Addr(
      reinterpret_cast<uint32_t *>(reinterpret_cast<uintptr_t>(dma_ptr) & ~static_cast<uintptr_t>(31)),
      static_cast<int32_t>((dma_size + 31U) & ~31U)
  );
#endif

    const HAL_StatusTypeDef ret =
        HAL_UART_Transmit_DMA(handle_, dma_ptr, dma_size);

    if (ret == HAL_OK) {
      uart_ll_detail::inc_u32(dma_tx_start_count_);
      return;
    }

    {
      const uint32_t primask = enter_critical();

      dma_tx_active_ = false;
      dma_tx_active_size_ = 0;

      restore_critical(primask);
    }

    uart_ll_detail::inc_u32(dma_tx_error_count_);
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
