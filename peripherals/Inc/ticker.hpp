#pragma once

#include "main.h"

#ifdef HAL_TIM_MODULE_ENABLED

#include <array>
#include <cassert>
#include <chrono>
#include <functional>
#include <cmath>
#include <vector>

namespace stm32_library::stm32_peripherals {

#ifndef MAX_TICKER_ID
#define MAX_TICKER_ID (16 - 1)
#endif

class Ticker {
public:
  Ticker(TIM_HandleTypeDef *handle)
      : handle_(handle), elapsed_us_(0), start_us_(0), pretime_() {
    register_instance(this);
#ifdef STM32_LIBRARY_USE_STATIC_INSTANCE
    HalStartupManager::add_startup_callback([this] { startup(); });
#else
    startup();
#endif
  }

  // void attach(Callback<void()> func, std::chrono::microseconds t):
  // void detach();

  void attach(std::function<void(void)> func, size_t division = 1) {
    if (division == 0)
      division = 1;
    cb_items_.push_back({func, division, 0});
  }

  void clear_callbacks() {
    cb_items_.clear();
  }

  HAL_StatusTypeDef configure_frequency(uint32_t frequency_hz) {
    if (handle_ == nullptr) {
      return HAL_ERROR;
    }

    if (frequency_hz == 0U) {
      frequency_hz = 1U;
    }

    stop();

    const uint32_t timer_clock = timer_input_clock_hz();
    uint32_t period_ticks = timer_clock / frequency_hz;
    if (period_ticks == 0U) {
      period_ticks = 1U;
    }

    handle_->Init.Prescaler = 0;
    handle_->Init.CounterMode = TIM_COUNTERMODE_UP;
    handle_->Init.Period = period_ticks - 1U;
    handle_->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    const HAL_StatusTypeDef ret = HAL_TIM_Base_Init(handle_);
    if (ret != HAL_OK) {
      return ret;
    }

    __HAL_TIM_SET_COUNTER(handle_, 0);
    __HAL_TIM_CLEAR_FLAG(handle_, TIM_FLAG_UPDATE);

    configured_frequency_hz_ = frequency_hz;
    period_us_ = calculate_period_us();
    reset_time_counters();

    return HAL_OK;
  }

  HAL_StatusTypeDef start() {
    if (handle_ == nullptr) {
      return HAL_ERROR;
    }

    const HAL_StatusTypeDef ret = HAL_TIM_Base_Start_IT(handle_);
    if (ret == HAL_OK) {
      if (period_us_ == 0U) {
        period_us_ = calculate_period_us();
      }
      started_ = true;
    }

    return ret;
  }

  void stop() {
    if (handle_ != nullptr) {
      (void)HAL_TIM_Base_Stop_IT(handle_);
    }
    started_ = false;
  }

  void reset_time_counters() {
    const uint32_t primask = enter_critical();
    elapsed_us_ = 0;
    interrupt_count_ = 0;
    pending_count_ = 0;
    start_us_ = 0;
    restore_critical(primask);
  }

  uint32_t interrupt_count() const { return interrupt_count_; }

  uint32_t take_pending() {
    const uint32_t primask = enter_critical();
    const uint32_t pending = pending_count_;
    pending_count_ = 0;
    restore_critical(primask);
    return pending;
  }

  uint32_t configured_frequency_hz() const { return configured_frequency_hz_; }

  uint32_t timer_period() const {
    return handle_ != nullptr ? handle_->Init.Period : 0U;
  }

  bool started() const { return started_; }

  void timer_reset(float second = 0) { start_us_ += (read_us() - static_cast<uint32_t>(std::round(second * 1e6))); }

  uint32_t get_counter(void) { return __HAL_TIM_GET_COUNTER(handle_); }

  uint32_t read_us() { return elapsed_us_ + counter_us() - start_us_; }

  uint32_t read_ms() { return read_us() / static_cast<uint32_t>(1000); }

  float lap_time(uint8_t ticker_id, bool first_lap_time_is_zero = false) {
    assert(0 <= ticker_id);
    assert(ticker_id <= MAX_TICKER_ID);

    if (first_lap_time_is_zero && pretime_.at(ticker_id) == 0) {
      pretime_.at(ticker_id) = read_us();
    }
    uint32_t us = read_us() - pretime_.at(ticker_id);
    pretime_.at(ticker_id) = read_us();
    return static_cast<float>(us / 1e6);
  }

  float read_time() { return read_us() / 1000000.0f; };

  bool wait(uint32_t ms, uint8_t ticker_id) {
    assert(0 <= ticker_id);
    assert(ticker_id <= MAX_TICKER_ID);

    if (pretime_.at(ticker_id) == 0U) {
      pretime_.at(ticker_id) = read_ms();
      return false;
    } else {
      if (pretime_.at(ticker_id) + ms <= read_ms()) {
        pretime_.at(ticker_id) = 0;
        return true;
      }
    }
    return false;
  }
  bool await(uint32_t ms, uint8_t ticker_id) {
    assert(0 <= ticker_id);
    assert(ticker_id <= MAX_TICKER_ID);

    if (pretime_.at(ticker_id) == 0U) {
      pretime_.at(ticker_id) = read_ms();
      return false;
    } else {
      if (pretime_.at(ticker_id) + ms <= read_ms()) {
        pretime_.at(ticker_id) += ms;
        return true;
      }
    }
    return false;
  }

  // 受信割り込み用
  // 手動で呼び出ししないこと
  static void tim_it(TIM_HandleTypeDef *htim) {
    Ticker *ticker = find_instance(htim);

    if (ticker != nullptr) {
      ticker->callback();
    }
  }

  static bool irq_handler(TIM_HandleTypeDef *htim) {
    if (htim == nullptr) {
      return false;
    }

    if (
        __HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE) != RESET &&
        __HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_UPDATE) != RESET
    ) {
      __HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);
      tim_it(htim);
      return true;
    }

    return false;
  }

protected:
  TIM_HandleTypeDef *handle_;

  void startup() { (void)start(); }

  void callback() {
    interrupt_count_ = interrupt_count_ + 1U;
    if (pending_count_ < UINT32_MAX) {
      pending_count_ = pending_count_ + 1U;
    }
    elapsed_us_ = elapsed_us_ + period_us_;
    for (auto &item : cb_items_) {
      item.cnt++;
      if (item.cnt == item.div) {
        item.func();
        item.cnt = 0;
      }
    }
  }

private:
  struct cb_item_t {
    std::function<void(void)> func;
    size_t div;
    size_t cnt;
  };

  std::vector<cb_item_t> cb_items_;

  volatile uint32_t elapsed_us_;
  volatile uint32_t interrupt_count_ = 0;
  volatile uint32_t pending_count_ = 0;
  uint32_t start_us_;
  uint32_t configured_frequency_hz_ = 0;
  uint32_t period_us_ = 0;
  bool started_ = false;
  std::array<uint32_t, MAX_TICKER_ID + 1> pretime_;

  static constexpr size_t instance_capacity_ = 16;

  static Ticker **instances() {
    static Ticker *list[instance_capacity_] = {};
    return list;
  }

  static void register_instance(Ticker *ticker) {
    if (ticker == nullptr || ticker->handle_ == nullptr) {
      return;
    }

    auto list = instances();

    for (size_t i = 0; i < instance_capacity_; i++) {
      if (list[i] != nullptr && list[i]->handle_ == ticker->handle_) {
        list[i] = ticker;
        return;
      }
    }

    for (size_t i = 0; i < instance_capacity_; i++) {
      if (list[i] == nullptr) {
        list[i] = ticker;
        return;
      }
    }
  }

  static Ticker *find_instance(TIM_HandleTypeDef *handle) {
    if (handle == nullptr) {
      return nullptr;
    }

    auto list = instances();

    for (size_t i = 0; i < instance_capacity_; i++) {
      if (list[i] != nullptr && list[i]->handle_ == handle) {
        return list[i];
      }
    }

    return nullptr;
  }

  uint32_t timer_input_clock_hz() const {
    if (handle_ == nullptr || handle_->Instance == nullptr) {
      return HAL_RCC_GetHCLKFreq();
    }

#if defined(TIM1)
    if (handle_->Instance == TIM1) {
      const uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
      return pclk2 == HAL_RCC_GetHCLKFreq() ? pclk2 : pclk2 * 2U;
    }
#endif
#if defined(TIM8)
    if (handle_->Instance == TIM8) {
      const uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
      return pclk2 == HAL_RCC_GetHCLKFreq() ? pclk2 : pclk2 * 2U;
    }
#endif
#if defined(TIM9)
    if (handle_->Instance == TIM9) {
      const uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
      return pclk2 == HAL_RCC_GetHCLKFreq() ? pclk2 : pclk2 * 2U;
    }
#endif
#if defined(TIM10)
    if (handle_->Instance == TIM10) {
      const uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
      return pclk2 == HAL_RCC_GetHCLKFreq() ? pclk2 : pclk2 * 2U;
    }
#endif
#if defined(TIM11)
    if (handle_->Instance == TIM11) {
      const uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
      return pclk2 == HAL_RCC_GetHCLKFreq() ? pclk2 : pclk2 * 2U;
    }
#endif

    const uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    return pclk1 == HAL_RCC_GetHCLKFreq() ? pclk1 : pclk1 * 2U;
  }

  uint32_t counter_us() const {
    if (handle_ == nullptr) {
      return 0;
    }

    const uint32_t timer_tick_hz =
        timer_input_clock_hz() / (handle_->Init.Prescaler + 1U);

    if (timer_tick_hz == 0U) {
      return 0;
    }

    const uint64_t us =
        static_cast<uint64_t>(__HAL_TIM_GET_COUNTER(handle_)) * 1000000ULL;
    return static_cast<uint32_t>(us / timer_tick_hz);
  }

  uint32_t calculate_period_us() const {
    if (handle_ == nullptr) {
      return 0;
    }

    const uint32_t timer_tick_hz =
        timer_input_clock_hz() / (handle_->Init.Prescaler + 1U);

    if (timer_tick_hz == 0U) {
      return 0;
    }

    const uint64_t us =
        static_cast<uint64_t>(handle_->Init.Period + 1U) * 1000000ULL;
    return static_cast<uint32_t>(us / timer_tick_hz);
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

#endif // HAL_TIM_MODULE_ENABLED
