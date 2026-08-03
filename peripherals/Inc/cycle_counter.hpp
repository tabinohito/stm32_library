#pragma once

#include "main.h"

#include <cstdint>

namespace stm32_library::stm32_peripherals {

class CycleCounter {
public:
  static bool is_supported() {
#if defined(DWT) && defined(CoreDebug) && defined(DWT_CTRL_CYCCNTENA_Msk)
    return true;
#else
    return false;
#endif
  }

  static bool enable() {
#if defined(DWT) && defined(CoreDebug) && defined(DWT_CTRL_CYCCNTENA_Msk)
    if (is_enabled()) {
      return true;
    }

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
#if defined(__CORTEX_M) && (__CORTEX_M == 7U)
    DWT->LAR = 0xC5ACCE55U;
#endif
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    return is_enabled();
#else
    return false;
#endif
  }

  static bool is_enabled() {
#if defined(DWT) && defined(DWT_CTRL_CYCCNTENA_Msk)
    return (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U;
#else
    return false;
#endif
  }

  static uint32_t now() {
#if defined(DWT)
    return DWT->CYCCNT;
#else
    return 0U;
#endif
  }

  static uint32_t cycles() {
    return now();
  }

  static uint32_t millis() {
    return HAL_GetTick();
  }

  static uint32_t core_clock_hz() {
    return HAL_RCC_GetHCLKFreq();
  }

  static uint32_t cycles_per_us() {
    return core_clock_hz() / 1000000U;
  }

  static uint32_t us_to_cycles(uint32_t microseconds) {
    return cycles_per_us() * microseconds;
  }

  static void delay_cycles(uint32_t cycles) {
    const uint32_t started = now();
    wait_elapsed(started, cycles);
  }

  static void wait_elapsed(uint32_t started, uint32_t cycles) {
    while ((now() - started) < cycles) {
    }
  }

  static void wait_until(uint32_t deadline) {
    while (static_cast<int32_t>(now() - deadline) < 0) {
    }
  }

  [[noreturn]] static void system_reset() {
    NVIC_SystemReset();
    while (true) {
    }
  }
};

} // namespace stm32_library::stm32_peripherals
