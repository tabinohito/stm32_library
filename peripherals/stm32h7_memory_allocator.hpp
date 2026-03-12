#pragma once
#include "main.h"

#ifdef STM32H7xx_H

#ifndef STM32LIB_H7_RAM_D2_RES_SIZE
#define STM32LIB_H7_RAM_D2_RES_SIZE 128
#endif

#include <cassert>

/*
// 変数はポインタ型に統一(.hpp)
    uint8_t *receive_data_;
// コンストラクタを書き換える(.cpp)
DualShock3::DualShock3(UartSerial serial) :
#ifdef STM32H7xx_H
        receive_data_(ram_d2_alocator::allocate(8)),
#else
    receive_data_(new uint8_t[8]),
#endif
        serial_(serial),
        button_data_ {},
        stick_data_ {}

*/

namespace abu2023::stm32_peripherals {

namespace ram_d2_alocator {
namespace internal {
static inline uint8_t res_ram_d2_[STM32LIB_H7_RAM_D2_RES_SIZE] __attribute__((section(".RAM_D2")))
__attribute__((aligned(4)));
static inline size_t allocated_size_ = 0;
} // namespace internal

static uint8_t *allocate(const size_t size) {
  assert((size % 4) == 0); // これ正しい?
  assert(internal::allocated_size_ + size <= STM32LIB_H7_RAM_D2_RES_SIZE);
  uint8_t *ptr = &internal::res_ram_d2_[internal::allocated_size_];
  internal::allocated_size_ += size;
  return ptr;
}

// コンパイル時assertにする場合, ram_d2_alocator::allocate<8> みたいに呼び出す
// template<size_t SIZE>
// static uint8_t* allocate()
// {
//     static_assert((SIZE %= 4) == 0); // これ正しい?
//     static_assert(internal::allocated_size_ + SIZE <= STM32LIB_H7_RAM_D2_RES_SIZE);
//     uint8_t *ptr = &internal::res_ram_d2_[internal::allocated_size_];
//     internal::allocated_size_ += SIZE;
//     return ptr;
// }

} // namespace ram_d2_alocator

} // namespace abu2023::stm32_peripherals

#endif // STM32H7xx_H