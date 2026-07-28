#pragma once

#include "main.h"

#ifdef HAL_I2C_MODULE_ENABLED

#include <vector>

namespace stm32_library::stm32_peripherals {
class I2c {
private:
  I2C_HandleTypeDef *handle_;
  bool dma_active_ = false;

public:
  enum class DmaStatus : uint8_t {
    Idle,
    Busy,
    Complete,
    Error,
  };

  I2c(I2C_HandleTypeDef *handle) : handle_(handle) {}

  bool read(uint8_t addr, char *data, size_t length) {
    if (HAL_I2C_Master_Receive(handle_, (uint16_t)addr << 1, (uint8_t *)data, length, 100) != HAL_OK)
      return false; // error
    return true;
  }

  bool read_reg(uint8_t addr, uint8_t reg, uint8_t *buffer, size_t length) {
    if (HAL_I2C_Master_Transmit(handle_, static_cast<uint16_t>(addr) << 1, &reg, 1, 100) != HAL_OK) {
      return false;
    }
    if (HAL_I2C_Master_Receive(handle_, static_cast<uint16_t>(addr) << 1, buffer, length, 100) != HAL_OK) {
      return false;
    }
    return true;
  }

  bool write(uint8_t addr, char *data, size_t length) {
    if (HAL_I2C_Master_Transmit(handle_, (uint16_t)addr << 1, (uint8_t *)data, length, 100) != HAL_OK)
      return false; // error
    return true;
  }

  /*
   * Start a register read and return immediately.
   *
   * The supplied buffer must remain valid until poll_dma() reports Complete
   * or Error.  On STM32F7 it must also live in memory reachable by DMA1/2
   * (SRAM1/SRAM2, not DTCM).
   */
  HAL_StatusTypeDef start_read_reg_dma(
      uint8_t addr,
      uint8_t reg,
      uint8_t *buffer,
      size_t length
  ) {
    if (
        handle_ == nullptr ||
        handle_->hdmarx == nullptr ||
        buffer == nullptr ||
        length == 0U ||
        length > UINT16_MAX ||
        dma_active_
    ) {
      return HAL_ERROR;
    }

    const HAL_StatusTypeDef status = HAL_I2C_Mem_Read_DMA(
        handle_,
        static_cast<uint16_t>(addr) << 1,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        buffer,
        static_cast<uint16_t>(length)
    );

    if (status == HAL_OK) {
      dma_active_ = true;
    }

    return status;
  }

  DmaStatus poll_dma() {
    if (!dma_active_) {
      return DmaStatus::Idle;
    }

    const HAL_I2C_StateTypeDef state = HAL_I2C_GetState(handle_);
    if (state == HAL_I2C_STATE_READY) {
      dma_active_ = false;
      return HAL_I2C_GetError(handle_) == HAL_I2C_ERROR_NONE
          ? DmaStatus::Complete
          : DmaStatus::Error;
    }

    if (state == HAL_I2C_STATE_RESET) {
      dma_active_ = false;
      return DmaStatus::Error;
    }

    return DmaStatus::Busy;
  }

  bool dma_busy() const {
    return dma_active_;
  }

  I2C_HandleTypeDef *get_handle() {
    return handle_;
  }

  const I2C_HandleTypeDef *get_handle() const {
    return handle_;
  }

  bool is_device_ready(uint8_t addr) {
    if (HAL_I2C_IsDeviceReady(handle_, (uint16_t)addr << 1, 5, 1000) != HAL_OK)
      return false; // error
    HAL_Delay(250);
    return true;
  };

  std::vector<uint8_t> get_is_device_ready(void) {
    std::vector<uint8_t> addr;
    for (uint8_t i = 1; i < 128; i++) {
      if (is_device_ready(i)) { // HAL_OK
        addr.push_back(i);      // Received an ACK at that address
      }
    }
    return addr;
  };
};

} // namespace stm32_library::stm32_peripherals

#endif // HAL_I2C_MODULE_ENABLED
