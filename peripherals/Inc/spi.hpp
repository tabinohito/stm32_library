#pragma once

#include "main.h"

#ifdef HAL_SPI_MODULE_ENABLED

namespace stm32_library::stm32_peripherals {
class Spi {
public:
  enum class DmaStatus : uint8_t {
    Idle,
    Busy,
    Complete,
    Error,
  };

  Spi(SPI_HandleTypeDef *handle) : handle_(handle) {}

  uint8_t write(uint8_t val) {
    uint8_t rx;
    if (HAL_SPI_TransmitReceive(handle_, (uint8_t *)&val, (uint8_t *)&rx, 1, 1000) != HAL_OK) {
      return 0;
    }
    return rx;
  }

  bool write(uint8_t *tx_buf, uint8_t *rx_buf, size_t length) {
    if (HAL_SPI_TransmitReceive(handle_, (uint8_t *)tx_buf, (uint8_t *)rx_buf, length, 1000) != HAL_OK) {
      return false;
    }
    return true;
  }

  uint8_t write_dma(uint8_t val) {
    uint8_t rx = 0;
    if (start_transfer_dma(&val, &rx, 1) != HAL_OK) {
      return 0;
    }

    DmaStatus status = DmaStatus::Busy;
    while (status == DmaStatus::Busy) {
      status = poll_dma();
    }

    return status == DmaStatus::Complete ? rx : 0;
  }

  bool write_dma(uint8_t *tx_buf, uint8_t *rx_buf, size_t length) {
    if (start_transfer_dma(tx_buf, rx_buf, length) != HAL_OK) {
      return false;
    }

    DmaStatus status = DmaStatus::Busy;
    while (status == DmaStatus::Busy) {
      status = poll_dma();
    }

    return status == DmaStatus::Complete;
  }

  /*
   * Start a full-duplex transfer and return immediately.
   *
   * Both buffers must remain valid until poll_dma() reports Complete or
   * Error.  On STM32F7 they must be in SRAM1/SRAM2 rather than DTCM.
   */
  HAL_StatusTypeDef start_transfer_dma(
      const uint8_t *tx_buf,
      uint8_t *rx_buf,
      size_t length
  ) {
    if (
        handle_ == nullptr ||
        handle_->hdmarx == nullptr ||
        handle_->hdmatx == nullptr ||
        tx_buf == nullptr ||
        rx_buf == nullptr ||
        length == 0U ||
        length > UINT16_MAX ||
        dma_active_
    ) {
      return HAL_ERROR;
    }

    const HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_DMA(
        handle_,
        tx_buf,
        rx_buf,
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

    const HAL_SPI_StateTypeDef state = HAL_SPI_GetState(handle_);
    if (state == HAL_SPI_STATE_READY) {
      dma_active_ = false;
      return HAL_SPI_GetError(handle_) == HAL_SPI_ERROR_NONE
          ? DmaStatus::Complete
          : DmaStatus::Error;
    }

    if (state == HAL_SPI_STATE_RESET || state == HAL_SPI_STATE_ERROR) {
      dma_active_ = false;
      return DmaStatus::Error;
    }

    return DmaStatus::Busy;
  }

  bool dma_busy() const {
    return dma_active_;
  }

private:
  SPI_HandleTypeDef *handle_;
  bool dma_active_ = false;
};

} // namespace stm32_library::stm32_peripherals

#endif // HAL_SPI_MODULE_ENABLED
