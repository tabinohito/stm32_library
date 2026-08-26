#pragma once

#include <cstddef>
#include <cstdint>

#include "main.h"

#ifdef HAL_UART_MODULE_ENABLED

#include "uart.hpp"

namespace stm32_library::stm32_peripherals {

struct UartDmaByteBridgeConfig {
    uint32_t idle_flush_us = 250;
    uint32_t max_hold_us = 1000;
    uint32_t tx_timeout_ms = 20;
};

struct UartDmaByteBridgeStats {
    uint32_t bytes = 0;
    uint32_t flush_count = 0;
    uint32_t last_size = 0;
    uint32_t write_busy_count = 0;
    uint32_t write_error_count = 0;
    uint32_t rx_recover_count = 0;
};

class UartDmaByteBridge {
public:
    UartDmaByteBridge(
        Uart& source,
        Uart& destination,
        uint8_t* buffer,
        size_t buffer_size,
        UartDmaByteBridgeConfig config = {}
    )
        : source_(source),
          destination_(destination),
          buffer_(buffer),
          capacity_(buffer_size),
          config_(config)
    {
    }

    void reset()
    {
        size_ = 0;
        first_cycle_ = 0;
        last_cycle_ = 0;
        stats_ = {};
    }

    bool recover_source_rx_dma()
    {
        const bool recovered = source_.restart_receive_dma_if_error();

        if (recovered) {
            stats_.rx_recover_count++;
        }

        return recovered;
    }

    size_t source_available() const
    {
        return source_.dma_receive_data_num();
    }

    const UartDmaByteBridgeStats& stats() const
    {
        return stats_;
    }

    size_t poll()
    {
        if (buffer_ == nullptr || capacity_ == 0) {
            return 0;
        }

        size_t flushed = 0;

        while (source_.dma_receive_data_num() > 0) {
            if (size_ >= capacity_) {
                flushed += flush();

                if (size_ >= capacity_) {
                    break;
                }
            }

            const uint32_t current_cycle = now_cycles();
            if (size_ == 0) {
                first_cycle_ = current_cycle;
            }

            buffer_[size_] = source_.dma_receive_data();
            size_++;
            last_cycle_ = current_cycle;
        }

        if (
            size_ > 0 &&
            (
                elapsed_cycles(last_cycle_, cycles_from_us(config_.idle_flush_us)) ||
                elapsed_cycles(first_cycle_, cycles_from_us(config_.max_hold_us))
            )
        ) {
            flushed += flush();
        }

        return flushed;
    }

    size_t flush()
    {
        if (buffer_ == nullptr || size_ == 0) {
            return 0;
        }

        const size_t write_size = size_;
        const HAL_StatusTypeDef ret = destination_.write(
            buffer_,
            static_cast<uint16_t>(write_size),
            config_.tx_timeout_ms
        );

        if (ret == HAL_OK) {
            stats_.bytes += static_cast<uint32_t>(write_size);
            stats_.flush_count++;
            stats_.last_size = static_cast<uint32_t>(write_size);
            size_ = 0;
            return write_size;
        }

        if (ret == HAL_BUSY) {
            stats_.write_busy_count++;
        } else {
            stats_.write_error_count++;
        }

        return 0;
    }

private:
    static uint32_t now_cycles()
    {
        return DWT->CYCCNT;
    }

    static uint32_t cycles_from_us(uint32_t us)
    {
        return (HAL_RCC_GetHCLKFreq() / 1000000U) * us;
    }

    static bool elapsed_cycles(uint32_t start, uint32_t interval)
    {
        return static_cast<uint32_t>(now_cycles() - start) >= interval;
    }

    Uart& source_;
    Uart& destination_;
    uint8_t* buffer_ = nullptr;
    size_t capacity_ = 0;
    size_t size_ = 0;
    uint32_t first_cycle_ = 0;
    uint32_t last_cycle_ = 0;
    UartDmaByteBridgeConfig config_{};
    UartDmaByteBridgeStats stats_{};
};

} // namespace stm32_library::stm32_peripherals

#endif // HAL_UART_MODULE_ENABLED
