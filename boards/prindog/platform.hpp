#pragma once

#include "main.h"
#include "stm32_library/boards/prindog/hardware.hpp"

extern "C" HAL_StatusTypeDef MX_I2C3_RX_DMA_Enable(void);

namespace stm32_library::boards::prindog {

/*
 * All Prindog HAL/stm32_library setup side effects live at this boundary.
 * Board profiles remain declarative and the generic board composition does
 * not alternate between raw HAL calls and wrapper calls.
 */
class PrindogPlatformSetup {
public:
    template <typename Config>
    static void setup_or_fail(stm32_library::boards::prindog::Peripherals& peripherals) {
        if constexpr (Config::UseSensorDma) {
            require_ok(MX_I2C3_RX_DMA_Enable());
        }

        if constexpr (Config::DisableRs485TxDmaQueues) {
            set_rs485_dma_enabled(peripherals, false);
        }

        if constexpr (Config::ConfigureRs485Baud) {
            require_ok(
                peripherals.rs485_uart2.configure_rs485(
                    Config::Rs485BaudRate
                )
            );
            require_ok(
                peripherals.rs485_uart3.configure_rs485(
                    Config::Rs485BaudRate
                )
            );
            require_ok(
                peripherals.rs485_uart4.configure_rs485(
                    Config::Rs485BaudRate
                )
            );
            require_ok(
                peripherals.rs485_uart5.configure_rs485(
                    Config::Rs485BaudRate,
                    true
                )
            );
            require_ok(
                peripherals.rs485_uart7.configure_rs485(
                    Config::Rs485BaudRate
                )
            );
            require_ok(
                peripherals.rs485_uart8.configure_rs485(
                    Config::Rs485BaudRate
                )
            );
        }

        if constexpr (Config::UseEstopPins) {
            configure_can_pins_as_estop_outputs();
        }
    }

private:
    static void require_ok(HAL_StatusTypeDef status) {
        if (status != HAL_OK) {
            Error_Handler();
        }
    }

    static void set_rs485_dma_enabled(
        stm32_library::boards::prindog::Peripherals& peripherals,
        bool enabled
    ) {
        peripherals.rs485_uart2.use_dma_transmit(enabled);
        peripherals.rs485_uart3.use_dma_transmit(enabled);
        peripherals.rs485_uart4.use_dma_transmit(enabled);
        peripherals.rs485_uart5.use_dma_transmit(enabled);
        peripherals.rs485_uart7.use_dma_transmit(enabled);
        peripherals.rs485_uart8.use_dma_transmit(enabled);
    }

    static void configure_can_pins_as_estop_outputs() {
        GPIO_InitTypeDef gpio{};

        (void)HAL_CAN_Stop(&hcan1);
        (void)HAL_CAN_Stop(&hcan2);
        (void)HAL_CAN_Stop(&hcan3);
        (void)HAL_CAN_DeInit(&hcan1);
        (void)HAL_CAN_DeInit(&hcan2);
        (void)HAL_CAN_DeInit(&hcan3);

        gpio.Mode = GPIO_MODE_OUTPUT_PP;
        gpio.Pull = GPIO_NOPULL;
        gpio.Speed = GPIO_SPEED_FREQ_LOW;
        gpio.Pin = GPIO_PIN_4 | GPIO_PIN_6;
        HAL_GPIO_Init(GPIOB, &gpio);
        gpio.Pin = GPIO_PIN_1;
        HAL_GPIO_Init(GPIOD, &gpio);
    }
};

} // namespace stm32_library::boards::prindog
