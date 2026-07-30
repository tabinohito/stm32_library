#pragma once

#include <cstddef>
#include <cstdint>

#include "realtime_bridge_interface/config/bridge_runtime_config.hpp"
#include "stm32_library/boards/prindog/hardware.hpp"

namespace stm32_library::boards::prindog {

/*
 * The only translation layer from portable runtime settings to STM32 HAL
 * constants.  Bridge/config code remains usable on non-STM32 platforms.
 */
class PrindogRuntimeConfigAdapter {
public:
    template <typename Registry>
    static bool apply_ports(
        const realtime_bridge_interface::config::BridgeRuntimeConfig& config,
        Peripherals& peripherals,
        Registry& registry
    ) {
        stop_all(peripherals);
        for (size_t i = 0; i < registry.size(); i++) {
            registry[i].set_enabled(false);
        }

        for (size_t i = 0; i < config.port_count; i++) {
            const auto& settings = config.ports[i];
            auto* port = find_port(registry, settings.logical_id);
            if (port == nullptr || !kind_matches(settings.kind, port->type())) {
                return false;
            }

            bool applied = false;
            if (
                settings.kind ==
                realtime_bridge_interface::config::BridgePortKind::Uart
            ) {
                auto* uart = uart_for(peripherals, settings.logical_id);
                applied = uart != nullptr &&
                    apply_uart(*uart, settings.uart, settings.enabled);
            } else if (
                settings.kind ==
                realtime_bridge_interface::config::BridgePortKind::Can
            ) {
                auto* can = can_for(peripherals, settings.logical_id);
                applied = can != nullptr &&
                    apply_can(*can, settings.can, settings.enabled);
            }

            if (!applied) {
                return false;
            }

            port->set_enabled(settings.enabled);
            if (
                settings.kind ==
                realtime_bridge_interface::config::BridgePortKind::Uart
            ) {
                port->set_batch_write_enabled(!settings.uart.tx_dma);
            }
            if (settings.enabled) {
                port->setup();
            }
        }

        return true;
    }

private:
    using ConfigPortKind =
        realtime_bridge_interface::config::BridgePortKind;
    using CommPortType =
        realtime_bridge_interface::components::CommPortType;
    using UartSettings =
        realtime_bridge_interface::config::UartSettings;
    using CanSettings =
        realtime_bridge_interface::config::CanSettings;
    using Uart = stm32_library::stm32_peripherals::Uart;
    using Can = stm32_library::stm32_peripherals::Can;

    template <typename Registry>
    static realtime_bridge_interface::components::CommPortRef* find_port(
        Registry& registry,
        uint8_t logical_id
    ) {
        for (size_t i = 0; i < registry.size(); i++) {
            if (registry[i].logical_id() == logical_id) {
                return &registry[i];
            }
        }
        return nullptr;
    }

    static bool kind_matches(ConfigPortKind kind, CommPortType type) {
        return
            (
                kind == ConfigPortKind::Uart &&
                type == CommPortType::Uart
            ) ||
            (
                kind == ConfigPortKind::Can &&
                type == CommPortType::Can
            );
    }

    static Uart* uart_for(Peripherals& peripherals, uint8_t logical_id) {
        switch (logical_id) {
        case 0:
            return &peripherals.rs485_uart2;
        case 1:
            return &peripherals.rs485_uart3;
        case 2:
            return &peripherals.rs485_uart4;
        case 3:
            return &peripherals.rs485_uart5;
        case 4:
            return &peripherals.rs485_uart7;
        case 5:
            return &peripherals.rs485_uart8;
        default:
            return nullptr;
        }
    }

    static Can* can_for(Peripherals& peripherals, uint8_t logical_id) {
        switch (logical_id) {
        case 6:
            return &peripherals.can1;
        case 7:
            return &peripherals.can2;
        case 8:
            return &peripherals.can3;
        default:
            return nullptr;
        }
    }

    static void stop_all(Peripherals& peripherals) {
        for (uint8_t logical_id = 0; logical_id < 6; logical_id++) {
            if (auto* uart = uart_for(peripherals, logical_id)) {
                uart->use_dma_transmit(false);
                (void)uart->stop();
            }
        }
        for (uint8_t logical_id = 6; logical_id < 9; logical_id++) {
            if (auto* can = can_for(peripherals, logical_id)) {
                (void)can->stop();
            }
        }
    }

    static bool apply_uart(
        Uart& uart,
        const UartSettings& settings,
        bool enabled
    ) {
        if (!enabled) {
            uart.use_dma_transmit(false);
            return uart.stop() == HAL_OK;
        }
        if (settings.tx_dma && !uart.dma_transmit_available()) {
            return false;
        }

        Uart::LineConfig config{};
        config.baud_rate = settings.baud_rate;
        config.physical_mode =
            settings.physical_mode ==
                realtime_bridge_interface::config::UartPhysicalMode::Rs485 ?
                Uart::PhysicalMode::Rs485 :
                Uart::PhysicalMode::Uart;
        config.word_length = uart_word_length(settings.word_length);
        config.stop_bits = uart_stop_bits(settings.stop_bits);
        config.parity = uart_parity(settings.parity);
        config.mode = uart_direction(settings.direction);
        config.hardware_flow_control =
            uart_flow_control(settings.flow_control);
        config.oversampling =
            settings.oversampling ==
                realtime_bridge_interface::config::UartOversampling::X8 ?
                UART_OVERSAMPLING_8 :
                UART_OVERSAMPLING_16;
        config.one_bit_sampling = settings.one_bit_sampling ?
            UART_ONE_BIT_SAMPLE_ENABLE :
            UART_ONE_BIT_SAMPLE_DISABLE;
        config.swap_rx_tx = settings.swap_rx_tx;
        config.invert_tx = settings.invert_tx;
        config.invert_rx = settings.invert_rx;
        config.data_invert = settings.data_invert;
        config.overrun_disable = settings.overrun_disable;
        config.dma_disable_on_rx_error =
            settings.dma_disable_on_rx_error;
        config.auto_baud = settings.auto_baud;
        config.auto_baud_mode =
            uart_auto_baud_mode(settings.auto_baud_mode);
        config.msb_first = settings.msb_first;
        config.rs485_de_polarity = settings.rs485_de_active_high ?
            UART_DE_POLARITY_HIGH :
            UART_DE_POLARITY_LOW;
        config.rs485_assertion_time = settings.rs485_assertion_time;
        config.rs485_deassertion_time =
            settings.rs485_deassertion_time;

        if (uart.configure(config) != HAL_OK) {
            return false;
        }
        uart.use_dma_transmit(settings.tx_dma);
        return true;
    }

    static uint32_t uart_word_length(
        realtime_bridge_interface::config::UartWordLength value
    ) {
        switch (value) {
        case realtime_bridge_interface::config::UartWordLength::Bits7:
            return UART_WORDLENGTH_7B;
        case realtime_bridge_interface::config::UartWordLength::Bits9:
            return UART_WORDLENGTH_9B;
        case realtime_bridge_interface::config::UartWordLength::Bits8:
        default:
            return UART_WORDLENGTH_8B;
        }
    }

    static uint32_t uart_stop_bits(
        realtime_bridge_interface::config::UartStopBits value
    ) {
        return
            value ==
                realtime_bridge_interface::config::UartStopBits::Two ?
                UART_STOPBITS_2 :
                UART_STOPBITS_1;
    }

    static uint32_t uart_parity(
        realtime_bridge_interface::config::UartParity value
    ) {
        switch (value) {
        case realtime_bridge_interface::config::UartParity::Even:
            return UART_PARITY_EVEN;
        case realtime_bridge_interface::config::UartParity::Odd:
            return UART_PARITY_ODD;
        case realtime_bridge_interface::config::UartParity::None:
        default:
            return UART_PARITY_NONE;
        }
    }

    static uint32_t uart_direction(
        realtime_bridge_interface::config::UartDirection value
    ) {
        switch (value) {
        case realtime_bridge_interface::config::UartDirection::Rx:
            return UART_MODE_RX;
        case realtime_bridge_interface::config::UartDirection::Tx:
            return UART_MODE_TX;
        case realtime_bridge_interface::config::UartDirection::TxRx:
        default:
            return UART_MODE_TX_RX;
        }
    }

    static uint32_t uart_flow_control(
        realtime_bridge_interface::config::UartFlowControl value
    ) {
        switch (value) {
        case realtime_bridge_interface::config::UartFlowControl::Rts:
            return UART_HWCONTROL_RTS;
        case realtime_bridge_interface::config::UartFlowControl::Cts:
            return UART_HWCONTROL_CTS;
        case realtime_bridge_interface::config::UartFlowControl::RtsCts:
            return UART_HWCONTROL_RTS_CTS;
        case realtime_bridge_interface::config::UartFlowControl::None:
        default:
            return UART_HWCONTROL_NONE;
        }
    }

    static uint32_t uart_auto_baud_mode(
        realtime_bridge_interface::config::UartAutoBaudMode value
    ) {
        switch (value) {
        case realtime_bridge_interface::config::
            UartAutoBaudMode::FallingEdge:
            return UART_ADVFEATURE_AUTOBAUDRATE_ONFALLINGEDGE;
        case realtime_bridge_interface::config::
            UartAutoBaudMode::Frame0x7f:
            return UART_ADVFEATURE_AUTOBAUDRATE_ON0X7FFRAME;
        case realtime_bridge_interface::config::
            UartAutoBaudMode::Frame0x55:
            return UART_ADVFEATURE_AUTOBAUDRATE_ON0X55FRAME;
        case realtime_bridge_interface::config::
            UartAutoBaudMode::StartBit:
        default:
            return UART_ADVFEATURE_AUTOBAUDRATE_ONSTARTBIT;
        }
    }

    static bool apply_can(
        Can& can,
        const CanSettings& settings,
        bool enabled
    ) {
        if (!enabled) {
            return can.stop() == HAL_OK;
        }

        Can::BitTimingConfig config{};
        config.prescaler = settings.prescaler;
        config.mode = can_mode(settings.mode);
        config.sync_jump_width = can_sjw(settings.sync_jump_width);
        config.time_segment1 = can_bs1(settings.time_segment1);
        config.time_segment2 = can_bs2(settings.time_segment2);
        config.time_triggered_mode =
            functional_state(settings.time_triggered_mode);
        config.auto_bus_off = functional_state(settings.auto_bus_off);
        config.auto_wakeup = functional_state(settings.auto_wakeup);
        config.auto_retransmission =
            functional_state(settings.auto_retransmission);
        config.receive_fifo_locked =
            functional_state(settings.rx_fifo_locked);
        config.transmit_fifo_priority =
            functional_state(settings.tx_fifo_priority);
        config.filter_id = settings.filter_id;
        config.filter_mask = settings.filter_mask;
        return can.configure(config) == HAL_OK;
    }

    static FunctionalState functional_state(bool enabled) {
        return enabled ? ENABLE : DISABLE;
    }

    static uint32_t can_mode(
        realtime_bridge_interface::config::CanMode mode
    ) {
        switch (mode) {
        case realtime_bridge_interface::config::CanMode::Loopback:
            return CAN_MODE_LOOPBACK;
        case realtime_bridge_interface::config::CanMode::Silent:
            return CAN_MODE_SILENT;
        case realtime_bridge_interface::config::CanMode::SilentLoopback:
            return CAN_MODE_SILENT_LOOPBACK;
        case realtime_bridge_interface::config::CanMode::Normal:
        default:
            return CAN_MODE_NORMAL;
        }
    }

    static uint32_t can_sjw(uint8_t value) {
        constexpr uint32_t Values[] = {
            CAN_SJW_1TQ,
            CAN_SJW_2TQ,
            CAN_SJW_3TQ,
            CAN_SJW_4TQ,
        };
        return Values[value >= 1 && value <= 4 ? value - 1U : 0U];
    }

    static uint32_t can_bs1(uint8_t value) {
        constexpr uint32_t Values[] = {
            CAN_BS1_1TQ, CAN_BS1_2TQ, CAN_BS1_3TQ, CAN_BS1_4TQ,
            CAN_BS1_5TQ, CAN_BS1_6TQ, CAN_BS1_7TQ, CAN_BS1_8TQ,
            CAN_BS1_9TQ, CAN_BS1_10TQ, CAN_BS1_11TQ, CAN_BS1_12TQ,
            CAN_BS1_13TQ, CAN_BS1_14TQ, CAN_BS1_15TQ, CAN_BS1_16TQ,
        };
        return Values[value >= 1 && value <= 16 ? value - 1U : 0U];
    }

    static uint32_t can_bs2(uint8_t value) {
        constexpr uint32_t Values[] = {
            CAN_BS2_1TQ, CAN_BS2_2TQ, CAN_BS2_3TQ, CAN_BS2_4TQ,
            CAN_BS2_5TQ, CAN_BS2_6TQ, CAN_BS2_7TQ, CAN_BS2_8TQ,
        };
        return Values[value >= 1 && value <= 8 ? value - 1U : 0U];
    }
};

} // namespace stm32_library::boards::prindog
