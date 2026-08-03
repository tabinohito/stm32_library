#pragma once

#ifndef REALTIME_BRIDGE_INTERFACE_ENABLE_DEBUG_COUNTERS
#define REALTIME_BRIDGE_INTERFACE_ENABLE_DEBUG_COUNTERS 1
#endif

#include "stm32_library/boards/prindog/hardware.hpp"
#include "stm32_library/boards/prindog/board.hpp"
#include "realtime_bridge_interface/adapters/stm32/f7/flash_config_store.hpp"
#include "stm32_library/boards/prindog/network.hpp"
#include "realtime_bridge_interface/adapters/stm32/f7/udp_realtime_bridge_runner.hpp"

namespace stm32_library::boards::prindog {

inline std::array<uint8_t, 256> config_uart_rx_dma_buffer
    __attribute__((section(".ConfigUartRxSection"), aligned(32)));

inline std::array<uint8_t, 4096> config_uart_tx_dma_buffer
    __attribute__((section(".ConfigUartTxSection"), aligned(32)));

enum class BridgeRunMode {
    BearMotorBridge,
    DynamixelBridge,
    CanBridge,
};

constexpr BridgeRunMode ActiveBridgeMode =
    BridgeRunMode::DynamixelBridge;

template <BridgeRunMode Mode>
struct BridgeRunModeTraits;

template <>
struct BridgeRunModeTraits<BridgeRunMode::BearMotorBridge> {
    using Board = ProductionBoard;
};

template <>
struct BridgeRunModeTraits<BridgeRunMode::DynamixelBridge> {
    using Board = DynamixelBoard;
};

template <>
struct BridgeRunModeTraits<BridgeRunMode::CanBridge> {
    using Board = CanBridgeBoard;
};

[[noreturn]] inline void run_active_bridge_app()
{
    stm32_library::boards::prindog::Peripherals::enable_std_printf();

    auto& peripherals = stm32_library::boards::prindog::Peripherals::get_instance();
    auto& modules = stm32_library::boards::prindog::Modules::get_instance();

    using ActiveBridgeBoard =
        typename BridgeRunModeTraits<ActiveBridgeMode>::Board;

    static realtime_bridge_interface::adapters::stm32::f7::FlashConfigStore
        config_store;

    static realtime_bridge_interface::adapters::stm32::f7::
        UdpRealtimeBridgeRunner<
        ActiveBridgeBoard,
        PrindogNetworkConfig,
        realtime_bridge_interface::adapters::stm32::f7::FlashConfigStore,
        Peripherals,
        Modules
    > runner{
        peripherals,
        modules,
        peripherals.timer_10khz,
        config_store,
        peripherals.usb_uart,
        heth,
        gnetif,
        config_uart_rx_dma_buffer.data(),
        config_uart_rx_dma_buffer.size(),
        config_uart_tx_dma_buffer.data(),
        config_uart_tx_dma_buffer.size()
    };

    runner.run();
}

} // namespace stm32_library::boards::prindog
