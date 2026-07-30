#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "realtime_bridge_interface/config/bridge_runtime_config.hpp"
#include "stm32_library/boards/prindog/factory_defaults.h"

namespace stm32_library::boards::prindog {

/*
 * Factory defaults are the former Dynamixel firmware behaviour.  They are
 * intentionally outside main.h/CubeMX generated files so one ELF can be used
 * on every board and only the Flash journal differs.
 */
inline std::array<
    realtime_bridge_interface::config::BridgePortSettings,
    9
> make_factory_port_settings()
{
    using realtime_bridge_interface::config::BridgePortKind;
    using realtime_bridge_interface::config::BridgePortSettings;
    using realtime_bridge_interface::config::UartPhysicalMode;

    std::array<BridgePortSettings, 9> ports{};

    for (size_t i = 0; i < 6; i++) {
        ports[i].logical_id = static_cast<uint8_t>(i);
        ports[i].kind = BridgePortKind::Uart;
        ports[i].enabled = true;
        ports[i].uart.baud_rate = 4000000;
        ports[i].uart.physical_mode = UartPhysicalMode::Rs485;
        ports[i].uart.tx_dma = false;
    }
    // UART5 is crossed on STM32_CAN_ETH and was already swapped in CubeMX.
    ports[3].uart.swap_rx_tx = true;

    for (size_t i = 6; i < ports.size(); i++) {
        ports[i].logical_id = static_cast<uint8_t>(i);
        ports[i].kind = BridgePortKind::Can;
        ports[i].enabled = false;
        ports[i].can.prescaler = 3;
        ports[i].can.sync_jump_width = 1;
        ports[i].can.time_segment1 = 15;
        ports[i].can.time_segment2 = 2;
    }

    return ports;
}

inline realtime_bridge_interface::config::BoardFeatureSettings
make_factory_feature_settings()
{
    realtime_bridge_interface::config::BoardFeatureSettings features{};
    features.board_id = PRINDOG_FACTORY_BOARD_ID;
    features.emergency_enabled = false;
    features.current_sensor_mask = 0;
    features.thermistor_sensor_mask = 0;
    features.acyclic_emergency_enabled = false;
    features.acyclic_emergency_index = 0;
    features.acyclic_emergency_active_value = 1;
    return features;
}

} // namespace stm32_library::boards::prindog
