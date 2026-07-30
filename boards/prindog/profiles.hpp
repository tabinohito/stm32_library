#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "realtime_bridge_interface/protocol/message_route_table.hpp"

namespace stm32_library::boards::prindog {

enum class BuildMode {
    UsbDebug,
    Production,
    Dynamixel,
    CanBridge,
    Unified,
};

template <BuildMode Mode>
struct BoardConfig;

template <>
struct BoardConfig<BuildMode::UsbDebug> {
    static constexpr size_t PortNum = 2;
    static constexpr size_t RouteNum = PortNum;
    static constexpr size_t MaxRouteNum = PortNum * 2;
    static constexpr uint8_t CarrierMessageId = 1;
    static constexpr size_t MaxPayloadSize = 128;
    static constexpr uint32_t BridgeTickHz = 10000;
    static constexpr bool UseEstopPins = false;
    static constexpr bool UseSensors = false;
    static constexpr bool UseSensorDma = false;
    static constexpr bool ConfigureRs485Baud = false;
    static constexpr uint32_t Rs485BaudRate = 8000000;
    static constexpr bool DisableRs485TxDmaQueues = false;
    static constexpr bool EnableSessionHeartbeat = false;
    static constexpr uint8_t SessionHeartbeatMessageId = 254;
    static constexpr uint32_t SessionHeartbeatIntervalMs = 20;
};

template <>
struct BoardConfig<BuildMode::Production> {
    static constexpr size_t PortNum = 3;
    static constexpr size_t RouteNum = PortNum;
    static constexpr size_t MaxRouteNum = PortNum * 2;
    static constexpr uint8_t CarrierMessageId = 1;
    static constexpr size_t MaxPayloadSize = 128;
    static constexpr uint32_t BridgeTickHz = 10000;
    static constexpr bool UseEstopPins = true;
    static constexpr bool UseSensors = true;
    static constexpr bool UseSensorDma = false;
    static constexpr bool ConfigureRs485Baud = false;
    static constexpr uint32_t Rs485BaudRate = 8000000;
    static constexpr bool DisableRs485TxDmaQueues = false;
    static constexpr bool EnableSessionHeartbeat = false;
    static constexpr uint8_t SessionHeartbeatMessageId = 254;
    static constexpr uint32_t SessionHeartbeatIntervalMs = 20;
};

template <>
struct BoardConfig<BuildMode::Dynamixel> {
    static constexpr size_t PortNum = 6;
    static constexpr size_t RouteNum = PortNum;
    static constexpr size_t MaxRouteNum = PortNum * 2;
    static constexpr uint8_t CarrierMessageId = 1;
    static constexpr size_t MaxPayloadSize = 128;
    static constexpr uint32_t BridgeTickHz = 20000;
    static constexpr bool UseEstopPins = false;
    static constexpr bool UseSensors = false;
    static constexpr bool UseSensorDma = false;
    static constexpr bool ConfigureRs485Baud = true;
    static constexpr uint32_t Rs485BaudRate = 4000000;
    static constexpr bool DisableRs485TxDmaQueues = true;
    static constexpr bool EnableSessionHeartbeat = true;
    static constexpr uint8_t SessionHeartbeatMessageId = 254;
    static constexpr uint32_t SessionHeartbeatIntervalMs = 20;
};

template <>
struct BoardConfig<BuildMode::CanBridge> {
    static constexpr size_t PortNum = 3;
    static constexpr size_t RouteNum = PortNum;
    static constexpr size_t MaxRouteNum = PortNum * 2;
    static constexpr uint8_t CarrierMessageId = 1;
    static constexpr size_t MaxPayloadSize = 13;
    static constexpr uint32_t BridgeTickHz = 10000;
    static constexpr bool UseEstopPins = false;
    static constexpr bool UseSensors = true;
    static constexpr bool UseSensorDma = true;
    static constexpr bool ConfigureRs485Baud = false;
    static constexpr uint32_t Rs485BaudRate = 0;
    static constexpr bool DisableRs485TxDmaQueues = true;
    static constexpr bool EnableSessionHeartbeat = false;
    static constexpr uint8_t SessionHeartbeatMessageId = 254;
    static constexpr uint32_t SessionHeartbeatIntervalMs = 20;
};

/*
 * One binary for every STM32_CAN_ETH board.  Physical ports are fixed and
 * Flash configuration controls which subset is enabled.
 */
template <>
struct BoardConfig<BuildMode::Unified> {
    static constexpr size_t PortNum = 9;
    static constexpr size_t RouteNum = 6;
    static constexpr size_t MaxRouteNum = 16;
    static constexpr uint8_t CarrierMessageId = 1;
    static constexpr size_t MaxPayloadSize = 128;
    static constexpr uint32_t BridgeTickHz = 20000;
    static constexpr bool UseEstopPins = false;
    static constexpr bool UseSensors = true;
    static constexpr bool UseSensorDma = false;
    static constexpr bool ConfigureRs485Baud = false;
    static constexpr uint32_t Rs485BaudRate = 4000000;
    static constexpr bool DisableRs485TxDmaQueues = false;
    static constexpr bool EnableSessionHeartbeat = true;
    static constexpr uint8_t SessionHeartbeatMessageId = 254;
    static constexpr uint32_t SessionHeartbeatIntervalMs = 20;
};

template <BuildMode Mode>
struct Routes {
    using Config = BoardConfig<Mode>;
    using MessageRoute = realtime_bridge_interface::protocol::MessageRoute;
    using Table =
        realtime_bridge_interface::protocol::MessageRouteTable<Config::MaxRouteNum>;

    Table table;

    Routes()
        : table(make_defaults())
    {
    }

private:
    static std::array<MessageRoute, Config::RouteNum> make_defaults() {
        std::array<MessageRoute, Config::RouteNum> routes{};
        for (size_t i = 0; i < routes.size(); i++) {
            routes[i] = MessageRoute{
                static_cast<uint8_t>(2U + i),
                static_cast<uint8_t>(i),
                static_cast<uint8_t>(Config::MaxPayloadSize)
            };
        }
        return routes;
    }
};

} // namespace stm32_library::boards::prindog
