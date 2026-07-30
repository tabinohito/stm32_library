#pragma once

/*
 * Prindog board composition.
 *
 * Profiles, physical communication ports, sensor/acyclic publishing and HAL
 * setup are separate policies.  Other boards can reuse the bridge core without
 * including this file or reproducing this member layout.
 */

#include <cstddef>

#include "stm32_library/boards/prindog/platform.hpp"
#include "stm32_library/boards/prindog/acyclic.hpp"
#include "stm32_library/boards/prindog/default_config.hpp"
#include "stm32_library/boards/prindog/ports.hpp"
#include "stm32_library/boards/prindog/profiles.hpp"
#include "stm32_library/boards/prindog/runtime_config_adapter.hpp"
#include "stm32_library/boards/prindog/sensor_service.hpp"
#include "realtime_bridge_interface/components/sensor_service.hpp"
#include "stm32_library/boards/prindog/hardware.hpp"

namespace stm32_library::boards::prindog {

template <BuildMode Mode>
class Board {
public:
    using Config = BoardConfig<Mode>;

    static constexpr size_t ChannelNum = 3;
    static constexpr BuildMode mode = Mode;
    static constexpr size_t PortNum = Config::PortNum;
    static constexpr size_t RouteNum = Config::MaxRouteNum;
    static constexpr size_t AcyclicSize = AcyclicMap::size;
    static constexpr uint8_t CarrierMessageId = Config::CarrierMessageId;
    static constexpr size_t MaxPayloadSize = Config::MaxPayloadSize;
    static constexpr uint32_t BridgeTickHz = Config::BridgeTickHz;
    static constexpr bool SensorDmaEnabled = Config::UseSensorDma;
    static constexpr bool EnableSessionHeartbeat =
        Config::EnableSessionHeartbeat;
    static constexpr uint8_t SessionHeartbeatMessageId =
        Config::SessionHeartbeatMessageId;
    static constexpr uint32_t SessionHeartbeatIntervalMs =
        Config::SessionHeartbeatIntervalMs;

    stm32_library::boards::prindog::Peripherals& peripherals;
    CommPorts<Mode> comm_ports;
    Routes<Mode> routes;
    PrindogAcyclicBuffer acyclic_buffer;

    Board(
        stm32_library::boards::prindog::Peripherals& board_peripherals,
        stm32_library::boards::prindog::Modules& modules
    )
        : peripherals(board_peripherals),
          comm_ports(board_peripherals),
          routes(),
          acyclic_buffer(),
          sensor_service_(modules, acyclic_buffer),
          sensors_(
              realtime_bridge_interface::components::SensorServiceRef::from(
                  sensor_service_
              )
          )
    {
    }

    void setup() {
        PrindogPlatformSetup::setup_or_fail<Config>(peripherals);
        if constexpr (Mode == BuildMode::Unified) {
            if (!runtime_config_applied_) {
                comm_ports.setup();
            }
        } else {
            comm_ports.setup();
        }

        for (size_t i = 0; i < comm_ports.registry.size(); i++) {
            comm_ports.registry[i].safety().set_emergency(true);
            comm_ports.registry[i].safety().set_estop(true);
        }

        sensors_.update_all();
    }

    void update_sensors_slow() {
        sensors_.update_all();
    }

    void update_sensors_slow_step() {
        sensors_.update_step();
    }

    void start_sensors_slow_step_dma() {
        sensors_.start_step_dma();
    }

    void poll_sensors_slow_step_dma() {
        sensors_.poll_step_dma();
    }

    bool sensors_slow_step_dma_busy() const {
        return sensors_.dma_busy();
    }

    size_t sensor_step_index() const {
        return sensors_.step_index();
    }

    static auto factory_port_settings() {
        return make_factory_port_settings();
    }

    static auto factory_feature_settings() {
        return make_factory_feature_settings();
    }

    bool apply_runtime_config(
        const realtime_bridge_interface::config::BridgeRuntimeConfig& config
    ) {
        constexpr uint8_t ValidSensorMask =
            static_cast<uint8_t>((1U << ChannelNum) - 1U);
        if (
            (config.features.current_sensor_mask & ~ValidSensorMask) != 0 ||
            (config.features.thermistor_sensor_mask & ~ValidSensorMask) != 0
        ) {
            return false;
        }

        if (!PrindogRuntimeConfigAdapter::apply_ports(
                config,
                peripherals,
                comm_ports.registry
            )) {
            return false;
        }

        features_ = config.features;
        runtime_config_applied_ = true;
        sensor_service_.set_enabled_masks(
            features_.current_sensor_mask,
            features_.thermistor_sensor_mask
        );

        const bool initial_emergency =
            features_.emergency_enabled &&
            !features_.acyclic_emergency_enabled;
        set_emergency_outputs(initial_emergency);
        return true;
    }

    void handle_acyclic_rx(uint8_t index, uint8_t value) {
        if (
            !features_.emergency_enabled ||
            !features_.acyclic_emergency_enabled ||
            index != features_.acyclic_emergency_index
        ) {
            return;
        }

        set_emergency_outputs(
            value == features_.acyclic_emergency_active_value
        );
    }

    bool sensors_enabled() const {
        return sensor_service_.enabled();
    }

private:
    void set_emergency_outputs(bool asserted) {
        for (auto& output : peripherals.emergency) {
            output = asserted ? 1 : 0;
        }
    }

    PrindogSensorService<Config, ChannelNum> sensor_service_;
    realtime_bridge_interface::components::SensorServiceRef sensors_;
    realtime_bridge_interface::config::BoardFeatureSettings features_{};
    bool runtime_config_applied_ = false;
};

using UsbDebugBoard = Board<BuildMode::UsbDebug>;
using ProductionBoard = Board<BuildMode::Production>;
using DynamixelBoard = Board<BuildMode::Dynamixel>;
using CanBridgeBoard = Board<BuildMode::CanBridge>;
using UnifiedBoard = Board<BuildMode::Unified>;

} // namespace stm32_library::boards::prindog
