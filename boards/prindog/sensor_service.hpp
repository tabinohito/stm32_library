#pragma once

#include <array>
#include <cstddef>

#include "realtime_bridge/adapters/stm32/common/sensor_hub.hpp"
#include "stm32_library/boards/prindog/acyclic.hpp"

namespace stm32_library::boards::prindog {

/*
 * Prindog-specific composition of physical sensors and acyclic encoding.
 *
 * The generic runner sees only SensorServiceRef. INA238/MCP3204 types and the
 * Prindog byte map are confined to this board adapter.
 */
template <typename Config, size_t ChannelNum>
class PrindogSensorService {
public:
    using MuxChannel =
        stm32_library::stm32_modules::MCP320xTypes::MCP3204::Channel;
    using Hub =
        realtime_bridge::adapters::stm32::common::
            SensorHub<ChannelNum>;

    PrindogSensorService(
        stm32_library::boards::prindog::Modules& modules,
        PrindogAcyclicBuffer& acyclic_buffer
    )
        : thermistor_channels_{{
              stm32_library::stm32_modules::MCP320xTypes::MCP3204::SINGLE_0,
              stm32_library::stm32_modules::MCP320xTypes::MCP3204::SINGLE_1,
              stm32_library::stm32_modules::MCP320xTypes::MCP3204::SINGLE_2,
          }},
          hub_(
              modules.ina238,
              *modules.mcp3204,
              thermistor_channels_
          ),
          acyclic_buffer_(acyclic_buffer)
    {
        static_assert(ChannelNum == 3, "Prindog sensor wiring has 3 channels");
    }

    void update_all() {
        if constexpr (Config::UseSensors) {
            hub_.update();
            publish();
        }
    }

    void update_step() {
        if constexpr (Config::UseSensors) {
            hub_.update_step();
            publish_if_cycle_complete();
        }
    }

    void start_step_dma() {
        if constexpr (Config::UseSensors && Config::UseSensorDma) {
            hub_.start_step_dma();
            publish_if_cycle_complete();
        }
    }

    void poll_step_dma() {
        if constexpr (Config::UseSensors && Config::UseSensorDma) {
            if (hub_.poll_step_dma()) {
                publish_if_cycle_complete();
            }
        }
    }

    bool dma_busy() const {
        if constexpr (Config::UseSensors && Config::UseSensorDma) {
            return hub_.step_dma_busy();
        }
        return false;
    }

    size_t step_index() const {
        if constexpr (Config::UseSensors) {
            return hub_.step_index();
        }
        return 0U;
    }

private:
    void publish_if_cycle_complete() {
        if (hub_.step_cycle_completed()) {
            publish();
        }
    }

    void publish() {
        update_acyclic_buffer_from_sensor_snapshot(
            hub_.latest(),
            acyclic_buffer_
        );
    }

    std::array<MuxChannel, ChannelNum> thermistor_channels_{};
    Hub hub_;
    PrindogAcyclicBuffer& acyclic_buffer_;
};

} // namespace stm32_library::boards::prindog
