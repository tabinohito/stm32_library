/*
 * prindog_acyclic.hpp
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <algorithm>

#include "realtime_bridge/components/sensor_types.hpp"
#include "realtime_bridge/protocol/acyclic_byte_buffer.hpp"

namespace stm32_library::boards::prindog {

struct AcyclicMap {
    static constexpr size_t voltage_ch0_V = 0;
    static constexpr size_t current_ch0_A = 2;
    static constexpr size_t fet_temp_ch0_cdeg = 4;

    static constexpr size_t voltage_ch1_V = 6;
    static constexpr size_t current_ch1_A = 8;
    static constexpr size_t fet_temp_ch1_cdeg = 10;

    static constexpr size_t voltage_ch2_V = 12;
    static constexpr size_t current_ch2_A = 14;
    static constexpr size_t fet_temp_ch2_cdeg = 16;

    static constexpr size_t adc_ref_mV = 18;
    static constexpr size_t board_temp_cdeg = 20;

    static constexpr size_t size = 22;
};

using PrindogAcyclicBuffer =
    realtime_bridge::protocol::AcyclicByteBuffer<AcyclicMap::size>;

inline uint16_t clamp_u16_from_float(float value) {
    if (value <= 0.0f) {
        return 0;
    }

    if (value >= 65535.0f) {
        return 65535;
    }

    return static_cast<uint16_t>(value);
}

inline int16_t clamp_i16_from_float(float value) {
    if (value <= -32768.0f) {
        return -32768;
    }

    if (value >= 32767.0f) {
        return 32767;
    }

    return static_cast<int16_t>(value);
}

template <size_t ChannelNum>
inline void update_acyclic_buffer_from_sensor_snapshot(
    const realtime_bridge::components::SensorSnapshot<ChannelNum>& snapshot,
    PrindogAcyclicBuffer& buffer
) {
    static_assert(ChannelNum >= 3, "Prindog requires at least 3 sensor channels");

    const auto& ch0 = snapshot.channels[0];
    const auto& ch1 = snapshot.channels[1];
    const auto& ch2 = snapshot.channels[2];

    buffer.write_u16_le(
        AcyclicMap::voltage_ch0_V,
        clamp_u16_from_float(ch0.bus_voltage_V * 1000.0f)
    );

    buffer.write_u16_le(
        AcyclicMap::current_ch0_A,
        clamp_u16_from_float(ch0.current_A)
    );

    buffer.write_i16_le(
        AcyclicMap::fet_temp_ch0_cdeg,
        clamp_i16_from_float(ch0.ina_temp_C * 100.0f)
    );

    buffer.write_u16_le(
        AcyclicMap::voltage_ch1_V,
        clamp_u16_from_float(ch1.bus_voltage_V * 1000.0f)
    );

    buffer.write_u16_le(
        AcyclicMap::current_ch1_A,
        clamp_u16_from_float(ch1.current_A)
    );

    buffer.write_i16_le(
        AcyclicMap::fet_temp_ch1_cdeg,
        clamp_i16_from_float(ch1.ina_temp_C * 100.0f)
    );

    buffer.write_u16_le(
        AcyclicMap::voltage_ch2_V,
        clamp_u16_from_float(ch2.bus_voltage_V * 1000.0f)
    );

    buffer.write_u16_le(
        AcyclicMap::current_ch2_A,
        clamp_u16_from_float(ch2.current_A)
    );

    buffer.write_i16_le(
        AcyclicMap::fet_temp_ch2_cdeg,
        clamp_i16_from_float(ch2.ina_temp_C * 100.0f)
    );

    // とりあえずMCP3204のch0をADC基準/代表値として入れる。
    // 必要なら別の意味に差し替え。
    buffer.write_u16_le(
        AcyclicMap::adc_ref_mV,
        ch0.thermistor_mV
    );

    // board_tempは現状センサ定義がまだ曖昧なので、ch0 thermistor rawを仮置き。
    // 後でサーミスタ変換式に差し替える。
    buffer.write_i16_le(
        AcyclicMap::board_temp_cdeg,
        static_cast<int16_t>(ch0.thermistor_raw)
    );
}

} // namespace stm32_library::boards::prindog
