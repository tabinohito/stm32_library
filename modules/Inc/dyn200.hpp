#pragma once

#include "main.h"

#ifdef HAL_UART_MODULE_ENABLED

#include "stm32_library/peripherals/stm32_peripherals.hpp"
#include <optional>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace stm32_library::stm32_modules::dyn200 {

enum class Error : uint8_t {
    None = 0,
    Timeout,
    CrcMismatch,
    InvalidResponse,
    InvalidArgument,
    BufferNotInitialized,
    NotReady,
};

struct Config {

};

struct Measurement {
    float torque_nm{0.0f};
    float speed_rpm{0.0f};
    float power_kw{0.0f};
};

enum class ParameterAddress : uint16_t {
    ZeroClearing             = 0x0000,
    FactoryDataReset         = 0x0002,
    DigitalFiltering         = 0x0006,
    DecimalPoint             = 0x0008,
    ZeroResetWhenPowerOn     = 0x000A,
    TransmitZero             = 0x000C,
    TransmittingFullRange    = 0x000E,
    TransmittingRange        = 0x0010,
    TorqueDirection          = 0x0012,
    CommunicationRate        = 0x0014,
    CommunicationMachineCode = 0x0016,
    StopBit                  = 0x0018,
    Coefficient              = 0x001A,

    TorqueValue              = 0x0000,
    SpeedValue               = 0x0002,
    PowerDiv10W              = 0x0004,
};

enum class ZeroResetOnPower : uint32_t {
    Disable = 0,
    Enable  = 1,
};

enum class StopBit : uint32_t {
    Two = 0,
    One = 1,
};

enum class TorqueDirection : uint32_t {
    Default = 0,
    Inverse = 1,
};

enum class BaudRateCode : uint32_t {
    Baud9600  = 1,
    Baud14400 = 2,
    Baud19200 = 3,
    Baud38400 = 4,
};

enum class PendingRead : uint8_t {
    None = 0,
    Torque,
    Speed,
    Power,
};

class Dyn200 {
public:
    static constexpr std::size_t kRxReadFrameSize = 9;
    static constexpr std::size_t kRxAckFrameSize  = 8;

    Dyn200(stm32_peripherals::Uart& uart, uint8_t slave_address);

    void begin(std::span<uint8_t> dma_buffer);

    Error request_torque();
    Error request_speed();
    Error request_power();

    Error poll_torque_nm(float& out_torque_nm);
    Error poll_speed_rpm(float& out_speed_rpm);
    Error poll_power_kw(float& out_power_kw);

    Error zero_clear();
    Error factory_reset();

    Error read_parameter(ParameterAddress address, uint32_t& out_value);
    Error write_parameter(ParameterAddress address, uint32_t value);

    Error read_filter(uint32_t& out_value);
    Error set_filter(uint32_t value);

    Error read_decimal_point(uint32_t& out_value);
    Error set_decimal_point(uint32_t value);

    Error read_zero_reset_on_power(uint32_t& out_value);
    Error set_zero_reset_on_power(ZeroResetOnPower mode);

    Error read_transmit_zero(uint32_t& out_value);
    Error set_transmit_zero(uint32_t value);

    Error read_transmitting_full_range(uint32_t& out_value);
    Error set_transmitting_full_range(uint32_t value);

    Error read_transmitting_range(uint32_t& out_value);
    Error set_transmitting_range(uint32_t value);

    Error read_torque_direction(uint32_t& out_value);
    Error set_torque_direction(TorqueDirection direction);

    Error read_baud_rate(uint32_t& out_value);
    Error set_baud_rate(BaudRateCode code);

    Error read_machine_code(uint32_t& out_value);
    Error set_machine_code(uint32_t address);

    Error read_stop_bit(uint32_t& out_value);
    Error set_stop_bit(StopBit stop_bit);

    Error read_coefficient(uint32_t& out_value);
    Error set_coefficient(uint32_t value);

    Error read_long_value(ParameterAddress address, int32_t& out_value);
    Error receive_read_response(uint8_t slave, uint8_t function, std::array<uint8_t, 9>& out_frame);
    Error receive_ack_response(uint8_t slave, uint8_t function, std::array<uint8_t, 8>& out_frame);


private:
    enum class FrameState : uint8_t {
        Empty = 0,
        Ready,
    };

    stm32_peripherals::Uart& uart_;
    std::span<uint8_t> dma_buffer_{};
    PendingRead pending_read_{PendingRead::None};
    uint8_t slave_address_;

    bool is_initialized() const;

    static uint16_t crc16_modbus(std::span<const uint8_t> data);
    static uint32_t parse_u32_be(const uint8_t* p);
    static int32_t parse_i32_be(const uint8_t* p);

    std::array<uint8_t, 8> build_read_request(uint16_t start_address, uint16_t reg_count) const;
    std::array<uint8_t, 8> build_write_single_request(uint16_t address, uint16_t value) const;
    std::array<uint8_t, 13> build_write_multiple_request(uint16_t start_address, uint32_t value) const;

    Error write_single_command(ParameterAddress address, uint16_t value);
    Error send_read_request(ParameterAddress address);

    Error try_extract_read_frame(uint8_t slave,
                                 uint8_t function,
                                 std::array<uint8_t, 9>& out_frame,
                                 FrameState& out_state);
};

} // namespace dyn200
#endif
