#include "../Inc/dyn200.hpp"

#ifdef HAL_UART_MODULE_ENABLED

namespace stm32_library::stm32_modules::dyn200 {

Dyn200::Dyn200(stm32_peripherals::Uart& uart, uint8_t slave_address = 0x01)
    : uart_{uart}, slave_address_(slave_address) {
}

void Dyn200::begin(std::span<uint8_t> dma_buffer) {
    dma_buffer_ = dma_buffer;
    uart_.start_receive_dma(dma_buffer_.data(), dma_buffer_.size(), false);
}

bool Dyn200::is_initialized() const {
    return !dma_buffer_.empty();
}

uint16_t Dyn200::crc16_modbus(std::span<const uint8_t> data) {
    uint16_t crc = 0xFFFFU;

    for (const uint8_t byte : data) {
        crc ^= byte;
        for (int i = 0; i < 8; ++i) {
            if ((crc & 0x0001U) != 0U) {
                crc = static_cast<uint16_t>((crc >> 1U) ^ 0xA001U);
            } else {
                crc >>= 1U;
            }
        }
    }

    return crc;
}

uint32_t Dyn200::parse_u32_be(const uint8_t* p) {
    return (static_cast<uint32_t>(p[0]) << 24U) |
           (static_cast<uint32_t>(p[1]) << 16U) |
           (static_cast<uint32_t>(p[2]) << 8U)  |
           (static_cast<uint32_t>(p[3]));
}

int32_t Dyn200::parse_i32_be(const uint8_t* p) {
    return static_cast<int32_t>(parse_u32_be(p));
}

std::array<uint8_t, 8> Dyn200::build_read_request(uint16_t start_address, uint16_t reg_count) const {
    std::array<uint8_t, 8> req{};

    req[0] = slave_address_;
    req[1] = 0x03;
    req[2] = static_cast<uint8_t>((start_address >> 8) & 0xFFU);
    req[3] = static_cast<uint8_t>(start_address & 0xFFU);
    req[4] = static_cast<uint8_t>((reg_count >> 8) & 0xFFU);
    req[5] = static_cast<uint8_t>(reg_count & 0xFFU);

    const uint16_t crc = crc16_modbus(std::span<const uint8_t>{req.data(), 6});
    req[6] = static_cast<uint8_t>(crc & 0xFFU);         // CRC low
    req[7] = static_cast<uint8_t>((crc >> 8) & 0xFFU);  // CRC high

    return req;
}

std::array<uint8_t, 8> Dyn200::build_write_single_request(uint16_t address, uint16_t value) const {
    std::array<uint8_t, 8> req{};

    req[0] = slave_address_;
    req[1] = 0x05;
    req[2] = static_cast<uint8_t>((address >> 8) & 0xFFU);
    req[3] = static_cast<uint8_t>(address & 0xFFU);
    req[4] = static_cast<uint8_t>((value >> 8) & 0xFFU);
    req[5] = static_cast<uint8_t>(value & 0xFFU);

    const uint16_t crc = crc16_modbus(std::span<const uint8_t>{req.data(), 6});
    req[6] = static_cast<uint8_t>(crc & 0xFFU);
    req[7] = static_cast<uint8_t>((crc >> 8) & 0xFFU);

    return req;
}

std::array<uint8_t, 13> Dyn200::build_write_multiple_request(uint16_t start_address, uint32_t value) const {
    std::array<uint8_t, 13> req{};

    req[0]  = slave_address_;
    req[1]  = 0x10;
    req[2]  = static_cast<uint8_t>((start_address >> 8) & 0xFFU);
    req[3]  = static_cast<uint8_t>(start_address & 0xFFU);
    req[4]  = 0x00;
    req[5]  = 0x02;
    req[6]  = 0x04;
    req[7]  = static_cast<uint8_t>((value >> 24) & 0xFFU);
    req[8]  = static_cast<uint8_t>((value >> 16) & 0xFFU);
    req[9]  = static_cast<uint8_t>((value >> 8) & 0xFFU);
    req[10] = static_cast<uint8_t>(value & 0xFFU);

    const uint16_t crc = crc16_modbus(std::span<const uint8_t>{req.data(), 11});
    req[11] = static_cast<uint8_t>(crc & 0xFFU);
    req[12] = static_cast<uint8_t>((crc >> 8) & 0xFFU);

    return req;
}

Error Dyn200::read_long_value(ParameterAddress address, int32_t& out_value) {
    if (!is_initialized()) {
        return Error::BufferNotInitialized;
    }

    const auto req = build_read_request(static_cast<uint16_t>(address), 0x0002);
    uart_.write(std::span<const uint8_t>{req.data(), req.size()});

    std::array<uint8_t, 9> resp{};
    const Error err = receive_read_response(slave_address_, 0x03, resp);
    if (err != Error::None) {
        return err;
    }

    out_value = parse_i32_be(resp.data() + 3);
    return Error::None;
}

Error Dyn200::write_single_command(ParameterAddress address, uint16_t value) {
    if (!is_initialized()) {
        return Error::BufferNotInitialized;
    }

    const auto req = build_write_single_request(static_cast<uint16_t>(address), value);
    uart_.write(std::span<const uint8_t>{req.data(), req.size()});

    std::array<uint8_t, 8> ack{};
    return receive_ack_response(slave_address_, 0x05, ack);
}

Error Dyn200::zero_clear() {
    return write_single_command(ParameterAddress::ZeroClearing, 0xFF00);
}

Error Dyn200::factory_reset() {
    return write_single_command(ParameterAddress::FactoryDataReset, 0xFF00);
}

Error Dyn200::read_parameter(ParameterAddress address, uint32_t& out_value) {
    int32_t raw = 0;
    const Error err = read_long_value(address, raw);
    if (err != Error::None) {
        return err;
    }

    out_value = static_cast<uint32_t>(raw);
    return Error::None;
}

Error Dyn200::write_parameter(ParameterAddress address, uint32_t value) {
    if (!is_initialized()) {
        return Error::BufferNotInitialized;
    }

    const auto req = build_write_multiple_request(static_cast<uint16_t>(address), value);
    uart_.write(std::span<const uint8_t>{req.data(), req.size()});

    std::array<uint8_t, 8> ack{};
    return receive_ack_response(slave_address_, 0x10, ack);
}

Error Dyn200::read_filter(uint32_t& out_value) {
    return read_parameter(ParameterAddress::DigitalFiltering, out_value);
}

Error Dyn200::set_filter(uint32_t value) {
    if (value < 1U || value > 100U) {
        return Error::InvalidArgument;
    }
    return write_parameter(ParameterAddress::DigitalFiltering, value);
}

Error Dyn200::read_decimal_point(uint32_t& out_value) {
    return read_parameter(ParameterAddress::DecimalPoint, out_value);
}

Error Dyn200::set_decimal_point(uint32_t value) {
    if (value > 4U) {
        return Error::InvalidArgument;
    }
    return write_parameter(ParameterAddress::DecimalPoint, value);
}

Error Dyn200::read_zero_reset_on_power(uint32_t& out_value) {
    return read_parameter(ParameterAddress::ZeroResetWhenPowerOn, out_value);
}

Error Dyn200::set_zero_reset_on_power(ZeroResetOnPower mode) {
    return write_parameter(ParameterAddress::ZeroResetWhenPowerOn, static_cast<uint32_t>(mode));
}

Error Dyn200::read_transmit_zero(uint32_t& out_value) {
    return read_parameter(ParameterAddress::TransmitZero, out_value);
}

Error Dyn200::set_transmit_zero(uint32_t value) {
    if (value > 16384U) {
        return Error::InvalidArgument;
    }
    return write_parameter(ParameterAddress::TransmitZero, value);
}

Error Dyn200::read_transmitting_full_range(uint32_t& out_value) {
    return read_parameter(ParameterAddress::TransmittingFullRange, out_value);
}

Error Dyn200::set_transmitting_full_range(uint32_t value) {
    if (value > 16384U) {
        return Error::InvalidArgument;
    }
    return write_parameter(ParameterAddress::TransmittingFullRange, value);
}

Error Dyn200::read_transmitting_range(uint32_t& out_value) {
    return read_parameter(ParameterAddress::TransmittingRange, out_value);
}

Error Dyn200::set_transmitting_range(uint32_t value) {
    if (value < 100U || value > 30000U) {
        return Error::InvalidArgument;
    }
    return write_parameter(ParameterAddress::TransmittingRange, value);
}

Error Dyn200::read_torque_direction(uint32_t& out_value) {
    return read_parameter(ParameterAddress::TorqueDirection, out_value);
}

Error Dyn200::set_torque_direction(TorqueDirection direction) {
    return write_parameter(ParameterAddress::TorqueDirection, static_cast<uint32_t>(direction));
}

Error Dyn200::read_baud_rate(uint32_t& out_value) {
    return read_parameter(ParameterAddress::CommunicationRate, out_value);
}

Error Dyn200::set_baud_rate(BaudRateCode code) {
    return write_parameter(ParameterAddress::CommunicationRate, static_cast<uint32_t>(code));
}

Error Dyn200::read_machine_code(uint32_t& out_value) {
    return read_parameter(ParameterAddress::CommunicationMachineCode, out_value);
}

Error Dyn200::set_machine_code(uint32_t address) {
    if (address > 120U) {
        return Error::InvalidArgument;
    }

    const Error err = write_parameter(ParameterAddress::CommunicationMachineCode, address);
    if (err != Error::None) {
        return err;
    }

    slave_address_ = static_cast<uint8_t>(address);
    return Error::None;
}

Error Dyn200::read_stop_bit(uint32_t& out_value) {
    return read_parameter(ParameterAddress::StopBit, out_value);
}

Error Dyn200::set_stop_bit(StopBit stop_bit) {
    return write_parameter(ParameterAddress::StopBit, static_cast<uint32_t>(stop_bit));
}

Error Dyn200::read_coefficient(uint32_t& out_value) {
    return read_parameter(ParameterAddress::Coefficient, out_value);
}

Error Dyn200::set_coefficient(uint32_t value) {
    if (value < 100U || value > 32700U) {
        return Error::InvalidArgument;
    }
    return write_parameter(ParameterAddress::Coefficient, value);
}

Error Dyn200::try_extract_read_frame(uint8_t slave,
                                           uint8_t function,
                                           std::array<uint8_t, 9>& out_frame,
                                           FrameState& out_state) {
    out_state = FrameState::Empty;

    if (uart_.dma_receive_data_num() < kRxReadFrameSize) {
        return Error::None;
    }

    while (uart_.dma_receive_data_num() >= kRxReadFrameSize) {
        const uint8_t b1 = uart_.dma_receive_data();
        if (b1 != slave) {
            continue;
        }

        const uint8_t b2 = uart_.dma_receive_data();
        if (b2 != function) {
            continue;
        }

        const uint8_t b3 = uart_.dma_receive_data();
        if (b3 != 0x04U) {
            continue;
        }

        out_frame[0] = b1;
        out_frame[1] = b2;
        out_frame[2] = b3;

        for (std::size_t i = 3; i < out_frame.size(); ++i) {
            out_frame[i] = uart_.dma_receive_data();
        }

        const uint16_t calc_crc = crc16_modbus(std::span<const uint8_t>{out_frame.data(), 7});
        const uint16_t recv_crc = static_cast<uint16_t>(out_frame[7]) |
                                  (static_cast<uint16_t>(out_frame[8]) << 8);

        if (calc_crc != recv_crc) {
            return Error::CrcMismatch;
        }

        out_state = FrameState::Ready;
        return Error::None;
    }

    return Error::None;
}

Error Dyn200::send_read_request(ParameterAddress address) {
    if (!is_initialized()) {
        return Error::BufferNotInitialized;
    }

    const auto req = build_read_request(static_cast<uint16_t>(address), 0x0002);
    uart_.write(std::span<const uint8_t>{req.data(), req.size()});
    return Error::None;
}

Error Dyn200::request_torque() {
    const Error err = send_read_request(ParameterAddress::TorqueValue);
    if (err == Error::None) {
        pending_read_ = PendingRead::Torque;
    }
    return err;
}

Error Dyn200::request_speed() {
    const Error err = send_read_request(ParameterAddress::SpeedValue);
    if (err == Error::None) {
        pending_read_ = PendingRead::Speed;
    }
    return err;
}

Error Dyn200::request_power() {
    const Error err = send_read_request(ParameterAddress::PowerDiv10W);
    if (err == Error::None) {
        pending_read_ = PendingRead::Power;
    }
    return err;
}

Error Dyn200::poll_torque_nm(float& out_torque_nm) {
    if (!is_initialized()) {
        return Error::BufferNotInitialized;
    }

    if (pending_read_ != PendingRead::Torque) {
        return Error::InvalidResponse;
    }

    std::array<uint8_t, 9> frame{};
    FrameState state = FrameState::Empty;
    const Error err = try_extract_read_frame(slave_address_, 0x03, frame, state);
    if (err != Error::None) {
        return err;
    }

    if (state != FrameState::Ready) {
        return Error::NotReady;
    }

    const int32_t raw = parse_i32_be(frame.data() + 3);
    out_torque_nm = static_cast<float>(raw) / 100.0f;
    pending_read_ = PendingRead::None;
    return Error::None;
}

Error Dyn200::poll_speed_rpm(float& out_speed_rpm) {
    if (!is_initialized()) {
        return Error::BufferNotInitialized;
    }

    if (pending_read_ != PendingRead::Speed) {
        return Error::InvalidResponse;
    }

    std::array<uint8_t, 9> frame{};
    FrameState state = FrameState::Empty;
    const Error err = try_extract_read_frame(slave_address_, 0x03, frame, state);
    if (err != Error::None) {
        return err;
    }

    if (state != FrameState::Ready) {
        return Error::NotReady;
    }

    const int32_t raw = parse_i32_be(frame.data() + 3);
    out_speed_rpm = static_cast<float>(raw);
    pending_read_ = PendingRead::None;
    return Error::None;
}

Error Dyn200::poll_power_kw(float& out_power_kw) {
    if (!is_initialized()) {
        return Error::BufferNotInitialized;
    }
    
    if (pending_read_ != PendingRead::Power) {
        return Error::InvalidResponse;
    }

    std::array<uint8_t, 9> frame{};
    FrameState state = FrameState::Empty;
    const Error err = try_extract_read_frame(slave_address_, 0x03, frame, state);
    if (err != Error::None) {
        return err;
    }

    if (state != FrameState::Ready) {
        return Error::NotReady;
    }

    const int32_t raw = parse_i32_be(frame.data() + 3);
    out_power_kw = static_cast<float>(raw) / 100.0f;
    pending_read_ = PendingRead::None;
    return Error::None;
}

} // namespace stm32_library::stm32_modules

#endif
