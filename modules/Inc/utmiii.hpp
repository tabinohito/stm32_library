#pragma once

#include "main.h"

#ifdef HAL_UART_MODULE_ENABLED

#include "stm32_library/peripherals/stm32_peripherals.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace stm32_library::stm32_modules::utmiii {

enum class Error : uint8_t {
    None = 0,
    BufferNotInitialized,
    NotReady,
    InvalidResponse,
    InvalidArgument,
    Busy,
};

struct Config {
    std::size_t max_line_length{64};
};

struct Status {
    bool temperature_over_high{false};
    bool temperature_over_low{false};
    bool speed_over{false};
    bool over_torque_plus{false};
    bool over_torque_minus{false};
    bool power_error{false};
    bool communication_error{false};
};

struct BulkA {
    float torque_filtered_percent_fs{0.0f};
    float speed_rpm{0.0f};
    int temperature_c{0};
};

struct BulkB {
    float torque_unfiltered_percent_fs{0.0f};
    float speed_rpm{0.0f};
    int temperature_c{0};
};

enum class PendingCommand : uint8_t {
    None = 0,
    RA,
    RB,
    RD,
    RE,
    RH,
    RI,
    RJ,
    RK,
};

enum class TorqueFilter : uint8_t {
    Hz1   = 0,
    Hz3   = 1,
    Hz10  = 2,
    Hz30  = 3,
    Hz100 = 4,
    Hz300 = 5,
    Hz1k  = 6,
    Pass  = 7,
};

enum class SpeedFilter : uint8_t {
    Off = 0,
    Avg2 = 1,
    Avg4 = 2,
    Avg8 = 3,
    Avg16 = 4,
    Avg32 = 5,
};

class UtmIii {
public:
    explicit UtmIii(stm32_peripherals::Uart& uart, std::size_t max_line_length);

    void begin(std::span<uint8_t> dma_buffer);

    Error request_torque_filtered();
    Error request_speed();
    Error request_temperature();
    Error request_status();
    Error request_torque_both();
    Error request_bulk_a();
    Error request_torque_unfiltered();
    Error request_bulk_b();

    Error poll_torque_filtered_percent_fs(float& out_value);
    Error poll_speed_rpm(float& out_value);
    Error poll_temperature_c(int& out_value);
    Error poll_status(Status& out_status);
    Error poll_torque_both_percent_fs(float& out_filtered, float& out_unfiltered);
    Error poll_bulk_a(BulkA& out_value);
    Error poll_torque_unfiltered_percent_fs(float& out_value);
    Error poll_bulk_b(BulkB& out_value);

    Error set_error_response(bool enabled);
    Error set_torque_filter(TorqueFilter filter);
    Error set_speed_filter(SpeedFilter filter);
    Error set_min_display_speed_rpm(uint8_t rpm);
    Error digital_zero();
    Error digital_zero_reset();

private:
    static constexpr char kTerminator = '\r';

    stm32_peripherals::Uart& uart_;
    std::size_t max_line_length_;
    std::span<uint8_t> dma_buffer_{};
    PendingCommand pending_{PendingCommand::None};

    bool is_initialized() const;

    Error send_ascii_command(std::span<const char> command, PendingCommand pending);
    Error send_ascii_command_no_wait(std::span<const char> command);
    Error read_line(char* out_buf, std::size_t out_buf_size, std::size_t& out_len);

    static bool parse_signed_float(const char* str, std::size_t len, float& out_value);
    static bool parse_signed_int(const char* str, std::size_t len, int& out_value);
    static bool split_once(const char* str,
                           std::size_t len,
                           char delimiter,
                           const char*& left_ptr,
                           std::size_t& left_len,
                           const char*& right_ptr,
                           std::size_t& right_len);
};

}// namespace stm32_library::stm32_modules

#endif
