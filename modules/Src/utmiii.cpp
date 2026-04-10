#include "../Inc/utmiii.hpp"

namespace stm32_library::stm32_modules::utmiii {

namespace {

const char* pending_command_prefix(PendingCommand pending) {
    switch (pending) {
    case PendingCommand::RA:  return "RA";
    case PendingCommand::RB:  return "RB";
    case PendingCommand::RD:  return "RD";
    case PendingCommand::RE:  return "RE";
    case PendingCommand::RH:  return "RH";
    case PendingCommand::RI:  return "RI";
    case PendingCommand::RJ:  return "RJ";
    case PendingCommand::RK:  return "RK";
    case PendingCommand::W32: return "W32";
    case PendingCommand::W41: return "W41";
    case PendingCommand::W42: return "W42";
    default:                  return "";
    }
}

bool starts_with_prefix(const char* str, std::size_t len, const char* prefix) {
    if (prefix == nullptr) {
        return false;
    }
    if (prefix[0] == '\0') {
        return true;
    }

    std::size_t prefix_len = 0;
    while (prefix[prefix_len] != '\0') {
        ++prefix_len;
    }

    if (len < prefix_len) {
        return false;
    }

    for (std::size_t i = 0; i < prefix_len; ++i) {
        if (str[i] != prefix[i]) {
            return false;
        }
    }
    return true;
}

void strip_prefix_if_present(PendingCommand pending, const char*& ptr, std::size_t& len) {
    const char* prefix = pending_command_prefix(pending);
    if (prefix == nullptr) {
        return;
    }

    std::size_t prefix_len = 0;
    while (prefix[prefix_len] != '\0') {
        ++prefix_len;
    }

    if (starts_with_prefix(ptr, len, prefix)) {
        ptr += prefix_len;
        len -= prefix_len;
    }
}

} // namespace

UtmIii::UtmIii(stm32_peripherals::Uart& uart, std::size_t max_line_length)
    : uart_{uart}, max_line_length_(max_line_length) {
}

void UtmIii::begin(std::span<uint8_t> dma_buffer) {
    dma_buffer_ = dma_buffer;
    uart_.start_receive_dma(dma_buffer_.data(), dma_buffer_.size(), false);
}

bool UtmIii::is_initialized() const {
    return !dma_buffer_.empty();
}

void UtmIii::flush_rx() {
    while (uart_.dma_receive_data_num() > 0U) {
        (void)uart_.dma_receive_data();
    }
}

Error UtmIii::send_ascii_command(std::span<const char> command, PendingCommand pending) {
    if (!is_initialized()) {
        return Error::BufferNotInitialized;
    }
    if (pending_ != PendingCommand::None) {
        return Error::Busy;
    }

    uart_.write(std::span<const uint8_t>(
        reinterpret_cast<const uint8_t*>(command.data()), command.size()));

    pending_ = pending;
    return Error::None;
}

Error UtmIii::send_ascii_command_no_wait(std::span<const char> command) {
    if (!is_initialized()) {
        return Error::BufferNotInitialized;
    }

    flush_rx();
    uart_.write(std::span<const uint8_t>(
        reinterpret_cast<const uint8_t*>(command.data()), command.size()));
    return Error::None;
}

Error UtmIii::request_torque_filtered() {
    static constexpr char cmd[] = {'R', 'A', '\r'};
    if (!is_initialized()) {
        return Error::BufferNotInitialized;
    }
    if (pending_ != PendingCommand::None) {
        return Error::Busy;
    }

    flush_rx();
    return send_ascii_command(std::span<const char>{cmd, sizeof(cmd)}, PendingCommand::RA);
}

Error UtmIii::request_speed() {
    static constexpr char cmd[] = {'R', 'B', '\r'};
    if (!is_initialized()) {
        return Error::BufferNotInitialized;
    }
    if (pending_ != PendingCommand::None) {
        return Error::Busy;
    }

    flush_rx();
    return send_ascii_command(std::span<const char>{cmd, sizeof(cmd)}, PendingCommand::RB);
}

Error UtmIii::request_temperature() {
    static constexpr char cmd[] = {'R', 'D', '\r'};
    if (!is_initialized()) {
        return Error::BufferNotInitialized;
    }
    if (pending_ != PendingCommand::None) {
        return Error::Busy;
    }

    flush_rx();
    return send_ascii_command(std::span<const char>{cmd, sizeof(cmd)}, PendingCommand::RD);
}

Error UtmIii::request_status() {
    static constexpr char cmd[] = {'R', 'E', '\r'};
    if (!is_initialized()) {
        return Error::BufferNotInitialized;
    }
    if (pending_ != PendingCommand::None) {
        return Error::Busy;
    }

    flush_rx();
    return send_ascii_command(std::span<const char>{cmd, sizeof(cmd)}, PendingCommand::RE);
}

Error UtmIii::request_torque_both() {
    static constexpr char cmd[] = {'R', 'H', '\r'};
    if (!is_initialized()) {
        return Error::BufferNotInitialized;
    }
    if (pending_ != PendingCommand::None) {
        return Error::Busy;
    }

    flush_rx();
    return send_ascii_command(std::span<const char>{cmd, sizeof(cmd)}, PendingCommand::RH);
}

Error UtmIii::request_bulk_a() {
    static constexpr char cmd[] = {'R', 'I', '\r'};
    if (!is_initialized()) {
        return Error::BufferNotInitialized;
    }
    if (pending_ != PendingCommand::None) {
        return Error::Busy;
    }

    flush_rx();
    return send_ascii_command(std::span<const char>{cmd, sizeof(cmd)}, PendingCommand::RI);
}

Error UtmIii::request_torque_unfiltered() {
    static constexpr char cmd[] = {'R', 'J', '\r'};
    if (!is_initialized()) {
        return Error::BufferNotInitialized;
    }
    if (pending_ != PendingCommand::None) {
        return Error::Busy;
    }

    flush_rx();
    return send_ascii_command(std::span<const char>{cmd, sizeof(cmd)}, PendingCommand::RJ);
}

Error UtmIii::request_bulk_b() {
    static constexpr char cmd[] = {'R', 'K', '\r'};
    if (!is_initialized()) {
        return Error::BufferNotInitialized;
    }
    if (pending_ != PendingCommand::None) {
        return Error::Busy;
    }

    flush_rx();
    return send_ascii_command(std::span<const char>{cmd, sizeof(cmd)}, PendingCommand::RK);
}

Error UtmIii::set_error_response(bool enabled) {
    const char cmd[9] = {'W', '0', '1', '0', '0', '0', '0',
                         static_cast<char>(enabled ? '0' : '1'), '\r'};

    if (!is_initialized()) {
        return Error::BufferNotInitialized;
    }
    if (pending_ != PendingCommand::None) {
        return Error::Busy;
    }

    return send_ascii_command_no_wait(std::span<const char>{cmd, sizeof(cmd)});
}

Error UtmIii::set_torque_filter(TorqueFilter filter) {
    const auto value = static_cast<uint8_t>(filter);
    if (value > 7U) {
        return Error::InvalidArgument;
    }

    const char cmd[9] = {'W', '3', '2', '0', '0', '0', '0',
                         static_cast<char>('0' + value), '\r'};

    if (!is_initialized()) {
        return Error::BufferNotInitialized;
    }
    if (pending_ != PendingCommand::None) {
        return Error::Busy;
    }

    return send_ascii_command_no_wait(std::span<const char>{cmd, sizeof(cmd)});
}

Error UtmIii::set_speed_filter(SpeedFilter filter) {
    const auto value = static_cast<uint8_t>(filter);
    if (value > 5U) {
        return Error::InvalidArgument;
    }

    const char cmd[9] = {'W', '4', '1', '0', '0', '0', '0',
                         static_cast<char>('0' + value), '\r'};

    if (!is_initialized()) {
        return Error::BufferNotInitialized;
    }
    if (pending_ != PendingCommand::None) {
        return Error::Busy;
    }

    return send_ascii_command_no_wait(std::span<const char>{cmd, sizeof(cmd)});
}

Error UtmIii::set_min_display_speed_rpm(uint8_t rpm) {
    if (rpm > 99U) {
        return Error::InvalidArgument;
    }

    const uint8_t tens = static_cast<uint8_t>(rpm / 10U);
    const uint8_t ones = static_cast<uint8_t>(rpm % 10U);

    const char cmd[10] = {
        'W', '4', '2', '0', '0', '0', '0',
        static_cast<char>('0' + tens),
        static_cast<char>('0' + ones),
        '\r'
    };

    if (!is_initialized()) {
        return Error::BufferNotInitialized;
    }
    if (pending_ != PendingCommand::None) {
        return Error::Busy;
    }

    return send_ascii_command_no_wait(std::span<const char>{cmd, sizeof(cmd)});
}

Error UtmIii::digital_zero() {
    static constexpr char cmd[] = {'C', 'G', '\r'};

    if (!is_initialized()) {
        return Error::BufferNotInitialized;
    }
    if (pending_ != PendingCommand::None) {
        return Error::Busy;
    }

    return send_ascii_command_no_wait(std::span<const char>{cmd, sizeof(cmd)});
}

Error UtmIii::digital_zero_reset() {
    static constexpr char cmd[] = {'C', 'H', '\r'};

    if (!is_initialized()) {
        return Error::BufferNotInitialized;
    }
    if (pending_ != PendingCommand::None) {
        return Error::Busy;
    }

    return send_ascii_command_no_wait(std::span<const char>{cmd, sizeof(cmd)});
}

Error UtmIii::read_line(char* out_buf, std::size_t out_buf_size, std::size_t& out_len) {
    if (!is_initialized()) {
        return Error::BufferNotInitialized;
    }
    if (out_buf == nullptr || out_buf_size == 0U) {
        return Error::InvalidArgument;
    }

    out_len = 0;

    while (uart_.dma_receive_data_num() > 0U) {
        const char ch = static_cast<char>(uart_.dma_receive_data());

        if (ch == '\n') {
            continue;
        }

        if (ch == kTerminator) {
            out_buf[out_len] = '\0';
            return Error::None;
        }

        if (out_len + 1U >= out_buf_size) {
            out_len = 0;
            return Error::InvalidResponse;
        }

        out_buf[out_len++] = ch;
    }

    return Error::NotReady;
}

bool UtmIii::parse_signed_float(const char* str, std::size_t len, float& out_value) {
    if (str == nullptr || len == 0U) {
        return false;
    }

    std::size_t i = 0;
    int sign = 1;
    if (str[i] == '+') {
        ++i;
    } else if (str[i] == '-') {
        sign = -1;
        ++i;
    }

    int32_t int_part = 0;
    bool has_digit = false;
    while (i < len && str[i] >= '0' && str[i] <= '9') {
        has_digit = true;
        int_part = static_cast<int32_t>(int_part * 10 + (str[i] - '0'));
        ++i;
    }

    float frac_part = 0.0f;
    float scale = 1.0f;
    if (i < len && str[i] == '.') {
        ++i;
        while (i < len && str[i] >= '0' && str[i] <= '9') {
            has_digit = true;
            scale *= 0.1f;
            frac_part += static_cast<float>(str[i] - '0') * scale;
            ++i;
        }
    }

    if (!has_digit || i != len) {
        return false;
    }

    out_value = static_cast<float>(sign) * (static_cast<float>(int_part) + frac_part);
    return true;
}

bool UtmIii::parse_signed_int(const char* str, std::size_t len, int& out_value) {
    if (str == nullptr || len == 0U) {
        return false;
    }

    std::size_t i = 0;
    int sign = 1;
    if (str[i] == '+') {
        ++i;
    } else if (str[i] == '-') {
        sign = -1;
        ++i;
    }

    int value = 0;
    bool has_digit = false;
    while (i < len && str[i] >= '0' && str[i] <= '9') {
        has_digit = true;
        value = value * 10 + (str[i] - '0');
        ++i;
    }

    if (!has_digit || i != len) {
        return false;
    }

    out_value = sign * value;
    return true;
}

bool UtmIii::parse_unsigned_decimal(const char* str, std::size_t len, uint32_t& out_value) {
    if (str == nullptr || len == 0U) {
        return false;
    }

    uint32_t value = 0;
    for (std::size_t i = 0; i < len; ++i) {
        if (str[i] < '0' || str[i] > '9') {
            return false;
        }
        value = value * 10U + static_cast<uint32_t>(str[i] - '0');
    }

    out_value = value;
    return true;
}

bool UtmIii::strip_fixed_read_payload(PendingCommand pending,
                              const char* line,
                              std::size_t len,
                              const char*& value_ptr,
                              std::size_t& value_len) {
    value_ptr = line;
    value_len = len;

    strip_prefix_if_present(pending, value_ptr, value_len);

    return value_len > 0U;
}

bool UtmIii::split_once(const char* str,
                        std::size_t len,
                        char delimiter,
                        const char*& left_ptr,
                        std::size_t& left_len,
                        const char*& right_ptr,
                        std::size_t& right_len) {
    for (std::size_t i = 0; i < len; ++i) {
        if (str[i] == delimiter) {
            left_ptr = str;
            left_len = i;
            right_ptr = str + i + 1U;
            right_len = len - i - 1U;
            return true;
        }
    }
    return false;
}

Error UtmIii::poll_torque_filtered_percent_fs(float& out_value) {
    if (pending_ != PendingCommand::RA) {
        return Error::InvalidResponse;
    }

    char line[64]{};
    std::size_t len = 0;
    const Error err = read_line(line, sizeof(line), len);
    if (err != Error::None) {
        return err;
    }

    if (len == 2U && line[0] == 'E' && line[1] == 'R') {
        pending_ = PendingCommand::None;
        return Error::InvalidResponse;
    }

    const char* value_ptr = line;
    std::size_t value_len = len;
    strip_prefix_if_present(PendingCommand::RA, value_ptr, value_len);

    if (!parse_signed_float(value_ptr, value_len, out_value)) {
        pending_ = PendingCommand::None;
        return Error::InvalidResponse;
    }

    pending_ = PendingCommand::None;
    return Error::None;
}

Error UtmIii::poll_speed_rpm(float& out_value) {
    if (pending_ != PendingCommand::RB) {
        return Error::InvalidResponse;
    }

    char line[64]{};
    std::size_t len = 0;
    const Error err = read_line(line, sizeof(line), len);
    if (err != Error::None) {
        return err;
    }

    if (len == 2U && line[0] == 'E' && line[1] == 'R') {
        pending_ = PendingCommand::None;
        return Error::InvalidResponse;
    }

    const char* value_ptr = line;
    std::size_t value_len = len;
    strip_prefix_if_present(PendingCommand::RB, value_ptr, value_len);

    if (!parse_signed_float(value_ptr, value_len, out_value)) {
        pending_ = PendingCommand::None;
        return Error::InvalidResponse;
    }

    pending_ = PendingCommand::None;
    return Error::None;
}

Error UtmIii::poll_temperature_c(int& out_value) {
    if (pending_ != PendingCommand::RD) {
        return Error::InvalidResponse;
    }

    char line[64]{};
    std::size_t len = 0;
    const Error err = read_line(line, sizeof(line), len);
    if (err != Error::None) {
        return err;
    }

    if (len == 2U && line[0] == 'E' && line[1] == 'R') {
        pending_ = PendingCommand::None;
        return Error::InvalidResponse;
    }

    const char* value_ptr = line;
    std::size_t value_len = len;
    strip_prefix_if_present(PendingCommand::RD, value_ptr, value_len);

    if (!parse_signed_int(value_ptr, value_len, out_value)) {
        pending_ = PendingCommand::None;
        return Error::InvalidResponse;
    }

    pending_ = PendingCommand::None;
    return Error::None;
}

Error UtmIii::poll_status(Status& out_status) {
    if (pending_ != PendingCommand::RE) {
        return Error::InvalidResponse;
    }

    char line[64]{};
    std::size_t len = 0;
    const Error err = read_line(line, sizeof(line), len);
    if (err != Error::None) {
        return err;
    }

    if (len == 2U && line[0] == 'E' && line[1] == 'R') {
        pending_ = PendingCommand::None;
        return Error::InvalidResponse;
    }

    const char* value_ptr = line;
    std::size_t value_len = len;
    strip_prefix_if_present(PendingCommand::RE, value_ptr, value_len);

    if (value_len != 7U) {
        pending_ = PendingCommand::None;
        return Error::InvalidResponse;
    }

    out_status.temperature_over_high = (value_ptr[0] == '1');
    out_status.temperature_over_low  = (value_ptr[1] == '1');
    out_status.speed_over            = (value_ptr[2] == '1');
    out_status.over_torque_plus      = (value_ptr[3] == '1');
    out_status.over_torque_minus     = (value_ptr[4] == '1');
    out_status.power_error           = (value_ptr[5] == '1');
    out_status.communication_error   = (value_ptr[6] == '1');

    pending_ = PendingCommand::None;
    return Error::None;
}

Error UtmIii::poll_torque_both_percent_fs(float& out_filtered, float& out_unfiltered) {
    if (pending_ != PendingCommand::RH) {
        return Error::InvalidResponse;
    }

    char line[64]{};
    std::size_t len = 0;
    const Error err = read_line(line, sizeof(line), len);
    if (err != Error::None) {
        return err;
    }

    if (len == 2U && line[0] == 'E' && line[1] == 'R') {
        pending_ = PendingCommand::None;
        return Error::InvalidResponse;
    }

    const char* value_ptr = line;
    std::size_t value_len = len;
    strip_prefix_if_present(PendingCommand::RH, value_ptr, value_len);

    const char* left = nullptr;
    const char* right = nullptr;
    std::size_t left_len = 0;
    std::size_t right_len = 0;
    if (!split_once(value_ptr, value_len, ',', left, left_len, right, right_len)) {
        pending_ = PendingCommand::None;
        return Error::InvalidResponse;
    }

    if (!parse_signed_float(left, left_len, out_filtered)) {
        pending_ = PendingCommand::None;
        return Error::InvalidResponse;
    }
    if (!parse_signed_float(right, right_len, out_unfiltered)) {
        pending_ = PendingCommand::None;
        return Error::InvalidResponse;
    }

    pending_ = PendingCommand::None;
    return Error::None;
}

Error UtmIii::poll_bulk_a(BulkA& out_value) {
    if (pending_ != PendingCommand::RI) {
        return Error::InvalidResponse;
    }

    char line[64]{};
    std::size_t len = 0;
    const Error err = read_line(line, sizeof(line), len);
    if (err != Error::None) {
        return err;
    }

    if (len == 2U && line[0] == 'E' && line[1] == 'R') {
        pending_ = PendingCommand::None;
        return Error::InvalidResponse;
    }

    const char* value_ptr = line;
    std::size_t value_len = len;
    strip_prefix_if_present(PendingCommand::RI, value_ptr, value_len);

    const char* p1 = nullptr;
    const char* p2 = nullptr;
    std::size_t l1 = 0;
    std::size_t l2 = 0;
    if (!split_once(value_ptr, value_len, ',', p1, l1, p2, l2)) {
        pending_ = PendingCommand::None;
        return Error::InvalidResponse;
    }

    const char* p3 = nullptr;
    const char* p4 = nullptr;
    std::size_t l3 = 0;
    std::size_t l4 = 0;
    if (!split_once(p2, l2, ',', p3, l3, p4, l4)) {
        pending_ = PendingCommand::None;
        return Error::InvalidResponse;
    }

    if (!parse_signed_float(p1, l1, out_value.torque_filtered_percent_fs)) {
        pending_ = PendingCommand::None;
        return Error::InvalidResponse;
    }
    if (!parse_signed_float(p3, l3, out_value.speed_rpm)) {
        pending_ = PendingCommand::None;
        return Error::InvalidResponse;
    }
    if (!parse_signed_int(p4, l4, out_value.temperature_c)) {
        pending_ = PendingCommand::None;
        return Error::InvalidResponse;
    }

    pending_ = PendingCommand::None;
    return Error::None;
}

Error UtmIii::poll_torque_unfiltered_percent_fs(float& out_value) {
    if (pending_ != PendingCommand::RJ) {
        return Error::InvalidResponse;
    }

    char line[64]{};
    std::size_t len = 0;
    const Error err = read_line(line, sizeof(line), len);
    if (err != Error::None) {
        return err;
    }

    if (len == 2U && line[0] == 'E' && line[1] == 'R') {
        pending_ = PendingCommand::None;
        return Error::InvalidResponse;
    }

    const char* value_ptr = line;
    std::size_t value_len = len;
    strip_prefix_if_present(PendingCommand::RJ, value_ptr, value_len);

    if (!parse_signed_float(value_ptr, value_len, out_value)) {
        pending_ = PendingCommand::None;
        return Error::InvalidResponse;
    }

    pending_ = PendingCommand::None;
    return Error::None;
}

Error UtmIii::poll_bulk_b(BulkB& out_value) {
    if (pending_ != PendingCommand::RK) {
        return Error::InvalidResponse;
    }

    char line[64]{};
    std::size_t len = 0;
    const Error err = read_line(line, sizeof(line), len);
    if (err != Error::None) {
        return err;
    }

    if (len == 2U && line[0] == 'E' && line[1] == 'R') {
        pending_ = PendingCommand::None;
        return Error::InvalidResponse;
    }

    const char* value_ptr = line;
    std::size_t value_len = len;
    strip_prefix_if_present(PendingCommand::RK, value_ptr, value_len);

    const char* p1 = nullptr;
    const char* p2 = nullptr;
    std::size_t l1 = 0;
    std::size_t l2 = 0;
    if (!split_once(value_ptr, value_len, ',', p1, l1, p2, l2)) {
        pending_ = PendingCommand::None;
        return Error::InvalidResponse;
    }

    const char* p3 = nullptr;
    const char* p4 = nullptr;
    std::size_t l3 = 0;
    std::size_t l4 = 0;
    if (!split_once(p2, l2, ',', p3, l3, p4, l4)) {
        pending_ = PendingCommand::None;
        return Error::InvalidResponse;
    }

    if (!parse_signed_float(p1, l1, out_value.torque_unfiltered_percent_fs)) {
        pending_ = PendingCommand::None;
        return Error::InvalidResponse;
    }
    if (!parse_signed_float(p3, l3, out_value.speed_rpm)) {
        pending_ = PendingCommand::None;
        return Error::InvalidResponse;
    }
    if (!parse_signed_int(p4, l4, out_value.temperature_c)) {
        pending_ = PendingCommand::None;
        return Error::InvalidResponse;
    }

    pending_ = PendingCommand::None;
    return Error::None;
}

Error UtmIii::poll_torque_filter(TorqueFilter& out_value) {
    if (pending_ != PendingCommand::W32) {
        return Error::InvalidResponse;
    }

    char line[64]{};
    std::size_t len = 0;
    const Error err = read_line(line, sizeof(line), len);
    if (err != Error::None) {
        return err;
    }

    if (len == 2U && line[0] == 'E' && line[1] == 'R') {
        pending_ = PendingCommand::None;
        return Error::InvalidResponse;
    }

    const char* value_ptr = nullptr;
    std::size_t value_len = 0;
    if (!strip_fixed_read_payload(PendingCommand::W32, line, len, value_ptr, value_len)) {
        pending_ = PendingCommand::None;
        return Error::InvalidResponse;
    }

    uint32_t raw = 0;
    if (!parse_unsigned_decimal(value_ptr, value_len, raw) || raw > 7U) {
        pending_ = PendingCommand::None;
        return Error::InvalidResponse;
    }

    out_value = static_cast<TorqueFilter>(raw);
    pending_ = PendingCommand::None;
    return Error::None;
}

Error UtmIii::poll_speed_filter(SpeedFilter& out_value) {
    if (pending_ != PendingCommand::W41) {
        return Error::InvalidResponse;
    }

    char line[64]{};
    std::size_t len = 0;
    const Error err = read_line(line, sizeof(line), len);
    if (err != Error::None) {
        return err;
    }

    if (len == 2U && line[0] == 'E' && line[1] == 'R') {
        pending_ = PendingCommand::None;
        return Error::InvalidResponse;
    }

    const char* value_ptr = nullptr;
    std::size_t value_len = 0;
    if (!strip_fixed_read_payload(PendingCommand::W41, line, len, value_ptr, value_len)) {
        pending_ = PendingCommand::None;
        return Error::InvalidResponse;
    }

    uint32_t raw = 0;
    if (!parse_unsigned_decimal(value_ptr, value_len, raw) || raw > 5U) {
        pending_ = PendingCommand::None;
        return Error::InvalidResponse;
    }

    out_value = static_cast<SpeedFilter>(raw);
    pending_ = PendingCommand::None;
    return Error::None;
}

Error UtmIii::poll_min_display_speed_rpm(uint8_t& out_value) {
    if (pending_ != PendingCommand::W42) {
        return Error::InvalidResponse;
    }

    char line[64]{};
    std::size_t len = 0;
    const Error err = read_line(line, sizeof(line), len);
    if (err != Error::None) {
        return err;
    }

    if (len == 2U && line[0] == 'E' && line[1] == 'R') {
        pending_ = PendingCommand::None;
        return Error::InvalidResponse;
    }

    const char* value_ptr = nullptr;
    std::size_t value_len = 0;
    if (!strip_fixed_read_payload(PendingCommand::W42, line, len, value_ptr, value_len)) {
        pending_ = PendingCommand::None;
        return Error::InvalidResponse;
    }

    uint32_t raw = 0;
    if (!parse_unsigned_decimal(value_ptr, value_len, raw) || raw > 99U) {
        pending_ = PendingCommand::None;
        return Error::InvalidResponse;
    }

    out_value = static_cast<uint8_t>(raw);
    pending_ = PendingCommand::None;
    return Error::None;
}

Error UtmIii::request_read_torque_filter() {
    static constexpr char cmd[] = {'W', '3', '2', '\r'};
    if (!is_initialized()) {
        return Error::BufferNotInitialized;
    }
    if (pending_ != PendingCommand::None) {
        return Error::Busy;
    }

    flush_rx();
    return send_ascii_command(std::span<const char>{cmd, sizeof(cmd)}, PendingCommand::W32);
}

Error UtmIii::request_read_speed_filter() {
    static constexpr char cmd[] = {'W', '4', '1', '\r'};
    if (!is_initialized()) {
        return Error::BufferNotInitialized;
    }
    if (pending_ != PendingCommand::None) {
        return Error::Busy;
    }

    flush_rx();
    return send_ascii_command(std::span<const char>{cmd, sizeof(cmd)}, PendingCommand::W41);
}

Error UtmIii::request_read_min_display_speed_rpm() {
    static constexpr char cmd[] = {'W', '4', '2', '\r'};
    if (!is_initialized()) {
        return Error::BufferNotInitialized;
    }
    if (pending_ != PendingCommand::None) {
        return Error::Busy;
    }

    flush_rx();
    return send_ascii_command(std::span<const char>{cmd, sizeof(cmd)}, PendingCommand::W42);
}

} // namespace stm32_library::stm32_modules::utmiii
