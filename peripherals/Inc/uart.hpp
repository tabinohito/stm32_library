#pragma once

#include "main.h"

#ifdef HAL_UART_MODULE_ENABLED

// Keep optional UART modes tied to the capabilities exposed by each STM32 HAL.
// STM32F4's legacy UART does not provide AdvancedInit or hardware DE control.
#if defined(UART_ADVFEATURE_TXINVERT_INIT) && \
    defined(UART_ADVFEATURE_RXINVERT_INIT) && \
    defined(UART_ADVFEATURE_SWAP_INIT) && \
    defined(UART_ADVFEATURE_TXINV_ENABLE) && \
    defined(UART_ADVFEATURE_TXINV_DISABLE) && \
    defined(UART_ADVFEATURE_RXINV_ENABLE) && \
    defined(UART_ADVFEATURE_RXINV_DISABLE) && \
    defined(UART_ADVFEATURE_SWAP_ENABLE) && \
    defined(UART_ADVFEATURE_SWAP_DISABLE)
#define STM32_LIBRARY_UART_ADVANCED_FEATURES_AVAILABLE
#endif

#if defined(UART_DE_POLARITY_HIGH) && defined(UART_DE_POLARITY_LOW)
#define STM32_LIBRARY_UART_RS485_AVAILABLE
#endif

#include "../misc/callback.hpp"
#include "../misc/format.hpp"

#include <algorithm>
#include <climits>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <span>
#include <string>
#include <vector>
#include <version>

namespace stm32_library::stm32_peripherals {

class Uart {
private:
    UART_HandleTypeDef *handle_ = nullptr;

    bool use_dma_transmit_ = false;

    // 既存のprintf用buffer
    std::string buffer_ = {};

    // RX DMA
    uint8_t *data_p_ = nullptr;
    int16_t data_size_ = 0;
    int16_t index_read_ = 0;

    // TX DMA queue
    uint8_t *tx_queue_ = nullptr;
    size_t tx_queue_capacity_ = 0;

    volatile size_t tx_head_ = 0;
    volatile size_t tx_tail_ = 0;

    volatile bool tx_dma_queue_enabled_ = false;
    volatile bool tx_dma_active_ = false;
    volatile uint16_t tx_dma_active_size_ = 0;

    volatile uint32_t tx_enqueue_count_ = 0;
    volatile uint32_t tx_sent_count_ = 0;
    volatile uint32_t tx_drop_count_ = 0;
    volatile uint32_t tx_dma_start_count_ = 0;
    volatile uint32_t tx_dma_complete_count_ = 0;
    volatile uint32_t tx_dma_error_count_ = 0;
    volatile uint32_t tx_dma_busy_count_ = 0;
    volatile uint32_t rx_error_count_ = 0;
    volatile uint32_t rx_dma_restart_count_ = 0;
    volatile bool rx_dma_restart_pending_ = false;

public:
    using CallbackFnType = void();

    enum class PhysicalMode {
        Uart,
#ifdef STM32_LIBRARY_UART_RS485_AVAILABLE
        Rs485,
#endif
    };

    struct LineConfig {
        uint32_t baud_rate = 115200;
        PhysicalMode physical_mode = PhysicalMode::Uart;
#ifdef STM32_LIBRARY_UART_ADVANCED_FEATURES_AVAILABLE
        uint32_t word_length = UART_WORDLENGTH_8B;
        uint32_t stop_bits = UART_STOPBITS_1;
        uint32_t parity = UART_PARITY_NONE;
        uint32_t mode = UART_MODE_TX_RX;
        uint32_t hardware_flow_control = UART_HWCONTROL_NONE;
        uint32_t oversampling = UART_OVERSAMPLING_16;
        uint32_t one_bit_sampling = UART_ONE_BIT_SAMPLE_DISABLE;
        bool swap_rx_tx = false;
        bool invert_tx = false;
        bool invert_rx = false;
#endif
#ifdef STM32_LIBRARY_UART_RS485_AVAILABLE
        bool data_invert = false;
        bool overrun_disable = false;
        bool dma_disable_on_rx_error = false;
        bool auto_baud = false;
        uint32_t auto_baud_mode =
            UART_ADVFEATURE_AUTOBAUDRATE_ONSTARTBIT;
        bool msb_first = false;
        uint32_t rs485_de_polarity = UART_DE_POLARITY_HIGH;
        uint32_t rs485_assertion_time = 0;
        uint32_t rs485_deassertion_time = 0;
#endif
    };

    explicit Uart(UART_HandleTypeDef *handle) : handle_(handle) {
        register_instance(this);
    }

    virtual ~Uart() {
        unregister_instance(this);
    }

    virtual UART_HandleTypeDef* get_handle() {
        return handle_;
    }

    virtual const UART_HandleTypeDef* get_handle() const {
        return handle_;
    }

    HAL_StatusTypeDef stop() {
        return handle_ != nullptr ? HAL_UART_Abort(handle_) : HAL_ERROR;
    }

    HAL_StatusTypeDef configure(const LineConfig& config) {
        if (handle_ == nullptr) {
            return HAL_ERROR;
        }

        (void)stop();

        handle_->Init.BaudRate = config.baud_rate;
        handle_->Init.WordLength = config.word_length;
        handle_->Init.StopBits = config.stop_bits;
        handle_->Init.Parity = config.parity;
        handle_->Init.Mode = config.mode;
        handle_->Init.HwFlowCtl = config.hardware_flow_control;
        handle_->Init.OverSampling = config.oversampling;
        handle_->Init.OneBitSampling = config.one_bit_sampling;
#ifdef STM32_LIBRARY_UART_ADVANCED_FEATURES_AVAILABLE
        handle_->AdvancedInit.AdvFeatureInit =
            UART_ADVFEATURE_TXINVERT_INIT |
            UART_ADVFEATURE_RXINVERT_INIT |
            UART_ADVFEATURE_DATAINVERT_INIT |
            UART_ADVFEATURE_SWAP_INIT |
            UART_ADVFEATURE_RXOVERRUNDISABLE_INIT |
            UART_ADVFEATURE_DMADISABLEONERROR_INIT |
            UART_ADVFEATURE_AUTOBAUDRATE_INIT |
            UART_ADVFEATURE_MSBFIRST_INIT;
        handle_->AdvancedInit.TxPinLevelInvert = config.invert_tx ?
            UART_ADVFEATURE_TXINV_ENABLE :
            UART_ADVFEATURE_TXINV_DISABLE;
        handle_->AdvancedInit.RxPinLevelInvert = config.invert_rx ?
            UART_ADVFEATURE_RXINV_ENABLE :
            UART_ADVFEATURE_RXINV_DISABLE;
        handle_->AdvancedInit.DataInvert = config.data_invert ?
            UART_ADVFEATURE_DATAINV_ENABLE :
            UART_ADVFEATURE_DATAINV_DISABLE;
        handle_->AdvancedInit.Swap = config.swap_rx_tx ?
            UART_ADVFEATURE_SWAP_ENABLE :
            UART_ADVFEATURE_SWAP_DISABLE;
        handle_->AdvancedInit.OverrunDisable =
            config.overrun_disable ?
                UART_ADVFEATURE_OVERRUN_DISABLE :
                UART_ADVFEATURE_OVERRUN_ENABLE;
        handle_->AdvancedInit.DMADisableonRxError =
            config.dma_disable_on_rx_error ?
                UART_ADVFEATURE_DMA_DISABLEONRXERROR :
                UART_ADVFEATURE_DMA_ENABLEONRXERROR;
        handle_->AdvancedInit.AutoBaudRateEnable = config.auto_baud ?
            UART_ADVFEATURE_AUTOBAUDRATE_ENABLE :
            UART_ADVFEATURE_AUTOBAUDRATE_DISABLE;
        handle_->AdvancedInit.AutoBaudRateMode = config.auto_baud_mode;
        handle_->AdvancedInit.MSBFirst = config.msb_first ?
            UART_ADVFEATURE_MSBFIRST_ENABLE :
            UART_ADVFEATURE_MSBFIRST_DISABLE;
#endif

#ifdef STM32_LIBRARY_UART_RS485_AVAILABLE
        if (config.physical_mode == PhysicalMode::Rs485) {
            return HAL_RS485Ex_Init(
                handle_,
                config.rs485_de_polarity,
                config.rs485_assertion_time,
                config.rs485_deassertion_time
            );
        }
#endif

        return HAL_UART_Init(handle_);
    }

    HAL_StatusTypeDef configure_uart(
        uint32_t baud_rate
#ifdef STM32_LIBRARY_UART_ADVANCED_FEATURES_AVAILABLE
        ,
        bool swap_rx_tx = false
#endif
    ) {
        LineConfig config{};
        config.baud_rate = baud_rate;
        config.physical_mode = PhysicalMode::Uart;
#ifdef STM32_LIBRARY_UART_ADVANCED_FEATURES_AVAILABLE
        config.swap_rx_tx = swap_rx_tx;
#endif
        return configure(config);
    }

#ifdef STM32_LIBRARY_UART_RS485_AVAILABLE
    HAL_StatusTypeDef configure_rs485(
        uint32_t baud_rate,
#ifdef STM32_LIBRARY_UART_ADVANCED_FEATURES_AVAILABLE
        bool swap_rx_tx = false,
#endif
        uint32_t de_polarity = UART_DE_POLARITY_HIGH,
        uint32_t assertion_time = 0,
        uint32_t deassertion_time = 0
    ) {
        LineConfig config{};
        config.baud_rate = baud_rate;
        config.physical_mode = PhysicalMode::Rs485;
#ifdef STM32_LIBRARY_UART_ADVANCED_FEATURES_AVAILABLE
        config.swap_rx_tx = swap_rx_tx;
#endif
        config.rs485_de_polarity = de_polarity;
        config.rs485_assertion_time = assertion_time;
        config.rs485_deassertion_time = deassertion_time;
        return configure(config);
    }
#endif

    void use_dma_transmit(bool use_dma = true) {
        use_dma_transmit_ = use_dma;
    }

    bool use_dma_transmit() const {
        return use_dma_transmit_;
    }

    bool dma_transmit_available() const {
        return handle_ != nullptr && handle_->hdmatx != nullptr;
    }

    bool dma_receive_available() const {
        return handle_ != nullptr && handle_->hdmarx != nullptr;
    }

    bool dma_receive_active() const {
        return dma_receive_available() &&
            handle_->RxState == HAL_UART_STATE_BUSY_RX;
    }

    void poll_tx_dma() {
        if (use_dma_transmit_ && tx_dma_queue_enabled_ && dma_transmit_available()) {
            kick_tx_dma();
        }
    }

    void poll_rx_dma() {
        if (handle_ == nullptr || data_p_ == nullptr || data_size_ <= 0) {
            return;
        }

        if (rx_dma_restart_pending_) {
            rx_dma_restart_pending_ = false;

            if (restart_receive_dma() == HAL_OK) {
                inc_u32(rx_dma_restart_count_);
            }

            return;
        }

        if (has_rx_error()) {
            inc_u32(rx_error_count_);

            if (restart_receive_dma() == HAL_OK) {
                inc_u32(rx_dma_restart_count_);
            }
        }
    }

    // -------------------------------------------------------------------------
    // TX DMA queue mode
    // -------------------------------------------------------------------------
    // bufferは呼び出し側でstatic/globalに置くこと。
    // F7 DCacheを考えるなら32byte align推奨。
    //
    // example:
    //   static uint8_t stlink_tx_queue[32768] __attribute__((aligned(32)));
    //   uart.use_dma_transmit(true);
    //   uart.use_dma_transmit_queue(stlink_tx_queue, sizeof(stlink_tx_queue));
    // -------------------------------------------------------------------------
    void use_dma_transmit_queue(uint8_t *buffer, size_t size) {
        const uint32_t primask = enter_critical();

        tx_queue_ = buffer;
        tx_queue_capacity_ = size;
        tx_head_ = 0;
        tx_tail_ = 0;
        tx_dma_active_ = false;
        tx_dma_active_size_ = 0;
        tx_dma_queue_enabled_ = (buffer != nullptr && size >= 2);

        restore_critical(primask);
    }

    void disable_dma_transmit_queue() {
        if (handle_ != nullptr && tx_dma_active_) {
            HAL_UART_AbortTransmit(handle_);
        }

        const uint32_t primask = enter_critical();

        tx_dma_queue_enabled_ = false;
        tx_queue_ = nullptr;
        tx_queue_capacity_ = 0;
        tx_head_ = 0;
        tx_tail_ = 0;
        tx_dma_active_ = false;
        tx_dma_active_size_ = 0;

        restore_critical(primask);
    }

    bool dma_transmit_queue_enabled() const {
        return tx_dma_queue_enabled_;
    }

    bool tx_dma_active() const {
        return tx_dma_active_;
    }

    uint32_t tx_enqueue_count() const {
        return tx_enqueue_count_;
    }

    uint32_t tx_sent_count() const {
        return tx_sent_count_;
    }

    uint32_t tx_drop_count() const {
        return tx_drop_count_;
    }

    uint32_t tx_dma_start_count() const {
        return tx_dma_start_count_;
    }

    uint32_t tx_dma_complete_count() const {
        return tx_dma_complete_count_;
    }

    uint32_t tx_dma_error_count() const {
        return tx_dma_error_count_;
    }

    uint32_t tx_dma_busy_count() const {
        return tx_dma_busy_count_;
    }

    uint32_t tx_queue_size() const {
        return static_cast<uint32_t>(queue_size_unsafe());
    }

    uint32_t tx_queue_capacity() const {
        if (tx_queue_capacity_ == 0) {
            return 0;
        }
        return static_cast<uint32_t>(tx_queue_capacity_ - 1);
    }

    uint32_t rx_error_count() const {
        return rx_error_count_;
    }

    uint32_t rx_dma_restart_count() const {
        return rx_dma_restart_count_;
    }

    void clear_tx_queue() {
        if (handle_ != nullptr && tx_dma_active_) {
            HAL_UART_AbortTransmit(handle_);
        }

        const uint32_t primask = enter_critical();

        tx_head_ = 0;
        tx_tail_ = 0;
        tx_dma_active_ = false;
        tx_dma_active_size_ = 0;

        restore_critical(primask);
    }

    void reset_tx_counters() {
        tx_enqueue_count_ = 0;
        tx_sent_count_ = 0;
        tx_drop_count_ = 0;
        tx_dma_start_count_ = 0;
        tx_dma_complete_count_ = 0;
        tx_dma_error_count_ = 0;
        tx_dma_busy_count_ = 0;
    }

    // -------------------------------------------------------------------------
    // write
    // -------------------------------------------------------------------------
    virtual HAL_StatusTypeDef write(
        const uint8_t *data,
        uint16_t size,
        uint32_t timeout = 10
    ) {
        if (handle_ == nullptr || data == nullptr || size == 0) {
            return HAL_ERROR;
        }

        if (use_dma_transmit_ && dma_transmit_available()) {
            if (tx_dma_queue_enabled_) {
                const bool queued = tx_enqueue_all(data, size);
                kick_tx_dma();

                if (!queued) {
                    add_u32(tx_drop_count_, static_cast<uint32_t>(size));
                    return HAL_BUSY;
                }

                return HAL_OK;
            }

            // 旧動作: queueなしDMA直接送信
            return HAL_UART_Transmit_DMA(
                handle_,
                const_cast<uint8_t *>(data),
                size
            );
        }

        return HAL_UART_Transmit(
            handle_,
            const_cast<uint8_t *>(data),
            size,
            timeout
        );
    }

    virtual HAL_StatusTypeDef write(
        uint8_t *data,
        uint16_t size,
        uint32_t timeout = 10
    ) {
        return write(static_cast<const uint8_t *>(data), size, timeout);
    }

#ifdef __cpp_lib_span
    virtual HAL_StatusTypeDef write(
        std::span<const uint8_t> data,
        uint32_t timeout = 10
    ) {
        if (data.size() > UINT16_MAX) {
            return HAL_ERROR;
        }

        return write(
            data.data(),
            static_cast<uint16_t>(data.size()),
            timeout
        );
    }
#endif

    virtual HAL_StatusTypeDef write_byte(uint8_t byte, uint32_t timeout = 10) {
        return write(&byte, 1, timeout);
    }

    HAL_StatusTypeDef write() {
        int buf_size = buffer_.length();
        if (buf_size > UINT16_MAX) {
            buf_size = UINT16_MAX;
        }

        if (buf_size <= 0) {
            return HAL_OK;
        }

        if (use_dma_transmit_ && tx_dma_queue_enabled_ && dma_transmit_available()) {
            const auto ret = write(
                reinterpret_cast<const uint8_t *>(buffer_.data()),
                static_cast<uint16_t>(buf_size)
            );

            if (ret == HAL_OK) {
                buffer_.erase(0, buf_size);
            }

            return ret;
        }

        if (handle_->gState == HAL_UART_STATE_READY) {
            static std::string send_str;
            send_str = buffer_.substr(0, buf_size);
            buffer_ = buffer_.substr(buf_size);

            return write(
                reinterpret_cast<const uint8_t *>(send_str.c_str()),
                static_cast<uint16_t>(buf_size)
            );
        }

        return HAL_BUSY;
    }

    template <class... Args>
    HAL_StatusTypeDef write(const char *fmt, Args... args) {
        if (use_dma_transmit_) {
            push_buffer(fmt, args...);
            return write();
        }

        std::vector<char> str = utility::format(fmt, args...);
        return write(
            reinterpret_cast<const uint8_t *>(str.data()),
            static_cast<uint16_t>(str.size() - 1)
        );
    }

    template <class... Args>
    void push_buffer(const char *fmt, Args... args) {
        std::vector<char> buf = utility::format(fmt, args...);
        buffer_ += std::string(buf.begin(), buf.end() - 1);
    }

    // -------------------------------------------------------------------------
    // RX
    // -------------------------------------------------------------------------
    HAL_StatusTypeDef read(void *buffer, size_t size, uint32_t time_out = 10) {
        HAL_StatusTypeDef ret = HAL_UART_Receive(
            handle_,
            static_cast<uint8_t *>(buffer),
            size,
            time_out
        );

        if (ret != HAL_OK) {
            HAL_UART_Abort(handle_);
        }

        return ret;
    }

    void attach(std::function<CallbackFnType> &&fn, uint8_t priority = 100) {
        callback::attach(
            reinterpret_cast<intptr_t>(handle_),
            std::move(fn),
            priority
        );
    }

    virtual void start_receive_dma(
        uint8_t *data_p,
        int data_size,
        bool is_dma_start_test = false
    ) {
        data_p_ = data_p;
        data_size_ = data_size;
        index_read_ = 0;

        HAL_UART_Receive_DMA(handle_, data_p, data_size);

        if (is_dma_start_test) {
            dma_receive_test(data_p, data_size);
        }
    }

    void dma_receive_test(uint8_t *data_p, size_t data_size) {
        size_t start_size = __HAL_DMA_GET_COUNTER(handle_->hdmarx);
        uint32_t start_ms = HAL_GetTick();

        while (
            start_size == __HAL_DMA_GET_COUNTER(handle_->hdmarx) &&
            (HAL_GetTick() - start_ms <= 1000)
        ) {
            if (
                __HAL_UART_GET_FLAG(handle_, UART_FLAG_ORE) ||
                __HAL_UART_GET_FLAG(handle_, UART_FLAG_NE) ||
                __HAL_UART_GET_FLAG(handle_, UART_FLAG_FE) ||
                __HAL_UART_GET_FLAG(handle_, UART_FLAG_PE)
            ) {
                HAL_UART_Abort(handle_);
                HAL_UART_Receive_DMA(handle_, data_p, data_size);
            }
        }
    }

    bool has_rx_error() const {
        if (handle_ == nullptr) {
            return false;
        }

        return
            __HAL_UART_GET_FLAG(handle_, UART_FLAG_ORE) ||
            __HAL_UART_GET_FLAG(handle_, UART_FLAG_NE) ||
            __HAL_UART_GET_FLAG(handle_, UART_FLAG_FE) ||
            __HAL_UART_GET_FLAG(handle_, UART_FLAG_PE);
    }

    void clear_rx_error() const {
        if (handle_ == nullptr) {
            return;
        }

#if defined(UART_CLEAR_OREF) && defined(UART_CLEAR_NEF) && \
    defined(UART_CLEAR_PEF) && defined(UART_CLEAR_FEF)
        __HAL_UART_CLEAR_FLAG(
            handle_,
            UART_CLEAR_OREF |
            UART_CLEAR_NEF |
            UART_CLEAR_PEF |
            UART_CLEAR_FEF
        );
#elif defined(__HAL_UART_CLEAR_OREFLAG)
        // Legacy UARTs (including STM32F4) clear all PE/FE/NE/ORE errors by
        // reading SR followed by DR; this HAL macro performs that sequence.
        __HAL_UART_CLEAR_OREFLAG(handle_);
#endif
    }

    HAL_StatusTypeDef restart_receive_dma() {
        if (handle_ == nullptr || data_p_ == nullptr || data_size_ <= 0) {
            return HAL_ERROR;
        }

        (void)HAL_UART_AbortReceive(handle_);
        clear_rx_error();
        index_read_ = 0;

        return HAL_UART_Receive_DMA(
            handle_,
            data_p_,
            static_cast<uint16_t>(data_size_)
        );
    }

    bool restart_receive_dma_if_error() {
        if (!has_rx_error()) {
            return false;
        }

        return restart_receive_dma() == HAL_OK;
    }

    uint16_t dma_receive_data_num() const {
        if (handle_ == nullptr || handle_->hdmarx == nullptr || data_size_ <= 0) {
            return 0;
        }

        int16_t index = data_size_ - __HAL_DMA_GET_COUNTER(handle_->hdmarx);
        int16_t remain_data = index - index_read_;
        return (remain_data < 0) ? remain_data + data_size_ : remain_data;
    }

    uint8_t dma_receive_data() {
        uint8_t read_data = 0;

        uint16_t remain_data = dma_receive_data_num();
        if (remain_data > 0) {
            read_data = data_p_[index_read_];
            index_read_++;

            if (index_read_ >= data_size_) {
                index_read_ = 0;
            }
        }

        return read_data;
    }

    size_t read_dma(uint8_t *dst, size_t max_len) {
        if (dst == nullptr || max_len == 0) {
            return 0;
        }

        const size_t read_size = std::min(
            max_len,
            static_cast<size_t>(dma_receive_data_num())
        );

        for (size_t i = 0; i < read_size; i++) {
            dst[i] = dma_receive_data();
        }

        return read_size;
    }

    // HAL_UART_TxCpltCallback から呼ぶ
    void on_tx_complete() {
        uint16_t completed_size = 0;

        {
            const uint32_t primask = enter_critical();

            completed_size = tx_dma_active_size_;

            if (tx_dma_active_) {
                tx_tail_ = (tx_tail_ + completed_size) % tx_queue_capacity_;
                tx_dma_active_size_ = 0;
                tx_dma_active_ = false;

                add_u32(tx_sent_count_, completed_size);
                inc_u32(tx_dma_complete_count_);
            }

            restore_critical(primask);
        }

        kick_tx_dma();
    }

    static void tx_complete_callback(UART_HandleTypeDef *huart) {
        Uart *uart = find_instance(huart);

        if (uart != nullptr) {
            uart->on_tx_complete();
        }
    }

    void on_error() {
        inc_u32(rx_error_count_);

        if (data_p_ != nullptr && data_size_ > 0) {
            rx_dma_restart_pending_ = true;
        }
    }

    static void error_callback(UART_HandleTypeDef *huart) {
        Uart *uart = find_instance(huart);

        if (uart != nullptr) {
            uart->on_error();
        }
    }

private:
    static constexpr size_t instance_capacity_ = 16;

    static Uart **instances() {
        static Uart *list[instance_capacity_] = {};
        return list;
    }

    static void register_instance(Uart *uart) {
        if (uart == nullptr || uart->handle_ == nullptr) {
            return;
        }

        auto list = instances();

        for (size_t i = 0; i < instance_capacity_; i++) {
            if (list[i] != nullptr && list[i]->handle_ == uart->handle_) {
                list[i] = uart;
                return;
            }
        }

        for (size_t i = 0; i < instance_capacity_; i++) {
            if (list[i] == nullptr) {
                list[i] = uart;
                return;
            }
        }
    }

    static void unregister_instance(Uart *uart) {
        auto list = instances();

        for (size_t i = 0; i < instance_capacity_; i++) {
            if (list[i] == uart) {
                list[i] = nullptr;
                return;
            }
        }
    }

    static Uart *find_instance(UART_HandleTypeDef *huart) {
        if (huart == nullptr) {
            return nullptr;
        }

        auto list = instances();

        for (size_t i = 0; i < instance_capacity_; i++) {
            if (list[i] != nullptr && list[i]->handle_ == huart) {
                return list[i];
            }
        }

        return nullptr;
    }

    size_t next_index(size_t index) const {
        return (index + 1U) % tx_queue_capacity_;
    }

    size_t queue_size_unsafe() const {
        if (tx_queue_capacity_ == 0) {
            return 0;
        }

        if (tx_head_ >= tx_tail_) {
            return tx_head_ - tx_tail_;
        }

        return tx_queue_capacity_ - tx_tail_ + tx_head_;
    }

    bool queue_empty_unsafe() const {
        return tx_head_ == tx_tail_;
    }

    size_t queue_free_unsafe() const {
        if (tx_queue_capacity_ < 2) {
            return 0;
        }

        return (tx_queue_capacity_ - 1U) - queue_size_unsafe();
    }

    bool tx_enqueue_all(const uint8_t *data, size_t size) {
        if (tx_queue_ == nullptr || tx_queue_capacity_ < 2) {
            return false;
        }

        size_t pushed = 0;

        const uint32_t primask = enter_critical();

        if (size > queue_free_unsafe()) {
            restore_critical(primask);
            return false;
        }

        while (pushed < size) {
            const size_t next = next_index(tx_head_);

            tx_queue_[tx_head_] = data[pushed];
            tx_head_ = next;
            pushed++;
        }

        add_u32(tx_enqueue_count_, static_cast<uint32_t>(pushed));

        restore_critical(primask);

        return true;
    }

    void kick_tx_dma() {
        if (
            handle_ == nullptr ||
            handle_->hdmatx == nullptr ||
            tx_queue_ == nullptr ||
            tx_queue_capacity_ < 2 ||
            !tx_dma_queue_enabled_
        ) {
            return;
        }

        uint8_t *dma_ptr = nullptr;
        uint16_t dma_size = 0;

        {
            const uint32_t primask = enter_critical();

            if (tx_dma_active_ || queue_empty_unsafe()) {
                restore_critical(primask);
                return;
            }

            size_t contiguous = 0;

            if (tx_head_ > tx_tail_) {
                contiguous = tx_head_ - tx_tail_;
            } else {
                contiguous = tx_queue_capacity_ - tx_tail_;
            }

            if (contiguous > UINT16_MAX) {
                contiguous = UINT16_MAX;
            }

            if (contiguous == 0) {
                restore_critical(primask);
                return;
            }

            dma_ptr = &tx_queue_[tx_tail_];
            dma_size = static_cast<uint16_t>(contiguous);

            tx_dma_active_ = true;
            tx_dma_active_size_ = dma_size;

            restore_critical(primask);
        }

        clean_dcache_for_dma(dma_ptr, dma_size);

        const HAL_StatusTypeDef ret =
            HAL_UART_Transmit_DMA(handle_, dma_ptr, dma_size);

        if (ret == HAL_OK) {
            inc_u32(tx_dma_start_count_);
            return;
        }

        {
            const uint32_t primask = enter_critical();

            tx_dma_active_ = false;
            tx_dma_active_size_ = 0;

            restore_critical(primask);
        }

        if (ret == HAL_BUSY) {
            inc_u32(tx_dma_busy_count_);
        } else {
            inc_u32(tx_dma_error_count_);
        }
    }

    static void clean_dcache_for_dma(uint8_t *ptr, size_t size) {
#if (__DCACHE_PRESENT == 1U)
        if (ptr == nullptr || size == 0) {
            return;
        }

        if ((SCB->CCR & SCB_CCR_DC_Msk) == 0U) {
            return;
        }

        const uintptr_t start =
            reinterpret_cast<uintptr_t>(ptr) & ~static_cast<uintptr_t>(31U);

        const uintptr_t end =
            (reinterpret_cast<uintptr_t>(ptr) + size + 31U) &
            ~static_cast<uintptr_t>(31U);

        SCB_CleanDCache_by_Addr(
            reinterpret_cast<uint32_t *>(start),
            static_cast<int32_t>(end - start)
        );
#else
        (void)ptr;
        (void)size;
#endif
    }

    static void add_u32(volatile uint32_t &value, uint32_t delta) {
        value = static_cast<uint32_t>(value + delta);
    }

    static void inc_u32(volatile uint32_t &value) {
        value = static_cast<uint32_t>(value + 1U);
    }

    static uint32_t enter_critical() {
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        return primask;
    }

    static void restore_critical(uint32_t primask) {
        if (primask == 0U) {
            __enable_irq();
        }
    }
};

} // namespace stm32_library::stm32_peripherals

#endif // HAL_UART_MODULE_ENABLED
