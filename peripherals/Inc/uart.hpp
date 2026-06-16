#pragma once

#include "main.h"

#ifdef HAL_UART_MODULE_ENABLED

#include "../misc/callback.hpp"
#include "../misc/format.hpp"

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

public:
    using CallbackFnType = void();

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

    void use_dma_transmit(bool use_dma = true) {
        use_dma_transmit_ = use_dma;
    }

    bool use_dma_transmit() const {
        return use_dma_transmit_;
    }

    void poll_tx_dma() {
        if (use_dma_transmit_ && tx_dma_queue_enabled_) {
            kick_tx_dma();
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

        if (use_dma_transmit_) {
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

        if (use_dma_transmit_ && tx_dma_queue_enabled_) {
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

    uint16_t dma_receive_data_num() {
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
