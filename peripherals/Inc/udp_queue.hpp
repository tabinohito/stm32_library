/*
 * udp_queue.hpp
 *
 *  SPSC UDP RX/TX queue for stm32_library
 *
 *  RX:
 *      producer = lwIP UDP callback / network pump context
 *      consumer = realtime timer ISR
 *
 *  TX:
 *      producer = realtime timer ISR
 *      consumer = network pump context
 */

#pragma once

#include "main.h"

#ifdef HAL_ETH_MODULE_ENABLED

#include <array>
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <algorithm>

extern "C" {
#include "lwip/pbuf.h"
#include "lwip/ip_addr.h"
}

namespace stm32_library::stm32_peripherals {

struct UdpDatagram {
    static constexpr size_t kDefaultMaxPayloadSize = 1472;

    ip_addr_t remote_ip{};
    uint16_t remote_port = 0;

    size_t size = 0;
    uint32_t debug_enqueue_cycle = 0;

    // 実データはQueue側のSlotが持つ
    uint8_t* data = nullptr;

    const uint8_t* payload() const {
        return data;
    }

    uint8_t* payload() {
        return data;
    }

    bool empty() const {
        return size == 0;
    }
};

enum class UdpQueueResult {
    Ok,
    Full,
    Empty,
    TooLarge,
    NullData,
};

template <size_t Depth, size_t MaxPayloadSize = UdpDatagram::kDefaultMaxPayloadSize>
class UdpDatagramQueue {
    static_assert(Depth >= 2, "UdpDatagramQueue Depth must be >= 2");

public:
    struct Slot {
        ip_addr_t remote_ip{};
        uint16_t remote_port = 0;
        size_t size = 0;
        uint32_t debug_enqueue_cycle = 0;
        std::array<uint8_t, MaxPayloadSize> payload{};
    };

    UdpDatagramQueue() = default;

    UdpDatagramQueue(const UdpDatagramQueue&) = delete;
    UdpDatagramQueue& operator=(const UdpDatagramQueue&) = delete;

    size_t capacity() const {
        return Depth - 1;
    }

    size_t max_payload_size() const {
        return MaxPayloadSize;
    }

    bool empty() const {
        return read_index_ == write_index_;
    }

    bool full() const {
        return next_index(write_index_) == read_index_;
    }

    size_t available() const {
        const size_t w = write_index_;
        const size_t r = read_index_;

        if (w >= r) {
            return w - r;
        }

        return Depth - r + w;
    }

    size_t dropped_count() const {
        return dropped_count_;
    }

    size_t pushed_count() const {
        return pushed_count_;
    }

    size_t popped_count() const {
        return popped_count_;
    }

    void clear() {
        read_index_ = 0;
        write_index_ = 0;
        dropped_count_ = 0;
        pushed_count_ = 0;
        popped_count_ = 0;
    }

    UdpQueueResult push(
        const uint8_t* data,
        size_t size,
        const ip_addr_t& remote_ip,
        uint16_t remote_port
    ) {
        if (data == nullptr && size > 0) {
            return UdpQueueResult::NullData;
        }

        if (size > MaxPayloadSize) {
            dropped_count_++;
            return UdpQueueResult::TooLarge;
        }

        const size_t next = next_index(write_index_);

        if (next == read_index_) {
            dropped_count_++;
            return UdpQueueResult::Full;
        }

        auto& slot = slots_[write_index_];

        slot.remote_ip = remote_ip;
        slot.remote_port = remote_port;
        slot.size = size;
        slot.debug_enqueue_cycle = DWT->CYCCNT;

        if (size > 0) {
            std::memcpy(slot.payload.data(), data, size);
        }

        memory_barrier();

        write_index_ = next;
        pushed_count_++;

        return UdpQueueResult::Ok;
    }

    UdpQueueResult push(
        const UdpDatagram& datagram
    ) {
        if (datagram.data == nullptr && datagram.size > 0) {
            return UdpQueueResult::NullData;
        }

        return push(
            datagram.data,
            datagram.size,
            datagram.remote_ip,
            datagram.remote_port
        );
    }

    UdpQueueResult pop(UdpDatagram& out) {
        if (read_index_ == write_index_) {
            return UdpQueueResult::Empty;
        }

        auto& slot = slots_[read_index_];

        out.remote_ip = slot.remote_ip;
        out.remote_port = slot.remote_port;
        out.size = slot.size;
        out.debug_enqueue_cycle = slot.debug_enqueue_cycle;
        out.data = slot.payload.data();

        memory_barrier();

        read_index_ = next_index(read_index_);
        popped_count_++;

        return UdpQueueResult::Ok;
    }

    /*
     * 最新の1個だけを取り出す。
     * 古いものは捨てる。
     *
     * 制御用途では「全部処理する」より
     * 「最新のUDP入力だけ採用する」方が嬉しいことが多い。
     */
    UdpQueueResult pop_latest(UdpDatagram& out) {
        if (read_index_ == write_index_) {
            return UdpQueueResult::Empty;
        }

        while (next_index(read_index_) != write_index_) {
            read_index_ = next_index(read_index_);
            dropped_count_++;
        }

        return pop(out);
    }

    /*
     * dstへコピーして取り出す版。
     * pop() は内部slotのポインタを返すので、
     * queue slotを長く保持したくない場合はこちらを使う。
     */
    UdpQueueResult pop_copy(
        uint8_t* dst,
        size_t dst_capacity,
        size_t& out_size,
        ip_addr_t& out_remote_ip,
        uint16_t& out_remote_port
    ) {
        if (dst == nullptr && dst_capacity > 0) {
            return UdpQueueResult::NullData;
        }

        if (read_index_ == write_index_) {
            return UdpQueueResult::Empty;
        }

        auto& slot = slots_[read_index_];

        if (slot.size > dst_capacity) {
            return UdpQueueResult::TooLarge;
        }

        out_remote_ip = slot.remote_ip;
        out_remote_port = slot.remote_port;
        out_size = slot.size;

        if (slot.size > 0) {
            std::memcpy(dst, slot.payload.data(), slot.size);
        }

        memory_barrier();

        read_index_ = next_index(read_index_);
        popped_count_++;

        return UdpQueueResult::Ok;
    }

    UdpQueueResult push_from_pbuf(
        pbuf* p,
        const ip_addr_t& remote_ip,
        uint16_t remote_port
    ) {
        if (p == nullptr) {
            return UdpQueueResult::NullData;
        }

        if (p->tot_len > MaxPayloadSize) {
            dropped_count_++;
            return UdpQueueResult::TooLarge;
        }

        const size_t next = next_index(write_index_);

        if (next == read_index_) {
            dropped_count_++;
            return UdpQueueResult::Full;
        }

        auto& slot = slots_[write_index_];

        slot.remote_ip = remote_ip;
        slot.remote_port = remote_port;
        slot.size = p->tot_len;
        slot.debug_enqueue_cycle = DWT->CYCCNT;

        if (slot.size > 0) {
            pbuf_copy_partial(
                p,
                slot.payload.data(),
                slot.size,
                0
            );
        }

        memory_barrier();

        write_index_ = next;
        pushed_count_++;

        return UdpQueueResult::Ok;
    }

private:
    static size_t next_index(size_t i) {
        return (i + 1) % Depth;
    }

    static void memory_barrier() {
#if defined(__GNUC__) || defined(__clang__)
        __asm volatile ("" ::: "memory");
#endif

#if defined(__CORTEX_M)
        __DMB();
#endif
    }

    std::array<Slot, Depth> slots_{};

    volatile size_t read_index_ = 0;
    volatile size_t write_index_ = 0;

    volatile size_t dropped_count_ = 0;
    volatile size_t pushed_count_ = 0;
    volatile size_t popped_count_ = 0;
};

template <size_t Depth, size_t MaxPayloadSize = UdpDatagram::kDefaultMaxPayloadSize>
using UdpRxQueue = UdpDatagramQueue<Depth, MaxPayloadSize>;

template <size_t Depth, size_t MaxPayloadSize = UdpDatagram::kDefaultMaxPayloadSize>
using UdpTxQueue = UdpDatagramQueue<Depth, MaxPayloadSize>;

} // namespace stm32_library::stm32_peripherals

#endif // HAL_ETH_MODULE_ENABLED
