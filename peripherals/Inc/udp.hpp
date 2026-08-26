/*
 * udp.hpp
 *
 * STM32 Library UDP peripheral wrapper
 */

#pragma once

#include "main.h"

#ifdef HAL_ETH_MODULE_ENABLED

#include "udp_queue.hpp"

#include <array>
#include <vector>
#include <functional>
#include <cstring>
#include <algorithm>

extern "C" {
#include "lwip/udp.h"
#include "lwip/pbuf.h"
#include "lwip/ip_addr.h"
}

namespace stm32_library::stm32_peripherals {

struct UdpPacket {
    static constexpr size_t kMaxSize = 1472; // Ethernet MTU 1500 - IPv4 20 - UDP 8

    ip_addr_t remote_ip{};
    uint16_t remote_port = 0;

    std::array<uint8_t, kMaxSize> data{};
    size_t size = 0;

    UdpPacket() = default;

    UdpPacket(const ip_addr_t* addr, uint16_t port, const uint8_t* src, size_t len)
        : remote_port(port)
    {
        if (addr != nullptr) {
            remote_ip = *addr;
        }

        size = std::min(len, data.size());

        if (src != nullptr && size > 0) {
            std::memcpy(data.data(), src, size);
        }
    }

    const uint8_t* payload() const {
        return data.data();
    }

    uint8_t* payload() {
        return data.data();
    }
};

class UdpSocket {
public:
    using CallbackFnType = void(const UdpPacket& packet);
    using PbufCallbackFnType = void(
        pbuf* packet,
        const ip_addr_t* addr,
        uint16_t port
    );

    explicit UdpSocket(uint16_t local_port)
        : local_port_(0)
    {
        (void)rebind(local_port);
    }

    ~UdpSocket() {
        if (pcb_ != nullptr) {
            udp_remove(pcb_);
            pcb_ = nullptr;
        }
    }

    bool ok() const {
        return pcb_ != nullptr;
    }

    uint16_t local_port() const {
        return local_port_;
    }

    bool rebind(uint16_t local_port) {
        if (local_port == 0) {
            return false;
        }

        if (pcb_ != nullptr && local_port_ == local_port) {
            return true;
        }

        udp_pcb* replacement = udp_new();
        if (replacement == nullptr) {
            return false;
        }

        const err_t bind_result =
            udp_bind(replacement, IP_ADDR_ANY, local_port);
        if (bind_result != ERR_OK) {
            udp_remove(replacement);
            return false;
        }

        udp_recv(replacement, UdpSocket::recv_callback_static, this);

        if (pcb_ != nullptr) {
            udp_remove(pcb_);
        }

        pcb_ = replacement;
        local_port_ = local_port;
        return true;
    }

    void attach(std::function<CallbackFnType>&& fn, uint8_t priority = 100) {
        if (!fn) return;

        callbacks_.push_back(CallbackEntry{
            std::move(fn),
            priority
        });

        std::sort(
            callbacks_.begin(),
            callbacks_.end(),
            [](const CallbackEntry& a, const CallbackEntry& b) {
                return a.priority < b.priority;
            }
        );
    }

    void attach_pbuf(
        std::function<PbufCallbackFnType>&& fn,
        uint8_t priority = 100
    ) {
        if (!fn) return;

        pbuf_callbacks_.push_back(PbufCallbackEntry{
            std::move(fn),
            priority
        });

        std::sort(
            pbuf_callbacks_.begin(),
            pbuf_callbacks_.end(),
            [](const PbufCallbackEntry& a, const PbufCallbackEntry& b) {
                return a.priority < b.priority;
            }
        );
    }

    void clear_callbacks() {
        callbacks_.clear();
        pbuf_callbacks_.clear();
    }

    bool write(
        const uint8_t* data,
        size_t size,
        const ip_addr_t* dest_ip,
        uint16_t dest_port
    ) {
        if (pcb_ == nullptr || data == nullptr || dest_ip == nullptr) {
            return false;
        }

        if (size == 0) {
            return true;
        }

        pbuf* p = pbuf_alloc(PBUF_TRANSPORT, size, PBUF_RAM);
        if (p == nullptr) {
            return false;
        }

        std::memcpy(p->payload, data, size);

        const err_t err = udp_sendto(pcb_, p, dest_ip, dest_port);

        pbuf_free(p);

        return err == ERR_OK;
    }

    bool write_reference(
        const uint8_t* data,
        size_t size,
        const ip_addr_t* dest_ip,
        uint16_t dest_port
    ) {
        if (pcb_ == nullptr || data == nullptr || dest_ip == nullptr) {
            return false;
        }

        if (size == 0) {
            return true;
        }

        if (size > UINT16_MAX) {
            return false;
        }

        pbuf* p = pbuf_alloc_reference(
            const_cast<uint8_t*>(data),
            static_cast<u16_t>(size),
            PBUF_REF
        );

        if (p == nullptr) {
            return false;
        }

        const err_t err = udp_sendto(pcb_, p, dest_ip, dest_port);

        pbuf_free(p);

        return err == ERR_OK;
    }

    bool write(
        const char* data,
        size_t size,
        const ip_addr_t* dest_ip,
        uint16_t dest_port
    ) {
        return write(
            reinterpret_cast<const uint8_t*>(data),
            size,
            dest_ip,
            dest_port
        );
    }

    bool write(
        const UdpPacket& packet,
        const ip_addr_t* dest_ip,
        uint16_t dest_port
    ) {
        return write(packet.data.data(), packet.size, dest_ip, dest_port);
    }

    bool write(
        const stm32_library::stm32_peripherals::UdpDatagram& datagram
    ) {
        return write(
            datagram.payload(),
            datagram.size,
            &datagram.remote_ip,
            datagram.remote_port
        );
    }

private:
    struct CallbackEntry {
        std::function<CallbackFnType> fn;
        uint8_t priority;
    };

    struct PbufCallbackEntry {
        std::function<PbufCallbackFnType> fn;
        uint8_t priority;
    };

    static void recv_callback_static(
        void* arg,
        udp_pcb*,
        pbuf* p,
        const ip_addr_t* addr,
        u16_t port
    ) {
        if (arg == nullptr || p == nullptr) {
            if (p != nullptr) {
                pbuf_free(p);
            }
            return;
        }

        auto* socket = reinterpret_cast<UdpSocket*>(arg);
        socket->handle_receive(p, addr, port);

        pbuf_free(p);
    }

    void handle_receive(pbuf* p, const ip_addr_t* addr, uint16_t port) {
        for (auto& callback : pbuf_callbacks_) {
            callback.fn(p, addr, port);
        }

        if (callbacks_.empty()) {
            return;
        }

        UdpPacket packet;
        packet.remote_port = port;

        if (addr != nullptr) {
            packet.remote_ip = *addr;
        }

        packet.size = std::min<size_t>(p->tot_len, packet.data.size());

        if (packet.size > 0) {
            pbuf_copy_partial(p, packet.data.data(), packet.size, 0);
        }

        for (auto& callback : callbacks_) {
            callback.fn(packet);
        }
    }

    uint16_t local_port_;
    udp_pcb* pcb_ = nullptr;
    std::vector<CallbackEntry> callbacks_{};
    std::vector<PbufCallbackEntry> pbuf_callbacks_{};
};

} // namespace stm32_library::stm32_peripherals

#endif // HAL_ETH_MODULE_ENABLED
