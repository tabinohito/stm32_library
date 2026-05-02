/*
 * udp.hpp
 *
 * STM32 Library UDP peripheral wrapper
 */

#pragma once

#include "main.h"
#include "udp_queue.hpp"

#ifdef HAL_ETH_MODULE_ENABLED

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

    explicit UdpSocket(uint16_t local_port)
        : local_port_(local_port)
    {
        pcb_ = udp_new();

        if (pcb_ != nullptr) {
            udp_bind(pcb_, IP_ADDR_ANY, local_port_);
            udp_recv(pcb_, UdpSocket::recv_callback_static, this);
        }
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

    void clear_callbacks() {
        callbacks_.clear();
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
};

} // namespace stm32_library::stm32_peripherals

#endif // HAL_ETH_MODULE_ENABLED
