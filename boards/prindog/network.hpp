/*
 * Prindog/lwIP network binding for STM32F7.
 */

#pragma once
#include <array>
#include <cstring>

#include "main.h"
#include "realtime_bridge/config/bridge_runtime_config.hpp"

extern "C" {
#include "lwip.h"
#include "lwip/netif.h"
}

extern struct netif gnetif;
extern ETH_HandleTypeDef heth;

namespace stm32_library::boards::prindog {

struct PrindogNetworkConfig {
    using NetworkSettings =
        realtime_bridge::config::NetworkSettings;

    static constexpr uint8_t Ip0 = STM32_ETH_BRIDGE_IP0;
    static constexpr uint8_t Ip1 = STM32_ETH_BRIDGE_IP1;
    static constexpr uint8_t Ip2 = STM32_ETH_BRIDGE_IP2;
    static constexpr uint8_t Ip3 = STM32_ETH_BRIDGE_IP3;

    static constexpr uint8_t Netmask0 = STM32_ETH_BRIDGE_NETMASK_ADDR0;
    static constexpr uint8_t Netmask1 = STM32_ETH_BRIDGE_NETMASK_ADDR1;
    static constexpr uint8_t Netmask2 = STM32_ETH_BRIDGE_NETMASK_ADDR2;
    static constexpr uint8_t Netmask3 = STM32_ETH_BRIDGE_NETMASK_ADDR3;

    static constexpr uint8_t Gateway0 = STM32_ETH_BRIDGE_NETMASK_GATEWAY0;
    static constexpr uint8_t Gateway1 = STM32_ETH_BRIDGE_NETMASK_GATEWAY1;
    static constexpr uint8_t Gateway2 = STM32_ETH_BRIDGE_NETMASK_GATEWAY2;
    static constexpr uint8_t Gateway3 = STM32_ETH_BRIDGE_NETMASK_GATEWAY3;

    static constexpr uint8_t Master0 = STM32_ETH_BRIDGE_NETMASK_GATEWAY0;
    static constexpr uint8_t Master1 = STM32_ETH_BRIDGE_NETMASK_GATEWAY1;
    static constexpr uint8_t Master2 = STM32_ETH_BRIDGE_NETMASK_GATEWAY2;
    static constexpr uint8_t Master3 = STM32_ETH_BRIDGE_NETMASK_GATEWAY3;

    static constexpr uint16_t LocalUdpPort = PRINDOG_LOCAL_UDP_PORT;
    static constexpr uint16_t MasterUdpPort = PRINDOG_MASTER_UDP_PORT;

    // Locally administered MAC address.
    // 02 のbitが「ローカル管理」を意味するので、実験用に安全。
    static constexpr uint8_t Mac0 = 0x02;
    static constexpr uint8_t Mac1 = 0x80;
    static constexpr uint8_t Mac2 = 0xE1;
    static constexpr uint8_t Mac3 = 0x00;
    static constexpr uint8_t Mac4 = 0x00;
    static constexpr uint8_t Mac5 = 0x01 + PRINDOG_BOARD_ID;

    static NetworkSettings defaults() {
        NetworkSettings settings{};
        settings.address.octets = {Ip0, Ip1, Ip2, Ip3};
        settings.netmask.octets = {
            Netmask0,
            Netmask1,
            Netmask2,
            Netmask3
        };
        settings.gateway.octets = {
            Gateway0,
            Gateway1,
            Gateway2,
            Gateway3
        };
        settings.master.octets = {Master0, Master1, Master2, Master3};
        settings.mac.octets = {Mac0, Mac1, Mac2, Mac3, Mac4, Mac5};
        settings.local_udp_port = LocalUdpPort;
        settings.master_udp_port = MasterUdpPort;
        return settings;
    }

    static void apply_mac_to_lwip_netif(const NetworkSettings& settings) {
        mac_storage_ = settings.mac.octets;
        heth.Init.MACAddr = mac_storage_.data();

        gnetif.hwaddr_len = ETH_HWADDR_LEN;
        std::memcpy(
            gnetif.hwaddr,
            mac_storage_.data(),
            mac_storage_.size()
        );

        /*
         * HAL_ETH_Init() has already programmed MAC address slot 0 by the
         * time cppmain() runs. Update the live hardware registers as well as
         * lwIP's copy.
         */
        ETH->MACA0HR =
            (static_cast<uint32_t>(mac_storage_[5]) << 8) |
            static_cast<uint32_t>(mac_storage_[4]);
        ETH->MACA0LR =
            (static_cast<uint32_t>(mac_storage_[3]) << 24) |
            (static_cast<uint32_t>(mac_storage_[2]) << 16) |
            (static_cast<uint32_t>(mac_storage_[1]) << 8) |
            static_cast<uint32_t>(mac_storage_[0]);
    }

    static void apply_address_to_lwip_netif(
        const NetworkSettings& settings
    ) {
        ip_addr_t ipaddr;
        ip_addr_t netmask;
        ip_addr_t gw;

        IP4_ADDR(
            &ipaddr,
            settings.address.octets[0],
            settings.address.octets[1],
            settings.address.octets[2],
            settings.address.octets[3]
        );
        IP4_ADDR(
            &netmask,
            settings.netmask.octets[0],
            settings.netmask.octets[1],
            settings.netmask.octets[2],
            settings.netmask.octets[3]
        );
        IP4_ADDR(
            &gw,
            settings.gateway.octets[0],
            settings.gateway.octets[1],
            settings.gateway.octets[2],
            settings.gateway.octets[3]
        );

        netif_set_addr(&gnetif, &ipaddr, &netmask, &gw);
    }

    static void apply_to_lwip_netif(const NetworkSettings& settings) {
        netif_set_down(&gnetif);

        apply_mac_to_lwip_netif(settings);
        apply_address_to_lwip_netif(settings);

        netif_set_up(&gnetif);
    }

    static ip_addr_t master_ip(const NetworkSettings& settings) {
        ip_addr_t ip;
        IP4_ADDR(
            &ip,
            settings.master.octets[0],
            settings.master.octets[1],
            settings.master.octets[2],
            settings.master.octets[3]
        );
        return ip;
    }

private:
    inline static std::array<uint8_t, 6> mac_storage_{
        Mac0,
        Mac1,
        Mac2,
        Mac3,
        Mac4,
        Mac5
    };
};

} // namespace stm32_library::boards::prindog
