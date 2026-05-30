#pragma once

#include "main.h"

#ifdef HAL_ETH_MODULE_ENABLED

#include <cstdint>
#include <cstdio>

extern "C" {
#include "lwip/netif.h"
#include "lwip/ip4_addr.h"
#include "lwip/opt.h"
}

/*
 * Optional debug counters.
 * If you added these counters in ethernetif.c, define:
 *
 *   #define STM32_LIBRARY_ETHERNETIF_COUNTERS_ENABLED 1
 *
 * before including this file, or in compiler defines.
 */
#ifndef STM32_LIBRARY_ETHERNETIF_COUNTERS_ENABLED
#define STM32_LIBRARY_ETHERNETIF_COUNTERS_ENABLED 0
#endif

#if STM32_LIBRARY_ETHERNETIF_COUNTERS_ENABLED
extern "C" volatile uint32_t ethif_input_count;
extern "C" volatile uint32_t ethif_low_input_count;
extern "C" volatile uint32_t ethif_rx_ok_count;
extern "C" volatile uint32_t ethif_rx_null_count;
extern "C" volatile uint32_t ethif_output_count;
extern "C" volatile uint32_t ethif_tx_ok_count;
extern "C" volatile uint32_t ethif_tx_err_count;
#endif

namespace stm32_library::stm32_peripherals {

class EthernetDiagnostics {
public:
    struct PhyInfo {
        bool found = false;
        uint32_t address = 0;
        uint32_t id1 = 0;
        uint32_t id2 = 0;
        uint32_t bmcr = 0;
        uint32_t bmsr1 = 0;
        uint32_t bmsr2 = 0;
        uint32_t anar = 0;
        uint32_t anlpar = 0;
        uint32_t scsr = 0;
    };

    explicit EthernetDiagnostics(ETH_HandleTypeDef* eth, struct netif* interface = nullptr)
        : eth_(eth), netif_(interface)
    {
    }

    void set_eth_handle(ETH_HandleTypeDef* eth)
    {
        eth_ = eth;
    }

    void set_netif(struct netif* interface)
    {
        netif_ = interface;
    }

    bool read_phy(uint32_t phy_addr, uint32_t reg, uint32_t& value)
    {
        if (eth_ == nullptr) {
            return false;
        }

        const uint32_t old_addr = eth_->Init.PhyAddress;
        eth_->Init.PhyAddress = phy_addr;

        const HAL_StatusTypeDef status =
            HAL_ETH_ReadPHYRegister(eth_, static_cast<uint16_t>(reg), &value);

        eth_->Init.PhyAddress = old_addr;

        return status == HAL_OK;
    }

    PhyInfo read_phy_info(uint32_t addr)
    {
        PhyInfo info{};
        info.address = addr;

        if (!read_phy(addr, PHY_ID1, info.id1)) {
            return info;
        }

        if (!read_phy(addr, PHY_ID2, info.id2)) {
            return info;
        }

        if ((info.id1 == 0xFFFF && info.id2 == 0xFFFF) ||
            (info.id1 == 0x0000 && info.id2 == 0x0000)) {
            return info;
        }

        info.found = true;

        read_phy(addr, PHY_BMCR, info.bmcr);

        /*
         * BMSR Link Status is latched, so read twice.
         */
        read_phy(addr, PHY_BMSR, info.bmsr1);
        read_phy(addr, PHY_BMSR, info.bmsr2);

        read_phy(addr, PHY_ANAR, info.anar);
        read_phy(addr, PHY_ANLPAR, info.anlpar);
        read_phy(addr, PHY_SCSR, info.scsr);

        return info;
    }

    void print_phy_scan()
    {
        printf("\r\n--- PHY scan start ---\r\n");

        int found = 0;

        for (uint32_t addr = 0; addr < 32; addr++) {
            PhyInfo info = read_phy_info(addr);

            if (!info.found) {
                continue;
            }

            found++;
            print_phy_info(info);
        }

        if (found == 0) {
            printf("\r\nNo PHY found.\r\n");
            printf("Check MDIO/MDC, PHY reset, PHY power, REF_CLK, and PHY address straps.\r\n");
        }

        printf("--- PHY scan end ---\r\n");
    }

    void print_phy_info(const PhyInfo& info)
    {
        printf("\r\n=== PHY found at addr %u ===\r\n", info.address);
        printf("ID1     = 0x%04lX\r\n", info.id1 & 0xFFFF);
        printf("ID2     = 0x%04lX\r\n", info.id2 & 0xFFFF);
        printf("BMCR    = 0x%04lX\r\n", info.bmcr & 0xFFFF);
        printf("BMSR#1  = 0x%04lX\r\n", info.bmsr1 & 0xFFFF);
        printf("BMSR#2  = 0x%04lX\r\n", info.bmsr2 & 0xFFFF);
        printf("ANAR    = 0x%04lX\r\n", info.anar & 0xFFFF);
        printf("ANLPAR  = 0x%04lX\r\n", info.anlpar & 0xFFFF);
        printf("SCSR    = 0x%04lX\r\n", info.scsr & 0xFFFF);

        printf("Link    = %s\r\n", link_up(info) ? "UP" : "DOWN");
        printf("AutoNeg = %s\r\n", autoneg_complete(info) ? "COMPLETE" : "NOT COMPLETE");
        printf("Speed   = %s\r\n", speed_string(info));
    }

    bool link_up(const PhyInfo& info) const
    {
        return (info.bmsr2 & (1u << 2)) != 0;
    }

    bool autoneg_complete(const PhyInfo& info) const
    {
        return (info.bmsr2 & (1u << 5)) != 0;
    }

    const char* speed_string(const PhyInfo& info) const
    {
        const uint32_t hcd = (info.scsr >> 2) & 0x7;

        switch (hcd) {
        case 1:
            return "10M Half";
        case 5:
            return "10M Full";
        case 2:
            return "100M Half";
        case 6:
            return "100M Full";
        default:
            return "Unknown";
        }
    }

    void print_lwip_status()
    {
        if (netif_ == nullptr) {
            printf("netif = nullptr\r\n");
            return;
        }

        printf("netif flags       = 0x%02X\r\n", netif_->flags);
        printf("IP                = %s\r\n", ip4addr_ntoa(netif_ip4_addr(netif_)));
        printf("NETMASK           = %s\r\n", ip4addr_ntoa(netif_ip4_netmask(netif_)));
        printf("GW                = %s\r\n", ip4addr_ntoa(netif_ip4_gw(netif_)));
        printf("netif_is_up       = %d\r\n", netif_is_up(netif_));
        printf("netif_is_link_up  = %d\r\n", netif_is_link_up(netif_));
    }

    void print_eth_registers()
    {
        if (eth_ == nullptr || eth_->Instance == nullptr) {
            printf("ETH handle is null\r\n");
            return;
        }

        printf("heth.State        = %u\r\n", eth_->State);
        printf("ETH->DMASR        = 0x%08lX\r\n", ETH->DMASR);
        printf("ETH->DMAOMR       = 0x%08lX\r\n", ETH->DMAOMR);
        printf("ETH->MACCR        = 0x%08lX\r\n", ETH->MACCR);
        printf("ETH->MACFFR       = 0x%08lX\r\n", ETH->MACFFR);
    }

    void print_checksum_config()
    {
#ifdef CHECKSUM_BY_HARDWARE
        printf("CHECKSUM_BY_HARDWARE = %d\r\n", CHECKSUM_BY_HARDWARE);
#else
        printf("CHECKSUM_BY_HARDWARE = undefined\r\n");
#endif

#ifdef CHECKSUM_GEN_IP
        printf("CHECKSUM_GEN_IP      = %d\r\n", CHECKSUM_GEN_IP);
#endif
#ifdef CHECKSUM_GEN_UDP
        printf("CHECKSUM_GEN_UDP     = %d\r\n", CHECKSUM_GEN_UDP);
#endif
#ifdef CHECKSUM_GEN_TCP
        printf("CHECKSUM_GEN_TCP     = %d\r\n", CHECKSUM_GEN_TCP);
#endif
#ifdef CHECKSUM_GEN_ICMP
        printf("CHECKSUM_GEN_ICMP    = %d\r\n", CHECKSUM_GEN_ICMP);
#endif

#ifdef CHECKSUM_CHECK_IP
        printf("CHECKSUM_CHECK_IP    = %d\r\n", CHECKSUM_CHECK_IP);
#endif
#ifdef CHECKSUM_CHECK_UDP
        printf("CHECKSUM_CHECK_UDP   = %d\r\n", CHECKSUM_CHECK_UDP);
#endif
#ifdef CHECKSUM_CHECK_TCP
        printf("CHECKSUM_CHECK_TCP   = %d\r\n", CHECKSUM_CHECK_TCP);
#endif
#ifdef CHECKSUM_CHECK_ICMP
        printf("CHECKSUM_CHECK_ICMP  = %d\r\n", CHECKSUM_CHECK_ICMP);
#endif

        if (eth_ != nullptr) {
            printf("heth.Init.ChecksumMode = 0x%08lX\r\n", eth_->Init.ChecksumMode);
        }

        warn_checksum_mismatch();
    }

    void warn_checksum_mismatch()
    {
#if defined(CHECKSUM_BY_HARDWARE)
        if (eth_ == nullptr) {
            return;
        }

        if (CHECKSUM_BY_HARDWARE == 0 &&
            eth_->Init.ChecksumMode == ETH_CHECKSUM_BY_HARDWARE) {
            printf("WARNING: lwIP uses software checksum, but ETH HAL is hardware checksum mode.\r\n");
        }

        if (CHECKSUM_BY_HARDWARE == 1 &&
            eth_->Init.ChecksumMode == ETH_CHECKSUM_BY_SOFTWARE) {
            printf("WARNING: lwIP uses hardware checksum, but ETH HAL is software checksum mode.\r\n");
        }
#endif
    }

#if STM32_LIBRARY_ETHERNETIF_COUNTERS_ENABLED
    void print_ethernetif_counters()
    {
        printf("ethif_input_count     = %u\r\n", ethif_input_count);
        printf("ethif_low_input_count = %u\r\n", ethif_low_input_count);
        printf("ethif_rx_ok_count     = %u\r\n", ethif_rx_ok_count);
        printf("ethif_rx_null_count   = %u\r\n", ethif_rx_null_count);
        printf("ethif_output_count    = %u\r\n", ethif_output_count);
        printf("ethif_tx_ok_count     = %u\r\n", ethif_tx_ok_count);
        printf("ethif_tx_err_count    = %u\r\n", ethif_tx_err_count);
    }
#else
    void print_ethernetif_counters()
    {
        printf("ethernetif counters disabled\r\n");
    }
#endif

    void print_summary(bool include_counters = false)
    {
        printf("\r\n--- Ethernet Diagnostics ---\r\n");
        print_lwip_status();
        print_eth_registers();
        print_checksum_config();

        if (include_counters) {
            print_ethernetif_counters();
        }

        printf("--- Ethernet Diagnostics End ---\r\n");
    }

private:
    ETH_HandleTypeDef* eth_ = nullptr;
    struct netif* netif_ = nullptr;

    static constexpr uint32_t PHY_BMCR   = 0x00;
    static constexpr uint32_t PHY_BMSR   = 0x01;
    static constexpr uint32_t PHY_ID1    = 0x02;
    static constexpr uint32_t PHY_ID2    = 0x03;
    static constexpr uint32_t PHY_ANAR   = 0x04;
    static constexpr uint32_t PHY_ANLPAR = 0x05;
    static constexpr uint32_t PHY_SCSR   = 0x1F;
};

} // namespace stm32_library::stm32_peripherals

#endif // HAL_ETH_MODULE_ENABLED