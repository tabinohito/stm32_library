#pragma once

#include "main.h"

#ifdef HAL_ETH_MODULE_ENABLED

#include <cstdint>
#include "ethernet_diagnostics.hpp"

#ifdef __cplusplus
extern "C" {
#endif

void MX_LWIP_Process(void);

#ifdef __cplusplus
}
#endif

namespace stm32_library::stm32_peripherals {

class EthernetService {
public:
    explicit EthernetService(EthernetDiagnostics& diagnostics)
        : diagnostics_(diagnostics)
    {
    }

    void tick()
    {
        MX_LWIP_Process();

        const uint32_t now = HAL_GetTick();

        if (periodic_print_enabled_ &&
            static_cast<uint32_t>(now - last_print_ms_) >= print_interval_ms_) {
            last_print_ms_ = now;
            diagnostics_.print_summary(include_counters_);
        }
    }

    void enable_periodic_print(uint32_t interval_ms, bool include_counters = false)
    {
        periodic_print_enabled_ = true;
        print_interval_ms_ = interval_ms;
        include_counters_ = include_counters;
        last_print_ms_ = HAL_GetTick();
    }

    void disable_periodic_print()
    {
        periodic_print_enabled_ = false;
    }

    void print_once(bool include_counters = false)
    {
        diagnostics_.print_summary(include_counters);
    }

    void print_phy_scan()
    {
        diagnostics_.print_phy_scan();
    }

private:
    EthernetDiagnostics& diagnostics_;
    bool periodic_print_enabled_ = false;
    bool include_counters_ = false;
    uint32_t print_interval_ms_ = 1000;
    uint32_t last_print_ms_ = 0;
};

} // namespace stm32_library::stm32_peripherals

#endif // HAL_ETH_MODULE_ENABLED