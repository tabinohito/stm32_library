#include "main.h"

#ifdef HAL_ETH_MODULE_ENABLED

extern "C" {
#include "lwip/opt.h"
}

extern "C" void stm32_library_ethernet_pre_init_hook(ETH_HandleTypeDef* heth)
{
    if (heth == nullptr) {
        return;
    }

#if CHECKSUM_BY_HARDWARE
    heth->Init.ChecksumMode = ETH_CHECKSUM_BY_HARDWARE;
#else
    heth->Init.ChecksumMode = ETH_CHECKSUM_BY_SOFTWARE;
#endif
}

#endif // HAL_ETH_MODULE_ENABLED