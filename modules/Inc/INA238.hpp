#pragma once

#include "main.h"

#ifdef HAL_I2C_MODULE_ENABLED

#include "INA237.hpp"
#include "stm32_library/peripherals/stm32_peripherals.hpp"
#include <cstdint>

namespace stm32_library::stm32_modules {
namespace stm32_peripherals = stm32_library::stm32_peripherals;

class INA238 : public INA237 {
public:
  INA238(stm32_peripherals::I2c& i2c,
		 const uint8_t i2c_addr = INA238_I2CADDR_DEFAULT,
		 bool skipReset = false);

  static constexpr uint8_t INA238_I2CADDR_DEFAULT = 0x40; ///< INA237/INA238 default i2c address
  static constexpr uint16_t INA238_DEVICE_ID = 0x238;     ///< INA238 device ID
};
}

#endif
