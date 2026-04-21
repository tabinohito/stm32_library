#include "../Inc/INA238.hpp"

#ifdef HAL_I2C_MODULE_ENABLED

namespace stm32_library::stm32_modules {

INA238::INA238(stm32_peripherals::I2c& i2c,
               uint8_t i2c_addr,
               bool skipReset)
    : INA237(i2c, i2c_addr, skipReset) {}

} // namespace stm32_library::stm32_modules

#endif
