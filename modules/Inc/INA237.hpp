#pragma once

#include "main.h"
#include "INA2xx.hpp"

#ifdef HAL_I2C_MODULE_ENABLED

#include "stm32_library/peripherals/stm32_peripherals.hpp"
#include <cstdint>

namespace stm32_library::stm32_modules {
namespace stm32_peripherals = stm32_library::stm32_peripherals;

class INA237 : public INA2xx {
public:
  enum class AlertType : uint8_t {
    ConversionReady = 0x01, ///< Trigger on conversion ready
    Overtemperature = 0x02, ///< Trigger on overtemperature
    Overpower       = 0x04, ///< Trigger on power over limit
    Undervoltage    = 0x08, ///< Trigger on bus voltage under limit
    Overvoltage     = 0x10, ///< Trigger on bus voltage over limit
    Undershunt      = 0x20, ///< Trigger on shunt voltage under limit
    Overshunt       = 0x40, ///< Trigger on shunt voltage over limit
    None            = 0x00, ///< Do not trigger alert pin (Default)
  };

  using INA237_AlertType = AlertType;

  INA237(stm32_peripherals::I2c& i2c,
         const uint8_t i2c_addr = INA237_I2CADDR_DEFAULT,
         bool skipReset = false);

  AlertType getAlertType(void);
  void setAlertType(AlertType alert);

  float readDieTemp(void) override;
  float readBusVoltage(void) override;
  float readShuntVoltage(void) override;
  float readCurrent(void) override;
  float readPower(void) override;
  void setShunt(float shunt_res = 0.1f, float max_current = 3.2f) override;

  static constexpr uint8_t INA237_I2CADDR_DEFAULT = 0x40; ///< INA237/INA238 default i2c address
  static constexpr uint16_t INA237_DEVICE_ID = 0x238;     ///< INA237 device ID

protected:
  void _updateShuntCalRegister(void) override;
};

} // namespace stm32_library::stm32_modules

#endif
