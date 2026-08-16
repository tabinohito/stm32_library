#pragma once

#include "main.h"

#ifdef HAL_I2C_MODULE_ENABLED

#include "INA2xx.hpp"
#include "stm32_library/peripherals/stm32_peripherals.hpp"
#include <cstdint>

namespace stm32_library::stm32_modules {
namespace stm32_peripherals = stm32_library::stm32_peripherals;

class INA237 : public INA2xx {
public:
  enum class DmaMeasurement : uint8_t {
    Current,
    BusVoltage,
    ShuntVoltage,
    Power,
    DieTemp,
  };

  enum class DmaStatus : uint8_t {
    Idle,
    Busy,
    Complete,
    Error,
  };

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

  /*
   * Non-blocking sensor read.  Only one INA device sharing an I2C instance
   * may have a transfer in flight at a time.
   */
  bool startReadDma(DmaMeasurement measurement);
  DmaStatus pollReadDma(float& value);
  bool readDmaBusy() const;

  static constexpr uint8_t INA237_I2CADDR_DEFAULT = 0x40; ///< INA237/INA238 default i2c address
  static constexpr uint16_t INA237_DEVICE_ID = 0x238;     ///< INA237 device ID

protected:
  void _updateShuntCalRegister(void) override;

private:
  DmaMeasurement dma_measurement_ = DmaMeasurement::Current;
  uint8_t dma_buffer_[3] = {};
  bool dma_active_ = false;
};

} // namespace stm32_library::stm32_modules

#endif
