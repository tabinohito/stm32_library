#include "../Inc/INA237.hpp"

#ifdef HAL_I2C_MODULE_ENABLED

namespace stm32_library::stm32_modules {

INA237::INA237(stm32_peripherals::I2c& i2c,
	               const uint8_t i2c_addr,
	               bool skipReset)
	    : INA2xx(i2c, i2c_addr, skipReset) {
	  if (!initialized_) {
	    return;
	  }

	  if (_device_id != INA237_DEVICE_ID) {
	    initialized_ = false;
	    return;
	  }
}

void INA237::_updateShuntCalRegister() {
  // Formula from INA237 datasheet:
  // SHUNT_CAL = 819.2e6 * CURRENT_LSB * RSHUNT * scale
  float scale = 1.0f;
  if (getADCRange()) {
    scale = 4.0f; // +/-40.96mV range
  }

  const float shunt_cal_f = 819.2e6f * _current_lsb * _shunt_res * scale;
  const uint16_t shunt_cal = static_cast<uint16_t>(shunt_cal_f);

  (void)writeRegister16(INA2XX_REG_SHUNTCAL, shunt_cal);
}

INA237::AlertType INA237::getAlertType(void) {
  // INA237 alert bits are DIAG_ALRT[11:5]
  return static_cast<AlertType>(readBits16(INA2XX_REG_DIAGALRT, 7, 5));
}

void INA237::setAlertType(AlertType alert) {
  // INA237 alert bits are DIAG_ALRT[11:5]
  writeBits16(INA2XX_REG_DIAGALRT, 7, 5, static_cast<uint16_t>(alert));
}

float INA237::readDieTemp(void) {
  uint16_t raw = 0;
  if (!readRegister16(INA2XX_REG_DIETEMP, raw)) {
    return 0.0f;
  }

  // INA237: temperature is bits[15:4], 125 m°C/LSB
  const int16_t t = static_cast<int16_t>(raw);
  return static_cast<float>(t >> 4) * 125.0f / 1000.0f;
}

float INA237::readBusVoltage(void) {
  uint16_t raw = 0;
  if (!readRegister16(INA2XX_REG_VBUS, raw)) {
    return 0.0f;
  }

  // INA237: 3.125 mV/LSB
  return static_cast<float>(raw) * 3.125f / 1000.0f;
}

float INA237::readShuntVoltage(void) {
  float scale_uv = 5.0f; // 5 uV/LSB
  if (getADCRange()) {
    scale_uv = 1.25f; // 1.25 uV/LSB
  }

  uint16_t raw = 0;
  if (!readRegister16(INA2XX_REG_VSHUNT, raw)) {
    return 0.0f;
  }

  const int16_t v = static_cast<int16_t>(raw);
  return static_cast<float>(v) * scale_uv / 1000000.0f;
}

float INA237::readCurrent(void) {
  uint16_t raw = 0;
  if (!readRegister16(INA2XX_REG_CURRENT, raw)) {
    return 0.0f;
  }

  const int16_t i = static_cast<int16_t>(raw);
  return static_cast<float>(i) * _current_lsb * 1000.0f;
}

float INA237::readPower(void) {
  uint32_t raw = 0;
  if (!readRegister24(INA2XX_REG_POWER, raw)) {
    return 0.0f;
  }

  // Adafruit implementation follows 0.2 * current_lsb
  return static_cast<float>(raw) * 0.2f * _current_lsb * 1000.0f;
}

void INA237::setShunt(float shunt_res, float max_current) {
  _shunt_res = shunt_res;
  _current_lsb = max_current / static_cast<float>(1UL << 15);
  _updateShuntCalRegister();
}

} // namespace stm32_library::stm32_modules

#endif
