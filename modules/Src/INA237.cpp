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

bool INA237::startReadDma(DmaMeasurement measurement) {
  if (dma_active_) {
    return false;
  }

  uint8_t reg = 0;
  size_t length = 2;

  switch (measurement) {
  case DmaMeasurement::Current:
    reg = INA2XX_REG_CURRENT;
    break;
  case DmaMeasurement::BusVoltage:
    reg = INA2XX_REG_VBUS;
    break;
  case DmaMeasurement::ShuntVoltage:
    reg = INA2XX_REG_VSHUNT;
    break;
  case DmaMeasurement::Power:
    reg = INA2XX_REG_POWER;
    length = 3;
    break;
  case DmaMeasurement::DieTemp:
    reg = INA2XX_REG_DIETEMP;
    break;
  }

  dma_measurement_ = measurement;
  dma_buffer_[0] = 0;
  dma_buffer_[1] = 0;
  dma_buffer_[2] = 0;

  if (
      i2c_.start_read_reg_dma(
          i2c_addr_,
          reg,
          dma_buffer_,
          length
      ) != HAL_OK
  ) {
    return false;
  }

  dma_active_ = true;
  return true;
}

INA237::DmaStatus INA237::pollReadDma(float& value) {
  if (!dma_active_) {
    return DmaStatus::Idle;
  }

  const auto status = i2c_.poll_dma();
  if (status == stm32_peripherals::I2c::DmaStatus::Busy) {
    return DmaStatus::Busy;
  }

  dma_active_ = false;

  if (status != stm32_peripherals::I2c::DmaStatus::Complete) {
    value = 0.0f;
    return DmaStatus::Error;
  }

  const uint16_t raw16 =
      (static_cast<uint16_t>(dma_buffer_[0]) << 8) |
      static_cast<uint16_t>(dma_buffer_[1]);

  switch (dma_measurement_) {
  case DmaMeasurement::Current:
    value =
        static_cast<float>(static_cast<int16_t>(raw16)) *
        _current_lsb *
        1000.0f;
    break;

  case DmaMeasurement::BusVoltage:
    value = static_cast<float>(raw16) * 3.125f / 1000.0f;
    break;

  case DmaMeasurement::ShuntVoltage: {
    const float scale_uv = _adc_range != 0U ? 1.25f : 5.0f;
    value =
        static_cast<float>(static_cast<int16_t>(raw16)) *
        scale_uv /
        1000000.0f;
    break;
  }

  case DmaMeasurement::Power: {
    const uint32_t raw24 =
        (static_cast<uint32_t>(dma_buffer_[0]) << 16) |
        (static_cast<uint32_t>(dma_buffer_[1]) << 8) |
        static_cast<uint32_t>(dma_buffer_[2]);
    value =
        static_cast<float>(raw24) *
        0.2f *
        _current_lsb *
        1000.0f;
    break;
  }

  case DmaMeasurement::DieTemp:
    value =
        static_cast<float>(static_cast<int16_t>(raw16) >> 4) *
        125.0f /
        1000.0f;
    break;
  }

  return DmaStatus::Complete;
}

bool INA237::readDmaBusy() const {
  return dma_active_;
}

} // namespace stm32_library::stm32_modules

#endif
