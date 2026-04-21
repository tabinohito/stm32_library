#include "../Inc/INA2xx.hpp"

#ifdef HAL_I2C_MODULE_ENABLED

namespace stm32_library::stm32_modules {
INA2xx::INA2xx(stm32_peripherals::I2c& i2c,
			   const uint8_t i2c_addr,
			   bool skipReset)
	: i2c_(i2c), i2c_addr_(i2c_addr) {
  if (!i2c_.is_device_ready(i2c_addr_)) {
	initialized_ = false;
	return;
  }

  uint16_t mfg_id = 0;
  if (!readRegister16(INA2XX_REG_MFG_UID, mfg_id)) {
	initialized_ = false;
	return;
  }
  if (mfg_id != 0x5449) {
	initialized_ = false;
	return;
  }

  uint16_t dev_uid = 0;
  if (!readRegister16(INA2XX_REG_DVC_UID, dev_uid)) {
	initialized_ = false;
	return;
  }
  _device_id = (dev_uid >> 4) & 0x0FFF;

  if (!skipReset) {
	reset();
	HAL_Delay(2);
  }

  initialized_ = true;
}

void INA2xx::reset(void) {
  writeBits16(INA2XX_REG_CONFIG, 1, 15, 1);
  writeBits16(INA2XX_REG_DIAGALRT, 1, 14, 1);
  setMode(MeasurementMode::Continuous);
}

void INA2xx::_updateShuntCalRegister(void) {
  // derived class override
}

void INA2xx::setShunt(float shunt_res, float max_current) {
  _shunt_res = shunt_res;
  _current_lsb = max_current / static_cast<float>(1UL << 19);
  _updateShuntCalRegister();
}

void INA2xx::setADCRange(uint8_t adc_range) {
  writeBits16(INA2XX_REG_CONFIG, 1, 4, adc_range & 0x01);
  _updateShuntCalRegister();
}

uint8_t INA2xx::getADCRange(void) {
  return static_cast<uint8_t>(readBits16(INA2XX_REG_CONFIG, 1, 4));
}

float INA2xx::readDieTemp(void) {
  uint16_t raw = 0;
  if (!readRegister16(INA2XX_REG_DIETEMP, raw)) {
    return 0.0f;
  }
  int16_t t = static_cast<int16_t>(raw);
  return static_cast<float>(t) * 7.8125f / 1000.0f;
}

float INA2xx::readBusVoltage(void) {
  uint32_t raw = 0;
  if (!readRegister24(INA2XX_REG_VBUS, raw)) {
    return 0.0f;
  }
  return static_cast<float>(raw >> 4) * 195.3125f / 1e6f;
}

float INA2xx::getBusVoltage_V(void) {
  return readBusVoltage();
}

float INA2xx::readCurrent(void) {
  uint32_t raw = 0;
  if (!readRegister24(INA2XX_REG_CURRENT, raw)) {
    return 0.0f;
  }
  int32_t i = signExtend24(raw);
  return static_cast<float>(i) / 16.0f * _current_lsb * 1000.0f;
}

float INA2xx::getCurrent_mA(void) {
  return readCurrent();
}

float INA2xx::readShuntVoltage(void) {
  float scale = getADCRange() ? 78.125f : 312.5f;

  uint32_t raw = 0;
  if (!readRegister24(INA2XX_REG_VSHUNT, raw)) {
    return 0.0f;
  }
  int32_t v = signExtend24(raw);
  return static_cast<float>(v) / 16.0f * scale / 1000000.0f;
}

float INA2xx::getShuntVoltage_mV(void) {
  return readShuntVoltage();
}

float INA2xx::readPower(void) {
  uint32_t raw = 0;
  if (!readRegister24(INA2XX_REG_POWER, raw)) {
    return 0.0f;
  }
  return static_cast<float>(raw) * 3.2f * _current_lsb * 1000.0f;
}

float INA2xx::getPower_mW(void) {
  return readPower();
}

INA2xx::MeasurementMode INA2xx::getMode(void) {
  return static_cast<MeasurementMode>(readBits16(INA2XX_REG_ADCCFG, 4, 12));
}

void INA2xx::setMode(MeasurementMode mode) {
  writeBits16(INA2XX_REG_ADCCFG, 4, 12, static_cast<uint16_t>(mode));
}

INA2xx::AveragingCount INA2xx::getAveragingCount(void) {
  return static_cast<AveragingCount>(readBits16(INA2XX_REG_ADCCFG, 3, 0));
}

void INA2xx::setAveragingCount(AveragingCount count) {
  writeBits16(INA2XX_REG_ADCCFG, 3, 0, static_cast<uint16_t>(count));
}

INA2xx::ConversionTime INA2xx::getCurrentConversionTime(void) {
  return static_cast<ConversionTime>(readBits16(INA2XX_REG_ADCCFG, 3, 6));
}

void INA2xx::setCurrentConversionTime(ConversionTime time) {
  writeBits16(INA2XX_REG_ADCCFG, 3, 6, static_cast<uint16_t>(time));
}

INA2xx::ConversionTime INA2xx::getVoltageConversionTime(void) {
  return static_cast<ConversionTime>(readBits16(INA2XX_REG_ADCCFG, 3, 9));
}

void INA2xx::setVoltageConversionTime(ConversionTime time) {
  writeBits16(INA2XX_REG_ADCCFG, 3, 9, static_cast<uint16_t>(time));
}

INA2xx::ConversionTime INA2xx::getTemperatureConversionTime(void) {
  return static_cast<ConversionTime>(readBits16(INA2XX_REG_ADCCFG, 3, 3));
}

void INA2xx::setTemperatureConversionTime(ConversionTime time) {
  writeBits16(INA2XX_REG_ADCCFG, 3, 3, static_cast<uint16_t>(time));
}

bool INA2xx::conversionReady(void) {
  return readBits16(INA2XX_REG_DIAGALRT, 1, 1) != 0;
}

INA2xx::AlertPolarity INA2xx::getAlertPolarity(void) {
  return static_cast<AlertPolarity>(readBits16(INA2XX_REG_DIAGALRT, 1, 12));
}

void INA2xx::setAlertPolarity(AlertPolarity polarity) {
  writeBits16(INA2XX_REG_DIAGALRT, 1, 12, static_cast<uint16_t>(polarity));
}

INA2xx::AlertLatch INA2xx::getAlertLatch(void) {
  return static_cast<AlertLatch>(readBits16(INA2XX_REG_DIAGALRT, 1, 15));
}

void INA2xx::setAlertLatch(AlertLatch state) {
  writeBits16(INA2XX_REG_DIAGALRT, 1, 15, static_cast<uint16_t>(state));
}

uint16_t INA2xx::alertFunctionFlags(void) {
  return readBits16(INA2XX_REG_DIAGALRT, 12, 0);
}

bool INA2xx::readRegister16(uint8_t reg, uint16_t& value) const {
  uint8_t buf[2] = {0, 0};
  if (!i2c_.read_reg(i2c_addr_, reg, buf, 2)) {
    value = 0;
    return false;
  }

  value = (static_cast<uint16_t>(buf[0]) << 8) |
          static_cast<uint16_t>(buf[1]);
  return true;
}

bool INA2xx::readRegister24(uint8_t reg, uint32_t& value) const {
  uint8_t buf[3] = {0, 0, 0};
  if (!i2c_.read_reg(i2c_addr_, reg, buf, 3)) {
    value = 0;
    return false;
  }

  value = (static_cast<uint32_t>(buf[0]) << 16) |
          (static_cast<uint32_t>(buf[1]) << 8) |
          static_cast<uint32_t>(buf[2]);
  return true;
}

bool INA2xx::writeRegister16(uint8_t reg, uint16_t value) const {
  uint8_t buf[3] = {
      reg,
      static_cast<uint8_t>((value >> 8) & 0xFF),
      static_cast<uint8_t>(value & 0xFF),
  };

  return i2c_.write(i2c_addr_, reinterpret_cast<char*>(buf), sizeof(buf));
}

uint16_t INA2xx::readBits16(uint8_t reg, uint8_t bit_count, uint8_t shift) const {
  uint16_t value = 0;
  if (!readRegister16(reg, value)) {
    return 0;
  }

  const uint16_t mask = static_cast<uint16_t>((1u << bit_count) - 1u);
  return static_cast<uint16_t>((value >> shift) & mask);
}

void INA2xx::writeBits16(uint8_t reg, uint8_t bit_count, uint8_t shift, uint16_t value) const {
  uint16_t current = 0;
  if (!readRegister16(reg, current)) {
    return;
  }

  const uint16_t mask = static_cast<uint16_t>(((1u << bit_count) - 1u) << shift);
  current = static_cast<uint16_t>((current & ~mask) | ((value << shift) & mask));
  (void)writeRegister16(reg, current);
}

int32_t INA2xx::signExtend24(uint32_t value) {
  if (value & 0x800000UL) {
    value |= 0xFF000000UL;
  }
  return static_cast<int32_t>(value);
}

} // namespace stm32_library::stm32_modules

#endif
