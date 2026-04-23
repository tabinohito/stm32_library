#pragma once

#include "../Inc/MCP320x.hpp"

#ifdef HAL_SPI_MODULE_ENABLED

#define div_round(n, d) (((n) + ((d) >> 2)) / (d))

namespace stm32_library::stm32_modules {

// channel configurations
using MCP3201Ch = MCP320xTypes::MCP3201::Channel;
using MCP3202Ch = MCP320xTypes::MCP3202::Channel;
using MCP3204Ch = MCP320xTypes::MCP3204::Channel;
using MCP3208Ch = MCP320xTypes::MCP3208::Channel;

template <typename T>
MCP320x<T>::MCP320x(uint16_t vref, stm32_peripherals::DigitalOut &cs,
                    stm32_peripherals::Spi &spi)
    : mVref(vref), mSplSpeed(0), spi_(spi), cs_(cs) {}

template <typename T>
void MCP320x<T>::calibrate(Channel ch) {
  mSplSpeed = testSplSpeed(ch, 256);
}

template <typename T>
uint16_t MCP320x<T>::read(Channel ch) const {
  return execute(createCmd(ch));
}

template <typename T>
uint32_t MCP320x<T>::testSplSpeed(Channel ch) const {
  return testSplSpeed(ch, 64);
}

template <typename T>
uint32_t MCP320x<T>::testSplSpeed(Channel ch, uint16_t num) const {
  auto cmd = createCmd(ch);

  uint32_t t1 = HAL_GetTick_us();
  for (uint16_t i = 0; i < num; i++) {
    execute(cmd);
  }
  uint32_t t2 = HAL_GetTick_us();

  return div_round((t2 - t1) * 1000, num);
}

template <typename T>
uint32_t MCP320x<T>::testSplSpeed(Channel ch, uint16_t num, uint32_t splFreq) {
  uint16_t delay = getSplDelay(ch, splFreq);
  auto cmd = createCmd(ch);

  uint32_t t1 = HAL_GetTick_us();
  for (uint16_t i = 0; i < num; i++) {
    execute(cmd);
    HAL_Delay_us(delay);
  }
  uint32_t t2 = HAL_GetTick_us();

  return div_round((t2 - t1) * 1000, num);
}

template <typename T>
uint16_t MCP320x<T>::toAnalog(uint16_t raw) const {
  return (static_cast<uint32_t>(raw) * mVref) / (kRes - 1);
}

template <typename T>
uint16_t MCP320x<T>::toDigital(uint16_t val) const {
  return (static_cast<uint32_t>(val) * (kRes - 1)) / mVref;
}

template <typename T>
uint16_t MCP320x<T>::getVref() const {
  return mVref;
}

template <typename T>
uint16_t MCP320x<T>::getAnalogRes() const {
  return (static_cast<uint32_t>(mVref) * 1000) / (kRes - 1);
}

template <typename T>
uint16_t MCP320x<T>::getSplDelay(Channel ch, uint32_t splFreq) {
  uint32_t splTime = div_round(1000000000, splFreq);

  if (!mSplSpeed) {
    calibrate(ch);
  }

  int32_t delay = static_cast<int32_t>(splTime) -
                  static_cast<int32_t>(mSplSpeed);
  delay /= 1000;

  return (delay < 0) ? 0 : static_cast<uint16_t>(delay);
}

template <>
MCP3201::Command<MCP3201Ch> MCP3201::createCmd(MCP3201Ch ch) {
  (void)ch;
  return {};
}

template <>
MCP3202::Command<MCP3202Ch> MCP3202::createCmd(MCP3202Ch ch) {
  return {
      .value = static_cast<uint16_t>(0x0120 | (ch << 6)),
  };
}

template <>
MCP3204::Command<MCP3204Ch> MCP3204::createCmd(MCP3204Ch ch) {
  return {
      .value = static_cast<uint16_t>(0x0400 | (ch << 6)),
  };
}

template <>
MCP3208::Command<MCP3208Ch> MCP3208::createCmd(MCP3208Ch ch) {
  return {
      .value = static_cast<uint16_t>(0x0400 | (ch << 6)),
  };
}

template <>
uint16_t MCP3201::execute(Command<MCP3201Ch> cmd) const {
  (void)cmd;
  return transfer();
}

template <>
uint16_t MCP3202::execute(Command<MCP3202Ch> cmd) const {
  return transfer(cmd);
}

template <>
uint16_t MCP3204::execute(Command<MCP3204Ch> cmd) const {
  return transfer(cmd);
}

template <>
uint16_t MCP3208::execute(Command<MCP3208Ch> cmd) const {
  return transfer(cmd);
}

template <typename T>
uint16_t MCP320x<T>::transfer() const {
  SpiData adc{};

  cs_.write(false);

  adc.hiByte = spi_.write(0x00) & 0x1F;
  adc.loByte = spi_.write(0x00);

  cs_.write(true);

  return (adc.value >> 1);
}

template <typename T>
uint16_t MCP320x<T>::transfer(SpiData cmd) const {
  SpiData adc{};

  cs_.write(false);

  spi_.write(cmd.hiByte);
  adc.hiByte = spi_.write(cmd.loByte) & 0x0F;
  adc.loByte = spi_.write(0x00);

  cs_.write(true);

  return adc.value;
}

/*
 * Explicit template instantiation for the channel types.
 */
template class MCP320x<MCP3201Ch>;
template class MCP320x<MCP3202Ch>;
template class MCP320x<MCP3204Ch>;
template class MCP320x<MCP3208Ch>;

} // namespace stm32_library::stm32_modules

#endif
