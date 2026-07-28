#pragma once

#include "main.h"

#ifdef HAL_I2C_MODULE_ENABLED

#include "stm32_library/peripherals/stm32_peripherals.hpp"
#include <cstdint>

namespace stm32_library::stm32_modules {
namespace stm32_peripherals = stm32_library::stm32_peripherals;

class INA2xx {
	public:
		INA2xx(stm32_peripherals::I2c& i2c,
			   uint8_t i2c_addr = INA2XX_I2CADDR_DEFAULT,
			   bool skipReset = false);
		virtual ~INA2xx() = default;

		static constexpr uint8_t INA2XX_REG_CONFIG      = 0x00;
		static constexpr uint8_t INA2XX_REG_ADCCFG      = 0x01;
		static constexpr uint8_t INA2XX_REG_SHUNTCAL    = 0x02;
		static constexpr uint8_t INA2XX_REG_VSHUNT      = 0x04;
		static constexpr uint8_t INA2XX_REG_VBUS        = 0x05;
		static constexpr uint8_t INA2XX_REG_DIETEMP     = 0x06;
		static constexpr uint8_t INA2XX_REG_CURRENT     = 0x07;
		static constexpr uint8_t INA2XX_REG_POWER       = 0x08;
		static constexpr uint8_t INA2XX_REG_DIAGALRT    = 0x0B;
		static constexpr uint8_t INA2XX_REG_SOVL        = 0x0C;
		static constexpr uint8_t INA2XX_REG_SUVL        = 0x0D;
		static constexpr uint8_t INA2XX_REG_BOVL        = 0x0E;
		static constexpr uint8_t INA2XX_REG_BUVL        = 0x0F;
		static constexpr uint8_t INA2XX_REG_TEMPLIMIT   = 0x10;
		static constexpr uint8_t INA2XX_REG_PWRLIMIT    = 0x11;
		static constexpr uint8_t INA2XX_REG_MFG_UID     = 0x3E;
		static constexpr uint8_t INA2XX_REG_DVC_UID     = 0x3F;

		static constexpr uint8_t INA2XX_I2CADDR_DEFAULT = 0x40;

		enum class MeasurementMode : uint8_t {
			Shutdown = 0x00,
			TrigBus = 0x01,
			TrigShunt = 0x02,
			TrigBusShunt = 0x03,
			TrigTemp = 0x04,
			TrigTempBus = 0x05,
			TrigTempShunt = 0x06,
			TrigTempBusShunt = 0x07,

			Shutdown2 = 0x08,
			ContBus = 0x09,
			ContShunt = 0x0A,
			ContBusShunt = 0x0B,
			ContTemp = 0x0C,
			ContTempBus = 0x0D,
			ContTempShunt = 0x0E,
			ContTempBusShunt = 0x0F,

			Triggered = TrigTempBusShunt,
			Continuous = ContTempBusShunt
		};

		enum class ConversionTime : uint8_t {
			Time50us = 0,
			Time84us,
			Time150us,
			Time280us,
			Time540us,
			Time1052us,
			Time2074us,
			Time4120us,
		};

		enum class AveragingCount : uint8_t {
			Count1 = 0,
			Count4,
			Count16,
			Count64,
			Count128,
			Count256,
			Count512,
			Count1024,
		};

		enum class AlertPolarity : uint8_t {
			Normal = 0x0,
			Inverted = 0x1,
		};

		enum class AlertLatch : uint8_t {
			Transparent = 0x0,
			Enabled = 0x1,
		};

		using INA2XX_MeasurementMode = MeasurementMode;
		using INA2XX_ConversionTime = ConversionTime;
		using INA2XX_AveragingCount = AveragingCount;
		using INA2XX_AlertPolarity = AlertPolarity;
		using INA2XX_AlertLatch = AlertLatch;

		virtual void reset(void);

		virtual void setShunt(float shunt_res = 0.1f, float max_current = 3.2f);
		void setADCRange(uint8_t adc_range);
		uint8_t getADCRange(void);

		virtual float readDieTemp(void);
		virtual float readBusVoltage(void);
		virtual float readCurrent(void);
		virtual float readShuntVoltage(void);
		virtual float readPower(void);

		float getBusVoltage_V(void);
		float getShuntVoltage_mV(void);
		float getCurrent_mA(void);
		float getPower_mW(void);

		void setMode(MeasurementMode mode);
		MeasurementMode getMode(void);

		bool conversionReady(void);
		uint16_t alertFunctionFlags(void);

		AlertLatch getAlertLatch(void);
		void setAlertLatch(AlertLatch state);
		AlertPolarity getAlertPolarity(void);
		void setAlertPolarity(AlertPolarity polarity);

		ConversionTime getCurrentConversionTime(void);
		void setCurrentConversionTime(ConversionTime time);
		ConversionTime getVoltageConversionTime(void);
		void setVoltageConversionTime(ConversionTime time);
		ConversionTime getTemperatureConversionTime(void);
		void setTemperatureConversionTime(ConversionTime time);
		AveragingCount getAveragingCount(void);
		void setAveragingCount(AveragingCount count);

		bool ok() const { return initialized_; }

	protected:
		virtual void _updateShuntCalRegister(void);

		bool readRegister16(uint8_t reg, uint16_t& value) const;
		bool readRegister24(uint8_t reg, uint32_t& value) const;
		bool writeRegister16(uint8_t reg, uint16_t value) const;

		uint16_t readBits16(uint8_t reg, uint8_t bit_count, uint8_t shift) const;
		void writeBits16(uint8_t reg, uint8_t bit_count, uint8_t shift, uint16_t value) const;

		static int32_t signExtend24(uint32_t value);

		stm32_peripherals::I2c& i2c_;
		const uint8_t i2c_addr_;
		bool initialized_ = false;

		float _shunt_res = 0.1f;
		float _current_lsb = 0.0f;
		uint16_t _device_id = 0;
		uint8_t _adc_range = 0;
};

} // namespace stm32_library::stm32_modules

#endif
