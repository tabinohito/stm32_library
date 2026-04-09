#pragma once

#include "main.h"

#ifdef HAL_SPI_MODULE_ENABLED

#include "stm32_library/peripherals/stm32_peripherals.hpp"
#include <optional>

namespace stm32_library::stm32_modules {
namespace stm32_peripherals = stm32_library::stm32_peripherals;
class Amt252d {
	public:
		Amt252d() = default;
		Amt252d(stm32_peripherals::DigitalOut &cs, stm32_peripherals::Spi &spi, bool is_reverse = false);
		uint16_t get_position();
		int16_t get_turn();
		bool reset_encoder();
		bool set_zero_point();
		bool read_position_turn();
		bool read_position();

	private:
	  stm32_peripherals::Spi &spi_;
	  stm32_peripherals::DigitalOut &cs_;
	  int16_t value_ = 0;
	  std::optional<int16_t> pre_value_;
	  int32_t count_ = 0;
	  int16_t delta_count_ = 0;
	  float rps_ = 0;
	  float encrot_to_pos_ = 1;
	  int dir_ = 1;
	  static constexpr int16_t cpr_ = 4095;
	  bool checksum(uint8_t high_byte, uint8_t low_byte) ;
	  int16_t signed14(uint16_t raw);
	  uint16_t pos = 0;
	  int16_t turn = 0;
};

}
#endif
