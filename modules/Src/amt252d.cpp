#pragma once
#include "../Inc/amt252d.hpp"

#ifdef HAL_SPI_MODULE_ENABLED

namespace stm32_library::stm32_modules {
	Amt252d::Amt252d(
			stm32_peripherals::DigitalOut &cs,
			stm32_peripherals::Spi &spi,
			bool is_reverse)
	  : Amt252d(cs, spi, ReadMode::LegacyNop, is_reverse) {
	}

	Amt252d::Amt252d(
			stm32_peripherals::DigitalOut &cs,
			stm32_peripherals::Spi &spi,
			ReadMode read_mode,
			bool is_reverse)
	  : spi_(spi), cs_(cs) {
		(void)is_reverse;
		cs_ = 1;
		set_read_mode(read_mode);
	}

	bool Amt252d::checksum(uint8_t high_byte, uint8_t low_byte) {
		  auto h = [&](uint8_t i) { return (bool)((high_byte >> i) & 0x01); };
		  auto l = [&](uint8_t i) { return (bool)((low_byte >> i) & 0x01); };
		  bool k1 = !(h(5) ^ h(3) ^ h(1) ^ l(7) ^ l(5) ^ l(3) ^ l(1));
		  bool k0 = !(h(4) ^ h(2) ^ h(0) ^ l(6) ^ l(4) ^ l(2) ^ l(0));
		  return (k1 == h(7)) && (k0 == h(6));
	  }
	int16_t Amt252d::signed14(uint16_t raw) {
	  raw &= 0x3FFF; // 14bitマスク
	  if (raw & 0x2000) {
		  return static_cast<int16_t>(raw | 0xC000);  // 上位2bitを1に
	  } else {
		  return static_cast<int16_t>(raw);
	  }
	}

	bool Amt252d::reset_encoder(){
	  uint8_t tx_data[2] = {0x00, 0x60};
	  uint8_t rx_data[2] = {0x00, 0x00};

	  cs_ = 0;
	  spi_.write(tx_data, rx_data, 2);
	  cs_ = 1;

	  if (checksum(rx_data[0], rx_data[1])) {
		  uint16_t raw = ((uint16_t)(rx_data[0] & 0x3F) << 8) | (uint16_t)rx_data[1];
		  pos = raw;
		  return true;
	  }
	  return false;
	}

	bool Amt252d::set_zero_point(){
	  uint8_t tx_data[2] = {0x00, 0x70};
	  uint8_t rx_data[2] = {0x00, 0x00};

	  cs_ = 0;
	  spi_.write(tx_data, rx_data, 2);
	  cs_ = 1;

	  if (checksum(rx_data[0], rx_data[1])) {
		  uint16_t raw = ((uint16_t)(rx_data[0] & 0x3F) << 8) | (uint16_t)rx_data[1];
		  pos = raw;
		  return true;
	  }
	  return false;
	}

	bool Amt252d::decode_position_turn(const uint8_t (&rx_data)[4]) {
	  bool ok = true;

	  if (checksum(rx_data[0], rx_data[1])) {
		  uint16_t raw_pos = ((uint16_t)(rx_data[0] & 0x3F) << 8) | rx_data[1];
		  pos = raw_pos;
	  } else {
		  ok = false;
	  }

	  if (checksum(rx_data[2], rx_data[3])) {
		  uint16_t raw_turn = ((uint16_t)(rx_data[2] & 0x3F) << 8) | rx_data[3];
		  turn = signed14(raw_turn);
	  } else {
		  ok = false;
	  }

	  return ok;
	}

	bool Amt252d::read_position_turn_legacy_nop() {
	  uint8_t tx_data[4] = {0x00, 0xA0, 0x00, 0x00}; // Multi-turn position read command
	  uint8_t rx_data[4] = {};

	  cs_ = 0;
	  for(size_t i = 0; i < sizeof(rx_data);i++){
		  rx_data[i] = spi_.write(tx_data[i]);
		  for (volatile int i = 0; i < 1000; ++i) __NOP();
	  }
	  cs_ = 1;

	  return decode_position_turn(rx_data);
	}

	void Amt252d::finish_timed_transfer() {
	  using CycleCounter = stm32_peripherals::CycleCounter;
	  CycleCounter::delay_cycles(tr_cycles_);
	  cs_ = 1;
	  last_cs_release_cycle_ = CycleCounter::now();
	  has_last_cs_release_ = true;
	}

	bool Amt252d::read_position_turn_timed_register_polling() {
	  using CycleCounter = stm32_peripherals::CycleCounter;

	  if (!read_mode_ready_ || !CycleCounter::is_enabled()) {
		  return false;
	  }

	  if (has_last_cs_release_) {
		  CycleCounter::wait_elapsed(last_cs_release_cycle_, tcs_cycles_);
	  }

	  constexpr uint8_t tx_data[4] = {0x00, 0xA0, 0x00, 0x00};
	  uint8_t rx_data[4] = {};

	  cs_ = 0;
	  CycleCounter::delay_cycles(tclk_tb_cycles_);
	  for (size_t i = 0; i < sizeof(rx_data); ++i) {
		  if (!spi_.transfer_byte_register_polling(
		          tx_data[i], rx_data[i], spi_timeout_cycles_)) {
			  finish_timed_transfer();
			  return false;
		  }
		  if (i + 1U < sizeof(rx_data)) {
			  CycleCounter::delay_cycles(tclk_tb_cycles_);
		  }
	  }
	  finish_timed_transfer();

	  return decode_position_turn(rx_data);
	}

	bool Amt252d::read_position_turn() {
	  switch (read_mode_) {
	  case ReadMode::LegacyNop:
		  return read_position_turn_legacy_nop();
	  case ReadMode::TimedRegisterPolling:
		  return read_position_turn_timed_register_polling();
	  }
	  return false;
	}

	bool Amt252d::set_read_mode(ReadMode read_mode) {
	  using CycleCounter = stm32_peripherals::CycleCounter;

	  read_mode_ = read_mode;
	  has_last_cs_release_ = false;
	  read_mode_ready_ = true;

	  if (read_mode_ == ReadMode::TimedRegisterPolling) {
		  read_mode_ready_ = CycleCounter::enable();
		  if (read_mode_ready_) {
			  const uint32_t cycles_per_us = CycleCounter::cycles_per_us();
			  // The datasheet values are minima. Keep margin for GPIO and
			  // peripheral timing variation in optimized builds.
			  tclk_tb_cycles_ = cycles_per_us * 5U;
			  tcs_cycles_ = cycles_per_us * 50U;
			  tr_cycles_ = cycles_per_us * 5U;
			  spi_timeout_cycles_ = cycles_per_us * 100U;
			  last_cs_release_cycle_ = CycleCounter::now();
			  has_last_cs_release_ = true;
		  }
	  }

	  return read_mode_ready_;
	}

	Amt252d::ReadMode Amt252d::get_read_mode() const {
	  return read_mode_;
	}

	bool Amt252d::is_read_mode_ready() const {
	  return read_mode_ready_;
	}

	bool Amt252d::read_position(){
	  uint8_t tx_data[2] = {0x00, 0x00};
	  uint8_t rx_data[2] = {0x00, 0x00};

	  cs_ = 0;
	  spi_.write(tx_data, rx_data, 2);
	  cs_ = 1;

	  if (checksum(rx_data[0], rx_data[1])) {
		  uint16_t raw = ((uint16_t)(rx_data[0] & 0x3F) << 8) | (uint16_t)rx_data[1];
		  pos = raw;
		  return true;
	  }
	  return false;
	}

	uint16_t Amt252d::get_position() { return pos; }
	int16_t Amt252d::get_turn() { return turn; }

}

#endif
