#pragma once
#include "../Inc/amt252d.hpp"

#ifdef HAL_SPI_MODULE_ENABLED

namespace stm32_library::stm32_modules {
	Amt252d::Amt252d(
			stm32_peripherals::DigitalOut &cs,
			stm32_peripherals::Spi &spi,
			bool is_reverse)
	  : cs_(cs),spi_(spi) {
		cs_ = 1;
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

	bool Amt252d::read_position_turn() {
	  uint8_t tx_data[4] = {0x00, 0xA0, 0x00, 0x00}; // Multi-turn position read command
	  uint8_t rx_data[4] = {};

	  // ===== Step 1: 送信コマンド =====
	  cs_ = 0;
	  for(size_t i = 0; i < sizeof(rx_data);i++){
		  rx_data[i] = spi_.write(tx_data[i]);
		  for (volatile int i = 0; i < 1000; ++i) __NOP();
	  }
	  cs_ = 1;

	  bool ok = true;

	  if (checksum(rx_data[0], rx_data[1])) {
		  uint16_t raw_pos = ((uint16_t)(rx_data[0] & 0x3F) << 8) | rx_data[1];
		  pos = raw_pos;
	  } else {
		  ok = false;
	  }

	  if (checksum(rx_data[2], rx_data[3])) {
		  uint16_t raw_turn = ((uint16_t)(rx_data[2] & 0x3F) << 8) | rx_data[3];
		  int16_t tmp = signed14(raw_turn);
		  turn = tmp;  // OK: 更新
	  } else {
		  // 更新しない（前回値を保持）
		  ok = false;
	  }

	  return ok;
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
