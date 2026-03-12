#pragma once

#ifdef HAL_UART_MODULE_ENABLED

#include "uart.hpp"
#include <cassert>

namespace stm32_library::stm32_peripherals {

#define INST_PING 0x01
#define INST_READ 0x02
#define INST_WRITE 0x03
#define INST_REG_WRITE 0x04
#define INST_REG_ACTION 0x05
#define INST_SYNC_READ 0x82
#define INST_SYNC_WRITE 0x83

class SCSerial {
public:
  SCSerial(UART_HandleTypeDef *huart) : serial_(huart) {
    // ブロードキャストコマンドを除くすべてのコマンドはアンサーを返す
    level_ = 1;
    error_ = 0;

    end_ = 0;
  }

  int gen_write(uint8_t id, uint8_t mem_addr, uint8_t *n_dat, uint8_t n_len) {
    read_flush();
    write_buf(id, mem_addr, n_dat, n_len, INST_WRITE);
    write_flush();
    return ask(id);
  }

  int reg_write(uint8_t id, uint8_t mem_addr, uint8_t *n_dat, uint8_t n_len) {
    read_flush();
    write_buf(id, mem_addr, n_dat, n_len, INST_REG_WRITE);
    write_flush();
    return ask(id);
  }

  int reg_write_action(uint8_t id = 0xfe) {
    read_flush();
    write_buf(id, 0, NULL, 0, INST_REG_ACTION);
    write_flush();
    return ask(id);
  }

  void sync_write(uint8_t id[], uint8_t idn, uint8_t mem_addr, uint8_t *n_dat, uint8_t n_len) {
    read_flush();
    uint8_t mes_len = ((n_len + 1) * idn + 4);
    uint8_t sum = 0;
    uint8_t b_buf[7];
    b_buf[0] = 0xff;
    b_buf[1] = 0xff;
    b_buf[2] = 0xfe;
    b_buf[3] = mes_len;
    b_buf[4] = INST_SYNC_WRITE;
    b_buf[5] = mem_addr;
    b_buf[6] = n_len;
    sc_write(b_buf, 7);

    sum = 0xfe + mes_len + INST_SYNC_WRITE + mem_addr + n_len;
    uint8_t i, j;
    for (i = 0; i < idn; i++) {
      sc_write(id[i]);
      sc_write(n_dat + i * n_len, n_len);
      sum += id[i];
      for (j = 0; j < n_len; j++) {
        sum += n_dat[i * n_len + j];
      }
    }
    sc_write(~sum);
    write_flush();
  }

  int write_byte(uint8_t id, uint8_t mem_addr, uint8_t b_dat) {
    read_flush();
    write_buf(id, mem_addr, &b_dat, 1, INST_WRITE);
    write_flush();
    return ask(id);
  }

  int write_word(uint8_t id, uint8_t mem_addr, uint16_t w_dat) {
    uint8_t b_buf[2];
    host_2_scs(b_buf + 0, b_buf + 1, w_dat);
    read_flush();
    write_buf(id, mem_addr, b_buf, 2, INST_WRITE);
    write_flush();
    return ask(id);
  }

  int read(uint8_t id, uint8_t mem_addr, uint8_t *n_data, uint8_t n_len) {
    read_flush();
    write_buf(id, mem_addr, &n_len, 1, INST_READ);
    write_flush();
    if (!check_head()) {
      return 0;
    }

    uint8_t b_buf[4];
    error_ = 0;
    if (!sc_read(b_buf, 3)) {
      return 0;
    }

    // int size = sc_read(n_data, n_len);
    if (!sc_read(n_data, n_len)) {
      return 0;
    }

    if (!sc_read(b_buf + 3, 1)) {
      return 0;
    }
    uint8_t cal_sum = b_buf[0] + b_buf[1] + b_buf[2];
    uint8_t i;
    for (i = 0; i < n_len; i++) {
      cal_sum += n_data[i];
    }
    cal_sum = ~cal_sum;
    if (cal_sum != b_buf[3]) {
      return 0;
    }
    error_ = b_buf[2];
    return n_len;
  }

  int read_byte(uint8_t id, uint8_t mem_addr) {
    uint8_t b_dat;
    int size = read(id, mem_addr, &b_dat, 1);
    if (size != 1) {
      return -1;
    } else {
      return b_dat;
    }
  }

  int read_word(uint8_t id, uint8_t mem_addr) {
    uint8_t n_dat[2];
    int size;
    uint16_t w_dat;
    size = read(id, mem_addr, n_dat, 2);
    if (size != 2)
      return -1;
    w_dat = scs_2_host(n_dat[0], n_dat[1]);
    return w_dat;
  }

  int ping(uint8_t id) {
    read_flush();
    write_buf(id, 0, NULL, 0, INST_PING);
    write_flush();
    error_ = 0;
    if (!check_head()) {
      return -1;
    }
    uint8_t b_buf[4];
    if (!sc_read(b_buf, 4)) {
      return -1;
    }
    if (b_buf[0] != id && id != 0xfe) {
      return -1;
    }
    if (b_buf[1] != 2) {
      return -1;
    }
    uint8_t cal_sum = ~(b_buf[0] + b_buf[1] + b_buf[2]);
    if (cal_sum != b_buf[3]) {
      return -1;
    }
    error_ = b_buf[2];
    return b_buf[0];
  }

  int sync_read_packet_tx(uint8_t id[], uint8_t idn, uint8_t mem_addr, uint8_t n_len) {
    sync_read_rx_packet_len_ = n_len;
    uint8_t check_sum = (4 + 0xfe) + idn + mem_addr + n_len + INST_SYNC_READ;
    uint8_t i;
    sc_write(0xff);
    sc_write(0xff);
    sc_write(0xfe);
    sc_write(idn + 4);
    sc_write(INST_SYNC_READ);
    sc_write(mem_addr);
    sc_write(n_len);
    for (i = 0; i < idn; i++) {
      sc_write(id[i]);
      check_sum += id[i];
    }
    check_sum = ~check_sum;
    sc_write(check_sum);
    return n_len;
  }

  int sync_read_packet_rx(uint8_t id, uint8_t *n_dat) {
    sync_read_rx_packet_ = n_dat;
    sync_read_rx_packet_index_ = 0;
    uint8_t b_buf[4];
    if (!check_head()) {
      return 0;
    }
    if (!sc_read(b_buf, 3)) {
      return 0;
    }
    if (!sc_read(b_buf, 3)) {
      return 0;
    }
    if (b_buf[0] != id) {
      return 0;
    }
    if (b_buf[1] != (sync_read_rx_packet_len_ + 2)) {
      return 0;
    }
    error_ = b_buf[2];
    if (!sc_read(n_dat, sync_read_rx_packet_len_)) {
      return 0;
    }
    return sync_read_rx_packet_len_;
  }

  int sync_read_rx_packet_to_byte() {
    if (sync_read_rx_packet_index_ >= sync_read_rx_packet_len_) {
      return -1;
    }
    return sync_read_rx_packet_[sync_read_rx_packet_index_++];
  }

  int sync_read_rx_packet_to_wrod(uint8_t neg_bit = 0) {
    if ((sync_read_rx_packet_index_ + 1) >= sync_read_rx_packet_len_) {
      return -1;
    }
    int Word = scs_2_host(sync_read_rx_packet_[sync_read_rx_packet_index_],
                          sync_read_rx_packet_[sync_read_rx_packet_index_ + 1]);
    sync_read_rx_packet_index_ += 2;
    if (neg_bit) {
      if (Word & (1 << neg_bit)) {
        Word = -(Word & ~(1 << neg_bit));
      }
    }
    return Word;
  }

  int get_err() { return err_; }

  // protected:
  void write_buf(uint8_t id, uint8_t mem_addr, uint8_t *n_dat, uint8_t n_len, uint8_t fun) {
    uint8_t msg_len = 2;
    uint8_t b_buf[6];
    uint8_t check_sum = 0;
    b_buf[0] = 0xff;
    b_buf[1] = 0xff;
    b_buf[2] = id;
    b_buf[4] = fun;
    if (n_dat) {
      msg_len += n_len + 1;
      b_buf[3] = msg_len;
      b_buf[5] = mem_addr;
      sc_write(b_buf, 6);

    } else {
      b_buf[3] = msg_len;
      sc_write(b_buf, 5);
    }
    check_sum = id + msg_len + fun + mem_addr;
    uint8_t i = 0;
    if (n_dat) {
      for (i = 0; i < n_len; i++) {
        check_sum += n_dat[i];
      }
      sc_write(n_dat, n_len);
    }
    sc_write(~check_sum);
  }

  void host_2_scs(uint8_t *data_l, uint8_t *data_h, uint16_t data) {
    if (end_) {
      *data_l = (data >> 8);
      *data_h = (data & 0xff);
    } else {
      *data_h = (data >> 8);
      *data_l = (data & 0xff);
    }
  }

  uint16_t scs_2_host(uint8_t data_l, uint8_t data_h) {
    uint16_t data;
    if (end_) {
      data = data_l;
      data <<= 8;
      data |= data_h;
    } else {
      data = data_h;
      data <<= 8;
      data |= data_l;
    }
    return data;
  }

  int ask(uint8_t id) {
    error_ = 0;
    if (id != 0xfe && level_) {
      if (!check_head()) {
        return 0;
      }
      uint8_t b_buf[4];
      if (!sc_read(b_buf, 4)) {
        return 0;
      }
      if (b_buf[0] != id) {
        return 0;
      }
      if (b_buf[1] != 2) {
        return 0;
      }
      uint8_t cal_sum = ~(b_buf[0] + b_buf[1] + b_buf[2]);
      if (cal_sum != b_buf[3]) {
        return 0;
      }
      error_ = b_buf[2];
    }
    return 1;
  }

  int check_head() {
    uint8_t b_dat;
    uint8_t b_buf[2] = {0, 0};
    uint8_t cnt = 0;
    while (1) {
      if (!sc_read(&b_dat, 1)) {
        return 0;
      }
      b_buf[1] = b_buf[0];
      b_buf[0] = b_dat;
      if (b_buf[0] == 0xff && b_buf[1] == 0xff) {
        break;
      }
      cnt++;
      if (cnt > 10) {
        return 0;
      }
    }
    return 1;
  }

  bool sc_write(uint8_t *n_dat, int n_len) {
    auto ret = serial_.write(n_dat, n_len);
    return (ret == HAL_OK);
  }
  bool sc_read(uint8_t *n_dat, int n_len) {
    auto ret = serial_.read(n_dat, n_len);
    return (ret == HAL_OK);
  }
  bool sc_write(uint8_t b_dat) {
    auto ret = serial_.write(&b_dat, 1);
    return (ret == HAL_OK);
  }
  void read_flush() { /* do not anything */
  }
  void write_flush() { /* do not anything */
  }

  // protected:
  uint8_t level_; // 舵机返回等级
  uint8_t end_;   // 处理器大小端结构 ビックエンディアンかリトルエンディアンか
  uint8_t error_; // サーボのステータス 舵机状态
  uint8_t sync_read_rx_packet_index_;
  uint8_t sync_read_rx_packet_len_;
  uint8_t *sync_read_rx_packet_;
  int err_;

private:
  uint32_t timeout_;
  Uart serial_;
};

} // namespace stm32_library::stm32_peripherals

#endif // HAL_UART_MODULE_ENABLED
