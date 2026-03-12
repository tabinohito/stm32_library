#pragma once

#include "main.h"

#ifdef HAL_TIM_MODULE_ENABLED

#include <numbers>

namespace abu2023::stm32_peripherals {
class Encoder {
private:
  TIM_HandleTypeDef *handle_;
  uint16_t ppr_;

  int count_dir_;
  int pos_cnt_;
  int16_t idx_cnt_;
  int now_dir_;

  float velocity_;
  float position_;
  int16_t delta_count_;
  uint32_t since_cw_ = INT32_MAX;
  uint32_t since_ccw_ = INT32_MAX;

public:
  Encoder(TIM_HandleTypeDef *handle, uint16_t ppr, bool is_reverse = false) : handle_(handle) {
    startup();
    set_ppr(ppr);
    set_pos(0.0);
    set_dir(is_reverse);
  }

  void reset_pos() { set_pos(0.0); }

  void set_ppr(uint16_t ppr) { ppr_ = ppr; }
  void set_dir(bool is_reverse) { count_dir_ = (is_reverse == true) ? -1 : 1; }
  void set_pos(float pos) {
    uint16_t CPR = ppr_ * 4; // (Count Per Revolusion)
    pos_cnt_ = pos * CPR;

    velocity_ = 0;
    idx_cnt_ = 0;

    if (pos_cnt_ < 0) {
      while (pos_cnt_ < 0) {
        pos_cnt_ += CPR;
        idx_cnt_ -= 1;
      }
    } else {
      idx_cnt_ += (pos_cnt_ / CPR);
      pos_cnt_ %= CPR;
    }
    position_ = (float)pos_cnt_ / CPR;
  }

  void update(float dt) {
    uint16_t CPR = ppr_ * 4; // (Count Per Revolusion)
    // uint16_t count = qei_getposition_(&qei);
    delta_count_ = count_dir_ * get_reset_cnt(); //(count - pastCount) * count_dir_;

    since_cw_++;
    since_ccw_++;
    if (2 < delta_count_)
      since_cw_ = 0;
    if (delta_count_ < -2)
      since_ccw_ = 0;

    now_dir_ = sign(delta_count_);

    if (delta_count_ == 0)
      now_dir_ = 0;

    // 速度計算
    velocity_ = delta_count_ / dt / CPR;

    // 位置計算
    pos_cnt_ += delta_count_;
    if (pos_cnt_ < 0) {
      pos_cnt_ += CPR;
      idx_cnt_ -= 1;
    } else {
      idx_cnt_ += (pos_cnt_ / CPR);
      pos_cnt_ %= CPR;
    }
    position_ = (float)pos_cnt_ / CPR;
  }

  int direction() { return now_dir_; }
  int revolutions() { return idx_cnt_; }
  float position() { return position_; }
  float rotation() { return (revolutions() + position()); }
  float angular_vel() { return (velocity_ * 2 * std::numbers::pi); }
  float rpm() { return (velocity_ * 60); }
  float rps() { return velocity_; }
  int16_t count() { return delta_count_; };
  bool is_connected(uint32_t timeout = 1000) { // エンコーダーを揺らしたときだけtrueになる 起動時もなぜかtrueになる
    return (since_cw_ < timeout) && (since_ccw_ < timeout);
  }

protected:
  void startup() { HAL_TIM_Encoder_Start(handle_, TIM_CHANNEL_ALL); }

private:
  int16_t get_reset_cnt() {
    int16_t val = __HAL_TIM_GET_COUNTER(handle_);
    __HAL_TIM_SET_COUNTER(handle_, 0);
    return val;
  }

  int sign(int16_t val) {
    if (val > 0)
      return 1;
    else if (val < 0)
      return -1;
    return 0;
  }
};
} // namespace abu2023::stm32_peripherals

#endif // HAL_TIM_MODULE_ENABLED
