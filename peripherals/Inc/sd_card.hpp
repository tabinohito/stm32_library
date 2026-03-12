#pragma once

#include "main.h"

#if defined(HAL_SD_MODULE_ENABLED) && __has_include(<fatfs.h>)

#include <fatfs.h>
#include <string.h>

namespace stm32_library::stm32_peripherals {
class SdCard {
public:
  SdCard(SD_HandleTypeDef *hsd) {
    res_ = FR_OK;
    HAL_SD_Init(hsd);
    res_ = f_mount(&fatfs_, "", 1); // 0;unmount 1;mount
  }
  ~SdCard() {
    res_ = f_mount(&fatfs_, "", 0); // 0;unmount 1;mount
  }
  FRESULT write(const char *str) {
    UINT bw;
    res_ = f_write(&file_, (void *)str, static_cast<UINT>(strlen(str)), &bw);
    res_ = f_sync(&file_);
    return res_;
  }
  FRESULT write(std::string str) { write(str.c_str()); }
  template <typename... Args> FRESULT write(const std::string &fmt, Args... args) {
    size_t len = std::snprintf(nullptr, 0, fmt.c_str(), args...);
    std::vector<char> buf(len + 1);
    std::snprintf(&buf[0], len + 1, fmt.c_str(), args...);
    write(std::string(&buf[0], &buf[0] + len));
  }
  FRESULT read(const char *fname, char *str, UINT byte) {
    res_ = f_open(&file_, fname, FA_OPEN_ALWAYS | FA_READ);
    if (res_ != FR_OK) {
      res_ = f_close(&file_);
      return res_;
    }
    UINT cnt = 0;
    res_ = f_read(&file_, str, byte, &cnt);
    f_close(&file_);
    return res_;
  }
  FRESULT del(const char *name) {
    res_ = f_unlink(name);
    return res_;
  }
  FRESULT open(const char *fname, bool is_move_file_end = true) {
    res_ = f_open(&file_, fname, FA_OPEN_ALWAYS | FA_WRITE);
    if (res_ != FR_OK) {
      res_ = f_close(&file_);
      return res_;
    }
    if (is_move_file_end) {
      res_ = f_lseek(&file_, f_size(&file_));
    }
    return res_;
  }
  FRESULT close() {
    res_ = f_close(&file_);
    return res_;
  }
  FRESULT make_directory(const char *directory) {
    res_ = f_mkdir(directory);
    return res_;
  }

private:
  FATFS fatfs_;
  FRESULT res_;
  FIL file_;
};

} // namespace stm32_library::stm32_peripherals

#endif // HAL_SD_MODULE_ENABLED
