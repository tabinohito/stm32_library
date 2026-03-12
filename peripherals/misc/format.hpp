#pragma once

#include <cstdio>
#include <vector>

namespace abu2023::stm32_peripherals::utility {
template <class... Args> inline std::vector<char> format(const char *fmt, Args... args) {
  size_t len = std::snprintf(nullptr, 0, fmt, args...) + 1;
  std::vector<char> buf(len);
  std::snprintf(buf.data(), len, fmt, args...);
  return buf;
}
} // namespace abu2023::stm32_peripherals::utility
