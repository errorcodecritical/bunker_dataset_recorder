// Copyright (c) Sensrad 2025

// Converts a byte count to "123.45 KB" / "12.34 MB" / …

#include <HumanReadableBytes.hpp>

#include <iomanip>

std::string humanReadableBytes(const std::size_t bytes) noexcept {

  const char *units[] = {"B", "KB", "MB", "GB", "TB", "PB"};
  std::size_t i = 0;

  double count = static_cast<double>(bytes);
  while (count >= 1024.0 && i < std::size(units) - 1) {
    count /= 1024.0;
    ++i;
  }

  std::ostringstream os;
  os << std::fixed << std::setprecision(2) << count << ' ' << units[i];
  return os.str();
}
