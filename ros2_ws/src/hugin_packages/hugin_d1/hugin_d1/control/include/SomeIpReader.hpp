// Copyright (c) Sensrad 2025

#pragma once

#include <array>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <vector>

namespace SomeIpReader {

// Parse some/ip response
template <typename T> T read(std::vector<uint8_t> &buffer);

// Parse some/ip response array
template <typename T, std::size_t N>
std::array<T, N> read_array(std::vector<uint8_t> &buffer);

// Read some/ip string
std::string read_string(std::vector<uint8_t> &buffer, const size_t num_bytes);

// Read some/ip bytes as hex string
std::string read_bytes_as_hex_string(std::vector<uint8_t> &buffer,
                                     const std::size_t num_bytes);

}; // namespace SomeIpReader

// Consume a value of type T from the buffer.
template <typename T> T SomeIpReader::read(std::vector<uint8_t> &buffer) {
  // Check that there is enough data in the buffer to read the requested type.
  if (buffer.size() < sizeof(T)) {
    throw std::runtime_error(
        "Not enough data in buffer to read the requested type.");
  }

  T value{};

  std::memcpy(&value, buffer.data(), sizeof(T));

  // Remove the bytes that have been read.
  buffer.erase(buffer.begin(), buffer.begin() + sizeof(T));

  return value;
}

// Consume an array<T, N> from the buffer
template <typename T, std::size_t N>
std::array<T, N> SomeIpReader::read_array(std::vector<uint8_t> &buffer) {
  // Check that there is enough data in the buffer to read the requested type.
  if (buffer.size() < sizeof(T) * N) {
    throw std::runtime_error(
        "Not enough data in buffer to read the requested type.");
  }

  std::array<T, N> array{};

  std::memcpy(array.data(), buffer.data(), sizeof(T) * N);

  // Remove the bytes that have been read.
  buffer.erase(buffer.begin(), buffer.begin() + sizeof(T) * N);

  return array;
}
