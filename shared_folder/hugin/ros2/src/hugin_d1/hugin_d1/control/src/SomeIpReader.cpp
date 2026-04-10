// Copyright (c) Sensrad 2025

#include <SomeIpReader.hpp>

#include <string>

// Consume a string of length N from the buffer.
std::string SomeIpReader::read_string(std::vector<uint8_t> &buffer,
                                      const size_t num_bytes) {

  if (buffer.size() < num_bytes) {
    // throw std::runtime_error(
    //       "Not enough data in buffer to read the requested number of
    //       bytes.");
    return {};
  }

  std::string str(buffer.begin(), buffer.begin() + num_bytes);
  buffer.erase(buffer.begin(), buffer.begin() + num_bytes);

  return str;
}

std::string
SomeIpReader::read_bytes_as_hex_string(std::vector<uint8_t> &buffer,
                                       const std::size_t num_bytes) {

  if (buffer.size() < num_bytes) {
    // throw std::runtime_error(
    //     "Not enough data in buffer to read the requested number of bytes.");
    return {};
  }

  std::ostringstream oss;
  oss << std::hex << std::setfill('0');

  // Iterate over the first num_bytes in the buffer.
  for (std::size_t i = 0; i < num_bytes; ++i) {
    oss << std::setw(2) << static_cast<int>(buffer[i]);
  }

  // Erase the bytes that have been processed.
  buffer.erase(buffer.begin(), buffer.begin() + num_bytes);

  return oss.str();
}
