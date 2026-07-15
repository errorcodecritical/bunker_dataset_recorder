// Copyright (c) Sensrad 2025

#pragma once

#include "ArbeGenericHeader.hpp"

#include <cstdint>
#include <functional>
#include <vector>

#include <rclcpp/rclcpp.hpp>

struct RTPPacket;

class ArbeReceiver {

  constexpr static size_t RECEIVE_BUFFER_RESERVE = 12 * 65536;

  // This is intentionally signed to allow for negative values.
  ssize_t left_to_receive_;

  ArbeGenericHeader current_arbe_generic_header_;
  std::vector<uint8_t> current_receive_buffer_;

  // ROS2 logger
  rclcpp::Logger logger_;

  // Type for callback hook
  using ArbePacketReceivedCallback = std::function<void(
      const ArbeGenericHeader &, const std::vector<uint8_t> &)>;

  // Vector of callback hooks
  std::vector<ArbePacketReceivedCallback> arbe_packet_received_hooks_;

  void log_arbe_generic_header(
      const ArbeGenericHeader &arbe_generic_header) const noexcept;

  rclcpp::Logger &get_logger() noexcept { return logger_; }

public:
  ArbeReceiver(rclcpp::Logger logger);
  ~ArbeReceiver();

  // Allow this class to be used a as a callback for RTPReceiver
  void operator()(const RTPPacket &packet, std::size_t packet_size);

  // Add a callback hook
  void add_arbe_packet_received_hook(ArbePacketReceivedCallback hook);
};
