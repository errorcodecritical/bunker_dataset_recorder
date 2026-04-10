# Copyright (c) Sensrad 2024-2025

# Standard header
std_msgs/Header header

uint64 radar_time_ms
uint16 frame_counter
uint8  is_4d
uint8  frame_type
uint32 number_of_points
uint16 crd_count
float32 range_resolution
float32 doppler_resolution
float32 azimuth_resolution
float32 elevation_resolution
float32 range_offset
uint32 azimuth_fft_size
uint32 elevation_fft_size
uint32 azimuth_fft_padding
uint32 elevation_fft_padding
uint32 range_fft_size
uint32 bytes_per_target
