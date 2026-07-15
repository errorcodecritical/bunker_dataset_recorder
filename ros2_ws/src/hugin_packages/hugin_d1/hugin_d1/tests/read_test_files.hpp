// Copyright (c) Sensrad 2025

#pragma once

// STL
#include <filesystem>
#include <vector>

// Eigen
#include <Eigen/Dense>

// Convert to matrix 3 x N.
Eigen::Matrix3Xf to_matrix(const std::vector<Eigen::Vector3f> &cols);

// Read a binary file into a vector of bytes.
std::vector<uint8_t> read_binary_file(const std::filesystem::path &path);

// Read a CSV file into a matrix 3 x N.
Eigen::Matrix3Xf read_vector3f_csv_file(const std::filesystem::path &path);
