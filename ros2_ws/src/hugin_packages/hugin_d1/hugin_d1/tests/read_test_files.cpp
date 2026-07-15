// Copyright (c) Sensrad 2025

#include "read_test_files.hpp"

// STL
#include <fstream>

// Boost qi
#include <boost/spirit/include/qi.hpp>

namespace qi = boost::spirit::qi;

// This is a spirit::qi grammar that skips comments and
// whitespace. It's used in the simple CSV parser below.
template <typename Iterator> struct comment_skipper : qi::grammar<Iterator> {
  comment_skipper() : comment_skipper::base_type(start) {
    // Skip rule:
    //   - ASCII whitespace
    //   - '#' followed by any chars until EOL
    //     (then optionally consume the EOL itself)
    start = qi::space                                     // normal whitespace
            | ('#' >> *(qi::char_ - qi::eol) >> -qi::eol) // # and rest of line
        ;
  }

  boost::spirit::qi::rule<Iterator> start;
};

// Convert to matrix 3 x N.
Eigen::Matrix3Xf to_matrix(const std::vector<Eigen::Vector3f> &cols) {
  Eigen::Matrix3Xf mat(3, cols.size());
  for (std::size_t i = 0; i < cols.size(); ++i) {
    mat.col(i) = cols[i];
  }
  return mat;
}

// Read a binary file into a vector of bytes.
std::vector<uint8_t> read_binary_file(const std::filesystem::path &path) {

  // Open file and seek to the end immediately
  std::ifstream file(path, std::ios::binary | std::ios::ate);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open file: " + path.string());
  }

  // Get the file size
  const auto file_size = file.tellg();

  // Allocate a buffer of the required size
  std::vector<uint8_t> buffer(file_size);

  // Move to the start of the file and read all data
  file.seekg(0, std::ios::beg);

  file.read(reinterpret_cast<char *>(buffer.data()), file_size);

  return buffer;
}

// Read a CSV file with 3 float columns into a 3xn matrix. Allows for
// white space and comments.
Eigen::Matrix3Xf read_vector3f_csv_file(const std::filesystem::path &path) {

  // Open file
  std::ifstream file(path);
  if (not file.is_open()) {
    throw std::runtime_error("Failed to open file: " + path.string());
  }

  comment_skipper<std::string::iterator> skipper;

  std::vector<Eigen::Vector3f> rows;

  // Read lines
  std::string line;
  while (std::getline(file, line)) {
    float x = 0;
    float y = 0;
    float z = 0;

    auto first = line.begin();
    auto last = line.end();

    bool success =
        qi::phrase_parse(first, last,
                         // Begin grammar
                         (qi::float_ >> ',' >> qi::float_ >> ',' >> qi::float_),
                         // End grammar
                         skipper, x, y, z);

    if (success && first == last) {
      rows.push_back(Eigen::Vector3f(x, y, z));
    }
  }

  return to_matrix(rows);
}
