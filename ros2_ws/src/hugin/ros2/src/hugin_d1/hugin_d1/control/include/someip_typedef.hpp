// Copyright (c) Sensrad 2025

#pragma once

#include <cassert>
#include <cstdbool>
#include <cstdint>
#include <vector>

#include <boost/endian/conversion.hpp>

// Start packing, this as same as __attribute__((packed))
#pragma pack(push, 1)

#define APP_MAX_SET_SEQUENCE (10)
#define API_VERSION_BUFFER_SIZE (128)

// someip protocol version
#define SOMEIP_PROTOCOL_VERSION (0x01)
// someip interface version
#define SOMEIP_INTERFACE_VERSION (0x01)

// service id const 0
#define SOMEIP_SERVICE_ID (0x0000)
// for now const 0
#define SOMEIP_CLIENT_ID_PREFIX (0x00)
// for now const 0
#define SOMEIP_CLIENT_ID (0x00)
// This is set at the beginning of each command payload. Will change
// every change of the api.
#define API_VERSION (1)

#define SOMEIP_DEFAULT_VALUES_BITMASK (0x00) // default value for the bitmask
// min size of the request payload (api_version +
// default_values_bitmask at least)
#define SOME_IP_MIN_PAYLOAD_SIZE (8)

typedef enum {
  MSG_TYPE_START_TX = 0x0101,
  MSG_TYPE_STOP_TX = 0x0102,
  MSG_TYPE_SET_TIME = 0x0103,
  MSG_TYPE_SET_STATIC_COARSE_THRESHOLD = 0x0104,
  MSG_TYPE_SET_STATIC_FINE_THRESHOLD = 0x0105,
  MSG_TYPE_SET_DYNAMIC_COARSE_THRESHOLD = 0x0106,
  MSG_TYPE_SET_DYNAMIC_FINE_THRESHOLD = 0x0107,
  MSG_TYPE_START_CW = 0x0108,
  MSG_TYPE_GET_VERSION = 0x0109,
  MSG_TYPE_GET_STATUS = 0x010a,
  MSG_TYPE_SET_FRAME_TRACE_REQ = 0x010b,
  MSG_TYPE_SET_SEQUENCE = 0x010c,
  MSG_TYPE_RELEASE_CW = 0x010d,
  MSG_TYPE_GET_FIRMWARE_VERSION = 0x010e,
  MSG_TYPE_GET_CHANGE_MODE = 0x010f,
  MSG_TYPE_GET_MODE = 0x0110,
  MSG_TYPE_START_CHIRP = 0x0111,
  MSG_TYPE_GET_BLOB_VERSION = 0x0112,
  MSG_TYPE_GET_CALIBRATION_METADATA = 0x0113,
  MSG_TYPE_FILE_READ = 0x0201,
  MSG_TYPE_FILE_WRITE = 0x0202,
  MSG_TYPE_FILE_ERASE = 0x0203,
  MSG_TYPE_FILE_GET_INFO = 0x0204
} some_ip_method_id_e;

/**--------------------------------api_struct------------------------------- */
struct some_ip_start_tx_t {
  uint32_t api_version_;
  uint32_t default_values_bitmask_;

  // Default constructor
  some_ip_start_tx_t()
      : api_version_(boost::endian::native_to_big(API_VERSION)),
        default_values_bitmask_(
            boost::endian::native_to_big(SOMEIP_DEFAULT_VALUES_BITMASK)) {};
};

struct some_ip_stop_tx_t {
  uint32_t api_version_;
  uint32_t default_values_bitmask_;

  // Default constructor
  explicit some_ip_stop_tx_t()
      : api_version_(boost::endian::native_to_big(API_VERSION)),
        default_values_bitmask_(
            boost::endian::native_to_big(SOMEIP_DEFAULT_VALUES_BITMASK)) {};
};

struct some_ip_set_dynamic_coarse_threshold_t {
  uint32_t api_version_;
  uint32_t default_values_bitmask_;
  uint32_t sub_array_type_;
  uint32_t backoff_azimuth_near_field_;
  uint32_t backoff_azimuth_far_field_;

  // Default constructor
  explicit some_ip_set_dynamic_coarse_threshold_t(
      uint32_t sub_array_type, uint32_t backoff_azimuth_near_field,
      uint32_t backoff_azimuth_far_field)
      : api_version_(boost::endian::native_to_big(API_VERSION)),
        default_values_bitmask_(
            boost::endian::native_to_big(SOMEIP_DEFAULT_VALUES_BITMASK)),
        sub_array_type_(boost::endian::native_to_big(sub_array_type)),
        backoff_azimuth_near_field_(
            boost::endian::native_to_big(backoff_azimuth_near_field)),
        backoff_azimuth_far_field_(
            boost::endian::native_to_big(backoff_azimuth_far_field)) {};
};

struct some_ip_set_dynamic_fine_threshold_t {
  uint32_t api_version_;
  uint32_t default_values_bitmask_;
  uint32_t array_type_;
  uint32_t backoff_azimuth_near_field_;
  uint32_t backoff_azimuth_far_field_;
  uint32_t backoff_elevation_far_field_;
  uint32_t backoff_elevation_near_field_;
  uint32_t backoff_azimuth_elevation_near_field_;
  uint32_t backoff_azimuth_elevation_far_field_;

  // Default constructor
  explicit some_ip_set_dynamic_fine_threshold_t(
      uint32_t array_type, uint32_t backoff_azimuth_near_field,
      uint32_t backoff_azimuth_far_field, uint32_t backoff_elevation_far_field,
      uint32_t backoff_elevation_near_field,
      uint32_t backoff_azimuth_elevation_near_field,
      uint32_t backoff_azimuth_elevation_far_field)
      : api_version_(boost::endian::native_to_big(API_VERSION)),
        default_values_bitmask_(
            boost::endian::native_to_big(SOMEIP_DEFAULT_VALUES_BITMASK)),
        array_type_(boost::endian::native_to_big(array_type)),
        backoff_azimuth_near_field_(
            boost::endian::native_to_big(backoff_azimuth_near_field)),
        backoff_azimuth_far_field_(
            boost::endian::native_to_big(backoff_azimuth_far_field)),
        backoff_elevation_far_field_(
            boost::endian::native_to_big(backoff_elevation_far_field)),
        backoff_elevation_near_field_(
            boost::endian::native_to_big(backoff_elevation_near_field)),
        backoff_azimuth_elevation_near_field_(
            boost::endian::native_to_big(backoff_azimuth_elevation_near_field)),
        backoff_azimuth_elevation_far_field_(boost::endian::native_to_big(
            backoff_azimuth_elevation_far_field)) {};
};

struct some_ip_set_static_coarse_thresholds_t {
  uint32_t api_version_;
  uint32_t default_values_bitmask_;
  int32_t coarse_threshold_;
  int32_t coarse_dop0_threshold_;

  // Default constructor
  explicit some_ip_set_static_coarse_thresholds_t(int32_t coarse_threshold,
                                                  int32_t coarse_dop0_threshold)
      : api_version_(boost::endian::native_to_big(API_VERSION)),
        default_values_bitmask_(
            boost::endian::native_to_big(SOMEIP_DEFAULT_VALUES_BITMASK)),
        coarse_threshold_(boost::endian::native_to_big(coarse_threshold)),
        coarse_dop0_threshold_(
            boost::endian::native_to_big(coarse_dop0_threshold)) {};
};

struct some_ip_set_static_fine_thresholds_t {
  uint32_t api_version_;
  uint32_t default_values_bitmask_;
  int32_t fine_threshold_;
  int32_t fine_dop0_threshold_;

  // Default constructor
  explicit some_ip_set_static_fine_thresholds_t(int32_t fine_threshold,
                                                int32_t fine_dop0_threshold)
      : api_version_(boost::endian::native_to_big(API_VERSION)),
        default_values_bitmask_(
            boost::endian::native_to_big(SOMEIP_DEFAULT_VALUES_BITMASK)),
        fine_threshold_(boost::endian::native_to_big(fine_threshold)),
        fine_dop0_threshold_(
            boost::endian::native_to_big(fine_dop0_threshold)) {};
};

struct some_ip_set_frame_trace_data_req_t {
  uint32_t api_version_;
  uint32_t default_values_bitmask_;
  uint32_t enable_;
  uint32_t request_type_;
  uint32_t num_of_time_;
  uint32_t p_;

  // Default constructor
  explicit some_ip_set_frame_trace_data_req_t(uint32_t enable,
                                              uint32_t request_type,
                                              uint32_t num_of_time, uint32_t p)
      : api_version_(boost::endian::native_to_big(API_VERSION)),
        default_values_bitmask_(
            boost::endian::native_to_big(SOMEIP_DEFAULT_VALUES_BITMASK)),
        enable_(boost::endian::native_to_big(enable)),
        request_type_(boost::endian::native_to_big(request_type)),
        num_of_time_(boost::endian::native_to_big(num_of_time)),
        p_(boost::endian::native_to_big(p)) {};
};

struct some_ip_set_tod_time_t {
  uint32_t api_version_;
  uint32_t default_values_bitmask_;
  uint32_t tod_config_;
  uint64_t milliseconds_;

  // Default constructor
  explicit some_ip_set_tod_time_t(uint32_t tod_config, uint64_t milliseconds)
      : api_version_(boost::endian::native_to_big(API_VERSION)),
        default_values_bitmask_(
            boost::endian::native_to_big(SOMEIP_DEFAULT_VALUES_BITMASK)),
        tod_config_(boost::endian::native_to_big(tod_config)),
        milliseconds_(boost::endian::native_to_big(milliseconds)) {};
};

struct some_ip_release_cw_t {
  uint32_t api_version_;
  uint32_t default_values_bitmask_;
  uint32_t un_tx_id_;

  // Default constructor
  explicit some_ip_release_cw_t(uint32_t un_tx_id)
      : api_version_(boost::endian::native_to_big(API_VERSION)),
        default_values_bitmask_(
            boost::endian::native_to_big(SOMEIP_DEFAULT_VALUES_BITMASK)),
        un_tx_id_(boost::endian::native_to_big(un_tx_id)) {};
};

struct some_ip_set_cw_t {
  uint32_t api_version_;
  uint32_t default_values_bitmask_;
  uint32_t un_tx_id_;
  uint32_t un_num_of_bf_channels_;
  uint32_t un_ant_id_;
  uint64_t ul_cw_base_freq_hz_;

  // Default constructor
  explicit some_ip_set_cw_t(uint32_t un_tx_id, uint32_t un_num_of_bf_channels,
                            uint32_t un_ant_id, uint64_t ul_cw_base_freq_hz)
      : api_version_(boost::endian::native_to_big(API_VERSION)),
        default_values_bitmask_(
            boost::endian::native_to_big(SOMEIP_DEFAULT_VALUES_BITMASK)),
        un_tx_id_(boost::endian::native_to_big(un_tx_id)),
        un_num_of_bf_channels_(
            boost::endian::native_to_big(un_num_of_bf_channels)),
        un_ant_id_(boost::endian::native_to_big(un_ant_id)),
        ul_cw_base_freq_hz_(boost::endian::native_to_big(ul_cw_base_freq_hz)) {
        };
};

struct some_ip_get_version_t {
  uint32_t api_version_;
  uint32_t default_values_bitmask_;

  // Default constructor
  explicit some_ip_get_version_t()
      : api_version_(boost::endian::native_to_big(API_VERSION)),
        default_values_bitmask_(
            boost::endian::native_to_big(SOMEIP_DEFAULT_VALUES_BITMASK)) {};
};

struct some_ip_status_result_t {
  uint32_t status_;

  /*************************************************************************
   * Error Code structure:                                                 *
   * CID - Core ID                                                         *
   * MID - Module ID                                                       *
   *************************************************************************
   * | MSBit 8 bits    |    8 bits      |     16 bits                    | *
   * |_________________|________________|________________________________| *
   * |Reserved|  CID   |     MID        |    Error Code                  | *
   * |___4____|___4____|________________|________________________________| *
   *************************************************************************/

  // Error code method
  uint16_t error_code() const noexcept {
    // Convert status to host byte order
    return boost::endian::big_to_native(status_) & 0xffff;
  }

  // Success == error_code == 0
  bool success() const noexcept { return error_code() == 0; }

  bool failure() const noexcept { return !success(); }

  // MID
  uint8_t error_mid() const noexcept { return (status_ >> 16) & 0xff; }

  // CID
  uint8_t error_cid() const noexcept { return (status_ >> 24) & 0xff; }
};

struct some_ip_get_version_result_t {
  some_ip_status_result_t status_;
  char version_[API_VERSION_BUFFER_SIZE];
  char date_[API_VERSION_BUFFER_SIZE];
  char name_[API_VERSION_BUFFER_SIZE];

  const char *version() const noexcept { return version_; }
  const char *date() const noexcept { return date_; }
  const char *name() const noexcept { return name_; }
};

// The whole struct is always sent regardless of sequence_length
struct some_ip_set_seq_t {
  uint32_t api_version_;
  uint32_t default_values_bitmask_;
  uint32_t sequence_id_;
  uint32_t sequence_length_;
  uint32_t frame_ids_[APP_MAX_SET_SEQUENCE];

  // Default constructor
  explicit some_ip_set_seq_t(uint32_t sequence_id,
                             const std::vector<uint32_t> &frame_ids)
      : api_version_(boost::endian::native_to_big(API_VERSION)),
        default_values_bitmask_(
            boost::endian::native_to_big(SOMEIP_DEFAULT_VALUES_BITMASK)),
        sequence_id_(boost::endian::native_to_big(sequence_id)), frame_ids_{} {

    assert(frame_ids.size() <= APP_MAX_SET_SEQUENCE);

    sequence_length_ =
        boost::endian::native_to_big(static_cast<uint32_t>(frame_ids.size()));

    for (size_t i = 0; i < frame_ids.size(); ++i) {
      frame_ids_[i] = boost::endian::native_to_big(frame_ids[i]);
    }
  };
};

struct some_ip_change_mode_t {
  uint32_t api_version_;
  uint32_t default_values_bitmask_;
  uint32_t mode_id_;

  // Default constructor
  explicit some_ip_change_mode_t(uint32_t mode_id)
      : api_version_(boost::endian::native_to_big(API_VERSION)),
        default_values_bitmask_(
            boost::endian::native_to_big(SOMEIP_DEFAULT_VALUES_BITMASK)),
        mode_id_(boost::endian::native_to_big(mode_id)) {};

  explicit some_ip_change_mode_t()
      : api_version_(boost::endian::native_to_big(API_VERSION)),
        default_values_bitmask_(
            boost::endian::native_to_big(SOMEIP_DEFAULT_VALUES_BITMASK)),
        mode_id_(0) {};

  uint32_t mode() const noexcept {
    return boost::endian::big_to_native(mode_id_);
  }
};

struct some_ip_get_modes_result_t {
  uint32_t status_;
  uint32_t current_mode_; // Added in FW 3.x
  uint32_t num_of_modes_;

  uint32_t status() const noexcept {
    return boost::endian::big_to_native(status_);
  }

  uint32_t num_of_modes() const noexcept {
    return boost::endian::big_to_native(num_of_modes_);
  }

  uint32_t current_mode() const noexcept {
    return boost::endian::big_to_native(current_mode_);
  }
};

struct some_ip_get_modes_result_2_10_5_t {
  uint32_t status_;
  uint32_t num_of_modes_;

  uint32_t status() const noexcept {
    return boost::endian::big_to_native(status_);
  }

  uint32_t num_of_modes() const noexcept {
    return boost::endian::big_to_native(num_of_modes_);
  }
};

struct some_ip_start_chirp_t {
  uint32_t api_version_;
  uint32_t default_values_bitmask_;
  uint32_t un_tx_id_;
  uint32_t un_num_of_bf_channels_;
  uint32_t un_ant_id_;
  uint64_t ul_base_freq_hz_;
  uint32_t num_of_chirps_;
};

struct some_ip_gen_rsp_t {
  uint32_t status_;
};

struct some_ip_file_write_req_t {
  uint32_t api_version_;
  uint32_t default_values_bitmask_;
  uint32_t file_type_id_;
  uint32_t payload_len_;

  // Default constructor
  explicit some_ip_file_write_req_t(uint32_t file_type_id, uint32_t payload_len)
      : api_version_(boost::endian::native_to_big(API_VERSION)),
        default_values_bitmask_(
            boost::endian::native_to_big(SOMEIP_DEFAULT_VALUES_BITMASK)),
        file_type_id_(boost::endian::native_to_big(file_type_id)),
        payload_len_(boost::endian::native_to_big(payload_len)) {};
};

struct some_ip_file_read_req_t {
  uint32_t api_version_;
  uint32_t default_values_bitmask_;
  uint32_t file_type_id_;
  uint32_t buffer_size_;

  // Default constructor
  explicit some_ip_file_read_req_t(uint32_t file_type_id, uint32_t buffer_size)
      : api_version_(boost::endian::native_to_big(API_VERSION)),
        default_values_bitmask_(
            boost::endian::native_to_big(SOMEIP_DEFAULT_VALUES_BITMASK)),
        file_type_id_(boost::endian::native_to_big(file_type_id)),
        buffer_size_(boost::endian::native_to_big(buffer_size)) {};
};

struct some_ip_file_read_rsp_t {
  uint32_t status_;
  uint32_t payload_len_;
  /* Payload (with padding) will follow */

  uint32_t status() const noexcept {
    return boost::endian::big_to_native(status_);
  }

  uint32_t payload_len() const noexcept {
    return boost::endian::big_to_native(payload_len_);
  }
};

struct some_ip_file_erase_req_t {
  uint32_t api_version_;
  uint32_t default_values_bitmask_;
  uint32_t file_type_id_;
};

struct some_ip_file_info_req_t {
  uint32_t api_version_;
  uint32_t default_values_bitmask_;
  uint32_t file_type_id_;

  // Default constructor
  explicit some_ip_file_info_req_t(uint32_t file_type_id)
      : api_version_(boost::endian::native_to_big(API_VERSION)),
        default_values_bitmask_(
            boost::endian::native_to_big(SOMEIP_DEFAULT_VALUES_BITMASK)),
        file_type_id_(boost::endian::native_to_big(file_type_id)) {};
};

struct some_ip_file_info_rsp_t {
  uint32_t status_;
  uint32_t size_;
  uint32_t capacity_;
  uint32_t attributes_;

  uint32_t status() const noexcept {
    return boost::endian::big_to_native(status_);
  }

  uint32_t size() const noexcept { return boost::endian::big_to_native(size_); }

  uint32_t capacity() const noexcept {
    return boost::endian::big_to_native(capacity_);
  }

  uint32_t attributes() const noexcept {
    return boost::endian::big_to_native(attributes_);
  }
};

// Stop packing
#pragma pack(pop)
