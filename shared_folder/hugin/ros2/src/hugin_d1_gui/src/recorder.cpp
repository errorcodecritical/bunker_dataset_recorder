// Copyright (c) Sensrad 2024-2025

#include "recorder.hpp"

// Start record function
void Recorder::startRecord(const std::string &recording_name,
                           const ERecordingState &record_select,
                           const bool do_file_compression) {

  // Check if a recording is already running
  if (process_id_ > 0) {
    updateStatus("Recording is already running with PID: " +
                 std::to_string(process_id_));
    return;
  }

  // Check if the recording name is already existing
  std::ifstream f(recording_name);
  if (f.good()) {
    updateStatus("File with recording name already exists. \n"
                 "Please choose a different name.");
    recording_name_ = "";
    return;
  }

  // Valid recording name. Update class variable
  recording_name_ = recording_name;

  try {
    // Create a child process to record the bag
    std::vector<std::string> child_argv = {"ros2", "bag", "record"};

    // Add topics based on the recording state
    if (record_select == ERecordingState::RecDefault) {

      // Use regex to filter topics, and wildcard to add specified topics of
      // all namespaces
      child_argv.insert(
          child_argv.end(),
          {"-e",
           "/radar_data$|/radar_header$|/meta_data$|/control_state$|"
           "/ethernet_logger$|/compressed$|/camera_info$|"
           "^/tf_static$|^/imu$|^/gnss/nav_sat_fix$|^/rosout$|^/filter/|"
           "^/gnss$|^/gnss_pose$|^/imu/acceleration$|^/imu/angular_velocity$|"
           "^/ouster_.*/metadata$|^/ouster_.*/lidar_packets$|"
           "^/ouster_.*/imu_packets$|^/falcon_.*/iv_points$"});

    } else if (record_select == ERecordingState::RecAll) {
      // Record all topics
      child_argv.push_back("-a");
    }
    // Add storage and output options
    child_argv.insert(child_argv.end(),
                      {"--max-bag-duration", "60", "--max-bag-size",
                       "4000000000", "-s", "mcap", "-o", recording_name_});

    // Add file compression options
    if (do_file_compression) {
      child_argv.insert(child_argv.end(), {"--compression-mode", "file",
                                           "--compression-format", "zstd"});
    }

    // File descriptors for the child process
    int stdin_ros2;
    int stdout_ros2;
    int stderr_ros2;

    // Spawn the child process
    Glib::spawn_async_with_pipes(".", child_argv,
                                 Glib::SPAWN_SEARCH_PATH |
                                     Glib::SPAWN_DO_NOT_REAP_CHILD,
                                 sigc::slot<void>(), &process_id_, &stdin_ros2,
                                 &stdout_ros2, &stderr_ros2);

    // Add a watch for the child process termination
    Glib::signal_child_watch().connect(
        sigc::mem_fun(*this, &Recorder::onChildWatch), process_id_);
    updateStatus("Recording started with PID: " + std::to_string(process_id_));

    // Catch exceptions
  } catch (const Glib::SpawnError &e) {
    updateStatus("Error starting recording process: " + e.what());
    process_id_ = 0;
  } catch (const std::exception &e) {
    updateStatus("Error starting recording process: " + std::string(e.what()));
    process_id_ = 0;
  }
}

// Stop record function
void Recorder::stopRecord() {
  // Check if a recording is running
  if (process_id_ > 0) {

    // Terminate with SIGINT to the recording process
    if (kill(process_id_, SIGINT) == 0) {
      updateStatus("Sent termination signal to PID: " +
                   std::to_string(process_id_));
    } else {
      updateStatus("Failed to send termination signal to PID: " +
                   std::to_string(process_id_));
    }
    process_id_ = 0;
  } else {
    updateStatus("No recording process is running.");
  }
}

// Child watch function
void Recorder::onChildWatch(Glib::Pid pid, int child_status) {

  // Read the current working directory
  std::string cwd = getCurrentWorkingDirectory();

  std::string status = "Child process " + std::to_string(pid) +
                       " exited with status " + std::to_string(child_status) +
                       "\n" + "Saved at: " + cwd + "/" + recording_name_;
  updateStatus(status);
}
