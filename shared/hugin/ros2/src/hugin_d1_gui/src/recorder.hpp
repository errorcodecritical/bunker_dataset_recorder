// Copyright (c) Sensrad 2024

#pragma once

#include <boost/format.hpp>
#include <csignal>
#include <fstream>
#include <glibmm.h>
#include <iostream>
#include <mutex>
#include <string>
#include <vector>

#include <unistd.h>

class Recorder {

public:
  // Possible recording states
  enum ERecordingState {
    RecDefault = 1,
    RecAll = 2,
  };

  // Signal for updating the status of the recording to GUI
  sigc::signal<void, const std::string &> signal_record_update;

  // Start record function
  void startRecord(const std::string &recording_name,
                   const ERecordingState &record_select,
                   const bool do_file_compression);

  // Stop record function
  void stopRecord();

  bool isRecording() { return process_id_ > 0; }

private:
  Glib::Pid process_id_;       // Process ID of the recording process
  std::string recording_name_; // Name of the recording file

  // Function to update the status of the recording
  void updateStatus(const std::string &status) {
    signal_record_update.emit(status);
  }

  // Child watch function
  void onChildWatch(Glib::Pid pid, int child_status);

  // Function to get current working directory
  std::string getCurrentWorkingDirectory() {
    char temp[PATH_MAX];
    return (getcwd(temp, sizeof(temp)) ? std::string(temp) : std::string(""));
  }
};
