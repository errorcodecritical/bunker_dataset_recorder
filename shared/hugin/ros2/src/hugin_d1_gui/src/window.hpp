// Copyright (c) Sensrad 2023-2025
#pragma once

#include <gdkmm/general.h> // For Gdk::RGBA
#include <gtkmm-3.0/gtkmm.h>

#include <fstream>
#include <glibmm.h>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept> // for std::invalid_argument and std::out_of_range
#include <string>
#include <unordered_map>
#include <vector>

#include "rafgui_node.hpp"

#include "radar_statistics_frame.hpp"
#include "radar_status_frame.hpp"

#include "filter.hpp"
#include "info_bar.hpp"

#include <rclcpp/rclcpp.hpp>

#include "recorder.hpp"

#include "application_finder.hpp"
#include "sensrad/version.hpp"

// Make a enum for the different slider values
enum class SliderValue {
  STATIC_SENSITIVITY = 10,
  DYNAMIC_ELEVATION = 15,
  DYNAMIC_AZIMUTH = 15,
  DYNAMIC_BASIC = 15,
};

class Window : public Gtk::Window {
private:
  // Initial backup values for the free space scanning interval
  // used in case of invalid input in GtkEntry
  float previous_lower_value_ = 0.5f;
  float previous_upper_value_ = 3.0f;
  bool initial_config_done_ = false;
  bool radar_active_;
  bool mode_config_received_ = false;
  bool perception_config_received_ = false;
  const float min_delta_time = 1.0F / 40.0F;
  const float PI_F = 3.14159265F;
  const float DEG_TO_RAD = PI_F / 180.0F;
  const float RAD_TO_DEG = 180.0F / PI_F;

  // Perception application timer polling
  sigc::connection application_poll_conn_;

  // Record animation
  sigc::connection timeout_connection_;
  const std::vector<std::string> record_frames_ = {
      "/image/frame_1.png", "/image/frame_2.png", "/image/frame_3.png",
      "/image/frame_4.png", "/image/frame_5.png", "/image/frame_6.png",
  };

  // Map for sequence strings
  std::unordered_map<uint32_t, std::string> sequence_strings_dynamic;

  // Map for perception configurations
  std::unordered_map<uint8_t, std::string> perception_configurations_dynamic;

public:
  Glib::RefPtr<Gtk::Builder> builder_;
  std::shared_ptr<RAFGuiNode> rafgui_node_;
  ApplicationFinder app_finder_; // ROS2 application finder instance

  // Widgets
  Gtk::Button *start_btn_ = nullptr;
  Gtk::Button *set_time_btn_ = nullptr;
  Gtk::ComboBoxText *sequence_combo_ = nullptr;
  Gtk::Scale *static_sensitivity_scale_ = nullptr;
  Gtk::Scale *dynamic_azimuth_scale_ = nullptr;
  Gtk::Scale *dynamic_elevation_scale_ = nullptr;
  Gtk::Scale *dynamic_basic_scale_ = nullptr;

  // Perception widgets
  Gtk::Scale *vertical_height_scale_ = nullptr;
  Gtk::Scale *elevation_angle_scale_ = nullptr;
  Gtk::Scale *cluster_search_scale_ = nullptr;
  Gtk::Scale *free_space_max_range_scale_ = nullptr;
  Gtk::Scale *stationary_updates_scale_ = nullptr;
  Gtk::Entry *free_space_min_scanning_interval_ = nullptr;
  Gtk::Entry *free_space_max_scanning_interval_ = nullptr;
  Gtk::Button *perception_reset_btn_ = nullptr;
  Gtk::ComboBoxText *perception_configs_combo_ = nullptr;

  // Region of interest widgets
  Gtk::Button *roi_window_btn_ = nullptr;
  Gtk::CheckButton *roi_onoff_ = nullptr;
  Gtk::Scale *roi_x_center_ = nullptr;
  Gtk::Scale *roi_y_center_ = nullptr;
  Gtk::Scale *roi_z_center_ = nullptr;
  Gtk::Scale *roi_width_ = nullptr;
  Gtk::Scale *roi_length_ = nullptr;
  Gtk::Scale *roi_height_ = nullptr;
  Gtk::Scale *roi_yaw_ = nullptr;
  Gtk::Scale *roi_pitch_ = nullptr;

  // New window widget
  Gtk::Button *info_btn_ = nullptr;

  // Record widgets
  Gtk::Button *record_btn_ = nullptr;
  Gtk::Entry *record_file_name_ = nullptr;
  Gtk::Label *record_label_ = nullptr;
  Gtk::ComboBoxText *record_select_ = nullptr;
  Gtk::CheckButton *record_compression_ = nullptr;
  Gtk::Image *record_image_ = nullptr;

  // Namespace widget
  Gtk::Label *rafgui_node_namespace_ = nullptr;

  // Applications
  Gtk::Label *no_application_lbl_ = nullptr;

  // Rockfall application
  Gtk::Button *rockfall_activate_btn_ = nullptr;
  Gtk::Label *rockfall_activate_lbl_ = nullptr;
  Glib::Pid rockfall_child_pid_{0};
  sigc::connection rockfall_watch_conn_;

  // Derived widgets
  RadarStatusFrame *radar_status_frame_ = nullptr;
  RadarStatisticsFrame *radar_statistics_frame_ = nullptr;
  InfoBar *info_bar_ = nullptr;

  // First order differential filter for frame rate smoothing.
  FirstOrderDiffFilter<double> frame_rate_filter_{/* alpha = */ 0.8};

  void on_sequence_changed() {

    const auto sequence = sequence_combo_->get_active_text();

    // Search for the sequence type
    bool found = false;
    for (const auto &pair : sequence_strings_dynamic) {
      if (pair.second == sequence) {
        rafgui_node_->set_sequence(pair.first);
        found = true;
        break;
      }
    }
    if (!found) {
      std::cerr << "Unknown sequence type: " << sequence << std::endl;
    }
  }

  // Signal callbacks
  void on_threshold_value_changed() {
    rafgui_node_->set_threshold(
        static_cast<float>(static_sensitivity_scale_->get_value()),
        static_cast<float>(dynamic_azimuth_scale_->get_value()),
        static_cast<float>(dynamic_elevation_scale_->get_value()),
        static_cast<float>(dynamic_basic_scale_->get_value()));
  }

  void on_control_state(RAFGuiNode::ControlStateSharedPtr msg) {

    if (radar_active_ != msg->tx_enabled) {
      //  Update state
      radar_active_ = msg->tx_enabled;

      // Update button text and theme
      auto style_context = start_btn_->get_style_context();
      if (radar_active_) {
        style_context->remove_class("start-theme");
        style_context->add_class("stop-theme");
        start_btn_->set_label("Stop Tx");
      } else {
        style_context->remove_class("stop-theme");
        style_context->add_class("start-theme");
        start_btn_->set_label("Start Tx");
      }
    }

    // Read the available modes from the message - but only at startup.
    // Incoming types are:
    //  int32[]  mode_ids
    //  string[] mode_names
    if (!mode_config_received_) {
      sequence_combo_->remove_all();
      const auto num_modes = msg->mode_ids.size();
      if (num_modes > 0) {
        mode_config_received_ = true;

        for (size_t i = 0; i < num_modes; i++) {
          sequence_strings_dynamic[msg->mode_ids[i]] = msg->mode_names[i];
        }

        for (const auto &pair : sequence_strings_dynamic) {
          sequence_combo_->append(pair.second);
        }
      }
    }

    if (mode_config_received_) {
      // Update the selected mode
      std::string sequence_name = "Unknown";
      // Check if the selected mode is present in the sequence_strings map
      if (sequence_strings_dynamic.find(msg->selected_mode_id) !=
          sequence_strings_dynamic.end()) {
        sequence_name = sequence_strings_dynamic.at(msg->selected_mode_id);
      }
      // Set active text
      sequence_combo_->set_active_text(sequence_name);
      radar_status_frame_->set_sequence_name(sequence_name);
    }

    radar_status_frame_->set_timestamp(msg->header.stamp.sec,
                                       msg->header.stamp.nanosec);
    radar_status_frame_->set_tx_enabled(msg->tx_enabled);

    set_time_btn_->set_sensitive(!msg->ptp_is_active);

    radar_status_frame_->set_thresholds(
        msg->thresholds.static_threshold, msg->thresholds.dynamic_azimuth,
        msg->thresholds.dynamic_elevation, msg->thresholds.dynamic_basic);

    radar_status_frame_->set_radar_connection_status(msg->is_connected);
  }

  void on_oden_control_state(RAFGuiNode::OdenControlStateSharedPtr msg) {

    // Read the available perception configs - but only at startup.
    // Incoming types are:
    //  uint8[] config_identifiers
    //  string[] config_names
    if (!perception_config_received_) {
      perception_configs_combo_->remove_all();
      const auto num_configs = msg->config_names.size();
      if (num_configs > 0) {
        perception_config_received_ = true;

        for (size_t i = 0; i < num_configs; i++) {
          perception_configurations_dynamic[msg->config_identifiers[i]] =
              msg->config_names[i];
        }

        for (const auto &pair : perception_configurations_dynamic) {
          perception_configs_combo_->append(pair.second);
        }
      }
    }

    if (perception_config_received_) {
      // Update the selected mode
      std::string config_name = "Unknown";
      // Check if the selected mode is present in the sequence_strings map
      if (perception_configurations_dynamic.find(msg->perception_config) !=
          perception_configurations_dynamic.end()) {
        config_name =
            perception_configurations_dynamic.at(msg->perception_config);
      }
      // Set active text
      perception_configs_combo_->set_active_text(config_name);
      radar_status_frame_->set_perception_config_status(config_name);
    }

    radar_status_frame_->set_vertical_height(msg->vertical_height);
    radar_status_frame_->set_elevation_angle(msg->elevation_angle);
    radar_status_frame_->set_search_radius(msg->cluster_scale);
    radar_status_frame_->set_free_space_range(msg->free_space_range);
    radar_status_frame_->set_free_space_interval(msg->free_space_lower_bound,
                                                 msg->free_space_upper_bound);
    radar_status_frame_->set_stationary_updates(msg->stationary_updates);

    // Ad-hoc solution to configure GUI with initial perception parameters
    if (!initial_config_done_) {
      vertical_height_scale_->set_value(msg->vertical_height);
      elevation_angle_scale_->set_value(msg->elevation_angle);
      cluster_search_scale_->set_value(msg->cluster_scale);
      free_space_max_range_scale_->set_value(msg->free_space_range);

      // String with one decimal
      std::ostringstream text_lower;
      text_lower << std::fixed << std::setprecision(1)
                 << msg->free_space_lower_bound;
      free_space_min_scanning_interval_->set_text(text_lower.str());
      std::ostringstream text_upper;
      text_upper << std::fixed << std::setprecision(1)
                 << msg->free_space_upper_bound;
      free_space_max_scanning_interval_->set_text(text_upper.str());

      stationary_updates_scale_->set_value(msg->stationary_updates);

      roi_onoff_->set_active(msg->roi_enabled); // Set the ROI checkbox state
      roi_x_center_->set_value(msg->roi_x_center);
      roi_y_center_->set_value(msg->roi_y_center);
      roi_z_center_->set_value(msg->roi_z_center);
      roi_width_->set_value(msg->roi_width);
      roi_length_->set_value(msg->roi_length);
      roi_height_->set_value(msg->roi_height);
      roi_yaw_->set_value(msg->roi_yaw * RAD_TO_DEG);     // Convert to degrees
      roi_pitch_->set_value(msg->roi_pitch * RAD_TO_DEG); // Convert to degrees

      initial_config_done_ = true;
    }
  }

  void on_oden_statistics(RAFGuiNode::OdenStatisticsSharedPtr msg) {
    radar_statistics_frame_->set_track_count(msg->num_tracks);
    radar_statistics_frame_->set_ego_speed(msg->ego_speed);
  }

  void on_pointcloud(RAFGuiNode::PointCloud2SharedPtr msg) {
    // Estimate frame rate
    const double t = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

    const auto target_count = msg->width;
    // Drop effective frame rate if empty point-clouds
    if (target_count > 0) {
      const double d_t = frame_rate_filter_.filter(t);
      // First sample will be 0, ignore that.
      if (d_t > min_delta_time) {
        radar_statistics_frame_->set_framerate(1.0 / d_t);
      }
    }

    radar_statistics_frame_->set_timestamp(msg->header.stamp.sec,
                                           msg->header.stamp.nanosec);

    radar_statistics_frame_->set_target_count(target_count);
  }

  void on_t11_pointcloud() {
    // std::cerr << "T11 pointcloud received" << std::endl;
  }

  void on_message(const std::string &message) {
    info_bar_->set_message(message);
  }

  void on_record_start() {
    // Function for record start

    // Get the record file name
    std::string record_file_name = record_file_name_->get_text();

    // Get the record message setting
    const std::unordered_map<std::string, Recorder::ERecordingState>
        record_types = {{"Preselected", Recorder::RecDefault},
                        {"All", Recorder::RecAll}};

    const auto record_select = record_select_->get_active_text();
    Recorder::ERecordingState selected_record_type = Recorder::RecDefault;
    if (const auto it = record_types.find(record_select);
        it != record_types.end()) {
      selected_record_type = it->second;

    } else {
      std::cerr << "Unknown record type: " << record_select << std::endl;
      return;
    }

    // Read the record compression setting
    bool record_compression = record_compression_->get_active();

    // Start the record
    rafgui_node_->start_record(record_file_name, selected_record_type,
                               record_compression);

    // Start the record animation
    if (rafgui_node_->is_recording()) {

      // Update the record image
      static int frame = 0;
      // Timer callback to update the image frame
      auto update_image_frame = [this]() -> bool {
        frame = (frame + 1) % static_cast<int>(record_frames_.size());
        record_image_->set_from_resource(record_frames_[frame]);

        // Update text and theme of button
        auto style_context = record_btn_->get_style_context();
        style_context->remove_class("start-theme");
        style_context->add_class("stop-theme");
        record_btn_->set_label("Stop Record");

        return true; // Continue calling this function
      };

      // Start the animation
      if (!timeout_connection_) {
        timeout_connection_ =
            Glib::signal_timeout().connect(update_image_frame, 200);
      }
    }
  }

  void on_record_update(const std::string &message) {
    // Function for record status update
    record_label_->set_text(message);
  }

  void on_record_stop() {
    // Function for record stop
    rafgui_node_->stop_record();

    // Stop the record animation
    if (!rafgui_node_->is_recording()) {
      timeout_connection_.disconnect();
      record_image_->set_from_resource("/image/frame_0.png");

      auto style_context = record_btn_->get_style_context();
      style_context->remove_class("stop-theme");
      style_context->add_class("start-theme");
      record_btn_->set_label("Start Record");
    }
  }

  void toggle_record() {
    // Function for record toggle

    if (rafgui_node_->is_recording()) {
      on_record_stop();
    } else {
      on_record_start();
    }
  }

  // Perception callbacks
  void on_perception_parameter_changed() {
    // Extract value of all parameters that can be changed

    Glib::ustring text_lower = free_space_min_scanning_interval_->get_text();
    Glib::ustring text_upper = free_space_max_scanning_interval_->get_text();

    float value_lower = previous_lower_value_;
    float value_upper = previous_upper_value_;
    try {
      // Attempt to convert the text to a float
      value_lower = static_cast<float>(std::stof(text_lower.raw()));
      value_upper = static_cast<float>(std::stof(text_upper.raw()));
      previous_lower_value_ = value_lower;
      previous_upper_value_ = value_upper;

    } catch (const std::invalid_argument &ia) {
      // Handle case where the input is not a valid float
      std::cerr << "Error: Invalid input, not a number" << std::endl;
    } catch (const std::out_of_range &oor) {
      // Handle case where the float is out of the valid range
      std::cerr << "Error: Number out of range" << std::endl;
    }

    // Find the selected perception configuration
    const auto config_name = perception_configs_combo_->get_active_text();
    bool found = false;
    uint8_t config_id = 0;
    for (const auto &pair : perception_configurations_dynamic) {
      if (pair.second == config_name) {
        config_id = pair.first;
        found = true;
        break;
      }
    }
    if (!found) {
      std::cerr << "Unknown perception config: " << config_name << std::endl;
    }

    rafgui_node_->set_perception_parameters(
        config_id, static_cast<float>(vertical_height_scale_->get_value()),
        static_cast<float>(elevation_angle_scale_->get_value()),
        static_cast<float>(cluster_search_scale_->get_value()),
        static_cast<float>(free_space_max_range_scale_->get_value()),
        value_lower, value_upper,
        static_cast<int>(stationary_updates_scale_->get_value()));
  }

  void on_roi_parameter_changed() {
    // Extract value of all parameters that can be changed
    rafgui_node_->set_roi_parameters(
        roi_onoff_->get_active(),
        static_cast<float>(roi_x_center_->get_value()),
        static_cast<float>(roi_y_center_->get_value()),
        static_cast<float>(roi_z_center_->get_value()),
        static_cast<float>(roi_width_->get_value()),
        static_cast<float>(roi_length_->get_value()),
        static_cast<float>(roi_height_->get_value()),
        static_cast<float>(roi_yaw_->get_value() * DEG_TO_RAD),
        static_cast<float>(roi_pitch_->get_value() * DEG_TO_RAD));
  }

  void on_info_window_clicked() {
    // Open a Gtk Window with id information_window
    Gtk::Window *info_window_lbl = nullptr;
    builder_->get_widget("info_window", info_window_lbl);
    g_assert(info_window_lbl);
    info_window_lbl->show();
  }

  void toggle_tx() {
    // Function for toggling the radar transmission

    if (radar_active_) {
      rafgui_node_->stop_tx();
    } else {
      rafgui_node_->start_tx();
    }
  }

  void on_roi_window_open() {
    Gtk::Window *roi_window = nullptr;
    builder_->get_widget("roi_window", roi_window);
    g_assert(roi_window);
    roi_window->show();
    // Function for opening the GUI window
    show();
  }

  void on_rockfall_open() {
    if (rockfall_child_pid_ != 0) {
      // Already running. Broadcast a message.
      info_bar_->set_message("Rockfall GUI is already running.");
      return;
    }

    // Executable to launch (must be on $PATH after sourcing setup.bash)
    Glib::ustring exe = "rockfall_gui_node";

    // Build the ROS2-remapping argument, e.g.  __ns:=/sensrad_1
    Glib::ustring ns = rafgui_node_->get_namespace();
    Glib::ustring nsarg = Glib::ustring("__ns:=") + ns;

    std::vector<Glib::ustring> argv{exe, "--ros-args", "-r", nsarg};

    // Ask Glib to leave the child unreaped so we can watch it.
    Glib::SpawnFlags flags =
        Glib::SPAWN_SEARCH_PATH | Glib::SPAWN_DO_NOT_REAP_CHILD;

    try {
      Glib::spawn_async(Glib::get_current_dir(), // working dir
                        argv,                    // argv
                        flags,
                        sigc::slot<void>(),    // no special setup
                        &rockfall_child_pid_); // get the PID back

      // Disable the launcher while Rockfall is alive
      rockfall_activate_btn_->set_sensitive(false);

      // Monitor child; this will call us when Rockfall exits
      rockfall_watch_conn_ = Glib::signal_child_watch().connect(
          sigc::mem_fun(*this, &Window::on_rockfall_child_exit),
          rockfall_child_pid_);
    } catch (const Glib::SpawnError &ex) {
      info_bar_->set_message(Glib::ustring("Failed to launch Rockfall GUI: ") +
                             ex.what());
      rockfall_child_pid_ = 0;
    }
  }
  void on_rockfall_child_exit(Glib::Pid /*pid*/, int /*status*/) {
    // Clean up
    rockfall_watch_conn_.disconnect();
    Glib::spawn_close_pid(rockfall_child_pid_);
    rockfall_child_pid_ = 0;

    // Re-enable the launcher button
    rockfall_activate_btn_->set_sensitive(true);
  }

  bool on_application_show() {
    // Function for showing active perception application

    // Get the list of active applications
    auto apps = app_finder_.present_applications();

    // If apps is empty, show "no application" message
    if (apps.empty()) {
      // Hide rockfall
      rockfall_activate_btn_->set_visible(false);
      rockfall_activate_lbl_->set_visible(false);

      // Show "no application" message
      no_application_lbl_->set_visible(true);
      no_application_lbl_->set_label(
          "No active applications found. \n"
          "To learn more about our solutions, please contact Sensrad Sales.");
      return true;
    }

    // If argument is not empty, hide "no application" message
    no_application_lbl_->set_visible(false);
    no_application_lbl_->set_label("");

    // Loop over all arguments
    for (const auto &app : apps) {
      if (app == "rockfall") {
        // Make rockfall button visible
        rockfall_activate_btn_->set_visible(true);
        rockfall_activate_lbl_->set_visible(true);
      }
    }
    return true;
  }

public:
  Window(BaseObjectType *cobject, const Glib::RefPtr<Gtk::Builder> &builder,
         std::shared_ptr<RAFGuiNode> &rafgui_node)
      : Gtk::Window(cobject), radar_active_(false), builder_(builder),
        rafgui_node_(rafgui_node), app_finder_() {

    g_assert(rafgui_node_);

    builder_->get_widget("no_applications_found_lbl", no_application_lbl_);
    g_assert(no_application_lbl_);

    builder_->get_widget("rockfall_activate_btn", rockfall_activate_btn_);
    g_assert(rockfall_activate_btn_);
    rockfall_activate_btn_->signal_clicked().connect(
        sigc::mem_fun(*this, &Window::on_rockfall_open));

    builder_->get_widget("rockfall_activate_lbl", rockfall_activate_lbl_);
    g_assert(rockfall_activate_lbl_);

    // Set namespace label
    builder_->get_widget("namespace_lbl", rafgui_node_namespace_);
    g_assert(rafgui_node_namespace_);
    rafgui_node_namespace_->set_text(rafgui_node_->get_namespace());

    // Connect signals
    // Start and stop button
    builder_->get_widget("start_btn", start_btn_);
    g_assert(start_btn_);
    start_btn_->signal_clicked().connect(
        sigc::mem_fun(*this, &Window::toggle_tx));
    start_btn_->get_style_context()->add_class("start-theme");

    // Set time button
    builder_->get_widget("set_time_btn", set_time_btn_);
    g_assert(set_time_btn_);
    set_time_btn_->signal_clicked().connect(
        sigc::mem_fun(*rafgui_node, &RAFGuiNode::set_time));

    // Sequence combo
    builder_->get_widget("sequence", sequence_combo_);
    g_assert(sequence_combo_);
    sequence_combo_->signal_changed().connect(
        sigc::mem_fun(this, &Window::on_sequence_changed));

    // Threshold scales
    builder_->get_widget("static_sensitivity", static_sensitivity_scale_);
    g_assert(static_sensitivity_scale_);
    builder_->get_widget("dynamic_azimuth", dynamic_azimuth_scale_);
    g_assert(dynamic_azimuth_scale_);
    builder_->get_widget("dynamic_elevation", dynamic_elevation_scale_);
    g_assert(dynamic_elevation_scale_);
    builder_->get_widget("dynamic_basic", dynamic_basic_scale_);
    g_assert(dynamic_basic_scale_);

    // Get derived widgets
    builder->get_widget_derived("info_bar", info_bar_);
    g_assert(info_bar_);

    builder_->get_widget_derived("radar_status_frame", radar_status_frame_);
    g_assert(radar_status_frame_);

    builder_->get_widget_derived("radar_statistics_frame",
                                 radar_statistics_frame_);
    g_assert(radar_statistics_frame_);

    Glib::RefPtr<Gdk::Pixbuf> iconPixbuf = Gdk::Pixbuf::create_from_resource(
        "/image/sensrad_eye_magenta_small.png");
    set_icon(iconPixbuf);

    // Update SDK version label
    Gtk::Label *sdk_version_lbl = nullptr;
    builder_->get_widget("sdk_version", sdk_version_lbl);
    g_assert(sdk_version_lbl);
    sdk_version_lbl->set_text(
        Glib::ustring::compose("SDK: %1", std::string(sensrad::sdk::version)));

    // Set up the scales
    // Set the correct value
    static_sensitivity_scale_->set_value(
        static_cast<double>(SliderValue::STATIC_SENSITIVITY));
    dynamic_azimuth_scale_->set_value(
        static_cast<double>(SliderValue::DYNAMIC_AZIMUTH));
    dynamic_elevation_scale_->set_value(
        static_cast<double>(SliderValue::DYNAMIC_ELEVATION));
    dynamic_basic_scale_->set_value(
        static_cast<double>(SliderValue::DYNAMIC_BASIC));

    // Add marks to the scales
    static_sensitivity_scale_->add_mark(
        static_cast<double>(SliderValue::STATIC_SENSITIVITY), Gtk::POS_BOTTOM,
        "");
    dynamic_azimuth_scale_->add_mark(
        static_cast<double>(SliderValue::DYNAMIC_AZIMUTH), Gtk::POS_BOTTOM, "");
    dynamic_elevation_scale_->add_mark(
        static_cast<double>(SliderValue::DYNAMIC_ELEVATION), Gtk::POS_BOTTOM,
        "");
    dynamic_basic_scale_->add_mark(
        static_cast<double>(SliderValue::DYNAMIC_BASIC), Gtk::POS_BOTTOM, "");
    // Connect them all to the same slot, they are always set at
    // the same time.
    static_sensitivity_scale_->signal_value_changed().connect(
        sigc::mem_fun(this, &Window::on_threshold_value_changed));
    dynamic_azimuth_scale_->signal_value_changed().connect(
        sigc::mem_fun(this, &Window::on_threshold_value_changed));
    dynamic_elevation_scale_->signal_value_changed().connect(
        sigc::mem_fun(this, &Window::on_threshold_value_changed));
    dynamic_basic_scale_->signal_value_changed().connect(
        sigc::mem_fun(this, &Window::on_threshold_value_changed));

    // Connect node signals
    rafgui_node_->signal_on_control_state().connect(
        sigc::mem_fun(*this, &Window::on_control_state));

    rafgui_node_->signal_on_oden_control_state().connect(
        sigc::mem_fun(*this, &Window::on_oden_control_state));

    rafgui_node_->signal_on_oden_statistics().connect(
        sigc::mem_fun(*this, &Window::on_oden_statistics));

    rafgui_node_->signal_on_pointcloud().connect(
        sigc::mem_fun(*this, &Window::on_pointcloud));

    rafgui_node_->signal_on_t11_pointcloud().connect(
        sigc::mem_fun(*this, &Window::on_t11_pointcloud));

    rafgui_node_->signal_on_message().connect(
        sigc::mem_fun(info_bar_, &InfoBar::set_message));

    // Perception widgets
    builder_->get_widget("perception_reset_btn", perception_reset_btn_);
    g_assert(perception_reset_btn_);
    perception_reset_btn_->signal_clicked().connect(
        sigc::mem_fun(*rafgui_node, &RAFGuiNode::reset_perception));

    builder_->get_widget("perception_config", perception_configs_combo_);
    g_assert(perception_configs_combo_);
    perception_configs_combo_->signal_changed().connect(
        sigc::mem_fun(*this, &Window::on_perception_parameter_changed));

    builder_->get_widget("vertical_height_sldr", vertical_height_scale_);
    g_assert(vertical_height_scale_);
    vertical_height_scale_->signal_value_changed().connect(
        sigc::mem_fun(*this, &Window::on_perception_parameter_changed));

    builder_->get_widget("elevation_angle_sldr", elevation_angle_scale_);
    g_assert(elevation_angle_scale_);
    elevation_angle_scale_->signal_value_changed().connect(
        sigc::mem_fun(*this, &Window::on_perception_parameter_changed));

    builder_->get_widget("cluster_scale_sldr", cluster_search_scale_);
    g_assert(cluster_search_scale_);
    cluster_search_scale_->signal_value_changed().connect(
        sigc::mem_fun(*this, &Window::on_perception_parameter_changed));

    builder_->get_widget("free_space_range_sldr", free_space_max_range_scale_);
    g_assert(free_space_max_range_scale_);
    free_space_max_range_scale_->signal_value_changed().connect(
        sigc::mem_fun(*this, &Window::on_perception_parameter_changed));

    builder_->get_widget("stationary_updates_sldr", stationary_updates_scale_);
    g_assert(stationary_updates_scale_);
    stationary_updates_scale_->signal_value_changed().connect(
        sigc::mem_fun(*this, &Window::on_perception_parameter_changed));

    builder_->get_widget("free_space_lower_bound",
                         free_space_min_scanning_interval_);
    g_assert(free_space_min_scanning_interval_);
    free_space_min_scanning_interval_->signal_changed().connect(
        sigc::mem_fun(*this, &Window::on_perception_parameter_changed));

    builder_->get_widget("free_space_upper_bound",
                         free_space_max_scanning_interval_);
    g_assert(free_space_max_scanning_interval_);
    free_space_max_scanning_interval_->signal_changed().connect(
        sigc::mem_fun(*this, &Window::on_perception_parameter_changed));

    // Additional windows
    builder_->get_widget("info_btn", info_btn_);
    g_assert(info_btn_);
    info_btn_->signal_clicked().connect(
        sigc::mem_fun(*this, &Window::on_info_window_clicked));

    // Record buttons
    builder_->get_widget("rcd_btn", record_btn_);
    g_assert(record_btn_);
    record_btn_->signal_clicked().connect(
        sigc::mem_fun(*this, &Window::toggle_record));
    // Add start theme
    record_btn_->get_style_context()->add_class("start-theme");

    builder_->get_widget("rcd_entry", record_file_name_);
    g_assert(record_file_name_);

    builder_->get_widget("rcd_label", record_label_);
    g_assert(record_label_);

    builder_->get_widget("record_select", record_select_);
    g_assert(record_select_);

    builder_->get_widget("record_compression", record_compression_);
    g_assert(record_compression_);

    builder_->get_widget("record_animation", record_image_);
    g_assert(record_image_);

    builder_->get_widget("roi_btn", roi_window_btn_);
    g_assert(roi_window_btn_);
    roi_window_btn_->signal_clicked().connect(
        sigc::mem_fun(*this, &Window::on_roi_window_open));

    builder_->get_widget("roi_onoff", roi_onoff_);
    g_assert(roi_onoff_);
    roi_onoff_->signal_toggled().connect(
        sigc::mem_fun(*this, &Window::on_roi_parameter_changed));

    builder_->get_widget("x_center_sldr", roi_x_center_);
    g_assert(roi_x_center_);
    roi_x_center_->signal_value_changed().connect(
        sigc::mem_fun(*this, &Window::on_roi_parameter_changed));

    builder_->get_widget("y_center_sldr", roi_y_center_);
    g_assert(roi_y_center_);
    roi_y_center_->signal_value_changed().connect(
        sigc::mem_fun(*this, &Window::on_roi_parameter_changed));

    builder_->get_widget("z_center_sldr", roi_z_center_);
    g_assert(roi_z_center_);
    roi_z_center_->signal_value_changed().connect(
        sigc::mem_fun(*this, &Window::on_roi_parameter_changed));

    builder_->get_widget("width_sldr", roi_width_);
    g_assert(roi_width_);
    roi_width_->signal_value_changed().connect(
        sigc::mem_fun(*this, &Window::on_roi_parameter_changed));

    builder_->get_widget("length_sldr", roi_length_);
    g_assert(roi_length_);
    roi_length_->signal_value_changed().connect(
        sigc::mem_fun(*this, &Window::on_roi_parameter_changed));

    builder_->get_widget("height_sldr", roi_height_);
    g_assert(roi_height_);
    roi_height_->signal_value_changed().connect(
        sigc::mem_fun(*this, &Window::on_roi_parameter_changed));

    builder_->get_widget("yaw_sldr", roi_yaw_);
    g_assert(roi_yaw_);
    roi_yaw_->signal_value_changed().connect(
        sigc::mem_fun(*this, &Window::on_roi_parameter_changed));

    builder_->get_widget("pitch_sldr", roi_pitch_);
    g_assert(roi_pitch_);
    roi_pitch_->signal_value_changed().connect(
        sigc::mem_fun(*this, &Window::on_roi_parameter_changed));

    // Connect RAFGuiNode signal
    rafgui_node_->signal_record_update.connect(
        sigc::mem_fun(*this, &Window::on_record_update));

    // Show active perception applications
    // on_application_show();
    application_poll_conn_ = Glib::signal_timeout().connect_seconds(
        sigc::mem_fun(*this, &Window::on_application_show), 2);

    // Set up CSS for window theme
    auto css_provider = Gtk::CssProvider::create();
    css_provider->load_from_resource("image/styles.css");

    auto screen = Gdk::Screen::get_default();
    Gtk::StyleContext::add_provider_for_screen(
        screen, css_provider, GTK_STYLE_PROVIDER_PRIORITY_USER);
    
    // AUTO-START TX: Start radar transmission automatically when GUI launches
    Glib::signal_idle().connect([this]() {
        if (!radar_active_) {
            toggle_tx();  // Automatically start transmission
        }
        return false;  // Execute only once
    });
  }

  virtual ~Window() {
    // Disconnect application polling signal
    application_poll_conn_.disconnect();

    // Disconnect all signals
    if (rockfall_child_pid_ != 0) {
      rockfall_watch_conn_.disconnect();
      Glib::spawn_close_pid(rockfall_child_pid_);
    }
  };
};
