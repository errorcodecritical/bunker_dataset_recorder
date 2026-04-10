// Copyright (c) Sensrad 2023-2025

#pragma once

#include <gtkmm-3.0/gtkmm.h>

class RadarStatusFrame : public Gtk::Frame {
  Glib::RefPtr<Gtk::Builder> builder_;

  // Radar status
  Gtk::Label *status_timestamp_lbl = nullptr;
  Gtk::Label *status_tx_enabled_lbl = nullptr;
  Gtk::Label *status_sequence_lbl = nullptr;
  Gtk::Label *status_threshold_lbl = nullptr;
  Gtk::Label *status_dynamic_azimuth_lbl = nullptr;
  Gtk::Label *status_dynamic_elevation_lbl = nullptr;
  Gtk::Label *status_dynamic_basic_lbl = nullptr;
  Gtk::Label *radar_connection_status_lbl = nullptr;

  // Perception status
  Gtk::Label *status_perception_config_lbl = nullptr;
  Gtk::Label *status_vertical_height_lbl = nullptr;
  Gtk::Label *status_elevation_angle_lbl = nullptr;
  Gtk::Label *status_search_radius_lbl = nullptr;
  Gtk::Label *status_free_space_range_lbl = nullptr;
  Gtk::Label *status_free_space_interval_lbl = nullptr;
  Gtk::Label *status_stationary_updates_lbl = nullptr;

protected:
  Glib::RefPtr<Gtk::Builder> builder() const;

public:
  RadarStatusFrame(BaseObjectType *cobject,
                   const Glib::RefPtr<Gtk::Builder> &builder)
      : Gtk::Frame(cobject), builder_(builder) {

    // Radar status
    builder_->get_widget("status_timestamp", status_timestamp_lbl);
    g_assert(status_timestamp_lbl);
    builder_->get_widget("status_tx_enabled", status_tx_enabled_lbl);
    g_assert(status_tx_enabled_lbl);
    builder_->get_widget("status_sequence", status_sequence_lbl);
    g_assert(status_sequence_lbl);
    builder_->get_widget("status_threshold", status_threshold_lbl);
    g_assert(status_threshold_lbl);
    builder_->get_widget("status_dynamic_azimuth", status_dynamic_azimuth_lbl);
    g_assert(status_dynamic_azimuth_lbl);
    builder_->get_widget("status_dynamic_elevation",
                         status_dynamic_elevation_lbl);
    g_assert(status_dynamic_elevation_lbl);
    builder_->get_widget("status_dynamic_basic", status_dynamic_basic_lbl);
    g_assert(status_dynamic_basic_lbl);
    builder_->get_widget("radar_connection_status",
                         radar_connection_status_lbl);
    g_assert(radar_connection_status_lbl);

    // Perception status
    builder_->get_widget("status_perception_config",
                         status_perception_config_lbl);
    g_assert(status_perception_config_lbl);
    builder_->get_widget("status_vertical_height", status_vertical_height_lbl);
    g_assert(status_vertical_height_lbl);
    builder_->get_widget("status_elevation_angle", status_elevation_angle_lbl);
    g_assert(status_elevation_angle_lbl);
    builder_->get_widget("status_search_radius", status_search_radius_lbl);
    g_assert(status_search_radius_lbl);
    builder_->get_widget("status_free_space_range",
                         status_free_space_range_lbl);
    g_assert(status_free_space_range_lbl);
    builder_->get_widget("status_free_space_interval",
                         status_free_space_interval_lbl);
    g_assert(status_free_space_interval_lbl);
    builder_->get_widget("status_stationary_updates",
                         status_stationary_updates_lbl);
    g_assert(status_stationary_updates_lbl);
  }

  void set_timestamp(const std::int32_t sec, const std::uint32_t nsec) {
    status_timestamp_lbl->set_text(
        Glib::ustring::sprintf("%d.%03d s", sec, nsec));
  }

  void set_tx_enabled(const bool enabled) {
    status_tx_enabled_lbl->set_markup(
        enabled ? "<span foreground='#1ABC9C'>Enabled</span>"
                : "<span foreground='#E0007B'>Disabled</span>");
  }

  void set_radar_connection_status(const bool connected) {
    radar_connection_status_lbl->set_markup(
        connected ? "<span foreground='#1ABC9C'>Yes</span>"
                  : "<span foreground='#E0007B'>No</span>");
  }

  void set_sequence_name(const std::string &sequence_name) {
    status_sequence_lbl->set_text(sequence_name);
  }

  void set_perception_config_status(const std::string &perception_config_name) {
    status_perception_config_lbl->set_text(perception_config_name);
  }

  void set_thresholds(const float static_threshold,
                      const float dynamic_azimutyh,
                      const float dynamic_elevation,
                      const float dynamic_basic) {

    status_threshold_lbl->set_text(
        Glib::ustring::sprintf("%.01f", static_threshold));
    status_dynamic_azimuth_lbl->set_text(
        Glib::ustring::sprintf("%.01f", dynamic_azimutyh));
    status_dynamic_elevation_lbl->set_text(
        Glib::ustring::sprintf("%.01f", dynamic_elevation));
    status_dynamic_basic_lbl->set_text(
        Glib::ustring::sprintf("%.01f", dynamic_basic));
  }

  void set_vertical_height(const float height) {
    status_vertical_height_lbl->set_text(
        Glib::ustring::sprintf("%.01f", height));
  }

  void set_elevation_angle(const float angle) {
    status_elevation_angle_lbl->set_text(
        Glib::ustring::sprintf("%.01f", angle));
  }

  void set_search_radius(const float scale) {
    status_search_radius_lbl->set_text(Glib::ustring::sprintf("%.01f", scale));
  }

  void set_free_space_range(const float range) {
    status_free_space_range_lbl->set_text(
        Glib::ustring::sprintf("%.01f", range));
  }

  void set_free_space_interval(const float min_value, const float max_value) {
    status_free_space_interval_lbl->set_text(
        Glib::ustring::sprintf("[%.01f , %.01f]", min_value, max_value));
  }

  void set_stationary_updates(const int stationary_updates) {
    status_stationary_updates_lbl->set_text(
        Glib::ustring::sprintf("%.01d", stationary_updates));
  }

  virtual ~RadarStatusFrame() {}
};
