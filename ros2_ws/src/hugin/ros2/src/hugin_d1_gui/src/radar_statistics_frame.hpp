// Copyright (c) Sensrad 2023-2024

#pragma once

#include <gtkmm-3.0/gtkmm.h>

class RadarStatisticsFrame : public Gtk::Frame {

  static constexpr double MS_TO_KMH = 3.6;

  Glib::RefPtr<Gtk::Builder> builder_;

  // Radar statistics
  Gtk::Label *statistics_timestamp_lbl = nullptr;
  Gtk::Label *statistics_framerate_lbl = nullptr;
  Gtk::Label *statistics_target_count_lbl = nullptr;
  Gtk::LevelBar *statistics_target_count_bar = nullptr;

  // Perception statistics
  Gtk::Label *statistics_track_count_lbl = nullptr;
  Gtk::Label *statistics_ego_speed_lbl = nullptr;
  Gtk::LevelBar *statistics_ego_speed_bar = nullptr;

protected:
  Glib::RefPtr<Gtk::Builder> builder() const;

public:
  RadarStatisticsFrame(BaseObjectType *cobject,
                       const Glib::RefPtr<Gtk::Builder> &builder)
      : Gtk::Frame(cobject), builder_(builder) {
    builder_->get_widget("statistics_timestamp", statistics_timestamp_lbl);
    g_assert(statistics_timestamp_lbl);
    builder_->get_widget("statistics_framerate", statistics_framerate_lbl);
    g_assert(statistics_framerate_lbl);
    builder_->get_widget("statistics_target_count",
                         statistics_target_count_lbl);
    g_assert(statistics_target_count_lbl);
    builder_->get_widget("statistics_point_count_bar",
                         statistics_target_count_bar);
    builder_->get_widget("statistics_track_count", statistics_track_count_lbl);
    g_assert(statistics_track_count_lbl);
    builder_->get_widget("statistics_ego_speed", statistics_ego_speed_lbl);
    g_assert(statistics_ego_speed_lbl);
    builder_->get_widget("statistics_rdr_ego_speed_bar",
                         statistics_ego_speed_bar);
  }

  virtual ~RadarStatisticsFrame() {}

  void set_timestamp(const std::int32_t sec, const std::uint32_t nanosec) {
    statistics_timestamp_lbl->set_text(
        Glib::ustring::sprintf("%d.%03d s", sec, nanosec));
  }

  void set_framerate(const double fps) {
    statistics_framerate_lbl->set_text(Glib::ustring::sprintf("%.0f Hz", fps));
  }

  void set_target_count(const std::uint32_t target_count) {
    // Set label
    statistics_target_count_lbl->set_text(
        Glib::ustring::sprintf("%d", target_count));
    // Set level bar
    statistics_target_count_bar->set_value(target_count);
  }

  void set_track_count(const std::uint32_t track_count) {
    statistics_track_count_lbl->set_text(
        Glib::ustring::sprintf("%d", track_count));
  }

  void set_ego_speed(const double ego_speed) {
    // Set label
    statistics_ego_speed_lbl->set_text(
        Glib::ustring::sprintf("%.1f km/h", MS_TO_KMH * ego_speed));
    // Set level bar
    statistics_ego_speed_bar->set_value(MS_TO_KMH * ego_speed);
  }
};
