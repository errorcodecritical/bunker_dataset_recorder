// Copyright (c) Sensrad 2023-2025
#pragma once

#include "window.hpp"
#include <gtkmm-3.0/gtkmm.h>

#include "rafgui_node.hpp"
#include <rclcpp/rclcpp.hpp>

#include <iostream>
#include <memory>
#include <thread>

class Application : public Gtk::Application {
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread executor_thread_;
  std::shared_ptr<RAFGuiNode> rafgui_node_;
  Window *window_;

  Glib::ustring rafgui_node_namespace_;
  Glib::ustring rafgui_node_name_;

protected:
  Application(int &argc, char **&argv,
              const Glib::ustring &application_id = Glib::ustring(),
              Gio::ApplicationFlags flags = Gio::APPLICATION_FLAGS_NONE)
      : Gtk::Application(argc, argv, application_id, flags),
        rafgui_node_(std::make_shared<RAFGuiNode>("hugin_d1_gui")),
        window_(nullptr) {

    // Get current active namespace and node name
    rafgui_node_namespace_ = rafgui_node_->get_namespace();
    rafgui_node_name_ = rafgui_node_->get_name();

    // Add node to executor
    executor_.add_node(rafgui_node_);
    // Run in it's own thread
    executor_thread_ = std::thread([this]() {
      executor_.spin();
      Glib::MainContext::get_default()->invoke([]() {
        Gtk::Application::get_default()->quit();
        return false;
      });
    });
  };

  // Called when main window is closed.
  bool on_window_delete_event(GdkEventAny *) {
    // Cancel any outstanding work
    executor_.cancel();
    // Wait for executor thread to finish
    executor_thread_.join();
    // Shutdown ROS2
    rclcpp::shutdown(nullptr, "ROS2 shutdown");
    // propagate the event to default handler which closes the window
    return false;
  }

public:
  static Glib::RefPtr<Application>
  create(int &argc, char **&argv,
         const Glib::ustring &application_id = Glib::ustring(),
         Gio::ApplicationFlags flags = Gio::APPLICATION_FLAGS_NONE) {

    return Glib::RefPtr<Application>(
        new Application(argc, argv, application_id, flags));
  }

  // Override default signal handlers:
  virtual void on_activate() override {
    // Load resources
    auto builder = Gtk::Builder::create_from_resource("/window/window.glade");

    // Pass a reference to the RAFGui node to the window
    builder->get_widget_derived("window", window_, rafgui_node_);
    g_assert(window_);
    add_window(*window_);
    // Show window
    window_->present();

    // Quit application when window is closed
    window_->signal_delete_event().connect(
        sigc::mem_fun(*this, &Application::on_window_delete_event));
  }

  virtual ~Application() {}
};
