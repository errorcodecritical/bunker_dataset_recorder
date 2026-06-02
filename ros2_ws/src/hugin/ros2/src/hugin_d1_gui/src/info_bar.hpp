// Copyright (c) Sensrad 2024

#pragma once

#include <gtkmm-3.0/gtkmm.h>

#include <iostream>

class InfoBar : public Gtk::InfoBar {
  Glib::RefPtr<Gtk::Builder> builder_;

  Gtk::Label *label_;

  void on_button_clicked(int) { hide(); }

protected:
  Glib::RefPtr<Gtk::Builder> builder() const;

public:
  InfoBar(BaseObjectType *cobject, const Glib::RefPtr<Gtk::Builder> &builder)
      : Gtk::InfoBar(cobject), builder_(builder) {

    builder_->get_widget("info_bar_label", label_);
    g_assert(label_);

    signal_response().connect(
        sigc::mem_fun(*this, &InfoBar::on_button_clicked));
  }

  void set_message(const Glib::ustring &message) {
    label_->set_text(message);
    show();
  }

  virtual ~InfoBar() {}
};
