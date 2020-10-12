#pragma once

#include "model.h"
#include <gtkmm.h>
#include <sigc++/sigc++.h>

// This is the top level class for the view
class SimulationWindow : public sigc::trackable {
public:
  SimulationWindow() = default;
  virtual ~SimulationWindow() = default;

  void from_file() throw();

  Gtk::Window &getWindow();

protected:
  Gtk::Window *window;
  sigc::connection timout;

  // main update loop
  bool step();

};
