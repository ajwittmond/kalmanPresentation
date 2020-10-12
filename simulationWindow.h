#pragma once

#include <gtkmm.h>
#include <sigc++/sigc++.h>


const double STEP_FREQUENCY = 60;

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

  Glib::RefPtr<Gtk::Builder> builder;

  bool running;
  // main update loop
  bool step();

};
