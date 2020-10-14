#pragma once

#include <gtkmm.h>
#include <sigc++/sigc++.h>
#include "gtkmm/drawingarea.h"
#include "simulation.h"

const double STEP_FREQUENCY = 60;

// This is the top level class for the view
class SimulationWindow : public sigc::trackable {
public:
  SimulationWindow() = default;

  virtual ~SimulationWindow() = default;

  void from_file() throw();

  Gtk::Window &getWindow();

private:
  Gtk::Window *window;
  sigc::connection timout;
  std::unique_ptr<Simulation> simulation;
  Glib::RefPtr<Gtk::Builder> builder;

  bool running;
  // main update loop
  bool step();

  void initalize_window();

  void initialize_simulation(Gtk::DrawingArea *area);
};
