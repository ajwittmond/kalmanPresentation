#include "simulationWindow.h"
#include <gtkmm.h>
#include <iostream>

int main(int argc, char **argv) {
  auto app = Gtk::Application::create(argc, argv, "kalman.sim");

  SimulationWindow sim;
  try {
    sim.from_file();
  } catch (Glib::Error e) {
    std::cerr << "failed to load glade file";
  }

  return app->run(sim.getWindow());
}
