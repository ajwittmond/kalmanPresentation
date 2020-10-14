
#define BOOST_STACKTRACE_USE_BACKTRACE T
#include "simulationWindow.h"
#include <boost/stacktrace/stacktrace_fwd.hpp>
#include <gtkmm.h>
#include <iostream>

#include <execinfo.h>

#include <boost/stacktrace.hpp>
#include <stdexcept>
#include <exception>

void handler(int sig) {
  std::cerr << "sigsegv called\n";

  std::cerr << boost::stacktrace::stacktrace();
  exit(1);
}

int main(int argc, char **argv) {
  signal(SIGSEGV, handler);
  std::set_terminate([](){
    std::cerr << "terminate called\n";
    std::cerr << boost::stacktrace::stacktrace();
    std::abort();
  });
  auto app = Gtk::Application::create(argc, argv, "kalman.sim");

  SimulationWindow sim;
  try {
    sim.from_file();
  } catch (Glib::Error e) {
    std::cerr << "failed to load glade file";
    return 1;
  }
  return app->run(sim.getWindow());
}
