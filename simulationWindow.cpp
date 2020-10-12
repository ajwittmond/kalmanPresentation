#include "simulationWindow.h"

void SimulationWindow::from_file() throw() {
  builder = Gtk::Builder::create();
  builder->add_from_file("simulation.glade");
  Gtk::DrawingArea *area;
  builder->get_widget("drawing_area",area);
  builder->get_widget("window", this->window);
  window->set_events(Gdk::ALL_EVENTS_MASK);
  //the timing for the main loop is set here
  timout =
      Glib::signal_timeout().connect(sigc::mem_fun(*this, &IsingWindow::step),
                                     16 /** 60 fps */);
}

Gtk::Window& IsingWindow::getWindow(){
  return *window;
}

//main update loop
bool IsingWindow::step() {
  //time keeping logic
  static std::vector<double> samples(128);
  static auto prev_time = std::chrono::system_clock::now();
  static auto curr_time = std::chrono::system_clock::now();
  static bool first = true;
  static double timer  = 0;
  const int INTERVAL = 1;
  double dt = 0;
  static double time =0; 
  //time keeping and reporting
  if (first){
    first = false;
  }else{
    curr_time = std::chrono::system_clock::now();
    dt = std::chrono::duration<double>(curr_time - prev_time).count();
    samples.push_back(dt);
    timer += dt;
    //print estimates of the mean and standard deviations of the samples
    if(timer >= (double)INTERVAL){
      timer = 0;
      double mean=0,variance =0;
      for(double x: samples){
        mean+= x;
      }
      mean /= samples.size();
      // n-1 estimator is the unbiased variance estimator
      for(double x: samples){
        variance += pow(mean - x,2);
      }
      variance/= samples.size()-1;
      std::cout << "for the last " << INTERVAL << " second(s)..." << std::endl;
      std::cout << "mean time: " << mean << std::endl;
      std::cout << "standard deviation: " << sqrt(variance) << std::endl;
      samples.clear();
    }
    prev_time = curr_time;
  }

  if(running->get_active()){
    //update logic
    static double step_timer = 0;
    step_timer+=dt;
    time+=dt;

    if (step_timer > 1.0 / *step_frequency) {
      step_timer -= 1.0 / *step_frequency;
      // model->step();
      // model->invalidate_rect();
    }
  }
  return true;
}
