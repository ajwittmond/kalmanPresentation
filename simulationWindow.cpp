#include <iostream>
#include <chrono>
#include <memory>
#include "filters.h"
#include "gtkmm/drawingarea.h"
#include "models.h"
#include "simulation.h"
#include "simulationWindow.h"
#include <numbers>

const double SAMPLE_RATE = 1000;

std::shared_ptr<LinearizeableMeasurement> get_total_measurement(arma::dvec position){
  std::vector<std::shared_ptr<LinearizeableMeasurement>> measurements{
    std::shared_ptr<LinearizeableMeasurement>(
                    new RangeMeasurement(position,arma::dmat{arma::dvec{0.001}})),
    std::shared_ptr<LinearizeableMeasurement>(
                    new RangeRateMeasurement(position,arma::dmat{arma::dvec{0.2}})),
    std::shared_ptr<LinearizeableMeasurement>(
                    new AnglesMeasurement(position,arma::dmat{arma::dvec{0.2}})),
  };

  // return std::shared_ptr<LinearizeableMeasurement>(new CompositeLinearizeableMeasurement(measurements));
  return measurements[0];
}

void SimulationWindow::initialize_simulation(Gtk::DrawingArea *area) {
  using namespace std::numbers;
  arma::dvec initial_state{0,4,1,0};
  std::shared_ptr<LangevinEquationModel> model(new OrbitalModel(SAMPLE_RATE,0,initial_state));

  arma::dvec state_prior{0.001,4.03,0.98,0.1};
  arma::dmat prior_covariance{{0.1,0,0,0},{0,0.1,0,0},{0,0,0.1,0},{0,0,0,0.1}};

  std::shared_ptr<LangevinEquationModel> filter_model(
      new OrbitalModel(SAMPLE_RATE, 0, state_prior));

  std::shared_ptr measurement1 = get_total_measurement(arma::dvec{0,1});
  std::shared_ptr measurement2 = get_total_measurement(arma::dvec{0,-1});

  ExtendedKalmanFilter *ekf = new ExtendedKalmanFilter(
        SAMPLE_RATE, state_prior, prior_covariance, filter_model, measurement1);

  std::shared_ptr<ExtendedKalmanFilter>
      filter(ekf);

  std::vector<Sensor> sensors{
    Sensor{measurement1,1.0/32,0,pi_v<double>,arma::dvec{0,1},0},
    Sensor{measurement1,1.0/32,pi_v<double>,2*pi_v<double>,arma::dvec{0,-1},0}
  };

  this->simulation = std::unique_ptr<Simulation>(new Simulation(
      area, filter, std::dynamic_pointer_cast<Model<arma::dvec>>(model),
      sensors, 1.0 / STEP_FREQUENCY));
}

void SimulationWindow::from_file() throw() {
  builder = Gtk::Builder::create();
  builder->add_from_file("simulation.glade");
  Gtk::DrawingArea *area;
  builder->get_widget("drawing_area",area);
  builder->get_widget("window", this->window);
  window->set_events(Gdk::ALL_EVENTS_MASK);
  //the timing for the main loop is set here
  timout =
      Glib::signal_timeout().connect(sigc::mem_fun(*this, &SimulationWindow::step),
                                     16 /** 60 fps */);

  initialize_simulation(area);
}

Gtk::Window& SimulationWindow::getWindow(){
  return *window;
}

//main update loop
bool SimulationWindow::step() {

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

  if(running){
    //update logic
    static double step_timer = 0;
    step_timer+=dt;
    time+=dt;

    if (step_timer > 1.0 / STEP_FREQUENCY) {
      step_timer -= 1.0 / STEP_FREQUENCY;
      simulation->step();
      simulation->invalidate_rect();
    }
  }
  
  return true;
}
