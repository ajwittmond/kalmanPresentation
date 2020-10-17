#include <cassert>
#include <iostream>
#include <chrono>
#include <memory>
#include "filters.h"
#include "gtkmm/drawingarea.h"
#include "models.h"
#include "simulation.h"
#include "simulationWindow.h"
#include <numbers>
#include "ini_parser.h"
#include <sstream>

const double SAMPLE_RATE = 1000;
const int BUFFER_SIZE = 500;


arma::dvec parse_vec(const std::string vec_string){
  std::stringstream stream(vec_string);
  std::vector<double> entries;
  double x;
  char delim;
  do{
    stream >> x;
    if(!stream.fail()){
      entries.push_back(x);
    }
    stream >> delim;
  }while(!stream.bad() && !stream.eof());
  return arma::dvec(entries);
};

std::vector<Sensor> parse_sensors(ParsedIniFile & configFile){
  using namespace std::numbers;
  std::vector<Sensor> out;
  for(auto sensor_config : configFile["sensor"]){
    Sensor sensor;
    sensor.position = parse_vec(sensor_config["position"]);
    sensor.start_angle = 2*pi_v<double>*parse_vec(sensor_config["start_angle"])(0)/360.0;
    sensor.end_angle = 2*pi_v<double>*parse_vec(sensor_config["end_angle"])(0)/360.0;
    sensor.rate = parse_vec(sensor_config["rate"])(0);
    arma::dvec measurement_noise = parse_vec(sensor_config["measurement_noise"]);
    std::vector<std::shared_ptr<LinearizeableMeasurement>> measurements;
    std::stringstream names(sensor_config["measurement"]);
    int i = 0;
    do{
      std::string name;
      std::getline(names,name,';');
      if(name == "Range"){
        measurements.push_back(std::shared_ptr<LinearizeableMeasurement>
                               (new RangeMeasurement
                                (sensor.position,arma::dmat{measurement_noise(i)} )
                                ));
      }else if(name == "RangeRate"){
        measurements.push_back(std::shared_ptr<LinearizeableMeasurement>
                               (new RangeRateMeasurement
                                (sensor.position,arma::dmat{measurement_noise(i)} )
                                ));
      }else if(name == "Angle"){
        measurements.push_back(std::shared_ptr<LinearizeableMeasurement>
                               (new AnglesMeasurement
                                (sensor.position,arma::dmat{measurement_noise(i)} )
                                ));
      }
      i++;
    }while(!names.eof());
    sensor.measurement = std::unique_ptr<LinearizeableMeasurement>
      (new CompositeLinearizeableMeasurement(measurements));
    sensor.prev_update_time = 0;
    out.push_back(sensor);
  }
  return out;
}

void SimulationWindow::initialize_simulation(Gtk::DrawingArea *area) {
  using namespace std::numbers;

  ParsedIniFile configFile = parse_ini_file("./options.ini",BUFFER_SIZE);

  arma::dvec initial_state =
    parse_vec(configFile["initial_conditions"][0]["initial_vector"]);
  std::shared_ptr<LangevinEquationModel> model(new OrbitalModel(SAMPLE_RATE,0,initial_state));

  arma::dvec state_prior =
      parse_vec(configFile["initial_conditions"][0]["filter_initial_vector"]);
  arma::dvec variance_vec =
      parse_vec(configFile["initial_conditions"][0]["prior_covariance"]);

  arma::dmat prior_covariance = arma::diagmat(variance_vec);

  std::shared_ptr<LangevinEquationModel> filter_model(
      new OrbitalModel(SAMPLE_RATE, 0, state_prior));

  std::vector<Sensor> sensors = parse_sensors(configFile);

  assert(sensors.size()>0);
  ExtendedKalmanFilter *ekf = new ExtendedKalmanFilter(
        SAMPLE_RATE, state_prior, prior_covariance, filter_model, sensors[0].measurement);

  std::shared_ptr<ExtendedKalmanFilter>
      filter(ekf);


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
