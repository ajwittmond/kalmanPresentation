#include "simulation.h"
#include "gdkmm/general.h"
#include <cmath>
#include <iostream>
#include <numbers>
#include <algorithm>


const double Y_UNITS = 12;
Simulation::Simulation(Gtk::DrawingArea *area, std::shared_ptr<ExtendedKalmanFilter> filter,
           std::shared_ptr<Model<arma::dvec>> model,
           std::vector<Sensor> sensors, double time_step)
    : AreaController(area),
      filter(filter), current_entry{0}, prev_update_time{0},
      model(model), time{0}, time_step{time_step}, sensors{sensors},
      earth("res/earth.png") {
  connect(*area);
  for (int i = 0; i < sensors.size(); i++) {
    sensor_pictures.push_back(Picture("res/radar.png"));
    const arma::dvec &position = sensors[i].position;
    const double angle = (sensors[i].start_angle + sensors[i].end_angle) / 2;
    sensor_pictures[i].set_position(position(0), position(1));
    sensor_pictures[i].set_angle(angle);
    sensor_pictures[i].set_dimensions(0.5, 0.5);
    sensor_activity.push_back(false);
  }
  earth.set_position(0, 0);
  earth.set_dimensions(2, 2);
  earth.set_fill_color(1, 0, 0, 1);
}

void Simulation::restart(){
  time = 0;
  positions.clear();
  covariances.clear();
  filtered_positions.clear();
}

bool angle_between(double angle, double start_angle, double end_angle){
  using namespace std::numbers;
  int start_rep = std::floor(start_angle/(2*pi_v<double>));
  int angle_rep = std::floor(angle/(2*pi_v<double>));
  angle+= 2*pi_v<double> * (start_rep - angle_rep);
  return angle >= start_angle && angle <= end_angle;
}

void Simulation::step(){
  using namespace std::numbers;
  time += time_step;
  arma::dvec state = model->extrapolate(time);

  for (int i =0 ; i< sensors.size(); i++) {
    Sensor &sensor = sensors[i];
    double angle = std::atan2(state(1) - sensor.position(1),
                              state(0) - sensor.position(0));

    // std::cout << "angle " << angle << std::endl;

    if (angle_between(angle, sensor.start_angle, sensor.end_angle) &&
        time - sensor.prev_update_time > sensor.rate){
      filter->set_measurement(sensor.measurement);
      arma::dvec observation = sensor.measurement->measure(time,state);
      arma::dvec noise = arma::sqrtmat_sympd(sensor.measurement->covariance(time))
        * arma::randn(observation.size());
      // std::cout << "noise " << noise;
      filter->update(time,observation +noise );
      current_entry = (current_entry + 1) % positions.size();
      sensor.prev_update_time = time;
      sensor_activity[i] = true;
    }else{
      sensor_activity[i] = false;
    }
  }

  arma::dvec eigen_values; arma::dmat eigen_vectors;//should be orthogonal
  arma::eig_sym(eigen_values,eigen_vectors,filter->covariance(time).submat(0,0,1,1));

  covariances.push_back(std::make_pair(eigen_values,eigen_vectors));
  positions.push_back(state);
  filtered_positions.push_back(filter->filtered_value(time));

  // std::cout << filtered_positions.back() << " " << time << std::endl;

  // std::cout << covariances.back() << " " << time << std::endl;
}


const int VARIANCES_TO_DRAW = 100;

bool Simulation::draw(const Cairo::RefPtr<Cairo::Context> &cr){
  using namespace std::numbers;
  if(!positions.empty()){
    const Gtk::Allocation allocation = area->get_allocation();
    const int w = allocation.get_width(), h = allocation.get_height();
    cr->set_source_rgb(0.5, 0.5, 0.5);
    cr->rectangle(0, 0, w, h);
    cr->fill();

    cr->translate((double)w/2.0, (double)h/2.0);
    double scale = h/Y_UNITS;
    cr->scale(scale,scale);

    earth.draw(cr);

    for(Picture sensor_picture : sensor_pictures){
      sensor_picture.draw(cr);
    }

    for(int i = 0; i < sensor_activity.size() ; i++ ){
      if (sensor_activity[i]){
        cr->begin_new_path();
        cr->set_source_rgb(0,1,0);
        cr->set_line_width(0.025);
        arma::dvec sensor_position = sensors[i].position;
        arma::dvec object_position = positions.back();
        cr->move_to(object_position(0),object_position(1));
        cr->line_to(sensor_position(0), sensor_position(1));
        cr->stroke();
      }
    }

    int start = std::max(0, (int)positions.size() - VARIANCES_TO_DRAW);

    for (int i = start; i < positions.size(); i++) {
      double alpha = 0.5*(1.0 -(double)(positions.size()-i)/(double)VARIANCES_TO_DRAW);

      const arma::dvec &filtered_position = filtered_positions[i];

      const arma::dvec eigen_values = covariances[i].first;
      const arma::dmat eigen_vectors = covariances[i].second;//should be orthogonal
      double angle = std::atan2(eigen_vectors(0,1),eigen_vectors(0,0));
      cr->save();
        cr->begin_new_path();
        cr->translate(filtered_position(0), filtered_position(1));
        cr->set_source_rgba(1, 0, 1, alpha);
        cr->rotate(angle);
        // std::cout << eigen_values << std::endl;
        // std::cout << eigen_vectors << std::endl;
        // std::cout << covariance << std::endl;
        // std::cout << angle << std::endl;
        double sw = 2 * std::sqrt(eigen_values(0)), sh=  2 * std::sqrt(eigen_values(1));
        cr->scale(sw,sh);
        cr->arc(0, 0, 1, 0, 2*pi_v<double>);
        cr->fill_preserve();
        if (i == positions.size() -1){
          cr->set_source_rgb(0,0,0);
          cr->set_line_width(0.025/ std::min(sw,sh));
          cr->stroke();
        }
      cr->restore();
    }

    cr->set_source_rgba(1, 1, 0, 1);
    cr->set_line_width(0.03);
    cr->move_to(positions[0](0), positions[0](1));
    for (int i = 1; i < positions.size(); i++) {
      const arma::dvec &filtered_position = filtered_positions[i];
      cr->line_to(filtered_position(0), filtered_position(1));
    }
    cr->stroke();

    cr->set_source_rgba(0, 1, 0, 1);
    cr->set_line_width(0.03);
    cr->move_to(positions[0](0), positions[0](1));
    for (int i = 1; i < positions.size(); i++) {
      const arma::dvec &position = positions[i];
      cr->line_to(position(0), position(1));
    }
    cr->stroke();

    cr->arc(positions.back()(0), positions.back()(1), 0.05, 0, 2* pi_v<double>);
    cr->close_path();
    cr->stroke();

  }
  drawChildren(cr);
  return true;
}
