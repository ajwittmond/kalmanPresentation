#include "simulation.h"
#include <cmath>
#include <iostream>
#include <numbers>
#include <algorithm>


const double Y_UNITS = 20;

void Simulation::restart(){
  time = 0;
  positions.clear();
  covariances.clear();
  filtered_positions.clear();
}

void Simulation::step(){
  using namespace std::numbers;
  time += time_step;
  arma::dvec state = model->extrapolate(time);

  for (Sensor &sensor : sensors) {

    double angle = std::atan2(state(1) - sensor.position(1),
                              state(0) - sensor.position(0)) + pi_v<double>;

    // std::cout << "angle " << angle << std::endl;

    if (angle <= sensor.end_angle && angle >= sensor.start_angle &&
        time - sensor.prev_update_time > sensor.rate){
      filter->set_measurement(sensor.measurement);
      arma::dvec observation = sensor.measurement->measure(time,state);
      arma::dvec noise = arma::sqrtmat_sympd(sensor.measurement->covariance(time))
        * arma::randn(observation.size());
      // std::cout << "noise " << noise;
      filter->update(time,observation +noise );
      current_entry = (current_entry + 1) % positions.size();
      sensor.prev_update_time = time;
    }
  }

  positions.push_back(state);
  filtered_positions.push_back(filter->filtered_value(time));
  covariances.push_back(filter->covariance(time));

  // std::cout << filtered_positions.back() << " " << time << std::endl;

  // std::cout << covariances.back() << " " << time << std::endl;
}


const int VARIANCES_TO_DRAW = 200;

bool Simulation::draw(const Cairo::RefPtr<Cairo::Context> &cr){
  using namespace std::numbers;
  if(!positions.empty()){
    const Gtk::Allocation allocation = area->get_allocation();
    const int w = allocation.get_width(), h = allocation.get_height();
    cr->set_source_rgb(1, 1, 1);
    cr->rectangle(0, 0, w, h);
    cr->fill();

    cr->translate((double)w/2.0, (double)h/2.0);
    double scale = h/Y_UNITS;
    cr->scale(scale,scale);

    cr->set_source_rgb(0, 0, 1);
    cr->begin_new_path();
    cr->arc(0,0,1,0,2 * pi_v<double>);
    cr->close_path();
    cr->fill();
    int start = std::max(0, (int)positions.size() - VARIANCES_TO_DRAW);

    for (int i = start; i < positions.size(); i++) {
      double alpha = 0.1*(1.0 -(double)(positions.size()-i)/(double)VARIANCES_TO_DRAW);
      const arma::dmat &covariance = covariances[i].submat(0,0,1,1);
      const arma::dvec &filtered_position = filtered_positions[i];
      arma::dvec eigen_values;
      arma::dmat eigen_vectors;//should be orthogonal
      arma::eig_sym(eigen_values,eigen_vectors,covariance);
      double angle = std::atan2(eigen_vectors(0,1),eigen_vectors(1,0));
      if(covariance(0,0) > 0 && covariance(1,1)>0){
        cr->save();
          cr->begin_new_path();
          cr->translate(filtered_position(0), filtered_position(1));
          cr->set_source_rgba(1, 0, 1, alpha);
          cr->rotate(angle);
          // std::cout << eigen_values << std::endl;
          // std::cout << eigen_vectors << std::endl;
          // std::cout << covariance << std::endl;
          // std::cout << angle << std::endl;
          cr->scale(2 * std::sqrt(eigen_values(0)), 2 * std::sqrt(eigen_values(1)));
          cr->arc(0, 0, 1, 0, 2*pi_v<double>);
          cr->fill_preserve();
          if (i == positions.size() -1){
            cr->set_source_rgb(0,0,0);
          }
          cr->set_line_width(0.025);
          cr->stroke();
        cr->restore();
      }
    }

    cr->set_source_rgba(1, 1, 0, 1);
    cr->set_line_width(0.05);
    cr->move_to(positions[0](0), positions[0](1));
    for (int i = 1; i < positions.size(); i++) {
      const arma::dvec &filtered_position = filtered_positions[i];
      cr->line_to(filtered_position(0), filtered_position(1));
    }
    cr->stroke();

    cr->set_source_rgba(0, 1, 0, 1);
    cr->set_line_width(0.05);
    cr->move_to(positions[0](0), positions[0](1));
    for (int i = 1; i < positions.size(); i++) {
      const arma::dvec &position = positions[i];
      cr->line_to(position(0), position(1));
    }
    cr->stroke();

  }
  drawChildren(cr);
  return true;
}
