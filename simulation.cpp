#include "simulation.h"
#include <iostream>
#include <numbers>
#include <algorithm>


const double Y_UNITS = 30;

void Simulation::restart(){
  time = 0;
  positions.clear();
  covariances.clear();
  filtered_positions.clear();
}

void Simulation::step(){
  time += time_step;
  positions.push_back(model->extrapolate(time));
  //std::cout << model->extrapolate(time) << " " << time << std::endl;
  filtered_positions.push_back(filter->filtered_value(time));
  covariances.push_back(filter->covariance(time));
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
      const arma::dmat &covariance = covariances[i];
      const arma::dvec &filtered_position = filtered_positions[i];
      cr->save();
        cr->translate(filtered_position(0), filtered_position(1));
        cr->set_source_rgba(1, 0, 1, alpha);
        cr->scale(2 * std::sqrt(covariance(0, 0)), 2 * std::sqrt(covariance(1, 1)));
        cr->arc(0, 0, 1, 0, 2*pi_v<double>);
        cr->fill();
      cr->restore();
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
