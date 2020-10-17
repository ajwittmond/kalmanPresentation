#pragma once

#include "drawable.h"
#include "filters.h"
#include "gtkmm/drawingarea.h"
#include <memory>
#include <vector>

struct Sensor{
  std::shared_ptr<LinearizeableMeasurement> measurement;
  double rate;
  double start_angle, end_angle;
  arma::dvec position;
  double prev_update_time;
};


class Simulation : public Drawable, public AreaController {
  std::vector<arma::dvec> positions;
  std::vector<std::pair<arma::dvec,arma::dmat>> covariances;
  std::vector<arma::dvec> filtered_positions;

  std::shared_ptr<ExtendedKalmanFilter> filter;
  std::shared_ptr<Model<arma::dvec>> model;
  double time_step;

  double time;
  double prev_update_time;

  int current_entry;

  std::vector<Sensor> sensors;
  std::vector<Picture> sensor_pictures;
  std::vector<bool> sensor_activity;
  Picture earth;

public :
  Simulation(Gtk::DrawingArea *area,
             std::shared_ptr<ExtendedKalmanFilter> filter,
             std::shared_ptr<Model<arma::dvec>> model,
             std::vector<Sensor> sensors,
             double time_step);

  void restart();

  void step();

  bool draw(const Cairo::RefPtr<Cairo::Context> &cr) override;
};
