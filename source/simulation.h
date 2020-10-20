#pragma once

#include "drawable.h"
#include "filters.h"
#include "gtkmm/drawingarea.h"
#include <memory>
#include <vector>

/**
 *  Representation of a radar sensor
 */
struct Sensor{
  ///
  ///the measurement used
  std::shared_ptr<LinearizeableMeasurement> measurement;
  ///
  ///the rate at which the measurements are made
  double rate;
  ///
  /// The min and max angle the measurements are taken at
  double start_angle, end_angle;
  ///
  /// The sensor's position
  arma::dvec position;
  ///
  /// The time of the last measurement
  double prev_update_time;
};

/**
 *  This class handles drawing and stepping the simulation
 */
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

  /***
   * restart the simulation
   */
  void restart();

  /**
   * step the simulation
   */
  void step();

  bool draw(const Cairo::RefPtr<Cairo::Context> &cr) override;
};
