#pragma once

#include "drawable.h"
#include "filters.h"
#include "gtkmm/drawingarea.h"
#include <memory>
#include <vector>

class Simulation : public Drawable, public AreaController{
  std::vector<arma::dvec> positions;
  std::vector<arma::dmat> covariances;
  std::vector<arma::dvec> filtered_positions;

  std::shared_ptr<Filter> filter;
  std::shared_ptr<Model<arma::dvec>> model;
  double time_step;

  double time;

  std::vector<double> update_schedule;

public :

  Simulation(Gtk::DrawingArea* area,
             std::shared_ptr<Filter> filter,
             std::shared_ptr<Model<arma::dvec>> model,
             double time_step)
    : AreaController(area),filter(filter), model(model), time{0}, time_step{time_step} {
    connect(*area);
  }

  void restart();

  void step();

  bool draw(const Cairo::RefPtr<Cairo::Context> &cr) override;

};
