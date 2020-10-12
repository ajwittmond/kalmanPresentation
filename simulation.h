#pragma once

#include "drawable.h"
#include "filters.h"
#include <memory>
#include <vector>

class Simulation : public Drawable, public AreaController{

public :

  std::unique_ptr<Filter> filter;
  std::unique_ptr<Model> model;
  double time_step;

  void step();

  bool draw(const Cairo::RefPtr<Cairo::Context> &cr) override;

};
