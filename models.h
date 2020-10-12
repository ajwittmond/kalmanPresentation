#pragma once

#include "filters.h"

//The gravitational constant for the Earth

const double MU = 3.986004418;

class OrbitalModel : public LangevinEquationModel {
 public:
  OrbitalModel(double rate, double current_t, arma::dvec current_state)
  : LangevinEquationModel{rate,current_t,current_state} {}

  OrbitalModel(OrbitalModel &other)
      : LangevinEquationModel{other.rate, other.current_t, other.current_state} {}

  arma::dmat differential(double time, arma::dvec state) override;

  arma::dmat noise_matrix(double t) override;

  arma::dvec derivative(double t, arma::dvec state) override;

  std::shared_ptr<LangevinEquationModel> clone() override;
};

class RangeMeasurement : public LinearizeableMeasurement {
public:
  arma::dmat differential(double t, arma::dvec) override;

  std::shared_ptr<LinearizeableMeasurement> clone() override;

  arma::dvec measure(double time, arma::dvec state) override;

  arma::dmat covariance(double time) override ;


};

class RangeRateMeasurement : public LinearizeableMeasurement {
public:
  arma::dmat differential(double t, arma::dvec) override;

  std::shared_ptr<LinearizeableMeasurement> clone() override;

  arma::dvec measure(double time, arma::dvec state) override;

  arma::dmat covariance(double time) override;
};

class AnglesMeasurement : public LinearizeableMeasurement {
public:
  arma::dmat differential(double t, arma::dvec) override;

  std::shared_ptr<LinearizeableMeasurement> clone() override;

  arma::dvec measure(double time, arma::dvec state) override;

  arma::dmat covariance(double time) override;
};
