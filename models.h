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
  arma::dvec position;
  arma::dmat covariance_mat;
public:
  RangeMeasurement(arma::dvec position,arma::dmat covariance_mat)
    : position{position}, covariance_mat{covariance_mat}{}

  RangeMeasurement(RangeMeasurement & other) = default;

  arma::dmat differential(double t, arma::dvec) override;

  std::shared_ptr<LinearizeableMeasurement> clone() override;

  arma::dvec measure(double time, arma::dvec state) override;

  arma::dmat covariance(double time) override{
    return covariance_mat;
  }
};

class RangeRateMeasurement : public LinearizeableMeasurement {
private:
  arma::vec position;

  arma::dmat covariance_mat;

public:
  RangeRateMeasurement(arma::vec position, arma::dmat covariance_mat)
      : position{position}, covariance_mat{covariance_mat} {
  }

  RangeRateMeasurement(RangeRateMeasurement &other) = default;

  arma::dmat differential(double t, arma::dvec) override;

  std::shared_ptr<LinearizeableMeasurement> clone() override;

  arma::dvec measure(double time, arma::dvec state) override;

  arma::dmat covariance(double time) override { return covariance_mat; }
};

class AnglesMeasurement : public LinearizeableMeasurement {
private:
  arma::vec position;

  arma::dmat covariance_mat;

  arma::vec plane;

public:
  AnglesMeasurement(arma::vec position, arma::dmat covariance_mat)
    : position{position}, covariance_mat{covariance_mat} {
    arma::dvec normal = position / arma::norm(position);
    plane(0) = -normal(1);
    plane(1) = normal(0);
  }

  AnglesMeasurement(AnglesMeasurement &other)  = default;

  arma::dmat differential(double t, arma::dvec) override;

  std::shared_ptr<LinearizeableMeasurement> clone() override;

  arma::dvec measure(double time, arma::dvec state) override;

  arma::dmat covariance(double time) override{
    return covariance_mat;
  }
};
