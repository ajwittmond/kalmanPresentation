#pragma once

#include "filters.h"

//The gravitational constant for the Earth

const double MU = 3.986004418;

/**
 *  This describes the model for a particle orbiting a fixed point mass.
 */
class OrbitalModel : public LangevinEquationModel {
 public:
  /**
   *  Initializes the model
   *  @param rate the sampling rate for integration in samples per second
   *  @param current_t the starting time
   *  @param current_state the initial state
   */
  OrbitalModel(double rate, double current_t, arma::dvec current_state)
  : LangevinEquationModel{rate,current_t,current_state} {}

  OrbitalModel(OrbitalModel &other)
      : LangevinEquationModel{other.rate, other.current_t, other.current_state} {}

  arma::dmat differential(double time, arma::dvec state) override;

  /**
   * Zero in this case since this is a deterministic model.
   */
  arma::dmat noise_matrix(double t) override;

  arma::dvec derivative(double t, arma::dvec state) override;

  std::shared_ptr<LangevinEquationModel> clone() override;
};

/**
 *  The is the range measurement from a sensor with a fixed location.
 */
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


/**
 *  This is a range rate measurement from a fixed sensor.
 */
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


/**
 *  This measures the angle cosine between an object and a fixed sensor
 */
class AnglesMeasurement : public LinearizeableMeasurement {
private:
  arma::vec position;

  arma::dmat covariance_mat;

  arma::vec plane;

public:
  AnglesMeasurement(arma::vec position, arma::dmat covariance_mat)
    : position{position}, covariance_mat{covariance_mat},plane(2) {
    plane = arma::normalise(position);
  }

  AnglesMeasurement(AnglesMeasurement &other)  = default;

  arma::dmat differential(double t, arma::dvec) override;

  std::shared_ptr<LinearizeableMeasurement> clone() override;

  arma::dvec measure(double time, arma::dvec state) override;

  arma::dmat covariance(double time) override{
    return covariance_mat;
  }
};
