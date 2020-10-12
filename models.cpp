#include "models.h"
#include "filters.h"


//::::::::::::::::::::::::::::::::
// OrbitalModel implementation
//::::::::::::::::::::::::::::::::
// TODO: add better comments


arma::dmat OrbitalModel::differential(double t, arma::vec state) {
  arma::dvec position = state.subvec(0, 1);
  arma::dvec velocity = state.subvec(2, 3);
  double x1 = position(0);
  double x2 = position(1);
  double r = arma::norm(position);
  double r3 = std::pow(r, 3);
  return arma::dmat{
        {0, 0, 1, 0},
        {0, 0, 0, 1},
        {(-MU * (1 / r) - (x1 * x1 / r3)), -MU * (x1 * x2 / r3)},
        {-MU *(x1 * x2 / r3), -MU * ((1/r) - (x2*x2/r3))} 
  };
}


//this is a deterministic model
arma::dmat OrbitalModel::noise_matrix(double t) {
  return arma::dmat(4,4,arma::fill::zeros);
}

arma::dvec OrbitalModel::derivative(double t, arma::dvec state) {
  arma::dvec position = state.subvec(0,1);
  arma::dvec velocity = state.subvec(2,3);
  arma::dvec acceleration = -MU * arma::norm(position) * position;
  return arma::join_rows(velocity,acceleration);
}

std::shared_ptr<LangevinEquationModel> OrbitalModel::clone() {
  return std::shared_ptr<LangevinEquationModel>(new OrbitalModel(*this));
}

//::::::::::::::::::::::::::::::::


//::::::::::::::::::::::::::::::::::::::
// RangeMeasurement implementation
//::::::::::::::::::::::::::::::::::::::

arma::dmat RangeMeasurement::differential(double t, arma::dvec) {}

std::shared_ptr<LinearizeableMeasurement> RangeMeasurement::clone() {}

arma::dvec RangeMeasurement::measure(double time, arma::dvec state) {}

arma::dmat RangeMeasurement::covariance(double time) {}
//::::::::::::::::::::::::::::::::::::::

//::::::::::::::::::::::::::::::::::::::
// RangeMeasurement implementation
//::::::::::::::::::::::::::::::::::::::

arma::dmat RangeRateMeasurement::differential(double t, arma::dvec) {}

std::shared_ptr<LinearizeableMeasurement> RangeRateMeasurement::clone() {}

arma::dvec RangeRateMeasurement::measure(double time, arma::dvec state) {}

arma::dmat RangeRateMeasurement::covariance(double time) {}
//::::::::::::::::::::::::::::::::::::::

//::::::::::::::::::::::::::::::::::::::
// RangeMeasurement implementation
//::::::::::::::::::::::::::::::::::::::

arma::dmat AnglesMeasurement::differential(double t, arma::dvec) {}

std::shared_ptr<LinearizeableMeasurement> AnglesMeasurement::clone() {}

arma::dvec AnglesMeasurement::measure(double time, arma::dvec state) {}

arma::dmat AnglesMeasurement::covariance(double time) {}
//::::::::::::::::::::::::::::::::::::::
