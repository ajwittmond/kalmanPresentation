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

arma::dmat RangeMeasurement::differential(double t, arma::dvec state) {
  arma::dvec rvec = state.subvec(0,1) - position;
  double r = arma::norm(rvec);
  return arma::dmat{{
      rvec(0) / r, rvec(1) / r, 0 , 0
  }};
}

std::shared_ptr<LinearizeableMeasurement> RangeMeasurement::clone() {
  return std::shared_ptr<LinearizeableMeasurement>(new RangeMeasurement(*this));
}

arma::dvec RangeMeasurement::measure(double time, arma::dvec state) {
  return arma::dvec { arma::norm(state.subvec(0,1) - position) };
}


//::::::::::::::::::::::::::::::::::::::

//::::::::::::::::::::::::::::::::::::::
// RangeMeasurement implementation
//::::::::::::::::::::::::::::::::::::::

arma::dmat RangeRateMeasurement::differential(double t, arma::dvec state) {
  arma::dvec rvec = state.subvec(0, 1) - position;
  arma::dvec v = state.subvec(2,3);
  double r_inv = 1/arma::norm(rvec);
  double r_inv3 = std::pow(r_inv,3);
  double dx =
      r_inv * (v(0) + rvec(1) * v(1)) -
    r_inv3 * rvec(0)*(v(0) * rvec(0) + rvec(1) * v(1));
  double dy =
    r_inv * (v(0)*rvec(0) + v(1)) -
              r_inv3 * rvec(1)*(v(0) * rvec(0) + rvec(1) * v(1));
  double ddx = r_inv * (rvec(0) + rvec(1) * v(1));
  double ddy = r_inv * (rvec(0) * v(0) + rvec(1));
  return arma::dmat{{dx, dy, ddx, ddy}};
}

std::shared_ptr<LinearizeableMeasurement> RangeRateMeasurement::clone() {
  return std::shared_ptr<LinearizeableMeasurement>(new RangeRateMeasurement(*this));
}

arma::dvec RangeRateMeasurement::measure(double time, arma::dvec state) {
  arma::dvec rvec = state.subvec(0, 1) - position;
  arma::dvec v = state.subvec(2, 3);
  double r_inv = 1 / arma::norm(rvec);
  return arma::dvec{r_inv * (rvec(0) *v(0) + rvec(1) * v(1))};
}

//::::::::::::::::::::::::::::::::::::::

//::::::::::::::::::::::::::::::::::::::
// RangeMeasurement implementation
//::::::::::::::::::::::::::::::::::::::

arma::dmat AnglesMeasurement::differential(double t, arma::dvec state) {
  return arma::dvec{ plane(0), plane(1), 0 ,0};
}

std::shared_ptr<LinearizeableMeasurement> AnglesMeasurement::clone() {
  return std::shared_ptr<LinearizeableMeasurement>(new AnglesMeasurement(*this));
}

arma::dvec AnglesMeasurement::measure(double time, arma::dvec state){
  return arma::dvec{arma::dot(plane, state.subvec(0, 1))};
}

//::::::::::::::::::::::::::::::::::::::
