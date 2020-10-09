#include "filters.h"


//::::::::::::::::::::::::::::::::::
// Model default integration implementation
//::::::::::::::::::::::::::::::::::
arma::vec Model::extrapolate(int steps, double t){
  if (std::abs(t - current_t) < __DBL_EPSILON__) {
    return current_state;
  }
  current_state = runge_kutta_4(
      steps, current_t, t, current_state,
      [&](double t, arma::vec state){return this->derivative(t, state); });
  this->current_t = t;
  return current_state;
}
//::::::::::::::::::::::::::::::::::


//::::::::::::::::::::::::::::::::::
// Langevin Equation Model implementation
//::::::::::::::::::::::::::::::::::

std::unique_ptr<LinearContinuousModel>
LangevinEquationModel::linear_perturbation_model(double time,
                                                 arma::vec state,
                                                 double rate ) {
   return std::unique_ptr<LinearContinuousModel>(
                        new LinearizedLangevinEquationModel(rate, time, state, clone()));
}
//:::::::::::::::::::::::::::::::::

//::::::::::::::::::::::::::::::::::
//KalmanFilter implementation
//::::::::::::::::::::::::::::::::::

arma::vec KalmanFilter::filtered_value(double t){

}

arma::vec KalmanFilter::update(double t, arma::vec){
  
}
//:::::::::::::::::::::::::::::::::

//::::::::::::::::::::::::::::::::::
// ExtendedKalmanFilter implementation
//::::::::::::::::::::::::::::::::::
//:::::::::::::::::::::::::::::::::

//::::::::::::::::::::::::::::::::::
//  Linear model implementation
//::::::::::::::::::::::::::::::::::

//:::::::::::::::::::::::::::::::::

//::::::::::::::::::::::::::::::::::
//  Linear measurement implementation
//::::::::::::::::::::::::::::::::::
//:::::::::::::::::::::::::::::::::
