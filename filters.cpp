#include "filters.h"
#include <algorithm>


//::::::::::::::::::::::::::::::::::
// Langevin Equation Model implementation
//::::::::::::::::::::::::::::::::::

std::shared_ptr<LinearContinuousModel>
LangevinEquationModel::linear_perturbation_model(double time,
                                                 arma::dvec state,
                                                 double rate ) {
   return std::shared_ptr<LinearContinuousModel>(
                        new LinearizedLangevinEquationModel(rate, time, state, clone()));
}
//:::::::::::::::::::::::::::::::::

//::::::::::::::::::::::::::::::::::
//KalmanFilter implementation
//::::::::::::::::::::::::::::::::::


arma::dvec KalmanFilter::update(double t, arma::vec observation){

  const arma::dmat M = measurement->measurement_matrix(t),
                  P = matrix_model->extrapolate( t),
                  R = measurement->measurement_matrix(t);

  // the Kalman gain
  const arma::dmat K = P * arma::trans(M) * arma::inv(M * P * trans(M) + R);

  //predict
  const arma::dvec old_state = model->extrapolate( t );
  const arma::dmat old_covariance = matrix_model->extrapolate( t );

  //adjust 

  //correct with residual multiplied by Kalman Gain
  const arma::dvec new_state = old_state + K*(observation - M*old_state);
  //adjust covariance with the change in uncertainty caused by observation
  const arma::dmat new_covariance = old_covariance - K*M*old_covariance;

  //now integrate from new initial estidmates
  model->set_initial_conditions(t , new_state );
  matrix_model->set_initial_conditions(t , new_covariance);

  return new_state;
}
//:::::::::::::::::::::::::::::::::

//::::::::::::::::::::::::::::::::::
// ExtendedKalmanFilter implementation
//::::::::::::::::::::::::::::::::::

arma::dvec ExtendedKalmanFilter::filtered_value(double t) {
  return perturbationProcessFilter.filtered_value(t ) + model->extrapolate(t );
}

arma::dmat ExtendedKalmanFilter::covariance(double t) {
  return perturbationProcessFilter.covariance(t );
}

arma::dvec ExtendedKalmanFilter::update(double t, arma::vec observation) {
  arma::dvec nominal_state = model->extrapolate(t );

  arma::dvec measurement_error = observation - measurement->measure(t , nominal_state);

  arma::dvec error = perturbationProcessFilter.update(t, measurement_error);

  arma::dvec state_estidmate = nominal_state + error;

  model->set_initial_conditions(t , state_estidmate);

  return state_estidmate;
}

//:::::::::::::::::::::::::::::::::

//:::::::::::::::::::::::::::::::::
// matrix amalgamation functions
//:::::::::::::::::::::::::::::::::
arma::dmat block_diagonal(std::vector<arma::dmat> diagonals) {
  int dim = 0;
  for (arma::dmat m : diagonals) {
    dim += m.n_rows;
  }
  int n = 0;
  arma::dmat out(dim, dim, arma::fill::zeros);
  for (arma::dvec mat : diagonals) {
    int s = mat.n_cols;
    out.submat(n, n, n + s, n + s) = mat;
    n += s;
  }
  return out;
}

arma::dvec block_vector(std::vector<arma::dvec> vecs){
  int dim = 0;
  for (arma::dvec vec: vecs) {
    dim += vec.size();
  }
  int n = 0;
  arma::dvec out(dim);
  for (arma::dvec vec : vecs) {
    int s = vec.size();
    out.subvec(n, n + s - 1) = vec;
    n += s;
  }
  return out;
}
//::::::::::::::::::::::::::::::::::



//::::::::::::::::::::::::::::::::
// CompositeLinearizeableMeasurement implementation
//::::::::::::::::::::::::::::::::


arma::dvec CompositeLinearizeableMeasurement::measure(double time, arma::dvec state) {
  std::vector<arma::dvec> measures(measurements.size());
  for (int i = 0; i < measurements.size(); ++i) {
    measures[i] = measurements[i]->measure(time, state);
  }
  return block_vector(measures);
}

arma::dmat CompositeLinearizeableMeasurement::covariance(double time) {
  std::vector<arma::dmat> covariances(measurements.size());
  for (int i = 0; i < measurements.size(); ++i) {
    covariances[i] = measurements[i]->covariance(time);
  }
  return block_diagonal(covariances);
}

arma::dmat CompositeLinearizeableMeasurement::differential(double t,
                                                           arma::dvec state) {
  std::vector<arma::dmat> differentials(measurements.size());
  for (int i = 0; i < measurements.size(); ++i) {
    differentials[i] = measurements[i]->differential(t , state);
  }
  return block_diagonal(differentials);
}

std::unique_ptr<LinearMeasurement>
CompositeLinearizeableMeasurement::linearize(std::shared_ptr<LangevinEquationModel> langevin_model) {
  std::vector<std::shared_ptr<LinearMeasurement>> linearized;
  for(auto measurement : measurements){
    linearized.push_back(measurement->linearize(langevin_model));
  }
  return std::unique_ptr<LinearMeasurement>(new CompositeLinearMeasurement(linearized));
}

//:::::::::::::::::::::::::::::::


//:::::::::::::::::::::::::::::::
// CompositeLinearMeasurement implementation
//:::::::::::::::::::::::::::::::


arma::dmat CompositeLinearMeasurement::covariance(double time) {
  std::vector<arma::dmat> covariances(measurements.size());
  for (int i = 0; i < measurements.size(); ++i) {
    covariances.push_back(measurements[i]->covariance(time));
  }
  return block_diagonal(covariances);
}

arma::dmat CompositeLinearMeasurement::measurement_matrix(double t) {
  std::vector<arma::dmat> differentials(measurements.size());
  for (int i = 0; i < measurements.size(); ++i) {
    differentials.push_back(measurements[i]->measurement_matrix(t));
  }
  return block_diagonal(differentials);
}
//:::::::::::::::::::::::::::::::


//:::::::::::::::::::::::::::::::::::::
// LinearizeableMeasurement implementation
//:::::::::::::::::::::::::::::::::::::
std::unique_ptr<LinearMeasurement>
LinearizeableMeasurement::linearize(std::shared_ptr<LangevinEquationModel> langevin_model) {
  return std::unique_ptr<LinearMeasurement>(new LinearizedMeasurement(this->clone(), langevin_model));
}
//:::::::::::::::::::::::::::::::::::::
