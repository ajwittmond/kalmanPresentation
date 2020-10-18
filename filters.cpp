#include "filters.h"
#include <algorithm>
#include <assert.h>

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
    R = measurement->covariance(t);

  // the Kalman gain
  const arma::dmat K = P * arma::trans(M) * arma::inv(M * P * arma::trans(M) + R);

  // std::cout << "K " << K << std::endl << "KM "<< K*M << std::endl;
  //predict
  const arma::dvec old_state = model->extrapolate( t );
  //adjust

  const arma::dmat KM = K*M;
  const arma::dmat I_KM = arma::eye(KM.n_rows,KM.n_cols) - KM;
  //correct with residual multiplied by Kalman Gain
  const arma::dvec new_state = old_state + K*(observation - M*old_state);
  //adjust covariance with the change in uncertainty caused by observation
  const arma::dmat new_covariance = I_KM*P*arma::trans(I_KM) + K*R*arma::trans(K);

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
  return  model->extrapolate(t );
}

arma::dmat ExtendedKalmanFilter::covariance(double t) {
  return perturbationProcessFilter.covariance(t );
}

arma::dvec ExtendedKalmanFilter::update(double t, arma::vec observation) {
  arma::dvec nominal_state = model->extrapolate(t );

  arma::dvec measurement_error = observation - measurement->measure(t , nominal_state);

  arma::dvec error = perturbationProcessFilter.update(t, measurement_error);

  arma::dvec state_estimate = nominal_state + error;

  model->set_initial_conditions(t , state_estimate);

  perturbationProcessFilter.get_model()->set_initial_conditions
    (t,arma::dvec(nominal_state.size(),arma::fill::zeros));


  return state_estimate;
}

//:::::::::::::::::::::::::::::::::

//:::::::::::::::::::::::::::::::::
// matrix amalgamation functions
//:::::::::::::::::::::::::::::::::
arma::dmat join_rows(std::vector<arma::dmat> mats) {
  assert(!mats.empty());
  int rows = 0;
  int cols = mats[0].n_cols;
  for (arma::dmat m : mats) {
    rows += m.n_rows;
    assert(m.n_cols == cols);
  }
  int row = 0;
  arma::dmat out(rows, cols, arma::fill::zeros);
  for (arma::dmat mat : mats) {
    int s = mat.n_rows;
    out.submat(row, 0, row+s - 1, cols - 1) = mat;
    row += s;
  }
  return out;
}

arma::dmat block_diagonal(std::vector<arma::dmat> diagonals) {
  assert(!diagonals.empty());
  int dim = 0;
  for (arma::dmat m : diagonals) {
    dim += m.n_rows;
    assert(m.is_diagmat());
  }
  int n = 0;
  arma::dmat out(dim, dim, arma::fill::zeros);
  for (arma::dmat mat : diagonals) {
    int s = mat.n_rows;
    out.submat(n, n, n + s - 1, n + s - 1) = mat;
    n += s;
  }
  return out;
}

arma::dvec join_rows(std::vector<arma::dvec> vecs){
  assert(!vecs.empty());
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
  return join_rows(measures);
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
  return join_rows(differentials);
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
  std::vector<arma::dmat> covariances;
  for (int i = 0; i < measurements.size(); ++i) {
    covariances.push_back(measurements[i]->covariance(time));
  }
  return block_diagonal(covariances);
}

arma::dmat CompositeLinearMeasurement::measurement_matrix(double t) {
  std::vector<arma::dmat> differentials;
  for (int i = 0; i < measurements.size(); ++i) {
    differentials.push_back(measurements[i]->measurement_matrix(t));
  }
  return join_rows(differentials);
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
