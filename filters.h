#pragma once

#include <armadillo>
#include <memory>
#include <cmath>

/**
 * This abstract class represent a deterministic model
 */
class Model{
protected:
  double current_t;
  arma::vec current_state;

public:


  /**
   * Sets the current trajectory to start at x at time t
   */
  virtual void set_initial_conditions(double t, arma::vec x){
    current_t = t;
    current_state = x;
  }

  /**
   * Calculates the value at time t along the current trajectory using n integration steps,
   * subsequent values will be calculated starting at this point.
   * The default implementation uses Runge-Kutta, can be overridden to allow for exact solutions
   */
  virtual arma::vec extrapolate(int steps,double t);

  /**
   *  Returns the derivative of the state vector at this point and time
   *  \f[ f(x,t) \f]
   *  where
   *  \f[dx = f(x,t)dt + d\beta_t \f]
   *  or
   *  f(x_n,n) where
   *  \f[x_{n+1} = f(x_n,n) + \nu_n \f]
   */
  virtual arma::vec derivative(double t, arma::vec state) = 0;
};

/**
 * Abstract class for models of the type
 * \f[x_{n+1} = Ax_n + \nu_n \f]
 * where \f$nu_n \sim N(0,Q_k)\f$ i.i.d
 */
class LinearDiscreteModel : public Model{
 public:
  /**
   *  The matrix \f$A\f$ in the above model
   */
  virtual arma::mat transform_matrix(int n) = 0;

  /**
   * The matrix Q_k in the above model
   */
  virtual arma::mat noise_matrix(int n) = 0;

};

class LinearContinuousModel;

/**
 * Abstract class for models with linear noise (i.e. most usable ones for Kalman
 * Filters and their variants)
 */
class LinearNoiseModel : public Model {
public:
  /**
   * The Covariance matrix at a given time
   */
  virtual arma::mat noise_matrix(double t) = 0;
};

/**
 * Abstract class for models of the type
 *  \f[dx = f(x,t)dt + d \beta_t \f]
 * where \f$nu_n \sim N(0,Q_k)\f$ i.i.d
 */
class LangevinEquationModel: public LinearNoiseModel{
 public:
  /**
   * The differential of \f$f(x,t)\f$ at the given time and position
   */
  virtual arma::mat differential(double time, arma::vec state) = 0;

  /**
   * Produces a model for a linear perturbation process with a nominal trajectory starting at
   * the passed time and state and updates the nominal trajectory at the passed rate.
   */
  virtual std::unique_ptr<LinearContinuousModel> linear_perturbation_model(double time, arma::vec state, double rate);

  virtual std::unique_ptr<LangevinEquationModel> clone() = 0;
};


/**
 * Abstract class for models of the type
 * \f[dx = F(t)dt + G(t) d \beta_t \f]
 * \f$\beta_t\f$ is a brownian motion process with \f$\mathbb{E}[d\beta_t d\beta_t^T] = I dt\f$
 */
class LinearContinuousModel : public LinearNoiseModel {
public:
  /**
   * F(t) in the above
   */
  virtual arma::mat transform_matrix(double t) = 0;

  arma::vec derivative(double t ,arma::vec state) override{
    return transform_matrix(t);
  }
};

/**
 *  An abstract class that provides the main functionality of a filter,
 *  updating based off of measurements and returning filtered data.
 */
class Filter{
 public:
   /**
    *  Returns a filtered value at the given time.
    *  If t is less than the current time then this is called smoothing.
    *  If t is greater than the current time then this is called prediction.
    *  If t is equal to the current time then this is called filtering.
    */
   virtual arma::vec filtered_value(double t) = 0;

   /**
    * Update the filter with new data
    */
   virtual arma::vec update(double t, arma::vec) = 0;
};

/**
 * This is an abstract class for representing
 * measurents of a process \f$x_t\f$ of the form
 * \f$h(x_t,t) + \nu_t\f$ where $nu_t \sim N(0, Q_t)$
 * Initialized with the dimension of measurement vector
 */
class Measurement {
 public:

  /**
   * Produces an uncorrupted measurement of the given state. This is
   * the mean and the mode of the measurement at this time.
   */
  virtual arma::vec measure(double time, arma::vec state);

  /**
   * Returns the covariance of the measurement at the given time
   **/
  virtual arma::mat covariance(double time);
};

class LinearMeasurement;

/**
 * Represents a measurement that can be locally linearized using a
 * Taylor expansion
 */
class LinearizeableMeasurement : public Measurement {
public:

  /**
   * The differential of the measurement function at the given time and point
   */
  virtual arma::mat differential(double t, arma::vec) = 0;

  /**
   *  The measurement function linearized around the given time and point
   */
  virtual std::unique_ptr<LinearMeasurement> linearize(double t, arma::vec) = 0;
};


/**
 * A measurement given by the formula
 * \f$A_t x_t + \nu_t\f$ where $nu_t \sim N(0, Q_t)$
 **/
class LinearMeasurement : public LinearizeableMeasurement {
 public:
  /**
   * The measurement matrix
   */
  virtual arma::mat measurement_matrix(double time) = 0;
};

/**
 * The classic Kalman filter.  Requires a linear model and measurement as well as a prior
 * estimate of the systems state and covariance (i.e a gaussian prior)
 */
class KalmanFilter : public Filter{
 private:
  std::unique_ptr<LinearContinuousModel> model;
  std::unique_ptr<LinearMeasurement> measurement;

public:
 KalmanFilter(arma::vec prior_position, arma::mat prior_covariance,
              std::unique_ptr<LinearContinuousModel> model,
               std::unique_ptr<LinearMeasurement> measurement):
  model(std::move(model)), measurement(std::move(measurement))
    {}

 arma::vec filtered_value(double t) override;


 arma::vec update(double t, arma::vec) override;
};

/**
 * The extended Kalman filter.  Requires a Langevin equation model and linearizeable measurement as well
 * as a prior estimate of the systems state and covariance (i.e a gaussian
 * prior)
 */
class ExtendedKalmanFilter : public Filter{
  KalmanFilter perturbationProcessFilter;

 public:
 ExtendedKalmanFilter(double rate,arma::vec prior_position, arma::mat prior_covariance,
                        std::unique_ptr<LangevinEquationModel> model,
                        std::unique_ptr<LinearizeableMeasurement> measurement)
     : perturbationProcessFilter(prior_position,prior_covariance,
                                 model->linear_perturbation_model(0,prior_position,rate),
                    measurement->linearize(0,prior_position)) {}

   arma::vec filtered_value(double t) override;

   arma::vec update(double t, arma::vec) override;
};


/**
 *  This class is the model for the first order perturbation process of a LangevinEquationModel
 */
class LinearizedLangevinEquationModel : public LinearContinuousModel {
private:
  std::unique_ptr<LangevinEquationModel> langevin_model;
  double rate;

public:
  /**
   * Takes the number of steps for runge kutta updates when calculating the nominal trajectory as well
   * as the initial conditions to use.  
   */
  LinearizedLangevinEquationModel(
      double rate, double start_time, arma::vec start_state,
      std::unique_ptr<LangevinEquationModel> langevin_model)
      : langevin_model(std::move(langevin_model)), rate{rate} {
    langevin_model->set_initial_conditions(start_time, start_state);
  }

  arma::vec derivative(double t, arma::vec state) override{
    return transform_matrix(t) * state;
  }

  arma::mat noise_matrix(double t) override{
    return langevin_model->noise_matrix(t);
  }

  arma::mat transform_matrix(double t) override{
    int steps = std::ceil(std::abs((t - current_t) * rate));

    arma::vec nominal_state = langevin_model->extrapolate(steps, t);
    arma::mat transition_matrix =
        langevin_model->differential(t, nominal_state);
    return transition_matrix;
  }

  /**
   *  In this case calling this will relinearize around the passed point and
   * time
   */
  void set_initial_conditions(double t, arma::vec x) override{
    langevin_model->set_initial_conditions(t, x);
  }

};

template<class F>
arma::vec runge_kutta_4(int steps, double start_t,double end_t, arma::vec start_state, F derivative) {
  double time_step = (end_t - start_t) / steps;
  double current_t = start_t;
  arma::vec current_state = start_state;
  for (int i = 0; i < steps; i++) {
    arma::vec k1 = derivative(current_t, current_state);
    arma::vec k2 = derivative(current_t + time_step / 2, (time_step / 2) * k1);
    arma::vec k3 = derivative(current_t + time_step / 2, (time_step / 2) * k2);
    arma::vec k4 =
        derivative(current_t + time_step, start_state + time_step * k3);
    current_t += time_step;
    current_state += (time_step / 6) * (k1 + 2 * k2 + 2 * k3 + k4);
  }
  return current_state;
}
