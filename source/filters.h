#pragma once

#include <armadillo>
#include <memory>
#include <cmath>

/**
 * Does Runge-Kutta for some vector type V using a callable F : (double, V) -> V
 * to compute the derivatives
 */
template <class V, class F>
V runge_kutta_4(int steps, double start_t, double end_t, V start_state,
                F derivative) {
  const double time_step = (end_t - start_t) / steps;
  double current_t = start_t;
  V current_state = start_state;
  for (int i = 0; i < steps; i++) {
    V k1 = derivative(current_t, current_state);
    V k2 = derivative(current_t + time_step / 2, current_state + (time_step / 2) * k1);
    V k3 = derivative(current_t + time_step / 2, current_state + (time_step / 2) * k2);
    V k4 = derivative(current_t + time_step, current_state + time_step * k3);
    current_t += time_step;
    current_state += (time_step / 6) * (k1 + 2 * k2 + 2 * k3 + k4);
  }
  return current_state;
}

/**
 * This abstract class represent a deterministic model for some vector valued function
 */
template<class V>
class Model{
protected:
  double current_t;
  double rate;
  V current_state;

public:
  Model(double rate,double current_t, V current_state)
    : rate{rate},current_t{current_t},current_state{current_state}{}

  /**
   * Sets the current trajectory to start at x at time t
   */
  virtual void set_initial_conditions(double t, V x){
    current_t = t;
    current_state = x;
  }

  void set_time (double time){
    this->current_t = current_t;
  }

  double get_time (){
    return this->current_t;
  }

  void set_state (V state){
    this->state = state;
  }

  V get_state (){
    return this->state;
  }



  /**
   * Calculates the value at time t along the current trajectory using n integration steps,
   * subsequent values will be calculated starting at this point.
   * The default implementation uses Runge-Kutta, can be overridden to allow for exact solutions
   */
   virtual V extrapolate( double t) {
     int steps = std::ceil(std::abs(rate*(t - current_t)));
     if (std::abs(t - current_t) < __DBL_EPSILON__) {
       return current_state;
     }
    current_state = runge_kutta_4(
        steps, current_t, t, current_state,
        [&](double t, V state) -> V { return this->derivative(t, state); });
    this->current_t = t;
    return current_state;
  }
  /**
     *  Returns the derivative of the state vector at this point and time
     *  \f[ f(x,t) \f]
     *  where
     *  \f[dx = f(x,t)dt + d\beta_t \f]
     *  or
     *  f(x_n,n) where
     *  \f[x_{n+1} = f(x_n,n) + \nu_n \f]
     */
  virtual V derivative(double t, V state) = 0;
};

/**
 * Abstract class for models of the type
 * \f[x_{n+1} = Ax_n + \nu_n \f]
 * where \f$ \nu_n \sim N(0,Q_k)\f$ i.i.d
 */
class LinearDiscreteModel : public Model<arma::dvec>{
 public:
  LinearDiscreteModel(double rate, double current_t, arma::dvec current_state)
     : Model{rate, current_t, current_state} {}

   /**
    *  The matrix \f$A\f$ in the above model
    */
   virtual arma::dmat transform_matrix(int n) = 0;

   /**
    * The matrix Q_k in the above model
    */
   virtual arma::dmat noise_matrix(int n) = 0;

};

class LinearContinuousModel;

/**
 * Abstract class for models with linear noise (i.e. most usable ones for Kalman
 * Filters and their variants)
 */
class LinearNoiseModel : public Model<arma::dvec> {
public:
  LinearNoiseModel(double rate, double current_t, arma::dvec current_state)
      : Model{rate, current_t, current_state} {}
  /**
   * The Covariance matrix at a given time
   */
  virtual arma::dmat noise_matrix(double t) = 0;
};

/**
 * Abstract class for models of the type
 *  \f[dx = f(x,t)dt + d \beta_t \f]
 * where \f$ \mathbb{E}[d\beta_t d\beta_t^T ] = Q(t) dt\f$ i.i.d
 */
class LangevinEquationModel: public LinearNoiseModel{
 public:
   LangevinEquationModel(double rate, double current_t, arma::dvec current_state)
     : LinearNoiseModel{rate, current_t, current_state} {}
   /**
    * The differential of \f$f(x,t)\f$ at the given time and position
    */
   virtual arma::dmat differential(double time, arma::dvec state) = 0;

   /**
    * Produces a model for a linear perturbation process with a nominal
    * trajectory starting at the passed time and state and updates the nominal
    * trajectory at the passed rate.
    */
   virtual std::shared_ptr<LinearContinuousModel>
   linear_perturbation_model(double time, arma::dvec state, double rate);

   virtual std::shared_ptr<LangevinEquationModel> clone() = 0;
};

/**
 * Abstract class for models of the type
 * \f[dx = F(t)dt + G(t) d \beta_t \f]
 * \f$\beta_t\f$ is a brownian motion process with \f$\dmathbb{E}[d\beta_t d\beta_t^T] = I dt\f$
 */
class LinearContinuousModel : public LinearNoiseModel {
public:
  LinearContinuousModel(double rate, double current_t, arma::dvec current_state)
      : LinearNoiseModel{rate, current_t, current_state} {}
  /**
   * F(t) in the above
   */
  virtual arma::dmat transform_matrix(double t) = 0;

  virtual arma::dvec extrapolate(double t) override{
    if(current_state.is_zero()){ //early exit
      current_t = t;
      return current_state;
    }else{
      return Model<arma::dvec>::extrapolate(t);
    }
  }

  arma::dvec derivative(double t ,arma::vec state) override{
    return transform_matrix(t)*state;
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
   virtual arma::dvec filtered_value(double t) = 0;

   /**
   * Returns an estidmate in the uncertainty of the filtered value at the passed time
   */
   virtual arma::dmat covariance(double t) = 0;

   /**
    * Update the filter with a new measurement and returns the new estidmate for the current time
    */
   virtual arma::dvec update(double t, arma::dvec) = 0;
};

/**
 * This is an abstract class for representing
 * measurents of a process \f$x_t\f$ of the form
 * \f$h(x_t,t) + \nu_t\f$ where \f$ nu_t \sim N(0, Q_t) \f$
 * Initialized with the dimension of measurement vector
 */
class Measurement {
 public:

  /**
   * Produces an uncorrupted measurement of the given state. This is
   * the mean and the mode of the measurement at this time.
   */
  virtual arma::dvec measure(double time, arma::dvec state) = 0;

  /**
   * Returns the covariance of the measurement at the given time
   **/
  virtual arma::dmat covariance(double time) = 0;
};

class LinearMeasurement;
class LinearizedMeasurement;

/**
 * Represents a measurement that can be locally linearized using a
 * Taylor expansion
 */
class LinearizeableMeasurement : public Measurement {
public:

  /**
   * The differential of the measurement function at the given time and point
   */
  virtual arma::dmat differential(double t, arma::dvec) = 0;

  /**
   *  The measurement function linearized around the given time and point
   */
  virtual std::unique_ptr<LinearMeasurement>
  linearize(std::shared_ptr<LangevinEquationModel> langevin_model);

  virtual std::shared_ptr<LinearizeableMeasurement> clone() = 0;
};




/**
 * A measurement given by the formula
 * \f$A_t x_t + \nu_t\f$ where \f$nu_t \sim N(0, Q_t)\f$
 **/
class LinearMeasurement : public Measurement {
 public:
   /**
    * Produces an uncorrupted measurement of the given state. This is
    * the mean and the mode of the measurement at this time.
    */
  virtual arma::dvec measure(double time, arma::vec state){
    return measurement_matrix(time)*state;
  }

   /**
    * The measurement matrix
    */
   virtual arma::dmat measurement_matrix(double time) = 0;
};

/**
 * The linearization produced by a linearizeable measurement.  The matrix
 * should be the differential of the original measurement function.
 */
class LinearizedMeasurement : public LinearMeasurement{
  std::shared_ptr<LinearizeableMeasurement> measurement;
  std::shared_ptr<LangevinEquationModel> nominal_model;
public:
  LinearizedMeasurement(std::shared_ptr<LinearizeableMeasurement> measurement,
                        std::shared_ptr<LangevinEquationModel> nominal_model)
    : measurement{measurement}, nominal_model{nominal_model} {}

  arma::dmat measurement_matrix(double time) override{
    return measurement->differential(time, nominal_model->extrapolate(time));
  };

  arma::dmat covariance(double time) override{
    return measurement->covariance(time);
  }
};


/**
 * Combines a collection of linear measurements into a single measurement.  The measurement
 * matrix is produced by joining the rows of the constituent measurement matrices.  All the
 * constituent measurements must measure from a state with the same dimension.
 */
class CompositeLinearMeasurement: public LinearMeasurement{
  std::vector<std::shared_ptr<LinearMeasurement>> measurements;

public:
  CompositeLinearMeasurement(
      std::vector<std::shared_ptr<LinearMeasurement>> measurements)
      : measurements(measurements) {}

  arma::dmat covariance(double time) override;

  virtual arma::dmat measurement_matrix(double time) override;
};


/**
 * Joins the rows of all the matrices in the passed vector to
 * produce a new matrix.  The vector must be nonempty and all
 * the passed matrices must have the same number of columns
 */
arma::dmat join_rows(std::vector<arma::dmat> mat);

/**
 * Creates a new matrix with all  the matrices in the  passed vector on the
 * diagonal.  All the matrices must be square and the vector must be nonempty.
 */
arma::dmat block_diagonal(std::vector<arma::dmat> mat);

/**
 * Joins all the vectors in the passed std::vector to
 * produce a new vector.  The vector must be nonempty.
 */
arma::dvec join_rows(std::vector<arma::dvec> vec);


/**
 *  Combines a collection of linearizeable measurement into a single measurement.  Then
 *  measurement vector produced is the concatenation of the measurement vectors of the
 *  constituent measurements.
 */
class CompositeLinearizeableMeasurement : public LinearizeableMeasurement {
  std::vector<std::shared_ptr<LinearizeableMeasurement>> measurements;

public:
  CompositeLinearizeableMeasurement(
      std::vector<std::shared_ptr<LinearizeableMeasurement>> measurements)
      : measurements(measurements) {}

  CompositeLinearizeableMeasurement(CompositeLinearizeableMeasurement &other) = default;

  arma::dvec measure(double time, arma::dvec state) override;

  arma::dmat covariance(double time) override;

  arma::dmat differential(double t, arma::dvec state) override;

  std::unique_ptr<LinearMeasurement>
  linearize(std::shared_ptr<LangevinEquationModel> langevin_model) override;

  std::shared_ptr<LinearizeableMeasurement> clone() override {
    return std::shared_ptr<LinearizeableMeasurement>(
        new CompositeLinearizeableMeasurement(*this));
  };
};

/**
 * The classic Kalman filter.  Requires a linear model and measurement as well as a prior
 * estidmate of the systems state and covariance (i.e a gaussian prior)
 */
class KalmanFilter : public Filter{
 protected:
  /**
   * This model has the dynamics for the covariance matrix of a Kalman Filter.
   * Since it uses a reference the the KalmanFilter it should be owned
   * by the KalmanFilter it is referencing.
   */
  class CovarianceMatrixModel : public Model<arma::dmat>{
    KalmanFilter & filter;
  public:

    CovarianceMatrixModel(KalmanFilter & filter,
                          double rate, double t,
                          arma::dmat starting_state)
      : filter{filter}, Model{rate,t,starting_state} {}

    arma::dmat derivative(double t, arma::dmat state) override{
      const arma::dmat F = filter.model->transform_matrix(t);
      const arma::dmat Q = filter.model->noise_matrix(t);
      const arma::dmat P = state;
      // std::cout << "det F" << arma::det(F.submat(2,0,3,1)) << std::endl;
      return F*P + P*arma::trans(F) + Q;
    }

    arma::dmat extrapolate(double t) override{
      Model<arma::dmat>::extrapolate(t);
      return current_state;
    }
  };


  std::shared_ptr<LinearContinuousModel> model;
  std::shared_ptr<LinearMeasurement> measurement;
  double rate;

  /**
   * It is possible that a CovarianceMatrixModel could be
   * replace by an exact model that has parametic solutions
   * that do not require integration.  This reference allows that
   * replacement
   */
  std::shared_ptr<Model<arma::dmat>> matrix_model;
public:

  KalmanFilter(double rate,
               arma::dvec prior_position, arma::dmat prior_covariance,
               std::shared_ptr<LinearContinuousModel>  model,
               std::shared_ptr<LinearMeasurement>  measurement):
    model(model), measurement(measurement), rate{rate},
    matrix_model(new CovarianceMatrixModel(*this,rate,0,prior_covariance))
    {
      model->set_initial_conditions(0, prior_position);
    }

  KalmanFilter(double rate, arma::dvec prior_position,
               arma::dmat prior_covariance,
               std::shared_ptr<LinearContinuousModel> model,
               std::shared_ptr<LinearMeasurement> measurement,
               std::shared_ptr<Model<arma::dmat>> & matrix_model)
      : model(model),
        measurement(measurement), rate{rate},
        matrix_model(matrix_model)
  {}

  void set_rate(double rate){
    this->rate = rate;
  }

  double get_rate() const{
    return rate;
  }

  arma::dmat covariance(double t) override{
    return matrix_model->extrapolate(t);
  }

  arma::dvec filtered_value(double t) override{
    return model->extrapolate(t);
  }

  std::shared_ptr<LinearMeasurement> get_measurement(){
    return measurement;
  }

  void set_measurement(std::shared_ptr<LinearMeasurement> measurement) {
    this->measurement = measurement;
  }

  std::shared_ptr<LinearContinuousModel> get_model(){
    return model;
  }

  void set_model(std::shared_ptr<LinearContinuousModel> model) {
    this->model = model;
  }


  arma::dvec update(double t, arma::vec) override;
};

/**
 * The extended Kalman filter.  Requires a Langevin equation model and linearizeable measurement as well
 * as a prior estidmate of the systems state and covariance (i.e a gaussian
 * prior)
 */
class ExtendedKalmanFilter : public Filter{
protected:
  KalmanFilter perturbationProcessFilter; ///< The filter for the linearized process
  std::shared_ptr<LangevinEquationModel> model; ///< The model, shared with the linearized process, for the nominal trajectory
  std::shared_ptr<LinearizeableMeasurement> measurement;

 public:
 ExtendedKalmanFilter(double rate,arma::dvec prior_position, arma::dmat prior_covariance,
                        std::shared_ptr<LangevinEquationModel> model,
                      std::shared_ptr<LinearizeableMeasurement> & measurement)
   : perturbationProcessFilter(rate,prior_position,prior_covariance,
                               model->linear_perturbation_model(0,prior_position,rate),
                               measurement->linearize(model)),
     model{model},
     measurement{measurement}
  {}

   std::shared_ptr<LinearizeableMeasurement> get_measurement() { return measurement; }

   void set_measurement(std::shared_ptr<LinearizeableMeasurement> measurement) {
     this->measurement = measurement;
     perturbationProcessFilter.set_measurement(measurement->linearize(model));
   }

    std::shared_ptr<LangevinEquationModel> get_model() {
      return model;
    }

    void set_model(std::shared_ptr<LangevinEquationModel> model) {
      this->model = model;
    }
   arma::dvec filtered_value(double t) override;

   arma::dmat covariance(double t) override;

   arma::dvec update(double t, arma::vec) override;
};


/**
 *  This class is the model for the first order perturbation process of a LangevinEquationModel
 */
class LinearizedLangevinEquationModel : public LinearContinuousModel {
private:
  std::shared_ptr<LangevinEquationModel> langevin_model;

public:
  /**
   * Takes the number of steps for runge-kutta updates when calculating the nominal trajectory as well
   * as the initial conditions to use.  The start_state and start_time here are for the underlying model.
   * Since this is used as a perturbation model the starting state is initialized to zero.
   */
  LinearizedLangevinEquationModel(
      double rate, double start_time, arma::dvec start_state,
      std::shared_ptr<LangevinEquationModel> langevin_model)
      : langevin_model(langevin_model),
        LinearContinuousModel{rate, current_t, start_state} {
    langevin_model->set_initial_conditions(start_time, start_state);
    current_t = current_t;
    current_state = arma::dvec(start_state.size(),arma::fill::zeros);
  }

  arma::dvec derivative(double t, arma::vec state) override{
    return transform_matrix(t) * state;
  }

  arma::dmat noise_matrix(double t) override{
    return langevin_model->noise_matrix(t);
  }

  arma::dmat transform_matrix(double t) override{
    int steps = std::ceil(std::abs((t - current_t) * rate));

    arma::dvec nominal_state = langevin_model->extrapolate( t);
    arma::dmat transition_matrix =
        langevin_model->differential(t, nominal_state);
    return transition_matrix;
  }


};

