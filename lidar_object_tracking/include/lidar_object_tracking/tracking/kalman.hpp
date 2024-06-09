/**
* Kalman filter implementation using Eigen. Based on the following
* introductory paper:
*
*     http://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf
*
* @author: David Chung, adapted from Hayk Martirosyan (original author)
* @date: 2024.03.31
*/

#pragma once

#include <Eigen/Dense>

class KalmanFilter {

public:

  /**
  * Create a Kalman filter with the specified matrices.
  *   A - State Transition Matrix
  *   H - Observation matrix. multiply a state vector by H to translate it to a measurement vector.
  *   Q - Process noise covariance
  *   R - Measurement noise covariance
  *   P - Estimate error covariance
  */
  // Matrices for computation
  Eigen::MatrixXd A, H, Q, R, P, K, P0;

  KalmanFilter(
      double dt,
      const Eigen::MatrixXd& A,
      const Eigen::MatrixXd& H,
      const Eigen::MatrixXd& Q,
      const Eigen::MatrixXd& R,
      const Eigen::MatrixXd& P0
  );

  /**
  * Create a blank estimator.
  */
  KalmanFilter();

  /**
  * Initialize the filter with initial states as zero.
  */
  void init();

  /**
  * Initialize the filter with a guess for initial states.
  */
  void init(double t0, const Eigen::VectorXd& x0 );

  /**
   * Predict/estimate next state given current state
  */
  void predict();

  /**
  * Update the estimated state based on measured values. The
  * time step is assumed to remain constant.
  */
  void update(const Eigen::VectorXd& y);


  // void update(const Eigen::VectorXd& y, double dt);

  /**
  * Update the estimated state based on measured values,
  * using the given time step and dynamics matrix.
  */
  // void update(const Eigen::VectorXd& y, double dt, const Eigen::MatrixXd A);

  // void changeStates(const Eigen::VectorXd& new_states); 
  /**
  * Return the current state and time.
  */
  Eigen::VectorXd state() { return x_hat; };
  Eigen::VectorXd predictedState() { return x_hat_new; };
  double time() { return t; };

private:

  // System dimensions
  int m, n;

  // Initial and current time
  double t0, t;

  // Discrete time step
  double dt;

  // Is the filter initialized?
  bool initialized;

  // n-size identity
  Eigen::MatrixXd I;

  // Estimated states
  Eigen::VectorXd x_hat, x_hat_new;
};