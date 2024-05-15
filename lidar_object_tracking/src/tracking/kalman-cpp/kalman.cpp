/**
* Implementation of KalmanFilter class.
*
* @author: Hayk Martirosyan
* @date: 2014.11.15
*/

#include <iostream>
#include <stdexcept>

#include "kalman.hpp"

KalmanFilter::KalmanFilter(
    double dt,
    const Eigen::MatrixXd& A,
    const Eigen::MatrixXd& H,
    const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& R,
    const Eigen::MatrixXd& P0)
  : A(A), H(H), Q(Q), R(R), P0(P0),
    m(H.rows()), n(A.rows()), dt(dt), initialized(false),
    I(n, n), x_hat(n), x_hat_new(n)
{
  I.setIdentity();
}

KalmanFilter::KalmanFilter() {}

void KalmanFilter::init(double t0, const Eigen::VectorXd& x0) {
  x_hat = x0;
  P = P0;
  this->t0 = t0;
  t = t0;
  initialized = true;

}

void KalmanFilter::init() {
  x_hat.setZero();
  P = P0;
  t0 = 0;
  t = t0;
  initialized = true;
}

void KalmanFilter::predict() {

  if(!initialized)
      throw std::runtime_error("Filter is not initialized!");
  x_hat_new = A * x_hat; //extrapolate the state (note that we have no control input)
  P = A*P*A.transpose() + Q; //extrapolate uncertainty

}

void KalmanFilter::update(const Eigen::VectorXd& y) { //where y is the measurement vector
  if(!initialized)
    throw std::runtime_error("Filter is not initialized!");

  K = P*H.transpose()*(H*P*H.transpose() + R).inverse(); //compute Kalman gain
  x_hat_new += K * (y - H*x_hat_new);
  P = (I - K*H)*P;
  x_hat = x_hat_new;

  t += dt;
}

// void KalmanFilter::changeStates(const Eigen::VectorXd& new_states) {

//   if(!initialized)
//     throw std::runtime_error("Filter is not initialized!");
//   if(x_hat.size() != new_states.size())
//     throw std::runtime_error("State vectors do not have the same size");
//   x_hat = new_states;
// }

// void KalmanFilter::update(const Eigen::VectorXd& y, double dt, const Eigen::MatrixXd A) {

//   this->A = A;
//   this->dt = dt;
//   update(y);
// }

// void KalmanFilter::update(const Eigen::VectorXd& y, double dt) {

//   this->dt = dt;
//   update(y);
// }