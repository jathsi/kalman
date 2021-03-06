

#include <Eigen/Dense>
#include <cmath>
#pragma once
#include <visualization_msgs/Marker.h>

class KalmanFilter {

public:

  /**
  * Create a Kalman filter with the specified matrices.
  *   A - System dynamics matrix
  *   C - Output matrix
  *   Q - Process noise covariance
  *   R - Measurement noise covariance
  *   P - Estimate error covariance
  */
  KalmanFilter(
      const Eigen::MatrixXd& A,
      const Eigen::MatrixXd& C,
      const Eigen::MatrixXd& Q,
      const Eigen::MatrixXd& R,
      const Eigen::MatrixXd& P
  );

  /**
  * Create a blank estimator.
  */
  void calcp(double r);
  
  KalmanFilter();

  /**
  * Initialize the filter with initial states as zero.
  */
  void init();

  /**
  * Initialize the filter with a guess for initial states.
  */
  void init(const Eigen::VectorXd& x0);

  /**
  * Update the estimated state based on measured values. The
  * time step is assumed to remain constant.
  */
  double update_pre(const Eigen::VectorXd& y);

  /**
  * Update the estimated state based on measured values,
  * using the given time step and dynamics matrix.
  */

  /**
  * Return the current state and time.
  */
  void update_if(int num);
  void update_else(const Eigen::VectorXd& y);

  Eigen::VectorXd state() { return x_hat; };
  double time() { return t; };
  void printkb();

private:

  // Matrices for computation
  Eigen::MatrixXd A, C, Q, R, P, K, P0, v, s;

  // System dimensions
  int m, n;

  // Initial and current time
  double t0, t;

  // Discrete time step
  double dt;

  // Is the filter initialized?
  bool initialized;
  bool kb[720];

  // n-size identity
  Eigen::MatrixXd I;
 

  // Estimated states
  Eigen::VectorXd x_hat, x_hat_new;
};
