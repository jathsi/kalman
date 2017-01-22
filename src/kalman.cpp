
#include <iostream>
#include <stdexcept>
#include <cmath>
#include <ros/console.h>
#include <ros/ros.h>
#include "kalman/kalman.h"

KalmanFilter::KalmanFilter(
    const Eigen::MatrixXd& A,
    const Eigen::MatrixXd& C,
    const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& R,
    const Eigen::MatrixXd& P)
  : A(A), C(C), Q(Q), R(R), P0(P),
    m(C.rows()), n(A.rows()), initialized(false),
    I(n, n), x_hat(n), x_hat_new(n)
{
 
  I.setIdentity();
}


void KalmanFilter::init(const Eigen::VectorXd& x0) {
  x_hat = x0;
  P = P0;
  initialized = true;
  for(int ht=0;ht<720;ht++)kb[ht]=false;
}

void KalmanFilter::init() {
  x_hat.setZero();
  P = P0;
  t0 = 0;
  t = t0;
  initialized = true;
}

double KalmanFilter::update_pre(const Eigen::VectorXd& y) {

  if(!initialized)
    throw std::runtime_error("Filter is not initialized!");

  x_hat_new = x_hat;
  P = A*P*A.transpose() + Q;
  v = y-C*x_hat_new;
  s = C*P*C.transpose()+R;
 return(v(0,0)*v(0,0)/s(0,0));

}



void KalmanFilter::update_if(int num){
  //kb[num]=true;
  kb[num]=true;
  //ROS_DEBUG("%d",num);

}

void KalmanFilter::update_else(const Eigen::VectorXd& y){

  K = P*C.transpose()*(C*P*C.transpose() + R).inverse();
  x_hat_new += K * (y - C*x_hat_new);
  P = (I - K*C)*P;
  x_hat = x_hat_new;
}

void KalmanFilter::calcp(double r) {

  double val1 = pow(1/tan((3.14/18)) * r * .00436717644 ,2);
  double val2 = pow(1/tan((3.14/18)) * r,2); 
  this->P0 << val1, 0, 0, val2;

}


void KalmanFilter::printkb(){
  int i=0;
while (i<720){
 if(kb[i]) ROS_DEBUG("%d",i);

i++;
}}