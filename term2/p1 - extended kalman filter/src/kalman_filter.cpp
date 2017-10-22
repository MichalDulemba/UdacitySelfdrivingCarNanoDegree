#include "kalman_filter.h"
#include <math.h>
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;   // or maybe x without _
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  this is for LIDAR
  */

  VectorXd y = z - H_ * x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ *Ht * Si;

  int size = x_.size();
  MatrixXd I = MatrixXd::Identity(size, size);

 // new state
  x_ = x_ + (K * y);
  P_ = (I - K * H_) * P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
  This is for RADAR
    * update the state by using Extended Kalman Filter equations
  */
  //    		meas_package.raw_measurements_ = VectorXd(3);


  float x = x_(0);
  float y = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  if (fabs(x*x + y*y) < 0.0001) {
    std::cout << "CalculateJacobian()  Div by 0" << std::endl;
    return;
  }



  float rho = sqrt(x*x+y*y);
  float theta = atan2(y,x);
  float ro_dot = (x*vx+y*vy)/rho;

  VectorXd z_pred = VectorXd(3);
  z_pred << rho, theta, ro_dot;
  VectorXd y_res = z - z_pred;


  y_res(1) = atan2(sin(y_res(1)), cos(y_res(1)));
   if (y_res(1) < -M_PI) {
     y_res(1) = y_res(1) + 2*M_PI;

   }
   else if (y_res(1) > M_PI) {
     y_res(1) = y_res(1) - 2*M_PI;
   }



  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;
  int size = x_.size();
  MatrixXd I = MatrixXd::Identity(size, size);

  // new state
  x_ = x_ + (K * y_res);
  P_ = (I - K * H_) * P_;


}
