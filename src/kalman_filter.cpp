#include "kalman_filter.h"
#include <iostream>
#include <math.h>

#define PI 3.14159265


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
  x_ = F_ * x_;

  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */

    VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  VectorXd z_pred;
  z_pred = VectorXd(3);
  z_pred << 1, 1, 1;
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);


  float c1 = sqrt(px*px+py*py);

  if(fabs(c1) < 0.0001){
      std::cout << "The location comes the error!" << std::endl;
      return;
  }
  else{

      float c2 = atan2(py,px);
      float c3 = (vx*px + vy*py)/c1;

      z_pred << c1,c2,c3;
      VectorXd y = z - z_pred;

      while(y(1)<(-1)*PI || y(1)>PI){
          if (y(1)<0){
              y(1) = y(1) + 2*PI;
          }
          else{
              y(1) = y(1) - 2*PI;
          }
      }

      MatrixXd Ht = H_.transpose();
      MatrixXd S = H_ * P_ * Ht + R_;
      MatrixXd Si = S.inverse();
      MatrixXd PHt = P_ * Ht;
      MatrixXd K = PHt * Si;

      //new estimate
      x_ = x_ + (K * y);
      long x_size = x_.size();
      MatrixXd I = MatrixXd::Identity(x_size, x_size);
      P_ = (I - K * H_) * P_;
  }




}
