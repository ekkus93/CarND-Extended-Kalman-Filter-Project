#include "kalman_filter.h"
#include <iostream>
#include <math.h>
#include <assert.h>
#include "tools.h"

using namespace std;

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {
  u_ = VectorXd(4);
	u_ << 0, 0, 0, 0;

  P_ = MatrixXd(4, 4);
  P_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1000, 0,
        0, 0, 0, 1000;  

	F_ = MatrixXd(4, 4);
	F_ << 1, 0, 1, 0,
			  0, 1, 0, 1,
			  0, 0, 1, 0,
			  0, 0, 0, 1;

  I_ = MatrixXd::Identity(4, 4);

  Q_ = MatrixXd(4, 4);
}

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

VectorXd KalmanFilter::h()
{
  float x = x_(0);
  float y = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  VectorXd z_pred = VectorXd(3);

  if (fabs(x) < 0.0001 && fabs(y) < 0.0001)
  {
    // catch div by 0
    z_pred << 0.0, 0.0, 0.0;
  }
  else
  {
    float rho = sqrt(x*x+y*y);
    float theta = atan2(y, x); 
    float rho_dot = (x*vx+y*vy)/rho;

    z_pred << rho, theta, rho_dot;
  }

  for(int i=0; i<3; i++)
  {
    assert(!isnan(z_pred(i)));
  }

  return z_pred;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  VectorXd z_pred = h();

  VectorXd y = z - z_pred;
  Tools tools;
  y(1) = tools.ConstrainAngle(y(1));

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;

  // new state
  /*
  cout << "###y: " << y.rows() << ", " << y.cols() << "\n";
  cout << "###K: " << K.rows() << ", " << K.cols() << "\n";
  cout << "###x_: " << x_.rows() << ", " << x_.cols() << "\n";
  */
  x_ = x_ + (K * y);
  P_ = (I_ - K * H_) * P_;
}

void KalmanFilter::DisplayData()
{
  cout << "x_: \n" << x_ << "\n";
  cout << "P_: \n" << P_ << "\n";
  cout << "u_: \n" << P_ << "\n";
  cout << "F_: \n" << F_ << "\n";
  cout << "H_: \n" << H_ << "\n";
  cout << "R_: \n" << R_ << "\n";
  cout << "I_: \n" << I_ << "\n";
  cout << "Q_: \n" << Q_ << "\n";
}
