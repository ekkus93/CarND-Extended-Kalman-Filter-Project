#include <limits.h>
#include <iostream>
#include "kalman_filter.h"
#include "gtest/gtest.h"

using namespace std;

TEST(Kalman_Filter_Predict, Test1) {
  KalmanFilter kf = KalmanFilter();

  //design the KF with 1D motion
	Eigen::VectorXd x = Eigen::VectorXd(2);
	x << 0.999001, 0;

	Eigen::MatrixXd P = Eigen::MatrixXd(2, 2);
	P << 0.999001, 0, 0, 1000;

	Eigen::MatrixXd F = Eigen::MatrixXd(2, 2);
	F << 1, 1, 0, 1;

	Eigen::MatrixXd H = Eigen::MatrixXd(1, 2);
	H << 1, 0;

	Eigen::MatrixXd R = Eigen::MatrixXd(1, 1);
	R << 1;
	Eigen::MatrixXd Q = Eigen::MatrixXd(2, 2);
	Q << 0, 0, 0, 0;

  kf.Init(x, P, F, H, R, Q);
  kf.Predict();

  Eigen::VectorXd expected_x = Eigen::VectorXd(2);
  expected_x << 0.999001, 0;
  Eigen::MatrixXd expected_P = Eigen::MatrixXd(2, 2);
  expected_P << 1001, 1000, 1000, 1000;

  Eigen::VectorXd predicted_x = kf.x_;
  for(int i=0; i<2; i++)
  {
    EXPECT_NEAR(expected_x(i), predicted_x(i), 0.000001); 
  }

  Eigen::MatrixXd predicted_P = kf.P_;
  for(int col=0; col < expected_P.cols(); col++)
  {
    for(int row=0; row < expected_P.rows(); row++)
    {
      float expected_val = expected_P(col, row);
      float predicted_val = predicted_P(col, row);
      EXPECT_NEAR(expected_val, predicted_val, 0.001);
    }
  }
}

TEST(Kalman_Filter_Predict, Test2) {
  KalmanFilter kf = KalmanFilter();

  //design the KF with 1D motion
	Eigen::VectorXd x = Eigen::VectorXd(2);
	x << 1.999, 0.999002;

	Eigen::MatrixXd P = Eigen::MatrixXd(2, 2);
	P << 0.999002, 0.998005, 0.998005, 1.99501;

	Eigen::MatrixXd F = Eigen::MatrixXd(2, 2);
	F << 1, 1, 0, 1;

	Eigen::MatrixXd H = Eigen::MatrixXd(1, 2);
	H << 1, 0;

	Eigen::MatrixXd R = Eigen::MatrixXd(1, 1);
	R << 1;
	Eigen::MatrixXd Q = Eigen::MatrixXd(2, 2);
	Q << 0, 0, 0, 0;

  kf.Init(x, P, F, H, R, Q);
  kf.Predict();

  Eigen::VectorXd expected_x = Eigen::VectorXd(2);
  expected_x << 2.998, 0.999002;
  Eigen::MatrixXd expected_P = Eigen::MatrixXd(2, 2);
  expected_P << 4.99002, 2.99302, 2.99302, 1.99501;

  Eigen::VectorXd predicted_x = kf.x_;
  for(int i=0; i<2; i++)
  {
    float expected_val = expected_x(i);
    float predicted_val = predicted_x(i);
    EXPECT_NEAR(expected_x(i), predicted_x(i), 0.001); 
  }

  Eigen::MatrixXd predicted_P = kf.P_;
  for(int col=0; col < expected_P.cols(); col++)
  {
    for(int row=0; row < expected_P.rows(); row++)
    {
      float expected_val = expected_P(col, row);
      float predicted_val = predicted_P(col, row);
      EXPECT_NEAR(expected_val, predicted_val, 0.001);
    }
  }
}
