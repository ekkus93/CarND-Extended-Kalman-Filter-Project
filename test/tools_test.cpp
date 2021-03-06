#include <limits.h> 
#include <iostream>
#include "tools.h"
#include "FusionEKF.h"
#include "gtest/gtest.h"

using namespace std;

TEST(CalculateRMSETest, Zero1) {
  Tools tools; 

  vector < VectorXd > estimations; 
  vector < VectorXd > ground_truth; 

  for(int i=0; i<10; i++) 
  {
    VectorXd estimate(4);
    VectorXd single_ground_truth(4);

    estimate << 0.0, 0.0, 0.0, 0.0; 
    estimations.push_back(estimate);

    single_ground_truth << 0.0, 0.0, 0.0, 0.0; 
    ground_truth.push_back(single_ground_truth);
  }

  VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth); 

  EXPECT_EQ(RMSE.size(), 4); 
  for (int i = 0; i < 4; i++) 
  {
    EXPECT_EQ(RMSE(i), 0.0); 
  }
}

TEST(CalculateRMSETest, One1) {
  Tools tools; 

  vector < VectorXd > estimations; 
  vector < VectorXd > ground_truth; 

  for(int i=0; i<10; i++) 
  {
    VectorXd estimate(4);
    VectorXd single_ground_truth(4);

    estimate << 1.0, 1.0, 1.0, 1.0; 
    estimations.push_back(estimate);

    single_ground_truth << 1.0, 1.0, 1.0, 1.0; 
    ground_truth.push_back(single_ground_truth);
  }

  VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth); 

  EXPECT_EQ(RMSE.size(), 4); 
  for (int i = 0; i < 4; i++) 
  {
    EXPECT_EQ(RMSE(i), 0.0); 
  }
}

TEST(CalculateRMSETest, PresetVectors) {
  Tools tools; 

	vector<VectorXd> estimations;
	vector<VectorXd> ground_truth;

	//the input list of estimations
	VectorXd e(4);
	e << 1, 1, 0.2, 0.1;
	estimations.push_back(e);
	e << 2, 2, 0.3, 0.2;
	estimations.push_back(e);
	e << 3, 3, 0.4, 0.3;
	estimations.push_back(e);

	//the corresponding list of ground truth values
	VectorXd g(4);
	g << 1.1, 1.1, 0.3, 0.2;
	ground_truth.push_back(g);
	g << 2.1, 2.1, 0.4, 0.3;
	ground_truth.push_back(g);
	g << 3.1, 3.1, 0.5, 0.4;
	ground_truth.push_back(g);

  VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth); 

  EXPECT_EQ(RMSE.size(), 4); 
  for (int i = 0; i < 4; i++) 
  {
    float curr_val = RMSE(i);
    std::cout << "RMSE(" << i << "): " << curr_val << "\n";
    EXPECT_NEAR(RMSE(i), 0.1, 0.001); 
  }
}

TEST(CalculateJacobian, PresetVector) {
  Tools tools; 

	//predicted state  example
	//px = 1, py = 2, vx = 0.2, vy = 0.4
	VectorXd x_predicted(4);
	x_predicted << 1, 2, 0.2, 0.4;

  MatrixXd expected_Hj(3, 4);
  std::cout << "expected_Hj - rows: " << expected_Hj.rows() << ", cols: " << expected_Hj.cols() << "\n";
  expected_Hj << 0.447214, 0.894427, 0, 0, 
                  -0.4, 0.2, 0, 0,
                  0, 0, 0.447214, 0.894427;

	MatrixXd Hj = tools.CalculateJacobian(x_predicted);
  std::cout << "Hj - rows: " << Hj.rows() << ", cols: " << Hj.cols() << "\n";

  for(int row=0; row<expected_Hj.rows(); row++) 
  {
    for(int col=0; col<expected_Hj.cols(); col++) 
    {
      float actual_val = Hj(row, col);
      float expected_val = expected_Hj(row, col);
      EXPECT_NEAR(actual_val, expected_val, 0.0001); 
    }
  }
}