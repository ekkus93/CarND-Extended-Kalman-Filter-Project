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