#include <limits.h>
#include <iostream>
#include "FusionEKF.h"
#include "sensor_data.h"
#include "tools.h"
#include "util.h"
#include "gtest/gtest.h"

using namespace std;

TEST(FusionEKFTest, Init) {
  FusionEKF fEKF;

  EXPECT_FALSE(fEKF.GetIsInitialized());

  EXPECT_EQ(2, fEKF.FusionEKF::GetRLaser().rows());
  EXPECT_EQ(2, fEKF.FusionEKF::GetRLaser().cols());  

  EXPECT_EQ(3, fEKF.GetRRadar().rows());
  EXPECT_EQ(3, fEKF.GetRRadar().cols());    

  EXPECT_EQ(2, fEKF.GetHLaser().rows());
  EXPECT_EQ(4, fEKF.GetHLaser().cols());    

  EXPECT_EQ(3, fEKF.GetHj().rows());
  EXPECT_EQ(4, fEKF.GetHj().cols());    

  EXPECT_EQ(9, fEKF.GetNoiseAx());
  EXPECT_EQ(9, fEKF.GetNoiseAy());     
}

TEST(FusionEKFTest, Init_Lidar) {
  // TODO: change this so the path isn't hard coded.  
  LaserRadarDataReader dr = LaserRadarDataReader("/Users/phillipcchin/work/carnd/CarND-Extended-Kalman-Filter-Project/data/obj_pose-laser-radar-synthetic-input.txt");

  char lineStr[256];

  dr.GetLine(lineStr);
  while (lineStr[0] != 'L')
  {
    dr.GetLine(lineStr);
  }

  LidarData ld = LidarData(lineStr);
  MeasurementPackage mp = ld.ToMeasurementPackage();

  FusionEKF fEKF;

  // Init
  fEKF.Init_Lidar(mp);

  EXPECT_NEAR(3.122427e-01, fEKF.ekf_.x_(0), 0.001);
  EXPECT_NEAR(5.803398e-01, fEKF.ekf_.x_(1), 0.001);
  EXPECT_NEAR(0.0, fEKF.ekf_.x_(2), 0.001);
  EXPECT_NEAR(0.0, fEKF.ekf_.x_(3), 0.001);

  EXPECT_EQ(mp.timestamp_, fEKF.GetPreviousTimestamp());
  EXPECT_TRUE(fEKF.GetIsInitialized());
}

TEST(FusionEKFTest, ProcessMeasurement_Lidar) 
{
  vector<VectorXd> estimation_vec;
  vector<VectorXd> ground_truth_vec;  

  // TODO: change this so the path isn't hard coded. 
  LaserRadarDataReader dr = LaserRadarDataReader("/Users/phillipcchin/work/carnd/CarND-Extended-Kalman-Filter-Project/data/obj_pose-laser-radar-synthetic-input.txt");

  char lineStr[256];

  dr.GetLine(lineStr);
  while (lineStr[0] != 'L')
  {
    dr.GetLine(lineStr);
  }

  LidarData ld = LidarData(lineStr);
  MeasurementPackage mp = ld.ToMeasurementPackage();

  ground_truth_vec.push_back(ld.GetGroundTruth());

  FusionEKF fEKF;
  Tools tools;

  // Init
  fEKF.Init_Lidar(mp);
  estimation_vec.push_back(fEKF.ekf_.x_);

  for(int i=0; i<10; i++)
  {
    cout << "###i: " << i << "\n";
    // get next Lidar reading
    dr.GetLine(lineStr);
    while (lineStr[0] != 'L')
    {
      dr.GetLine(lineStr);
    }

    ld = LidarData(lineStr);
    mp = ld.ToMeasurementPackage();

    fEKF.ProcessMeasurement(mp);

    cout << "x_groundtruth_: " << ld.x_groundtruth_ << "\n";
    cout << "y_groundtruth_: " << ld.y_groundtruth_ << "\n";
    cout << "vx_groundtruth_: " << ld.vx_groundtruth_ << "\n";  
    cout << "vy_groundtruth_: " << ld.vy_groundtruth_ << "\n"; 

    ground_truth_vec.push_back(ld.GetGroundTruth());
    estimation_vec.push_back(fEKF.ekf_.x_);
  }

  VectorXd rmse = tools.CalculateRMSE(estimation_vec, ground_truth_vec);
  cout << "###RMSE: " << rmse << "\n";
}

TEST(FusionEKFTest, Init_Radar) {
  // TODO: change this so the path isn't hard coded.  
  LaserRadarDataReader dr = LaserRadarDataReader("/Users/phillipcchin/work/carnd/CarND-Extended-Kalman-Filter-Project/data/obj_pose-laser-radar-synthetic-input.txt");

  char lineStr[256];

  dr.GetLine(lineStr);
  while (lineStr[0] != 'R')
  {
    dr.GetLine(lineStr);
  }

  RadarData rd = RadarData(lineStr);
  MeasurementPackage mp = rd.ToMeasurementPackage();

  FusionEKF fEKF;

  // Init
  fEKF.Init_Radar(mp);

  float expected_val, actual_val;

  expected_val = rd.x_groundtruth_;
  actual_val = fEKF.ekf_.x_(0);
  cout << "###x_groundtruth_: " << rd.x_groundtruth_ << "\n";
  cout << "###actual_val: " << fEKF.ekf_.x_(0) << "\n";
  float err = calcErr(expected_val, actual_val) ;
  cout << "###err: " << err << "\n";
  EXPECT_TRUE(err <= 0.05);

  expected_val = rd.y_groundtruth_;
  actual_val = fEKF.ekf_.x_(1);  
  cout << "###y_groundtruth_: " << rd.y_groundtruth_ << "\n";
  cout << "###actual_val: " << fEKF.ekf_.x_(1) << "\n";
  err = calcErr(expected_val, actual_val) ;
  cout << "###err: " << err << "\n";
  EXPECT_TRUE(err <= 0.11);

  EXPECT_NEAR(0.0, fEKF.ekf_.x_(2), 0.001);
  EXPECT_NEAR(0.0, fEKF.ekf_.x_(3), 0.001);

  EXPECT_EQ(mp.timestamp_, fEKF.GetPreviousTimestamp());
  EXPECT_TRUE(fEKF.GetIsInitialized());
}
