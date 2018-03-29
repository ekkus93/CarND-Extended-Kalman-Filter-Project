#include <limits.h> 
#include <iostream>
#include "sensor_data.h"
#include "util.h"
#include "measurement_package.h"
#include "gtest/gtest.h"

using namespace std;

TEST(RadarDataTest, LineStr)
{
  // TODO: change this so the path isn't hard coded.  
  LaserRadarDataReader dr = LaserRadarDataReader("/Users/phillipcchin/work/carnd/CarND-Extended-Kalman-Filter-Project/data/obj_pose-laser-radar-synthetic-input.txt");

  char lineStr[256];

  dr.GetLine(lineStr);
  while (lineStr[0] != 'R')
  {
    dr.GetLine(lineStr);
  }

  RadarData rd = RadarData(lineStr);

  EXPECT_EQ(1477010443050000, rd.timestamp_);
  EXPECT_NEAR(1.014892e+00, rd.rho_measured_, 0.0001);
  EXPECT_NEAR(5.543292e-01, rd.phi_measured_, 0.0001);
  EXPECT_NEAR(4.892807e+00, rd.rhodot_measured_, 0.0001);

  dr.Close();
}

TEST(RadarDataTest, ToMeasurementPackage)
{
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

  EXPECT_EQ(MeasurementPackage::RADAR, mp.sensor_type_);
  EXPECT_EQ(1477010443050000, mp.timestamp_);
  EXPECT_NEAR(1.014892e+00, mp.raw_measurements_(0), 0.0001);
  EXPECT_NEAR(5.543292e-01, mp.raw_measurements_(1), 0.0001);
  EXPECT_NEAR(4.892807e+00, mp.raw_measurements_(2), 0.0001);  

  dr.Close();
}

TEST(RadarDataTest, GetXY)
{
  
}

TEST(LidarDataTest, LineStr)
{
  // TODO: change this so the path isn't hard coded.  
  LaserRadarDataReader dr = LaserRadarDataReader("/Users/phillipcchin/work/carnd/CarND-Extended-Kalman-Filter-Project/data/obj_pose-laser-radar-synthetic-input.txt");

  char lineStr[256];

  dr.GetLine(lineStr);
  while (lineStr[0] != 'L')
  {
    dr.GetLine(lineStr);
  }

  LidarData ld = LidarData(lineStr);

  EXPECT_EQ(1477010443000000, ld.timestamp_);
  EXPECT_NEAR(3.122427e-01, ld.x_measured_, 0.0001);
  EXPECT_NEAR(5.803398e-01, ld.y_measured_, 0.0001);

  dr.Close();
}

TEST(LidarDataTest, ToMeasurementPackage)
{
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

  EXPECT_EQ(MeasurementPackage::LASER, mp.sensor_type_);
  EXPECT_EQ(1477010443000000, mp.timestamp_);
  EXPECT_NEAR(3.122427e-01, mp.raw_measurements_(0), 0.0001);
  EXPECT_NEAR(5.803398e-01, mp.raw_measurements_(1), 0.0001);

  dr.Close();
}