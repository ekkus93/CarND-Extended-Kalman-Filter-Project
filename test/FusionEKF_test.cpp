#include <limits.h>
#include <iostream>
#include "FusionEKF.h"
#include "util.h"
#include "gtest/gtest.h"

TEST(FusionEKFTest, Lidar0) {
  LaserRadarDataReader dr = LaserRadarDataReader("/Users/phillipcchin/work/carnd/CarND-Extended-Kalman-Filter-Project/data/obj_pose-laser-radar-synthetic-input.txt");

  vector<string> parsedWords = dr.GetNextParsedLineByType("L");

  UTIL_LIDAR_STRUCT lidarData = dr.ParseLidarData(parsedWords);

  cout << "###FusionEKFTest, Lidar0\n";
  dr.OutputLidar(lidarData);
}
