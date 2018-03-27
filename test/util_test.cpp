#include <iostream>
#include "util.h"
#include "gtest/gtest.h"

using namespace std;

TEST(LaserRadarDataReaderTest, ReadDataFile) {
  LaserRadarDataReader dr = LaserRadarDataReader("/Users/phillipcchin/work/carnd/CarND-Extended-Kalman-Filter-Project/data/obj_pose-laser-radar-synthetic-input.txt");

  vector<string> parsedWords = dr.GetParsedLine();

  for(int i=0; i < 4 && parsedWords.size() > 0; i++)
  {
    if (parsedWords[0] == "L")
    {
      cout << "###" << i << ": Lidar\n";
      UTIL_LIDAR_STRUCT lidarData = dr.ParseLidarData(parsedWords);
      dr.OutputLidar(lidarData);
    }
    else
    {
      cout << "###" << i << ": Radar\n";
      UTIL_RADAR_STRUCT radarData = dr.ParseRadarData(parsedWords);
      dr.OutputRadar(radarData);
    }

    parsedWords = dr.GetParsedLine();
  }
  
  dr.Close();
}