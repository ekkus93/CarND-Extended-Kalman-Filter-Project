#ifndef SENSOR_DATA_H_
#define SENSOR_DATA_H_

#include "measurement_package.h"

class RadarData {
  public:
    RadarData();
    RadarData(const MeasurementPackage &measurement_pack);
    RadarData(string lineStr);
    virtual ~RadarData();

    MeasurementPackage ToMeasurementPackage();
    void GetXY(float &x, float &y);

    float rho_measured_;
    float phi_measured_;
    float rhodot_measured_;
    long timestamp_;
    float x_groundtruth_;

    float y_groundtruth_;
    float vx_groundtruth_;
    float vy_groundtruth_;
    float yaw_groundtruth_;
    float yawrate_groundtruth_;
};

class LidarData {
  public:
    LidarData();
    LidarData(const MeasurementPackage &measurement_pack);
    LidarData(string lineStr);
    virtual ~LidarData();

    MeasurementPackage ToMeasurementPackage();

    float x_measured_;
    float y_measured_;
    long timestamp_;
    float x_groundtruth_;
    float y_groundtruth_;

    float vx_groundtruth_;
    float vy_groundtruth_;
    float yaw_groundtruth_;
    float yawrate_groundtruth_;
};

#endif /* SENSOR_DATA_H_ */