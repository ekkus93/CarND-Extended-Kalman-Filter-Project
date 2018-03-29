#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
#include "tools.h"
#include "sensor_data.h"

class FusionEKF {
public:
  /**
  * Constructor.
  */
  FusionEKF();

  /**
  * Destructor.
  */
  virtual ~FusionEKF();

  void Init(const MeasurementPackage &measurement_pack);
  void Init_Radar(const MeasurementPackage &measurement_pack);
  void Init_Lidar(const MeasurementPackage &measurement_pack);

  void PredictAndUpdate_Radar(const MeasurementPackage &measurement_pack);
  void PredictAndUpdate_Lidar(const MeasurementPackage &measurement_pack);

  void SetF_(float dt);
  void SetQ_(float dt);
  float CalcDt(long long t0, long long t1);

  /**
  * Run the whole flow of the Kalman Filter from here.
  */
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  void DisplayData();

  bool GetIsInitialized();
  long long GetPreviousTimestamp();
  Eigen::MatrixXd GetRLaser();
  Eigen::MatrixXd GetRRadar();
  Eigen::MatrixXd GetHLaser();
  Eigen::MatrixXd GetHj();  
  float GetNoiseAx();
  float GetNoiseAy();

  /**
  * Kalman Filter update and prediction math lives in here.
  */
  KalmanFilter ekf_;

private:
  // check whether the tracking toolbox was initialized or not (first measurement)
  bool is_initialized_;

  // previous timestamp
  long long previous_timestamp_;

  // tool object used to compute Jacobian and RMSE
  Tools tools;
  Eigen::MatrixXd R_laser_;
  Eigen::MatrixXd R_radar_;
  Eigen::MatrixXd H_laser_;
  Eigen::MatrixXd Hj_;
  
	//acceleration noise components
	float noise_ax_;
	float noise_ay_;
};

#endif /* FusionEKF_H_ */
