#include "FusionEKF.h"
#include "tools.h"
#include "sensor_data.h"
#include "Eigen/Dense"
#include <iostream>
#include <vector>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  ekf_ = KalmanFilter();

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);

  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

	noise_ax_ = 9;
	noise_ay_ = 9;

  DisplayData();
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::Init(const MeasurementPackage &measurement_pack)
{
   /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      Init_Radar(measurement_pack);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      Init_Lidar(measurement_pack);
    }
}

void FusionEKF::Init_Radar(const MeasurementPackage &measurement_pack)
{
  ekf_.x_ = VectorXd(4);
  ekf_.x_ << 1, 1, 0, 0;

  /**
  Convert radar from polar to cartesian coordinates and initialize state.
  */
  RadarData radarData = RadarData(measurement_pack);

  float x_measured, y_measured;
  radarData.GetXY(x_measured, y_measured);

  if (x_measured != 0 && y_measured != 0)
  {
    ekf_.x_(0) = x_measured;
    ekf_.x_(1) = y_measured;    
  } 

  previous_timestamp_ = measurement_pack.timestamp_;

  // done initializing, no need to predict or update
  is_initialized_ = true;  
}

void FusionEKF::Init_Lidar(const MeasurementPackage &measurement_pack)
{
  ekf_.x_ = VectorXd(4);
  ekf_.x_ << 1, 1, 0, 0;

  /**
  Initialize state.
  */
  LidarData lidarData = LidarData(measurement_pack);

  ekf_.x_(0) = lidarData.x_measured_;
  ekf_.x_(1) = lidarData.y_measured_;

  previous_timestamp_ = measurement_pack.timestamp_;

  // done initializing, no need to predict or update
  is_initialized_ = true;  
}

void FusionEKF::PredictAndUpdate_Radar(const MeasurementPackage &measurement_pack)
{
    ekf_.Predict();

    // Radar updates
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = R_radar_;

    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
}

void FusionEKF::PredictAndUpdate_Lidar(const MeasurementPackage &measurement_pack)
{
    ekf_.Predict();

    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;

    ekf_.Update(measurement_pack.raw_measurements_);
}

void FusionEKF::SetF_(float dt)
{
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;
}

void FusionEKF::SetQ_(float dt)
{
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  ekf_.Q_ << dt_4/4*noise_ax_, 0, dt_3/2*noise_ax_, 0,
			        0, dt_4/4*noise_ay_, 0, dt_3/2*noise_ay_,
			        dt_3/2*noise_ax_, 0, dt_2*noise_ax_, 0,
			        0, dt_3/2*noise_ay_, 0, dt_2*noise_ay_;
}

float FusionEKF::CalcDt(long long t0, long long t1)
{
  return (t1 - t0)/1000000.0;
}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    Init(measurement_pack);
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  float dt = CalcDt(previous_timestamp_, measurement_pack.timestamp_);

  previous_timestamp_ = measurement_pack.timestamp_;

  SetF_(dt);
  SetQ_(dt);

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    cout << "###RADAR\n";
    PredictAndUpdate_Radar(measurement_pack);
  } else {
    cout << "###LIDAR\n";   
    PredictAndUpdate_Lidar(measurement_pack);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}

void FusionEKF::DisplayData()
{
  cout << "R_laser_: \n" << R_laser_ << "\n";
  cout << "R_radar_: \n" << R_radar_ << "\n";
  cout << "H_laser_: \n" << H_laser_ << "\n";
  cout << "Hj_: \n" << Hj_ << "\n";
  cout << "noise_ax: \n" << noise_ax_ << "\n";  
  cout << "noise_ay: \n" << noise_ay_ << "\n"; 
  cout << "ekf_:\n";
  ekf_.DisplayData();
}

bool FusionEKF::GetIsInitialized()
{
  return is_initialized_;
}

long long FusionEKF::GetPreviousTimestamp()
{
  return previous_timestamp_;
}

Eigen::MatrixXd FusionEKF::GetRLaser()
{
  return R_laser_;
}

Eigen::MatrixXd FusionEKF::GetRRadar()
{
  return R_radar_;
}

Eigen::MatrixXd FusionEKF::GetHLaser()
{
  return H_laser_;
}

Eigen::MatrixXd FusionEKF::GetHj()
{
  return Hj_;
}

float FusionEKF::GetNoiseAx()
{
  return noise_ax_;
}

float FusionEKF::GetNoiseAy()
{
  return noise_ay_;
}
