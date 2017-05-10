#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  s = default
  m = modiffied
  d = deffined
  c = Calculated at later step

  [d] is_initialized_
  [s] use_laser_
  [s] use_radar_
  [ ] x_
  [d] P_
  [ ] Xsig_pred_
  [d] time_us
  [m] std_a
  [m] sdt_yawdd_
  [s] std_laspx_
  [s] std_laspy_
  [s] std_radr_
  [s] std_radphi_
  [s] std_radrd_
  [ ] weights
  [d] n_x_
  [d] n_aug_
  [d] lambda_
  [ ] NIS_radar_
  [ ] Nis_laser_

  */
  //Typical bicycle accel
  //https://www.researchgate.net/publication/223922575_Design_speeds_and_acceleration_characteristics_of_bicycle_traffic_for_use_in_planning_design_and_appraisal
  std_a_ = .355;
  std_yawdd_ = 5.52;
  is_initialized_ = false;
  //elapsed time (us)
  time_us_= 0;
  //state size
  n_x_ = 5;
  //augmented state size
  n_aug_ = 7;
  //Spreading param
  lambda = 3 - n_x_;
  //Sensor noise
  R_laser = MatrixXd(2,2);
  R_laser <<  std_laspx_*std_laspx_, 0,
              0, std_laspy_*std_laspy_;
  R_radar = MatrixXd(3,3);
  R_radar <<  std_radr_*std_radr_, 0, 0,
              0, std_radphi_*std_radphi_, 0,
              0, 0, std_radrd_*std_radrd_;

  //Uncertainity x (0.2m on posx&posy, 0.2m/s vel, 0.1 rad in yaw&yaw_dot)
  P_ = MatrixXd(n_x_, n_x_);
  P_ << 0.2, 0, 0, 0, 0,
        0, 0.2, 0, 0, 0,
        0, 0, 0.2, 0, 0,
        0, 0, 0, 0.1, 0,
        0, 0, 0, 0, 0.1;


}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  if(!is_initialized_){
    cout << "UKF: " << endl;
    //Starting state
    x_ = VectorXd(n_x_);
    x_.fill(1);
    if (measurement_pack.sensor_type_ == MeasurementPackage::Radar){
      float ro = measurement_pack.raw_measurements_(0);
      float phi = measurement_pack.raw_measurements_(1);
      float ro_dot = measurement_pack.raw_measurements_(2);
      
    }


  }

}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
}
