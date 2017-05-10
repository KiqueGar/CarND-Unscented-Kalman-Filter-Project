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
  [d] time_us_
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
  lambda_ = 3 - n_x_;
  

  //Uncertainity x (0.2m on posx&posy, 0.2m/s vel, 0.1 rad in yaw&yaw_dot)
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
  if((meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_)||
    (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_)){
    if(!is_initialized_){
      //cout << "UKF: " << endl;
      //Starting state
      x_.fill(0);
      time_us_= meas_package.timestamp_;
      if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_){
        float ro = meas_package.raw_measurements_(0);
        float phi = meas_package.raw_measurements_(1);
        float ro_dot = meas_package.raw_measurements_(2);
        x_(0) = meas_package.raw_measurements_(0)*cos(phi);
        x_(1) = meas_package.raw_measurements_(0)*sin(phi);
      }
      else if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_){
        x_(0) = meas_package.raw_measurements_(0);
        x_(1) = meas_package.raw_measurements_(1);
      }
      //Finish Initialization
      is_initialized_ = true;
      return;
    }
  
  /**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
    float delta_t = (meas_package.timestamp_ - time_us_)/1000000.0;
    time_us_ = meas_package.timestamp_;
    Prediction(delta_t);

    if (meas_package.sensor_type_==MeasurementPackage::RADAR){
      UpdateRadar(meas_package);
    }
    else if (meas_package.sensor_type_==MeasurementPackage::LASER){
      UpdateLidar(meas_package);
    }
  }
}


void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  ///*** Generate Sigma Points ***///
  //Xsig = [x, x+root((lamda+n_x)*P), x-root((lamda+n_x)*P)]
  MatrixXd Xsig = MatrixXd (n_x_, 2*n_x_ +1);
  Xsig.fill(0);
  MatrixXd A = P_.llt().matrixL();
  double lambda_term = pow(lambda_+n_x_,0.5);
  MatrixXd pos_root = lambda_term*A;
  MatrixXd neg_root = -pos_root;
  Xsig.block<5,5>(0,1) = pos_root;
  Xsig.block<5,5>(0,6) = neg_root;
  for(int i = 0; i< 2*n_x_ +1; i++){
    Xsig.col(i)+=x_;
  }
  ///*** Augment Sigma Points ***///
  //Augmented state
  VectorXd x_aug= VectorXd(n_aug_);
  x_aug.fill(0);
  x_aug.head(5)=x_;
  //Augmented covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  P_aug.fill(0);
  P_aug.topLeftCorner(5,5)=P_;
  P_aug.bottomRightCorner(2,2) << std_a_*std_a_, 0,
                                  0, std_yawdd_*std_yawdd_;
  lambda_ = 3- n_aug_;

  //Matrix augmented points
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2*n_aug_ +1);
  Xsig_aug.fill(0);
  lambda_term = pow(lambda_+n_aug_,0.5);
  MatrixXd A_aug = P_aug.llt().matrixL();
  MatrixXd pos_root_aug = lambda_term*A_aug;
  MatrixXd neg_root_aug = - pos_root_aug;
  Xsig_aug.block<7,7>(0,1) = pos_root_aug;
  Xsig_aug.block<7,7>(0,8) = neg_root_aug;
  for (int i= 0; i< 2*n_aug_ +1; i++){
    Xsig_aug.col(i)+=x_aug;
  }
  ///*** Estimate sigma points ***///
  //New state x = differential*delta_t + process noise
  /*differential =  (v/yaw_dot)(sin(yaw+yaw_dot*delta_t)-sin(yaw))
                    (v/yaw_dot)(-cos(yaw+yaw_dot*delta_t)+cos(yaw))
                    0
                    yaw_dot*delta_t
                    0
  process noise = 0.5(delta_t^2)cos(yaw)*nu_a
                  0.5(delta_t^2)sin(yaw)*nu_a
                  delta_t*nu_a
                  0.5(delta_t^2)nu_yawdd
                  delta_t*nu_yawdd
  */
  for(int i = 0; i< 1+ (2*n_aug_); i++){
    double px = Xsig_aug(0,i);
    double py = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yaw_dot = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    float px_p, py_p, v_p, yaw_p, yaw_dot_p;

    //Avoid cero division
    if(fabs(yaw_dot)<.001){
      px_p = px + v*cos(yaw)*delta_t;
      py_p = py + v*sin(yaw)*delta_t;
      yaw_p = yaw;
    }
    else{
      
    }

  }
  ///*** convert Sigma Points to mean and covariance ***///

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
  //Sensor noise (Move)
  /*
  R_laser = MatrixXd(2,2);
  R_laser <<  std_laspx_*std_laspx_, 0,
              0, std_laspy_*std_laspy_;
  R_radar = MatrixXd(3,3);
  R_radar <<  std_radr_*std_radr_, 0, 0,
              0, std_radphi_*std_radphi_, 0,
              0, 0, std_radrd_*std_radrd_;
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
