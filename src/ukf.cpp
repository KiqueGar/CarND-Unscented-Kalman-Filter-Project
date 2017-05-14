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
  [c] x_
  [d] P_
  [c] Xsig_pred
  [d] time_us_
  [m] std_a
  [m] sdt_yawdd_
  [s] std_laspx_
  [s] std_laspy_
  [s] std_radr_
  [s] std_radphi_
  [s] std_radrd_
  [c] weights
  [d] n_x_
  [d] n_aug_
  [d] lambda_
  [c] NIS_radar_
  [c] Nis_laser_

  */
  //Typical bicycle accel = .71 (Table 1)
  //https://www.researchgate.net/publication/223922575_Design_speeds_and_acceleration_characteristics_of_bicycle_traffic_for_use_in_planning_design_and_appraisal
  std_a_ = .355;
  //On downhill, max vel = 8.05 =v_tan
  //yawdd = v_tan^2 / min_rad;
  //min safe radius is 24m, according to http://www.dot.state.mn.us/bike/pdfs/manual/Chapter5.pdf
  //assuming 30km/h (~8m/s) (Table 5.3B)
  //yawwdd = 8.05*8.05/24 =2.7
  std_yawdd_ = 1.35;
  is_initialized_ = false;
  //elapsed time (us)
  time_us_= 0;
  //state size
  n_x_ = 5;
  //augmented state size
  n_aug_ = 7;
  //Spreading param 
  lambda_ = 3 - n_x_;
  Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_ +1);
  weights_ = VectorXd(2*n_aug_ +1);
  debugg_=false;

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
      //Uncertainity x (0.2m on posx&posy, 0.2m/s vel, 0.1 rad in yaw&yaw_dot)
      P_ << 0.2, 0, 0, 0, 0,
            0, 0.2, 0, 0, 0,
            0, 0, 0.2, 0, 0,
            0, 0, 0, .1, 0,
            0, 0, 0, 0, .1;
      //Set weights
      lambda_ = 3 - n_aug_;
      for(int i =1; i<2*n_aug_ +1; i++){
        weights_(i)=0.5/(lambda_+n_aug_);
      }
      weights_(0)=lambda_/(lambda_+n_aug_);

      if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_){
        float ro = meas_package.raw_measurements_(0);
        float phi = meas_package.raw_measurements_(1);
        float ro_dot = meas_package.raw_measurements_(2);
        x_(0) = ro*cos(phi);
        x_(1) = ro*sin(phi);
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
    if(debugg_){
      std::cout << "time:" <<  meas_package.timestamp_ << endl;
      std::cout << "Delta:" <<  delta_t << endl;
      std::cout << "Sensor:" <<  meas_package.sensor_type_ << endl;
      std::cout << "Actual state:" << endl << x_ << endl;
      std::cout << "Actual covariance:" << endl << P_ << endl;
    }
    
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
  lambda_ = 3 - n_x_;
  float lambda_term = sqrt(lambda_ + n_x_);
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
  lambda_term = sqrt(lambda_+n_aug_);
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
      float cte_a = v/yaw_dot;
      px_p= px + cte_a*(sin(yaw+ yaw_dot*delta_t) - sin(yaw));
      py_p = py + cte_a*(-cos(yaw + yaw_dot*delta_t) + cos(yaw));
      yaw_p = yaw + yaw_dot*delta_t;
    }
    v_p = v;
    yaw_dot_p = yaw_dot;

    //Add noise
    float delta_sq=delta_t*delta_t/2;
    px_p += delta_sq*cos(yaw)*nu_a;
    py_p += delta_sq*sin(yaw)*nu_a;
    v_p += delta_t*nu_a;
    yaw_p += delta_sq*nu_yawdd;
    yaw_dot_p += delta_t*nu_yawdd;

    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yaw_dot_p;

  }
  ///*** convert Sigma Points to mean and covariance ***///
  /*
  w(i)=(0.5)/(lamda + n_aug_point(i))
  w(1)=lambda/(lamda + n_aug_point(i))
  next state x = SUM(w(i)*X(i)), i:1..n_aug_point(i)
  next P = SUM(w(i)*(X_pred-x)*(X_pred-x)^T)
  */
  /*
  //Set weights
  for(int i =1; i<2*n_aug_ +1; i++){
    weights_(i)=0.5/(lambda_+n_aug_);
  }
  weights_(0)=lambda_/(lambda_+n_aug_);
*/
  //Predict state mean
  x_.fill(0);
  for(int i = 0; i<2*n_aug_ +1; i++){
    x_ += weights_(i)*Xsig_pred_.col(i);
  }
  //Predict covariance
  P_.fill(0);
  for (int i = 0; i< 2*n_aug_+1; i++){
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //Yaw normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
    //Yaw_dot normalization
    while (x_diff(4)> M_PI) x_diff(4)-=2.*M_PI;
    while (x_diff(4)<-M_PI) x_diff(4)+=2.*M_PI;
    P_ += weights_(i)*x_diff*x_diff.transpose();
  }
  if(debugg_){
    std::cout << "Predicted state covariance" << endl;
    std::cout << P_ << endl;
  }
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
  if(debugg_) std::cout << "Updating via LIDAR" << endl;
  int n_z = 2;
  VectorXd z = meas_package.raw_measurements_;
  MatrixXd Zsig = MatrixXd(n_z, 2*n_aug_ +1);
  MatrixXd S = MatrixXd(n_z, n_z);
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0);

  for(int i = 0; i< 2*n_aug_ +1; i++){
    Zsig(0,i) = Xsig_pred_(0,i);
    Zsig(1,i) = Xsig_pred_(1,i);
    z_pred += weights_(i)*Zsig.col(i);
  }
  
  if(debugg_) std::cout << "Zsigma generated" << endl;

  MatrixXd R = MatrixXd(n_z, n_z);
  R <<  std_laspx_*std_laspx_, 0,
        0, std_laspy_*std_laspy_;
  
  S.fill(0);
  for(int i = 0; i < 2*n_aug_ +1; i++){
    // Maybe this diff can be stored, used again in Tc
    VectorXd diff = Zsig.col(i)-z_pred;
    S += weights_(i)*diff*diff.transpose();
  }
  S += R;

  if(debugg_) std::cout << "Covariance and mean in Z_aug calculated" << endl;

  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0);
  
  if(debugg_) std::cout << "Updating state..." << endl;

  for(int i = 0; i< 2*n_aug_ +1 ; i++){
    VectorXd diff_x = Xsig_pred_.col(i) - x_;
    VectorXd diff_z = Zsig.col(i) - z_pred;

    Tc += weights_(i)*diff_x*diff_z.transpose();
  }
  if(debugg_) std::cout << "    Tc created" << endl;
  MatrixXd K = Tc*S.inverse();
  if(debugg_) std::cout << "    K created" << endl;
  VectorXd z_diff = z - z_pred;
  //angle normalization
  while (z_diff(1)> M_PI) z_diff(1) -= 2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1) += 2.*M_PI;
  x_ += K*z_diff;
  P_ -= K*S*K.transpose();
  if(debugg_) std::cout << "    State updated" << endl;
  if(debugg_) std::cout << "Update Done" << endl << endl;
  NIS_laser_= z_diff.transpose()*S.inverse()*z_diff;
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
  ///***Augmented points to sensor space***///
  if(debugg_) std::cout << "Updating via RADAR" << endl;
  int n_z = 3;
  VectorXd z = meas_package.raw_measurements_;
  MatrixXd Zsig = MatrixXd(n_z, 2*n_aug_ +1);
  MatrixXd S = MatrixXd(n_z, n_z);
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0);

  for (int i=0; i<1 + (2*n_aug_); i++){
    //std::cout << Xsig_pred_.col(i) << endl;
    double px = Xsig_pred_(0,i);
    double py = Xsig_pred_(1,i);
    double v = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);
    double yaw_d = Xsig_pred_(4,i);

    Zsig(0,i) = sqrt(px*px + py*py);
    Zsig(1,i) = atan2(py,px);
    Zsig(2,i) = (v*px*cos(yaw) + v*py*sin(yaw))/Zsig(0,i);
    //Calculate mean predicted measurement
    z_pred += weights_(i)*Zsig.col(i);
  }
  if(debugg_) std::cout << "Zsigma generated" << endl;
  
  ///***Predicted covariance***///
  
  MatrixXd R = MatrixXd(n_z,n_z);
  R <<  std_radr_*std_radr_, 0, 0,
        0, std_radphi_*std_radphi_, 0,
        0, 0, std_radrd_*std_radrd_;

  S.fill(0);
  for(int i=0; i<2*n_aug_ +1; i++){
    VectorXd diff = Zsig.col(i)-z_pred;
    //angle normalization
    while (diff(1)> M_PI) diff(1) -= 2.*M_PI;
    while (diff(1)<-M_PI) diff(1) += 2.*M_PI;
    S+=weights_(i)*diff*diff.transpose();
  }
  S += R;
  if(debugg_) std::cout << "Covariance and mean in Z_aug calculated" << endl;
  
  ///***State Update***//
  
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0);
  
  if(debugg_) std::cout << "Updating state..." << endl;
  for(int i =0; i<2*n_aug_ +1; i++){
    VectorXd diff_x = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (diff_x(3)> M_PI) diff_x(1) -= 2.*M_PI;
    while (diff_x(3)<-M_PI) diff_x(1) += 2.*M_PI;
    VectorXd diff_z = Zsig.col(i) - z_pred;
    //angle normalization
    while (diff_z(1)> M_PI) diff_z(1) -= 2.*M_PI;
    while (diff_z(1)<-M_PI) diff_z(1) += 2.*M_PI;
    Tc += weights_(i)*diff_x*diff_z.transpose();
  }
  if(debugg_) std::cout << "    Tc created" << endl;
  MatrixXd K = Tc*S.inverse();
  if(debugg_) std::cout << "    K created" << endl;
  VectorXd z_diff = z - z_pred;
  //angle normalization
  while (z_diff(1)> M_PI) z_diff(1) -= 2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1) += 2.*M_PI;
  x_ += K*z_diff;
  P_ -= K*S*K.transpose();
  if(debugg_) std::cout << "    State updated" << endl;
  if(debugg_) std::cout << "Update Done" << endl << endl;
  NIS_radar_ = z_diff.transpose()*S.inverse()*z_diff;
}
