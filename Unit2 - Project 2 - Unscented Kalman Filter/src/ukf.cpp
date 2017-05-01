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
  std_a_ = 1.5;   // was 30

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.5;      // was 30

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
  
  ///* State dimension
  n_x_ = 5;
  
  ///* State dimension in radar space
  n_z_ = 3;
    
  ///* Augmented state dimension
  n_aug_ = 7;
    
  ///* Sigma point spreading parameter
  lambda_ = 3 - n_aug_;
  
  ///* the current NIS for radar
  //NIS_radar_;
    
  ///* the current NIS for laser
  //NIS_laser_;
  
  ///* predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
  
  //matrix for sigma points in measurement space
  Zsig_ = MatrixXd(n_z_, 2 * n_aug_ + 1);
  
  //mean predicted measurement in measurement space
  z_pred_ = VectorXd(n_z_);
  
  ///* Weights of sigma points
  weights_ = VectorXd(2 * n_aug_ + 1);
  
  //measurement covariance matrix S
  S_ = MatrixXd::Zero(n_z_,n_z_);
  
  //initialize weights
  weights_(0) = double(lambda_ / (lambda_ + n_aug_));
  for (int i=1; i<(2 * n_aug_ + 1); i++) {
    weights_(i) = double(1 / (2 * (lambda_ + n_aug_)));
  }

  is_initialized_=false;
  
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
  
  Takes measurements and initialize the state using the first measurement - depending on Lidar or radar.
  
  */
  
//long long previous_timestamp_;
  
  if (!is_initialized_){
    //Need to initialize previous time stamp, state, covariance matrix
    
    // Initialize previous time stamp
    time_us_ = meas_package.timestamp_;
    
    //Initialization will vary based on the nature of the first data (Lidar or Radar)
    
    //Let's initialize when the initial data is for a Lidar (in that case cartesian model)
    if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
      x_(0) = meas_package.raw_measurements_[0];
      x_(1) = meas_package.raw_measurements_[1];
      
    } else if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
      // Need to convert polar coordinates into cartesian coordinates
      
      //(rho, phi, rho_dot) -> (rho*cos(phi),rho*sin(phi))
      double rho = meas_package.raw_measurements_[0];
      double phi = meas_package.raw_measurements_[1];
      //double rho_dot = meas_package.raw_measurements_[2];
      
      //initialize the position
      x_(0) = rho*cos(phi);
      x_(1) = rho*sin(phi);
      
    }
    
    // we need to initialize the other state variables (velocity, yaw and yaw rate)
    // I don't believe we have enough information to compute an initial velocity
    
    x_(2) = 0;
    x_(3) = 0;
    x_(4) = 0;
    
    // initialize the covariance matrix P_; we know it's a symetrical matrix with values on the diagonal only
    P_ <<  0.0225, 0, 0, 0, 0,
    0, 0.0225, 0, 0, 0,
    0, 0, 0.6, 0, 0,
    0, 0, 0, 1.15, 0,
    0, 0, 0, 0, .15;
    
    // set is_initialized to TRUE since we have initialize our state & covariance matrix
    is_initialized_ = true;
  } else {
    float delta_time = (meas_package.timestamp_ - time_us_) / 1000000.0; // delta time expressed in seconds
    time_us_ = meas_package.timestamp_;
    
    std::cout << "Delta Time:" << delta_time << std::endl;
    
    //initialization of sigma points matrices
    Xsig_pred_.fill(0.0);
  
    // Prediction step
    while (delta_time > 0.1)
    {
      const double dt = 0.05;
      Prediction(dt);
      delta_time -= dt;
    }
    Prediction(delta_time);
    
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
      // Radar updates
      PredictionRadarSpace();
      UpdateRadar(meas_package);
      
    } else {
      // Laser updates
        if (use_laser_)
            UpdateLidar(meas_package);
    }
  
  std::cout << x_ << std::endl;
  
  }
}

/**
 * Normalize angle to keep it within -PI and +PI. @paramter {double} is the angle provided in radian
 * Method returns normalized angle
 */

double UKF::NormalizeAngle(double angle) {

  //angle normalization -- add or remove 2 PIs if the angle is too large
  
  if (angle > M_PI)
    angle = fmod(angle - M_PI, 2*M_PI) - M_PI;
  if (angle < -M_PI)
    angle = fmod(angle + M_PI, 2*M_PI) + M_PI;
  
  return angle;
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
  
  // Step-1 - Generate sigma points for the augmented state (conditions+noise)
  
  //Initialize augmented variables
  
  //create augmented mean vector
  VectorXd x_aug = VectorXd::Zero(n_aug_);
  
  //create augmented state covariance
  MatrixXd P_aug = MatrixXd::Zero(n_aug_, n_aug_);
  
  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd::Zero(n_aug_, 2 * n_aug_ + 1);
  
  //create augmented mean state
  x_aug << x_, 0, 0;
  
  //create augmented covariance matrix
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  
  //create Q matrix of dimension n_aug-n_x
  MatrixXd Q = MatrixXd(2,2);
  Q << pow(std_a_,2), 0, 0, pow(std_yawdd_,2);
  
  //add Q to bottom right of P_aug
  P_aug.bottomRightCorner(2,2) = Q;
  
  //create square root matrix of matrix P_aug
  MatrixXd A = P_aug.llt().matrixL();
  
  std::cout << P_aug << endl;
  std::cout << A << endl;
  
  //create augmented sigma points
  
  //calculate sigma points of Matrix P_aug...
  //calculate the square root of (lambda+n_a)
  double lambda_sqrt=sqrt(lambda_ + n_aug_);
  
  //set sigma points as columns of matrix Xsig
  Xsig_aug.col(0) = x_aug;
  
  std::cout << x_aug << endl;
  
  for (int i=0; i< n_aug_; i++) {
    Xsig_aug.col(i+1)=(x_aug + lambda_sqrt * A.col(i));
    Xsig_aug.col(n_aug_+1+i)=(x_aug - lambda_sqrt * A.col(i));
  }
  
  //display augmented sigma points
  std::cout << "Xsig_aug = " << std::endl << Xsig_aug << std::endl;
  
  // Step-2 - Pass Sigma points to the Predict function
  
  double p_x;
  double p_y;
  double velocity;
  double yaw;
  double yaw_rate;
  double noise_long_acc;
  double noise_yaw_acc;
  
  //we create the prediction vector
  VectorXd pred_x(n_x_);
  
  //we create a loop to process each sigma point
  for (int i=0; i<2*n_aug_+1; i++) {
    
    //we initialize our variables for each sigma point
    p_x = Xsig_aug(0,i);
    p_y = Xsig_aug(1,i);
    velocity = Xsig_aug(2,i);
    yaw = Xsig_aug(3,i);
    yaw_rate = Xsig_aug(4,i);
    noise_long_acc = Xsig_aug(5,i);
    noise_yaw_acc = Xsig_aug(6,i);
    
    //we compute the predictions using each sigma point coordinate
    //we consider if yaw_rate is close to 0 or not to avoid division by 0 case
    
    if (fabs(yaw_rate) > 0.001) {
      pred_x(0) = p_x + (velocity/yaw_rate)*(sin(yaw+yaw_rate*delta_t)-sin(yaw));
      pred_x(1) = p_y + (velocity/yaw_rate)*(-cos(yaw+yaw_rate*delta_t)+cos(yaw));
    }else {
      pred_x(0) = p_x + velocity*cos(yaw)*delta_t;
      pred_x(1) = p_y + velocity*sin(yaw)*delta_t;
    }
    
    pred_x(2) = velocity;
    pred_x(3) = yaw + yaw_rate*delta_t;
    pred_x(4) = yaw_rate;
    
    //we still need to add the noise acceleration component
    pred_x(0) = pred_x(0) + 0.5*pow(delta_t,2)*cos(yaw)*noise_long_acc;
    pred_x(1) = pred_x(1) + 0.5*pow(delta_t,2)*sin(yaw)*noise_long_acc;
    pred_x(2) = pred_x(2) + delta_t*noise_long_acc;
    pred_x(3) = pred_x(3) + 0.5*pow(delta_t,2)*noise_yaw_acc;
    pred_x(4) = pred_x(4) + delta_t*noise_yaw_acc;
    
    //write predicted sigma points into right column
    Xsig_pred_.col(i) = pred_x;
    
  }
  
  //print Xsig_pred result
  std::cout << "Xsig_pred = " << std::endl << Xsig_pred_ << std::endl;
  
  // Step-3 - Predict Mean and Covariance
  
  //predict state mean. We can use a simple matrix product.
  x_ = Xsig_pred_ * weights_;
  
  P_.fill(0.0);
  
  //predict state covariance matrix (tried with matrix ops but couldn't make it work)
  for (int i=0; i<(2 * n_aug_ +1); i++) {
    
    // state difference

    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    std::cout << x_diff << std::endl;
    
    x_diff(3)=NormalizeAngle(x_diff(3));
    
    P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
    std::cout << P_ << std::endl;
    
  }
  
  //print prediction results
  std::cout << "Predicted state" << std::endl;
  std::cout << x_ << std::endl;
  std::cout << "Predicted covariance matrix" << std::endl;
  std::cout << P_ << std::endl;
  
}

/**
 * Transform the state and the state covariance matrix into the Radar space.
 *
 */


void UKF::PredictionRadarSpace(void) {
  
  VectorXd pred_sigma_point = VectorXd::Zero(n_z_);
  
  Zsig_.fill(0.0);
  
  //transform sigma points into measurement space
  for(int i=0; i<2 * n_aug_ + 1; i++) {
    
    //rho comversion to measurement space
    pred_sigma_point(0) = sqrt(pow(Xsig_pred_(0,i),2)+pow(Xsig_pred_(1,i),2));
    
    //phi conversion to measurement space
    pred_sigma_point(1) = atan2(Xsig_pred_(1,i), Xsig_pred_(0,i));
    
    //rho change conversion to measurement space
    //need to check we don't divide by 0 or close to 0.
    if (pred_sigma_point(0) >= 0.0001)
      pred_sigma_point(2) = Xsig_pred_(2,i)*(Xsig_pred_(0,i)*cos(Xsig_pred_(3, i))+Xsig_pred_(1,i)*sin(Xsig_pred_(3,i)))/pred_sigma_point(0);
    else
      pred_sigma_point(2) = 0;
    
    //add pred_sigma_point vector to pred sigma point matrix in measurement space
    Zsig_.col(i)=pred_sigma_point;
  }
  // std::cout << Zsig << std::endl;
  
  //calculate mean predicted measurement
  z_pred_ = Zsig_*weights_;
  
  //calculate measurement covariance matrix S
  S_.fill(0.0);
  
  //predict state covariance matrix in measurement space
  for (int i=0; i<(2 * n_aug_ +1); i++) {
    // state difference
    VectorXd z_diff = Zsig_.col(i) - z_pred_;
    
    //angle normalization -- add or remove 2 PIs if the angle is too large
    z_diff(1) = NormalizeAngle(z_diff(1));
    
    S_ = S_ + weights_(i) * z_diff * z_diff.transpose() ;
    
  }
  
  MatrixXd R = MatrixXd(n_z_, n_z_);
  R <<    pow(std_radr_, 2), 0, 0,
          0, pow(std_radphi_, 2), 0,
          0, 0, pow(std_radrd_, 2);
  
  // Covariances get added
  S_ = S_ + R;
  
  //print result
  std::cout << "z_pred: " << std::endl << z_pred_ << std::endl;
  std::cout << "S: " << std::endl << S_ << std::endl;
  
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
  
  //We can use the regular Kalman Filter methodology because LIDAR is linear
  
  MatrixXd H_ = MatrixXd(2,5);
  MatrixXd R_ = MatrixXd(2,2);
  
  // measurement update matrix; the only info we get from LIDAR is positions (px, py), so the other items are 0.
  H_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0;
  
  //measurement covariance matrix - laser (usually provided by manufacturer)
  R_ <<   0.0225, 0,
          0, 0.0225;
  
  //initialize vector z based on latest measurement
  VectorXd z = VectorXd(2);
  
  z <<  meas_package.raw_measurements_(0),
        meas_package.raw_measurements_(1);
  
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
  
  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
  
  //LIDAR NIS computation
  NIS_laser_ = y.transpose()*Si*y;

  
  //print result
  std::cout << "Updated state x (after LIDAR data): " << std::endl << x_ << std::endl;
  std::cout << "Updated state covariance P (after LIDAR data): " << std::endl << P_ << std::endl;
  std::cout << "NIS (Lidar)" << std::endl << NIS_laser_ << std::endl;

}

/**
 * the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  
  VectorXd z = VectorXd(n_z_);
  z <<
  meas_package.raw_measurements_(0),    //rho in m
  meas_package.raw_measurements_(1),    //phi in rad
  meas_package.raw_measurements_(2);     //rho_dot in m/s
  
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z_);
  
  //initialize cross correlation matrix Tc
  Tc.fill(0.0);
  
  VectorXd x_diff = VectorXd(n_x_);
  VectorXd z_diff = VectorXd(n_z_);
  
  //calculate cross correlation matrix
  for (int i=0; i<2 * n_aug_ + 1; i++) {
    
    // state differences in both spaces
    x_diff = Xsig_pred_.col(i) - x_;
    z_diff = Zsig_.col(i) - z_pred_;
    
    //angle normalization -- add or remove 2 PIs if the angle is too large
    x_diff(3) = NormalizeAngle(x_diff(3));
    z_diff(1) = NormalizeAngle(z_diff(1));
    
    Tc = Tc + weights_(i) * x_diff * z_diff.transpose() ;
    
  }
  
  //calculate Kalman gain K;
  MatrixXd K = Tc * S_.inverse();
  
  //update state mean and covariance matrix
  x_ = x_ + K * (z - z_pred_);
  
  MatrixXd Ktrans = K.transpose();
  P_ = P_ - K * S_ * Ktrans;
  
  //Radar NIS computation
  z_diff = z - z_pred_;
  z_diff(1) = NormalizeAngle(z_diff(1));
  NIS_radar_ = z_diff.transpose()*S_.inverse()*z_diff;
  
  //print result
  std::cout << "Updated state x: (after RADAR data)" << std::endl << x_ << std::endl;
  std::cout << "Updated state covariance P: (after RADAR data)" << std::endl << P_ << std::endl;
  std::cout << "NIS (Radar)" << std::endl << NIS_radar_ << std::endl;

  
}
