#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;

UKF::UKF() {
  
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
 
 
  is_initialized_  = false;

  bool use_laser_  = true;
  bool use_radar_  = true;
  
  n_x_    = 5;             // State dimension
  n_aug_  = 7;             // Augmented state dimension
  lambda_ = 3 - n_x_;      // Sigma point spreading parameter
  
  time_us_ = 0.0;
  
  std_a_     = 1.5;         // Process noise standard deviation longitudinal acceleration
  std_yawdd_ = 2.0;           // Process noise standard deviation yaw acceleration
  

  // initial state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  x_ = VectorXd(n_x_);

  // initial state covariance matrix
  P_ = MatrixXd(n_x_, n_x_);

  // predicted sigma points matrix
  Xsig_pred_ = Eigen::MatrixXd(n_x_, 1 + 2 * n_aug_);

  // Weights of sigma points
  weights_ = Eigen::VectorXd(1 + 2 * n_aug_);
  weights_(0) = lambda_/(lambda_+n_aug_);
  for (int i=1; i < 1 + 2 * n_aug_; ++i) 
  {
    weights_(i) = 0.5/(n_aug_+lambda_);
  }


}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
 
  // initialization by using recent available sensor 
  if (is_initialized_ == false)
  {
    //cout<<"Init..."<<endl;
    
    if ((meas_package.sensor_type_ == MeasurementPackage::LASER))
    {
      double p_x = meas_package.raw_measurements_[0];
      double p_y = meas_package.raw_measurements_[1];
      
      x_ << p_x, p_y, 0, 0, 0;
      P_ << pow(std_laspx_,2),0,0,0,0,
            0,pow(std_laspy_,2),0,0,0,
            0,0,1,0,0,
            0,0,0,1,0,
            0,0,0,0,1;
    }
    
    if ((meas_package.sensor_type_ == MeasurementPackage::RADAR))
    {
      double p_x = meas_package.raw_measurements_[0] * cos(meas_package.raw_measurements_[1]);
      double p_y = meas_package.raw_measurements_[0] * sin(meas_package.raw_measurements_[1]);

      double v_x = meas_package.raw_measurements_[2] * cos(meas_package.raw_measurements_[1]);
      double v_y = meas_package.raw_measurements_[2] * sin(meas_package.raw_measurements_[1]);

      double v = sqrt(pow(v_x,2)+pow(v_y,2));  

      x_ << p_x, p_y, v, meas_package.raw_measurements_[1], meas_package.raw_measurements_[2];
      P_ << pow(std_radr_,2),0,0,0,0,
            0,pow(std_radr_,2),0,0,0,
            0,0,pow(std_radrd_,2),0,0,
            0,0,0,pow(std_radphi_,2),0,
            0,0,0,0,pow(std_radrd_,2);
    }

    is_initialized_ = true;

    time_us_ = meas_package.timestamp_;
    
    //cout<<"Init completed!"<<endl;
    
    return;
  }

  double dt = (double)((meas_package.timestamp_ - time_us_) / 1000000.0);
  time_us_ = meas_package.timestamp_;

  Prediction(dt);

  if ((meas_package.sensor_type_ == MeasurementPackage::LASER))
  { 
    UpdateLidar(meas_package);
  }
  
  if ((meas_package.sensor_type_ == MeasurementPackage::RADAR))
  {
    UpdateRadar(meas_package);
  }
  
  return;
}

void UKF::Prediction(double delta_t) {
  /*
  * Estimate the object's location. 
  * Modify the state vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  
  // sigma points with augumentation
  VectorXd x_aug = VectorXd(n_aug_);                       // create augmented mean vector
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);               // create augmented state covariance
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 1 + 2 * n_aug_);    // create sigma point matrix
 
  // create augmented mean state
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  // create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;

  // create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  // create augmented sigma points
  Xsig_aug.col(0) = x_aug;
  for (int i = 0; i< n_aug_; ++i) 
  {
    Xsig_aug.col(i+1)        = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
  }
  
  // predict sigma points
  for (int i = 0; i < 1+ 2*n_aug_; ++i) 
  {
    // extract values for better readability
    double p_x  = Xsig_aug(0,i);
    double p_y  = Xsig_aug(1,i);
    double v    = Xsig_aug(2,i);
    double yaw  = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    // predicted state values
    double px_p, py_p;

    // avoid division by zero
    if (fabs(yawd) > 0.001) 
    {
        px_p = p_x + v/yawd * (sin(yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * (cos(yaw) - cos(yaw+yawd*delta_t));
    } 
    else 
    {
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p    = v;
    double yaw_p  = yaw + yawd*delta_t;
    double yawd_p = yawd;

    // add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p  = v_p  + nu_a*delta_t;

    yaw_p = yaw_p   + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    // write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }

   
  VectorXd x = VectorXd(n_x_);              // create vector for predicted state
  MatrixXd P = MatrixXd(n_x_, n_x_);        // create covariance matrix for predictio


  // predicted state mean
  x.fill(0.0);
  for (int i = 0; i < 1+ 2 * n_aug_; ++i) 
  { 
    x = x + weights_(i) * Xsig_pred_.col(i);
  }

  // predicted state covariance matrix
  P.fill(0.0);
  for (int i = 0; i < 1 + 2 * n_aug_; ++i) 
  { 
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x;
    // angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P = P + weights_(i) * x_diff * x_diff.transpose() ;
  }
  
  x_ = x;
  P_ = P;

}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
   /*
   * Use lidar data to update the belief about the object's position. 
   * Modify the state vector, x_, and covariance, P_.
   */
  
  int n_z = 2;
  
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);      // create matrix for sigma points in measurement space
  VectorXd z_pred = VectorXd(n_z);                    // mean predicted measurement
  MatrixXd S = MatrixXd(n_z, n_z);                    // measurement covariance matrix S

  // transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) 
  {  
    // extract values for better readability
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v   = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    // measurement model
    Zsig(0,i) = p_x;                       // x
    Zsig(1,i) = p_y;                       // y

  }

  // mean predicted measurement
  z_pred.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) 
  {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  // innovation covariance matrix S
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) 
  {
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  // add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R <<  pow(std_laspx_,2),0,
        0,pow(std_laspy_,2);
  S = S + R;



  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  // calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) 
  {  
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  // Kalman gain K;
  MatrixXd K = Tc * S.inverse();
  VectorXd z = meas_package.raw_measurements_;

  // residual
  VectorXd z_diff = z - z_pred;

  // angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  // update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();

}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
   /*
   * Use radar data to update the belief about the object's position. 
   * Modify the state vector, x_, and covariance, P_.
   */
   
  int n_z = 3;
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);      // create matrix for sigma points in measurement space
  VectorXd z_pred = VectorXd(n_z);                    // mean predicted measurement
  MatrixXd S = MatrixXd(n_z, n_z);                    // measurement covariance matrix S

  // transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) 
  {  
    // extract values for better readability
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v   = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                       // r
    Zsig(1,i) = atan2(p_y,p_x);                                // phi
    Zsig(2,i) = (p_x*v1 + p_y*v2) / sqrt(p_x*p_x + p_y*p_y);   // r_dot
  }

  // mean predicted measurement
  z_pred.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) 
  {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  // innovation covariance matrix S
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) 
  {
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  // add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R <<  std_radr_*std_radr_, 0, 0,
        0, std_radphi_*std_radphi_, 0,
        0, 0, std_radrd_*std_radrd_;
  S = S + R;



  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  // calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) 
  {  
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    
    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  // Kalman gain K;
  MatrixXd K = Tc * S.inverse();
  VectorXd z = meas_package.raw_measurements_;

  // residual
  VectorXd z_diff = z - z_pred;

  // angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  // update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();

  
}