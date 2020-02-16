#include "ukf.h"

UKF::UKF() {
  use_laser_ = true;           // If this is false, laser measurements will be ignored (except during init)
  use_radar_ = true;           // If this is false, radar measurements will be ignored (except during init)
  x_ = Eigen::VectorXd(5);     // Initial state vector
  P_ = Eigen::MatrixXd(5, 5);  // Initial covariance matrix
  std_a_ = 30;                 // Process noise standard deviation longitudinal acceleration in m/s^2
  std_yawdd_ = 30;             // Process noise standard deviation yaw acceleration in rad/s^2

  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */
  std_laspx_ = 0.15;   // Laser measurement noise standard deviation position1 in m
  std_laspy_ = 0.15;   // Laser measurement noise standard deviation position2 in m
  std_radr_ = 0.3;     // Radar measurement noise standard deviation radius in m
  std_radphi_ = 0.03;  // Radar measurement noise standard deviation angle in rad
  std_radrd_ = 0.3;    // Radar measurement noise standard deviation radius change in m/s

  /**
   * End DO NOT MODIFY section for measurement noise values
   */

  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */
}

UKF::~UKF() = default;

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */
}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location.
   * Modify the state vector, x_. Predict sigma points, the state,
   * and the state covariance matrix.
   */
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief
   * about the object's position. Modify the state vector, x_, and
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief
   * about the object's position. Modify the state vector, x_, and
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
}
