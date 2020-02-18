#ifndef UNSCENTED_KALMAN_FILTER_UKF_H
#define UNSCENTED_KALMAN_FILTER_UKF_H

#include <utility>

#include "Eigen/Dense"
#include "measurement_package.h"

class UKF {
 public:
  UKF();
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage& meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);

  /**
   * Generates sigma points
   * @return Eigen::MatrixXd Augmented sigma points matrix
   */
  void AugmentedSigmaPoints(Eigen::MatrixXd& Xsig_aug);

  Eigen::VectorXd GetState() { return x_; }

  void SetState(Eigen::VectorXd x) { x_ = std::move(x); }

 private:
  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param measurement_package The measurement at k+1
   */
  void UpdateLidar(MeasurementPackage& measurement_package);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param measurement_package The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage& measurement_package);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void SigmaPointsPrediction(const Eigen::MatrixXd& Xsig_aug, double delta_t);

  void PredictMeanAndCovariance();

  void InitializeMeasurement(MeasurementPackage& measurement_package);

  void MeasurementUpdate(MeasurementPackage& measurement_package, Eigen::MatrixXd& Zsig, int n_z);

  double NormalizeAngle(double angle);

  bool is_initialized_;        // Initially set to false, set to true in first call of ProcessMeasurement
  bool use_laser_;             // If this is false, laser measurements will be ignored (except for init)
  bool use_radar_;             // If this is false, radar measurements will be ignored (except for init)
  Eigen::VectorXd x_;          // State vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  Eigen::MatrixXd P_;          // State vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  Eigen::MatrixXd Xsig_pred_;  // Predicted sigma points matrix
  long time_us_;               // Time when the state is true, in us
  double std_a_;               // Process noise standard deviation longitudinal acceleration in m/s^2
  double std_yawdd_;           // Process noise standard deviation yaw acceleration in rad/s^2
  double std_laspx_;           // Laser measurement noise standard deviation position1 in m
  double std_laspy_;           // Laser measurement noise standard deviation position2 in m
  double std_radr_;            // Radar measurement noise standard deviation radius in m
  double std_radphi_;          // Radar measurement noise standard deviation angle in rad
  double std_radrd_;           // Radar measurement noise standard deviation radius change in m/s
  Eigen::VectorXd weights_;    // Weights of sigma points
  int n_x_;                    // State dimension
  int n_aug_;                  // Augmented state dimension
  double lambda_;              // Sigma point spreading parameter
  double nis_radar_;           // Normalized Innovation Squared for radar
  double nis_lidar_;           // Normalized Innovation Squared for lidar
};

#endif /* UNSCENTED_KALMAN_FILTER_UKF_H */
