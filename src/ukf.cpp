#include "ukf.h"

UKF::UKF() {
  is_initialized_ = false;

  use_laser_ = true;  // If this is false, laser measurements will be ignored (except during init)
  use_radar_ = true;  // If this is false, radar measurements will be ignored (except during init)

  x_ = Eigen::VectorXd(5);     // Initial state vector
  P_ = Eigen::MatrixXd(5, 5);  // Initial covariance matrix
  P_ << 1.0, 0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 0, 1000, 0, 0, 0, 0, 0, 1000, 0, 0, 0, 0, 0, 1.0;

  std_a_ = 1;      // Process noise standard deviation longitudinal acceleration in m/s^2
  std_yawdd_ = 1;  // Process noise standard deviation yaw acceleration in rad/s^2

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

  n_x_ = 5;
  n_aug_ = 7;
  lambda_ = 3 - n_x_;

  weights_ = Eigen::VectorXd(2 * n_aug_ + 1);
  weights_(0) = 1 / (1 + n_aug_ / lambda_);
  for (int i = 1; i < 2 * n_aug_ + 1; ++i) {
    weights_(i) = 0.5 / (n_aug_ + lambda_);
  }

  Xsig_pred_ = Eigen::MatrixXd(n_x_, 2 * n_aug_ + 1);
}

UKF::~UKF() = default;

void UKF::ProcessMeasurement(MeasurementPackage& measurement_package) {
  if (!is_initialized_) {
    InitializeMeasurement(measurement_package);
  }

  double delta_t = static_cast<double>(measurement_package.timestamp_ - time_us_) / 1000000.0;
  time_us_ = measurement_package.timestamp_;
  Prediction(delta_t);
  if (measurement_package.sensor_type_ == MeasurementPackage::SensorType::LASER && use_laser_) {
    UpdateLidar(measurement_package);
  } else if (measurement_package.sensor_type_ == MeasurementPackage::SensorType::RADAR && use_radar_) {
    UpdateRadar(measurement_package);
  }
}

void UKF::Prediction(double delta_t) {
  Eigen::MatrixXd Xsig_aug;
  AugmentedSigmaPoints(Xsig_aug);
  SigmaPointsPrediction(Xsig_aug, delta_t);
  PredictMeanAndCovariance();
}

void UKF::UpdateLidar(MeasurementPackage& measurement_package) {
  int n_z = 2;

  Eigen::MatrixXd Zsig(n_z, 2 * n_aug_ + 1);

  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    Zsig(0, i) = Xsig_pred_(0, i);
    Zsig(1, i) = Xsig_pred_(1, i);
  }

  MeasurementUpdate(measurement_package, Zsig, n_z);
}

void UKF::UpdateRadar(MeasurementPackage& measurement_package) {
  int n_z = 3;

  Eigen::MatrixXd Zsig(n_z, 2 * n_aug_ + 1);  // Sigma points matrix in measurement space

  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    double p_x = Xsig_pred_(0, i);
    double p_y = Xsig_pred_(1, i);
    double v = Xsig_pred_(2, i);
    double yaw = Xsig_pred_(3, i);

    double v1 = v * cos(yaw);
    double v2 = v * sin(yaw);

    Zsig(0, i) = std::sqrt(p_x * p_x + p_y * p_y);                          // r
    Zsig(1, i) = std::atan2(p_y, p_x);                                      // phi
    Zsig(2, i) = (p_x * v1 + p_y * v2) / std::sqrt(p_x * p_x + p_y * p_y);  // r_dot
  }

  MeasurementUpdate(measurement_package, Zsig, n_z);
}

void UKF::AugmentedSigmaPoints(Eigen::MatrixXd& Xsig_aug) {
  Eigen::VectorXd x_aug(n_aug_);
  Eigen::MatrixXd P_aug(n_aug_, n_aug_);

  Xsig_aug = Eigen::MatrixXd(n_aug_, 2 * n_aug_ + 1);

  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  P_aug.fill(0.0);
  P_aug.topLeftCorner(5, 5) = P_;
  P_aug(5, 5) = std_a_ * std_a_;
  P_aug(6, 6) = std_yawdd_ * std_yawdd_;

  Eigen::MatrixXd L = P_aug.llt().matrixL();

  Xsig_aug.col(0) = x_aug;
  for (int i = 0; i < n_aug_; ++i) {
    Xsig_aug.col(i + 1) = x_aug + std::sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug.col(i + 1 + n_aug_) = x_aug - std::sqrt(lambda_ + n_aug_) * L.col(i);
  }
}

void UKF::SigmaPointsPrediction(const Eigen::MatrixXd& Xsig_aug, double delta_t) {
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    double p_x = Xsig_aug(0, i);
    double p_y = Xsig_aug(1, i);
    double v = Xsig_aug(2, i);
    double yaw = Xsig_aug(3, i);
    double yawd = Xsig_aug(4, i);
    double nu_a = Xsig_aug(5, i);
    double nu_yawdd = Xsig_aug(6, i);

    // predicted state values
    double px_p;
    double py_p;

    // avoid division by zero
    if (std::fabs(yawd) > 0.001) {
      px_p = p_x + v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
      py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd * delta_t));
    } else {
      px_p = p_x + v * delta_t * cos(yaw);
      py_p = p_y + v * delta_t * sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd * delta_t;
    double yawd_p = yawd;

    // add noise
    px_p = px_p + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
    py_p = py_p + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
    v_p = v_p + nu_a * delta_t;

    yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
    yawd_p = yawd_p + nu_yawdd * delta_t;

    // write predicted sigma point into right column
    Xsig_pred_(0, i) = px_p;
    Xsig_pred_(1, i) = py_p;
    Xsig_pred_(2, i) = v_p;
    Xsig_pred_(3, i) = yaw_p;
    Xsig_pred_(4, i) = yawd_p;
  }
}

void UKF::InitializeMeasurement(MeasurementPackage& measurement_package) {
  if (measurement_package.sensor_type_ == MeasurementPackage::SensorType::LASER && use_laser_) {
    x_ << measurement_package.raw_measurements_[0], measurement_package.raw_measurements_[1], 0, 0, 0;
    time_us_ = measurement_package.timestamp_;
    is_initialized_ = true;
  } else if (measurement_package.sensor_type_ == MeasurementPackage::SensorType::RADAR && use_radar_) {
    double r = measurement_package.raw_measurements_[0];
    double phi = measurement_package.raw_measurements_[1];
    double phi_dot = measurement_package.raw_measurements_[2];
    double cos_phi = cos(phi);
    double sin_phi = sin(phi);

    x_[0] = r * cos_phi;
    x_[1] = r * sin_phi;
    x_[3] = std::sqrt(pow(phi_dot * cos_phi, 2) + pow(phi_dot * sin_phi, 2));
    x_[4] = 0;
    x_[5] = 0;
    time_us_ = measurement_package.timestamp_;

    is_initialized_ = true;
  }
}

void UKF::PredictMeanAndCovariance() {
  Eigen::VectorXd x(n_x_);
  x.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    x += weights_(i) * Xsig_pred_.col(i);
  }

  Eigen::MatrixXd P(n_x_, n_x_);
  P.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // iterate over sigma points
    Eigen::VectorXd x_diff = Xsig_pred_.col(i) - x;
    x_diff(3) = NormalizeAngle(x_diff(3));

    P += weights_(i) * x_diff * x_diff.transpose();
  }

  x_ = x;
  P_ = P;
}

void UKF::MeasurementUpdate(MeasurementPackage& measurement_package, Eigen::MatrixXd& Zsig, int n_z) {
  Eigen::VectorXd z_pred(n_z);  // Mean predicted measurement
  Eigen::MatrixXd S(n_z, n_z);  // Measurement covariance matrix

  z_pred.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    z_pred += weights_(i) * Zsig.col(i);
  }

  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    Eigen::VectorXd z_diff = Zsig.col(i) - z_pred;

    if(measurement_package.sensor_type_ == MeasurementPackage::RADAR) {
      z_diff(1) = NormalizeAngle(z_diff(1));
    }

    S += weights_(i) * z_diff * z_diff.transpose();
  }

  Eigen::MatrixXd R(n_z, n_z);
  Eigen::VectorXd z(n_z);
  if (measurement_package.sensor_type_ == MeasurementPackage::LASER) {
    z << measurement_package.raw_measurements_[0], measurement_package.raw_measurements_[1];
    R << std_laspx_ * std_laspx_, 0, 0, std_laspy_ * std_laspy_;
  } else if (measurement_package.sensor_type_ == MeasurementPackage::RADAR) {
    z << measurement_package.raw_measurements_[0], measurement_package.raw_measurements_[1], measurement_package.raw_measurements_[2];
    R << std_radr_ * std_radr_, 0, 0, 0, std_radphi_ * std_radphi_, 0, 0, 0, std_radrd_ * std_radrd_;
  }
  S = S + R;

  Eigen::MatrixXd Tc(n_x_, n_z);
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    // residual
    Eigen::VectorXd z_diff = Zsig.col(i) - z_pred;

    if(measurement_package.sensor_type_ == MeasurementPackage::RADAR) {
      z_diff(1) = NormalizeAngle(z_diff(1));
    }

    Eigen::VectorXd x_diff = Xsig_pred_.col(i) - x_;
    x_diff(3) = NormalizeAngle(x_diff(3));

    Tc += weights_(i) * x_diff * z_diff.transpose();
  }

  // Kalman gain K;
  Eigen::MatrixXd Si = S.inverse();
  Eigen::MatrixXd K = Tc * Si;

  // Residual
  Eigen::VectorXd z_diff = z - z_pred;

  // Angle normalization
  if(measurement_package.sensor_type_ == MeasurementPackage::RADAR) {
    z_diff(1) = NormalizeAngle(z_diff(1));
  }

  x_ += K * z_diff;
  P_ -= K * S * K.transpose();

  if (measurement_package.sensor_type_ == MeasurementPackage::RADAR) {
    nis_radar_ = z_diff.transpose() * Si * z_diff;
  } else if (measurement_package.sensor_type_ == MeasurementPackage::LASER){
    nis_lidar_ = z_diff.transpose() * Si * z_diff;
  }
}

inline double UKF::NormalizeAngle(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}
