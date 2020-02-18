#include "tools.h"

double Tools::Noise(double stddev, long long seedNum) {
  std::mt19937::result_type seed = seedNum;
  auto dist = std::bind(std::normal_distribution<double>{0, stddev}, std::mt19937(seed));
  return dist();
}

// sense where a car is located using lidar measurement
LMarker Tools::LidarSense(Car& car, pcl::visualization::PCLVisualizer::Ptr& viewer, long long timestamp, bool visualize) {
  MeasurementPackage meas_package;
  meas_package.sensor_type_ = MeasurementPackage::LASER;
  meas_package.raw_measurements_ = Eigen::VectorXd(2);

  LMarker marker = LMarker(car.position.x() + Noise(0.15, timestamp), car.position.y() + Noise(0.15, timestamp + 1));
  if (visualize) {
    viewer->addSphere(pcl::PointXYZ(marker.x, marker.y, 3.0), 0.5, 1, 0, 0, car.name + "_lmarker");
  }

  meas_package.raw_measurements_ << marker.x, marker.y;
  meas_package.timestamp_ = timestamp;

  car.ukf.ProcessMeasurement(meas_package);

  return marker;
}

// sense where a car is located using radar measurement
RMarker Tools::RadarSense(Car& car, Car ego, pcl::visualization::PCLVisualizer::Ptr& viewer, long long timestamp, bool visualize) {
  double rho = sqrt(
      (car.position.x() - ego.position.x()) * (car.position.x() - ego.position.x()) +
      (car.position.y() - ego.position.y()) * (car.position.y() - ego.position.y()));
  double phi = atan2(car.position.y() - ego.position.y(), car.position.x() - ego.position.x());
  double rhod = (car.velocity * cos(car.angle) * rho * cos(phi) + car.velocity * sin(car.angle) * rho * sin(phi)) / rho;

  RMarker marker = RMarker(rho + Noise(0.3, timestamp + 2), phi + Noise(0.03, timestamp + 3), rhod + Noise(0.3, timestamp + 4));
  if (visualize) {
    viewer->addLine(
        pcl::PointXYZ(ego.position.x(), ego.position.y(), 3.0),
        pcl::PointXYZ(ego.position.x() + marker.rho * cos(marker.phi), ego.position.y() + marker.rho * sin(marker.phi), 3.0),
        1,
        0,
        1,
        car.name + "_rho");
    viewer->addArrow(
        pcl::PointXYZ(ego.position.x() + marker.rho * cos(marker.phi), ego.position.y() + marker.rho * sin(marker.phi), 3.0),
        pcl::PointXYZ(
            ego.position.x() + marker.rho * cos(marker.phi) + marker.rhod * cos(marker.phi),
            ego.position.y() + marker.rho * sin(marker.phi) + marker.rhod * sin(marker.phi),
            3.0),
        1,
        0,
        1,
        car.name + "_rhod");
  }

  MeasurementPackage meas_package;
  meas_package.sensor_type_ = MeasurementPackage::RADAR;
  meas_package.raw_measurements_ = Eigen::VectorXd(3);
  meas_package.raw_measurements_ << marker.rho, marker.phi, marker.rhod;
  meas_package.timestamp_ = timestamp;

  car.ukf.ProcessMeasurement(meas_package);

  return marker;
}

// Show UKF tracking and also allow showing predicted future path
// double time:: time ahead in the future to predict
// int steps:: how many steps to show between present and time and future time
void Tools::UkfResults(Car car, pcl::visualization::PCLVisualizer::Ptr& viewer, double time, int steps) {
  UKF ukf = car.ukf;
  Eigen::VectorXd x = ukf.GetState();
  viewer->addSphere(pcl::PointXYZ(x[0], x[1], 3.5), 0.5, 0, 1, 0, car.name + "_ukf");
  viewer->addArrow(
      pcl::PointXYZ(x[0], x[1], 3.5), pcl::PointXYZ(x[0] + x[2] * cos(x[3]), x[1] + x[2] * sin(x[3]), 3.5), 0, 1, 0, car.name + "_ukf_vel");
  if (time > 0) {
    double dt = time / steps;
    double ct = dt;
    while (ct <= time) {
      ukf.Prediction(dt);
      viewer->addSphere(pcl::PointXYZ(x[0], x[1], 3.5), 0.5, 0, 1, 0, car.name + "_ukf" + std::to_string(ct));
      viewer->setShapeRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0 - 0.8 * (ct / time), car.name + "_ukf" + std::to_string(ct));
      // viewer->addArrow(pcl::PointXYZ(x[0], x[1],3.5),
      // pcl::PointXYZ(x[0]+x[2]*cos(x[3]),x[1]+x[2]*sin(x[3]),3.5), 0, 1, 0, car.name+"_ukf_vel"+std::to_string(ct));
      // viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0-0.8*(ct/time), car.name+"_ukf_vel"+std::to_string(ct));
      ct += dt;
    }
  }
}

Eigen::VectorXd Tools::CalculateRMSE(const std::vector<Eigen::VectorXd>& estimations, const std::vector<Eigen::VectorXd>& ground_truth) {
  Eigen::VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if (estimations.size() != ground_truth.size() || estimations.size() == 0) {
    cout << "Invalid estimation or ground_truth data" << endl;
    return rmse;
  }

  // accumulate squared residuals
  for (unsigned int i = 0; i < estimations.size(); ++i) {
    Eigen::VectorXd residual = estimations[i] - ground_truth[i];

    // coefficient-wise multiplication
    residual = residual.array() * residual.array();
    rmse += residual;
  }

  // calculate the mean
  rmse = rmse / estimations.size();

  // calculate the squared root
  rmse = rmse.array().sqrt();

  // return the result
  return rmse;
}

void Tools::SavePcd(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string file) {
  pcl::io::savePCDFileASCII(file, *cloud);
  std::cerr << "Saved " << cloud->points.size() << " data points to " + file << std::endl;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Tools::LoadPcd(std::string file) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file, *cloud) == -1)  //* load the file
  {
    PCL_ERROR("Couldn't read file \n");
  }
  // std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

  return cloud;
}
