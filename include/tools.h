#ifndef UNSCENTED_KALMAN_FILTER_TOOLS_H
#define UNSCENTED_KALMAN_FILTER_TOOLS_H

#include <pcl/io/pcd_io.h>

#include <iostream>
#include <random>
#include <vector>

#include "Eigen/Dense"
#include "render.h"

struct LMarker {
  double x, y;
  LMarker(double set_x, double set_y) : x(set_x), y(set_y) {}
};

struct RMarker {
  double rho, phi, rho_dot;
  RMarker(double set_rho, double set_phi, double set_rho_dot) : rho(set_rho), phi(set_phi), rho_dot(set_rho_dot) {}
};

class Tools {
 public:
  Tools() = delete;
  virtual ~Tools() = delete;

  static double Noise(double stddev, long long seedNum);
  static LMarker LidarSense(Car& car, pcl::visualization::PCLVisualizer::Ptr& viewer, long long timestamp, bool visualize);
  static RMarker RadarSense(Car& car, Car ego, pcl::visualization::PCLVisualizer::Ptr& viewer, long long timestamp, bool visualize);
  static void UkfResults(Car car, pcl::visualization::PCLVisualizer::Ptr& viewer, double time, int steps);
  static Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd>& estimations, const std::vector<Eigen::VectorXd>& ground_truth);
  static void SavePcd(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string file);
  static pcl::PointCloud<pcl::PointXYZ>::Ptr LoadPcd(std::string file);
};

#endif /* UNSCENTED_KALMAN_FILTER_TOOLS_H */
