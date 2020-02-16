#ifndef UNSCENTED_KALMAN_FILTER_HIGHWAY_H
#define UNSCENTED_KALMAN_FILTER_HIGHWAY_H

#include <memory>

#include "lidar.h"
#include "render.h"
#include "tools.h"

class Highway {
 public:
  std::vector<Car> traffic;
  Car ego_car;
  bool pass = true;
  std::vector<double> rmse_threshold = {0.30, 0.16, 0.95, 0.70};
  std::vector<double> rmse_faillog = {0.0, 0.0, 0.0, 0.0};
  std::unique_ptr<Lidar> lidar;

  std::vector<bool> track_cars = {true, true, true};  // Set which cars to track with UKF
  bool visualize_lidar = true;
  bool visualize_radar = true;
  bool visualize_pcd = false;
  // Predict path in the future using UKF
  double projected_time = 0;
  int projected_steps = 0;

  Highway(pcl::visualization::PCLVisualizer::Ptr& viewer);
  void StepHighway(double egoVelocity, long long timestamp, int frame_per_sec, pcl::visualization::PCLVisualizer::Ptr& viewer);
};

#endif /* UNSCENTED_KALMAN_FILTER_HIGHWAY_H */
