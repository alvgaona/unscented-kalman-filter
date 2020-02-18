#ifndef UNSCENTED_KALMAN_FILTER_LIDAR_H
#define UNSCENTED_KALMAN_FILTER_LIDAR_H

#include <chrono>
#include <ctime>
#include <utility>

#include "render.h"

const double pi = 3.1415;

struct Ray {
  Eigen::Vector3d origin;
  double resolution;
  Eigen::Vector3d direction;
  Eigen::Vector3d cast_position;
  double cast_distance;

  /**
   * Constructor
   * @param _origin  Starting position from where the ray is cast
   * @param horizontal_angle Angle of direction the ray travels on the XY plane
   * @param vertical_angle Angle of direction between XY plane and ray for example 0 radians is along XY plane and pi/2 radians is stright up
   * @param resolution Ray's step magnitude, used for ray casting, the smaller the more accurate but the more expensive
   */

  Ray(Eigen::Vector3d _origin, double horizontal_angle, double vertical_angle, float _resolution)
      : origin(std::move(_origin)),
        resolution(_resolution),
        direction(
            resolution * cos(vertical_angle) * cos(horizontal_angle),
            resolution * cos(vertical_angle) * sin(horizontal_angle),
            resolution * sin(vertical_angle)),
        cast_position(origin),
        cast_distance(0) {}

  void RayCast(
      const std::vector<Car>& cars,
      double min_distance,
      double max_distance,
      pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
      double slope_angle,
      double sderr) {
    // reset ray
    cast_position = origin;
    cast_distance = 0;

    bool collision = false;

    while (!collision && cast_distance < max_distance &&
           (cast_position.y() <= 6 && cast_position.y() >= -6 && cast_position.x() <= 50 && cast_position.x() >= -15)) {
      cast_position = cast_position + direction;
      cast_distance += resolution;

      // check if there is any collisions with ground slope
      collision = (cast_position.z() <= cast_position.x() * tan(slope_angle));

      // check if there is any collisions with cars
      if (!collision && cast_distance < max_distance) {
        for (Car car : cars) {
          collision |= car.CheckCollision(cast_position);
          if (collision) break;
        }
      }
    }

    if ((cast_distance >= min_distance) && (cast_distance <= max_distance) &&
        (cast_position.y() <= 6 && cast_position.y() >= -6 && cast_position.x() <= 50 && cast_position.x() >= -15)) {
      // add noise based on standard deviation error
      auto rx = static_cast<double>(rand()) / RAND_MAX;
      auto ry = static_cast<double>(rand()) / RAND_MAX;
      auto rz = static_cast<double>(rand()) / RAND_MAX;
      cloud->points.emplace_back(cast_position.x() + rx * sderr, cast_position.y() + ry * sderr, cast_position.z() + rz * sderr);
    }
  }
};

struct Lidar {
  std::vector<Ray> rays;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  std::vector<Car> cars;
  Eigen::Vector3d position;
  double ground_slope;
  double min_distance;
  double max_distance;
  double resoultion;
  double sderr;

  Lidar(std::vector<Car> _cars, double _ground_slope) : cloud(new pcl::PointCloud<pcl::PointXYZ>()), position(0, 0, 3.0) {
    // TODO:: set minDistance to 5 to remove points from roof of ego car
    min_distance = 0;
    max_distance = 120;
    resoultion = 0.2;
    // TODO:: set sderr to 0.2 to get more interesting pcd files
    sderr = 0.02;
    cars = _cars;
    ground_slope = _ground_slope;

    int num_layers = 64;
    // the steepest vertical angle
    double steepest_angle = 24.8 * (-pi / 180);
    double angle_range = 26.8 * (pi / 180);
    // TODO:: set to pi/64 to get higher resoultion pcd
    double horizontal_angle_inclination = pi / 2250;

    double angle_increment = angle_range / num_layers;

    for (double angle_vertical = steepest_angle; angle_vertical < steepest_angle + angle_range; angle_vertical += angle_increment) {
      for (double angle = 0; angle <= 2 * pi; angle += horizontal_angle_inclination) {
        Ray ray(position, angle, angle_vertical, resoultion);
        rays.push_back(ray);
      }
    }
  }

  ~Lidar() = default;

  void UpdateCars(std::vector<Car> _cars) { cars = _cars; }

  pcl::PointCloud<pcl::PointXYZ>::Ptr Scan() {
    cloud->points.clear();
    auto startTime = std::chrono::steady_clock::now();
    for (Ray ray : rays) {
      ray.RayCast(cars, min_distance, max_distance, cloud, ground_slope, sderr);
    }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    cout << "ray casting took " << elapsedTime.count() << " milliseconds" << endl;
    cloud->width = cloud->points.size();
    cloud->height = 1;  // one dimensional unorganized point cloud dataset
    return cloud;
  }
};

#endif /* UNSCENTED_KALMAN_FILTER_LIDAR_H */
