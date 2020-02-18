#include "highway.h"

Highway::Highway(pcl::visualization::PCLVisualizer::Ptr& viewer) {
  ego_car = Car(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(4, 2, 2), Color(0, 1, 0), 0, 0, 2, "egoCar");

  Car car1(Eigen::Vector3d(-10, 4, 0), Eigen::Vector3d(4, 2, 2), Color(0, 0, 1), 5, 0, 2, "car1");

  std::vector<Instruction> car1_instructions;
  Instruction a = Instruction(0.5 * 1e6, 0.5, 0.0);
  car1_instructions.emplace_back(a);
  a = Instruction(2.2 * 1e6, 0.0, -0.2);
  car1_instructions.emplace_back(a);
  a = Instruction(3.3 * 1e6, 0.0, 0.2);
  car1_instructions.emplace_back(a);
  a = Instruction(4.4 * 1e6, -2.0, 0.0);
  car1_instructions.emplace_back(a);

  car1.SetInstructions(car1_instructions);
  if (track_cars[0]) {
    UKF ukf1;
    car1.SetUKF(ukf1);
  }
  traffic.emplace_back(car1);

  Car car2(Eigen::Vector3d(25, -4, 0), Eigen::Vector3d(4, 2, 2), Color(0, 0, 1), -6, 0, 2, "car2");
  std::vector<Instruction> car2_instructions;
  a = Instruction(4.0 * 1e6, 3.0, 0.0);
  car2_instructions.emplace_back(a);
  a = Instruction(8.0 * 1e6, 0.0, 0.0);
  car2_instructions.emplace_back(a);
  car2.SetInstructions(car2_instructions);

  if (track_cars[1]) {
    UKF ukf2;
    car2.SetUKF(ukf2);
  }
  traffic.emplace_back(car2);

  Car car3(Eigen::Vector3d(-12, 0, 0), Eigen::Vector3d(4, 2, 2), Color(0, 0, 1), 1, 0, 2, "car3");
  std::vector<Instruction> car3_instructions;
  a = Instruction(0.5 * 1e6, 2.0, 1.0);
  car3_instructions.emplace_back(a);
  a = Instruction(1.0 * 1e6, 2.5, 0.0);
  car3_instructions.emplace_back(a);
  a = Instruction(3.2 * 1e6, 0.0, -1.0);
  car3_instructions.emplace_back(a);
  a = Instruction(3.3 * 1e6, 2.0, 0.0);
  car3_instructions.emplace_back(a);
  a = Instruction(4.5 * 1e6, 0.0, 0.0);
  car3_instructions.emplace_back(a);
  a = Instruction(5.5 * 1e6, -2.0, 0.0);
  car3_instructions.emplace_back(a);
  a = Instruction(7.5 * 1e6, 0.0, 0.0);
  car3_instructions.emplace_back(a);
  car3.SetInstructions(car3_instructions);
  if (track_cars[2]) {
    UKF ukf3;
    car3.SetUKF(ukf3);
  }
  traffic.emplace_back(car3);

  lidar = std::make_unique<Lidar>(traffic, 0);

  // render environment
  RenderHighway(0, viewer);
  ego_car.Render(viewer);
  car1.Render(viewer);
  car2.Render(viewer);
  car3.Render(viewer);
}

void Highway::StepHighway(double egoVelocity, long long timestamp, int frame_per_sec, pcl::visualization::PCLVisualizer::Ptr& viewer) {
  if (visualize_pcd) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr trafficCloud = Tools::LoadPcd("../src/sensors/data/pcd/highway_" + std::to_string(timestamp) + ".pcd");
    RenderPointCloud(viewer, trafficCloud, "trafficCloud", Color(184.0 / 256.0, 223.0 / 256.0, 252.0 / 256.0));
  }

  std::vector<Eigen::VectorXd> estimations;
  std::vector<Eigen::VectorXd> ground_truth;

  // render highway environment with poles
  RenderHighway(static_cast<double>(egoVelocity * timestamp) / 1e6, viewer);
  ego_car.Render(viewer);

  for (int i = 0; i < traffic.size(); i++) {
    traffic[i].Move(1.0 / frame_per_sec, static_cast<double>(timestamp));
    if (!visualize_pcd) traffic[i].Render(viewer);
    // Sense surrounding cars with lidar and radar
    if (track_cars[i]) {
      Eigen::VectorXd gt(4);
      gt << traffic[i].position.x(), traffic[i].position.y(), traffic[i].velocity * cos(traffic[i].angle),
          traffic[i].velocity * sin(traffic[i].angle);
      ground_truth.emplace_back(gt);
      Tools::LidarSense(traffic[i], viewer, timestamp, visualize_lidar);
      Tools::RadarSense(traffic[i], ego_car, viewer, timestamp, visualize_radar);
      Tools::UkfResults(traffic[i], viewer, projected_time, projected_steps);
      Eigen::VectorXd estimate(4);
      Eigen::VectorXd x = traffic[i].ukf.GetState();
      double v = x(2);
      double yaw = x(3);
      double v1 = cos(yaw) * v;
      double v2 = sin(yaw) * v;
      estimate << x[0], x[1], v1, v2;
      estimations.emplace_back(estimate);
    }
  }
  viewer->addText("Accuracy - RMSE:", 30, 300, 20, 1, 1, 1, "rmse");
  Eigen::VectorXd rmse = Tools::CalculateRMSE(estimations, ground_truth);
  viewer->addText("X: " + std::to_string(rmse[0]), 30, 275, 20, 1, 1, 1, "rmse_x");
  viewer->addText("Y: " + std::to_string(rmse[1]), 30, 250, 20, 1, 1, 1, "rmse_y");
  viewer->addText("Vx: " + std::to_string(rmse[2]), 30, 225, 20, 1, 1, 1, "rmse_vx");
  viewer->addText("Vy: " + std::to_string(rmse[3]), 30, 200, 20, 1, 1, 1, "rmse_vy");

  if (timestamp > 1.0e6L) {
    if (rmse[0] > rmse_threshold[0]) {
      rmse_faillog[0] = rmse[0];
      pass = false;
    }
    if (rmse[1] > rmse_threshold[1]) {
      rmse_faillog[1] = rmse[1];
      pass = false;
    }
    if (rmse[2] > rmse_threshold[2]) {
      rmse_faillog[2] = rmse[2];
      pass = false;
    }
    if (rmse[3] > rmse_threshold[3]) {
      rmse_faillog[3] = rmse[3];
      pass = false;
    }
  }
  if (!pass) {
    viewer->addText("RMSE Failed Threshold", 30, 150, 20, 1, 0, 0, "rmse_fail");
    if (rmse_faillog[0] > 0) {
      viewer->addText(" X: " + std::to_string(rmse_faillog[0]), 30, 125, 20, 1, 0, 0, "rmse_fail_x");
    }
    if (rmse_faillog[1] > 0) {
      viewer->addText(" Y: " + std::to_string(rmse_faillog[1]), 30, 100, 20, 1, 0, 0, "rmse_fail_y");
    }
    if (rmse_faillog[2] > 0) {
      viewer->addText("Vx: " + std::to_string(rmse_faillog[2]), 30, 75, 20, 1, 0, 0, "rmse_fail_vx");
    }
    if (rmse_faillog[3] > 0) {
      viewer->addText("Vy: " + std::to_string(rmse_faillog[3]), 30, 50, 20, 1, 0, 0, "rmse_fail_vy");
    }
  }
}
