#ifndef UNSCENTED_KALMAN_FILTER_RENDER_H
#define UNSCENTED_KALMAN_FILTER_RENDER_H

#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "box.h"
#include "boxq.h"
#include "ukf.h"

struct Color {
  float r, g, b;

  Color(float _r, float _g, float _b) : r(_r), g(_g), b(_b) {}
};

enum CameraAngle { kXY, kTopDown, kSide, kFPS };

struct Instruction {
  float time_us;
  float acceleration;
  float steering;

  Instruction(float t, float acc, float s) : time_us(t), acceleration(acc), steering(s) {}
};

struct Car {
  // units in meters
  Eigen::Vector3d position;
  Eigen::Vector3d dimensions;
  Eigen::Quaternionf orientation;
  std::string name;
  Color color;
  float velocity;
  float angle;
  float acceleration;
  float steering;
  // distance between front of vehicle and center of gravity
  float lf;

  UKF ukf;

  std::vector<Instruction> instructions;
  int accuate_index;

  double sin_negative_theta;
  double cos_negative_theta;

  Car() : position(Eigen::Vector3d(0, 0, 0)), dimensions(Eigen::Vector3d(0, 0, 0)), color(Color(0, 0, 0)) {}

  Car(Eigen::Vector3d _position, Eigen::Vector3d _dimensions, Color _color, float _velocity, float _angle, float _lf, std::string _name)
      : position(std::move(_position)),
        dimensions(std::move(_dimensions)),
        color(_color),
        velocity(_velocity),
        angle(_angle),
        lf(_lf),
        name(std::move(_name)),
        acceleration(0),
        steering(0),
        accuate_index(-1) {
    orientation = ComputeQuaternion(angle);
    sin_negative_theta = sin(-angle);
    cos_negative_theta = cos(-angle);
  }

  // angle around z axis
  Eigen::Quaternionf ComputeQuaternion(float theta) {
    Eigen::Matrix3f rotation_mat;
    rotation_mat << cos(theta), -sin(theta), 0, sin(theta), cos(theta), 0, 0, 0, 1;
    Eigen::Quaternionf q(rotation_mat);
    return q;
  }

  void Render(pcl::visualization::PCLVisualizer::Ptr& viewer) {
    // render bottom of car
    viewer->addCube(
        Eigen::Vector3f(position.x(), position.y(), dimensions.z() * 1 / 3),
        orientation,
        dimensions.x(),
        dimensions.y(),
        dimensions.z() * 2 / 3,
        name);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name);
    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, name);
    viewer->addCube(
        Eigen::Vector3f(position.x(), position.y(), dimensions.z() * 1 / 3),
        orientation,
        dimensions.x(),
        dimensions.y(),
        dimensions.z() * 2 / 3,
        name + "frame");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 0, name + "frame");
    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, name + "frame");

    // render top of car
    viewer->addCube(
        Eigen::Vector3f(position.x(), position.y(), dimensions.z() * 5 / 6),
        orientation,
        dimensions.x() / 2,
        dimensions.y(),
        dimensions.z() * 1 / 3,
        name + "Top");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name + "Top");
    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, name + "Top");
    viewer->addCube(
        Eigen::Vector3f(position.x(), position.y(), dimensions.z() * 5 / 6),
        orientation,
        dimensions.x() / 2,
        dimensions.y(),
        dimensions.z() * 1 / 3,
        name + "Topframe");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 0, name + "Topframe");
    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, name + "Topframe");
  }

  void SetAcceleration(float setAcc) { acceleration = setAcc; }

  void SetSteering(float setSteer) { steering = setSteer; }

  void SetInstructions(const std::vector<Instruction>& _instructions) {
    for (auto a : _instructions) {
      instructions.emplace_back(a);
    }
  }

  void SetUKF(const UKF& tracker) { ukf = tracker; }

  void Move(float dt, int time_us) {
    if (!instructions.empty() && accuate_index < (int)instructions.size() - 1) {
      if (time_us >= instructions[accuate_index + 1].time_us) {
        SetAcceleration(instructions[accuate_index + 1].acceleration);
        SetSteering(instructions[accuate_index + 1].steering);
        accuate_index++;
      }
    }

    position.x() += velocity * cos(angle) * dt;
    position.y() += velocity * sin(angle) * dt;
    angle += velocity * steering * dt / lf;
    orientation = ComputeQuaternion(angle);
    velocity += acceleration * dt;

    sin_negative_theta = sin(-angle);
    cos_negative_theta = cos(-angle);
  }

  // collision helper function
  inline bool InBetween(double point, double center, double range) { return (center - range <= point) && (center + range >= point); }

  bool CheckCollision(Eigen::Vector3d point) {
    // check collision for rotated car
    double xPrime = ((point.x() - position.x()) * cos_negative_theta - (point.y() - position.y()) * sin_negative_theta) + position.x();
    double yPrime = ((point.y() - position.y()) * cos_negative_theta + (point.x() - position.x()) * sin_negative_theta) + position.y();

    return (InBetween(xPrime, position.x(), dimensions.x() / 2) && InBetween(yPrime, position.y(), dimensions.y() / 2) &&
            InBetween(point.z(), position.z() + dimensions.z() / 3, dimensions.z() / 3)) ||
           (InBetween(xPrime, position.x(), dimensions.x() / 4) && InBetween(yPrime, position.y(), dimensions.y() / 2) &&
            InBetween(point.z(), position.z() + dimensions.z() * 5 / 6, dimensions.z() / 6));
  }
};

void RenderHighway(double distance_pos, pcl::visualization::PCLVisualizer::Ptr& viewer);
void RenderRays(pcl::visualization::PCLVisualizer::Ptr& viewer, const Eigen::Vector3d& origin, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
void ClearRays(pcl::visualization::PCLVisualizer::Ptr& viewer);
void RenderPointCloud(
    pcl::visualization::PCLVisualizer::Ptr& viewer,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const std::string& name,
    Color color = Color(1, 1, 1));
void RenderPointCloud(
    pcl::visualization::PCLVisualizer::Ptr& viewer,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    const std::string& name,
    Color color = Color(-1, -1, -1));
void RenderBox(pcl::visualization::PCLVisualizer::Ptr& viewer, Box box, int id, Color color = Color(1, 0, 0), float opacity = 1);
void RenderBox(pcl::visualization::PCLVisualizer::Ptr& viewer, const BoxQ& box, int id, Color color = Color(1, 0, 0), float opacity = 1);

#endif /* UNSCENTED_KALMAN_FILTER_RENDER_H */
