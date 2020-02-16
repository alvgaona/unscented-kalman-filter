#ifndef UNSCENTED_KALMAN_FILTER_BOXQ_H
#define UNSCENTED_KALMAN_FILTER_BOXQ_H

#include "Eigen/Geometry"

struct BoxQ {
  Eigen::Vector3f bboxTransform;
  Eigen::Quaternionf bboxQuaternion;
  float cube_length;
  float cube_width;
  float cube_height;
};

#endif  // UNSCENTED_KALMAN_FILTER_BOXQ_H
