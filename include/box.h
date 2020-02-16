#ifndef UNSCENTED_KALMAN_FILTER_BOX_H
#define UNSCENTED_KALMAN_FILTER_BOX_H

struct Box {
  float x_min;
  float y_min;
  float z_min;
  float x_max;
  float y_max;
  float z_max;
};
#endif /* UNSCENTED_KALMAN_FILTER_BOX_H */
