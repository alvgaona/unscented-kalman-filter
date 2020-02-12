# Generating Additional Data

If you'd like to generate your own radar and lidar modify the code in `highway.h` to alter the cars.
Also check out `tools.cpp` to change how measurements are taken, for instance lidar markers could be the (x,y)
center of bounding boxes by scanning the PCD environment and performing clustering.
This is similar to what was done in Sensor Fusion [Lidar Obstacle Detection].

[Lidar Obstacle Detection]: https://github.com/alvgaona/lidar-obstacle-detection
