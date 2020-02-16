#include "render.h"

void RenderHighway(double distance_pos, pcl::visualization::PCLVisualizer::Ptr& viewer) {
  // units in meters
  double road_length_ahead = 50.0;
  double road_length_behind = -15.0;
  double road_width = 12.0;
  double road_height = 0.2;

  viewer->addCube(road_length_behind, road_length_ahead, -road_width / 2, road_width / 2, -road_height, 0, .2, .2, .2, "highwayPavement");
  viewer->setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, "highwayPavement");
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, "highwayPavement");
  viewer->addLine(
      pcl::PointXYZ(road_length_behind, -road_width / 6, 0.01), pcl::PointXYZ(road_length_ahead, -road_width / 6, 0.01), 1, 1, 0, "line1");
  viewer->addLine(pcl::PointXYZ(road_length_behind, road_width / 6, 0.01), pcl::PointXYZ(road_length_ahead, road_width / 6, 0.01), 1, 1, 0, "line2");

  // render poles
  // spacing in meters between poles, poles start at x = 0
  double pole_space = 10;
  // pole distance from road curve
  double pole_curve = 4;
  double pole_width = 0.5;
  double pole_height = 3;

  // double distancePos = 7;
  double marker_pos = (road_length_behind / pole_space) * pole_space - distance_pos;
  while (marker_pos < road_length_behind) {
    marker_pos += pole_space;
  }
  int pole_index = 0;
  while (marker_pos <= road_length_ahead) {
    //	left pole
    viewer->addCube(
        -pole_width / 2 + marker_pos,
        pole_width / 2 + marker_pos,
        -pole_width / 2 + road_width / 2 + pole_curve,
        pole_width / 2 + road_width / 2 + pole_curve,
        0,
        pole_height,
        1,
        0.5,
        0,
        "pole_" + std::to_string(pole_index) + "l");
    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,
        "pole_" + std::to_string(pole_index) + "l");
    viewer->addCube(
        -pole_width / 2 + marker_pos,
        pole_width / 2 + marker_pos,
        -pole_width / 2 + road_width / 2 + pole_curve,
        pole_width / 2 + road_width / 2 + pole_curve,
        0,
        pole_height,
        0,
        0,
        0,
        "pole_" + std::to_string(pole_index) + "lframe");
    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
        "pole_" + std::to_string(pole_index) + "lframe");

    //	right pole
    viewer->addCube(
        -pole_width / 2 + marker_pos,
        pole_width / 2 + marker_pos,
        -pole_width / 2 - road_width / 2 - pole_curve,
        pole_width / 2 - road_width / 2 - pole_curve,
        0,
        pole_height,
        1,
        0.5,
        0,
        "pole_" + std::to_string(pole_index) + "r");
    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,
        "pole_" + std::to_string(pole_index) + "r");
    viewer->addCube(
        -pole_width / 2 + marker_pos,
        pole_width / 2 + marker_pos,
        -pole_width / 2 - road_width / 2 - pole_curve,
        pole_width / 2 - road_width / 2 - pole_curve,
        0,
        pole_height,
        0,
        0,
        0,
        "pole_" + std::to_string(pole_index) + "rframe");
    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
        "pole_" + std::to_string(pole_index) + "rframe");

    marker_pos += pole_space;
    pole_index++;
  }
}

int count_rays = 0;
void RenderRays(pcl::visualization::PCLVisualizer::Ptr& viewer, const Eigen::Vector3d& origin, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
  for (pcl::PointXYZ point : cloud->points) {
    viewer->addLine(pcl::PointXYZ(origin.x(), origin.y(), origin.z()), point, 1, 0, 0, "ray" + std::to_string(count_rays));
    count_rays++;
  }
}

void ClearRays(pcl::visualization::PCLVisualizer::Ptr& viewer) {
  while (count_rays) {
    count_rays--;
    viewer->removeShape("ray" + std::to_string(count_rays));
  }
}

void RenderPointCloud(
    pcl::visualization::PCLVisualizer::Ptr& viewer, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::string& name, Color color) {
  viewer->addPointCloud<pcl::PointXYZ>(cloud, name);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, name);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name);
}

void RenderPointCloud(
    pcl::visualization::PCLVisualizer::Ptr& viewer, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const std::string& name, Color color) {
  if (color.r == -1) {
    // Select color based off of cloud intensity
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(cloud, "intensity");
    viewer->addPointCloud<pcl::PointXYZI>(cloud, intensity_distribution, name);
  } else {
    // Select color based off input value
    viewer->addPointCloud<pcl::PointXYZI>(cloud, name);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name);
  }

  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name);
}

// Draw wire frame box with filled transparent color
void RenderBox(pcl::visualization::PCLVisualizer::Ptr& viewer, Box box, int id, Color color, float opacity) {
  if (opacity > 1.0) opacity = 1.0;
  if (opacity < 0.0) opacity = 0.0;

  std::string cube = "box" + std::to_string(id);
  // viewer->addCube(box.bboxTransform, box.bboxQuaternion, box.cube_length, box.cube_width, box.cube_height, cube);
  viewer->addCube(box.x_min, box.x_max, box.y_min, box.y_max, box.z_min, box.z_max, color.r, color.g, color.b, cube);
  viewer->setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube);
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cube);
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, cube);

  std::string cubeFill = "boxFill" + std::to_string(id);
  // viewer->addCube(box.bboxTransform, box.bboxQuaternion, box.cube_length, box.cube_width, box.cube_height, cubeFill);
  viewer->addCube(box.x_min, box.x_max, box.y_min, box.y_max, box.z_min, box.z_max, color.r, color.g, color.b, cubeFill);
  viewer->setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, cubeFill);
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cubeFill);
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity * 0.3, cubeFill);
}

void RenderBox(pcl::visualization::PCLVisualizer::Ptr& viewer, const BoxQ& box, int id, Color color, float opacity) {
  if (opacity > 1.0) {
    opacity = 1.0;
  }
  if (opacity < 0.0) {
    opacity = 0.0;
  }

  std::string cube = "box" + std::to_string(id);
  viewer->addCube(box.bboxTransform, box.bboxQuaternion, box.cube_length, box.cube_width, box.cube_height, cube);
  viewer->setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube);
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cube);
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, cube);

  std::string cube_fill = "boxFill" + std::to_string(id);
  viewer->addCube(box.bboxTransform, box.bboxQuaternion, box.cube_length, box.cube_width, box.cube_height, cube_fill);
  viewer->setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, cube_fill);
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cube_fill);
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity * 0.3, cube_fill);
}
