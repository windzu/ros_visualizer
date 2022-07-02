/*
 * @Author: windzu
 * @Date: 2022-05-18 11:37:02
 * @LastEditTime: 2022-05-23 18:26:12
 * @LastEditors: windzu
 * @Description:
 * @FilePath: /detected_objects_visualizer/include/detected_objects_visualizer/visualize_detected_objects.hpp
 * @Copyright (C) 2021-2022 plusgo Company Limited. All rights reserved.
 * @Licensed under the Apache License, Version 2.0 (the License)
 */
#pragma once
// cpp system headers
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>
// third party headers
// ros
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Header.h"
#include "tf/transform_datatypes.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
// opencv
#include "opencv2/highgui/highgui.hpp"
// autoware
#include "autoware_msgs/DetectedObject.h"
#include "autoware_msgs/DetectedObjectArray.h"

#define __APP_NAME__ "visualize_detected_objects"

class VisualizeDetectedObjects {
 public:
  VisualizeDetectedObjects(ros::NodeHandle nh, ros::NodeHandle pnh);

 private:
  const double arrow_height_;
  const double label_height_;
  const double object_max_linear_size_ = 50.;
  double object_speed_threshold_;
  double arrow_speed_threshold_;
  double marker_display_duration_;

  int marker_id_;

  std_msgs::ColorRGBA label_color_, box_color_, hull_color_, arrow_color_, centroid_color_, model_color_;

  std::string input_topic_, ros_namespace_;

  ros::NodeHandle node_handle_;
  ros::Subscriber subscriber_detected_objects_;

  ros::Publisher publisher_markers_;

  visualization_msgs::MarkerArray ObjectsToLabels(const autoware_msgs::DetectedObjectArray &in_objects);

  visualization_msgs::MarkerArray ObjectsToArrows(const autoware_msgs::DetectedObjectArray &in_objects);

  visualization_msgs::MarkerArray ObjectsToBoxes(const autoware_msgs::DetectedObjectArray &in_objects);

  visualization_msgs::MarkerArray ObjectsToModels(const autoware_msgs::DetectedObjectArray &in_objects);

  visualization_msgs::MarkerArray ObjectsToHulls(const autoware_msgs::DetectedObjectArray &in_objects);

  visualization_msgs::MarkerArray ObjectsToCentroids(const autoware_msgs::DetectedObjectArray &in_objects);

  std::string ColorToString(const std_msgs::ColorRGBA &in_color);

  void DetectedObjectsCallback(const autoware_msgs::DetectedObjectArray &in_objects);

  bool IsObjectValid(const autoware_msgs::DetectedObject &in_object);

  float CheckColor(double value);

  float CheckAlpha(double value);

  std_msgs::ColorRGBA ParseColor(const std::vector<double> &in_color);
};
