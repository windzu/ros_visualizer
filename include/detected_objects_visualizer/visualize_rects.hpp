/*
 * @Author: windzu
 * @Date: 2022-05-18 11:37:02
 * @LastEditTime: 2022-05-18 13:27:13
 * @LastEditors: windzu
 * @Description:
 * @FilePath: /detected_objects_visualizer/include/visualize_rects.h
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
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Header.h"
// opencv
#include "opencv2/highgui/highgui.hpp"
// autoware
#include "autoware_msgs/DetectedObject.h"
#include "autoware_msgs/DetectedObjectArray.h"

#define __APP_NAME__ "visualize_rects"

class VisualizeRects {
 private:
  std::string input_topic_;

  ros::NodeHandle node_handle_;
  ros::Subscriber subscriber_detected_objects_;
  image_transport::Subscriber subscriber_image_;

  message_filters::Subscriber<autoware_msgs::DetectedObjectArray> *detection_filter_subscriber_;
  message_filters::Subscriber<sensor_msgs::Image> *image_filter_subscriber_;

  ros::Publisher publisher_image_;

  cv::Mat image_;
  std_msgs::Header image_header_;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, autoware_msgs::DetectedObjectArray>
      SyncPolicyT;

  message_filters::Synchronizer<SyncPolicyT> *detections_synchronizer_;

  void SyncedDetectionsCallback(const sensor_msgs::Image::ConstPtr &in_image_msg,
                                const autoware_msgs::DetectedObjectArray::ConstPtr &in_range_detections);

  bool IsObjectValid(const autoware_msgs::DetectedObject &in_object);

  cv::Mat ObjectsToRects(cv::Mat in_image, const autoware_msgs::DetectedObjectArray::ConstPtr &in_objects);

 public:
  VisualizeRects();
};
