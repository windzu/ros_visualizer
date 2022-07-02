/*
 * @Author: windzu
 * @Date: 2022-05-18 11:37:02
 * @LastEditTime: 2022-05-23 18:27:08
 * @LastEditors: windzu
 * @Description:
 * @FilePath: /detected_objects_visualizer/src/visualize_detected_objects_main.cpp
 * @Copyright (C) 2021-2022 plusgo Company Limited. All rights reserved.
 * @Licensed under the Apache License, Version 2.0 (the License)
 */

#include "detected_objects_visualizer/visualize_detected_objects.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "visualize_detected_objects");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  VisualizeDetectedObjects app(nh, pnh);
  ros::spin();

  return 0;
}
