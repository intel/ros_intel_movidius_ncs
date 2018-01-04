/*
 * Copyright (c) 2017 Intel Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <object_msgs/ClassifyObject.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "movidius_ncs_example");

  if (argc != 2)
  {
    ROS_INFO("Usage: rosrun movidius_ncs_example movidius_ncs_example_image_classification <image_path>");
    return -1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client;
  client = n.serviceClient<object_msgs::ClassifyObject>("/movidius_ncs_image/classify_object");
  object_msgs::ClassifyObject srv;

  cv_bridge::CvImage cv_image;
  sensor_msgs::Image ros_image;
  cv_image.image = cv::imread(argv[1]);
  cv_image.encoding = "bgr8";
  cv_image.toImageMsg(ros_image);
  srv.request.image = ros_image;

  if (!client.call(srv))
  {
    ROS_ERROR("failed to call service ClassifyObject");
    return 1;
  }

  for (unsigned int i = 0; i < srv.response.objects.objects_vector.size(); i++)
  {
    ROS_INFO("%d: object: %s\nprobability: %lf%%", i,
             srv.response.objects.objects_vector[i].object_name.c_str(),
             srv.response.objects.objects_vector[i].probability * 100);
  }

  ROS_INFO("inference time: %fms", srv.response.objects.inference_time_ms);
  return 0;
}
