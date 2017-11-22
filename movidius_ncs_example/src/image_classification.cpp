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

#include <ros/ros.h>
#include <movidius_ncs_msgs/ClassifyObject.h>

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
  client = n.serviceClient<movidius_ncs_msgs::ClassifyObject>("/movidius_ncs_image/classify_object");
  movidius_ncs_msgs::ClassifyObject srv;
  srv.request.image_path = argv[1];

  if (!client.call(srv))
  {
    ROS_ERROR("failed to call service ClassifyObject");
    return 1;
  }

  for (unsigned int i = 0; i < srv.response.objects.objects_vector.size(); i++)
  {
    ROS_INFO("%d: object: %s  probability: %lf%%", i,
             srv.response.objects.objects_vector[i].object_name.c_str(),
             srv.response.objects.objects_vector[i].probability * 100);
  }

  ROS_INFO("inference time: %fms", srv.response.objects.inference_time_ms);
  return 0;
}
