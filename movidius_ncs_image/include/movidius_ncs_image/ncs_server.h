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

#ifndef MOVIDIUS_NCS_IMAGE_NCS_SERVER_H
#define MOVIDIUS_NCS_IMAGE_NCS_SERVER_H

#include <string>
#include <vector>

#include <ros/ros.h>

#include <object_msgs/ClassifyObject.h>
#include <object_msgs/DetectObject.h>
#include "movidius_ncs_lib/ncs.h"
#include "movidius_ncs_lib/ncs_manager.h"

namespace movidius_ncs_image
{
class NCSServer
{
public:
  explicit NCSServer(ros::NodeHandle& nh);

private:
  void getParameters();
  void init();

  bool cbDetectObject(object_msgs::DetectObject::Request& request, object_msgs::DetectObject::Response& response);
  bool cbClassifyObject(object_msgs::ClassifyObject::Request& request, object_msgs::ClassifyObject::Response& response);

  ros::ServiceServer service_;

  std::shared_ptr<movidius_ncs_lib::NCSManager> ncs_manager_handle_;
  ros::NodeHandle nh_;

  int max_device_number_;
  int start_device_index_;
  int log_level_;
  std::string cnn_type_;
  std::string graph_file_path_;
  std::string category_file_path_;
  int network_dimension_;
  std::vector<float> mean_;
  float scale_;
  int top_n_;
};
}  // namespace movidius_ncs_image

#endif  // MOVIDIUS_NCS_IMAGE_NCS_SERVER_H
