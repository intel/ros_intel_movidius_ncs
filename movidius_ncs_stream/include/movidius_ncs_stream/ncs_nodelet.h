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

#ifndef MOVIDIUS_NCS_STREAM_NCS_NODELET_H
#define MOVIDIUS_NCS_STREAM_NCS_NODELET_H

#include <string>
#include <vector>

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include "movidius_ncs_lib/ncs.h"

namespace movidius_ncs_stream
{
class NCSImpl
{
public:
  NCSImpl(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~NCSImpl();

private:
  void cbClassify(const sensor_msgs::ImageConstPtr& image);
  void cbDetect(const sensor_msgs::ImageConstPtr& image);
  void getParameters();
  void init();

  std::shared_ptr<movidius_ncs_lib::NCS> ncs_handle_;

  ros::Publisher pub_;
  image_transport::Subscriber sub_;
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  int device_index_;
  int log_level_;
  std::string graph_file_path_;
  std::string category_file_path_;
  int network_dimension_;
  std::string cnn_type_;
  std::vector<float> mean_;
  int top_n_;
};

class NCSNodelet: public nodelet::Nodelet
{
public:
  virtual ~NCSNodelet();

private:
  virtual void onInit();

  std::shared_ptr<NCSImpl> impl_;
};
}  // namespace movidius_ncs_stream

#endif  // MOVIDIUS_NCS_STREAM_NCS_NODELET_H
