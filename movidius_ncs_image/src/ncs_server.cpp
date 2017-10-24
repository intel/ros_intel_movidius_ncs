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

#include <string>
#include <vector>
#include <boost/filesystem/operations.hpp>
#include "movidius_ncs_image/ncs_server.h"

using movidius_ncs_lib::ResultPtr;
using movidius_ncs_lib::Device;

namespace movidius_ncs_image
{
NcsServer::NcsServer(ros::NodeHandle&  nh)
  : ncs_handle_(nullptr)
  , nh_(nh)
  , device_index_(0)
  , log_level_(Device::Errors)
  , graph_file_path_("")
  , category_file_path_("")
  , network_dimension_(0)
  , top_n_(3)
{
  getParameters();
  init();
}

void NcsServer::getParameters()
{
  ROS_DEBUG("NcsServer get parameters");

  if (!nh_.getParam("device_index", device_index_))
  {
    ROS_WARN("param device_index not set, use default");
  }

  if (device_index_ < 0)
  {
    ROS_ERROR_STREAM("invalid param device_index = " << device_index_);
    throw std::exception();
  }

  ROS_INFO_STREAM("use device_index = " << device_index_);

  if (!nh_.getParam("log_level", log_level_))
  {
    ROS_WARN("param log_level not set, use default");
  }

  if (log_level_ < Device::Nothing || log_level_ > Device::Verbose)
  {
    ROS_WARN_STREAM("invalid param log_level = " << log_level_);
    throw std::exception();
  }

  ROS_INFO_STREAM("use log_level = " << log_level_);

  if (!nh_.getParam("graph_file_path", graph_file_path_))
  {
    ROS_WARN("param graph_file_path not set, use default");
  }

  if (!boost::filesystem::exists(graph_file_path_))
  {
    ROS_ERROR_STREAM("graph_file_path = " << graph_file_path_ << " not exists");
    throw std::exception();
  }

  ROS_INFO_STREAM("use graph_file_path = " << graph_file_path_);

  if (!nh_.getParam("category_file_path", category_file_path_))
  {
    ROS_WARN("param category_file_path not set, use default");
  }

  if (!boost::filesystem::exists(category_file_path_))
  {
    ROS_ERROR_STREAM("category_file_path = " << category_file_path_ << " not exists");
    throw std::exception();
  }

  ROS_INFO_STREAM("use category_file_path = " << category_file_path_);

  if (!nh_.getParam("network_dimension", network_dimension_))
  {
    ROS_WARN("param network_dimension not set, use default");
  }

  if (network_dimension_ < 0)
  {
    ROS_WARN_STREAM("invalid network_dimension=" << network_dimension_);
    throw std::exception();
  }

  ROS_INFO_STREAM("use network_dimension = " << network_dimension_);

  for (int i = 0; i < 3; i++)
  {
    std::ostringstream oss;
    oss << "channel" << (i + 1) << "_mean";
    std::string mean_param_name = oss.str();
    float mean_val;
    if (!nh_.getParam(mean_param_name, mean_val))
    {
      ROS_WARN_STREAM("param " << mean_param_name << "not set, use default");
    }
    ROS_INFO_STREAM("use " << mean_param_name << "= " << mean_val);
    mean_.push_back(mean_val);
  }

  if (!nh_.getParam("top_n", top_n_))
  {
    ROS_WARN("param top_n not set, use default");
  }

  if (top_n_ < 1)
  {
    ROS_WARN_STREAM("invalid top_n=" << top_n_);
    throw std::exception();
  }

  ROS_INFO_STREAM("use top_n = " << top_n_);
}


void NcsServer::init()
{
  ROS_DEBUG("NcsServer init");
  ncs_handle_ = std::make_shared<movidius_ncs_lib::Ncs>(
                  device_index_,
                  static_cast<Device::LogLevel>(log_level_),
                  graph_file_path_,
                  category_file_path_,
                  network_dimension_,
                  mean_);
  service_ = nh_.advertiseService(
                  "classify_object",
                  &NcsServer::cbClassifyObject,
                  this);
}

bool NcsServer::cbClassifyObject(
        movidius_ncs_msgs::ClassifyObject::Request&   request,
        movidius_ncs_msgs::ClassifyObject::Response&  response)
{
  cv::Mat imageData = cv::imread(request.image_path);
  ResultPtr result = ncs_handle_->infer(imageData, top_n_);

  if (result == nullptr)
  {
    return false;
  }

  for (auto item : result->items)
  {
    movidius_ncs_msgs::Object obj;
    obj.object_name = item.category;
    obj.probability = item.probability;
    response.objects.objects_vector.push_back(obj);
  }

  response.objects.inference_time_ms = result->time_taken;
  return true;
}
}  // namespace movidius_ncs_image

int main(int argc, char** argv)
{
  ros::init(argc, argv, "movidius_ncs_node");
  ros::NodeHandle nh("~");

  try
  {
    movidius_ncs_image::NcsServer node(nh);
    ros::spin();
  }
  catch (...)
  {
    ROS_ERROR("exception caught in movidius_ncs_node");
  }
}
