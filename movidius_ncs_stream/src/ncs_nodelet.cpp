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

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <boost/filesystem/operations.hpp>

#include <image_transport/image_transport.h>
#include <pluginlib/class_list_macros.h>

#include <movidius_ncs_msgs/Objects.h>
#include "movidius_ncs_stream/ncs_nodelet.h"

using image_transport::ImageTransport;
using movidius_ncs_lib::ResultPtr;
using movidius_ncs_lib::Device;

namespace movidius_ncs_stream
{
NcsImpl::NcsImpl(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : ncs_handle_(nullptr)
  , nh_(nh)
  , pnh_(pnh)
  , device_index_(0)
  , log_level_(Device::Errors)
  , graph_file_path_("")
  , category_file_path_("")
  , network_dimension_(0)
  , mean_({0, 0, 0})
  , top_n_(3)
{
  getParameters();
  init();
}

NcsImpl::~NcsImpl()
{
}

void NcsImpl::getParameters()
{
  ROS_DEBUG("NcsImpl get parameters");

  if (!pnh_.getParam("device_index", device_index_))
  {
    ROS_WARN("param device_index not set, use default");
  }

  if (device_index_ < 0)
  {
    ROS_ERROR_STREAM("invalid param device_index = " << device_index_);
    throw std::exception();
  }

  ROS_INFO_STREAM("use device_index = " << device_index_);

  if (!pnh_.getParam("log_level", log_level_))
  {
    ROS_WARN("param log_level not set, use default");
  }

  if (log_level_ < Device::Nothing || log_level_ > Device::Verbose)
  {
    ROS_WARN_STREAM("invalid param log_level = " << log_level_);
    throw std::exception();
  }

  ROS_INFO_STREAM("use log_level = " << log_level_);

  if (!pnh_.getParam("graph_file_path", graph_file_path_))
  {
    ROS_WARN("param graph_file_path not set, use default");
  }

  if (!boost::filesystem::exists(graph_file_path_))
  {
    ROS_ERROR_STREAM("graph_file_path = " << graph_file_path_ << " not exists");
    throw std::exception();
  }

  ROS_INFO_STREAM("use graph_file_path = " << graph_file_path_);

  if (!pnh_.getParam("category_file_path", category_file_path_))
  {
    ROS_WARN("param category_file_path not set, use default");
  }

  if (!boost::filesystem::exists(category_file_path_))
  {
    ROS_ERROR_STREAM("category_file_path = " << category_file_path_ << " not exists");
    throw std::exception();
  }

  ROS_INFO_STREAM("use category_file_path = " << category_file_path_);

  if (!pnh_.getParam("network_dimension", network_dimension_))
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
    if (!pnh_.getParam(mean_param_name, mean_val))
    {
      ROS_WARN_STREAM("param " << mean_param_name << "not set, use default");
    }
    ROS_INFO_STREAM("use " << mean_param_name << "= " << mean_val);
    mean_.push_back(mean_val);
  }

  if (!pnh_.getParam("top_n", top_n_))
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

void NcsImpl::init()
{
  ROS_DEBUG("NcsImpl onInit");
  ncs_handle_ = std::make_shared<movidius_ncs_lib::Ncs>(device_index_, static_cast<Device::LogLevel>(log_level_),
                                                        graph_file_path_, category_file_path_, network_dimension_,
                                                        mean_);
  boost::shared_ptr<ImageTransport> it = boost::make_shared<ImageTransport>(nh_);
  sub_ = it->subscribe("/camera/rgb/image_raw", 1, &NcsImpl::cbInfer, this);
  pub_ = nh_.advertise<movidius_ncs_msgs::Objects>("classified_object", 1);
}

void NcsImpl::cbInfer(const sensor_msgs::ImageConstPtr& image_msg)
{
  if (pub_.getNumSubscribers() == 0)
  {
    ROS_DEBUG_STREAM("skip inferring for no subscriber");
    return;
  }

  cv::Mat cameraData = cv_bridge::toCvCopy(image_msg, "bgr8")->image;
  boost::posix_time::ptime start = boost::posix_time::microsec_clock::local_time();
  ResultPtr result = ncs_handle_->infer(cameraData, top_n_);
  boost::posix_time::ptime end = boost::posix_time::microsec_clock::local_time();
  boost::posix_time::time_duration msdiff = end - start;
  movidius_ncs_msgs::Objects objs;

  for (auto item : result->items)
  {
    movidius_ncs_msgs::Object obj;
    obj.object_name = item.category;
    obj.probability = item.probability;
    objs.objects_vector.push_back(obj);
  }

  objs.header = image_msg->header;
  objs.inference_time_ms = result->time_taken;
  objs.fps = 1000.0 / msdiff.total_milliseconds();
  ROS_DEBUG_STREAM("Total time: " << msdiff.total_milliseconds() << "ms");
  ROS_DEBUG_STREAM("Inference time: " << objs.inference_time_ms << "ms");
  pub_.publish(objs);
}

NcsNodelet::~NcsNodelet()
{
}

void NcsNodelet::onInit()
{
  ros::NodeHandle nh = getNodeHandle();
  ros::NodeHandle pnh = getPrivateNodeHandle();

  try
  {
    impl_.reset(new NcsImpl(nh, pnh));
  }
  catch (...)
  {
    ROS_ERROR("exception caught while starting NcsNodelet");
    ros::shutdown();
  }
}

}  // namespace movidius_ncs_stream

PLUGINLIB_EXPORT_CLASS(movidius_ncs_stream::NcsNodelet, nodelet::Nodelet);
