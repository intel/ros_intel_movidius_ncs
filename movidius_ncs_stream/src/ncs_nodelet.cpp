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
#include <sensor_msgs/RegionOfInterest.h>
#include <pluginlib/class_list_macros.h>

#include <movidius_ncs_msgs/Objects.h>
#include <movidius_ncs_msgs/ObjectsInBoxes.h>
#include "movidius_ncs_stream/ncs_nodelet.h"
#include "movidius_ncs_lib/exception.h"
#include "movidius_ncs_lib/exception_util.h"

using image_transport::ImageTransport;
using movidius_ncs_lib::ClassificationResultPtr;
using movidius_ncs_lib::DetectionResultPtr;
using movidius_ncs_lib::Device;

namespace movidius_ncs_stream
{
NCSImpl::NCSImpl(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : ncs_handle_(nullptr),
      nh_(nh),
      pnh_(pnh),
      device_index_(0),
      log_level_(Device::Errors),
      cnn_type_(""),
      graph_file_path_(""),
      category_file_path_(""),
      network_dimension_(0),
      mean_(0),
      scale_(1.0),
      top_n_(1)
{
  getParameters();
  init();
}

NCSImpl::~NCSImpl()
{
}

void NCSImpl::getParameters()
{
  ROS_DEBUG("NCSImpl get parameters");

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

  if (!pnh_.getParam("cnn_type", cnn_type_))
  {
    ROS_WARN("param cnn_type not set, use default");
  }

  if (cnn_type_.compare("alexnet") && cnn_type_.compare("googlenet")
      && cnn_type_.compare("inception_v1") && cnn_type_.compare("inception_v2")
      && cnn_type_.compare("inception_v3") && cnn_type_.compare("inception_v4")
      && cnn_type_.compare("squeezenet") && cnn_type_.compare("tinyyolo_v1")
      && cnn_type_.compare("mobilenetssd"))
  {
    ROS_WARN_STREAM("invalid cnn_type_=" << cnn_type_);
    throw std::exception();
  }

  ROS_INFO_STREAM("use cnn_type_ = " << cnn_type_);

  if (!pnh_.getParam("network_dimension", network_dimension_))
  {
    ROS_WARN("param network_dimension not set, use default");
  }

  if (network_dimension_ < 0)
  {
    ROS_WARN_STREAM("invalid network_dimension = " << network_dimension_);
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
    ROS_INFO_STREAM("use " << mean_param_name << " = " << mean_val);
    mean_.push_back(mean_val);
  }

  if (!pnh_.getParam("top_n", top_n_))
  {
    ROS_WARN("param top_n not set, use default");
  }

  if (top_n_ < 1)
  {
    ROS_WARN_STREAM("invalid top_n = " << top_n_);
    throw std::exception();
  }

  ROS_INFO_STREAM("use top_n = " << top_n_);

  if (!pnh_.getParam("scale", scale_))
  {
    ROS_WARN("param scale not set, use default");
  }
  if (scale_ < 0)
  {
    ROS_WARN_STREAM("invalid param scale = " << scale_);
    throw std::exception();
  }

  ROS_INFO_STREAM("use scale = " << scale_);
}

void NCSImpl::init()
{
  ROS_DEBUG("NCSImpl onInit");
  ncs_handle_ = std::make_shared<movidius_ncs_lib::NCS>(device_index_,
                                                        static_cast<Device::LogLevel>(log_level_),
                                                        cnn_type_,
                                                        graph_file_path_,
                                                        category_file_path_,
                                                        network_dimension_,
                                                        mean_,
                                                        scale_,
                                                        top_n_);
  boost::shared_ptr<ImageTransport> it = boost::make_shared<ImageTransport>(nh_);

  if (!cnn_type_.compare("alexnet") || !cnn_type_.compare("googlenet")
      || !cnn_type_.compare("inception_v1") || !cnn_type_.compare("inception_v2")
      || !cnn_type_.compare("inception_v3") || !cnn_type_.compare("inception_v4")
      || !cnn_type_.compare("squeezenet"))
  {
    sub_ = it->subscribe("/camera/rgb/image_raw", 1, &NCSImpl::cbClassify, this);
    pub_ = nh_.advertise<movidius_ncs_msgs::Objects>("classified_objects", 1);
  }
  else
  {
    sub_ = it->subscribe("/camera/rgb/image_raw", 1, &NCSImpl::cbDetect, this);
    pub_ = nh_.advertise<movidius_ncs_msgs::ObjectsInBoxes>("detected_objects", 1);
  }
}

void NCSImpl::cbClassify(const sensor_msgs::ImageConstPtr& image_msg)
{
  if (pub_.getNumSubscribers() == 0)
  {
    ROS_DEBUG_STREAM("skip inferring for no subscriber");
    return;
  }

  cv::Mat cameraData = cv_bridge::toCvCopy(image_msg, "bgr8")->image;
  boost::posix_time::ptime start = boost::posix_time::microsec_clock::local_time();
  ncs_handle_->loadTensor(cameraData);
  ncs_handle_->classify();
  ClassificationResultPtr result = ncs_handle_->getClassificationResult();
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

void NCSImpl::cbDetect(const sensor_msgs::ImageConstPtr& image_msg)
{
  cv::Mat cameraData = cv_bridge::toCvCopy(image_msg, "bgr8")->image;
  boost::posix_time::ptime start = boost::posix_time::microsec_clock::local_time();
  ncs_handle_->loadTensor(cameraData);
  ncs_handle_->detect();
  DetectionResultPtr result = ncs_handle_->getDetectionResult();
  boost::posix_time::ptime end = boost::posix_time::microsec_clock::local_time();
  boost::posix_time::time_duration msdiff = end -start;
  movidius_ncs_msgs::ObjectsInBoxes objs_in_boxes;

  for (auto item : result->items_in_boxes)
  {
    movidius_ncs_msgs::ObjectInBox obj;
    obj.object.object_name = item.item.category;
    obj.object.probability = item.item.probability;
    obj.roi.x_offset = item.bbox.x;
    obj.roi.y_offset = item.bbox.y;
    obj.roi.width = item.bbox.width;
    obj.roi.height = item.bbox.height;
    objs_in_boxes.objects_vector.push_back(obj);
  }

  objs_in_boxes.header = image_msg->header;
  objs_in_boxes.inference_time_ms = result->time_taken;
  objs_in_boxes.fps = 1000.0 / msdiff.total_milliseconds();
  ROS_DEBUG_STREAM("Total time: " << msdiff.total_milliseconds() << "ms");
  ROS_DEBUG_STREAM("Inference time: " << objs_in_boxes.inference_time_ms << "ms");
  pub_.publish(objs_in_boxes);
}

NCSNodelet::~NCSNodelet()
{
}

void NCSNodelet::onInit()
{
  ros::NodeHandle nh = getNodeHandle();
  ros::NodeHandle pnh = getPrivateNodeHandle();
  try
  {
    impl_.reset(new NCSImpl(nh, pnh));
  }
  catch (movidius_ncs_lib::MvncException& e)
  {
    ROS_ERROR_STREAM("Error: " << e.what());
    ros::shutdown();
  }
  catch (...)
  {
    ROS_ERROR("exception caught while starting NCSNodelet");
    ros::shutdown();
  }
}

}  // namespace movidius_ncs_stream

PLUGINLIB_EXPORT_CLASS(movidius_ncs_stream::NCSNodelet, nodelet::Nodelet);
