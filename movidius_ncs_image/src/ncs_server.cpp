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
#include <object_msgs/Object.h>
#include <object_msgs/Objects.h>
#include <object_msgs/ObjectInBox.h>
#include <object_msgs/ObjectsInBoxes.h>
#include "movidius_ncs_image/ncs_server.h"
#include <cv_bridge/cv_bridge.h>

using movidius_ncs_lib::ClassificationResultPtr;
using movidius_ncs_lib::DetectionResultPtr;
using movidius_ncs_lib::Device;

namespace movidius_ncs_image
{
NCSServer::NCSServer(ros::NodeHandle& nh)
    : ncs_handle_(nullptr),
      nh_(nh),
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

void NCSServer::getParameters()
{
  ROS_DEBUG("NCSServer get parameters");

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

  if (!nh_.getParam("cnn_type", cnn_type_))
  {
    ROS_WARN("param cnn_type not set, use default");
  }

  if (cnn_type_.compare("alexnet") && cnn_type_.compare("googlenet")
      && cnn_type_.compare("inception_v1") && cnn_type_.compare("inception_v2")
      && cnn_type_.compare("inception_v3") && cnn_type_.compare("inception_v4")
      && cnn_type_.compare("mobilenet") && cnn_type_.compare("squeezenet")
      && cnn_type_.compare("tinyyolo_v1") && cnn_type_.compare("mobilenetssd"))
  {
    ROS_WARN_STREAM("invalid cnn_type_=" << cnn_type_);
    throw std::exception();
  }

  ROS_INFO_STREAM("use cnn_type_ = " << cnn_type_);

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
    if (!nh_.getParam(mean_param_name, mean_val))
    {
      ROS_WARN_STREAM("param " << mean_param_name << "not set, use default");
    }
    ROS_INFO_STREAM("use " << mean_param_name << " = " << mean_val);
    mean_.push_back(mean_val);
  }

  if (!nh_.getParam("top_n", top_n_))
  {
    ROS_WARN("param top_n not set, use default");
  }

  if (top_n_ < 1)
  {
    ROS_WARN_STREAM("invalid top_n = " << top_n_);
    throw std::exception();
  }

  ROS_INFO_STREAM("use top_n = " << top_n_);

  if (!nh_.getParam("scale", scale_))
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


void NCSServer::init()
{
  ROS_DEBUG("NCSServer init");
  ncs_handle_ = std::make_shared<movidius_ncs_lib::NCS>(device_index_,
                                                        static_cast<Device::LogLevel>(log_level_),
                                                        cnn_type_,
                                                        graph_file_path_,
                                                        category_file_path_,
                                                        network_dimension_,
                                                        mean_,
                                                        scale_,
                                                        top_n_);
  if (!cnn_type_.compare("alexnet") || !cnn_type_.compare("googlenet")
      || !cnn_type_.compare("inception_v1") || !cnn_type_.compare("inception_v2")
      || !cnn_type_.compare("inception_v3") || !cnn_type_.compare("inception_v4")
      || !cnn_type_.compare("mobilenet") || !cnn_type_.compare("squeezenet"))
  {
    service_ = nh_.advertiseService("classify_object",
                                    &NCSServer::cbClassifyObject,
                                    this);
  }
  else
  {
    service_ = nh_.advertiseService("detect_object",
                                    &NCSServer::cbDetectObject,
                                    this);
  }
}

bool NCSServer::cbClassifyObject(object_msgs::ClassifyObject::Request& request,
                                 object_msgs::ClassifyObject::Response& response)
{
  cv::Mat imageData = cv_bridge::toCvCopy(request.image, "bgr8")->image;
  ncs_handle_->loadTensor(imageData);
  ncs_handle_->classify();
  ClassificationResultPtr result = ncs_handle_->getClassificationResult();

  if (result == nullptr)
  {
    return false;
  }

  for (auto item : result->items)
  {
    object_msgs::Object obj;
    obj.object_name = item.category;
    obj.probability = item.probability;
    response.objects.objects_vector.push_back(obj);
  }

  response.objects.inference_time_ms = result->time_taken;
  return true;
}

bool NCSServer::cbDetectObject(object_msgs::DetectObject::Request& request,
                               object_msgs::DetectObject::Response& response)
{
  cv::Mat imageData = cv_bridge::toCvCopy(request.image, "bgr8")->image;
  ncs_handle_->loadTensor(imageData);
  ncs_handle_->detect();
  DetectionResultPtr result = ncs_handle_->getDetectionResult();

  if (result == nullptr)
  {
    return false;
  }

  for (auto item : result->items_in_boxes)
  {
    object_msgs::ObjectInBox obj;
    obj.object.object_name = item.item.category;
    obj.object.probability = item.item.probability;
    obj.roi.x_offset = item.bbox.x;
    obj.roi.y_offset = item.bbox.y;
    obj.roi.width = item.bbox.width;
    obj.roi.height = item.bbox.height;
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
    movidius_ncs_image::NCSServer node(nh);
    ros::spin();
  }
  catch (...)
  {
    ROS_ERROR("exception caught in movidius_ncs_node");
  }
}
