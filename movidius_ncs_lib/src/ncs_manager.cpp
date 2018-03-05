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
#include <image_transport/image_transport.h>

#include "movidius_ncs_lib/ncs_manager.h"

namespace movidius_ncs_lib
{
NcsManager::NcsManager(int device_index,
                       Device::LogLevel log_level,
                       const std::string& cnn_type,
                       const std::string& graph_file_path,
                       const std::string& category_file_path,
                       const int network_dimension,
                       const std::vector<float>& mean,
                       const float& scale,
                       const int& top_n)
        : device_index_(device_index),
          log_level_(log_level),
          cnn_type_(cnn_type),
          graph_file_path_(graph_file_path),
          category_file_path_(category_file_path),
          network_dimension_(network_dimension),
          mean_(mean),
          scale_(scale),
          top_n_(top_n),
          user_param_(nullptr),
          device_count_(0)
{
  //****
  ROS_INFO("in ncs manager constructor");
  initDeviceManager();
}

NcsManager::~NcsManager()
{
}

void NcsManager::initDeviceManager()
{
  // new NCS instances

  char device_name[100];
  while (mvncGetDeviceName(device_count_, device_name, 100) != MVNC_DEVICE_NOT_FOUND)
  {
    auto ncs_handle = std::make_shared<movidius_ncs_lib::NCS>(device_count_,
                                                          static_cast<Device::LogLevel>(log_level_),
                                                          cnn_type_,
                                                          graph_file_path_,
                                                          category_file_path_,
                                                          network_dimension_,
                                                          mean_,
                                                          scale_,
                                                          top_n_);
    ncs_handle_list_.push_back(ncs_handle);
   // image or stream, move to inference
   // threads_.push_back(std::thread(&NcsManager::deviceThread, this, device_count_));
    device_count_++;
  }
 
}

void do_join(std::thread& t) 
{
  t.join();
}

void NcsManager::deviceThread(int device_index)
{
  //****
  ROS_INFO("inside thread %d", device_index);

  while(1)
  {
    //****
    //int while_count = 0;
    //ROS_INFO("inside while %d times", while_count++);

    if (!image_list_.empty())
    {
      //****
      ROS_INFO("image list is not empty now");

      mtx_.lock();

      auto first_image = image_list_[0].image;
      auto first_image_header = image_list_[0].header;

      if (!image_list_.empty())
      {
        image_list_.erase(image_list_.begin());
      }
      else
      {
        mtx_.unlock();
        continue;
      }
      mtx_.unlock();
      
      if (!cnn_type_.compare("alexnet") || !cnn_type_.compare("googlenet")
          || !cnn_type_.compare("inception_v1") || !cnn_type_.compare("inception_v2")
          || !cnn_type_.compare("inception_v3") || !cnn_type_.compare("inception_v4")
          || !cnn_type_.compare("mobilenet") || !cnn_type_.compare("squeezenet"))
      {
        ncs_handle_list_[device_index]->loadTensor(first_image);
        ncs_handle_list_[device_index]->classify();
        ClassificationResultPtr result = ncs_handle_list_[device_index]->getClassificationResult();
        //****
        ROS_INFO("get one result");

        (*p_c_)(result, first_image_header);

        //****
        ROS_INFO("return one result");
        
      }
      else
      {
        ncs_handle_list_[device_index]->loadTensor(first_image);
        ncs_handle_list_[device_index]->detect();
        DetectionResultPtr result = ncs_handle_list_[device_index]->getDetectionResult();
        (*p_d_)(result, first_image_header);
      }
     
    }
    else
    {
      //****
      //ROS_INFO("image list is empty now");
    }
  }
}

void NcsManager::startThreads()
{
  //****
  ROS_INFO("starting %d threads", device_count_);

  for(int i = 0; i < device_count_; i++)
  {
    threads_.push_back(std::thread(&NcsManager::deviceThread, this, i));
  }
    
  std::for_each(threads_.begin(), threads_.end(), do_join);

  //****
  ROS_INFO("started %d threads", device_count_);
}


std::vector<ClassificationResultPtr> NcsManager::classify_image(std::vector<std::string> images)
{
  //****
  for(unsigned int i = 0; i < images.size(); i++)
  {
    ROS_INFO("%u image name is %s", i, images[i].c_str());
  }

  //****
  ROS_INFO("begin ncs manager -> classify_image: begin ");
  ROS_INFO("device count is %d", device_count_);

  int image_size = int(images.size()); 

  std::vector<ClassificationResultPtr> results(image_size);

  int parallel_group = image_size / device_count_;
  int parallel_left = image_size % device_count_;

  //****
  ROS_INFO("total size is %d", image_size);
  ROS_INFO("group is %d", parallel_group);
  ROS_INFO("left is %d", parallel_left);

  //****
  ROS_INFO("ncs manager -> classify_image: before parallel");
  
  for (int i = 0; i < parallel_group; i++)
  {
    //****
    ROS_INFO("begin of loop i = %d", i);

    #pragma omp parallel for
    for (int device_index = 0; device_index < device_count_; device_index++)
    {
      //****
      ROS_INFO("read image: %d", i * device_count_ + device_index);
      cv::Mat imageData = cv::imread(images[i * device_count_ + device_index]);

      ROS_INFO("load %d image into No. %d NCS", i * device_count_ + device_index, device_index);
      ncs_handle_list_[device_index]->loadTensor(imageData);

      ROS_INFO("call classify in No. %d NCS", device_index);
      ncs_handle_list_[device_index]->classify();

      ROS_INFO("get result from No. %d NCS", device_index);
      ClassificationResultPtr result = ncs_handle_list_[device_index]->getClassificationResult();

      ROS_INFO("put result into %d of results", i * device_count_ + device_index);
      results[i * device_count_ + device_index] = result;
    }
    
    //****
    ROS_INFO("done for loop i = %d", i);
  }

  for (int j = 0; j < parallel_left; j++)
  {
    cv::Mat imageData = cv::imread(images[parallel_group * device_count_ + j]);
    ncs_handle_list_[j]->loadTensor(imageData);
    ncs_handle_list_[j]->classify();
    ClassificationResultPtr result = ncs_handle_list_[j]->getClassificationResult();
    results[parallel_group * device_count_ + j] = result;

    //****
    ROS_INFO("second for: device_index = %d", j);
  }
  
  //****
  ROS_INFO("ncs manager -> classify_image: after parallel and end");
  
  return results;
}

std::vector<DetectionResultPtr> NcsManager::detect_image(std::vector<std::string> images)
{
  //****
  ROS_INFO("begin ncs manager -> classify_image: begin ");
  ROS_INFO("device count is %d", device_count_);

  int image_size = int(images.size());
  std::vector<DetectionResultPtr> results(image_size);

  int parallel_group = image_size / device_count_;
  int parallel_left = image_size % device_count_;
  
  //****
  ROS_INFO("total size is %d", image_size);
  ROS_INFO("group is %d", parallel_group);
  ROS_INFO("left is %d", parallel_left);

  //****
  ROS_INFO("ncs manager -> classify_image: before parallel");

  for (int i = 0; i < parallel_group; i++)
  {
    //****
    ROS_INFO("begin first for at i = %d", i);

    //#pragma omp parallel for private (imageData)
    #pragma omp parallel for
    for (int device_index = 0; device_index < device_count_; device_index++)
    {
      cv::Mat imageData = cv::imread(images[i * device_count_ + device_index]);
      ncs_handle_list_[device_index]->loadTensor(imageData);
      ncs_handle_list_[device_index]->detect();
      DetectionResultPtr result = ncs_handle_list_[device_index]->getDetectionResult();
      results[i * device_count_ + device_index] = result;
    }

    for (int i = 0; i < parallel_left; i++)
    {
      cv::Mat imageData = cv::imread(images[parallel_group * device_count_ + i]);
      ncs_handle_list_[i]->loadTensor(imageData);
      ncs_handle_list_[i]->detect();
      DetectionResultPtr result = ncs_handle_list_[i]->getDetectionResult();
      results[parallel_group * device_count_ + i] = result;
    }
  }

  //****
  ROS_INFO("ncs manager -> classify_image: after parallel and end");

  return results;
}

void NcsManager::classify_stream(const cv::Mat& image, FUNP_C cbGetClassificationResult, const sensor_msgs::ImageConstPtr& image_msg)
{
  //****
  ROS_INFO("NCSManager: classify_stream is called");

  // regsiter callback
  p_c_ = cbGetClassificationResult;

  //
  Image_frame image_frame;
  image_frame.header = image_msg->header;
  image_frame.image = image;

  // push frame into buffer 
  mtx_.lock();
  image_list_.push_back(image_frame);
  mtx_.unlock();
}

void NcsManager::detect_stream(const cv::Mat& image, FUNP_D cbGetDetectionResult, const sensor_msgs::ImageConstPtr& image_msg)
{
  // regsiter callback
  p_d_ = cbGetDetectionResult;

  // 
  Image_frame image_frame;
  image_frame.header = image_msg->header;
  image_frame.image = image;

  // push frame into buffer 
  mtx_.lock();
  image_list_.push_back(image_frame);
  mtx_.unlock();
}
}  // namespace movidius_ncs_lib
