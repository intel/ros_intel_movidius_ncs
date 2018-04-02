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
NCSManager::NCSManager(const int& max_device_number, const int& start_device_index, const Device::LogLevel& log_level,
                       const std::string& cnn_type, const std::string& graph_file_path,
                       const std::string& category_file_path, const int& network_dimension,
                       const std::vector<float>& mean, const float& scale, const int& top_n)
  : max_device_number_(max_device_number)
  , start_device_index_(start_device_index)
  , log_level_(log_level)
  , cnn_type_(cnn_type)
  , graph_file_path_(graph_file_path)
  , category_file_path_(category_file_path)
  , network_dimension_(network_dimension)
  , mean_(mean)
  , scale_(scale)
  , top_n_(top_n)
  , user_param_(nullptr)
  , device_count_(0)
{
  initDeviceManager();
}

NCSManager::~NCSManager()
{
}

void NCSManager::initDeviceManager()
{
  char device_name[MVNC_MAX_NAME_SIZE];
  while (mvncGetDeviceName(device_count_, device_name, sizeof(device_name)) != MVNC_DEVICE_NOT_FOUND)
  {
    device_count_++;

    if (device_count_ == max_device_number_)
    {
      break;
    }
  }

  assert(start_device_index_ <= device_count_);

  for (int device_index = start_device_index_; device_index < device_count_; device_index++)
  {
    auto ncs_handle = std::make_shared<movidius_ncs_lib::NCS>(device_index, static_cast<Device::LogLevel>(log_level_),
                                                              cnn_type_, graph_file_path_, category_file_path_,
                                                              network_dimension_, mean_, scale_, top_n_);
    ncs_handle_list_.push_back(ncs_handle);
  }
}

void join(std::thread& t)
{
  t.join();
}

void NCSManager::deviceThread(int device_index)
{
  while (1)
  {
    if (!image_list_.empty())
    {
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

      if (!cnn_type_.compare("alexnet") || !cnn_type_.compare("googlenet") || !cnn_type_.compare("inception_v1") ||
          !cnn_type_.compare("inception_v2") || !cnn_type_.compare("inception_v3") ||
          !cnn_type_.compare("inception_v4") || !cnn_type_.compare("mobilenet") || !cnn_type_.compare("squeezenet"))
      {
        ncs_handle_list_[device_index]->loadTensor(first_image);
        ncs_handle_list_[device_index]->classify();
        ClassificationResultPtr result = ncs_handle_list_[device_index]->getClassificationResult();

        p_c_(result, first_image_header);
      }
      else
      {
        ncs_handle_list_[device_index]->loadTensor(first_image);
        ncs_handle_list_[device_index]->detect();
        DetectionResultPtr result = ncs_handle_list_[device_index]->getDetectionResult();

        p_d_(result, first_image_header);
      }
    }
    else
    {
      usleep(1);
    }
  }
}

void NCSManager::startThreads()
{
  for (int i = 0; i < device_count_ - start_device_index_; i++)
  {
    threads_.push_back(std::thread(&NCSManager::deviceThread, this, i));
  }

  std::for_each(threads_.begin(), threads_.end(), join);

  ROS_INFO("started %d threads", device_count_ - start_device_index_);
}

std::vector<ClassificationResultPtr> NCSManager::classifyImage(const std::vector<std::string>& images)
{
  int image_size = static_cast<int>(images.size());
  std::vector<ClassificationResultPtr> results(image_size);

  int parallel_group = image_size / (device_count_ - start_device_index_);
  int parallel_left = image_size % (device_count_ - start_device_index_);

  for (int i = 0; i < parallel_group; i++)
  {
#pragma omp parallel for
    for (int device_index = 0; device_index < device_count_ - start_device_index_; device_index++)
    {
      cv::Mat imageData = cv::imread(images[i * (device_count_ - start_device_index_) + device_index]);
      ncs_handle_list_[device_index]->loadTensor(imageData);
      ncs_handle_list_[device_index]->classify();
      ClassificationResultPtr result = ncs_handle_list_[device_index]->getClassificationResult();
      results[i * (device_count_ - start_device_index_) + device_index] =
          std::make_shared<ClassificationResult>(*result);
    }
  }

  for (int j = 0; j < parallel_left; j++)
  {
    cv::Mat imageData = cv::imread(images[parallel_group * (device_count_ - start_device_index_) + j]);
    ncs_handle_list_[j]->loadTensor(imageData);
    ncs_handle_list_[j]->classify();
    ClassificationResultPtr result = ncs_handle_list_[j]->getClassificationResult();
    results[parallel_group * (device_count_ - start_device_index_) + j] =
        std::make_shared<ClassificationResult>(*result);
  }

  return results;
}

std::vector<DetectionResultPtr> NCSManager::detectImage(const std::vector<std::string>& images)
{
  int image_size = static_cast<int>(images.size());
  std::vector<DetectionResultPtr> results(image_size);

  int parallel_group = image_size / (device_count_ - start_device_index_);
  int parallel_left = image_size % (device_count_ - start_device_index_);

  for (int i = 0; i < parallel_group; i++)
  {
#pragma omp parallel for
    for (int device_index = 0; device_index < device_count_ - start_device_index_; device_index++)
    {
      cv::Mat imageData = cv::imread(images[i * (device_count_ - start_device_index_) + device_index]);
      ncs_handle_list_[device_index]->loadTensor(imageData);
      ncs_handle_list_[device_index]->detect();
      DetectionResultPtr result = ncs_handle_list_[device_index]->getDetectionResult();
      results[i * (device_count_ - start_device_index_) + device_index] = std::make_shared<DetectionResult>(*result);
    }
  }

  for (int i = 0; i < parallel_left; i++)
  {
    cv::Mat imageData = cv::imread(images[parallel_group * (device_count_ - start_device_index_) + i]);
    ncs_handle_list_[i]->loadTensor(imageData);
    ncs_handle_list_[i]->detect();
    DetectionResultPtr result = ncs_handle_list_[i]->getDetectionResult();
    results[parallel_group * (device_count_ - start_device_index_) + i] = std::make_shared<DetectionResult>(*result);
  }

  return results;
}

void NCSManager::classifyStream(const cv::Mat& image, FUNP_C cbGetClassificationResult,
                                const sensor_msgs::ImageConstPtr& image_msg)
{
  p_c_ = cbGetClassificationResult;

  ImageFrame imageFrame;
  imageFrame.header = image_msg->header;
  imageFrame.image = image;

  mtx_.lock();
  if (image_list_.size() > IMAGE_BUFFER_SIZE)
  {
    image_list_.erase(image_list_.begin());
  }
  image_list_.push_back(imageFrame);
  mtx_.unlock();
}

void NCSManager::detectStream(const cv::Mat& image, FUNP_D cbGetDetectionResult,
                              const sensor_msgs::ImageConstPtr& image_msg)
{
  p_d_ = cbGetDetectionResult;

  ImageFrame imageFrame;
  imageFrame.header = image_msg->header;
  imageFrame.image = image;

  mtx_.lock();
  if (image_list_.size() > IMAGE_BUFFER_SIZE)
  {
    image_list_.erase(image_list_.begin());
  }
  image_list_.push_back(imageFrame);
  mtx_.unlock();
}
}  // namespace movidius_ncs_lib
