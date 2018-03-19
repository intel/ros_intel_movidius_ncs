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

#ifndef MOVIDIUS_NCS_LIB_NCS_MANAGER_H
#define MOVIDIUS_NCS_LIB_NCS_MANAGER_H

#include <string>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <vector>
#include <thread>

#include "boost/bind.hpp"
#include "boost/function.hpp"

#include "movidius_ncs_lib/device.h"
#include "movidius_ncs_lib/ncs.h"

namespace movidius_ncs_lib
{
typedef boost::function<void(ClassificationResultPtr result, std_msgs::Header header)> FUNP_C;
typedef boost::function<void(DetectionResultPtr result, std_msgs::Header header)> FUNP_D;

struct ImageFrame
{
  cv::Mat image;
  std_msgs::Header header;
};

class NcsManager
{
public:
  NcsManager(int max_device_number, int device_index, Device::LogLevel log_level, const std::string& cnn_type,
             const std::string& graph_file_path, const std::string& category_file_path, const int network_dimension,
             const std::vector<float>& mean, const float& scale, const int& top_n);
  ~NcsManager();

  void startThreads();

  void classify_stream(const cv::Mat& image, FUNP_C cbGetClassificationResult,
                       const sensor_msgs::ImageConstPtr& image_msg);
  void detect_stream(const cv::Mat& image, FUNP_D cbGetDetectionResult, const sensor_msgs::ImageConstPtr& image_msg);
  std::vector<ClassificationResultPtr> classify_image(std::vector<std::string> images);
  std::vector<DetectionResultPtr> detect_image(std::vector<std::string> images);

private:
  void initDeviceManager();
  void deviceThread(int device_index);

  int max_device_number_;
  int device_index_;
  const Device::LogLevel log_level_;
  const std::string cnn_type_;
  const std::string& graph_file_path_;
  const std::string& category_file_path_;
  const int network_dimension_;
  const std::vector<float> mean_;
  const float scale_;
  const int top_n_;
  void* user_param_;

  std::vector<std::shared_ptr<movidius_ncs_lib::NCS>> ncs_handle_list_;
  int device_count_;

  FUNP_C p_c_;
  FUNP_D p_d_;

  std::vector<ImageFrame> image_list_;
  std::mutex mtx_;
  std::vector<std::thread> threads_;
};
}  // namespace movidius_ncs_lib
#endif  // MOVIDIUS_NCS_LIB_NCS_MANAGER_H
