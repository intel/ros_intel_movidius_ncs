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

#include <algorithm>
#include <string>
#include <utility>
#include <vector>
#include <fstream>
#include <streambuf>
#include <cerrno>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include "movidius_ncs_lib/exception.h"
#include "movidius_ncs_lib/exception_util.h"
#include "movidius_ncs_lib/ncs.h"
#include "movidius_ncs_lib/tensor.h"

namespace movidius_ncs_lib
{
NCS::NCS(int device_index,
         Device::LogLevel log_level,
         const std::string& cnn_type,
         const std::string& graph_file_path,
         const std::string& category_file_path,
         const int network_dimension,
         const std::vector<float>& mean,
         const float& scale,
         const int& top_n)
    : device_(nullptr),
      graph_(nullptr),
      tensor_(nullptr),
      device_index_(device_index),
      log_level_(log_level),
      cnn_type_(cnn_type),
      network_dimension_(network_dimension),
      mean_(mean),
      scale_(scale),
      top_n_(top_n),
      user_param_(nullptr)
{
  initDevice();
  loadGraph(graph_file_path);
  loadCategories(category_file_path);
  tensor_ = std::make_shared<Tensor>(std::pair<int, int>(network_dimension_, network_dimension_),
                                     mean_, scale_);
  result_ = std::make_shared<Result>(cnn_type_);
}

NCS::~NCS()
{
}

void NCS::classify()
{
  assert(graph_->getHandle() != nullptr);
  try
  {
    uint16_t* probabilities;
    unsigned int length;
    int ret = mvncGetResult(graph_->getHandle(), reinterpret_cast<void**>(&probabilities),
                            &length, &user_param_);
    ExceptionUtil::tryToThrowMvncException(ret);
    std::vector<uint16_t> result_vector(reinterpret_cast<uint16_t*>(probabilities),
                                        reinterpret_cast<uint16_t*>(probabilities) + length);
    ItemsPtr items = std::make_shared<Items>();
    for (size_t index = 0; index < length / 2; ++index)
    {
      float fp32;
#ifdef SUPPORT_F16C
      fp32 = _cvtsh_ss(probabilities[index]);
#else
      Tensor::fp16tofp32(&fp32, probabilities[index]);
#endif
      Item item;
      item.category = categories_[index];
      item.probability = fp32;
      items->push_back(item);
    }

    auto cmp = [](const Item & a, const Item & b)
    {
      return a.probability > b.probability;
    };
    std::sort(items->begin(), items->end(), cmp);

    if (!result_->getClassificationResult()->items.empty())
    {
      result_->getClassificationResult()->items.clear();
    }

    for (auto i : *items)
    {
      result_->setClassificationResult(i);
      if (static_cast<int>(result_->getClassificationResult()->items.size()) == top_n_)
      {
        break;
      }
    }
    result_->setClassificationResult(graph_->getTimeTaken());
    device_->monitorThermal();
  }
  catch (MvncMyriadError& e)
  {
    ROS_ERROR_STREAM(e.what());
    std::string debug_info = graph_->getDebugInfo();
    ROS_ERROR_STREAM("myriad debug info: " << debug_info);
  }
  catch (NCSException& e)
  {
    ROS_ERROR_STREAM(e.what());
  }
}

void NCS::detect()
{
  assert(graph_->getHandle() != nullptr);
  try
  {
    uint16_t* result;
    unsigned int length;
    int ret = mvncGetResult(graph_->getHandle(),
                            reinterpret_cast<void**>(&result),
                            &length,
                            &user_param_);
    ExceptionUtil::tryToThrowMvncException(ret);

    std::vector<uint16_t> result16_vector(reinterpret_cast<uint16_t*>(result),
                                          reinterpret_cast<uint16_t*>(result) + length / 2);
    std::vector<float> result32_vector;

    for (auto fp16 : result16_vector)
    {
      float fp32;
#ifdef SUPPORT_F16C
      fp32 = _cvtsh_ss(fp16);
#else
      Tensor::fp16tofp32(&fp32, fp16);
#endif
      result32_vector.push_back(fp32);
    }

    if (!cnn_type_.compare("tinyyolo_v1"))
    {
      result_->parseYoloResult(result32_vector, categories_, tensor_->getImageWidth(), tensor_->getImageHeight());
      result_->setDetectionResult(graph_->getTimeTaken());
    }
    else if (!cnn_type_.compare("mobilenetssd"))
    {
      result_->parseSSDResult(result32_vector, categories_, tensor_->getImageWidth(), tensor_->getImageHeight());
      result_->setDetectionResult(graph_->getTimeTaken());
    }
    else
    {
      ROS_ERROR("unsupported cnn model");
    }
    device_->monitorThermal();
  }
  catch (MvncMyriadError& e)
  {
    ROS_ERROR_STREAM(e.what());
    std::string debug_info = graph_->getDebugInfo();
    ROS_ERROR_STREAM("myriad debug info: " << debug_info);
  }
  catch (std::exception& e)
  {
    ROS_ERROR_STREAM(e.what());
  }
}

void NCS::loadTensor(const cv::Mat& image)
{
  tensor_->clearTensor();
  tensor_->loadImageData(image);

  assert(graph_->getHandle() != nullptr);
  int ret = mvncLoadTensor(graph_->getHandle(),
                           tensor_->raw(),
                           tensor_->size(),
                           user_param_);
  ExceptionUtil::tryToThrowMvncException(ret);
}

ClassificationResultPtr NCS::getClassificationResult()
{
  return result_->getClassificationResult();
}

DetectionResultPtr NCS::getDetectionResult()
{
  return result_->getDetectionResult();
}

void NCS::initDevice()
{
  device_.reset(new Device(device_index_,
                           static_cast<Device::LogLevel>(log_level_)));
}

void NCS::loadGraph(const std::string& graph_file_path)
{
  std::string graph = getFileContent(graph_file_path);
  graph_.reset(new Graph(device_, graph, network_dimension_));

  ROS_INFO("graph is loaded");
}

void NCS::loadCategories(const std::string& category_file_path)
{
  try
  {
    std::string content = getFileContent(category_file_path);
    splitIntoLines(content, categories_);
    std::string first = categories_[0];
    boost::trim_right(first);

    if (boost::iequals(first, "classes"))
    {
      categories_.erase(categories_.begin());
    }
  }
  catch (int& e)
  {
    throw NCSLoadCategoriesError();
  }
}

void NCS::splitIntoLines(const std::string& content,
                         std::vector<std::string>& lines)
{
  std::stringstream ss(content);
  std::string line;

  while (std::getline(ss, line, '\n'))
  {
    lines.push_back(line);
  }
}

std::string NCS::getFileContent(const std::string& filename)
{
  std::ifstream in(filename.c_str(), std::ios::in | std::ios::binary);

  if (!in)
  {
    throw(errno);
  }

  std::string content;
  in.seekg(0, std::ios::end);
  content.reserve(in.tellg());
  in.seekg(0, std::ios::beg);
  content.assign(std::istreambuf_iterator<char>(in),
                 std::istreambuf_iterator<char>());
  in.close();
  return content;
}
}  // namespace movidius_ncs_lib
