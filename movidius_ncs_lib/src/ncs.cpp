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
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>

#include "movidius_ncs_lib/exception.h"
#include "movidius_ncs_lib/inference.h"
#include "movidius_ncs_lib/ncs.h"

namespace movidius_ncs_lib
{
Ncs::Ncs(int device_index,
         Device::LogLevel log_level,
         const std::string& cnn_type,
         const std::string& graph_file_path,
         const std::string& category_file_path,
         const int network_dimension,
         const std::vector<float>& mean)
  : device_(nullptr)
  , graph_(nullptr)
  , device_index_(device_index)
  , log_level_(log_level)
  , cnn_type_(cnn_type)
  , graph_file_path_(graph_file_path)
  , category_file_path_(category_file_path)
  , network_dimension_(network_dimension)
  , mean_(mean)
{
  init();
}

Ncs::~Ncs()
{
}

ClassificationResultPtr Ncs::classify(cv::Mat image, uint32_t top_n)
{
  ROS_DEBUG("Ncs::classify");

  try
  {
    Tensor::Ptr tensor = std::make_shared<Tensor>(
                    image,
                    graph_->getMean(),
                    std::pair<int, int>(graph_->getNetworkDim(), graph_->getNetworkDim()),
                    cnn_type_);
    Inference inference(top_n, tensor, graph_, device_);
    return inference.classify();
  }
  catch (std::exception& e)
  {
    ROS_ERROR_STREAM("inference error: " << e.what());
  }

  return nullptr;
}

DetectionResultPtr Ncs::detect(cv::Mat image)
{
  ROS_DEBUG("Ncs::detect");

  try
  {
    Tensor::Ptr tensor = std::make_shared<Tensor>(
                    image,
                    graph_->getMean(),
                    std::pair<int, int>(graph_->getNetworkDim(), graph_->getNetworkDim()),
                    cnn_type_);
    Inference inference(0, tensor, graph_, device_);
    return inference.detect();
  }
  catch (std::exception& e)
  {
    ROS_ERROR_STREAM("inference error: " << e.what());
  }

  return nullptr;
}

void Ncs::init()
{
  ROS_DEBUG("Ncs::init called");
  getDevice();
  loadGraph();
}

void Ncs::loadGraph()
{
  std::vector<std::string> categories = loadCategories();
  std::string graph = getFileContent(graph_file_path_);
  std::vector<float> scaled_mean;
  for (auto &i : mean_)
  {
    scaled_mean.push_back(i * 255.0);
  }
  graph_.reset(new Graph(device_,
                         cnn_type_,
                         graph,
                         network_dimension_,
                         scaled_mean,
                         categories));
}

void Ncs::getDevice()
{
  device_.reset(new Device(device_index_,
                           static_cast<Device::LogLevel>(log_level_)));
}

std::vector<std::string> Ncs::loadCategories()
{
  try
  {
    std::string content = getFileContent(category_file_path_);
    std::vector<std::string> lines;
    splitIntoLines(content, lines);
    std::string first = lines[0];
    boost::trim_right(first);

    if (boost::iequals(first, "classes"))
    {
      lines.erase(lines.begin());
    }

    return lines;
  }
  catch (int& e)
  {
    throw NcsLoadCategoriesError();
  }
}

void Ncs::splitIntoLines(const std::string& content,
                         std::vector<std::string>& lines)
{
  std::stringstream ss(content);
  std::string line;

  while (std::getline(ss, line, '\n'))
  {
    lines.push_back(line);
  }
}

std::string Ncs::getFileContent(const std::string& filename)
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
