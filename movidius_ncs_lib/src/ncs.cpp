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
         const std::string& network_conf_path)
  : device_(nullptr)
  , graph_(nullptr)
  , device_index_(device_index)
  , log_level_(log_level)
  , base_path_(appendPathSeparator(network_conf_path))
  , graph_file_path_(base_path_ + "/graph")
  , stat_file_path_(base_path_ + "/stat.txt")
  , inputsize_file_path_(base_path_ + "/inputsize.txt")
  , categories_file_path_(base_path_ + "/categories.txt")
{
  init();
}

Ncs::~Ncs()
{
}

ResultPtr Ncs::infer(cv::Mat image, uint32_t top_n)
{
  ROS_DEBUG("Ncs::infer");

  try
  {
    Tensor::Ptr tensor = std::make_shared<Tensor>(
                    image,
                    graph_->getMean(),
                    graph_->getStddev(),
                    std::pair<int, int>(graph_->getInputSize(), graph_->getInputSize()));
    Inference inference(top_n, tensor, graph_, device_);
    return inference.run();
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
  int inputsize = loadInputSize();
  std::vector<std::string> categories = loadCategories();
  std::string graph = getFileContent(graph_file_path_);
  std::vector<float> mean;
  std::vector<float> stddev;
  loadStatistics(mean, stddev);
  graph_.reset(new Graph(device_,
                         graph,
                         inputsize,
                         mean,
                         stddev,
                         categories));
}

void Ncs::getDevice()
{
  device_.reset(new Device(device_index_,
                           static_cast<Device::LogLevel>(log_level_)));
}

void Ncs::loadStatistics(std::vector<float>& mean,
                         std::vector<float>& stddev)
{
  std::ifstream fs(stat_file_path_);
  std::vector<float> vec;

  if (!fs.is_open())
  {
    throw(errno);
  }

  float value;

  while (!fs.eof())
  {
    fs >> value;
    vec.push_back(value);
  }

  fs.close();

  for (int i = 0; i < 3; i++)
  {
    mean.push_back(vec[i] * 255.0);
    stddev.push_back(1.0 / (vec[i + 3] * 255.0));
  }
}

int Ncs::loadInputSize()
{
  try
  {
    std::string content = getFileContent(inputsize_file_path_);
    boost::trim_right(content);
    return boost::lexical_cast<int>(content);
  }
  catch (int& e)
  {
    throw NcsInputSizeFileError();
  }
  catch (boost::bad_lexical_cast const&)
  {
    throw NcsInputSizeError();
  }
}

std::vector<std::string> Ncs::loadCategories()
{
  try
  {
    std::string content = getFileContent(categories_file_path_);
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

std::string Ncs::appendPathSeparator(const std::string& path)
{
  if (path[path.length()] == '/')
  {
    return path;
  }

  return path + "/";
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
