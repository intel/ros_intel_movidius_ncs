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
#include <ros/console.h>
#include "movidius_ncs_lib/exception.h"
#include "movidius_ncs_lib/exception_util.h"
#include "movidius_ncs_lib/graph.h"
#include "movidius_ncs_lib/tensor.h"

namespace movidius_ncs_lib
{

Graph::Graph(const std::shared_ptr<Device>& device,
             const std::string& graph_buf,
             int input_size,
             const std::vector<float>& mean,
             const std::vector<float>& stddev,
             const std::vector<std::string> categories)
  : device_(device)
  , graph_buf_(graph_buf)
  , input_size_(input_size)
  , mean_(mean)
  , stddev_(stddev)
  , categories_(categories)
  , handle_(nullptr)
  , user_param_(nullptr)
{
  allocate();
}

Graph::~Graph()
{
  try
  {
    deallocate();
  }
  catch (MvncException& e)
  {
    ROS_ERROR_STREAM(e.what());
  }
}

void Graph::loadTensor(const Tensor::ConstPtr& tensor)
{
  assert(handle_ != nullptr);
  int ret = mvncLoadTensor(handle_,
                           tensor->raw(),
                           tensor->size(),
                           user_param_);
  ExceptionUtil::tryToThrowMvncException(ret);
}

ItemsPtr Graph::getDetectedItems()
{
  assert(handle_ != nullptr);
  uint16_t* probabilities;
  unsigned int length;
  int ret = mvncGetResult(handle_,
                          reinterpret_cast<void**>(&probabilities),
                          &length,
                          &user_param_);
  ExceptionUtil::tryToThrowMvncException(ret);
  std::vector<uint16_t> result_vector(reinterpret_cast<uint16_t*>(probabilities),
                                      reinterpret_cast<uint16_t*>(probabilities) + length);
  ItemsPtr items = std::make_shared<Items>();

  for (size_t index = 0; index < length / 2; ++index)
  {
    float fp32;
    Tensor::fp16tofp32(&fp32, probabilities[index]);
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
  return items;
}

std::string Graph::getDebugInfo()
{
  assert(handle_ != nullptr);
  char* debug_info;
  unsigned int length;
  int ret = mvncGetGraphOption(handle_,
                               MVNC_DEBUG_INFO,
                               reinterpret_cast<void**>(&debug_info),
                               &length);
  ExceptionUtil::tryToThrowMvncException(ret);
  std::string result(debug_info);
  return result;
}

float Graph::getTimeTaken()
{
  assert(handle_ != nullptr);
  float* time_taken;
  unsigned int length;
  int ret = mvncGetGraphOption(handle_,
                               MVNC_TIME_TAKEN,
                               reinterpret_cast<void**>(&time_taken),
                               &length);
  ExceptionUtil::tryToThrowMvncException(ret);
  length /= sizeof(*time_taken);
  float sum = 0;

  for (unsigned int i = 0; i < length; ++i)
  {
    sum += time_taken[i];
  }

  return sum;
}

void* Graph::getHandle()
{
  assert(handle_ != nullptr);
  return handle_;
}

void Graph::allocate()
{
  int ret = mvncAllocateGraph(device_->getHandle(),
                              &handle_,
                              graph_buf_.c_str(),
                              graph_buf_.size());
  ExceptionUtil::tryToThrowMvncException(ret);
}

void Graph::deallocate()
{
  int ret = mvncDeallocateGraph(handle_);
  ExceptionUtil::tryToThrowMvncException(ret);
}
}  // namespace movidius_ncs_lib
