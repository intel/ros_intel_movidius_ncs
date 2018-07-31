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
             int network_dimension)
    : graph_buf_(graph_buf),
      network_dimension_(network_dimension),
      handle_(nullptr)
{
  allocate(device->getHandle());
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

void Graph::allocate(void* device_handle)
{
  int ret = mvncAllocateGraph(device_handle,
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
