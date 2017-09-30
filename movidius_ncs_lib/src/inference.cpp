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
#include <memory>
#include <ros/ros.h>
#include "movidius_ncs_lib/exception.h"
#include "movidius_ncs_lib/inference.h"

namespace movidius_ncs_lib
{
Inference::Inference(unsigned int top_n,
                     Tensor::Ptr& tensor,
                     Graph::Ptr& graph,
                     Device::Ptr& device)
  : top_n_(top_n)
  , tensor_(tensor)
  , graph_(graph)
  , device_(device)
{
}

ResultPtr Inference::run()
{
  ResultPtr result = std::make_shared<Result>();

  try
  {
    graph_->loadTensor(tensor_);
    ItemsPtr items = graph_->getDetectedItems();
    float time_taken = graph_->getTimeTaken();

    for (auto i : *items)
    {
      result->items.push_back(i);

      if (result->items.size() == top_n_)
      {
        break;
      }
    }

    result->time_taken = time_taken;
    device_->monitorThermal();
  }
  catch (MvncMyriadError& e)
  {
    ROS_ERROR_STREAM(e.what());
    std::string debug_info = graph_->getDebugInfo();
    ROS_ERROR_STREAM("myriad debug info: " << debug_info);
  }
  catch (NcsException& e)
  {
    ROS_ERROR_STREAM(e.what());
  }

  return result;
}

}  // namespace movidius_ncs_lib
