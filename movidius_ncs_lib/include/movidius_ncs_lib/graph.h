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

#ifndef MOVIDIUS_NCS_LIB_GRAPH_H
#define MOVIDIUS_NCS_LIB_GRAPH_H

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "movidius_ncs_lib/mvnc_cpp.h"
#include "movidius_ncs_lib/result.h"
#include "movidius_ncs_lib/tensor.h"
#include "movidius_ncs_lib/device.h"

namespace movidius_ncs_lib
{
class Graph
{
public:
  using Ptr = std::shared_ptr<Graph>;
  using ConstPtr = std::shared_ptr<Graph const>;

  Graph(const Device::Ptr& device,
        std::string cnn_type,
        const std::string& graph_file,
        int network_dimension,
        const std::vector<float>& mean,
        const std::vector<std::string> categories_);
  ~Graph();

  void allocate();
  void deallocate();
  float getTimeTaken();
  ItemsPtr classifyObjects();
  ItemInBBoxArrayPtr detectObjects(int img_width, int img_height);
  void loadTensor(const Tensor::ConstPtr& tensor);
  std::string getDebugInfo();
  void* getHandle();

  int getNetworkDim() const
  {
    return network_dimension_;
  }
  std::vector<float> getMean() const
  {
    return mean_;
  }

private:
  std::shared_ptr<Device> device_;
  std::string cnn_type_;
  std::string graph_buf_;
  const int network_dimension_;
  const std::vector<float> mean_;
  const std::vector<std::string> categories_;
  void* handle_;
  void* user_param_;

  float iou(ItemInBBox box1, ItemInBBox box2);
  ItemInBBoxArrayPtr parseYoloResult(const std::vector<float>& result, int img_width, int img_height);
};
}   // namespace movidius_ncs_lib

#endif  // MOVIDIUS_NCS_LIB_GRAPH_H
