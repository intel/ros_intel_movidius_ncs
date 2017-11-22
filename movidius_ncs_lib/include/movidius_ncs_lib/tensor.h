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

#ifndef MOVIDIUS_NCS_LIB_TENSOR_H
#define MOVIDIUS_NCS_LIB_TENSOR_H

#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <opencv2/opencv.hpp>
#include "movidius_ncs_lib/mvnc_cpp.h"

namespace movidius_ncs_lib
{
class Tensor
{
public:
  using Ptr = std::shared_ptr<Tensor>;
  using ConstPtr = std::shared_ptr<Tensor const>;

  Tensor(const cv::Mat& image,
         const std::vector<float>& mean,
         const std::pair<int, int>& size,
         const std::string& cnn_type);

  inline const uint16_t* raw() const
  {
    return &tensor_[0];
  }
  inline const size_t size() const
  {
    return 3 * net_height_ * net_width_ * sizeof(uint16_t);
  }
  inline int getImageWidth()
  {
    return image_width_;
  }
  inline int getImageHeight()
  {
    return image_height_;
  }
#if !defined(__i386__) && !defined(__x86_64__)
  static void fp32tofp16(uint16_t* __restrict out, float in);
  static void fp16tofp32(float* __restrict out, uint16_t in);
#endif

private:
  std::vector<uint16_t> tensor_;
  const int net_width_;
  const int net_height_;
  const int image_width_;
  const int image_height_;
};
}   // namespace movidius_ncs_lib

#endif  // MOVIDIUS_NCS_LIB_TENSOR_H
