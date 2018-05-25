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

#include <utility>
#include <vector>
#include <string>
#ifdef SUPPORT_F16C
#include <x86intrin.h>
#endif
#include <opencv2/core/mat.hpp>
#include "movidius_ncs_lib/tensor.h"

namespace movidius_ncs_lib
{
Tensor::Tensor(const std::pair<int, int>& net_dim,
               const std::vector<float>& mean,
               const float& scale)
  : tensor_(0),
    net_width_(net_dim.first),
    net_height_(net_dim.second),
    image_width_(0),
    image_height_(0),
    mean_(mean),
    scale_(scale)
{
}

void Tensor::loadImageData(const cv::Mat& image)
{
  image_width_ = image.cols;
  image_height_ = image.rows;

  cv::Mat resized;
  cv::resize(image, resized, cv::Size(net_width_, net_height_), 0, 0);

  cv::Mat colored;
  cv::cvtColor(resized, colored, CV_BGR2RGB);

  cv::Mat converted;
  colored.convertTo(converted, CV_32FC3);

  using TensorIt = cv::MatConstIterator_<cv::Vec3f>;

  for (TensorIt it = converted.begin<cv::Vec3f>(); it != converted.end<cv::Vec3f>(); ++it)
  {
    float r32 = ((*it)[0] - mean_[0]) * scale_;
    float g32 = ((*it)[1] - mean_[1]) * scale_;
    float b32 = ((*it)[2] - mean_[2]) * scale_;
    uint16_t r16;
    uint16_t g16;
    uint16_t b16;
#ifdef SUPPORT_F16C
    r16 = _cvtss_sh(r32, 0);
    g16 = _cvtss_sh(g32, 0);
    b16 = _cvtss_sh(b32, 0);
#else
    Tensor::fp32tofp16(&r16, r32);
    Tensor::fp32tofp16(&g16, g32);
    Tensor::fp32tofp16(&b16, b32);
#endif
    tensor_.push_back(r16);
    tensor_.push_back(g16);
    tensor_.push_back(b16);
  }
}

void Tensor::clearTensor()
{
  if (!tensor_.empty())
  {
    tensor_.clear();
  }
}

#ifndef SUPPORT_F16C
void Tensor::fp16tofp32(float* __restrict out, uint16_t in)
{
  uint32_t t1;
  uint32_t t2;
  uint32_t t3;
  t1 = in & 0x7fff;
  t2 = in & 0x8000;
  t3 = in & 0x7c00;
  t1 <<= 13;
  t2 <<= 16;
  t1 += 0x38000000;
  t1 = (t3 == 0 ? 0 : t1);
  t1 |= t2;
  *(reinterpret_cast<uint32_t*>(out)) = t1;
}

void Tensor::fp32tofp16(uint16_t* __restrict out, float in)
{
  uint32_t inu = *(reinterpret_cast<uint32_t*>(&in));
  uint32_t t1;
  uint32_t t2;
  uint32_t t3;
  t1 = inu & 0x7fffffff;
  t2 = inu & 0x80000000;
  t3 = inu & 0x7f800000;
  t1 >>= 13;
  t2 >>= 16;
  t1 -= 0x1c000;
  t1 = (t3 < 0x38800000) ? 0 : t1;
  t1 = (t3 > 0x47000000) ? 0x7bff : t1;
  t1 = (t3 == 0 ? 0 : t1);
  t1 |= t2;
  *(reinterpret_cast<uint16_t*>(out)) = t1;
}
#endif

}   // namespace movidius_ncs_lib
