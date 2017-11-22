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
#if defined(__i386__) || defined(__x86_64__)
#include <x86intrin.h>
#endif
#include <ros/console.h>
#include "movidius_ncs_lib/exception.h"
#include "movidius_ncs_lib/exception_util.h"
#include "movidius_ncs_lib/graph.h"
#include "movidius_ncs_lib/tensor.h"

namespace movidius_ncs_lib
{

Graph::Graph(const std::shared_ptr<Device>& device,
             std::string cnn_type,
             const std::string& graph_buf,
             int network_dimension,
             const std::vector<float>& mean,
             const std::vector<std::string> categories)
  : device_(device)
  , cnn_type_(cnn_type)
  , graph_buf_(graph_buf)
  , network_dimension_(network_dimension)
  , mean_(mean)
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

ItemsPtr Graph::classifyObjects()
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
#if defined(__i386__) || defined(__x86_64__)
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
  return items;
}

ItemInBBoxArrayPtr Graph::detectObjects(int img_width, int img_height)
{
  assert(handle_ != nullptr);
  uint16_t* result;
  unsigned int length;
  int ret = mvncGetResult(handle_,
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
#if defined(__i386__) || defined(__x86_64__)
    fp32 = _cvtsh_ss(fp16);
#else
    Tensor::fp16tofp32(&fp32, fp16);
#endif
    result32_vector.push_back(fp32);
  }

  if (!cnn_type_.compare("tiny_yolo"))
  {
    return parseYoloResult(result32_vector, img_width, img_height);
  }
}

ItemInBBoxArrayPtr Graph::parseYoloResult(const std::vector<float>& result, int img_width, int img_height)
{
  constexpr int grid_width = 7;
  constexpr int grid_height = 7;
  constexpr int bbox_num = 2;
  constexpr float threshold = 0.2;
  constexpr int bbox_conf_num = grid_width * grid_height * bbox_num;
  int class_num = categories_.size();
  int prob_num = grid_width * grid_height * class_num;
  ItemInBBoxArrayPtr objs_in_bboxes = std::make_shared<ItemInBBoxArray>();

  for (int i = 0; i < grid_height; i++)
  {
    for (int j = 0; j < grid_width; j++)
    {
      for (int k = 0; k < bbox_num; k++)
      {
        int index = i * grid_width * bbox_num + j * bbox_num + k;
        ItemInBBox obj_in_box;

        std::vector<float> probs(result.begin() + index / 2 * class_num,
                                 result.begin() + index / 2 * class_num + class_num);
        float scale = result[prob_num + index];
        std::vector<float>::iterator max_iter = std::max_element(std::begin(probs), std::end(probs));
        obj_in_box.item.probability = *max_iter * scale;

        if (obj_in_box.item.probability > threshold)
        {
          obj_in_box.item.category = categories_[std::distance(std::begin(probs), max_iter)];
          obj_in_box.bbox.x = ((result[prob_num + bbox_conf_num + index * 4] + j) / 7.0) * img_width;
          obj_in_box.bbox.y = ((result[prob_num + bbox_conf_num + index * 4 + 1] + i) / 7.0) * img_height;
          obj_in_box.bbox.width = (result[prob_num + bbox_conf_num + index * 4 + 2])
                                   * (result[prob_num + bbox_conf_num + index * 4 + 2]) * img_width;
          obj_in_box.bbox.height = (result[prob_num + bbox_conf_num + index * 4 + 3])
                                    * (result[prob_num + bbox_conf_num + index * 4 + 3]) * img_height;
          objs_in_bboxes->push_back(obj_in_box);
        }
      }
    }
  }

  auto cmp = [](const ItemInBBox &a, const ItemInBBox &b)
  {
    return a.item.probability > b.item.probability;
  };
  std::sort(objs_in_bboxes->begin(), objs_in_bboxes->end(), cmp);

  for (auto iter1 = objs_in_bboxes->begin(); iter1 != objs_in_bboxes->end(); iter1++)
  {
    if (iter1->item.probability == 0)
    {
      continue;
    }
    for (auto iter2 = iter1 + 1; iter2 != objs_in_bboxes->end(); iter2++)
    {
      if (iou(*iter1, *iter2) > 0.5)
      {
        iter2->item.probability = 0;
      }
    }
  }

  for (auto iter = objs_in_bboxes->begin(); iter != objs_in_bboxes->end(); )
  {
    if (iter->item.probability == 0)
    {
      iter = objs_in_bboxes->erase(iter);
    }
    else
    {
      iter++;
    }
  }

  return objs_in_bboxes;
}

float Graph::iou(ItemInBBox box1, ItemInBBox box2)
{
  int xmax = (box1.bbox.x + 0.5 * box1.bbox.width < box2.bbox.x + 0.5 * box2.bbox.width)?
             box1.bbox.x + 0.5 * box1.bbox.width : box2.bbox.x + 0.5 * box2.bbox.width;
  int xmin = (box1.bbox.x - 0.5 * box1.bbox.width > box2.bbox.x - 0.5 * box2.bbox.width)?
             box1.bbox.x - 0.5 * box1.bbox.width : box2.bbox.x - 0.5 * box2.bbox.width;
  int ymax = (box1.bbox.y + 0.5 * box1.bbox.height < box2.bbox.y + 0.5 * box2.bbox.height)?
             box1.bbox.y + 0.5 * box1.bbox.height : box2.bbox.y + 0.5 * box2.bbox.height;
  int ymin = (box1.bbox.y - 0.5 * box1.bbox.height > box2.bbox.y - 0.5 * box2.bbox.height)?
             box1.bbox.y - 0.5 * box1.bbox.height : box2.bbox.y - 0.5 * box2.bbox.height;
  int inter_w = xmax - xmin;
  int inter_h = ymax - ymin;
  int inter_area = 0;
  if (inter_w > 0 && inter_h > 0)
  {
    inter_area = inter_w * inter_h;
  }
  return inter_area * 1.0 / (box1.bbox.width * box1.bbox.height + box2.bbox.width * box2.bbox.height - inter_area);
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
