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

#ifndef MOVIDIUS_NCS_LIB_RESULT_H
#define MOVIDIUS_NCS_LIB_RESULT_H

#include <string>
#include <memory>
#include <vector>

namespace movidius_ncs_lib
{
struct Item
{
  std::string category;
  float probability;
};

struct BBox
{
  int x;
  int y;
  int width;
  int height;
};

struct ItemInBBox
{
  Item item;
  BBox bbox;
};

using Items = std::vector<Item>;
using ItemsPtr = std::shared_ptr<Items>;
using ItemInBBoxArray = std::vector<ItemInBBox>;
using ItemInBBoxArrayPtr = std::shared_ptr<ItemInBBoxArray>;

struct ClassificationResult
{
  Items items;
  float time_taken;
};

struct DetectionResult
{
  ItemInBBoxArray items_in_boxes;
  float time_taken;
};

using ClassificationResultPtr = std::shared_ptr<ClassificationResult>;
using DetectionResultPtr = std::shared_ptr<DetectionResult>;


class Result
{
public:
  using Ptr = std::shared_ptr<Result>;
  using ConstPtr = std::shared_ptr<Result const>;

  explicit Result(const std::string& cnn_type);
  ClassificationResultPtr getClassificationResult();
  DetectionResultPtr getDetectionResult();
  void setClassificationResult(Item item);
  void setClassificationResult(float time);
  void setDetectionResult(ItemInBBox item);
  void setDetectionResult(float time);
  void parseYoloResult(const std::vector<float>& result, const std::vector<std::string> categories,
                       int img_width, int img_height);
  void parseSSDResult(const std::vector<float>& result, const std::vector<std::string> categories,
                       int img_width, int img_height);

private:
  void NMS(ItemInBBoxArrayPtr objs_in_bboxes);
  float IOU(ItemInBBox box1, ItemInBBox box2);

  ClassificationResultPtr classification_result;
  DetectionResultPtr detection_result;
};
}  // namespace movidius_ncs_lib

#endif  // MOVIDIUS_NCS_LIB_RESULT_H
