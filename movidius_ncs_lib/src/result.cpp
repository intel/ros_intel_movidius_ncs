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
#include <vector>
#include <boost/algorithm/string.hpp>
#include "movidius_ncs_lib/result.h"

namespace movidius_ncs_lib
{
Result::Result(const std::string& cnn_type)
    : classification_result(nullptr),
      detection_result(nullptr)
{
  if (!cnn_type.compare("tiny_yolo"))
  {
    detection_result = std::make_shared<DetectionResult>();
  }
  else
  {
    classification_result = std::make_shared<ClassificationResult>();
  }
}

ClassificationResultPtr Result::getClassificationResult()
{
  return classification_result;
}

DetectionResultPtr Result::getDetectionResult()
{
  return detection_result;
}

void Result::parseYoloResult(const std::vector<float>& result, const std::vector<std::string> categories,
                             int img_width, int img_height)
{
  constexpr int grid_width = 7;
  constexpr int grid_height = 7;
  constexpr int bbox_num = 2;
  constexpr float prob_threshold = 0.2;
  constexpr float iou_threshold = 0.5;
  constexpr int bbox_conf_num = grid_width * grid_height * bbox_num;
  int class_num = categories.size();
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

        if (obj_in_box.item.probability > prob_threshold)
        {
          obj_in_box.item.category = categories[std::distance(std::begin(probs), max_iter)];
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
      if (iou(*iter1, *iter2) > iou_threshold)
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

    if (!detection_result->items_in_boxes.empty())
    {
      detection_result->items_in_boxes.clear();
    }

  for (auto item : *objs_in_bboxes)
  {
    detection_result->items_in_boxes.push_back(item);
  }
}

void Result::setClassificationResult(Item item)
{
  classification_result->items.push_back(item);
}

void Result::setClassificationResult(float time)
{
  classification_result->time_taken = time;
}

void Result::setDetectionResult(ItemInBBox item)
{
  detection_result->items_in_boxes.push_back(item);
}

void Result::setDetectionResult(float time)
{
  detection_result->time_taken = time;
}

float Result::iou(ItemInBBox box1, ItemInBBox box2)
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

}   // namespace movidius_ncs_lib
