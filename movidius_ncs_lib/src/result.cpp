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
  if (!cnn_type.compare("tinyyolo_v1") || !cnn_type.compare("mobilenetssd"))
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
        ItemInBBox obj_in_bbox;

        std::vector<float> probs(result.begin() + index / 2 * class_num,
                                 result.begin() + index / 2 * class_num + class_num);
        float scale = result[prob_num + index];
        std::vector<float>::iterator max_iter = std::max_element(std::begin(probs), std::end(probs));
        obj_in_bbox.item.probability = *max_iter * scale;

        if (obj_in_bbox.item.probability > prob_threshold)
        {
          obj_in_bbox.item.category = categories[std::distance(std::begin(probs), max_iter)];
          int x_center = ((result[prob_num + bbox_conf_num + index * 4] + j) / 7.0) * img_width;
          int y_center = ((result[prob_num + bbox_conf_num + index * 4 + 1] + i) / 7.0) * img_height;
          obj_in_bbox.bbox.width = (result[prob_num + bbox_conf_num + index * 4 + 2])
                                   * (result[prob_num + bbox_conf_num + index * 4 + 2]) * img_width;
          obj_in_bbox.bbox.height = (result[prob_num + bbox_conf_num + index * 4 + 3])
                                    * (result[prob_num + bbox_conf_num + index * 4 + 3]) * img_height;
          obj_in_bbox.bbox.x = (x_center - 0.5 * obj_in_bbox.bbox.width) < 0?
                                0 : (x_center - 0.5 * obj_in_bbox.bbox.width);
          obj_in_bbox.bbox.y = (y_center - 0.5 * obj_in_bbox.bbox.height) < 0?
                                0 : (y_center - 0.5 * obj_in_bbox.bbox.height);
          objs_in_bboxes->push_back(obj_in_bbox);
        }
      }
    }
  }

  NMS(objs_in_bboxes);

  if (!detection_result->items_in_boxes.empty())
  {
    detection_result->items_in_boxes.clear();
  }

  for (auto item : *objs_in_bboxes)
  {
    detection_result->items_in_boxes.push_back(item);
  }
}

void Result::parseSSDResult(const std::vector<float>& result, const std::vector<std::string> categories,
                             int img_width, int img_height)
{
  constexpr int num_in_group = 7;
  int num_detection = result.at(0);
  ItemInBBoxArrayPtr objs_in_bboxes = std::make_shared<ItemInBBoxArray>();

  for (int i = 0; i < num_detection; i++)
  {
    int category_id = result[(i + 1) * num_in_group + 1];
    float probability = result[(i + 1) * num_in_group + 2];
    int xmin = result[(i + 1) * num_in_group + 3] * img_width;
    int ymin = result[(i + 1) * num_in_group + 4] * img_height;
    int xmax = result[(i + 1) * num_in_group + 5] * img_width;
    int ymax = result[(i + 1) * num_in_group + 6] * img_height;

    ItemInBBox obj_in_bbox;
    if (std::isnan(category_id) || std::isnan(probability) || std::isnan(xmin)
        || xmin < 0 || xmin > img_width || std::isnan(ymin) || ymin < 0 || ymin > img_height
        || std::isnan(xmax) || xmax < 0 || xmax > img_width || std::isnan(ymax)
        || ymax < 0 || ymax > img_height)
    {
      continue;
    }
    else
    {
      obj_in_bbox.item.category = categories.at(category_id);
      obj_in_bbox.item.probability = probability;
      obj_in_bbox.bbox.width = xmax - xmin;
      obj_in_bbox.bbox.height = ymax - ymin;
      obj_in_bbox.bbox.x = xmin;
      obj_in_bbox.bbox.y = ymin;
      objs_in_bboxes->push_back(obj_in_bbox);
    }
  }

  NMS(objs_in_bboxes);

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

void Result::NMS(ItemInBBoxArrayPtr objs_in_bboxes)
{
  constexpr float iou_threshold = 0.5;

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
      if (IOU(*iter1, *iter2) > iou_threshold)
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
}

float Result::IOU(ItemInBBox box1, ItemInBBox box2)
{
  int xmax = (box1.bbox.x + box1.bbox.width < box2.bbox.x + box2.bbox.width)?
             box1.bbox.x + box1.bbox.width : box2.bbox.x + box2.bbox.width;
  int xmin = (box1.bbox.x > box2.bbox.x)? box1.bbox.x : box2.bbox.x;
  int ymax = (box1.bbox.y + box1.bbox.height < box2.bbox.y + box2.bbox.height)?
             box1.bbox.y + box1.bbox.height : box2.bbox.y + box2.bbox.height;
  int ymin = (box1.bbox.y > box2.bbox.y)? box1.bbox.y : box2.bbox.y;
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
