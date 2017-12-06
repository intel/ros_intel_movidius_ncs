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

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>

#include <movidius_ncs_msgs/ObjectInBox.h>
#include <movidius_ncs_msgs/ObjectsInBoxes.h>

#define LINESPACING 20

void syncCb(const sensor_msgs::ImageConstPtr& img,
            const movidius_ncs_msgs::ObjectsInBoxes::ConstPtr& objs_in_boxes)
{
  cv::Mat cvImage = cv_bridge::toCvShare(img, "bgr8")->image;
  int width = img->width;
  int height = img->height;

  for (auto obj : objs_in_boxes->objects_vector)
  {
    std::stringstream ss;
    ss << obj.object.object_name << ": " << obj.object.probability * 100 << '%';

    int x = obj.roi.x_offset;
    int y = obj.roi.y_offset;
    int w = obj.roi.width;
    int h = obj.roi.height;

    int xmin = ((x - w / 2) > 0)? (x - w / 2) : 0;
    int xmax = ((x + w / 2) < width)? (x + w / 2) : width;
    int ymin = ((y - h / 2) > 0)? (y - h / 2) : 0;
    int ymax = ((y + h / 2) < height)? (y + h / 2) : height;

    cv::Point left_top = cv::Point(xmin, ymin);
    cv::Point right_bottom = cv::Point(xmax, ymax);
    cv::rectangle(cvImage, left_top, right_bottom, cv::Scalar(0, 255, 0), 1, cv::LINE_8, 0);
    cv::rectangle(cvImage, cvPoint(xmin, ymin), cvPoint(xmax, ymin + 20), cv::Scalar(0, 255, 0), -1);
    cv::putText(cvImage, ss.str(), cvPoint(xmin + 5, ymin + 20), cv::FONT_HERSHEY_PLAIN,
                1, cv::Scalar(0, 0, 255), 1);
  }

  std::stringstream ss;
  ss << "FPS: " << objs_in_boxes->fps;
  cv::putText(cvImage, ss.str(), cvPoint(LINESPACING, LINESPACING),
              cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0));
  cv::imshow("image_viewer", cvImage);
  cv::waitKey(5);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "movidius_ncs_example_stream");
  ros::NodeHandle nh;
  message_filters::Subscriber<sensor_msgs::Image> camSub(nh,
                                                         "/camera/color/image_raw",
                                                         1);
  message_filters::Subscriber<movidius_ncs_msgs::ObjectsInBoxes> objSub(nh,
                                                                        "/movidius_ncs_nodelet/detected_objects",
                                                                        1);
  message_filters::TimeSynchronizer<sensor_msgs::Image, movidius_ncs_msgs::ObjectsInBoxes> sync(camSub,
                                                                                                objSub,
                                                                                                60);
  sync.registerCallback(boost::bind(&syncCb, _1, _2));
  ros::spin();
  return 0;
}
