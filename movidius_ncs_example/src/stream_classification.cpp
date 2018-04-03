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

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <object_msgs/Object.h>
#include <object_msgs/Objects.h>

#define LINESPACING 50

int getFPS()
{
  static int fps = 0;
  static boost::posix_time::ptime duration_start = boost::posix_time::microsec_clock::local_time();
  static int frame_cnt = 0;

  frame_cnt++;

  boost::posix_time::ptime current = boost::posix_time::microsec_clock::local_time();
  boost::posix_time::time_duration msdiff = current - duration_start;

  if (msdiff.total_milliseconds() > 1000)
  {
    fps = frame_cnt;
    frame_cnt = 0;
    duration_start = current;
  }

  return fps;
}

void syncCb(const sensor_msgs::ImageConstPtr& img, const object_msgs::Objects::ConstPtr& objs)
{
  cv::Mat cvImage = cv_bridge::toCvShare(img, "bgr8")->image;
  int cnt = 0;

  for (auto obj : objs->objects_vector)
  {
    std::stringstream ss;
    ss << obj.object_name << ": " << obj.probability * 100 << '%';
    cv::putText(cvImage, ss.str(), cvPoint(LINESPACING, LINESPACING * (++cnt)), cv::FONT_HERSHEY_SIMPLEX, 1,
                cv::Scalar(0, 255, 0));
  }

  std::stringstream ss;
  int fps = getFPS();
  ss << "FPS: " << fps;

  cv::putText(cvImage, ss.str(), cvPoint(LINESPACING, LINESPACING * (++cnt)), cv::FONT_HERSHEY_SIMPLEX, 1,
              cv::Scalar(0, 255, 0));
  cv::imshow("image_viewer", cvImage);

  int key = cv::waitKey(5);
  if (key == 13 || key == 27 || key == 32 || key == 113)
  {
    ros::shutdown();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "movidius_ncs_example_stream");
  ros::NodeHandle nh;

  message_filters::Subscriber<sensor_msgs::Image> camSub(nh, "/camera/color/image_raw", 1);
  message_filters::Subscriber<object_msgs::Objects> objSub(nh, "/movidius_ncs_nodelet/classified_objects", 1);

  message_filters::TimeSynchronizer<sensor_msgs::Image, object_msgs::Objects> sync(camSub, objSub, 60);
  sync.registerCallback(boost::bind(&syncCb, _1, _2));

  ros::spin();
  return 0;
}
