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
#include <object_msgs/Object.h>
#include <object_msgs/Objects.h>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>

#define LINESPACING 50
#define DEFAULT_PARALLEL_FLAG 1
#define MOVEWINDOW 1000

int getFPS()
{
  static int fps = 0;
  static boost::posix_time::ptime duration_start =boost::posix_time::microsec_clock::local_time();
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

int parallel_flag;
void syncCb(const sensor_msgs::ImageConstPtr& img, const object_msgs::Objects::ConstPtr& objs)
{
  cv::Mat cvImage = cv_bridge::toCvShare(img, "bgr8")->image;
  int cnt = 0;

  for (auto obj : objs->objects_vector) {
    std::stringstream ss;
    ss << obj.object_name << ": " << obj.probability * 100 << '%';
    cv::putText(cvImage, ss.str(), cvPoint(LINESPACING, LINESPACING * (++cnt)),
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0));
  }

  std::stringstream ss;
  int fps = getFPS();
  ss << "FPS: " << fps;

  if (parallel_flag == 0)
  {
    cv::putText(cvImage, ss.str(), cvPoint(LINESPACING, LINESPACING * (++cnt)),
              cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0));
    cv::imshow("image classification with single device", cvImage);
  }
  else
  {

    cv::putText(cvImage, ss.str(), cvPoint(LINESPACING, LINESPACING * (++cnt)),
              cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0));
    cv::namedWindow("image classification with multiple devices");
    cv::moveWindow("image classification with multiple devices", MOVEWINDOW, 0);
    cv::imshow("image classification with multiple devices", cvImage);
  }

  int key = cv::waitKey(5);
  if (key == 13 || key == 27 || key == 32 || key == 113) {
    ros::shutdown();
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "movidius_ncs_example_stream");
  ros::NodeHandle nh("~");

  parallel_flag = DEFAULT_PARALLEL_FLAG;
  if (!nh.getParam("parallel_flag", parallel_flag))
  {
    ROS_WARN("param parallel_flag not set, use default");
  }
  ROS_INFO_STREAM("use parallel_flag = " << parallel_flag);

  if (parallel_flag == 0)
  {
    message_filters::Subscriber<sensor_msgs::Image> camSub(nh, "/camera/color/image_raw", 1);
    message_filters::Subscriber<object_msgs::Objects> objSub(nh, "/movidius_ncs_nodelet_single/classified_objects", 1);
    message_filters::TimeSynchronizer<sensor_msgs::Image, object_msgs::Objects> sync(camSub, objSub, 60);
    sync.registerCallback(boost::bind(&syncCb, _1, _2));
    ros::spin();
  }
  else
  {
    message_filters::Subscriber<sensor_msgs::Image> camSub(nh, "/camera/color/image_raw", 1);
    message_filters::Subscriber<object_msgs::Objects> objSub(nh, "/movidius_ncs_nodelet_multiple/classified_objects", 1);
    message_filters::TimeSynchronizer<sensor_msgs::Image, object_msgs::Objects> sync(camSub, objSub, 60);
    sync.registerCallback(boost::bind(&syncCb, _1, _2));
    ros::spin();
  }

  return 0;
}
