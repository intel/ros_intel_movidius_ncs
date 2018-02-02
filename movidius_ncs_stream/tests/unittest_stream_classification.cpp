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

#include <string>

#include <cv_bridge/cv_bridge.h>
#include <gtest/gtest.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>

#include <object_msgs/Object.h>
#include <object_msgs/Objects.h>

static bool test_pass = false;

void syncCb(const sensor_msgs::ImageConstPtr& img,
            const object_msgs::Objects::ConstPtr& objs)
{
  test_pass = true;
  EXPECT_TRUE(true);
  ros::shutdown();
}

TEST(UnitTestStreamClassification, testClassification)
{
  ros::NodeHandle nh("~");
  std::string camera, camera_topic;
  nh.getParam("camera", camera);
  if (camera == "usb")
  {
    camera_topic = "/usb_cam/image_raw";
  }
  else if (camera == "realsense")
  {
    camera_topic = "/camera/color/image_raw";
  }
  message_filters::Subscriber<sensor_msgs::Image> camSub(nh,
                                                         camera_topic,
                                                         1);
  message_filters::Subscriber<object_msgs::Objects> objSub(nh,
                                                           "/movidius_ncs_nodelet/classified_objects",
                                                           1);
  message_filters::TimeSynchronizer<sensor_msgs::Image, object_msgs::Objects> sync(camSub, objSub, 60);
  sync.registerCallback(boost::bind(&syncCb, _1, _2));
  ros::spin();
  EXPECT_TRUE(test_pass);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "movidius_ncs_stream_tests");
  return RUN_ALL_TESTS();
}
