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

#include <gtest/gtest.h>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <object_msgs/ClassifyObject.h>

TEST(UnitTestClassification, testImage)
{
  ros::NodeHandle n;
  ros::ServiceClient client;
  client = n.serviceClient<object_msgs::ClassifyObject>("/movidius_ncs_image/classify_object");
  object_msgs::ClassifyObject srv;

  cv_bridge::CvImage cv_image;
  sensor_msgs::Image ros_image;
  cv_image.image = cv::imread(ros::package::getPath("movidius_ncs_lib") + "/../data/images/bicycle.jpg");
  cv_image.encoding = "bgr8";
  cv_image.toImageMsg(ros_image);
  srv.request.image = ros_image;

  client.waitForExistence(ros::Duration(60));
  EXPECT_TRUE(client.call(srv));
  EXPECT_TRUE(srv.response.objects.objects_vector.size());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "movidius_ncs_image_tests");
  return RUN_ALL_TESTS();
}
