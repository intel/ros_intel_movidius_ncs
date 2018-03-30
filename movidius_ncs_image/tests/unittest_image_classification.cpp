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
  client = n.serviceClient<object_msgs::ClassifyObject>("/movidius_ncs_image_multiple/classify_object");
  object_msgs::ClassifyObject srv;

  srv.request.image_paths.push_back(ros::package::getPath("movidius_ncs_lib") + "/../data/images/bicycle.jpg");

  client.waitForExistence(ros::Duration(80));
  EXPECT_TRUE(client.call(srv));
  EXPECT_TRUE(srv.response.objects[0].objects_vector.size());
  EXPECT_TRUE(srv.response.objects[0].objects_vector[0].object_name.find("cycle") != std::string::npos);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "movidius_ncs_image_tests");
  return RUN_ALL_TESTS();
}
