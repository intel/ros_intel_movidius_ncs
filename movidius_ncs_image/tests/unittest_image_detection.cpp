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
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <object_msgs/DetectObject.h>

TEST(UnitTestDetection, testImage)
{
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<object_msgs::DetectObject>("/movidius_ncs_image_multiple/detect_object");
  object_msgs::DetectObject srv;

  std::vector<std::string> image_format = {".jpg", ".jpeg", ".png", ".bmp"};
  for (std::string suffix : image_format)
  {
    srv.request.image_paths.push_back(ros::package::getPath("movidius_ncs_lib") + "/../data/images/bicycle"+suffix);

    client.waitForExistence(ros::Duration(60));
    EXPECT_TRUE(client.call(srv));
    EXPECT_TRUE(srv.response.objects[0].objects_vector.size());
    EXPECT_EQ(srv.response.objects[0].objects_vector[0].object.object_name, "bicycle");
    EXPECT_TRUE(srv.response.objects[0].objects_vector[0].roi.x_offset > 130 &&
      srv.response.objects[0].objects_vector[0].roi.x_offset < 150 &&
      srv.response.objects[0].objects_vector[0].roi.y_offset > 90 &&
      srv.response.objects[0].objects_vector[0].roi.y_offset < 110 &&
      srv.response.objects[0].objects_vector[0].roi.width > 410 &&
      srv.response.objects[0].objects_vector[0].roi.width < 470 &&
      srv.response.objects[0].objects_vector[0].roi.height > 340 &&
      srv.response.objects[0].objects_vector[0].roi.height < 360);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "movidius_ncs_image_tests");
  return RUN_ALL_TESTS();
}
