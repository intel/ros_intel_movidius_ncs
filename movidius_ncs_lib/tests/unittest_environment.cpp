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

#include <boost/filesystem/operations.hpp>
#include <gtest/gtest.h>

TEST(UnitTestEnvironment, testSDK)
{
  EXPECT_TRUE(boost::filesystem::exists("/opt/movidius/version.txt"));
}

TEST(UnitTestEnvironment, testAppZoo)
{
  EXPECT_TRUE(boost::filesystem::exists("/opt/movidius/ncappzoo"));
  std::vector<std::string> caffe_dirs = { "AlexNet", "GoogLeNet", "SqueezeNet", "SSD_MobileNet", "TinyYolo" };
  std::vector<std::string> tf_dirs = { "inception_v1", "inception_v2", "inception_v3", "inception_v4", "mobilenets" };
  for (std::string caffe : caffe_dirs)
  {
    EXPECT_TRUE(boost::filesystem::exists("/opt/movidius/ncappzoo/caffe/" + caffe + "/graph"));
  }

  for (std::string tf : tf_dirs)
  {
    EXPECT_TRUE(boost::filesystem::exists("/opt/movidius/ncappzoo/tensorflow/" + tf + "/graph"));
  }
}

TEST(UnitTestEnvironment, testCategories)
{
  EXPECT_TRUE(boost::filesystem::exists("/opt/movidius/ncappzoo/data/ilsvrc12/imagenet1000.txt"));
  EXPECT_TRUE(boost::filesystem::exists("/opt/movidius/ncappzoo/data/ilsvrc12/imagenet1001.txt"));
  EXPECT_TRUE(boost::filesystem::exists("/opt/movidius/ncappzoo/data/ilsvrc12/voc20.txt"));
  EXPECT_TRUE(boost::filesystem::exists("/opt/movidius/ncappzoo/data/ilsvrc12/voc21.txt"));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
