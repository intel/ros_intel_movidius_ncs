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

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>

#include <object_msgs/ClassifyObject.h>

#include <vector>
#include <dirent.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <chrono>
#include <thread>

std::vector<std::string> getImagePath(std::string image_dir)
{
  if (image_dir.back() != '/')
  {
    image_dir += "/";
  }

  std::vector<std::string> files;

  DIR* dir;
  struct dirent* ptr;

  if ((dir = opendir(image_dir.c_str())) == NULL)
  {
    perror("Open Dir error...");
    exit(1);
  }

  while ((ptr = readdir(dir)) != NULL)
  {
    if (strcmp(ptr->d_name, ".") == 0 || strcmp(ptr->d_name, "..") == 0)
      continue;
    else if (ptr->d_type == 8)
      files.push_back(image_dir + ptr->d_name);
  }
  closedir(dir);

  return files;
}

void parseResults(object_msgs::ClassifyObject srv)
{
  // parse
  // srv.response;
  for (unsigned int i = 0; i < srv.response.objects.size(); i++)
  {
    ROS_INFO("Classification result for image No. %u:", i);
    for (unsigned int j = 0; j < srv.response.objects[i].objects_vector.size(); j++)
    {
      ROS_INFO("%d: object: %s\nprobability: %lf%%", j, srv.response.objects[i].objects_vector[j].object_name.c_str(),
      srv.response.objects[i].objects_vector[j].probability * 100);
    }
  }

  // display
  // cv::
}

void singleDeviceThread(std::vector<std::string> images_path)
{
  while (1)
  {
    // init
    ros::NodeHandle n;
    ros::ServiceClient client_for_single;
    client_for_single = n.serviceClient<object_msgs::ClassifyObject>("/movidius_ncs_image_single/classify_object");
    object_msgs::ClassifyObject srv_for_single;

    // randomly pick one picture
    srv_for_single.request.images_path = images_path;

    // call service
    if (!client_for_single.call(srv_for_single))
    {
      ROS_ERROR("failed to call service ClassifyObject");
      exit(1);
    }

    // parse result
    parseResults(srv_for_single);
  }
}

void multipleDeviceThread(std::vector<std::string> images_path)
{
  while (1)
  {
    // init
    ros::NodeHandle n;
    ros::ServiceClient client_for_multiple;
    client_for_multiple = n.serviceClient<object_msgs::ClassifyObject>("/movidius_ncs_image_multiple/classify_object");
    object_msgs::ClassifyObject srv_for_multiple;

    // randomly pick one picture
    srv_for_multiple.request.images_path = images_path;

    // call service
    if (!client_for_multiple.call(srv_for_multiple))
    {
      ROS_ERROR("failed to call service ClassifyObject");
      exit(1);
    }

    // parse result
    parseResults(srv_for_multiple);
  }
}

void join(std::thread& t)
{
  t.join();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "movidius_ncs_example");

  if (argc != 2)
  {
    ROS_INFO("Usage: rosrun movidius_ncs_example movidius_ncs_example_device_parallel_demo <image_path>");
    return -1;
  }

  std::vector<std::string> images_path = getImagePath(argv[1]);
  //ros::NodeHandle n;

  // start two threads
  std::vector<std::thread> threads;
  threads.push_back(std::thread(singleDeviceThread, images_path));
  threads.push_back(std::thread(multipleDeviceThread, images_path));
  std::for_each(threads.begin(), threads.end(), join);

  return 0;
}
