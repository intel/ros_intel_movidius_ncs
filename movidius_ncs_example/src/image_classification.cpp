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

#include <dirent.h>
#include <vector>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <object_msgs/ClassifyObject.h>

#define LINESPACING 30

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
    std::cerr << "Open Dir error..." << std::endl;
    exit(1);
  }

  while ((ptr = readdir(dir)) != NULL)
  {
    if (strcmp(ptr->d_name, ".") == 0 || strcmp(ptr->d_name, "..") == 0)
      continue;
    else if (ptr->d_type == DT_REG)
      files.push_back(image_dir + ptr->d_name);
  }
  closedir(dir);

  return files;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "movidius_ncs_example");

  if (argc != 2)
  {
    ROS_INFO("Usage: rosrun movidius_ncs_example movidius_ncs_example_image_classification <image_dir>");
    return -1;
  }

  std::vector<std::string> image_paths = getImagePath(*(argv + 1));

  ros::NodeHandle n;
  ros::ServiceClient client;
  client = n.serviceClient<object_msgs::ClassifyObject>("/movidius_ncs_image/classify_object");
  object_msgs::ClassifyObject srv;

  srv.request.image_paths = image_paths;

  boost::posix_time::ptime start = boost::posix_time::microsec_clock::local_time();

  if (!client.call(srv))
  {
    ROS_ERROR("failed to call service ClassifyObject");
    return 1;
  }

  boost::posix_time::ptime end = boost::posix_time::microsec_clock::local_time();
  boost::posix_time::time_duration msdiff = end - start;

  for (unsigned int i = 0; i < srv.response.objects.size(); i++)
  {
    cv_bridge::CvImage cv_image;
    cv_image.image = cv::imread(image_paths[i]);
    cv_image.encoding = "bgr8";
    int cnt = 0;

    ROS_INFO("Classification result for image No.%u:", i + 1);
    for (unsigned int j = 0; j < srv.response.objects[i].objects_vector.size(); j++)
    {
      std::stringstream ss;
      ss << srv.response.objects[i].objects_vector[j].object_name << ": "
         << srv.response.objects[i].objects_vector[j].probability * 100 << "%";

      ROS_INFO("%d: object: %s\nprobability: %lf%%", j, srv.response.objects[i].objects_vector[j].object_name.c_str(),
               srv.response.objects[i].objects_vector[j].probability * 100);

      cv::putText(cv_image.image, ss.str(), cvPoint(LINESPACING, LINESPACING * (++cnt)), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                  cv::Scalar(0, 255, 0), 1);
    }

    cv::imshow("image_classification", cv_image.image);
    cv::waitKey(0);
  }
  ROS_INFO("inference %lu images during %ld ms", srv.response.objects.size(), msdiff.total_milliseconds());

  return 0;
}
