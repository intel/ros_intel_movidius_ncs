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

#include <object_msgs/DetectObject.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "movidius_ncs_example");

  if (argc != 2)
  {
    ROS_INFO("Usage: rosrun movidius_ncs_example movidius_ncs_example_image_detection <image_path>");
    return -1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client;
  client = n.serviceClient<object_msgs::DetectObject>("/movidius_ncs_image/detect_object");
  object_msgs::DetectObject srv;

  cv_bridge::CvImage cv_image;
  sensor_msgs::Image ros_image;
  cv_image.image = cv::imread(argv[1]);
  cv_image.encoding = "bgr8";
  cv_image.toImageMsg(ros_image);
  int width = cv_image.image.cols;
  int height = cv_image.image.rows;

  srv.request.image = ros_image;

  if (!client.call(srv))
  {
    ROS_ERROR("failed to call service DetectObject");
    return 1;
  }

  for (unsigned int i = 0; i < srv.response.objects.objects_vector.size(); i++)
  {
    std::stringstream ss;
    ss << srv.response.objects.objects_vector[i].object.object_name << ": "
       << srv.response.objects.objects_vector[i].object.probability * 100 << "%";

    ROS_INFO("%d: object: %s", i, srv.response.objects.objects_vector[i].object.object_name.c_str());
    ROS_INFO("prob: %f", srv.response.objects.objects_vector[i].object.probability);
    ROS_INFO("location: (%d, %d, %d, %d)",
             srv.response.objects.objects_vector[i].roi.x_offset,
             srv.response.objects.objects_vector[i].roi.y_offset,
             srv.response.objects.objects_vector[i].roi.width,
             srv.response.objects.objects_vector[i].roi.height);

    int xmin = srv.response.objects.objects_vector[i].roi.x_offset;
    int ymin = srv.response.objects.objects_vector[i].roi.y_offset;
    int w = srv.response.objects.objects_vector[i].roi.width;
    int h = srv.response.objects.objects_vector[i].roi.height;

    int xmax = ((xmin + w) < width)? (xmin + w) : width;
    int ymax = ((ymin + h) < height)? (ymin + h) : height;

    cv::Point left_top = cv::Point(xmin, ymin);
    cv::Point right_bottom = cv::Point(xmax, ymax);
    cv::rectangle(cv_image.image, left_top, right_bottom, cv::Scalar(0, 255, 0), 1, cv::LINE_8, 0);
    cv::rectangle(cv_image.image, cvPoint(xmin, ymin), cvPoint(xmax, ymin + 20), cv::Scalar(0, 255, 0), -1);
    cv::putText(cv_image.image, ss.str(), cvPoint(xmin + 5, ymin + 20), cv::FONT_HERSHEY_PLAIN,
                1, cv::Scalar(0, 0, 255), 1);
  }
    ROS_INFO("inference time: %fms", srv.response.objects.inference_time_ms);
    cv::imshow("image_detection", cv_image.image);
    cv::waitKey(0);
    return 0;
}
