#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "ros/ros.h"
#include "sensor_msgs/image_encodings.h"
#include <cv_bridge/cv_bridge.h>

void imageCallback(const sensor_msgs::ImageConstPtr&);

int main(int argc, char** argv) {
  ros::init(argc, argv, "edgedetect");
  ros::NodeHandle n;
  ros::Subscriber s = n.subscribe("image", 1000, imageCallback);
  ros::spin();
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr;
  if (msg) {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat I = cv_ptr->image;
    cv::Mat out;
    cv::Canny(I, out, 50, 200);
    cv::imshow("Edges", out);
  } else {
    ROS_DEBUG("No message found");
  }
}
