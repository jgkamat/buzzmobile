#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "image_utils.hpp"

image_transport::Publisher pub;
void imageCallback(const sensor_msgs::ImageConstPtr&);

int main(int argc, char** argv) {
  ros::init(argc, argv, "projector");
  ros::NodeHandle n;
  image_transport::ImageTransport it(n);

  ros::Subscriber s = n.subscribe("/image_raw", 1, imageCallback);
  pub = it.advertise("/image_projected", 1);

  ros::spin();
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  // Convert ROS to OpenCV
  cv_bridge::CvImagePtr cv_ptr;

  try {
    cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Define pointers
  cv::Mat in = cv_ptr->image;
  cv::Mat out;
  sensor_msgs::Image image_projected;

  // Transform perspective.
  image_utils::transform_perspective(in, out);

  // Publish image_projected.
  cv_ptr->image = out;
  cv_ptr->encoding = msg->encoding;
  cv_ptr->toImageMsg(image_projected);
  pub.publish(image_projected);
}
