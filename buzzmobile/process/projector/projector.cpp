#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "image_transform.hpp"

image_transport::Publisher pub;
void imageCallback(const sensor_msgs::ImageConstPtr&);

int main(int argc, char** argv) {
  ros::init(argc, argv, "projector");
  ros::NodeHandle n;
  image_transport::ImageTransport it(n);

  ros::Subscriber s = n.subscribe("/usb_cam/image_raw", 1, imageCallback);
  pub = it.advertise("/image_projected", 1);

  ros::param::get("image_width", image_transform::persp_transform_width);
  ros::param::get("image_height", image_transform::persp_transform_height);

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

  // TODO: once we sample images form the car's camera, it may be a good idea to
  // crop them before projecting.

  // Transform perspective.
  image_transform::transform_perspective(in, out);

  // Publish image_projected.
  cv_ptr->image = out;
  cv_ptr->encoding = msg->encoding;
  cv_ptr->toImageMsg(image_projected);
  pub.publish(image_projected);
}
