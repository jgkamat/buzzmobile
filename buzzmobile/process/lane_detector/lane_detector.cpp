#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp> 
#include <opencv2/imgproc/imgproc.hpp> 

image_transport::Publisher pub;
void imageCallback(const sensor_msgs::ImageConstPtr&);

int main(int argc, char** argv) {
  ros::init(argc, argv, "lane_detector");
  ros::NodeHandle n;
  image_transport::ImageTransport it(n);

  ros::Subscriber s = n.subscribe("/image_projected", 1, imageCallback);
  pub = it.advertise("image_lanes", 1);

  ros::spin();
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  // Convert ROS ro OpenCV image
  cv_bridge::CvImagePtr cv_ptr;

  try {
    cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception %s", e.what());
  }

  // Define pointers
  cv::Mat gray;
  cv::Mat edgy;
  cv::Mat out;
  sensor_msgs::Image image_lanes;

  // Convert to grayscale and Gaussian blur.
  cv::cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);
  cv::GaussianBlur(gray, gray, cv::Size(3, 3), 0, 0);
  // Detect edges.
  cv::Canny(gray, edgy, 50, 125, 3);
  // Convert back to BGR8
  cv::cvtColor(edgy, out, CV_GRAY2BGR);

  // Back to ROS image
  cv_ptr->image = out;
  cv_ptr->encoding = msg->encoding;
  cv_ptr->toImageMsg(image_lanes);
  pub.publish(image_lanes);
}
