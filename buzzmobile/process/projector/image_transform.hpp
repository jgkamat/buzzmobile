#ifndef IMAGE_TRANSFORM_HPP
#define IMAGE_TRANSFORM_HPP

#include <stdio.h>
#include <opencv2/opencv.hpp>

namespace image_transform {

// The width and height of the final image. Will depend on the size of the
// initial image.
// TODO: modify to fit the car's image dimensions.
int persp_transform_width = 480;
int persp_transform_height = 640;

// The coords of the edges of a chessboard lying on the ground,
// in a training image.
// These coords work for when the camera is at exactly 42.25 inches above
// ground, at a slight angle downwards (you can open rviz to title the angle).
cv::Point2f persp_transform_src_pts[4] = {
  cv::Point2f(307,350),
  cv::Point2f(342,350),
  cv::Point2f(343,373),
  cv::Point2f(303,373)
};

// The points where each of the above will be projected onto.
// In our case, in the middle wrt x, and close to bottom wrt y.
cv::Point2f persp_transform_dst_pts[4] = {
  cv::Point2f(-10+(persp_transform_width/2),persp_transform_height-20-30),
  cv::Point2f(10+(persp_transform_width/2),persp_transform_height-20-30),
  cv::Point2f(10+(persp_transform_width/2),persp_transform_height-30),
  cv::Point2f(-10+(persp_transform_width/2),persp_transform_height-30)
};

cv::Mat transform = getPerspectiveTransform(persp_transform_src_pts,
                                            persp_transform_dst_pts);

void transform_perspective(const cv::Mat &input, cv::Mat &output) {
  cv::warpPerspective(input, output, transform,
                      cv::Size(persp_transform_width, persp_transform_height));
}

}

#endif // IMAGE_UTILS_HPP
