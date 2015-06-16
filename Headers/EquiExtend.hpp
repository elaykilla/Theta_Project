/*
 * Extend an image in equirectangular image for view interpolation.
 *   This is for comparison with 3D unit sphere triangluation.
 */
#ifndef _EQUIEXTEND_HPP
#define _EQUIEXTEND_HPP
#include "cv_headers.hpp"

class EquiExtend {

private:
  cv::Mat top(cv::Mat image, cv::Mat &dst);
  cv::Mat bottom(cv::Mat image, cv::Mat &dst);
  cv::Mat right(cv::Mat image, cv::Mat &dst);

public:
  EquiExtend();

  cv::Mat extend(cv::Mat image);

};
#endif
