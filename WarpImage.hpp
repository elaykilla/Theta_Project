#ifndef WARPIMAGE_H
#define WARPIMAGE_H

/*
 * WarpImage
 */  

#include "cv_headers.hpp"

using namespace cv;

class WarpImage {
private:
  enum Method {NEAREST, LINEAR};
  Method method;
  Mat warpLinear(Mat src, Mat mapI, Mat mapJ, Mat dst);
  Mat warpNearest(Mat src, Mat mapI, Mat mapJ, Mat dst);

public:
  WarpImage();
  Mat warp(Mat src, Mat mapI, Mat mapJ, Mat dst);
  void setNearest();
  void setLinear();
};
#endif
