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
  bool warpLinear(Mat src, Mat mapI, Mat mapJ, Mat dst);
  bool warpNearest(Mat src, Mat mapI, Mat mapJ, Mat dst);

public:
  WarpImage();
  bool warp(Mat src, Mat mapI, Mat mapJ, Mat dst);
  void setNearest();
  void setLinear();
};
#endif
