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
  bool wrapping;  // horizontal wrapping around for equirectangular images.
  void warpLinear(Mat src, Mat mapI, Mat mapJ, Mat &dst);
  void warpNearest(Mat src, Mat mapI, Mat mapJ, Mat &dst);

public:
  WarpImage();
  void warp(Mat src, Mat mapI, Mat mapJ, Mat &dst);
  void setNearest();
  void setLinear();
};
#endif
