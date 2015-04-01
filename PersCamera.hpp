/*
 * Perspective camera configulation for converting with omnidirctional imagse in equirectangular format.
 */
#ifndef _PERSCAMERA_HPP
#define _PERSCAMERA_HPP
#include <opencv2/opencv.hpp>
#include "ViewDirection.hpp"

class PersCamera {

public:
  Mat image; // image
  double fov_h;  // horizontal field of view angle
  double fov_v;  // vertical fiedl of view angle
  double focal_length; // focal length in pixels 
  ViewDirection view_dir; // viewing direction represented by pan/tilt angles.

  PersCamera();
  void setCamera(Mat image, double h_fov, double v_fov, ViewDirection vd);

};
#endif
