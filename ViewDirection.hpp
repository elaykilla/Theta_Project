/*
 * To hold viewing angles in equirectanguler format
 *     center origin                            
 */
#ifndef _VIEW_DIRECTION_HPP
#define _VIEW_DIRECTION_HPP
#include "cv_headers.hpp"

using namespace cv;

class ViewDirection {

public:
  double pan; // panning angle either in degrees or radian
  double tilt; // tiling angle either in degrees or radian
};
#endif
