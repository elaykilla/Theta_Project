/*
 * Triangle handling
 *   convert it from equirectangular to the perspective image
 */
#ifndef _TRIANGLE_HPP
#define _TRINANGLE_HPP
#include "MathCalcs.hpp"

#include "ViewDirection.hpp"
#include "PersCamera.hpp"
#include "EquiTrans.hpp"

using namespace cv;

class Triangle {

private:

public:
  Triangle();
  cv::Mat convToPersRectImage(cv::Mat equi_image, cv::Vec6f vertices);
  cv::Mat convToPersRectImageTwo(cv::Mat equi_image, cv::Vec6f vertices1, cv::Vec6f vertices2);
  cv::Mat NewFunction(cv::Mat equi_image, cv::Vec6f vertices1, cv::Vec6f vertices2);
  PersCamera getPersCamParams(cv::Mat equi_image, cv::Vec6f vertices);
  PersCamera getPersCamParamsTwo(cv::Mat equi_image, cv::Vec6f vertices1, cv::Vec6f vertices2 );
  cv::Vec6f convToPersTriangle(cv::Mat equi_image, PersCamera pers_cam, cv::Vec6f vertices);
  vector<cv::Point2f> convToPointVector(Vec6f vec);
  void showTriangle(cv::Mat image, Vec6f vec); 
  void showTriangle(cv::Mat image, vector<Point2f> point_vector); 

};
#endif
