#ifndef CV_HEADERS_H
#define CV_HEADERS_H

#include <opencv2/opencv.hpp>
#include <opencv2/core/core_c.h>
#include <opencv2/core/core.hpp>
#include <opencv2/flann/miniflann.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/legacy/legacy.hpp>

#include "opencv2/video/tracking.hpp"
#include "standard_headers.hpp"
//using namespace cv;
typedef cv::Vec<float, 9> Vec9f;

struct PointWithColor {
  double x;
  double y;
  cv::Scalar color; 
} ;


/**Structure to define a triangle by:
* - It's 3 Vertices given in pixel coordinates
* - The list of points inside the triangle with their color
*/
struct Triangle2D {
	cv::Vec6f points;
	std::vector<PointWithColor> content;
};


typedef cv::Mat Cube[6];
#endif
  
  
