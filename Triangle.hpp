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
  /*
 * convert an image in equirectangular to the perspective image, 
 *  which includes the triangle      
 *  equi_image: the ominidirectinal image in equirectangular format
 *  vertices:   the three vertices of the triangle in equi_image
 *              (origin: top-left)
 *               x: panning angle, y: tilting angle (in pixel)
 *  return:     the perpective image that includes the triagle.
 */
  cv::Mat convToPersRectImage(cv::Mat equi_image, cv::Vec6f vertices);
  
 /**
 * Same as above function but using 2 Triangles
 */
  cv::Mat convToPersRectImageTwo(cv::Mat equi_image, cv::Vec6f vertices1, cv::Vec6f vertices2);
  
  /**
  * Function to generate perspective view from triangle defined by its 3 coordinates in Euclidien space and
  *   cartesian coordinates
  * @Input:
  * - Equi_image: image in equirectangular format (W = 2*H)
  * - vertices3D: triangle defined by its 3 vertices in (x,y,z) format
  */
  cv::Mat convToPersRectImageBarycentric(cv::Mat equi_image, Vec9f vertices3D);
  
  
    /**
  * Function to generate perspective view from triangle defined by its 3 coordinates in Euclidien space and
  *   cartesian coordinates
  * @Input:
  * - Equi_image: image in equirectangular format (W = 2*H)
  * - triangle3D1, triangle3D2: 2 triangles given by their parameters in 3D 
  */
  cv::Mat convToPersRectImageBarycentricTwo(cv::Mat equi_image, Vec9f triangle3D1, Vec9f triangle3D2);
  
  
  /*
 * Get the perpective camera that includes the list of triangles.
 *     Triangles could be one or a pair of matching
 */
  PersCamera getPersCamParams(cv::Mat equi_image, cv::Vec6f vertices);
  
  /** 
  * This function returns the intermediate camera parameters given 
  * - 2 perspective cameras
  * - the desired interpolated position defined by pos and dist = pos/dist 
  */
  PersCamera getInterpolatedPersCamParams( PersCamera cam1, PersCamera cam2, double dist, double pos);
  
   /*
 * Get the perpective camera that includes a triangle by using the barycenter as point and the 
 * radius of the circumcircle.
 * @Input
 * - vertices: the 3 vertices of the triangle given in 3D cartesian coordinates
 */
  PersCamera getPersCamParamsBarycentric(cv::Mat equi_image, Vec9f vertices);
  
  
    /*
 * Get the perpective camera that includes both triangles by using the barycenter as point and the 
 * radius of the circumcircle.
 * @Input
 * - vertices: the 3 vertices of the triangle given in 3D cartesian coordinates
 */
  PersCamera getPersCamParamsBarycentricTwo(cv::Mat equi_image, Vec9f triangle3D1, Vec9f triangle3D2);
  
  /** 
  * Same as above but using 2 triangles 
  */
  PersCamera getPersCamParamsTwo(cv::Mat equi_image, cv::Vec6f vertices1, cv::Vec6f vertices2 );
  
  /*
 * Convert the image coordinates of a triangle from equirectangular to perspective
 */
  cv::Vec6f convToPersTriangle(cv::Mat equi_image, PersCamera pers_cam, cv::Vec6f vertices);
  
  /*
 * convert 6 element floating type to a set of points.
 */
  vector<cv::Point2f> convToPointVector(Vec6f vec);
  
  
  void showTriangle(cv::Mat image, Vec6f vec); 
  
  /*
 * Show single triangle
 */
  void showTriangle(cv::Mat image, vector<Point2f> point_vector); 

};
#endif
