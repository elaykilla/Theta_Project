#ifndef _EQUITRANS_HPP
#define _EQUITRANS_HPP

#include "cv_headers.hpp"
#include "ViewDirection.hpp"
#include "WarpImage.hpp"
#include "PersCamera.hpp"

//using namespace cv;

/*
 * Transform with Equirectangular form
 */
class EquiTrans {
  
private:
  double focal_length; // in file size pixel
  int film_width;   // Film size in pixel
  int film_height;
  double hfov;  // Horizontal field of view in degrees
  double vfov;  // Vertical field of view in degrees
  
  /**
  * Get current Field of View.
  */
  int getFOV(Mat, int*, int*, double*);
  
  /*
 * Get image size in pixels to the Field Of view
 *     assuming square image plane
 */
  void getCubeCam(Mat, int*, int*, double*);
  
  /**
  * Set the Camera Parameters for the transform.
  */
  void setCamera(int, int, double);
  
  /*
 * Convert angles for a direction to images coordinates in equirectangular format.
 *    Coodinate system for angles:
 *       Center origin:
 *       pan: left-to-right, tilt: bottom-to-top in radian
 *
 *    Coordinate system in eqirectangular format: top-left origin
 *      x: horizontal pixel coordinate
 *      y: vertical pixel coordinate
 */
  void convAnglesToEquiCoord(double pan, double tilt, Mat equi_img, double *x, double *y);

  /*
   * Convert a point in equirectangular image coordinates to pan/tilt angles in radian
   *   pan/titl angles is in the coorinate system whose origin at the center.
   */
  void convEquiToAngles(Mat equi_image, Point2f equi_point, double &pan, double &tilt);
  
  
  Rect getEquiRegionPoints(PersCamera cam, Mat image);
  
  Rect getEquiRegionFull(PersCamera cam, Mat image);
  
  Rect getEquiRegionBorder(PersCamera cam, Mat image);
  

  void initMinus(Mat &image);
  
  bool isInsideTriangle(Vec6f vertices, Point2f point);
  
  bool checkVectorAngle(Point3d a, Point3d b);

public:
  EquiTrans();
  EquiTrans(double);

 /*
  * Convert a 3d point on the unit sphere to pan and tilt angles
  *   Coordinate system is the same with  Sacht's MS thesis.
  *
  *  point: a 3D point on the unit sphere
  *  pan, tilt: point angle from the center coordinate in radian.
  * 
  */
  void convSpherePointToAngles(Point3f point, double *pan, double *tilt);

  /* Convert a point in 3D unit sphere to that on the perspective camera. */
  Point2d convSpherePointtoPersCoord(PersCamera cam, Point3d point);

  
  /**
  * Convert 3d points on the unit sphere to points on a given perspective camera
  * @Inputs
  * triangle3d:	vector of 3 points in 3d
  * cam:	a PersCamera 
  * @Outputs
  * triangle2d:	a vector of 3 points in 2d
  */
  Vec6f conv3DTriangletoPers(vector<Point3d> triangle3d, PersCamera cam);
 /*
  * make 6 cube face images from an image in equirectangular format.
  *    Starting from panning of 0.0 degree: 
  *    front, right, back, left, top-front, and bottom-front
  */
  void makeCubeFaces(Mat, Mat[6]);
  
  /**
  * Function to convert an Equirectangular image into cube and returns:
  * - 6 cube faces 
  * - corresponding PersCamera for each of the faces
  */
  void makeCubeFaces2(Mat, Mat[6], PersCamera[6]);
  
  /**
  * Functions returns the viewing direction of each of the 6 faces of the cube
  */
  void getCubeFaceViewingDirections(ViewDirection[6]);
  
  
  /**
  * Returns the PersCameras for each of the 6 cube faces
  */
  void getCubeFaceCams(PersCamera[6]);
  
  //void getCubeFaceViewingDirections(ViewDirection[6]);
  
  bool setFOV(double, double);
  
   /*
 * Convert a 3d point on the unit sphere to pan and tilt angles
 *   Coordinate system is the same with  Sacht's MS thesis.
 *
 *  point: a 3D point on the unit sphere
 *  pan, tilt: point angle from the center coordinate in radian.
 * 
 */
  void convSpherePointToAngles(Point3f point, double &pan, double &tilt);
  

  Mat toPerspective(Mat, double, double);
  
  /**
  * Extracts a perpective view from an Equirectangular Image using the given PersCamera
  * parameters
  * @input:
  *	- image: Mat of the Omnidirectional Image
  *	- cam: The Perspective camera to get image
  * @output:
  *	- Perpespective camera image
  */
  Mat toPerspective(Mat image, PersCamera &cam);
  
  /**
  * Extracts Perspective image using panning and tilting viewing angles
  * @input:
  * 	- src: Equirectangular image
  * 	- cam_phi_deg: camera pannng in degrees (center origin, left-right)
  * 	- cam_theta_deg: camera tilting in degrees (center origin, botom-top)
  *
  * @output:
  *  	- Perspective camera image
  */
  Mat toPerspectiveCore(Mat, double, double);
  
  /**
  * Removes the current Field of View settings
  */
  void unsetFOV();
  
  /*
 * Convert a point coordinates  in perspective to those in equirectangular
 *    Both origins are at the centers.
 *
 *    Notation is the same with Leonard Koller Sach' MS thesis.
 *
 *    i_c, i_j; input point coordinates in the perpective camera (origin at the center)
 *    focal_length: in pixel
 *    vd: camera viewing direction (pan (left-right) and tilt (bottom-top))
 *    d_nrows, d_ncols: equirenctangular image size in double
 *    ec_i, ec_j: outpuot point coordinates in the equirectangular image
 *
 */
  void toEquirectCore(double i_c, double j_c, double focal_length, ViewDirection vd, double d_rows, double d_ncols, double *ec_i, double *ec_j);
  
  
  
  /*
 * Convert a 3d point on the unit sphere to image coordinates in equirectangular format.
 *    Coordinate system in eqirectangular format: top-left origin
 *      x: horizontal pixel coordinate
 *      y: vertical pixel coordinate
 */
  void convSpherePointToEquiCoord(Point3f point, Mat equi_img, double *x, double *y);
  
  
  /*
 * Convert the image coordinates in equirectangular format to those in the perspectivei image.
 *
 * equi_point: image coordinates in equirectangular format (origin: top-left)
 */
  Point2f toPersPoint(Mat image, PersCamera cam, Point2f point);
  
  
  /*
 * Convert a perspective image to a portion in the equirectangular image.
 * 
 *  pers: perspective camera including its image.
 *  equi: equirectangular image to render
 */
  void toEquirectangular(PersCamera cam, Mat &image);
  
  
  /*
 * Convert a traingle in perspective image to the portion in the equirectangular image.
 * 
 *  pers: perspective camera including its image.
 *  tri:  triangle in the perspecive camera
 *  equi: equirectangular image to render
 */
  void toEquirectangular(PersCamera cam, Vec6f triangle, Mat &image);
  
  
  
/*
 * 3D rotation around the tilting axis. 
 *    Y axis in Sacht's notation
 *   
 *    angle: rotational angle in radian
 *    point: 3D point
 */
  Point3d rotateTilt(double angle_rad, Point3d point);
  
  /*
 * 3D rotation around the tilting axis. 
 *    Y axis in Sacht's notation
 *   
 *    angle: rotational angle in radian
 *    point: 3D point
 */
  Point3d rotateTilt(Mat invTilt, Point3d point);
  
  
  /*
 * 3D rotation around the panning axis. 
 *    Z axis in Sacht's notation
 *   
 *    angle: rotational angle in radian
 *    point: 3D point
 */
  Point3d rotatePan(double angle_rad, Point3d point);
  
  
  /*
 * Convert image cooridnates of a point in perpective camera to that in equirectangular.
 *     p_p: point image coordinates. (origin: top-left).
 */
  Point2f toEquiPoint(PersCamera cam, Mat image, Point2f point);
  
  /*
 * Convert coordinates of a point from center origin to top-left origin.
 */
  Point2f toCenterCoord(Point2f point, Mat image);
  
  /*
 * Convert coordinates of a point from center origin to top-left origin.
 */
  Point2f toTopLeftCoord(Point2f point, Mat image);
};

#endif
