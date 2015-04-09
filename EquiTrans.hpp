#ifndef _EQUITRANS_HPP
#define _EQUITRANS_HPP

#include "cv_headers.hpp"
#include "ViewDirection.hpp"
#include "WarpImage.hpp"
#include "PersCamera.hpp"

using namespace cv;

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
  int getFOV(Mat, int*, int*, double*);
  void getCubeCam(Mat, int*, int*, double*);
  void setCamera(int, int, double);
  void convSpherePointToAngles(Point3f point, double *pan, double *tilt);
  void convAnglesToEquiCoord(double pan, double tilt, Mat equi_img, double *x, double *y);
  void convEquiToAngles(Mat iamge, Point2f point, double &pan, double &tilt);
  Rect getEquiRegionPoints(PersCamera cam, Mat image);
  Rect getEquiRegionFull(PersCamera cam, Mat image);
  Rect getEquiRegionBorder(PersCamera cam, Mat image);

  void initMinus(Mat &image);
  bool isInsideTriangle(Vec6f vertices, Point2f point);
  bool checkVectorAngle(Point3d a, Point3d b);

public:
  EquiTrans();
  EquiTrans(double);

  void makeCubeFaces(Mat, Mat[6]);
  void makeCubeFaces2(Mat, Mat[6], PersCamera[6]);
  void getCubeFaceViewingDirections(ViewDirection[6]);
  void getCubeFaceCams(PersCamera[6]);
  //void getCubeFaceViewingDirections(ViewDirection[6]);
  bool setFOV(double, double);
  Mat toPerspective(Mat, double, double);
  Mat toPerspective(Mat image, PersCamera &cam);
  Mat toPerspectiveCore(Mat, double, double);
  void unsetFOV();
  void toEquirectCore(double i_c, double j_c, double focal_length, ViewDirection vd, double d_rows, double d_ncols, double *ec_i, double *ec_j);
  void convSpherePointToEquiCoord(Point3f point, Mat equi_img, double *x, double *y);
  Point2f toPersPoint(Mat image, PersCamera cam, Point2f point);
  void toEquirectangular(PersCamera cam, Mat &image);
  void toEquirectangular(PersCamera cam, Vec6f triangle, Mat &image);
  Point3d rotateTilt(double angle_rad, Point3d point);
  Point3d rotatePan(double angle_rad, Point3d point);
  Point2f toEquiPoint(PersCamera cam, Mat image, Point2f point);
  Point2f toCenterCoord(Point2f point, Mat image);
  Point2f toTopLeftCoord(Point2f point, Mat image);
};

#endif
