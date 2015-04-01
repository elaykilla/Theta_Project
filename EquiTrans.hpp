#ifndef EquiTrans_H
#define EquiTrans_H

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


public:
  EquiTrans();
  EquiTrans(double);

  void makeCubeFaces(Mat, Mat[6]);
  void makeCubeFaces2(Mat, Mat[6], ViewDirection[6]);
  void getCubeFaceViewingDirections(ViewDirection[6]);
  //void getCubeFaceViewingDirections(ViewDirection[6]);
  bool setFOV(double, double);
  Mat toPerspective(Mat, double, double);
  void toPerspective(Mat image, PersCamera &cam);
  Mat toPerspectiveCore(Mat, double, double);
  void unsetFOV();
  void toEquirectCore(double i_c, double j_c, double focal_length, ViewDirection vd, double d_rows, double d_ncols, double *ec_i, double *ec_j);
  void convSpherePointToEquiCoord(Point3f point, Mat equi_img, double *x, double *y);
  Point2f toPersPoint(Mat image, PersCamera cam, Point2f point);
};

#endif
