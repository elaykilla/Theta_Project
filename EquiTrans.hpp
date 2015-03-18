#ifndef PclManip
#define PclMnip

#include "cv_headers.hpp"
#include "ViewDirection.hpp"

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

public:
  EquiTrans();
  EquiTrans(double);

  void makeCubeFaces(Mat, Mat[6]);
  bool setFOV(double, double);
  Mat toPerspective(Mat, double, double);
  Mat toPerspectiveCore(Mat, double, double);
  void unsetFOV();
  Point2f toEquirectPoint(Mat src, ViewDirection dir, Point2f pers);
  void toEquirectCore(double i_c, double j_c, double focal_length, ViewDirection vd, double d_rows, double d_ncols, double *ec_i, double *ec_j);
};

#endif
