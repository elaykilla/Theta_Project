#ifndef PclManip
#define PclMnip

#include "cv_headers.hpp"

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
  void unsetFOV();
};

#endif
