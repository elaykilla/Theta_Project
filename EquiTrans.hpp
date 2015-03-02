#include "cv_headers.hpp"

using namespace cv;

/*
 * Transform with Equirectangular form
 */
class EquiTrans {
  
private:
  double focal_length; // 35mm equivalent for regular camera projection
  double film_width;   // 35mm equivalent film size
  double film_height;

public:
  EquiTrans();
  EquiTrans(double);
  Mat toRegular(Mat, double, double);
};
