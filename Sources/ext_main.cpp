/*
 * Equirectangualr image extension.
 */
#include <stdio.h>
#include "EquiExtend.hpp"

using namespace cv;

main(){
  EquiExtend ext;
  Mat image = imread("../images/R0010103_small.JPG");
  Mat dst = ext.extend(image);

  imshow("Extended image", dst);
  waitKey(0);
}
