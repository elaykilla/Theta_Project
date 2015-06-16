/*
 * Extend an image in equirectangular image for view interpolation.
 *   This is for comparison with 3D unit sphere triangluation.
 */
#include "EquiExtend.hpp"

using namespace std;
using namespace cv;

EquiExtend::EquiExtend(){
}

/*
 * Extend the top
 */
Mat
EquiExtend::top(Mat image, Mat &dst){
  int rmax = image.rows/2;
  int width = image.cols;
  int cmax = image.cols;
  int half_width = image.cols/2;
  int half_height = image.rows/2;

  for(int r = 0;r < rmax;r++){
    for(int c = 0;c < cmax;c++){
      int n_c = c + half_width;
      if(n_c > width){
	n_c -= width;
      }
      int n_r = half_height - r;

      Vec3b pix_src = image.at<Vec3b>(n_r, n_c);
      dst.at<Vec3b>(r, c) = pix_src;
    }
  }
  
  return dst;
}

/*
 * Extend the bottom
 */
Mat
EquiExtend::bottom(Mat image, Mat &dst){
  int half_width = image.cols/2;
  int half_height = image.rows/2;
  int rmin = image.rows + half_height;
  int rmax = image.rows * 2;
  int width = image.cols, height = image.rows;
  int cmax = image.cols;

  for(int r = rmin;r < rmax;r++){
    for(int c = 0;c < cmax;c++){
      int n_c = c + half_width;
      if(n_c > width){
	n_c -= width;
      }
      int n_r = height - (r - rmin) - 1;

      Vec3b pix_src = image.at<Vec3b>(n_r, n_c);
      dst.at<Vec3b>(r, c) = pix_src;
    }
  }
  
  return dst;
}

/*
 * Extend the right
 */
Mat
EquiExtend::right(Mat image, Mat &dst){
  int width = image.cols, height = image.rows;
  int half_width = image.cols/2;
  int half_height = image.rows/2;
  int rmin = half_height;
  int rmax = half_height + height;

  int cmin = width;
  int cmax = width + half_width;

  for(int r = rmin;r < rmax;r++){
    for(int c = cmin;c < cmax;c++){
      int n_c = c - cmin;
      int n_r = r - rmin;

      Vec3b pix_src = image.at<Vec3b>(n_r, n_c);
      dst.at<Vec3b>(r, c) = pix_src;
    }
  }
  
  return dst;
}


/*
 * Extend the image and return the result.
 */
Mat
EquiExtend::extend(Mat image){
  int width = image.cols + image.cols/2;
  int height = 2 * image.rows;

  // copy the original
  Mat dst(height, width, image.type());
  image.copyTo(dst(Rect(Point(0, image.rows/2), image.size())));

  // extend top
  dst = top(image, dst);

  // extend bottom
  dst = bottom(image, dst);

  // extend right
  dst = right(image, dst);

  return dst;
}
