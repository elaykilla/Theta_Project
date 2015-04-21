/*
 * WarpImage according to warping vectors for pixels.
 */
#include "WarpImage.hpp"

WarpImage::WarpImage(){
  method = LINEAR;
  wrapping = true;
}

void WarpImage::warp(Mat src, Mat mapI, Mat mapJ, Mat &dst){
  if(method == LINEAR){
    warpLinear(src, mapI, mapJ, dst);
  }else{
    warpNearest(src, mapI, mapJ, dst);
  }
}


/*
 * Warp Image from src to dst using bilinear pixel interpolation
 *    src: source image
 *    dst: destination image
 *    mapI: Warping column vectors for each pixel
 *    mapJ: Warping row vectors for each pixel
 */
void WarpImage::warpLinear(Mat src, Mat mapI, Mat mapJ, Mat &dst){
  bool ret = true;
  int nrows = dst.rows;
  int ncols = dst.cols;
  int i_rmax = src.rows - 1;
  int i_cmax = src.cols - 1;
  float f_cmin = -1.0, f_rmin = -1.0;
  float f_cmax = (float)src.cols;
  float f_rmax = (float)src.rows;

  for(int r = 0;r < nrows;r++){
    for(int c = 0;c < ncols;c++){
      float wi = mapI.at<float>(r, c);
      float wj = mapJ.at<float>(r, c);
      
      if(wi <= f_cmin ||  wj <= f_rmin || wi >= f_cmax || wj >= f_rmax){
	continue;
      }else{
	int i_low = (int)floor(wi);
	int j_low = (int)floor(wj);

	double i_frac = (double)wi - (double)i_low;
	double j_frac = (double)wj - (double)j_low;
	int i_high = (int)i_low + 1;
	int j_high = (int)j_low + 1;

	int e_j = wj;
	int e_i = wi;

	if(wrapping){ // wrapping around the borders.
	  if(i_low < 0) i_low = i_cmax;
	  if(j_low < 0) j_low = i_rmax;
	  if(i_high > i_cmax) i_high = 0;
	  if(j_high > i_rmax) j_high = 0;
	}
	Vec3b pix_00, pix_01, pix_10, pix_11;
	pix_00 = src.at<Vec3b>(j_low, i_low); 
	pix_01 = src.at<Vec3b>(j_low, i_high); 
	pix_10 = src.at<Vec3b>(j_high, i_low); 
	pix_11 = src.at<Vec3b>(j_high, i_high); 


	Vec3b pix_dst;
	for(int k = 0;k < 3;k++){
	  double val_00 = (double)pix_00.val[k];
	  double val_01 = (double)pix_01.val[k];
	  double val_10 = (double)pix_10.val[k];
	  double val_11 = (double)pix_11.val[k];
	  
	  double nval_r0 = (1.0 - i_frac) * val_00 + i_frac * val_01;
	  double nval_r1 = (1.0 - i_frac) * val_10 + i_frac * val_11;
	  double nval = (1.0 - j_frac) * nval_r0 + j_frac * nval_r1;
	  unsigned char nval_c = 0;
	  if(nval > 255.0){
	    nval_c = 255;
	  }if(nval < 0.0){
	    nval_c = 0;
	  }else{
	    nval_c = (unsigned char)nval;
	  }
	  pix_dst.val[k] = nval_c;
	}
	dst.at<Vec3b>(r, c) = pix_dst;
      }
    }
  }

}

/*
 *
 */
void WarpImage::warpNearest(Mat src, Mat mapI, Mat mapJ, Mat &dst){
  bool ret = true;
  int nrows = dst.rows;
  int ncols = dst.cols;
  int rmax = src.rows - 1;
  int cmax = src.cols - 1;

  for(int r = 0;r < nrows;r++){
    for(int c = 0;c < ncols;c++){
      float wi = mapI.at<float>(r, c);
      float wj = mapJ.at<float>(r, c);
      
      if(wi < 0.0 ||  wj < 0.0 || wi > cmax || wj > rmax){
	ret = false;
      }else{
	int e_j = (int)wj;
	int e_i = (int)wi;

	Vec3b intensity;
	intensity = src.at<Vec3b>(e_j, e_i); 
	dst.at<Vec3b>(r, c) = intensity;
      }
    }
  }
}


/*
 * Set nearest neighbor for setting pixel values.
 */
void WarpImage::setNearest(){
  method = NEAREST;
}

/*
 * Set bi-linear interpolation for setting pixel values.
 */
void WarpImage::setLinear(){
  method = LINEAR;
}

