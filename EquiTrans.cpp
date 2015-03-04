/*
 * Transform images in Equirectangular form
 */
#include <math.h>
#include "EquiTrans.hpp"

using namespace std;

EquiTrans::EquiTrans(){
  film_width  = 36.0f;
  film_height = 24.0f;
  focal_length = 36.0f;
}

EquiTrans::EquiTrans(double focal_val){
  film_width  = 36.0f;
  film_height = 24.0f;
  focal_length = focal_val;
}

/*
 * Convert plane image coordinates to rho and theta angles.
 */

/*
 * src: Equirectangular image
 * cam_phi_deg: camera pannng in degrees (center origin, left-right)
 * cam_theta_deg: camera tilting in degrees (center origin, botom-top)
 *
 */
Mat EquiTrans::toRegular(Mat src, double cam_phi_deg, double cam_theta_deg){
  bool debug = false;
  
  int wd = 640;
  int ht = 480;
  double focal_length = 640.0;
  double focal_square = focal_length * focal_length;

  Mat dst = Mat::zeros(ht, wd, CV_8UC3);

  double phi_range = 2.0 * atan(film_width/focal_length);
  double theta_range = 2.0 * atan(film_height/focal_length);

  double cam_phi_pi = cam_phi_deg/180.0*M_PI;
  double cam_theta_pi = - cam_theta_deg/180.0*M_PI;

  if(debug){
    cout << "phi_range" << phi_range;
    cout << "theta_range" << theta_range;
  }

  int nrows = src.rows, ncols = src.cols;
  double d_nrows = (double)nrows, d_ncols = (double)ncols;

  if(debug){
    cout << "nrows = " << nrows << "\n";
    cout << "ncols = " << ncols << "\n";
  }

  int half_ht = ht/2, half_wd = wd/2;
  double full_hor = 2.0 * M_PI;

  // i_c, j_c: center origin, left-right, top-down
  double min_theta = - 0.5 * M_PI, max_theta = 0.5 * M_PI;
  int i_c, j_c;
  
  // Rotation around Y (vertical-down) axis.
  for(int j = 0;j < ht;j++){
    double y = (double)(j - half_ht);
    j_c = j - half_ht;
  
    for(int i = 0;i < wd;i++){
      i_c = i - half_ht;
      double phi_c = atan((double)i_c/focal_length);
      double theta = atan(cos(phi_c) * (double)j_c/(double)focal_length);

      double phi = phi_c;
      
      if(debug)
	cout  << "( " << phi << ", " << theta << " )\n";

      //ec_i, ec_j: center origin, left-right, top-down
      int ec_i = (int)(phi/(M_PI*2.0) * d_ncols);
      int ec_j = (int)(theta/M_PI * d_nrows);

      // Convert to 3D
      double R = sqrt((double)(ec_j * ec_j) + focal_square);
      theta = atan(- (double)ec_j/focal_length); // bottom-top
      theta += (- cam_theta_pi);

      bool theta_flip = false;
      if(abs(theta) > M_PI/2.0) theta_flip = true;
      
      double Z = sin(theta) * R; // vertical coordinate
      double X = cos(theta) * R; // depth
      double Y = ec_i; // horizontal
      phi = atan(Y/X);
      double RYX = sqrt(Y*Y + X*X);
      theta = atan(Z/RYX);

      // case for overflow of tilting
      if(theta_flip){
	phi += M_PI;
      }
      phi += cam_phi_pi;

      // case for underflow of tilting
      if(theta < -M_PI/2.0){
	theta = - M_PI - theta;
	phi += M_PI; 
      } 
      if(phi <= -M_PI){
	phi += (2.0 * M_PI);
      }
      if(phi > M_PI){
	phi -= (2.0 * M_PI);
      }

      //ec_i, ec_j: center origin, left-right, top-down
      ec_i = (int)(phi/(M_PI*2.0) * d_ncols);
      ec_j = (int)(-theta/M_PI * d_nrows);

 
      // Render warped image
      int e_i = ec_i + ncols/2;   // image coordinates in Equirectangular format
      int e_j = ec_j + nrows/2;   // top-left origin, left-right, top-bottom.

     if(debug)
	cout  << "( " << ec_i << ", " << ec_j << " )\n";

      if(debug)
	cout  << "( " << e_i << ", " << e_j << " )\n";

      Vec3b intensity;
      intensity = src.at<Vec3b>(e_j, e_i);     
      dst.at<Vec3b>(j, i) = intensity;
    }
  }

  return dst;
}

