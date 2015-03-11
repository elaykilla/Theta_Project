/*
 * Transform images in Equirectangular form
 */
#include <math.h>
#include "EquiTrans.hpp"

using namespace std;

EquiTrans::EquiTrans(){
  film_width  = 640;
  film_height = 480;
  focal_length = 640.0f;
  hfov = -1.0f;
  vfov = -1.0f;
}

EquiTrans::EquiTrans(double focal_val){
  film_width  = 640;
  film_height = 480;
  focal_length = focal_val;
}

bool EquiTrans::setFOV(double hfov_deg, double vfov_deg){
  bool ret = true;
  if(hfov_deg < 0.0 || hfov_deg > 360.0 || vfov_deg < 0.0 || vfov_deg > 360.0){
    ret = false;
  }else{
    hfov = hfov_deg;
    vfov = vfov_deg;
  }
  return ret;
}

void EquiTrans::unsetFOV(){
  hfov = -1.0f;
  vfov = -1.0f;
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
Mat EquiTrans::toPerspective(Mat src, double cam_phi_deg, double cam_theta_deg){
  bool debug = false;
  

  double focal_square = focal_length * focal_length;

  Mat dst = Mat::zeros(film_height, film_width, CV_8UC3);

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

  int half_ht = film_height/2, half_wd = film_width/2;
  double full_hor = 2.0 * M_PI;
  double hfov_rad = hfov / 180.0 * M_PI, vfov_rad = vfov / 180.0 * M_PI;

  // i_c, j_c: center origin, left-right, top-down
  double min_theta = - 0.5 * M_PI, max_theta = 0.5 * M_PI;
  int i_c, j_c;

  
  // Rotation around Y (vertical-down) axis.
  for(int j = 0;j < film_height;j++){
    double y = (double)(j - half_ht);
    j_c = j - half_ht;
  
    for(int i = 0;i < film_width;i++){
      i_c = i - half_ht;
      double phi_c = atan((double)i_c/focal_length);
      double theta = atan(cos(phi_c) * (double)j_c/(double)focal_length);

      double phi = phi_c;
      
      if(debug)
	cout  << "( " << phi << ", " << theta << " )\n";

      // Convert to 3D
      double R = sqrt((double)(j_c * j_c) + focal_square);
      theta = atan(- (double)j_c/focal_length); // bottom-top
      theta += (- cam_theta_pi);

      bool theta_flip = false;
      if(abs(theta) > M_PI/2.0) theta_flip = true;
      
      double Z = sin(theta) * R; // vertical coordinate
      double X = cos(theta) * R; // depth
      double Y = i_c; // horizontal
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

      // Convert to equirectangular coordinate system
      // ec_i, ec_j: center origin, left-right, top-down
      double ec_i = (int)(phi/(M_PI*2.0) * d_ncols);
      double ec_j = (int)(-theta/M_PI * d_nrows);

 
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

//
// Get image size in pixels to the Field Of view
//     assuming square image plane
//
void EquiTrans::getCubeCam(Mat src, int *width, int *height, double *focal_length)
{
  *width = (int)((double)src.cols * hfov/360.0);
  *height = *width; 
  double fov_rad = hfov *M_PI / 180.0;

  *focal_length = (double)*width/(2.0 * tan(fov_rad/2.0));
  
  bool debug = true;
  if(debug){
    cout << "width = ";
    cout << *width << endl;

    cout << "focal length = ";
    cout << *focal_length << endl;
  }
}

/*
 * make 6 cube face images from an image in equirectangular format.
 *    Starting from panning of 0.0 degree: 
 *    front, right, back, left, top-front, and bottom-front
 */
void EquiTrans::makeCubeFaces(Mat src, Mat faces[6]){
  double cube_fov = 90.0;
  double pan_deg = 0.0, tilt_deg = 0.0;
  
  setFOV(cube_fov, cube_fov);
  int width, height;
  double focal_val;
  getCubeCam(src, &width, &height, &focal_val);
  setCamera(width, height, focal_val);

  Mat face;
  // side
  int i;
  for(i = 0;i < 4;i++, pan_deg += 90.0){
    face = toPerspective(src, pan_deg, tilt_deg);
    faces[i] = face;
  }
  // top
  pan_deg = 0.0, tilt_deg = 90.0;
  face = toPerspective(src, pan_deg, tilt_deg);
  faces[i++] = face;

  // bottom
  pan_deg = 0.0, tilt_deg = -90.0;
  face = toPerspective(src, pan_deg, tilt_deg);
  faces[i++] = face;
}


/*
 * Set a camera (film size and focal length) to render
 */
void EquiTrans::setCamera(int width, int height, double focal_val){
  film_width =  width;
  film_height = height;
  focal_length = focal_val;
}
