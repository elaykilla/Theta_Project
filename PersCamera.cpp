/*
 * Perspective Camera
 */
#include "PersCamera.hpp"

PersCamera::PersCamera(){
  fov_h = -1.0;
  fov_v = -1.0;
  focal_length = -1.0;
  view_dir.pan = 0.0;
  view_dir.tilt = 0.0;
}

/*
 * Set the perspective camera 
 *    equi_image: omnidirectional image in equirectangular format
 *    h_fov: horizontal field of view in radian
 *    v_fov: vertical field of view in radian
 *    vd:    vieweing direction in radian (center origin)
 */
void PersCamera::setCamera(Mat equi_image, double h_fov, double v_fov, ViewDirection vd){
  double d_wd = equi_image.cols;
  double d_ht = equi_image.rows;

  double pd_wd = h_fov / (2.0 * M_PI) * d_wd;
  double pd_ht = v_fov / (M_PI) * d_ht;



  int pi_wd = ceil(pd_wd);
  int pi_ht = ceil(pd_ht);

  if(pi_wd % 2 != 0){
    pi_wd++;
  }

  if(pi_ht % 2 != 0){
    pi_ht++;
  }

  // set parameters  
  image = Mat::zeros(pi_ht, pi_wd, equi_image.type());
  fov_h = h_fov;
  fov_v = v_fov;
  focal_length = (double)pi_wd/(2.0 *tan(h_fov/2.0));
  view_dir = vd;
} 
