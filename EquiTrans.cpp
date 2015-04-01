/*
 * Transform images in Equirectangular form to those in perspective cameara.
 */
#include <math.h>
#include "EquiTrans.hpp"

using namespace std;
using namespace cv;

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
 * src: Equirectangular image
 * cam_phi_deg: camera pannng in degrees (center origin, left-right)
 * cam_theta_deg: camera tilting in degrees (center origin, botom-top)
 *
 */
Mat EquiTrans::toPerspective(Mat src, double cam_phi_deg, double cam_theta_deg){
  bool debug = false;

  ViewDirection vd; // camera viewing direction
  vd.pan = cam_phi_deg/180.0*M_PI;
  vd.tilt = cam_theta_deg/180.0*M_PI;

  Mat dst = Mat::zeros(film_height, film_width, CV_8UC3);
  int nrows = src.rows, ncols = src.cols;
  double d_nrows = (double)nrows, d_ncols = (double)ncols;

  if(debug){
    cout << "nrows = " << nrows << "\n";
    cout << "ncols = " << ncols << "\n";
  }

  // warping vectors
  Mat warpI = Mat::zeros(film_height, film_width, CV_32F);
  Mat warpJ = Mat::zeros(film_height, film_width, CV_32F);

  int half_ht = film_height/2, half_wd = film_width/2;

  // i_c, j_c: center origin, left-right, top-down
  double i_c, j_c;
  double ec_i, ec_j; // image coordinates in equirectangular format.
  double half_equi_ht = (double)(nrows/2);
  double half_equi_wd = (double)(ncols/2);
  
  // Rotation around Y (vertical-down) axis.
  for(int j = 0;j < film_height;j++){
    j_c = (double)j - half_ht + 0.5;
  
    for(int i = 0;i < film_width;i++){
      i_c = (double)(i - half_wd) + 0.5;

      toEquirectCore(i_c, j_c, focal_length, vd, d_nrows, d_ncols, &ec_i, &ec_j);
 
      // Render warped image
      ec_i += (half_equi_wd - 0.5);
      ec_j += (half_equi_ht - 0.5);

      int e_i = (int)ec_i + ncols/2;   // image coordinates in Equirectangular format
      int e_j = (int)ec_j + nrows/2;   // top-left origin, left-right, top-bottom

      warpI.at<float>(j, i) =  ec_i;
      warpJ.at<float>(j, i) =  ec_j;

     if(debug)
	cout  << "( " << ec_i << ", " << ec_j << " )\n";

     if(debug)
       cout  << "( " << e_i << ", " << e_j << " )\n";
    }
  }
  
  WarpImage wi;

  wi.setLinear();  // Bi-linear interpolation
  wi.warp(src, warpI, warpJ, dst);

  warpI.release();
  warpJ.release();

  return dst;
}

/*
 * Get image size in pixels to the Field Of view
 *     assuming square image plane
 */
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

void EquiTrans::makeCubeFaces2(Mat src, Mat faces[6], ViewDirection v_directions[6]){
  double cube_fov = 90.0;
  double pan_deg = 0.0, tilt_deg = 0.0;
  
  setFOV(cube_fov, cube_fov);
  int width, height;
  double focal_val;
  getCubeCam(src, &width, &height, &focal_val);
  setCamera(width, height, focal_val);

  Mat face;
  ViewDirection vd;
  // side
  int i;
  for(i = 0;i < 4;i++, pan_deg += 90.0){
    face = toPerspective(src, pan_deg, tilt_deg);
    faces[i] = face;
    
    //Add Viewing Direction
    vd.pan = pan_deg;
    vd.tilt = tilt_deg;
    v_directions[i] = vd;
  }
  // top
  pan_deg = 0.0, tilt_deg = 90.0;
  face = toPerspective(src, pan_deg, tilt_deg);
  faces[i++] = face;
  
  //Vd 
  vd.pan = pan_deg, vd.tilt = tilt_deg;
  v_directions[i++] = vd;

  // bottom
  pan_deg = 0.0, tilt_deg = -90.0;
  face = toPerspective(src, pan_deg, tilt_deg);
  faces[i++] = face;
  
  //Vd 
  vd.pan = pan_deg, vd.tilt = tilt_deg;
  v_directions[i++] = vd;

}


/*
* Get the viewing directions for each of the 6 cubic faces
*/
void getCubeFaceViewingDirections(ViewDirection v_directions[6]){
	double pan_deg = 0.0, tilt_deg = 0.0;
	ViewDirection vd;
	int i =0;
	for(i = 0;i < 4;i++, pan_deg += 90.0){
		
   		vd.pan = pan_deg;
   		vd.tilt = tilt_deg;
  		v_directions[i] = vd;
  	}
	// top
  	vd.pan = 0.0, vd.tilt = 90.0;
	v_directions[i++] = vd;
	
	// bottom
  	vd.pan = 0.0, vd.tilt = -90.0;
  	v_directions[i++] = vd;
}


/*
 * Set a camera (film size and focal length) to render
 */
void EquiTrans::setCamera(int width, int height, double focal_val){
  film_width =  width;
  film_height = height;
  focal_length = focal_val;
}

/*
 * Convert a point coordinates  in perspective to those in equirectangular
 *    Both origins are at the centers.
 *
 *    Notation is the same with Leonard Koller Sach' MS thesis.
 *
 *    i_c, i_j; input point coordinates in the perpective camera (origin at the center)
 *    focal_length: in pixel
 *    vd: camera viewing direction (pan (left-right) and tilt (bottom-top))
 *    d_nrows, d_ncols: equirenctangular image size in double
 *    ec_i, ec_j: outpuot point coordinates in the equirectangular image
 *
 */
void EquiTrans::toEquirectCore(double i_c, double j_c, double focal_length, ViewDirection vd, double d_nrows, double d_ncols, double *ec_i, double *ec_j){

  double lambda = atan(i_c/focal_length); // panning
  // Convert to 3D
  double R = sqrt(j_c*j_c + focal_length*focal_length);


 // bottom-top along vertical axis
  double phi = atan(- j_c/focal_length);
  phi += vd.tilt;

  bool tilt_flip = false;
  if(abs(phi) > M_PI/2.0) tilt_flip = true;
      
  double Z = sin(phi) * R; // vertical coordinate in 3D
  double X = cos(phi) * R; // depth
  double Y = i_c; // horizontal coordinate in 3D
  lambda = atan(Y/X);
  double RYX = sqrt(Y*Y + X*X);
  phi = atan(Z/RYX); // tilting angle for each pixel

  // case for overflow of panning
  if(tilt_flip){
    lambda += M_PI;
  }
  lambda += vd.pan;

  // case for underflow of tilting
  if(phi < -M_PI/2.0){
    phi = - M_PI - phi;
    lambda += M_PI; 
  } 
  if(lambda <= -M_PI){
    lambda += (2.0 * M_PI);
  }

  // case for overflow of panning
  if(lambda > M_PI){
    lambda -= (2.0 * M_PI);
  }

  // Convert to equirectangular coordinate system
  // ec_i, ec_j: center origin, left-right, top-down
  *ec_i = lambda/(M_PI*2.0) * d_ncols;
  *ec_j = -phi/M_PI * d_nrows;
}

/*
 * Convert a 3d point on the unit sphere to pan and tilt angles
 *   Coordinate system is the same with  Sacht's MS thesis.
 *
 *  point: a 3D point on the unit sphere
 *  pan, tilt: point angle from the center coordinate in radian.
 * 
 */
void EquiTrans::convSpherePointToAngles(Point3f point, double *pan, double *tilt){
  *tilt = - asin((double)point.z);

  if(point.x >= 0.0){
    *pan= atan((double)point.y/(double)point.x);
  }else{
    float nx = - point.x;
    if(point.y >= 0.0){
      *pan = atan(nx/(double)point.y) + M_PI/2.0;
    }else{
      *pan = atan(nx/(double)point.y) - M_PI/2.0;
    }
  }
}


/*
 * Convert angles for a direction to images coordinates in equirectangular format.
 *    Coodinate system for angles:
 *       Center origin:
 *       pan: left-to-right, tilt: bottom-to-top in radian
 *
 *    Coordinate system in eqirectangular format: top-left origin
 *      x: horizontal pixel coordinate
 *      y: vertical pixel coordinate
 */
void EquiTrans::convAnglesToEquiCoord(double pan, double tilt, Mat equi_img, double *x, double *y){
  double width = (double)equi_img.cols;
  double height = (double)equi_img.rows;
  
  double x_offset =  width/2.0 - 0.5;
  double y_offset =  height/2.0 - 0.5;

  *x = width/(2.0*M_PI)*pan + x_offset;
  *y = height/M_PI*tilt + y_offset;
}


/*
 * Convert a 3d point on the unit sphere to image coordinates in equirectangular format.
 *    Coordinate system in eqirectangular format: top-left origin
 *      x: horizontal pixel coordinate
 *      y: vertical pixel coordinate
 */
void EquiTrans::convSpherePointToEquiCoord(Point3f point, Mat equi_img, double *x, double *y){
  double pan = 0.0, tilt= 0.0;

  convSpherePointToAngles(point, &pan, &tilt);
  convAnglesToEquiCoord(pan, tilt, equi_img, x, y);
}

/*
 * Generate a perspective image specified by the perspective camera
 *     viewing angles and field of views need to be provided.
 *     
 */

 void EquiTrans::toPerspective(Mat src, PersCamera &cam){
  bool debug = false;

  ViewDirection vd = cam.view_dir;
  int nrows = src.rows, ncols = src.cols;
  double d_nrows = (double)nrows, d_ncols = (double)ncols;
  int film_height = cam.image.rows;
  int film_width = cam.image.cols;

  // set focal_length
  focal_length = cam.focal_length;
  
  if(debug){
    cout << "nrows = " << nrows << "\n";
    cout << "ncols = " << ncols << "\n";
  }

  // warping vectors
  Mat warpI = Mat::zeros(film_height, film_width, CV_32F);
  Mat warpJ = Mat::zeros(film_height, film_width, CV_32F);

  int half_ht = film_height/2, half_wd = film_width/2;

  // i_c, j_c: center origin, left-right, top-down
  double i_c, j_c;
  double ec_i, ec_j; // image coordinates in equirectangular format.
  double half_equi_ht = (double)(nrows/2);
  double half_equi_wd = (double)(ncols/2);
  
  // Rotation around Y (vertical-down) axis.
  for(int j = 0;j < film_height;j++){
    j_c = (double)j - half_ht + 0.5;
  
    for(int i = 0;i < film_width;i++){
      i_c = (double)(i - half_wd) + 0.5;

      toEquirectCore(i_c, j_c, focal_length, vd, d_nrows, d_ncols, &ec_i, &ec_j);
 
      // Render warped image
      ec_i += (half_equi_wd - 0.5);
      ec_j += (half_equi_ht - 0.5);

      int e_i = (int)ec_i + ncols/2;   // image coordinates in Equirectangular format
      int e_j = (int)ec_j + nrows/2;   // top-left origin, left-right, top-bottom

      warpI.at<float>(j, i) =  ec_i;
      warpJ.at<float>(j, i) =  ec_j;

     if(debug)
	cout  << "( " << ec_i << ", " << ec_j << " )\n";

     if(debug)
       cout  << "( " << e_i << ", " << e_j << " )\n";
    }
  }
  
  WarpImage wi;

  wi.setLinear();  // Bi-linear interpolation
  wi.warp(src, warpI, warpJ, cam.image);

  warpI.release();
  warpJ.release();
}


/*
 * Convert a point in equirectangular image coordinates to pan/tilt angles in radian
 *   pan/titl angles is in the coorinate system whose origin at the center.
 */
void EquiTrans::convEquiToAngles(Mat equi_image, Point2f equi_point, double &pan, double &tilt){
  double d_wd = equi_image.cols;
  double d_ht = equi_image.rows;

  double p_pan = equi_point.x - d_wd/2.0 + 0.5; // panning in pixel
  double p_tilt = d_ht/2.0 - equi_point.y - 0.5; // tilting in pixel

  pan = p_pan / d_wd * 2.0 * M_PI;
  tilt = p_tilt / d_ht * M_PI;
}

/*
 * Convert the image coordinates in equirectangular format to those in the perspectivei image.
 *
 * equi_point: image coordinates in equirectangular format (origin: top-left)
 */
Point2f EquiTrans::toPersPoint(Mat equi_image, PersCamera cam, Point2f equi_point){

  double r_pan = 0.0, r_tilt = 0.0; // in radian
  convEquiToAngles(equi_image, equi_point, r_pan, r_tilt);

  r_pan -= cam.view_dir.pan;
  r_tilt -= cam.view_dir.tilt;

  // center origin
  double c_i = cam.focal_length * tan(r_pan); // horizontal 
  double c_j = cam.focal_length * tan(r_tilt); // vertical bottom-top

  // top-left origin
  double d_wd = (double)cam.image.cols;
  double d_ht = (double)cam.image.rows;

  Point2f pers_point;
  pers_point.x = (float)(c_i + d_wd/2.0 - 0.5);
  pers_point.y = (float)(d_ht/2.0 - c_j - 0.5);

  return pers_point;
}
