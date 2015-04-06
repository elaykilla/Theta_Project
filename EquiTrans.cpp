/*
 * Transform images between equirectangular form 
 *  to that on the perspective cameara.
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
 * src: Equirectangular image
 * cam_phi_deg: camera pannng in degrees (center origin, left-right)
 * cam_theta_deg: camera tilting in degrees (center origin, botom-top)
 *
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
  dst = wi.warp(src, warpI, warpJ, dst);

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

Mat EquiTrans::toPerspective(Mat src, PersCamera cam){
  bool debug = false;

  ViewDirection vd = cam.view_dir;
  int nrows = src.rows, ncols = src.cols;
  double d_nrows = (double)nrows, d_ncols = (double)ncols;
  int film_height = cam.image.rows;
  int film_width = cam.image.cols;


  Mat dst = Mat::zeros(cam.image.rows, cam.image.cols, cam.image.type());

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
  wi.warp(src, warpI, warpJ, dst);

  warpI.release();
  warpJ.release();

  return dst;
}

/*
 * Convert the image coordinates in equirectangular format to those in the perspectivei image.
 *
 * equi_point: image coordinates in equirectangular format (origin: top-left)
 */
Point2f EquiTrans::toPersPoint(Mat equi_image, PersCamera cam, Point2f equi_point){

  double r_pan = 0.0, r_tilt = 0.0; // in radian
  convEquiToAngles(equi_image, equi_point, r_pan, r_tilt);

  // center origin
  // unit vector 3D coordinates.
  double X = cos(r_pan)*cos(r_tilt); // depth
  double Y = sin(r_pan)*cos(r_tilt); // horizontal
  double Z = sin(r_tilt); // vertical

  Point3d i_point; // initial point
  i_point.x = X;
  i_point.y = Y;
  i_point.z = Z;

  Point3d v_point; // viewing direction
  v_point.x = cos(cam.view_dir.pan) * cos(cam.view_dir.tilt);
  v_point.y = sin(cam.view_dir.pan) * cos(cam.view_dir.tilt);
  v_point.z = sin(cam.view_dir.tilt);

  bool angle_diff = checkVectorAngle(i_point, v_point);

  Point2f pers_point;
  if(angle_diff){
    Point3d r_point = i_point;
    if(cam.view_dir.pan != 0.0){
      r_point = rotatePan(-cam.view_dir.pan, i_point);
    }else{
      r_point = i_point;
    }

    if(cam.view_dir.tilt != 0.0){
      r_point = rotateTilt(-cam.view_dir.tilt, r_point);
    }

    double scale = cam.focal_length / r_point.x;
    double c_i = scale * r_point.y; // horizontal 
    double c_j = scale * r_point.z; // vertical bottom-top

    // top-left origin
    double d_wd = (double)cam.image.cols;
    double d_ht = (double)cam.image.rows;


    pers_point.x = (float)(c_i + d_wd/2.0 - 0.5);
    pers_point.y = (float)(d_ht/2.0 - c_j - 0.5);
  }else{
    pers_point.x = -1.0;
    pers_point.y = -1.0;
  }

  return pers_point;
}


/*
 * Convert coordinates of a point from center origin to top-left origin.
 */
Point2f EquiTrans::toTopLeftCoord(Point2f c_p, Mat image){
  Point2f i_p;
  
  i_p.x = c_p.x + (float)image.cols/2.0 - 0.5;
  i_p.y = c_p.y + (float)image.rows/2.0 - 0.5;

  return i_p;
}

/*
 * Convert coordinates of a point from center origin to top-left origin.
 */
Point2f EquiTrans::toCenterCoord(Point2f i_p, Mat image){
  Point2f c_p;
  
  c_p.x = i_p.x - (float)image.cols/2.0 + 0.5;
  c_p.y = i_p.y - (float)image.rows/2.0 + 0.5;

  return c_p;
}



/*
 * Convert image cooridnates of a point in perpective camera to that in equirectangular.
 *     p_p: point image coordinates. (origin: top-left).
 */
Point2f EquiTrans::toEquiPoint(PersCamera cam, Mat equi_img, Point2f p_p){
  Point2f c_p = toCenterCoord(p_p, cam.image);
  double d_nrows = (double)equi_img.rows;
  double d_ncols = (double)equi_img.cols;

  double ec_i = -1.0, ec_j = -1.0;
  toEquirectCore((double)c_p.x, (double)c_p.y, cam.focal_length, cam.view_dir, d_nrows, d_ncols, &ec_i, &ec_j);
  Point2f e_p;

  e_p.x = (float)ec_i;
  e_p.y = (float)ec_j;

  e_p = toTopLeftCoord(e_p, equi_img);

  return e_p;
}


/*
 * get the rectangular region in equirectangular image by examining all of the image pixels in perspective image.
 */
Rect EquiTrans::getEquiRegionFull(PersCamera pers, Mat equi){
  
  // take the border pixel in the perspective camera.
  int i_max = pers.image.cols - 1;
  int j_max = pers.image.rows - 1;

  vector<Point2f> point_list;
  float f_wd = (float)pers.image.cols - 1.0;
  float f_ht = (float)pers.image.rows - 1.0;


  for(int j =  0;j < j_max;j++){
    for(int i =  0;i < i_max;i++){
      Point2f point;
      point.x = (float)i;
      point.y = (float)j;
      point_list.push_back(point);
    }
  }

  size_t size = point_list.size();
  vector<Point2f> e_pt(size);
  for(int i = 0;i < size;i++){
    e_pt[i] = toEquiPoint(pers, equi, point_list[i]);
  }

  Rect rect;

  float min_x = -1.0, min_y = -1.0;
  float max_x = -1.0, max_y = -1.0;

  for(int i = 0;i < size;i++){

    if(e_pt[i].x < min_x || min_x == -1.0){
      min_x = e_pt[i].x;
    }
    if(e_pt[i].y < min_y || min_y == -1.0){
      min_y = e_pt[i].y;
    }
    if(e_pt[i].x > max_x || max_x == -1.0){
      max_x = e_pt[i].x;
    }
    if(e_pt[i].y > max_y || max_y == -1.0){
      max_y = e_pt[i].y;
    }
  }

  rect.x = min_x;
  rect.y = min_y;
  rect.width = max_x - min_x + 1.0;
  rect.height = max_y - min_y + 1.0;

  return rect;
}


/*
 * get the rectangular region by image border in equirectangular image
 */
Rect EquiTrans::getEquiRegionBorder(PersCamera pers, Mat equi){
  
  // take the border pixel in the perspective camera.
  int i_max = pers.image.cols - 1;
  int j_max = pers.image.rows - 1;

  vector<Point2f> point_list;
  float f_wd = (float)pers.image.cols - 1.0;
  float f_ht = (float)pers.image.rows - 1.0;


  for(int j =  0;j < j_max;j++){

    if(j == 0 || j == j_max){
      for(int i =  0;i < i_max;i++){
	Point2f point;
	point.x = (float)i;
	point.y = (float)j;
	point_list.push_back(point);
      }
    }else{
      
      Point2f point;
      // left border
      point.x = (float)0;
      point.y = (float)j;
      point_list.push_back(point);
      
      // right border
      point.x = (float)i_max;
      point.y = (float)j;
      point_list.push_back(point);
    }
  }

  size_t size = point_list.size();
  vector<Point2f> e_pt(size);
  for(int i = 0;i < size;i++){
    e_pt[i] = toEquiPoint(pers, equi, point_list[i]);
  }

  Rect rect;

  float min_x = -1.0, min_y = -1.0;
  float max_x = -1.0, max_y = -1.0;

  for(int i = 0;i < size;i++){

    if(e_pt[i].x < min_x || min_x == -1.0){
      min_x = e_pt[i].x;
    }
    if(e_pt[i].y < min_y || min_y == -1.0){
      min_y = e_pt[i].y;
    }
    if(e_pt[i].x > max_x || max_x == -1.0){
      max_x = e_pt[i].x;
    }
    if(e_pt[i].y > max_y || max_y == -1.0){
      max_y = e_pt[i].y;
    }
  }

  rect.x = min_x;
  rect.y = min_y;
  rect.width = max_x - min_x + 1.0;
  rect.height = max_y - min_y + 1.0;

  return rect;
}


/*
 * get the rectangular region by some points in equirectangular image
 */
Rect EquiTrans::getEquiRegionPoints(PersCamera pers, Mat equi){
  // convert 8 points of the perspective camera;
  
  size_t size = 9;

  vector<Point2f> points(size);
  float f_wd = (float)pers.image.cols - 1.0;
  float f_ht = (float)pers.image.rows - 1.0;


  points[0].x = 0.0;
  points[0].y = 0.0;
  points[1].x = f_wd/2.0 - 1.0;
  points[1].y = 0.0;
  points[2].x = f_wd - 1.0;
  points[2].y = 0.0;
  points[3].x = 0.0;
  points[3].y = f_ht/2.0 - 1.0;
  points[4].x = f_wd - 1.0;
  points[4].y = f_ht/2.0 - 1.0;
  points[5].x = 0.0;
  points[5].y = f_ht - 1.0;
  points[6].x = f_wd/2.0 - 1.0;
  points[6].y = f_ht - 1.0;
  points[7].x = f_wd - 1.0;
  points[7].y = f_ht - 1.0;
  points[8].x = f_wd/2.0 - 1.0;
  points[8].y = f_ht/2.0 - 1.0;

  vector<Point2f> e_pt(size);
  for(int i = 0;i < size;i++){
    e_pt[i] = toEquiPoint(pers, equi, points[i]);
  }

  Rect rect;

  float min_x = -1.0, min_y = -1.0;
  float max_x = -1.0, max_y = -1.0;

  for(int i = 0;i < size;i++){

    if(e_pt[i].x < min_x || min_x == -1.0){
      min_x = e_pt[i].x;
    }
    if(e_pt[i].y < min_y || min_y == -1.0){
      min_y = e_pt[i].y;
    }
    if(e_pt[i].x > max_x || max_x == -1.0){
      max_x = e_pt[i].x;
    }
    if(e_pt[i].y > max_y || max_y == -1.0){
      max_y = e_pt[i].y;
    }
  }

  rect.x = min_x;
  rect.y = min_y;
  rect.width = max_x - min_x + 1.0;
  rect.height = max_y - min_y + 1.0;

  return rect;
}

/*
 * Initialize a matrix with minus one
 */
void EquiTrans::initMinus(Mat image){

  float val = -1.0f;

  for(int r = 0;r < image.rows;r++){
    for(int c = 0;c < image.cols;c++){
      image.at<float>(r, c) = val;
    }
  }
}

/*
 * Convert a perspective image to a portion in the equirectangular image.
 * 
 *  pers: perspective camera including its image.
 *  equi: equirectangular image to render
 */
Mat EquiTrans::toEquirectangular(PersCamera cam, Mat equi){

  // get the rectangular region in equirectangular image
  Rect rect = getEquiRegionFull(cam, equi);

  // Warping vectors
  int width = equi.cols;
  int height = equi.rows;

  Mat warpI = Mat::ones(height, width, CV_32F);
  Mat warpJ = Mat::ones(height, width, CV_32F);
  warpI.mul(-1.0f);
  warpJ.mul(-1.0f);

  // convert pixels
  int max_i = rect.x + rect.width;
  int max_j = rect.y + rect.height;


  double max_x = (double)cam.image.cols - 1.0;
  double max_y = (double)cam.image.rows - 1.0;
  for(int j= rect.y;j < rect.y + rect.height;j++){
    for(int i = rect.x;i < rect.x + rect.width;i++){
      Point2f e_point;
      e_point.x = (float)i;
      e_point.y = (float)j;

      Point2f p_point = toPersPoint(equi, cam, e_point);
      double x = p_point.x;
      double y = p_point.y;
      if(x < 0.0 || y < 0.0 || x > max_x || y > max_y){
	continue;
      }else{
	warpI.at<float>(j, i) =  x;
	warpJ.at<float>(j, i) =  y;
      }
    }
  }
  
   WarpImage wi;

  wi.setLinear();  // Bi-linear interpolation
  wi.warp(cam.image, warpI, warpJ, equi);

  warpI.release();
  warpJ.release();

  return equi;
}


/*
 * Convert a traingle in perspective image to the portion in the equirectangular image.
 * 
 *  pers: perspective camera including its image.
 *  tri:  triangle in the perspecive camera
 *  equi: equirectangular image to render
 */
Mat EquiTrans::toEquirectangular(PersCamera cam, Vec6f tri, Mat equi){

  // get the rectangular region in equirectangular image
  Rect rect = getEquiRegionFull(cam, equi);

  // Warping vectors
  int width = equi.cols;
  int height = equi.rows;

  Mat warpI = Mat::ones(height, width, CV_32F);
  Mat warpJ = Mat::ones(height, width, CV_32F);
  warpI.mul(-1.0f);
  warpJ.mul(-1.0f);

  // convert pixels
  int max_i = rect.x + rect.width;
  int max_j = rect.y + rect.height;


  double max_x = (double)cam.image.cols - 1.0;
  double max_y = (double)cam.image.rows - 1.0;
  for(int j= rect.y;j < rect.y + rect.height;j++){
    for(int i = rect.x;i < rect.x + rect.width;i++){
      Point2f e_point;
      e_point.x = (float)i;
      e_point.y = (float)j;

      Point2f p_point = toPersPoint(equi, cam, e_point);
      double x = p_point.x;
      double y = p_point.y;
      if(!isInsideTriangle(tri, p_point))
	continue;
      if(x < 0.0 || y < 0.0 || x > max_x || y > max_y){
	continue;
      }else{
	warpI.at<float>(j, i) =  x;
	warpJ.at<float>(j, i) =  y;
      }
    }
  }
  
   WarpImage wi;

  wi.setLinear();  // Bi-linear interpolation
  wi.warp(cam.image, warpI, warpJ, equi);

  warpI.release();
  warpJ.release();

  return equi;
}


/* 
 *  Check if the point is inside the triangle
 *    Based on Barycentric coordinate system on Wikipedia.
 *    http://en.wikipedia.org/wiki/Barycentric_coordinate_system
 */
bool EquiTrans::isInsideTriangle(Vec6f vertices, Point2f point){
  double x = (double)point.x;
  double y = (double)point.y;

  double x1 = (double)vertices[0]; 
  double y1 = (double)vertices[1]; 
  double x2 = (double)vertices[2]; 
  double y2 = (double)vertices[3]; 
  double x3 = (double)vertices[4]; 
  double y3 = (double)vertices[5]; 

  double lambda1, lambda2, lambda3;
  double det;

  det = (y2 - y3)*(x1 - x3) + (x3 - x2)*(y1 - y3);

  bool ret = false;
  if(det != 0.0){
    lambda1 = ((y2 - y3) * (x - x3) + (x3 - x2)*(y - y3))/det; 
    lambda2 = ((y3 - y1) * (x - x3) + (x1 - x3)*(y - y3))/det; 
    lambda3 = 1.0 - lambda1 - lambda2;
    if(lambda1 >= 0.0 && lambda2 >= 0.0 && lambda3 >= 0.0){
      ret = true;
    }
  }

  return ret;
}


/*
 * 3D rotation around the tilting axis. 
 *    Y axis in Sacht's notation
 *   
 *    angle: rotational angle in radian
 *    point: 3D point
 */
Point3d EquiTrans::rotateTilt(double angle, Point3d point){
  Mat RY = (Mat_<double>(3, 3) << 
	    cos(angle), 0.0, -sin(angle),
	    0.0, 1.0, 0.0,
	    sin(angle), 0.0, cos(angle));

  Mat vec = (Mat_<double>(3,1) <<
	     point.x,
             point.y,
             point.z);

  Mat dst = RY * vec;

  Point3d r_p;
  r_p.x = dst.at<double>(0,0);
  r_p.y = dst.at<double>(1,0);
  r_p.z = dst.at<double>(2,0);

  return r_p;
}

/*
 * Check if two vector angels in 3D are over 90 degrees or not.
 */
bool EquiTrans::checkVectorAngle(Point3d a, Point3d b){
  double dot_val =  a.dot(b);
  bool ret = true;

  if(dot_val < 0.0){
    ret = false;
  }

  return ret;
}

/*
 * 3D rotation around the panning axis. 
 *    Z axis in Sacht's notation
 *   
 *    angle: rotational angle in radian
 *    point: 3D point
 */
Point3d EquiTrans::rotatePan(double angle, Point3d point){
  Mat RZ = (Mat_<double>(3, 3) << 
	    cos(angle), -sin(angle), 0.0,
	    sin(angle), cos(angle), 0.0,
	    0.0, 0.0, 1.0);

  Mat vec = (Mat_<double>(3,1) <<
	     point.x,
             point.y,
             point.z);

  Mat dst = RZ * vec;

  Point3d r_p;
  r_p.x = dst.at<double>(0,0);
  r_p.y = dst.at<double>(1,0);
  r_p.z = dst.at<double>(2,0);

  return r_p;
}
