/*
 * Triangle handling
 *   convert it from equirectangular to the perspective image
 */

#include "Triangle.hpp"


using namespace cv;

Triangle::Triangle(){
}

/*
 * convert an image in equirectangular to the perspective image, 
 *  which includes the triangle      
 *  equi_image: the ominidirectinal image in equirectangular format
 *  vertices:   the three vertices of the triangle in equi_image
 *              (origin: top-left)
 *               x: panning angle, y: tilting angle (in pixel)
 *  return:     the perpective image that includes the triagle.
 */
Mat Triangle::convToPersRectImage(Mat equi_image, Vec6f vertices){

  PersCamera cam = getPersCamParams(equi_image, vertices);

  EquiTrans trans;
  trans.toPerspective(equi_image, cam);

  return cam.image;
}

/*
 * Get the perpective camera that includes the list of triangles.
 *     Triangles could be one or a pair of matching
 */
PersCamera Triangle::getPersCamParams(Mat equi_image, Vec6f vertices){

  // horizontal field of view
  float min_x = -1.0, max_x = -1.0; 
  for(int i = 0;i < 6; i += 2){
    if(vertices[i] < min_x || min_x == -1.0){
      min_x = vertices[i];
    }

    if(vertices[i] > max_x || max_x == -1.0){
      max_x = vertices[i];
    }
  }
  double d_wd = (double)equi_image.cols;
  double d_ht = (double)equi_image.rows;
  
  double p_hfov = (double)max_x - (double)min_x; // in pixel
  double r_hfov = p_hfov /d_wd * 2.0*M_PI;       // in radian

  // vertical field of view
  float min_y = -1.0, max_y = -1.0; 
  for(int i = 0;i < 6; i += 2){
    if(vertices[i+1] < min_y || min_y == -1.0){
      min_y = vertices[i+1];
    }

    if(vertices[i+1] > max_y || max_y == -1.0){
      max_y = vertices[i+1];
    }
  }
  double p_vfov = (double)max_y - (double)min_y; // in pixel
  double r_vfov = p_vfov /d_ht * M_PI;           // in radian
  ViewDirection vd;

  double p_pan = min_x - d_wd/2.0 + 0.5 + p_hfov/2.0; //in pixel
  double p_tilt = d_ht/2.0 - min_y - 0.5 - p_vfov/2.0; //in pixel

  vd.pan = p_pan / d_wd *2.0*M_PI;
  vd.tilt = p_tilt / d_ht*M_PI;

  PersCamera cam;
  cam.setCamera(equi_image, r_hfov, r_vfov, vd);

  return cam;
}

/*
 * Convert the image coordinates of a triangle from equirectangular to perspective
 */
Vec6f Triangle::convToPersTriangle(Mat equi_image, PersCamera cam, Vec6f vertices){

  Vec6f p_vertices;
  
  EquiTrans trans;
  for(int i = 0;i < 6;i += 2){
    Point2f e_p, p_p;
    e_p.x = vertices[i];
    e_p.y = vertices[i+1];
    p_p = trans.toPersPoint(equi_image, cam, e_p);
    p_vertices[i] = p_p.x;
    p_vertices[i+1] = p_p.y;
  }

  return p_vertices;
}

/*
 * convert 6 element floating type to a set of points.
 */
vector<Point2f> Triangle::convToPointVector(Vec6f vec){
  vector<Point2f> p_vec;


  for(int i = 0;i < 6; i += 2){
    Point p;
    p.x = vec[i];
    p.y = vec[i+1];
    p_vec.push_back(p);
  }

  return p_vec;
}

/* 
 *
 */
void Triangle::showTriangle(Mat image, vector<Point2f> point){

  for(int i = 0;i < 3;i++){
    Point p1, p2;

    p1.x = (int)point[i].x;
    p1.y = (int)point[i].y;

    int k;
    if(i < 2){
      k = i + 1;
    }else{
      k = 0;
    }
    p2.x = (int)point[k].x;
    p2.y = (int)point[k].y;

    cv::line(image, p1, p2, cv::Scalar(0,255,0));
  }
  
  imshow("Triangle", image);
  waitKey(0);    
}

/*
 * Show single triangle
 */
void Triangle::showTriangle(Mat image, Vec6f vec){
  Mat dst = image;
  vector<Point2f> p_vec = convToPointVector(vec);
  showTriangle(image, p_vec);
}

/**
* This function given 2 triangles, generates a perspective image containing both triangles
*/
cv::Mat Triangle::convToPersRectImageTwo(cv::Mat equi_image, cv::Vec6f vertices1, cv::Vec6f vertices2){
	PersCamera cam = getPersCamParamsTwo(equi_image, vertices1,vertices2);

  EquiTrans trans;
  trans.toPerspective(equi_image, cam);

  return cam.image;
}

PersCamera Triangle::getPersCamParamsTwo(cv::Mat equi_image, cv::Vec6f vertices1, cv::Vec6f vertices2){
		// horizontal field of view
  
  float x_es []  = {vertices1[0],vertices1[2],vertices1[4],vertices2[0],vertices2[2],vertices2[4]};
  float y_es []  = {vertices1[1],vertices1[3],vertices1[5],vertices2[1],vertices2[3],vertices2[5]};
  
  float min_x = findMin(x_es, 6); 
  float max_x = findMax(x_es, 6);
  
  double d_wd = (double)equi_image.cols;
  double d_ht = (double)equi_image.rows;
  
  double p_hfov = (double)max_x - (double)min_x; // in pixel
  double r_hfov = p_hfov /d_wd * 2.0*M_PI;       // in radian

   //vertical field of view
  float min_y = findMin(y_es, 6);  
  float max_y = findMax(y_es, 6);  
  
  double p_vfov = (double)max_y - (double)min_y; // in pixel
  double r_vfov = p_vfov /d_ht * M_PI;           // in radian
  ViewDirection vd;

  double p_pan = min_x - d_wd/2.0 + 0.5 + p_hfov/2.0; //in pixel
  double p_tilt = d_ht/2.0 - min_y - 0.5 - p_vfov/2.0; //in pixel

  vd.pan = p_pan / d_wd *2.0*M_PI;
  vd.tilt = p_tilt / d_ht*M_PI;

  PersCamera cam;
  cam.setCamera(equi_image, r_hfov, r_vfov, vd);

  return cam;  
}
