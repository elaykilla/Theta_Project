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

/** 
* 
*/
cv::Mat Triangle::convToPersRectImageBarycentric(cv::Mat equi_image, Vec9f vertices3D){
 	//cout << "convToPersRectImageBarycentric: Getting Perspective Camera" << endl;
 	PersCamera  cam = getPersCamParamsBarycentric(equi_image, vertices3D);
	//cout <<"convToPersRectImageBarycentric: PersCamera (hfov,vfov) = (" << cam.fov_h << "," << cam.fov_v << " )" << endl;  	
	
 	EquiTrans trans;
 	//cout << "convToPersRectImageBarycentric: Converting using perspective cam" << endl;
 	trans.toPerspective(equi_image, cam);
 	
 	return cam.image;
 }

/**
* 
*/
cv::Mat Triangle::convToPersRectImageBarycentricTwo(cv::Mat equi_image, Vec9f triangle3D1, Vec9f triangle3D2){
  	PersCamera  cam = getPersCamParamsBarycentricTwo(equi_image, triangle3D1,triangle3D2);
  	
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
  cout << "getPersCamParams: Viewing Direction: (" << vd.pan << "," << vd.tilt << ")" << endl;

  PersCamera cam;
  cout<< "getPersCamParams: PersCamera (hfov,vfov) = (" << r_hfov << "," << r_vfov << ")" << endl;
  cam.setCamera(equi_image, r_hfov, r_vfov, vd);

  return cam;
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
   /*
 * Get the perpective camera that includes a triangle by using the barycenter as point and the 
 * radius of the circumcircle
 */
  PersCamera Triangle::getPersCamParamsBarycentric(cv::Mat equi_image, Vec9f vertices){
  	//First convert the triangle to get spheric coordinates of vertices
  	cv::Vec6f triangle = triangle2SphericSacht(equi_image.rows, equi_image.cols,1,vertices);
  	//Get the 3 Vertices as points 
  	Point2f p1,p2,p3;
  	
  	//First vertice
  	p1.x = triangle[0];
  	p1.y = triangle[1];
  	
  	//Second vertice
  	p2.x = triangle[2];
  	p2.y = triangle[3];
  	
  	//Theturn cam;ird vertice
  	p3.x = triangle[4];
  	p3.y = triangle[5];
  	
  	
  	// Horizontal
  	// Get the maximum angular difference between the points in terms of theta (horizontal)
  	// in order to make correct viewing direction we have to make sure that the points are all
  	// in correct order and interval. We take the first point as anchor and cheick the angular 		// distance with the other points
  	double thetad1_2, thetad1_3, thetad2_3, max_thetad;
  	thetad1_2 = p2.x - p1.x;
  	thetad1_3 = p3.x - p1.x;
  	thetad2_3 = p3.x - p2.x;
  	max_thetad = max(thetad1_2, max(thetad1_3,thetad2_3));
  	
  	//If the angle is larger than PI then we have to shift the point to the other side
  	// depending on the sign of p1, we either add or remove 2PI
  	if(abs(thetad1_2) > M_PI) {
  		p2.x += (p1.x/abs(p1.x))*2*M_PI;
  		triangle[2] = p2.x;
  	}
  	if(abs(thetad1_3) > M_PI){
  		p3.x += (p1.x/abs(p1.x))*2*M_PI;
  		triangle[4] = p3.x;
  	}
  	
  	
  	//Get the angular phi difference between points
  	double phid1_2, phid1_3, phid2_3, max_phid;
  	phid1_2 = p2.y - p1.y;
  	phid1_3 = p3.y - p1.y;
  	phid2_3 = p3.y - p2.y;
  	max_phid = max(phid1_2, max(phid1_3,phid2_3));
  	
  	//Get the barycenter
  	cv::Point2f center = triangleCenter(triangle);
  	
  	//Get pan and tilt angles by finding max angle distance
  	double v_angle, h_angle;
  	h_angle = max_phid;
  	v_angle = max_thetad;
  	
  	//Set Viewing direction as the barycenter of the triangle
  	ViewDirection vd;
  	vd.pan = center.y;
  	vd.tilt = center.x;
  	//vd.pan = 0.164793;
  	//vd.tilt = -0.276993;
  	cout << "getPersCamParamsBarycentric: Viewing Direction: (" << vd.pan << "," << vd.tilt << ")" << endl;
  	//Finally make the Perspective camera
  	PersCamera cam;
  	cam.setCamera(equi_image, h_angle, v_angle, vd);

  	return cam;
  	
  }
  
/*
* By using the associative property of the barycenter, we can get the barycenter of 2 triangles as the barrycenter of the barycenters of each triangle;
*/
  PersCamera Triangle::getPersCamParamsBarycentricTwo(cv::Mat equi_image, Vec9f triangle3D1, Vec9f triangle3D2){
  	//First convert the triangle to get spheric coordinates of vertices
  	cv::Vec6f triangle1 = triangle2SphericSacht(equi_image.rows, equi_image.cols,1,triangle3D1);
  	cv::Vec6f triangle2 = triangle2SphericSacht(equi_image.rows, equi_image.cols,1,triangle3D2);
  	//Get the 3 Vertices as points 
  	Point2f p1,p2,p3,q1,q2,q3;
  	//Centers for each triangle
  	Point2f center, center1, center2;
  	
  	
  	//First vertice
  	p1.x = triangle1[0];
  	p1.y = triangle1[1];
  	q1.x = triangle2[0];
  	q1.y = triangle2[1];
  	
  	//Second vertice
  	p2.x = triangle1[2];
  	p2.y = triangle1[3];
  	q2.x = triangle2[2];
  	q2.y = triangle2[3];
  	
  	//Third vertice
  	p3.x = triangle1[4];
  	p3.y = triangle1[5];
  	q3.x = triangle2[4];
  	q3.y = triangle2[5];
  	
  	/********************************************************************************/
  	//Horizontal 
  	/*******************************************************************************/
  	
  	// Get the maximum angular difference between the points in terms of theta (horizontal)
  	// in order to make correct viewing direction we have to make sure that the points are all
  	// in correct order and interval. We take the first point as anchor and cheick the angular 		// distance with the other points
  	double t1_thetad1_2, t1_thetad1_3, t1_thetad2_3, t2_thetad1_2, t2_thetad1_3, t2_thetad2_3, t1_max_thetad, t2_max_thetad;
  	t1_thetad1_2 = p2.x - p1.x;
  	t2_thetad1_2 = q2.x - q1.x;
  	
  	t1_thetad1_3 = p3.x - p1.x;
  	t2_thetad1_2 = q2.x - q1.x;
  	
  	t1_thetad2_3 = p3.x - p2.x;
  	t2_thetad2_3 = q3.x - q2.x;
  	
  	t1_max_thetad = max(t1_thetad1_2, max(t1_thetad1_3,t1_thetad2_3));
  	t2_max_thetad = max(t2_thetad1_2, max(t2_thetad1_3,t2_thetad2_3));
  	
  	//Make sure there are no loopings in the point angle calculations
  	//If the angle is larger than PI then we have to shift the point to the other side
  	// depending on the sign of p1, we either add or remove 2PI
  	//For first Triangle
  	if(abs(t1_thetad1_2) > M_PI) {
  		p2.x += (p1.x/abs(p1.x))*2*M_PI;
  		triangle1[2] = p2.x;
  	}
  	if(abs(t1_thetad1_3) > M_PI){
  		p3.x += (p1.x/abs(p1.x))*2*M_PI;
  		triangle1[4] = p3.x;
  	}
  	
  	//For Second Triangle
  	if(abs(t2_thetad1_2) > M_PI) {
  		p2.x += (p1.x/abs(p1.x))*2*M_PI;
  		triangle2[2] = q2.x;
  	}
  	if(abs(t2_thetad1_3) > M_PI){
  		p3.x += (p1.x/abs(p1.x))*2*M_PI;
  		triangle2[4] = q3.x;
  	}
  	
  	
  	
  	/***************************************************************************************/
  	//Vertical
  	/**************************************************************************************/
  	//Get the angular phi difference between points
  	double t1_phid1_2, t1_phid1_3, t1_phid2_3, t2_phid1_2, t2_phid1_3, t2_phid2_3, t1_max_phid,t2_max_phid;
  	//Triangle1
  	t1_phid1_2 = p2.y - p1.y;
  	t1_phid1_3 = p3.y - p1.y;
  	t1_phid2_3 = p3.y - p2.y;
  	t1_max_phid = max(t1_phid1_2, max(t1_phid1_3,t1_phid2_3));
  	
  	//Triangle2
  	t2_phid1_2 = p2.y - p1.y;
  	t2_phid1_3 = p3.y - p1.y;
  	t2_phid2_3 = p3.y - p2.y;
  	t2_max_phid = max(t2_phid1_2, max(t2_phid1_3,t2_phid2_3));
  	
  	
  	
  	
  	//Get barycenters for each triangle
  	center1 = triangleCenter(triangle1);
  	center2 = triangleCenter(triangle2);
  	
  	//Get the barycenter of the centers
  	center.x = (center1.x + center2.x)/2;
  	center.y = (center1.y + center2.y)/2;
  	
  	//Set Viewing direction as the barycenter of the triangles
  	ViewDirection vd;
  	vd.pan = center.y;
  	vd.tilt = center.x;
  	
  	//Calculate the field of view as the difference between the barycenters to which we add the radius 
  	// of the circumcircles
  	double bary_theta = abs(center1.x - center2.x);
  	double bary_phi = abs(center2.y - center2.y);
  	
  	//Get pan and tilt angles by finding max angle distance and adding to barycenter difference
  	double v_angle, h_angle;
  	h_angle = bary_phi + t2_max_phid + t2_max_phid;
  	v_angle = bary_theta + t2_max_thetad + t2_max_thetad;
  	
  	//Finally make the Perspective camera
  	PersCamera cam;
  	cam.setCamera(equi_image, h_angle, v_angle, vd);

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


