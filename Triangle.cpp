/*
 * Triangle handling
 *   convert it from equirectangular to the perspective image
 */

#include "Triangle.hpp"


//using namespace cv;

Triangle::Triangle(){
  m_MAX_ANGLE = triangle_max_angle;
}

/*
 * find a minimum value.
 */
double Triangle::findMin(float numbers[], int size){
	float min = FLT_MAX;
	for(int i=0;i<size;i++){
		if(min>numbers[i]){
			min = numbers[i];
		}
	}
	return min;
}

/*
 * find the maximum value.
 */
double Triangle::findMax(float numbers[], int size){
	float max = -FLT_MAX;
	for(int i=0;i<size;i++){
		if(max<numbers[i]){
			max = numbers[i];
		}
	}
	return max;
}

/**
 * This function returns the center of a 2D triangle defined by 3 vertices in (x,y) coordinates 
 * or angular values (theta,phi)
 */
cv::Point2f Triangle::triangleCenter(cv::Vec6f triangle){
	cv::Point2f p;
	float x = (triangle[0] + triangle[2] + triangle[4])/3;
	float y = (triangle[1] + triangle[3] + triangle[5])/3;
	p.x = x;
	p.y = y;
	
	return p;

}
/*
 *  Normalize a vector to a unit vector
 */
cv::Point3d normalizePoint(cv::Point3d p){

  double scale = 1.0/sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
  Point3d np;

  np.x = p.x * scale;
  np.y = p.y * scale;
  np.z = p.z * scale;

  return np;
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
 * Convert a triangle in Vec9f to Point3d format.
 */
vector<Point3d> Triangle::convVec9fToPoint3d(Vec9f vertices){
  size_t size = 3;
  vector<cv::Point3d> points(size);

  int j = 0;
  for(int i = 0;i < 9;i += 3, j++){
    points[j].x = (double)vertices[i];
    points[j].y = (double)vertices[i+1];
    points[j].z = (double)vertices[i+2];
  }

  return points;
}

/*
 * Covnert a trinagle in Point3d list to Vec9f
 */
Vec9f Triangle::convPoint3dToVec9f(vector<cv::Point3d> points){
  size_t size = 3;
  Vec9f vertices;

  int j = 0;
  for(int i = 0;i < 9;i += 3, j++){
    vertices[i  ] = (float) points[j].x;
    vertices[i+1] = (float)points[j].y;
    vertices[i+2] = (float)points[j].z;
  }

  return vertices;
}

/*
 * vector angle
 */
double getVectorAngle(Point3d a, Point3d b){
  // cos(theta) = a * b/|a||b|;

  double len_a = sqrt(a.x *a.x + a.y*a.y + a.z*a.z);
  double len_b = sqrt(b.x *b.x + b.y*b.y + b.z*b.z);
  double theta = acos(a.dot(b)/(len_a * len_b));

  return theta;
}

/*
 *  Obtain the field of the view in the perspective camera
 *   points: vertices in 3D unit sphere
 *   vd:     viewing direction of the camera pointing to the center of the vertices.
 *   
 *   h_fov, v_fov: field of view
 *   return: adjuested new viewing direction.
 */
ViewDirection
Triangle::getFOV(vector<Point3d> points, ViewDirection vd, Point3d center, double &h_fov, double &v_fov){

  // measure panning by vector angle.
  bool debug_flag = true;

  double max_theta = getMaxAngle(points);

  if(debug_flag){
    if(max_theta > m_MAX_ANGLE){
      double theta_deg = max_theta/M_PI*180.0;
      cout << "Triangle is too big: " << theta_deg << " degrees.\n";
    }
  }


  h_fov = max_theta * 3.0;
  v_fov = max_theta * 3.0;
}

void Triangle::nPointsFOV(vector<Point3d> points, ViewDirection vd, Point3d center, double &h_fov, double &v_fov){

  // measure panning by vector angle.
  bool debug_flag = true;

  double max_theta = nPointsMaxAngle(points);

  if(debug_flag){
    if(max_theta > m_MAX_ANGLE){
      double theta_deg = max_theta/M_PI*180.0;
      cout << "Triangle is too big: " << theta_deg << " degrees.\n";
    }
  }


  h_fov = max_theta * 3.0;
  v_fov = max_theta * 3.0;
}



/*
 * Adjust the camera to include all of the vertices of the triangle.
 *    equi_image: equirectangular image
 *    cam: initial camera
 *    vertices: 
 */
PersCamera Triangle::adjustPersCam(Mat equi_image, PersCamera cam, vector<Point3d> vertices){
  // coordinate on the perspective camera
  bool debug_flag = true;

  EquiTrans tran;
  double r_max = (double)(cam.image.rows - 1);
  double c_max = (double)(cam.image.cols - 1);
  double min_x = 0.0, max_x = c_max;
  double min_y = 0.0, max_y = r_max;

  if(vertices.size() == 3){
    for(int i = 0;i < 3;i++){
      double x = -1.0, y = -1.0;
      Point3f w_d;
      w_d.x = vertices[i].x;
      w_d.y = vertices[i].y;
      w_d.z = vertices[i].z;
      
      tran.convSpherePointToEquiCoord(w_d, equi_image, &x, &y);
      Point2f e_p;
      e_p.x = (float)x;
      e_p.y = (float)y;
      Point2f point = tran.toPersPoint(equi_image, cam, e_p);
      //      Point2d point = tran.convSpherePointtoPersCoord(cam, vertices[i]);

      if(point.x < min_x) {
	min_x = point.x;
      }
      if(point.x > max_x) {
	max_x = point.x;
      }
      if(point.y < min_y) {
	min_y = point.y;
      }
      if(point.y > max_y) {
	max_y = point.y;
      }
    }
  }

  // outside the image border.
  cout << "min_x " << min_x;
  cout << "max_x " << max_x;
  cout << "min_y " << min_y;
  cout << "max_y " << max_y;

  double under_x = 0.0, over_x = 0.0;
  double under_y = 0.0, over_y = 0.0;
  if(min_x < 0.0) under_x = - min_x;
  if(max_x > c_max) over_x = max_x - c_max;

  if(min_y < 0.0) under_y = - min_y;
  if(max_y > r_max) over_y = max_y - r_max;

  double offset_x = over_x - under_x;
  double offset_y = over_y - under_y;

  double half_width = ((double)cam.image.cols + under_x +  over_x)/2.0;
  double h_fov = atan(half_width/cam.focal_length) * 2.0;

  double half_height = ((double)cam.image.rows + under_y +  over_y)/2.0;
  double v_fov = atan(half_height/cam.focal_length) * 2.0;

  double ec_i, ec_j;
  tran.toEquirectCore(offset_x, offset_y, cam.focal_length, cam.view_dir, equi_image.rows, equi_image.cols, &ec_i, &ec_j);

  PersCamera n_cam;
  ViewDirection vd;

  /*
  vd.pan = ec_i/(double)equi_image.cols * 2.0 * M_PI;
  vd.tilt =  - ec_j/(double)equi_image.rows * M_PI;
  */
  vd = cam.view_dir;

  h_fov = 0.5 * M_PI;
  v_fov = 0.5 * M_PI;
  n_cam.setCamera(equi_image, h_fov, v_fov, vd);

  return n_cam;
}

/*
 * Get the perpective camera that includes the triangle in 3D.
 *
 *    equi_image: original equirectangular image
 *    vertices: 3D coordinates on unit sphere
 *   
 *    coordinate system: x: depth, y:left-right, z:bottom-top
 */
PersCamera Triangle::getPersCamParams(Mat equi_image, Vec9f vertices){
  // store in Point3d;
  vector<Point3d> points = convVec9fToPoint3d(vertices);

  // get the center of vertices in 3D.
  Point3d center = triangleCenter3d(points);
  center = normalizePoint(center);
  EquiTrans tran;
  double pan = 0.0, tilt = 0.0;
  tran.convSpherePointToAngles(center, &pan, &tilt);

  ViewDirection vd;
  vd.pan = pan;
  vd.tilt = tilt;

  // Obtained the initial field of view.
  double h_fov = -1.0, v_fov = -1.0;
  getFOV(points, vd, center, h_fov, v_fov);


  // set the intial camera.
  PersCamera cam;
  cam.setCamera(equi_image, h_fov, v_fov, vd);

  // adjust the camera.
  //PersCamera n_cam = adjustPersCam(equi_image, cam, points);

  return cam;
}

PersCamera Triangle::getPersCamParamsTwo(Mat equi_image, Vec9f vertices1, Vec9f vertices2){
	//Get 3d points
	vector<Point3d> points1 = convVec9fToPoint3d(vertices1);
	vector<Point3d> points2 = convVec9fToPoint3d(vertices2);
	
	//Put the points together
	vector<Point3d> points;
	points.insert(points.end(),points1.begin(),points1.end());
	points.insert(points.end(),points2.begin(),points2.end());
	
	// get the center of vertices in 3D.
  	Point3d center = nPointsCenter3d(points);
  	center = normalizePoint(center);
  	EquiTrans tran;
  	double pan = 0.0, tilt = 0.0;
  	tran.convSpherePointToAngles(center, &pan, &tilt);
  	ViewDirection vd;
  	vd.pan = pan;
  	vd.tilt = tilt;
  	
  	
  	// Obtained the initial field of view.
  	double h_fov = -1.0, v_fov = -1.0;
  	nPointsFOV(points, vd, center, h_fov, v_fov);
  	
  	// set the intial camera.
  	PersCamera cam;
  	cam.setCamera(equi_image, h_fov, v_fov, vd);
  
  	return cam;
	

}

/*
 * get the center coordinates of triangle vertices in 3D.
 */
cv::Point3d Triangle::triangleCenter3d(vector<cv::Point3d> vertices){
  cv::Point3d p;
  double xsum = 0.0, ysum = 0.0, zsum = 0.0;

  if(vertices.size() == 3){
    for(int i = 0;i < 3;i++){
      xsum += vertices[i].x;
      ysum += vertices[i].y;
      zsum += vertices[i].z;
    }
	
    p.x = xsum/3.0;
    p.y = ysum/3.0;
    p.z = zsum/3.0;
  }

  return p;
}

/**
* Get barrycenter for n points in 3d space
*/
cv::Point3d Triangle::nPointsCenter3d(vector<cv::Point3d> vertices){
	cv::Point3d p;
	double xsum = 0.0, ysum = 0.0, zsum = 0.0;
	
	for(int i = 0;i < vertices.size();i++){
      		xsum += vertices[i].x;
      		ysum += vertices[i].y;
      		zsum += vertices[i].z;
    	}
    	
    	p.x = xsum/vertices.size();
    	p.y = ysum/vertices.size();
    	p.z = zsum/vertices.size();
    	
    	return p;

}

/*
 * Get the perpective camera that includes the list of triangles.
 *     Triangles could be one or a pair of matching
 *
 *    equi_image: original equirectangular image
 *    vertices: image coordinates
 *      THIS IS TRICKY.
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
  if(r_hfov!=r_hfov || r_vfov!=r_vfov){
  	r_hfov = r_vfov = 0;
  }
  else{
  	cam.setCamera(equi_image, r_hfov, r_vfov, vd);
  }
  
  //cam.setCamera(equi_image, r_hfov, r_vfov, vd);

  return cam;
}
   /*
 * Get the perpective camera that includes a triangle by using the barycenter as point and the 
 * radius of the circumcircle
 */
  PersCamera Triangle::getPersCamParamsBarycentric(cv::Mat equi_image, Vec9f vertices){
  
  	EquiTrans equi;
  	//First convert the triangle to get spheric coordinates of vertices
  	//cv::Vec6f triangle = triangle2SphericSacht(equi_image.rows, equi_image.cols,1,vertices);
  	cv::Vec6f triangle = convToSphericTriangle(vertices);
  	//Get the 3 Vertices as points in 3D then convert to angular position
  	//Point3f p3d1,p3d2,p3d3;
  	
  	
  	Point2f p1,p2,p3;
  	

  	
  	//First vertice
  	//p3d1.x = triangle[0];
  	//p3d1.y = triangle[1];
  	//p3d1.z = triangle[2];
  	
  	//Second vertice
  	//p3d2.x = triangle[3];
  	//p3d2.y = triangle[4];
  	//p3d2.z = triangle[5];
  	
  	//Theturn cam;ird vertice
  	//p3d3.x = triangle[6];
  	//p3d3.y = triangle[7];
  	//p3d3.z = triangle[8];
  	
  	//double x=0.0,y=0.0;
  	//equi.convSpherePointToAngles(p3d1, &x,&y);
  	
  	///equi.convSpherePointToAngles(p3d2, &x,&y);
  	//p2.x = x;
  	//p2.y = y;
  	
  	//equi.convSpherePointToAngles(p3d3, &x,&y);
  	//p3.x = x;
  	//p3.y = y;
  	
  	p1.x = triangle[0];
  	p1.y = triangle[1];
  	p2.x = triangle[2];
  	p2.y = triangle[3];
  	p3.x = triangle[4];
  	p3.y = triangle[5];
  	
  	// Horizontal
  	// Get the maximum angular difference between the points in terms of theta (horizontal)
  	// in order to make correct viewing direction we have to make sure that the points are all
  	// in correct order and interval. We take the first point as anchor and cheick the angular 		// distance with the other points
  	double thetad1_2, thetad1_3, thetad2_3, max_thetad;
  	thetad1_2 = abs(p2.x - p1.x);
  	thetad1_3 = abs(p3.x - p1.x);
  	thetad2_3 = abs(p3.x - p2.x);
  	max_thetad = max(thetad1_2, max(thetad1_3,thetad2_3));
  	
  	
  	
  	
  	//Get the angular phi difference between points
  	double phid1_2, phid1_3, phid2_3, max_phid;
  	phid1_2 = abs(p2.y - p1.y);
  	phid1_3 = abs(p3.y - p1.y);
  	phid2_3 = abs(p3.y - p2.y);
  	//max_phid = max(phid1_2, max(phid1_3,phid2_3));
  	
  	//If the angle is larger than PI then we have to shift the point to the other side
  	// depending on the sign of p1, we either add or remove 2PI
  	if(abs(thetad1_2) > M_PI) {
  		p2.y += (p1.y/abs(p1.y))*2*M_PI;
  		triangle[3] = p2.y;
  	}
  	if(abs(thetad1_3) > M_PI){
  		p3.y += (p1.y/abs(p1.y))*2*M_PI;
  		triangle[5] = p3.y;
  	}
  	
  	//Recalculate the angular phi difference between points
  	phid1_2 = abs(p2.y - p1.y);
  	phid1_3 = abs(p3.y - p1.y);
  	phid2_3 = abs(p3.y - p2.y);
  	max_phid = max(phid1_2, max(phid1_3,phid2_3));
  	
  	//Get the barycenter
  	cv::Point2f center = triangleCenter(triangle);
  	
  	//Get pan and tilt angles by finding max angle distance
  	double v_angle, h_angle;
  	h_angle = max_phid;
  	v_angle = max_thetad;
  	cout << "getPersCamParamsBarycentric: Cam image: (" << h_angle << "," << v_angle << ")" << endl;
  	//Set Viewing direction as the barycenter of the triangle
  	ViewDirection vd;
  	vd.pan = center.y;
  	vd.tilt = center.x;
  	//vd.pan = 0.164793;
  	//vd.tilt = -0.276993;
  	cout << "getPersCamParamsBarycentric: Viewing Direction: (" << vd.pan << "," << vd.tilt << ")" << endl;
  	//Finally make the Perspective camera
  	PersCamera cam;
  	
  	if(h_angle!=h_angle || v_angle!=v_angle){
  		h_angle = v_angle = 0;
  	}
  	else{
  		cam.setCamera(equi_image, h_angle, v_angle, vd);
	}
  	return cam;
  	
  }
  
/*
* By using the associative property of the barycenter, we can get the barycenter of 2 triangles as the barrycenter of the barycenters of each triangle;
*/
  PersCamera Triangle::getPersCamParamsBarycentricTwo(cv::Mat equi_image, Vec9f triangle3D1, Vec9f triangle3D2){
  
  EquiTrans equi;
  	//First convert the triangle to get spheric coordinates of vertices
  	
  	//cv::Vec6f triangle1 = triangle2SphericSacht(equi_image.rows, equi_image.cols,1,triangle3D1);
  	cv::Vec6f triangle1 = convToSphericTriangle(triangle3D1);
  	
  	//cv::Vec6f triangle2 = triangle2SphericSacht(equi_image.rows, equi_image.cols,1,triangle3D2);
  	cv::Vec6f triangle2 = convToSphericTriangle(triangle3D2);
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
  	t1_thetad1_2 = abs(p2.x - p1.x);
  	t2_thetad1_2 = abs(q2.x - q1.x);
  	
  	t1_thetad1_3 = abs(p3.x - p1.x);
  	t2_thetad1_2 = abs(q2.x - q1.x);
  	
  	t1_thetad2_3 = abs(p3.x - p2.x);
  	t2_thetad2_3 = abs(q3.x - q2.x);
  	
  	t1_max_thetad = max(t1_thetad1_2, max(t1_thetad1_3,t1_thetad2_3));
  	t2_max_thetad = max(t2_thetad1_2, max(t2_thetad1_3,t2_thetad2_3));
  	
  	
  	
  	
  	
  	/***************************************************************************************/
  	//Vertical
  	/**************************************************************************************/
  	//Get the angular phi difference between points
  	double t1_phid1_2, t1_phid1_3, t1_phid2_3, t2_phid1_2, t2_phid1_3, t2_phid2_3, t1_max_phid,t2_max_phid;
  	//Triangle1
  	t1_phid1_2 = abs(p2.y - p1.y);
  	t1_phid1_3 = abs(p3.y - p1.y);
  	t1_phid2_3 = abs(p3.y - p2.y);
  	//t1_max_phid = max(t1_phid1_2, max(t1_phid1_3,t1_phid2_3));
  	
  	//Triangle2
  	t2_phid1_2 = abs(p2.y - p1.y);
  	t2_phid1_3 = abs(p3.y - p1.y);
  	t2_phid2_3 = abs(p3.y - p2.y);
  	//t2_max_phid = max(t2_phid1_2, max(t2_phid1_3,t2_phid2_3));
  	
  	//Make sure there are no loopings in the point angle calculations
  	//If the angle is larger than PI then we have to shift the point to the other side
  	// depending on the sign of p1, we either add or remove 2PI
  	//For first Triangle
  	if(abs(t1_thetad1_2) > M_PI) {
  		p2.y += (p1.y/abs(p1.y))*2*M_PI;
  		triangle1[3] = p2.y;
  	}
  	if(abs(t1_thetad1_3) > M_PI){
  		p3.y += (p1.y/abs(p1.y))*2*M_PI;
  		triangle1[5] = p3.y;
  	}
  	
  	//For Second Triangle
  	if(abs(t2_thetad1_2) > M_PI) {
  		q2.y += (q1.y/abs(q1.y))*2*M_PI;
  		triangle2[3] = q2.y;
  	}
  	if(abs(t2_thetad1_3) > M_PI){
  		q3.y += (q1.y/abs(q1.y))*2*M_PI;
  		triangle2[5] = q3.y;
  	}
  	//Recalculate the new maxes and mins
  	t1_phid1_2 = abs(p2.y - p1.y);
  	t1_phid1_3 = abs(p3.y - p1.y);
  	t1_phid2_3 = abs(p3.y - p2.y);
  	t1_max_phid = max(t1_phid1_2, max(t1_phid1_3,t1_phid2_3));
  	
  	//Triangle2
  	t2_phid1_2 = abs(p2.y - p1.y);
  	t2_phid1_3 = abs(p3.y - p1.y);
  	t2_phid2_3 = abs(p3.y - p2.y);
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
  	h_angle = bary_phi + t1_max_phid + t2_max_phid;
  	v_angle = bary_theta + t1_max_thetad + t2_max_thetad;
  	
  	//Finally make the Perspective camera
  	PersCamera cam;
  	cam.setCamera(equi_image, h_angle, v_angle, vd);

  	return cam;
  }
  
/*
 * Convert 3D points on the unit sphere in Vec9f to those in equirectanuglar
 * in Vec6f.
 *  
 */ 
Vec6f Triangle::convVec9fToVec6f(Mat equi_image, Vec9f vec9f){
  Vec6f vec6f;
  vector<Point3f> p3f_list(3);

  int k = 0;
  for(int i = 0;i < 3;i++){

    p3f_list[i].x = vec9f[k++];
    p3f_list[i].y = vec9f[k++];
    p3f_list[i].z = vec9f[k++];
  }

  k = 0;
  for(int i = 0;i < 3;i++){
    EquiTrans trans;
    Point3f p3f = p3f_list[i];
    double x = -1.0, y = -1.0;
    trans.convSpherePointToEquiCoord(p3f, equi_image, &x, &y);
    vec6f[k++] = (float)x;
    vec6f[k++] = (float)y;
  }


  return vec6f;
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

  /**
  * convert a triangle from coordinates given in 3D cartesian to spheric angular coordinates
  */
  cv::Vec6f Triangle::convToSphericTriangle(Vec9f triangle){
  	EquiTrans equi;
  
  	//Get the 3 points that make the triangle
  	Point3f p3d1,p3d2,p3d3;
  	
  	//Resulting triangle
  	cv::Vec6f striangle;
  	
  	//First vertice
  	p3d1.x = triangle[0];
  	p3d1.y = triangle[1];
  	p3d1.z = triangle[2];
  	
  	//Second vertice
  	p3d2.x = triangle[3];
  	p3d2.y = triangle[4];
  	p3d2.z = triangle[5];
  	
  	//Theturn cam;ird vertice
  	p3d3.x = triangle[6];
  	p3d3.y = triangle[7];
  	p3d3.z = triangle[8];
  	
  	double x=0.0,y=0.0;
  	equi.convSpherePointToAngles(p3d1, &x,&y);
  	striangle[0] = y;
  	striangle[1] = x;
  	
  	equi.convSpherePointToAngles(p3d2, &x,&y);
  	striangle[2] = y;
  	striangle[3] = x;
  	
  	equi.convSpherePointToAngles(p3d3, &x,&y);
  	striangle[4] = y;
  	striangle[5] = x;
  	
  	return striangle;
  }

 /** 
  * This function returns the intermediate camera parameters given 
  * - 2 perspective cameras
  * - the desired interpolated position defined by pos and dist = pos/dist 
  */
  PersCamera Triangle::getInterpolatedPersCamParams(PersCamera cam1, PersCamera cam2, double dist, double pos){
  	//Get actual position
  	double act_pos = pos/dist;
  	PersCamera cam;
  	ViewDirection vd;
  	double v_angle, h_angle;
  	
  	
  	//Calculate the cam parameters
  	vd.pan = cam1.view_dir.pan * (1-act_pos) + cam2.view_dir.pan*act_pos;
  	vd.tilt = cam1.view_dir.tilt * (1-act_pos) + cam2.view_dir.tilt*act_pos;
  	v_angle = cam1.fov_v * (1-act_pos) + cam2.fov_h*act_pos;
  	h_angle = cam1.fov_h * (1-act_pos) + cam2.fov_v*act_pos;
  	
  	//Set Image size
  	int rows = cam1.image.rows* (1-act_pos) + cam2.image.rows* act_pos;
  	int cols = cam1.image.cols* (1-act_pos) + cam2.image.cols* act_pos;
  	cv::Mat image_inter (rows,cols, cam1.image.type());
  	
  	cam.setCamera(image_inter, h_angle, v_angle, vd);
  	
  	return cam;
  	
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
 * Draw a triangle (Vec6f)
 */
void Triangle::drawTriangle(Mat image, cv::Vec6f vertices){
  vector<Point2f> p_vec = convToPointVector(vertices);  
  drawTriangle(image, p_vec);
}

/*
 * Draw a triangle (Point2f)
 */
void Triangle::drawTriangle(Mat image, vector<Point2f> point){

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

}

/* 
 * Show a triangle on the image
 */
void Triangle::showTriangle(Mat image, vector<Point2f> point){

  drawTriangle(image, point);
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

/*
 *
 */
cv::Vec6f Triangle::triangle2SphericSacht(int rows, int cols, double r, Vec9f triangle){
	//Final triangle
	cv::Vec6f new_triangle;

	//Variables to get panning and tilting angles
	double theta, phi;
	Point3d p;

	for(int i=0, j=0;i<9;i=i+3,j=j+2){
		p.x=triangle[i];
		p.y=triangle[i+1];
		p.z=triangle[i+2];

		//Convert to Spheric using Sacht notations
		cartesian2SphericSacht(p,r,theta,phi);
		new_triangle[j] = theta;
		new_triangle[j+1] = phi;
		//j=j+2;
	}

	return new_triangle;
}

/** 
 * Using Sacht's Notations
 */
void Triangle::cartesian2SphericSacht(Point3d p, double r, double &theta, double &phi){
	r = sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
	phi = asin(p.z);
	//theta = asin(p.y/(p.x*p.x + p.y*p.y));
	//theta = acos(p.x/(p.x*p.x + p.y*p.y));
	theta = atan2(p.y,p.x);
	//	
	//if(phi>PI) phi -= 2*PI;
	//if (theta>PI) theta -= 2*PI;
}

/* 
 * Get the mid point between Point a and Point b in 3D.
 *     Normaalized to be unit vector (on the unit sphere)
 */
cv::Point3d Triangle::midPoint(cv::Point3d a, cv::Point3d b){
  Point3d mid;

  mid.x = (a.x + b.x)/2.0;
  mid.y = (a.y + b.y)/2.0;
  mid.z = (a.z + b.z)/2.0;

  mid = normalizePoint(mid);

  return mid;
}

/*
 * Subdivide a list of triangles for a few times, if some of them are big.
 *     tri_list: triangle list (mostly one triangle)
 *            n: number of times of iterration
 */
vector<Vec9f>
Triangle::subdivide(vector<Vec9f> tri_list){

  int n = 4;
  vector<Vec9f> n_list;

  for(int k = 0;k < n;k++){ // repeat n times.
    bool no_sub = true;
    n_list.clear();
    for(vector<Vec9f>::iterator it  = tri_list.begin();it !=tri_list.end();++it){
      Vec9f vec = *it;
      vector<Vec9f> sub_list = subdivide(vec);
      if(sub_list.size() > 1){
	for(vector<Vec9f>::iterator s_it = sub_list.begin();s_it != sub_list.end();++s_it){
	  n_list.push_back(*s_it);
	}
	no_sub = false;
      }else{
	n_list.push_back(vec);
      }
    }
    if(no_sub){ // finish if there is no subdivion in the loop.
      break;
    }
    tri_list = n_list;
  }

  return tri_list;
}


/*
 * Subdivide one triangle to four triangles when it has angles to the center are larger than 45.0 dereees.
 */
vector<Vec9f>
Triangle::subdivide(Vec9f vertices){
  vector<cv::Point3d> points = convVec9fToPoint3d(vertices);
  vector< vector<cv::Point3d> >  tri_list = subdivide(points);
  vector<Vec9f> vec_list;

  // convert Point3d list to Vec9f list
  for(int i = 0;i < tri_list.size();i++){
    points = tri_list[i];
    Vec9f vec = convPoint3dToVec9f(points);
    vec_list.push_back(vec);
  }

  return vec_list;
}

/*
 * Subdivide one triangle to four triangles when it has angles to the center are larger than 45.0 dereees.
 */
vector< vector<cv::Point3d> >
Triangle::subdivide(vector<cv::Point3d> vertices){
  double max_angle = getMaxAngle(vertices);

  vector < vector<cv::Point3d> > triangle_list;
  cv::Point3d p01, p12, p02;
  if(max_angle > m_MAX_ANGLE){
    if(vertices.size() == 3){
      p01 = midPoint(vertices[0], vertices[1]);
      p12 = midPoint(vertices[1], vertices[2]);
      p02 = midPoint(vertices[0], vertices[2]);
    }

    // Lower left sub-triangle
    vector<cv::Point3d> tri0;
    tri0.push_back(vertices[0]);
    tri0.push_back(p01);
    tri0.push_back(p02);
    triangle_list.push_back(tri0);

    // top sub-triangle
    vector<cv::Point3d> tri1;
    tri1.push_back(p01);
    tri1.push_back(vertices[1]);
    tri1.push_back(p12);
    triangle_list.push_back(tri1);

    // Lower right sub-triangle
    vector<cv::Point3d> tri2;
    tri2.push_back(p02);
    tri2.push_back(p12);
    tri2.push_back(vertices[2]);
    triangle_list.push_back(tri2);

    // center sub-triangle
    vector<cv::Point3d> tri3;
    tri3.push_back(p01);
    tri3.push_back(p12);
    tri3.push_back(p02);
    triangle_list.push_back(tri3);
  }else{
    triangle_list.push_back(vertices);
  }

  return triangle_list;
}

/**
* Subdivide triangle into 4 smaller triangles
*/
vector<Vec9f> Triangle::forceSubdivide(Vec9f triangle){
	//final list
	vector<Vec9f> triangle_list;
	
	vector<cv::Point3d> vertices = convVec9fToPoint3d(triangle);
	
      //Get the mid points of each of the segments	
      cv::Point3d p01, p12, p02;
      p01 = midPoint(vertices[0], vertices[1]);
      p12 = midPoint(vertices[1], vertices[2]);
      p02 = midPoint(vertices[0], vertices[2]);
      
      // Lower left sub-triangle
    vector<cv::Point3d> tri0; Vec9f tri0f;
    tri0.push_back(vertices[0]);
    tri0.push_back(p01);
    tri0.push_back(p02);
    tri0f = convPoint3dToVec9f(tri0);
    triangle_list.push_back(tri0f);

    // top sub-triangle
    vector<cv::Point3d> tri1;Vec9f tri1f;
    tri1.push_back(p01);
    tri1.push_back(vertices[1]);
    tri1.push_back(p12);
    tri1f = convPoint3dToVec9f(tri1);
    triangle_list.push_back(tri1f);

    // Lower right sub-triangle
    vector<cv::Point3d> tri2;Vec9f tri2f;
    tri2.push_back(p02);
    tri2.push_back(p12);
    tri2.push_back(vertices[2]);
    tri2f = convPoint3dToVec9f(tri2);
    triangle_list.push_back(tri2f);

    // center sub-triangle
    vector<cv::Point3d> tri3;Vec9f tri3f;
    tri3.push_back(p01);
    tri3.push_back(p12);
    tri3.push_back(p02);
    tri3f = convPoint3dToVec9f(tri3);
    triangle_list.push_back(tri3f);
    
    return triangle_list;
}

/**
  * Given 2 lists of matched triangles, subdivides the triangles that are too big to make smaller 
  * matched triangles
  */
  void Triangle::subdivideMatched(vector<Vec9f> &triangle_list1, vector<Vec9f> &triangle_list2){

	vector<Vec9f> new_list1, new_list2, tmp_new1, tmp_new2;
 // for(int k = 0;k < n;k++){ // repeat n times.
   // bool found_flag = false;
   
   new_list1 = triangle_list1;
   new_list2 = triangle_list2;
   
   bool exists_one = true;
   
   while(exists_one){
   exists_one = false;
   vector<Vec9f> tmp_new_list1, tmp_new_list2;
   
    for(int j=0;j<new_list1.size();j++){
      Vec9f vec1 = new_list1[j];
      Vec9f vec2 = new_list2[j];
      
      double max_angle1 = getMaxAngle(vec1);
      double max_angle2 = getMaxAngle(vec2);
      
      if(max_angle1>m_MAX_ANGLE || max_angle2>m_MAX_ANGLE){
        exists_one = true;
      	tmp_new1 = forceSubdivide(vec1);
      	tmp_new2 = forceSubdivide(vec2);
      	
      	for(int l=0;l<tmp_new1.size();l++){
      		tmp_new_list1.push_back(tmp_new1[l]);
      		tmp_new_list2.push_back(tmp_new2[l]);
      	}
      }
      else{
      	tmp_new_list1.push_back(vec1);
      	tmp_new_list2.push_back(vec2);
      }
      
      
     
      }
      
      new_list1 = tmp_new_list1;
      new_list2 = tmp_new_list2;
    }
 
  //Get final resulting triangles
  triangle_list1 = new_list1;
  triangle_list2 = new_list2;
}

/*
 * Get the maximum angle between the center and vertices in 3D.
 */
double
Triangle::getMaxAngle(vector<Point3d> vertices){
  size_t size = 3;

  double max_theta = -1.0;
  double theta = -1.0;
  cv::Point3d center = triangleCenter3d(vertices);

  for(int i = 0;i < 3;i++){
    theta = getVectorAngle(vertices[i], center);
    if(theta > max_theta || max_theta == -1.0){
      max_theta = theta;
    }
  }

  return max_theta;
}

double Triangle::getMaxAngle(Vec9f triangle){
	vector<Point3d> vertices = convVec9fToPoint3d(triangle);
	
	return getMaxAngle(vertices);

}

/*
 * Get the maximum angle between the center and vertices in 3D.
 */
double
Triangle::nPointsMaxAngle(vector<Point3d> vertices){

  double max_theta = -1.0;
  double theta = -1.0;
  cv::Point3d center = nPointsCenter3d(vertices);

  for(int i = 0;i < vertices.size();i++){
    theta = getVectorAngle(vertices[i], center);
    if(theta > max_theta || max_theta == -1.0){
      max_theta = theta;
    }
  }

  return max_theta;
}
