/*
 * PointFeature
 *   detect, match, read and save point featurs from/to a file.
 */
#include "PointFeature.hpp"

using namespace std;
using namespace cv;

PointFeature::PointFeature(){
  method = SURF;
  minHessian = 400;
}

/*
 * Detect features with the selected method.
 */
bool PointFeature::detect(Mat image, vector<KeyPoint> &keypoints){
  bool ret = false;
  
  if(image.data){
    if(method == SURF){
      SurfFeatureDetector surf( minHessian);
      surf.detect(image, keypoints);
    }
  }

  return false;
}

/*
 * SURF parameter of minHessian.
 */
void PointFeature::setMinHessian(int val){
  minHessian = val;
}

/*
 * show KeyPoints with the image on a display
 */
void PointFeature::showKeyPoints(Mat image, vector<KeyPoint> &keypoints){
  Mat img_keypoints;
  
  drawKeypoints(image, keypoints, img_keypoints, Scalar::all(-1), DrawMatchesFlags::DEFAULT );

  imshow("Keypoints", img_keypoints);
  waitKey(0);
}

/*
 * print Points
 */
void PointFeature::printPoints(vector<KeyPoint> &keypoints){
  cout << "Keypoints:\n";
  for(vector<KeyPoint>::iterator it = keypoints.begin(); it != keypoints.end(); ++it){
    cout << it->pt.x << ", " << it->pt.y << "\n";
  }
}

/*
 * convert point image coordinates to 3D coordinates on a unit sphere.
 *   coordinate system: (same as Sache's MS theis)
 *        Content-based Projections for Paranoramic Images and Videos
 *        April 5, 2010
 *
 *         x: depth
 *         y: horizontal (left-to-right)
 *         z: height (bottom-to-top)
 */
void PointFeature::toSpherePoints(Mat image, vector<KeyPoint> &keypoints, vector<Point3d> &points){
  int ncols = image.cols;
  int nrows = image.rows;
  double half_width = (double)ncols/2.0;
  double half_height = (double)nrows/2.0;
  double rmax = (double)nrows - 1.0;

  vector<Point3d>::iterator it_3p = points.begin();
  for(vector<KeyPoint>::iterator it = keypoints.begin(); it != keypoints.end(); ++it, ++it_3p){
    // to center origin
    double c_x = it->pt.x - half_width + 0.5;
    double c_y = half_height - it->pt.y - 0.5;

    // to radian
    double pan = c_x / ncols * M_PI *2.0;
    double tilt = c_y / nrows * M_PI;

    // coodinate system is the same with Sachit's MS thesis.
    it_3p->x = cos(tilt)*cos(pan); // depth
    it_3p->y = cos(tilt)*sin(pan); // left-to-right
    it_3p->z = sin(tilt);          // height

  }
}

  /**
  * Converts a list of 3d points into their equi corresponding points
  */
  vector<Point2f> PointFeature::toEquiPoints(Mat image, vector<Point3d> points3d){
  	EquiTrans trans_class;
  	vector<Point2f> points;
  	
  	double x,y;
  	for(int i=0;i<points3d.size();i++){
  		Point3d p = points3d[i];
  		trans_class.convSpherePointToEquiCoord(p, image, &x, &y);
  		
  		Point2f p2;
  		p2.x = x;
  		p2.y = y;
  		points.push_back(p2);
  	}
  	
  	return points;
  }


/*
 * convert a point in equirectangular image to that in 3d unit sphere.
 */
vector<Point3d> PointFeature::toSpherePoint(Mat image, vector<Point2f> point_list){
  int ncols = image.cols;
  int nrows = image.rows;
  double pan_offset = (double)ncols/2.0 + 0.5;
  double tilt_offset = (double)nrows/2.0 + 0.5;
  double rmax = (double)nrows - 1.0;

  vector<Point3d> dp_list;

  for(vector<Point2f>::iterator it = point_list.begin(); it != point_list.end(); ++it){
    // to center origin
    double c_x = (double)it->x - pan_offset;
    double c_y = rmax - (double)it->y - tilt_offset;

    // to radian
    double pan = c_x / ncols * M_PI *2.0;
    double tilt = c_y / nrows * M_PI;

    Point3d dp;
    // coodinate system is the same with Sachit's MS thesis.
    dp.x = cos(tilt)*cos(pan); // depth
    dp.y = cos(tilt)*sin(pan); // left-to-right
    dp.z = sin(tilt);          // height

    dp_list.push_back(dp);
  }
  
  return dp_list;
}

/*
 * Convert a point in Point3d to Vec9f
 */
Vec9f PointFeature::point3dToVec9f(vector<Point3d> p_list){
  Vec9f vec;
  int ele_num = 9;

  if(p_list.size() == 3){
    for(int i = 0, k = 0;i < 3;i++){
      vec[k++] = p_list[i].x;
      vec[k++] = p_list[i].y;
      vec[k++] = p_list[i].z;
    }
  }

  return vec;
}


/*
 * convert a point in equirectangular image to that in 3d unit sphere.
 */
Vec9f PointFeature::toSpherePoint(Mat image, Vec6f vec){
  vector<Point2f> point_list = convVec6fToPoint2f(vec);
  vector<Point3d> p_list = toSpherePoint(image, point_list);
  Vec9f vec9 = point3dToVec9f(p_list);
  
  return vec9;
}

/*
 * print 3D Points
 */
void PointFeature::printPoints3d(vector<Point3d> &points){
  cout << "Point3:\n";
  for(vector<Point3d>::iterator it = points.begin(); it != points.end(); ++it){
    cout << it->x << ", " << it->y << ", " << it->z << "\n";
  }
}

/*
 * write 3D points to the specified file
 */
void PointFeature::writePoints3d(vector<Point3d> &points, string filename ){
  ofstream file;
  file.open(filename.c_str());
  for(vector<Point3d>::iterator it = points.begin(); it != points.end(); ++it){
    file << it->x << ", " << it->y << ", " << it->z << "\n";
  }
  file.close();
}

/**
* Read points from file into vector of points 3D
*/
vector<Point3d> PointFeature::readPoints3d(string filename ){
  ifstream infile (filename.c_str());
  string value;
  vector<Point3d> points3d;
  
  //Verifying the read function
  ofstream logFile;
  ostringstream newFile;
  newFile << "RW_ReadPoints3d.txt" ;
  string name = newFile.str();
  logFile.open(name.c_str());
  
  logFile << filename << endl;
  Point3d p;
  
  double factor = 100000;
  bool debug_flag = false;
  while(infile.good()){
    string line;
    while(getline(infile, line)){
	istringstream line_stream(line);
	size_t size = 3;
	vector<float>  vec(size);

	int i = 0;
	while(getline(line_stream, value, ',')){
	    double dvalue = atof(value.c_str());
	    logFile << "read value: " << dvalue << "||" << "Written value: " << round((float)dvalue*factor)/factor << endl;
	    //vec[i] = round((float)dvalue*factor)/factor;
	    vec[i] = dvalue;
	    if(debug_flag){
	      cout << dvalue << ", ";
	    }
	    i++;
	}
	
	p.x = vec[0];
	p.y = vec[1];
	p.z = vec[2];
	points3d.push_back(p);
      }
  }
	logFile.close();
	return points3d;	
}


/**
* Function to write 3d triangles into file in the following format:
* p1.x ,p1.y, p1.z ,p2.x ,p2.y, p2.z, p3.x ,p3.y, p3.z
*/
void PointFeature::writeTriangles3d(vector<Vec9f> triangles, string filename){
	Vec9f triangle;
	ofstream logFile;
	
	//Open file
	logFile.open(filename.c_str());
	for(int i=0;i<triangles.size();i++){
		triangle = triangles[i];
		
		for(int j=0;j<8;j++){
			logFile << triangle[j] << ",";
		}
		logFile << triangle[8] << endl;
	}
	
	logFile.close();

}

/*
 * read 3D triangles on the unit sphere.
 *    format:
 *      each line includes 3 vertecies in Euclidean 3D.
 */
void PointFeature::readTriangles3d(string filename, vector< vector<float> > *tri_list){

  ifstream infile (filename.c_str());
  string value;

  bool debug_flag = false;

  while(infile.good()){
    string line;
    while(getline(infile, line)){
	istringstream line_stream(line);
	size_t size = 9;
	vector<float>  vec(size);

	int i = 0;
	while(getline(line_stream, value, ',')){
	    double dvalue = atof(value.c_str());
	    vec[i] = (float)dvalue;
	    if(debug_flag){
	      cout << dvalue << ", ";
	    }
	    i++;
	}
	if(debug_flag){
	  cout << "\n";
	}

	tri_list->push_back(vec);
      }
  }
}

/*
 * read 3D triangles on the unit sphere.
 *    format:
 *      each line includes 3 vertecies in Euclidean 3D.
 */
vector<Vec9f> PointFeature::readTriangles3d(string filename){

  vector<Vec9f> tri_list;

  ifstream infile (filename.c_str());
  string value;

  bool debug_flag = false;

  while(infile.good()){
    string line;
    while(getline(infile, line)){
	istringstream line_stream(line);

	Vec9f vec_9;

	int i = 0;
	while(getline(line_stream, value, ',')){
	    double dvalue = atof(value.c_str());
	    vec_9[i] = (float)dvalue;
	    if(debug_flag){
	      cout << dvalue << ", ";
	    }
	    i++;
	}
	if(debug_flag){
	  cout << "\n";
	}

	tri_list.push_back(vec_9);
      }
  }

  return tri_list;
}

/*
 * Delete the memory for storing triangles.
 */
void PointFeature::deleteTriangles3d(vector< vector<float> > &tri_list){
  if(tri_list.size() != 0){
    int size = tri_list.size();
    for(int i = 0;i < size;i++){
      vector<float> vec = tri_list[i];
      vec.clear();
    }
    tri_list.clear();
  }
}

/*
 * Print triangles from the data type of vector< vector<float> >
 */
void PointFeature::printTriangles3d(vector< vector<float> > &tri_list){
  if(tri_list.size() != 0){
    int tri_num = tri_list.size();
    int tri_size = 9;

    for(int i = 0;i < tri_num;i++){
      vector<float> vec = tri_list[i];
      for(int j = 0;j < tri_size;j++){
	    cout << vec[j];
	  if(j == 8){
	    cout << "\n";
	  }else{
	    cout << ", ";
	  }
      }
    }
  }
}

/*
 * Print triangles from the data type of vector< vector<float> >
 */
void PointFeature::printTriangles3d(vector<Vec9f> &tri_list){
  if(tri_list.size() != 0){
    int tri_num = tri_list.size();
    int tri_size = 9;

    for(int i = 0;i < tri_num;i++){
      Vec9f vec = tri_list[i];
      for(int j = 0;j < tri_size;j++){
	    cout << vec[j];
	  if(j == 8){
	    cout << "\n";
	  }else{
	    cout << ", ";
	  }
      }
    }
  }
}




/*
 * Convert trianlges on the unit sphere to those in equirectangular image.
 *   triangles are in vector< vector<float> >.   
 */
void PointFeature::convTriangles3dToEqui(vector< vector<float> >&tri3d_list, Mat equi_img, vector< vector<Point2f> > *trieq_list){
    int tri_num = tri3d_list.size();
    int tri_size = 9;
    bool debug_flag = false;
      
    for(int i = 0;i < tri_num;i++){
      vector<float> vec = tri3d_list[i];
      size_t ele_num = 9;
      size_t point_num = 3;
      vector<Point2f> eq_points(point_num);


      for(int j = 0, k = 0;j < ele_num;j += 3, k++){
	Point3f point_3d; 
	point_3d.x = vec[j];
	point_3d.y = vec[j+1];
	point_3d.z = vec[j+2];
	EquiTrans trans;
	double x = -1.0, y = -1.0;
	trans.convSpherePointToEquiCoord(point_3d, equi_img, &x, &y);
	eq_points[k].x = (float)x;
	eq_points[k].y = (float)y;
	if(debug_flag){
	  cout << "(x, y) = (" << x << ", " << y << ")\n";
	}
      }

      trieq_list->push_back(eq_points);
    }
}

/*
 * Convert trianlges on the unit sphere to those in equirectangular image.
 *   triangles are in vector< vector<float> >.   
 */
vector<Vec6f> PointFeature::convTriangles3dToEqui(vector<Vec9f> tri3d_list, Mat equi_img){
    cout << "convTriangles3dToEqui called with: " << tri3d_list.size() << " triangles" << endl;
    int tri_num = tri3d_list.size();
    bool debug_flag = false;

    vector<Vec6f> tri2d_list;  // Triangles in equirectangular image
      
    for(int i = 0;i < tri_num;i++){
      Vec9f vec = tri3d_list[i];
      int ele_num = 9;
      size_t point_num = 3;
      Vec6f eq_points;

      for(int j = 0, k = 0;j < ele_num;j += 3, k += 2){
	Point3f point_3d; 
	point_3d.x = vec[j];
	point_3d.y = vec[j+1];
	point_3d.z = vec[j+2];
	EquiTrans trans;
	double x = -1.0, y = -1.0;
	trans.convSpherePointToEquiCoord(point_3d, equi_img, &x, &y);

	eq_points[k] = (float)x;
	eq_points[k+1] = (float)y;
	if(debug_flag){

	  cout << "(x, y) = (" << x << ", " << y << ")\n";

	 // cout << "(x, y) = (" << x << ", " << y << "\n";


	}
      }

      tri2d_list.push_back(eq_points);
    }

    return tri2d_list;
}

/*
 * convert Vec6f to Point2f
 */
vector< vector<Point2f> > PointFeature::convVec6fToPoint2f(vector<Vec6f> &tri_list){
  vector< vector<Point2f> > point_list;

  int tri_num = tri_list.size();
  for(int i = 0;i < tri_num;i++){
    Vec6f vec = tri_list[i];
    int ele_num = 6;
    size_t point_num = 3;
    vector<Point2f> points(point_num); 
    for(int i = 0, k = 0;i < ele_num; i += 2, k++){
      points[k].x = vec[i];
      points[k].y = vec[i+1];
    }
    point_list.push_back(points);
  }

  return point_list;
}


/*
 * convert Vec6f to Point2f
 */
vector<Point2f> PointFeature::convVec6fToPoint2f(Vec6f vec){

  int ele_num = 6;
  size_t point_num = 3;
  vector<Point2f> point_list; 

  for(int i = 0, k = 0;i < ele_num; i += 2, k++){
    Point2f p;
    p.x = vec[i];
    p.y = vec[i+1];
    point_list.push_back(p);      
  }

  return point_list;
}

/* 
 * Convert Point2f list to Vec6f
 */
Vec6f PointFeature::convPoint2fToVec6f(vector<Point2f> point_list){
  int num = point_list.size();
  Vec6f p6f;

  int k = 0;
  for(int i = 0;i < num;i++){
    p6f[k++] = point_list[i].x;
    p6f[k++] = point_list[i].y;
  }

  return p6f;
}

/*
 * Show Triangle Vertices with the image on a display
 */
void PointFeature::showTriangleVertices(Mat image, vector< vector<Point2f> > &tri_list){

  Mat img_points = image;

  int tri_num = tri_list.size();
  int radius = 5;

  for(int i = 0;i < tri_num;i++){
    vector<Point2f> points = tri_list[i];
    for(vector<Point2f>::iterator it = points.begin(); it != points.end(); ++it){
      Point ipoint;
      ipoint.x = (int)it->x;
      ipoint.y = (int)it->y;

      circle(img_points, ipoint, radius, Scalar(0, 255, 0));
    }
  }

  imshow("Triangle vertices", img_points);
  waitKey(0);
}


/*
 * Show Triangle Vertices with the image on a display
 */
void PointFeature::showTriangleVertices(Mat image, vector<Vec6f> &tri_list){
  vector < vector<Point2f> > point_list = convVec6fToPoint2f(tri_list);

  showTriangleVertices(image, point_list);
}
