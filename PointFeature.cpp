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
  double pan_offset = (double)ncols/2.0 + 0.5;
  double tilt_offset = (double)nrows/2.0 + 0.5;
  double rmax = (double)nrows - 1.0;

  vector<Point3d>::iterator it_3p = points.begin();
  for(vector<KeyPoint>::iterator it = keypoints.begin(); it != keypoints.end(); ++it, ++it_3p){
    // to center origin
    double c_x = it->pt.x - pan_offset;
    double c_y = rmax - it->pt.y - tilt_offset;

    // to radian
    double pan = c_x / ncols * M_PI *2.0;
    double tilt = c_y / nrows * M_PI;

    // coodinate system is the same with Sachit's MS thesis.
    it_3p->x = cos(tilt)*cos(pan); // depth
    it_3p->y = cos(tilt)*sin(pan); // left-to-right
    it_3p->z = sin(tilt);          // height

  }
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
    bool debug_flag = true;
      
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
	  cout << "(x, y) = (" << x << ", " << y << "\n";
	}
      }

      trieq_list->push_back(eq_points);
    }
}

/*
 * Convert trianlges on the unit sphere to those in equirectangular image.
 *   triangles are in vector< vector<float> >.   
 */
vector<Vec6f> PointFeature::convTriangles3dToEqui(vector<Vec9f>&tri3d_list, Mat equi_img){
    int tri_num = tri3d_list.size();
    bool debug_flag = true;

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
	  cout << "(x, y) = (" << x << ", " << y << "\n";
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
