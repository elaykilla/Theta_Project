/*
 * PointFeature
 *   detect, match, read and save point featurs from/to a file.
 */
#ifndef _POINT_FEATURE_HPP
#define _POINT_FEATURE_HPP
#include "cv_headers.hpp"
#include <iostream>
#include <fstream>
#include <string>
#include "EquiTrans.hpp"

using namespace cv;
using namespace std; 

class PointFeature {
  enum Method {SURF, SIFT};

private:  
  Method method;
  int minHessian; // for SURF
  vector< vector<Point2f> > convVec6fToPoint2f(vector<Vec6f> &tri_list);
  vector<Point2f> convVec6fToPoint2f(Vec6f vec);   
  vector<Point3d> convVec6fToPoint3d(Vec6f p); 
  Vec9f point3dToVec9f(vector<Point3d> p_list);

public:
  PointFeature();
  bool detect(Mat image, vector<KeyPoint> &keypoints);
  void setMinHessian(int);
  void showKeyPoints(Mat imge, vector<KeyPoint> &keypoints);
  void printPoints(vector<KeyPoint> &keypoints);
  void toSpherePoints(Mat image, vector<KeyPoint> &keypoints, vector<Point3d> &points);
  Vec9f toSpherePoint(Mat image, Vec6f vec);
  vector<Point3d> toSpherePoint(Mat image, vector<Point2f> vec);
  void printPoints3d(vector<Point3d> &points);
  void writePoints3d(vector<Point3d> &points, string filename); 
  vector<cv::Point3d> readPoints3d(string filename);
  void readTriangles3d(string filename, vector< vector<float> > *tri_list ); 
  void writeTriangles3d(vector<Vec9f> triangles, string filename);
  vector<Vec9f> readTriangles3d(string filename);
  void deleteTriangles3d(vector< vector<float> > &tri_list);
  void printTriangles3d(vector< vector<float> > &tri_list);
  void printTriangles3d(vector<Vec9f> &tri_list);
  void convTriangles3dToEqui(vector< vector<float> >&tri3d_list, Mat img, vector< vector<Point2f> > *trieq_list);
  vector<Vec6f> convTriangles3dToEqui(vector<Vec9f> tri3d_list, Mat equi_img);
  void showTriangleVertices(Mat image, vector< vector<Point2f> > &tri_list);
  void showTriangleVertices(Mat image, vector<Vec6f> &tri_list);
};
#endif
