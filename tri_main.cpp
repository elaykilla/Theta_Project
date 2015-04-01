/*
 * Sphere Triangulation
 */
#include <stdio.h>
#include <iostream>
#include <string>
#include "cv_headers.hpp"
#include "PointFeature.hpp"
#include "PersCamera.hpp"
#include "Triangle.hpp"

//using namespace cv;
//using namespace std;

/*
 * extract keypoints
 */
vector<Vec6f> extract_points(Mat image1){
  PointFeature feat;
  vector<KeyPoint> keypoints;

  feat.detect(image1, keypoints);
  feat.showKeyPoints(image1, keypoints);
  feat.printPoints(keypoints);

  vector<Point3d> sphere_points(keypoints.size());
  feat.toSpherePoints(image1, keypoints, sphere_points);
  //  feat.printPoints3d(sphere_points);
  feat.writePoints3d(sphere_points, "../data/3d_points.txt");
  vector< vector<float> > tri_list;

  bool use_vec9f = true;
  vector<Vec9f> vec9f_list;

  if(use_vec9f){
    vec9f_list = feat.readTriangles3d("../data/sphere_triangles.txt");
    feat.printTriangles3d(tri_list);
  }else{
    feat.readTriangles3d("../data/sphere_triangles.txt", &tri_list);
    feat.printTriangles3d(tri_list);
  }

  //  feat.deleteTriangles3d(tri_list);

  vector< vector<Point2f> > trieq_list;
  //  feat.convTriangles3dToEqui(tri_list, image1, &trieq_list);
  vector<Vec6f> vec6f_list = feat.convTriangles3dToEqui(vec9f_list, image1);

  //  feat.showTriangleVertices(image1, trieq_list);
  feat.showTriangleVertices(image1, vec6f_list);
  //  cout << "tri_list size: " << tri_list.size() << "\n";

  return vec6f_list;
}


int main(int argc, char** argv){
  string file1 = "../images/R0010103_small.JPG";
  bool extract_flag = false;

  Mat image1 = imread(file1);

  Vec6f vec(389.0, 219.0, 538.0, 329.0, 553.0, 197.0);
  if(extract_flag){
    vector<Vec6f> vec_list = extract_points(image1);
    vec = vec_list[0];
  }
  // convert the first triangle to a perspective image
  Triangle tr;

  Mat per_image = tr.convToPersRectImage(image1, vec);
  PersCamera cam = tr.getPersCamParams(image1, vec);
  Vec6f p_tri = tr.convToPersTriangle(image1, cam, vec);

  //  imshow("Perspective image", per_image);
  //  waitKey(0);

  tr.showTriangle(per_image, p_tri);
}
