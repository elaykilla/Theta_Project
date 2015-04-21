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
using namespace std;

/*
 * Render triangles
 */
void render_triangles(Mat image){
  bool test_single = false;

  PointFeature feat;

  Mat dst = Mat::zeros(image.rows, image.cols, image.type());

  if(test_single){
    //Vec6f vec(480.0, 240.0, 480.0, 140.0, 580.0, 240.0);  
    Vec6f vec(10.0, 265.0, 152.0, 128.0, 155.0, 297.0);  
    // convert the first triangle to a perspective image
    Triangle tr;

    Vec9f vec3d = feat.toSpherePoint(image, vec);
    PersCamera cam = tr.getPersCamParams(image, vec3d);
     
    EquiTrans trans;
    Mat per_image = trans.toPerspective(image, cam);
    imshow("Perspective", cam.image);

    Vec6f p_tri = tr.convToPersTriangle(image, cam, vec);

    tr.showTriangle(per_image, p_tri);
    waitKey(0);

    Triangle tri;
    trans.toEquirectangular(cam, p_tri, dst);
    imshow("Triangles", dst);
    waitKey(0);

  }else{

    string file[] = {"Txt_files/trianglesPoints3D_test3_1_Satch.txt", "../data/triangles_sphere_points.txt"};
    vector<Vec9f> vec9f_list = feat.readTriangles3d(file[0]);
    vector<Vec6f> vec6f_list = feat.convTriangles3dToEqui(vec9f_list, image);

    cout << vec9f_list.size() << " triangles.\n";
    
    Triangle tr;
    int i = 0;
    
    bool show_flag = false, draw_flag = true, count_flag = true;
    for(vector<Vec6f>::iterator it = vec6f_list.begin(); it != vec6f_list.end(); it++){
      Vec9f vec3d = vec9f_list[i];

      // Subdivide the triangle if necessary
      vector<Vec9f> sub_list;
      sub_list.push_back(vec3d);
      sub_list = tr.subdivide(sub_list);

      for(int j = 0;j < sub_list.size();j++){
	vec3d = sub_list[j];
	PersCamera cam = tr.getPersCamParams(image, vec3d);
     
	EquiTrans trans;
	Vec6f vec = tr.convVec9fToVec6f(image, vec3d);
	Mat per_image = trans.toPerspective(image, cam);
	Vec6f p_tri = tr.convToPersTriangle(image, cam, vec);
	if(show_flag){
	  imshow("Perspective", cam.image);
	  waitKey(0);
	  tr.showTriangle(per_image, p_tri);
	}
	if(draw_flag){
	  tr.drawTriangle(cam.image, p_tri);
	}

	trans.toEquirectangular(cam, p_tri, dst);

	if(show_flag){
	  imshow("Triangles", dst);
	  waitKey(0);
	}
      }
      if(count_flag){
	cout << i;
	if(i % 20 == 0){
	  cout << "\n";
	}else{
	  cout << ", ";
	}
	cout.flush();
      }
      i++;
    }
    imshow("Triangles", dst);
    waitKey(0);
    imwrite("../images/tri.png", dst);
  }
}

/*
 * Verify convertion b/w equirectangular and 3d unit sphere.
 */
void verify_conv(Mat image){
  PointFeature feat;
  vector<KeyPoint> keypoints;

  feat.detect(image, keypoints);
  int id = 100;
  cout << "keypoint\n";
  cout << keypoints[id].pt.x << ", " << keypoints[id].pt.y << "\n";

  vector<Point3d> sphere_points(keypoints.size());
  feat.toSpherePoints(image, keypoints, sphere_points);
  feat.writePoints3d(sphere_points, "../data/3d_points.txt");

  Point3d p = sphere_points[id];
  cout << "sphere_point\n";
  cout << p.x << ", " << p.y << ", " << p.z << "\n";

  EquiTrans trans;
  double x = -1.0, y = -1.0;
  trans.convSpherePointToEquiCoord(p, image, &x, &y);
  cout << "read point\n";
  cout << x << ", " << y << "\n";
  
}

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
  vector<Vec6f> vec6f_list = feat.convTriangles3dToEqui(vec9f_list, image1);

  feat.showTriangleVertices(image1, vec6f_list);

  return vec6f_list;
}

void show_single_triangle(Mat image){
  Vec6f vec(389.0, 219.0, 538.0, 329.0, 553.0, 197.0);

  vector<Vec6f> vec_list = extract_points(image);
  vec = vec_list[0];

  // convert the first triangle to a perspective image
  Triangle tr;

  Mat per_image = tr.convToPersRectImage(image, vec);
  PersCamera cam = tr.getPersCamParams(image, vec);
  Vec6f p_tri = tr.convToPersTriangle(image, cam, vec);

  tr.showTriangle(per_image, p_tri);
}

/*
 * Devide big triangles.
 */
void divide_triangle(){
  PointFeature feat;

    string file[] = {"Txt_files/trianglesPoints3D_test3_1_Satch.txt", "../data/triangles_sphere_points.txt"};
    vector<Vec9f> vec9f_list = feat.readTriangles3d(file[0]);
    vector<Vec9f> new_list;

    cout << vec9f_list.size() << " triangles.\n";
    for(vector<Vec9f>::iterator it = vec9f_list.begin(); it != vec9f_list.end(); it++){
      Triangle tr;
      vector<Vec9f> sub_list;
      sub_list.push_back(*it);
      sub_list =  tr.subdivide(sub_list);
      for(vector<Vec9f>::iterator it = sub_list.begin(); it != sub_list.end(); it++){
	new_list.push_back(*it);
      }
    }
    cout << new_list.size() << " sub-triangles.\n";
}

int main(int argc, char** argv){
  string file1 = "../images/R0010103_small.JPG";
  bool extract_flag = true;
  Mat image1;
  bool shrink_flag = true;

  if(shrink_flag){
    image1 = cv::imread("test3.JPG",1);
    cv::Mat inter = cv::Mat::ones(image1.rows/2, image1.cols/2, image1.type());cv::resize(image1,inter,inter.size(),0,0,INTER_CUBIC); 
    image1 = inter;
  }else{
    image1 = imread(file1);
  }
   
  int mode = 2;

  switch(mode){
  case 0:
    extract_points(image1);
    break;

  case 1:
    show_single_triangle(image1);
    break;

  case 2:
    render_triangles(image1);
    break;

  case 3:
    verify_conv(image1);
    break;

  case 4:
  default:
    divide_triangle();
    break;
  }

}
