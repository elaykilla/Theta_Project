#include <stdio.h>
#include <iostream>
#include <string>
#include "cv_headers.hpp"
#include "EquiTrans.hpp"

//using namespace cv;
using namespace std;


/*
 * get cube face images
 */
void get_cube_faces(Mat image1){
  Mat cube_faces[6];

  EquiTrans equi;
  equi.setFOV(90.0, 90.0);
  equi.makeCubeFaces(image1, cube_faces);
  for(int i = 0;i < 6;i++){
    imshow("cube", cube_faces[i]);
    waitKey(0);
  } 
}

/*
 * Get panning and tilting angles from arguments
 */
int get_angles(int argc, char** argv, double *pan_deg, double *tilt_deg){
  if(argc > 2){
    for(int i = 1;i < argc;i++){
      string str = argv[i];
      if(argc >= (i+1)){
	if(str == "-p"){
	  str = argv[i+1];
	  *pan_deg = atof(str.c_str());
	  i++;
	}else if(str == "-t"){
	  str = argv[i+1];
	  *tilt_deg = atof(str.c_str());
	  i++;
	}
      }
    }
  }
}


/* 
 * get a perspective image
 */
void get_pers(int argc, char** argv, Mat image1){
  double pan_deg, tilt_deg;

  get_angles(argc, argv, &pan_deg, &tilt_deg); 
  printf("Pan: %lf\n", pan_deg);
  printf("Tilt: %lf\n", tilt_deg);
  EquiTrans equi;
  equi.setFOV(90.0, 90.0);
  Mat cam = equi.toPerspective(image1, pan_deg, tilt_deg);
  imwrite("../images/room_regular.png", cam);
  imshow("Perspective image", cam);
  waitKey(0);

}

/*
 * convert perspective to equirectangular
 */
void pers_equi(Mat equi_img){
  PersCamera cam;
  
  double fov_h = M_PI/2.0; // 90.0 degrees
  double fov_v = M_PI/2.0;
  ViewDirection vd;
  vd.pan = 170.0/180.0 * M_PI;
  vd.tilt = 45.0/180.0 * M_PI;

  cam.setCamera(equi_img, fov_h, fov_v, vd);
  EquiTrans trans;
  cam.image = trans.toPerspective(equi_img, cam);
  Mat per_img = cam.image;
  imshow("Perspective", per_img);
  waitKey(0);
  imwrite("../images/per.png", per_img);

  Mat equi = Mat::zeros(equi_img.rows, equi_img.cols, equi_img.type());

  bool tri = true;
  if(tri){
    Vec6f vertices;
    vertices[0] = (float)cam.image.cols/2.0;
    vertices[1] = 0.0;
    vertices[2] = 0.0;
    vertices[3] = (float)cam.image.rows - 1.0;
    vertices[4] = (float)cam.image.cols-1.0;
    vertices[5] = (float)cam.image.rows - 1.0;
    equi = trans.toEquirectangular(cam, vertices, equi);
  }else{
    equi = trans.toEquirectangular(cam, equi);
  }

  imwrite("../images/equi.png",equi);
  imshow("Equirectangular", equi);
  waitKey(0);
  
}

void check_rotation(){
  Point3d point;

  point.x = 1.0;
  point.y = 0.0;
  point.z = 0.0;

  EquiTrans equi;
  double angle = 90.0/180.0*M_PI;
  Point3d np = equi.rotateTilt(angle, point);
}

int main(int argc, char** argv){
  string file1 = "../images/R0010103_small.JPG";
  double pan_deg = 0.0, tilt_deg = 90.0;


  Mat image1 = imread(file1);
  double focal_length = 36.0;

  int mode  = 2;

  switch(mode){
  case 0:
    get_cube_faces(image1);
    break;
  case 1:
    get_pers(argc, argv, image1);
    break;
  case 2:
    pers_equi(image1);
    break;
  case 3:
  default:
    check_rotation();
    break;
  }

}

