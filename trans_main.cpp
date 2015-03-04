#include <stdio.h>
#include <iostream>
#include <string>
#include "cv_headers.hpp"
#include "EquiTrans.hpp"

using namespace cv;
using namespace std;

int get_angles(int argc, char** argv, double *pan_deg, double *tilt_deg);

int main(int argc, char** argv){
  string file1 = "../images/R0010103.JPG";
  double pan_deg = 0.0, tilt_deg = 0.0;

  get_angles(argc, argv, &pan_deg, &tilt_deg); 
  printf("Pan: %lf\n", pan_deg);
  printf("Tilt: %lf\n", tilt_deg);

  Mat image1 = imread(file1);
  double focal_length = 36.0;
  EquiTrans equi(focal_length);
  Mat cam = equi.toRegular(image1, pan_deg, tilt_deg);

  imwrite("../images/room_regular.png", cam);
  
  imshow("Perspective image", cam);
  waitKey(0);
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
