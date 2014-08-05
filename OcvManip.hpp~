/*
* @author: ELay Maiga
* This class contains all the methods related to calculating coordinates and making projections
* from 2D to 3D or vice-versa.
*/

#ifndef OcvManip
#define OcvManip

#include"MathCalcs.hpp"





/***************************************************Image Manipulation Cv***********************************************/

/**
* This function takes the image prefix name, adds the position i and saves in a Mat
*
*
*/
void loadImagei(string name, int i, Mat &image){
	ostringstream file;
	file << name <<i << ".jpg" ; 
	image = imread(file.str(),CV_LOAD_IMAGE_COLOR);
}

/**
* This function returns a cv::vector containing the Keypoints from the input image using SURF
*/
vector<KeyPoint> get2DKeypoints(Mat image){
//-- Step 1: Detect the keypoints using SURF Detector
  int minHessian = 400;
  
  SurfFeatureDetector detector( minHessian );
  vector<KeyPoint> keypoints;
  
  detector.detect( image, keypoints );
  
  return keypoints;
}

#endif
