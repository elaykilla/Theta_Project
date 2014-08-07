/*
* @author: ELay Maiga
* This class contains all the methods related to calculating coordinates and making projections
* from 2D to 3D or vice-versa.
*/

#ifndef OcvManip
#define OcvManip

#include"MathCalcs.hpp"
//#include "boost_headers.hpp"




/***************************************************Image Manipulation Cv***********************************************/

/**
* This function takes the image prefix name, adds the position i and saves in a cv::Mat
*
*
*/
void loadImagei(string name, int i, cv::Mat &image){
	ostringstream file;
	file << name <<i << ".jpg" ; 
	image = cv::imread(file.str(),1);
}

/**
* This function returns a cv::vector containing the Keypoints from the input image using SURF
*/
vector<cv::KeyPoint> get2DKeypoints(cv::Mat image){
//-- Step 1: Detect the keypoints using SURF Detector
  int minHessian = 400;
  
  cv::SurfFeatureDetector detector( minHessian );
  vector<cv::KeyPoint> keypoints;
  
  detector.detect( image, keypoints );
  
  return keypoints;
}

#endif
