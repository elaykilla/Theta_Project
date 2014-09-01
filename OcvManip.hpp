/*
* @author: ELay Maiga
* This class contains all the methods related to calculating coordinates and making projections
* from 2D to 3D or vice-versa.
*/

#ifndef OCVMANIP_H
#define OCVMANIP_H



#include"MathCalcs.hpp"
#include "matcher.h"
//#include "boost_headers.hpp"



using namespace cv;
/***************************************************Image Manipulation Cv***********************************************/

/**
* This function takes the image prefix name, adds the position i and saves in a cv::Mat
*/
void loadImagei(string name, int i, cv::Mat &image);
/**
* Function to load top and bottom images
*/
void loadImageTop(string name, cv::Mat &image, string topOrbottom);


/**
* This function returns a cv::vector containing the Keypoints from the input image using SURF
*/
vector<cv::KeyPoint> get2DKeypoints(cv::Mat image);

/**
* This function takes 2 input images, finds the keypoints, matches them then draws Epipolar lines on both images
*/
void drawEpipolarLines(Mat &image1, Mat &image2, Mat &imageMatches);
#endif
