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
* This function, given a keypoint array returns the points with scale 1
*/
vector<cv::Point2f> convertKeypoints(vector<cv::KeyPoint> keypoints);

//Ratio test
int ratioTest(std::vector<std::vector<cv::DMatch> >& matches, float ratio);

// Symetrie between matches
void symmetryTest(const std::vector<std::vector<cv::DMatch> >& matches1,
		const std::vector<std::vector<cv::DMatch> >& matches2,
		std::vector<cv::DMatch>& symMatches);
//Computes the matches between 2 images and returns matches
vector<cv::DMatch> getMatches(cv::Mat image1, cv::Mat image2);


//Returns a vector of all matching points in image 1 and image 2 therefore: points1[i] matches to points2[i] 
vector<vector<cv::Point2f> > getMatchedPoints(vector<cv::KeyPoint> keypoints1, vector<cv::KeyPoint> keypoints2, vector<cv::DMatch> matches);


//Return a vector of matching keypoints in image1 and image2. Only returns the keypoints that have matches and therefore keypoints1[i] matches with keypoints2[i]  
vector<vector<cv::KeyPoint> > getMatchedKeypoints(vector<cv::KeyPoint> keypoints1, vector<cv::KeyPoint> keypoints2, vector<cv::DMatch> matches);

//Computes keypoints and matches between 2 images
void getKeypointsAndMatches(Mat image1, Mat image2, vector<KeyPoint> &keypoints1, vector<KeyPoint> &keypoints2,vector<DMatch> &matches);
/**
* This function takes 2 input images, finds the keypoints, matches them then draws Epipolar lines on both images
*/
void drawEpipolarLines(Mat &image1, Mat &image2, Mat &imageMatches);

/** 
* Function to interpolate between 2 images where the center has been translated 
*/
cv::Mat linearInterpolate(cv::Mat image1, cv::Mat image2, double dist, double pos);


/**
* Function calculates the optical flow map between 2 images
*/
void optFlowMap(cv::Mat image1, cv::Mat image2, cv::Mat &flow);

void lkOptFlowMap(cv::Mat image1, cv::Mat image2);


/** 
* This function calculates the delaunay triangles using keypoitns and rows and cols of an image
*/
cv::Subdiv2D getDelaunayTriangles(vector<cv::KeyPoint> keypoints, int rows, int cols);


/**
* This function find the position of a given triangle in a list of triangles. It returns -1 if no triangle is found. A triangle is represented as a list of 6 floating points
*/
int findTriangleInVector(Vec6f triangle,vector<Vec6f> trianglesList );


/**
* This function find the position of a given facet in a list of facets. It returns -1 if no facet is found. A facet is represented as a list of 3 edges
*/
int findFacetInVector(vector<Point2f> facet,vector<vector<Point2f> > facetsList );

/**
* This function given a point p and a Delaunay subdivision, returns the index of the triangle containing the point 
*/
int locateTriangleIndex(cv::Subdiv2D subdiv ,cv::Point2f p ); 

/**
* This function, given 2 lists of matching triangles, computes the affine transformation between each of the triangles
*/
vector<cv::Mat> getAffineTriangleTransforms (vector<Vec6f> triangles1, vector<Vec6f> triangles2);


#endif
