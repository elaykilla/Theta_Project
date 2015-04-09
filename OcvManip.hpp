/*
* @author: ELay Maiga
* This class contains all the methods related to calculating coordinates and making projections
* from 2D to 3D or vice-versa.
*/

#ifndef OCVMANIP_H
#define OCVMANIP_H



#include"MathCalcs.hpp"
#include "matcher.h"
//#include "ViewDirection.hpp"
//#include "EquiTrans.hpp"
#include "Triangle.hpp"
#include "PointFeature.hpp"
#include "boost_headers.hpp"
#include "PclManip.hpp"


using namespace cv;

//Define new types 
//typedef Vec<float, 9> Vec9f;
/***********************************Image Manipulation Cv***********************************************/

/**
* This function displays the mat values
*/
void showMat(Mat mat);


/**
* This function takes a list of images and makes a video
*/
void imageListToVideo(vector<cv::Mat> images, string fileName) ; 


/**
* This function returns an array of the 3 color histograms of the given image
*/
vector<Mat> getHistograms(Mat img);

////////////////////////// KeyPoints and Matching ////////////////////////////////////////////////////
/**
* This function takes the image prefix name, adds the position i and saves in a cv::Mat
*/
void loadImagei(string name, int i, cv::Mat &image);
/**
* Function to load top and bottom images
*/
void loadImageTop(string name, cv::Mat &image, string topOrbottom);

/**
rotate image y axis by y degrees celcius
**/
Mat rotateImagey(Mat image, double y);

/**
* This function returns a cv::vector containing the Keypoints from the input image using SURF
*/
vector<cv::KeyPoint> get2DKeypoints(cv::Mat image);

/**
* This function returns a cv::vector containing the Keypoints from the input image using SIFT
*/
vector<cv::KeyPoint> getSiftKeypoints(cv::Mat image);


/**
* This function extracts keypoints from a cube of 6 images. 
* These keypoints are normalized to an equirectangular image which was at the origin of the 6 cube faces
*/
vector<cv::KeyPoint> getCubeKeypoints(cv::Mat origin);

/** 
* This functions extracts cube faces and matches points on corresponding faces
*/
vector<vector<cv::KeyPoint> > getMatchedCubeKeypoints(cv::Mat img1, cv::Mat img2);

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


//Computes the matches between 2 images and returns matches using BruteForce
vector<cv::DMatch> getMatches(cv::Mat image1, cv::Mat image2,vector<cv::KeyPoint> keypoints1 ,vector<cv::KeyPoint> keypoints2);

//Computes the matches between 2 images and returns matches using Flann
vector<cv::DMatch> getFlannMatches(cv::Mat image1, cv::Mat image2,vector<cv::KeyPoint> keypoints1 ,vector<cv::KeyPoint> keypoints2);

//Computes the matches between 2 images and returns matches using BruteForce
//vector<cv::DMatch> getSiftMatches(cv::Mat image1, cv::Mat image2);


//Returns a vector of all matching points in image 1 and image 2 therefore: points1[i] matches to points2[i] 
vector<vector<cv::Point2f> > getMatchedPoints(vector<cv::KeyPoint> keypoints1, vector<cv::KeyPoint> keypoints2, vector<cv::DMatch> matches);


//Return a vector of matching keypoints in image1 and image2. Only returns the keypoints that have matches and therefore keypoints1[i] matches with keypoints2[i]  
vector<vector<cv::KeyPoint> > getMatchedKeypoints(vector<cv::KeyPoint> keypoints1, vector<cv::KeyPoint> keypoints2, vector<cv::DMatch> matches);

//Computes keypoints and matches between 2 images
void getKeypointsAndMatches(Mat image1, Mat image2, vector<KeyPoint> &keypoints1, vector<KeyPoint> &keypoints2,vector<DMatch> &matches);


//Computes Sift keypoints and matches between 2 images
void getSiftKeypointsAndMatches(Mat image1, Mat image2, vector<KeyPoint> &keypoints1, vector<KeyPoint> &keypoints2,vector<DMatch> &matches);


/**
* This function takes 2 input images, finds the keypoints, matches them then draws Epipolar lines on both images
*/
void drawEpipolarLines(Mat &image1, Mat &image2, Mat &imageMatches);

//////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////// Line Detection //////////////////////////////////////////////////////
vector<cv::Vec4i> getLinesProb(cv::Mat image);

cv::Mat detectEdges(cv::Mat img);


///////////////////////////Image Interpolation and Triangulation /////////////////////////////////////////
/** 
* Function to interpolate between 2 images where the center has been translated 
*/
cv::Mat linearInterpolate(cv::Mat image1, cv::Mat image2, double dist, double pos);

/** 
* Function to interpolate between 2 images using Delaunay triangulation.
* Full function...
*	- First extracts and matches keypoints
*	- Compute affine transformations
*	- Generate interpolated image by interpolating individual trianges
*
*/
cv::Mat delaunayInterpolate(cv::Mat image1, cv::Mat image2, double dist, double pos);


/**
* Function to retrieve only interpolated content. Given 2 images with their respective triangle positions,
* this function interpolates the content of the intermediate triangle. 
* 
*/
cv::Mat getInterpolatedTriangleContent(cv::Mat image1, cv::Mat image2, cv::Vec6f triangle1, cv::Vec6f triangle2, cv::Vec6f &trianglesInter, vector<PointWithColor> &content, double dist, double pos);


/**
* Function to retrieve only interpolated content. Given 2 images with their respective triangle positions,
* this function interpolates the content of the intermediate triangle using different perspective cameras. 
* 
*/
cv::Mat getInterpolatedTriangleContentDiffCam(PersCamera cam1, PersCamera cam2, cv::Vec6f triangle1, cv::Vec6f triangle2, cv::Vec6f &trianglesInter, vector<PointWithColor> &content, double dist, double pos);



/**
* Function to retrieve only interpolated content. Given 2 images with their respective triangle positions and perspective camera parameters.
* this function interpolates the content of the intermediate triangle. 
*/
cv::Mat getDiffPerspInterpolate(cv::Mat img1, cv::Mat img2, PersCamera cam1, PersCamera cam2, cv::Vec6f triangle1, cv::Vec6f triangle2, vector<PointWithColor> &content, double dist, double pos);

/**
* Function to interpolate between 2 images having already extracted keypoints and matched them and computed
* triangulation
* Inputs:
*	- Img1, Img2 
*	- Triangles1, Triangles2: matched triangles
* Outputs:
*	- Interpolated image
*/
cv::Mat interpolateWithGivenTriangles(cv::Mat image1, cv::Mat image2, vector<cv::Vec6f> triangles1, vector<cv::Vec6f> triangles2, vector<cv::Vec6f> &trianglesInter, double dist, double pos);


/** 
* Function to interpolate between 2 images using Delaunay triangulation using triangles on the surface of the sphere
*/
cv::Mat delaunayInterpolateSphere(cv::Mat img1, cv::Mat img2, double dist, double pos);
/** 
* Function to interpolate n times between 2 images using Delaunay triangulation
*/
vector<cv::Mat> delaunayInterpolateMultiple(cv::Mat image1, cv::Mat image2, double dist, int n);

/** 
* Function to interpolate between 2 images using Delaunay triangulation using triangles on the surface of the sphere and cube faces
*/
void delaunayInterpolateCubeMakeTriangles(cv::Mat img1, cv::Mat img2, double dist, double pos, string file1, string file2);


/** 
* Function to interpolate between 2 images using Delaunay triangulation using triangles on the surface of the sphere and cube faces. This is the 2nd part of the previous function. Supposes the 3D triangles have been interpolated
*/
cv::Mat delaunayInterpolateCubeFromTriangles(cv::Mat img1, cv::Mat img2, double dist, double pos, string triangles_file, vector<PointXYZRGB> points1c, vector<PointXYZRGB> points2c);
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
* Given 2 lists of keypoints, this function returns the list of corresponding triangles
*/
void getCorrespondingDelaunayTriangles(vector<cv::KeyPoint> keypoints1, vector<cv::KeyPoint> keypoints2, vector<cv::Vec6f> &trianglesList1, vector<cv::Vec6f> &trianglesList2);

//Given a list of triangles using keypoints 1, this function calculates the relating triangles obtained using keypoint matches
void makeCorrespondingDelaunayTriangles(vector<cv::Point2f> points1, vector<cv::Point2f> points2, vector<cv::Vec6f> &trianglesList1, vector<cv::Vec6f> &trianglesList2);



/**
* Given 2 images this function:
* - Extracts and matches keypoints 
* - Computes triangles in first image and gets matching triangles in 2nd image
*/
void getMatchingTrianglesFromImages(Mat image1, Mat image2, vector<Vec6f> &triangles1, vector<Vec6f> &triangles2);
/**
* This function calculates the corresponding vertices of a triangle list from 2D (i,j) to 3D (x,y,z) using * the warping of the 2D image onto a Sphere 
*/
void get3DSphereTriangles(int rows, int cols, double r, vector<Vec6f> triangles2D,vector<Vec9f> triangles3D);
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
int locateTriangleIndex(cv::Subdiv2D subdiv , vector<cv::Vec6f> triangles, cv::Point2f p ); 

/**
* This function, given 2 lists of matching triangles, computes the affine transformation between each of the triangles
*/
vector<cv::Mat> getAffineTriangleTransforms (vector<Vec6f> triangles1, vector<Vec6f> triangles2);



/**
* This function determines wether a point is on a given faces
* - Point defined by it's (i,j) coordinates on equirectangular image1
* - a face is determined by the theta and phi angles of it's center point
*/
bool isOnFaceRaw(int rows, int cols, int i1, int j1, double theta,double phi);


/**
* This function determines wether 2 points given in equirectangular coordinates are in the same cubic faces
*/
bool onSameCubicFaceRaw(int rows, int cols, int i1, int j1, int i2, int j2);

////////////////////////////////////////////Image Classification /////////////////////////////////////////


/** 
* Given a list of images, this function returns a Kmean clustering of the images using intensity histograms for classification.
*/
vector<vector< cv::Mat> > histoCluster(vector<cv::Mat>, int nbClusters);
#endif
