/*
* @author: ELay Maiga
* This class contains all the methods related to calculating coordinates and making projections
* from 2D to 3D or vice-versa.
*/

#ifndef PclManip
#define PclMnip

#include"MathCalcs.hpp"
#include"OcvManip.hpp"
//#include "boost_headers.hpp"


/***************************************************Image Manipulation PCL***********************************************/
/**
* This function computes the keypoints on a cloud of PointXYZRGB. 
*******************FLANN compatibility problem between OpenCv and PCL ***************************
*/

void get3DKepoints(PointCloud<PointXYZRGB>::Ptr &points, float min_scale, int nr_octaves, int nr_scales_per_octave, float min_contrast, PointCloud<PointWithScale>::Ptr &keypoints_out);


/**
*This function takes an input image in Equirectangular pixel coordinates (i,j) cv::Mat and returns 
* a Point cloud. The point cloud is the Spherical projection of this image onto a sphere of
* radius r and centered in (xc,zc);
* @Inputs
* Cv::cv::Mat ori: the input image
* double r : the radius of the sphere
* double xc and zc: the (x,z) coordinates of the center of the Sphere
*
*
* @Output
* Ptr cloud : a point cloud of the sphere
* Ptr VctCloud: A vector cloud of vectors (Sc,Pt) with Sc being the center of the sphere.
*/
PointCloud<PointXYZRGB>::Ptr EquiToSphere(cv::Mat ori, double radius, double xc, double yc, double zc);

/**

*/
void sphereToEqui(PointCloud<PointXYZRGB>::Ptr sphere, double r, int rows, int cols, cv::Mat &image);

/**
*This function takes an input a PointCloud of points on a Sphere (not necessarily the full sphere) and returns a PointCloud of 
*those points with x= i and y = j being the pixel coordinates. It also takes the radius and original input image
* 
*/
void sphere2Equi(cv::Mat ori, double r,PointCloud<PointXYZRGB>::Ptr spherePts, PointCloud<PointXYZRGB>::Ptr &result);


/**
* Returns line.size() points on the line between o and u
*/
void hPointLine(PointXYZRGB o, PointXYZRGB u, vector<PointXYZRGB> &line);


/*
* Given a point u and a pointcloud cloud, this function returns the mean value of K nearest neighbors
*/
void kMeanValue(PointXYZRGB &u, PointCloud<PointXYZRGB>::Ptr &cloud, PointCloud<PointXYZRGB>::Ptr &outCloud, int K);


/**
* This function is intended to be used when multi-threading. 
* Given an Array of points, the id of the thread i , the total number of threads n, an input pointCloud and an Output pointCloud
* it first separates the array into n equal parts. Then it retrieves the KmeanValue for the part of the array that concerns the 
* thread in question.
* 
*/
void multiKMeanValue(vector<PointXYZRGB> points, int id, int nbThreads, PointCloud<PointXYZRGB>::Ptr cloud,PointCloud<PointXYZRGB>::Ptr &outCloud, int k );


void matchingSphere(PointCloud<PointXYZRGB>::Ptr firstCloud, PointCloud<PointXYZRGB>::Ptr secondCloud);

//Given 2 point clouds representing spheric images taken from a distance d, this function interpolates and returns the cloud for a position betwee 0 and d
PointCloud<PointXYZRGB>::Ptr sphereInterpolate(cv::Mat image1, cv::Mat image2, double dist, double pos);


/** 
* Given a point cloud on a plane, representing an image, this function creates the CV mat corresponding to the given height and width
*/
cv::Mat imageFromPcPlane(PointCloud<PointXYZRGB>::Ptr cloud, cv::Mat ori, int height, int width);
//void testfunction(){
//	cout << "test function called" << endl;
//	return;
//}



#endif
