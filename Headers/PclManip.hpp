/*
* @author: ELay Maiga
* This class contains all the methods related to calculating coordinates and making projections
* from 2D to 3D or vice-versa.
*/

#ifndef _PCLMANIP
#define _PCLMANIP

#include"MathCalcs.hpp"
#include"OcvManip.hpp"
//#include "boost_headers.hpp"


//class PclManip{

//public:
/***************************************************Simple Functions **************************************/

/**
* Function returns the index of a given point in an array, if not found it gives the size of the 
* Vector
* @param p 	a PointXYZRGB 
* @param points a vector of PointXYZRGB 
* @return 	The index of the point in the array, or the size of the vector if not found
* 
*/
int findPointInVector(PointXYZRGB p, vector<PointXYZRGB> points);

/**
* Simple function to convert a list of PointXYZRGB to a list of Point3D
* @param points a vector of PointXYZRGB 
* @return a vector of cv::Point3D 
*/
vector<cv::Point3d> fromxyzrgbtoPoint3D(vector<PointXYZRGB> points);


/**
* Simple function to convert a list of Point3D  to a list of PointXYZRGB
* @param points vector of cv::Point3D 
* @return a vector PointXYZRGB 
*/
vector<PointXYZRGB> fromPoint3Dtoxyzrgb(vector<cv::Point3d> points);


/***************************************************End Simple Functions ******************/

/***************************************************Image Manipulation PCL***********************************************/
/**
* This function computes the keypoints on a cloud of PointXYZRGB. 
* @param 
* @param 
* @param 
*/
/*******************FLANN compatibility problem between OpenCv and PCL ***************************/
void get3DKepoints(PointCloud<PointXYZRGB>::Ptr &points, float min_scale, int nr_octaves, int nr_scales_per_octave, float min_contrast, PointCloud<PointWithScale>::Ptr &keypoints_out);



/**
*This function takes an input image in Equirectangular pixel coordinates (i,j) cv::Mat and returns 
* a Point cloud. The point cloud is the Spherical projection of this image onto a sphere of
* radius r and centered in (xc,zc);
* @Input
* Cv::cv::Mat ori: the input image
* @Input
* double r : the radius of the sphere
* @Input
* double xc and zc: the (x,z) coordinates of the center of the Sphere
*
*
* @Output
* Ptr cloud : a point cloud of the sphere
* @Output
* Ptr VctCloud: A vector cloud of vectors (Sc,Pt) with Sc being the center of the sphere.
*/
PointCloud<PointXYZRGB>::Ptr EquiToSphere(cv::Mat ori, double radius, double xc, double yc, double zc);

/**
* This function is the inverse of the previous function. Given an image represented by a sphere in 3D, returns the image in equirectangular format.
*/
void sphereToEqui(PointCloud<PointXYZRGB>::Ptr sphere, double r, int rows, int cols, cv::Mat &image);

/**
*This function takes an input a PointCloud of points on a Sphere (not necessarily the full sphere) and returns a PointCloud of 
*those points with x= i and y = j being the pixel coordinates. It also takes the radius and original input image
* 
*/
void sphere2Equi(cv::Mat ori, double r,PointCloud<PointXYZRGB>::Ptr spherePts, PointCloud<PointXYZRGB>::Ptr &result);



/**
* This function, given a triangle in 3D and a cloud of points, returns the points which projected onto the * plane defined by the triangle, are inside the triangle
*/
PointCloud<PointXYZRGB>::Ptr getTriangleContent3D(PointCloud<PointXYZRGB>::Ptr sphere, Vec9f triangle );

/**
* This function takes a 2 point clouds and 2 triangles as input, and returns a Point cloud of the interpolated triangle between the 2 given triangles
*/
PointCloud<PointXYZRGB>::Ptr singleTriangleInterpolate3D(PointCloud<PointXYZRGB>::Ptr sphere1,PointCloud<PointXYZRGB>::Ptr sphere2,double r,int rows, int cols, double dist, double pos, Vec9f triangle1, Vec9f triangle2 );

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

/** 
* Given a set of 3D points, this functions creates a triangle mesh without any upsampling. 
* However the sphere is not closed and this uses the ball pivoting algorithm
*/
GreedyProjectionTriangulation<PointXYZRGBNormal> get3DTriangulation(PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<Normal>::Ptr normals, double max_dist);

/**
* Using triangles from first sphere, and matched points, make corresponding triangels on second image
*/
void makeCorrespondingDelaunayTriangles3D(vector<PointXYZRGB> points3D1, vector<PointXYZRGB> points3D2, vector<Vec9f> &triangles3D1, vector<Vec9f> &triangles3D2); 


/**
* Interpolate directly using 3D points and 3D triangles on sphere
*/

cv::Mat delaunaySphereInterpolateFromTriangles(cv::Mat img1, cv::Mat img2, double dist, double pos, string triangles_file, vector<PointXYZRGB> points1c, vector<PointXYZRGB> points2c);

//};
#endif
