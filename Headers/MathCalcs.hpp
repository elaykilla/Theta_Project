/*
* @author: ELay Maiga
* This class contains all the methods related to calculating coordinates and making projections
* from 2D to 3D or vice-versa.
*/




#ifndef MATHCALCS_H
#define MATHCALCS_H

#include "standard_headers.hpp"
#include "cv_headers.hpp"
#include "pcl_headers.hpp"

//Define new types 
//typedef Vec<float, 9> Vec9f;

#include "boost_headers.hpp"
/*
* 
*/

//int round(double x){

//	return floor(x + 0.5);
//}


double findMin(float numbers[], int size);

double findMax(float numbers[], int size);

double norm(cv::Point2f p);

/**
* Norm of a vector (O,u) with O the center of the coordinate system
*/
double norm(PointXYZRGB u);

double dotProduct(PointXYZRGB u, PointXYZRGB v);


/**
* The cross product of u and v
*/
PointXYZRGB crossProduct(PointXYZRGB u, PointXYZRGB v);
/**
* returns weather or not u is in [a,b] 
*/
bool inInterval(double u, double a, double b);


/**
* returns weather or not an angle is between 2 angles 
*/
bool inBetweenAngles(double angle, double angle_min, double angle_max);



/** 
* Returns true if the 2 triangles, defined by 6 floats are the same
*/
bool sameTriangle(cv::Vec6f t1, cv::Vec6f t2);


/**
* This function returns the center of a 2D triangle defined by 3 vertices in (x,y) coordinates 
* or angular values (theta,phi)
*/
cv::Point2f triangleCenter(cv::Vec6f triangle);

/**
* Return the point at the center of the trianlge defined by 3 vertices in (x,y,z) coordinates
*/
PointXYZRGB triangleCenter3D(Vec9f triangle);


/**
* Return the length of the radius of the circumcircle of a given triangle defined by  (x,y)
*/
double triangleCircumRadius(cv::Vec6f triangle);

/**
* This function converts a triangle given by the coordinates of its vertices in (x,y,z) format, 
* to a triangle by the vertices in (phi,theta) spherical using Sacht's notations
*/
cv::Vec6f triangle2SphericSacht(int rows, int cols, double r,Vec9f triangle);


/**
* This function converts a triangle given by the vertices in (phi,theta) spherical format, 
* to a triangle by the coordinates of its vertices in (x,y,z) using Sacht's notations
*/
Vec9f triangle2CartesianSacht(int rows, int cols, Vec9f triangle);

/** 
* Calculate the area of a triangle
*/
double triangleArea(double x1,double y1,double x2,double y2,double x3,double y3);


/*
* Calculate the area of a triangle defined by 3 points in 3D 
*/
double triangleArea3D(PointXYZRGB p1,PointXYZRGB p2,PointXYZRGB p3 );
/** 
This function returns the area difference between 2 triangles
*/
double triangleDifference(cv::Vec6f t1, cv::Vec6f t2);
/** 
* Given a point and a triangle, this function verifies where the point is located inside the triangle using the areas of the 4 triangles 
*/
bool inTriangleArea(cv::Point2f p, cv::Vec6f triangle);

/** 
* Given a point and a triangle, this function verifies where the point is located inside the triangle
*/
bool inTriangle(cv::Point2f p, cv::Vec6f triangle);

/** 
* Given a point in 3D P(x,y,z) and a triangle also in 3D defined by P1,P2,P3 ,
* this function:
	- projects the point onto the plane defined by the 3 Points
	- verifies if the point lays within the triangle 
*/
bool inTriangle3D(PointXYZRGB p, Vec9f triangle3D);

/** 
* Given a point and a triangle, this function verifies where the point is located inside the triangle
*/
bool inTriangle(cv::Point2f p, cv::Vec6f triangle);

/**
* Given a point p and an affine transform tWarp where p is the position of a given pixel in 
* the interpolated image. This function returns the point on image 1 corresponding to the interpolated 
* position.
* P = (1-d)P' + dP'' where P' is the point in image 1, P'' in image 2 and P'' = tWarp*P' 
*/
cv::Point2f getInverseInterpPosition(cv::Point2f p, double dist, double pos, cv::Mat tWarp);

/** 
* returns the affine transformation between 2 triangles defined as a list of 6 floats
*/
cv::Mat getAffine2D(cv::Vec6f triangle1, cv::Vec6f triangle2);

/**
* Function returns interpolated triangles between 2 given triangles
*/
cv::Vec6f getInterpolatedTriangle(cv::Vec6f triangle1, cv::Vec6f triangle2, cv::Mat &affine, double dist, double pos);


/**
* This function given 2 triangles in (x,y,z) vertice coordinates, computes the interpolation between the 2 triangles (linearly) at a given position between 0 and dist;
*/
Vec9f getInterpolatedTriangle3D(Vec9f triangle1,Vec9f triangle2, double dist, double pos);


/**
* Sample points in a triangle defined by it's 3 Points in 3D. The Sampling is done depending on the size of the biggest vertex of the triangle
*/
vector<PointXYZRGB> sampleTriangle(Vec9f triangle);


/**
* Compute the affine 3*4 matrix between 2 triangles given with the 3D coordinates of each point
*/
cv::Mat getAffine3D(Vec9f t1, Vec9f t2);


/** 
* This function given a triangle, defined by pixel coordinates of the 3 points, in an 
* equirectangular image, computes the triangle in 3D defined by their 3D coordinates on the 
* surface of a sphere
*/
Vec9f get3DTriangleFrom2DTriangleSacht(int rows, int cols, cv::Vec6f triangle);

/** 
* This function given a set of triangles, defined by pixel coordinates of the 3 points, in an 
* equirectangular image, compues an array of the triangles in 3D defined by their 3D coordinates on the 
* surface of a spehre
*/
vector<Vec9f> get3DTrianglesFrom2DTriangles(int rows, int cols, vector<cv::Vec6f> triangles);

/** 
Rotate a point by an angle theta around the X axis
*/
void rotateX(PointXYZRGB &p, double alpha);


/** 
Rotate a point by an angle theta around the X axis
*/
void rotateY(PointXYZRGB &p, double phi);

/** 
Rotate a point by an angle theta around the X axis
*/
void rotateZ(PointXYZRGB &p, double theta);

/**
* Returns the euclidian distance between 2 points
*/
double distanceP(PointXYZRGB p1,PointXYZRGB p2);

/**
 * Given a list of points this function returns the mean point in 2D
 */
cv::Point meanPoint(vector<cv::Point> points);

/**
 * Given a list of points this function returns the mean point in 3D
 */
PointXYZRGB meanPoint(vector<PointXYZRGB> points);

/**
* This function given a point u (x,y,z) returns the (x,y) coordinates of the projection onto the XY plane of u
*/
void projectXY(PointXYZRGB u, double &x, double &y);



/**
* This function converts a pixel point (i,j) given in top left coordinates, to (i',j') center image coordinates 
*/
void toImageCenterCoordinate(double i, double j, int rows, int cols, double &ip, double &jp);


/**
* This function converts a pixel point (i,j) given in center image coordinates , to (i',j') center image coordinates top left coordinates
*/
void toImageTopLeftCoordinate(double i, double j, int rows, int cols, double &ip, double &jp);

/**
* This function given a point u (x,y,z) returns the (x,z) coordinates of the projection onto the XZ plane of u
*/
void projectXZ(PointXYZRGB u, double &x, double &z);

/** Given a point defined by it's (x,y,z) cartesian coordinates, this functions returns it's spherical (r,theta,phi) coordinates 
*
*/
void cartesian2Spheric(PointXYZRGB p, double r, double &theta, double &phi);


/**
* This function is the same as the previous one execpt using different conversion. The conversion if the one used in Sacht's Thesis
*/
void cartesian2SphericSacht(PointXYZRGB p, double r, double &theta, double &phi);


/**
* Inverse of previous function
*/
void spheric2Cartesian(double r, double theta, double phi, PointXYZRGB &p);


/**
* Inverse of previous function using notation from Sacht's Thesis
*/
void spheric2CartesianSacht(double r, double theta, double phi, PointXYZRGB &p);


/*Given a point (i,j) in a 2D image of Rows * Cols points, this function returns the coordinates of that point on 
* a Sphere of Radius r centered around (0,0)
* @INPUTS
* 	(i,j): the pixel coordinates of the point on the image
*	r: the radius of the sphere
*	rows: the height of the image
* 	cols: the width of the image
* @Outputs 
* 	(x,y,z) are the cartesian coordinates of the point on the surface of the sphere
*/
void sphereCoordinates(float i, float j, double r, int rows, int cols, double &x, double &y, double &z);


/*Given a point (i,j) in a 2D image of Rows * Cols points, this function returns the coordinates of that point on 
* a Sphere of Radius r centered around (0,0) considering image center coordinates
* @INPUTS
* 	(i,j): the pixel coordinates of the point on the image in center
*	r: the radius of the sphere
*	rows: the height of the image
* 	cols: the width of the image
* @Outputs 
* 	(x,y,z) are the cartesian coordinates of the point on the surface of the sphere
*/
void sphereCoordinatesSacht(float i, float j, double r, int rows, int cols, double &x, double &y, double &z);

/**
* Given a point's (i,j) coordinates on equirectangular image, this function returns the corresponding theta and phi angles
*/
void SphericFromPixelCoordinates(float i, float j, int rows, int cols, double &theta, double &phi);


/**
* Given a point's (i,j) coordinates on equirectangular image, this function returns the corresponding theta and phi angles using Sacht notation
*/
void SphericFromPixelCoordinatesSacht(float i, float j, int rows, int cols, double &theta, double &phi);

/*
* Applying sphereCoordinates to an array of points and returns a list of 3D points
*/
vector<PointXYZRGB> sphereCoordinatesList(int rows, int cols, vector<cv::Point2f> points);


/*
* Applying sphereCoordinates to an array of points and returns a list of 3D points 
* using Sacht notation
*/
vector<PointXYZRGB> sphereCoordinatesListSacht(int rows, int cols, vector<cv::Point2f> points);


/**
* This is the inverse of the previous functions. Given a point on the surface of the sphere, it gives its (i,j) pixel 
* coordinates
*/
void pixelCoordinates(double x, double y, double z, double r, int rows, int cols, int &i, int &j );


/**
* This is the inverse of the previous functions. Given a point on the surface of the sphere, it
* gives its (i,j) pixel coordinates using Sacht notation
* 
*/
void pixelCoordinatesSacht(double x, double y, double z, double r, int rows, int cols, int &i, int &j );


/**This functions returns 2 points of intersection between 
*	- the line passing by u parrallel to the the x axis
*	- the sphere centered at (0,0)
*/
void circularXcut(PointXYZRGB u, double r, PointXYZ Tmin,  PointXYZ Tmax);

/** 
* This functions returns 2 points of intersection between 
*	- the line passing by u parrallel to the the y axis
*	- the sphere centered at (0,0)
*/
void circularYcut(PointXYZRGB u, double r, PointXYZ Tmin, PointXYZ Tmax);

/**
* This functions returns 2 points of intersection between 
*	- the line passing by u parrallel to the the z axis
*	- the sphere centered at (0,0)
*/
void circularZcut(PointXYZRGB u, double r, PointXYZ Tmin, PointXYZ Tmax);



/**This function returns the center of the ith sphere when rotating around an angle alpha 
and radius r
*	@Input variables:
*	alpha: rotating angle in degrees
*	i: the number of rotation
*	r: the radius of the rotating cercle
*
*	@Output Variables:
*	xc: x coordinate of the center
*	yc: y coordinate of the center
*/
void sphereCenter(double alpha, int i, double r, double &xc, double &zc);

/**

*/
void translateCenter(double xc, double yc, double &x, double &y);

/**
* Find the orthogonal projection of a point p onto a plane defined by a point and it's normal vector
*/
PointXYZRGB orthogonalProjection2Plane(PointXYZRGB p, PointXYZRGB u, PointXYZRGB v);

/**
* Find the projection of a point p onto a plane defined by a point and it's normal vector. Project in the direction of
* a different vector. 
*/
PointXYZRGB nonOrthogonalProjection2Plane(PointXYZRGB p, PointXYZRGB u, PointXYZRGB v, PointXYZRGB v2);
/**
* This function, given a Point u (ux,uy,uz) and a direction vector v(vx,vy,vz) and a radius r
* returns the intersection point between the sphere and the line from u parallel to v
*/
PointXYZRGB project2Sphere(PointXYZRGB u, PointXYZRGB v, double r, bool &projected);
/**
* This function, given a Point u (ux,uy,uz) and a direction vector v(vx,vy,vz), and angle alpha compared to v and a radius r
* returns the intersection point between the sphere and the line from u parallel to ()v+alpha)
*/
PointXYZRGB project2SphereWithAngle(PointXYZRGB u, PointXYZRGB v, double alpha, double beta, double r, bool &projected);


/**
* Given 3 points in 3D, this function projects them to the surface of a sphere of radius r
*
*/
Vec9f projectTriangle2Sphere(Vec9f triangle, double r);

/** 
* This function, given a point u(ux,uy,uz) located inside the sphere and direction vector v, gives the Points Pmin, Pmax, Tmin and Tmax
* which correspond to the intersection between a horizontal line from u with the sphere
*/
void getPlane(PointXYZRGB u, PointXYZRGB v, double r, PointXYZRGB &xmin, PointXYZRGB &xmax, PointXYZRGB &ymin, PointXYZRGB &ymax);

/*
* This function given a point u and and vector v, returns a sampling of ps*ps points on the Plane perpendicular to v passing
* by u.
* 
*/

void samplePlane(PointXYZRGB u, PointXYZRGB v, vector<PointXYZRGB> &points , double radius, int ps);


/**
* This function, given an angle and the delta of angles between images (alpha) returns the closest image
* in that direction
*/
int closestImDirectionOrigin(double angle, double alpha);

/**
* This function, given a point u, a direction v, and a delta of angles (in degrees) alpha, returns which image position is
* closest to that of v
*/
int closestImDirection(PointXYZRGB u, double angle, double alpha, double r, double &newAngle );

/**
* Given a direction, a horizontal angel and a vertical angle, this function returns the maxium and minimum horizontal and vertical viewing angles from the origin
*/
void viewingLimitsOrigin(PointXYZRGB v, double v_angle, double h_angle, double &theta_min, double &theta_max, double &phi_min, double &phi_max);

/**
* Same as above function but viewing from a different point
*/
void viewingLimits(PointXYZRGB u, PointXYZRGB v, double v_angle, double h_angle, double &theta_min, double &theta_max, double &phi_min, double &phi_max);

/** 
* This function, given double values ip and jp interpolates the pixel values from floor(ip,jp) and ceil(ip,jp). This is bilinear * projection
*/
void pixelInterpolate(PointXYZRGB &u, int r, cv::Mat image);

/**
* Given a floating point in 2d, this function returns the bilinear interpolation of the pixel values * around
*/
cv::Vec3b bilinearInterpolate(cv::Mat image, double i, double j);
/**
* Given a point p, this function verifies wether or not this point is on the ray with a direction of v and passing through o.
*/
bool isOnRay(PointXYZRGB p, PointXYZRGB o, PointXYZRGB v);

/**
* This function returns true if a ray from o in the direction of v passes within a cube of lenght c of point p
* This function returns:
	0: if the ray does not pass near the point
	1: if the ray passes near the point in the same direction as v
	-1: if the ray passes near the point in the opposite direction of v
*/
bool isCloseToRayCube(PointXYZRGB p, PointXYZRGB o, PointXYZRGB v, double c);






#endif
