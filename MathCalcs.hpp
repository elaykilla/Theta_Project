/*
* @author: ELay Maiga
* This class contains all the methods related to calculating coordinates and making projections
* from 2D to 3D or vice-versa.
*/




#ifndef MathCalcs
#define MathCalcs

#include "standard_headers.hpp"
#include "cv_headers.hpp"
#include "pcl_headers.hpp"
//#include "boost_headers.hpp"
/*
* 
*/

//int round(double x){

//	return floor(x + 0.5);
//}
/**
* Norm of a vector (O,u) with O the center of the coordinate system
*/
double norm(PointXYZRGB u);


double dotProduct(PointXYZRGB u, PointXYZRGB v);

/**
* returns weather or not u is in [a,b] 
*/
bool inInterval(double u, double a, double b);


/**
* returns weather or not an angle is between 2 angles 
*/
bool inBetweenAngles(double angle, double angle_min, double angle_max);

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
* This function given a point u (x,y,z) returns the (x,z) coordinates of the projection onto the XZ plane of u
*/
void projectXZ(PointXYZRGB u, double &x, double &z);

/** Given a point defined by it's (x,y,z) cartesian coordinates, this functions returns it's spherical (r,theta,phi) coordinates 
*
*/
void cartesian2Spheric(PointXYZRGB p, double r, double &theta, double &phi);

/**
* Inverse of previous function
*/
void spheric2Cartesian(double r, double theta, double phi, PointXYZRGB &p);



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
void sphereCoordinates(int i, int j, double r, int rows, int cols, double &x, double &y, double &z);
/**
* This is the inverse of the previous functions. Given a point on the surface of the sphere, it gives its (i,j) pixel 
* coordinates
*/
void pixelCoordinates(double x, double y, double z, double r, int rows, int cols, int &i, int &j );

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
