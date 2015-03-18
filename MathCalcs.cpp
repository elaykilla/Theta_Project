/*
 * @author: ELay Maiga
 * This class contains all the methods related to calculating coordinates and making projections
 * from 2D to 3D or vice-versa.
 */

#include "MathCalcs.hpp"


//#include "pcl_headers.hpp";
/*
 * 
 */

//int round(double x){

//	return floor(x + 0.5);
//}
/**
 * Norm of a vector (O,u) with O the center of the coordinate system
 */
double norm(PointXYZRGB u){
	return sqrt(u.x*u.x + u.y*u.y + u.z*u.z);

}

double dotProduct(PointXYZRGB u, PointXYZRGB v){
	return (u.x * v.x + u.y*v.y + u.z * v.z);
}


/**
* The cross product of u and v
*/
PointXYZRGB crossProduct(PointXYZRGB u, PointXYZRGB v){

	PointXYZRGB w;

	w.x = u.y*v.z - u.z *v.y;
	w.y = u.z * v.x - u.x*v.z;
	w.z = u.x*v.y - u.y*v.x;
	
	return w;
}
/**
 * returns weather or not u is in [a,b] 
 */
bool inInterval(double u, double a, double b){
	double mint,maxt;

	mint = min(a,b);
	maxt = max(a,b);


	return (mint<=u & u<= maxt); 

}

bool inBetweenAngles(double angle, double min_angle, double max_angle){
	double tempAngle = fmod((fmod((max_angle-min_angle),2*PI)+2*PI), 2*PI);
	
	if(tempAngle>=PI){
		swap(min_angle,max_angle);
	}
	
	if (min_angle<=max_angle){
		return angle >= min_angle && angle <= max_angle;
	}
	else {
	
		return angle >= min_angle || angle <= max_angle;
	}
}

bool sameTriangle(cv::Vec6f t1, cv::Vec6f t){
//	cv::Point2f a1,b1,c1,a2,b2,c2;
//	
//	a1.x = t1[0];
//	a1.y = t1[1];	
//	b1.x = t1[2];
//	b1.y = t1[3];
//	c1.x = t1[4];
//	c1.y = t1[5];
//	
//	a2.x = t[0];
//	a2.y = t[1];	
//	b2.x = t[2];
//	b2.y = t[3];
//	c2.x = t[4];
//	c2.y = t[5];
	
	vector<float> t2;
	t2.push_back(t[0]);
	t2.push_back(t[1]);	
	t2.push_back(t[2]);
	t2.push_back(t[3]);
	t2.push_back(t[4]);
	t2.push_back(t[5]);
	
	
	bool inside = find(t2.begin(), t2.end(), t1[0]) != t2.end()
			& find(t2.begin(), t2.end(), t1[1]) != t2.end()
			& find(t2.begin(), t2.end(), t1[2]) != t2.end()
			& find(t2.begin(), t2.end(), t1[3]) != t2.end()
			& find(t2.begin(), t2.end(), t1[4]) != t2.end()
			& find(t2.begin(), t2.end(), t1[5]) != t2.end();
			
	return inside;

}



double triangleArea(double x1,double y1,double x2,double y2,double x3,double y3){

	return abs((x1*(y2-y3) + x2*(y3-y1)+ x3*(y1-y2))/2.0);
}


double triangleArea3D(PointXYZRGB p1,PointXYZRGB p2,PointXYZRGB p3 ){
	PointXYZRGB u,v;
	u.x = p2.x - p1.x ; 
	u.y = p2.y - p1.y ; 
	u.z = p2.z - p1.z ; 
	
	v.x = p3.x - p1.x ; 
	v.y = p3.y - p1.y ; 
	v.z = p3.z - p1.z ;
	
	return norm(crossProduct(u,v))/2;
	
}

double triangleDifference(cv::Vec6f t1, cv::Vec6f t2){
	//Points from the 1st triangle
	double x1,y1,x2,y2,x3,y3;
	
	//Points from the 2nd triangle
	double xp1,yp1,xp2,yp2,xp3,yp3;
	
	//triangle areas
	double area1,area2;
	
	x1 = t1[0];
	y1 = t1[1];
	x2 = t1[2];
	y2 = t1[3];
	x3 = t1[4];
	y3 = t1[5];
	
	xp1 = t2[0];
	yp1 = t2[1];
	xp2 = t2[2];
	yp2 = t2[3];
	xp3 = t2[4];
	yp3 = t2[5];
	
	area1 = triangleArea(x1,y1,x2,y2,x3,y3);
	area2 = triangleArea(xp1,yp1,xp2,yp2,xp3,yp3);
	
	return abs(area1 - area2);

}

bool inTriangleArea(cv::Point2f p, cv::Vec6f triangle){
	cv::Point2f p1,p2,p3; 
	double epsilon = 0.01;
	
	p1.x = triangle[0];
	p1.y = triangle[1];
	p2.x = triangle[2];
	p2.y = triangle[3];
	p3.x = triangle[4];
	p3.y = triangle[5];
	
	
	/* Calculate area of triangle ABC */
   float A = triangleArea (p1.x, p1.y, p2.x, p2.y, p3.x, p3.y);
 
   /* Calculate area of triangle PBC */  
   float A1 = triangleArea (p.x, p.y, p2.x, p2.y, p3.x, p3.y);
 
   /* Calculate area of triangle PAC */  
   float A2 = triangleArea (p1.x, p1.y, p.x, p.y, p3.x, p3.y);
 
   /* Calculate area of triangle PAB */   
   float A3 = triangleArea (p1.x, p1.y, p2.x, p2.y, p.x, p.y);
   
   /* Check if sum of A1, A2 and A3 is same as A */
   return (A1 + A2 + A3 - epsilon <= A && A <= A1 + A2 + A3 + epsilon);

}


bool inTriangle(cv::Point2f p, cv::Vec6f triangle){

	//Epsilon is used for points on the edges 
	double epsilon = 0.001;
	
	
	cv::Point2f p1,p2,p3; 
	p1.x = triangle[0];
	p1.y = triangle[1];
	p2.x = triangle[2];
	p2.y = triangle[3];
	p3.x = triangle[4];
	p3.y = triangle[5];
	
	double xmax = max(p1.x, max(p2.x,p3.x)) + epsilon;
	double ymax = max(p1.x, max(p2.y,p3.y)) + epsilon;
	double xmin = min(p1.x, min(p2.x,p3.x)) - epsilon;
	double ymin = min(p1.x, min(p2.y,p3.y)) - epsilon;
	
	if(p.x < xmin || p.y < ymin || p.x > xmax || p.y > ymax){
		return false;
	}
	
	else{
//		double denum = p1.x*(p2.y - p3.y) + p1.x*(p3.x-p2.x) + p2.x*p3.y - p2.y*p3.x;
//		double t1 = (p.x*(p3.y - p1.x) + p.y*(p1.x - p3.x) - p1.x*p3.y + p1.x*p3.x) / denum;
//		double t2 = (p.x*(p2.y - p1.x) + p.y*(p1.x - p2.x) - p1.x*p2.y + p1.x*p2.x) / denum;
//  		double s = t1 + t2;
  		
		double denum = ((p2.y - p3.y)*(p1.x - p3.x) + (p3.x - p2.x)*(p1.y - p3.y));		
		double t1 =  ((p2.y - p3.y)*(p.x - p3.x) + (p3.x - p2.x)*(p.y - p3.y))/ denum;
		double t2 = ((p3.y - p1.y)*(p.x - p3.x) + (p1.x - p3.x)*(p.y - p3.y)) / denum;
  		double s = t1 + t2;

  		//return 0<= t1 && t1 <= 1 && 0<= t2 && t2<= 1 && s <=1;
  		return 0<= t1 && 0<= t2  && s <=1;
	}
	
}


/** 
* Given a point in 3D P(x,y,z) and a triangle also in 3D defined by P1,P2,P3 ,
* this function:
	- projects the point P onto the plane defined by the 3 Points
	- verifies if the point lays within the triangle 
*/
bool inTriangle3D(PointXYZRGB p, Vec9f triangle3D){
	//Get triangle points
	PointXYZRGB p1,p2,p3;
	
	p1.x = triangle3D[0];
	p1.y = triangle3D[1];
	p1.z = triangle3D[2];
	
	p2.x = triangle3D[3];
	p2.y = triangle3D[4];
	p2.z = triangle3D[5];
	
	p3.x = triangle3D[6];
	p3.y = triangle3D[7];
	p3.z = triangle3D[8];

	//Get the normal vector to the plane
	// u = P1 --> P2
	// v = P1 --> P3
	// w normal vector of the plane formed by P1,P2, and P3
	PointXYZRGB u, v, w; 
	u.x = p2.x - p1.x ; 
	u.y = p2.y - p1.y ; 
	u.z = p2.z - p1.z ; 
	
	v.x = p3.x - p1.x ; 
	v.y = p3.y - p1.y ; 
	v.z = p3.z - p1.z ;
	
	//The cross product of u and v
	w.x = u.y*v.z - u.z *v.y;
	w.y = u.z * v.x - u.x*v.z;
	w.z = u.x*v.y - u.y*v.x;
	
	//Get the projection of P onto the plane (P1,w);
	PointXYZRGB pproj = orthogonalProjection2Plane(p,p1,w);
	
	
	//Create vectors pP1, pP2, pP3 which are vectors from each triangle vertex, 
	// to the projected point on the triangle plane
	PointXYZRGB pP1, pP2, pP3;
	pP1.x =  p1.x - pproj.x ; 
	pP1.y =  p1.y - pproj.y ;
	pP1.z =  p1.z - pproj.z ;
	
	pP2.x = + p2.x - pproj.x ; 
	pP2.y = + p2.y - pproj.y ;
	pP2.z = + p2.z - pproj.z ;
	
	pP3.x = + p3.x - pproj.x ; 
	pP3.y = + p3.y - pproj.y ;
	pP3.z = + p3.z - pproj.z ;
	
	//Compare the surfaces of the 3 sub triangles: 
	//T1:(P1,P,P2), T2:(P1,P,P3), T3:(P2,P,P3) with that of T:(P1,P2,P3) 
	double A1,A2,A3,A;
	//T1, T2, T3
	A1 = triangleArea3D(p1,pproj,p2);
	A2 = triangleArea3D(p1,pproj,p3);
	A3 = triangleArea3D(p2,pproj,p3);
	
	A = norm(crossProduct(u,v))/2.;
	
	//We define epsilon as a small variation equal to about 5% of the mean length of arrays
	// We then verify that the sum of the aires is 
	// equal to the big air at epsilon difference
	
	double epsilon = 5*(norm(u)+norm(v))/200;
	double aSum = A1+A2+A3;
	return inInterval(aSum, A-epsilon, A+epsilon);
	

}

/**
* Compute the affine 3*4 matrix between 2 triangles given with the 3D coordinates of each point
*/
cv::Mat getAffine3D(Vec9f t1, Vec9f t2){
	//Points from 1st triangle
	float x1,x2,x3,y1,y2,y3,z1,z2,z3;
	
	//Matrices
	cv::Mat tri_map1(4,3,CV_32FC1), tri_map2(4,3,CV_32FC1);
	cv::Mat tri_map1_t(3,4,CV_32FC1);
	cv::Mat t1_t1t(4,4,CV_32FC1), t2_t1t(4,4,CV_32FC1), t1_t1t_invert(4,4,CV_32FC1),affine_transform(4,4,CV_32FC1);

	x1 = t1[0];
	y1 = t1[1];
	z1 = t1[2];
	x2 = t1[3];
	y2 = t1[4];
	z2 = t1[5];
	x3 = t1[6];
	y3 = t1[7];
	z3 = t1[8];


	//Points from 2nd triangle
	float xp1,xp2,xp3,yp1,yp2,yp3,zp1,zp2,zp3;	
	xp1 = t2[0];
	yp1  = t2[1];
	zp1 = t2[2];
	xp2 = t2[3];
	yp2 = t2[4];
	zp2 = t2[5];
	xp3 = t2[6];
	yp3 = t2[7];
	zp3 = t2[8];
	
	
	
	//Fill Tmap1
	float* r0 = tri_map1.ptr<float>(0);
	float* r1 = tri_map1.ptr<float>(1);
	float* r2 = tri_map1.ptr<float>(2);
	float* r3 = tri_map1.ptr<float>(3);
	
	r0[0] = x1;
	r0[1] = x2;
	r0[2] = x3;
	
	r1[0] = y1;
	r1[1] = y2;
	r1[2] = y3;
	
	r2[0] = z1;
	r2[1] = z2;
	r2[2] = z3;
	
	r3[0] = 1;
	r3[1] = 1;
	r3[2] = 1;
	
	
	//Fill Tmap2
	float* rt0 = tri_map2.ptr<float>(0);
	float* rt1 = tri_map2.ptr<float>(1);
	float* rt2 = tri_map2.ptr<float>(2);
	float* rt3 = tri_map2.ptr<float>(3);
	
	rt0[0] = xp1;
	rt0[1] = xp2;
	rt0[2] = xp3;
	
	rt1[0] = yp1;
	rt1[1] = yp2;
	rt1[2] = yp3;
	
	rt2[0] = zp1;
	rt2[1] = zp2;
	rt2[2] = zp3;
	
	rt3[0] = 0;
	rt3[1] = 0;
	rt3[2] = 1;
	
	//Get the transpose of T_map1
	cv::transpose(tri_map1, tri_map1_t);
	
	//Get T1*T1t and T2*T1t which are square matrices
	t1_t1t = tri_map1 * tri_map1_t;
	t2_t1t = tri_map2 * tri_map1_t;
	
	cv::invert(t1_t1t, t1_t1t_invert);
	
	affine_transform = t2_t1t * t1_t1t_invert;
	
	cout<< "T1:" << tri_map1 << endl;
	cout<< "T1_transpose:" << tri_map1_t << endl;
	cout<< "T1*T1_transpose:" << t1_t1t << endl;
	cout<< "Inverse T1*T1_transpose:" << t1_t1t_invert << endl;
	cout<< "T1*T1_transpose * Inverse T1*T1_transpose:" << t1_t1t * t1_t1t_invert << endl;
	
	cout<< "Affine:" << affine_transform << endl;
	
	cout<< "T2:" << tri_map2 << endl;
	cout << "Verification: affine*t1:" << affine_transform * tri_map1 << endl;
	
	cout<< "T2*T1_transpose:" << t2_t1t << endl;
	cout << "Verification: affine*t1*t1_transpose:" << affine_transform * t1_t1t << endl;
	
	
	return affine_transform;
	
	
	

}


/********************************************************
* Compute 3D position of triangles from 2D triangles
*********************************************************/
vector<Vec9f> get3DTrianglesFrom2DTriangles(int rows, int cols, vector<cv::Vec6f> triangles){
	vector<Vec9f> td_triangles;
	
	for(int i=0;i<triangles.size();i++){
		cv::Vec6f triangle = triangles[i];
		//Get the 3 points making the triangle
		//cv::Point2f a,b,c; 
		//a.x = triangle[0];
		//a.y = triangle[1];
		//b.x = triangle[2];
		//b.y = triangle[3];
		//c.x = triangle[4];
		//c.y = triangle[5];	
		
		//Compute 3D positions
		Vec9f triangle3D;
		
		//point coordinates
		double x1,y1,z1,x2,y2,z2,x3,y3,z3;
		//For the first point
		sphereCoordinates(triangle[0], triangle[1], 
		1, rows, cols, 
		x1,y1,z1);
		
		triangle3D[0] = x1;
		triangle3D[1] = x2;
		triangle3D[2] = x3;
		
		//Second point
		sphereCoordinates(triangle[2], triangle[3], 
		1, rows, cols, 
		x2,y2,z2);
		
		triangle3D[3] = x2;
		triangle3D[4] = y2;
		triangle3D[5] = z2;
		
		//Third point
		sphereCoordinates(triangle[4], triangle[5], 
		1, rows, cols,
		x3,y3,z3);
		 
		triangle3D[6] = x3;
		triangle3D[7] = y3;
		triangle3D[8] = z3;
		
		//Add the new 3D triangle
		td_triangles.push_back(triangle3D);
	}

	return td_triangles;
}





/**
* Sample points in a triangle defined by it's 3 Points in 3D. The Sampling is done depending on the size of the biggest vertex of the triangle
*/
vector<PointXYZRGB> sampleTriangle(Vec9f triangle){
	PointXYZ a,b,c,temp ; 
	vector<PointXYZRGB> points; 
	double abnorm, acnorm;
	
	a.x = triangle[1];
	a.y = triangle[2];
	a.z = triangle[3];
	b.x = triangle[4];
	b.y = triangle[5];
	b.z = triangle[6];
	c.x = triangle[7];
	c.y = triangle[8];
	c.z = triangle[9];
	
	//Calculate norm of ab
	temp.x = b.x - a.x;
	temp.y = b.y - a.y;
	temp.z = b.z - a.z;
	
	//abnorm = norm(temp);

	//Calculate norm of ac
	temp.x = c.x - a.x;
	temp.y = c.y - a.y;
	temp.z = c.z - a.z;
	//acnorm = norm(temp);
	
	return points;
	
}


void rotateX(PointXYZRGB &p, double alpha)
{

	double x,y,z;
	x = p.x;
	y = p.y;
	z = p.z;
	
	p.y = cos(alpha)*y - sin(alpha)*z;
	p.z = sin(alpha)*y + cos(alpha)*z;
}


void rotateY(PointXYZRGB &p, double phi)
{

	double x,y,z;
	x = p.x;
	y = p.y;
	z = p.z;
	
	p.x = cos(phi)*x + sin(phi)*z;
	p.z = -sin(phi)*x + cos(phi)*z;
}

void rotateZ(PointXYZRGB &p, double theta)
{

	double x,y,z;
	x = p.x;
	y = p.y;
	z = p.z;
	
	p.x = cos(theta)*x - sin(theta)*y;
	p.y = sin(theta)*x + cos(theta)*y;
}
/**
 * Returns the euclidian distanc between 2 points
 */
double distanceP(PointXYZRGB p1,PointXYZRGB p2){
	double dx,dy,dz;
	dx = p1.x-p2.x;
	dy = p1.y-p2.y;
	dz = p1.z-p2.z;
	return sqrt(dx*dx + dy*dy + dz*dz);
}

/**
 * Given a list of points this function returns the mean point between a list of points in 2D
 */
cv::Point meanPoint(vector<cv::Point> points){
	int s = points.size();
	cv:: Point p;
	if(s==0){
		return p;
	} 
	else{
		
		double spx, spy;
		for(int i=0;i<s;i++){
			p = points[i];
			spx += p.x;
			spy += p.y;
		}

		p.x = spx/s;
		p.y = spy/s; 
		return p;
	}
}


/**
 * Given a list of points this function returns the mean point between a list of points in 3D
 */
PointXYZRGB meanPoint(vector<PointXYZRGB> points){
	int s = points.size();
	PointXYZRGB p;
	if(s==0){
		cout << "MeanPoint on empty lists returns (0,0,0)" << endl;
		return p;
	} 
	else{
		double spx, spy, spz;
		for(int i=0;i<s;i++){
			p = points[i];
			spx += p.x;
			spy += p.y;
			spz += p.z;
		}

		p.x = spx/s;
		p.y = spy/s; 
		p.z = spz/s;
		return p;
	}
}

/**
 * This function given a point u (x,y,z) returns the (x,y) coordinates of the projection onto the XY plane of u
 */
void projectXY(PointXYZRGB u, double &x, double &y){
	double phi;
	//Phi is the angle between OUvect and the X axis
	phi = atan2(u.y,u.x);
	x = cos(phi);
	y = sin(phi);
}

/**
 * This function given a point u (x,y,z) returns the (x,z) coordinates of the projection onto the XZ plane of u
 */
void projectXZ(PointXYZRGB u, double &x, double &z){
	double phi;
	//Phi is the angle between OUvect and the X axis
	phi = atan2(u.z,u.x);
	x = cos(phi);
	z = sin(phi);
}


/** Given a point defined by it's (x,y,z) cartesian coordinates, this functions returns it's spherical (r,theta,phi) coordinates 
 *
 */
void cartesian2Spheric(PointXYZRGB p, double r, double &theta, double &phi){
	r = sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
	phi = acos(p.z/r);
	theta = atan2(p.y,p.x);
//	
	if(phi>PI) phi -= 2*PI;
	if (theta>PI) theta -= 2*PI;
}


/**
 * Inverse of previous function
 */
void spheric2Cartesian(double r, double theta, double phi, PointXYZRGB &p){
	p.x = r*sin(phi)*cos(theta);
	p.y = r * sin(theta) * sin(phi);
	p.z = r * cos(phi);
}






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
void sphereCoordinates(float i, float j, double r, int rows, int cols, double &x, double &y, double &z)
{
	//Convert from (i,j) pixel values to (theta,phi) angle values 
	double theta,phi;
	theta = i * PI/rows ;
	phi = j * 2*PI/cols;

	//Convert from (theta,phi) geographic values to (x,y,z) cartesian values
	x = r * sin(theta) * cos(phi);
	y = r * sin(theta) * sin(phi);
	z = r * cos(theta);
}


/*
* Applying sphereCoordinates to an array of points and returns a list of 3D points
*/
vector<PointXYZRGB> sphereCoordinatesList(int rows, int cols, vector<cv::Point2f> points){
	//Radius
	double r = 1.;
	
	//list of 3D points
	vector<PointXYZRGB> points3D;
	
	//3D point
	
	
	
	//Image point in 2D
	cv::Point2f p;
	int i,j;
	
	for(int k=0;k<points.size();k++){
		//cout << "Point number: " << k << endl;
		double x,y,z;
		PointXYZRGB np;
		p = points[k];
		i = p.y;
		j = p.x;
		
		//cout << "Sphere Coordinates for: (i,j): (" << i << "," << j << ")" << endl; 
		sphereCoordinates(i,j,r,rows,cols,x,y,z);
		np.x = x;
		np.y = y;
		np.z = z;
		
		//np.b = img.at<cv::Vec3b>(i,j)[0];
		//np.g = img.at<cv::Vec3b>(i,j)[1];
		//np.r = img.at<cv::Vec3b>(i,j)[2];
		
		points3D.push_back(np);
	}
	
	return points3D;

}

/**
 * This is the inverse of the previous functions. Given a point on the surface of the sphere, it gives its (i,j) pixel 
 * coordinates
 */
void pixelCoordinates(double x, double y, double z, double r, int rows, int cols, int &i, int &j )
{
	//Convert (x,y,z) cartesian values from to (theta,phi) geographic values
	double theta,phi;
	theta = acos(z/r);
	phi = atan2(y,x);


	//Convert from  (theta,phi) angle values to (i,j) pixel values  
	i  = theta * rows/PI;
	j = phi * cols/(2*PI);
}

/**This functions returns 2 points of intersection between 
 *	- the line passing by u parrallel to the the x axis
 *	- the sphere centered at (0,0)
 */
void circularXcut(PointXYZRGB u, double r, PointXYZ Tmin,  PointXYZ Tmax)
{
	double ux,uy,uz,tx,ty,tz;

	//Get the coordinates of u
	uy = ty = u.y;
	ux = u.x;
	uz = tz = u.z;

	//Calcule the coordinates of ty
	tx = sqrt(r - ty*ty - tz*tz);

	//Set the Points coordinates

	Tmin.y = Tmax.y = ty;
	Tmin.z = Tmax.z = tz;

	Tmin.x = tx*(-1);
	Tmax.x = tx;
}

/** 
 * This functions returns 2 points of intersection between 
 *	- the line passing by u parrallel to the the y axis
 *	- the sphere centered at (0,0)
 */
void circularYcut(PointXYZRGB u, double r, PointXYZ Tmin, PointXYZ Tmax)
{
	double ux,uy,uz,tx,ty,tz;

	//Get the coordinates of u
	ux = tx = u.x;
	uy = u.y;
	uz = tz = u.z;

	//Calcule the coordinates of ty
	ty = sqrt(r - tx*tx - tz*tz);

	//Set the Points coordinates

	Tmin.x = Tmax.x = tx;
	Tmin.z = Tmax.z = tz;

	Tmin.y = ty*(-1);
	Tmax.y = ty;
}

/**
 * This functions returns 2 points of intersection between 
 *	- the line passing by u parrallel to the the z axis
 *	- the sphere centered at (0,0)
 */
void circularZcut(PointXYZRGB u, double r, PointXYZ Tmin, PointXYZ Tmax)
{
	double ux,uy,uz,tx,ty,tz;

	//Get the coordinates of u
	ux = tx = u.x;
	uz = u.z;
	uy = ty = u.y;

	//Calcule the coordinates of ty
	tz = sqrt(r - tx*tx - ty*ty);

	//Set the Points coordinates

	Tmin.x = Tmax.x = tx;
	Tmin.y = Tmax.y = ty;

	Tmin.z = tz*(-1);
	Tmax.z = tz;
}



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
void sphereCenter(double alpha, int i, double r, double &xc, double &zc){
	double alphaRad = (-alpha/180)*PI;
	xc = r * cos(i*alphaRad);
	zc = r * sin(i*alphaRad);
}


/**

 */
void translateCenter(double xc, double yc, double &x, double &y){
	x = x - xc;
	y = y - yc;

}


/**
 * Find the orthogonal projection of a point p onto a plane defined by a point u and it's normal vector v
 */
PointXYZRGB orthogonalProjection2Plane(PointXYZRGB p, PointXYZRGB u, PointXYZRGB v){
	//Define the equation of the plane ax + by + cz = d;
	PointXYZRGB pro;
	double a,b,c,d;
	a = v.x;
	b = v.y;
	c = v.z;
	d = a*u.x + b*u.y + c*u.z;


	//Define the line passing by p and parallele to v
	double r = norm(v)*norm(v);
	double A = d - (a*p.x + b*p.y + c*p.z);
	double k = A/r;

	pro.x = k*a + p.x;
	pro.y = k*b + p.y;
	pro.z = k*c + p.z;
	pro.r = p.r;
	pro.g = p.g;
	pro.b = p.b;

	return pro;
}

/**
 * Find the projection of a point p onto a plane defined by a point and it's normal vector n. Project in the direction of
 * a different vector. 
 */
PointXYZRGB nonOrthogonalProjection2Plane(PointXYZRGB p,PointXYZRGB v, PointXYZRGB u, PointXYZRGB n){
	//Define the equation of the plane ax + by + cz = d;
	PointXYZRGB pro;
	
	if(norm(v)==0){
		cout << "NonOrthogonalProjection: Impossible to project in the direction of a null vector please verify" << endl;
		return pro;
	}
	else if(norm(n)==0){
		cout << "NonOrthogonalProjection: Impossible to define a plane with a null vector" << endl;
		return pro;
	}
	else{
	double a,b,c,d;
	a = n.x;
	b = n.y;
	c = n.z;
	d = a*u.x + b*u.y + c*u.z;

	v.x /= norm(v);
	v.y /= norm(v);
	v.z /= norm(v);
	
	//Define the line passing by p and parallele to v2
	// The line passing by p in v direction can be parametrized by 
	//x = kvx + px, 
	//y = kvy + py and 
	//z = kvy + pz
	// We can then inject this in the equation for the plan ax + by + cz = d and get k
	double D = a*v.x + b*v.y + c*v.z;
	double A = d- (a*p.x + b*p.y + c*p.z);
	double k = A/D;

	pro.x = k*v.x + p.x;
	pro.y = k*v.y + p.y;
	pro.z = k*v.z + p.z;
	pro.r = p.r;
	pro.g = p.g;
	pro.b = p.b;

	return pro;
	}
}

/**
 * This function, given a Point u (ux,uy,uz) and a direction vector v(vx,vy,vz) and a radius r
 * returns the intersection point between the sphere and the line from u parallel to v
 */
PointXYZRGB project2Sphere(PointXYZRGB u, PointXYZRGB v, double r, bool &projected){
	//Impact points with the sphere. There are always 2 impact points. 1 in the right direction and the other in
	//the opposite direction direction
	PointXYZRGB i1,i2;


	//cout << "u: "<< u.x << "," << u.y << "," << u.z << endl;
	//cout << "u^2: "<< u.x*u.x << "," << u.y*u.y << "," << u.z*u.z << endl;
	//For future and safe coding we could verify that the point u is inside the sphere by veryfying that 
	//x^2 + x^2 + z^2 < 1;
	double a,b,c;
	double t1,t2;
	double deltap,dotp1,dotp2;

	//cout << "norm u: " << norm(u);
	//cout << " radius in project2Sphere: " << r <<endl;
	if(norm(u)<r){
		a = v.x*v.x + v.y*v.y + v.z*v.z;
		b = u.x*v.x + u.y*v.y + u.z*v.z;
		c = u.x*u.x + u.y*u.y + u.z*u.z - r*r ;
		//cout<< "a,b,c: " << a << "," << b <<"," << c << endl;
		deltap = b*b - a*c;
		//cout<< "delta Prime: " << deltap << endl;


		t1 = (-b + sqrt(deltap))/a;
		t2 = (-b - sqrt(deltap))/a;
		//cout<< "t1, t2: " << t1 << "," << t2 << endl;


		//Calculate both points
		i1.x = v.x*t1 + u.x; 
		i1.y = v.y*t1 + u.y;
		i1.z = v.z*t1 + u.z;

		i2.x = v.x*t2 + u.x; 
		i2.y = v.y*t2 + u.y;
		i2.z = v.z*t2 + u.z;

		//Calculate dot product to see which one is in the direction of v
		dotp1 = i1.x*v.x + i1.y*v.y + i1.z*v.z;
		dotp1 = i2.x*v.x + i2.y*v.y + i2.z*v.z;

		projected = true;
		if(dotp1>=0){
			return i2;
		}
		else{
			return i1;
		}
	}

	//If the point is not within the sphere then there is no inner projection of this point
	else{
		//cout << "Point not within the sphere" << endl;
		projected = false;
		return u;

	}
}
/**
 * This function, given a Point u (ux,uy,uz) and a direction vector v(vx,vy,vz), and angle alpha compared to v and a radius r
 * returns the intersection point between the sphere and the line from u parallel to ()v+alpha)
 */
PointXYZRGB project2SphereWithAngle(PointXYZRGB u, PointXYZRGB v, double alpha, double beta, double r, bool &projected){

	PointXYZRGB p;
	double alpharad = alpha*PI/180;
	double betarad = beta*PI/180;
	double theta,phi;

	cartesian2Spheric(v, r, theta, phi);
	//New direction Vector
	PointXYZRGB nv;
	nv.x = cos(alpharad-theta)*norm(v);
	nv.y = sin(alpharad-theta)*norm(v);
	nv.z = cos(beta-phi)*norm(v);

	//cout << "radius in project with angle: " << r << endl;
	p = project2Sphere(u, nv, r, projected);
	//cout << "p: " << p << endl;
	if(!projected){

		//wcerr << "The Point u is not within the sphere. Please verify" << endl;
		return p;
	}

	else{
		return p;
	}
}


/** 
 * This function, given a point u(ux,uy,uz) located inside the sphere and direction vector v, gives the Points Pmin, Pmax, Tmin and Tmax
 * which correspond to the intersection between a horizontal line from u with the sphere
 */
void getPlane(PointXYZRGB u, PointXYZRGB v, double r, PointXYZRGB &xmin, PointXYZRGB &xmax, PointXYZRGB &ymin, PointXYZRGB &ymax){
	//BOOST_LOG_TRIVIAL(trace) << "getPlane Function Has begun" <<endl;
	//Tmin and Tmax correspond to vertical min and max
	double theta,thetap,phi,phip, thetamin, thetamax, phimin,phimax;
	double h,h1;
	double r1 = r;
	bool projected;
	cout <<"r:" << r;
	//theta = phi = PI;
	PointXYZRGB I = project2Sphere(u,v,r1,projected);
	cout << "I" << I;
	cartesian2Spheric(I,r1,theta,phi);
	cout<< "u spherical: " << "(" << r1 << "," <<  theta << "," << phi << ")" <<endl;

	h = r;
	cout<< "h:" << h <<endl;
	h1 = norm(u);
	cout << "h1: " << h1 << endl;


	thetap = atan(h1*tan(theta)/h);
	phip = thetap;

	thetamin = theta - thetap/2;
	thetamax = theta + thetap/2;
	//cout << "thetamin"
	phimin = phi - thetap/2;
	phimax = phi + thetap/2;

	spheric2Cartesian(r,thetamin,0,xmin);
	spheric2Cartesian(r,thetamax,0,xmax);

	spheric2Cartesian(r,0,phimin,ymin);
	spheric2Cartesian(r,0,phimax,ymax);

	//BOOST_LOG_TRIVIAL(trace) << "getPlane Function Has ended" <<endl;
	return;

}

/*
 * This function given a point u and and vector v, returns a sampling of ps*ps points on the Plane perpendicular to v passing
 * by u.
 * 
 */

void samplePlane(PointXYZRGB u, PointXYZRGB v, vector<PointXYZRGB> &points , double radius, int ps){
	//BOOST_LOG_TRIVIAL(trace) << "Sampleplane Function Has begun" <<endl;
	//int r = radius;
	double x,y,z;
	double a,b,c,d;
	PointXYZRGB p;
	p.b = 255;

	a = v.x;
	b = v.y;
	c = v.z;
	d = a*u.x + b*u.y + c*u.z;
	cout << "(a,b,c,d): " << a << ","<< b << "," << c << "," << d << endl;

	int nbpoints = ps*ps;
	//vector<PointXYZRGB> points(nbpoints);
	points.resize(nbpoints);
	//	cout << "nb points total: " << nbpoints << endl;
	//	cout << "points.size in samplePlane: " << points.size() <<endl;

	if(a==0 & b==0 & c==0){

		wcerr << "SamplePlane cannot be called with a null v vector" <<endl;
		//BOOST_LOG_TRIVIAL(error) << "samplePlane cannot be called with null v vector" <<endl;
		//return points;
	}
	else{
		if(c!=0){
			cout << "c!=0" << endl;
			for(int i=0;i<nbpoints;i++){
				p.x = (rand()%10000/10000.)* radius * (pow(-1,rand()%2));
				p.y = (rand()%10000/10000.)* radius * (pow(-1,rand()%2));
				p.z = (d - a*p.x - b*p.y)/c;
				//cout << "rand: " << rand()%10000/10000. <<endl;
				//cout << "(a,b,c,d): " << a << ","<< b << "," << c << "," << d << endl;
				//cout << "right p: " << p << endl;
				points[i] = p;
			}
		}

		else if(b!=0){
			cout << "c=0 & b!=0" << endl;
			for(int i=0;i<nbpoints;i++){
				p.x = (rand()%10000/10000.)* radius * (pow(-1,rand()%2));
				p.y = (d - a*x - b*y)/b;
				p.z = (rand()%10000/10000.)* radius * (pow(-1,rand()%2));
				points[i] = p;
			}
		}
		else{
			cout << "c=b=0 & a!=0" << endl;
			for(int i=0;i<nbpoints;i++){
				p.x = (d - b*y - c*z)/a;
				p.y = (rand()%10000/10000.)* radius * (pow(-1,rand()%2));
				p.z = (rand()%10000/10000.) * radius * (pow(-1,rand()%2));
				points[i] = p;
			}
		}
	}
	//BOOST_LOG_TRIVIAL(trace) << "samplePlane Function Has ended" <<endl;
	//return points;
}






/**
 * This function given an angle gives from origin, gives closest image direction
 * ATTENTION: angle is given in rad and alpha in degrees
 */
int closestImDirectionOrigin(double angle, double alpha ){

	int i;
	double alpharad = alpha*PI/180;

	while(angle<0){
		angle += 2*PI;
	}

	i = round(angle/alpharad);
	return i % 24;
}

/**
 * This function, given a point u, a direction v, and a delta of angles (in degrees) alpha, returns which image position is
 * closest to that of v
 */
int closestImDirection(PointXYZRGB u, double angle, double alpha, double r, double &newAngle){
	PointXYZRGB v;
	PointXYZRGB p;
	bool projected;
	double anglerad = PI/2 - angle*PI/180 ;
	double alpharad = alpha*PI/180;

	//Get the direction vector with the given angle
	spheric2Cartesian(r,anglerad,0,v);

	//Project u to sphere in v direction
	p = project2Sphere(u,v,r,projected);

	if(!projected){
		cout << "Could not Project point to Sphere, please verify u:" << endl;
		return -1;
	}

	else{
		//cout << "Project u to: " << p << endl;
		double theta, phi;
		cartesian2Spheric(p,r,theta,phi);

		if(p.x > 0) {
			theta = PI/2 - theta;
		}
		else{
			theta = theta + PI/2;
		}
		//cout << "Angle from Origin after projection: " << theta*180/PI << endl;
		newAngle = theta*180/PI;
		return closestImDirectionOrigin(theta,alpha);
	}
}
void viewingLimitsOrigin(PointXYZRGB v, double v_angle, double h_angle, double &theta_min, double &theta_max, double &phi_min, double &phi_max){
	//Angles of the direction
	double theta, phi;
	//double theta_min,theta_max, phi_min, phi_max;
	double r;
	cartesian2Spheric(v,r,theta,phi);
	//cout << "v spheric (theta,phi): " << theta << "," << phi << endl; 
	//Convert to radian
	double v_angle_rad = v_angle*PI/180;
	double h_angle_rad = h_angle*PI/180;

	
	theta_min = theta - v_angle_rad/2;
	theta_max = theta + v_angle_rad/2;
	phi_min = phi - h_angle_rad/2;
	phi_max = phi + h_angle_rad/2;
	
	if(theta_min<(-PI)) theta_min += 2*PI;
	if(theta_min>PI) theta_min -=2*PI;
	if(theta_max<(-PI)) theta_max += 2*PI;
	if(theta_max>PI) theta_max -=2*PI;

	if(phi_min<(-PI)) phi_min += 2*PI;
	if(phi_min>PI) phi_min -=2*PI;

	if(phi_max<(-PI)) phi_max += 2*PI;
	if(phi_max>PI) phi_max -=2*PI;
}

void viewingLimits(PointXYZRGB u, PointXYZRGB v, double v_angle, double h_angle, double &theta_min, double &theta_max, double &phi_min, double &phi_max){
	//Angles of the direction
	double theta, phi,thetau,phiu;
	//double theta_min,theta_max, phi_min, phi_max;
	double r;
	cartesian2Spheric(u,r,thetau,phiu);
	cartesian2Spheric(v,r,theta,phi);
	//cout << "v spheric (theta,phi): " << theta << "," << phi << endl; 
	//Convert to radian
	double v_angle_rad = v_angle*PI/180;
	double h_angle_rad = h_angle*PI/180;

	theta_min = theta + thetau - v_angle_rad;
	theta_max = theta + thetau + v_angle_rad;
	phi_min = phi + phiu - h_angle_rad;
	phi_max = phi + phiu + h_angle_rad;

}

/** 
 * This function, given double values ip and jp interpolates the pixel values from floor(ip,jp) and ceil(ip,jp). This is bilinear * projection
 */
void pixelInterpolate(PointXYZRGB &u, int r, cv::Mat image){
	double i,j;
	int imin,imax,jmin,jmax;
	int rij1,rij2,rij3,rij4,bij1,bij2,bij3,bij4,gij1,gij2,gij3,gij4;
	double rtemp1,rtemp2,btemp1,btemp2,gtemp1,gtemp2;
	//vector<int [2]> points;
	//int point [2];
	cv::Vec3b color1,color2,color3,color4;

	//Convert (x,y,z) cartesian values from to (theta,phi) geographic values
	double theta,phi;
	theta = acos(u.z/r);
	phi = atan2(u.y,u.x);


	//Convert from  (theta,phi) angle values to (i,j) pixel values  
	i  = theta * image.rows/PI;
	j = phi * image.cols/(2*PI);

	//Pixel interpolation points on image
	imin = floor(i);
	imax = ceil(i);
	jmin = floor(j);
	jmax = ceil(j);

	//Pixel R,B,G values at each of those 4 points
	color1 = image.at<cv::Vec3b>(imin, jmin);
	color2 = image.at<cv::Vec3b>(imin, jmax);
	color3 = image.at<cv::Vec3b>(imax, jmin);
	color4 = image.at<cv::Vec3b>(imax, jmax);

	bij1 = color1[0];
	gij1 = color1[1];
	rij1 = color1[2];

	bij2 = color2[0];
	gij2 = color2[1];
	rij2 = color2[2];

	bij3 = color3[0];
	gij3 = color3[1];
	rij3 = color3[2];

	bij4 = color4[0];
	gij4 = color4[1];
	rij4 = color4[2];

	btemp1 = (imax-i)*bij1 + (i-imin)*bij2;
	gtemp1 = (imax-i)*gij1 + (i-imin)*gij2;
	rtemp1 = (imax-i)*rij1 + (i-imin)*rij2;

	btemp2 = (imax-i)*bij3 + (i-imin)*bij4;
	gtemp2 = (imax-i)*gij3 + (i-imin)*gij4;
	rtemp2 = (imax-i)*rij3 + (i-imin)*rij4;

	u.b = (jmax-j)*btemp1 + (j-jmin)*btemp2;
	u.g = (jmax-j)*gtemp1 + (j-jmin)*gtemp2;
	u.r = (jmax-j)*rtemp1 + (j-jmin)*rtemp2;
}

/**
 * Given a point p, this function verifies wether or not this point is on the ray with a direction of v and passing through o.
 */
bool isOnRay(PointXYZRGB p, PointXYZRGB o, PointXYZRGB v){
	double A,B,C;

	if(v.x== 0 && v.y== 0 && v.z== 0){
		wcerr << "There is no such ray with a direction of 0. Please verify v vector" << endl;
		return false;
	}
	else{
		if(v.x!= 0 && v.y!= 0 && v.z!= 0){
			A = (p.x - o.x)/v.x;
			B = (p.y - o.y)/v.y;
			C = (p.z - o.z)/v.z;
			return (A==B && B==C);
		}
		else {
			if(v.x==0){
				if(v.y ==0){
					return (p.x == o.x & p.y == o.y);
				}
				else if (v.z==0){
					return (p.x == o.x & p.z == o.z);
				}
				else{
					B = (p.y - o.y)/v.y;
					C = (p.z - o.z)/v.z;
					return (p.x == o.x & B==C );
				}
			}
			else if (v.y ==0){
				if(v.z == 0){
					return (p.y == o.y & p.z == o.z);
				}
				else{
					A = (p.x - o.x)/v.x;
					C = (p.z - o.z)/v.z;
					return (p.y== o.y & A==C); 
				}
			}
			//Case z ==0 
			else{
				A = (p.x - o.x)/v.x;
				B = (p.y - o.y)/v.y;
				return (p.z == o.z && A==B);
			}
		}
	}
}

/**
 * This function returns true if a ray from o in the direction of v passes within a cube of lenght c of point p
 * This function returns:
0: if the ray does not pass near the point
1: if the ray passes near the point in the same direction as v
-1: if the ray passes near the point in the opposite direction of v
 */
bool isCloseToRayCube(PointXYZRGB p, PointXYZRGB o, PointXYZRGB v, double c){
	//cout << "p: " << p << endl;
	double t;
	double xinf,xsup,yinf,ysup,zinf,zsup;
	bool truex, truey,truez;
	double dotp = dotProduct(p,v);
	if(v.x== 0 && v.y== 0 && v.z== 0){
		wcerr << "There is no such ray with a direction of 0. Please verify v vector" << endl;
		return false;
	}
	else{
		//cout << "c: " << c << endl;
		if(v.x!=0){
			t = (p.x - o.x)/v.x;
		}
		else if (v.y!=0){
			t = (p.y - o.y)/v.y;
		}
		else{
			t = (p.z - o.z)/v.z;
		}

		xinf = v.x*t + o.x - c;
		yinf = v.y*t + o.y - c;
		zinf = v.z*t + o.z - c;
		xsup = v.x*t + o.x + c;
		ysup = v.y*t + o.y + c;
		zsup = v.z*t + o.z + c;
		//cout << "v: " << v << " Dot Product: " << dotp << endl;
		//cout << "xinf: " << xinf << " xsup: " << xsup << endl;
		//cout << "yinf: " << yinf << " ysup: " << ysup << endl;
		//cout << "zinf: " << zinf << " zsup: " << zsup << endl;
		truex = inInterval(p.x,xinf,xsup);
		truey = inInterval(p.y,yinf,ysup);
		truez = inInterval(p.z,zinf,zsup);

		return (truex & truey& truez);// & dotp>=0);
	}
}

