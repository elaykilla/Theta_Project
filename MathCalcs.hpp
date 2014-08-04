/*
* @author: ELay Maiga
* This class contains all the methods related to calculating coordinates and making projections
* from 2D to 3D or vice-versa.
*/




#ifndef MathCalcs
#define MathCalcs
/**
* Norm of a vector (O,u) with O the center of the coordinate system
*/
double norm(PointXYZRGB u){
	return sqrt(u.x*u.x + u.y*u.y + u.z*u.z);

}

/** Given a point defined by it's (x,y,z) cartesian coordinates, this functions returns it's spherical (r,theta,phi) coordinates 
*
*/
void cartesian2Spheric(PointXYZRGB p, double &r, double &theta, double &phi){
	r = sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
	theta = acos(p.z/r);
	phi = atan2(p.y,p.x);
}


/**
* Inverse of previous function
*/
void spheric2Cartesian(double r, double theta, double phi, PointXYZRGB &p){
	p.x = r*sin(theta)*cos(phi);
	p.y = r * sin(theta) * sin(phi);
	p.z = r * cos(theta);
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
void sphereCoordinates(int i, int j, double r, int rows, int cols, double &x, double &y, double &z)
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
void sphereCenter(double alpha, int i, double r, double &xc, double &yc){
	double alphaRad = (alpha/180)*PI;
	xc = r * cos(i*alphaRad);
	yc = r * sin(i*alphaRad);
}


/**

*/
void translateCenter(double xc, double yc, double &x, double &y){
	x = x - xc;
	y = y - yc;

}

/**
* This function, given a Point u (ux,uy,uz) and a direction vector v(vx,vy,vz) and a radius r
* returns the intersection point between the sphere and the line from u parallel to v
*/
PointXYZRGB project2Sphere(PointXYZRGB u, PointXYZRGB v, double r){
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
	
	if(dotp1>=0){
		return i2;
	}
	else{
		return i1;
	}
}


/** 
* This function, given a point u(ux,uy,uz) located inside the sphere and direction vector v, gives the Points Pmin, Pmax, Tmin and Tmax
* which correspond to the intersection between a horizontal line from u with the sphere
*/
void getPlane(PointXYZRGB u, PointXYZRGB v, double r, PointXYZRGB &xmin, PointXYZRGB &xmax, PointXYZRGB &ymin, PointXYZRGB &ymax){
	//Tmin and Tmax correspond to vertical min and max
	double theta,thetap,phi,phip, thetamin, thetamax, phimin,phimax;
	double h,h1;
	double r1 = r;
	cout <<"r:" << r;
	//theta = phi = PI;
	PointXYZRGB I = project2Sphere(u,v,r1);
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
	
	return;
}

#endif
