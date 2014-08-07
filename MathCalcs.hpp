/*
* @author: ELay Maiga
* This class contains all the methods related to calculating coordinates and making projections
* from 2D to 3D or vice-versa.
*/

//#include "boost_headers.hpp"


#ifndef MathCalcs
#define MathCalcs


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

/**
* This function, given a point u, a direction v, and a delta of angles (in degrees) alpha, returns which image position is
* closest to that of v
*/
int closestImDirection(PointXYZRGB u, PointXYZRGB v, double alpha ){
	double x,y,xv,yv,vnorm,snorm, alpharad, alpha_prime;
	PointXYZRGB vn, uv, su;
	int i;
	
	alpharad = alpha*PI/180;
	cout<< "alpha: " << alpharad << endl;
	//su = project2Sphere(u,v,r);
	
	snorm = norm(su);
	vnorm = norm(v);
	
	uv.x = v.x + u.x;
	uv.y = v.y + u.y;
	uv.z = v.z + u.z;


	//Project u and v onto XY place
	projectXY(u,x,y);
	projectXY(v,xv,yv);
	
//	cout << "v.x: " << v.x << endl;
//	cout << "v.y: " << v.y << endl;
//	cout << "xv: " << xv << endl;
	//Alpha prime is the angle between the vector u->v and the X axis
	alpha_prime = acos(xv/norm(v)); 
	cout<< "alphaPrime: " << alpha_prime <<endl;
	
	
	i = round(alpha_prime/alpharad);
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
		projected = false;
		return u;
	
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
* by u
* TO FINNISH IMPLEMENTING
*/

void samplePlane(PointXYZRGB u, PointXYZRGB v, vector<PointXYZRGB> &points , double radius, int ps){
	BOOST_LOG_TRIVIAL(trace) << "Sampleplane Function Has begun" <<endl;
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
		BOOST_LOG_TRIVIAL(error) << "samplePlane cannot be called with null v vector" <<endl;
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
	BOOST_LOG_TRIVIAL(trace) << "samplePlane Function Has ended" <<endl;
	//return points;
}

/** 
* This function, given double values ip and jp interpolates the pixel values from floor(ip,jp) and ceil(ip,jp) 
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


#endif
