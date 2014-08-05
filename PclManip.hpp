/*
* @author: ELay Maiga
* This class contains all the methods related to calculating coordinates and making projections
* from 2D to 3D or vice-versa.
*/

#ifndef PclManip
#define PclMnip

#include"MathCalcs.hpp"

/***************************************************Image Manipulation PCL***********************************************/
/**
* This function computes the keypoints on a cloud of PointXYZRGB. 
*******************FLANN compatibility problem between OpenCv and PCL ***************************
*/

//PointCloud<PointWithScale>::Ptr get3DKepoints(PointCloud<PointXYZRGB>::Ptr points, float min_scale, int nr_octaves, int nr_scales_per_octave, float min_contrast) {

//	//the extracted keypoints
//	PointCloud<PointWithScale>::Ptr keypoints_out;

//	SIFTKeypoint<PointXYZRGB, PointWithScale> sift_detect;

//	// Use a FLANN-based KdTree to perform neighborhood searches
//	search::KdTree<PointXYZRGB>::Ptr tree(new search::KdTree<PointXYZRGB> ());
//	sift_detect.setSearchMethod(tree);
//	
//	// Set the detection parameters
//	sift_detect.setScales(min_scale, nr_octaves, nr_scales_per_octave);
//	sift_detect.setMinimumContrast(min_contrast);
//	
//	// Set the input
//	sift_detect.setInputCloud(points);
//	
//	// Detect the keypoints and store them in "keypoints_out"
//	sift_detect.compute(*keypoints_out);
//	
//	return keypoints_out;
//}

/**
*This function takes an input image in Equirectangular pixel coordinates (i,j) Mat and returns 
* a Point cloud. The point cloud is the Spherical projection of this image onto a sphere of
* radius r and centered in (xc,yc);
* @Inputs
* Cv::Mat ori: the input image
* double r : the radius of the sphere
* double xc and yc: the (x,y) coordinates of the center of the Sphere
*
*
* @Output
* Ptr cloud : a point cloud of the sphere
* Ptr VctCloud: A vector cloud of vectors (Sc,Pt) with Sc being the center of the sphere.
*/
PointCloud<PointXYZRGB>::Ptr EquiToSphere(Mat ori, double r, double xc, double yc){
	//We define our point cloud and a Point
	PointCloud<PointXYZRGB>::Ptr cloud (new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr vectorCloud (new PointCloud<PointXYZRGB>);
	
	
	PointXYZRGB iPoint;
	PointXYZRGB iVect;
	
	//Math variables
	double theta,phi,x,y,z;
	double radius = r;
	
	//Receive the size 
	Size s = ori.size();
	int rows = s.height;
	int cols = s.width;
	
	//Color vector 
	Vec3b colorOri;
	
	//For each pixel value we calculate it's new cooridnates and assign the pixel value to those
	//coordinates
	for (int i = 0; i < rows; i++)
	{
		//j corresponds to the x and hence runs through the columns = width
		for (int j = 0; j < cols; j++)
		{
			//Get the Spherical Coordinates on a sphere or radius centered in (0,0)
			sphereCoordinates(i,j,radius,rows,cols,x,y,z);
			
			//Put the coordinates in a PointXYZ and shift center to (xc,yc,0)
			iPoint.x = x + xc;
			iPoint.y = y + yc;
			iPoint.z = z;
				
				
			//Put the coordinates of the vector directed from iPoint to Sc(xc,yc,0)
			iVect.x = x;
			iVect.y = y;
			iVect.z = z;
				
			//Add pixel color values
			colorOri = ori.at<Vec3b>(i, j);
		       	iPoint.b = iVect.b = colorOri[0];
			iPoint.g = iVect.g = colorOri[1];
			iPoint.r = iVect.r = colorOri[2];			
			cloud->points.push_back(iPoint);	
			vectorCloud->points.push_back(iVect);		       						
		}
		
	}
	
	return cloud;
}
/**
*This function takes an input image in Equirectangular pixel coordinates (i,j) and returns 
* a Point cloud. The point cloud is the Spherical projection of this image onto a sphere of
* radius r and centered in (xc,yc);
* @Inputs
* Cv::Mat ori: the input image
* double r : the radius of the sphere
*
* @Output
* Ptr cloud : a point cloud of the sphere
*/

/************************************************ Cloud Viewer Call back functions **************************************/
void hPointLine(PointXYZRGB o, PointXYZRGB u, vector<PointXYZRGB> &line)
{

	//line.push_back(o);
	//line.push_back(u);
//	cout << "hPointLine function called" << endl;
//	
	//vector<PointXYZRGB> line(nbPts);
	int nbPts = line.size();
	double distmin =0;
	double step = abs(u.x - o.x)/nbPts;
	double distmax = abs(u.x - o.x);
	//cout << dist << endl;
	
	PointXYZ v;
	v.x = (u.x - o.x)/nbPts ;
	v.y = (u.y - o.y)/nbPts ;
	//cout << "vy: "<< v.y << endl;
	v.z = (u.z - o.z)/nbPts ;
	
	PointXYZRGB p;
	p.x = o.x;
	p.y = o.y;
	p.z = o.z;
	p.r = u.r;
	p.g = u.g;
	p.b = u.b;
	
	int i =0;
	while(i<nbPts){
		//cout << "hPointLine function called: " << i << endl;
		line.push_back(p);
		//cout << p.x << "," <<  p.y << ","  << p.z << endl;
		p.x += v.x;
		p.y += v.y;
		//cout << "py: " << p.y << endl;
		p.z += v.z;
		
//		PointXYZRGB np;
//		np.x = p.x;
//		np.y = p.y;
//		np.z = p.z;
//		np.r = p.r;
//		np.g = p.g;
//		np.b = p.g;
			
		//line[i]  = p;
		i++;
		//cloud->points.push_back(p);
		distmin += step;
	}
	
	//return line;
}


#endif
