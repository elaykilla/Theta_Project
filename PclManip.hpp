/*
* @author: ELay Maiga
* This class contains all the methods related to calculating coordinates and making projections
* from 2D to 3D or vice-versa.
*/

#ifndef PclManip
#define PclMnip

#include"MathCalcs.hpp"
//#include "boost_headers.hpp"

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
*This function takes an input image in Equirectangular pixel coordinates (i,j) cv::Mat and returns 
* a Point cloud. The point cloud is the Spherical projection of this image onto a sphere of
* radius r and centered in (xc,yc);
* @Inputs
* Cv::cv::Mat ori: the input image
* double r : the radius of the sphere
* double xc and yc: the (x,y) coordinates of the center of the Sphere
*
*
* @Output
* Ptr cloud : a point cloud of the sphere
* Ptr VctCloud: A vector cloud of vectors (Sc,Pt) with Sc being the center of the sphere.
*/
PointCloud<PointXYZRGB>::Ptr EquiToSphere(cv::Mat ori, double radius, double xc, double yc){
	//We define our point cloud and a Point
	PointCloud<PointXYZRGB>::Ptr cloud (new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr vectorCloud (new PointCloud<PointXYZRGB>);
	
	
	PointXYZRGB iPoint;
	PointXYZRGB iVect;
	PointXYZRGB t;
	t.x = xc;
	t.y = yc;
	double normt = norm(t);
	
	//Math variables
	double theta,phi,x,y,z;
	//double radius = r;
	
	//Receive the size 
	cv::Size s = ori.size();
	int rows = s.height;
	int cols = s.width;
	
	//Color vector 
	cv::Vec3b colorOri;
	
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
			
			
//			
//			iPoint.x = x + xc;
//			iPoint.y = y + yc;
			//Knowing the Center, apply rotation and translation from world Center
			iPoint.x = ((x*xc/normt)-(y*yc/normt)); //+ xc;
			iPoint.y = ((x*yc/normt) + (y*xc/normt)); //+ yc;
			
//			if(rand()%10000<1){
//				cout << "xp: " << xp << endl;
//			}
			iPoint.z = z;
				
				
			//Put the coordinates of the vector directed from iPoint to Sc(xc,yc,0)
//			iVect.x = x;
//			iVect.y = y;
//			iVect.z = z;
				
			//Add pixel color values
			colorOri = ori.at<cv::Vec3b>(i, j);
		       	iPoint.b = iVect.b = colorOri[0];
			iPoint.g = iVect.g = colorOri[1];
			iPoint.r = iVect.r = colorOri[2];			
			cloud->points.push_back(iPoint);	
//			vectorCloud->points.push_back(iVect);		       						
		}
		
	}
	
	return cloud;
}

/**
*This function takes an input a PointCloud of points on a Sphere (not necessarily the full sphere) and returns a PointCloud of 
*those points with x= i and y = j being the pixel coordinates. It also takes the radius and original input image
* 
*/
 void sphere2Equi(cv::Mat ori, double r,PointCloud<PointXYZRGB>::Ptr spherePts, PointCloud<PointXYZRGB>::Ptr &result){
	double rows = ori.rows;
	double cols = ori.cols;
	
	PointXYZRGB po,pi;
	int i,j;
	
	cv::Vec3b colorOri;
	
	
	for(int n=0;n<spherePts->size();n++){
		po = spherePts->points[n];
		pixelCoordinates(po.x,po.y,po.z,r,rows,cols,i,j);
		pi.x = i;
		pi.y =j;
		//cout << "pi: " << pi << endl;
		colorOri = ori.at<cv::Vec3b>(i, j);
		pi.r =  colorOri[2];	
		pi.g =  colorOri[1];
		pi.b =  colorOri[0];
		result->push_back(pi);
	}	
}
/**
*This function takes an input image in Equirectangular pixel coordinates (i,j) and returns 
* a Point cloud. The point cloud is the Spherical projection of this image onto a sphere of
* radius r and centered in (xc,yc);
* @Inputs
* Cv::cv::Mat ori: the input image
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


/*
* Given a point u and a pointcloud cloud, this function returns the mean value of K nearest neighbors
*/

void kMeanValue(PointXYZRGB &u, PointCloud<PointXYZRGB>::Ptr &cloud, PointCloud<PointXYZRGB>::Ptr &outCloud, int K){
	//cout<< "cloud size in KmeanValue: " << cloud->size() << endl;
	double rt,gt,bt, xt, yt, zt;
	PointXYZRGB p;
	
	
	//PointCloud<PointXYZRGB>::Ptr sight (new PointCloud<PointXYZRGB>);
	//sight->points.resize(cloud->size());
//	for (size_t i = 0; i < cloud.points.size (); ++i)
//	{
//	    	//sight->points[i] = cloud->points[i];
//	    	sight->points.push_back(cloud.points[i]);
//	    	//cout << "cloud points numb: " << i << " coord: " << cloud->points[i] <<endl;
//	}
	
	
	//cloud->points.resize(cloud->width*cloud->height);
	//cloud->points.push_back(p);
	KdTreeFLANN<PointXYZRGB> kdtree;
	kdtree.setInputCloud (cloud);
	
	
	vector<int> pointIdxNKNSearch(K);
	vector<float> pointNKNSquaredDistance(K);
	if ( kdtree.nearestKSearch (u, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
		xt = yt = zt = 0;
		//cout << "pointIdxNKNSearch.size (): " << pointIdxNKNSearch.size () <<endl;
		for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i){
			p = cloud->points[ pointIdxNKNSearch[i] ];
			//cout << "pxb: " << p.x <<" pyb: " << p.y <<" pzb: " << p.z <<endl;
//	      		std::cout << "    "  <<   p.x 
//		        << " " << p.y 
//		        << " " << p.z 
//		        << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
		        
		        xt += p.x;
		        yt += p.y;
		        zt += p.z;
		        rt += p.r;
		        gt += p.g;
		        bt += p.b; 
	  	}	
	
	}
	
	int t = pointIdxNKNSearch.size();
	//cout << "size: " << t << endl; 
	//cout << "xt: " << xt <<" yt: " << yt <<" zt: " << zt <<endl;
	p.x = xt/t;
	p.y = yt/t;
	p.z = zt/t;
	p.r = rt/t;
	p.g = gt/t;
	p.b = bt/t;
	//cout << "px: " << p.x <<" py: " << p.y <<" pz: " << p.z <<endl;
	//cout << "--------------------------------------------------------" << endl;
	
	outCloud->points.push_back(p);
	return;
}


/**
* This function is intended to be used when multi-threading. 
* Given an Array of points, the id of the thread i , the total number of threads n, an input pointCloud and an Output pointCloud
* it first separates the array into n equal parts. Then it retrieves the KmeanValue for the part of the array that concerns the 
* thread in question.
* 
*/
void multiKMeanValue(vector<PointXYZRGB> points, int id, int nbThreads, PointCloud<PointXYZRGB>::Ptr cloud,PointCloud<PointXYZRGB>::Ptr &outCloud, int k ){

	int size = points.size();
	//cout<< "points size: "<< size << endl;
	int perT = (int)(size/nbThreads);
	//cout << "perT: " << perT << endl;
	int start = (id-1)*perT;
	//cout << "start: " << start << endl;
	int end;
	if(id<nbThreads){
		end = id*perT; 
	}
	else{
		end = size;
	}
	
	//cout<< "end: " << end << endl;
	for(int i = start; i<end;i++){
		cout << "i: " << i <<endl; 
		PointXYZRGB p = points[i];
		kMeanValue(p,cloud,outCloud,k);
	}
	
	return;
}

void testfunction(){
	cout << "test function called" << endl;
	return;
}
#endif
