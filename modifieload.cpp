#include "standard_headers.hpp"
#include "cv_headers.hpp"
#include "pcl_headers.hpp"


/***************************************************Math Calculations***********************************************/
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
void circularXcut(PointXYZ u, double r, PointXYZ Tmin,  PointXYZ Tmax)
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
void circularYcut(PointXYZ u, double r, PointXYZ Tmin, PointXYZ Tmax)
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
void circularZcut(PointXYZ u, double r, PointXYZ Tmin, PointXYZ Tmax)
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
* This function, given a point u(ux,uy,uz) located inside the sphere, gives the Points Pmin, Pmax, Tmin and Tmax
* which correspond to the intersection between a horizontal line 
*/


/***************************************************Image Manipulation***********************************************/

/**
* This function takes the image prefix name, adds the position i and saves in a Mat
*
*
*/
void loadImagei(string name, int i, Mat &image){
	ostringstream file;
	file << name <<i << ".jpg" ; 
	image = imread(file.str(),CV_LOAD_IMAGE_COLOR);
}

/**
* This function returns a cv::vector containing the Keypoints from the input image using SURF
*/
vector<KeyPoint> get2DKeypoints(Mat image){
//-- Step 1: Detect the keypoints using SURF Detector
  int minHessian = 400;
  
  SurfFeatureDetector detector( minHessian );
  vector<KeyPoint> keypoints;
  
  detector.detect( image, keypoints );
  
  return keypoints;
}

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
*This function takes an input image in Equirectangular pixel coordinates (i,j) and returns 
* a Point cloud. The point cloud is the Spherical projection of this image onto a sphere of
* radius r
* @Inputs
* Cv::Mat ori: the input image
* double r : the radius of the sphere
*
* @Output
* Ptr cloud : a point cloud of the sphere
*/
PointCloud<PointXYZRGB>::Ptr EquiToSphere(Mat ori, double r){
	//We define our point cloud and a Point
	PointCloud<PointXYZRGB>::Ptr cloud (new PointCloud<PointXYZRGB>);
	PointXYZRGB iPoint;
	
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
			
			sphereCoordinates(i,j,radius,rows,cols,x,y,z);
			//Put the coordinates in a PointXYZ
			iPoint.x = x;
			iPoint.y = y;
			iPoint.z = z;
				
			//Add pixel color values
			colorOri = ori.at<Vec3b>(i, j);
		       	iPoint.b = colorOri[0];
			iPoint.g = colorOri[1];
			iPoint.r = colorOri[2];			
			cloud->points.push_back(iPoint);			       						
		}
		
	}
	
	return cloud;
	
}



/***************************************************  Main  ***********************************************/
int main(int argc, char** argv)
{
	/*for first code*/
	//Math variables
	double theta,phi,x,y,z;
	
	//b,g,r color values of input image
	float b,g,r;
	
	//empty color vector
	Vec3f color(0.0,0.0,0.0);
	
	double radius = 1;
	double phi0 = 0.0;
	double phi1 = 2*PI;
	double theta0 = 0.0;
	double theta1 = PI;
	
	
	Vec3b colorOri;
	/**
	Second code variables
	*/
	double R;
	double X, Y, Z;
	
	
	//Pcl point cloud for sphere
	PointCloud<PointXYZRGB>::Ptr cloud (new PointCloud<PointXYZRGB>);
	PointXYZRGB iPoint;
	
	//Setup 3D visualizer
//	boost::shared_ptr<visualization::PCLVisualizer> viewer(new visualization::PCLVisualizer("3D viewer"));
//	viewer->setBackgroundColor (0, 0, 0);

	visualization::CloudViewer cViewer ("Sphere points");
  
	
	//Loading images
	Mat ori1, ori2, ori3;
	string name = argv[1];
	loadImagei(name,1,ori1);
//	loadImagei(name,3,ori2);
//	loadImagei(name,4,ori3);
	cvNamedWindow("Original",0);
	cvNamedWindow("Keypoints 2D",0);
	
	
	
//	if(!ori1.data || !ori2.data){
//		cout << "Input image was not loaded, please verify: " << argv[1] << endl;
//		
//		return -1;
//	}
	
	
	//recover mat props
	Size s = ori1.size();
	int rows = s.height;
	int cols = s.width;
	
	cvResizeWindow("Original",rows,cols);
	imshow("Original",ori1);
	
	double rectH = rows/180;
	double rectW = cols/360;
	
	// Mat for spherical image
	Mat sph = Mat::zeros(rows, cols, CV_8UC3);
	
	int count=1;
	//i corresponds to the y and hence runs through the rows = height 
	Mat ori;
	int cx,cy;
	
	//Test simple 1 image
	ori  = ori1;
//	for(int k=0; k<count; k++){
//		if(k==0){
//			ori = ori1;
//			cx=1;
//			cy=0;
//		}
//		else if(k==1){
//			ori = ori2;
//			cx = 0.5;
//			cy = 0.5;
//		}
//		else if(k==2){
//			ori = ori3;
//			cx = 0;
//			cy = -1;
//		}
		for (int i = 0; i < rows; i++)
		{
			//j corresponds to the x and hence runs through the columns = width
			for (int j = 0; j < cols; j++)
			{
				//Convert from (i,j) pixel values to (theta,phi) angle values 
				theta = i * theta1/rows ;
				phi = j * phi1/cols;
			
				//Convert from (theta,phi) geographic values to (x,y,z) cartesian values
				x = radius * sin(theta) * cos(phi);
				y = radius * sin(theta) * sin(phi);
				z = radius * cos(theta);



				//Test circular cut
				PointXYZ u;
				u.x = 0.5;
				u.y = 0.5;
				u.z = 0.5;
				
				PointXYZ Tmin, Tmax;
				circularYcut(u,radius,Tmin,Tmax);
				
				//Add Pixel Color
//				colorOri = ori.at<Vec3b>(i, j);
//				iPoint.b = colorOri[0];
//				iPoint.g = colorOri[1];
//				iPoint.r = colorOri[2];			
//				cloud->points.push_back(iPoint);
				
				if(true){
				//	if(y+cy>u.y){
					//Put the coordinates in a PointXYZ
					iPoint.x = x + cx;
					iPoint.y = y + cy;
					iPoint.z = z;
				
					//Add pixel color values
					colorOri = ori.at<Vec3b>(i, j);
				       	iPoint.b = colorOri[0];
					iPoint.g = colorOri[1];
					iPoint.r = colorOri[2];			
					cloud->points.push_back(iPoint);
					//cViewer.addLine(u,iPoint,"line");	
					//}
				}
//				else{
//					iPoint.x = x + cx - 0.25;
//					iPoint.y = y;
//					iPoint.z = z;
//				}		       						
			} // End of columns j loop
		
		} //End of rows i loop
	
//	} //End of loop for cycling through images
	//Display point cloud 
	cloud->width=50;
	cloud->height=50;
	cViewer.showCloud (cloud);
	
//	viewer->addPointCloud<PointXYZRGB>(cloud, "Sphere");
//  	viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Sphere");
//	viewer->addCoordinateSystem (10.0);
//  	viewer->initCameraParameters ();
	
//	while(!viewer->wasStopped()){
//		viewer->spinOnce(100);
//		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
//	}
	
	//close viewer
//	viewer->close();
	//get keypoints on 2D image 
//	cvResizeWindow("Keypoints 2D",rows,cols);
//	vector<KeyPoint> keypoints;
//	keypoints = get2DKeypoints(ori);
	
	//drawKeypoints( ori, keypoints, ori, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
	//imshow("Keypoints 2D" , ori);
	
	
	waitKey(0);

	return 0;
}

