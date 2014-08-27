/*
* @author: ELay Maiga
* This class contains all the methods related to calculating coordinates and making projections
* from 2D to 3D or vice-versa.
*/

#include "standard_headers.hpp"
#include "cv_headers.hpp"
#include "pcl_headers.hpp"
#include "boost_headers.hpp"

#include"MathCalcs.hpp"
#include"OcvManip.hpp"
#include"PclManip.hpp"
//#include"test.hpp"


//Constants for images
const double alpha = 15;
const double nb_images = 24;
const double dist_c = 100;


const double radius = .006;
const double phi0 = 0.0;
const double phi1 = 2*PI;
const double theta0 = 0.0;
const double theta1 = PI;

bool update;
/************************************************* Setup **************************************************/
void setupImageAndClouds(string name, vector<cv::Mat> &images, vector<PointCloud<PointXYZRGB>::Ptr> &cloud){


}





/************************************************* End Setup ***********************************************/

/************************************************ Visualizer **********************************************/


/***********************************************End Visualizer ********************************************/
//void visualize(visualization::PCLVisualizer viewer, PointCloud<PointXYZRGB>::Ptr &cloud, string cloudTag)  
//    {  
//        // prepare visualizer named "viewer"

//        while (!viewer.wasStopped ())
//        {
//            viewer.spinOnce (100);
//            // Get lock on the boolean update and check if cloud was updated
//            //boost::mutex::scoped_lock updateLock(updateModelMutex);
//            if(update)
//            {
//                if(!viewer.updatePointCloud(cloud, cloudTag))
//                  viewer.addPointCloud(cloud);
//                update = false;
//            }
//            //updateLock.unlock();

//        }   
//   }  


/***************************************************  Main  ***********************************************/
int main(int argc, char** argv)
{
	//InitLog("load");
	BOOST_LOG_TRIVIAL(trace) << "Main Function Has begun" <<endl;
	
	//clock_t t1;
	//clock_t t;
	//t = clock();
	//Verify function call
	string name = argv[1];
	if(argc<2){
		//cout << "Input image was not loaded, please verify: " << argv[1] << endl;
		cout << "Application usage is not correc, please refer to usage" << endl;
		cout << "path: is the folder for images, imagename: is the name of the images without the number" << endl;
		cout << "Exemple for images called I1,I2,I3 etc.. in folder IMG: load IMG/I" << endl;
		cout << "Usage: load path_to_images/imagename" << endl;
		return -1;
	}
	
	//setupImageAndClouds(name,)
	/********************************* Define parameters ***************************************/
	//Define the constants that we will be using. 
	// - radius is the desired radius of the sphere
	// - phi0 and phi1 represent min and max values of angle phi [0,2PI]
	// - theta0 and theta1 represent min and max values of angle theta [0,PI]
	// alpha the rotation angel between successive images

	
	//Math variables, point coordinates and angles
	double theta,phi,x,y,z;
	
	//b,g,r color values of input image
	float b,g,r;
	
	//empty color vector
	cv::Vec3f color(0.0,0.0,0.0);
	cv::Vec3b colorOri;
	
	
	//Pcl point cloud for sphere
	PointCloud<PointXYZRGB>::Ptr cloud (new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr sight (new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr sightFlat (new PointCloud<PointXYZRGB>);
	PointXYZRGB iPoint;
	PointXYZRGB vPoint;
	PointXYZRGB u;
	PointXYZRGB v;
	
	//Sphere Centers are all defined in the XY plane because we do not rotate outside that plane
	double xc,zc;
	double yc = 0;
	
  	
  
  	//Setup 3D visualizer
	visualization::PCLVisualizer viewer("3D viewer");
	//visualization::CloudViewer cViewer ("Simple Cloud Viewer");
//	//viewer.setBackgroundColor (0, 0, 0);


	
	//String names for line
	ostringstream line;
	
	// Vectors for mats and pointClouds
	vector<cv::Mat> allImages(nb_images);
	vector<PointCloud<PointXYZRGB>::Ptr> allPtClouds(nb_images);
	vector<PointCloud<PointXYZRGB>::Ptr> CTBPtClouds(3);
	vector<PointCloud<PointXYZRGB>::Ptr> allVtClouds(nb_images);
	
	// cv::Mat for original and spherical image
	cv::Mat ori,top,bottom;
	//cvNamedWindow("Original",0);
	//cvNamedWindow("recalculated",0);
	//cvNamedWindow("Keypoints 2D",0);
	
	//Number of lines to draw
	int nbLines = 1000000;
	int lnCount = 0;
	/*********************************************** end Define parameters *************************************************/
	
	
	//Loading images
	//cv::Mat ori1, ori2, ori3;
	
	
	
	//Load first image to get size and data
	int im_num = 0;
	loadImagei(name,im_num+1,ori);
	
	//Verify image has content
	if(!ori.data){
		cout << "Input image was not loaded, please verify: " << argv[1] << im_num + 1 <<".jpg" << endl;
		return -1;
	}

	
	allImages[im_num] = ori;
	sphereCenter(alpha, im_num, dist_c, xc, zc);
	//cout << "xc: "<< xc << endl;
	cloud = EquiToSphere(ori, radius,xc,yc,zc);
	
	allPtClouds[im_num] = cloud;
	im_num++;
	//recover cv::Mat props
	cv::Size s = ori.size();
	int rows = s.height;
	int cols = s.width;
	//cvResizeWindow("Original",rows,cols);
	//cvNamedWindow("Keypoints 2D",0);
	//imshow("Original",ori);
	
	//For testing
	cv::Mat sph = cv::Mat::zeros(rows, cols, CV_8UC3);
	//end for testing
	
	//loadImagei(name,3,ori2);
	//loadImagei(name,4,ori3);
	

	/**************************************************** End of Initiate ************************************************/	
	
	//For testing in order not to load all images every time
	int tempCount =atoi(argv[2]);
	cout << "Images to be loaded: " << tempCount<< endl;
	for(int k=im_num; k<tempCount; k++){
		//cout << "k: " << k << endl;
		loadImagei(name,k,ori);
		allImages[k] = ori;
		//cout << "width: " << allImages[k].cols << endl;
		//cout << "Height: " << allImages[k].rows << endl;
		//Verify image has content
		if(!ori.data){
			cout << "Input image was not loaded, please verify: " << argv[1] << im_num <<".jpg" << endl;
			return -1;
		}

		//Get the center of the Kth sphere in World Coordinates
		sphereCenter(alpha, k, dist_c, xc, zc);
		
		//Create Sphere from Equirectangular image and store in Point Cloud
		yc=0;
		cloud = EquiToSphere(ori, radius,xc,yc,zc);
		allPtClouds[k] = cloud;			
	}//End of K loop of image
	
	//If Top and Bottom given
//	string tb = argv[3];
//	if(tb==""){
//		//Load top and bottom Images
//		loadImageTop(name,top,"top");
//		xc = 0;		
//		yc = dist_c;
//		zc = 0;
//		cloud = EquiToSphere(top, radius,xc,yc,zc);
//		
//		loadImageTop(name,bottom,"bottom");
//		xc = 0;		
//		yc = -dist_c;
//		zc = 0;
//		cloud = EquiToSphere(top, radius,xc,yc,zc);
//		
//	}
	
//t1 = clock() - t;
//cout << "Time to execute firs part before Kmean: " << (float)t1/CLOCKS_PER_SEC <<endl;

/******************************************* Test Zone **********************************************************************/
	//=====================Testing space variables==================================//
	cloud = allPtClouds[0];
	ori = allImages[0];
	 
	int nbPoints = 10;
	vector<PointXYZRGB> points;
	int m = 0;
	PointXYZRGB ikm;	
	PointXYZRGB o;
	//set o to origin
	o.x = 0;
	o.y = 0;
	o.z = 0;
	
	//Set u to anywhere in the sphere and v any direction
	u.x = 0;
	u.y = 0;
	u.z = 0;;
	v.x = -0.4;
	v.y = 0;
	v.z = -0.3;
	
	
	double step = 2*radius/nbPoints;
	iPoint.x = 0;
	iPoint.z = 0;
	iPoint.b = 255;
	v.r = 255;
	ikm.g = 255;
	vector<PointXYZRGB> linep(50);
	
	
	// Project to Sphere test 
//	projectToSphereTest(nbPoints,step,sight);
	

	//Test of get plane function and project to Sphere with this plane
//	getPlaneAndProjectToSphere(PointXYZRGB u, PointXYZRGB v, PointCloud<PointXYZRGB>::Ptr sight)


	
	//Test of projection with Angle from 1 point
//	projectionWithAngleFromOnePointTest(1,radius);

	//Test of closest Direction
//	closestImDirectionTest(alpha,20);

	
	
	//////////////////////////////////Test of viewing angle origin ////////////////////////////////////
//	
//	/////////////////////////////// end Test of viewing angle origin //////////////////////////////////
//	viewingAngleOriginTest( u,  v, radius,  rows, cols,cloud, sightFlat);
	//Viewing angles
	///////////////////////////////////////  Test point on ray ////////////////////////////////////////
	iPoint.x = 0;
	iPoint.y = 0;
	iPoint.z = 0;
	
	o.r = o.g = o.b = 255;
	sight->points.push_back(u);
	//cout << "Ipoint on Ray: " << isOnRay(o,u,v) << endl;
	//cout << "Ipoint on Ray: " << isCloseToRayCube(o,u,v,0) << endl;
	
	//for all the pt clouds converted
	
	for(int m=0;m<tempCount;m++){
		//Get the sphere center of projection
		sphereCenter(alpha, m, dist_c, xc, zc);
		o.x = xc;
		o.z = zc;
		cloud = allPtClouds[m];
		for(int n=0;n<cloud->size();n++){
			iPoint = cloud->points[n];
		
			if(isCloseToRayCube(u,o,iPoint,.1)){
				sightFlat->points.push_back(iPoint);
				//cout << "ray passing through u: " << iPoint << endl;
			}
		}
		//cout << "Number of points: " << sightFlat->size() << endl;
	}
	
	//for Top and Bottom
	
	
	
	//////////////////////////////////////  end Test point on ray ////////////////////////////////////
/******************************************* End of Test Zone ***************************************************************/

// Display point cloud 
	//cloud->width=50;
	//cloud->height=50;
	
	
	//cvResizeWindow("Keypoints 2D",rows,cols);
	
	
	//cViewer.showCloud (sightFlat);
	//cViewer.showCloud(sightFlat);
	//viewer.addPointCloud(sightFlat, "Sphere");
//	viewer.addPointCloud(sight, "Sphere");
	//viewer.addPointCloud(sight, "Sphere1");
	
	viewer.addPointCloud(allPtClouds[0], "Sphere");
	viewer.addPointCloud(allPtClouds[1], "Sphere1");
//	viewer.addPointCloud(allPtClouds[2], "Sphere2");
//	viewer.addPointCloud(allPtClouds[3], "Sphere3");
	//imshow("Original",allImages[23]);
  	
	viewer.addCoordinateSystem (radius);
	viewer.setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Sphere");
	while (!viewer.wasStopped ()){
	// This seems to cause trouble when having cloud viewer and viewr running
		//cv::imshow("Keypoints 2D" , sightMat);
		viewer.spinOnce (100);
		
		//cv::waitKey(0);
		boost::this_thread::sleep (boost::posix_time::microseconds (10000));
	  }
	
	
	//close viewer

	
	//get keypoints on 2D image 
	
	//vector<cv::KeyPoint> keypoints;
	//keypoints = cv::get2DKeypoints(ori);
	
	//cv::drawKeypoints( ori, keypoints, ori, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
	//cv::imshow("Keypoints 2D" , ori);
	//cv::imshow("Keypoints 2D" , sightMat);
	
	//cv::waitKey(0);


	 
	return 0;
}

