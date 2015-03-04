/*
 * @author: ELay Maiga
 * This class contains all the methods related to calculating coordinates and making projections
 * from 2D to 3D or vice-versa.
 */

//#include "standard_headers.hpp"
//#include "cv_headers.hpp"
//#include "pcl_headers.hpp"
//#include "boost_headers.hpp"

#include"MathCalcs.hpp"
#include"OcvManip.hpp"
#include"PclManip.hpp"
#include"test.hpp"


//Constants for images
const double alpha = 15;
const double nb_images = 24;
const double dist_c = 100;


const double radius = 1;
const double phi0 = 0.0;
const double phi1 = 2*PI;
const double theta0 = 0.0;
const double theta1 = PI;

bool update;
/************************************************* Setup **************************************************/
//void initOptions(agrv,argv){
//	while((c=getopt(argc,argv))!=-1){
//		switch (c){
//			case 'init'
//			
//		}
//	}
//}

void setupClouds(string name, vector<cv::Mat> &images, vector<PointCloud<PointXYZRGB>::Ptr> &allPtClouds){
	//cout << "k: " << k << endl;
	cv::Mat ori;
	double xc,yc,zc;
	PointCloud<PointXYZRGB>::Ptr cloud (new PointCloud<PointXYZRGB>);

	for(int k=0;k<nb_images;k++){
		ori = images[k];
		//loadImagei(name,k+1,ori);
		//allImages[k] = ori;
		//cout << "width: " << allImages[k].cols << endl;
		//cout << "Height: " << allImages[k].rows << endl;
		//Verify image has content
		if(!ori.data){
			cout << "Input image was not loaded, please verify: " << name << k <<".jpg" << endl;
			return ;
		}

		//Get the center of the Kth sphere in World Coordinates
		sphereCenter(alpha, k, dist_c, xc, zc);

		//Create Sphere from Equirectangular image and store in Point Cloud
		yc=0;
		cloud = EquiToSphere(ori, radius,xc,yc,zc);
		allPtClouds[k] = cloud;	
	}

}





/************************************************* End Setup ******************************************/


void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void){
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
	if (event.getKeySym () == "n" && event.keyDown ())
	{
		std::cout << "n was pressed => changing viewing point" << std::endl;
		viewer->removeAllPointClouds();
		//viewer->addPointCloud(cloud,"keyboardId");
	}
}
/***************************************************  Main  ***********************************************/
int main(int argc, char** argv)
{
	//InitLog("load");
	//BOOST_LOG_TRIVIAL(trace) << "Main Function Has begun" <<endl;

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
	//	visualization::CloudViewer cViewer ("Simple Cloud Viewer");
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
	//cvNamedWindow("Original",0);
	//cvNamedWindow("SightMat",0);

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
		loadImagei(name,k+1,ori);
		allImages[k] = ori;
		//cout << "width: " << allImages[k].cols << endl;
		//cout << "Height: " << allImages[k].rows << endl;
		//Verify image has content
		if(!ori.data){
			cout << "Input image was not loaded, please verify: " << argv[1] << k+1 <<".jpg" << endl;
			return -1;
		}

		//Get the center of the Kth sphere in World Coordinates
		sphereCenter(alpha, k, dist_c, xc, zc);

		//Create Sphere from Equirectangular image and store in Point Cloud
		yc=0;
		cloud = EquiToSphere(ori, radius,xc,yc,zc);
		allPtClouds[k] = cloud;	

		//Save the PCD files
		ostringstream file;
		file << "./theta_pcds/" ;
		cloud->width = cols;
		cloud->height = rows;
		cloud->points.resize(cols*rows);
		cloud->is_dense = true;
		file << name <<k << ".pcd"; 
		String s = file.str();
		//io::savePCDFileASCII(s,*cloud);	

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
	v.x = 0;
	v.y = 1;
	v.z = 0;



	double step = 2*radius/nbPoints;
	iPoint.x = 0;
	iPoint.z = 0;
	iPoint.b = 255;
	v.r = 255;
	ikm.g = 255;
	vector<PointXYZRGB> linep(50);


	
	//Image rotation test multiple
	double ang = 20;
	//for(int i=10;i<13;i++){
	//	ostringstream filenameb, filenamea;
		//filenameb << "ToRotate/Bottom (" << i << ").jpg" ; 
		//filenamea << "Rotated/Bottom (" << i << ").jpg" ;
	
		//ori = cv::imread(filenameb.str(),1);
		//ori = allImages[2];
	
		//cv::Mat templ = cv::imread("ToRotate/Bottom (14).JPG",1);
	
		//cv::imwrite(filenamea.str(), rotateImagey(ori,ang));
		//ang += 20;
	
	//}
	// Project to Sphere test 
	//	projectToSphereTest(nbPoints,step,sight);




	//Test of get plane function and project to Sphere with this plane
	//	getPlaneAndProjectToSphere(PointXYZRGB u, PointXYZRGB v, PointCloud<PointXYZRGB>::Ptr sight)



	//Test of projection with Angle from 1 point
	//	projectionWithAngleFromOnePointTest(1,radius);



	/////////////////////////////// end Test of viewing angle origin //////////////////////////////////
	//viewingAngleOriginTest( u,  v, radius,  rows, cols,cloud, sightFlat);
	//int trows = round(2048*70/360);
	//int tcols = round(1024*57/180);
	//sph = imageFromPcPlane(sightFlat,allImages[0],trows, tcols);
	//cv::imshow("SightMat" , sph);
//	cv::waitKey(0);
	//Viewing angles
	///////////////////////////////////////  Test point on ray ////////////////////////////////////////
	//	iPoint.x = 0;
	//	iPoint.y = 0;
	//	iPoint.z = 0;
	//	
	//	o.r = o.g = o.b = 255;
	//	sight->points.push_back(u);
	//	//cout << "Ipoint on Ray: " << isOnRay(o,u,v) << endl;
	//	//cout << "Ipoint on Ray: " << isCloseToRayCube(o,u,v,0) << endl;
	//	
	//	//for all the pt clouds converted
	//	
	//	for(int m=0;m<tempCount;m++){
	//		//Get the sphere center of projection
	//		sphereCenter(alpha, m, dist_c, xc, zc);
	//		o.x = xc;
	//		o.z = zc;
	//		cloud = allPtClouds[m];
	//		for(int n=0;n<cloud->size();n++){
	//			iPoint = cloud->points[n];
	//		

	//			if(isCloseToRayCube(u,o,iPoint,.9)){
	//				sightFlat->points.push_back(iPoint);
	//				//cout << "ray passing through u: " << iPoint << endl;
	//			}
	//		}
	//	
		//cout << "Number of points: " << sightFlat->size() << endl;
	//	}
	//	
	//	//for Top and Bottom
	//////////////////////////////////////  end Test point on ray ////////////////////////////////////


	//Test of 2D keypoints and Matches
	//KeyPointAndMatchesTest(allImages[0], allImages[1]);
	
	//Test of image interpolation
	//ori = cv::imread("test3.JPG",1);
	ori = allImages[0];
	
	//cv::Mat templ = cv::imread("test4.JPG",1);
	cv::Mat templ = allImages[1];
//	interpolate2DTest(allImages[0], allImages[1], 8, 4);
	
	//Test of sphereInterpolate
	//sightFlat = sphereInterpolate(allImages[0], allImages[1], 8, 4);
	
//	//Test of optical flow 
	//optFlowMapTest(ori, templ);
	
	//Line detection testing
	//getLinesTest(ori);
	
//	edge detection test
//	cv::Mat result1 = detectEdges(ori);
//	cv::Mat result2 = detectEdges(templ);
//	cv::namedWindow("edges1", 0);
//	cv::namedWindow("edges2", 0);
//	cv::imshow("edges1", result1);
//	cv::imshow("edges2", result2);
	
	
	cv::Mat inter = cv::Mat::ones(ori.rows/1, ori.cols/1, ori.type());
	cv::resize(ori,inter,inter.size(),0,0,INTER_CUBIC);
	//cv::pyrDown(ori,inter,Size(ori.cols/2,ori.rows/2));
	ori = inter;
	
	//cv::pyrDown(templ,inter,Size(ori.cols/2,ori.rows/2));
	cv::Mat inter2 = cv::Mat::ones(ori.rows/1, ori.cols/1, ori.type());
	cv::resize(templ,inter2,inter.size(),0,0,INTER_CUBIC);
	templ = inter2;
	
	//Test of Delaunay Triangles
	//delaunayTriangleTest(ori, "T1");
	//delaunayTriangleTestBound(ori, "T1");
	//delaunayTriangleTest(allImages[1], "T2");
	
	//Test with Maching
	//delaunayMatchedTrianglesTest(ori, templ,sightFlat);
	//delaunayMatchedTrianglesBoundTest(ori, templ,sightFlat);
	
	//Test delaunay interpolation
	double nb_inter = 50;
	int i=0;
	//vector<cv::Mat> interpolated = delaunayInterpolateMultiple(ori,templ,1,nb_inter);
	
	
	//Multi image vector for test
	vector<cv::Mat> images;
		
	//cout<< "Number of interpolated images: " << interpolated.size() <<endl;
	cv::Mat result;
//	for(i=0;i<nb_inter;i++){
//		ostringstream nameWindow;
//		nameWindow << "temp/Interpolated Image_"<< i ;
//		cout << nameWindow.str() << endl;
//		cv::Mat result = delaunayInterpolate(ori,templ,1,i/nb_inter);
//		//cv::Mat result = interpolated[i];
//		//cv::namedWindow(nameWindow.str(), 0);
//		//cv::imshow(nameWindow.str(), result);
//		nameWindow << ".jpg" ;
//		cv::imwrite(nameWindow.str(),result);	
//		//images[i] = result;
//		images.push_back(result);
//	}


	//Read and write files from tmp
	for(i=0;i<nb_inter;i++){
		ostringstream nameWindow;
		nameWindow << "temp/Interpolated Image_"<< i << ".jpg" ;
		//nameWindow << "Bottom/Bottom"<< i+1 << ".jpg" ;
		Mat image = cv::imread(nameWindow.str(),1);
		if(!image.data()){}
		
		else{
			images.push_back(image);
		}
	}
	string videoName = "temp/Interpolated Video" ;
	imageListToVideo(images,videoName);
//	sightFlat = EquiToSphere(result, 1,0,0,0);
	
	
	
	
	
	//cv::waitKey(0);

	//Test of Epipolar Lines
	//cv::Mat temp = cv::imread("template3.jpg",1);
//	//sightFlat = EquiToSphere(temp, radius,xc,yc,zc);
	
	
//	if(temp.data){
//		EpipolarLinesTest(allImages[0],temp); 	
//	}
//	//Test of Angles
//	o.x = 0;
//	o.y = 0;
//	o.z = 0;

//	u.x = 0.8*radius;
//	u.y = 0;
//	u.z = 0.5*radius;

//	double angle = 10;
//	double newAngle;
	//	cout << "Closest direction with angle from origin: " << angle << " is : " 
	//	<< closestImDirectionOrigin (angle*PI/180, alpha) << endl;
	//	
	//	cout << "Closest direction with angle from u:" << u << "with angle :" << angle << " is: " 
	//	<< closestImDirection (u,angle,alpha,radius,newAngle) << endl;


	////////////Test of circular slits///////////////////////////
//	double t,p;
//	cout << "Beginning slit recovery " << endl;
//	for(int angle=0;angle<360;angle++){
//		int num = closestImDirection (u,angle,alpha,radius,newAngle);
//		//cout << "New Angle: " << newAngle << endl;
//		if(num>=tempCount){
//			//cout << num << "too big to find" << endl;
//			//break;
//		}
//		else{
//			cloud = allPtClouds[num];

//			for (int m=0;m<cloud->size();m++){
//				iPoint = cloud->points[m];
//				cartesian2Spheric(iPoint,radius,t,p);
//				// t in degrees
//				p *= 180/PI;
////				cout << "New Angle: " << newAngle ;
////				cout << "   theta for iPoint: " << t << endl;
//				if(newAngle <= p && p<= newAngle+1){
//				
//					//testing different colors for the points
////					if(num%3==0){
////						iPoint.r = 255;
////					}
////					if(num%3==1){
////						iPoint.b = 255;
////					}
////					if(num%3==2){
////						iPoint.g = 255;
////					}
//					
//					sightFlat->points.push_back(iPoint);
//				}
//			}
//			//cout << "Finished point cloud number: " << num <<endl;
//		}
//	}
//	cout << "ended slit recovery" << endl;
//	
////	ori.setTo(cv::Scalar(0,0,0));
//	cv::Mat sightMat(ori.rows,ori.cols,ori.type());
//	sightMat.setTo(cv::Scalar(0,0,0));
////	cout << "Original: " <<ori.rows << ori.cols << ori.type() << endl;
////	cout << "sightMat: " <<sightMat.rows << sightMat.cols << sightMat.type() << endl;
//	sphereToEqui(sightFlat,radius,rows,cols,sightMat);
//	cv::imshow("SightMat",sightMat);
//	cv::waitKey();

	////////////Test of circular slits///////////////////////////
	
	//3D Keypoints Test
//	PointCloud<PointWithScale>::Ptr sightFlat1;
//	threeDKeypointsTest(cloud,sightFlat1);
//	PointWithScale pws;
//	PointXYZRGB pxy;
	//cout << "OutCloud size: " << sightFlat1->size() << endl;
//	for(int m=0;m<sightFlat1->size();m++){
//		pws = sightFlat1->points[m];
//		pxy.x = pws.x;
//		pxy.y =  pws.y;
//		pxy.z = pws.z;
//		sightFlat->points.push_back(pxy);
//	}


	////////////////////////////////////////Test of 3D affine transformations
//	Vec9f t1, t2; cv::Mat affine_transform(4,4,CV_32FC1);
//	
//	t1[0] = 1;
//	t1[1] = 1;
//	t1[2] = 0;
//	t1[3] = -1;
//	t1[4] = 1;
//	t1[5] = 1;
//	t1[6] = 1;
//	t1[7] = 0;
//	t1[8] = 1;
//	
//	t2[0] = 1; 
//	t2[1] = 1;
//	t2[2] = 0;
//	t2[3] = -1;
//	t2[4] = 1;
//	t2[5] = 1;
//	t2[6] = 1;
//	t2[7] = 0;
//	t2[8] = 1;
	
	
	//affine_transform = getAffine3D(t1,t2);
	
	
	
	/******************************************* End of Test Zone ***************************************************************/


	//cvResizeWindow("Keypoints 2D",rows,cols);



	//	cViewer.showCloud (allPtClouds[0]);
	//	cViewer.showCloud (sightFlat);
	//cViewer.showCloud(sightFlat);
	viewer.addPointCloud(sightFlat, "Sphere");

	//	viewer.addPointCloud(sight, "Sphere");
	//viewer.addPointCloud(sight, "Sphere1");

	//viewer.addPointCloud(allPtClouds[0], "Sphere");
	//	viewer.addPointCloud(allPtClouds[1], "Sphere1");
		//viewer.addPointCloud(allPtClouds[2], "Sphere2");
	//	viewer.addPointCloud(allPtClouds[3], "Sphere3");
	//imshow("Original",allImages[0]);

	//viewer.addCoordinateSystem (radius*2);
	viewer.setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Sphere");
	viewer.registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
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






