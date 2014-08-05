/*
* @author: ELay Maiga
* This class contains all the methods related to calculating coordinates and making projections
* from 2D to 3D or vice-versa.
*/

#include "standard_headers.hpp"
#include "cv_headers.hpp"
#include "pcl_headers.hpp"

#include"MathCalcs.hpp"
#include"OcvManip.hpp"
#include"PclManip.hpp"


//Constants for images
const double alpha = 15;
const double nb_images = 24;
const double dist_c = 0.5;


const double radius = 1;
const double phi0 = 0.0;
const double phi1 = 2*PI;
const double theta0 = 0.0;
const double theta1 = PI;
/************************************************* Setup **************************************************/
void setupImageAndClouds(string name, vector<Mat> &images, vector<PointCloud<PointXYZRGB>::Ptr> &cloud){


}





/************************************************* End Setup ***********************************************/


/***************************************************  Main  ***********************************************/
int main(int argc, char** argv)
{
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
	Vec3f color(0.0,0.0,0.0);
	Vec3b colorOri;
	
	
	//Pcl point cloud for sphere
	PointCloud<PointXYZRGB>::Ptr cloud (new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr sight (new PointCloud<PointXYZRGB>);
	PointXYZRGB iPoint;
	PointXYZRGB vPoint;
	PointXYZRGB u;
	PointXYZRGB v;
	
	//Sphere Centers are all defined in the XY plane because we do not rotate outside that plane
	double xc,yc;
	double zc = 0;
	
  	
  
  	//Setup 3D visualizer
	visualization::PCLVisualizer viewer("3D viewer");
	//visualization::CloudViewer cViewer ("Simple Cloud Viewer");
//	//viewer.setBackgroundColor (0, 0, 0);


	
	//String names for line
	ostringstream line;
	
	// Vectors for mats and pointClouds
	vector<Mat> allImages(24);
	vector<PointCloud<PointXYZRGB>::Ptr> allPtClouds(24);
	vector<PointCloud<PointXYZRGB>::Ptr> allVtClouds(24);
	
	// Mat for original and spherical image
	Mat ori;
	cvNamedWindow("Original",0);
	//Mat sph = Mat::zeros(rows, cols, CV_8UC3);
	
	
	//Number of lines to draw
	int nbLines = 1000000;
	int lnCount = 0;
	/********************************* Define parameters ***************************************/
	
	
	//Loading images
	//Mat ori1, ori2, ori3;
	
	
	//Load first image to get size and data
	int im_num = 0;
	loadImagei(name,im_num+1,ori);
	
	//Verify image has content
	if(!ori.data){
		cout << "Input image was not loaded, please verify: " << argv[1] << im_num + 1 <<".jpg" << endl;
		return -1;
	}
	
	
	allImages[im_num] = ori;
	sphereCenter(alpha, im_num, dist_c, xc, yc);
	cout << "xc: "<< xc << endl;
	cloud = EquiToSphere(ori, radius,xc,yc);
	allPtClouds[im_num] = cloud;
	im_num++;
	//recover mat props
	Size s = ori.size();
	int rows = s.height;
	int cols = s.width;
	cvResizeWindow("Original",rows,cols);
	//imshow("Original",ori);
	
	//loadImagei(name,3,ori2);
	//loadImagei(name,4,ori3);
	
	//cvNamedWindow("Keypoints 2D",0);

	/************************** End of Initiate ********************************/	
	

	//Test simple 1 image
	//ori  = ori1;
	//For testing in order not to load all images every time
	int tempCount =24;
	for(int k=im_num; k<tempCount; k++){
		cout << "k: " << k << endl;
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
		sphereCenter(alpha, k, dist_c, xc, yc);
		//cout << "xc: "<< xc << endl;
		cloud = EquiToSphere(ori, radius,xc,yc);
		allPtClouds[k] = cloud;
		//cout << "Cloud Size: " << cloud->size() << endl;

				//if(x+cx<u.x){
//					//if(y=0.5){
//					//Put the coordinates in a PointXYZ
//					iPoint.x = x + cx;
//					iPoint.y = y + cy;
//					iPoint.z = z;
//				
//					//Put view
//					vPoint.x = i;
//					vPoint.y =j;
//					vPoint.z = 0;
//					
//					//Add pixel color values
//					colorOri = ori.at<Vec3b>(i, j);
//				       	iPoint.b = vPoint.b = colorOri[0];
//					iPoint.g = vPoint.g = colorOri[1];
//					iPoint.r = vPoint.r = colorOri[2];			
//					cloud->points.push_back(iPoint);
//					//sight->points.push_back(vPoint);
//					
//					////////////////////////////Test for Add lines from 1 point //////////////////
////					if(lnCount < nbLines){ //&& (rand()%10)<0.001){
////						vector<PointXYZRGB> line(10);
////						hPointLine(v,iPoint,line);
////						
////						for(int m=0;m<line.size();m++){
////							cloud->points.push_back(line[m]);
////						}
////					}
////					////////////////////////////End of Addlines from 1 point test //////////////		
//				//} //End of if on x values
////				else{
////					iPoint.x = x + cx - 0.25;
////					iPoint.y = y;
////					iPoint.z = z;
////				}		       						
//			} // end of j loop of columns
//		
//		}// End of i loop or rows
	
	}//End of K loop of image
	
//	


/******************************************* Test Zone **********************************************************************/
	//Testing space variables 
	int nbPoints = 10000;
	//double r;
	int m;
	PointXYZRGB i;	
	//Set u to origin
	u.x = u.y = u.z = 0;
	v.x = 0;
	v.y = 1;
	v.z = 1;
	
	//r = 1;
	m = 0;
	
	double step = 2*radius/nbPoints;
	iPoint.x = 0;
	iPoint.z = 0;
	iPoint.b = 255;
	v.r = 255;
	vector<PointXYZRGB> linep(50);
	
	////////////////////////////////////// Project to Sphere test ///////////////////////////
//	hPointLine(u,v,linep);
//	while(m<nbPoints-1){	
//		//Calculated points on cirlce
////		iPoint.x += step;
////		iPoint.y = sqrt(abs(r - iPoint.x * iPoint.x));

//		//random points on same z plane
//		double d = radius/sqrt(2);
//		iPoint.x = (rand()%1000)*d/1000.  ;
//		iPoint.y = (rand()%1000)*d/1000.  ;
//		
//		//cout << iPoint.x << "," << iPoint.y << endl;
//		
//		i = project2Sphere(iPoint,v,radius);
//		i.r= 255;
//		i.b = i.g = 0;
//		//cout<< "i: " << i.x << ","<< i.y << "," << i.z << endl;
//		cloud->points.push_back(iPoint);
//		cloud->points.push_back(i);
//		
//		//hPointLine(iPoint,i,linep);
//		
//		for(int n=0;n<linep.size();n++){
//			cloud->points.push_back(linep[n]);
//			//cout << "p_" << n << ": " << linep[m]
//		}
//		m++;
//	}
//	////////////////////////////////////// end of Project to Sphere test ///////////////////////////

	/////////////////////////////////////Test of get plane function///////////////////////////////
//	PointXYZRGB xmin,xmax,ymin,ymax;
//	
//	getPlane(u,v,radius,xmin,xmax,ymin,ymax);
//	sight->points.push_back(u);
//	sight->points.push_back(xmax);
//	sight->points.push_back(xmin);
//	sight->points.push_back(ymax);
//	sight->points.push_back(ymin);
//	u.r = xmin.r = xmax.r =255;
//	cout << "u" << u << endl;
//	cout << "xmin" << xmin << endl;
//	cout << "xmax" << xmax << endl;
//	cout << "ymin" << ymin << endl;
//	cout << "ymax" << ymax << endl;
	///////////////////////////////////End of get Plane function test ////////////////////////////////
	
	//////////////////////////////////Test of closest Direction ////////////////////////////////////
	
	PointXYZRGB vp;
	double alpharad = alpha*PI/180;
	
	u.x = 1;
	u.y = 1;
	u.z = 1;
	
	v.x = cos(10*alpharad);
	v.y = sin(10*alpharad);
	v.z = 0;
	
	vp.x = u.x + v.x;
	vp.y = u.y + v.y;
	vp.z = u.z + v.z;
	vp.r = 255;
	
	
	int cli = closestImDirection(u,v,alpha);
	cout<< "closest i: "<< cli <<endl;
	sight->points.push_back(u);
	sight->points.push_back(v);
	
	
	hPointLine(u,vp,linep);
		
		for(int n=0;n<linep.size();n++){
			sight->points.push_back(linep[n]);
			//cout << "p_" << n << ": " << line[m]
		}
		
	viewer.addPointCloud(sight, "Sight");
	
	//////////////////////////////////Test of closest Direction ////////////////////////////////////
/******************************************* End of Test Zone ***************************************************************/

// Display point cloud 
	cloud->width=50;
	cloud->height=50;
	
	
	//cViewer.showCloud (sight);
	
	viewer.addPointCloud(allPtClouds[cli], "Sphere");
	//viewer.addPointCloud(allPtClouds[1], "Sphere1");
	//viewer.addPointCloud(allPtClouds[2], "Sphere2");
	//viewer.addPointCloud(allPtClouds[3], "Sphere3");
	//imshow("Original",allImages[23]);
  	
	viewer.addCoordinateSystem (1.0);
	viewer.setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Sphere");
	while (!viewer.wasStopped ()){
		viewer.spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	  }
	

	
	//close viewer

	
	//get keypoints on 2D image 
	//cvResizeWindow("Keypoints 2D",rows,cols);
	//vector<KeyPoint> keypoints;
	//keypoints = get2DKeypoints(ori);
	
	//drawKeypoints( ori, keypoints, ori, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
	//imshow("Keypoints 2D" , ori);
	
	
	//waitKey(0);



	return 0;
}

