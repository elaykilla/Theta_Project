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


//Constants for images
const double alpha = 15;
const double nb_images = 24;
const double dist_c = 9;


const double radius = 1;
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
	double xc,yc;
	double zc = 0;
	
  	
  
  	//Setup 3D visualizer
	visualization::PCLVisualizer viewer("3D viewer");
	//visualization::CloudViewer cViewer ("Simple Cloud Viewer");
//	//viewer.setBackgroundColor (0, 0, 0);


	
	//String names for line
	ostringstream line;
	
	// Vectors for mats and pointClouds
	vector<cv::Mat> allImages(24);
	vector<PointCloud<PointXYZRGB>::Ptr> allPtClouds(24);
	vector<PointCloud<PointXYZRGB>::Ptr> allVtClouds(24);
	
	// cv::Mat for original and spherical image
	cv::Mat ori;
	//cvNamedWindow("Original",0);
	//cvNamedWindow("recalculated",0);
	
	
	//Number of lines to draw
	int nbLines = 1000000;
	int lnCount = 0;
	/********************************* Define parameters ***************************************/
	
	
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
	sphereCenter(alpha, im_num, dist_c, xc, yc);
	//cout << "xc: "<< xc << endl;
	cloud = EquiToSphere(ori, radius,xc,yc);
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
	

	/************************** End of Initiate ********************************/	
	

	//Test simple 1 image
	//ori  = ori1;
	//For testing in order not to load all images every time
	int tempCount =1;
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
		sphereCenter(alpha, k, dist_c, xc, yc);
		//cout << "xc: "<< xc << endl;
		cloud = EquiToSphere(ori, radius,xc,yc);
		allPtClouds[k] = cloud;
		//cout << "Cloud size: " << cloud->size() << endl;

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
//					colorOri = ori.at<cv::Vec3b>(i, j);
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

//t1 = clock() - t;
//cout << "Time to execute firs part before Kmean: " << (float)t1/CLOCKS_PER_SEC <<endl;;
/******************************************* Test Zone **********************************************************************/
	//Testing space variables
	cloud = allPtClouds[0];
	ori = allImages[0];
	 
	int nbPoints = 10;
	vector<PointXYZRGB> points;
	//double r;
	int m;
	PointXYZRGB ikm;	
	//Set u to origin
	u.x = 0;
	u.y = 0;
	u.z = 0;
	v.x = 0;
	v.y = 1;
	v.z = 0;
	
	//r = 1;
	m = 0;
	
	//double step = 2*radius/nbPoints;
	iPoint.x = 0;
	iPoint.z = 0;
	iPoint.b = 255;
	v.r = 255;
	ikm.g = 255;
	vector<PointXYZRGB> linep(50);
	bool was_projected;
	
	////////////////////////////////////// Project to Sphere test ///////////////////////////
//	//hPointLine(u,v,linep);
	
////	for(int ttt =0;ttt<allPtClouds[0]->size();ttt++){
////		PointXYZRGB tt = cloud->points[ttt];
////		cout << "cloud points numb: " << m << " coord: " << tt <<endl;
////	}
//	vector<PointXYZRGB> points(nbPoints);
//	while(m<nbPoints){	
//		//cout << "m: " << m << endl;
//		//Calculated points on cirlce
////		iPoint.x += step;
////		iPoint.y = sqrt(abs(r - iPoint.x * iPoint.x));

//		//random points on same z plane
//		double d = radius/sqrt(2);
//		iPoint.x = (rand()%1000)/1000.  ;
//		iPoint.y = (rand()%1000)/1000.  ;
//		while(!(iPoint.y < sqrt(abs(radius*radius - iPoint.x * iPoint.x)))){
//			iPoint.y = (rand()%1000)/1000.  ;
//		}
//		
//		sight->points.push_back(iPoint);
//		
//		//cout << iPoint.x << "," << iPoint.y << endl;
//		
//		ikm = project2Sphere(iPoint,v,radius);
//		points[m] = ikm;
//		ikm.r= 255;
//		//i.b = i.g = 0;
//		//cout<< "i: " << i.x << ","<< i.y << "," << i.z << endl;
//		//sight->points.push_back(ikm);
//		
//		
//		
//		//cout<< "cloud size before function call: " << allPtClouds[0]->size() << endl;
//		//kMeanValue(ikm,cloud,sight, 1);
//		pixelInterpolate(ikm,radius,ori);
//		//usleep(100);
//		sight->points.push_back(ikm);
//		//cout << "ith " << ikm <<endl;
//		
//		//testfunction(*cloud);
//		
//		//PointXYZRGB tt = cloud->points[m*m];
//		//cout << "cloud points numb: " << m << " coord: " << tt <<endl;
//		//hPointLine(iPoint,i,linep);
//		
////		for(int n=0;n<linep.size();n++){
////			sight->points.push_back(linep[n]);
////			//cout << "p_" << n << ": " << linep[m]
////		}
//		m++;
//	}
////	boost::thread w1(multiKMeanValue,points,2,3,cloud,sight,1);
////	boost::thread w2(multiKMeanValue,points,3,3,cloud,sight,1);
////	multiKMeanValue(points,1,3,cloud,sight,1);
//	
////	w1.join();
////	w2.join();
//	//Excecution time
//	//t = clock() - t1;
//	//cout << "Time to execute Kmean: " << (float)t/CLOCKS_PER_SEC <<endl;
//	////////////////////////////////////// end of Project to Sphere test ///////////////////////////

	///////////////////////////Test of get plane function and project to Sphere with this plane////////////////////////
//	//double xmin, xmax, ymin, ymax;
//	PointXYZRGB xmin,xmax,ymin,ymax;
//	PointXYZRGB p;
//	
//	p.x = u.x + v.x;
//	p.y = u.y + v.y;
//	p.z = u.z + v.z;
//	hPointLine(u,p,linep);
//	p.r=255;
//	for(int n=0;n<linep.size();n++){
//		linep[n].r = 255;
//		sight->points.push_back(linep[n]);
//		//cout << "p_" << n << ": " << linep[m]
//	}
//	
//	
//	samplePlane(u,v,points,radius,sqrt(1000000)); 
//	
////	cout << "points.size in Main: " << points.size();
//	//cout << "max vector size: " << points.max_size() << endl;
//	for(int n=0;n<points.size();n++){
//			p = points[n];
//			sight->points.push_back(p);
//			ikm = project2Sphere(points[n],v,radius,was_projected);
//			ikm.r = 255;
//			if(was_projected){
//				sight->points.push_back(ikm);
//				p.r = p.b = 0;
//				p.g = 255;
//				sight->points.push_back(p);
//				//p.y += .5;
//				//sight->points.push_back(p);
//				pixelInterpolate(ikm,radius,ori);
//				p.r = ikm.r;
//				p.g = ikm.g;
//				p.b = ikm.b;
//				//p.z -= 1;
//				sightFlat->points.push_back(ikm);
////				ikm.x += 0.5;
//				p.x -= v.x/10;
//				p.y -= v.y/10;
//				p.z -= v.z/10;
//				//sightFlat->points.push_back(p);
//				
//				//sight->points.push_back(ikm);
//			}
//			//cout << "p_" << n << ": " << points[m] <<endl;
//	}
//	
//	//sphere2Equi(allImages[0], radius,cloud,sightFlat);
////	for(int m=0;m<sightFlat->size();m++){
////		p = sightFlat->points[m];
////		int r = p.x;
////		int c = p.y;
////		if(r<rows & c<cols){
////		sph.at<cv::Vec3b>(r, c)[0] = p.b;
////		sph.at<cv::Vec3b>(r, c)[1] = p.g;
////		sph.at<cv::Vec3b>(r, c)[2] = p.r;
////		}
////	}
////	cvResizeWindow("recalculated",rows,cols);
////	cv::imshow("recalculated",sph);
//	
//	
////	getPlane(u,v,radius,xmin,xmax,ymin,ymax);
////	sight->points.push_back(u);
////	sight->points.push_back(xmax);
////	sight->points.push_back(xmin);
////	sight->points.push_back(ymax);
////	sight->points.push_back(ymin);
////	u.r = xmin.r = xmax.r =255;
////	cout << "u" << u << endl;
////	cout << "xmin" << xmin << endl;
////	cout << "xmax" << xmax << endl;
////	cout << "ymin" << ymin << endl;
////	cout << "ymax" << ymax << endl;
	///////////////////////////////////End of get Plane function test ////////////////////////////////
	
	/////////////////////////////Test of projection with Angle from 1 point////////////////////////
//	double alpha1, beta1;
//	int fac = 1;
//	PointXYZRGB p;
//	int alphamin, alphamax,betamin,betamax;
//	
//	alphamin = 0;
//	betamin = 0;
//	alphamax = 90;
//	betamax = 120;
//	p.b=255;
//	
//	samplePlane(u,v,points,radius,sqrt(100000));
//	for(int n=0;n<points.size();n++){
//		p = points[n];
//		sight->points.push_back(p);
//	}
//	
//	p.x = u.x + v.x;
//	p.y = u.y + v.y;
//	p.z = u.z + v.z;
//	hPointLine(u,p,linep);
//	p.g = 255;
//	for(int n=0;n<linep.size();n++){
//		linep[n].r = 255;
//		sight->points.push_back(linep[n]);
//		//cout << "p_" << n << ": " << linep[m]
//	}
//	
//	
//	points.resize(2*fac*alphamax*betamax);
//	
//	
//	
//	for(int n=alphamin*fac;n<alphamax*fac;n++){
//		alpha1 = n/fac;
//		for(int k=betamin*fac;k<betamax*fac;k++){
//			beta1 = k/fac;
//			p = project2SphereWithAngle(u,v, alpha1, beta1, radius, was_projected);
//			p.r = 255;
//			//cout << "p:" << p;
//			points[n*(betamax)+k] = p;
//		}	
//	}
//	for(int n=0;n<points.size();n++){
//		p = points[n];
//		sight->points.push_back(p);
//	}
	
	///////////////////////////////////End of get Plane function test ////////////////////////////////
	//////////////////////////////////Test of closest Direction ////////////////////////////////////
	
//	PointXYZRGB vp;
//	double alpharad = alpha*PI/180;
//	
//	u.x = 1;
//	u.y = 1;
//	u.z = 1;
//	
//	v.x = cos(10*alpharad);
//	v.y = sin(10*alpharad);
//	v.z = 0;
//	
//	vp.x = u.x + v.x;
//	vp.y = u.y + v.y;
//	vp.z = u.z + v.z;
//	vp.r = 255;
//	
//	
//	int cli = closestImDirection(u,v,alpha);
//	cout<< "closest i: "<< cli <<endl;
//	sight->points.push_back(u);
//	sight->points.push_back(v);
//	
//	
//	hPointLine(u,vp,linep);
//		
//		for(int n=0;n<linep.size();n++){
//			sight->points.push_back(linep[n]);
//			//cout << "p_" << n << ": " << line[m]
//		}
//		
//	viewer.addPointCloud(sight, "Sight");
	
	//////////////////////////////////Test of closest Direction ////////////////////////////////////
	
	
	//////////////////////////////////Test of viewing angle origin ////////////////////////////////////
	//viewing angles
	double v_angle = 27.;
	double h_angle = 40.;
	vector<PointXYZRGB> linel(50);
	vector<PointXYZRGB> liner(50);
	
	double theta_min,theta_max,phi_min,phi_max;
	PointXYZRGB lbot,rbot,ltop,rtop;
	viewingLimitsOrigin(v, v_angle,h_angle,theta_min, theta_max, phi_min, phi_max);
//	cout << "theta_min:" << theta_min << endl;
//	cout << "theta_max:" << theta_max << endl;
//	cout << "phi_min:" << phi_min << endl;
//	cout << "phi_max:" << phi_max << endl;
	
	
	spheric2Cartesian(radius, theta_min,phi_min,lbot);
	//cout << "lbot:" << lbot << endl;
	spheric2Cartesian(radius,theta_min,phi_max,ltop);
	//cout << "ltop:" << ltop <<endl;
	spheric2Cartesian(radius,theta_max,phi_min,rbot);
	//cout << "rbot:" << rbot << endl;
	spheric2Cartesian(radius,theta_max,phi_max,rtop);
	//cout << "rtop:" << rtop <<endl;
	
	lbot.r = rbot.r = ltop.r = rtop.r = 255; 
	sightFlat->points.push_back(lbot);
	sightFlat->points.push_back(ltop);
	sightFlat->points.push_back(rbot);
	sightFlat->points.push_back(rtop);
	
	hPointLine(rbot,rtop,liner);
	for(int n=0;n<liner.size();n++){
		sightFlat->points.push_back(liner[n]);

	}
	
	hPointLine(lbot,ltop,linel);
	for(int n=0;n<linel.size();n++){
		sightFlat->points.push_back(linel[n]);

	}
	
//	for(int n=0;n<linel.size();n++){
//		hPointLine(liner[n],linel[n],linep);
//		for(int m=0;m<linep.size();m++){
//			sightFlat->points.push_back(linep[m]);
//		}
//	
//	}
	
//	
	hPointLine(rbot,lbot,linep);
	for(int n=0;n<linep.size();n++){
		sightFlat->points.push_back(linep[n]);
		//cout << "p_" << n << ": " << line[m]
	}
	
	hPointLine(rtop,ltop,linep);
	for(int n=0;n<linep.size();n++){
		sightFlat->points.push_back(linep[n]);
		//cout << "p_" << n << ": " << line[m]
	}
	double tempmin, tempmax;
	tempmin = min(theta_min,theta_max);
	tempmax = max(theta_min,theta_max);
	theta_min = tempmin;
	theta_max = tempmax; 
	
	tempmin = min(phi_min,phi_max);
	tempmax = max(phi_min,phi_max);
	phi_min = tempmin;
	phi_max = tempmax; 
	for(int n=0;n<cloud->size();n++){
		iPoint = cloud->points[n];
		cartesian2Spheric(iPoint,radius,theta,phi);
		if(theta>theta_min & theta<theta_max & phi<phi_max & phi>phi_min){
			sightFlat->points.push_back(iPoint);
		}
	
	}
//	
	
	
	/////////////////////////////// end Test of viewing angle origin //////////////////////////////////
/******************************************* End of Test Zone ***************************************************************/

// Display point cloud 
	//cloud->width=50;
	//cloud->height=50;
	
	
	
	
	//cViewer.showCloud (sightFlat);
	//cViewer.showCloud(sightFlat);
	//viewer.addPointCloud(sight, "Sphere");
	viewer.addPointCloud(sightFlat, "Sphere");
	//viewer.addPointCloud(sight, "Sphere1");
	
//	viewer.addPointCloud(allPtClouds[0], "Sphere");
//	viewer.addPointCloud(allPtClouds[12], "Sphere1");
//	viewer.addPointCloud(allPtClouds[2], "Sphere2");
//	viewer.addPointCloud(allPtClouds[3], "Sphere3");
	//imshow("Original",allImages[23]);
  	
	viewer.addCoordinateSystem (radius);
	viewer.setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Sphere");
	while (!viewer.wasStopped ()){
	// This seems to cause trouble when having cloud viewer and viewr running
		viewer.spinOnce (100);
		//cv::waitKey(0);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	  }
	
	
	//close viewer

	
	//get keypoints on 2D image 
	//cvResizeWindow("Keypoints 2D",rows,cols);
	//vector<cv::KeyPoint> keypoints;
	//keypoints = cv::get2DKeypoints(ori);
	
	//cv::drawKeypoints( ori, keypoints, ori, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
	//cv::imshow("Keypoints 2D" , ori);
	
	
	//cv::waitKey(0);


	 
	return 0;
}

