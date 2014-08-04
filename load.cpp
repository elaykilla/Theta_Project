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


/***************************************************  Main  ***********************************************/
int main(int argc, char** argv)
{


	//Define the constants that we will be using. 
	// - radius is the desired radius of the sphere
	// - phi0 and phi1 represent min and max values of angle phi [0,2PI]
	// - theta0 and theta1 represent min and max values of angle theta [0,PI]
	// alpha the rotation angel between successive images
	double radius = 1;
	double phi0 = 0.0;
	double phi1 = 2*PI;
	double theta0 = 0.0;
	double theta1 = PI;
	double alpha =15;
	
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
	//Sphere Centers
	double xc,yc;
	//visualization::CloudViewer cViewer ("Simple Cloud Viewer");
  	
  
  	//Setup 3D visualizer
	visualization::PCLVisualizer viewer("3D viewer");
//	//viewer.setBackgroundColor (0, 0, 0);
	
	//Loading images
	Mat ori1, ori2, ori3;
	string name = argv[1];
	loadImagei(name,1,ori1);
	loadImagei(name,3,ori2);
	loadImagei(name,4,ori3);
	cvNamedWindow("Original",0);
	//cvNamedWindow("Keypoints 2D",0);
	
	
	//for lines
	ostringstream line;
	
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
	
	int count=3;
	//i corresponds to the y and hence runs through the rows = height 
	Mat ori;
	int cx,cy;
	
	
	//Number of lines to draw
	int nbLines = 1000000;
	int lnCount = 0;
	
	//Test simple 1 image
	//ori  = ori1;
	for(int k=0; k<count; k++){
		if(k==0){
			ori = ori1;
			cx = 1;
			cy=0;
		}
		else if(k==1){
			ori = ori2;
			cx = 1;
			cy = -1;
		}
		else if(k==2){
			ori = ori3;
			cx = 0;
			cy = -1;	
		}
		//sphereCenter(alpha, k, radius, xc, yc);
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
				
				u.x = 0.5;
				u.y = 0;
				u.z = 0;
				
				v.x = 1;
				v.y = v.z = 0;
				PointXYZ Tmin, Tmax;
				circularYcut(u,radius,Tmin,Tmax);
				
				//Add Pixel Color
//				colorOri = ori.at<Vec3b>(i, j);
//				iPoint.b = colorOri[0];
//				iPoint.g = colorOri[1];
//				iPoint.r = colorOri[2];			
//				cloud->points.push_back(iPoint);
				
				//if(x+cx<u.x){
					//if(y=0.5){
					//Put the coordinates in a PointXYZ
					iPoint.x = x + cx;
					iPoint.y = y + cy;
					iPoint.z = z;
				
					//Put view
					vPoint.x = i;
					vPoint.y =j;
					vPoint.z = 0;
					
					//Add pixel color values
					colorOri = ori.at<Vec3b>(i, j);
				       	iPoint.b = vPoint.b = colorOri[0];
					iPoint.g = vPoint.g = colorOri[1];
					iPoint.r = vPoint.r = colorOri[2];			
					cloud->points.push_back(iPoint);
					//sight->points.push_back(vPoint);
					
					////////////////////////////Test for Add lines from 1 point //////////////////
//					if(lnCount < nbLines){ //&& (rand()%10)<0.001){
//						vector<PointXYZRGB> line(10);
//						hPointLine(v,iPoint,line);
//						
//						for(int m=0;m<line.size();m++){
//							cloud->points.push_back(line[m]);
//						}
//					}
//					////////////////////////////End of Addlines from 1 point test //////////////		
				//} //End of if on x values
//				else{
//					iPoint.x = x + cx - 0.25;
//					iPoint.y = y;
//					iPoint.z = z;
//				}		       						
			} // end of j loop of columns
		
		}// End of i loop or rows
	
	}//End of K loop of image
	
	


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
//		//hPointLine(iPoint,i,line);
//		
//		for(int n=0;n<linep.size();n++){
//			cloud->points.push_back(linep[n]);
//			//cout << "p_" << n << ": " << line[m]
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
/******************************************* End of Test Zone ***************************************************************/

// Display point cloud 
	cloud->width=50;
	cloud->height=50;
	
	
	//cViewer.showCloud (sight);
	
	viewer.addPointCloud(cloud, "Sphere");
  	
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

