#include "standard_headers.hpp" 

#include "cv_headers.hpp"
#include "pcl_headers.hpp"
#include "boost_headers.hpp"

#include"MathCalcs.hpp"
#include"OcvManip.hpp"
#include"PclManip.hpp"



////////////////////////////////////// Project to Sphere test ///////////////////////////

void projectToSphereTest(int nbPoints, double step, double radius, PointXYZRGB v,PointCloud<PointXYZRGB>::Ptr sight){
//	//hPointLine(u,v,linep);
	PointXYZRGB iPoint,ikm;
	vector<PointXYZRGB> points(nbPoints);
	bool was_projected;
	
	int m=0;
	while(m<nbPoints){	
//		//Calculated points on cirlce
////		iPoint.x += step;
////		iPoint.y = sqrt(abs(r - iPoint.x * iPoint.x));

		//random points on same z plane
		double d = radius/sqrt(2);
		iPoint.x = (rand()%1000)/1000.  ;
		iPoint.y = (rand()%1000)/1000.  ;
		while(!(iPoint.y < sqrt(abs(radius*radius - iPoint.x * iPoint.x)))){
			iPoint.y = (rand()%1000)/1000.  ;
		}
		
		sight->points.push_back(iPoint);
		
//		//cout << iPoint.x << "," << iPoint.y << endl;
		
		//Project Point onto the sphere
		ikm = project2Sphere(iPoint,v,radius,was_projected);
		points[m] = ikm;
		ikm.r= 255;

		sight->points.push_back(ikm);
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
	}
////	boost::thread w1(multiKMeanValue,points,2,3,cloud,sight,1);
////	boost::thread w2(multiKMeanValue,points,3,3,cloud,sight,1);
////	multiKMeanValue(points,1,3,cloud,sight,1);
//	
////	w1.join();
////	w2.join();
//	//Excecution time
//	//t = clock() - t1;
//	//cout << "Time to execute Kmean: " << (float)t/CLOCKS_PER_SEC <<endl;
}
//	////////////////////////////////////// end of Project to Sphere test ///////////////////////////


///////////////////////////Test of get plane function and project to Sphere with this plane////////////////////////
void getPlaneAndProjectToSphere(PointXYZRGB u, PointXYZRGB v,double radius, cv::Mat ori,PointCloud<PointXYZRGB>::Ptr &sight,PointCloud<PointXYZRGB>::Ptr &sightFlat){

	//double xmin, xmax, ymin, ymax;
	PointXYZRGB xmin,xmax,ymin,ymax;
	PointXYZRGB p,ikm;
	vector<PointXYZRGB> points;
	vector<PointXYZRGB> linep(50);
	bool was_projected;
	
	
	p.x = u.x + v.x;
	p.y = u.y + v.y;
	p.z = u.z + v.z;
	hPointLine(u,p,linep);
	p.r=255;
	for(int n=0;n<linep.size();n++){
		linep[n].r = 255;
		sight->points.push_back(linep[n]);
		//cout << "p_" << n << ": " << linep[m]
	}
	
//	
	samplePlane(u,v,points,radius,sqrt(1000000)); 
//	
////	cout << "points.size in Main: " << points.size();
//	//cout << "max vector size: " << points.max_size() << endl;
	for(int n=0;n<points.size();n++){
			p = points[n];
			sight->points.push_back(p);
			ikm = project2Sphere(points[n],v,radius,was_projected);
			ikm.r = 255;
			if(was_projected){
				sight->points.push_back(ikm);
				p.r = p.b = 0;
				p.g = 255;
				sight->points.push_back(p);
				//p.y += .5;
				//sight->points.push_back(p);
				pixelInterpolate(ikm,radius,ori);
				p.r = ikm.r;
				p.g = ikm.g;
				p.b = ikm.b;
				//p.z -= 1;
				sightFlat->points.push_back(ikm);
//				ikm.x += 0.5;
				p.x -= v.x/10;
				p.y -= v.y/10;
				p.z -= v.z/10;
				//sightFlat->points.push_back(p);
				
				//sight->points.push_back(ikm);
			}
			//cout << "p_" << n << ": " << points[m] <<endl;
	}
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
}
	///////////////////////////////////End of get Plane function test ////////////////////////////////


	/////////////////////////////Test of projection with Angle from 1 point////////////////////////
void projectionWithAngleFromOnePointTest(PointXYZRGB u,PointXYZRGB v, int fac,double radius,PointCloud<PointXYZRGB>::Ptr sight ){

	double alpha1, beta1;
	PointXYZRGB p;
	vector<PointXYZRGB> points,linep;
	int alphamin, alphamax,betamin,betamax;
	bool was_projected;
	
	alphamin = 0;
	betamin = 0;
	alphamax = 90;
	betamax = 120;
	p.b=255;
	
	samplePlane(u,v,points,radius,sqrt(100000));
	for(int n=0;n<points.size();n++){
		p = points[n];
		sight->points.push_back(p);
	}
	
	p.x = u.x + v.x;
	p.y = u.y + v.y;
	p.z = u.z + v.z;
	hPointLine(u,p,linep);
	p.g = 255;
	for(int n=0;n<linep.size();n++){
		linep[n].r = 255;
		sight->points.push_back(linep[n]);
		//cout << "p_" << n << ": " << linep[m]
	}
	
	
	points.resize(2*fac*alphamax*betamax);
	
	
	
	for(int n=alphamin*fac;n<alphamax*fac;n++){
		alpha1 = n/fac;
		for(int k=betamin*fac;k<betamax*fac;k++){
			beta1 = k/fac;
			p = project2SphereWithAngle(u,v, alpha1, beta1, radius, was_projected);
			p.r = 255;
			//cout << "p:" << p;
			points[n*(betamax)+k] = p;
		}	
	}
	for(int n=0;n<points.size();n++){
		p = points[n];
		sight->points.push_back(p);
	}
	
}
	///////////////////////////////////End of get Plane function test ////////////////////////////////
	
void closestDirectionTest(double alpha, double angle){
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
}
	//////////////////////////////////Test of closest Direction ////////////////////////////////////

	
	//////////////////////////////////Test of viewing angle origin ////////////////////////////////////
void viewingAngleOriginTest(PointXYZRGB u, PointXYZRGB v, double radius, int rows, int cols,PointCloud<PointXYZRGB>::Ptr &cloud, PointCloud<PointXYZRGB>::Ptr &sightFlat){
	//Viewing angles
	double theta,phi;
	double v_angle = 40.;
	double h_angle = 27.;
	int i,j;
	//for conversion to MAT
	int imin = rows;
	int jmin = cols;
	int imax = -1;
	int jmax = -1;
	vector<PointXYZRGB> linel(50);
	vector<PointXYZRGB> liner(50);
	vector<PointXYZRGB> linep(50);
	PointXYZRGB iPoint,pro;
	
	double theta_min,theta_max,phi_min,phi_max;
	PointXYZRGB lbot,rbot,ltop,rtop;
	viewingLimitsOrigin(v, v_angle,h_angle,theta_min, theta_max, phi_min, phi_max);
//	viewingLimits(u,v, v_angle,h_angle,theta_min, theta_max, phi_min, phi_max);
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
	
	//Draw lines to origin
	
	hPointLine(u,ltop,linel);
	for(int n=0;n<linel.size();n++){
		sightFlat->points.push_back(linel[n]);

	}
	
	hPointLine(u,lbot,linel);
	for(int n=0;n<linel.size();n++){
		sightFlat->points.push_back(linel[n]);

	}
	
	hPointLine(u,rtop,linel);
	for(int n=0;n<linel.size();n++){
		sightFlat->points.push_back(linel[n]);

	}
	
	hPointLine(u,rbot,linel);
	for(int n=0;n<linel.size();n++){
		sightFlat->points.push_back(linel[n]);

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

	u.x = (ltop.x + rbot.x)/2;
	u.y = (ltop.y + rbot.y)/2;
	u.z = (ltop.z + rbot.z)/2;
	
	
	for(int n=0;n<cloud->size();n++){
		iPoint = cloud->points[n];
		
		cartesian2Spheric(iPoint,radius,theta,phi);
		if(theta>theta_min & theta<theta_max & phi<phi_max & phi>phi_min){
			sightFlat->points.push_back(iPoint);
			pro = nonOrthogonalProjection2Plane(iPoint,u,v,v);
			pixelCoordinates(iPoint.x, iPoint.y, iPoint.z, radius, rows, cols, i, j );
//			if(i<imin) imin = i;
//			if(i>imax) imax =i;
//			if(j<jmin) jmin =j;
//			if(j>jmax) jmax =j;
//			iPoint.x = i;
//			iPoint.z = j;
//			iPoint.y=0;
//			sight->points.push_back(iPoint);
			//sightFlat->points.push_back(pro);
		}
	
	}
//	cv::Mat sightMat(jmax-jmin,imax-imin,CV_8U);
//	cv::Vec3b colorSight; 
////	sightMat.convert(sightMat,CV_8U);
//	for(int ii=0;ii<sightFlat->points.size();ii++){
//		iPoint = sightFlat->points[ii];
//		colorOri = ori.at<cv::Vec3b>(iPoint.z,iPoint.x);
//		colorSight = sightMat.at<cv::Vec3b>(iPoint.z - imin, iPoint.x - jmin);
//		colorSight[0] = colorOri[0];
//		colorSight[1] = colorOri[0];
//		colorSight[2] = colorOri[0];
//	}
}
	/////////////////////////////// end Test of viewing angle origin //////////////////////////////////
	
	///////////////////////////////////////  Test point on ray ////////////////////////////////////////

void pointOnRayTest(){
	iPoint.x = 0;
	iPoint.y = 0;
	iPoint.z = 0;
	
	o.r = o.g = o.b = 255;
	sight->points.push_back(u);
	//cout << "Ipoint on Ray: " << isOnRay(o,u,v) << endl;
	//cout << "Ipoint on Ray: " << isCloseToRayCube(o,u,v,0) << endl;
	
	//for all the pt clouds converted
	
	
	for(int m=0;m<tempCount;m++){
		cloud = allPtClouds[m];
		
		for(int n=0;n<cloud->size();n++){
			iPoint = cloud->points[n];
		
			if(isCloseToRayCube(u,o,iPoint,0.5)){
				sightFlat->points.push_back(iPoint);
				//cout << "ray passing through u: " << iPoint << endl;
			}
		}
		//cout << "Number of points: " << sightFlat->size() << endl;
	}
}
	//////////////////////////////////////  end Test point on ray ////////////////////////////////////	

void EpipolarLinesTest(cv::Mat top, cv::Mat bottom){
	
//	int row = top.rows;
//	int cols = top.cols;
	cv::namedWindow("Matches",0);
	cvNamedWindow("First Image",0);
	cvNamedWindow("Second Image",0);
	cv::Mat imageMatches;
	//top = allImages[0];
	//bottom = allImages[1];
	//	cv::imshow("First Image", top);
	//	cv::imshow("Second Image", bottom);
	drawEpipolarLines(top, bottom, imageMatches);

	//Show Images

	cv::imshow("Matches",imageMatches);

	//cvResizeWindow("First Image",top.rows,top.cols);
	//cvResizeWindow("Second Image",bottom.rows,bottom.cols);
	cv::imshow("First Image", top);
	cv::imshow("Second Image", bottom);
	cv::waitKey(0);               
}
	
