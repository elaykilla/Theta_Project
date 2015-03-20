#include "standard_headers.hpp" 

#include "cv_headers.hpp"
#include "pcl_headers.hpp"
#include "boost_headers.hpp"

#include"MathCalcs.hpp"
#include"OcvManip.hpp"
#include"PclManip.hpp"
#include"EquiTrans.hpp"

//////////////////////////////////////Rotate Image Test /////////////////////////////////

void rotateImageTest(cv::Mat image, double y){
	cv::Mat result;
	
	result = rotateImagey(image,y);
	
	cv::namedWindow("Original",0);
	cv::namedWindow("Rotated ",0);
	
	cv::imshow("Original", image);
	cv::imshow("Rotated ", result);

}


////////////////////////////////////// Project to Sphere test ///////////////////////////



////////////////////////////////////////Test of 3D affine transformations
void ThreeDAffineTransformTest(){
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
	

}



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
	//Viewing angles theta is on the vertical angle and phi on the horizontal angle with phi in [0,2PI]
	double theta,phi;
	double h_angle = 57.;
	double v_angle = 70.;
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
	cv::Mat sph = cv::Mat::zeros(rows, cols, CV_8UC3);
	cvNamedWindow("sph",0);

	double theta_min,theta_max,phi_min,phi_max;
	PointXYZRGB lbot,rbot,ltop,rtop;
	viewingLimitsOrigin(v, v_angle,h_angle,theta_min, theta_max, phi_min, phi_max);
	//	viewingLimits(u,v, v_angle,h_angle,theta_min, theta_max, phi_min, phi_max);

	double tempmin, tempmax;
	//The lower the point in height, the bigger theta is
	tempmin = min(theta_min,theta_max);
	tempmax = max(theta_min,theta_max);
	theta_min = tempmin;
	theta_max = tempmax;
	//	phi_min = tempmin;
	//	phi_max = tempmax; 

	//depending on the sign of the angle:


	tempmin = min(phi_min,phi_max);
	tempmax = max(phi_min,phi_max);
	phi_min = tempmin;
	phi_max = tempmax; 
	//	theta_min = tempmin;
	//	theta_max = tempmax;

	cout << "theta_min:" << theta_min*180/PI << endl;
	cout << "theta_max:" << theta_max*180/PI << endl;
	cout << "phi_min:" << phi_min*180/PI << endl;
	cout << "phi_max:" << phi_max*180/PI << endl;

	spheric2Cartesian(radius, theta_min,phi_max,rbot);
	spheric2Cartesian(radius,theta_min,phi_min,rtop);
	spheric2Cartesian(radius,theta_max,phi_max,lbot);
	spheric2Cartesian(radius,theta_max,phi_min,ltop);

	cout << "lbot:" << lbot << endl;
	cout << "ltop:" << ltop <<endl;
	cout << "rbot:" << rbot << endl;
	cout << "rtop:" << rtop <<endl;

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

	//	hPointLine(u,ltop,linel);
	//	for(int n=0;n<linel.size();n++){
	//		sightFlat->points.push_back(linel[n]);

	//	}
	//	
	hPointLine(u,lbot,linel);
	for(int n=0;n<linel.size();n++){
		sightFlat->points.push_back(linel[n]);

	}
	//	
	//	hPointLine(u,rtop,linel);
	//	for(int n=0;n<linel.size();n++){
	//		sightFlat->points.push_back(linel[n]);

	//	}

	hPointLine(u,rbot,linel);
	for(int n=0;n<linel.size();n++){
		sightFlat->points.push_back(linel[n]);

	}




	//	u.x = (ltop.x + rbot.x)/2;
	//	u.y = (ltop.y + rbot.y)/2;
	//	u.z = (ltop.z + rbot.z)/2;

	bool projected;
	double theta_prime, phi_prime;
	
	u = project2Sphere(u,v,radius,projected);
	cartesian2Spheric(u,radius,theta_prime,phi_prime);
	PointXYZRGB lbotpro = nonOrthogonalProjection2Plane(lbot,lbot,u,v);
	rotateZ(lbotpro, theta_prime);
	rotateX(lbotpro, phi_prime);
	
	double h = v_angle * rows/180;
	double w = h_angle * rows/180;
	
	double xmin, ymin, xmax, ymax;
	xmin = rows; 
	ymin = cols;
	xmax = -rows;
	ymax = -cols;
	if(v.z==0){
		//for(int n=0;n<cloud->size();n++){
		for(int j=0;j<cloud->width;j++){
			for(int i=0;i<cloud->height;i++){
				iPoint = cloud->at(j,i);

				cartesian2Spheric(iPoint,radius,theta,phi);
				//		if(n%4000==0){
				//			cout << "theta: " << theta*180/PI << endl;
				//		}
				if(inInterval(theta,theta_min,theta_max) & inInterval(phi,phi_max,phi_min)){
					//sightFlat->points.push_back(iPoint);
					pro = nonOrthogonalProjection2Plane(iPoint,iPoint,u,v);
	//				double x = pro.x*cos(theta_prime)*cos(phi_prime) 
	//					   -pro.y*sin(phi_prime)*cos(theta_prime) 
	//					   + pro.z*sin(phi_prime);
	//				double y = pro.x*sin(phi_prime)*cos(theta_prime) 
	//					  + pro.y*cos(phi_prime)*cos(theta_prime) 
	//					  - sin(theta)*pro.z;
	//				double z = - pro.x*sin(theta_prime)*cos(phi_prime) 
	//					   + pro.y*sin(theta_prime)*sin(phi_prime) 
	//					   + pro.z*cos(theta_prime);
				
	//				pro.x = x;
	//				pro.y = y;
	//				pro.z = z;
					rotateZ(pro, theta_prime);
					rotateY(pro,phi_prime);
				
					//pro.x = pro.x * v_angle * w;
					//pro.y = pro.y * h_angle * h;
					pro.z = 0;
					
					sph.at<Vec3b>(i,j)[0] = pro.b;
					sph.at<Vec3b>(i,j)[1] = pro.g;
					sph.at<Vec3b>(i,j)[2] = pro.r;
					//pixelCoordinates(iPoint.x, iPoint.y, iPoint.z, radius, rows, cols, i, j );
					if(pro.x<xmin) xmin = pro.x;
					if(pro.x>xmax) xmax = pro.x;
					if(pro.y<ymin) ymin = pro.y;
					if(pro.y>ymax) ymax = pro.y;
					//			iPoint.x = i;
					//			iPoint.z = j;
					//			iPoint.y=0;
								//sightFlat->points.push_back(iPoint);
					sightFlat->points.push_back(pro);
				}
			}
		}
		
		sightFlat->height = h;
		sightFlat -> width = w;
		cv::imshow("sph", sph);
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


////////////////////////////////////Test of EquiTrans /////////////////////////////////////////////
void EquitransTest(Mat img, double phi, double theta ){
	//img = cv::imread("test3.JPG",1);
	if(!img.data){
		cout << "EquitransTest: Please provide valid image as input" << endl;
		return;
	}
	//double focal_length = 36.0;
	double focal_length; 
	int width,height;
 	EquiTrans equi;
	//Mat pers = equi.toPerspective(img, phi, theta );



	//imwrite("./temp/persp.jpg", pers);
	
	
	//cv::namedWindow("Omni Image", 0);
	//cv::namedWindow("Perspective image", 0);	
	//imshow("Omni Image", img);	  
	//imshow("Perspective image", pers);
	//waitKey(0);
	
	//Testing Cube Faces
	//Get Params
	//equi.getCubeCam(img,width,height,focal_length);
	
	//Make faces
	Mat faces[6];
	equi.setFOV(90.0, 90.0);
	equi.makeCubeFaces(img,faces);
	
	//Display 6 faces
	cv::namedWindow("Image", 0);
	cv::namedWindow("Front", 0);
	cv::namedWindow("Back", 0);
	cv::namedWindow("Left", 0);
	cv::namedWindow("Right", 0);
	cv::namedWindow("Up", 0);
	cv::namedWindow("Down", 0);
	
	imshow("Image", img);
	imshow("Front", faces[0]);
	imshow("Back", faces[2]);
	imshow("Left", faces[3]);
	imshow("Right", faces[1]);
	imshow("Up", faces[4]);
	imshow("Down", faces[5]);	
	
	waitKey(0);
	
}
////////////////////////////////////Test of EquiTrans /////////////////////////////////////////////


///////////////////////////////////////  Test point on ray ////////////////////////////////////////

//void pointOnRayTest(){
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
//	
//	for(int m=0;m<tempCount;m++){
//		cloud = allPtClouds[m];
//		
//		for(int n=0;n<cloud->size();n++){
//			iPoint = cloud->points[n];
//		
//			if(isCloseToRayCube(u,o,iPoint,0.5)){
//				sightFlat->points.push_back(iPoint);
//				//cout << "ray passing through u: " << iPoint << endl;
//			}
//		}
//		//cout << "Number of points: " << sightFlat->size() << endl;
//	}
//}
//////////////////////////////////////  end Test point on ray ////////////////////////////////////	
void KeyPointAndMatchesTest(cv::Mat image1, cv::Mat image2){
	
	cv::Mat imageMatches;
	vector<KeyPoint> keypoints1, keypoints2 ; 
	vector<DMatch> matches;
	getKeypointsAndMatches(image1, image2, keypoints1, keypoints2,matches);

	cv::namedWindow("Matches",0);
	
	cv::drawMatches(image1,keypoints1,  // 1st image and its keypoints
			image2,keypoints2,  // 2nd image and its keypoints
			matches,                        // the matches
			imageMatches,           // the image produced
			cv::Scalar(0,255,0),DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS); 
			
	cv::imshow("Matches",imageMatches);
	cv::waitKey();
}





//////////////////////////////////////  EpipolarLinesTest ////////////////////////////////////
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
//////////////////////////////////////  End EpipolarLinesTest ////////////////////////////////////


////////////Test of circular slits///////////////////////////

void circulatSlitsTest(PointCloud<PointXYZRGB>::Ptr &cloud){
//	double newAngle;
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
//			//cloud = allPtClouds[num];

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
}
	////////////Test of circular slits///////////////////////////

void interpolate2DTest(cv::Mat image1, cv::Mat image2, double dist, double pos){
	cv::Mat interpolate = linearInterpolate(image1, image2, dist, pos);
	
	cv::namedWindow("image1",0);
	cv::namedWindow("image2",0);
	cv::namedWindow("interpolated",0);
	
	cv::imshow("image1",image1);
	cv::imshow("image2",image2);
	cv::imshow("interpolated",interpolate);
	cv::waitKey(0);  
	

}

void sphereInterpolateTest(cv::Mat image1, cv::Mat image2, double dist, double pos,PointCloud<PointXYZRGB>::Ptr &output ){
	output = sphereInterpolate(image1, image2, dist, pos);

}
void threeDKeypointsTest(PointCloud<PointXYZRGB>::Ptr &points, PointCloud<PointWithScale>::Ptr &outCloud){
	cout << "test threeDKeypoints called with cloud: " << points->size() << endl; 
	float min_scale = 0.025; //0.0005 
	int nr_octaves = 4; //4 
        int nr_scales_per_octave = 5; //5 
        float min_contrast = 1; //1 
        int k_sift = 10; 
        
         get3DKepoints(points, min_scale, nr_octaves, nr_scales_per_octave, min_contrast,outCloud);

}

void optFlowMapTest(cv::Mat image1, cv::Mat image2){
	cv::Mat flow, cflow;
	
	//Get optical flow
	optFlowMap(image1,  image2, flow);
	
	//Convert to color and draw arrows and circles
	//cv::cvtColor(image,cflow,COLOR_GRAY2BGR);
	cflow = image1.clone();
	
	int step = 16;
	cv::Scalar color(0,255,0);
	
	for(int y = 0; y < cflow.rows; y += step){
		for(int x = 0; x < cflow.cols; x += step)
		{
		    const Point2f& fxy = flow.at<Point2f>(y, x);
		    cv::line(cflow, Point(x,y), Point(cvRound(x+fxy.x), cvRound(y+fxy.y)),color);
		    cv::circle(cflow, Point(x,y), 2, color, -1);
		}
	}
	cv::namedWindow("First Image", 0);
	cv::namedWindow("Second Image", 0);
	cv::namedWindow("flow", 0);
	
	cv::imshow("First Image",image1);
	cv::imshow("Second Image",image2);
	cv::imshow("flow",cflow);
	cv::waitKey(0);

}


void getLinesTest(cv::Mat image){
	vector<cv::Vec4i> lines;
	Mat imagebw;
	lines = getLinesProb(image);
	
	cv::cvtColor(image, imagebw, CV_BGR2GRAY);
	
	for( size_t i = 0; i < lines.size(); i++ ){
	  Vec4i l = lines[i];
	  line( imagebw, Point(l[0], l[1]), Point(l[2], l[3]), cv::Scalar(0,0,255), 3, CV_AA);
	}
	
	cv::namedWindow("Image", 0);
	cv::namedWindow("Lines", 0);
	cv::imshow("Image",image);
	cv::imshow("Lines",imagebw);
	


}

/////////////////////////////////////// Testing triangulation ////////////////////////////////////////////
void delaunayTriangleTest(cv::Mat img, string name){

	cv::Mat image = img.clone();
	cv::Mat coloredtriangles(img.rows, img.cols, img.type());
	cv::Mat squares(img.rows, img.cols, img.type());
	vector<KeyPoint> keypoints  = get2DKeypoints(image);
	cout << "Number of Keypoints Original " << name << " : " << keypoints.size()<< endl;
	cv::Subdiv2D subdiv = getDelaunayTriangles(keypoints, image.rows, image.cols);
	
	vector<Vec6f> triangleList;
	subdiv.getTriangleList(triangleList);
	vector<cv::Point> pt(3);
	cv::Scalar delaunay_color(255,255,255);
	
	
	vector<cv::Scalar> colors;
	for( size_t i = 0; i < triangleList.size(); i++ ){
		cv::Scalar delaunay_color(rand() % 255 + 1, rand() % 255 + 1, rand() % 255 + 1);
		Vec6f t = triangleList[i];
		pt[0] = Point(cvRound(t[0]), cvRound(t[1]));
		pt[1] = Point(cvRound(t[2]), cvRound(t[3]));
		pt[2] = Point(cvRound(t[4]), cvRound(t[5]));
		cv::line(image, pt[0], pt[1], delaunay_color, 1, CV_AA, 0);
		cv::line(image, pt[1], pt[2], delaunay_color, 1, CV_AA, 0);
		cv::line(image, pt[2], pt[0], delaunay_color, 1, CV_AA, 0);
		colors.push_back(delaunay_color);
	}
	
	for(int i=0;i<img.rows;i++){
		for(int k=0;k<img.cols;k++){
			//if((i+k)%1000 == 0){
				cv::Point2f p;
				p.x = k;
				p.y = i;
			
			
				int j = locateTriangleIndex(subdiv,triangleList,p);
			
				if(j!=-1){
					coloredtriangles.at<Vec3b>(i,j)[0] = colors[j].val[0];
					coloredtriangles.at<Vec3b>(i,j)[1] = colors[j].val[1];
					coloredtriangles.at<Vec3b>(i,j)[2] = colors[j].val[2];	
				
				}
			
		
	
		}	
	}
	

	cv::drawKeypoints( image, keypoints, image, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT );
	cv::namedWindow(name, 0);
	cv::namedWindow("Triangles", 0);
	cv::namedWindow("Squares", 0);
	cv::imshow(name,image);
	cv::imshow("Triangles",coloredtriangles);
	cv::imshow("Squares", squares);
	
}


///////////////////////////// Test to see if using triangle boudaries works /////////////////////////////
void delaunayTriangleTestBound(cv::Mat img, string name){

	cv::Mat image = img.clone();
	cv::Mat coloredtriangles(img.rows, img.cols, img.type());
	cv::Mat squares(img.rows, img.cols, img.type());
	vector<KeyPoint> keypoints  = get2DKeypoints(image);
	cout << "Number of Keypoints Original " << name << " : " << keypoints.size()<< endl;
	cv::Subdiv2D subdiv = getDelaunayTriangles(keypoints, image.rows, image.cols);
	
	vector<Vec6f> triangleList;
	subdiv.getTriangleList(triangleList);
	vector<cv::Point> pt(3);
	cv::Scalar delaunay_color(255,255,255);
	
	for( size_t i = 0; i < triangleList.size(); i++ ){
		Vec6f t = triangleList[i];
		pt[0] = Point(cvRound(t[0]), cvRound(t[1]));
		pt[1] = Point(cvRound(t[2]), cvRound(t[3]));
		pt[2] = Point(cvRound(t[4]), cvRound(t[5]));
		cv::line(image, pt[0], pt[1], delaunay_color, 1, CV_AA, 0);
		cv::line(image, pt[1], pt[2], delaunay_color, 1, CV_AA, 0);
		cv::line(image, pt[2], pt[0], delaunay_color, 1, CV_AA, 0);
	}
	
//	for(int i=0;i<img.rows;i++){
//		for(int k=0;k<img.cols;k++){
//			//if((i+k)%1000 == 0){
//				cv::Point2f p;
//				p.x = k;
//				p.y = i;
//			
//			
//				int j = locateTriangleIndex(subdiv,triangleList,p);
//			
//				if(j!=-1){
//					Vec6f t = triangleList[j];
//					pt[0] = Point(cvRound(t[0]), cvRound(t[1]));
//					pt[1] = Point(cvRound(t[2]), cvRound(t[3]));
//					pt[2] = Point(cvRound(t[4]), cvRound(t[5]));
//				
//				}
//			
//			
////				cout << "------------------------------------" << endl;
////				cout << "Considered point p: " << p << endl;		

////			
////				cout << "Located in triangle: " << j << endl;
////				cout << "Triangle points: " << pt[0] << "|||" << pt[1] << "|||" << pt[2] << endl;
//			//}
//	
//		}	
//	}
//	
	cv::Point2f p;
	bool once = true;
	int nbTriangles = 0;
	for(int k=0;k<triangleList.size();k++){			
		Vec6f triangle = triangleList[k];
		
		cv::Point2f a,b,c; 
		a.x = triangle[0];
		a.y = triangle[1];
		b.x = triangle[2];
		b.y = triangle[3];
		c.x = triangle[4];
		c.y = triangle[5];
	
		//Get the bouding box around the triangle
		int xmax = cvRound(max(a.x, max(b.x,c.x)));
		int ymax = cvRound(max(a.y, max(b.y,c.y)));
		int xmin = cvRound(min(a.x, min(b.x,c.x)));
		int ymin = cvRound(min(a.y, min(b.y,c.y)));
	
		cv::Scalar delaunay_color(rand() % 255 + 1, rand() % 255 + 1, rand() % 255 + 1);
		for(int i=ymin;i<ymax;i++){
			for(int j=xmin;j<xmax;j++){
				if(i>0 && j>0 && j<img.cols && i<img.rows){
					p.x = j;
					p.y = i;
					
//					squares.at<Vec3b>(i,j)[0] = img.at<Vec3b>(i,j)[0];
//					squares.at<Vec3b>(i,j)[1] = img.at<Vec3b>(i,j)[1];
//					squares.at<Vec3b>(i,j)[2] = img.at<Vec3b>(i,j)[2];
//					
//					image.at<Vec3b>(i,j)[0] = delaunay_color.val[0];
//					image.at<Vec3b>(i,j)[1] = delaunay_color.val[1];
//					image.at<Vec3b>(i,j)[2] = delaunay_color.val[2];
					
					if(inTriangleArea(p,triangle)){
						if(once) {					
							nbTriangles++ ;
							once = false;
						}
						squares.at<Vec3b>(i,j)[0] = delaunay_color.val[0];
						squares.at<Vec3b>(i,j)[1] = delaunay_color.val[1];
						squares.at<Vec3b>(i,j)[2] = delaunay_color.val[2];
												
						coloredtriangles.at<Vec3b>(i,j)[0] = img.at<Vec3b>(i,j)[0];
						coloredtriangles.at<Vec3b>(i,j)[1] = img.at<Vec3b>(i,j)[1];
						coloredtriangles.at<Vec3b>(i,j)[2] = img.at<Vec3b>(i,j)[2];	
					}
				}
			}
		}
		once = true;
	}
	cout << "Number of Triangles Original " << name << " : " << triangleList.size()<< endl;
	cv::drawKeypoints( image, keypoints, image, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT );
	cv::namedWindow(name, 0);
	cv::namedWindow("Triangles", 0);
	cv::namedWindow("Squares", 0);
	cv::imshow(name,image);
	cv::imshow("Triangles",coloredtriangles);
	cv::imshow("Squares", squares);
	
}


////////////////////////////Delaunay Triangle interpolation matching Triangles test ////////////////////
void delaunayMatchedTrianglesTest(cv::Mat img1, cv::Mat img2, PointCloud<PointXYZRGB>::Ptr &sightFlat ){
	
	//For writing to files
	ofstream logFile;
	
	cv::Mat image1, image2;
	
	image1 = img1.clone();
	image2 = img2.clone();
	vector<KeyPoint> keypoints1, keypoints2 ; 
	vector<Point2f> points1, points2 ; 
	vector<cv::DMatch> matches;
	vector<vector<cv::DMatch> > matchedKeypoints;
	
	
	//Retrive keypoints from each image, and match them
	getKeypointsAndMatches(image1, image2, keypoints1, keypoints2,matches);

	//For every point in Keypoints1 -- it's matched keypoint is in Keypoints2 at the same position
	//Organise matched keypoints in order
	vector<vector<cv::KeyPoint> > matched = getMatchedKeypoints(keypoints1, keypoints2, matches);
	cout << "DelaunayMatched Keypoints 1 Size : " << keypoints1.size() << endl;
	cout << "DelaunayMatched Keypoints 2 Size : " << keypoints2.size() << endl;
	
	//Recover only points from keypoints1
	vector<vector<cv::Point2f> > matchedPts = getMatchedPoints(keypoints1, keypoints2, matches);
	
	cout << "DelaunayMatched test, number of matches : " << matches.size() << endl;
	keypoints1 = matched[0];
	keypoints2 = matched[1];
	
	points1 = matchedPts[0];
	points2 = matchedPts[1];
	cout << "DelaunayMatched Points 1 Size : " << points1.size() << endl;
	cout << "DelaunayMatched Points 2 Size : " << points2.size() << endl;
	
	logFile.open("Points1.txt");
	for(int i=0;i<points1.size();i++){
		logFile << points1[i] << endl;
	}
	
	logFile.close();
	
	cv::Subdiv2D subdiv1, subdiv2;
	
	subdiv1 = getDelaunayTriangles(keypoints1, image1.rows, image1.cols);
	subdiv2 = getDelaunayTriangles(keypoints2, image2.rows, image2.cols);
	cout << "DelaunayMatched Machted Keypoints 1 Size : " << keypoints1.size() << endl;
	cout << "DelaunayMatched Matched Keypoints 2 Size : " << keypoints1.size() << endl;
	
	
	vector<Vec6f> triangles1, triangles2;
	subdiv1.getTriangleList(triangles1);
	
	//////////////////////////////////////////////////////
	//Get triangles using subdiv
	//subdiv2.getTriangleList(triangles2);
//	//Keep only corresponding triangles
	//getCorrespondingDelaunayTriangles(keypoints1,keypoints2,triangles1,triangles2);
	//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
	
	/********************************************************/
	//Get triangles from 2 as the same triangles from 1
//	cout << "Making Triangle Correspondances..." << endl;
	makeCorrespondingDelaunayTriangles(points1,points2,triangles1,triangles2);
	cout << "Triangles in first image: " << triangles1.size() << endl;
	cout << "Triangles in second image: " << triangles2.size() << endl;
	/**********************************************************/
	
	
	vector<cv::Point> pt(3);
	
	//Get TRanforms
	vector<cv::Mat> transforms;
	
	transforms = getAffineTriangleTransforms(triangles1, triangles2);
	//cv::Scalar delaunay_color(255,255,255);
	
	vector<cv::Scalar> colors;
	
	cv::drawKeypoints( image1, matched[0], image1, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT );
	cv::drawKeypoints( image2, matched[1], image2, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT );
	cout << "Keypoint Drawn on both images" << endl;
	
	//Writing to file to test1

	logFile.open("Triangles1.txt");
	
	for(int i=0;i<keypoints1.size();i++){
		logFile << keypoints1[i].pt << " -------> " <<keypoints2[i].pt << endl;
	
	}
	logFile << "=======================================================================" << endl << endl << endl ;
	//End of log testing
	
	cout << "Beginning triangles drawing..." << endl;
	for( size_t i = 0; i < triangles1.size(); i++ ){
		cv::Scalar delaunay_color(rand() % 255 + 1, rand() % 255 + 1, rand() % 255 + 1);
		Vec6f t = triangles1[i];
		pt[0] = Point(cvRound(t[0]), cvRound(t[1]));
		pt[1] = Point(cvRound(t[2]), cvRound(t[3]));
		pt[2] = Point(cvRound(t[4]), cvRound(t[5]));
		cv::line(image1, pt[0], pt[1], delaunay_color, 1, CV_AA, 0);
		cv::line(image1, pt[1], pt[2], delaunay_color, 1, CV_AA, 0);
		cv::line(image1, pt[2], pt[0], delaunay_color, 1, CV_AA, 0);
		logFile << "Points Forming Triangles1 " << i << " : " << pt[0] << "," << pt[1] << "," << pt[2] << endl;
		
		
		t = triangles2[i];
		pt[0] = Point(cvRound(t[0]), cvRound(t[1]));
		pt[1] = Point(cvRound(t[2]), cvRound(t[3]));
		pt[2] = Point(cvRound(t[4]), cvRound(t[5]));
		cv::line(image2, pt[0], pt[1], delaunay_color, 1, CV_AA, 0);
		cv::line(image2, pt[1], pt[2], delaunay_color, 1, CV_AA, 0);
		cv::line(image2, pt[2], pt[0], delaunay_color, 1, CV_AA, 0);
		//logFile << "Matching Points Forming Triangles2 " << i << " : "<< pt[0] << "," << pt[1] << "," << pt[2] << endl;
		logFile << "-----------------------------------------------------------------------------------------"<< endl;
		
		colors.push_back(delaunay_color);
	}
	
	cout << "Triangles finished drawing on both images" << endl;
	logFile.close();
	
	cv::Mat tWarp(2,3,CV_32FC1) ;
	cv::Mat result = cv::Mat::zeros(img1.rows*2,img1.cols*2,img1.type());
	cv::Mat resultT = cv::Mat::zeros(img1.rows*2,img1.cols*2,img1.type());
	cv::Point2f p;
	PointXYZRGB p1;
	
	
	cout << "Beginning Image point calculations" << endl;
	logFile.open("Warp_log.txt");
	for(int i=0;i<img2.rows;i++){
		for(int j=0;j<img2.cols;j++){
			//if((i+k)%1000 == 0){
				
				p.x = j;
				p.y = i;
				//cout << "Initial point: " << p << endl;
			
				int k = locateTriangleIndex(subdiv2,triangles2, p);
				logFile << "Triangles number: " << k << endl;
				//cout << "Triangles number: " << k << endl;
				//int k = 1;
				if(k!=-1){
//					//cv::Scalar delaunay_color(rand() % 255 + 1, rand() % 255 + 1, rand() % 255 + 1);
//					//cout << "triange: " << j << endl;
//					
					tWarp = transforms[k];
					double* uprow = tWarp.ptr<double>(0);
					double* downrow = tWarp.ptr<double>(1);
//					
					logFile << "Warp Matrix: " << tWarp << endl;
//					//cout << "Warp Matrix: " << tWarp << endl;
					float x = uprow[0] * p.x + uprow[1]* p.y + uprow[2];
					float y = downrow[0] * p.x + downrow[1]* p.y + downrow[2];
					p1.x = x;
					p1.y = y;
					p1.b = img2.at<Vec3b>(i,j)[0];
					p1.g = img2.at<Vec3b>(i,j)[1];
					p1.r = img2.at<Vec3b>(i,j)[2];
					sightFlat->points.push_back(p1);
					
					if(x>=0 & y>=0 & x<result.cols & y<result.rows){
						x = cvRound(x);
						y = cvRound(y);
						logFile << "original: " << p << " ------>> " << "calculated: (" << x << "," << y << ")" << endl;
						//cout<< "calculated coordinates: " << x << "," << y << endl;
						result.at<Vec3b>(y,x)[0] = img2.at<Vec3b>(i,j)[0];
						result.at<Vec3b>(y,x)[1] =  img2.at<Vec3b>(i,j)[1];
						result.at<Vec3b>(y,x)[2] =  img2.at<Vec3b>(i,j)[2];
						
						
						

						resultT.at<Vec3b>(y,x)[0] = colors[k].val[0];
						resultT.at<Vec3b>(y,x)[1] =  colors[k].val[1];
						resultT.at<Vec3b>(y,x)[2] =  colors[k].val[2];


//						//result.at<Vec3b>(k,i)[2] = img2.at<Vec3b>(k,i)[2];
//	//					Vec6f t = triangles1[j];
//	//					pt[0] = Point(cvRound(t[0]), cvRound(t[1]));
//	//					pt[1] = Point(cvRound(t[2]), cvRound(t[3]));
//	//					pt[2] = Point(cvRound(t[4]), cvRound(t[5]));
					}
				}
			
				
				//cout << "------------------------------------" << endl;
				//cout << "Considered point p: " << p << endl;		

			
				//cout << "Located in triangle: " << j << endl;
				//cout << "Triangle points: " << pt[0] << "|||" << pt[1] << "|||" << pt[2] << endl;
			//}
	
		}
		//cout << "Considered point p: " << p << endl;	
//		
	}
	logFile.close();
	
	cout << "Warp Matrix: " << tWarp << endl;
	
	cout << "used matrix: " << tWarp.at<float>(0,0) << "," << tWarp.at<float>(0,1) << "," << tWarp.at<float>(0,2) << endl;
	cout << tWarp.at<float>(1,0) << "," << tWarp.at<float>(1,1) << "," << tWarp.at<float>(1,2) << endl;
	cout << "Number of Triangles Matched First Image: " << triangles1.size() << endl;
	cout << "Number of Triangles Matched Second Image: " << triangles2.size() << endl;
	cv::namedWindow("first image", 0);
	cv::imshow("first image",image1);
	
	cv::namedWindow("Second Image", 0);
	cv::imshow("Second Image",image2);
	
	cv::namedWindow("Result", 0);
	cv::imshow("Result", result);
	
	cv::namedWindow("Triangles Result", 0);
	cv::imshow("Triangles Result", resultT);
	
	return;
}

//////////////////////////////Delaumay Interpolation test using Triangle Boudaries//////////////////////
void delaunayMatchedTrianglesBoundTest(cv::Mat img1, cv::Mat img2, PointCloud<PointXYZRGB>::Ptr &sightFlat ){
	cv::Mat image1(img1.rows, img1.cols + img1.cols/2,img1.type());
	cv::Mat image2(img2.rows, img2.cols + img2.cols/2,img2.type());
	cv::Mat result = cv::Mat::zeros(img1.rows,img1.cols + img1.cols/2,img1.type());
	cv::Mat resultT = cv::Mat::zeros(img1.rows,img1.cols,img1.type());
	cv::Subdiv2D subdiv1;
	image1 = img1.clone();
	image2 = img2.clone();
	vector<cv::KeyPoint> keypoints1, keypoints2 ; 
	vector<cv::Point2f> points1, points2;
	//Extend the image 2 to the left
	//img1.copyTo(image1(cv::Rect(cv::Point(0, 0), img1.size())));
	//img2.copyTo(image2(cv::Rect(cv::Point(0, 0), img2.size())));
	
	//Add half of the image to the right side of the second image for boundary consistency
	vector<cv::DMatch> matches;
	
	
	
	
	//Retrive keypoints from each image, and match them
	getKeypointsAndMatches(image1, image2, keypoints1, keypoints2,matches);

	//For every point in Keypoints1 -- it's matched keypoint is in Keypoints2 at the same position
	vector<vector<cv::KeyPoint> > matched = getMatchedKeypoints(keypoints1, keypoints2, matches);
	vector<vector<cv::Point2f> > matchedPts = getMatchedPoints(keypoints1, keypoints2, matches);
	
	
	cout << "DelaunayMatched test, number of matches : " << matches.size() << endl;
	keypoints1 = matched[0];
	keypoints2 = matched[1];
	
	points1 = matchedPts[0];
	points2 = matchedPts[1];
	
	subdiv1 = getDelaunayTriangles(matched[0], image1.rows, image1.cols);

	//Fill the right side with values of left side
	int io = img2.rows;
	int jo = img2.cols;
	
//	for(int i=0;i<io;i++){
//		for(int j=jo;j<jo+jo/2;j++){
//			//cout << "i,j" << i << "," << j << endl;
//			image2.at<Vec3b>(i,j)[0] = img2.at<Vec3b>(i, j-jo)[0];
//			image2.at<Vec3b>(i,j)[1] = img2.at<Vec3b>(i, j-jo)[1];
//			image2.at<Vec3b>(i,j)[2] = img2.at<Vec3b>(i, j-jo)[2];
//		}
//	}
	
	//Go through all the matched keypoints, and if the point is on the first half, put it on the 		//other side if the distance to the point in the 1st image is more than 1/3 of image size
	double max_dist = sqrt(img1.rows*img1.rows + img1.cols*img1.cols)/4;
	for(int i=0; i<points2.size();i++){
		PointXYZRGB p1,p2;
		p1.x = points1[i].x;
		p1.y = points1[i].y;
		
		p2.x = points2[i].x;
		p2.y = points2[i].y;
		
		if (distanceP(p1,p2)>max_dist){
			points2[i].x += img2.cols;
		} 
	}
	
	
	
	vector<Vec6f> triangles1, triangles2;
	subdiv1.getTriangleList(triangles1);
	cout << "Number of triangles before correspance: " << triangles1.size() << endl;
	
	
	//Make matched triangles in image 2
	makeCorrespondingDelaunayTriangles(points1, points2, triangles1,triangles2);
	cout << "Number of triangles after correspondance: " << triangles1.size() << endl;
	
	//subdiv2.getTriangleList(triangles2);

	ofstream logFile;
	vector<cv::Point2f> pt(3);

	
	vector<cv::Mat> transforms;
	
	//Get the affine transform between each of the triangles
	transforms = getAffineTriangleTransforms(triangles1, triangles2);
	//cv::Scalar delaunay_color(255,255,255);
	
	vector<cv::Scalar> colors;
	
	cv::drawKeypoints( image1, matched[0], image1, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT );
	cv::drawKeypoints( image2, matched[1], image2, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT );
	
	
	
	//Writing to file to test1
	logFile.open("T_log2.txt");
	for(int i=0;i<keypoints1.size();i++){
		logFile << keypoints1[i].pt << " -------> " <<keypoints2[i].pt << endl;
	
	}
	logFile << "=======================================================================" << endl << endl << endl ;
	//End of log testing
	
	for( size_t i = 0; i < triangles1.size(); i++ ){
		cv::Scalar delaunay_color(rand() % 255 + 1, rand() % 255 + 1, rand() % 255 + 1);
		Vec6f t = triangles1[i];
		pt[0] = Point(cvRound(t[0]), cvRound(t[1]));
		pt[1] = Point(cvRound(t[2]), cvRound(t[3]));
		pt[2] = Point(cvRound(t[4]), cvRound(t[5]));
		cv::line(image1, pt[0], pt[1], delaunay_color, 1, CV_AA, 0);
		cv::line(image1, pt[1], pt[2], delaunay_color, 1, CV_AA, 0);
		cv::line(image1, pt[2], pt[0], delaunay_color, 1, CV_AA, 0);
		logFile << "Points Forming Triangles1 " << i << " : " << pt[0] << "," << pt[1] << "," << pt[2] << endl;
		
		//Draw the line on resulting image
		//cv::line(result, pt[0], pt[1], delaunay_color, 1, CV_AA, 0);
		//cv::line(result, pt[1], pt[2], delaunay_color, 1, CV_AA, 0);
		//cv::line(result, pt[2], pt[0], delaunay_color, 1, CV_AA, 0);
		
		t = triangles2[i];
		pt[0] = Point(cvRound(t[0]), cvRound(t[1]));
		pt[1] = Point(cvRound(t[2]), cvRound(t[3]));
		pt[2] = Point(cvRound(t[4]), cvRound(t[5]));
		cv::line(image2, pt[0], pt[1], delaunay_color, 1, CV_AA, 0);
		cv::line(image2, pt[1], pt[2], delaunay_color, 1, CV_AA, 0);
		cv::line(image2, pt[2], pt[0], delaunay_color, 1, CV_AA, 0);
		logFile << "Matching Points Forming Triangles2 " << i << " : "<< pt[0] << "," << pt[1] << "," << pt[2] << endl;
		
		cv::Mat tWarp = transforms[i];
		
		logFile << "Affine Transformation matrix" << tWarp << endl;
		logFile << "-----------------------------------------------------------------------------------------"<< endl;
		
		colors.push_back(delaunay_color);
	}
	logFile.close();
	
	cv::Mat tWarp(2,3,CV_32FC1) ;
	
	cv::Point2f p;
	PointXYZRGB p1;
	
	
	cout << "Beginning Image point calculations" << endl;
	int nbTriangles = 0;
	bool once = true;
	for(int k=0;k<triangles1.size();k++){
		tWarp = transforms[k];
		double* uprow = tWarp.ptr<double>(0);
		double* downrow = tWarp.ptr<double>(1);
//					
		Vec6f triangle = triangles2[k];
		Vec6f triangleOri = triangles1[k];
		
		cv::Point2f a,b,c; 
		a.x = triangle[0];
		a.y = triangle[1];
		b.x = triangle[2];
		b.y = triangle[3];
		c.x = triangle[4];
		c.y = triangle[5];
	
		//Get the bouding box around the triangle
		int xmax = cvRound(max(a.x, max(b.x,c.x)));
		int ymax = cvRound(max(a.y, max(b.y,c.y)));
		int xmin = cvRound(min(a.x, min(b.x,c.x)));
		int ymin = cvRound(min(a.y, min(b.y,c.y)));
	
		
		cv::Point2f p,pOri;
		PointXYZRGB p1;
		for(int i=ymin;i<=ymax;i++){
			for(int j=xmin;j<=xmax;j++){
				p.x = j;
				p.y = i;
				
				
				if(i>0 && j>0 && j<img2.cols && i<img2.rows){
					float x = uprow[0] * p.x + uprow[1]* p.y + uprow[2];
					float y = downrow[0] * p.x + downrow[1]* p.y + downrow[2];
//					p1.x = x;
//					p1.y = -y; 
//					p1.b = img2.at<Vec3b>(i,j)[0];
//					p1.g = img2.at<Vec3b>(i,j)[1];
//					p1.r = img2.at<Vec3b>(i,j)[2];
//					sightFlat->points.push_back(p1);
				
					if(inTriangleArea(p,triangle)){
						if(once) {					
							nbTriangles++ ;
							once = false;
						}
						float x = uprow[0] * p.x + uprow[1]* p.y + uprow[2];
						float y = downrow[0] * p.x + downrow[1]* p.y + downrow[2];
						p1.x = x;
						p1.y = -y;
						p1.b = img2.at<Vec3b>(i,j)[0];
						p1.g = img2.at<Vec3b>(i,j)[1];
						p1.r = img2.at<Vec3b>(i,j)[2];
						//sightFlat->points.push_back(p1);
					
						
						//For copying into image
						x = cvRound(x);
						y = cvRound(y);
						pOri.x = x;
						pOri.y = y;
						if(inTriangleArea(pOri,triangleOri)){
							//sightFlat->points.push_back(p1);
							result.at<Vec3b>(y,x)[0] = img1.at<Vec3b>(i,j)[0];
							result.at<Vec3b>(y,x)[1] = img1.at<Vec3b>(i,j)[1];
							result.at<Vec3b>(y,x)[2] = img1.at<Vec3b>(i,j)[2];
							
//							result.at<Vec3b>(y,x)[0] = img2.at<Vec3b>(i,j)[0] - img1.at<Vec3b>(y,x)[0];
//							result.at<Vec3b>(y,x)[1] = img2.at<Vec3b>(i,j)[1] - img1.at<Vec3b>(y,x)[1];
//							result.at<Vec3b>(y,x)[2] = img2.at<Vec3b>(i,j)[2] - img1.at<Vec3b>(y,x)[2];
						
							//Image with just triangles
//							if(inTriangleArea(pOri,triangleOri)){
//								resultT.at<Vec3b>(y,x)[0] = img2.at<Vec3b>(i,j)[0]- img1.at<Vec3b>(y,x)[0];
//								resultT.at<Vec3b>(y,x)[1] = img2.at<Vec3b>(i,j)[1]- img1.at<Vec3b>(y,x)[1];
//								resultT.at<Vec3b>(y,x)[2] = img2.at<Vec3b>(i,j)[2]- img1.at<Vec3b>(y,x)[2];
//							}
						}	
					}
				}
			}
		}
		once = true;
	}
	//sightFlat = EquiToSphere(resultT, 1,0,0,0);

	cv::namedWindow("first image", 0);
	cv::imshow("first image",image1);
	
	cv::namedWindow("Second Image", 0);
	cv::imshow("Second Image",image2);
	
	cv::namedWindow("Result", 0);
	cv::imshow("Result", result);
	
//	cv::namedWindow("Result Triangles", 0);
//	cv::imshow("Result Triangles", resultT);
	
	return;
}

//void testAffineTransform2D(cv::Mat img1, cv::Mat img2){
//cv::Mat image1;
//	//Extend second Image beyond border for border triangles
//	cv::Mat image2(img2.rows, img2.cols + img2.cols/2,img2.type());

//	//Resulting image
//	cv::Mat result = cv::Mat::zeros(img1.rows,img1.cols + img1.cols/2,img1.type());

//	//Create Subdivision of Image1
//	cv::Subdiv2D subdiv1;
//	image1 = img1.clone();
//	img2.copyTo(image2(cv::Rect(cv::Point(0, 0), img2.size())));
//	//Fill the right side with values of left side to complete second image
//	int io = img2.rows;
//	int jo = img2.cols;

//	


//	//Vectors for keypoints
//	vector<cv::KeyPoint> keypoints1, keypoints2 ; 
//	vector<cv::Point2f> points1, points2;



//	//Vector for matches
//	vector<cv::DMatch> matches;




//	//Retrive keypoints from each image, and match them
//	getKeypointsAndMatches(image1, image2, keypoints1, keypoints2,matches);

//	
//	
//	
//	//For every point in Keypoints1 -- it's matched keypoint is in Keypoints2 at the same position
//	vector<vector<cv::KeyPoint> > matched = getMatchedKeypoints(keypoints1, keypoints2, matches);
//	vector<vector<cv::Point2f> > matchedPts = getMatchedPoints(keypoints1, keypoints2, matches);

//	//Matched keypoints only
//	keypoints1 = matched[0];
//	keypoints2 = matched[1];

//	//Extracted points from the Keypoints
//	points1 = matchedPts[0];
//	points2 = matchedPts[1];

//	//Create Delaunay triangulation of the first image
//	subdiv1 = getDelaunayTriangles(matched[0], image1.rows, image1.cols);

//	//Extend 2nd image by half of image size
//	for(int i=0;i<io;i++){
//		for(int j=jo;j<jo+jo/2;j++){
//			//cout << "i,j" << i << "," << j << endl;
//			image2.at<Vec3b>(i,j)[0] = img2.at<Vec3b>(i, j-jo)[0];
//			image2.at<Vec3b>(i,j)[1] = img2.at<Vec3b>(i, j-jo)[1];
//			image2.at<Vec3b>(i,j)[2] = img2.at<Vec3b>(i, j-jo)[2];
//		}
//	}
//	
//	//Go through all the matched keypoints, and if the point is on the first half, put it on the 		//other side if the distance to the point in the 1st image is more than 1/3 of image size
//	double max_dist = sqrt(img1.rows*img1.rows + img1.cols*img1.cols)/4;
//	for(int i=0; i<points2.size();i++){
//		PointXYZRGB p1,p2;
//		p1.x = points1[i].x;
//		p1.y = points1[i].y;
//		
//		p2.x = points2[i].x;
//		p2.y = points2[i].y;
//		
//		if (distanceP(p1,p2)>max_dist){
//			points2[i].x += img2.cols;
//		} 
//	}
//	
//	//Vectors for the triangles
//	vector<Vec6f> triangles1, triangles2;

//	//Retrive the triangles from Image1
//	subdiv1.getTriangleList(triangles1);

//	//Make matched triangles in image 2
//	makeCorrespondingDelaunayTriangles(points1, points2, triangles1,triangles2);

//	//Vector for the transformations between all triangles
//	vector<cv::Mat> transforms;

//	//Get the affine transform between each of the triangles
//	transforms = getAffineTriangleTransforms(triangles1, triangles2);


//	cv::Mat tWarp(2,3,CV_32FC1) ;
//	cv::Point2f p;
//	
//	
//	int nbErrors = 0;
//	double epsilon = 0.01;
//	for(int k=0;k<triangles1.size();k++){
//	//for(int k=10;k<11;k++){
//		tWarp = transforms[k];
//		//cout << "transform matrix: " << tWarp << endl;
//		double* uprow = tWarp.ptr<double>(0);
//		double* downrow = tWarp.ptr<double>(1);
//		
//		//Triangle from Image1					
//		Vec6f triangle = triangles1[k];
//		
//		//Triangle from image2
//		Vec6f triangle2 = triangles2[k];
//		
//		cv::Point2f a,b,c,a1,b1,c1,acal,bcal,ccal; 
//		a.x = triangle[0];
//		a.y = triangle[1];
//		b.x = triangle[2];
//		b.y = triangle[3];
//		c.x = triangle[4];
//		c.y = triangle[5];
//		

//		
//		//////For printing purposes
//		a1.x = triangle2[0];
//		a1.y = triangle2[1];
//		b1.x = triangle2[2];
//		b1.y = triangle2[3];
//		c1.x = triangle2[4];
//		c1.y = triangle2[5];
//	

//		
//		
//		//Picture 1 Triangles
//		delaunay_color = cv::Scalar(255, 0, 0);
//		cv::line(result, a, b, delaunay_color, 1, CV_AA, 0);
//		cv::line(result, b, c, delaunay_color, 1, CV_AA, 0);
//		cv::line(result, c, a, delaunay_color, 1, CV_AA, 0);
//		
//		//Pic 2 triangles1
//		delaunay_color = cv::Scalar(0, 0, 255);
//		cv::line(result, a1, b1, delaunay_color, 1, CV_AA, 0);
//		cv::line(result, b1, c1, delaunay_color, 1, CV_AA, 0);
//		cv::line(result, c1, a1, delaunay_color, 1, CV_AA, 0);
//		
//		

//		
//		
//		//Apply transform to triangle points
//		acal.x = uprow[0] * a.x + uprow[1]* a.y + uprow[2];
//		acal.y = downrow[0] * a.x + downrow[1]* a.y + downrow[2];	
//		bcal.x = uprow[0] * b.x + uprow[1]* b.y + uprow[2];
//		bcal.y = downrow[0] * b.x + downrow[1]* b.y + downrow[2];	
//		ccal.x = uprow[0] * c.x + uprow[1]* c.y + uprow[2];
//		ccal.y = downrow[0] * c.x + downrow[1]* c.y + downrow[2];
//		
//		cout << "Triangle " << k << "errors" << endl;
//		if(!abs(acal.x-a1.x)<=epsilon || !abs(acal.y-a1.y)<=epsilon){
//			cout << "Error, expected: " << a1 << "calculated: " << acal << endl;
//			cout << "============================"
//		}
//		if(!abs(bcal.x-b1.x)<=epsilon || !abs(bcal.y-b1.y)<=epsilon){
//			cout << "Error, expected: " << b1 << "calculated: " << bcal << endl;
//			cout << "============================"
//		}
//		if(!abs(acal.x-a1.x)<=epsilon || !abs(acal.y-a1.y)<=epsilon){
//			cout << "Error, expected: " << c1 << "calculated: " << ccal << endl;
//			cout << "============================"
//		}	
//	}
	
	
//} 

////////////////////////////////////Test Multiple Interpolation /////////////////////////////////////
void multipleInterpolateTest(Mat ori, Mat templ, int nb_inter){
	cv::Mat result;
	for(int i=0;i<nb_inter;i++){
		ostringstream nameWindow;
		nameWindow << "temp/Interpolated Image_"<< i ;
		cout << nameWindow.str() << endl;
		cv::Mat result = delaunayInterpolate(ori,templ,1,i/(double)nb_inter);
		//cv::Mat result = delaunayInterpolateSphere(ori,templ,1,i/(double)nb_inter);
		//cv::Mat result = interpolated[i];
		//cv::namedWindow(nameWindow.str(), 0);
		//cv::imshow(nameWindow.str(), result);
		nameWindow << ".jpg" ;
		cv::imwrite(nameWindow.str(),result);	
		//images[i] = result;
		//images.push_back(result);
	}
	
	
	//Read and write files from tmp
	//for(i=0;i<nb_inter;i++){
	//	ostringstream nameWindow;
	//	nameWindow << "temp/Interpolated Image_"<< i << ".jpg" ;
		//nameWindow << "Bottom/Bottom"<< i+1 << ".jpg" ;
	//	Mat image = cv::imread(nameWindow.str(),1);
		//if(!image.data){
		//	cout << "Please verify image names" << endl;
	//		break;
		//}
	//	else{
	//		images.push_back(image);
	//	}
	//}
	//string videoName = "temp/Interpolated Video" ;
	//imageListToVideo(images,videoName);
}

////////////////////////////////////End Test Multiple Interpolation ////////////////////////////////

//////////////////////////////////// Test of Ply Writer ///////////////////////////////////
void plyWriterTest(Mat img1, Mat img2){
	//Vectors for keypoints
	vector<cv::KeyPoint> keypoints1, keypoints2 ; 
	vector<cv::Point2f> points1, points2;

	//Vector of 3D points and cloud
	vector<PointXYZRGB> points3D;
	PointCloud<PointXYZRGB>::Ptr cloud;
	
	//Vector for matches
	vector<cv::DMatch> matches;

	//Retrive keypoints from each image, and match them
	getKeypointsAndMatches(img1, img2, keypoints1, keypoints2,matches);

	//For every point in Keypoints1 -- it's matched keypoint is in Keypoints2 at the same position
	vector<vector<cv::KeyPoint> > matched = getMatchedKeypoints(keypoints1, keypoints2, matches);
	vector<vector<cv::Point2f> > matchedPts = getMatchedPoints(keypoints1, keypoints2, matches);

	//Matched keypoints only
	keypoints1 = matched[0];
	keypoints2 = matched[1];

	//Extracted points from the Keypoints
	points1 = matchedPts[0];
	points2 = matchedPts[1];
	
	//points3D = sphereCoordinatesList(img1.rows,img1.cols,points);
	//cloud->points = points3D;
	
}

//////////////////////////////////// End Test of Ply Writer ///////////////////////////////////



void testCloudObj(PointCloud<PointXYZRGB>::Ptr cloud){
	//For writing to files
	ofstream logFile;
	
	//Export 3D points
	logFile.open("Full_Image.obg");
	
	PointXYZRGB p3d;
	int b,g,r;
	for(int i=0;i<cloud->points.size();i++){
		
		p3d = cloud->points[i];
		
		b = p3d.b;
		g = p3d.g;
		r = p3d.r;
		//cloud->points.push_back(p3d);
		logFile << "v"  << " " << p3d.x << " " << p3d.y << " " << p3d.z << " " << r/255. << " " << g/255. << " " << b/255. << endl;
	
	}
	
	//Add normals
	for(int i=0;i<cloud->points.size();i++){
		
		p3d = cloud->points[i];
		
		//b = p3d.b;
		//g = p3d.g;
		//r = p3d.r;
		//cloud->points.push_back(p3d);
		logFile << "vn"  << " " << p3d.x << " " << p3d.y << " " << p3d.z << endl;
	
	}
	
	
	logFile.close();
}
//////////////////////////////////// End Test of 2D to 3D keypoints ///////////////////////////////////

//////////////////////////////////// Test of 3D Triangulation ///////////////////////////////////
void test3DTriangulation(PointCloud<PointXYZRGB>::Ptr cloud){

	cout << "test3DTriangulation: Started with cloud: " << cloud->points.size() << " points" << endl;
	//Compute normals for each point
	PointCloud<Normal>::Ptr normals (new PointCloud<Normal>); 
	PointXYZRGB p; 
	//Normal n;
	
	cout << "test3DTriangulation: Creating Normals" << endl;
	for(int i=0; i<cloud->points.size();i++){
		Normal n;
		p = cloud->points[i];
		
		//Normal n(p.x,p.y,p.z);
		n.normal[0]= p.x;
		n.normal[1] = p.y;
		n.normal[2] = p.z;
		normals->points.push_back(n);
	}
	
	cout << "test3DTriangulation: Normals created" << endl;
	
	//Compute triangles
	cout << "Computing Triangles unsing greedyProjection" << endl;
	GreedyProjectionTriangulation<PointXYZRGBNormal> gp3 ;
	gp3 = get3DTriangulation(cloud,normals,1.);
	
	PolygonMesh triangles;
	gp3.reconstruct(triangles);
	
	io::saveVTKFile("temp/triangleMesh.vtk", triangles);
}
//////////////////////////////////// End Test of 3D Triangulation ///////////////////////////////////

//////////////////////////////////// Test of 2D to 3D keypoints ///////////////////////////////////
void twoDToThreeDkeypoints(Mat img){
	cout << "Launched twoDToThreeDkeypoints" << endl; 
	//Keypoints vector
	vector<cv::KeyPoint> keypoints;
	
	//2D points
	std::vector<cv::Point2f> points;
	
	//3D points
	vector<PointXYZRGB> points3D;
	
	//PointCloud
	PointCloud<PointXYZRGB>::Ptr cloud (new PointCloud<PointXYZRGB>);
	
	//Compute keypoints, extract just points and convert to 3D points
	cout << "twoDToThreeDkeypoints: Extracting Keypoints" << endl;
	keypoints = get2DKeypoints(img);
	cout << "Extracted " << keypoints.size() << " keypoints" << endl;
	
	cout << "twoDToThreeDkeypoints: Converting to 2D points" << endl;
	points = convertKeypoints(keypoints);
	
	cout << "twoDToThreeDkeypoints: Converting to 3D points" << endl;
	points3D = sphereCoordinatesList(img.rows,img.cols,points);
	
	
	cout << "twoDToThreeDkeypoints: Creating point cloud" << endl;
	
	//For writing to files
	ofstream logFile;
	
	//Export 3D points
	logFile.open("Keypoints.obj");
	
	
	cv::Point2f p2d; 
	PointXYZRGB p3d;
	int r,g,b;
	for(int i=0;i<points3D.size();i++){
		p2d = points[i];
		p3d = points3D[i];
		
		b = img.at<cv::Vec3b>(p2d)[0];
		g = img.at<cv::Vec3b>(p2d)[1];
		r = img.at<cv::Vec3b>(p2d)[2];
		cloud->points.push_back(p3d);
		logFile << "v"  << " " << p3d.x << " " << p3d.y << " " << p3d.z << " " << r/255. << " " << g/255. << " " << b/255. << endl;
	
	}
	
	
	cout << "twoDToThreeDkeypoints: Created point cloud" << endl;
	//Add normals
	for(int i=0;i<points3D.size();i++){
		
		p3d = points3D[i];
		
		//b = p3d.b;
		//g = p3d.g;
		//r = p3d.r;
		//cloud->points.push_back(p3d);
		logFile << "vn"  << " " << p3d.x << " " << p3d.y << " " << p3d.z << endl;
	
	}
	
	//Create mesh for keypoints
	test3DTriangulation(cloud);
	
	logFile.close();
	//cloud->points = points3D;
	
	//For visualizing
	cout << "twoDToThreeDkeypoints: Creating Visualizer" << endl;
	visualization::PCLVisualizer viewer("3D viewer");
	viewer.setBackgroundColor (0, 0, 0);
	
	viewer.addPointCloud(cloud, "keypoints");
	viewer.setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 1, "keypoints");
	
	while (!viewer.wasStopped ()){
		// This seems to cause trouble when having cloud viewer and viewr running
		//cv::imshow("Keypoints 2D" , sightMat);
		viewer.spinOnce (100);

		//cv::waitKey(0);
//		boost::this_thread::sleep (boost::posix_time::microseconds (10000));
	}
}
//////////////////////////////////// end of Test of 2D to 3D keypoints ///////////////////////////////////






