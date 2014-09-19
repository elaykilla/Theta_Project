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

void delaunayTriangleTest(cv::Mat img, string name){

	cv::Mat image = img.clone();
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
	
	cout << "Number of Triangles Original " << name << " : " << triangleList.size()<< endl;
	cv::drawKeypoints( image, keypoints, image, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT );
	cv::namedWindow(name, 0);
	cv::imshow(name,image);
	
}

void delaunayMatchedTrianglesTest(cv::Mat img1, cv::Mat img2){
	cv::Mat image1, image2;
	
	image1 = img1.clone();
	image2 = img2.clone();
	vector<KeyPoint> keypoints1, keypoints2 ; 
	//vector<Point2f> points1, points2 ; 
	vector<cv::DMatch> matches;
	vector<vector<cv::DMatch> > matchedKeypoints;
	getKeypointsAndMatches(image1, image2, keypoints1, keypoints2,matches);

	vector<vector<cv::KeyPoint> > matched = getMatchedKeypoints(keypoints1, keypoints2, matches);
	cout << "DelaunayMatched Keypoints 1 Size : " << keypoints1.size() << endl;
	cout << "DelaunayMatched Keypoints 2 Size : " << keypoints2.size() << endl;
	
	
	cout << "DelaunayMatched test, number of matches : " << matches.size() << endl;
	keypoints1 = matched[0];
	keypoints2 = matched[1];
	
	
	cv::Subdiv2D subdiv1, subdiv2;
	
	subdiv1 = getDelaunayTriangles(matched[0], image1.rows, image1.cols);
	subdiv2 = getDelaunayTriangles(matched[1], image2.rows, image2.cols);
	cout << "DelaunayMatched Machted Keypoints 1 Size : " << matched[0].size() << endl;
	cout << "DelaunayMatched Matched Keypoints 2 Size : " << matched[1].size() << endl;
	
	
	vector<Vec6f> triangles1, triangles2;
	subdiv1.getTriangleList(triangles1);
	subdiv2.getTriangleList(triangles2);
	
	vector<cv::Point> pt(3);
	
	//cv::Scalar delaunay_color(255,255,255);
	
	cv::drawKeypoints( image1, matched[0], image1, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT );
	cv::drawKeypoints( image2, matched[1], image2, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT );
	for( size_t i = 0; i < triangles1.size(); i++ ){
		cv::Scalar delaunay_color(rand() % 255 + 1, rand() % 255 + 1, rand() % 255 + 1);
		Vec6f t = triangles1[i];
		pt[0] = Point(cvRound(t[0]), cvRound(t[1]));
		pt[1] = Point(cvRound(t[2]), cvRound(t[3]));
		pt[2] = Point(cvRound(t[4]), cvRound(t[5]));
		cv::line(image1, pt[0], pt[1], delaunay_color, 1, CV_AA, 0);
		cv::line(image1, pt[1], pt[2], delaunay_color, 1, CV_AA, 0);
		cv::line(image1, pt[2], pt[0], delaunay_color, 1, CV_AA, 0);
		
		
		t = triangles2[i];
		pt[0] = Point(cvRound(t[0]), cvRound(t[1]));
		pt[1] = Point(cvRound(t[2]), cvRound(t[3]));
		pt[2] = Point(cvRound(t[4]), cvRound(t[5]));
		cv::line(image2, pt[0], pt[1], delaunay_color, 1, CV_AA, 0);
		cv::line(image2, pt[1], pt[2], delaunay_color, 1, CV_AA, 0);
		cv::line(image2, pt[2], pt[0], delaunay_color, 1, CV_AA, 0);
	}
	
	cout << "Number of Triangles Matched First Image: " << triangles1.size() << endl;
	cout << "Number of Triangles Matched Second Image: " << triangles2.size() << endl;
	cv::namedWindow("first image", 0);
	cv::imshow("first image",image1);
	
	cv::namedWindow("Second Image", 0);
	cv::imshow("Second Image",image2);
}


