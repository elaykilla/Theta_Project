/*
 * @author: ELay Maiga
 * This class contains all the methods related to calculating coordinates and making projections
 * from 2D to 3D or vice-versa.
 */


#include"OcvManip.hpp"



/***************************************************Image Manipulation Cv***********************************************/

/**
 * This function takes the image prefix name, adds the position i and saves in a cv::Mat
 *
 *
 */
void loadImagei(string name, int i, cv::Mat &image){
	ostringstream file;
	file << name <<i << ".jpg" ; 
	image = cv::imread(file.str(),1);
}

/** 
 * Load top and bottom images
 */
void loadImageTop(string name, cv::Mat &image, string topOrbottom){
	ostringstream file;
	file << name <<topOrbottom << ".jpg" ; 
	image = cv::imread(file.str(),1);
}


/**
 * This function returns a cv::vector containing the Keypoints from the input image using SURF
 */
vector<cv::KeyPoint> get2DKeypoints(cv::Mat image){
	//-- Step 1: Detect the keypoints using SURF Detector
	int minHessian = 400;

	cv::SurfFeatureDetector detector( minHessian );
	vector<cv::KeyPoint> keypoints;

	detector.detect( image, keypoints );

	return keypoints;
}

int ratioTest(std::vector<std::vector<cv::DMatch> >& matches, float ratio) {

	int removed=0;

	// for all matches
	for (std::vector<std::vector<cv::DMatch> >::iterator matchIterator= matches.begin();
			matchIterator!= matches.end(); ++matchIterator) {

		// if 2 NN has been identified
		if (matchIterator->size() > 1) {

			// check distance ratio
			if ((*matchIterator)[0].distance/(*matchIterator)[1].distance > ratio) {

				matchIterator->clear(); // remove match
				removed++;
			}

		} else { // does not have 2 neighbours

			matchIterator->clear(); // remove match
			removed++;
		}
	}

	return removed;
}


// Insert symmetrical matches in symMatches vector
void symmetryTest(const std::vector<std::vector<cv::DMatch> >& matches1,
		const std::vector<std::vector<cv::DMatch> >& matches2,
		std::vector<cv::DMatch>& symMatches) {

	// for all matches image 1 -> image 2
	for (std::vector<std::vector<cv::DMatch> >::const_iterator matchIterator1= matches1.begin();
			matchIterator1!= matches1.end(); ++matchIterator1) {

		if (matchIterator1->size() < 2) // ignore deleted matches 
			continue;

		// for all matches image 2 -> image 1
		for (std::vector<std::vector<cv::DMatch> >::const_iterator matchIterator2= matches2.begin();
				matchIterator2!= matches2.end(); ++matchIterator2) {

			if (matchIterator2->size() < 2) // ignore deleted matches 
				continue;

			// Match symmetry test
			if ((*matchIterator1)[0].queryIdx == (*matchIterator2)[0].trainIdx  && 
					(*matchIterator2)[0].queryIdx == (*matchIterator1)[0].trainIdx) {

				// add symmetrical match
				symMatches.push_back(cv::DMatch((*matchIterator1)[0].queryIdx,
							(*matchIterator1)[0].trainIdx,
							(*matchIterator1)[0].distance));
				break; // next match in image 1 -> image 2
			}
		}
	}
}

vector<cv::Point2f> convertKeypoints(vector<cv::KeyPoint> keypoints){
	std::vector<cv::Point2f> points;

	for(int i=0;i<keypoints.size();i++){
		points.push_back(keypoints[i].pt);
	}

	return points;
}

vector<cv::DMatch> getMatches(cv::Mat image1, cv::Mat image2,vector<cv::KeyPoint> keypoints1 ,vector<cv::KeyPoint> keypoints2 ){
	cv::Ptr<cv::DescriptorExtractor> extractor = new cv::SurfDescriptorExtractor();;
	cv::BFMatcher matcher(cv::NORM_L2,false);

	std::vector<cv::DMatch> matches;
	vector<vector<cv::DMatch> > matches1, matches2;

	//Match using nearest neighboor with 2


	// 1b. Extraction of the SURF descriptors
	cv::Mat descriptors1, descriptors2;
	extractor->compute(image1,keypoints1,descriptors1);
	extractor->compute(image2,keypoints2,descriptors2);


	matcher.knnMatch(descriptors1,descriptors2, matches1,2);
	matcher.knnMatch(descriptors2,descriptors1, matches2,2);

	ratioTest(matches1, 0.65f);
	ratioTest(matches2, 0.65f);	

	symmetryTest(matches1,matches2,matches);
	//matcher.match(descriptors1,descriptors2,matches);

	return matches;
}


/**
 * This function returns keypoints and matches between 2 images
 */

void getKeypointsAndMatches(Mat image1, Mat image2, vector<KeyPoint> &keypoints1, vector<KeyPoint> &keypoints2,vector<DMatch> &matches){
	if (!image1.data || !image2.data){
		cout << "getKeypointsAndMatches Error:" <<endl;
		cout<< "One of the input Images does not contains any data please verify" << endl;
		return ;
	}

	keypoints1 = get2DKeypoints(image1);
	keypoints2 = get2DKeypoints(image2);

	matches = getMatches(image1, image2, keypoints1, keypoints2);
	//	RobustMatcher rmatcher;
	//	rmatcher.setConfidenceLevel(0.98);
	//	rmatcher.setMinDistanceToEpipolar(1.0);
	//	rmatcher.setRatio(0.65f);
	//	cv::Ptr<cv::FeatureDetector> pfd= new cv::SurfFeatureDetector(10); 
	//	rmatcher.setFeatureDetector(pfd);

	//	Mat fundemental= rmatcher.match(image1,image2,matches, keypoints1, keypoints2);
}


vector<vector<cv::Point2f> > getMatchedPoints(vector<cv::KeyPoint> keypoints1, vector<cv::KeyPoint> keypoints2, vector<cv::DMatch> matches){

	vector<vector<cv::Point2f> > matched;

	std::vector<cv::Point2f> points1, points2;
	for (std::vector<cv::DMatch>::const_iterator it= matches.begin();
			it!= matches.end(); ++it) {

		// Get the position of left keypoints
		float x= keypoints1[it->queryIdx].pt.x;
		float y= keypoints1[it->queryIdx].pt.y;
		points1.push_back(cv::Point2f(x,y));
		// Get the position of right keypoints
		x= keypoints2[it->trainIdx].pt.x;
		y= keypoints2[it->trainIdx].pt.y;
		points2.push_back(cv::Point2f(x,y));
	}

	matched.push_back(points1);
	matched.push_back(points2);

	return matched;
}


vector<vector<cv::KeyPoint> > getMatchedKeypoints(vector<cv::KeyPoint> keypoints1, vector<cv::KeyPoint> keypoints2, vector<cv::DMatch> matches){

	vector<vector<cv::KeyPoint> > matched;

	std::vector<cv::KeyPoint> points1, points2;
	for (std::vector<cv::DMatch>::const_iterator it= matches.begin();
			it!= matches.end(); ++it) {

		// Get the position of left keypoints
		points1.push_back(keypoints1[it->queryIdx]);

		// Get the position of right keypoints
		points2.push_back(keypoints2[it->trainIdx]);
	}

	matched.push_back(points1);
	matched.push_back(points2);

	return matched;
}
/**
 * Draw epipolar lines between 2 images
 */
void drawEpipolarLines(Mat &image1, Mat &image2, Mat &imageMatches){
	//image1= cv::imread("lena.jpg",0);
	//image2= cv::imread("lena2.jpg",0);
	if (!image1.data || !image2.data){
		cout << "drawEpipolarLines Error:" <<endl;
		cout<< "One of the input Images does not contains any data please verify" << endl;
		return ;
	}
	// Prepare the matcher
	RobustMatcher rmatcher;
	rmatcher.setConfidenceLevel(0.98);
	rmatcher.setMinDistanceToEpipolar(1.0);
	rmatcher.setRatio(0.65f);
	//rmatcher.setRatio(1.0f);
	cv::Ptr<cv::FeatureDetector> pfd= new cv::SurfFeatureDetector(10); 
	rmatcher.setFeatureDetector(pfd);

	// Match the two images
	std::vector<cv::DMatch> matches;
	std::vector<cv::KeyPoint> keypoints1, keypoints2;
	cv::Mat fundemental= rmatcher.match(image1,image2,matches, keypoints1, keypoints2);
	cv::drawKeypoints( image1, keypoints1, image1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
	cv::drawKeypoints( image2, keypoints2, image2, Scalar::all(-1), DrawMatchesFlags::DEFAULT );

	// draw the matches
	//cv::Mat imageMatches;
	cv::drawMatches(image1,keypoints1,  // 1st image and its keypoints
			image2,keypoints2,  // 2nd image and its keypoints
			matches,                        // the matches
			imageMatches,           // the image produced
			cv::Scalar(0,255,0),DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS); // color of the lines

	// Convert keypoints into Point2f       
	std::vector<cv::Point2f> points1, points2;

	for (std::vector<cv::DMatch>::const_iterator it= matches.begin();
			it!= matches.end(); ++it) {
		// Get the position of left keypoints
		float x= keypoints1[it->queryIdx].pt.x;
		float y= keypoints1[it->queryIdx].pt.y;
		points1.push_back(cv::Point2f(x,y));
		//cv::circle(image1,cv::Point(x,y),3,cv::Scalar(255,255,255),3);
		// Get the position of right keypoints
		x= keypoints2[it->trainIdx].pt.x;
		y= keypoints2[it->trainIdx].pt.y;
		//cv::circle(image2,cv::Point(x,y),3,cv::Scalar(255,255,255),3);
		points2.push_back(cv::Point2f(x,y));
	}

	// Draw the epipolar lines
	std::vector<cv::Vec3f> lines1; 
	cv::computeCorrespondEpilines(cv::Mat(points1),.1,fundemental,lines1);

	for (vector<cv::Vec3f>::const_iterator it= lines1.begin();
			it!=lines1.end(); ++it) {
		cv::line(image2,cv::Point(0,-(*it)[2]/(*it)[1]),
				cv::Point(image2.cols,-((*it)[2]+(*it)[0]*image2.cols)/(*it)[1]),
				cv::Scalar::all(-1));
	}
	std::vector<cv::Vec3f> lines2; 
	cv::computeCorrespondEpilines(cv::Mat(points2),2,fundemental,lines2);

	for (vector<cv::Vec3f>::const_iterator it= lines2.begin();it!=lines2.end(); ++it){
		cv::line(image1,cv::Point(0,-(*it)[2]/(*it)[1]),
				cv::Point(image1.cols,-((*it)[2]+(*it)[0]*image1.cols)/(*it)[1]),
				cv::Scalar::all(-1));
	} 

	//        //Show Images
	//        cvNamedWindow("First Image",0);
	//	cvNamedWindow("Second Image",0);
	//	cvResizeWindow("First Image",image1.rows,image1.cols);
	//	cvResizeWindow("Second Image",image1.rows,image1.cols);
	//	cv::imshow("First Image", image1);
	//	cv::imshow("First Image", image2);
	//	 cv::waitKey(0);               
}

void drawEpipolarLines2(Mat &image1, Mat &image2, Mat &imageMatches){
	std::vector<cv::KeyPoint> keypoints1, keypoints2;

}

cv::Mat linearInterpolate(cv::Mat image1, cv::Mat image2, double d, double pos){
	cv::Mat interpol(max(image1.rows,image2.rows), max(image1.cols,image2.cols), image1.type());
	vector<cv::KeyPoint> keypoints1;
	vector<cv::KeyPoint> keypoints2;
	vector<cv::DMatch> matches;

	getKeypointsAndMatches(image1,image2,keypoints1,keypoints2,matches);
	//	
	std::vector<cv::Point2f> points1, points2;
	cv::Vec3b color1, color2; 
	//	
	for (std::vector<cv::DMatch>::const_iterator it= matches.begin();
			it!= matches.end(); ++it) {

		//		// Get the position of left keypoints
		float x1= keypoints1[it->queryIdx].pt.x;
		float y1= keypoints1[it->queryIdx].pt.y;
		if(x1<image1.cols & y1 < image1.rows){
			cout << "(x1,y1) in image1: " << x1 << "," << y1 << endl;
			color1 = image1.at<cv::Vec3b>(y1,x1);
		}
		//		points1.push_back(cv::Point2f(x,y));
		//		// Get the position of right keypoints
		float x2= keypoints2[it->trainIdx].pt.x;
		float y2= keypoints2[it->trainIdx].pt.y;
		if(x2<image2.cols & y2 < image2.rows){
			cout << "(x2,y2) in image2: " << x2 << "," << y2 << endl;
			color2 = image2.at<cv::Vec3b>(y2,x2);
		}
		//		points2.push_back(cv::Point2f(x,y));
		//		
		float x = (1/d) * (x1* (d-pos) + x2*pos);
		float y = (1/d) * (y1* (d-pos) + y2*pos);

		if(x<interpol.cols && y < interpol.rows){
			cout << "(i,j) in interpolate: " << round(x) << "," << round(y) << endl;
			interpol.at<cv::Vec3b>(y, x)[0] =  color1[0]; 
			interpol.at<cv::Vec3b>(y, x)[1] =  color1[1];
			interpol.at<cv::Vec3b>(y, x)[2] =  color1[2];
		}
	}

	return interpol;

}

void optFlowMap(cv::Mat image1, cv::Mat image2, cv::Mat &flow){
	cout << "Began optFlowMat" << endl;
	if( !image1.data || !image2.data ){
		cout << "Error in optFlowMap, one of the images is empty, please verify" << endl;
		return;	
	}

	else {
		cv::Mat tempImg1, tempImg2;

		tempImg1 = image1.clone();
		tempImg2 = image2.clone();
		cv::cvtColor(image1,tempImg1,COLOR_BGR2GRAY);
		cv::cvtColor(image2,tempImg2,COLOR_BGR2GRAY);

		cv::Mat cflow;	

		cv::calcOpticalFlowFarneback(tempImg1, tempImg2, flow, 0.5, 3, 15, 3, 5, 1.2, 0);
	}

	cout << "End optFlowMap" << endl;
	//	cv::cvtColor(flow,cflow,COLOR_GRAY2BGR);
	//	
	//	int step = 16;
	//	cv::Scalar color(0,255,0);
	//	
	//	for(int y = 0; y < cflow.rows; y += step){
	//		for(int x = 0; x < cflow.cols; x += step)
	//		{
	//		    const Point2f& fxy = flow.at<Point2f>(y, x);
	//		    cv::line(cflow, Point(x,y), Point(cvRound(x+fxy.x), cvRound(y+fxy.y)),color);
	//		    cv::circle(cflow, Point(x,y), 2, color, -1);
	//		}
	//	}
	//	cv::namedWindow("flow", 0);
	//	cv::imshow("flow",cflow);
	//	cv::WaitKey(0);
}


void lkOptFlowMap(cv::Mat image1, cv::Mat image2){
	vector<cv::KeyPoint> keypoints1, keypoints2;
	keypoints1 = get2DKeypoints(image1);
	keypoints2 = get2DKeypoints(image2);

	cv::Mat tempImg1, tempImg2;
	tempImg1 = image1.clone();
	tempImg2 = image2.clone();

	cv::cvtColor(image1,tempImg1,COLOR_BGR2GRAY);
	cv::cvtColor(image2,tempImg2,COLOR_BGR2GRAY);

	std::vector<cv::Point2f> points1, points2;

	for(int i=0;i<keypoints1.size();i++){
		points1.push_back(keypoints1[i].pt);
	}
}

cv::Subdiv2D getDelaunayTriangles(vector<cv::KeyPoint> keypoints, int rows, int cols){
	cv::Rect rect(0, 0, cols, rows);

	//CvMemStorage store;
	cv::Subdiv2D subdiv(rect);
	//subdiv = cvCreateSubdivDelaunay2D(rect, store);

	for(int i=0;i<keypoints.size();i++){
		//cvSubdivDelaunay2DInsert(subdiv,keypoints[i].pt );
		subdiv.insert(keypoints[i].pt);
	}

	return subdiv;
}

int findTriangleInVector(Vec6f triangle,vector<Vec6f> trianglesList ){
	if(find(trianglesList.begin(), trianglesList.end(), triangle)!=trianglesList.end()){
		return find(trianglesList.begin(), trianglesList.end(), triangle) - trianglesList.begin();
	}
	else{
		return -1;
	}

}

int findFacetInVector(vector<Point2f> facet,vector<vector<Point2f> > facetsList ){
	int num = -1; 
	Point2f a,b,c;
	
	
	if(facet.size()==0 || facetsList.size()==0){
		cout << "findFacetInVector: Input values are incorrect. Please verify" << endl;
		return -1;
	
	}
	a = facet[0];
	b = facet[1];
	c = facet[2];
	
	
	for(int i = 0; i<facetsList.size(); i++){
		vector<Point2f> f = facetsList[i];
		if( find(f.begin(), f.end(), a) != f.end()
		&& find(f.begin(), f.end(), b) != f.end()
		&& find(f.begin(), f.end(), c) != f.end()){
			return i;
		}
	}
	
	return -1;
//	if(find(facetsList.begin(), facetsList.end(), facet)!=facetsList.end()){
//		return find(facetsList.begin(), facetsList.end(), facet) - facetsList.begin();
//	}
//	else{
//		return -1;
//	}

}

int locateTriangleIndex(CvSubdiv2D subdiv ,cv::Point2f p ){

//	CvSubdiv2DEdge edge,leftEdge, rightEdge; 
//	CvSubdiv2DPoint &vertex;
//	
//	
//	vector<Vec6f> triangles;

//	
//	subdiv.getTriangleList(triangles);
//	
//	//Calculate the coordinates of the Voronoi diagram cells...each virtual point corresponds to 
//	cvCalcSubdivVoronoi2D(subdiv);
//	vector<vector<Point2f> > facets = subdiv.getVoronoiFacetList();
//	
//	Vec6f t;
//	//cvSubdiv2DPointLocation loc = cvSubdiv2DLocate(subdiv,p,edge,vertex);
//	
//	//If the point was found inside a facet
//	int eo=0, v = 0;
//	if(subdiv.locate(p,eo,v) = CV_PTLOC_INSIDE  ){
//		//leftEdge = cvSubdiv2DGetEdge(edge, CV_PREV_AROUND_RIGHT );
//		//rightEdge = cvSubdiv2DGetEdge(edge, CV_NEXT_AROUND_RIGHT );
//		
//	}
//	
//	else if(cvSubdiv2DLocate(&subdiv,p,&edge,&vertex) = CV_PTLOC_ON_EDGE ){}
//	
//	else if(cvSubdiv2DLocate(&subdiv,p,&edge,&vertex) = CV_PTLOC_VERTEX){}
//	
//	else{ return -1;}

}

vector<cv::Mat> getAffineTriangleTransforms (vector<Vec6f> triangles1, vector<Vec6f> triangles2){


	vector<cv::Mat> transforms;
	
	if(triangles1.size() != triangles2.size() || triangles1.size()==0){
		cout << "getAffineTriangleTransforms error: The input triangles list do not match or are empty." << endl;
		return transforms;
	}
	
	//An affine transform matrix 
	cv::Mat tWarp (2,3,CV_32FC1);
	
	cv::Point2f tri1[3];
	cv::Point2f tri2[3];
	
	
	//For all the triangles in the list
	for(int i = 0; i < triangles1.size(); i++){
		//Recover the triangles in first list
		Vec6f t = triangles1[i];
	
		//Recover the vertices of the triangle
		tri1[0] = cv::Point2f(t[0], t[1]); 
		tri1[1] = cv::Point2f(t[2], t[3]); 
		tri1[2] = cv::Point2f(t[4], t[5]); 
		
		
		
		//Recover the triangles in second list
		t = triangles2[i];
		tri2[0] = cv::Point2f(t[0], t[1]); 
		tri2[1] = cv::Point2f(t[2], t[3]); 
		tri2[2] = cv::Point2f(t[4], t[5]); 
		
		/// Get the Affine Transform
   		tWarp = getAffineTransform( tri1, tri2 );
		transforms.push_back(tWarp);
	}
	
	return transforms;
}


