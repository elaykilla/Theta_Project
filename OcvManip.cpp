/*
 * @author: ELay Maiga
 * This class contains all the methods related to calculating coordinates and making projections
 * from 2D to 3D or vice-versa.
 */


#include"OcvManip.hpp"

/**
* This function displays the mat values
*/
void showMat(Mat mat){
	cout << "[" ;
	for(int i=0;i<mat.rows;i++){
	
		for(int j=0;j<mat.cols;j++){
			cout << mat.at<float>(i,j);
			
			if(j!=mat.cols-1){ cout << "," ;}
		}
		
		cout << ";" << endl;
	}
	cout << "]" << endl;
}



void imageListToVideo(vector<cv::Mat> images , string fileName) {
	if(images.size() == 0){
		cout << "Please provide images" << endl;
		return;
	}
	
	
	
	else{
		//Get the first image as reference for frame size 
		Mat img = images[0]; 
	
		//Setup parameters for the video writer
		ostringstream name;
		name << fileName << ".mpeg";
	
		//Define framerate
		double fps = 20;
	
		//Define frame size
		Size fSize = img.size();
		
	
	
		int ex = CV_FOURCC('M','P','E','G');
		VideoWriter outputVideo;
		
		outputVideo.open(name.str(),ex,fps,fSize,true); 	
	
		if(!outputVideo.isOpened()){
			cout  << "Could not open the output video for write: " << name << endl;
        		return ;
		}
		
		else{
			for(int i=0;i<images.size();i++){
				img = images[i];
				outputVideo.write(img);
			}
		}
	}

} 
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

**/
Mat rotateImagey(Mat image, double y){
	Mat rotated = cv::Mat::zeros(image.rows,image.cols,image.type());
	int shift = cvRound(image.cols*y/360);
	
	for(int i=0;i<image.rows;i++){
		for(int j=0;j<image.cols;j++){
			int posj = (j+shift)%image.cols;
			rotated.at<Vec3b>(i,posj) = image.at<Vec3b>(i,j);
		}
	}
	
	return rotated;
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


vector<cv::KeyPoint> getSiftKeypoints(cv::Mat image){
	cv::SiftFeatureDetector detector(0,4,0.04,10,1.6);
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

vector<cv::DMatch> getFlannMatches(cv::Mat image1, cv::Mat image2,vector<cv::KeyPoint> keypoints1 ,vector<cv::KeyPoint> keypoints2){
	//SurfDescriptorExtractor extractor;
	SiftDescriptorExtractor extractor;
	
	cv::FlannBasedMatcher matcher;
	Mat descriptors1, descriptors2;

	//Extract Descriptors
	extractor.compute( image1, keypoints1, descriptors1 );
	extractor.compute( image2, keypoints2, descriptors2 );


	std::vector<cv::DMatch> matches;
	std::vector< DMatch > good_matches;


	matcher.match( descriptors1, descriptors2, matches );
	double max_dist = 0; double min_dist = 100;

	//-- Quick calculation of max and min distances between keypoints
	for( int i = 0; i < descriptors1.rows; i++ ){
		double dist = matches[i].distance;
		if( dist < min_dist ) min_dist = dist;
		if( dist > max_dist ) max_dist = dist;
	}

	//Keep only matches close to min distance
	for( int i = 0; i < descriptors1.rows; i++ ){ 
		if( matches[i].distance <= max(2*min_dist, 0.02) ){ 
			good_matches.push_back( matches[i]); 
		}
	}

	return good_matches;
}


vector<cv::DMatch> getMatches(cv::Mat image1, cv::Mat image2,vector<cv::KeyPoint> keypoints1 ,vector<cv::KeyPoint> keypoints2 ){
	//cv::Ptr<cv::DescriptorExtractor> extractor = new cv::SurfDescriptorExtractor();
	cv::Ptr<cv::DescriptorExtractor> extractor = new cv::SiftDescriptorExtractor();
	
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
	
	//keypoints1 = getSiftKeypoints(image1);
	//keypoints2 = getSiftKeypoints(image2);

	matches = getMatches(image1, image2, keypoints1, keypoints2);

	//matches = getFlannMatches(image1, image2, keypoints1, keypoints2);

	//To compute fundamental matrix at the same time!

	//	RobustMatcher rmatcher;
	//	rmatcher.setConfidenceLevel(0.98);
	//	rmatcher.setMinDistanceToEpipolar(1.0);
	//	rmatcher.setRatio(0.65f);
	//	cv::Ptr<cv::FeatureDetector> pfd= new cv::SurfFeatureDetector(10); 
	//	rmatcher.setFeatureDetector(pfd);

	//	Mat fundemental= rmatcher.match(image1,image2,matches, keypoints1, keypoints2);
}

void getSiftKeypointsAndMatches(Mat img1, Mat img2, vector<KeyPoint> &keypoints1, vector<KeyPoint> &keypoints2,vector<DMatch> &matches){
	if (!img1.data || !img2.data){
		cout << "getKeypointsAndMatches Error:" <<endl;
		cout<< "One of the input Images does not contains any data please verify" << endl;
		return ;
	}
	
	cv::Mat image1,image2;
	//Convert images to grayscale
	cv::cvtColor(img1,image1,COLOR_BGR2GRAY);
	cv::cvtColor(img2,image2,COLOR_BGR2GRAY);
	
	keypoints1 = getSiftKeypoints(image1);
	keypoints2 = getSiftKeypoints(image2);

	//matches = getMatches(image1, image2, keypoints1, keypoints2);

	matches = getFlannMatches(image1, image2, keypoints1, keypoints2);

	//To compute fundamental matrix at the same time!

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

////////////Line Detection
vector<cv::Vec4i> getLinesProb(cv::Mat image){

	vector<Vec4i> lines;
	if(image.empty())
	{
		cout << "getLines: Can not open Image, please verify" << endl;
		vector<Vec4i> lines;
	}

	//Detect image edges using canny detection
	Mat dst, cdst;
	cv::Canny(image, dst, 50, 200, 3);
	//cvtColor(dst, cdst, CV_GRAY2BGR);


	//Normal Hough Transform
	//	vecotr<Vec2f> lines;
	//	HoughLines(dst, lines, 1, CV_PI/180, 100, 0, 0 );

	//Probabilistic Hough Transform

	HoughLinesP(dst, lines, 1, CV_PI/180, 20, 50, 10 );

	return lines;
}


//Edge Detection
cv::Mat detectEdges(cv::Mat img){

	int lowThreshold = 50;
	int ratio = 3;
	int kernel_size = 3;
	
	Mat img_gray, dst, detected_edges;
	
	 /// Convert the image to grayscale
  	cv::cvtColor( img, img_gray, CV_BGR2GRAY );
	
	//Reduce noise with gaussian kernel 3x3
	cv::blur(img_gray, detected_edges, cv::Size(3,3));
	
	cv::Canny(detected_edges, detected_edges,lowThreshold, lowThreshold*ratio,kernel_size );
	
	img.copyTo(dst,detected_edges);
	
	return detected_edges;	
	
}





//////////Interpolation and Triangulation
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
	//	if(find(trianglesList.begin(), trianglesList.end(), triangle)!=trianglesList.end()){
	//		return find(trianglesList.begin(), trianglesList.end(), triangle) - trianglesList.begin();
	//	}
	//	else{
	//		return -1;
	//	}
	//cout<< "Finding triangle in Vector" << endl;
	int index = -1;
	int i =0;
	bool found = false;
	while(i<trianglesList.size() & !found){
		Vec6f t = trianglesList[i];
		found = sameTriangle(triangle,t);

		if(found){
			index = i;
		}
		i++;
	}

	return index;
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

void getCorrespondingDelaunayTriangles(vector<cv::KeyPoint> keypoints1, vector<cv::KeyPoint> keypoints2, vector<cv::Vec6f> &trianglesList1, vector<cv::Vec6f> &trianglesList2){

	cout << "Began getCorrespondingDelaunayTriangles" << endl;
	vector<Vec6f> newtrianglesList1, newtrianglesList2;

	//Vector containing 2 matching points
	Vec6f t1,t2; 
	Point2f p1,p2,p3,pp1,pp2,pp3;	

	vector<Point2f> points1, points2;

	for(int i=0;i<keypoints1.size();i++){
		points1.push_back(keypoints1[i].pt);
		points2.push_back(keypoints2[i].pt);
	}


	for(int i=0;i<trianglesList1.size();i++){
		t1 = trianglesList1[i];

		//For each point in triangle 1 we search for the corresponding point in triangle 2
		p1.x = t1[0];
		p1.y = t1[1];
		int ptPos1 = find(points1.begin(),points1.end(),p1) - points1.begin(); 
		//cout << "t1 point 1: " << p1 << " at position: " << ptPos1 << endl;

		p2.x = t1[2];
		p2.y = t1[3];
		int ptPos2 = find(points1.begin(),points1.end(),p2) - points1.begin(); 
		//cout << "t1 point 2: " << p2 << " at position: " << ptPos2 << endl;

		p3.x = t1[4];
		p3.y = t1[5];
		int ptPos3 = find(points1.begin(),points1.end(),p3) - points1.begin(); 
		//cout << "t1 point 3: " << p3 << " at position: " << ptPos3 << endl;

		if(ptPos1 < points1.size() && ptPos2 < points1.size() && ptPos3 < points1.size()){

			pp1 = points2[ptPos1];
			pp2 = points2[ptPos2];
			pp3 = points2[ptPos3];

			//Construct the second triangle and try to locate it in the trianglesList2
			t2[0] = pp1.x;
			t2[1] = pp1.y;

			t2[2] = pp2.x;
			t2[3] = pp2.y;

			t2[4] = pp3.x;
			t2[5] = pp3.y;

			//Locate the given triangle in the second list and place it in the same position as it's corresponding 				// triangle  
			int j = findTriangleInVector(t2,trianglesList2);
			//cout << "triangle found at position: " << j << endl;
			if(j!=-1){
				newtrianglesList1.push_back(trianglesList1[i]); 
				newtrianglesList2.push_back(trianglesList2[j]);
			}
			else{
				//cout << "---------------- All points located -------------" << endl;
				//				cout << "======================== Triangle not Found ===============================" << endl;
				//				cout << "Original Triangle: " << p1 << "," << p2 << "," << p3 << endl;
				//				cout << "Corresponding: " << pp1 << "," << pp2 << "," << pp3 << endl;
			}
		}



	}

	//Now the triangles in trianglesList at position i correspond to the triangles in trianglesList1 at the same postion
	trianglesList1 = newtrianglesList1;
	trianglesList2 = newtrianglesList2;
	cout << "End of getCorrespondingDelaunayTriangles" << endl;
	//return trianglesList;
}

void makeCorrespondingDelaunayTriangles(vector<cv::Point2f> points1, vector<cv::Point2f> points2, vector<cv::Vec6f> &trianglesList1, vector<cv::Vec6f> &trianglesList2){
	cout <<"makeCorrespondingDelaunayTriangles Points in Points1" << points1.size() << endl;
	vector<Vec6f> newtrianglesList1,newtrianglesList2;
	Vec6f t,t2;
	//logFile.open("TrianglePoints.txt");
	Point2f p1,p2,p3,pp1,pp2,pp3;
	for(int i=0;i<trianglesList1.size();i++){
		t = trianglesList1[i];
		p1 = Point2f(t[0], t[1]);
		p2 = Point2f(t[2], t[3]);
		p3 = Point2f(t[4], t[5]);

		//logFile << p1 << "||" << p2 << "||" << p3 << endl;
		//For each point in triangle 1 we search for the corresponding point in triangle 2
		//int ptPos1 = find(points1.begin(),points1.end(),p1) - points1.begin(); 

		int ptPos1 = points1.size();
		for(int j=0;j<points1.size();j++){
			//cout << "Points in point1: " << points1[j] << endl;
			if((points1[j].x == p1.x && points1[j].y == p1.y) ||(points1[j].x == p1.y && points1[j].y == p1.x)){
				//cout << "Points in point1: " << points1[j] << endl;
				ptPos1 = j;
				break;
			}
		}	
		int ptPos2 = find(points1.begin(),points1.end(),p2) - points1.begin(); 		
		int ptPos3 = find(points1.begin(),points1.end(),p3) - points1.begin(); 



		if(ptPos1 < points1.size() && ptPos2 < points1.size() && ptPos3 < points1.size()){	
			//Construct the second triangle and try to locate it in the trianglesList2
			//We find the corresponding point in the second triangles list
			pp1 = points2[ptPos1];
			pp2 = points2[ptPos2];
			pp3 = points2[ptPos3];
			t2[0] = pp1.x;
			t2[1] = pp1.y;

			t2[2] = pp2.x;
			t2[3] = pp2.y;

			t2[4] = pp3.x;
			t2[5] = pp3.y;
			newtrianglesList1.push_back(t);
			newtrianglesList2.push_back(t2);
		}else{
			if(!ptPos1 < points1.size()){
				//cout<< "Point not found in points: " << pp1 << endl;
				pp1 = p1;
			}
			else{
				pp1 = points2[ptPos1];
			}

			if(!ptPos2 < points1.size()){
				pp2 = p2;
			}
			else{
				pp2 = points2[ptPos2];
			}


			if(!ptPos2 < points1.size()){
				pp3 = p3;
			}
			else{
				pp3 = points2[ptPos3];
			}

			t2[0] = pp1.x;
			t2[1] = pp1.y;

			t2[2] = pp2.x;
			t2[3] = pp2.y;

			t2[4] = pp3.x;
			t2[5] = pp3.y;
			newtrianglesList1.push_back(t);
			newtrianglesList2.push_back(t2);
		}
	}

	//logFile.close();
	//trianglesList1 = newtrianglesList1;
	trianglesList2 = newtrianglesList2;

}


int locateTriangleIndex(Subdiv2D subdiv , vector<Vec6f> triangles, cv::Point2f p ){

	//cout<< "Locating triangle for point: " << p << endl;
	CvSubdiv2DEdge edge,leftEdge, rightEdge; 
	CvSubdiv2DPoint vertex;	

	Vec6f t;
	//vector<Vec6f> triangles;

	//	
	//subdiv.getTriangleList(triangles);
	//	
	//	//Calculate the coordinates of the Voronoi diagram cells...each virtual point corresponds to 
	//	cvCalcSubdivVoronoi2D(subdiv);
	//	vector<vector<Point2f> > facets = subdiv.getVoronoiFacetList();
	//	

	//	//cvSubdiv2DPointLocation loc = cvSubdiv2DLocate(subdiv,p,edge,vertex);
	//	
	//	//If the point was found inside a facet
	int eo=0, v = 0;
	//Subdiv locate gives back edge number and vertex number in int!! 
	if(subdiv.locate(p,eo,v) == CV_PTLOC_INSIDE  ){
		//cout << "Point located near edge: " << eo << endl;
		Point2f p1,p2,p3,p4,p5,p6;
		//Recover the origin and destination of each edge knowing that each triangle has 3 edges
		edge = eo;
		subdiv.edgeOrg(edge, &p1);
		subdiv.edgeDst(edge, &p2);

		leftEdge = subdiv.getEdge(eo, CV_PREV_AROUND_RIGHT );
		subdiv.edgeOrg(leftEdge, &p3);
		if(p3==p1 || p3==p2){
			subdiv.edgeDst(leftEdge, &p4);
			p3 = p4;
		}



		//rightEdge = cvSubdiv2DGetEdge(eo, CV_NEXT_AROUND_RIGHT );
		//subdiv.edgeOrg(rightEdge, &p5);
		//subdiv.edgeOrg(rightEdge, &p6);


		//Make triangle from 3 points
		t[0] = p1.x;
		t[1] = p1.y;
		t[2] = p2.x;
		t[3] = p2.y;
		t[4] = p3.x;
		t[5] = p3.y;

		return findTriangleInVector(t,triangles);
		//		
	}
	//	
	//	else if(cvSubdiv2DLocate(&subdiv,p,&edge,&vertex) = CV_PTLOC_ON_EDGE ){}
	//	
	//	else if(cvSubdiv2DLocate(&subdiv,p,&edge,&vertex) = CV_PTLOC_VERTEX){}
	//	
	else{ return -1;}

}


//Get 3D triangle from 2D triangle list
void get3DSphereTriangles(int rows, int cols, double r, vector<Vec6f> triangles2D,vector<Vec9f> triangles3D){
	Vec6f t2D;
	Vec9f t3D;
	int i,j;
	double x,y,z;
	
	//For every triangle
	for(int k=0;k<triangles2D.size();k++){
		t2D = triangles2D[k];
		int p = 0;
		
		//For each point in the triangle
		for(int l=0;l<6;l+=2){
			//Get (i,j) coordinates of the point
			i = cvRound(t2D[l]);
			j = cvRound(t2D[l+1]);
			
			//Calculate position on Sphere
			sphereCoordinates(i,j, r, rows,cols, x, y, z);
			
			t3D[l+p] = x;
			t3D[l+p+1] = y;
			t3D[l+p+2] = z;
			p++;
		}
		triangles3D.push_back(t3D);
	}
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


cv::Mat delaunayInterpolate(cv::Mat img1, cv::Mat img2, double dist, double pos){
	
	cv::Mat image1;
	//Extend second Image beyond border for border triangles
	cv::Mat image2(img2.rows, img2.cols + img2.cols/2,img2.type());

	//Resulting image
	cv::Mat result = cv::Mat::zeros(img1.rows,img1.cols + img1.cols/2,img1.type());
	cv::Mat resultNormalSize = cv::Mat::zeros(img1.rows,img1.cols,img1.type());
	//Create Subdivision of Image1
	cv::Subdiv2D subdiv1;
	image1 = img1.clone();
	img2.copyTo(image2(cv::Rect(cv::Point(0, 0), img2.size())));
	//Fill the right side with values of left side to complete second image
	int io = img2.rows;
	int jo = img2.cols;

	


	//Vectors for keypoints
	vector<cv::KeyPoint> keypoints1, keypoints2 ; 
	vector<cv::Point2f> points1, points2;



	//Vector for matches
	vector<cv::DMatch> matches;




	//Retrive keypoints from each image, and match them
	getKeypointsAndMatches(image1, image2, keypoints1, keypoints2,matches);
	//Using Sift
	//getSiftKeypointsAndMatches(image1, image2, keypoints1, keypoints2,matches);
	
	
	//For every point in Keypoints1 -- it's matched keypoint is in Keypoints2 at the same position
	vector<vector<cv::KeyPoint> > matched = getMatchedKeypoints(keypoints1, keypoints2, matches);
	vector<vector<cv::Point2f> > matchedPts = getMatchedPoints(keypoints1, keypoints2, matches);

	//Matched keypoints only
	keypoints1 = matched[0];
	keypoints2 = matched[1];

	//Extracted points from the Keypoints
	points1 = matchedPts[0];
	points2 = matchedPts[1];

	//Create Delaunay triangulation of the first image
	subdiv1 = getDelaunayTriangles(matched[0], image1.rows, image1.cols);

	//Extend 2nd image by half of image size
	for(int i=0;i<io;i++){
		for(int j=jo;j<jo+jo/2;j++){
			//cout << "i,j" << i << "," << j << endl;
			image2.at<Vec3b>(i,j)[0] = img2.at<Vec3b>(i, j-jo)[0];
			image2.at<Vec3b>(i,j)[1] = img2.at<Vec3b>(i, j-jo)[1];
			image2.at<Vec3b>(i,j)[2] = img2.at<Vec3b>(i, j-jo)[2];
		}
	}
	
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
	
	//Vectors for the triangles
	vector<Vec6f> triangles1, triangles2;

	//Retrive the triangles from Image1
	subdiv1.getTriangleList(triangles1);

	//Make matched triangles in image 2
	makeCorrespondingDelaunayTriangles(points1, points2, triangles1,triangles2);

	//Vector for the transformations between all triangles
	vector<cv::Mat> transforms;

	//Get the affine transform between each of the triangles
	transforms = getAffineTriangleTransforms(triangles1, triangles2);


	cv::Mat tWarp(2,3,CV_32FC1) ;
	cv::Point2f p;
	
	
	cout << "Beginning Image point calculations" << endl;
	//Go through all the Triangles in the second image
	for(int k=0;k<triangles1.size();k++){
	//for(int k=10;k<11;k++){
		tWarp = transforms[k];
		//cout << "transform matrix: " << tWarp << endl;
		double* uprow = tWarp.ptr<double>(0);
		double* downrow = tWarp.ptr<double>(1);
		
		//Triangle from Image1					
		Vec6f triangle = triangles1[k];
		
		//Triangle from image2
		Vec6f triangle2 = triangles2[k];
		
		cv::Point2f a,b,c,a1,b1,c1; 
		a.x = triangle[0];
		a.y = triangle[1];
		b.x = triangle[2];
		b.y = triangle[3];
		c.x = triangle[4];
		c.y = triangle[5];
		
		
		//Skip big triangels outside image1
		if(a.x < 0 || a.x > img1.cols || a.y<0 || a.y > img1.rows || b.x < 0 || b.x > img1.cols || b.y<0 || b.y > img1.rows || c.x < 0 || c.x > img1.cols || c.y<0 || c.y > img1.rows){
			//continue;
		}
		
		//////For printing purposes
		a1.x = triangle2[0];
		a1.y = triangle2[1];
		b1.x = triangle2[2];
		b1.y = triangle2[3];
		c1.x = triangle2[4];
		c1.y = triangle2[5];
	
		//Triangle from interpolated Image
		cv::Point2f ainter,binter,cinter;
		ainter.x = (a1.x*(dist-pos) + a.x*pos)/dist; 
		ainter.y = (a1.y*(dist-pos) + a.y*pos)/dist; 
		
		binter.x = (b1.x*(dist-pos) + b.x*pos)/dist; 
		binter.y = (b1.y*(dist-pos) + b.y*pos)/dist;
			
		cinter.x = (c1.x*(dist-pos) + c.x*pos)/dist; 
		cinter.y = (c1.y*(dist-pos) + c.y*pos)/dist;
		
		cv::Scalar delaunay_color(255, 255, 255);
//		cv::line(result, ainter, binter, delaunay_color, 1, CV_AA, 0);
//		cv::line(result, binter, cinter, delaunay_color, 1, CV_AA, 0);
//		cv::line(result, cinter, ainter, delaunay_color, 1, CV_AA, 0);
		
		
		//Picture 1 Triangles
		delaunay_color = cv::Scalar(255, 0, 0);
//		cv::line(result, a, b, delaunay_color, 1, CV_AA, 0);
//		cv::line(result, b, c, delaunay_color, 1, CV_AA, 0);
//		cv::line(result, c, a, delaunay_color, 1, CV_AA, 0);
		
		//Draw on image !
		cv::line(image1, a, b, delaunay_color, 1, CV_AA, 0);
		cv::line(image1, b, c, delaunay_color, 1, CV_AA, 0);
		cv::line(image1, c, a, delaunay_color, 1, CV_AA, 0);
		
		//Pic 2 triangles1
		delaunay_color = cv::Scalar(0, 0, 255);
//		cv::line(result, a1, b1, delaunay_color, 1, CV_AA, 0);
//		cv::line(result, b1, c1, delaunay_color, 1, CV_AA, 0);
//		cv::line(result, c1, a1, delaunay_color, 1, CV_AA, 0);
		
		
		//Draw on image 2
		cv::line(image2, a1, b1, delaunay_color, 1, CV_AA, 0);
		cv::line(image2, b1, c1, delaunay_color, 1, CV_AA, 0);
		cv::line(image2, c1, a1, delaunay_color, 1, CV_AA, 0);
		
		
		Vec6f triangle_inter;
		triangle_inter[0] = ainter.x;
		triangle_inter[1] = ainter.y;
		triangle_inter[2] = binter.x;
		triangle_inter[3] = binter.y;
		triangle_inter[4] = cinter.x;
		triangle_inter[5] = cinter.y;
		
		//Boolean to know which Image to use 
		bool use_first, use_second;
		
		use_first = false;
		use_second = false;
		
		
		if(triangleDifference(triangle2,triangle_inter)==triangleDifference(triangle_inter,triangle)){
			use_first = false;
			use_second = false;
		}
		else if(triangleDifference(triangle,triangle_inter) > triangleDifference(triangle_inter,triangle2)){
			//cout << "Triangle" << k << endl;
			//cout << "Image1 difference: " << triangleDifference(triangle,triangle_inter) << "     " << "Image2 difference:" << triangleDifference(triangle2,triangle_inter) << endl;
			use_first = false;
			use_second = true;
		}
		else if (triangleDifference(triangle,triangle_inter) < triangleDifference(triangle_inter,triangle2) ){
			use_second =  false;
			use_first = true;
		}
		
		//////////////////////////Printing triangle values: //////////////////////////////////
		//cout << "Triangle 2: " << "(" << a.x << "," << a.y << ")" << "|| " << "("<< b.x << "," << b.y << ")" << "||"  << "(" << c.x << "," << c.y << ")"<< endl;
		
		//cout << "Triangle 1: " << "(" << a1.x << "," << a1.y << ")" << "|| " << "("<< b1.x << "," << b1.y << ")" << "||"  << "(" << c1.x << "," << c1.y << ")"<< endl;
		
//		cv::Point2f ainter,binter,cinter;
//		ainter.x = cvRound(uprow[0] * a1.x + uprow[1]* a1.y + uprow[2]);
//		ainter.y = cvRound(downrow[0] * a1.x + downrow[1]* a1.y + downrow[2]);
//		
//		binter.x = cvRound(uprow[0] * b1.x + uprow[1]* b1.y + uprow[2]);
//		binter.y = cvRound(downrow[0] * b1.x + downrow[1]* b1.y + downrow[2]);
//			
//		cinter.x = cvRound(uprow[0] * c1.x + uprow[1]* c1.y + uprow[2]);
//		cinter.y = cvRound(downrow[0] * c1.x + downrow[1]* c1.y + downrow[2]);
		
		
		//cout << "Triangle Interpolated: " << "(" << ainter.x << "," << ainter.y << ")" << "|| " << "("<< binter.x << "," << binter.y << ")" << "||"  << "(" << cinter.x << "," << cinter.y << ")"<< endl;
		//////////////////////////////////////////////////////////////////////////////////////
		
		
		
		//Get the bouding box around the triangle
		int xmax = cvRound(max(a.x, max(b.x,c.x)));
		int ymax = cvRound(max(a.y, max(b.y,c.y)));
		int xmin = cvRound(min(a.x, min(b.x,c.x)));
		int ymin = cvRound(min(a.y, min(b.y,c.y)));


		cv::Point2f p,pinter,p2;

		for(int i=ymin;i<=ymax;i++){
			for(int j=xmin;j<=xmax;j++){
				p.x = j;
				p.y = i;


				if(i>0 && j>0 && j<img2.cols && i<img2.rows){
					//float x = uprow[0] * p.y + uprow[1]* p.x + uprow[2];
					//float y = downrow[0] * p.y + downrow[1]* p.x + downrow[2];


					if(inTriangleArea(p,triangle)){
						//Get the position of the point in the first image
						float x = uprow[0] * p.x + uprow[1]* p.y + uprow[2];
						float y = downrow[0] * p.x + downrow[1]* p.y + downrow[2];
						
						//calculate the resulting position in intermediate image
						float xinter = (x*(dist-pos) + p.x*pos)/dist; 
						float yinter = (y*(dist-pos) + p.y*pos)/dist; 
						
						//
						xinter = cvRound(xinter);
						yinter = cvRound(yinter);
						pinter.x = xinter;
						pinter.y = yinter;
						p2.x = cvRound(x);
						p2.y = cvRound(y);
//						if(cvRound(x)>0 & cvRound(y)>0 && cvRound(y)<result.cols && cvRound(x)<result.rows){
						
							//result.at<Vec3b>(p)[0] = img1.at<Vec3b>(p)[0];
							//result.at<Vec3b>(p)[1] = img1.at<Vec3b>(p)[1];
							//result.at<Vec3b>(p)[2] = img1.at<Vec3b>(p)[2];
//						if(inTriangleArea(pinter,triangle_inter)){
							//Get interpolated pixel values
							uchar b,g,r = 0;

							if(!use_first & !use_second){
								if(pos<dist/2){
									use_second = true;
								}
								else{
									use_first = true;
								}
							}
							if(use_first){
								//b = img1.at<Vec3b>(p)[0];
								b = 255;
								g = img1.at<Vec3b>(p)[1];
								r = img1.at<Vec3b>(p)[2];
//								
//								
//								result.at<Vec3b>(p2)[0] = b;
//								result.at<Vec3b>(p2)[1] = g;
//								result.at<Vec3b>(p2)[2] = r;
							}
							
							else if(use_second){
								b = img2.at<Vec3b>(p2)[0];
								g = img2.at<Vec3b>(p2)[1];
								//r = img2.at<Vec3b>(p2)[2];
								r = 255;
							
//								result.at<Vec3b>(yinter,xinter)[0] = b;
//								result.at<Vec3b>(yinter,xinter)[1] = g;
//								result.at<Vec3b>(yinter,xinter)[2] = r;
							}
//							
							else{
////								b = 255;
////								g = 255;
////								r = 255;
//								
								//b = (img2.at<Vec3b>(p2)[0]*pos + img1.at<Vec3b>(p)[0]*(dist-pos))/dist;
								//g = (img2.at<Vec3b>(p2)[1]*pos + img1.at<Vec3b>(p)[1]*(dist-pos))/dist;
								//r = (img2.at<Vec3b>(p2)[2]*pos + img1.at<Vec3b>(p)[2]*(dist-pos))/dist;
//								result.at<Vec3b>(yinter,xinter)[0] = b;
//								result.at<Vec3b>(yinter,xinter)[1] = g;
//								result.at<Vec3b>(yinter,xinter)[2] = r;
//							
							}
							
							result.at<Vec3b>(pinter)[0] = b;
							result.at<Vec3b>(pinter)[1] = g;
							result.at<Vec3b>(pinter)[2] = r;
//						}	
					}
				}
			}
		}
		
	}
	
	//cv::namedWindow("first image", 0);
	//cv::imshow("first image",image1);
	
	//cv::namedWindow("Second Image", 0);
	//cv::imshow("Second Image",image2);
	return result;
} 

vector<cv::Mat> delaunayInterpolateMultiple(cv::Mat img1, cv::Mat img2, double dist, int n){

	cout << "Called DelaunayInterpolateMultiple with: " << n << " images to interpoate" << endl;
	vector<cv::Mat> interpolated;

	
	cv::Mat image1;
	//Extend second Image beyond border for border triangles
	cv::Mat image2(img2.rows, img2.cols + img2.cols/2,img2.type());

	//Resulting image
	cv::Mat result = cv::Mat::zeros(img1.rows,img1.cols + img1.cols/2,img1.type());

//	for(int r=0;r<n;r++){
//		interpolated.push_back(result);
//	}
	
	cout << "Interpolated size: " << interpolated.size() << endl;
	//Create Subdivision of Image1
	cv::Subdiv2D subdiv1;
	image1 = img1.clone();
	img2.copyTo(image2(cv::Rect(cv::Point(0, 0), img2.size())));
	//Fill the right side with values of left side to complete second image
	int io = img2.rows;
	int jo = img2.cols;

	


	//Vectors for keypoints
	vector<cv::KeyPoint> keypoints1, keypoints2 ; 
	vector<cv::Point2f> points1, points2;



	//Vector for matches
	vector<cv::DMatch> matches;




	//Retrive keypoints from each image, and match them
	getKeypointsAndMatches(image1, image2, keypoints1, keypoints2,matches);

	
	
	
	//For every point in Keypoints1 -- it's matched keypoint is in Keypoints2 at the same position
	vector<vector<cv::KeyPoint> > matched = getMatchedKeypoints(keypoints1, keypoints2, matches);
	vector<vector<cv::Point2f> > matchedPts = getMatchedPoints(keypoints1, keypoints2, matches);

	//Matched keypoints only
	keypoints1 = matched[0];
	keypoints2 = matched[1];

	//Extracted points from the Keypoints
	points1 = matchedPts[0];
	points2 = matchedPts[1];

	//Create Delaunay triangulation of the first image
	subdiv1 = getDelaunayTriangles(matched[0], image1.rows, image1.cols);

	//Extend 2nd image by half of image size
	for(int i=0;i<io;i++){
		for(int j=jo;j<jo+jo/2;j++){
			//cout << "i,j" << i << "," << j << endl;
			image2.at<Vec3b>(i,j)[0] = img2.at<Vec3b>(i, j-jo)[0];
			image2.at<Vec3b>(i,j)[1] = img2.at<Vec3b>(i, j-jo)[1];
			image2.at<Vec3b>(i,j)[2] = img2.at<Vec3b>(i, j-jo)[2];
		}
	}
	
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
	
	//Vectors for the triangles
	vector<Vec6f> triangles1, triangles2;

	//Retrive the triangles from Image1
	subdiv1.getTriangleList(triangles1);

	//Make matched triangles in image 2
	makeCorrespondingDelaunayTriangles(points1, points2, triangles1,triangles2);

	//Vector for the transformations between all triangles
	vector<cv::Mat> transforms;

	//Get the affine transform between each of the triangles
	transforms = getAffineTriangleTransforms(triangles2, triangles1);

	cv::Mat tWarp(2,3,CV_32FC1) ;
	cv::Point2f p;
	
	
	cout << "Beginning Image point calculations" << endl;
	//Go through all the Triangles in the second image
	
	for(int l=0;l<n;l++){

		double pos = (dist/n)*l;
		result = cv::Mat::ones(img1.rows,img1.cols + img1.cols/2,img1.type());

		for(int k=0;k<triangles1.size();k++){
			tWarp = transforms[k];
			double* uprow = tWarp.ptr<double>(0);
			double* downrow = tWarp.ptr<double>(1);
			//					
			Vec6f triangle = triangles1[k];
			Vec6f triangle2 = triangles2[k];
			
			cv::Point2f a,b,c; 
			a.x = triangle[0];
			a.y = triangle[1];
			b.x = triangle[2];
			b.y = triangle[3];
			c.x = triangle[4];
			c.y = triangle[5];


			
			//****************************************************//
			//Drawing the triangles on the interpolated images
			//if(draw_triangle){
//			cv::Scalar delaunay_color(rand() % 255 + 1, rand() % 255 + 1, rand() % 255 + 1);
//			cv::Point2f ainter,binter,cinter;
//			ainter.x = cvRound(uprow[0] * a.x + uprow[1]* a.y + uprow[2]);
//			ainter.y = cvRound(downrow[0] * a.x + downrow[1]* a.y + downrow[2]);
//			binter.x = cvRound(uprow[0] * b.x + uprow[1]* b.y + uprow[2]);
//			binter.y = cvRound(downrow[0] * b.x + downrow[1]* b.y + downrow[2]);
//			
//			cinter.x = cvRound(uprow[0] * c.x + uprow[1]* c.y + uprow[2]);
//			cinter.y = cvRound(downrow[0] * c.x + downrow[1]* c.y + downrow[2]);
//						
//			if((ainter.y>0 && ainter.x>0 && ainter.x<img2.cols && ainter.y<img2.rows) && (binter.y>0 && binter.x>0 && binter.x<img2.cols && binter.y<img2.rows) && (cinter.y>0 && cinter.x>0 && cinter.x<img2.cols && cinter.y<img2.rows) ){	
//			cv::line(interpolated[l], ainter, binter, delaunay_color, 1, CV_AA, 0);
//			cv::line(interpolated[l], binter, cinter, delaunay_color, 1, CV_AA, 0);
//			cv::line(interpolated[l], cinter, ainter, delaunay_color, 1, CV_AA, 0);
//			}
			
			//}
			//******************************************************//
			
			
			//Get the bouding box around the triangle
			int xmax = cvRound(max(a.x, max(b.x,c.x)));
			int ymax = cvRound(max(a.y, max(b.y,c.y)));
			int xmin = cvRound(min(a.x, min(b.x,c.x)));
			int ymin = cvRound(min(a.y, min(b.y,c.y)));


			cv::Point2f p,p2;

			for(int i=ymin;i<=ymax;i++){
				for(int j=xmin;j<=xmax;j++){
					p.x = j;
					p.y = i;


					if(i>0 && j>0 && j<img2.cols && i<img2.rows){
						//float x = uprow[0] * p.x + uprow[1]* p.y + uprow[2];
						//float y = downrow[0] * p.x + downrow[1]* p.y + downrow[2];


						if(inTriangleArea(p,triangle)){
						
							//Get the position of the point in the first image
							float x = uprow[0] * p.x + uprow[1]* p.y + uprow[2];
							float y = downrow[0] * p.x + downrow[1]* p.y + downrow[2];
						
							//calculate the resulting position in each intermediate image
							//for(int l=0;l<n;l++){
								//Get image at position
								//result = interpolated[l];
								//double pos = (dist/n)*l;
								float xinter = (x*(dist-pos) + p.x*pos)/dist; 
								float yinter = (y*(dist-pos) + p.y*pos)/dist; 
						
								//
							
								xinter = cvRound(xinter);
								yinter = cvRound(yinter);
								
								p2.x = xinter;
								p2.y = yinter;
								if(inTriangleArea(p2,triangle2)){
									//Get interpolated pixel values
									uchar b,g,r;
//									b = (img2.at<Vec3b>(i,j)[0]*pos + img1.at<Vec3b>(y,x)[0]*(dist-pos))/dist;
//							g = (img2.at<Vec3b>(i,j)[1]*pos + img1.at<Vec3b>(y,x)[1]*(dist-pos))/dist;
//							r = (img2.at<Vec3b>(i,j)[2]*pos + img1.at<Vec3b>(y,x)[2]*(dist-pos))/dist;
//							if(pos<dist/3){
								b = img1.at<Vec3b>(y,x)[0];
								g = img1.at<Vec3b>(y,x)[1];
								r = img1.at<Vec3b>(y,x)[2];
//							}
							
//							else if(pos>2*dist/3){
//								b = img2.at<Vec3b>(i,j)[0];
//								g = img2.at<Vec3b>(i,j)[1];
//								r = img2.at<Vec3b>(i,j)[2];
//							}
//							else{
//								b = (img2.at<Vec3b>(i,j)[0]*pos + img1.at<Vec3b>(y,x)[0]*(dist-pos))/dist;
//								g = (img2.at<Vec3b>(i,j)[1]*pos + img1.at<Vec3b>(y,x)[1]*(dist-pos))/dist;
//								r = (img2.at<Vec3b>(i,j)[2]*pos + img1.at<Vec3b>(y,x)[2]*(dist-pos))/dist;
//							
//							}

									result.at<Vec3b>(yinter,xinter)[0] = b;
									result.at<Vec3b>(yinter,xinter)[1] = g;
									result.at<Vec3b>(yinter,xinter)[2] = r;
									
								}	
							//}
						}
					}
				}
			}
		
		}
	
		interpolated.push_back(result);
		result.release();
	}
	
	
	
		
	//****************************************************//
	//Drawing the triangles on the interpolated images
//	for(int l=0;l<n;l++){
//	
//		double pos = (dist/n)*l;
//		cv::Scalar delaunay_color(rand() % 255 + 1, rand() % 255 + 1, rand() % 255 + 1);
//		for(int k=0;k<triangles2.size();k++){
//			tWarp = transforms[k];
//			double* uprow = tWarp.ptr<double>(0);
//			double* downrow = tWarp.ptr<double>(1);
//			//					
//			Vec6f triangle = triangles1[k];

//			cv::Point2f a,b,c,a1,b1,c1; 
//			a.x = triangle[0];
//			a.y = triangle[1];
//			b.x = triangle[2];
//			b.y = triangle[3];
//			c.x = triangle[4];
//			c.y = triangle[5];


//		
//			
//			cv::Point2f ainter,binter,cinter;
//			a1.x = cvRound(uprow[0] * a.x + uprow[1]* a.y + uprow[2]);
//			a1.y = cvRound(downrow[0] * a.x + downrow[1]* a.y + downrow[2]);
//			b1.x = cvRound(uprow[0] * b.x + uprow[1]* b.y + uprow[2]);
//			b1.y = cvRound(downrow[0] * b.x + downrow[1]* b.y + downrow[2]);
//			
//			c1.x = cvRound(uprow[0] * c.x + uprow[1]* c.y + uprow[2]);
//			c1.y = cvRound(downrow[0] * c.x + downrow[1]* c.y + downrow[2]);
//				
//				
//			ainter.x = (a1.x*(dist-pos) + a.x*pos)/dist; 
//			ainter.y = (a1.y*(dist-pos) + a.y*pos)/dist; 
////			
//			binter.x = (b1.x*(dist-pos) + b.x*pos)/dist; 
//			binter.y = (b1.y*(dist-pos) + b.y*pos)/dist; 
////			
//			ainter.x = (c1.x*(dist-pos) + c.x*pos)/dist; 
//			ainter.y = (c1.y*(dist-pos) + c.y*pos)/dist; 		
//			//if((ainter.y>0 && ainter.x>0 && ainter.x<img2.cols && ainter.y<img2.rows) && (binter.y>0 && binter.x>0 && binter.x<img2.cols && binter.y<img2.rows) && (cinter.y>0 && cinter.x>0 && cinter.x<img2.cols && cinter.y<img2.rows) ){	
//			cv::line(interpolated[l], ainter, binter, delaunay_color, 1, CV_AA, 0);
//			cv::line(interpolated[l], binter, cinter, delaunay_color, 1, CV_AA, 0);
//			cv::line(interpolated[l], cinter, ainter, delaunay_color, 1, CV_AA, 0);
//		}
//	}
//			
			//******************************************************//
			
	
	return interpolated;
}

cv::Mat delaunayInterpolateSphere(cv::Mat img1, cv::Mat img2, double dist, double pos){
	
	cv::Mat image1;
	//Extend second Image beyond border for border triangles
	cv::Mat image2(img2.rows, img2.cols + img2.cols/2,img2.type());

	//Resulting image
	cv::Mat result = cv::Mat::zeros(img1.rows,img1.cols + img1.cols/2,img1.type());

	//Create Subdivision of Image1
	cv::Subdiv2D subdiv1;
	image1 = img1.clone();
	img2.copyTo(image2(cv::Rect(cv::Point(0, 0), img2.size())));
	//Fill the right side with values of left side to complete second image
	int io = img2.rows;
	int jo = img2.cols;

	


	//Vectors for keypoints
	vector<cv::KeyPoint> keypoints1, keypoints2 ; 
	vector<cv::Point2f> points1, points2;



	//Vector for matches
	vector<cv::DMatch> matches;




	//Retrive keypoints from each image, and match them
	getKeypointsAndMatches(image1, image2, keypoints1, keypoints2,matches);

	
	
	
	//For every point in Keypoints1 -- it's matched keypoint is in Keypoints2 at the same position
	vector<vector<cv::KeyPoint> > matched = getMatchedKeypoints(keypoints1, keypoints2, matches);
	vector<vector<cv::Point2f> > matchedPts = getMatchedPoints(keypoints1, keypoints2, matches);

	//Matched keypoints only
	keypoints1 = matched[0];
	keypoints2 = matched[1];

	//Extracted points from the Keypoints
	points1 = matchedPts[0];
	points2 = matchedPts[1];

	//Create Delaunay triangulation of the first image
	subdiv1 = getDelaunayTriangles(matched[0], image1.rows, image1.cols);

	//Extend 2nd image by half of image size
	for(int i=0;i<io;i++){
		for(int j=jo;j<jo+jo/2;j++){
			//cout << "i,j" << i << "," << j << endl;
			image2.at<Vec3b>(i,j)[0] = img2.at<Vec3b>(i, j-jo)[0];
			image2.at<Vec3b>(i,j)[1] = img2.at<Vec3b>(i, j-jo)[1];
			image2.at<Vec3b>(i,j)[2] = img2.at<Vec3b>(i, j-jo)[2];
		}
	}
	
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
	
	//Vectors for the triangles
	vector<Vec6f> triangles1, triangles2;

	//Retrive the triangles from Image1
	subdiv1.getTriangleList(triangles1);

	//Make matched triangles in image 2
	makeCorrespondingDelaunayTriangles(points1, points2, triangles1,triangles2);

	//Vector for the transformations between all triangles
	vector<cv::Mat> transforms;

	//Get the affine transform between each of the triangles
	transforms = getAffineTriangleTransforms(triangles2, triangles1);


	cv::Mat tWarp(2,3,CV_32FC1) ;
	cv::Point2f p;
	
	
	cout << "Beginning Image point calculations" << endl;
	//Go through all the Triangles in the second image
	for(int k=0;k<triangles2.size();k++){
	//for(int k=10;k<11;k++){
		tWarp = transforms[k];
		//cout << "transform matrix: " << tWarp << endl;
		double* uprow = tWarp.ptr<double>(0);
		double* downrow = tWarp.ptr<double>(1);
		
		//Triangle from Image1					
		Vec6f triangle = triangles1[k];
		
		//Triangle from image2
		Vec6f triangle2 = triangles2[k];
		
		cv::Point2f a,b,c,a1,b1,c1; 
		a.x = triangle[0];
		a.y = triangle[1];
		b.x = triangle[2];
		b.y = triangle[3];
		c.x = triangle[4];
		c.y = triangle[5];
		
		//////For printing purposes
		a1.x = triangle2[0];
		a1.y = triangle2[1];
		b1.x = triangle2[2];
		b1.y = triangle2[3];
		c1.x = triangle2[4];
		c1.y = triangle2[5];
	
		//Triangle from interpolated Image
		cv::Point2f ainter,binter,cinter;
		ainter.x = (a.x*(dist-pos) + a1.x*pos)/dist; 
		ainter.y = (a.y*(dist-pos) + a1.y*pos)/dist; 
		
		binter.x = (b.x*(dist-pos) + b1.x*pos)/dist; 
		binter.y = (b.y*(dist-pos) + b1.y*pos)/dist;
			
		cinter.x = (c.x*(dist-pos) + c1.x*pos)/dist; 
		cinter.y = (c.y*(dist-pos) + c1.y*pos)/dist;
		
		cv::Scalar delaunay_color(0, 0, 0);
		cv::line(result, ainter, binter, delaunay_color, 1, CV_AA, 0);
		cv::line(result, binter, cinter, delaunay_color, 1, CV_AA, 0);
		cv::line(result, cinter, ainter, delaunay_color, 1, CV_AA, 0);
		
		
		Vec6f triangle_inter;
		triangle_inter[0] = ainter.x;
		triangle_inter[1] = ainter.y;
		triangle_inter[2] = binter.x;
		triangle_inter[3] = binter.y;
		triangle_inter[4] = cinter.x;
		triangle_inter[5] = cinter.y;
		
		//Boolean to know which Image to use 
		bool use_first, use_second;
		
		use_first = false;
		use_second = false;
		
		
		if(triangleDifference(triangle2,triangle_inter)==triangleDifference(triangle_inter,triangle)){
			use_first = false;
			use_second = false;
		}
		else if(triangleDifference(triangle,triangle_inter) > triangleDifference(triangle_inter,triangle2)){
			//cout << "Triangle" << k << endl;
			//cout << "Image1 difference: " << triangleDifference(triangle,triangle_inter) << "     " << "Image2 difference:" << triangleDifference(triangle2,triangle_inter) << endl;
			use_first = false;
			use_second = true;
		}
		else if (triangleDifference(triangle,triangle_inter) < triangleDifference(triangle_inter,triangle2) ){
			use_second =  false;
			use_first = true;
		}
		
		//////////////////////////Printing triangle values: //////////////////////////////////
		//cout << "Triangle 2: " << "(" << a.x << "," << a.y << ")" << "|| " << "("<< b.x << "," << b.y << ")" << "||"  << "(" << c.x << "," << c.y << ")"<< endl;
		
		//cout << "Triangle 1: " << "(" << a1.x << "," << a1.y << ")" << "|| " << "("<< b1.x << "," << b1.y << ")" << "||"  << "(" << c1.x << "," << c1.y << ")"<< endl;
		
//		cv::Point2f ainter,binter,cinter;
//		ainter.x = cvRound(uprow[0] * a1.x + uprow[1]* a1.y + uprow[2]);
//		ainter.y = cvRound(downrow[0] * a1.x + downrow[1]* a1.y + downrow[2]);
//		
//		binter.x = cvRound(uprow[0] * b1.x + uprow[1]* b1.y + uprow[2]);
//		binter.y = cvRound(downrow[0] * b1.x + downrow[1]* b1.y + downrow[2]);
//			
//		cinter.x = cvRound(uprow[0] * c1.x + uprow[1]* c1.y + uprow[2]);
//		cinter.y = cvRound(downrow[0] * c1.x + downrow[1]* c1.y + downrow[2]);
		
		
		//cout << "Triangle Interpolated: " << "(" << ainter.x << "," << ainter.y << ")" << "|| " << "("<< binter.x << "," << binter.y << ")" << "||"  << "(" << cinter.x << "," << cinter.y << ")"<< endl;
		//////////////////////////////////////////////////////////////////////////////////////
		
		
		
		//Get the bouding box around the triangle
		int xmax = cvRound(max(a.x, max(b.x,c.x)));
		int ymax = cvRound(max(a.y, max(b.y,c.y)));
		int xmin = cvRound(min(a.x, min(b.x,c.x)));
		int ymin = cvRound(min(a.y, min(b.y,c.y)));


		cv::Point2f p,p2;

		for(int i=ymin;i<=ymax;i++){
			for(int j=xmin;j<=xmax;j++){
				p.x = j;
				p.y = i;


				if(i>0 && j>0 && j<img2.cols && i<img2.rows){
					float x = uprow[0] * p.x + uprow[1]* p.y + uprow[2];
					float y = downrow[0] * p.x + downrow[1]* p.y + downrow[2];


					if(inTriangleArea(p,triangle)){
						//Get the position of the point in the first image
						float x = uprow[0] * p.x + uprow[1]* p.y + uprow[2];
						float y = downrow[0] * p.x + downrow[1]* p.y + downrow[2];
						
						//calculate the resulting position in intermediate image
						float xinter = (x*(dist-pos) + p.x*pos)/dist; 
						float yinter = (y*(dist-pos) + p.y*pos)/dist; 
						
						//
						xinter = cvRound(xinter);
						yinter = cvRound(yinter);
						p2.x = xinter;
						p2.y = yinter;
						if(inTriangleArea(p2,triangle2)){
							//Get interpolated pixel values
							uchar b,g,r = 0;

							if(use_first){
								b = img1.at<Vec3b>(y,x)[0];
								g = img1.at<Vec3b>(y,x)[1];
								r = img1.at<Vec3b>(y,x)[2];
								
								
								result.at<Vec3b>(yinter,xinter)[0] = b;
								result.at<Vec3b>(yinter,xinter)[1] = g;
								result.at<Vec3b>(yinter,xinter)[2] = r;
							}
							
							else if(use_second){
								b = img2.at<Vec3b>(i,j)[0];
								g = img2.at<Vec3b>(i,j)[1];
								r = img2.at<Vec3b>(i,j)[2];
							
								result.at<Vec3b>(yinter,xinter)[0] = b;
								result.at<Vec3b>(yinter,xinter)[1] = g;
								result.at<Vec3b>(yinter,xinter)[2] = r;
							}
							
							else{
								b = 255;
								g = 255;
								r = 255;
								
//								b = (img2.at<Vec3b>(i,j)[0]*pos + img1.at<Vec3b>(y,x)[0]*(dist-pos))/dist;
//								g = (img2.at<Vec3b>(i,j)[1]*pos + img1.at<Vec3b>(y,x)[1]*(dist-pos))/dist;
//								r = (img2.at<Vec3b>(i,j)[2]*pos + img1.at<Vec3b>(y,x)[2]*(dist-pos))/dist;
								result.at<Vec3b>(yinter,xinter)[0] = b;
								result.at<Vec3b>(yinter,xinter)[1] = g;
								result.at<Vec3b>(yinter,xinter)[2] = r;
							
							}
							
//							result.at<Vec3b>(yinter,xinter)[0] = b;
//							result.at<Vec3b>(yinter,xinter)[1] = g;
//							result.at<Vec3b>(yinter,xinter)[2] = r;
						}	
					}
				}
			}
		}
		
	}
	
	return result;
} 



