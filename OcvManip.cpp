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


vector<vector<cv::DMatch> > getMatches(cv::Mat image1, cv::Mat image2,vector<cv::KeyPoint> keypoints1 ,vector<cv::KeyPoint> keypoints2 ){
	cv::Ptr<cv::DescriptorExtractor> extractor = new cv::SurfDescriptorExtractor();;
	cv::BFMatcher matcher(cv::NORM_L2,false);

	std::vector<std::vector<cv::DMatch> > matches;
	//Match using nearest neighboor with 2


	// 1b. Extraction of the SURF descriptors
	cv::Mat descriptors1, descriptors2;
	extractor->compute(image1,keypoints1,descriptors1);
	extractor->compute(image2,keypoints2,descriptors2);

	matcher.knnMatch(descriptors1,descriptors2,matches,2);

	return matches;
}
/**
 * This function returns keypoints and matches between 2 images
 */

void getKeypointsAndMatches(Mat image1, Mat image2, vector<KeyPoint> keypoints1, vector<KeyPoint> keypoints2,vector<DMatch> matches){
	if (!image1.data || !image2.data){
		cout << "getKeypointsAndMatches Error:" <<endl;
		cout<< "One of the input Images does not contains any data please verify" << endl;
		return ;
	}
	RobustMatcher rmatcher;
	rmatcher.setConfidenceLevel(0.98);
	rmatcher.setMinDistanceToEpipolar(1.0);
	rmatcher.setRatio(0.65f);
	cv::Ptr<cv::FeatureDetector> pfd= new cv::SurfFeatureDetector(10); 
	rmatcher.setFeatureDetector(pfd);

	Mat fundemental= rmatcher.match(image1,image2,matches, keypoints1, keypoints2);
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

//cv::Mat linearInterpolate(cv::Mat image1, cv::Mat image2, double dist, double pos){
//	cv::Mat interpol(min(image1.rows,image2.rows), min(image1.cols,image2.cols), image1.type());
//	vector<cv::Keypoint> keypoints1 = getKeypoints(image1);
//	vector<cv::Keypoint> keypoints2 = getKeypoints(image2);
//	
//	std::vector<cv::Point2f> points1, points2;
//	
//	
//	for (std::vector<cv::DMatch>::const_iterator it= outMatches.begin();
//			it!= outMatches.end(); ++it) {

//		// Get the position of left keypoints
//		float x1= keypoints1[it->queryIdx].pt.x;
//		float y1= keypoints1[it->queryIdx].pt.y;
//		points1.push_back(cv::Point2f(x,y));
//		// Get the position of right keypoints
//		float x2= keypoints2[it->trainIdx].pt.x;
//		float y2= keypoints2[it->trainIdx].pt.y;
//		points2.push_back(cv::Point2f(x,y));
//		
//		float x = x1*
//	}

//}




