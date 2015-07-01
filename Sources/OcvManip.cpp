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

/**
* This function draws a triangle on an image
*/
cv::Mat drawTriangleOnImage(Mat image, Vec6f triangle){
	Mat result = image.clone();
	
	Point2f a,b,c;
	
	a.x = triangle[0];
	a.y = triangle[1];
	b.x = triangle[2];
	b.y = triangle[3];
	c.x = triangle[4];
	c.y = triangle[5];
	
	cv::Scalar delaunay_color(255,255,255);
	cv::line(result, a, b, delaunay_color, 1, CV_AA, 0);
	cv::line(result, b, c, delaunay_color, 1, CV_AA, 0);
	cv::line(result, c, a, delaunay_color, 1, CV_AA, 0);
	
	return result;
}

/**********************************************************
* Make video out of images
**********************************************************/
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
		double fps = 24;
	
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

void imageListToVideo(string folder, string basename, int number,string fileName){
	//ostringstream image_name;
	cv::Mat image;
	vector<cv::Mat> images;
	
	
	for(int i=0;i<number;i++){
		ostringstream image_name;
		image_name << folder << "/" << basename << i << ".JPG" ;
		image = imread(image_name.str(),1);
		
		if(!image.data){
			cout << "Error with reading image: " << image_name.str() << endl;
		}
		
		images.push_back(image);
	}
	
	imageListToVideo(images,fileName);

}


/********************************************************
* Get histogram
*********************************************************/
vector<Mat> getHistograms(Mat img){
	//Vector for final histos
	vector<Mat> histograms;
	
	//Vector for splitting channels
	vector<Mat> bgr_planes;


	 /// number of bins
  	int histSize = 256;
  	
  	/// Set the ranges ( for B,G,R) )
	float range[] = { 0, 256 } ;
	const float* histRange = { range };

	bool uniform = true; bool accumulate = false;
	
	Mat b_hist, g_hist, r_hist;
	
	calcHist( &bgr_planes[0], 1, 0, Mat(), b_hist, 1, &histSize, &histRange, uniform, accumulate );
  	calcHist( &bgr_planes[1], 1, 0, Mat(), g_hist, 1, &histSize, &histRange, uniform, accumulate );
  	calcHist( &bgr_planes[2], 1, 0, Mat(), r_hist, 1, &histSize, &histRange, uniform, accumulate );
	
	
	histograms.push_back(b_hist);
	histograms.push_back(g_hist);
	histograms.push_back(r_hist);
	
	return histograms;
}





/***********************************Image Manipulation Cv********************************************/

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


/**
* This function extracts keypoints from a cube of 6 images. 
* These keypoints are normalized to an equirectangular image which was at the origin of the 6 cube faces.
* - this function assumes that the cube was obtained using the makeCubeFaces function
*/
vector<cv::KeyPoint> getCubeKeypoints(cv::Mat origin){
	//All parameters of the cube 
	EquiTrans equi;
	Cube cube;
	Mat faces[6];
	PersCamera cams[6];
	vector<cv::KeyPoint> keypoints;
	vector<cv::KeyPoint> final_keypoints;
	cv::KeyPoint keypoint;


	//Extract Cube faces
	equi.setFOV(90.0, 90.0);
	cout << "getCubeKeypoints: Making Cube Faces" << endl;
	equi.makeCubeFaces2(origin,cube,cams);

	
//	ViewDirection vds[6];
	ViewDirection vd;
	PersCamera cam;
//	Mat mats[6];
	
	//Extract points from each Face and convert coordinates back to Equi Image
	for(int i=0;i<6;i++){
		
		if(!cube[i].data){
			cout << "getCubeKeypoints Cube image: " << i << endl;
			cout << "Sorry the cube is missing some images please verify content" << endl;
			break;
		}
		cout << "getCubeKeypoints: Extracting Keypoints from image:" << i << endl;
		//keypoints = get2DKeypoints(cube[i]);
		keypoints = getSiftKeypoints(cube[i]);
		//kepointsList.push_back(keypoints);
		
		// In order to match the keypoints and keep track of the actually point positions,
		// Convert each keypoint to the Equirectangular initial position 
		// And generate a single keypoints vector
		
		for(int j=0;j<keypoints.size();j++){
			//vd = vds[i];
			keypoint = keypoints[j];
			cv::Point2f point = keypoint.pt;
			//Convert point to Equi coordinates
			//equi.toEquirectCore(double i_c, double j_c, double focal_length, ViewDirection vd, double d_nrows, double d_ncols, double *ec_i, double *ec_j)
			point = equi.toEquiPoint(cams[i],origin,point);
			//keypoint.pt.x = cvRound(point.x);
			//keypoint.pt.y = cvRound(point.y);
			keypoint.pt = point;
			//Set new Keypoint coordinates
			final_keypoints.push_back(keypoint);
		}
		
		cout << "Size of Keypoints in Image: " << i << ": " << keypoints.size() << endl;
	}
	cout << "Size of Final Keypoints  " << ": " << final_keypoints.size() << endl;
	return final_keypoints;

	

}


/** 
* This functions extracts cube faces and matches points on corresponding faces
*/
vector<vector<cv::KeyPoint> > getMatchedCubeKeypoints(cv::Mat img1, cv::Mat img2){
	//All parameters of the cube 
	EquiTrans equi;
	Cube cube1, cube2;
	Mat faces1[6], faces2[6];
	PersCamera cams1[6], cams2[6];
	vector<cv::KeyPoint> keypoints1,keypoints2;
	vector<vector<cv::KeyPoint> > matched_keypoints, final_matched_keypoints;
	vector<cv::KeyPoint> final_keypoints1, final_keypoints2;
	cv::KeyPoint keypoint1, keypoint2;


	//Extract Cube faces
	equi.setFOV(90.0, 90.0);
	//Get Cube Faces for both images
	cout << "getCubeKeypoints: Making Cube Faces" << endl;
	equi.makeCubeFaces2(img1,cube1,cams1);
	equi.makeCubeFaces2(img2,cube2,cams2);
	
	
	//For matches
	vector<cv::DMatch> matches;
	
	//Get Matches on individual faces
	for(int i=0;i<6;i++){
		if(!cube1[i].data ||!cube2[i].data ){
			cout << "getCubeKeypoints Cube image: " << i << endl;
			cout << "Sorry the cube is missing some images please verify content" << endl;
			break;
		}
		
		//Get keypoints from face
		keypoints1 = getSiftKeypoints(cube1[i]);
		keypoints2 = getSiftKeypoints(cube2[i]);
		
		matches = getMatches(cube1[i], cube2[i], keypoints1,keypoints2);
		matched_keypoints = getMatchedKeypoints(keypoints1,keypoints2,matches);
		keypoints1 = matched_keypoints[0];
		keypoints2 = matched_keypoints[1];
		
		
		
		//Convert matches keypoints to Equi Format 
		for(int j=0;j<keypoints1.size();j++){
			keypoint1 = keypoints1[j];
			keypoint2 = keypoints2[j];
			
			cv::Point2f point1 = keypoint1.pt;
			cv::Point2f point2 = keypoint2.pt;
			
			point1 = equi.toEquiPoint(cams1[i],img1,point1);
			point2 = equi.toEquiPoint(cams2[i],img2,point2);			
		
			final_keypoints1.push_back(keypoint1);
			final_keypoints2.push_back(keypoint2);
		}
		
		
	}
	
	final_matched_keypoints.push_back(final_keypoints1);
	final_matched_keypoints.push_back(final_keypoints2);
	
	return final_matched_keypoints;

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
	SiftDescriptorExtractor extractor;
	 //SurfDescriptorExtractor extractor;
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
	
	//keypoints1 = getCubeKeypoints(image1);
	//keypoints2 = getCubeKeypoints(image2);

	//matches = getMatches(image1, image2, keypoints1, keypoints2);

	//matches = getFlannMatches(image1, image2, keypoints1, keypoints2);

	//To compute fundamental matrix at the same time!

		RobustMatcher rmatcher;
		rmatcher.setConfidenceLevel(0.98);
		rmatcher.setMinDistanceToEpipolar(1.0);
		rmatcher.setRatio(0.65f);
		cv::Ptr<cv::FeatureDetector> pfd= new cv::SurfFeatureDetector(10); 
		rmatcher.setFeatureDetector(pfd);

		Mat fundemental= rmatcher.match(image1,image2,matches, keypoints1, keypoints2);
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
	//for (std::vector<cv::DMatch>::const_iterator it= matches.begin();
	//		it!= matches.end(); ++it) {

		// Get the position of left keypoints
	//	points1.push_back(keypoints1[it->queryIdx]);

		// Get the position of right keypoints
		//points2.push_back(keypoints2[it->trainIdx]);
	//}

	for( int i = 0; i < matches.size(); i++ ){ 
	
		points1.push_back(keypoints1[matches[i].queryIdx]);
		points2.push_back(keypoints2[matches[i].trainIdx]);
		//printf( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx ); 
	
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
	trianglesList1 = newtrianglesList1;
	trianglesList2 = newtrianglesList2;

}



/**
*
*/
void getMatchingTrianglesFromImages(Mat image1, Mat image2, vector<Vec6f> &triangles1, vector<Vec6f> &triangles2){
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

	//Subdivision into delaunay
	cv::Subdiv2D subdiv1;
	
	//Create Delaunay triangulation of the first image
	subdiv1 = getDelaunayTriangles(matched[0], image1.rows, image1.cols);
	
	//Vectors for the triangles
	//vector<Vec6f> triangles1, triangles2;

	//Retrive the triangles from Image1
	subdiv1.getTriangleList(triangles1);

	//Make matched triangles in image 2
	makeCorrespondingDelaunayTriangles(points1, points2, triangles1,triangles2);

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

/**
* This function determines wether a point is on a given faces
* - Point defined by it's (i,j) coordinates on equirectangular image1
* - a face is determined by the theta and phi angles of it's center point
*/
bool isOnFaceRaw(int rows, int cols, int i, int j, double theta,double phi){
	//Get the theta and phi angles of the given point
	double new_theta, new_phi;
	SphericFromPixelCoordinates(i,j,cols,rows,new_theta,new_phi);
	
	
	//Have to verify the different cases distinguishing top, bottom from the others
	
	//Case top
	if(theta > 3*PI/4){
	
	}
	
	else if(theta < PI/4){
	
	}
	
	else if(phi<PI/4 || phi>7*PI/4){
	
	}
	//Cases of front side, left, right sides
	else{
		double lower_bound = phi - PI/4;
		double higher_bound = phi + PI/4;
		
		return inInterval(new_phi,lower_bound,higher_bound);
	
	}
	//Verify if point angles are in [] x 
}

/**
* This function determines wether 2 points given in equirectangular coordinates are in the same cubic faces
*/
bool onSameCubicFaceRaw(int width, int height, int i1, int j1, int i2, int j2){

	//Get the theta and phi angles of the given points
	
	
}

/**
* Function to retrieve only interpolated content. Given 2 images with their respective triangle positions and perspective camera parameters.
* this function interpolates the content of the intermediate triangle. 
*/
cv::Mat getDiffPerspInterpolate(cv::Mat img1, cv::Mat img2, PersCamera cam1, PersCamera cam2, cv::Vec6f triangle1, cv::Vec6f triangle2, vector<PointWithColor> &content, double dist, double pos){
	//Resulting image 
	cv::Mat result;
	
	
	//Affine transform
	cv::Mat tWarp;
	
	//Get the interpolated triangle
	Vec6f triangle_inter;
	triangle_inter =  getInterpolatedTriangle( triangle1, triangle2, tWarp,  dist, pos);
	return result;
}





cv::Mat getInterpolatedTriangleContent(cv::Mat img1, cv::Mat img2, cv::Vec6f triangle1, cv::Vec6f triangle2, cv::Vec6f &triangle_inter, vector<PointWithColor> &content, double dist, double pos, int method){
	//Method represents wether to use just image1, just image2, both or triangle blend
	
	//Result image with only triangle
	int rows, cols;
	rows = ceil(img1.rows * (1-pos/dist) + img2.rows * pos/dist);
	cols = ceil(img1.cols * (1-pos/dist) + img2.cols * pos/dist);
	
	cv::Mat result = cv::Mat::zeros(rows,cols,img1.type());
	cv::Mat bi_result =  cv::Mat::zeros(rows,cols,img1.type());
	 
	//Affine transform
	cv::Mat tWarp, affine_inter_img1, affine_inter_img2;
	
	//Get all the affine transformations
	//triangle_inter =  getInterpolatedTriangle( triangle1, triangle2, tWarp,  dist, pos);
	affine_inter_img1 = getAffine2D(triangle_inter,triangle1);
	affine_inter_img2 = getAffine2D(triangle_inter,triangle2);
	
	double* uprow1 = affine_inter_img1.ptr<double>(0);
	double* downrow1 = affine_inter_img1.ptr<double>(1);
	double* uprow2 = affine_inter_img2.ptr<double>(0);
	double* downrow2 =affine_inter_img2.ptr<double>(1);
	
	cv::Point2f a,b,c,a1,b1,c1,a_inter,b_inter,c_inter; 
	a_inter.x = triangle_inter[0];
	a_inter.y = triangle_inter[1];
	b_inter.x = triangle_inter[2];
	b_inter.y = triangle_inter[3];
	c_inter.x = triangle_inter[4];
	c_inter.y = triangle_inter[5];
	
	//Boolean to know which Image to use 
	bool use_first, use_second, use_both, mixed;
		
	use_first = method==1;
	use_second = method==2;
	use_both = method==3;
	mixed = method==4;
	
	
	if(mixed){
		if(triangleDifference(triangle2,triangle_inter)==triangleDifference(triangle_inter,triangle1)){
			use_first = false;
			use_second = false;
			use_both = true;
		}
		else if(triangleDifference(triangle1,triangle_inter) > triangleDifference(triangle_inter,triangle2)){
			//cout << "Triangle" << k << endl;
			//cout << "Image1 difference: " << triangleDifference(triangle,triangle_inter) << "     " << "Image2 difference:" << triangleDifference(triangle2,triangle_inter) << endl;
			use_first = false;
			use_second = true;
		}
		else if (triangleDifference(triangle1,triangle_inter) < triangleDifference(triangle_inter,triangle2) ){
			use_second =  false;
			use_first = true;
		}
	
	}
		
	//Get the bouding box around the triangle
	//int xmax = cvRound(max(a_inter.x, max(b_inter.x,c_inter.x)));
	//int ymax = cvRound(max(a_inter.y, max(b_inter.y,c_inter.y)));
	//int xmin = cvRound(min(a_inter.x, min(b_inter.x,c_inter.x)));
	//int ymin = cvRound(min(a_inter.y, min(b_inter.y,c_inter.y)));
	
	int xmax = (int)ceil(max(a_inter.x, max(b_inter.x,c_inter.x)));
	int ymax = (int)ceil(max(a_inter.y, max(b_inter.y,c_inter.y)));
	int xmin = (int)floor(min(a_inter.x, min(b_inter.x,c_inter.x)));
	int ymin = (int)floor(min(a_inter.y, min(b_inter.y,c_inter.y)));
	
	cv::Point2f p1,pinter,p2;

	//Go through all the points in the intermediate triangle and find positions 
	// in preceeding image
	cv::Scalar color;
	cv::Vec3b icolor,icolor1,icolor2;
	PointWithColor pt_color;
	for(int i=ymin;i<=ymax;i++){
		for(int j=xmin;j<=xmax;j++){
			pinter.x = j;
			pinter.y = i;
			
			uchar b,g,r = 0;
			if(inTriangleArea(pinter,triangle_inter)){
				//Get the position of the point in image 1
				p1.x = uprow1[0]*pinter.x + uprow1[1]*pinter.y + uprow1[2];
				p1.y = downrow1[0]*pinter.x + downrow1[1]*pinter.y + downrow1[2];
				
				//Get the position in image 2
				p2.x = uprow2[0] * p1.x + uprow2[1]* p1.y + uprow2[2];
				p2.y = downrow2[0] * p1.x + downrow2[1]* p1.y + downrow2[2];
						
				if(p1.x>0 && p1.y>0 && p1.y<result.rows && p1.x<result.cols){
						//cout << "Point Position in interp: " << xinter << "," << yinter << endl;			
					if(use_first){
						b = img1.at<Vec3b>(p1)[0];
						g = img1.at<Vec3b>(p1)[1];
						r = img1.at<Vec3b>(p1)[2];
						icolor = bilinearInterpolate(img1,p1.y,p1.x);
					}
					else if(use_second){
						b = img2.at<Vec3b>(p2)[0];
						g = img2.at<Vec3b>(p2)[1];
						r = img2.at<Vec3b>(p2)[2];
						icolor = bilinearInterpolate(img2,p2.y,p2.x);
					}
					else if(use_both){
						icolor1 = bilinearInterpolate(img1,p1.y,p1.x);
						icolor2 = bilinearInterpolate(img2,p2.y,p2.x);
						
						icolor[0] = (icolor1[0]*(dist-pos) + icolor2[0]*pos)/dist;
						icolor[1] = (icolor1[1]*(dist-pos) + icolor2[1]*pos)/dist;
						icolor[2] = (icolor1[2]*(dist-pos) + icolor2[2]*pos)/dist;
						
					} 
					
					color = cv::Scalar(b,g,r);
					//pt_color.x = xinter;
					//pt_color.y = yinter;
					//pt_color.color = color;
					content.push_back(pt_color);
					result.at<Vec3b>(pinter)[0] = b;
					result.at<Vec3b>(pinter)[1] = g;
					result.at<Vec3b>(pinter)[2] = r;
					
					bi_result.at<Vec3b>(pinter)[0] = icolor[0];
					bi_result.at<Vec3b>(pinter)[1] = icolor[1];
					bi_result.at<Vec3b>(pinter)[2] = icolor[2];
				}
	
			}
				
		}
	}
		
		
	return bi_result;
}


/**
* Function to retrieve only interpolated content. Given 2 images with their respective triangle positions,
* this function interpolates the content of the intermediate triangle using different perspective cameras. 
* 
*/
cv::Mat getInterpolatedTriangleContentDiffCam(PersCamera cam1, PersCamera cam2, cv::Vec6f triangle1, cv::Vec6f triangle2, cv::Vec6f &triangle_inter, vector<PointWithColor> &content, double dist, double pos, int method){
	//For function calling
	Triangle tri_class;
	EquiTrans trans_class;
	

	//Get perspective images
	cv::Mat img1 = cam1.image, img2 = cam2.image;
	
	//Result image with only triangle
	int rows, cols;
	rows = img1.rows * (1-pos/dist) + img2.rows * pos/dist;
	cols = img1.cols * (1-pos/dist) + img2.cols * pos/dist;
	cv::Mat result = cv::Mat::zeros(rows,cols,img1.type());
	cv::Mat bi_result = result.clone();
	
	//Affine transform
	cv::Mat tWarp, affine_inter_img1, affine_inter_img2;
	
	//Get all the affine transformations
	triangle_inter =  getInterpolatedTriangle( triangle1, triangle2, tWarp,  dist, pos);
	affine_inter_img1 = getAffine2D(triangle_inter,triangle1);
	affine_inter_img2 = getAffine2D(triangle_inter,triangle2);
	
	double* uprow1 = affine_inter_img1.ptr<double>(0);
	double* downrow1 = affine_inter_img1.ptr<double>(1);
	double* uprow2 = affine_inter_img2.ptr<double>(0);
	double* downrow2 =affine_inter_img2.ptr<double>(1);
	
	cv::Point2f a1,b1,c1,a_inter,b_inter,c_inter; 
	a_inter.x = triangle_inter[0];
	a_inter.y = triangle_inter[1];
	b_inter.x = triangle_inter[2];
	b_inter.y = triangle_inter[3];
	c_inter.x = triangle_inter[4];
	c_inter.y = triangle_inter[5];
	
	//Boolean to know which Image to use 
	bool use_first, use_second, use_both, mixed;
		
	use_first = method==1;
	use_second = method==2;
	use_both = method==3;
	mixed = method==4;
	
	
	if(mixed){
		if(triangleDifference(triangle2,triangle_inter)==triangleDifference(triangle_inter,triangle1)){
			use_first = false;
			use_second = false;
			use_both = true;
		}
		else if(triangleDifference(triangle1,triangle_inter) > triangleDifference(triangle_inter,triangle2)){
			//cout << "Triangle" << k << endl;
			//cout << "Image1 difference: " << triangleDifference(triangle,triangle_inter) << "     " << "Image2 difference:" << triangleDifference(triangle2,triangle_inter) << endl;
			use_first = false;
			use_second = true;
		}
		else if (triangleDifference(triangle1,triangle_inter) < triangleDifference(triangle_inter,triangle2) ){
			use_second =  false;
			use_first = true;
		}
	
	}
		
	//Get the bouding box around the triangle
	int xmax = cvRound(max(a_inter.x, max(b_inter.x,c_inter.x)));
	int ymax = cvRound(max(a_inter.y, max(b_inter.y,c_inter.y)));
	int xmin = cvRound(min(a_inter.x, min(b_inter.x,c_inter.x)));
	int ymin = cvRound(min(a_inter.y, min(b_inter.y,c_inter.y)));
	
	cv::Point2f p1,pinter,p2;

	//Go through all the points in the intermediate triangle and find positions 
	// in preceeding image
	cv::Scalar color;
	cv::Vec3b icolor,icolor1,icolor2;
	PointWithColor pt_color;
	
	if(inTriangleArea(pinter,triangle_inter)){
				uchar b,g,r = 0;
				//Get the position of the point in image 1
				p1.x = uprow1[0]*pinter.x + uprow1[1]*pinter.y + uprow1[2];
				p1.y = downrow1[0]*pinter.x + downrow1[1]*pinter.y + downrow1[2];
				
				//Get the position in image 2
				p2.x = uprow2[0] * p1.x + uprow2[1]* p1.y + uprow2[2];
				p2.y = downrow2[0] * p1.x + downrow2[1]* p1.y + downrow2[2];
						
				if(p1.x>0 && p1.y>0 && p1.y<result.rows && p1.x<result.cols){
						//cout << "Point Position in interp: " << xinter << "," << yinter << endl;			
					if(use_first){
						b = img1.at<Vec3b>(p1)[0];
						g = img1.at<Vec3b>(p1)[1];
						r = img1.at<Vec3b>(p1)[2];
						icolor = bilinearInterpolate(img1,p1.y,p1.x);
					}
					else if(use_second){
						b = img2.at<Vec3b>(p2)[0];
						g = img2.at<Vec3b>(p2)[1];
						r = img2.at<Vec3b>(p2)[2];
						icolor = bilinearInterpolate(img2,p2.y,p2.x);
					}
					else if(use_both){
						icolor1 = bilinearInterpolate(img1,p1.y,p1.x);
						icolor2 = bilinearInterpolate(img2,p2.y,p2.x);
						
						icolor[0] = (icolor1[0]*(dist-pos) + icolor2[0]*pos)/dist;
						icolor[1] = (icolor1[1]*(dist-pos) + icolor2[1]*pos)/dist;
						icolor[2] = (icolor1[2]*(dist-pos) + icolor2[2]*pos)/dist;
						
					} 
					
					color = cv::Scalar(b,g,r);
					//pt_color.x = xinter;
					//pt_color.y = yinter;
					//pt_color.color = color;
					content.push_back(pt_color);
					result.at<Vec3b>(pinter)[0] = b;
					result.at<Vec3b>(pinter)[1] = g;
					result.at<Vec3b>(pinter)[2] = r;
					
					bi_result.at<Vec3b>(pinter)[0] = icolor[0];
					bi_result.at<Vec3b>(pinter)[1] = icolor[1];
					bi_result.at<Vec3b>(pinter)[2] = icolor[2];
				}
	
			}
				
		
	
	return bi_result;
}

/**
* Function to interpolate between 2 images having already extracted keypoints and matched them and computed
* triangulation
* Inputs:
*	- Img1, Img2 
*	- Triangles1, Triangles2: matched triangles
* Outputs:
*	- Interpolated image
*/
cv::Mat InterpolateWithGivenTriangles(cv::Mat img1, cv::Mat img2, vector<cv::Vec6f> triangles1, vector<cv::Vec6f> triangles2, double dist, double pos){
	
	//Resulting Mat;
	 cv::Mat result = cv::Mat::zeros(img1.rows,img1.cols,img1.type());
	 
	 
	//cv::Mat img1 = image1;
	//cv::Mat img2 = image2;
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
		Vec6f triangle1 = triangles1[k];
		
		//Triangle from image2
		Vec6f triangle2 = triangles2[k];
		
		cv::Point2f a,b,c,a1,b1,c1; 
		a.x = triangle1[0];
		a.y = triangle1[1];
		b.x = triangle1[2];
		b.y = triangle1[3];
		c.x = triangle1[4];
		c.y = triangle1[5];
		

		//Boolean to know which Image to use 
		bool use_first, use_second;
		
		use_first = false;
		use_second = false;
		
		
			
		
		//Get the bouding box around the triangle
		int xmax = cvRound(max(a.x, max(b.x,c.x)));
		int ymax = cvRound(max(a.y, max(b.y,c.y)));
		int xmin = cvRound(min(a.x, min(b.x,c.x)));
		int ymin = cvRound(min(a.y, min(b.y,c.y)));
		
		
		cv::Vec6f triangle_inter;
		triangle_inter =  getInterpolatedTriangle( triangle1, triangle2, tWarp,  dist, pos);

		cv::Point2f p,pinter,p2;

		for(int i=ymin;i<=ymax;i++){
			for(int j=xmin;j<=xmax;j++){
				p.x = j; 
				p.y = i;


				if(i>0 && j>0 && j<img2.cols && i<img2.rows){
					//float x = uprow[0] * p.y + uprow[1]* p.x + uprow[2];
					//float y = downrow[0] * p.y + downrow[1]* p.x + downrow[2];


					if(inTriangleArea(p,triangle1)){
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
							if(triangleDifference(triangle2,triangle_inter)==triangleDifference(triangle_inter,triangle1)){
			use_first = false;
			use_second = false;
		}
		else if(triangleDifference(triangle1,triangle_inter) > triangleDifference(triangle_inter,triangle2)){
			//cout << "Triangle" << k << endl;
			//cout << "Image1 difference: " << triangleDifference(triangle,triangle_inter) << "     " << "Image2 difference:" << triangleDifference(triangle2,triangle_inter) << endl;
			use_first = false;
			use_second = true;
		}
		else if (triangleDifference(triangle1,triangle_inter) < triangleDifference(triangle_inter,triangle2) ){
			use_second =  false;
			use_first = true;
		}
							if(!use_first & !use_second){
								if(pos<dist/2){
									use_second = true;
								}
								else{
									use_first = true;
								}
							}
							if(use_first){
								b = img1.at<Vec3b>(p)[0];
								//b = 255;
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
								r = img2.at<Vec3b>(p2)[2];
								//r = 255;
							
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
}


cv::Mat delaunayInterpolate(cv::Mat img1, cv::Mat img2, double dist, double pos){
	
	cv::Mat image1;
	//Extend second Image beyond border for border triangles
	cv::Mat image2(img2.rows, img2.cols + img2.cols/2,img2.type());

	//Resulting image
	cv::Mat result = cv::Mat::zeros(img1.rows,img1.cols,img1.type());
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
		
		
		//Skip big triangles outside image1
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
								if(pos<dist/3){
									use_second = true;
								}
								else if(pos>dist*2/3){
									use_first = true;
								}
							}
							if(use_first){
								b = img1.at<Vec3b>(p)[0];
								//b = 255;
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
								r = img2.at<Vec3b>(p2)[2];
								//r = 255;
							
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
								b = (img2.at<Vec3b>(p2)[0]*pos + img1.at<Vec3b>(p)[0]*(dist-pos))/dist;
								g = (img2.at<Vec3b>(p2)[1]*pos + img1.at<Vec3b>(p)[1]*(dist-pos))/dist;
								r = (img2.at<Vec3b>(p2)[2]*pos + img1.at<Vec3b>(p)[2]*(dist-pos))/dist;
						
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
	
	double fov_h = 91.0/180.0 *M_PI; // 90.0 degrees
  	double fov_v = 91.0/180.0 * M_PI;
	ViewDirection vd;
	vd.pan = 180/180.0 * M_PI;
    	vd.tilt = 0.0/180.0 * M_PI;
    	PersCamera cam;
    	cam.setCamera(result, fov_h, fov_v, vd);
    	
    	EquiTrans trans_class;
    	
    	//Change to get forward or full image
    	bool interpolateForward = false;
    	if(!interpolateForward){
		return result;
	}
	else{
		return trans_class.toPerspective(result,cam);
	}
} 

/** 
* 
*/
cv::Mat delaunayInterpolateBilinear(cv::Mat img1, cv::Mat img2, double dist, double pos){
	cv::Mat image1;
	//Extend second Image beyond border for border triangles
	cv::Mat image2(img2.rows, img2.cols + img2.cols/2,img2.type());

	//Resulting image
	cv::Mat result = cv::Mat::zeros(img1.rows,img1.cols,img1.type());
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

	Vec6f triangle1, triangle2, triangle_inter;
	for(int k=0;k<triangles1.size();k++){
		//Get triangle
		triangle1 = triangles1[k];
		triangle2 = triangles2[k];
		
		//Affine transform
		cv::Mat tWarp, affine_inter_img1, affine_inter_img2;
	
		//Get all the affine transformations
		triangle_inter =  getInterpolatedTriangle( triangle1, triangle2, tWarp,  dist, pos);
		affine_inter_img1 = getAffine2D(triangle_inter,triangle1);
		affine_inter_img2 = getAffine2D(triangle_inter,triangle2);
	
		
		double* uprow1 = affine_inter_img1.ptr<double>(0);
		double* downrow1 = affine_inter_img1.ptr<double>(1);
		double* uprow2 = affine_inter_img2.ptr<double>(0);
		double* downrow2 =affine_inter_img2.ptr<double>(1);
	
		cv::Point2f a,b,c,a1,b1,c1,a_inter,b_inter,c_inter; 
		a_inter.x = triangle_inter[0];
		a_inter.y = triangle_inter[1];
		b_inter.x = triangle_inter[2];
		b_inter.y = triangle_inter[3];
		c_inter.x = triangle_inter[4];
		c_inter.y = triangle_inter[5];
		
		
		//Boolean to know which Image to use 
		bool use_first, use_second;
		
		use_first = false;
		use_second = false;
		
		
		if(triangleDifference(triangle2,triangle_inter)==triangleDifference(triangle_inter,triangle1)){
			use_first = false;
			use_second = false;
		}
		else if(triangleDifference(triangle1,triangle_inter) > triangleDifference(triangle_inter,triangle2)){
			use_first = false;
			use_second = true;
		}
		else if (triangleDifference(triangle1,triangle_inter) < triangleDifference(triangle_inter,triangle2) ){
			use_second =  false;
			use_first = true;
		}
		
		
		
		//Get the bouding box around the triangle
		int xmax = cvRound(max(a_inter.x, max(b_inter.x,c_inter.x)));
		int ymax = cvRound(max(a_inter.y, max(b_inter.y,c_inter.y)));
		int xmin = cvRound(min(a_inter.x, min(b_inter.x,c_inter.x)));
		int ymin = cvRound(min(a_inter.y, min(b_inter.y,c_inter.y)));
		
		cv::Point2f p1,pinter,p2;

		//Go through all the points in the intermediate triangle and find positions 
		// in preceeding image
		cv::Scalar color;
		cv::Vec3b icolor,icolor1,icolor2;
		
		for(int i=ymin;i<=ymax;i++){
			for(int j=xmin;j<=xmax;j++){
				pinter.x = j;
				pinter.y = i;
			
				if(inTriangleArea(pinter,triangle_inter)){
					
					
					
					//Get the position of the point in image 1
					p1.x = uprow1[0]*pinter.x + uprow1[1]*pinter.y + uprow1[2];
					p1.y = downrow1[0]*pinter.x + downrow1[1]*pinter.y + downrow1[2];
				
					//Get the position in image 2
					p2.x = uprow2[0] * pinter.x + uprow2[1]* pinter.y + uprow2[2];
					p2.y = downrow2[0] * pinter.x + downrow2[1]* pinter.y + downrow2[2];
						
					if(p1.x>0 && p1.y>0 && p1.y<result.rows && p1.x<result.cols){
						//cout << "Point pinter:" << pinter << endl;
						//cout << "affine_inter_img1: " << affine_inter_img1 << endl;
						//cout << "affine_inter_img2: " << affine_inter_img2 << endl;			
						//if(use_first){
							icolor = bilinearInterpolate(img1,p1.y,p1.x);
						//}
						//else if(use_second){
							icolor = bilinearInterpolate(img2,p2.y,p2.x);
						//}
						//else{
							//cout << "Point p1:" << p1 << endl;
							icolor1 = bilinearInterpolate(img1,p1.y,p1.x);
							//cout << "P1 color:" << icolor1 << endl; 

							
							//cout << "Point p2:" << p2 << endl;
							icolor2 = bilinearInterpolate(img2,p2.y,p2.x);
							//cout << "P1 color:" << icolor1 << endl; 
							
							
						
							icolor[0] = (icolor1[0]*(dist-pos) + icolor2[0]*pos)/dist;
							icolor[1] = (icolor1[1]*(dist-pos) + icolor2[1]*pos)/dist;
							icolor[2] = (icolor1[2]*(dist-pos) + icolor2[2]*pos)/dist;
						
						//} 
						
						//cout << "Color: " << icolor << endl;
						//cout << "----------------------------------------" << endl;
						result.at<Vec3b>(pinter) = icolor;
						//result.at<Vec3b>(pinter)[0] = icolor[0];
						//result.at<Vec3b>(pinter)[1] = icolor[1];
						//result.at<Vec3b>(pinter)[2] = icolor[2];
					}
	
				}
				
			}
		}
	
	}
	
	double fov_h = 91.0/180.0 *M_PI; // 90.0 degrees
  	double fov_v = 91.0/180.0 * M_PI;
	ViewDirection vd;
	vd.pan = 180/180.0 * M_PI;
    	vd.tilt = 0.0/180.0 * M_PI;
    	PersCamera cam;
    	cam.setCamera(result, fov_h, fov_v, vd);
    	
    	EquiTrans trans_class;
    	
    	//Change to get forward or full image
    	bool interpolateForward = false;
    	if(!interpolateForward){
		return result;
	}
	else{
		return trans_class.toPerspective(result,cam);
	}

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
	PersCamera cam;
	
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
							if(!use_first & !use_second){
								if(pos<dist/2){
									use_second = true;
								}
								else{
									use_first = true;
								}
							}
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

/** 
* Function to interpolate between 2 images using Delaunay triangulation using triangles on the surface of the sphere and cube faces. This first function generates and writes the triangles onto file
*/
void delaunayInterpolateCubeMakeTriangles(cv::Mat img1, cv::Mat img2, double dist, double pos,string file1, string file2){
	//Final Output image
	cv::Mat result;
	
	//Convert images to Cubes 
	EquiTrans equi;
	Cube cube1, cube2;
	Mat faces1[6] ,faces2[6];
	PersCamera cams1[6], cams2[6];
	//ViewDirection vds[6];
	vector<vector<cv::KeyPoint> > matched_keypoints;
	vector<cv::KeyPoint> keypoints1, keypoints2;
	vector<cv::Point2f> points1, points2;
	vector<PointXYZRGB> points3D1, points3D2;
	vector<cv::Point3d> points3D1c, points3D2c;
	vector<cv::DMatch> matches;
	
	//equi.setFOV(90.0, 90.0);
	//equi.makeCubeFaces2(img1,cube1,cams1);
	//equi.makeCubeFaces2(img2,cube2,cam2);
	
	
	//Extract Keypoints on Perspective images, and match them on Omnidirectional Images
	//keypoints1 =  getCubeKeypoints(img1);
	keypoints1 = get2DKeypoints(img1);
	//keypoints2 =  getCubeKeypoints(img2);
	keypoints2 =  get2DKeypoints(img2);
	matches = getFlannMatches(img1, img2,keypoints1 ,keypoints2);
	matched_keypoints = getMatchedKeypoints(keypoints1, keypoints2, matches);
	//Matched Keypoints on OmniDirectional Images
	keypoints1 = matched_keypoints[0];
	keypoints1 = matched_keypoints[1];
	//Get Points from Keypoints
	vector<vector<cv::Point2f> > matchedPts = getMatchedPoints(keypoints1, keypoints2, matches);
	points1 = matchedPts[0];
	points2 = matchedPts[1];
	
	
	//Convert Keypoints from Omni to 3D euclidien based on Elay's Function
	//points3D1 = sphereCoordinatesList(img1.rows, img1.cols, points1);
	//points3D2 = sphereCoordinatesList(img2.rows, img2.cols, points2);
	
	//Convert Keypoints from Omni to 3D euclidien based on Chiba's Function
	PointFeature feat;
	feat.toSpherePoints(img1,keypoints1,points3D1c);
	feat.toSpherePoints(img2,keypoints2,points3D2c);
	//Save to file 
	feat.writePoints3d(points3D1c, file1);
	feat.writePoints3d(points3D2c, file2);
	
	
} 

/** 
* Function to interpolate between 2 images using Delaunay triangulation using triangles on the surface of * the sphere and cube faces. This is the 2nd part of the previous function. Supposes the 3D triangles * have been interpolated:
* @Input:
*	Img1, Img2: Omnidirectional images
*	dist, pos: distance between 2 images and position being interpolated
*	Points1c, Points2c: Matched points from img1 and img2 respectively
*	triangles_file: file name for triangles from img1 in 3D
*	
*/
cv::Mat delaunayInterpolateCubeFromTriangles(cv::Mat img1, cv::Mat img2, double dist, double pos, string triangles_file, vector<PointXYZRGB> points1c, vector<PointXYZRGB> points2c, int nb_inter, int method, string outfile){
	cout << "delaunayInterpolateCubeFromTriangles Called" << endl;
	cout << "delaunayInterpolateCubeFromTriangles: Making " << nb_inter << " Interpolated images" << endl ;
	
	
	//For calling functiona
	EquiTrans trans_class;
	 PointFeature feat;
	 Triangle tri_class;
	 
	//3D Triangles
	//vector<Vec9f> triangles3D1, triangles3D2;
	
	//vector< vector<float> > triangles3D1c, triangles3D2c;
	vector<Vec9f> triangles3D1c, triangles3D2c;
	
	triangles3D1c = feat.readTriangles3d(triangles_file);
	cout << "Triangles 3D size: " << triangles3D1c.size() << endl;
	//feat.readTriangles3d(triangles2,&triangles3D2c );
	

	
	//First get matched triangles
	vector<cv::Vec6f> triangles_list1, triangles_list2;
	
	makeCorrespondingDelaunayTriangles3D(points1c, points2c, triangles3D1c, triangles3D2c);
	
	//Cut the bigger triangles into smaller triangles
	tri_class.subdivideMatched(triangles3D1c,triangles3D2c);
	
	//Write Corresponding
	//feat.writeTriangles3d(triangles3D1c, "Txt_files/TrianglePoints3D_test3_1_matched.txt");
	//feat.writeTriangles3d(triangles3D2c, "Txt_files/TrianglePoints3D_test4_1_matched.txt");
	
	triangles_list1 = feat.convTriangles3dToEqui(triangles3D1c, img1);
	triangles_list2 = feat.convTriangles3dToEqui(triangles3D2c, img2);
	cout << "Triangles 2D size: " << triangles_list1.size() << endl;
	
	//convToPersRectImage2()
	//Testing 
	//for(int i=0;i<triangles_list1.size();i++){
	//	Vec6f triangle = triangles_list1[i];
	//	cout << "Triangle number " << i << ": " << triangle[0] << "," << triangle[1] << endl;
	//}
	//End testing
	
	
	//Go through the triangles
	Vec6f triangle1, triangle2, triangle_inter, triangle_persp1,triangle_persp2, triangle_inter_persp ;
	Vec9f triangle3d1,triangle3d2, triangle3d_inter;
	PersCamera cam1, cam2, cam_inter;
	cv::Mat persp1, persp2, inter, temp;
	vector<PointWithColor> content;
	
	
	
	//Use boolean sameView to test using single view and multiple view
	bool sameView = true;
	//Change to get forward or full image
    	bool interpolateForward = false;
	
	
	//Define list to make video
	vector<cv::Mat> images;
	cv::Mat result;
	PersCamera cam;
	
	cout << "*******Brace yourself, this will take about: " << nb_inter*triangles3D1c.size() << " seconds " << endl;
	for(int internum=0; internum<nb_inter; internum++){
	
		result = cv::Mat::zeros(img1.rows, img1.cols, img1.type());
		//Orignals reconstructed
		cv::Mat result1 = result.clone();
		cv::Mat result2 = result.clone();
	
		//Get position 
		if(nb_inter > 1){
			pos = (double)(internum+1)/(double)nb_inter;
		}
		cout << "Interpolating position: " << pos << endl;
		ostringstream result_file; 
		result_file <<  outfile << internum << ".JPG" ;
		
		if(sameView){
			for(int i=0;i<triangles_list1.size();i++){
				cout << "Processing Triangle " << i << endl;
				//for(int i=0;i<10;i++){
				ostringstream img1_file,img2_file,img_inter_file;
				 
				 
				//3D triangle
				 triangle3d1 = triangles3D1c[i];
				 triangle3d2 = triangles3D2c[i];
				 cout << "3D Triangle1 : " << triangle3d1 << endl;
				 cout << "3D Triangle2 : " << triangle3d2 << endl;
				 
				 //Get interpolated 3D triangle
				 triangle3d_inter = getInterpolatedTriangle3D(triangle3d1, triangle3d2, dist,pos);
				 triangle3d_inter = projectTriangle2Sphere(triangle3d_inter,1);
				 cout << "Interpolated 3D Triangle: " << triangle3d_inter << endl;
				
				//Equirectangular triangle
				 triangle1 = triangles_list1[i];
				 triangle2 = triangles_list2[i];
				 triangle_inter = tri_class.convVec9fToVec6f(img1,triangle3d_inter);
				
				//Generate the perspective view for the given triangle using the same view for 
				// both triangles
				//inter = cv::Mat::zeros(img1.rows, img1.cols, img1.type());
				//Image 1 and camera 1
				//cam1 = tri_class.getPersCamParamsTwo(img1,triangle1,triangle2);
				cam1 = tri_class.getPersCamParamsTwo(img1,triangle3d1,triangle3d2);
				//cam1 = tri_class.getPersCamParamsBarycentricTwo(img1,triangles3D1c[i],triangles3D2c[i]);
		
		
				persp1 = trans_class.toPerspective(img1,cam1);
				cam1.image = persp1;
		
		
				img1_file << "OldPerpResults/PerspImg1/Img1 Perpspective " << i << ".JPG"; 
				imwrite(img1_file.str(), persp1);
		
				//Image 2 and camera 2
				//cam2 = tri_class.getPersCamParamsTwo(img2,triangle1,triangle2);
				cam2 = tri_class.getPersCamParamsTwo(img2,triangle3d1,triangle3d2);
				//cam2 = tri_class.getPersCamParamsBarycentricTwo(img2,triangles3D1c[i],triangles3D2c[i]);
				persp2 = trans_class.toPerspective(img2,cam2);
				img2_file << "OldPerpResults/PerspImg2/Img2 Perpspective " << i << ".JPG"; 
				imwrite(img2_file.str(), persp2);
				cam2.image = persp2;
		
		
		
				//Get perspective triangle locations
				triangle_persp1 = tri_class.convToPersTriangle(img1,cam1,triangle1);
				triangle_persp2 = tri_class.convToPersTriangle(img2,cam2,triangle2);
				triangle_inter_persp = tri_class.convToPersTriangle(img1,cam1,triangle_inter);
				expandTriangle(img1, triangle_inter, 4);
		
		
				//Output images with drawn triangles
				//temp = drawTriangleOnImage(persp1,triangle_persp1);
				//imwrite(img1_file.str(), temp);
				//temp = drawTriangleOnImage(persp2,triangle_persp2);
				//imwrite(img2_file.str(), temp);
		
				//Interpolated camera
				cam_inter = cam1;
				//cam_inter = tri_class.getInterpolatedPersCamParams(cam1,cam2,dist,pos);
				
				
				clock_t t_int = clock();
				//Get inter triangle on given perspective camera
				//triangle_inter_persp = trans_class.conv3DTriangletoPers(tri_class.convVec9fToPoint3d(triangle3d_inter), cam_inter);
				cout << "Triangle Inter Perspective: " << triangle_inter_persp << endl;
				inter = getInterpolatedTriangleContent(persp1,persp2,triangle_persp1,triangle_persp2,triangle_inter_persp,content,dist,pos,method);
				printf("Single Triangle Execution Time: %.2fs\n", (double)(clock() - t_int)/CLOCKS_PER_SEC);
		
				//For drawing purposes
				inter = drawTriangleOnImage(inter,triangle_inter_persp);
		
				img_inter_file << "OldPerpResults/PerspInter/Img Inter Perpspective " << i << ".JPG"; 
				//imwrite(img_inter_file.str(), inter);
				cam_inter.image = inter;
		
				//Draw inter with triangle
		
				//imwrite(img_inter_file.str(), inter);
				//triangle_inter_persp = tri_class.convToPersTriangle(result,cam_inter,triangle_inter);
		
		
				//For testing purposes
				//persp1 = drawTriangleOnImage(persp1,triangle_persp1);
				//cam1.image = persp1;
				//persp2 = drawTriangleOnImage(persp2,triangle_persp2);
				//cam2.image = persp2;
				//Reconstruct original images using 
				//trans_class.toEquirectangular(cam1, triangle_persp1, result1);
				//trans_class.toEquirectangular(cam2, triangle_persp2, result2);
				//result1 = drawTriangleOnImage(result1,triangle1);
				//result2 = drawTriangleOnImage(result2,triangle2);
		
				//End of testing
		
		
		
				trans_class.toEquirectangular(cam_inter, triangle_inter_persp, result);
				//	temp = result.clone();
				//	images.push_back(temp);
			}
			
			//imageListToVideo(images,"OldPerpResults/triangle_formation.mpeg");
		
		}
	
		else{
			
			for(int i=0;i<triangles_list1.size();i++){
				cout << "Processing Triangle " << i << endl;
				//for(int i=0;i<2;i++){
				ostringstream img1_file,img2_file,img_inter_file;
		
				//Equirectangular image triangle
				 triangle1 = triangles_list1[i];
				 triangle2 = triangles_list2[i];
				 cv::Mat tWarp;
				 triangle_inter = getInterpolatedTriangle( triangle1, triangle2, tWarp,  dist, pos);
				 
				 
				 //3D triangle
				 triangle3d1 = triangles3D1c[i];
				 triangle3d2 = triangles3D2c[i];
				 
				 
	
		
				//Generate the perspective view for the given triangle using the same view for 
				// both triangles
				//inter = cv::Mat::zeros(img1.rows, img1.cols, img1.type());
				//Image 1 and camera 1
				//cam1 = tri_class.getPersCamParams(img1,triangle1);
				cam1 = tri_class.getPersCamParams(img1,triangle3d1);
				//cam1 = tri_class.getPersCamParamsBarycentric(img1,triangles3D1c[i]);
				persp1 = trans_class.toPerspective(img1,cam1);
				cam1.image = persp1;
				img1_file << "OldPerpResults/PerspImg1/Img1 Perpspective " << i << ".JPG"; 
				imwrite(img1_file.str(), persp1);
		
		
				//Image 2 and camera 2
				//cam2 = tri_class.getPersCamParams(img2,triangle2);
				cam2 = tri_class.getPersCamParams(img2,triangle3d2);
				//cam2 = tri_class.getPersCamParamsBarycentric(img2,triangles3D2c[i]);
				persp2 = trans_class.toPerspective(img2,cam2);
				cam2.image = persp2;
				img2_file << "OldPerpResults/PerspImg2/Img2 Perpspective " << i << ".JPG";
		
				imwrite(img2_file.str(), persp2);
		
		
				//Get perspective triangle locations
				triangle_persp1 = tri_class.convToPersTriangle(img1,cam1,triangle1);
				triangle_persp2 = tri_class.convToPersTriangle(result,cam2,triangle2);
		
				//Interpolated camera
				cam_inter = tri_class.getPersCamParams(img1,triangle_inter);
				//cam_inter = tri_class.getInterpolatedPersCamParams(cam1,cam2,dist,pos);
				//inter = getInterpolatedTriangleContentDiffCam(cam1,cam2,triangle_persp1,triangle_persp2,triangle_inter_persp,content,dist,pos,3);
				clock_t t_int = clock();
				inter = getInterpolatedTriangleContent(persp1,persp2,triangle_persp1,triangle_persp2,triangle_inter_persp,content,dist,pos,method);
				printf("Single Triangle Execution Time: %.2fs\n", (double)(clock() - t_int)/CLOCKS_PER_SEC);
				img_inter_file << "OldPerpResults/PerspInter/Img Inter Perpspective " << i << ".JPG"; 
				imwrite(img_inter_file.str(), inter);
				cam_inter.image = inter;
		
				//Put the content in the resulting image...Iteratively fill the image
				//cam_inter.image = inter;
				trans_class.toEquirectangular(cam_inter, triangle_inter_persp, result);
		
				//cvNamedWindow("Temp Result",0);
				//imshow("Temp Result", result);
				//waitKey(0);
			}
		}
	
		
	
		
    		
		double fov_h = 91.0/180.0 *M_PI; // 90.0 degrees
  		double fov_v = 91.0/180.0 * M_PI;
		ViewDirection vd;
		vd.pan = 180/180.0 * M_PI;
    		vd.tilt = 0.0/180.0 * M_PI;
    		
    		cam.setCamera(result, fov_h, fov_v, vd);
		
		if(interpolateForward){
			result = trans_class.toPerspective(result,cam);
		}
		
		
		//Write Results 
		imwrite(result_file.str(),result);	
		temp = result.clone();
		images.push_back(temp);
		
	
	}
	bool video_flag = false;
	
	if(video_flag){
		ostringstream video_file;
		video_file << outfile << ".mpeg";
		imageListToVideo(images,video_file.str());
	}
	
    	
    	//EquiTrans trans_class;
    	if(!interpolateForward){
		return result;
	}
	else{
		return trans_class.toPerspective(result,cam);
	}
	//return result;
}

/*
 * set the background to gray for debugging.
 */
void setBackground(cv::Mat &src, cv::Scalar color){
  src = color;
}

/*
 * expand triangle to fill the gaps.
 */
void expandTriangle(Mat image, cv::Vec6f &triangle, int width){
  PointFeature feat;
  vector<Point2f> points = feat.convVec6fToPoint2f(triangle);
  vector<Point2f> ex_list;

  // find center;
  double cx = 0.0, cy = 0.0;

  for(int i = 0;i < 3; i++){
    cx += (double)points[i].x;
    cy += (double)points[i].y;
  }
  
  cx /= 3.0;
  cy /= 3.0;

  for(int i = 0;i < 3;i++){
    Point2f ex_p;
    double dx = (double)points[i].x - cx;
    double dy = (double)points[i].y - cy;
    double len = sqrt(dx * dx + dy + dy);

    ex_p.x = points[i].x + (float)(dx/len)*(float)(width);
    ex_p.y = points[i].y + (float)(dy/len)*(float)(width);
    
    ex_list.push_back(ex_p);
  }

  Vec6f ex_p6f = feat.convPoint2fToVec6f(ex_list);
}

