/*
* @author: ELay Maiga
* This class contains all the methods related to calculating coordinates and making projections
* from 2D to 3D or vice-versa.
*/



#include"PclManip.hpp"
//#include "boost_headers.hpp"

/***************************************************Simple Functions **************************************/
//Function returns the index of a given point in an array, if not found it gives the size of the 
//Vector
int findPointInVector(PointXYZRGB p, vector<PointXYZRGB> points){

	int pos = 0;
	while(pos<points.size() && 
	((points[pos].x != p.x) ||(points[pos].y != p.y) || (points[pos].z != p.z))){
		pos++;
	} 
	
	return pos;



}


/**
* Simple function to convert a list of PointXYZRGB to a list of Point3D
*/
vector<cv::Point3d> fromxyzrgbtoPoint3D(vector<PointXYZRGB> points){
	vector<cv::Point3d> points3D;
	
	for(int i=0;i<points.size();i++){
		Point3d p (points[i].x,points[i].y,points[i].z);
		points3D.push_back(p);
	}

	return points3D;
}
/***************************************************Image Manipulation PCL***********************************************/


/**
* This function computes the keypoints on a cloud of PointXYZRGB. 
*******************FLANN compatibility problem between OpenCv and PCL ***************************
*/

void get3DKepoints(PointCloud<PointXYZRGB>::Ptr &points, float min_scale, int nr_octaves, int nr_scales_per_octave, float min_contrast, PointCloud<PointWithScale>::Ptr &keypoints_out) {
	cout << "get3DKeypoints called with cloud: " << points->size() << endl;

	SIFTKeypoint<PointXYZRGB, PointWithScale> sift_detect;

	// Use a FLANN-based KdTree to perform neighborhood searches
	search::KdTree<PointXYZRGB>::Ptr tree(new search::KdTree<PointXYZRGB> ());
	sift_detect.setSearchMethod(tree);
//	
//	// Set the detection parameters
	sift_detect.setScales(min_scale, nr_octaves, nr_scales_per_octave);
	sift_detect.setMinimumContrast(min_contrast);
//	
//	// Set the input
	sift_detect.setInputCloud(points);
//	
//	// Detect the keypoints and store them in "keypoints_out"
	//sift_detect.compute(*keypoints_out);
	
	cout << "Number of keypoints found: " <<endl;
	//return keypoints_out;
}

/**
*This function takes an input image in Equirectangular pixel coordinates (i,j) cv::Mat and returns 
* a Point cloud. The point cloud is the Spherical projection of this image onto a sphere of
* radius r and centered in (xc,zc);
* @Inputs
* Cv::cv::Mat ori: the input image
* double r : the radius of the sphere
* double xc and zc: the (x,z) coordinates of the center of the Sphere
*
*
* @Output
* Ptr cloud : a point cloud of the sphere
* Ptr VctCloud: A vector cloud of vectors (Sc,Pt) with Sc being the center of the sphere.
*/
PointCloud<PointXYZRGB>::Ptr EquiToSphere(cv::Mat ori, double radius, double xc, double yc, double zc){
	//We define our point cloud and a Point
	PointCloud<PointXYZRGB>::Ptr cloud (new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr vectorCloud (new PointCloud<PointXYZRGB>);
	
	double alpha_rad = 15*PI/180;
	PointXYZRGB iPoint;
	PointXYZRGB iVect;
	PointXYZRGB t;
	t.x = xc;
	t.y = yc;
	t.z = zc;
	double normt = norm(t);
	
	//Math variables
	double theta,phi,x,y,z;
	//double radius = r;
	
	//Receive the size 
	cv::Size s = ori.size();
	int rows = s.height;
	int cols = s.width;
	
	//Color vector 
	cv::Vec3b colorOri;
	
	//For each pixel value we calculate it's new cooridnates and assign the pixel value to those
	//coordinates
	for (int i = 0; i < s.height; i++)
	{
		//j corresponds to the x and hence runs through the columns = width
		for (int j = 0; j < s.width; j++)
		{
			//Get the Spherical Coordinates on a sphere or radius centered in (0,0)
			
			sphereCoordinates(i,j,radius,rows,cols,x,y,z);
			
			//Put the coordinates in a PointXYZ and shift center to (xc,zc,0)
			
			
//			
//			iPoint.x = x + xc;
//			iPoint.y = y + zc;
			//Knowing the Center, apply rotation and translation from world Center
			// The real life and PCL y and z are switched
			if(normt!=0){
				iPoint.x = ((x*xc/normt)-(z*zc/normt));// - xc;
				iPoint.z = -((x*zc/normt) + (z*xc/normt));// - zc;
			}
			else{
				iPoint.x = x;// - xc;
				iPoint.z = z;
			}
			//test pour 4
//			iPoint.x = ((x*cos(alpha_rad))-(y*sin(alpha_rad)));
//			iPoint.y = ((x*sin(alpha_rad)) + (y*cos(alpha_rad))) ;
			
//			if(rand()%10000<1){
//				cout << "xp: " << xp << endl;
//			}
			iPoint.y = y;
				
				
			//Put the coordinates of the vector directed from iPoint to Sc(xc,zc,0)
//			iVect.x = x;
//			iVect.y = y;
//			iVect.z = z;
				
			//Add pixel color values
			colorOri = ori.at<cv::Vec3b>(i, j);
		       	iPoint.b = iVect.b = colorOri[0];
			iPoint.g = iVect.g = colorOri[1];
			iPoint.r = iVect.r = colorOri[2];			
			cloud->points.push_back(iPoint);
			//cloud->points+=iPoint;	
//			vectorCloud->points.push_back(iVect);		       						
		}
		
	}
	
	cloud->width = cols;
	cloud->height = rows;
	return cloud;
}


/**
* This function is the inverse of the previous function
*/
void sphereToEqui(PointCloud<PointXYZRGB>::Ptr sphere, double r, int rows, int cols, cv::Mat &image){
	//image.resize(rows,cols);
	PointXYZRGB p;
//	PointXYZRGB n;
	
	
	int i,j;
//	for(int m=0;m<sphere->size();m++){
//		p = sphere->points[m];
//		pixelCoordinates(p.x,p.z,p.y,r,rows,cols,i,j);
//		//cout << "(i,j):" << i << "," << j << endl;
//		if(i<rows && j<cols){
////			n = p;
////			n.x = i;
////			n.z = j;
////			n.y = 0;
//			//cout << "new point: " << n << endl;
//			//result->points.push_back(n);
//			cv::Vec3b color = image.at<cv::Vec3b>(cv::Point(i,j));
//			image.at<cv::Vec3b>(i,j)[0] = p.b;
//			image.at<cv::Vec3b>(i,j)[1] = p.g;
//			image.at<cv::Vec3b>(i,j)[2] = p.r;
//		}
//	}

	for (i=0;i<rows;i++){
		for(j=0;j<cols;j++){
			//cv::Vec3b color = image.at<cv::Vec3b>(cv::Point(i,j));
			image.at<cv::Vec3b>(i,j)[0] = p.b;
			image.at<cv::Vec3b>(i,j)[1] = p.g;
			image.at<cv::Vec3b>(i,j)[2] = p.r;
		}
	}
}

/**
*This function takes an input a PointCloud of points on a Sphere (not necessarily the full sphere) and returns a PointCloud of 
*those points with x= i and y = j being the pixel coordinates. It also takes the radius and original input image
* 
*/
 void sphere2EquiCloud(cv::Mat ori, double r,PointCloud<PointXYZRGB>::Ptr spherePts, PointCloud<PointXYZRGB>::Ptr &result){
	double rows = ori.rows;
	double cols = ori.cols;
	
	PointXYZRGB po,pi;
	int i,j;
	
	cv::Vec3b colorOri;
	
	
	for(int n=0;n<spherePts->size();n++){
		po = spherePts->points[n];
		pixelCoordinates(po.x,po.y,po.z,r,rows,cols,i,j);
		pi.x = i;
		pi.y =j;
		//cout << "pi: " << pi << endl;
		colorOri = ori.at<cv::Vec3b>(i, j);
		pi.r =  colorOri[2];	
		pi.g =  colorOri[1];
		pi.b =  colorOri[0];
		result->push_back(pi);
	}	
}
/**
*This function takes an input image in Equirectangular pixel coordinates (i,j) and returns 
* a Point cloud. The point cloud is the Spherical projection of this image onto a sphere of
* radius r and centered in (xc,yc);
* @Inputs
* Cv::cv::Mat ori: the input image
* double r : the radius of the sphere
*
* @Output
* Ptr cloud : a point cloud of the sphere
*/

/************************************************ Cloud Viewer Call back functions **************************************/
void hPointLine(PointXYZRGB o, PointXYZRGB u, vector<PointXYZRGB> &line)
{

	//line.push_back(o);
	//line.push_back(u);
//	cout << "hPointLine function called" << endl;
//	
	//vector<PointXYZRGB> line(nbPts);
	int nbPts = line.size();
	double distmin =0;
	double step = abs(u.x - o.x)/nbPts;
	double distmax = abs(u.x - o.x);
	//cout << dist << endl;
	
	PointXYZ v;
	v.x = (u.x - o.x)/nbPts ;
	v.y = (u.y - o.y)/nbPts ;
	//cout << "vy: "<< v.y << endl;
	v.z = (u.z - o.z)/nbPts ;
	
	PointXYZRGB p;
	p.x = o.x;
	p.y = o.y;
	p.z = o.z;
	p.r = u.r;
	p.g = u.g;
	p.b = u.b;
	
	int i =0;
	while(i<nbPts){
		//cout << "hPointLine function called: " << i << endl;
		line.push_back(p);
		//cout << p.x << "," <<  p.y << ","  << p.z << endl;
		p.x += v.x;
		p.y += v.y;
		//cout << "py: " << p.y << endl;
		p.z += v.z;
		
//		PointXYZRGB np;
//		np.x = p.x;
//		np.y = p.y;
//		np.z = p.z;
//		np.r = p.r;
//		np.g = p.g;
//		np.b = p.g;
			
		//line[i]  = p;
		i++;
		//cloud->points.push_back(p);
		distmin += step;
	}
	
	//return line;
}


/*
* Given a point u and a pointcloud cloud, this function returns the mean value of K nearest neighbors
*/

void kMeanValue(PointXYZRGB &u, PointCloud<PointXYZRGB>::Ptr &cloud, PointCloud<PointXYZRGB>::Ptr &outCloud, int K){
	//cout<< "cloud size in KmeanValue: " << cloud->size() << endl;
	double rt,gt,bt, xt, yt, zt;
	PointXYZRGB p;
	
	
	//PointCloud<PointXYZRGB>::Ptr sight (new PointCloud<PointXYZRGB>);
	//sight->points.resize(cloud->size());
//	for (size_t i = 0; i < cloud.points.size (); ++i)
//	{
//	    	//sight->points[i] = cloud->points[i];
//	    	sight->points.push_back(cloud.points[i]);
//	    	//cout << "cloud points numb: " << i << " coord: " << cloud->points[i] <<endl;
//	}
	
	
	//cloud->points.resize(cloud->width*cloud->height);
	//cloud->points.push_back(p);
	KdTreeFLANN<PointXYZRGB> kdtree;
	kdtree.setInputCloud (cloud);
	
	
	vector<int> pointIdxNKNSearch(K);
	vector<float> pointNKNSquaredDistance(K);
	if ( kdtree.nearestKSearch (u, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
		xt = yt = zt = 0;
		//cout << "pointIdxNKNSearch.size (): " << pointIdxNKNSearch.size () <<endl;
		for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i){
			p = cloud->points[ pointIdxNKNSearch[i] ];
			//cout << "pxb: " << p.x <<" pyb: " << p.y <<" pzb: " << p.z <<endl;
//	      		std::cout << "    "  <<   p.x 
//		        << " " << p.y 
//		        << " " << p.z 
//		        << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
		        
		        xt += p.x;
		        yt += p.y;
		        zt += p.z;
		        rt += p.r;
		        gt += p.g;
		        bt += p.b; 
	  	}	
	
	}
	
	int t = pointIdxNKNSearch.size();
	//cout << "size: " << t << endl; 
	//cout << "xt: " << xt <<" yt: " << yt <<" zt: " << zt <<endl;
	p.x = xt/t;
	p.y = yt/t;
	p.z = zt/t;
	p.r = rt/t;
	p.g = gt/t;
	p.b = bt/t;
	//cout << "px: " << p.x <<" py: " << p.y <<" pz: " << p.z <<endl;
	//cout << "--------------------------------------------------------" << endl;
	
	outCloud->points.push_back(p);
	return;
}


/**
* This function is intended to be used when multi-threading. 
* Given an Array of points, the id of the thread i , the total number of threads n, an input pointCloud and an Output pointCloud
* it first separates the array into n equal parts. Then it retrieves the KmeanValue for the part of the array that concerns the 
* thread in question.
* 
*/
void multiKMeanValue(vector<PointXYZRGB> points, int id, int nbThreads, PointCloud<PointXYZRGB>::Ptr cloud,PointCloud<PointXYZRGB>::Ptr &outCloud, int k ){

	int size = points.size();
	//cout<< "points size: "<< size << endl;
	int perT = (int)(size/nbThreads);
	//cout << "perT: " << perT << endl;
	int start = (id-1)*perT;
	//cout << "start: " << start << endl;
	int end;
	if(id<nbThreads){
		end = id*perT; 
	}
	else{
		end = size;
	}
	
	//cout<< "end: " << end << endl;
	for(int i = start; i<end;i++){
		cout << "i: " << i <<endl; 
		PointXYZRGB p = points[i];
		kMeanValue(p,cloud,outCloud,k);
	}
	
	return;
}


PointCloud<PointXYZRGB>::Ptr  sphereInterpolate(cv::Mat image1, cv::Mat image2, double d, double pos){
//	
	//Output point cloud
	PointCloud<PointXYZRGB>::Ptr match_cloud(new PointCloud<PointXYZRGB>); 
	//match_cloud->width = image1.cols;
	//match_cloud->height = image1.rows;
	
	vector<cv::KeyPoint> keypoints1;
	vector<cv::KeyPoint> keypoints2;
	vector<cv::DMatch> matches;
	
	
	//Compute Keypoints and matches 
	getKeypointsAndMatches(image1,image2,keypoints1,keypoints2,matches);
	
	
	//Convert images to pointClouds
	PointCloud<PointXYZRGB>::Ptr cloud1 (new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr cloud2 (new PointCloud<PointXYZRGB>);
	cloud1 = EquiToSphere(image1, 1,0,0,0);
	cloud2 = EquiToSphere(image2,1,0,0,0);
	
	PointXYZRGB p,p1,p2;
	//Iterate through Keypoints and interpolate position in new image for each keypoint
	for (std::vector<cv::DMatch>::const_iterator it= matches.begin();
			it!= matches.end(); ++it) {

//		// Get the position of left keypoints
		float x1= keypoints1[it->queryIdx].pt.x;
		float y1= keypoints1[it->queryIdx].pt.y;
		
		if(x1<cloud1->width & y1 < cloud1->height){
			p1 = cloud1->at(round(x1),round(y1));
			cout << "(x1,y1) in image1: " << p1 << endl;
		}
//		points1.push_back(cv::Point2f(x,y));
//		// Get the position of right keypoints
		float x2= keypoints2[it->trainIdx].pt.x;
		float y2= keypoints2[it->trainIdx].pt.y;
		if(x1<cloud1->width & y1 < cloud1->height){
			p2 = cloud2->at(round(x2),round(y2));
			cout << "(x2,y2) in image2: " << p2 << endl;
		}	
//		points2.push_back(cv::Point2f(x,y));
//		
		p.x = (1/d) * (p1.x* (d-pos) + p2.x*pos);
		p.y = (1/d) * (p1.y* (d-pos) + p2.y*pos);
		p.z = (1/d) * (p1.z* (d-pos) + p2.z*pos);
		p.r = (1/d) * (p1.r* (d-pos) + p2.r*pos);
		p.b = (1/d) * (p1.b* (d-pos) + p2.b*pos);
		p.g = (1/d) * (p1.g* (d-pos) + p2.g*pos);
		
		match_cloud->points.push_back(p);
		
	}

	return match_cloud;


}


cv::Mat imageFromPcPlane(PointCloud<PointXYZRGB>::Ptr cloud,cv::Mat ori, int rows, int cols)
{
	cv::Mat output(rows, cols, ori.type());
	PointXYZRGB p;
	int istart, ifinish, jstart, jfinish;
	
	istart = -rows/2;
	ifinish = rows/2;
	jstart = -cols/2;
	jfinish = cols/2;

	for (int m=0; m<cloud->size();m++){
		p= cloud->points[m];
		int j = m/cols;
		int i = m%cols;
		
		if(j<cols){
			output.at<Vec3b>(i,j)[0] = p.b;
			output.at<Vec3b>(i,j)[1] = p.g;
			output.at<Vec3b>(i,j)[2] = p.r;
		}
	}	
//	for(int i=istart; i<ifinish; i++){
//		int ci = i + rows/2;
//		for(int j=jstart; j<jfinish;j++){
//			int cj = j + cols/2;
//			output.at<cv::Vec3b>(i,j)[]
//		}
//	}
	return output;
	
	

}


/** 
* Given a set of 3D points, this functions creates a triangle mesh without any upsampling
*/
GreedyProjectionTriangulation<PointXYZRGBNormal> get3DTriangulation(PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<Normal>::Ptr normals, double max_dist){

	//First concatenate the points and the normals 
 	PointCloud<PointXYZRGBNormal>::Ptr cloud_with_normals (new PointCloud<PointXYZRGBNormal>);
  	pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);

	 // Create search tree*
  	pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
  	tree2->setInputCloud (cloud_with_normals);
  	
  	// Initialize objects
  	pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;
  	pcl::PolygonMesh triangles;
  	
  	// Set the maximum distance between connected points (maximum edge length)
  	gp3.setSearchRadius (max_dist);
  	
  	 // Set typical values for the parameters
  	gp3.setMu (2.5);
  	gp3.setMaximumNearestNeighbors (100);
  	gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  	gp3.setMinimumAngle(M_PI/18); // 10 degrees
  	gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  	gp3.setNormalConsistency(false);
  	
  	 // Get result
  	gp3.setInputCloud (cloud_with_normals);
  	gp3.setSearchMethod (tree2);
  	gp3.reconstruct (triangles);
  	
  	
  	//Return final result
  	return gp3;

}






//void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void){
//  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
//  if (event.getKeySym () == "n" && event.keyDown ())
//  {
//    std::cout << "n was pressed => removing all text" << std::endl;

////    char str[512];
////    for (unsigned int i = 0; i < text_id; ++i)
////    {
////      sprintf (str, "text#%03d", i);
////      viewer->removeShape(str);
////    }
////    text_id = 0;
//  }
//}

//void testfunction(){
//	cout << "test function called" << endl;
//	return;
//}

void makeCorrespondingDelaunayTriangles3D(vector<PointXYZRGB> points3D1, vector<PointXYZRGB> points3D2, vector<Vec9f> &triangles3D1, vector<Vec9f> &triangles3D2){
	
	vector<Vec9f> newtriangles3D1,newtriangles3D2;
	Vec9f t,t2;
	//logFile.open("TrianglePoints.txt");
	PointXYZRGB p1,p2,p3,pp1,pp2,pp3;
	for(int i=0;i<triangles3D1.size();i++){
		t = triangles3D1[i];
		p1 = PointXYZRGB(t[0], t[1], t[2]);
		p2 = PointXYZRGB(t[3], t[4], t[5]);
		p3 = PointXYZRGB(t[6], t[7], t[8]);

		//logFile << p1 << "||" << p2 << "||" << p3 << endl;
		//For each point in triangle 1 we search for the corresponding point in triangle 2
		int ptPos1 = findPointInVector(p1, points3D1) ; 
		int ptPos2 = findPointInVector(p2, points3D1) ; 		
		int ptPos3 = findPointInVector(p3, points3D1) ; 



		if(ptPos1 < points3D1.size() && ptPos2 < points3D1.size() && ptPos3 < points3D1.size()){	
			//Construct the second triangle and try to locate it in the trianglesList2
			//We find the corresponding point in the second triangles list
			pp1 = points3D2[ptPos1];
			pp2 = points3D2[ptPos2];
			pp3 = points3D2[ptPos3];
			t2[0] = pp1.x;
			t2[1] = pp1.y;
			t2[2] = pp1.z;
			
			t2[3] = pp2.x;
			t2[4] = pp2.y;
			t2[5] = pp2.z;
			
			t2[6] = pp3.x;
			t2[7] = pp3.y;
			t2[8] = pp3.z;			
			newtriangles3D1.push_back(t);
			newtriangles3D2.push_back(t2);
		}
	}

	//logFile.close();
	triangles3D1 = newtriangles3D1;
	triangles3D2 = newtriangles3D2;


}



