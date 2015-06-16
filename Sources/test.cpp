#include "standard_headers.hpp"
#include "cv_headers.hpp"
#include "pcl_headers.hpp"

int
main (int argc, char *argv[]) {

	PointCloud<PointXYZRGB>::Ptr cloud (new PointCloud<PointXYZRGB>);

	// Visualization
	visualization::CloudViewer viewer("View Lines");

 
	//viewer.addPointCloud (cloud, "cloud"); // Method #1

	PointXYZRGB o;
	o.x = 0;
	o.y = 0;
	o.z = 0;
	
	PointXYZRGB u;
	u.x = 10;
	u.y = 5;
	u.z = 4;
	u.r = 10;
	u.g =10;
	u.b = 10;

	cloud->points.push_back(u);
	cloud->points.push_back(o);
	
	double distmin =0;
	double step = abs(u.x - o.x)/100;
	double distmax = abs(u.x - o.x);
	//cout << dist << endl;
	
	PointXYZ v;
	v.x = (u.x - o.x)/100 ;
	v.y = (u.y - o.y)/100 ;
	v.z = (u.z - o.z)/100 ;
	
	PointXYZRGB p;
	p.x = o.x;
	p.y = o.y;
	p.z = o.z;
	
	cout << o.x << endl;
	while(distmin < distmax){
		p.x += v.x;
		p.y += v.y;
		p.z += v.z;
		cout << "(" << p.x << "," << p.y << "," << p.z << ")" << endl;
		p.r = 255;
		p.g = 255;
		p.b = 255;
		cloud->points.push_back(p);
		distmin += step;
	}
	
	cloud->width=50;
	cloud->height=50;
	viewer.showCloud(cloud);
	while(!viewer.wasStopped()){
	
	}
	
	return (0);
}

