// Point cloud library

#ifndef PCL_HEADERS_H
#define PCL_HEADERS_H
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/visualization/keyboard_event.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/keypoints/sift_keypoint.h>

using namespace pcl;


/**Structure to define a triangle by:
* - It's 3 Vertices given in euclidien x,y,z coordinates
* - The list of points inside the triangle with their color
*/

struct Triangle3D {
	cv::Vec6f points;
	std::vector<PointXYZRGB> content;
};

#endif
