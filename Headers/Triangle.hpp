/*
 * Triangle handling
 *   convert it from equirectangular to the perspective image
 */
#ifndef _TRIANGLE_HPP
#define _TRINANGLE_HPP
#include "cv_headers.hpp"
#include "ViewDirection.hpp"
#include "PersCamera.hpp"
#include "EquiTrans.hpp"
//#include "MathCalcs.hpp"

///using namespace cv;
const double triangle_max_angle = 45.0/180.0 * M_PI;

class Triangle {

	private:
		double m_MAX_ANGLE;

		double findMin(float numbers[], int size);
		double findMax(float numbers[], int size);

		void cartesian2SphericSacht(Point3d p, double r, double &theta, double &phi);
		cv::Vec6f triangle2SphericSacht(int rows, int cols, double r, Vec9f triangle);
		cv::Point2f triangleCenter(cv::Vec6f triangle);
		PersCamera adjustPersCam(Mat equi_image, PersCamera cam, vector<Point3d> vertices);
		cv::Point3d midPoint(cv::Point3d a, cv::Point3d b);
		double getMaxAngle(vector<cv::Point3d> vertices);
		double getMaxAngle(Vec9f triangle);
		double nPointsMaxAngle(vector<Point3d> vertices);
	public:
		Triangle();
		/*
		 * convert an image in equirectangular to the perspective image, 
		 *  which includes the triangle      
		 *  equi_image: the ominidirectinal image in equirectangular format
		 *  vertices:   the three vertices of the triangle in equi_image
		 *              (origin: top-left)
		 *               x: panning angle, y: tilting angle (in pixel)
		 *  return:     the perpective image that includes the triagle.
		 */
		cv::Mat convToPersRectImage(cv::Mat equi_image, cv::Vec6f vertices);

		/**
		 * Same as above function but using 2 Triangles
		 */
		cv::Mat convToPersRectImageTwo(cv::Mat equi_image, cv::Vec6f vertices1, cv::Vec6f vertices2);

		/**
		 * Function to generate perspective view from triangle defined by its 3 coordinates in Euclidien space and
		 *   cartesian coordinates
		 * @Input:
		 * - Equi_image: image in equirectangular format (W = 2*H)
		 * - vertices3D: triangle defined by its 3 vertices in (x,y,z) format
		 */
		cv::Mat convToPersRectImageBarycentric(cv::Mat equi_image, Vec9f vertices3D);


		/**
		 * Function to generate perspective view from triangle defined by its 3 coordinates in Euclidien space and
		 *   cartesian coordinates
		 * @Input:
		 * - Equi_image: image in equirectangular format (W = 2*H)
		 * - triangle3D1, triangle3D2: 2 triangles given by their parameters in 3D 
		 */
		cv::Mat convToPersRectImageBarycentricTwo(cv::Mat equi_image, Vec9f triangle3D1, Vec9f triangle3D2);

		/*
		 * Covnert a trinagle in Point3d list to Vec9f
		 */
		Vec9f convPoint3dToVec9f(vector<cv::Point3d> points);

		/*
		 * Get the perpective camera that includes the list of triangles.
		 *     Triangles could be one or a pair of matching
		 */
		PersCamera getPersCamParams(cv::Mat equi_image, cv::Vec6f vertices);

		/* Convert a triangle in Vec9f to Point3d format. */
		vector<Point3d> convVec9fToPoint3d(Vec9f vertices);


		/*
		 * Convert 3D points on the unit sphere in Vec9f to those in equirectanuglar
		 * in Vec6f.
		 */ 
		Vec6f convVec9fToVec6f(Mat equi_image, Vec9f vec9f);

		/* Obtain PersCamera FOV and adjust the viewing direction */
		ViewDirection getFOV(vector<Point3d> points, ViewDirection vd, Point3d center, double &h_fov, double &v_fov);

		/**
		 * 
		 */
		void nPointsFOV(vector<Point3d> points, ViewDirection vd, Point3d center, double &h_fov, double &v_fov);
		
		/* in 3D on the unit sphere */
		PersCamera getPersCamParams(Mat equi_image, Vec9f vertices);


		/**
		 * Same perspective camera containing both triangles
		 */
		PersCamera getPersCamParamsTwo(Mat equi_image, Vec9f vertices1, Vec9f vertices2);

		/** 
		 * This function returns the intermediate camera parameters given 
		 * - 2 perspective cameras
		 * - the desired interpolated position defined by pos and dist = pos/dist 
		 */
		PersCamera getInterpolatedPersCamParams( PersCamera cam1, PersCamera cam2, double dist, double pos);

		/*
		 * Get the perpective camera that includes a triangle by using the barycenter as point and the 
		 * radius of the circumcircle.
		 * @Input
		 * - vertices: the 3 vertices of the triangle given in 3D cartesian coordinates
		 */
		PersCamera getPersCamParamsBarycentric(cv::Mat equi_image, Vec9f vertices);


		/*
		 * Get the perpective camera that includes both triangles by using the barycenter as point and the 
		 * radius of the circumcircle.
		 * @Input
		 * - vertices: the 3 vertices of the triangle given in 3D cartesian coordinates
		 */
		PersCamera getPersCamParamsBarycentricTwo(cv::Mat equi_image, Vec9f triangle3D1, Vec9f triangle3D2);

		/** 
		 * Same as above but using 2 triangles 
		 */
		PersCamera getPersCamParamsTwo(cv::Mat equi_image, cv::Vec6f vertices1, cv::Vec6f vertices2 );

		/*
		 * Convert the image coordinates of a triangle from equirectangular to perspective
		 */
		cv::Vec6f convToPersTriangle(cv::Mat equi_image, PersCamera pers_cam, cv::Vec6f vertices);

		/**
		 * convert a triangle from coordinates given in 3D cartesian to spheric angular coordinates
		 */
		cv::Vec6f convToSphericTriangle( Vec9f triangle);

		/*
		 * convert 6 element floating type to a set of points.
		 */
		vector<cv::Point2f> convToPointVector(Vec6f vec);


		/*
		 * Subdivide one triangle to four triangles when it has angles to the center are larger than 45.0 dereees.
		 */
		vector<Vec9f> subdivide(vector<Vec9f> triangle_list);
		vector<Vec9f> subdivide(Vec9f vertices);
		vector< vector<cv::Point3d> > subdivide(vector<cv::Point3d> vertices);

		/**
		 * Function that subdivides a triangle into 4 no matter the angles
		 */
		vector<Vec9f> forceSubdivide(Vec9f triangle);

		/**
		 * Given 2 lists of matched triangles, subdivides the triangles that are too big to make smaller 
		 * matched triangles
		 */
		void subdivideMatched(vector<Vec9f> &triangle_list1, vector<Vec9f> &triangle_list2);

		/*
		 * Draw single triangle
		 */
		void drawTriangle(Mat image, cv::Vec6f vertices);
		void drawTriangle(cv::Mat image, vector<cv::Point2f> point_vector);

		/*
		 * Show single triangle
		 */
		void showTriangle(cv::Mat image, Vec6f vec); 
		void showTriangle(cv::Mat image, vector<Point2f> point_vector); 

		/*
		 * Get the center of a triangle in 3D.
		 */
		cv::Point3d triangleCenter3d(vector<cv::Point3d> vertices);

		/**
		 * Get barrycenter for n points in 3d space
		 */
		cv::Point3d nPointsCenter3d(vector<cv::Point3d> vertices);

};
#endif

