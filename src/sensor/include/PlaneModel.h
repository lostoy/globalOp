#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>

#include <sys/types.h>

#include <errno.h>
#include <string>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>


#include <boost/filesystem.hpp>
#include <boost/algorithm/string/predicate.hpp>


// plane model for a plane in the point cloud
class PlaneModel
{
public:
	PlaneModel();

	PlaneModel(pcl::ModelCoefficients::Ptr coefficients, pcl::PointIndices::Ptr inliers);

	~PlaneModel();

	//the coefficient for a plane: Ax + By + Cz + D = 0
	pcl::ModelCoefficients::Ptr coefficients;
	//the indices of point in the plane, through which the number of the points is obtained.
	pcl::PointIndices::Ptr inliers;

	//the distance to another plane, decided by the numbers of the two planes
	float distanceToByPointCount(const PlaneModel & other) const;

	//the distance to another plane
	float distanceTo(const PlaneModel & other) const;

	//decided by the numbers of the two plane
	bool operator< (const PlaneModel & other) const;

	//decided by the coefficients and the number of points 
	bool operator== (const PlaneModel & other) const;

	//hash value of this plane
	operator size_t() const;

	//set the coefficients to positive if all the coefficients are negative.
	void normalize();

	//to string method
	std::string toString()
	{
		std::ostringstream oss;

		for each (float f in this->coefficients->values)
		{
			oss << f << ",";
		}
		oss << this->inliers->indices.size();

		return oss.str();
	}

	//get the rotation matrix through which this plane is rotated to another
	Eigen::Matrix4f rotationTo( PlaneModel & other);

	//get the norm vector of the plane
	Eigen::Vector3f normalVec();

	static bool comparePlaneByPointCount(PlaneModel & plane1, PlaneModel & plane2);

	//decide which side this plane is, 0 for floor, 1 for left wall, 2 for ceiling, 3 for right wall
	//when viewing along with the z-positive axis of the kinect
	static int whichSide(PlaneModel & plane);

	//get an intersection point of two planes, given a specific x
	static Eigen::Vector3f intersectionPoint(const PlaneModel & plane1, const PlaneModel & plane2, float x);

};

