#pragma once

#include <string>
#include <vector>
#include <fstream>

#include <boost/filesystem.hpp>

#include <pcl/registration/icp.h>
#include <pcl/pcl_macros.h>

//some tools usually needed
namespace utility
{
	//load the file to the byte buffer
	//parameters:
	//fileLoc: the file to read
	//buffer: the space used to store the bytes of the input file
	//length: the number of bytes in the input file
	void readAllBytes(std::string fileLoc, char * buffer, size_t & length);

	//list all the files in the root folder with the specific extension
	std::vector<boost::filesystem::path> listFiles(const boost::filesystem::path & root, const std::string & ext);

	//see if the float is invalid or not
	bool isinf(float f);

	//degree to radian
	float deg2rad(float degrees);

	//the cross product of two quaternion
	Eigen::Quaternionf quaternionProduct(Eigen::Quaternionf a, Eigen::Quaternionf b);

	//extend the rotation matrix to a transform matrix with the translation fields set to be zeros
	Eigen::Matrix4f rm2tm(const Eigen::Matrix3f & rm);
	 
	//filter out the invalid points in a point cloud
	//parameters:
	//cloud_in: input cloud
	//cloud_out: the version of input cloud without invalid points
	//index: the indices of the invalid points
	void removeNaNFromPointCloud(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud_in, 
								pcl::PointCloud<pcl::PointXYZRGBA> &cloud_out,
								std::vector<int> &index);

	//load the transform matrix 
	bool readTransformMatrix(std::string matrixfile, Eigen::Matrix4f &transform);

	//load points: Sheng Shen's output file
	std::vector<pcl::PointXYZRGBA> loadPoints(std::string fileLoc);

	//the eculidean distance of p1 and p2
	float distance(const pcl::PointXYZRGBA & p1, const pcl::PointXYZRGBA & p2);

	//see if the p is inside a cylinder
	//parameters:
	//p1: the center of the top circle of the cylinder
	//p2: the center of the bottom circle of the cylinder
	//p: the input point
	//r: the radius of the circles of the cylinder
	bool insideCylinder(const pcl::PointXYZRGBA & p1, const pcl::PointXYZRGBA & p2, const pcl::PointXYZRGBA & p, float r);



}

