#pragma once

#include <vector>

#include <pcl/Vertices.h>
#include <pcl/common/common.h>
#include <pcl/surface/convex_hull.h>



#include "PlaneModel.h"

class PlaneModelSegmentation
{
public:
	PlaneModelSegmentation();
	~PlaneModelSegmentation();

	//find the planes of the input cloud 
	//set downsample to true to down-sample the input cloud, which is used to accelerate the process of 
	//plane segmentation
	//parameters:
	//planes: the retured planes
	static void planeSegmentation(	const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr & cloud, 
									std::vector<PlaneModel> & planes,
									bool downsample = true);

	//find the planes of the input cloud 
	//set downsample to true to down-sample the input cloud, which is used to accelerate the process of 
	//plane segmentation
	//parameters:
	//cloud_f:  the remained part of the cloud after extractiong the planes
	//concaveHulls: the concave hulls consisting of boundary points
	//topPlanesCount: how much planes would be extraced from the input cloud
	//noiseFilter: indicates if remove the nan points from the input cloud or not.
	//downsample: indicates if down-sample the input cloud
	static void planeSegmentation(	const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr & cloud,
									pcl::PointCloud<pcl::PointXYZRGBA>::Ptr & cloud_f,
									std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> & concaveHulls,
									size_t topPlanesCount = 3,
									bool noiseFilter = true, 
									bool downsample = true
									);
	//find the planes of the input cloud 
	//set downsample to true to down-sample the input cloud, which is used to accelerate the process of 
	//plane segmentation
	//parameters:
	//cloud_f:  the remained part of the cloud after extractiong the planes
	//concaveHulls: the concave hulls consisting of boundary points
	//polygons: the indices of the points which consist of the boundary of the concave hulls
	//planePointCloud: the point clouds of the planes
	//topPlanesCount: how much planes would be extraced from the input cloud
	//noiseFilter: indicates if remove the nan points from the input cloud or not.
	//downsample: indicates if down-sample the input cloud
	static void planeSegmentation(	const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr & cloud,
									pcl::PointCloud<pcl::PointXYZRGBA>::Ptr & cloud_f,
									std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> & concaveHulls,
									std::vector<std::vector<pcl::Vertices>> & polygons,
									std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> & planePointCloud,
									size_t topPlanesCount = 3,
									bool noiseFilter = true,
									bool downsample = true
									);
	//find the planes of the input cloud 
	//set downsample to true to down-sample the input cloud, which is used to accelerate the process of 
	//plane segmentation
	//parameters:
	//cloud_f:  the remained part of the cloud after extractiong the planes
	//concaveHulls: the concave hulls consisting of boundary points
	//polygons: the indices of the points which consist of the boundary of the concave hulls
	//planes: the plane models of the extracted planes.
	//topPlanesCount: how much planes would be extraced from the input cloud
	//noiseFilter: indicates if remove the nan points from the input cloud or not.
	//downsample: indicates if down-sample the input cloud
	static void planeSegmentation(	const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr & cloud,
									pcl::PointCloud<pcl::PointXYZRGBA>::Ptr & cloud_f,
									std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> & concaveHulls,
									std::vector<std::vector<pcl::Vertices>> & polygons,
									std::vector<PlaneModel> & planes,
									size_t topPlanesCount = 3,
									bool noiseFilter = true,
									bool downsample = true
									);

	//return the norm vector of the two walls in the input cloud
	//parameters:
	//wall1: 0, 1, 2, 3
	//wall2: 0, 1, 2, 3
	static Eigen::Vector3f normVectorOfWallCorner(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr & cloud, size_t wall1, size_t wall2);

};

