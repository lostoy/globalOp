#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>

#include "PlaneModelSegmentation.h"
#include "utility.h"


PlaneModelSegmentation::PlaneModelSegmentation()
{
}


PlaneModelSegmentation::~PlaneModelSegmentation()
{
}


void PlaneModelSegmentation::planeSegmentation(	const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr & cloud, 
												std::vector<PlaneModel> & planes,
												bool downsample)
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>),
											cloud_p(new pcl::PointCloud<pcl::PointXYZRGBA>),
											cloud_f(new pcl::PointCloud<pcl::PointXYZRGBA>),
											cloud_c(new pcl::PointCloud<pcl::PointXYZRGBA>);
	//make sure that the input cloud is clear by filtering out the invalid points
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*cloud, *cloud_c, indices);

	//voxell grid used to down-sample the input cloud
	/*optional filter opeartion */
	pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
	sor.setInputCloud(cloud_c);
	sor.setLeafSize(0.01f, 0.01f, 0.01f);
	sor.filter(*cloud_filtered);

	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
	//// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(1000);
	seg.setDistanceThreshold(0.05);

	// Create the filtering object
	pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
	size_t i = 0, nr_points = (int)cloud_filtered->points.size();

	//extrace the first 7 planes with bigger areas if possible.
	while (i < 4)
	{
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud(cloud_filtered);
		seg.segment(*inliers, *coefficients);

		if (inliers->indices.size() == 0)
		{
			std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}

		//store the extracted plane
		planes.push_back(PlaneModel(coefficients, inliers));

		//extract the points in this plane from the point cloud.
		extract.setInputCloud(cloud_filtered);
		extract.setIndices(inliers);
		extract.setNegative(true);
		extract.filter(*cloud_f);
		cloud_filtered.swap(cloud_f);
		i++;
	}

	//sort the planes by the points in them descendingly.
	std::sort(planes.begin(), planes.end(), PlaneModel::comparePlaneByPointCount);

	//filter out the planes whose number of points is less than 400
	for (i = 0; i < planes.size(); i++)
	{
		if (planes.at(i).inliers->indices.size() < 400)
		{
			break;
		}
	}
	planes.erase(planes.begin()+i, planes.end());
}


void PlaneModelSegmentation::planeSegmentation(	const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr & cloud,
												pcl::PointCloud<pcl::PointXYZRGBA>::Ptr & cloud_f,
												std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> & concaveHulls,
												size_t topPlanesCount,
												bool noiseFilter,
												bool downsample)
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudInput = cloud;
	//make sure that the input cloud is clear by filtering out the invalid points
	if (noiseFilter)
	{
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>);
		
		pcl::removeNaNFromPointCloud(*cloudInput, *cloud_filtered, std::vector<int>());

		cloudInput = cloud_filtered;
	}

	//voxell grid used to down-sample the input cloud
	/*optional filter opeartion */
	if (downsample)
	{
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_downsampleed(new pcl::PointCloud<pcl::PointXYZRGBA>);

		pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
		sor.setInputCloud(cloudInput);
		sor.setLeafSize(0.01f, 0.01f, 0.01f);
		sor.filter(*cloud_downsampleed);

		cloudInput = cloud_downsampleed;
	}


	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
	//// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(1000);
	seg.setDistanceThreshold(0.05);

	// Create the filtering object
	pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
	size_t i = 0;

	while (i <= topPlanesCount)
	{
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud(cloudInput);
		seg.segment(*inliers, *coefficients);

		if (inliers->indices.size() == 0)
		{
			std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}

		// Project the model inliers
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::ProjectInliers<pcl::PointXYZRGBA> proj;
		proj.setModelType(pcl::SACMODEL_PLANE);
		proj.setIndices(inliers);
		proj.setInputCloud(cloudInput);
		proj.setModelCoefficients(coefficients);
		proj.filter(*cloud_projected);
		std::cerr << "PointCloud after projection has: "
			<< cloud_projected->points.size() << " data points." << std::endl;

		// Create a Concave Hull representation of the projected inliers
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::ConcaveHull<pcl::PointXYZRGBA> chull;
		chull.setInputCloud(cloud_projected);
		chull.setDimension(2);
		chull.setAlpha(0.1);
		chull.reconstruct(*cloud_hull);

		concaveHulls.push_back(cloud_hull);

		extract.setInputCloud(cloudInput);
		extract.setIndices(inliers);
		extract.setNegative(true);
		extract.filter(*cloud_f);
		
		cloudInput = cloud_f;

		i++;
	}

}



void PlaneModelSegmentation::planeSegmentation(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr & cloud,
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr & cloud_f,
	std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> & concaveHulls,
	std::vector<std::vector<pcl::Vertices>> & polygons,
	std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> & planePointCloud,
	size_t topPlanesCount,
	bool noiseFilter,
	bool downsample
	)
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudInput = cloud;
	//make sure that the input cloud is clear by filtering out the invalid points
	if (noiseFilter)
	{
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>);

		pcl::removeNaNFromPointCloud(*cloudInput, *cloud_filtered, std::vector<int>());

		cloudInput = cloud_filtered;
	}

	//voxell grid used to down-sample the input cloud
	/*optional filter opeartion */
	if (downsample)
	{
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_downsampleed(new pcl::PointCloud<pcl::PointXYZRGBA>);

		pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
		sor.setInputCloud(cloudInput);
		sor.setLeafSize(0.1f, 0.1f, 0.1f);
		sor.filter(*cloud_downsampleed);

		cloudInput = cloud_downsampleed;
	}


	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
	//// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(1000);
	seg.setDistanceThreshold(0.45);

	// Create the filtering object
	pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
	size_t i = 0;

	while (i <= topPlanesCount)
	{
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud(cloudInput);
		seg.segment(*inliers, *coefficients);

		if (inliers->indices.size() == 0)
		{
			std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}

		// Project the model inliers
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::ProjectInliers<pcl::PointXYZRGBA> proj;
		proj.setModelType(pcl::SACMODEL_PLANE);
		proj.setIndices(inliers);
		proj.setInputCloud(cloudInput);
		proj.setModelCoefficients(coefficients);
		proj.filter(*cloud_projected);
		std::cerr << "PointCloud after projection has: "
			<< cloud_projected->points.size() << " data points." << std::endl;

		// Create a Concave Hull representation of the projected inliers
		std::vector<pcl::Vertices> plygs;

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::ConcaveHull<pcl::PointXYZRGBA> chull;
		chull.setInputCloud(cloud_projected);
		chull.setDimension(2);
		chull.setAlpha(0.1);
		chull.reconstruct(*cloud_hull, plygs);


		//plane point cloud
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planeCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
		planeCloud->resize(inliers->indices.size());
		for (size_t k = 0; k < inliers->indices.size(); k++)
		{
			planeCloud->points[k] = cloudInput->points[inliers->indices.at(k)];
		}


		concaveHulls.push_back(cloud_hull);
		polygons.push_back(plygs);
		planePointCloud.push_back(cloud_projected);

		extract.setInputCloud(cloudInput);
		extract.setIndices(inliers);
		extract.setNegative(true);
		extract.filter(*cloud_f);

		cloudInput = cloud_f;

		i++;
	}

}



	void PlaneModelSegmentation::planeSegmentation(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr & cloud,
						pcl::PointCloud<pcl::PointXYZRGBA>::Ptr & cloud_f,
						std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> & concaveHulls,
						std::vector<std::vector<pcl::Vertices>> & polygons,
						std::vector<PlaneModel> & planes,
						size_t topPlanesCount,
						bool noiseFilter,
						bool downsample
						)
{

	cloud->is_dense = false;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudInput = cloud;
	//make sure that the input cloud is clear by filtering out the invalid points
	if (noiseFilter)
	{
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>);

		pcl::removeNaNFromPointCloud(*cloudInput, *cloud_filtered, std::vector<int>());

		cloudInput = cloud_filtered;
	}

	//voxell grid used to down-sample the input cloud
	/*optional filter opeartion */
	if (downsample)
	{
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_downsampleed(new pcl::PointCloud<pcl::PointXYZRGBA>);

		pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
		sor.setInputCloud(cloudInput);
		sor.setLeafSize(0.01f, 0.01f, 0.01f);
		sor.filter(*cloud_downsampleed);

		cloudInput = cloud_downsampleed;
	}


	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
	//// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(1000);
	seg.setDistanceThreshold(0.05);

	// Create the filtering object
	pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
	size_t i = 0;

	while (i <= topPlanesCount)
	{
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud(cloudInput);
		seg.segment(*inliers, *coefficients);

		if (inliers->indices.size() == 0)
		{
			std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}

		// Project the model inliers
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::ProjectInliers<pcl::PointXYZRGBA> proj;
		proj.setModelType(pcl::SACMODEL_PLANE);
		proj.setIndices(inliers);
		proj.setInputCloud(cloudInput);
		proj.setModelCoefficients(coefficients);
		proj.filter(*cloud_projected);
		std::cerr << "PointCloud after projection has: "
			<< cloud_projected->points.size() << " data points." << std::endl;

		// Create a Concave Hull representation of the projected inliers
		std::vector<pcl::Vertices> plygs;

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::ConcaveHull<pcl::PointXYZRGBA> chull;
		chull.setInputCloud(cloud_projected);
		chull.setDimension(2);
		chull.setAlpha(0.1);
		chull.reconstruct(*cloud_hull, plygs);

		concaveHulls.push_back(cloud_hull);
		polygons.push_back(plygs);
		planes.push_back(PlaneModel(coefficients, inliers));


		extract.setInputCloud(cloudInput);
		extract.setIndices(inliers);
		extract.setNegative(true);
		extract.filter(*cloud_f);

		cloudInput = cloud_f;

		i++;
	}
}

Eigen::Vector3f PlaneModelSegmentation::normVectorOfWallCorner(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr & cloud, size_t wall1, size_t wall2)
{
	std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> concaveHulls;
	std::vector<std::vector<pcl::Vertices>> polygons;
	std::vector<PlaneModel> planes;

	//extract the planes
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_output(new pcl::PointCloud<pcl::PointXYZRGBA>);
	PlaneModelSegmentation::planeSegmentation(cloud, cloud_output, concaveHulls, polygons, planes, 4);

	//decide which plane is wall1 and which plane is wall2
	int idx1 = -1, idx2 = -1;
	for (size_t i = 0; i < planes.size(); i++)
	{
		int side = PlaneModel::whichSide(planes.at(i));
		if (side == wall1)
		{
			idx1 = i;
		}
		else if (side == wall2)
		{
			idx2 = i;
		}
		
		if (idx1 >= 0 && idx2 >= 0)
		{
			break;
		}
	}


	Eigen::Vector3f n1 = planes.at(idx1).normalVec();
	Eigen::Vector3f n2 = planes.at(idx2).normalVec();

	//calculate the returned norm vector according to the norm vectors of the two planes
	Eigen::Vector3f retVal = n1.cross(n2);
	retVal.normalize();

	return retVal;
}
