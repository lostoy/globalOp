#include <string>
#include <sstream>

#include <pcl/pcl_macros.h>


#include "utility.h"

namespace utility
{
	void readAllBytes(std::string fileLoc, char * buffer, size_t & length)
	{
		std::ifstream is(fileLoc, std::ifstream::in | std::ifstream::binary);

		is.exceptions(std::ios::failbit);
		try
		{
			if (is)
			{
				// get length of file:
				is.seekg(0, is.end);
				length = is.tellg();
				is.seekg(0, is.beg);

				// read data as a block:
				is.read(buffer, length);
				is.close();
			}
		} catch (std::ios_base::failure & fail)	
		{
			std::cout << fail.code() << "\topen file failure: " << "\"" << fileLoc << "\"" << std::endl;
		}
	}


	std::vector<boost::filesystem::path> listFiles(const boost::filesystem::path & root, const std::string & ext)
	{
		std::vector<boost::filesystem::path> ret;

		if (!boost::filesystem::exists(root)) return ret;

		if (boost::filesystem::is_directory(root))
		{
			boost::filesystem::recursive_directory_iterator it(root);
			boost::filesystem::recursive_directory_iterator endit;
			while (it != endit)
			{
				if (boost::filesystem::is_regular_file(*it) && it->path().extension() == ext)
				{
					ret.push_back(it->path());
				}
				++it;
			}
		}

		return ret;
	}


	bool isinf(float f)
	{
		return _isnanf(f) || (!_finitef(f)) ||  f < -500 || f > 500;
	}

	float deg2rad(float degrees)
	{
		return (float)(M_PI / 180) * degrees;
	}

	Eigen::Quaternionf quaternionProduct(Eigen::Quaternionf a, Eigen::Quaternionf b)
	{
		float a1 = a.w(), a2 = a.x(), a3 = a.y(), a4 = a.z();
		float b1 = b.w(), b2 = b.x(), b3 = b.y(), b4 = b.z();

		Eigen::Quaternionf retValue(a1*b1 - a2*b2 - a3*b3 - a4*b4,
									a1*b2 + a2*b1 + a3*b4 - a4*b3,
									a1*b3 - a2*b4 + a3*b1 + a4*b2,
									a1*b4 + a2*b3 - a3*b2 + a4*b1);

		return retValue;
	}


	Eigen::Matrix4f rm2tm(const Eigen::Matrix3f & rm)
	{
		Eigen::Matrix4f retValue = Eigen::Matrix4f::Identity();

		//extend the rotation matrix to a transformation matrix
		for (size_t i = 0; i < 3; i++)
		{
			for (size_t j = 0;  j < 3;  j++)
			{
				retValue(i, j) = rm(i, j);
			}

		}
		return retValue;
	}


	void removeNaNFromPointCloud(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud_in, pcl::PointCloud<pcl::PointXYZRGBA> &cloud_out,
		std::vector<int> &index)
	{
		// If the clouds are not the same, prepare the output
		if (&cloud_in != &cloud_out)
		{
			cloud_out.header = cloud_in.header;
			cloud_out.points.resize(cloud_in.points.size());
		}
		// Reserve enough space for the indices
		index.resize(cloud_in.points.size());
		size_t j = 0;

		// If the data is dense, we don't need to check for NaN
		if (cloud_in.is_dense)
		{
			cloud_out.points = cloud_in.points;
			for (j = 0; j < cloud_out.points.size(); ++j)
			{
				index[j] = j;
			}
		}
		else
		{
			for (size_t i = 0; i < cloud_in.points.size(); ++i)
			{
				if (!pcl_isfinite(cloud_in.points[i].x) ||
					!pcl_isfinite(cloud_in.points[i].y) ||
					!pcl_isfinite(cloud_in.points[i].z))
					continue;
				cloud_out.points[j] = cloud_in.points[i];
				index[j] = i;
				j++;
			}
			if (j != cloud_in.points.size())
			{
				// Resize to the correct size
				cloud_out.points.resize(j);
				index.resize(j);
				cloud_out.height = 1;
				cloud_out.width = j;
			}
			// Removing bad points => dense (note: 'dense' doesn't mean 'organized')
			cloud_out.is_dense = true;
		}
	}



	bool readTransformMatrix(std::string matrixfile, Eigen::Matrix4f &transform)
	{
		std::fstream file;
		file.open(matrixfile, std::ios::in);
		if (!file.is_open())
		{
			std::cout << "wrong!";
			return false;
		}
		for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			file >> transform(i, j);
		return true;
	}

	float distance(const pcl::PointXYZRGBA & p1, const pcl::PointXYZRGBA & p2)
	{
		Eigen::Vector3f vp1(p1.x, p1.y, p1.z);
		Eigen::Vector3f vp2(p2.x, p2.y, p2.z);

		return (vp1 - vp2).norm();
	}

	std::vector<pcl::PointXYZRGBA> loadPoints(std::string fileLoc)
	{
		std::vector<pcl::PointXYZRGBA> points;

		std::ifstream fin(fileLoc);
		while (true)
		{
			std::string line;
			if (std::getline(fin, line))
			{
				pcl::PointXYZRGBA point;
				std::istringstream iss(line);

				iss >> point.x >> point.y >> point.z;

				points.push_back(point);
			}
			else
			{
				break;
			}
		}

		return points;

	}


	bool insideCylinder(const pcl::PointXYZRGBA & p1, const pcl::PointXYZRGBA & p2, const pcl::PointXYZRGBA & p, float r)
	{
		Eigen::Vector3f vp1(-p1.x, p1.y, p1.z);
		Eigen::Vector3f vp2(-p2.x, p2.y, p2.z);
		Eigen::Vector3f vp(p.x, p.y, p.z);


		Eigen::Vector3f p1_p = vp - vp1;
		Eigen::Vector3f p2_p = vp - vp2;

		Eigen::Vector3f p1p2 = vp2 - vp1;

		return p1_p.dot(p1p2) > 0 && p2_p.dot(-p1p2) > 0 && (p1_p.cross(p1p2) / p1p2.norm()).norm() < r;
		
	}
}

