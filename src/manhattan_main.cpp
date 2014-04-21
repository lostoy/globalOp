#include <Eigen/Core>
#include <Eigen/Geometry>

#include <iostream>
#include <fstream>
#include <vector>
#include <map>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>

#include <PlaneModel.h>
#include <PlaneModelSegmentation.h>

int getFrameIDFromPath(std::string path)
{
	boost::filesystem::path s(path);
	std::string filename = s.filename().string();


	int se = filename.find_last_of("_mat");
	return (atoi(filename.substr(0, se).c_str()));
}

bool isFilenameSmaller(std::string s1, std::string s2)
{
	int i1 = getFrameIDFromPath(s1);
	int i2 = getFrameIDFromPath(s2);
	return i1<i2;
}
std::vector<std::string> getFilenames(std::string dirname, std::string ext)
{
	boost::filesystem::path dir(dirname);
	std::vector<std::string> Files;
	boost::filesystem::directory_iterator pos(dir);
	boost::filesystem::directory_iterator end;

	for (; pos != end; pos++)
	if (boost::filesystem::is_regular_file(pos->status()))
	if (boost::filesystem::extension(*pos) == ext)
	{
		Files.push_back(pos->path().string());
	}


	std::sort(Files.begin(), Files.end(), isFilenameSmaller);
	return Files;
}

void loadTransform(std::string path, Eigen::Matrix4f& transform,int& info)
{
	std::fstream file(path, std::ios::in);
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
			file >> transform(i, j);
	}
	file >> info;
	file.close();
}

void outputTransform(std::string path, std::string ext,Eigen::Matrix4f transform,int info)
{
	boost::filesystem::path filepath(path);
	filepath.replace_extension(ext);
	path = filepath.string();
	std::fstream file(path, std::ios::out);

	file << transform << std::endl;
	file << info;
	file.close();
}

bool findFloor(std::vector<PlaneModel> planes, PlaneModel &floor)
{
	for (int i = 0; i < planes.size(); i++)
	{
		//std::cout << *planes[i].coefficients << std::endl;
		if (PlaneModel::whichSide(planes[i]) == 0)
		{
			floor = planes[i];
			return true;
		}
	}
	std::cout << "no floor!\n";
	return false;
}
bool getAlignGroundTransform(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr next_cloud, PlaneModel globalFloor, Eigen::Matrix4f &inc,Eigen::Matrix4f worldTransform)
{

	std::vector<PlaneModel> prev_planes,next_planes;
	
	PlaneModelSegmentation::planeSegmentation(next_cloud, next_planes);

	PlaneModel next_floor;
	
	
	if (!findFloor(next_planes, next_floor))
		return false;
	Eigen::Vector4f next_floor_vect(next_floor.coefficients->values[0], next_floor.coefficients->values[1], next_floor.coefficients->values[2], next_floor.coefficients->values[3]);
	next_floor_vect = worldTransform.inverse().transpose()*next_floor_vect;
	next_floor.coefficients->values[0] = next_floor_vect[0];
	next_floor.coefficients->values[1] = next_floor_vect[1];
	next_floor.coefficients->values[2] = next_floor_vect[2];
	next_floor.coefficients->values[3] = next_floor_vect[3];
	inc = next_floor.rotationTo(globalFloor);
	//std::cout << "!!"<<std::endl<<inc.block<3,3>(0,0)*(Eigen::Vector4f(next_floor.coefficients->values[0], next_floor.coefficients->values[1], next_floor.coefficients->values[2], next_floor.coefficients->values[3])).block<3,1>(0,0) << std::endl;
	
	return true;
}
void transform2ground(std::string path, std::string inext,std::string ext, bool use_user_floor, PlaneModel &globalFloor)
{
	std::vector<std::string> cloudFiles, matFiles;
	cloudFiles = getFilenames(path + "/" + "_cloud", ".ply");
	matFiles = getFilenames(path + "/" + "_mat", inext);
	
	if (matFiles.size() != cloudFiles.size())
		return;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr next_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	
	Eigen::Matrix4f world_transform;
	world_transform.setIdentity();
	for (int i = 0; i < matFiles.size(); i++)
	{

			
		pcl::io::loadPLYFile(cloudFiles[i],*next_cloud);
		Eigen::Matrix4f transform;
		int info;
		loadTransform(matFiles[i], transform, info);


		pcl::PointCloud<pcl::PointXYZRGBA> tmp_cloud;
		
		Eigen::Matrix4f inc;
		if (i == 0 && !use_user_floor)
		{
			pcl::ModelCoefficients::Ptr cof(new pcl::ModelCoefficients);
			cof->values.resize(4);
			cof->values[0] = cof->values[2] = 0;
			cof->values[1] = 1;
			cof->values[3] = 0.5;
			pcl::PointIndices::Ptr ind;
			globalFloor = PlaneModel(cof, ind);
			
		}
		if (getAlignGroundTransform(next_cloud, globalFloor, inc,transform))
		{
			transform = inc*transform;
		}
		
		outputTransform(matFiles[i] , ext, transform,info);
		//pcl::transformPointCloud(*next_cloud, tmp_cloud, world_transform);
		//pcl::io::savePLYFileBinary(cloudFiles[i] + "man", tmp_cloud);
		
	}

}
void manhattan(std::string configfile)
{
	std::fstream file(configfile, std::ios::in);
	std::string wd,inext,ext;
	int n;
	file >> wd >> n>>inext>>ext;
	PlaneModel global_floor;
	for (int i = 0; i < n; i++)
	{
		std::string path;
		file >> path;
		transform2ground(wd + "/" + path,inext,ext,i!=0,global_floor);
	}
	file.close();
}
int main()
{
	manhattan("../config/manhattan_config.txt");
	return 0;
}