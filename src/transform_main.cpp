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

std::vector<std::string> getFilenames(std::string dirname,std::string ext)
{
	boost::filesystem::path dir(dirname);
	std::vector<std::string> Files;
	boost::filesystem::directory_iterator pos(dir);
	boost::filesystem::directory_iterator end;

	for (; pos != end; pos++)
	if (boost::filesystem::is_regular_file(pos->status()))
	{
		std::string fileext=boost::filesystem::extension(*pos);
		if ( fileext== ext)
		{
			Files.push_back(pos->path().string());
		}
	}
	

	
	return Files;
}
std::string int2string(int a)
{

	std::stringstream  ss;
	ss<<a;
	return ss.str();
}
int getFrameIDFromPath(std::string path)
{
	boost::filesystem::path s(path);
	std::string filename=s.filename().string();
	
	
	int se = filename.find_last_of("_mat");
	return (atoi(filename.substr(0, se).c_str()));
}

void transformCloudInDir(std::string wd,std::string path,std::string ext)
{
	std::vector<std::string> files;
	files=getFilenames(wd+"/"+path+"/"+"_mat/",ext);
	boost::filesystem::create_directory(wd+"/"+path+"/"+"_newcloud");
	for (int i=0;i<files.size();i++)
	{
		//load transform matrix
		std::fstream matrixfile(files[i],std::ios::in);
		Eigen::Matrix4f transform;
		for (int r=0;r<4;r++)
			for (int c=0;c<4;c++)
				matrixfile>>transform(r,c);
		//read in ply and transform it to a new dir
		
		std::string infile=wd+"/"+path+"/"+"_cloud/"+int2string(getFrameIDFromPath(files[i]))+"_cloud.ply";
		std::string outfile=wd+"/"+path+"/"+"_newcloud/"+int2string(getFrameIDFromPath(files[i]))+"_newcloud.ply";
		pcl::PointCloud<pcl::PointXYZRGBA> pc;
		pcl::io::loadPLYFile(infile,pc);
		pcl::transformPointCloud(pc,pc,transform);
		pcl::io::savePLYFileBinary(outfile,pc);
	}
}
int main(int argc,char *argv[])
{
	std::string wd;
	int n;

	std::fstream file("transform_config.txt",std::ios::in);
	std::string ext;

	file>>wd>>n>>ext;
	for (int i=0;i<n;i++)
	{
		
		std::string path;
		file>>path;
		transformCloudInDir(wd,path,ext);
	}

	return 0;
}
