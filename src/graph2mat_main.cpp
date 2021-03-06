#include <Eigen/Core>
#include <Eigen/Geometry>

#include <iostream>
#include <fstream>
#include <vector>
#include <map>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
typedef Eigen::Matrix< float, 7, 1,0,7,1 > Vector7f;


std::string int2str(int n)
{
	std::stringstream ss;
	ss << n;
	return ss.str();
}

//transform the optimized graph from g2o representation to transform matrix, and store them aside the orignal _mat.txt file
int main()
{

	std::fstream file("../config/graph2mat_config.txt");
	
	std::string ext,matpathfile,finalpose;
	file >> ext >> matpathfile>>finalpose;
	file.close();
	std::fstream vspath("../config/"+matpathfile,std::ios::in);
	std::fstream finalposefile("../config/"+finalpose,std::ios::in);

	while(true)
	{
		std::string type;
		Vector7f vec;
		int id;
		if (!(finalposefile>>type))
			break;
		if (type!="VERTEX_SE3:QUAT")
			continue;
		
		if (!(finalposefile>>id>>vec[0]>>vec[1]>>vec[2]>>vec[3]>>vec[4]>>vec[5]>>vec[6]))
			break;
		//get mat path
		std::string path;
		if (!(vspath>>path))
			break;;
		
		//transform quat to matrix
		Eigen::Matrix4f transform;
		Eigen::Quaternion<float> q;
		transform.setZero();

		q.x()=vec[3];
		q.y()=vec[4];
		q.z()=vec[5];
		q.w()=vec[6];

		transform.block<3,3>(0,0)=q.matrix();
		transform(0,3)=vec[0];
		transform(1,3)=vec[1];
		transform(2,3)=vec[2];
		transform(3,3)=1;

		
		boost::filesystem::path s(path);
		s.replace_extension(ext);
		
		//output new mat
		std::fstream mat_file(s.c_str(),std::ios::out);
		mat_file<<transform;
		mat_file.close();
	}
	finalposefile.close();
	vspath.close();

	return 0;

}