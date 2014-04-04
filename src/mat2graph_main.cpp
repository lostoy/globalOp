#include <Eigen/Core>
#include <Eigen/Geometry>

#include <iostream>
#include <fstream>
#include <vector>
#include <map>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

//vertex vec[0-2] tx,ty,tz; vec[3-6] (qx,qy,qz,qw),quaternion representation of 3d rotation
typedef Eigen::Matrix< float, 7, 1,0,7,1 > Vector7f;
//information matrix of measurement, tx,ty,tz,alpha,beta,theta
typedef Eigen::Matrix<float, 6, 6,0,6,6> information;
//edge vec[0-6] follows the same defination of vector7f, the rest is the upper traiangular block of the information matrix, stored in a row-major order
typedef Eigen::Matrix<float, 28, 1,0,28,1> Vector28f;

//different representations of 3d rigid transformation,  from matrix to vector7f
Vector7f matrix2quat(Eigen::Matrix4f &matrix)
{
	Vector7f ans;
	Eigen::Quaternion <float> q(matrix.block<3, 3>(0, 0));
	q.normalize();
	ans[3] = q.x(); ans[4] = q.y(); ans[5] = q.z(); ans[6] = q.w();
	ans.block<3, 1>(0, 0) = matrix.block<3, 1>(0, 3);
	return ans;
}

//graph vertex
class Vertex
{
public:
	Eigen::Matrix4f matrix;

	int id;
	std::string path;

	Vector7f matrix2quat()
	{
		Vector7f ans;
		Eigen::Quaternion <float> q(matrix.block<3, 3>(0, 0));
		q.normalize();
		ans[3] = q.x(); ans[4] = q.y(); ans[5] = q.z(); ans[6] = q.w();
		ans.block<3, 1>(0,0) = matrix.block<3, 1>(0, 3);
		return ans;
	}
};

//graph edge
class Edge
{
public:
	
	Eigen::Matrix4f transform;
	int id;
	int pre_v, next_v;
	information info;
	
	Vector28f edge2quat()
	{
		Vector28f ans;
		ans.block<7, 1>(0, 0) = matrix2quat(transform);
		int ind = 7;
		for (int i = 0; i < info.rows();i++)
		for (int j = i; j < info.cols(); j++)
		{
			ans[ind++] = info(i, j);
		}
		return ans;
	}
};

//extract frameid from path(filename),filename of the visual odometry information follows the pattern of [0-9]*_mat.txt
int getFrameIDFromPath(std::string path);
//used in sort, to sort the files by frameid
bool isFilenameSmaller(std::string s1,std::string s2)
{
	int i1=getFrameIDFromPath(s1);
	int i2=getFrameIDFromPath(s2);
	return i1<i2;
}

//get all files with the extension $ext in the directory $dirname, 
std::vector<std::string> getFilenames(std::string dirname,std::string ext)
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
	

	std::sort(Files.begin(), Files.end(),isFilenameSmaller);
	return Files;
}


int getFrameIDFromPath(std::string path)
{
	boost::filesystem::path s(path);
	std::string filename=s.filename().string();
	
	
	int se = filename.find_last_of("_mat");
	return (atoi(filename.substr(0, se).c_str()));
}

//transform a number to string
std::string int2string(int a)
{

	std::stringstream  ss;
	ss<<a;
	return ss.str();
}

//add vertex and edge to the graph, $path is the directory of possible new vertices, $pre_folder is the previous folder, $pre_frameid is the previous frameid, $transform is the transform matrix of the two vertices, set $edge_only to true if you just want to add edge(vertex is already added)
void addVEFromDir(std::string path, std::string pre_folder,std::string pre_frameid, std::string next_folder,std::string next_frameid, Eigen::Matrix4f &transform, std::vector<Vertex> & vertices, std::vector<Edge> &edges, std::map<std::string, int> &idmap,bool edge_only)
{
	if (edge_only)
	{
		int id_pre=idmap[pre_folder+pre_frameid];
		int id_next=idmap[next_folder+next_frameid];
		Edge e;
		e.pre_v=id_pre;
		e.next_v=id_next;
		e.transform=transform;
		e.id=edges.size();
		e.info=Eigen::Matrix<float, 6, 6>::Identity() * 1000.0;
		edges.push_back(e);
		return;
	}
	std::vector<std::string> Files;
	Files=getFilenames(path,".txt");
	

	int startid = vertices.size();
	for (int i = 0; i < Files.size(); i++)
	{
		std::string filename = Files[i];
		
		Vertex v;
		
		v.id = vertices.size();
		v.path=Files[i];
		
		idmap[(pre_frameid==""?pre_folder:next_folder)+int2string(getFrameIDFromPath(Files[i]))] = v.id;

		std::fstream file(Files[i], std::ios::in);
		for (int r = 0; r < 4;r++)
		for (int c = 0; c < 4; c++)
		{
			file >> v.matrix(r, c);
		}
		int inlier_n;
		file >> inlier_n;
		vertices.push_back(v);
		if (i>0)
		{
			Edge e;
			e.id = edges.size();
			e.pre_v = vertices.size() - 2;
			e.next_v = vertices.size()-1;

			e.transform = v.matrix;
			e.info = inlier_n*Eigen::Matrix<float,6,6>::Identity();
			edges.push_back(e);
		}
	}

	//transform the vetrices
	if (pre_frameid != "")
	{
		Eigen::Matrix4f last_transform = vertices[idmap[pre_folder+pre_frameid]].matrix, inc_transform = transform;

		for (int i = idmap[next_folder+next_frameid]; i >= startid; i--)
		{
			Eigen::Matrix4f tmp = vertices[i].matrix.inverse();

			vertices[i].matrix = last_transform*inc_transform;
			last_transform = vertices[i].matrix;
			inc_transform = tmp;
		}

		last_transform = vertices[idmap[next_folder+next_frameid]].matrix;
		for (int i = idmap[next_folder+next_frameid]+1; i < vertices.size(); i++)
		{
			vertices[i].matrix = last_transform*vertices[i].matrix;
		}
		Edge e;
		e.id = edges.size();
		e.info = Eigen::Matrix<float, 6, 6>::Identity() * 1000.0;
		e.pre_v = idmap[pre_folder+pre_frameid];
		e.next_v = idmap[next_folder+next_frameid];
		e.transform = transform;
		edges.push_back(e);
	}
	else
	{
		for (int i = startid + 1; i < vertices.size(); i++)
			vertices[i].matrix = vertices[i].matrix*vertices[i - 1].matrix;
	}
	
}

//load all mat files specified in $config_file
void loadData(std::string config_file,std::vector<Vertex> & vertices, std::vector<Edge> &edges)
{
	vertices.clear();
	edges.clear();
	std::string wd;

	std::fstream file(config_file, std::ios::in);
	int n;
	file >> wd;
	file >> n;
	
	
	std::map<std::string, int> idmap;
	for (int i = 0; i < n; i++)
	{
		
		std::string pre_frameid,next_frameid;
		int edge_only;
		std::string str1,str2;
		Eigen::Matrix4f transform;
		file>>str1>>pre_frameid>>str2>>next_frameid>>edge_only;
		
		for (int i = 0; i < 4;i++)
		for (int j = 0; j < 4; j++)
			file >> transform(i, j);

		
		//the start id of the new data folder
	
		if (i == 0)
		{
			//path, pre_id, new_frame,transform,vs,es,new_idmap
			addVEFromDir(wd + "/" + str1+"/_mat/", str1,"",str2,next_frameid,transform, vertices, edges,idmap,edge_only);
		}

		addVEFromDir(wd + "/" + str2+"/_mat/",str1,pre_frameid,str2,next_frameid,transform, vertices, edges,idmap,edge_only);

	}
}

//output the graph in g2o format
void outputg2o(std::vector<Vertex> & vertices, std::vector<Edge> &edges)
{
	for (int i = 0; i < vertices.size(); i++)
	{
		std::cout << "VERTEX_SE3:QUAT " << vertices[i].id << " ";
		Vector7f quat = vertices[i].matrix2quat();
		for (int j = 0; j < 7; j++)
			std::cout << quat[j] << " ";
		std::cout << std::endl;
	}

	for (int i = 0; i < edges.size(); i++)
	{
		std::cout << "EDGE_SE3:QUAT " << edges[i].pre_v << " " << edges[i].next_v << " ";
		Vector28f quat = edges[i].edge2quat();
		for (int j = 0; j < 28; j++)
			std::cout << quat[j] << " ";
		std::cout << std::endl;

	}
}

//output all the _mat.txt file paths
void outputVerticesPath(std::vector<Vertex> vs,std::string filename)
{
	std::fstream file(filename,std::ios::out);
	for (int i=0;i<vs.size();i++)
	{
		file<<vs[i].path<<std::endl;
	}
	file.close();
	
}
int main()
{
	freopen("../config/1-12.g2o","w",stdout);
	std::vector<Vertex> vs;
	std::vector<Edge> es;
	loadData("../config/mat2graph_config.txt", vs, es);
	outputg2o(vs, es);
	outputVerticesPath(vs,"../config/vspath.txt");
	return 0;

}