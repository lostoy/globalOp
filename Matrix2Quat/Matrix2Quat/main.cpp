#include <Eigen/Core>
#include <Eigen/Geometry>

#include <iostream>
#include <fstream>
#include <vector>
#include <map>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

typedef Eigen::Matrix< float, 7, 1,0,7,1 > Vector7f;
typedef Eigen::Matrix<float, 6, 6,0,6,6> information;

Vector7f matrix2quat(Eigen::Matrix4f &matrix)
{
	Vector7f ans;
	Eigen::Quaternion <float> q(matrix.block<3, 3>(0, 0));
	q.normalize();
	ans[3] = q.x(); ans[4] = q.y(); ans[5] = q.z(); ans[6] = q.w();
	ans.block<3, 1>(0, 0) = matrix.block<3, 1>(0, 0);
	return ans;
}

class Vertex
{
public:
	Eigen::Matrix4f matrix;

	int id;
	
	Vector7f matrix2quat()
	{
		Vector7f ans;
		Eigen::Quaternion <float> q(matrix.block<3, 3>(0, 0));
		q.normalize();
		ans[3] = q.x(); ans[4] = q.y(); ans[5] = q.z(); ans[6] = q.w();
		ans.block<3, 1>(0, 0) = matrix.block<3, 1>(0, 0);
		return ans;
	}
};
class Edge
{
public:
	typedef Eigen::Matrix<float, 28, 1,0,28,1> Vector28;
	Eigen::Matrix4f transform;
	int id;
	int pre_v, next_v;
	information info;
	
	Vector28 edge2quat()
	{
		Vector28 ans;
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
std::vector<std::string> getFilenames(std::string dirname)
{

	boost::filesystem::path dir(dirname);
	std::vector<std::string> Files;
	boost::filesystem::directory_iterator pos(dir);
	boost::filesystem::directory_iterator end;

	for (; pos != end; pos++)
	if (boost::filesystem::is_regular_file(pos->status()))
	if (boost::filesystem::extension(*pos) == "_mat.txt")
		Files.push_back(pos->path().string());
	std::sort(Files.begin(), Files.end());
	return Files;
}

int getFrameIDFromPath(std::string path)
{
	int ss = path.find_last_of("\\");
	if (path[ss + 1] == '\\')
		ss++;
	int se = path.find_last_of("_mat");
	return (atoi(path.substr(ss, se - ss + 1).c_str()));
}
void addVEFromDir(std::string path, int pre_id, int new_frame_id, Eigen::Matrix4f &transform, std::vector<Vertex> & vertices, std::vector<Edge> &edges, std::map<int, int> &idmap)
{
	std::vector<std::string> Files;
	getFilenames(path);
	idmap.clear();

	int startid = vertices.size();
	for (int i = 0; i < Files.size(); i++)
	{
		std::string filename = Files[i];
		int frameid = getFrameIDFromPath(filename);
		Vertex v;
		
		v.id = vertices.size();
		idmap[frameid] = v.id;

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
			e.next_v = vertices.size();

			e.transform = v.matrix;
			e.info = inlier_n*Eigen::Matrix<float,6,6>::Identity();
			edges.push_back(e);
		}
	}

	//transform the vetrices
	if (pre_id > 0)
	{
		Eigen::Matrix4f last_transform = vertices[pre_id].matrix, inc_transform = transform;

		for (int i = idmap[new_frame_id]; i >= startid; i--)
		{
			Eigen::Matrix4f tmp = vertices[i].matrix.inverse();

			vertices[i].matrix = inc_transform*last_transform;
			last_transform = vertices[i].matrix;
			inc_transform = tmp;
		}

		last_transform = vertices[idmap[new_frame_id]].matrix;
		for (int i = idmap[new_frame_id]+1; i < vertices.size(); i++)
		{
			vertices[i].matrix = vertices[i].matrix*last_transform;
		}
	}
	else
	{
		for (int i = startid + 1; i < vertices.size(); i++)
			vertices[i].matrix = vertices[i].matrix*vertices[i - 1].matrix;
	}
	Edge e;
	e.id = edges.size();
	e.info = Eigen::Matrix<float, 6, 6>::Identity() * 1000.0;
	e.pre_v = pre_id;
	e.next_v = idmap[new_frame_id];
	e.transform = transform;
	edges.push_back(e);
}
void loadData(std::string config_file,std::vector<Vertex> & vertices, std::vector<Edge> &edges)
{
	vertices.clear();
	edges.clear();
	std::string wd;

	std::fstream file(config_file, std::ios::in);
	int n;
	file >> wd;
	file >> n;
	
	
	std::map<int, int> idmap;
	for (int i = 0; i < n; i++)
	{
		std::string line;
		Eigen::Matrix4f transform;
		file >> line;
		
		for (int i = 0; i < 4;i++)
		for (int j = 0; j < 4; j++)
			file >> transform(i, j);

		std::vector<std::string> strs;
		boost::split(strs, line, boost::is_any_of("\t"));

		//the start id of the new data folder
	
		if (i == 0)
		{
			//path, pre_id, new_frame,transform,vs,es,new_idmap
			addVEFromDir(wd + "/" + strs[0], -1,atoi(strs[3].c_str()),transform, vertices, edges,idmap);
		}

		addVEFromDir(wd + "/" + strs[1],idmap[atoi(strs[1].c_str())],atoi(strs[3].c_str()),transform, vertices, edges,idmap);

	}
}
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
		Vector7f quat = matrix2quat(edges[i].transform);
		for (int j = 0; j < 7; j++)
			std::cout << quat[j] << " ";
		std::cout << std::endl;

	}
}
int main()
{
	std::vector<Vertex> vs;
	std::vector<Edge> es;
	loadData("config.txt", vs, es);
	outputg2o(vs, es);
	return 0;

}