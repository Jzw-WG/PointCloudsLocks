#include <cvfh_generate.h>

void SplitString(const string& s, vector<string>& v, const string& c)
{
	string::size_type pos1, pos2;
	pos2 = s.find(c);
	pos1 = 0;
	while (string::npos != pos2)
	{
		v.push_back(s.substr(pos1, pos2 - pos1));

		pos1 = pos2 + c.size();
		pos2 = s.find(c, pos1);
	}
	if (pos1 != s.length())
		v.push_back(s.substr(pos1));
}

void eraseInfPoint1(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in)
{
	//去除NaN点 防止compute报错
	pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud_in->points.begin();
	while (it != cloud_in->points.end())
	{
		float x, y, z;
		x = it->x;
		y = it->y;
		z = it->z;
		if (!pcl_isfinite(x) || !pcl_isfinite(y) || !pcl_isfinite(z))
		{
			it = cloud_in->points.erase(it);
		}
		else
			++it;
	}
}

int voxelFilter1(pcl::PointCloud<pcl::PointXYZ>::Ptr inputcloud, pcl::PointCloud<pcl::PointXYZ>::Ptr outputcloud, float lx, float ly, float lz) {
	if (inputcloud->points.size() == 0) {
		return -1;
	}
	pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
	voxel_grid.setInputCloud(inputcloud);//输入
	voxel_grid.setLeafSize(lx, ly, lz);//分别率,越小越密,参数分别是xyz
	voxel_grid.filter(*outputcloud);//输出
	return 0;
}

//cvfh全局特性
int save_cvfh(string path, vector<string> files)
{
	for (string s : files)
	{
		string::size_type iPos = s.find_last_of('\\') + 1;
		string filename = s.substr(iPos, s.length() - iPos);
		string name = filename.substr(0, filename.rfind("."));
		string suffix_str = filename.substr(filename.find_last_of('.') + 1);
		//vector<string> v;
		//SplitString(filename, v, ".ply");
		//读取点云
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::io::loadPLYFile<pcl::PointXYZ>(s, *cloud_in);
		eraseInfPoint1(cloud_in);
		voxelFilter1(cloud_in, cloud_in, 0.005, 0.005, 0.005);
		//获取包围盒
		pcl::PointXYZ minpt, maxpt;
		pcl::getMinMax3D(*cloud_in, minpt, maxpt);
		float maxh = maxpt.y - minpt.y; //相机为水平视角时计算可用
		float maxw = maxpt.x - minpt.x;
		//估计法线
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> est_normal;
		est_normal.setKSearch(16);         //设置k邻域搜索阈值为20个点
		est_normal.setInputCloud(cloud_in);   //设置输入模型点云
		est_normal.setSearchMethod(tree);
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		est_normal.compute(*normals);//计算点云法线

		//VFH
		pcl::CVFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> est_vfh;
		est_vfh.setInputCloud(cloud_in);
		est_vfh.setInputNormals(normals);
		//创建一个空的kd树表示法
		pcl::search::KdTree<PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>);
		est_vfh.setSearchMethod(tree1);
		//输出的数据集
		pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs(new pcl::PointCloud<pcl::VFHSignature308>());
		est_vfh.compute(*vfhs);
		//vfh文件带包围盒信息小数*10000倍
		string vfh_filename = path + "\\" + name + "_vfh" + "_box_" + std::to_string((int)(maxh*10000)) + "-" + std::to_string((int)(maxw * 10000)) + ".pcd";
		pcl::io::savePCDFile(vfh_filename, *vfhs);

		////显示vfh特征
		//pcl::visualization::PCLPlotter plotter;
		//plotter.addFeatureHistogram<pcl::VFHSignature308>(*vfhs, "vfh", 0);
		//plotter.plot();
	}
	cout << "ok" << endl;
	return 0;
}


