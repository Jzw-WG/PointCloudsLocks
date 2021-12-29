#include <cvfh_generate.h>

float model_ss_(7.5f); //0.01f
float scene_ss_(20.0f);//0.03f-20
float rf_rad_(10.0f);  //0.015f-10
float descr_rad_(0.02f);//0.02f -19 - 
float cg_size_(10.0f);  //0.01f-10
float cg_thresh_(5.0f);

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

Eigen::Matrix4f translate1(const float x, const float y, const float z) {
	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
	transform(0, 3) = x;
	transform(1, 3) = y;
	transform(2, 3) = z;
	return transform;
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
		float mind = minpt.z;
		//z方向平移使mind固定为常量（参考 Pose Estimation Technique of Scattered Pistons Based on CAD Model and Global Feaetur）(TODO：效果待测试)
		Eigen::Matrix4f trans_mat = translate1(0, 0, GConst::min_distance - mind);
		pcl::transformPointCloud(*cloud_in, *cloud_in, trans_mat);
		//pcl::getMinMax3D(*cloud_in, minpt, maxpt);
		//maxh = maxpt.y - minpt.y;
		//maxw = maxpt.x - minpt.x;
		//mind = minpt.z;//测试查看变化
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
		////显示vfh特征
		//pcl::visualization::PCLPlotter plotter;
		//plotter.addFeatureHistogram<pcl::VFHSignature308>(*vfhs, "vfh", 0);
		//plotter.plot();
		//vfh文件带包围盒信息小数*10000倍
		string vfh_filename = path + "\\" + name + "_" + GConst::g_vfh + "_" + GConst::g_box + "_" + std::to_string((int)(maxh*10000)) + "-" + std::to_string((int)(maxw * 10000)) + ".pcd";
		pcl::io::savePCDFile(vfh_filename, *vfhs);
	}
	cout << "ok" << endl;
	return 0;
}

//shot特征
int save_shot(string path, vector<string> files)
{
	for (string s : files)
	{
		string::size_type iPos = s.find_last_of('\\') + 1;
		string filename = s.substr(iPos, s.length() - iPos);
		string name = filename.substr(0, filename.rfind("."));
		string suffix_str = filename.substr(filename.find_last_of('.') + 1);
		//读取点云
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::io::loadPLYFile<pcl::PointXYZ>(s, *cloud_in);
		eraseInfPoint1(cloud_in);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_filtered(new pcl::PointCloud<pcl::PointXYZ>);
		voxelFilter1(cloud_in, cloud_in_filtered, 0.005, 0.005, 0.005);
		//获取包围盒
		pcl::PointXYZ minpt, maxpt;
		pcl::getMinMax3D(*cloud_in, minpt, maxpt);
		float maxh = maxpt.y - minpt.y; //相机为水平视角时计算可用
		float maxw = maxpt.x - minpt.x;
		float mind = minpt.z;
		//估计法线
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
		pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> est_normal;
		est_normal.setKSearch(16);         //设置k邻域搜索阈值为20个点
		//est_normal.setNumberOfThreads(4);
		est_normal.setInputCloud(cloud_in_filtered);   //设置输入模型点云
		//est_normal.setSearchMethod(tree);
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		est_normal.compute(*normals);//计算点云法线

		//SHOT
		pcl::SHOTEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> est_shot;
		est_shot.setRadiusSearch(descr_rad_);
		est_shot.setInputCloud(cloud_in_filtered);
		est_shot.setInputNormals(normals);
		////创建一个空的kd树表示法
		//pcl::search::KdTree<PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>);
		//est_shot.setSearchMethod(tree1);
		est_shot.setSearchSurface(cloud_in_filtered);
		//输出的数据集
		pcl::PointCloud<pcl::SHOT352>::Ptr shots(new pcl::PointCloud<pcl::SHOT352>());
		est_shot.compute(*shots);
		////显示shot特征
		//pcl::visualization::PCLPlotter plotter;
		//plotter.addFeatureHistogram<pcl::SHOT352>(*shots, GConst::g_shot, 0);
		//plotter.plot();
		//shot文件带包围盒信息小数*10000倍
		string shot_filename = path + "\\" + name + "_" + GConst::g_shot + "_" + GConst::g_box + "_" + std::to_string((int)(maxh * 10000)) + "-" + std::to_string((int)(maxw * 10000)) + ".pcd";
		pcl::io::savePCDFile(shot_filename, *shots);
	}
	cout << "ok" << endl;
	return 0;
}

int save_feature(string path, vector<string> files, string feature_name) {
	if (feature_name == GConst::g_shot) {
		return save_shot(path, files);
	}
	else if (feature_name == GConst::g_vfh) {
		return save_cvfh(path, files);
	}
	return -1;
}


