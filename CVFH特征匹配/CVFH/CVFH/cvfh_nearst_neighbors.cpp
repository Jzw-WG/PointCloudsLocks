#include <cvfh_nearst_neighbors.h>

//k近邻搜素
void nearestKSearch(flann::Index<flann::ChiSquareDistance<float> >& index, const feature_model& model, int k, flann::Matrix<int>& indices, flann::Matrix<float>& distances)
{
	// Query point
	flann::Matrix<float> p = flann::Matrix<float>(new float[model.second.size()], 1, model.second.size());
	memcpy(&p.ptr()[0], &model.second[0], p.cols * p.rows * sizeof(float));

	indices = flann::Matrix<int>(new int[k], 1, k);
	distances = flann::Matrix<float>(new float[k], 1, k);
	index.knnSearch(p, indices, distances, k, flann::SearchParams(512));
	delete[] p.ptr();
}
//从.list文件中加载各模型名称
bool loadFileList(std::vector<feature_model>& models, const std::string& filename)
{
	ifstream fs;
	fs.open(filename.c_str());
	if (!fs.is_open() || fs.fail())
		return (false);

	std::string line;
	while (!fs.eof())
	{
		getline(fs, line);
		if (line.empty())
			continue;
		feature_model m;
		m.first = line;
		models.push_back(m);
	}
	fs.close();
	return (true);
}

void eraseInfPoint(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in)
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

int voxelFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr inputcloud, pcl::PointCloud<pcl::PointXYZ>::Ptr outputcloud, float lx, float ly, float lz) {
	if (inputcloud->points.size() == 0) {
		return -1;
	}
	pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
	voxel_grid.setInputCloud(inputcloud);//输入
	voxel_grid.setLeafSize(lx, ly, lz);//分别率,越小越密,参数分别是xyz
	voxel_grid.filter(*outputcloud);//输出
	return 0;
}

Eigen::Matrix4f translate(const float x, const float y, const float z) {
	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
	transform(0, 3) = x;
	transform(1, 3) = y;
	transform(2, 3) = z;
	return transform;
}

//计算cvfh特征
void calcuate_cvfh(const string name, pcl::PointCloud<pcl::VFHSignature308>& cvfhs, float normal_r)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPLYFile<pcl::PointXYZ>(name, *cloud);
	eraseInfPoint(cloud);
	voxelFilter(cloud, cloud, 0.005, 0.005, 0.005);
	cout << "滤波后点云数量：" << cloud->size() << endl;
	//获取包围盒
	pcl::PointXYZ minpt, maxpt;
	pcl::getMinMax3D(*cloud, minpt, maxpt);
	float maxh = maxpt.y - minpt.y; //相机为水平视角时计算可用
	float maxw = maxpt.x - minpt.x;
	float mind = minpt.z;
	//z方向平移使mind固定为常量（参考 Pose Estimation Technique of Scattered Pistons Based on CAD Model and Global Feaetur）(TODO：效果待测试)
	Eigen::Matrix4f trans_mat = translate(0, 0, GConst::min_distance - mind);
	pcl::transformPointCloud(*cloud, *cloud, trans_mat);
	//估计法线
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud);
	//ne.setRadiusSearch(normal_r);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	ne.setKSearch(16);	//使用半径在查询点周围0.65范围内的所有邻元素
	ne.compute(*cloud_normals);

	//CVFH
	pcl::CVFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> cvfh;
	cvfh.setInputCloud(cloud);
	cvfh.setInputNormals(cloud_normals);
	//创建一个空的kd树表示法
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>);
	cvfh.setSearchMethod(tree1);
	cvfh.compute(cvfhs);
}

//计算vfh特征
void calcuate_vfh(const string name, pcl::PointCloud<pcl::VFHSignature308>& vfhs, float normal_r)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPLYFile<pcl::PointXYZ>(name, *cloud);
	eraseInfPoint(cloud);
	voxelFilter(cloud, cloud, 0.005, 0.005, 0.005);
	cout << "滤波后点云数量：" << cloud->size() << endl;
	//获取包围盒
	pcl::PointXYZ minpt, maxpt;
	pcl::getMinMax3D(*cloud, minpt, maxpt);
	float maxh = maxpt.y - minpt.y; //相机为水平视角时计算可用
	float maxw = maxpt.x - minpt.x;
	float mind = minpt.z;
	//z方向平移使mind固定为常量（参考 Pose Estimation Technique of Scattered Pistons Based on CAD Model and Global Feaetur）(TODO：效果待测试)
	//Eigen::Matrix4f trans_mat = translate(0, 0, GConst::min_distance - mind);
	//pcl::transformPointCloud(*cloud, *cloud, trans_mat);
	//估计法线
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud);
	//ne.setRadiusSearch(normal_r);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	ne.setKSearch(16);	//使用半径在查询点周围0.65范围内的所有邻元素
	ne.compute(*cloud_normals);

	//VFH
	pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
	vfh.setInputCloud(cloud);
	vfh.setInputNormals(cloud_normals);
	//创建一个空的kd树表示法
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>);
	vfh.setSearchMethod(tree1);
	vfh.compute(vfhs);
}

//计算shot特征
void calcuate_shot(const string name, pcl::PointCloud<pcl::SHOT352>& shots, float radius)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPLYFile<pcl::PointXYZ>(name, *cloud);
	eraseInfPoint(cloud);
	voxelFilter(cloud, cloud, 0.005, 0.005, 0.005);
	cout << "滤波后点云数量：" << cloud->size() << endl;
	//获取包围盒
	pcl::PointXYZ minpt, maxpt;
	pcl::getMinMax3D(*cloud, minpt, maxpt);
	float maxh = maxpt.y - minpt.y; //相机为水平视角时计算可用
	float maxw = maxpt.x - minpt.x;
	float mind = minpt.z;
	//估计法线
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud);
	//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	//ne.setSearchMethod(tree);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	ne.setKSearch(16);	//使用半径在查询点周围0.65范围内的所有邻元素
	ne.compute(*cloud_normals);

	//SHOT
	pcl::SHOTEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> shot;
	shot.setRadiusSearch(radius);
	shot.setInputCloud(cloud);
	shot.setInputNormals(cloud_normals);
	shot.setSearchSurface(cloud);
	////创建一个空的kd树表示法
	//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>);
	//shot.setSearchMethod(tree1);
	shot.compute(shots);
}
