#include <cvfh_nearst_neighbors.h>

//k近邻搜素
void nearestKSearch(flann::Index<flann::ChiSquareDistance<float> >& index, const cvfh_model& model, int k, flann::Matrix<int>& indices, flann::Matrix<float>& distances)
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
bool loadFileList(std::vector<cvfh_model>& models, const std::string& filename)
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
		cvfh_model m;
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

//计算vfh特征
void calcuate_cvfh(const string name, pcl::PointCloud<pcl::VFHSignature308>& vfhs, float normal_r)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPLYFile<pcl::PointXYZ>(name, *cloud);
	eraseInfPoint(cloud);
	voxelFilter(cloud, cloud, 0.005, 0.005, 0.005);
	cout << "滤波后点云数量：" << cloud->size() << endl;
	//估计法线
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	ne.setKSearch(16);	//使用半径在查询点周围0.65范围内的所有邻元素
	ne.compute(*cloud_normals);

	//VFH
	pcl::CVFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> cvfh;
	cvfh.setInputCloud(cloud);
	cvfh.setInputNormals(cloud_normals);
	//创建一个空的kd树表示法
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>);
	cvfh.setSearchMethod(tree1);
	cvfh.compute(vfhs);
}
